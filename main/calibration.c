#include "calibration.h"
#include "nvs_storage.h"
#include "processing.h"

#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "CAL";

#define CAL_QUEUE_TIMEOUT_MS      200
#define CAL_C1_SAMPLES            300
#define CAL_C2_SAMPLES            500
#define CAL_STAGE_LIVE_STRIDE     10
#define CAL_STAGE_MOTION_START    6.0f
#define CAL_STAGE_TARGET_MIN_DEG  12.0f
#define CAL_STAGE_TARGET_HOLD_SAMPLES  10
#define CAL_STAGE_RETURN_HOLD_SAMPLES  10
#define CAL_STAGE_SIGN_DISCOVERY  6.0f
#define CAL_STAGE_CENTER_SAMPLES  8
#define CAL_STAGE_CENTER_OMEGA_DPS 12.0f
#define CAL_STAGE_HOLD_OMEGA_DPS  10.0f
#define CAL_STAGE_HOLD_BAND_DEG   4.0f
#define CAL_STAGE_RETURN_BAND_DEG 4.0f
#define CAL_STAGE_FORCE_MIN_N     0.08f
#define CAL_STAGE_FIG8_PP_MIN_DEG 18.0f
#define CAL_STAGE_FIG8_F95_MAX_HZ 12.0f
#define CAL_C3_GRIP_HOLD_SAMPLES      100
#define CAL_C3_GRIP_TIMEOUT_SAMPLES   400
#define CAL_C3_GRIP_DELTA_MIN_N       0.03f
#define CAL_C3_GRIP_FORCE_SPAN_MAX_N  0.08f
#define CAL_C3_GRIP_OMEGA_MAX_DPS     12.0f
#define CAL_C3_FREE_MOTION_SAMPLES    250
#define CAL_C3_FREE_TIMEOUT_SAMPLES   450
#define CAL_C3_MOTION_START_DPS       8.0f
#define CAL_C3_MOTION_MIN_PP_DEG      6.0f
#define CAL_C3_MOTION_MIN_OMEGA_RMS   6.0f
#define CAL_DIRECTION_WINDOW_SAMPLES 520
#define CAL_FIG8_WINDOW_SAMPLES      700
#define CAL_WORKSPACE_MAX_SAMPLES    CAL_FIG8_WINDOW_SAMPLES

static raw_sample_t s_c1_samples[CAL_C1_SAMPLES];
static float s_cal_omega_window[CAL_WORKSPACE_MAX_SAMPLES];
static float s_cal_roll_window[CAL_WORKSPACE_MAX_SAMPLES];
static float s_cal_pitch_window[CAL_WORKSPACE_MAX_SAMPLES];
static float s_cal_yaw_window[CAL_WORKSPACE_MAX_SAMPLES];
static float s_cal_tremor_window[CAL_WORKSPACE_MAX_SAMPLES];

typedef struct {
    int   n;
    float mean;
    float M2;
} welford_t;

typedef enum {
    MOTION_AXIS_YAW,
    MOTION_AXIS_PITCH,
    MOTION_AXIS_FIG8,
} motion_axis_t;

typedef struct {
    const char   *name;
    motion_axis_t axis;
    float         target_deg;
    int           min_samples;
    int           timeout_samples;
} motion_stage_def_t;

typedef struct {
    float peak;
    float pp_roll;
    float pp_pitch;
    float f95;
    float mean_force;
    float target_offset[3];
    float return_center[3];
    float target_stability_deg;
    float return_stability_deg;
    int   axis_index;
    int   sign;
    int   samples;
} motion_stage_result_t;

typedef struct {
    int axis_index;
    int sign;
} direction_context_t;

static const motion_stage_def_t MOTION_STAGES[] __attribute__((unused)) = {
    { "yaw_right",     MOTION_AXIS_YAW,   22.0f, 150, 500 },
    { "yaw_left",      MOTION_AXIS_YAW,   22.0f, 150, 500 },
    { "pitch_forward", MOTION_AXIS_PITCH, 18.0f, 150, 500 },
    { "pitch_back",    MOTION_AXIS_PITCH, 18.0f, 150, 500 },
    { "figure8_a",     MOTION_AXIS_FIG8,   0.0f, 260, 650 },
    { "figure8_b",     MOTION_AXIS_FIG8,   0.0f, 260, 650 },
};

static const uint32_t MOTION_STAGE_BITS[] __attribute__((unused)) = {
    C3_STAGE_BIT_YAW_RIGHT,
    C3_STAGE_BIT_YAW_LEFT,
    C3_STAGE_BIT_PITCH_FORWARD,
    C3_STAGE_BIT_PITCH_BACK,
    C3_STAGE_BIT_FIG8_A,
    C3_STAGE_BIT_FIG8_B,
};

static void welford_reset(welford_t *w)
{
    w->n = 0;
    w->mean = 0.0f;
    w->M2 = 0.0f;
}

static void welford_update(welford_t *w, float x)
{
    w->n++;
    float delta = x - w->mean;
    w->mean += delta / (float)w->n;
    float delta2 = x - w->mean;
    w->M2 += delta * delta2;
}

static float welford_variance(const welford_t *w)
{
    if (w->n < 2) {
        return 0.0f;
    }
    return w->M2 / (float)(w->n - 1);
}

static float welford_stddev(const welford_t *w)
{
    return sqrtf(welford_variance(w));
}

static void uart_send(const char *msg)
{
    uart_write_bytes(UART_NUM_0, msg, strlen(msg));
}

static void uart_sendf(char *buf, int buf_len, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, (size_t)buf_len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        uart_write_bytes(UART_NUM_0, buf, (size_t)n);
    }
}

static void drain_queue(QueueHandle_t raw_q)
{
    raw_sample_t discard;
    while (xQueueReceive(raw_q, &discard, 0) == pdTRUE) {}
}

static void log_stack_watermark(const char *label)
{
    UBaseType_t words = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "CAL stack watermark after %s: %u words (~%u bytes)",
             label, (unsigned)words, (unsigned)(words * sizeof(StackType_t)));
}

static void reset_c3_progress(cal_params_t *params)
{
    params->f_ref_open = 0.0f;
    params->f95_ref = 0.0f;
    params->pp_roll_ref = 0.0f;
    params->pp_pitch_ref = 0.0f;
    params->motion_rms_ref = 0.0f;
    params->motion_tremor_ratio_ref = 0.0f;
    params->yaw_right_ref = 0.0f;
    params->yaw_left_ref = 0.0f;
    params->pitch_fwd_ref = 0.0f;
    params->pitch_back_ref = 0.0f;
    memset(params->c3_stage_peak, 0, sizeof(params->c3_stage_peak));
    memset(params->c3_stage_mean_force, 0, sizeof(params->c3_stage_mean_force));
    memset(params->c3_stage_f95, 0, sizeof(params->c3_stage_f95));
    memset(params->c3_stage_pp_roll, 0, sizeof(params->c3_stage_pp_roll));
    memset(params->c3_stage_pp_pitch, 0, sizeof(params->c3_stage_pp_pitch));
    memset(params->c3_stage_target, 0, sizeof(params->c3_stage_target));
    memset(params->c3_stage_return_center, 0, sizeof(params->c3_stage_return_center));
    params->c3_stage_mask = 0;
    params->yaw_axis_index = -1;
    params->yaw_sign = 0;
    params->pitch_axis_index = -1;
    params->pitch_sign = 0;
}

static void __attribute__((unused)) persist_stage_result(cal_params_t *params, int stage_idx,
                                                         const motion_stage_result_t *result)
{
    params->c3_stage_peak[stage_idx] = result->peak;
    params->c3_stage_mean_force[stage_idx] = result->mean_force;
    params->c3_stage_f95[stage_idx] = result->f95;
    params->c3_stage_pp_roll[stage_idx] = result->pp_roll;
    params->c3_stage_pp_pitch[stage_idx] = result->pp_pitch;
    memcpy(params->c3_stage_target[stage_idx], result->target_offset, sizeof(result->target_offset));
    memcpy(params->c3_stage_return_center[stage_idx], result->return_center, sizeof(result->return_center));
    params->c3_stage_mask |= MOTION_STAGE_BITS[stage_idx];
}

static float angle_diff_deg(float current, float neutral)
{
    float diff = current - neutral;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

static float maxf3(float a, float b, float c)
{
    float m = (a > b) ? a : b;
    return (m > c) ? m : c;
}

static const char *c1_reason(float rms_omega, float spread)
{
    if (rms_omega > 5.0f || spread > 5.0f) {
        return "not_still";
    }
    return "";
}

static const char *c2_reason(bool pressure_present, bool unstable, bool drifting)
{
    if (pressure_present) {
        return "pressure_present";
    }
    if (unstable) {
        return "unstable_pressure";
    }
    if (drifting) {
        return "drift";
    }
    return "";
}

static float window_rms_local(const float *values, int n)
{
    if (values == NULL || n <= 0) {
        return 0.0f;
    }
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        sum += values[i] * values[i];
    }
    return sqrtf(sum / (float)n);
}

static void emit_stage_event(const char *stage,
                             const char *phase,
                             float progress,
                             const char *reason,
                             const char *checkpoint,
                             const char *axis)
{
    char buf[320];
    const char *safe_reason = (reason != NULL) ? reason : "";
    const char *safe_checkpoint = (checkpoint != NULL) ? checkpoint : "";
    const char *safe_axis = (axis != NULL) ? axis : "";
    ESP_LOGI(TAG, "C3_STAGE stage=%s phase=%s progress=%.2f checkpoint=%s axis=%s reason=%s",
             stage, phase, (double)progress, safe_checkpoint, safe_axis, safe_reason);
    uart_sendf(buf, sizeof(buf),
               "{\"cal\":\"C3_STAGE\",\"stage\":\"%s\",\"phase\":\"%s\","
               "\"progress\":%.3f,\"checkpoint\":\"%s\",\"axis\":\"%s\",\"reason\":\"%s\"}\r\n",
               stage, phase, (double)progress, safe_checkpoint, safe_axis, safe_reason);
}

static void emit_capture_stage_event(const char *cal_name,
                                     const char *stage,
                                     const char *phase,
                                     float progress,
                                     const char *reason,
                                     const char *checkpoint,
                                     const char *axis)
{
    char buf[320];
    const char *safe_reason = (reason != NULL) ? reason : "";
    const char *safe_checkpoint = (checkpoint != NULL) ? checkpoint : "";
    const char *safe_axis = (axis != NULL) ? axis : "";
    ESP_LOGI(TAG, "%s_STAGE stage=%s phase=%s progress=%.2f checkpoint=%s axis=%s reason=%s",
             cal_name, stage, phase, (double)progress, safe_checkpoint, safe_axis, safe_reason);
    uart_sendf(buf, sizeof(buf),
               "{\"cal\":\"%s_STAGE\",\"stage\":\"%s\",\"phase\":\"%s\","
               "\"progress\":%.3f,\"checkpoint\":\"%s\",\"axis\":\"%s\",\"reason\":\"%s\"}\r\n",
               cal_name, stage, phase, (double)progress, safe_checkpoint, safe_axis, safe_reason);
}

static void emit_c3_stage_event(const char *stage,
                                const char *phase,
                                float progress,
                                const char *reason,
                                const char *checkpoint,
                                const char *axis)
{
    emit_capture_stage_event("C3", stage, phase, progress, reason, checkpoint, axis);
}

static void emit_c4_stage_event(const char *stage,
                                const char *phase,
                                float progress,
                                const char *reason,
                                const char *checkpoint,
                                const char *axis)
{
    emit_capture_stage_event("C4", stage, phase, progress, reason, checkpoint, axis);
}

static esp_err_t run_c3_grip_hold(QueueHandle_t raw_q,
                                  const cal_params_t *params,
                                  float *mean_force_out)
{
    const char *stage = "grip_hold";
    float baseline_force = params->mu[0] + params->mu[1] + params->mu[2];
    float required_force = baseline_force + CAL_C3_GRIP_DELTA_MIN_N;
    if (required_force < CAL_STAGE_FORCE_MIN_N) {
        required_force = CAL_STAGE_FORCE_MIN_N;
    }

    int stable_count = 0;
    bool saw_force = false;
    float force_sum = 0.0f;
    float force_min = 0.0f;
    float force_max = 0.0f;

    emit_c3_stage_event(stage, "prompt", 0.0f, "", "", "");

    for (int i = 0; i < CAL_C3_GRIP_TIMEOUT_SAMPLES; i++) {
        raw_sample_t samp;
        if (xQueueReceive(raw_q, &samp, pdMS_TO_TICKS(CAL_QUEUE_TIMEOUT_MS)) != pdTRUE) {
            emit_c3_stage_event(stage, "fail", 0.0f, "timeout", "", "");
            return ESP_ERR_TIMEOUT;
        }

        float f_sum = samp.fsr_n[0] + samp.fsr_n[1] + samp.fsr_n[2];
        float gx = samp.gyro_dps[0] - params->gyro_bias[0];
        float gy = samp.gyro_dps[1] - params->gyro_bias[1];
        float gz = samp.gyro_dps[2] - params->gyro_bias[2];
        float omega_mag = sqrtf(gx * gx + gy * gy + gz * gz);

        bool candidate = (f_sum >= required_force && omega_mag <= CAL_C3_GRIP_OMEGA_MAX_DPS);
        if (f_sum >= required_force) {
            saw_force = true;
        }

        if (candidate) {
            if (stable_count == 0) {
                force_min = f_sum;
                force_max = f_sum;
                force_sum = 0.0f;
            }
            if (f_sum < force_min) force_min = f_sum;
            if (f_sum > force_max) force_max = f_sum;
            if ((force_max - force_min) > CAL_C3_GRIP_FORCE_SPAN_MAX_N) {
                stable_count = 0;
                force_min = f_sum;
                force_max = f_sum;
                force_sum = 0.0f;
            }
            force_sum += f_sum;
            stable_count++;
        } else {
            stable_count = 0;
            force_sum = 0.0f;
        }

        if (i % CAL_STAGE_LIVE_STRIDE == 0) {
            float progress = 0.5f * ((float)stable_count / (float)CAL_C3_GRIP_HOLD_SAMPLES);
            if (progress > 0.49f) progress = 0.49f;
            emit_c3_stage_event(stage, "capturing", progress, "", "", "");
        }

        if (stable_count >= CAL_C3_GRIP_HOLD_SAMPLES) {
            *mean_force_out = force_sum / (float)stable_count;
            emit_c3_stage_event(stage, "pass", 1.0f, "", "grip", "");
            return ESP_OK;
        }
    }

    emit_c3_stage_event(stage, "fail", 1.0f, saw_force ? "grip_unstable" : "pressure_absent", "", "");
    return ESP_FAIL;
}

static esp_err_t run_c4_hand_motion(QueueHandle_t raw_q,
                                    const cal_params_t *params,
                                    float *f95_out,
                                    float *pp_roll_out,
                                    float *pp_pitch_out,
                                    float *motion_rms_out,
                                    float *motion_tremor_ratio_out)
{
    const char *stage = "hand_motion";
    float *omega_window = s_cal_omega_window;
    float *roll_window = s_cal_roll_window;
    float *pitch_window = s_cal_pitch_window;
    float *tremor_window = s_cal_tremor_window;
    int samples = 0;
    bool started = false;

    processing_tremor_baseline_reset();
    emit_c4_stage_event(stage, "prompt", 0.0f, "", "", "");

    for (int i = 0; i < CAL_C3_FREE_TIMEOUT_SAMPLES; i++) {
        raw_sample_t samp;
        if (xQueueReceive(raw_q, &samp, pdMS_TO_TICKS(CAL_QUEUE_TIMEOUT_MS)) != pdTRUE) {
            emit_c4_stage_event(stage, "fail", 0.0f, "timeout", "", "");
            return ESP_ERR_TIMEOUT;
        }

        float gx = samp.gyro_dps[0] - params->gyro_bias[0];
        float gy = samp.gyro_dps[1] - params->gyro_bias[1];
        float gz = samp.gyro_dps[2] - params->gyro_bias[2];
        float omega_mag = sqrtf(gx * gx + gy * gy + gz * gz);
        float unbiased[3] = { gx, gy, gz };
        float roll_delta = angle_diff_deg(samp.euler_deg[0], params->euler_neutral[0]);
        float pitch_delta = angle_diff_deg(samp.euler_deg[1], params->euler_neutral[1]);

        if (!started) {
            if (omega_mag >= CAL_C3_MOTION_START_DPS ||
                fabsf(roll_delta) >= CAL_C3_MOTION_MIN_PP_DEG ||
                fabsf(pitch_delta) >= CAL_C3_MOTION_MIN_PP_DEG) {
                started = true;
            } else if (i % CAL_STAGE_LIVE_STRIDE == 0) {
                emit_c4_stage_event(stage, "capturing", 0.0f, "", "", "");
            }
        }

        if (!started) {
            continue;
        }

        if (samples < CAL_C3_FREE_MOTION_SAMPLES) {
            omega_window[samples] = omega_mag;
            roll_window[samples] = roll_delta;
            pitch_window[samples] = pitch_delta;
            tremor_window[samples] = processing_tremor_baseline_step(unbiased);
            samples++;
        }

        if (i % CAL_STAGE_LIVE_STRIDE == 0) {
            float progress = (float)samples / (float)CAL_C3_FREE_MOTION_SAMPLES;
            if (progress > 0.99f) progress = 0.99f;
            emit_c4_stage_event(stage, "capturing", progress, "", "", "");
        }

        if (samples >= CAL_C3_FREE_MOTION_SAMPLES) {
            break;
        }
    }

    if (samples < CAL_C3_FREE_MOTION_SAMPLES) {
        emit_c4_stage_event(stage, "fail", 1.0f, "motion_too_short", "", "");
        return ESP_FAIL;
    }

    float roll_min = roll_window[0], roll_max = roll_window[0];
    float pitch_min = pitch_window[0], pitch_max = pitch_window[0];
    for (int i = 1; i < samples; i++) {
        if (roll_window[i] < roll_min) roll_min = roll_window[i];
        if (roll_window[i] > roll_max) roll_max = roll_window[i];
        if (pitch_window[i] < pitch_min) pitch_min = pitch_window[i];
        if (pitch_window[i] > pitch_max) pitch_max = pitch_window[i];
    }

    float pp_roll = roll_max - roll_min;
    float pp_pitch = pitch_max - pitch_min;
    float omega_rms = window_rms_local(omega_window, samples);
    if (omega_rms < CAL_C3_MOTION_MIN_OMEGA_RMS &&
        pp_roll < CAL_C3_MOTION_MIN_PP_DEG &&
        pp_pitch < CAL_C3_MOTION_MIN_PP_DEG) {
        emit_c4_stage_event(stage, "fail", 1.0f, "range_too_small", "", "");
        return ESP_FAIL;
    }

    *pp_roll_out = pp_roll;
    *pp_pitch_out = pp_pitch;
    *motion_rms_out = omega_rms;
    *f95_out = processing_compute_f95_window(omega_window, samples);
    if (*f95_out < 0.5f) {
        *f95_out = 0.5f;
    }
    float tremor_rms = window_rms_local(tremor_window, samples);
    *motion_tremor_ratio_out = tremor_rms / fmaxf(*motion_rms_out, TREMOR_RATIO_FLOOR_DPS);
    emit_c4_stage_event(stage, "pass", 1.0f, "", "motion", "");
    return ESP_OK;
}

static void emit_stage_pass(const char *stage,
                            float progress,
                            const motion_stage_result_t *result,
                            const char *axis)
{
    char buf[320];
    const char *safe_axis = (axis != NULL) ? axis : "";
    ESP_LOGI(TAG, "C3_STAGE stage=%s phase=pass progress=%.2f axis=%s samples=%d peak=%.2f f95=%.2f",
             stage, (double)progress, safe_axis, result->samples,
             (double)result->peak, (double)result->f95);
    uart_sendf(buf, sizeof(buf),
               "{\"cal\":\"C3_STAGE\",\"stage\":\"%s\",\"phase\":\"pass\","
               "\"progress\":%.3f,\"checkpoint\":\"return_center\",\"axis\":\"%s\","
               "\"samples\":%d,\"peak\":%.2f,\"f95\":%.2f}\r\n",
               stage, (double)progress, safe_axis, result->samples,
               (double)result->peak, (double)result->f95);
}

static void compute_direction_deltas(const raw_sample_t *samp,
                                     const cal_params_t *params,
                                     float deltas[3])
{
    deltas[0] = angle_diff_deg(samp->euler_deg[0], params->euler_neutral[0]);
    deltas[1] = angle_diff_deg(samp->euler_deg[1], params->euler_neutral[1]);
    deltas[2] = angle_diff_deg(samp->euler_deg[2], params->euler_neutral[2]);
}

static const char *direction_axis_name(int axis_index)
{
    switch (axis_index) {
    case 0: return "roll";
    case 1: return "pitch";
    case 2: return "yaw";
    default: return "unknown";
    }
}

static float __attribute__((unused)) stage_target_magnitude(const motion_stage_result_t *result)
{
    if (result->axis_index >= 0 && result->axis_index < 3) {
        return fabsf(result->target_offset[result->axis_index]);
    }
    return fabsf(result->peak);
}

static int strongest_direction_axis(const float deltas[3])
{
    int axis = 0;
    float best_mag = fabsf(deltas[0]);
    for (int i = 1; i < 3; i++) {
        float mag = fabsf(deltas[i]);
        if (mag > best_mag) {
            best_mag = mag;
            axis = i;
        }
    }
    return axis;
}

static int strongest_direction_axis_from_center(const float deltas[3], const float center[3])
{
    float centered[3] = {
        deltas[0] - center[0],
        deltas[1] - center[1],
        deltas[2] - center[2],
    };
    return strongest_direction_axis(centered);
}

static esp_err_t finish_direction_stage(const float *omega_window,
                                        const float *roll_window,
                                        const float *pitch_window,
                                        int samples,
                                        float peak,
                                        float force_sum,
                                        const float target_offset[3],
                                        const float return_center[3],
                                        float target_stability_deg,
                                        float return_stability_deg,
                                        int axis_index,
                                        int sign,
                                        motion_stage_result_t *out,
                                        const char **reason)
{
    if (samples < 80) {
        *reason = "motion_too_short";
        return ESP_FAIL;
    }
    if (peak < CAL_STAGE_TARGET_MIN_DEG) {
        *reason = "range_too_small";
        return ESP_FAIL;
    }

    float roll_min = roll_window[0], roll_max = roll_window[0];
    float pitch_min = pitch_window[0], pitch_max = pitch_window[0];
    for (int i = 1; i < samples; i++) {
        if (roll_window[i] < roll_min) roll_min = roll_window[i];
        if (roll_window[i] > roll_max) roll_max = roll_window[i];
        if (pitch_window[i] < pitch_min) pitch_min = pitch_window[i];
        if (pitch_window[i] > pitch_max) pitch_max = pitch_window[i];
    }

    out->peak = peak;
    out->pp_roll = roll_max - roll_min;
    out->pp_pitch = pitch_max - pitch_min;
    out->f95 = processing_compute_f95_window(omega_window, samples);
    out->mean_force = (samples > 0) ? (force_sum / (float)samples) : 0.0f;
    memcpy(out->target_offset, target_offset, sizeof(out->target_offset));
    memcpy(out->return_center, return_center, sizeof(out->return_center));
    out->target_stability_deg = target_stability_deg;
    out->return_stability_deg = return_stability_deg;
    out->axis_index = axis_index;
    out->sign = sign;
    out->samples = samples;
    *reason = "";
    return ESP_OK;
}

static esp_err_t __attribute__((unused)) run_direction_stage(QueueHandle_t raw_q,
                                                             const cal_params_t *params,
                                                             const motion_stage_def_t *def,
                                                             int forced_axis,
                                                             int forced_sign,
                                                             direction_context_t *resolved_ctx,
                                                             motion_stage_result_t *out)
{
    float *omega_window = s_cal_omega_window;
    float *roll_window = s_cal_roll_window;
    float *pitch_window = s_cal_pitch_window;
    float force_sum = 0.0f;
    float peak = 0.0f;
    int active_axis = forced_axis;
    int motion_sign = forced_sign;
    int samples = 0;
    int target_hold_count = 0;
    int return_hold_count = 0;
    bool started = false;
    bool target_captured = false;
    bool target_checkpoint_sent = false;
    bool return_checkpoint_sent = false;
    bool center_captured = false;
    int center_samples = 0;
    float center_sum[3] = {0.0f, 0.0f, 0.0f};
    float center_deltas[3] = {0.0f, 0.0f, 0.0f};
    float target_sum[3] = {0.0f, 0.0f, 0.0f};
    float target_axis_min = 0.0f, target_axis_max = 0.0f;
    float target_offset[3] = {0.0f, 0.0f, 0.0f};
    float target_stability = 0.0f;
    float return_sum[3] = {0.0f, 0.0f, 0.0f};
    float return_axis_min = 0.0f, return_axis_max = 0.0f;
    float return_center[3] = {0.0f, 0.0f, 0.0f};
    float return_stability = 0.0f;
    const char *reason = "timeout";

    emit_stage_event(def->name, "prompt", 0.0f, "", "", "");

    for (int i = 0; i < def->timeout_samples; i++) {
        raw_sample_t samp;
        if (xQueueReceive(raw_q, &samp, pdMS_TO_TICKS(CAL_QUEUE_TIMEOUT_MS)) != pdTRUE) {
            emit_stage_event(def->name, "fail", 0.0f, "timeout", "", "");
            return ESP_ERR_TIMEOUT;
        }

        float deltas[3];
        compute_direction_deltas(&samp, params, deltas);

        if (!center_captured) {
            float gx = samp.gyro_dps[0] - params->gyro_bias[0];
            float gy = samp.gyro_dps[1] - params->gyro_bias[1];
            float gz = samp.gyro_dps[2] - params->gyro_bias[2];
            float omega_mag = sqrtf(gx * gx + gy * gy + gz * gz);
            if (omega_mag <= CAL_STAGE_CENTER_OMEGA_DPS) {
                center_sum[0] += deltas[0];
                center_sum[1] += deltas[1];
                center_sum[2] += deltas[2];
                center_samples++;
                center_deltas[0] = center_sum[0] / (float)center_samples;
                center_deltas[1] = center_sum[1] / (float)center_samples;
                center_deltas[2] = center_sum[2] / (float)center_samples;
                if (center_samples >= CAL_STAGE_CENTER_SAMPLES) {
                    center_captured = true;
                    ESP_LOGI(TAG, "C3 stage=%s center locked roll=%.2f pitch=%.2f yaw=%.2f",
                             def->name,
                             (double)center_deltas[0],
                             (double)center_deltas[1],
                             (double)center_deltas[2]);
                }
            } else {
                center_samples = 0;
                center_sum[0] = center_sum[1] = center_sum[2] = 0.0f;
            }
        }

        if (!center_captured) {
            if (i % CAL_STAGE_LIVE_STRIDE == 0) {
                float progress = 0.33f * ((float)center_samples / (float)CAL_STAGE_CENTER_SAMPLES);
                if (progress > 0.32f) progress = 0.32f;
                emit_stage_event(def->name, "capturing_center", progress, "", "", "");
            }
            continue;
        }

        if (center_captured && !started && !target_checkpoint_sent) {
            target_checkpoint_sent = true;
            emit_stage_event(def->name, "center_captured", 0.33f, "", "center_start", "");
        }

        int best_axis = (active_axis >= 0 && active_axis < 3) ?
                        active_axis : strongest_direction_axis_from_center(deltas, center_deltas);
        float best_delta = deltas[best_axis];
        float best_centered_delta = best_delta - center_deltas[best_axis];
        float best_mag = fabsf(best_centered_delta);

        if (active_axis < 0 && best_mag >= CAL_STAGE_SIGN_DISCOVERY) {
            active_axis = best_axis;
            if (motion_sign == 0) {
                motion_sign = (best_centered_delta >= 0.0f) ? 1 : -1;
            }
            ESP_LOGI(TAG, "C3 stage=%s locked axis=%s sign=%d",
                     def->name, direction_axis_name(active_axis), motion_sign);
        }

        float axis_delta = (active_axis >= 0 && active_axis < 3) ?
                           deltas[active_axis] : best_delta;
        float axis_center = (active_axis >= 0 && active_axis < 3) ?
                            center_deltas[active_axis] : 0.0f;
        float axis_from_center = axis_delta - axis_center;
        if (motion_sign == 0 && fabsf(axis_from_center) >= CAL_STAGE_SIGN_DISCOVERY) {
            motion_sign = (axis_from_center >= 0.0f) ? 1 : -1;
        }

        float signed_delta = (motion_sign == 0) ? axis_from_center : (float)motion_sign * axis_from_center;
        float gx = samp.gyro_dps[0] - params->gyro_bias[0];
        float gy = samp.gyro_dps[1] - params->gyro_bias[1];
        float gz = samp.gyro_dps[2] - params->gyro_bias[2];
        float omega_mag = sqrtf(gx * gx + gy * gy + gz * gz);
        if (!started && signed_delta >= CAL_STAGE_MOTION_START) {
            started = true;
        }
        if (!started) {
            if (i % CAL_STAGE_LIVE_STRIDE == 0) {
                float progress = 0.33f + 0.20f * (((active_axis >= 0) ? fabsf(axis_from_center) : best_mag) / CAL_STAGE_TARGET_MIN_DEG);
                if (progress > 0.52f) progress = 0.52f;
                const char *axis_name = (active_axis >= 0 && active_axis < 3) ?
                                        direction_axis_name(active_axis) : "";
                emit_stage_event(def->name, "moving_to_target", progress, "", "", axis_name);
            }
            continue;
        }

        if (samples < CAL_DIRECTION_WINDOW_SAMPLES) {
            omega_window[samples] = sqrtf(gx * gx + gy * gy + gz * gz);
            roll_window[samples] = samp.euler_deg[0];
            pitch_window[samples] = samp.euler_deg[1];
            force_sum += samp.fsr_n[0] + samp.fsr_n[1] + samp.fsr_n[2];
            samples++;
        }

        if (signed_delta > peak) {
            peak = signed_delta;
        }
        if (!target_captured) {
            bool hold_candidate = signed_delta >= CAL_STAGE_TARGET_MIN_DEG &&
                                  omega_mag <= CAL_STAGE_HOLD_OMEGA_DPS;
            if (hold_candidate) {
                if (target_hold_count == 0) {
                    target_axis_min = signed_delta;
                    target_axis_max = signed_delta;
                    memset(target_sum, 0, sizeof(target_sum));
                }
                if (signed_delta < target_axis_min) target_axis_min = signed_delta;
                if (signed_delta > target_axis_max) target_axis_max = signed_delta;
                if ((target_axis_max - target_axis_min) > CAL_STAGE_HOLD_BAND_DEG) {
                    target_hold_count = 0;
                    target_axis_min = signed_delta;
                    target_axis_max = signed_delta;
                    memset(target_sum, 0, sizeof(target_sum));
                }
                target_sum[0] += deltas[0];
                target_sum[1] += deltas[1];
                target_sum[2] += deltas[2];
                target_hold_count++;
                if (target_hold_count >= CAL_STAGE_TARGET_HOLD_SAMPLES) {
                    target_captured = true;
                    target_stability = target_axis_max - target_axis_min;
                    for (int j = 0; j < 3; j++) {
                        target_offset[j] = (target_sum[j] / (float)target_hold_count) - center_deltas[j];
                    }
                    emit_stage_event(def->name, "target_captured", 0.66f, "", "target",
                                     direction_axis_name(active_axis));
                }
            } else {
                target_hold_count = 0;
                memset(target_sum, 0, sizeof(target_sum));
            }
        } else {
            bool return_candidate = fabsf(axis_from_center) <= CAL_STAGE_RETURN_BAND_DEG &&
                                    omega_mag <= CAL_STAGE_HOLD_OMEGA_DPS;
            if (return_candidate) {
                if (return_hold_count == 0) {
                    return_axis_min = axis_from_center;
                    return_axis_max = axis_from_center;
                    memset(return_sum, 0, sizeof(return_sum));
                }
                if (axis_from_center < return_axis_min) return_axis_min = axis_from_center;
                if (axis_from_center > return_axis_max) return_axis_max = axis_from_center;
                if ((return_axis_max - return_axis_min) > CAL_STAGE_HOLD_BAND_DEG) {
                    return_hold_count = 0;
                    return_axis_min = axis_from_center;
                    return_axis_max = axis_from_center;
                    memset(return_sum, 0, sizeof(return_sum));
                }
                return_sum[0] += deltas[0];
                return_sum[1] += deltas[1];
                return_sum[2] += deltas[2];
                return_hold_count++;
            } else {
                return_hold_count = 0;
                memset(return_sum, 0, sizeof(return_sum));
            }
        }

        if (i % CAL_STAGE_LIVE_STRIDE == 0) {
            float progress;
            const char *phase;
            const char *checkpoint = "";
            if (!target_captured) {
                float depart_prog = peak / CAL_STAGE_TARGET_MIN_DEG;
                if (depart_prog > 1.0f) depart_prog = 1.0f;
                float hold_prog = (float)target_hold_count / (float)CAL_STAGE_TARGET_HOLD_SAMPLES;
                if (hold_prog > 1.0f) hold_prog = 1.0f;
                progress = 0.33f + 0.18f * depart_prog + 0.15f * hold_prog;
                if (progress > 0.65f) progress = 0.65f;
                phase = "moving_to_target";
                checkpoint = "center_start";
            } else {
                float return_prog = (float)return_hold_count / (float)CAL_STAGE_RETURN_HOLD_SAMPLES;
                if (return_prog > 1.0f) return_prog = 1.0f;
                progress = 0.66f + 0.34f * return_prog;
                if (progress > 0.99f) progress = 0.99f;
                phase = "returning";
                checkpoint = "target";
            }
            emit_stage_event(def->name, phase, progress, "", checkpoint,
                             direction_axis_name(active_axis));
        }

        if (target_captured && return_hold_count >= CAL_STAGE_RETURN_HOLD_SAMPLES) {
            return_stability = return_axis_max - return_axis_min;
            for (int j = 0; j < 3; j++) {
                return_center[j] = return_sum[j] / (float)return_hold_count;
            }
            esp_err_t ret = finish_direction_stage(omega_window, roll_window, pitch_window,
                                                   samples, peak, force_sum,
                                                   target_offset, return_center,
                                                   target_stability, return_stability,
                                                   active_axis, motion_sign, out, &reason);
            if (ret == ESP_OK) {
                if (resolved_ctx != NULL) {
                    resolved_ctx->axis_index = active_axis;
                    resolved_ctx->sign = motion_sign;
                }
                if (!return_checkpoint_sent) {
                    return_checkpoint_sent = true;
                    emit_stage_event(def->name, "return_center_captured", 1.0f, "", "return_center",
                                     direction_axis_name(active_axis));
                }
                ESP_LOGI(TAG, "C3 stage=%s complete axis=%s sign=%d peak=%.2f",
                         def->name, direction_axis_name(active_axis), motion_sign, (double)peak);
                emit_stage_pass(def->name, 1.0f, out, direction_axis_name(active_axis));
            } else {
                emit_stage_event(def->name, "fail", 1.0f, reason, "", direction_axis_name(active_axis));
            }
            return ret;
        }
    }

    if (peak < CAL_STAGE_TARGET_MIN_DEG) {
        reason = "range_too_small";
    } else if (!target_captured) {
        reason = "motion_too_noisy";
    } else if (samples < 80) {
        reason = "motion_too_short";
    } else if (target_captured && return_hold_count < CAL_STAGE_RETURN_HOLD_SAMPLES) {
        reason = "returned_too_little";
    }
    emit_stage_event(def->name, "fail", 1.0f, reason, target_captured ? "target" : "",
                     direction_axis_name(active_axis));
    return ESP_FAIL;
}

static int derivative_turns(const float *values, int n)
{
    int turns = 0;
    int prev_sign = 0;
    for (int i = 1; i < n; i++) {
        float d = values[i] - values[i - 1];
        int sign = 0;
        if (d > 0.6f) sign = 1;
        else if (d < -0.6f) sign = -1;
        if (sign == 0) {
            continue;
        }
        if (prev_sign != 0 && sign != prev_sign) {
            turns++;
        }
        prev_sign = sign;
    }
    return turns;
}

static esp_err_t __attribute__((unused)) run_figure8_stage(QueueHandle_t raw_q,
                                                           const cal_params_t *params,
                                                           const motion_stage_def_t *def,
                                                           motion_stage_result_t *out)
{
    float *omega_window = s_cal_omega_window;
    float *yaw_window = s_cal_yaw_window;
    float *pitch_window = s_cal_pitch_window;
    float *roll_window = s_cal_roll_window;
    float force_sum = 0.0f;
    int samples = 0;
    bool started = false;
    float yaw_min = 0.0f, yaw_max = 0.0f, pitch_min = 0.0f, pitch_max = 0.0f;
    const char *reason = "timeout";

    emit_stage_event(def->name, "prompt", 0.0f, "", "", "");

    for (int i = 0; i < def->timeout_samples; i++) {
        raw_sample_t samp;
        if (xQueueReceive(raw_q, &samp, pdMS_TO_TICKS(CAL_QUEUE_TIMEOUT_MS)) != pdTRUE) {
            emit_stage_event(def->name, "fail", 0.0f, "timeout", "", "");
            return ESP_ERR_TIMEOUT;
        }

        float yaw_delta = angle_diff_deg(samp.euler_deg[2], params->euler_neutral[2]);
        float pitch_delta = angle_diff_deg(samp.euler_deg[1], params->euler_neutral[1]);
        float motion_mag = sqrtf(yaw_delta * yaw_delta + pitch_delta * pitch_delta);

        if (!started && motion_mag >= 8.0f) {
            started = true;
            yaw_min = yaw_max = yaw_delta;
            pitch_min = pitch_max = pitch_delta;
        }
        if (!started) {
            if (i % CAL_STAGE_LIVE_STRIDE == 0) {
                float progress = motion_mag / 20.0f;
                if (progress > 0.18f) progress = 0.18f;
                emit_stage_event(def->name, "moving_to_target", progress, "", "", "");
            }
            continue;
        }

        if (samples < CAL_FIG8_WINDOW_SAMPLES) {
            float gx = samp.gyro_dps[0] - params->gyro_bias[0];
            float gy = samp.gyro_dps[1] - params->gyro_bias[1];
            float gz = samp.gyro_dps[2] - params->gyro_bias[2];
            omega_window[samples] = sqrtf(gx * gx + gy * gy + gz * gz);
            yaw_window[samples] = yaw_delta;
            pitch_window[samples] = pitch_delta;
            roll_window[samples] = samp.euler_deg[0];
            force_sum += samp.fsr_n[0] + samp.fsr_n[1] + samp.fsr_n[2];
            samples++;
        }

        if (yaw_delta < yaw_min) yaw_min = yaw_delta;
        if (yaw_delta > yaw_max) yaw_max = yaw_delta;
        if (pitch_delta < pitch_min) pitch_min = pitch_delta;
        if (pitch_delta > pitch_max) pitch_max = pitch_delta;

        float yaw_pp = yaw_max - yaw_min;
        float pitch_pp = pitch_max - pitch_min;

        if (i % CAL_STAGE_LIVE_STRIDE == 0) {
            float time_prog = (float)samples / (float)def->min_samples;
            if (time_prog > 1.0f) time_prog = 1.0f;
            float yaw_prog = yaw_pp / CAL_STAGE_FIG8_PP_MIN_DEG;
            if (yaw_prog > 1.0f) yaw_prog = 1.0f;
            float pitch_prog = pitch_pp / CAL_STAGE_FIG8_PP_MIN_DEG;
            if (pitch_prog > 1.0f) pitch_prog = 1.0f;
            int yaw_turns = derivative_turns(yaw_window, samples);
            int pitch_turns = derivative_turns(pitch_window, samples);
            float turn_prog = ((float)yaw_turns + (float)pitch_turns) / 6.0f;
            if (turn_prog > 1.0f) turn_prog = 1.0f;
            float progress = 0.40f * time_prog + 0.25f * yaw_prog +
                             0.25f * pitch_prog + 0.10f * turn_prog;
            if (progress > 0.99f) progress = 0.99f;
            emit_stage_event(def->name, "moving_to_target", progress, "", "", "");
        }

        if (samples >= def->min_samples) {
            int yaw_turns = derivative_turns(yaw_window, samples);
            int pitch_turns = derivative_turns(pitch_window, samples);
            if (yaw_pp >= CAL_STAGE_FIG8_PP_MIN_DEG &&
                pitch_pp >= CAL_STAGE_FIG8_PP_MIN_DEG &&
                yaw_turns >= 2 &&
                pitch_turns >= 2) {
                float roll_min = roll_window[0], roll_max = roll_window[0];
                for (int k = 1; k < samples; k++) {
                    if (roll_window[k] < roll_min) roll_min = roll_window[k];
                    if (roll_window[k] > roll_max) roll_max = roll_window[k];
                }
                out->peak = (yaw_pp > pitch_pp) ? yaw_pp : pitch_pp;
                out->pp_roll = roll_max - roll_min;
                out->pp_pitch = pitch_pp;
                out->f95 = processing_compute_f95_window(omega_window, samples);
                out->mean_force = force_sum / (float)samples;
                out->samples = samples;
                if (out->f95 > CAL_STAGE_FIG8_F95_MAX_HZ) {
                    emit_stage_event(def->name, "fail", 1.0f, "motion_too_noisy", "", "");
                    return ESP_FAIL;
                }
                emit_stage_pass(def->name, 1.0f, out, "");
                return ESP_OK;
            }
        }
    }

    if (samples < def->min_samples) {
        reason = "motion_too_short";
    } else {
        float yaw_pp = yaw_max - yaw_min;
        float pitch_pp = pitch_max - pitch_min;
        if (yaw_pp < CAL_STAGE_FIG8_PP_MIN_DEG || pitch_pp < CAL_STAGE_FIG8_PP_MIN_DEG) {
            reason = "range_too_small";
        } else {
            reason = "motion_too_noisy";
        }
    }
    emit_stage_event(def->name, "fail", 1.0f, reason, "", "");
    return ESP_FAIL;
}

static esp_err_t cal_c1(QueueHandle_t raw_q, cal_params_t *params)
{
    const int N = CAL_C1_SAMPLES;
    welford_t wf_omega, wf_roll, wf_pitch, wf_yaw, wf_tremor, wf_quat[4], wf_g[3];
    welford_reset(&wf_omega);
    welford_reset(&wf_roll);
    welford_reset(&wf_pitch);
    welford_reset(&wf_yaw);
    welford_reset(&wf_tremor);
    for (int i = 0; i < 4; i++) welford_reset(&wf_quat[i]);
    for (int i = 0; i < 3; i++) welford_reset(&wf_g[i]);

    drain_queue(raw_q);
    uart_send("{\"cal\":\"C1_START\"}\r\n");

    char buf[256];
    for (int i = 0; i < N; i++) {
        raw_sample_t samp;
        if (xQueueReceive(raw_q, &samp, pdMS_TO_TICKS(CAL_QUEUE_TIMEOUT_MS)) != pdTRUE) {
            uart_send("{\"cal\":\"C1\",\"status\":\"FAIL\",\"reason\":\"timeout\"}\r\n");
            return ESP_ERR_TIMEOUT;
        }
        s_c1_samples[i] = samp;

        float gx = samp.gyro_dps[0];
        float gy = samp.gyro_dps[1];
        float gz = samp.gyro_dps[2];
        float omega = sqrtf(gx * gx + gy * gy + gz * gz);
        welford_update(&wf_omega, omega);
        welford_update(&wf_g[0], gx);
        welford_update(&wf_g[1], gy);
        welford_update(&wf_g[2], gz);
        welford_update(&wf_roll, samp.euler_deg[0]);
        welford_update(&wf_pitch, samp.euler_deg[1]);
        welford_update(&wf_yaw, samp.euler_deg[2]);
        for (int k = 0; k < 4; k++) {
            welford_update(&wf_quat[k], samp.quat[k]);
        }
        if (i % 10 == 0) {
            uart_sendf(buf, sizeof(buf),
                       "{\"cal\":\"C1_LIVE\",\"omega\":%.3f}\r\n", (double)omega);
        }
    }

    float rms_omega = wf_omega.mean;
    float spread = maxf3(welford_stddev(&wf_roll),
                         welford_stddev(&wf_pitch),
                         welford_stddev(&wf_yaw));
    float gyro_bias[3] = { wf_g[0].mean, wf_g[1].mean, wf_g[2].mean };

    processing_tremor_baseline_reset();
    for (int i = 0; i < N; i++) {
        float unbiased[3] = {
            s_c1_samples[i].gyro_dps[0] - gyro_bias[0],
            s_c1_samples[i].gyro_dps[1] - gyro_bias[1],
            s_c1_samples[i].gyro_dps[2] - gyro_bias[2],
        };
        float tremor_mag = processing_tremor_baseline_step(unbiased);
        if (i >= 50) {
            welford_update(&wf_tremor, tremor_mag);
        }
    }
    float tremor_baseline = wf_tremor.mean + 3.0f * welford_stddev(&wf_tremor);
    if (tremor_baseline < TREMOR_BASELINE_FLOOR_DPS) {
        tremor_baseline = TREMOR_BASELINE_FLOOR_DPS;
    }

    params->gyro_bias[0] = gyro_bias[0];
    params->gyro_bias[1] = gyro_bias[1];
    params->gyro_bias[2] = gyro_bias[2];
    params->tremor_rms_ref = tremor_baseline;
    params->euler_neutral[0] = wf_roll.mean;
    params->euler_neutral[1] = wf_pitch.mean;
    params->euler_neutral[2] = wf_yaw.mean;
    for (int k = 0; k < 4; k++) {
        params->q_neutral[k] = wf_quat[k].mean;
    }
    if (fabsf(params->q_neutral[0]) < 1e-6f &&
        fabsf(params->q_neutral[1]) < 1e-6f &&
        fabsf(params->q_neutral[2]) < 1e-6f &&
        fabsf(params->q_neutral[3]) < 1e-6f) {
        params->q_neutral[0] = 1.0f;
    }
    reset_c3_progress(params);

    const char *reason = c1_reason(rms_omega, spread);
    bool pass = (reason[0] == '\0');
    uart_sendf(buf, sizeof(buf),
               "{\"cal\":\"C1\",\"status\":\"%s\",\"reason\":\"%s\","
               "\"rms_omega\":%.3f,\"spread\":%.3f,\"tremor_ref\":%.3f}\r\n",
               pass ? "PASS" : "FAIL", reason,
               (double)rms_omega, (double)spread, (double)tremor_baseline);
    return pass ? ESP_OK : ESP_FAIL;
}

static esp_err_t cal_c2(QueueHandle_t raw_q, cal_params_t *params)
{
    const int N = CAL_C2_SAMPLES;
    welford_t wf[3];
    float first_sum[3] = {0.0f, 0.0f, 0.0f};
    float last_sum[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; i++) {
        welford_reset(&wf[i]);
    }

    drain_queue(raw_q);
    uart_send("{\"cal\":\"C2_START\"}\r\n");

    char buf[320];
    for (int s = 0; s < N; s++) {
        raw_sample_t samp;
        if (xQueueReceive(raw_q, &samp, pdMS_TO_TICKS(CAL_QUEUE_TIMEOUT_MS)) != pdTRUE) {
            uart_send("{\"cal\":\"C2\",\"status\":\"FAIL\",\"reason\":\"timeout\"}\r\n");
            return ESP_ERR_TIMEOUT;
        }

        for (int i = 0; i < 3; i++) {
            welford_update(&wf[i], samp.fsr_n[i]);
            if (s < 50) {
                first_sum[i] += samp.fsr_n[i];
            }
            if (s >= (N - 50)) {
                last_sum[i] += samp.fsr_n[i];
            }
        }

        if (s % 10 == 0) {
            uart_sendf(buf, sizeof(buf),
                       "{\"cal\":\"C2_LIVE\",\"f\":[%.4f,%.4f,%.4f]}\r\n",
                       (double)samp.fsr_n[0], (double)samp.fsr_n[1], (double)samp.fsr_n[2]);
        }
    }

    bool pressure_present = false;
    bool unstable = false;
    bool drifting = false;
    for (int i = 0; i < 3; i++) {
        params->mu[i] = wf[i].mean;
        params->sigma[i] = welford_stddev(&wf[i]);
        float on_cand = wf[i].mean + 5.0f * params->sigma[i];
        float off_cand = wf[i].mean + 3.0f * params->sigma[i];
        params->on_thresh[i] = (on_cand > 0.30f) ? on_cand : 0.30f;
        params->off_thresh[i] = (off_cand > 0.15f) ? off_cand : 0.15f;

        if (params->mu[i] > 0.20f) {
            pressure_present = true;
        }
        if (params->sigma[i] > 0.08f) {
            unstable = true;
        }
        float drift = fabsf((last_sum[i] / 50.0f) - (first_sum[i] / 50.0f));
        if (drift > 0.05f) {
            drifting = true;
        }
    }
    reset_c3_progress(params);

    const char *reason = c2_reason(pressure_present, unstable, drifting);
    bool pass = (reason[0] == '\0');
    uart_sendf(buf, sizeof(buf),
               "{\"cal\":\"C2\",\"status\":\"%s\",\"reason\":\"%s\","
               "\"mu\":[%.4f,%.4f,%.4f],"
               "\"sigma\":[%.4f,%.4f,%.4f],"
               "\"on_th\":[%.4f,%.4f,%.4f],"
               "\"off_th\":[%.4f,%.4f,%.4f]}\r\n",
               pass ? "PASS" : "FAIL", reason,
               (double)params->mu[0], (double)params->mu[1], (double)params->mu[2],
               (double)params->sigma[0], (double)params->sigma[1], (double)params->sigma[2],
               (double)params->on_thresh[0], (double)params->on_thresh[1], (double)params->on_thresh[2],
               (double)params->off_thresh[0], (double)params->off_thresh[1], (double)params->off_thresh[2]);

    return pass ? ESP_OK : ESP_FAIL;
}

static esp_err_t cal_c3_grip(QueueHandle_t raw_q, cal_params_t *params)
{
    float grip_force = 0.0f;
    char buf[256];

    drain_queue(raw_q);
    reset_c3_progress(params);

    ESP_LOGI(TAG, "C3 grip capture starting");
    uart_send("{\"cal\":\"C3_START\",\"stages\":[\"grip_hold\"]}\r\n");

    esp_err_t ret = run_c3_grip_hold(raw_q, params, &grip_force);
    if (ret != ESP_OK) {
        uart_send("{\"cal\":\"C3\",\"status\":\"FAIL\",\"reason\":\"stage_failed\",\"stage\":\"grip_hold\"}\r\n");
        return ret;
    }

    params->f_ref_open = grip_force;
    if (params->f_ref_open < CAL_STAGE_FORCE_MIN_N) {
        uart_send("{\"cal\":\"C3\",\"status\":\"FAIL\",\"reason\":\"pressure_absent\"}\r\n");
        return ESP_FAIL;
    }
    uart_sendf(buf, sizeof(buf),
               "{\"cal\":\"C3\",\"status\":\"PASS\",\"f_ref_open\":%.4f}\r\n",
               (double)params->f_ref_open);

    esp_err_t save_ret = nvs_save_calibration(params);
    uart_sendf(buf, sizeof(buf),
               "{\"cal\":\"COMPLETE\",\"status\":\"%s\",\"score\":%d}\r\n",
               (save_ret == ESP_OK) ? "PASS" : "FAIL",
               (save_ret == ESP_OK) ? 100 : 90);
    return (save_ret == ESP_OK) ? ESP_OK : save_ret;
}

static esp_err_t cal_c4_motion(QueueHandle_t raw_q, cal_params_t *params)
{
    float f95_ref = 0.0f;
    float pp_roll_ref = 0.0f;
    float pp_pitch_ref = 0.0f;
    float motion_rms_ref = 0.0f;
    float motion_tremor_ratio_ref = 0.0f;
    char buf[256];

    drain_queue(raw_q);

    ESP_LOGI(TAG, "C4 normal hand motion capture starting");
    log_stack_watermark("C4 start");
    uart_send("{\"cal\":\"C4_START\",\"stages\":[\"hand_motion\"]}\r\n");

    esp_err_t ret = run_c4_hand_motion(raw_q, params,
                                       &f95_ref, &pp_roll_ref, &pp_pitch_ref,
                                       &motion_rms_ref, &motion_tremor_ratio_ref);
    if (ret != ESP_OK) {
        uart_send("{\"cal\":\"C4\",\"status\":\"FAIL\",\"reason\":\"stage_failed\",\"stage\":\"hand_motion\"}\r\n");
        return ret;
    }

    params->f95_ref = f95_ref;
    params->pp_roll_ref = pp_roll_ref;
    params->pp_pitch_ref = pp_pitch_ref;
    params->motion_rms_ref = motion_rms_ref;
    params->motion_tremor_ratio_ref = motion_tremor_ratio_ref;

    uart_sendf(buf, sizeof(buf),
               "{\"cal\":\"C4\",\"status\":\"PASS\","
               "\"motion_rms_ref\":%.2f,\"motion_tremor_ratio_ref\":%.3f,"
               "\"f95_ref\":%.2f,\"pp_roll_ref\":%.2f,\"pp_pitch_ref\":%.2f}\r\n",
               (double)params->motion_rms_ref,
               (double)params->motion_tremor_ratio_ref,
               (double)params->f95_ref,
               (double)params->pp_roll_ref,
               (double)params->pp_pitch_ref);

    esp_err_t save_ret = nvs_save_calibration(params);
    uart_sendf(buf, sizeof(buf),
               "{\"cal\":\"COMPLETE\",\"status\":\"%s\",\"score\":%d}\r\n",
               (save_ret == ESP_OK) ? "PASS" : "FAIL",
               (save_ret == ESP_OK) ? 100 : 90);
    return (save_ret == ESP_OK) ? ESP_OK : save_ret;
}

esp_err_t calibration_run(char step, QueueHandle_t raw_q, cal_params_t *params)
{
    ESP_LOGI(TAG, "Starting calibration step C%c", step);

    switch (step) {
    case '1':
        return cal_c1(raw_q, params);
    case '2':
        return cal_c2(raw_q, params);
    case '3':
        return cal_c3_grip(raw_q, params);
    case '4':
        return cal_c4_motion(raw_q, params);
    default:
        uart_send("{\"cal\":\"ERROR\",\"reason\":\"unknown_step\"}\r\n");
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t calibration_c3_rep(QueueHandle_t raw_q, cal_params_t *params)
{
    (void)raw_q;
    (void)params;
    uart_send("{\"cal\":\"C3_REP\",\"status\":\"FAIL\",\"reason\":\"deprecated\"}\r\n");
    return ESP_ERR_INVALID_STATE;
}

esp_err_t calibration_c4_cycle(QueueHandle_t raw_q, cal_params_t *params)
{
    (void)raw_q;
    (void)params;
    uart_send("{\"cal\":\"C4_CYCLE\",\"status\":\"FAIL\",\"reason\":\"deprecated\"}\r\n");
    return ESP_ERR_INVALID_STATE;
}
