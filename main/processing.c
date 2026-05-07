#include "processing.h"
#include "filters.h"
#include "motor_control.h"
#include "led_status.h"
#include "calibration.h"
#include "nvs_storage.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
/* esp-dsp not required: f95 uses a self-contained DFT (see compute_f95 below) */

#include <math.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "PROC";

/* ── Module state ─────────────────────────────────────────────────────── */
static QueueHandle_t  s_raw_q;
static QueueHandle_t  s_uart_q;
static cal_params_t  *s_cal;

/* GPIO toggle state for Core 1 debug pin */
static int s_dbg_state = 0;

/* ── Filter states (one set per signal) ──────────────────────────────── */
/* FSR magnitude filter: LP 10 Hz, 3 channels */
static filter_state_t s_lp_fsr[3];
/* IMU stability filter: LP 12 Hz, 3 axes of omega */
static filter_state_t s_lp_omega[3];
/* Tremor filters: BP 6–12 Hz, one per gyro axis */
static filter_state_t s_bp_tremor[3];
/* Separate C1-baseline tremor filters so scoring filters stay independent. */
static filter_state_t s_bp_tremor_ref[3];
/* FSR derivative */
static diff5_state_t  s_diff5_fsr;

/* ── Contact detection state (per finger) ────────────────────────────── */
static bool    s_contact[3];
static uint8_t s_on_cnt[3];
static uint8_t s_off_cnt[3];

/* ── Board / state machine ───────────────────────────────────────────── */
static board_state_t s_state     = BOARD_IDLE;
static int           s_hold_cnt  = 0;
static bool          s_imu_fallback_engaged = false;
static int           s_imu_gate_on_cnt = 0;
static int           s_imu_gate_off_cnt = 0;

/* ── Sliding windows for threshold calculations ───────────────────────── */
#define WIN_250MS   25   /* 250 ms at 100 Hz */
#define WIN_1S     100   /* 1 s at 100 Hz */
#define IMU_GATE_ON_SAMPLES   15   /* 150 ms */
#define IMU_GATE_OFF_SAMPLES  25   /* 250 ms */

static float  s_omega_win[WIN_250MS];
static float  s_fsig_win[WIN_250MS];
static float  s_roll_win[WIN_250MS];
static float  s_pitch_win[WIN_250MS];
static float  s_gyro_axis_1s[3][WIN_1S];
static float  s_bp_omega_win[WIN_250MS]; /* bandpass-filtered omega_norm */
static int    s_win_idx   = 0;
static int    s_win_1s_idx = 0;
static int    s_win_filled = 0;
static int    s_win_1s_filled = 0;

/* ── DFT buffer for f95_omega ───────────────────────────────────────── */
/* 64-sample DFT over 0-20 Hz range (14 bins at 100 Hz / 64 = 1.5625 Hz/bin).
 * No external library needed — computed as real DFT directly. */
#define DFT_N    64
#define DFT_BINS 13   /* bins 0-12 cover 0-18.75 Hz */
/* Module-level to avoid blowing the 8 kB task stack */
static float  s_omega_dft_buf[DFT_N];
static int    s_fft_idx    = 0;
static int    s_fft_filled = 0;

/* ── Measured acquisition rate (updated by acquisition task log) ─────── */
static float s_actual_hz = 100.0f;

/* ── Active force thresholds (updated by E/M/H commands) ─────────────── */
static float s_warn_sum_n = FORCE_WARN_SUM_N;  /* 2.0 N default (medium) */
static float s_err_sum_n  = FORCE_ERR_SUM_N;   /* 4.0 N default (medium) */
static float s_warn_tremor_excess_dps = TREMOR_MED_WARN_EXCESS_DPS;
static float s_err_tremor_excess_dps  = TREMOR_MED_ERR_EXCESS_DPS;
static float s_warn_tremor_ratio = TREMOR_MED_WARN_RATIO;
static float s_err_tremor_ratio  = TREMOR_MED_ERR_RATIO;

/* ── Sustained error counters for force compression ──────────────────── */
static int s_warn_force_dur_cnt = 0;
static int s_err_force_dur_cnt  = 0;

/* ── Running score ───────────────────────────────────────────────────── */
static float s_score = 100.0f;

/* ── Helpers ────────────────────────────────────────────────────────── */
static void reset_session_metrics(void)
{
    s_score = 100.0f;
    s_state = BOARD_IDLE;
    s_hold_cnt = 0;
    s_imu_fallback_engaged = false;
    s_imu_gate_on_cnt = 0;
    s_imu_gate_off_cnt = 0;
    s_warn_force_dur_cnt = 0;
    s_err_force_dur_cnt = 0;
    memset(s_contact, 0, sizeof(s_contact));
    memset(s_on_cnt, 0, sizeof(s_on_cnt));
    memset(s_off_cnt, 0, sizeof(s_off_cnt));
    memset(s_omega_win, 0, sizeof(s_omega_win));
    memset(s_fsig_win, 0, sizeof(s_fsig_win));
    memset(s_roll_win, 0, sizeof(s_roll_win));
    memset(s_pitch_win, 0, sizeof(s_pitch_win));
    memset(s_gyro_axis_1s, 0, sizeof(s_gyro_axis_1s));
    memset(s_bp_omega_win, 0, sizeof(s_bp_omega_win));
    memset(s_omega_dft_buf, 0, sizeof(s_omega_dft_buf));
    s_win_idx = 0;
    s_win_1s_idx = 0;
    s_win_filled = 0;
    s_win_1s_filled = 0;
    s_fft_idx = 0;
    s_fft_filled = 0;
    for (int i = 0; i < 3; i++) {
        filter_lp_10hz_init(&s_lp_fsr[i]);
        filter_lp_12hz_init(&s_lp_omega[i]);
        filter_bp_6_12hz_init(&s_bp_tremor[i]);
    }
    filter_diff5_init(&s_diff5_fsr);
}

static float tremor_baseline_dps(void)
{
    float baseline = s_cal->tremor_rms_ref;
    if (baseline < TREMOR_BASELINE_FLOOR_DPS) {
        baseline = TREMOR_BASELINE_FLOOR_DPS;
    }
    return baseline;
}

static float motion_tremor_ratio_baseline(void)
{
    float baseline = s_cal->motion_tremor_ratio_ref;
    if (baseline < 0.0f) {
        baseline = 0.0f;
    }
    return baseline;
}

static float tremor_warn_ratio_threshold(void)
{
    float learned = motion_tremor_ratio_baseline() + 0.10f;
    return (learned > s_warn_tremor_ratio) ? learned : s_warn_tremor_ratio;
}

static float tremor_err_ratio_threshold(void)
{
    float learned = motion_tremor_ratio_baseline() + 0.20f;
    return (learned > s_err_tremor_ratio) ? learned : s_err_tremor_ratio;
}

static float window_rms(const float *w, int n)
{
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        sum += w[i] * w[i];
    }
    return sqrtf(sum / (float)n);
}

static float window_mean(const float *w, int n)
{
    float s = 0.0f;
    for (int i = 0; i < n; i++) {
        s += w[i];
    }
    return s / (float)n;
}

static float window_stddev(const float *w, int n)
{
    float m = window_mean(w, n);
    float v = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = w[i] - m;
        v += d * d;
    }
    return sqrtf(v / (float)n);
}

static float window_min(const float *w, int n)
{
    float mn = w[0];
    for (int i = 1; i < n; i++) {
        if (w[i] < mn) mn = w[i];
    }
    return mn;
}

static float window_max(const float *w, int n)
{
    float mx = w[0];
    for (int i = 1; i < n; i++) {
        if (w[i] > mx) mx = w[i];
    }
    return mx;
}

static float compute_swing_rate(void)
{
    int wn1s = s_win_1s_filled;
    if (wn1s <= 1) {
        return 0.0f;
    }

    int start = (wn1s < WIN_1S) ? 0 : s_win_1s_idx;
    float mean_abs[3] = {0.0f, 0.0f, 0.0f};
    for (int axis = 0; axis < 3; axis++) {
        for (int i = 0; i < wn1s; i++) {
            int idx = (start + i) % WIN_1S;
            mean_abs[axis] += fabsf(s_gyro_axis_1s[axis][idx]);
        }
        mean_abs[axis] /= (float)wn1s;
    }

    int dominant_axis = 0;
    if (mean_abs[1] > mean_abs[dominant_axis]) {
        dominant_axis = 1;
    }
    if (mean_abs[2] > mean_abs[dominant_axis]) {
        dominant_axis = 2;
    }

    int sign_changes = 0;
    int prev_sign = 0;
    for (int i = 0; i < wn1s; i++) {
        int idx = (start + i) % WIN_1S;
        float sample = s_gyro_axis_1s[dominant_axis][idx];
        int sign = 0;
        if (sample > SWING_SIGN_DEADBAND_DPS) {
            sign = 1;
        } else if (sample < -SWING_SIGN_DEADBAND_DPS) {
            sign = -1;
        }
        if (sign == 0) {
            continue;
        }
        if (prev_sign != 0 && sign != prev_sign) {
            sign_changes++;
        }
        prev_sign = sign;
    }

    return (float)sign_changes / ((float)wn1s / (float)SAMPLE_RATE_HZ);
}

/* ── Compute f95_omega via real DFT (no external library) ────────────── */
/*
 * DFT_BINS bins from 0 to (DFT_BINS-1) × freq_res cover 0–18.75 Hz at 100 Hz / 64.
 * O(N·K) = 64 × 13 = 832 multiplies — well within 10 ms processing budget.
 */
float processing_compute_f95_window(const float *omega_window, int n)
{
    if (omega_window == NULL || n < DFT_N) {
        return 0.0f;
    }

    const float *x = &omega_window[n - DFT_N];
    /* Re-order circular buffer into a contiguous sequence */
    float power[DFT_BINS];
    float total_power = 0.0f;
    float freq_res = (float)SAMPLE_RATE_HZ / (float)DFT_N; /* 1.5625 Hz/bin */

    for (int k = 0; k < DFT_BINS; k++) {
        float re = 0.0f, im = 0.0f;
        float angle_step = 2.0f * 3.14159265f * (float)k / (float)DFT_N;
        for (int n = 0; n < DFT_N; n++) {
            re += x[n] * cosf((float)n * angle_step);
            im -= x[n] * sinf((float)n * angle_step);
        }
        power[k] = (re * re + im * im) / (float)(DFT_N * DFT_N);
        total_power += power[k];
    }

    if (total_power < 1e-9f) {
        return 0.0f;
    }

    float cum    = 0.0f;
    float target = 0.95f * total_power;
    for (int k = 0; k < DFT_BINS; k++) {
        cum += power[k];
        if (cum >= target) {
            return (float)k * freq_res;
        }
    }
    return (float)(DFT_BINS - 1) * freq_res;
}

static float compute_f95(void)
{
    float x[DFT_N];
    for (int i = 0; i < DFT_N; i++) {
        x[i] = s_omega_dft_buf[(s_fft_idx + i) % DFT_N];
    }
    return processing_compute_f95_window(x, DFT_N);
}

/* ── Contact detection (per finger) ──────────────────────────────────── */
static void update_contact(int i, float fsr_filtered)
{
    if (fsr_filtered > s_cal->on_thresh[i]) {
        s_off_cnt[i] = 0;
        if (++s_on_cnt[i] >= 3) {
            s_contact[i] = true;
            s_on_cnt[i]  = 0;
        }
    } else if (fsr_filtered < s_cal->off_thresh[i]) {
        s_on_cnt[i] = 0;
        if (++s_off_cnt[i] >= 5) {
            s_contact[i] = false;
            s_off_cnt[i] = 0;
        }
    } else {
        s_on_cnt[i]  = 0;
        s_off_cnt[i] = 0;
    }
}

/* ── UART command dispatch ────────────────────────────────────────────── */
static void dispatch_command(const char *cmd, int len)
{
    if (len <= 0) {
        return;
    }

    char response[64];

    /* Multi-byte commands first */
    if (len >= 5 && strncmp(cmd, "C3_REP", 6) == 0) {
        led_status_blink_hz(30);
        calibration_c3_rep(s_raw_q, s_cal);
        return;
    }
    if (len >= 8 && strncmp(cmd, "C4_CYCLE", 8) == 0) {
        led_status_blink_hz(30);
        calibration_c4_cycle(s_raw_q, s_cal);
        return;
    }
    if (len >= 2 && cmd[0] == 'C' && cmd[1] >= '1' && cmd[1] <= '4') {
        led_status_blink_hz(30);
        calibration_run(cmd[1], s_raw_q, s_cal);
        return;
    }

    /* Single-byte commands */
    switch (cmd[0]) {
    case 'I':
        led_status_solid_on();
        uart_write_bytes(UART_NUM_0, "ESP32_TRAINER\r\n", 15);
        uart_write_bytes(UART_NUM_0, "READY: fw=v1.0 proto=cal-v3\r\n", 31);
        break;
    case 'S':
        led_status_off();
        s_state    = BOARD_IDLE;
        s_hold_cnt = 0;
        uart_write_bytes(UART_NUM_0, "STOPPED\r\n", 9);
        break;
    case 'E':
        led_status_blink_hz(1);
        reset_session_metrics();
        s_warn_sum_n = s_cal->f_ref_open * FORCE_EASY_WARN_X;
        s_err_sum_n  = s_cal->f_ref_open * FORCE_EASY_ERR_X;
        s_warn_tremor_excess_dps = TREMOR_EASY_WARN_EXCESS_DPS;
        s_err_tremor_excess_dps  = TREMOR_EASY_ERR_EXCESS_DPS;
        s_warn_tremor_ratio = TREMOR_EASY_WARN_RATIO;
        s_err_tremor_ratio  = TREMOR_EASY_ERR_RATIO;
        uart_write_bytes(UART_NUM_0, "EASY\r\n", 6);
        break;
    case 'M':
        led_status_blink_hz(2);
        reset_session_metrics();
        s_warn_sum_n = s_cal->f_ref_open * FORCE_MED_WARN_X;
        s_err_sum_n  = s_cal->f_ref_open * FORCE_MED_ERR_X;
        s_warn_tremor_excess_dps = TREMOR_MED_WARN_EXCESS_DPS;
        s_err_tremor_excess_dps  = TREMOR_MED_ERR_EXCESS_DPS;
        s_warn_tremor_ratio = TREMOR_MED_WARN_RATIO;
        s_err_tremor_ratio  = TREMOR_MED_ERR_RATIO;
        uart_write_bytes(UART_NUM_0, "INTERMEDIATE\r\n", 14);
        break;
    case 'H':
        led_status_blink_hz(4);
        reset_session_metrics();
        s_warn_sum_n = s_cal->f_ref_open * FORCE_HARD_WARN_X;
        s_err_sum_n  = s_cal->f_ref_open * FORCE_HARD_ERR_X;
        s_warn_tremor_excess_dps = TREMOR_HARD_WARN_EXCESS_DPS;
        s_err_tremor_excess_dps  = TREMOR_HARD_ERR_EXCESS_DPS;
        s_warn_tremor_ratio = TREMOR_HARD_WARN_RATIO;
        s_err_tremor_ratio  = TREMOR_HARD_ERR_RATIO;
        uart_write_bytes(UART_NUM_0, "HARD\r\n", 6);
        break;
    case 'X':
        led_status_off();
        uart_write_bytes(UART_NUM_0, "EXITED\r\n", 8);
        s_state = BOARD_EXITED;
        /* Suspend self — acquisition task continues, motors off */
        motor_control_init(); /* drives all motors LOW */
        vTaskSuspend(NULL);
        break;
    case 'Z':
        nvs_erase_calibration();
        nvs_get_defaults(s_cal);
        uart_write_bytes(UART_NUM_0, "{\"nvs\":\"ERASED\"}\r\n", 18);
        break;
    default: {
        int n = snprintf(response, sizeof(response),
                         "{\"err\":\"unknown_cmd\",\"cmd\":\"%c\"}\r\n", cmd[0]);
        uart_write_bytes(UART_NUM_0, response, (size_t)n);
        break;
    }
    }
}

/* ── Public API ────────────────────────────────────────────────────────── */

void processing_init(QueueHandle_t raw_q,
                     QueueHandle_t uart_event_q,
                     cal_params_t  *cal)
{
    s_raw_q  = raw_q;
    s_uart_q = uart_event_q;
    s_cal    = cal;

    /* Initialize all filter states */
    for (int i = 0; i < 3; i++) {
        filter_lp_10hz_init(&s_lp_fsr[i]);
        filter_lp_12hz_init(&s_lp_omega[i]);
    }
    for (int i = 0; i < 3; i++) {
        filter_bp_6_12hz_init(&s_bp_tremor[i]);
        filter_bp_6_12hz_init(&s_bp_tremor_ref[i]);
    }
    filter_diff5_init(&s_diff5_fsr);

    memset(s_contact,  0, sizeof(s_contact));
    memset(s_on_cnt,   0, sizeof(s_on_cnt));
    memset(s_off_cnt,  0, sizeof(s_off_cnt));
    memset(s_omega_win, 0, sizeof(s_omega_win));
    memset(s_fsig_win,  0, sizeof(s_fsig_win));
    memset(s_roll_win,  0, sizeof(s_roll_win));
    memset(s_pitch_win, 0, sizeof(s_pitch_win));
    memset(s_gyro_axis_1s,  0, sizeof(s_gyro_axis_1s));

    /* No external FFT library required — DFT is self-contained */

    /* Initialise force thresholds to medium (default) mode */
    s_warn_sum_n = FORCE_WARN_SUM_N;
    s_err_sum_n  = FORCE_ERR_SUM_N;
    s_warn_tremor_excess_dps = TREMOR_MED_WARN_EXCESS_DPS;
    s_err_tremor_excess_dps  = TREMOR_MED_ERR_EXCESS_DPS;
    s_warn_tremor_ratio = TREMOR_MED_WARN_RATIO;
    s_err_tremor_ratio  = TREMOR_MED_ERR_RATIO;

}

void processing_tremor_baseline_reset(void)
{
    for (int i = 0; i < 3; i++) {
        filter_bp_6_12hz_init(&s_bp_tremor_ref[i]);
    }
}

float processing_tremor_baseline_step(const float gyro_dps[3])
{
    float bp_axis[3];
    for (int i = 0; i < 3; i++) {
        bp_axis[i] = filter_bp_6_12hz_apply(&s_bp_tremor_ref[i], gyro_dps[i]);
    }
    return sqrtf(bp_axis[0] * bp_axis[0] +
                 bp_axis[1] * bp_axis[1] +
                 bp_axis[2] * bp_axis[2]);
}

/* ── Processing task (Core 1) ────────────────────────────────────────── */
/*
 * This task runs exclusively on Core 1 (APP CPU).
 *
 * It is an event-driven consumer loop.  It does not use a timer —
 * it simply blocks on xQueueReceive, waking exactly once per sample
 * that Core 0 produces.  At 100 Hz acquisition rate, this task runs
 * 100 times per second.  JSON is emitted every 5th call (20 Hz).
 *
 * Because this task and acquisition_task are on different cores,
 * they truly run in parallel — Core 1 is processing sample N while
 * Core 0 is already sampling N+1.
 */
void processing_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "PROC TASK: running on Core %d", xPortGetCoreID());

    raw_sample_t samp;
    int sample_count = 0;

    for (;;) {
        /*
         * Block here until Core 0 posts a raw_sample_t.
         * portMAX_DELAY means wait forever — the task is scheduled out
         * and consumes no CPU until the queue has data.
         * Core 0 and Core 1 are now running truly in parallel:
         *   Core 0 is filling the next sample while Core 1 processes this one.
         */
        if (xQueueReceive(s_raw_q, &samp, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* Toggle debug GPIO */
        s_dbg_state ^= 1;
        gpio_set_level(GPIO_DBG_CORE1, s_dbg_state);

        /* ── 1. Filter FSR signals ─────────────────────────────────── */
        float fsr_f[3];
        for (int i = 0; i < 3; i++) {
            fsr_f[i] = filter_lp_10hz_apply(&s_lp_fsr[i], samp.fsr_n[i]);
        }
        float f_sigma = fsr_f[0] + fsr_f[1] + fsr_f[2];
        float df_dt   = filter_diff5_apply(&s_diff5_fsr, f_sigma);

        /* ── 2. Filter IMU signals ─────────────────────────────────── */
        float omega_f[3];
        for (int i = 0; i < 3; i++) {
            float gyro_unbiased = samp.gyro_dps[i] - s_cal->gyro_bias[i];
            omega_f[i] = filter_lp_12hz_apply(&s_lp_omega[i], gyro_unbiased);
        }
        float omega_norm = sqrtf(omega_f[0] * omega_f[0] +
                                 omega_f[1] * omega_f[1] +
                                 omega_f[2] * omega_f[2]);

        /* Bandpass each gyro axis for tremor. Filtering omega_norm directly
         * rectifies sign-changing shake and can hide real oscillation energy. */
        float bp_omega_axis[3];
        for (int i = 0; i < 3; i++) {
            bp_omega_axis[i] = filter_bp_6_12hz_apply(&s_bp_tremor[i], omega_f[i]);
        }
        float bp_omega = sqrtf(bp_omega_axis[0] * bp_omega_axis[0] +
                               bp_omega_axis[1] * bp_omega_axis[1] +
                               bp_omega_axis[2] * bp_omega_axis[2]);

        /* ── 3. Update sliding windows ─────────────────────────────── */
        s_omega_win[s_win_idx]    = omega_norm;
        s_fsig_win[s_win_idx]     = f_sigma;
        s_roll_win[s_win_idx]     = samp.euler_deg[0];
        s_pitch_win[s_win_idx]    = samp.euler_deg[1];
        s_bp_omega_win[s_win_idx] = bp_omega;
        s_win_idx = (s_win_idx + 1) % WIN_250MS;
        if (s_win_filled < WIN_250MS) s_win_filled++;

        for (int i = 0; i < 3; i++) {
            s_gyro_axis_1s[i][s_win_1s_idx] = omega_f[i];
        }
        s_win_1s_idx = (s_win_1s_idx + 1) % WIN_1S;
        if (s_win_1s_filled < WIN_1S) s_win_1s_filled++;

        /* FFT circular buffer */
        s_omega_dft_buf[s_fft_idx] = omega_norm;
        s_fft_idx = (s_fft_idx + 1) % DFT_N;
        if (s_fft_filled < DFT_N) s_fft_filled++;

        /* ── 4. Derived features ───────────────────────────────────── */
        int wn = s_win_filled;

        float omega_rms_250 = window_rms(s_omega_win, wn);
        float tremor_rms   = window_rms(s_bp_omega_win, wn);
        float tremor_ratio = tremor_rms / fmaxf(omega_rms_250, TREMOR_RATIO_FLOOR_DPS);
        float tremor_excess_dps = tremor_rms - tremor_baseline_dps();
        float warn_tremor_ratio_thresh = tremor_warn_ratio_threshold();
        float err_tremor_ratio_thresh = tremor_err_ratio_threshold();
        if (tremor_excess_dps < 0.0f) {
            tremor_excess_dps = 0.0f;
        }
        float tremor_index_amp   = tremor_excess_dps / s_warn_tremor_excess_dps;
        float tremor_index_ratio = tremor_ratio / warn_tremor_ratio_thresh;
        float tremor_index = (tremor_index_amp > tremor_index_ratio) ?
                             tremor_index_amp : tremor_index_ratio;

        /* f95 — only meaningful once FFT buffer is full */
        float f95 = (s_fft_filled >= DFT_N) ? compute_f95() : 0.0f;

        /* CV of force */
        float cv_f = (f_sigma > 0.1f) ?
                     (window_stddev(s_fsig_win, wn) / f_sigma) : 0.0f;

        /* Peak-to-peak roll and pitch over 250ms window */
        float pp_roll  = (wn > 1) ? (window_max(s_roll_win,  wn) - window_min(s_roll_win,  wn)) : 0.0f;
        float pp_pitch = (wn > 1) ? (window_max(s_pitch_win, wn) - window_min(s_pitch_win, wn)) : 0.0f;

        /* Swing rate: signed reversals of the dominant gyro axis per second. */
        float swing_rate = compute_swing_rate();

        /* ── 5. Contact detection (per finger) ────────────────────── */
        for (int i = 0; i < 3; i++) {
            update_contact(i, fsr_f[i]);
        }

        /* ── 6. State detection / engagement gate ──────────────────── */
        bool any_contact = s_contact[0] || s_contact[1] || s_contact[2];
        bool imu_gate_on_motion =
            (omega_rms_250 > 2.5f) || (pp_roll > 4.0f) || (pp_pitch > 4.0f);
        bool imu_gate_off_motion =
            (omega_rms_250 < 1.5f) && (pp_roll < 2.0f) && (pp_pitch < 2.0f);

        if (any_contact) {
            s_imu_fallback_engaged = false;
            s_imu_gate_on_cnt = 0;
            s_imu_gate_off_cnt = 0;
        } else if (wn >= WIN_250MS) {
            if (!s_imu_fallback_engaged) {
                if (imu_gate_on_motion) {
                    if (++s_imu_gate_on_cnt >= IMU_GATE_ON_SAMPLES) {
                        s_imu_fallback_engaged = true;
                        s_imu_gate_on_cnt = 0;
                        s_imu_gate_off_cnt = 0;
                    }
                } else {
                    s_imu_gate_on_cnt = 0;
                }
            } else {
                if (imu_gate_off_motion) {
                    if (++s_imu_gate_off_cnt >= IMU_GATE_OFF_SAMPLES) {
                        s_imu_fallback_engaged = false;
                        s_imu_gate_on_cnt = 0;
                        s_imu_gate_off_cnt = 0;
                    }
                } else {
                    s_imu_gate_off_cnt = 0;
                }
            }
        }
        bool instrument_engaged = any_contact || s_imu_fallback_engaged;
        const char *gate_str = any_contact ? "FSR" :
                               s_imu_fallback_engaged ? "IMU" : "NONE";

        if (s_state != BOARD_EXITED) {
            if (!instrument_engaged) {
                s_state    = BOARD_IDLE;
                s_hold_cnt = 0;
            } else if (omega_norm < 10.0f) {
                if (++s_hold_cnt >= 15) { /* 150 ms = 15 samples at 100 Hz */
                    s_state = BOARD_HOLD;
                }
            } else {
                s_hold_cnt = 0;
                s_state    = BOARD_ACTIVE;
            }
        }

        const char *state_str =
            (s_state == BOARD_HOLD)   ? "HOLD"   :
            (s_state == BOARD_ACTIVE) ? "ACTIVE" :
            (s_state == BOARD_EXITED) ? "EXITED" : "IDLE";

        /* ── 7. Threshold logic → warning / error bitmasks ───────────── */
        uint8_t warn_flags = 0;
        uint8_t err_flags  = 0;

        if (s_state == BOARD_HOLD && wn >= WIN_250MS) {
            float sd_roll       = window_stddev(s_roll_win, wn);
            float sd_pitch      = window_stddev(s_pitch_win, wn);
            float sd_max        = (sd_roll > sd_pitch) ? sd_roll : sd_pitch;

            /* Hold instability */
            if (omega_rms_250 > 5.0f || sd_max > 2.0f) {
                warn_flags |= WARN_HOLD_INSTABILITY;
            }
            if (omega_rms_250 > 8.0f || sd_max > 3.0f) {
                err_flags |= ERR_HOLD_INSTABILITY;
            }

            /* Hold force variability */
            float sd_fsig = window_stddev(s_fsig_win, wn);
            if (sd_fsig > 0.4f || cv_f > 0.25f) {
                warn_flags |= WARN_FORCE_VARIABILITY;
            }
        }

        /* Hand shaking / tremor: only score while the surgeon is actually
         * gripping the instrument, and scale tolerance by mode. */
        if (instrument_engaged && wn >= WIN_250MS) {
            if (tremor_excess_dps > s_warn_tremor_excess_dps &&
                tremor_ratio > warn_tremor_ratio_thresh) {
                warn_flags |= WARN_TREMOR;
            }
            if (tremor_excess_dps > s_err_tremor_excess_dps &&
                tremor_ratio > err_tremor_ratio_thresh) {
                err_flags |= ERR_TREMOR;
            }
        }

        if (any_contact) {
            /* Per-finger thresholds (Horeman et al. 2010) */
            for (int fi = 0; fi < 3; fi++) {
                if (fsr_f[fi] > FORCE_ERR_FINGER_N) {
                    warn_flags |= WARN_FORCE_OPEN;
                    err_flags  |= ERR_FORCE_OPEN;
                } else if (fsr_f[fi] > FORCE_WARN_FINGER_N) {
                    warn_flags |= WARN_FORCE_OPEN;
                }
            }

            /* F_sum sustained thresholds — 100 ms window at 100 Hz */
            if (f_sigma > s_warn_sum_n) {
                s_warn_force_dur_cnt++;
            } else {
                s_warn_force_dur_cnt = 0;
            }
            if (f_sigma > s_err_sum_n) {
                s_err_force_dur_cnt++;
            } else {
                s_err_force_dur_cnt = 0;
            }
            if (s_warn_force_dur_cnt >= FORCE_SUSTAIN_SAMPLES) {
                warn_flags |= WARN_FORCE_OPEN;
            }
            if (s_err_force_dur_cnt >= FORCE_SUSTAIN_SAMPLES) {
                err_flags |= ERR_FORCE_OPEN | ERR_SUSTAINED_COMPRESS;
            }

            /* Force spike: |dF/dt| > 20 N/s */
            if (fabsf(df_dt) > 20.0f) {
                warn_flags |= WARN_FORCE_SPIKE;
            }
        }

        /* ── 8. Motor pulses ───────────────────────────────────────── */
        if (warn_flags & WARN_FORCE_OPEN)       motor_pulse(0, 200);
        if (err_flags  & ERR_FORCE_OPEN)         motor_pulse(1, 200);
        if (warn_flags & WARN_HOLD_INSTABILITY)  motor_pulse(2, 200);
        if (warn_flags & WARN_TREMOR)            motor_pulse(3, 200);

        /* ── 9. Running score ──────────────────────────────────────── */
        if (warn_flags) s_score = (s_score > 0.2f) ? s_score - 0.2f : 0.0f;
        if (err_flags)  s_score = (s_score > 0.5f) ? s_score - 0.5f : 0.0f;

        /* ── 10. Check for UART commands (non-blocking) ────────────── */
        uart_event_t ev;
        if (xQueueReceive(s_uart_q, &ev, 0) == pdTRUE) {
            if (ev.type == UART_DATA) {
                uint8_t cmd_buf[32];
                memset(cmd_buf, 0, sizeof(cmd_buf));
                int n_read = uart_read_bytes(UART_NUM_0, cmd_buf,
                                             (ev.size < 31) ? (uint32_t)ev.size : 31U,
                                             0);
                if (n_read > 0) {
                    dispatch_command((char *)cmd_buf, n_read);
                }
            }
        }

        /* ── 11. JSON output every 5th sample (≈20 Hz) ────────────── */
        sample_count++;
        if (sample_count % 5 == 0) {
            char buf[640];
            int n = snprintf(buf, sizeof(buf),
                "{\"t\":%lld"
                ",\"f0\":%.3f,\"f1\":%.3f,\"f2\":%.3f"
                ",\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f"
                ",\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f"
                ",\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f"
                ",\"f_sum\":%.3f,\"tremor\":%.3f,\"f95\":%.2f"
                ",\"pp_roll\":%.2f,\"pp_pitch\":%.2f"
                ",\"cv_f\":%.3f,\"swing\":%.2f"
                ",\"contact\":[%d,%d,%d]"
                ",\"engaged\":%d,\"gate\":\"%s\""
                ",\"state\":\"%s\""
                ",\"warn\":%u,\"err\":%u"
                ",\"score\":%.1f,\"actual_hz\":%.2f"
                "}\r\n",
                samp.timestamp_us / 1000LL,
                (double)samp.fsr_n[0], (double)samp.fsr_n[1], (double)samp.fsr_n[2],
                (double)samp.accel_g[0], (double)samp.accel_g[1], (double)samp.accel_g[2],
                (double)samp.gyro_dps[0], (double)samp.gyro_dps[1], (double)samp.gyro_dps[2],
                (double)samp.euler_deg[0], (double)samp.euler_deg[1], (double)samp.euler_deg[2],
                (double)f_sigma,
                (double)tremor_index,
                (double)f95,
                (double)pp_roll, (double)pp_pitch,
                (double)cv_f, (double)swing_rate,
                (int)s_contact[0], (int)s_contact[1], (int)s_contact[2],
                instrument_engaged ? 1 : 0, gate_str,
                state_str,
                (unsigned)warn_flags, (unsigned)err_flags,
                (double)s_score,
                (double)s_actual_hz);

            if (n > 0) {
                uart_write_bytes(UART_NUM_0, buf, (size_t)n);
            }
        }
    }
}
