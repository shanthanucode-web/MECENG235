#include "processing.h"
#include "acquisition.h"
#include "filters.h"
#include "motor_control.h"
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
#include <inttypes.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "PROC";

/* ── Module state ─────────────────────────────────────────────────────── */
static QueueHandle_t  s_raw_q;
static QueueHandle_t  s_ctrl_q;
static cal_params_t  *s_cal;
static volatile bool  s_mt_mode_enabled = false;

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
static float s_warn_finger_n = FORCE_MED_WARN_FINGER_N;
static float s_err_finger_n  = FORCE_MED_ERR_FINGER_N;
static float s_warn_tremor_excess_dps = TREMOR_MED_WARN_EXCESS_DPS;
static float s_err_tremor_excess_dps  = TREMOR_MED_ERR_EXCESS_DPS;
static float s_warn_tremor_ratio = TREMOR_MED_WARN_RATIO;
static float s_err_tremor_ratio  = TREMOR_MED_ERR_RATIO;
static float s_warn_hold_omega_dps = HOLD_MED_WARN_OMEGA_DPS;
static float s_err_hold_omega_dps  = HOLD_MED_ERR_OMEGA_DPS;
static float s_warn_hold_sd_deg    = HOLD_MED_WARN_SD_DEG;
static float s_err_hold_sd_deg     = HOLD_MED_ERR_SD_DEG;
static float s_warn_force_var_sd_n = FORCE_VAR_MED_SD_N;
static float s_warn_force_var_cv   = FORCE_VAR_MED_CV;
static float s_warn_force_spike_nps = FORCE_SPIKE_MED_DFDT_NPS;
static float s_warn_score_penalty = SCORE_MED_WARN_PENALTY;
static float s_err_score_penalty  = SCORE_MED_ERR_PENALTY;

/* ── Sustained error counters for force compression ──────────────────── */
static int s_warn_force_dur_cnt = 0;
static int s_err_force_dur_cnt  = 0;

/* ── Running score ───────────────────────────────────────────────────── */
static float s_score = 100.0f;

#define MT_CYCLE_RING_SIZE        8
#define MT_COMPLETE_HISTORY_SIZE 32

typedef struct {
    bool     in_use;
    bool     complete;
    uint32_t cycle_id;
    uint32_t ctrl_running_before_timer;
    int64_t  t_timer_cb_begin_us;
    int64_t  t_sem_give_us;
    int64_t  t_acq_wake_us;
    int64_t  t_acq_done_us;
    int64_t  t_ctrl_resume_us;
} mt_cycle_trace_t;

typedef struct {
    mt_cycle_trace_t cycles[MT_CYCLE_RING_SIZE];
    mt_cycle_trace_t history[MT_COMPLETE_HISTORY_SIZE];
    uint32_t         history_count;
    uint32_t         history_head;
    uint32_t         cycle_miss;
    bool             have_latest;
    mt_cycle_trace_t latest;
} mt_trace_state_t;

static mt_trace_state_t s_mt_trace_state;

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

static void mt_trace_state_reset(void)
{
    memset(&s_mt_trace_state, 0, sizeof(s_mt_trace_state));
}

/* Active proof cycles are kept in a small ring because Core 1 is rebuilding
 * them from raw events, not receiving one pre-packaged packet per cycle. */
static mt_cycle_trace_t *mt_trace_find_cycle(uint32_t cycle_id, bool create)
{
    mt_cycle_trace_t *free_slot = NULL;

    for (int i = 0; i < MT_CYCLE_RING_SIZE; i++) {
        mt_cycle_trace_t *slot = &s_mt_trace_state.cycles[i];
        if (slot->in_use && slot->cycle_id == cycle_id) {
            return slot;
        }
        if (!slot->in_use && free_slot == NULL) {
            free_slot = slot;
        }
    }

    if (!create) {
        return NULL;
    }

    if (free_slot != NULL) {
        memset(free_slot, 0, sizeof(*free_slot));
        free_slot->in_use = true;
        free_slot->cycle_id = cycle_id;
        return free_slot;
    }

    uint32_t oldest_idx = 0;
    uint32_t oldest_cycle_id = UINT32_MAX;
    for (uint32_t i = 0; i < MT_CYCLE_RING_SIZE; i++) {
        if (s_mt_trace_state.cycles[i].cycle_id < oldest_cycle_id) {
            oldest_cycle_id = s_mt_trace_state.cycles[i].cycle_id;
            oldest_idx = i;
        }
    }

    mt_cycle_trace_t *slot = &s_mt_trace_state.cycles[oldest_idx];
    if (!slot->complete) {
        s_mt_trace_state.cycle_miss++;
    }
    memset(slot, 0, sizeof(*slot));
    slot->in_use = true;
    slot->cycle_id = cycle_id;
    return slot;
}

static void mt_trace_complete_cycle(mt_cycle_trace_t *cycle)
{
    if (cycle == NULL || cycle->complete) {
        return;
    }

    /* A complete cycle is both the latest dashboard example and one sample in
     * the rolling timing statistics. */
    cycle->complete = true;
    s_mt_trace_state.latest = *cycle;
    s_mt_trace_state.have_latest = true;
    s_mt_trace_state.history[s_mt_trace_state.history_head] = *cycle;
    s_mt_trace_state.history_head =
        (s_mt_trace_state.history_head + 1u) % MT_COMPLETE_HISTORY_SIZE;
    if (s_mt_trace_state.history_count < MT_COMPLETE_HISTORY_SIZE) {
        s_mt_trace_state.history_count++;
    }

    cycle->in_use = false;
}

static void mt_trace_collect_stats(double   *period_us_avg,
                                   double   *jitter_us,
                                   double   *wake_us_avg,
                                   double   *acq_us_avg,
                                   double   *ctrl_resume_us_avg,
                                   double   *ctrl_ratio,
                                   uint32_t *sample_count)
{
    /* These statistics are computed from assembled firmware cycles, not from
     * host-side packet arrival spacing. */
    double prev_tb = 0.0;
    double period_sum = 0.0;
    double period_sum_sq = 0.0;
    uint32_t period_n = 0;
    double wake_sum = 0.0;
    double acq_sum = 0.0;
    double ctrl_sum = 0.0;
    double ctrl_true = 0.0;
    uint32_t n = s_mt_trace_state.history_count;

    for (uint32_t i = 0; i < n; i++) {
        uint32_t idx = (s_mt_trace_state.history_head + MT_COMPLETE_HISTORY_SIZE - n + i)
                       % MT_COMPLETE_HISTORY_SIZE;
        const mt_cycle_trace_t *c = &s_mt_trace_state.history[idx];
        if (!c->complete) {
            continue;
        }
        if (prev_tb > 0.0) {
            double period = (double)(c->t_timer_cb_begin_us - (int64_t)prev_tb);
            period_sum += period;
            period_sum_sq += period * period;
            period_n++;
        }
        prev_tb = (double)c->t_timer_cb_begin_us;
        wake_sum += (double)(c->t_acq_wake_us - c->t_timer_cb_begin_us);
        acq_sum += (double)(c->t_acq_done_us - c->t_acq_wake_us);
        ctrl_sum += (double)(c->t_ctrl_resume_us - c->t_acq_done_us);
        ctrl_true += c->ctrl_running_before_timer ? 1.0 : 0.0;
    }

    *sample_count = n;
    *period_us_avg = (period_n > 0) ? (period_sum / (double)period_n) : 0.0;
    if (period_n > 1) {
        double mean = *period_us_avg;
        double variance = (period_sum_sq / (double)period_n) - (mean * mean);
        *jitter_us = (variance > 0.0) ? sqrt(variance) : 0.0;
    } else {
        *jitter_us = 0.0;
    }
    *wake_us_avg = (n > 0) ? (wake_sum / (double)n) : 0.0;
    *acq_us_avg = (n > 0) ? (acq_sum / (double)n) : 0.0;
    *ctrl_resume_us_avg = (n > 0) ? (ctrl_sum / (double)n) : 0.0;
    *ctrl_ratio = (n > 0) ? (ctrl_true / (double)n) : 0.0;
}

static void mt_trace_emit_snapshot(void)
{
    if (!s_mt_trace_state.have_latest) {
        return;
    }

    double period_us = 0.0;
    double jitter_us = 0.0;
    double wake_us = 0.0;
    double acq_us = 0.0;
    double ctrl_resume_us = 0.0;
    double ctrl_ratio = 0.0;
    uint32_t n = 0;
    char buf[384];

    mt_trace_collect_stats(&period_us, &jitter_us, &wake_us,
                           &acq_us, &ctrl_resume_us, &ctrl_ratio, &n);

    const mt_cycle_trace_t *c = &s_mt_trace_state.latest;
    /* Snapshot packets run at 20 Hz so proof mode stays presentation-safe and
     * does not perturb the Core 0 task we are trying to reason about. */
    int len = snprintf(buf, sizeof(buf),
                       "{\"mt\":\"SN\",\"latest\":{\"ctrl\":%" PRIu32 ",\"tb\":%" PRId64 ",\"sg\":%" PRId64
                       ",\"aw\":%" PRId64 ",\"ad\":%" PRId64 ",\"cr\":%" PRId64 "},"
                       "\"stats\":{\"period_us\":%.1f,\"jitter_us\":%.1f,\"wake_us\":%.1f,"
                       "\"acq_us\":%.1f,\"ctrl_resume_us\":%.1f,\"ctrl_ratio\":%.3f,\"n\":%u},"
                       "\"health\":{\"trace_drop\":%lu,\"cycle_miss\":%lu}}\r\n",
                       c->ctrl_running_before_timer,
                       c->t_timer_cb_begin_us,
                       c->t_sem_give_us,
                       c->t_acq_wake_us,
                       c->t_acq_done_us,
                       c->t_ctrl_resume_us,
                       period_us, jitter_us, wake_us, acq_us, ctrl_resume_us,
                       ctrl_ratio, (unsigned)n,
                       (unsigned long)acquisition_mt_trace_dropped(),
                       (unsigned long)s_mt_trace_state.cycle_miss);
    if (len > 0) {
        uart_write_bytes(UART_NUM_0, buf, (size_t)len);
    }
}

static void mt_trace_drain_events(void)
{
    mt_trace_event_t ev;

    /* Core 1 owns the expensive proof work: drain raw events, rebuild cycles,
     * compute stats, and serialize snapshots over UART0. */
    while (acquisition_mt_trace_pop(&ev)) {
        mt_cycle_trace_t *cycle = mt_trace_find_cycle(ev.cycle_id, ev.code == MT_TRACE_TIMER_CB_BEGIN);
        if (cycle == NULL) {
            s_mt_trace_state.cycle_miss++;
            continue;
        }

        switch (ev.code) {
        case MT_TRACE_TIMER_CB_BEGIN:
            cycle->ctrl_running_before_timer = ev.aux;
            cycle->t_timer_cb_begin_us = ev.t_us;
            break;
        case MT_TRACE_SEM_GIVE:
            cycle->t_sem_give_us = ev.t_us;
            break;
        case MT_TRACE_ACQ_WAKE:
            cycle->t_acq_wake_us = ev.t_us;
            break;
        case MT_TRACE_ACQ_DONE:
            cycle->t_acq_done_us = ev.t_us;
            break;
        case MT_TRACE_CTRL_RESUME:
            cycle->t_ctrl_resume_us = ev.t_us;
            if (cycle->t_timer_cb_begin_us > 0 &&
                cycle->t_sem_give_us > 0 &&
                cycle->t_acq_wake_us > 0 &&
                cycle->t_acq_done_us > 0) {
                mt_trace_complete_cycle(cycle);
            } else {
                s_mt_trace_state.cycle_miss++;
                cycle->in_use = false;
            }
            break;
        default:
            break;
        }
    }
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

static void apply_difficulty(float warn_force_scale,
                             float err_force_scale,
                             float warn_finger_n,
                             float err_finger_n,
                             float warn_tremor_excess_dps,
                             float err_tremor_excess_dps,
                             float warn_tremor_ratio,
                             float err_tremor_ratio,
                             float warn_hold_omega_dps,
                             float err_hold_omega_dps,
                             float warn_hold_sd_deg,
                             float err_hold_sd_deg,
                             float warn_force_var_sd_n,
                             float warn_force_var_cv,
                             float warn_force_spike_nps,
                             float warn_score_penalty,
                             float err_score_penalty)
{
    reset_session_metrics();
    s_warn_sum_n = s_cal->f_ref_open * warn_force_scale;
    s_err_sum_n  = s_cal->f_ref_open * err_force_scale;
    s_warn_finger_n = warn_finger_n;
    s_err_finger_n  = err_finger_n;
    s_warn_tremor_excess_dps = warn_tremor_excess_dps;
    s_err_tremor_excess_dps  = err_tremor_excess_dps;
    s_warn_tremor_ratio = warn_tremor_ratio;
    s_err_tremor_ratio  = err_tremor_ratio;
    s_warn_hold_omega_dps = warn_hold_omega_dps;
    s_err_hold_omega_dps  = err_hold_omega_dps;
    s_warn_hold_sd_deg    = warn_hold_sd_deg;
    s_err_hold_sd_deg     = err_hold_sd_deg;
    s_warn_force_var_sd_n = warn_force_var_sd_n;
    s_warn_force_var_cv   = warn_force_var_cv;
    s_warn_force_spike_nps = warn_force_spike_nps;
    s_warn_score_penalty = warn_score_penalty;
    s_err_score_penalty  = err_score_penalty;
}

static void handle_control_msg(const processing_control_msg_t *msg)
{
    if (msg == NULL) {
        return;
    }

    /* Core 0 decides that a command arrived; Core 1 decides what that means
     * for scoring, calibration, and runtime state. */
    switch (msg->cmd) {
    case PROCESSING_CTRL_IDENTIFY:
        uart_write_bytes(UART_NUM_0, "ESP32_TRAINER\r\n", 15);
        uart_write_bytes(UART_NUM_0, "READY: fw=v1.0 proto=cal-v3\r\n", 31);
        break;
    case PROCESSING_CTRL_STOP:
        s_state    = BOARD_IDLE;
        s_hold_cnt = 0;
        uart_write_bytes(UART_NUM_0, "STOPPED\r\n", 9);
        break;
    case PROCESSING_CTRL_SET_EASY:
        apply_difficulty(FORCE_EASY_WARN_X,
                         FORCE_EASY_ERR_X,
                         FORCE_EASY_WARN_FINGER_N,
                         FORCE_EASY_ERR_FINGER_N,
                         TREMOR_EASY_WARN_EXCESS_DPS,
                         TREMOR_EASY_ERR_EXCESS_DPS,
                         TREMOR_EASY_WARN_RATIO,
                         TREMOR_EASY_ERR_RATIO,
                         HOLD_EASY_WARN_OMEGA_DPS,
                         HOLD_EASY_ERR_OMEGA_DPS,
                         HOLD_EASY_WARN_SD_DEG,
                         HOLD_EASY_ERR_SD_DEG,
                         FORCE_VAR_EASY_SD_N,
                         FORCE_VAR_EASY_CV,
                         FORCE_SPIKE_EASY_DFDT_NPS,
                         SCORE_EASY_WARN_PENALTY,
                         SCORE_EASY_ERR_PENALTY);
        uart_write_bytes(UART_NUM_0, "EASY\r\n", 6);
        break;
    case PROCESSING_CTRL_SET_MEDIUM:
        apply_difficulty(FORCE_MED_WARN_X,
                         FORCE_MED_ERR_X,
                         FORCE_MED_WARN_FINGER_N,
                         FORCE_MED_ERR_FINGER_N,
                         TREMOR_MED_WARN_EXCESS_DPS,
                         TREMOR_MED_ERR_EXCESS_DPS,
                         TREMOR_MED_WARN_RATIO,
                         TREMOR_MED_ERR_RATIO,
                         HOLD_MED_WARN_OMEGA_DPS,
                         HOLD_MED_ERR_OMEGA_DPS,
                         HOLD_MED_WARN_SD_DEG,
                         HOLD_MED_ERR_SD_DEG,
                         FORCE_VAR_MED_SD_N,
                         FORCE_VAR_MED_CV,
                         FORCE_SPIKE_MED_DFDT_NPS,
                         SCORE_MED_WARN_PENALTY,
                         SCORE_MED_ERR_PENALTY);
        uart_write_bytes(UART_NUM_0, "INTERMEDIATE\r\n", 14);
        break;
    case PROCESSING_CTRL_SET_HARD:
        apply_difficulty(FORCE_HARD_WARN_X,
                         FORCE_HARD_ERR_X,
                         FORCE_HARD_WARN_FINGER_N,
                         FORCE_HARD_ERR_FINGER_N,
                         TREMOR_HARD_WARN_EXCESS_DPS,
                         TREMOR_HARD_ERR_EXCESS_DPS,
                         TREMOR_HARD_WARN_RATIO,
                         TREMOR_HARD_ERR_RATIO,
                         HOLD_HARD_WARN_OMEGA_DPS,
                         HOLD_HARD_ERR_OMEGA_DPS,
                         HOLD_HARD_WARN_SD_DEG,
                         HOLD_HARD_ERR_SD_DEG,
                         FORCE_VAR_HARD_SD_N,
                         FORCE_VAR_HARD_CV,
                         FORCE_SPIKE_HARD_DFDT_NPS,
                         SCORE_HARD_WARN_PENALTY,
                         SCORE_HARD_ERR_PENALTY);
        uart_write_bytes(UART_NUM_0, "HARD\r\n", 6);
        break;
    case PROCESSING_CTRL_EXIT:
        s_state = BOARD_EXITED;
        motor_control_init();
        uart_write_bytes(UART_NUM_0, "EXITED\r\n", 8);
        break;
    case PROCESSING_CTRL_RESET_CAL:
        nvs_erase_calibration();
        nvs_get_defaults(s_cal);
        uart_write_bytes(UART_NUM_0, "{\"nvs\":\"ERASED\"}\r\n", 18);
        break;
    case PROCESSING_CTRL_CAL_RUN_STEP:
        calibration_run(msg->step, s_raw_q, s_cal);
        break;
    case PROCESSING_CTRL_CAL_C3_REP:
        calibration_c3_rep(s_raw_q, s_cal);
        break;
    case PROCESSING_CTRL_CAL_C4_CYCLE:
        calibration_c4_cycle(s_raw_q, s_cal);
        break;
    default:
        break;
    }
}

static void drain_control_queue(void)
{
    processing_control_msg_t msg;

    /* Drain-to-empty keeps command handling responsive without forcing the hot
     * sample path to branch on one command at a time. */
    while (s_ctrl_q != NULL && xQueueReceive(s_ctrl_q, &msg, 0) == pdTRUE) {
        handle_control_msg(&msg);
    }
}

/* ── Public API ────────────────────────────────────────────────────────── */

void processing_init(QueueHandle_t raw_q,
                     cal_params_t  *cal)
{
    s_raw_q  = raw_q;
    s_cal    = cal;
    /* This queue is the handoff from the Core 0 control plane to the Core 1
     * owner of processing/calibration state. */
    s_ctrl_q = xQueueCreate(8, sizeof(processing_control_msg_t));
    configASSERT(s_ctrl_q != NULL);

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
    s_warn_finger_n = FORCE_MED_WARN_FINGER_N;
    s_err_finger_n  = FORCE_MED_ERR_FINGER_N;
    s_warn_tremor_excess_dps = TREMOR_MED_WARN_EXCESS_DPS;
    s_err_tremor_excess_dps  = TREMOR_MED_ERR_EXCESS_DPS;
    s_warn_tremor_ratio = TREMOR_MED_WARN_RATIO;
    s_err_tremor_ratio  = TREMOR_MED_ERR_RATIO;
    s_warn_hold_omega_dps = HOLD_MED_WARN_OMEGA_DPS;
    s_err_hold_omega_dps  = HOLD_MED_ERR_OMEGA_DPS;
    s_warn_hold_sd_deg    = HOLD_MED_WARN_SD_DEG;
    s_err_hold_sd_deg     = HOLD_MED_ERR_SD_DEG;
    s_warn_force_var_sd_n = FORCE_VAR_MED_SD_N;
    s_warn_force_var_cv   = FORCE_VAR_MED_CV;
    s_warn_force_spike_nps = FORCE_SPIKE_MED_DFDT_NPS;
    s_warn_score_penalty = SCORE_MED_WARN_PENALTY;
    s_err_score_penalty  = SCORE_MED_ERR_PENALTY;

}

bool processing_submit_control(const processing_control_msg_t *msg,
                               TickType_t                     timeout_ticks)
{
    if (msg == NULL || s_ctrl_q == NULL) {
        return false;
    }
    return xQueueSend(s_ctrl_q, msg, timeout_ticks) == pdTRUE;
}

void processing_set_mt_mode(bool enabled)
{
    /* Proof mode does not alter scoring logic; it only swaps normal runtime
     * JSON for proof snapshots and clears stale trace state. */
    s_mt_mode_enabled = enabled;
    mt_trace_state_reset();
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
        drain_control_queue();
        if (s_mt_mode_enabled) {
            mt_trace_drain_events();
        }

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
            if (omega_rms_250 > s_warn_hold_omega_dps || sd_max > s_warn_hold_sd_deg) {
                warn_flags |= WARN_HOLD_INSTABILITY;
            }
            if (omega_rms_250 > s_err_hold_omega_dps || sd_max > s_err_hold_sd_deg) {
                err_flags |= ERR_HOLD_INSTABILITY;
            }

            /* Hold force variability */
            float sd_fsig = window_stddev(s_fsig_win, wn);
            if (sd_fsig > s_warn_force_var_sd_n || cv_f > s_warn_force_var_cv) {
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
                if (fsr_f[fi] > s_err_finger_n) {
                    warn_flags |= WARN_FORCE_OPEN;
                    err_flags  |= ERR_FORCE_OPEN;
                } else if (fsr_f[fi] > s_warn_finger_n) {
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
            if (fabsf(df_dt) > s_warn_force_spike_nps) {
                warn_flags |= WARN_FORCE_SPIKE;
            }
        }

        /* ── 8. Motor pulses ───────────────────────────────────────── */
        if (warn_flags & WARN_FORCE_OPEN)       motor_pulse(0, 200);
        if (err_flags  & ERR_FORCE_OPEN)         motor_pulse(1, 200);
        if (warn_flags & WARN_HOLD_INSTABILITY)  motor_pulse(2, 200);
        if (warn_flags & WARN_TREMOR)            motor_pulse(3, 200);

        /* ── 9. Running score ──────────────────────────────────────── */
        if (warn_flags) s_score = (s_score > s_warn_score_penalty) ? s_score - s_warn_score_penalty : 0.0f;
        if (err_flags)  s_score = (s_score > s_err_score_penalty) ? s_score - s_err_score_penalty : 0.0f;

        drain_control_queue();
        if (s_mt_mode_enabled) {
            mt_trace_drain_events();
        }

        /* ── 10. JSON output every 5th sample (≈20 Hz) ────────────── */
        sample_count++;
        if (s_mt_mode_enabled && sample_count % 5 == 0) {
            mt_trace_emit_snapshot();
        } else if (!s_mt_mode_enabled && sample_count % 5 == 0) {
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
