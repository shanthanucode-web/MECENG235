#pragma once

#include <stdint.h>

/* ── Biquad state (Direct Form II Transposed) ───────────────────────────── */
typedef struct {
    float w1;
    float w2;
} biquad_state_t;

/* Up to 4 biquad sections per filter (4th-order needs 2) */
typedef struct {
    biquad_state_t s[4];
} filter_state_t;

/* ── 5-sample backward difference state ─────────────────────────────────── */
typedef struct {
    float   buf[6];   /* circular buffer: 6 slots for x[n]..x[n-5] */
    uint8_t idx;      /* write index */
    uint8_t filled;   /* samples inserted so far, capped at 6 */
} diff5_state_t;

/* ── 2nd-order Butterworth low-pass 12 Hz (fs=100 Hz) ──────────────────── */
/* Used for IMU angular velocity stability branch */
void  filter_lp_12hz_init  (filter_state_t *st);
float filter_lp_12hz_apply (filter_state_t *st, float x);

/* ── 4th-order Butterworth bandpass 6–12 Hz (fs=100 Hz) ────────────────── */
/* Used for tremor quantification (2 biquad sections in series) */
void  filter_bp_6_12hz_init  (filter_state_t *st);
float filter_bp_6_12hz_apply (filter_state_t *st, float x);

/* ── 2nd-order Butterworth low-pass 10 Hz (fs=100 Hz) ──────────────────── */
/* Used for FSR magnitude smoothing */
void  filter_lp_10hz_init  (filter_state_t *st);
float filter_lp_10hz_apply (filter_state_t *st, float x);

/* ── 5-sample backward difference ──────────────────────────────────────── */
/* Returns approximate derivative in units/second at 100 Hz sample rate.
 * Formula: (x[n] - x[n-5]) * 20.0  (i.e., Δ over 50 ms × 1/0.05 s) */
void  filter_diff5_init  (diff5_state_t *st);
float filter_diff5_apply (diff5_state_t *st, float x);
