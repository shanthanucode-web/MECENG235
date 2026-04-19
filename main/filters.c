#include "filters.h"

#include <string.h>

/* ── Direct Form II Transposed biquad step ──────────────────────────────── */
/*
 * Update equations:
 *   y    = b0*x + w1
 *   w1'  = b1*x - a1*y + w2
 *   w2'  = b2*x - a2*y
 *
 * Coefficients stored with the standard sign convention where the
 * denominator polynomial is 1 + a1*z^-1 + a2*z^-2.
 */
static inline float biquad_step(biquad_state_t *s,
                                 float x,
                                 float b0, float b1, float b2,
                                 float a1, float a2)
{
    float y = b0 * x + s->w1;
    s->w1   = b1 * x - a1 * y + s->w2;
    s->w2   = b2 * x - a2 * y;
    return y;
}

/* ── 2nd-order Butterworth LP 12 Hz (fs=100 Hz) ─────────────────────────── */
/*
 * Designed with scipy.signal.butter(2, 12, 'low', fs=100, output='sos')
 * Verified: -3.01 dB @ 12 Hz, -21.7 dB @ 30 Hz
 */
#define LP12_B0  0.0913149004f
#define LP12_B1  0.1826298009f
#define LP12_B2  0.0913149004f
#define LP12_A1 (-0.9824057931f)
#define LP12_A2  0.3476653949f

void filter_lp_12hz_init(filter_state_t *st)
{
    memset(st, 0, sizeof(*st));
}

float filter_lp_12hz_apply(filter_state_t *st, float x)
{
    return biquad_step(&st->s[0], x, LP12_B0, LP12_B1, LP12_B2, LP12_A1, LP12_A2);
}

/* ── 4th-order Butterworth BP 6–12 Hz (fs=100 Hz) ───────────────────────── */
/*
 * Designed with scipy.signal.butter(2, [6,12], 'band', fs=100, output='sos')
 * Produces 2 biquad sections (4th-order BP from 2nd-order LP prototype).
 * Verified: -3.01 dB @ 6 Hz and 12 Hz, < -0.2 dB across 7–11 Hz
 */
#define BP_S0_B0  0.0278597661f
#define BP_S0_B1  0.0557195322f
#define BP_S0_B2  0.0278597661f
#define BP_S0_A1 (-1.3261538021f)
#define BP_S0_A2  0.7207856834f

#define BP_S1_B0  1.0000000000f
#define BP_S1_B1 (-2.0000000000f)
#define BP_S1_B2  1.0000000000f
#define BP_S1_A1 (-1.6612057415f)
#define BP_S1_A2  0.8142774219f

void filter_bp_6_12hz_init(filter_state_t *st)
{
    memset(st, 0, sizeof(*st));
}

float filter_bp_6_12hz_apply(filter_state_t *st, float x)
{
    float y = biquad_step(&st->s[0], x,
                          BP_S0_B0, BP_S0_B1, BP_S0_B2,
                          BP_S0_A1, BP_S0_A2);
    return biquad_step(&st->s[1], y,
                       BP_S1_B0, BP_S1_B1, BP_S1_B2,
                       BP_S1_A1, BP_S1_A2);
}

/* ── 2nd-order Butterworth LP 10 Hz (fs=100 Hz) ─────────────────────────── */
/*
 * Designed with scipy.signal.butter(2, 10, 'low', fs=100, output='sos')
 * Verified: -3.01 dB @ 10 Hz
 */
#define LP10_B0  0.0674552739f
#define LP10_B1  0.1349105478f
#define LP10_B2  0.0674552739f
#define LP10_A1 (-1.1429805025f)
#define LP10_A2  0.4128015981f

void filter_lp_10hz_init(filter_state_t *st)
{
    memset(st, 0, sizeof(*st));
}

float filter_lp_10hz_apply(filter_state_t *st, float x)
{
    return biquad_step(&st->s[0], x, LP10_B0, LP10_B1, LP10_B2, LP10_A1, LP10_A2);
}

/* ── 5-sample backward difference ──────────────────────────────────────── */
/*
 * Approximates dF/dt in units-per-second at 100 Hz:
 *   d = (x[n] - x[n-5]) / (5 * 0.01 s) = (x[n] - x[n-5]) * 20.0
 * Returns 0 until 6 samples have been collected.
 */
void filter_diff5_init(diff5_state_t *st)
{
    memset(st, 0, sizeof(*st));
}

float filter_diff5_apply(diff5_state_t *st, float x)
{
    uint8_t write_idx = st->idx;
    float   oldest    = st->buf[(write_idx + 1) % 6]; /* x[n-5] slot */

    st->buf[write_idx] = x;
    st->idx = (uint8_t)((write_idx + 1) % 6);

    if (st->filled < 6) {
        st->filled++;
        return 0.0f;
    }
    return (x - oldest) * 20.0f;
}
