#pragma once

#include <stdint.h>

/*
 * Initialize motor GPIO outputs (always LOW).
 * When MOTORS_ENABLED == 0, GPIO outputs are still configured to LOW
 * so the transistor drivers are kept off by hardware default.
 */
void motor_control_init(void);

/*
 * Fire a single motor pulse.
 *
 * idx          — motor index 0–3 (GPIO 25, 26, 27, 14)
 * duration_ms  — pulse duration in milliseconds (then auto-off)
 *
 * When MOTORS_ENABLED == 0 this is a compile-time no-op.
 * Calling with an out-of-range idx is silently ignored.
 */
void motor_pulse(uint8_t idx, uint32_t duration_ms);
