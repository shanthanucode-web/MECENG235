#pragma once

#include <stdint.h>

/*
 * Built-in status LED controller.
 *
 * GPIO_STATUS_LED is intentionally separate from GPIO_FREQ_PROOF so the class
 * LED demo does not disturb the 100 Hz acquisition proof pin.
 */
void led_status_init(void);
void led_status_off(void);
void led_status_solid_on(void);
void led_status_blink_hz(uint32_t hz);
