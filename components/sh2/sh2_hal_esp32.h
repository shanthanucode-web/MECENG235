#pragma once

#include "sh2_hal.h"

/*
 * Populate *hal with ESP32 UART HAL function pointers.
 * Call this before sh2_open(hal, ...).
 *
 * UART2 (RX=GPIO32, TX=GPIO33) at 3 Mbaud is initialised inside hal_open
 * when sh2_open() calls it.  No handles need to be passed in.
 */
void sh2_hal_esp32_init(sh2_Hal_t *hal);
