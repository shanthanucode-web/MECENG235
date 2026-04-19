#pragma once

#include "sh2_hal.h"
#include "driver/i2c_master.h"

/*
 * Populate *hal with ESP32 I2C HAL function pointers.
 * Call this before sh2_open(hal, ...).
 *
 * bus          — already-created i2c_master_bus_handle_t (from i2c_new_master_bus)
 * i2c_addr     — 7-bit I2C address (0x4A for BNO085)
 */
void sh2_hal_esp32_init(sh2_Hal_t *hal,
                        i2c_master_bus_handle_t bus,
                        uint8_t i2c_addr);
