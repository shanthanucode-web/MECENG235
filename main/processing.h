#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "data_types.h"

/*
 * CORE 1 — Processing Module
 * ══════════════════════════
 * This module owns everything that runs on Core 1 (APP CPU).
 *
 * It is a pure consumer: it blocks waiting for raw_sample_t packets
 * from Core 0, processes each one, and emits JSON over UART.
 *
 * Core 1 never touches the ADC, the I2C bus, or the timer semaphore.
 * Those belong to Core 0.  The only shared resource is the FreeRTOS
 * queue (raw_q), which FreeRTOS keeps thread-safe internally.
 *
 * Per-sample pipeline (runs at 100 Hz on Core 1):
 *
 *   xQueueReceive(raw_q) ← blocks until Core 0 posts a sample
 *     ↓
 *   Butterworth IIR filtering (LP 10 Hz FSR, LP 12 Hz IMU, BP 6-12 Hz tremor)
 *     ↓
 *   Contact detection (per finger, with hysteresis debounce)
 *     ↓
 *   State machine: IDLE → HOLD / ACTIVE
 *     ↓
 *   Feature extraction: omega_norm, f_sigma, tremor_ratio, f95, CV_F, swing_rate
 *     ↓
 *   11 threshold checks → warn_flags / err_flags bitmasks
 *     ↓
 *   Motor pulses (if MOTORS_ENABLED)
 *     ↓
 *   UART command poll (non-blocking, from uart_event_q)
 *     ↓
 *   JSON output every 5th sample (20 Hz) via uart_write_bytes
 */

/*
 * Initialise the processing module.
 * Call from app_main before spawning processing_task.
 *
 * raw_q        — the inter-core queue; Core 1 reads from it
 * uart_event_q — UART driver event queue for incoming commands
 * cal          — calibration parameters (pointer stored; must remain valid)
 */
void processing_init(QueueHandle_t raw_q,
                     QueueHandle_t uart_event_q,
                     cal_params_t  *cal);

/*
 * Core 1 task entry point — pin with xTaskCreatePinnedToCore(..., 1).
 */
void processing_task(void *arg);
