#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/*
 * CORE 0 — Acquisition Module
 * ═══════════════════════════
 * This module owns everything that runs on Core 0 (PRO CPU).
 * Its sole responsibility is sampling sensors at exactly 100 Hz and
 * forwarding raw_sample_t packets to Core 1 via a FreeRTOS queue.
 *
 * Core 0 does NO filtering, NO threshold checks, NO UART output.
 * Keeping it this lean is what makes the 100 Hz cadence rock-solid.
 *
 * Timing chain (how 100 Hz is achieved):
 *
 *   esp_timer (hardware timer, fires every 10 000 µs)
 *     └─ timer_isr_cb() — runs in esp_timer task (Core 0, priority 22)
 *           └─ xSemaphoreGive(s_timer_sem)
 *                 └─ acquisition_task wakes (Core 0, priority 10)
 *                       └─ reads ADC × 3, reads IMU if present
 *                             └─ xQueueSend → Core 1
 *
 * The binary semaphore (s_timer_sem) is the handoff between the
 * high-priority timer task and the acquisition task.  The timer task
 * completes in microseconds and immediately blocks again, leaving Core 0
 * free for the acquisition task for the remaining ~9.9 ms of each tick.
 */

/*
 * Initialize all Core 0 hardware.
 * Must be called from app_main (Core 0) before spawning acquisition_task.
 *
 *   - ADC1 oneshot (GPIO 34=CH6, GPIO 39=CH3, GPIO 32=CH4)
 *   - I2C master bus (SDA=GPIO22, SCL=GPIO20) — non-fatal if wiring absent
 *   - BNO085 via CEVA sh2 library — probed first; sets s_imu_ok=false if absent
 *   - BNO085 INT falling-edge GPIO ISR (GPIO 15) — only if IMU present
 *   - 100 Hz esp_timer (ESP_TIMER_TASK dispatch)
 *   - GPIO 33 (freq proof toggle) and GPIO 12 (Core 1 debug) as outputs
 *
 * out_queue — the inter-core queue created by app_main; Core 0 writes to it,
 *             Core 1 reads from it.
 */
esp_err_t acquisition_init(QueueHandle_t out_queue);

/*
 * Core 0 task entry point — pin with xTaskCreatePinnedToCore(..., 0).
 *
 * Each iteration:
 *   1. Block on s_timer_sem (released by 100 Hz timer callback)
 *   2. Read ADC channels for all 3 FSR sensors
 *   3. If IMU is present: service sh2, copy latest IMU data
 *   4. Post raw_sample_t to inter-core queue (non-blocking; drops oldest if full)
 *   5. Every 100 samples: log actual measured acquisition rate
 */
void acquisition_task(void *arg);
