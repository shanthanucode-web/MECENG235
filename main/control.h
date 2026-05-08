#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
/*
 * CORE 0 — Control Module
 * ═══════════════════════
 * This module owns the project control plane on Core 0:
 *
 *   - UART0 host command ingress
 *   - command parsing / dispatch
 *   - coordination with the Core 1 processing module
 *   - multitasking-proof mode control on behalf of the host
 *
 * This task is always present in the real product. It exists because host
 * control and calibration coordination should not live inside the hard
 * real-time acquisition loop, and they should not bloat the Core 1
 * signal-processing pipeline either.
 */

void control_init(QueueHandle_t uart_event_q);
void control_task(void *arg);
