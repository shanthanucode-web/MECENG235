#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "data_types.h"

/*
 * Run the calibration state machine.
 * Called from the processing task (blocks Core 1 for the duration).
 * Core 0 acquisition continues posting raw_sample_t to raw_q.
 *
 * step        — calibration step character:
 *               '1' (C1 still hand),
 *               '2' (C2 relaxed fingers),
 *               '3' (C3 normal light grip),
 *               '4' (C4 normal hand motion)
 * raw_q       — the same queue used by the processing task
 * params      — in/out: existing parameters; updated in-place on success
 *
 * Returns ESP_OK on success.
 * Steps C3 and C4 save params to NVS automatically when they complete
 * successfully.
 */
esp_err_t calibration_run(char step, QueueHandle_t raw_q, cal_params_t *params);

/*
 * Legacy repeat hook kept for compatibility with older GUIs.
 * Returns ESP_ERR_INVALID_STATE and emits a "deprecated" status.
 */
esp_err_t calibration_c3_rep(QueueHandle_t raw_q, cal_params_t *params);

/*
 * Legacy cycle hook kept for compatibility with older GUIs.
 * Returns ESP_ERR_INVALID_STATE and emits a "deprecated" status.
 */
esp_err_t calibration_c4_cycle(QueueHandle_t raw_q, cal_params_t *params);
