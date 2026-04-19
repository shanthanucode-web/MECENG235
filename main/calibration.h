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
 * step        — calibration step character: '1' (C1) through '4' (C4)
 * raw_q       — the same queue used by the processing task
 * params      — in/out: existing parameters; updated in-place on success
 *
 * Returns ESP_OK on success.
 * On CAL_COMPLETE (step '4' succeeds), saves params to NVS automatically.
 */
esp_err_t calibration_run(char step, QueueHandle_t raw_q, cal_params_t *params);

/*
 * Handle a C3 repeat sub-command (called when "C3_REP" is received mid-calibration).
 * Only valid while a C3 calibration is in progress.
 * Returns ESP_ERR_INVALID_STATE if no C3 is running.
 */
esp_err_t calibration_c3_rep(QueueHandle_t raw_q, cal_params_t *params);

/*
 * Handle a C4 cycle sub-command (called when "C4_CYCLE" is received).
 */
esp_err_t calibration_c4_cycle(QueueHandle_t raw_q, cal_params_t *params);
