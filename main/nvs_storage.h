#pragma once

#include "esp_err.h"
#include "data_types.h"

/*
 * Load calibration parameters from NVS.
 * Returns ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if never saved.
 */
esp_err_t nvs_load_calibration(cal_params_t *out);

/*
 * Save calibration parameters to NVS.
 */
esp_err_t nvs_save_calibration(const cal_params_t *params);

/*
 * Erase calibration from NVS (triggered by 'Z' command).
 * Next boot will use research-derived defaults.
 */
esp_err_t nvs_erase_calibration(void);

/*
 * Fill *out with research-derived defaults.
 * Called when NVS load fails.
 */
void nvs_get_defaults(cal_params_t *out);
