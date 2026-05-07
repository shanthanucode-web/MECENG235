#include "nvs_storage.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG     = "NVS";
static const char *NVS_NS  = "trainer_cal";
static const char *NVS_KEY = "cal_params";

esp_err_t nvs_load_calibration(cal_params_t *out)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        return ret;
    }

    size_t sz = 0;
    ret = nvs_get_blob(h, NVS_KEY, NULL, &sz);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(h);
        return ret;
    }
    if (ret != ESP_OK) {
        nvs_close(h);
        return ret;
    }

    if (sz != sizeof(cal_params_t)) {
        ESP_LOGW(TAG, "NVS blob size mismatch (%u vs %u), erasing stale calibration",
                 (unsigned)sz, (unsigned)sizeof(cal_params_t));
        esp_err_t erase_ret = nvs_erase_key(h, NVS_KEY);
        if (erase_ret == ESP_OK || erase_ret == ESP_ERR_NVS_NOT_FOUND) {
            erase_ret = nvs_commit(h);
        }
        if (erase_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to erase stale calibration blob: %s", esp_err_to_name(erase_ret));
        }
        nvs_close(h);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    ret = nvs_get_blob(h, NVS_KEY, out, &sz);
    nvs_close(h);
    return ret;
}

esp_err_t nvs_save_calibration(const cal_params_t *params)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(h, NVS_KEY, params, sizeof(cal_params_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_save failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t nvs_erase_calibration(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_erase_key(h, NVS_KEY);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ret = ESP_OK; /* already absent, that's fine */
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);
    return ret;
}

void nvs_get_defaults(cal_params_t *out)
{
    memset(out, 0, sizeof(*out));

    /* FSR baseline: no contact, small noise floor */
    for (int i = 0; i < 3; i++) {
        out->mu[i]         = 0.0f;
        out->sigma[i]      = 0.1f;
        out->on_thresh[i]  = 0.30f;   /* max(0.30, 0 + 5*0.1) = 0.50 — use 0.30 */
        out->off_thresh[i] = 0.15f;
    }

    /* IMU neutral: identity quaternion, zero bias */
    out->q_neutral[0] = 1.0f; /* w */

    /* Open-grip reference: expert mean from Horeman et al. 2010 */
    out->f_ref_open   = 0.9f;   /* N — expert mean (Horeman et al. 2010) */
    out->f95_ref      = 4.8f;   /* Hz */
    out->pp_roll_ref  = 30.0f;  /* deg */
    out->pp_pitch_ref = 20.0f;  /* deg */
    out->motion_rms_ref = 12.0f;          /* deg/s */
    out->motion_tremor_ratio_ref = 0.25f; /* unitless */
    out->yaw_right_ref = 24.0f; /* deg */
    out->yaw_left_ref  = 24.0f; /* deg */
    out->pitch_fwd_ref = 18.0f; /* deg */
    out->pitch_back_ref = 18.0f; /* deg */
    out->tremor_rms_ref = TREMOR_BASELINE_FLOOR_DPS;
    out->yaw_axis_index = -1;
    out->pitch_axis_index = -1;
}
