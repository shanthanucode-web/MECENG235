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
    esp_err_t ret = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (ret != ESP_OK) {
        return ret;
    }

    size_t sz = sizeof(cal_params_t);
    ret = nvs_get_blob(h, NVS_KEY, out, &sz);
    nvs_close(h);

    if (ret == ESP_OK && sz != sizeof(cal_params_t)) {
        /* Struct layout changed — treat as missing */
        ESP_LOGW(TAG, "NVS blob size mismatch (%u vs %u), ignoring",
                 (unsigned)sz, (unsigned)sizeof(cal_params_t));
        return ESP_ERR_NVS_NOT_FOUND;
    }
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
}
