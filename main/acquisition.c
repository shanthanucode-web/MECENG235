#include "acquisition.h"
#include "data_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "sh2_hal_esp32.h"

#include <math.h>
#include <string.h>

#define RAD_TO_DEG  57.2957795131f
#define G_TO_MS2    9.80665f      /* used to convert m/s² → g */

static const char *TAG = "ACQ";

/* ── Module state ──────────────────────────────────────────────────────── */
static QueueHandle_t             s_out_q;
static SemaphoreHandle_t         s_timer_sem;
static SemaphoreHandle_t         s_imu_sem;

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;

static i2c_master_bus_handle_t   s_i2c_bus;
static sh2_Hal_t                 s_sh2_hal;

/* Latest IMU data updated by sensor callback (acquisition task reads it) */
static volatile struct {
    float accel_g[3];
    float gyro_dps[3];
    float quat[4];    /* w, x, y, z */
    float euler[3];   /* roll, pitch, yaw (deg) */
    bool  fresh;
} s_imu_data;

/* GPIO toggle state for freq proof pin */
static volatile int s_freq_state = 0;

/* Set to false when BNO085 init fails; acquisition task skips all sh2 calls */
static bool s_imu_ok = false;

/* ── FSR 402 voltage → Newton lookup table ─────────────────────────────── */
/*
 * Pull-down circuit: FSR from 3.3V to ADC pin, 10kΩ from ADC pin to GND.
 * Values derived from FSR 402 datasheet resistance vs force curve.
 */
typedef struct { float v_mv; float f_n; } fsr_lut_entry_t;

static const fsr_lut_entry_t FSR402_LUT[] = {
    {    0.0f,  0.000f },
    {  825.0f,  0.196f },
    { 1833.3f,  0.490f },
    { 2538.5f,  0.981f },
    { 3000.0f,  1.962f },
    { 3203.9f,  4.905f },
    { 3260.9f,  9.810f },
    { 3280.3f, 19.620f },
    { 3293.4f, 49.050f },
};
#define FSR402_LUT_LEN  ((int)(sizeof(FSR402_LUT) / sizeof(FSR402_LUT[0])))

static float fsr402_mv_to_newton(float v_mv)
{
    if (v_mv <= FSR402_LUT[0].v_mv) {
        return 0.0f;
    }
    if (v_mv >= FSR402_LUT[FSR402_LUT_LEN - 1].v_mv) {
        return FSR402_LUT[FSR402_LUT_LEN - 1].f_n;
    }
    for (int i = 1; i < FSR402_LUT_LEN; i++) {
        if (v_mv <= FSR402_LUT[i].v_mv) {
            float t = (v_mv - FSR402_LUT[i - 1].v_mv) /
                      (FSR402_LUT[i].v_mv - FSR402_LUT[i - 1].v_mv);
            return FSR402_LUT[i - 1].f_n + t * (FSR402_LUT[i].f_n - FSR402_LUT[i - 1].f_n);
        }
    }
    return 0.0f;
}

/* ── Timer callback — fires at 100 Hz in esp_timer task context ─────────── */
/* ESP_TIMER_TASK dispatch runs in a high-priority task (pri 22), not a real ISR.
 * Regular FreeRTOS calls are safe here. */
static void timer_isr_cb(void *arg)
{
    (void)arg;
    /* Toggle GPIO 33 for software frequency proof measurement */
    s_freq_state ^= 1;
    gpio_set_level(GPIO_FREQ_PROOF, s_freq_state);

    /* Wake acquisition task */
    xSemaphoreGive(s_timer_sem);
}

/* ── BNO085 INT GPIO ISR — falling edge ─────────────────────────────────── */
static void IRAM_ATTR bno085_int_isr(void *arg)
{
    (void)arg;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_imu_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

/* ── BNO085 sensor data callback ────────────────────────────────────────── */
static void sh2_sensor_cb(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;

    sh2_SensorValue_t val;
    if (sh2_decodeSensorEvent(&val, event) != SH2_OK) {
        return;
    }

    switch (val.sensorId) {
    case SH2_LINEAR_ACCELERATION:
        s_imu_data.accel_g[0] = val.un.linearAcceleration.x / G_TO_MS2;
        s_imu_data.accel_g[1] = val.un.linearAcceleration.y / G_TO_MS2;
        s_imu_data.accel_g[2] = val.un.linearAcceleration.z / G_TO_MS2;
        break;

    case SH2_GYROSCOPE_CALIBRATED:
        /* sh2 gyro output is rad/s → convert to deg/s */
        s_imu_data.gyro_dps[0] = val.un.gyroscope.x * RAD_TO_DEG;
        s_imu_data.gyro_dps[1] = val.un.gyroscope.y * RAD_TO_DEG;
        s_imu_data.gyro_dps[2] = val.un.gyroscope.z * RAD_TO_DEG;
        break;

    case SH2_GAME_ROTATION_VECTOR: {
        /* sh2 quaternion: i, j, k, real  →  store as w, x, y, z */
        float w = val.un.gameRotationVector.real;
        float x = val.un.gameRotationVector.i;
        float y = val.un.gameRotationVector.j;
        float z = val.un.gameRotationVector.k;
        s_imu_data.quat[0] = w;
        s_imu_data.quat[1] = x;
        s_imu_data.quat[2] = y;
        s_imu_data.quat[3] = z;

        /* Derive Euler angles (ZYX / aerospace convention) */
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        s_imu_data.euler[0] = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG; /* roll */

        float sinp = 2.0f * (w * y - z * x);
        if (fabsf(sinp) >= 1.0f) {
            s_imu_data.euler[1] = copysignf(90.0f, sinp);
        } else {
            s_imu_data.euler[1] = asinf(sinp) * RAD_TO_DEG; /* pitch */
        }

        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        s_imu_data.euler[2] = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG; /* yaw */

        s_imu_data.fresh = true;
        break;
    }

    default:
        break;
    }
}

/* ── BNO085 initialization helper ───────────────────────────────────────── */
static esp_err_t bno085_init(void)
{
    /* Probe the I2C address first (50 ms timeout).
     * sh2_open() loops internally waiting for SHTP advertisement — it never
     * returns when the sensor is absent.  The probe exits in ≤50 ms on failure. */
    esp_err_t probe = i2c_master_probe(s_i2c_bus, BNO085_I2C_ADDR, 50);
    if (probe != ESP_OK) {
        ESP_LOGE(TAG, "BNO085 not found at 0x%02X — check SDA/SCL wiring (%s)",
                 BNO085_I2C_ADDR, esp_err_to_name(probe));
        return ESP_ERR_NOT_FOUND;
    }

    sh2_hal_esp32_init(&s_sh2_hal, s_i2c_bus, BNO085_I2C_ADDR);

    int rc = sh2_open(&s_sh2_hal, NULL, NULL);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "sh2_open failed: %d", rc);
        return ESP_FAIL;
    }

    sh2_ProductIds_t pids;
    rc = sh2_getProdIds(&pids);
    if (rc == SH2_OK) {
        ESP_LOGI(TAG, "BNO085 part %u sw %u.%u.%u",
                 (unsigned)pids.entry[0].swPartNumber,
                 (unsigned)pids.entry[0].swVersionMajor,
                 (unsigned)pids.entry[0].swVersionMinor,
                 (unsigned)pids.entry[0].swVersionPatch);
    } else {
        ESP_LOGW(TAG, "sh2_getProdIds failed (%d) — IMU may not respond", rc);
    }

    sh2_setSensorCallback(sh2_sensor_cb, NULL);

    /* Enable sensor reports at 10 ms period (100 Hz) */
    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.reportInterval_us = 10000; /* 10 ms = 100 Hz */

    sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &cfg);
    sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &cfg);
    sh2_setSensorConfig(SH2_LINEAR_ACCELERATION,  &cfg);

    return ESP_OK;
}

/* ── ADC initialization ─────────────────────────────────────────────────── */
static esp_err_t adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    /* GPIO 34 = ADC1_CH6 (FSR0), GPIO 39 = ADC1_CH3 (FSR1), GPIO 32 = ADC1_CH4 (FSR2) */
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_6, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_3, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_4, &chan_cfg));

    /* ADC calibration using line-fitting scheme */
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    /* Check if efuse has a valid reference voltage; supply default if not */
    adc_cali_line_fitting_efuse_val_t efuse_val;
    esp_err_t efuse_ret = adc_cali_scheme_line_fitting_check_efuse(&efuse_val);
    if (efuse_ret == ESP_OK &&
        efuse_val == ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF) {
        cali_cfg.default_vref = 1100;
    }

    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali));

    return ESP_OK;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t acquisition_init(QueueHandle_t out_queue)
{
    s_out_q = out_queue;

    /* Semaphores */
    s_timer_sem = xSemaphoreCreateBinary();
    s_imu_sem   = xSemaphoreCreateBinary();
    configASSERT(s_timer_sem && s_imu_sem);

    /* GPIO 33 — frequency proof toggle output */
    gpio_reset_pin(GPIO_FREQ_PROOF);
    gpio_set_direction(GPIO_FREQ_PROOF, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_FREQ_PROOF, 0);

    /* GPIO 12 — Core 1 debug toggle (init here; driven by processing task) */
    gpio_reset_pin(GPIO_DBG_CORE1);
    gpio_set_direction(GPIO_DBG_CORE1, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_DBG_CORE1, 0);

    /* ADC */
    ESP_ERROR_CHECK(adc_init());

    /* I2C master bus — non-fatal so a bad SCL/SDA pin does not abort boot */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = GPIO_SDA,
        .scl_io_num        = GPIO_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t i2c_ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (i2c_ret != ESP_OK) {
        ESP_LOGE(TAG, "ACQ: I2C bus init FAILED (%s) — IMU disabled",
                 esp_err_to_name(i2c_ret));
        /* s_imu_ok stays false; skip all BNO085 setup */
        goto start_timer;
    }

    /* BNO085 — non-fatal: probe first, then open sh2 */
    if (bno085_init() == ESP_OK) {
        s_imu_ok = true;
        /* Only register INT ISR when the sensor is confirmed present */
        gpio_reset_pin(GPIO_BNO_INT);
        gpio_set_direction(GPIO_BNO_INT, GPIO_MODE_INPUT);
        gpio_set_pull_mode(GPIO_BNO_INT, GPIO_PULLUP_ONLY);
        gpio_set_intr_type(GPIO_BNO_INT, GPIO_INTR_NEGEDGE);
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_BNO_INT, bno085_int_isr, NULL));
    } else {
        ESP_LOGE(TAG, "ACQ: BNO085 init FAILED — FSR-only mode");
    }

start_timer:;

    /* 100 Hz hardware timer */
    esp_timer_create_args_t timer_args = {
        .callback        = timer_isr_cb,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK, /* ISR dispatch requires CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD */
        .name            = "acq_timer",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t timer_h;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_h));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_h, 10000)); /* 10 000 µs = 100 Hz */

    ESP_LOGI(TAG, "ACQ TASK: running on Core %d at 100 Hz", xPortGetCoreID());
    return ESP_OK;
}

/* ── Acquisition task (Core 0) ──────────────────────────────────────────── */
/*
 * This task runs exclusively on Core 0 (PRO CPU).
 *
 * It is a tight producer loop: wake → sample → send → sleep.
 * All heavy work (filtering, thresholds, UART) happens on Core 1.
 *
 * The 100 Hz cadence is driven by s_timer_sem, not by vTaskDelay.
 * Using a semaphore rather than a delay means the task wakes at exactly
 * the moment the hardware timer fires, not after an OS tick rounding error.
 */
void acquisition_task(void *arg)
{
    (void)arg;

    static const adc_channel_t FSR_CHANNELS[3] = {
        ADC_CHANNEL_6, /* GPIO 34 — thumb  (ADC1_CH6) */
        ADC_CHANNEL_3, /* GPIO 39 — index  (ADC1_CH3) */
        ADC_CHANNEL_4, /* GPIO 32 — middle (ADC1_CH4) */
    };

    int64_t  last_rate_t  = esp_timer_get_time();
    int      rate_cnt     = 0;

    for (;;) {
        /*
         * STEP 1 — wait for the 100 Hz timer tick.
         *
         * The esp_timer fires every 10 000 µs on Core 0 (ESP_TIMER_TASK
         * dispatch).  Its callback (timer_isr_cb) takes < 5 µs:
         *   1. toggles GPIO 33 (frequency proof)
         *   2. calls xSemaphoreGive(s_timer_sem)
         *   3. returns
         *
         * This task then wakes from portMAX_DELAY and owns Core 0 for
         * the rest of the 10 ms window.
         */
        xSemaphoreTake(s_timer_sem, portMAX_DELAY);

        raw_sample_t samp;
        memset(&samp, 0, sizeof(samp));
        samp.timestamp_us = esp_timer_get_time();

        /*
         * STEP 2 — sample ADC.
         *
         * adc_oneshot_read is synchronous and takes ~50 µs per channel.
         * Three channels = ~150 µs total.  Well within the 10 ms window.
         * Voltage is converted to Newtons via the FSR 402 LUT.
         */
        /* ── ADC: read all 3 FSR channels ─────────────────────────────── */
        for (int i = 0; i < 3; i++) {
            int raw = 0;
            int v_mv = 0;
            adc_oneshot_read(s_adc, FSR_CHANNELS[i], &raw);
            adc_cali_raw_to_voltage(s_cali, raw, &v_mv);
            samp.fsr_n[i] = fsr402_mv_to_newton((float)v_mv);
        }

        /*
         * STEP 3 — read IMU (if present).
         *
         * s_imu_ok is set once in acquisition_init() and never changes.
         * When false (IMU not wired or probe failed), this entire block
         * is skipped — no semaphore wait, no I2C, no sh2 call.
         * IMU fields in samp remain zero from the memset above.
         *
         * When true: s_imu_sem is given by the BNO085 INT falling-edge ISR
         * (bno085_int_isr on GPIO 15).  We take it non-blocking (timeout=0)
         * so we never stall waiting for the IMU.  If the semaphore is not
         * ready this tick, we reuse the previous sample's IMU values.
         */
        /* ── IMU: service sh2 only if BNO085 initialised successfully ─────── */
        if (s_imu_ok) {
            if (xSemaphoreTake(s_imu_sem, 0) == pdTRUE) {
                sh2_service(); /* pumps SHTP, fires sh2_sensor_cb callbacks */
            }
            memcpy(samp.accel_g,  (void *)s_imu_data.accel_g,  sizeof(samp.accel_g));
            memcpy(samp.gyro_dps, (void *)s_imu_data.gyro_dps, sizeof(samp.gyro_dps));
            memcpy(samp.quat,     (void *)s_imu_data.quat,      sizeof(samp.quat));
            memcpy(samp.euler_deg,(void *)s_imu_data.euler,     sizeof(samp.euler_deg));
        }
        /* If s_imu_ok is false, IMU fields remain zero from memset above */

        /*
         * STEP 4 — hand off to Core 1.
         *
         * xQueueSend copies samp by value into the queue's internal DRAM
         * buffer (thread-safe across cores).  Timeout=0 means non-blocking:
         * if Core 1 is behind, we drop the oldest sample rather than
         * stalling Core 0.  Core 0 must never block on Core 1's pace.
         */
        /* ── Post to processing queue ──────────────────────────────────── */
        if (xQueueSend(s_out_q, &samp, 0) != pdTRUE) {
            /* Queue full — Core 1 is behind; drop oldest to keep data fresh */
            raw_sample_t discard;
            xQueueReceive(s_out_q, &discard, 0);
            xQueueSend(s_out_q, &samp, 0);
        }

        /* ── Actual rate measurement every 100 samples ─────────────────── */
        if (++rate_cnt >= 100) {
            int64_t now = esp_timer_get_time();
            float actual_hz = 100.0f / ((float)(now - last_rate_t) * 1e-6f);
            ESP_LOGI(TAG, "measured period = %.3f ms (%.2f Hz)",
                     1000.0f / actual_hz, actual_hz);
            last_rate_t = now;
            rate_cnt    = 0;
        }
    }
}
