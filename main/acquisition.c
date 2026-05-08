#include "acquisition.h"
#include "data_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#define G_MS2_TO_G  9.81f

static const char *TAG = "ACQ";

/* ── Module state ──────────────────────────────────────────────────────── */
static QueueHandle_t             s_out_q;
static SemaphoreHandle_t         s_timer_sem;
static QueueHandle_t             s_mt_trace_q;

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;

/* GPIO toggle state for freq proof pin */
static volatile int s_freq_state = 0;

/*
 * Diagnostic buffer — filled during acquisition_init() before the UART0 driver
 * is installed, then flushed via uart_write_bytes() at the start of
 * acquisition_task().
 */
static char s_diag_buf[512];
static int  s_diag_len = 0;

#define MT_TRACE_QUEUE_DEPTH  128

static volatile bool     s_mt_trace_enabled = false;
static volatile uint32_t s_mt_trace_seq = 0;
static volatile uint32_t s_mt_trace_dropped = 0;
static volatile uint32_t s_mt_cycle_id = 0;
static volatile bool     s_mt_ctrl_running = false;
static volatile bool     s_mt_ctrl_resume_armed = false;
static volatile uint32_t s_mt_resume_cycle_id = 0;

static void diag_append(const char *fmt, ...)
{
    int remaining = (int)sizeof(s_diag_buf) - s_diag_len;
    if (remaining <= 1) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_diag_buf + s_diag_len, (size_t)remaining, fmt, ap);
    va_end(ap);
    if (n > 0) {
        s_diag_len += (n < remaining) ? n : (remaining - 1);
    }
}

static void mt_trace_emit_ex(mt_trace_event_code_t code,
                             uint32_t              cycle_id,
                             uint32_t              aux)
{
    if (!s_mt_trace_enabled || s_mt_trace_q == NULL) {
        return;
    }

    mt_trace_event_t ev = {
        .seq = __atomic_add_fetch(&s_mt_trace_seq, 1u, __ATOMIC_RELAXED),
        .cycle_id = cycle_id,
        .t_us = esp_timer_get_time(),
        .core_id = (uint8_t)xPortGetCoreID(),
        .code = code,
        .aux = aux,
    };

    if (xQueueSend(s_mt_trace_q, &ev, 0) != pdTRUE) {
        __atomic_add_fetch(&s_mt_trace_dropped, 1u, __ATOMIC_RELAXED);
    }
}

static void mt_trace_emit(mt_trace_event_code_t code)
{
    uint32_t cycle_id = __atomic_load_n(&s_mt_cycle_id, __ATOMIC_RELAXED);
    mt_trace_emit_ex(code, cycle_id, 0u);
}

/* ── FSR 402 voltage → Newton lookup table ─────────────────────────────── */
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

/* ── Timer callback — fires at 100 Hz ───────────────────────────────────── */
static void timer_isr_cb(void *arg)
{
    (void)arg;
    uint32_t cycle_id = __atomic_add_fetch(&s_mt_cycle_id, 1u, __ATOMIC_RELAXED);
    bool ctrl_running = s_mt_ctrl_running;
    mt_trace_emit_ex(MT_TRACE_TIMER_CB_BEGIN, cycle_id, ctrl_running ? 1u : 0u);
    s_freq_state ^= 1;
    gpio_set_level(GPIO_FREQ_PROOF, s_freq_state);
    xSemaphoreGive(s_timer_sem);
    mt_trace_emit_ex(MT_TRACE_SEM_GIVE, cycle_id, 0u);
}

/* ── UART-RVC 19-byte packet reader ─────────────────────────────────────── */
/*
 * Packet layout (BNO085 UART-RVC, P0=3.3V, 115200 baud):
 *   [0]    0xAA  header byte 1
 *   [1]    0xAA  header byte 2
 *   [2]    index (ignored)
 *   [3-4]  yaw   int16 LE, /100.0 = degrees
 *   [5-6]  pitch int16 LE, /100.0 = degrees
 *   [7-8]  roll  int16 LE, /100.0 = degrees
 *   [9-10] ax    int16 LE, /100.0 = m/s²
 *  [11-12] ay    int16 LE, /100.0 = m/s²
 *  [13-14] az    int16 LE, /100.0 = m/s²
 *  [15-18] reserved
 */
static bool rvc_read_packet(float *yaw, float *pitch, float *roll,
                            float *ax, float *ay, float *az)
{
    /* Search for 0xAA 0xAA header — non-blocking, drain up to 64 bytes */
    for (int i = 0; i < 64; i++) {
        uint8_t b0;
        if (uart_read_bytes(IMU_UART_NUM, &b0, 1, 0) != 1) return false;
        if (b0 != 0xAA) continue;
        uint8_t b1;
        if (uart_read_bytes(IMU_UART_NUM, &b1, 1, 0) != 1) return false;
        if (b1 == 0xAA) goto header_found;
        /* b1 wasn't 0xAA; start over treating b1 as a potential first byte */
        if (b1 == 0xAA) break; /* unreachable — silences compiler */
    }
    return false;

header_found:;
    /* Read remaining 17 bytes: index + 6×int16 + 4 reserved */
    uint8_t pkt[17];
    int n = uart_read_bytes(IMU_UART_NUM, pkt, sizeof(pkt), pdMS_TO_TICKS(5));
    if (n < (int)sizeof(pkt)) return false;

    /* pkt[0] = index (ignored) */
    int16_t raw_yaw   = (int16_t)((uint16_t)pkt[1]  | ((uint16_t)pkt[2]  << 8));
    int16_t raw_pitch = (int16_t)((uint16_t)pkt[3]  | ((uint16_t)pkt[4]  << 8));
    int16_t raw_roll  = (int16_t)((uint16_t)pkt[5]  | ((uint16_t)pkt[6]  << 8));
    int16_t raw_ax    = (int16_t)((uint16_t)pkt[7]  | ((uint16_t)pkt[8]  << 8));
    int16_t raw_ay    = (int16_t)((uint16_t)pkt[9]  | ((uint16_t)pkt[10] << 8));
    int16_t raw_az    = (int16_t)((uint16_t)pkt[11] | ((uint16_t)pkt[12] << 8));

    *yaw   = raw_yaw   / 100.0f;
    *pitch = raw_pitch / 100.0f;
    *roll  = raw_roll  / 100.0f;
    *ax    = raw_ax    / 100.0f;
    *ay    = raw_ay    / 100.0f;
    *az    = raw_az    / 100.0f;

    return true;
}

/* Gyro-from-Euler state (differentiator at 100 Hz) */
static float s_prev_roll    = 0.0f;
static float s_prev_pitch   = 0.0f;
static float s_prev_yaw     = 0.0f;
static bool  s_first_sample = true;

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
    /* GPIO 34=ADC1_CH6(FSR0), GPIO 39=ADC1_CH3(FSR1), GPIO 36=ADC1_CH0(FSR2) */
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_6, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_3, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_0, &chan_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
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

    s_timer_sem = xSemaphoreCreateBinary();
    configASSERT(s_timer_sem);

    s_mt_trace_q = xQueueCreate(MT_TRACE_QUEUE_DEPTH, sizeof(mt_trace_event_t));
    configASSERT(s_mt_trace_q);

    gpio_reset_pin(GPIO_FREQ_PROOF);
    gpio_set_direction(GPIO_FREQ_PROOF, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_FREQ_PROOF, 0);

    gpio_reset_pin(GPIO_DBG_CORE1);
    gpio_set_direction(GPIO_DBG_CORE1, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_DBG_CORE1, 0);

    gpio_reset_pin(GPIO_IMU_TX);
    gpio_set_direction(GPIO_IMU_TX, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_IMU_TX, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(adc_init());

    /* IMU — BNO085 UART-RVC mode (P0=3.3V), 115200 baud, RX-only */
    uart_config_t rvc_cfg = {
        .baud_rate  = IMU_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(IMU_UART_NUM, &rvc_cfg));
    ESP_ERROR_CHECK(uart_set_pin(IMU_UART_NUM,
                                 UART_PIN_NO_CHANGE, GPIO_IMU_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(IMU_UART_NUM, 512, 0, 0, NULL, 0));
    diag_append("IMU: BNO085 UART-RVC RX=GPIO%d %d baud\r\n",
                (int)GPIO_IMU_RX, (int)IMU_UART_BAUD);

    esp_timer_create_args_t timer_args = {
        .callback        = timer_isr_cb,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "acq_timer",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t timer_h;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_h));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_h, 10000));

    ESP_LOGI(TAG, "ACQ TASK: running on Core %d at 100 Hz", xPortGetCoreID());
    return ESP_OK;
}

/* ── Acquisition task (Core 0) ──────────────────────────────────────────── */
/*
 * Producer loop: wake (100 Hz timer) → sample ADC → read RVC packet → queue.
 */
void acquisition_task(void *arg)
{
    (void)arg;

    static const adc_channel_t FSR_CHANNELS[3] = {
        ADC_CHANNEL_6, /* GPIO 34 — thumb  (ADC1_CH6) */
        ADC_CHANNEL_3, /* GPIO 39 — index  (ADC1_CH3) */
        ADC_CHANNEL_0, /* GPIO 36 — middle (ADC1_CH0) */
    };

    /* Flush startup diagnostics via UART0 (driver now installed) */
    if (s_diag_len > 0) {
        uart_write_bytes(UART_NUM_0, s_diag_buf, (size_t)s_diag_len);
    }

    int64_t last_rate_t = esp_timer_get_time();
    int     rate_cnt    = 0;

    for (;;) {
        /*
         * STEP 1 — wait for 100 Hz timer tick.
         */
        xSemaphoreTake(s_timer_sem, portMAX_DELAY);
        mt_trace_emit(MT_TRACE_ACQ_WAKE);

        raw_sample_t samp;
        memset(&samp, 0, sizeof(samp));
        samp.timestamp_us = esp_timer_get_time();

        /*
         * STEP 2 — sample all 3 FSR channels via ADC.
         */
        for (int i = 0; i < 3; i++) {
            int raw = 0;
            int v_mv = 0;
            adc_oneshot_read(s_adc, FSR_CHANNELS[i], &raw);
            adc_cali_raw_to_voltage(s_cali, raw, &v_mv);
            samp.fsr_n[i] = fsr402_mv_to_newton((float)v_mv);
        }

        /*
         * STEP 3 — read UART-RVC packet and derive gyro from Euler differences.
         *
         * euler_deg[0]=roll, [1]=pitch, [2]=yaw  (matches processing.c convention)
         * gyro_dps is approximated by finite differences at dt=10 ms.
         */
        float yaw, pitch, roll, ax_ms2, ay_ms2, az_ms2;
        if (rvc_read_packet(&yaw, &pitch, &roll, &ax_ms2, &ay_ms2, &az_ms2)) {
            samp.euler_deg[0] = roll;
            samp.euler_deg[1] = pitch;
            samp.euler_deg[2] = yaw;
            samp.accel_g[0] = ax_ms2 / G_MS2_TO_G;
            samp.accel_g[1] = ay_ms2 / G_MS2_TO_G;
            samp.accel_g[2] = az_ms2 / G_MS2_TO_G;
            memset(samp.quat, 0, sizeof(samp.quat));

            if (!s_first_sample) {
                float dt   = 0.01f;
                float dyaw = yaw - s_prev_yaw;
                if (dyaw >  180.0f) dyaw -= 360.0f;
                if (dyaw < -180.0f) dyaw += 360.0f;
                samp.gyro_dps[0] = (roll  - s_prev_roll)  / dt;
                samp.gyro_dps[1] = (pitch - s_prev_pitch) / dt;
                samp.gyro_dps[2] = dyaw / dt;
            }
            s_first_sample = false;
            s_prev_roll    = roll;
            s_prev_pitch   = pitch;
            s_prev_yaw     = yaw;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        /*
         * STEP 4 — hand off to Core 1 via the inter-core queue.
         */
        if (xQueueSend(s_out_q, &samp, 0) != pdTRUE) {
            raw_sample_t discard;
            xQueueReceive(s_out_q, &discard, 0);
            xQueueSend(s_out_q, &samp, 0);
        }

        if (++rate_cnt >= 100) {
            int64_t now = esp_timer_get_time();
            float actual_hz = 100.0f / ((float)(now - last_rate_t) * 1e-6f);
            ESP_LOGI(TAG, "measured period = %.3f ms (%.2f Hz)",
                     1000.0f / actual_hz, actual_hz);
            last_rate_t = now;
            rate_cnt    = 0;
        }

        s_mt_resume_cycle_id = __atomic_load_n(&s_mt_cycle_id, __ATOMIC_RELAXED);
        s_mt_ctrl_resume_armed = true;
        mt_trace_emit(MT_TRACE_ACQ_DONE);
    }
}

void acquisition_mt_trace_set_enabled(bool enabled)
{
    s_mt_trace_enabled = false;
    s_mt_ctrl_running = false;
    s_mt_ctrl_resume_armed = false;

    if (s_mt_trace_q != NULL) {
        xQueueReset(s_mt_trace_q);
    }

    __atomic_store_n(&s_mt_trace_seq, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&s_mt_trace_dropped, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&s_mt_cycle_id, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&s_mt_resume_cycle_id, 0u, __ATOMIC_RELAXED);

    if (enabled) {
        s_mt_trace_enabled = true;
    }
}

bool acquisition_mt_trace_pop(mt_trace_event_t *out_event)
{
    if (out_event == NULL || s_mt_trace_q == NULL) {
        return false;
    }
    return xQueueReceive(s_mt_trace_q, out_event, 0) == pdTRUE;
}

uint32_t acquisition_mt_trace_dropped(void)
{
    return __atomic_load_n(&s_mt_trace_dropped, __ATOMIC_RELAXED);
}

void acquisition_mt_trace_ctrl_enter(void)
{
    s_mt_ctrl_running = true;

    if (s_mt_trace_enabled && s_mt_ctrl_resume_armed) {
        uint32_t cycle_id = __atomic_load_n(&s_mt_resume_cycle_id, __ATOMIC_RELAXED);
        s_mt_ctrl_resume_armed = false;
        mt_trace_emit_ex(MT_TRACE_CTRL_RESUME, cycle_id, 0u);
    }
}

void acquisition_mt_trace_ctrl_exit(void)
{
    s_mt_ctrl_running = false;
}
