#include "sh2_hal_esp32.h"

#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "SH2_HAL";

#define SH2_UART_NUM      UART_NUM_2
#define SH2_UART_BAUD     115200
#define SH2_UART_RX_GPIO  32
#define SH2_UART_TX_GPIO  33
#define SH2_RX_BUF_BYTES  4096
#define SH2_TX_BUF_BYTES  1024

typedef struct {
    bool open;
} sh2_hal_ctx_t;

static sh2_hal_ctx_t s_ctx;

/* ── HAL callbacks ──────────────────────────────────────────────────────── */

static int hal_open(sh2_Hal_t *self)
{
    (void)self;

    uart_config_t cfg = {
        .baud_rate  = SH2_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(SH2_UART_NUM, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return -1;
    }

    ret = uart_set_pin(SH2_UART_NUM,
                       SH2_UART_TX_GPIO, SH2_UART_RX_GPIO,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return -1;
    }

    ret = uart_driver_install(SH2_UART_NUM,
                              SH2_RX_BUF_BYTES, SH2_TX_BUF_BYTES,
                              0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return -1;
    }

    s_ctx.open = true;
    return 0;
}

static void hal_close(sh2_Hal_t *self)
{
    (void)self;
    if (s_ctx.open) {
        uart_driver_delete(SH2_UART_NUM);
        s_ctx.open = false;
    }
}

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    (void)self;
    *t_us = (uint32_t)esp_timer_get_time();
    int n = uart_read_bytes(SH2_UART_NUM, pBuffer, (uint32_t)len,
                            pdMS_TO_TICKS(50));
    return (n > 0) ? n : 0;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    (void)self;
    int n = uart_write_bytes(SH2_UART_NUM, pBuffer, (size_t)len);
    return (n > 0) ? n : 0;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

/* ── Public initializer ─────────────────────────────────────────────────── */

void sh2_hal_esp32_init(sh2_Hal_t *hal)
{
    memset(&s_ctx, 0, sizeof(s_ctx));

    hal->open      = hal_open;
    hal->close     = hal_close;
    hal->read      = hal_read;
    hal->write     = hal_write;
    hal->getTimeUs = hal_getTimeUs;
}
