#include "sh2_hal_esp32.h"

#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "SH2_HAL";

#define SH2_I2C_TIMEOUT_MS  50

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t                 addr;
} sh2_hal_ctx_t;

/* Module-level context (only one BNO085 instance expected) */
static sh2_hal_ctx_t s_ctx;

/* ── HAL callbacks ──────────────────────────────────────────────────────── */

static int hal_open(sh2_Hal_t *self)
{
    (void)self;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = s_ctx.addr,
        .scl_speed_hz    = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(s_ctx.bus, &cfg, &s_ctx.dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(ret));
        return -1;
    }
    return 0;
}

static void hal_close(sh2_Hal_t *self)
{
    (void)self;
    i2c_master_bus_rm_device(s_ctx.dev);
    s_ctx.dev = NULL;
}

/*
 * Read from the BNO085.
 *
 * The SHTP protocol begins every transfer with a 4-byte header whose first two
 * bytes contain the cargo length (little-endian, MSB continuation bit masked).
 * We perform a two-phase read:
 *   Phase 1: read exactly 4 bytes to get the header.
 *   Phase 2: if cargo length > 4, read the remaining bytes.
 *
 * Returns the total number of bytes placed in pBuffer, or 0 if no data ready.
 */
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    (void)self;
    *t_us = (uint32_t)esp_timer_get_time();

    if (len < 4) {
        return 0;
    }

    /* Read the 4-byte SHTP header first */
    esp_err_t ret = i2c_master_receive(s_ctx.dev, pBuffer, 4, SH2_I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return 0;
    }

    /* Decode cargo length from header bytes 0–1 (mask continuation bit) */
    unsigned cargo_len = ((unsigned)pBuffer[0] | ((unsigned)(pBuffer[1] & 0x7F) << 8));
    if (cargo_len == 0 || cargo_len > len) {
        return (cargo_len == 0) ? 0 : 4;
    }

    if (cargo_len > 4) {
        /* Read the rest of the transfer */
        ret = i2c_master_receive(s_ctx.dev, pBuffer + 4, cargo_len - 4, SH2_I2C_TIMEOUT_MS);
        if (ret != ESP_OK) {
            return 4;
        }
    }
    return (int)cargo_len;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    (void)self;
    esp_err_t ret = i2c_master_transmit(s_ctx.dev, pBuffer, (size_t)len, SH2_I2C_TIMEOUT_MS);
    return (ret == ESP_OK) ? (int)len : 0;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

/* ── Public initializer ─────────────────────────────────────────────────── */

void sh2_hal_esp32_init(sh2_Hal_t *hal,
                        i2c_master_bus_handle_t bus,
                        uint8_t i2c_addr)
{
    s_ctx.bus  = bus;
    s_ctx.addr = i2c_addr;
    s_ctx.dev  = NULL;

    hal->open      = hal_open;
    hal->close     = hal_close;
    hal->read      = hal_read;
    hal->write     = hal_write;
    hal->getTimeUs = hal_getTimeUs;
}
