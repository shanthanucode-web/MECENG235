#include "led_status.h"
#include "data_types.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "LED_STATUS";

static esp_timer_handle_t s_led_timer;
static volatile int s_led_level = 0;

/* The LED uses esp_timer for the same reason the rest of the project does:
 * blinking should never block a task just to wait on wall-clock time. */
static void led_timer_cb(void *arg)
{
    (void)arg;
    s_led_level ^= 1;
    gpio_set_level(GPIO_STATUS_LED, s_led_level);
}

static void led_timer_stop(void)
{
    if (s_led_timer != NULL && esp_timer_is_active(s_led_timer)) {
        ESP_ERROR_CHECK(esp_timer_stop(s_led_timer));
    }
}

void led_status_init(void)
{
    gpio_reset_pin(GPIO_STATUS_LED);
    gpio_set_direction(GPIO_STATUS_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_STATUS_LED, 0);
    s_led_level = 0;

    if (s_led_timer == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = led_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "status_led",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_led_timer));
    }

    ESP_LOGI(TAG, "Status LED initialized on GPIO%d", (int)GPIO_STATUS_LED);
}

void led_status_off(void)
{
    led_timer_stop();
    s_led_level = 0;
    gpio_set_level(GPIO_STATUS_LED, 0);
}

void led_status_solid_on(void)
{
    led_timer_stop();
    s_led_level = 1;
    gpio_set_level(GPIO_STATUS_LED, 1);
}

void led_status_blink_hz(uint32_t hz)
{
    if (hz == 0) {
        led_status_off();
        return;
    }

    led_timer_stop();
    s_led_level = 1;
    gpio_set_level(GPIO_STATUS_LED, 1);

    uint64_t half_period_us = 1000000ULL / ((uint64_t)hz * 2ULL);
    if (half_period_us == 0) {
        half_period_us = 1;
    }
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_led_timer, half_period_us));
}
