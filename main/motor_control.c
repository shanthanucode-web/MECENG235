#include "motor_control.h"
#include "data_types.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

static const gpio_num_t MOTOR_PINS[4] = {
    GPIO_MOTOR0, GPIO_MOTOR1, GPIO_MOTOR2, GPIO_MOTOR3
};

#if MOTORS_ENABLED

static esp_timer_handle_t s_timers[4];

static void motor_off_cb(void *arg)
{
    int idx = (int)(intptr_t)arg;
    gpio_set_level(MOTOR_PINS[idx], 0);
}

void motor_control_init(void)
{
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(MOTOR_PINS[i]);
        gpio_set_direction(MOTOR_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(MOTOR_PINS[i], 0);

        esp_timer_create_args_t ta = {
            .callback        = motor_off_cb,
            .arg             = (void *)(intptr_t)i,
            .dispatch_method = ESP_TIMER_TASK,
            .name            = "motor_off",
        };
        ESP_ERROR_CHECK(esp_timer_create(&ta, &s_timers[i]));
    }
    ESP_LOGI(TAG, "Motor GPIOs init (ENABLED)");
}

void motor_pulse(uint8_t idx, uint32_t duration_ms)
{
    if (idx >= 4) {
        return;
    }
    gpio_set_level(MOTOR_PINS[idx], 1);
    if (esp_timer_is_active(s_timers[idx])) {
        esp_timer_stop(s_timers[idx]);
    }
    esp_timer_start_once(s_timers[idx], (uint64_t)duration_ms * 1000UL);
}

#else   /* MOTORS_ENABLED == 0 */

void motor_control_init(void)
{
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(MOTOR_PINS[i]);
        gpio_set_direction(MOTOR_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(MOTOR_PINS[i], 0);
    }
    ESP_LOGI(TAG, "Motor GPIOs init (DISABLED by compile flag)");
}

void motor_pulse(uint8_t idx, uint32_t duration_ms)
{
    (void)idx;
    (void)duration_ms;
}

#endif  /* MOTORS_ENABLED */
