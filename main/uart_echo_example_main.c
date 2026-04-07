/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/**
 * This example receives command bytes on the configured UART and updates the
 * board state accordingly.
 *
 * Supported commands:
 * - I: force the LED on to confirm the UART link is working
 * - E: enter easy mode and blink slowly
 * - H: enter hard mode and blink quickly
 * - S: stop the active mode and return to idle
 * - X: exit the program until the board is reset or reflashed
 */

#define ECHO_TEST_TXD          CONFIG_EXAMPLE_UART_TXD
#define ECHO_TEST_RXD          CONFIG_EXAMPLE_UART_RXD
#define ECHO_TEST_RTS          UART_PIN_NO_CHANGE
#define ECHO_TEST_CTS          UART_PIN_NO_CHANGE
#define ECHO_UART_PORT_NUM     CONFIG_EXAMPLE_UART_PORT_NUM
#define ECHO_UART_BAUD_RATE    CONFIG_EXAMPLE_UART_BAUD_RATE
#define ECHO_TASK_STACK_SIZE   CONFIG_EXAMPLE_TASK_STACK_SIZE

#define LED_GPIO               GPIO_NUM_13
#define BUF_SIZE               1024
#define EASY_BLINK_PERIOD_MS   500
#define HARD_BLINK_PERIOD_MS   100
#define IDLE_POLL_PERIOD_MS    50

typedef enum {
    BOARD_STATE_IDLE = 0,
    BOARD_STATE_CONNECTED,
    BOARD_STATE_EASY,
    BOARD_STATE_HARD,
    BOARD_STATE_EXITED,
} board_state_t;

static volatile board_state_t s_board_state = BOARD_STATE_IDLE;
static uint8_t s_led_state = 1;

static void set_led(bool on)
{
    s_led_state = on ? 1 : 0;
    gpio_set_level(LED_GPIO, s_led_state);
}

static void toggle_led(void)
{
    s_led_state = !s_led_state;
    gpio_set_level(LED_GPIO, s_led_state);
}

static void uart_send_line(const char *message)
{
    uart_write_bytes(ECHO_UART_PORT_NUM, message, strlen(message));
    uart_write_bytes(ECHO_UART_PORT_NUM, "\r\n", 2);
}

static void blink_task(void *arg)
{
    while (1) {
        board_state_t state = s_board_state;

        switch (state) {
        case BOARD_STATE_IDLE:
            set_led(true);
            vTaskDelay(pdMS_TO_TICKS(IDLE_POLL_PERIOD_MS));
            break;

        case BOARD_STATE_CONNECTED:
            set_led(true);
            vTaskDelay(pdMS_TO_TICKS(IDLE_POLL_PERIOD_MS));
            break;

        case BOARD_STATE_EASY:
            toggle_led();
            vTaskDelay(pdMS_TO_TICKS(EASY_BLINK_PERIOD_MS));
            break;

        case BOARD_STATE_HARD:
            toggle_led();
            vTaskDelay(pdMS_TO_TICKS(HARD_BLINK_PERIOD_MS));
            break;

        case BOARD_STATE_EXITED:
            set_led(false);
            vTaskDelete(NULL);
            break;
        }
    }
}

static void echo_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;
    uint8_t data[BUF_SIZE];

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    uart_send_line("READY: I E H S X");

    while (1) {
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(20));

        if (len <= 0) {
            continue;
        }

        data[len] = '\0';

        switch (data[0]) {
        case 'I':
            s_board_state = BOARD_STATE_CONNECTED;
            set_led(true);
            uart_send_line("CONNECTED");
            break;

        case 'E':
            s_board_state = BOARD_STATE_EASY;
            uart_send_line("EASY");
            break;

        case 'H':
            s_board_state = BOARD_STATE_HARD;
            uart_send_line("HARD");
            break;

        case 'S':
            if (s_board_state == BOARD_STATE_EASY || s_board_state == BOARD_STATE_HARD) {
                s_board_state = BOARD_STATE_IDLE;
                set_led(true);
                uart_send_line("IDLE");
            } else {
                uart_send_line("ALREADY IDLE");
            }
            break;

        case 'X':
            s_board_state = BOARD_STATE_EXITED;
            uart_send_line("EXITED");
            vTaskDelete(NULL);
            break;

        default:
            uart_send_line("UNKNOWN COMMAND");
            break;
        }
    }
}

void app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    set_led(false);

    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(blink_task, "blink_led_task", 2048, NULL, 5, NULL);
}
