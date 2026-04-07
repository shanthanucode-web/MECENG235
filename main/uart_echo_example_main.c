/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */
 #define CONFIG_EXAMPLE_UART_BAUD_RATE 115200


#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define DEFAULT_PERIOD 1000

static uint8_t s_led_state = 1;

static uint32_t flash_period = DEFAULT_PERIOD;
static uint32_t flash_period_dec = DEFAULT_PERIOD/10;


TaskHandle_t myTaskHandle = NULL;

#define BUF_SIZE (1024)

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(13, s_led_state);
}

static void blink_task(void *arg)
{
    while(1)
    {
    s_led_state = !s_led_state;
    blink_led();
    vTaskDelay(flash_period/ portTICK_PERIOD_MS);
    }

}

static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    uart_write_bytes(ECHO_UART_PORT_NUM, "Commands", strlen("Commands"));
    while (1)
    {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        // Write data back to the UART
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
        if (len)
        {
            data[len] = '\0';
            switch(data[0])
            {
                case 'I':
                    s_led_state = 1;
                    blink_led();
                    uart_write_bytes(ECHO_UART_PORT_NUM, "ESP32", strlen("ESP32"));
                    break;
                case 'T':
                    flash_period -= flash_period_dec;
                    if(flash_period <= flash_period_dec) flash_period = flash_period_dec;
                    break;
                case 'A':
                    vTaskResume(myTaskHandle);
                    break;
                case 'B':
                    vTaskSuspend(myTaskHandle);
                    break;
                case 'R':
                    flash_period = DEFAULT_PERIOD;
                    break;
                default:
                    break;
            }
        }
    }
}

void app_main(void)
{
    gpio_reset_pin(13);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    blink_led();

    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(blink_task, "blink_LED", 1024, NULL, 5, &myTaskHandle);
    vTaskSuspend(myTaskHandle);
}
