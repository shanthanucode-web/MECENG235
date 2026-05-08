#include "control.h"

#include "acquisition.h"
#include "led_status.h"
#include "processing.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/uart.h"

#include <stdio.h>
#include <string.h>

static QueueHandle_t   s_uart_q;
static bool            s_mt_mode_enabled = false;

static bool post_processing_cmd(processing_control_cmd_t cmd, char step)
{
    processing_control_msg_t msg = {
        .cmd = cmd,
        .step = step,
    };

    if (processing_submit_control(&msg, pdMS_TO_TICKS(50))) {
        return true;
    }

    uart_write_bytes(UART_NUM_0,
                     "{\"err\":\"control_queue_busy\"}\r\n",
                     30);
    return false;
}

static void dispatch_command(const char *cmd, int len)
{
    if (len <= 0) {
        return;
    }

    char response[64];

    if (len >= 5 && strncmp(cmd, "MT_ON", 5) == 0) {
        acquisition_mt_trace_set_enabled(true);
        processing_set_mt_mode(true);
        s_mt_mode_enabled = true;
        uart_write_bytes(UART_NUM_0, "{\"mt\":\"ON\"}\r\n", 13);
        return;
    }
    if (len >= 6 && strncmp(cmd, "MT_OFF", 6) == 0) {
        s_mt_mode_enabled = false;
        processing_set_mt_mode(false);
        acquisition_mt_trace_set_enabled(false);
        uart_write_bytes(UART_NUM_0, "{\"mt\":\"OFF\"}\r\n", 14);
        return;
    }
    if (len >= 6 && strncmp(cmd, "C3_REP", 6) == 0) {
        led_status_blink_hz(30);
        post_processing_cmd(PROCESSING_CTRL_CAL_C3_REP, 0);
        return;
    }
    if (len >= 8 && strncmp(cmd, "C4_CYCLE", 8) == 0) {
        led_status_blink_hz(30);
        post_processing_cmd(PROCESSING_CTRL_CAL_C4_CYCLE, 0);
        return;
    }
    if (len >= 2 && cmd[0] == 'C' && cmd[1] >= '1' && cmd[1] <= '4') {
        led_status_blink_hz(30);
        post_processing_cmd(PROCESSING_CTRL_CAL_RUN_STEP, cmd[1]);
        return;
    }

    switch (cmd[0]) {
    case 'I':
        led_status_solid_on();
        post_processing_cmd(PROCESSING_CTRL_IDENTIFY, 0);
        break;
    case 'S':
        led_status_off();
        post_processing_cmd(PROCESSING_CTRL_STOP, 0);
        break;
    case 'E':
        led_status_blink_hz(1);
        post_processing_cmd(PROCESSING_CTRL_SET_EASY, 0);
        break;
    case 'M':
        led_status_blink_hz(2);
        post_processing_cmd(PROCESSING_CTRL_SET_MEDIUM, 0);
        break;
    case 'H':
        led_status_blink_hz(4);
        post_processing_cmd(PROCESSING_CTRL_SET_HARD, 0);
        break;
    case 'X':
        led_status_off();
        post_processing_cmd(PROCESSING_CTRL_EXIT, 0);
        break;
    case 'Z':
        post_processing_cmd(PROCESSING_CTRL_RESET_CAL, 0);
        break;
    default: {
        int n = snprintf(response, sizeof(response),
                         "{\"err\":\"unknown_cmd\",\"cmd\":\"%c\"}\r\n", cmd[0]);
        uart_write_bytes(UART_NUM_0, response, (size_t)n);
        break;
    }
    }
}

static bool poll_uart_once(TickType_t timeout_ticks)
{
    uart_event_t ev;

    if (xQueueReceive(s_uart_q, &ev, timeout_ticks) != pdTRUE) {
        return false;
    }

    if (ev.type == UART_DATA) {
        uint8_t cmd_buf[32];
        memset(cmd_buf, 0, sizeof(cmd_buf));
        int n_read = uart_read_bytes(UART_NUM_0, cmd_buf,
                                     (ev.size < 31) ? (uint32_t)ev.size : 31U,
                                     0);
        if (n_read > 0) {
            dispatch_command((char *)cmd_buf, n_read);
        }
    }

    return true;
}

void control_init(QueueHandle_t uart_event_q)
{
    s_uart_q = uart_event_q;
}

void control_task(void *arg)
{
    (void)arg;

    for (;;) {
        if (s_mt_mode_enabled) {
            acquisition_mt_trace_ctrl_enter();

            bool did_uart = poll_uart_once(0);
            if (!did_uart) {
                taskYIELD();
            }
            continue;
        }

        acquisition_mt_trace_ctrl_exit();
        poll_uart_once(portMAX_DELAY);
    }
}
