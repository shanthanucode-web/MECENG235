#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "data_types.h"
#include "acquisition.h"
#include "processing.h"
#include "motor_control.h"
#include "nvs_storage.h"

static const char *TAG = "MAIN";

/* Redirect ESP_LOG output to /dev/null after the startup banner has been
 * printed.  All runtime data flows through uart_write_bytes (JSON + command
 * responses) which bypasses the VFS layer and writes directly to the driver
 * TX ring buffer — keeping log messages from fragmenting JSON lines. */
static int null_log_vprintf(const char *fmt, va_list args)
{
    (void)fmt;
    (void)args;
    return 0;
}

void app_main(void)
{
    /* ── Startup banner (printed via ROM UART before driver install) ──── */
    printf("=== HAPTIC SURGICAL SKILL TRAINER ===\r\n");
    printf("Firmware: v1.0 | ESP-IDF v6.0\r\n");
    printf("Core 0: Acquisition Task (100 Hz)\r\n");
    printf("Core 1: Processing Task (event-driven)\r\n");
    printf("Loading calibration from NVS...\r\n");

    /* ── NVS flash init ─────────────────────────────────────────────── */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    /* ── Load or derive calibration parameters ──────────────────────── */
    static cal_params_t s_cal; /* static so it outlives app_main */
    if (nvs_load_calibration(&s_cal) == ESP_OK) {
        printf("CAL: loaded from NVS\r\n");
    } else {
        nvs_get_defaults(&s_cal);
        printf("CAL: using defaults\r\n");
    }

    printf("Interrupts: Timer ISR | BNO085 INT | UART RX\r\n");

    /* ── Inter-core queue ───────────────────────────────────────────── */
    /*
     * This queue is the ONLY communication channel between the two cores.
     *
     *   Core 0 (acquisition_task) → xQueueSend()    — producer
     *   Core 1 (processing_task)  → xQueueReceive() — consumer
     *
     * FreeRTOS makes queue operations thread-safe across cores internally
     * using a short critical section; no application-level mutex is needed.
     *
     * Depth of SAMPLE_QUEUE_DEPTH (10) gives Core 1 up to 100 ms of slack
     * before Core 0 must start dropping samples (100 Hz × 10 slots = 100 ms).
     * Each slot holds one raw_sample_t (76 bytes): timestamp + FSR forces +
     * IMU data. The queue is allocated from shared DRAM, accessible by both cores.
     */
    QueueHandle_t raw_q = xQueueCreate(SAMPLE_QUEUE_DEPTH, sizeof(raw_sample_t));
    configASSERT(raw_q != NULL);

    /* ── Hardware init ──────────────────────────────────────────────── */
    ESP_ERROR_CHECK(acquisition_init(raw_q));
    motor_control_init();

    /* ── UART driver install ────────────────────────────────────────── */
    /* TX buffer (4096 B) prevents uart_write_bytes from blocking at 20 Hz JSON output.
     * At 115200 baud the driver drains ~11.5 kB/s; 20 Hz × 640 B ≈ 12.8 kB/s —
     * the TX buffer absorbs brief bursts without stalling the processing task. */
    uart_config_t uart_cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_cfg));
    /* UART0 default pins (GPIO1=TX, GPIO3=RX) are set by the ROM bootloader;
     * uart_set_pin is not needed unless remapping. */

    QueueHandle_t uart_event_q;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,
                                        2048,          /* RX ring buffer */
                                        4096,          /* TX ring buffer */
                                        10,            /* event queue depth */
                                        &uart_event_q,
                                        0));

    /* Silence ESP_LOG so log messages do not fragment JSON output */
    esp_log_set_vprintf(null_log_vprintf);

    /* ── Processing module init ─────────────────────────────────────── */
    processing_init(raw_q, uart_event_q, &s_cal);

    /* ── Spawn tasks — each pinned to a dedicated core ─────────────── */
    /*
     * xTaskCreatePinnedToCore is an ESP-IDF extension of xTaskCreate.
     * The last argument (0 or 1) is the core affinity — the scheduler
     * will ONLY ever run that task on the specified core, regardless of
     * load on the other core.  This is what makes the architecture truly
     * dual-core rather than time-sliced on one core.
     *
     * Core 0 — PRO CPU — acquisition_task
     *   Dedicated to hard-real-time sensor sampling.  Nothing else
     *   runs here that could introduce jitter.  Priority 10 ensures
     *   the esp_timer task (priority 22) can still pre-empt it to
     *   fire the 100 Hz semaphore, but no lower-priority work competes.
     *   Stack: 4096 B — only needs ADC reads, I2C calls, and queue post.
     *
     * Core 1 — APP CPU — processing_task
     *   Handles all computation, filtering, JSON formatting, and UART I/O.
     *   Priority 9 (one below acquisition).  Because these tasks are on
     *   different cores, relative priority between them does NOT cause
     *   preemption — it only matters if both ever needed the same core
     *   (they don't).  Stack: 8192 B — larger to accommodate the DFT
     *   x[64] local array and 640-byte JSON snprintf buffer.
     */
    xTaskCreatePinnedToCore(acquisition_task,
                            "acq_task",
                            4096,   /* stack: ADC + I2C + queue post */
                            NULL,
                            10,     /* priority */
                            NULL,
                            0);     /* ← Core 0 (PRO CPU) */

    xTaskCreatePinnedToCore(processing_task,
                            "proc_task",
                            8192,   /* stack: DFT + IIR state + JSON buf */
                            NULL,
                            9,      /* priority */
                            NULL,
                            1);     /* ← Core 1 (APP CPU) */

    uart_write_bytes(UART_NUM_0, "System ready. Awaiting GUI connection...\r\n", 42);
    uart_write_bytes(UART_NUM_0, "Commands>\r\n", 11);

    /* app_main returns here.  The FreeRTOS scheduler takes over and runs
     * acq_task on Core 0 and proc_task on Core 1 independently and in
     * parallel for the lifetime of the device. */
    ESP_LOGI(TAG, "Tasks launched");
}
