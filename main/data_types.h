#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

/* ── Tuning constants ─────────────────────────────────────────────────── */
#define SAMPLE_QUEUE_DEPTH  10
#define SAMPLE_RATE_HZ      100
#define MOTORS_ENABLED      0   /* compile-time guard: set to 1 to enable motors */

/* ── GPIO pin assignments ──────────────────────────────────────────────── */
#define GPIO_FSR0        GPIO_NUM_34   /* ADC1_CH6 — input-only */
#define GPIO_FSR1        GPIO_NUM_39   /* ADC1_CH3 — input-only */
#define GPIO_FSR2        GPIO_NUM_36   /* ADC1_CH0 — moved from GPIO 32 */
#define GPIO_IMU_RX      GPIO_NUM_32   /* UART2 RX — BNO085 UART-RVC output */
#define GPIO_IMU_TX      GPIO_NUM_33   /* BNO085 SCL/UART input — unused in RVC */
#define GPIO_MOTOR0      GPIO_NUM_25
#define GPIO_MOTOR1      GPIO_NUM_26
#define GPIO_MOTOR2      GPIO_NUM_27
#define GPIO_MOTOR3      GPIO_NUM_14
#define GPIO_STATUS_LED  GPIO_NUM_13  /* built-in Feather/HUZZAH32 user LED */
#define GPIO_FREQ_PROOF  GPIO_NUM_4   /* A5 — toggled every timer ISR for frequency verification */
#define GPIO_DBG_CORE1   GPIO_NUM_12  /* toggled every Core 1 processing cycle */

/* ── IMU — BNO085 UART-RVC ────────────────────────────────────────────── */
#define IMU_UART_NUM   UART_NUM_2
#define IMU_UART_BAUD  115200   /* UART-RVC mode (P0=3.3V) */

/* ── Inter-core data packet: produced on Core 0, consumed on Core 1 ─────── */
/*
 * raw_sample_t is the unit of work that crosses the core boundary.
 *
 * Flow:
 *   Core 0 (acquisition_task)
 *     fills one raw_sample_t per 100 Hz tick
 *     → xQueueSend(raw_q, &samp, 0)          [non-blocking]
 *
 *   Core 1 (processing_task)
 *     xQueueReceive(raw_q, &samp, portMAX_DELAY)  [blocks until data ready]
 *     runs full processing pipeline on the copy
 *
 * The FreeRTOS queue copies the struct by value into its internal ring
 * buffer (in shared DRAM), so there is no shared pointer and no need for
 * any additional synchronisation between cores.
 *
 * Size: 4 + (3+3+3+4+3) × 4 = 4 + 64 = 68 bytes per sample.
 * At 100 Hz: 6.8 kB/s across the queue.
 */
typedef struct {
    int64_t timestamp_us;    /* esp_timer_get_time() when ADC was sampled (Core 0) */
    float   fsr_n[3];        /* force in Newtons: [0]=thumb, [1]=index, [2]=middle */
    float   accel_g[3];      /* linear acceleration in g: x, y, z */
    float   gyro_dps[3];     /* angular velocity in deg/s: x, y, z */
    float   quat[4];         /* quaternion: w, x, y, z from BNO085 game rotation vector */
    float   euler_deg[3];    /* roll, pitch, yaw in degrees (derived from quat) */
} raw_sample_t;

/* ── Board state (shared between acquisition and processing) ────────────── */
typedef enum {
    BOARD_IDLE   = 0,
    BOARD_HOLD,
    BOARD_ACTIVE,
    BOARD_EXITED,
} board_state_t;

/* ── Calibration parameters (persisted to NVS) ──────────────────────────── */
typedef struct {
    float mu[3];           /* FSR baseline mean per finger (N) */
    float sigma[3];        /* FSR baseline std dev per finger (N) */
    float on_thresh[3];    /* contact-ON  threshold = max(0.30, mu[i] + 5*sigma[i]) */
    float off_thresh[3];   /* contact-OFF threshold = max(0.15, mu[i] + 3*sigma[i]) */
    float gyro_bias[3];    /* gyroscope DC bias in deg/s */
    float q_neutral[4];    /* quaternion at neutral rest pose: w, x, y, z */
    float f_ref_open;      /* open-grip FSR reference (N), from C3 calibration */
    float f95_ref;         /* 95% power frequency reference (Hz) */
    float pp_roll_ref;     /* peak-to-peak roll reference (deg) */
    float pp_pitch_ref;    /* peak-to-peak pitch reference (deg) */
    float tremor_rms_ref;  /* no-motion 6-12 Hz gyro RMS baseline from C1 (deg/s) */
} cal_params_t;

/* ── Clinical force thresholds (Horeman et al. 2010) ────────────────────── */
/* Expert mean force: 0.9 N (Horeman et al. 2010)                           */
/* Novice mean force: 2.1 N (Horeman et al. 2010)                           */
/* Novice max force:  4.7 N (Horeman et al. 2010)                           */
/* Warning at 2.0 N = above expert mean, below novice mean                  */
/* Error   at 4.0 N = near novice maximum                                   */
#define FORCE_WARN_SUM_N       2.0f  /* F_sum warning threshold (N) */
#define FORCE_ERR_SUM_N        4.0f  /* F_sum error threshold   (N) */
#define FORCE_WARN_FINGER_N    0.8f  /* per-finger warning (N)      */
#define FORCE_ERR_FINGER_N     1.5f  /* per-finger error   (N)      */
#define FORCE_SUSTAIN_SAMPLES  10    /* 100 ms sustain at 100 Hz    */

/* Force mode multipliers applied to cal_params_t.f_ref_open */
#define FORCE_EASY_WARN_X   2.5f
#define FORCE_EASY_ERR_X    4.0f
#define FORCE_MED_WARN_X    2.0f
#define FORCE_MED_ERR_X     3.5f
#define FORCE_HARD_WARN_X   1.5f
#define FORCE_HARD_ERR_X    2.5f

/* Tremor mode thresholds. Values are applied above the C1 no-motion baseline. */
#define TREMOR_EASY_WARN_DPS   18.0f
#define TREMOR_EASY_ERR_DPS    28.0f
#define TREMOR_MED_WARN_DPS    12.0f
#define TREMOR_MED_ERR_DPS     20.0f
#define TREMOR_HARD_WARN_DPS    8.0f
#define TREMOR_HARD_ERR_DPS    14.0f
#define TREMOR_BASELINE_FLOOR_DPS  3.0f

/* ── Warning / error bitmasks ────────────────────────────────────────────── */
#define WARN_HOLD_INSTABILITY   (1u << 0)
#define WARN_TREMOR             (1u << 1)
#define WARN_SMOOTHNESS         (1u << 2)
#define WARN_SWING_RATE         (1u << 3)
#define WARN_FORCE_OPEN         (1u << 4)
#define WARN_FORCE_VARIABILITY  (1u << 5)
#define WARN_FORCE_SPIKE        (1u << 6)

#define ERR_HOLD_INSTABILITY    (1u << 0)
#define ERR_TREMOR              (1u << 1)
#define ERR_FORCE_OPEN          (1u << 2)
#define ERR_SUSTAINED_COMPRESS  (1u << 3)
