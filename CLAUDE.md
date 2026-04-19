# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32 lab project (Haptic Surgical Skill Trainer — Lab 4) with three parts:
1. **ESP-IDF firmware** (`main/`, `components/`) — C code running on the ESP32 HUZZAH32
2. **Python desktop GUI** (`gui/`) — PySide6 app running on the host computer
3. **FSR diagnostic tool** (`tests/fsr_test.py`) — standalone terminal display for force sensor testing

---

## Firmware (ESP-IDF)

### Build and Flash

```bash
# Activate ESP-IDF environment
source ~/.espressif/tools/activate_idf_v6.0.sh

# Build only
python3 ~/.espressif/v6.0/esp-idf/tools/idf.py build

# Flash and open monitor
python3 ~/.espressif/v6.0/esp-idf/tools/idf.py -p /dev/cu.usbserial-59691016461 flash monitor
```

Exit the monitor with `Ctrl-]`.

Note: `idf.py` is not on PATH by default. Always invoke via `python3 ~/.espressif/v6.0/esp-idf/tools/idf.py`.

### File Structure

```
main/
  main.c               — app_main: queue creation, hardware init, task pinning
  data_types.h         — shared structs (raw_sample_t, cal_params_t) and all #defines
  acquisition.c/.h     — Core 0: 100 Hz timer, ADC (FSR × 3), BNO085, queue post
  processing.c/.h      — Core 1: IIR filters, state machine, thresholds, JSON, commands
  calibration.c/.h     — calibration sub-state machine (C1–C4, NVS save)
  filters.c/.h         — Butterworth biquad IIR: LP 12 Hz, BP 6–12 Hz, LP 10 Hz
  motor_control.c/.h   — vibration motor GPIO pulse (MOTORS_ENABLED=0 by default)
  nvs_storage.c/.h     — NVS load/save for cal_params_t

components/
  sh2/                 — vendored CEVA BNO085 sh2 driver + ESP-IDF I2C HAL
```

---

## Firmware Architecture

### Dual-Core Task Assignment

The firmware uses `xTaskCreatePinnedToCore` to pin each task to a dedicated CPU core.
This is confirmed at boot: `I (200) cpu_start: Multicore app`.

| Task | Core | Stack | Priority | Role |
|------|------|-------|----------|------|
| `acq_task` | 0 (PRO) | 4096 B | 10 | ADC + IMU sampling at 100 Hz |
| `proc_task` | 1 (APP) | 8192 B | 9  | Filtering, state machine, JSON output |

### Inter-Core Communication

The only data path between cores is a **FreeRTOS queue** (`QueueHandle_t raw_q`, depth 10).

- Core 0 calls `xQueueSend(&samp, 0)` — non-blocking, drops oldest if full
- Core 1 calls `xQueueReceive(&samp, portMAX_DELAY)` — blocks until data arrives
- The queue copies `raw_sample_t` by value (no shared pointers, no mutex needed)

### State Machine

`IDLE → HOLD` (any finger contact + omega < 10 deg/s for 150 ms)
`IDLE → ACTIVE` (any finger contact + omega ≥ 10 deg/s)
Any state → `EXITED` (via X command)

### UART Protocol

UART0, 115200 baud, GPIO 1=TX, GPIO 3=RX.

**Commands (host → ESP32):** `I` (identify), `E` (easy), `M` (medium), `H` (hard),
`S` (stop), `C1`–`C4` (calibration steps), `X` (exit), `Z` (erase NVS)

**Telemetry (ESP32 → host):** JSON at 20 Hz (every 5th sample of 100 Hz acquisition).
All output via `uart_write_bytes()` directly — ESP_LOG is silenced after the startup
banner to prevent log messages from fragmenting JSON lines.

Example JSON:
```json
{"t":12340,"f0":0.196,"f1":0.0,"f2":0.0,"ax":0.01,"ay":-0.02,"az":1.0,
 "gx":0.1,"gy":-0.2,"gz":0.0,"roll":1.2,"pitch":-0.5,"yaw":45.0,
 "f_sum":0.196,"tremor":0.02,"f95":1.6,"pp_roll":0.5,"pp_pitch":0.3,
 "cv_f":0.04,"swing":0.0,"contact":[1,0,0],"state":"HOLD",
 "warn":0,"err":0,"score":100.0,"actual_hz":100.00}
```

### IMU-Absent Mode

If the BNO085 is not connected, `bno085_init()` probes the I2C address with
`i2c_master_probe()` (50 ms timeout) before calling `sh2_open()`. On failure it sets
`s_imu_ok = false` and continues — FSR sampling runs normally, IMU fields are zero.

### Force Thresholds (Horeman et al. 2010)

- Per-finger warning: > 0.8 N | Per-finger error: > 1.5 N
- F_sum warning: > 2.0 N sustained 100 ms | F_sum error: > 4.0 N sustained 100 ms
- F_ref_open default: 0.9 N (expert mean)
- Mode multipliers (E/M/H commands) scale warn/err relative to `f_ref_open`

### Key GPIO Assignments

| Signal | GPIO | Notes |
|--------|------|-------|
| FSR thumb | 34 | ADC1_CH6, input-only |
| FSR index | 39 | ADC1_CH3, input-only |
| FSR middle | 32 | ADC1_CH4 |
| BNO085 SDA | 22 | I2C0 |
| BNO085 SCL | 20 | I2C0 |
| BNO085 INT | 15 | falling-edge ISR |
| Freq proof | 33 | toggled each 100 Hz tick |
| Core 1 debug | 12 | toggled each processing cycle |

---

## FSR Diagnostic Tool

Standalone terminal display — does not require the GUI:

```bash
python tests/fsr_test.py              # auto-detect port
python tests/fsr_test.py --list-ports # list all serial ports
python tests/fsr_test.py --log        # also write CSV log
```

Only one process can hold the serial port at a time. Stop `idf.py monitor` before running.

---

## Python GUI

### Run

```bash
pip install -r gui/requirements.txt
python gui/esp32_controller.py
```

Requires Python 3.10+ for `X | Y` union type hints.

---

## GUI Architecture

**Single source file:** `gui/esp32_controller.py`

Three logical layers:

1. **`SerialWorker(QThread)`** — runs blocking `serial.readline()` in a background thread. Commands flow in via `queue.Queue` (main thread puts, worker writes). State changes flow out via Qt signals (`data_received`, `error_occurred`) which Qt auto-marshals to the main thread.

2. **`MainWindow(QMainWindow)`** — all widgets and slots on the Qt main thread. Never touches `serial.Serial` directly.

3. **`LedIndicator(QWidget)`** — custom-painted circle using `QPainter`. Heartbeat animation via `QSequentialAnimationGroup` (rise → fall → pause) on a `Property(float)` glow value.

**State mapping:** `RESPONSE_TO_STATE` dict maps raw firmware strings (e.g. `"READY"`, `"HARD"`) to `BoardState` enum values. `ACTIVITY_MESSAGES` maps `BoardState` to human-readable feed card text. Add entries to both dicts when the firmware protocol changes.

**Terminal toggle:** the raw serial log (`QTextEdit`) is hidden inside `_terminal_container`. `_toggle_terminal()` shows/hides it and resizes the window.

---

## Branches

- `main` — stable LED blink baseline (original demo)
- `pythonGUI` — GUI + early dual-core firmware
- `hardware-integration` — **current**: full sensor firmware (FSR + IMU + calibration + JSON)
