| Supported Targets | ESP32 |
| ----------------- | ----- |

# Haptic Surgical Skill Trainer — Lab 4

ESP32 firmware and Python tools for the VitalSigns Lab 4 haptic surgical skill trainer.
The system measures three-finger grip force via FSR 402 sensors, tracks hand motion via
a BNO085 IMU, and streams live JSON telemetry to a host GUI at 20 Hz.

---

## Dual-Core Architecture

This is the most important design decision in the firmware. The ESP32 HUZZAH32 contains
two independent Xtensa LX6 CPU cores running at 160 MHz. The firmware splits work across
both cores so that sensor sampling is never interrupted by computation.

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ESP32 HUZZAH32                              │
│                                                                     │
│   CORE 0 — PRO CPU (acquisition)                                    │
│   ┌─────────────────────────────────────┐                           │
│   │  esp_timer fires at 100 Hz          │                           │
│   │    → gives s_timer_sem              │                           │
│   │  acquisition_task wakes             │                           │
│   │    → reads ADC (3× FSR 402)         │                           │
│   │    → reads BNO085 via I2C (if OK)   │                           │
│   │    → packs raw_sample_t             │                           │
│   │    → xQueueSend ─────────────────────┼──────────────────┐       │
│   └─────────────────────────────────────┘                  │       │
│                                                             │       │
│                   FreeRTOS queue (10 × raw_sample_t)        │       │
│                   — thread-safe cross-core FIFO             │       │
│                                                             ▼       │
│   CORE 1 — APP CPU (processing)                                     │
│   ┌─────────────────────────────────────┐                           │
│   │  processing_task blocks on          │                           │
│   │    xQueueReceive (waits for data)   │                           │
│   │    → Butterworth IIR filtering      │                           │
│   │    → contact detection              │                           │
│   │    → state machine (IDLE/HOLD/ACTIVE│                           │
│   │    → 11 threshold checks            │                           │
│   │    → motor pulses                   │                           │
│   │    → JSON → uart_write_bytes        │──── UART0 ──► host GUI    │
│   │    → UART RX command handler        │◄─── UART0 ─── host GUI    │
│   └─────────────────────────────────────┘                           │
└─────────────────────────────────────────────────────────────────────┘
```

### Why two cores?

| Concern | Single-core problem | Dual-core solution |
|---|---|---|
| ADC timing jitter | JSON formatting (ms) would delay ADC reads | Core 0 only does ADC + IMU — nothing else |
| 100 Hz guarantee | Heavy computation causes missed timer ticks | Core 0 timer semaphore is never contested |
| JSON latency | 100 Hz × 640 B = 25 kB/s — stalls UART | Core 1 handles all I/O; Core 0 is unaffected |
| ISR safety | UART event ISR + timer ISR on same core = conflicts | ISRs land on Core 0 only; UART events handled Core 1 |

### Confirmed dual-core boot

The boot log confirms multicore mode:
```
I (200) cpu_start: Multicore app
I (208) cpu_start: Pro cpu start user code
```

`xTaskCreatePinnedToCore` with explicit core IDs enforces the assignment.
On a single-core build this function does not exist — its presence is proof of dual-core operation.

### How the cores communicate

The only data path between cores is a **FreeRTOS queue** (`QueueHandle_t raw_q`):
- Core 0 calls `xQueueSend()` — non-blocking, drops oldest if full
- Core 1 calls `xQueueReceive()` — blocks until a sample arrives
- FreeRTOS internally uses a mutex + critical section to make queue operations
  thread-safe across cores without any application-level locking

`raw_sample_t` is the unit of transfer: 76 bytes containing timestamp, FSR forces (N),
accelerometer (g), gyroscope (deg/s), quaternion, and Euler angles.

---

## Project Structure

```
uart_echo_VitalSignsLab4/
├── main/
│   ├── main.c               # app_main: task creation, queue, UART driver
│   ├── data_types.h         # shared structs (raw_sample_t, cal_params_t) + defines
│   ├── acquisition.c/.h     # Core 0: timer ISR, ADC, BNO085, queue post
│   ├── processing.c/.h      # Core 1: filters, state machine, JSON output, commands
│   ├── calibration.c/.h     # calibration sub-state machine (C1–C4)
│   ├── filters.c/.h         # Butterworth IIR biquad implementations
│   ├── motor_control.c/.h   # vibration motor GPIO pulse control
│   ├── nvs_storage.c/.h     # NVS persistence for calibration parameters
│   ├── CMakeLists.txt
│   └── idf_component.yml
├── components/
│   └── sh2/                 # CEVA BNO085 sh2 driver + ESP-IDF I2C HAL
├── gui/
│   ├── esp32_controller.py  # PySide6 desktop GUI
│   └── requirements.txt
├── tests/
│   └── fsr_test.py          # standalone FSR diagnostic display (no GUI needed)
└── CMakeLists.txt
```

---

## Hardware

| Component | GPIO | Notes |
|---|---|---|
| FSR 402 thumb | GPIO 34 | ADC1_CH6, input-only |
| FSR 402 index | GPIO 39 | ADC1_CH3, input-only |
| FSR 402 middle | GPIO 32 | ADC1_CH4 |
| BNO085 SDA | GPIO 22 | I2C0 |
| BNO085 SCL | GPIO 20 | I2C0 |
| BNO085 INT | GPIO 15 | falling-edge ISR |
| Motor 0 (force warn) | GPIO 25 | NPN driver, MOTORS_ENABLED=0 by default |
| Motor 1 (force err) | GPIO 26 | NPN driver |
| Motor 2 (instability) | GPIO 27 | NPN driver |
| Motor 3 (tremor) | GPIO 14 | NPN driver |
| Freq proof | GPIO 33 | toggled every 100 Hz tick for scope verification |
| Core 1 debug | GPIO 12 | toggled every processing cycle |

---

## Force Thresholds

Values from Horeman et al. 2010 (surgical simulation literature):

| Level | Per-finger | F_sum | Notes |
|---|---|---|---|
| Expert mean | — | 0.9 N | F_ref_open default |
| Warning | > 0.8 N | > 2.0 N sustained 100 ms | Above expert mean |
| Error | > 1.5 N | > 4.0 N sustained 100 ms | Near novice maximum (4.7 N) |

Mode commands `E` / `M` / `H` scale the F_sum thresholds by multiplying F_ref_open:

| Mode | Warn multiplier | Error multiplier |
|---|---|---|
| Easy | 2.5× | 4.0× |
| Medium (default) | 2.0× | 3.5× |
| Hard | 1.5× | 2.5× |

---

## UART Protocol

All communication is over UART0 (115200 baud, GPIO 1=TX, 3=RX).

### Commands (host → ESP32)

| Byte | Response | Effect |
|---|---|---|
| `I` | `ESP32_TRAINER` | Identify |
| `E` | `EASY` | Easy force thresholds |
| `M` | `INTERMEDIATE` | Medium force thresholds |
| `H` | `HARD` | Hard force thresholds |
| `S` | `STOPPED` | Reset state to IDLE |
| `C1`–`C4` | JSON status | Run calibration step |
| `X` | `EXITED` | Suspend all tasks |
| `Z` | JSON | Erase NVS calibration |

### Telemetry (ESP32 → host, 20 Hz)

```json
{"t":12340,"f0":0.196,"f1":0.000,"f2":0.000,
 "ax":0.01,"ay":-0.02,"az":1.00,
 "gx":0.1,"gy":-0.2,"gz":0.0,
 "roll":1.2,"pitch":-0.5,"yaw":45.0,
 "f_sum":0.196,"tremor":0.02,"f95":1.6,
 "pp_roll":0.5,"pp_pitch":0.3,"cv_f":0.04,"swing":0.0,
 "contact":[1,0,0],"state":"HOLD",
 "warn":0,"err":0,"score":100.0,"actual_hz":100.00}
```

JSON is output at 20 Hz (every 5th sample) to stay within 115200 baud capacity.
The `actual_hz` field reports the true acquisition rate measured on Core 0.

---

## Build and Flash

```bash
# Source ESP-IDF (path depends on local install)
source ~/.espressif/tools/activate_idf_v6.0.sh

# Build
python3 ~/.espressif/v6.0/esp-idf/tools/idf.py build

# Flash and monitor
python3 ~/.espressif/v6.0/esp-idf/tools/idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

Exit the monitor with `Ctrl-]`.

---

## FSR Diagnostic Tool

Test force sensors without the full GUI:

```bash
python tests/fsr_test.py              # auto-detect port
python tests/fsr_test.py --list-ports # show all serial ports
python tests/fsr_test.py --log        # also write CSV log
```

---

## Python GUI

```bash
pip install -r gui/requirements.txt
python gui/esp32_controller.py
```

---

## Troubleshooting

### Board crash-loops on boot
Check I2C wiring (SDA=GPIO22, SCL=GPIO20). If BNO085 is not connected, the firmware
continues in FSR-only mode — IMU fields in JSON will be zero. The crash loop was
caused by `sh2_open()` blocking indefinitely when the sensor is absent; the fix is an
`i2c_master_probe()` call that times out in 50 ms before attempting `sh2_open()`.

### fsr_test.py connects to wrong port
Run `python tests/fsr_test.py --list-ports` to see all detected ports.
`/dev/cu.wlan-debug` is a macOS system port, not the ESP32.
The script prefers `usbserial`/`usbmodem` ports and skips `wlan-debug`, `Bluetooth`, and `BTLE`.

### JSON output stops / 0 samples
Only one process can hold the serial port at a time.
Stop `idf.py monitor` (`Ctrl-]`) before running `fsr_test.py` or the GUI.

### `X` was pressed and commands no longer work
Expected — `X` suspends both FreeRTOS tasks. Reset the board to resume.
