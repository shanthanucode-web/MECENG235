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
│   │    → reads BNO085 UART-RVC if data  │                           │
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

### What to look for in the monitor

Open the serial monitor and reset the ESP32. The important lines appear only during
boot and early task startup, so watch the first few seconds after reset.

This is the expected sequence:

```text
I (...) boot: Multicore bootloader
I (...) cpu_start: Multicore app
I (...) main_task: Started on CPU0
I (...) ACQ: ACQ TASK: running on Core 0 at 100 Hz
I (...) PROC: PROC TASK: running on Core 1
```

How to interpret these lines:

- `boot: Multicore bootloader`
  - The ESP32 is using the multicore boot path.
- `cpu_start: Multicore app`
  - The application image is running in multicore mode.
- `main_task: Started on CPU0`
  - ESP-IDF started the initial application task on CPU0.
- `ACQ TASK: running on Core 0`
  - The acquisition task is pinned where the firmware expects it.
- `PROC TASK: running on Core 1`
  - The processing task is pinned to the second core.

The strongest runtime evidence for this project is seeing both task-placement lines
after a `Multicore app` boot.

If you do not see the expected lines:

- Make sure you are looking at the serial monitor output, not the build or flash log.
- Reset the board after opening the monitor.
- Watch from the very first boot line; these messages do not repeat during normal runtime.
- If `ACQ TASK` appears but `PROC TASK` does not, the app booted in multicore mode
  but the processing task either did not start cleanly or its log was missed.

### How the cores communicate

The only data path between cores is a **FreeRTOS queue** (`QueueHandle_t raw_q`):
- Core 0 calls `xQueueSend()` — non-blocking, drops oldest if full
- Core 1 calls `xQueueReceive()` — blocks until a sample arrives
- FreeRTOS internally uses a mutex + critical section to make queue operations
  thread-safe across cores without any application-level locking

`raw_sample_t` is the unit of transfer: 72 bytes containing timestamp, FSR forces (N),
accelerometer (g), derived gyroscope (deg/s), quaternion placeholder, and Euler angles.
In UART-RVC mode the BNO085 does not provide quaternion or gyro reports; quaternion is
zeroed and gyro is approximated from Euler angle differences at 100 Hz.

---

## Project Structure

```
uart_echo_VitalSignsLab4/
├── main/
│   ├── main.c               # app_main: task creation, queue, UART driver
│   ├── data_types.h         # shared structs (raw_sample_t, cal_params_t) + defines
│   ├── acquisition.c/.h     # Core 0: timer ISR, ADC, BNO085 UART-RVC, queue post
│   ├── processing.c/.h      # Core 1: filters, state machine, JSON output, commands
│   ├── calibration.c/.h     # calibration sub-state machine (C1–C4)
│   ├── filters.c/.h         # Butterworth IIR biquad implementations
│   ├── motor_control.c/.h   # vibration motor GPIO pulse control
│   ├── nvs_storage.c/.h     # NVS persistence for calibration parameters
│   ├── CMakeLists.txt
│   └── idf_component.yml
├── components/
│   └── sh2/                 # vendored SH-2 driver, currently disabled
├── gui/
│   ├── esp32_controller.py  # PySide6 desktop GUI
│   └── requirements.txt
├── tests/
│   ├── fsr_test.py          # standalone FSR diagnostic display
│   ├── imu_test.py          # standalone IMU diagnostic display
│   └── dual_core_monitor.py # acquisition/processing timing monitor
└── CMakeLists.txt
```

---

## Hardware

| Component | GPIO | Notes |
|---|---|---|
| FSR 402 thumb | GPIO 34 | ADC1_CH6, input-only |
| FSR 402 index | GPIO 39 | ADC1_CH3, input-only |
| FSR 402 middle | GPIO 36 | ADC1_CH0, input-only |
| BNO085 SDA | GPIO 32 | UART2 RX, BNO085 data out to ESP32 |
| BNO085 SCL | GPIO 33 | BNO085 UART input, unused in RVC; firmware leaves it input/pull-up |
| BNO085 P0/PS0 | 3.3 V | High selects UART-RVC when BNO085 resets |
| BNO085 P1/PS1 | Unconnected | Low by default for UART-RVC |
| Motor 0 (force warn) | GPIO 25 | NPN driver, MOTORS_ENABLED=0 by default |
| Motor 1 (force err) | GPIO 26 | NPN driver |
| Motor 2 (instability) | GPIO 27 | NPN driver |
| Motor 3 (tremor) | GPIO 14 | NPN driver |
| Built-in status LED | GPIO 13 | command/mode indicator |
| Freq proof | GPIO 4 / A5 | toggled every 100 Hz tick for scope verification |
| Core 1 debug | GPIO 12 | toggled every processing cycle |

### BNO085 UART-RVC wiring notes

UART-RVC is output-only from the IMU for this project. The required connections are:

- ESP32 3V → BNO085 VIN
- ESP32 GND → BNO085 GND
- ESP32 GPIO32 → BNO085 SDA
- ESP32 3V → BNO085 P0/PS0
- BNO085 P1/PS1 left unconnected

The BNO085 samples P0/P1 at reset. After changing P0/P1 wiring, power-cycle the IMU;
resetting only the ESP32 may leave the BNO085 in its previous mode.

Here, "power-cycle the IMU" means removing electrical power from the IMU itself
for a moment so it fully turns off, then restoring power. Pressing `EN`, rebooting,
or resetting only the ESP32 does not guarantee that the BNO085 re-reads `P0/P1`.

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
The `actual_hz` field is currently a nominal firmware value; use
`tests/dual_core_monitor.py` or a scope on GPIO4/A5 for timing verification.

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

## Diagnostic Tools

Test force sensors without the full GUI:

```bash
python tests/fsr_test.py              # auto-detect port
python tests/fsr_test.py --list-ports # show all serial ports
python tests/fsr_test.py --log        # also write CSV log
```

Test the BNO085 UART-RVC stream:

```bash
python tests/imu_test.py              # auto-detect port
python tests/imu_test.py --raw        # show raw JSON lines too
```

Monitor software timing:

```bash
python tests/dual_core_monitor.py
python tests/dual_core_monitor.py --list-ports
python tests/dual_core_monitor.py --port /dev/cu.usbserial-XXXX
```

`dual_core_monitor.py` provides software evidence that acquisition timing remains
stable while processing and UART output are active. It reconstructs the 100 Hz
acquisition period from firmware timestamps and compares that against the host-side
JSON receive cadence. This is consistent with the dual-core design, but it is not
formal proof of multicore execution by itself. The stronger proof is:

- the ESP32 multicore boot log,
- `xTaskCreatePinnedToCore(..., 0/1)` in the firmware, and
- hardware observation of the debug pins:
  - GPIO4 / A5 = Core 0 acquisition heartbeat
  - GPIO12 = Core 1 processing heartbeat

---

## Python GUI

```bash
pip install -r gui/requirements.txt
python gui/esp32_controller.py
```

---

## Troubleshooting

### I only see the flash output, not the boot log
The ESP-IDF flash task output is not the same thing as the ESP32 runtime monitor output.
`Flash Done` only means the image was written successfully.

To verify multicore boot and task placement:

1. Open `idf.py monitor` or `ESP-IDF: Monitor Device`.
2. Press the ESP32 `EN` / reset button.
3. Watch the first lines printed after reset.

You are looking for:

- `boot: Multicore bootloader`
- `cpu_start: Multicore app`
- `ACQ TASK: running on Core 0`
- `PROC TASK: running on Core 1`

If you open the monitor after the board already booted, reset it again or you will
miss the boot log.

### IMU fields stay zero
The firmware is in BNO085 UART-RVC mode, not I2C/SH-2 mode. Check:

- BNO085 SDA is wired to ESP32 GPIO32.
- BNO085 P0/PS0 is tied to 3.3 V.
- BNO085 P1/PS1 is unconnected/low.
- BNO085 and ESP32 share ground.
- The BNO085 was power-cycled after P0/P1 wiring changed.

What "power-cycled" means in practice:

- Unplug USB from the ESP32, or disconnect the IMU `VIN` / `3.3 V` lead.
- Wait 1-2 seconds so the IMU fully loses power.
- Restore power, then test again.

Why this matters:

- The ESP32 can reboot while the BNO085 stays powered.
- The BNO085 chooses its interface mode only when it powers up or resets.
- If the IMU never lost power, it may stay in its previous interface mode even if
  the wiring is now correct.

RVC does not require ESP32 TX. If debugging, disconnect BNO085 SCL from GPIO33 and
leave only SDA→GPIO32 plus power, ground, and P0 high.

### IMU has power, but still no motion data
The green LED on the IMU only shows that the board is powered. It does not prove
that the BNO085 is in UART-RVC mode or transmitting valid packets.

If the IMU LED is on but the JSON `ax/ay/az/roll/pitch/yaw/gx/gy/gz` fields stay zero:

- Verify `P0/PS0` is high before power-up.
- Prefer tying `P1/PS1` explicitly to GND during bring-up instead of relying on it
  floating low.
- Fully remove power from the IMU and power it back on.
- Run `python tests/imu_test.py --raw` with `idf.py monitor` closed.

### fsr_test.py connects to wrong port
Run `python tests/fsr_test.py --list-ports` to see all detected ports.
`/dev/cu.wlan-debug` is a macOS system port, not the ESP32.
The script prefers `usbserial`/`usbmodem` ports and skips `wlan-debug`, `Bluetooth`, and `BTLE`.

### JSON output stops / 0 samples
Only one process can hold the serial port at a time.
Stop `idf.py monitor` (`Ctrl-]`) before running `fsr_test.py` or the GUI.

### `X` was pressed and commands no longer work
Expected — `X` suspends both FreeRTOS tasks. Reset the board to resume.
