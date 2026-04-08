| Supported Targets | ESP32 |
| ----------------- | ----- |

# Haptic Surgical Skill Trainer — Lab 4

ESP32 firmware and Python GUI for the VitalSigns Lab 4 project. The firmware controls an LED via UART commands across both ESP32 cores. The Python app provides a surgical-themed desktop interface to send commands and monitor state.

---

## Project Structure

```
uart_echo_VitalSignsLab4/
├── main/
│   ├── uart_echo_example_main.c   # Firmware: UART command handler + LED state machine
│   ├── Kconfig.projbuild          # Menuconfig options (UART port, baud, stack size)
│   └── CMakeLists.txt
├── gui/
│   ├── esp32_controller.py        # PySide6 desktop GUI
│   └── requirements.txt           # Python dependencies
└── CMakeLists.txt
```

---

## Firmware

### What It Does

Listens for single-character UART commands and drives an LED on `GPIO13` through a state machine.

### Commands

| Key | Response | Behavior |
|-----|----------|----------|
| `I` | `CONNECTED` | LED on steady |
| `E` | `EASY` | LED blinks at 500 ms |
| `H` | `HARD` | LED blinks at 100 ms |
| `S` | `READY` | Stop blinking, LED on |
| `X` | `EXITED` | LED off, tasks deleted — reset to resume |

Commands are uppercase. In the ESP-IDF monitor: `Shift+I`, `Shift+E`, etc.

### Dual-Core Task Assignment

Both FreeRTOS tasks are pinned to separate cores using `xTaskCreatePinnedToCore`:

| Task | Core | Role |
|------|------|------|
| `uart_echo_task` | Core 0 (PRO) | UART I/O and command processing |
| `blink_led_task` | Core 1 (APP) | LED state machine |

### Hardware

- LED: `GPIO13`
- UART: `UART0`, TX = `GPIO1`, RX = `GPIO3`, 115200 baud

### Build and Flash

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with your board's serial port (e.g. `/dev/cu.usbserial-0001` on macOS).

To exit the monitor: `Ctrl-]`

---

## Python GUI (`gui/`)

A PySide6 desktop app that connects to the ESP32 over USB-serial and provides a surgical-themed control interface.

### Features

- Auto-detects serial ports (prefers `cu.*` on macOS)
- Trainer state indicator with heartbeat animation in EASY/HARD mode
- Recent Activity feed: color-coded event cards for each state change
- Sensor Data placeholder panel (Force L, Force R, Tremor — for future live data)
- Five command buttons: I / IDENTIFY, E / EASY, H / HARD, S / STOP, X / EXIT
- Collapsible raw serial terminal (hidden by default, toggled with **SHOW TERMINAL**)

### Install and Run

```bash
pip install -r gui/requirements.txt
python gui/esp32_controller.py
```

### Requirements

- Python 3.10+
- `PySide6 >= 6.6.0`
- `pyserial >= 3.5`

---

## Setup Requirements

Each teammate needs:

- ESP-IDF installed and sourced in the terminal
- An ESP32 HUZZAH32 development board
- A USB cable connected to the board

If `idf.py` is not recognized, source ESP-IDF first.

---

## Troubleshooting

### `idf.py: command not found`
Your ESP-IDF environment is not loaded. Source it, then rerun.

### GUI shows "No ports found"
Click the refresh button (⟳). Make sure the board is plugged in and the correct driver is installed.

### Nothing happens when typing commands in the monitor
- Use uppercase letters
- Board is not already in `EXITED` state
- Correct serial port selected
- Firmware was flashed successfully

### `X` was pressed and commands no longer work
Expected — `X` terminates both FreeRTOS tasks. Reset the board or reflash to resume.

### LED behavior does not match your code changes
Rebuild and reflash — editing source files does not update the ESP32 automatically.
