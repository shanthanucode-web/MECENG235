# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32 lab project (Haptic Surgical Skill Trainer — Lab 4) with two parts:
1. **ESP-IDF firmware** (`main/`) — C code running on the ESP32 HUZZAH32
2. **Python desktop GUI** (`gui/`) — PySide6 app running on the host computer

---

## Firmware (ESP-IDF)

### Build and Flash

```bash
# Source ESP-IDF first (path depends on local install)
source ~/.espressif/v6.0/esp-idf/export.sh

# Build, flash, and open monitor
idf.py -p /dev/cu.usbserial-59691016461 flash monitor

# Build only
idf.py build

# Open menuconfig (UART port, baud rate, task stack size)
idf.py menuconfig
```

Exit the monitor with `Ctrl-]`.

### Key Configuration (sdkconfig / Kconfig)

Active values from `sdkconfig`:
- `CONFIG_EXAMPLE_UART_PORT_NUM=0` — UART0 (same as console)
- `CONFIG_EXAMPLE_UART_TXD=1`, `CONFIG_EXAMPLE_UART_RXD=3`
- `CONFIG_EXAMPLE_UART_BAUD_RATE=115200`
- `CONFIG_EXAMPLE_TASK_STACK_SIZE=3072`

Change these via `idf.py menuconfig`, not by editing `sdkconfig` directly. `sdkconfig` is gitignored.

---

## Python GUI

### Run

```bash
pip install -r gui/requirements.txt
python gui/esp32_controller.py
```

Requires Python 3.10+ for `X | Y` union type hints.

---

## Firmware Architecture

**Single source file:** `main/uart_echo_example_main.c`

Two FreeRTOS tasks pinned to separate cores:

| Task | Core | Stack | Priority | Role |
|------|------|-------|----------|------|
| `echo_task` | 0 (PRO) | 3072 B | 10 | UART read loop, command dispatch |
| `blink_task` | 1 (APP) | 2048 B | 5 | LED state machine, timing |

State is shared via `volatile board_state_t s_board_state`. No mutex — single writer (`echo_task`), single reader (`blink_task`), and the enum transitions are atomic on this architecture.

**State machine:** `IDLE → CONNECTED → EASY/HARD → IDLE` (via S), or any state `→ EXITED` (via X). `EXITED` is terminal until reset.

**UART protocol:** single uppercase byte in, `\r\n`-terminated ASCII line out. Firmware sends `READY: I E H S X` on startup.

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

- `main` — stable firmware baseline
- `pythonGUI` — current active branch with GUI + dual-core firmware
