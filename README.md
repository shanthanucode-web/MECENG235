| Supported Targets | ESP32 |
| ----------------- | ----- |

# UART LED Command Lab

This project is an ESP32 lab app built from the UART echo example and adapted to control an LED from UART commands typed in the ESP-IDF monitor.

## What This Project Does

The firmware listens for single-character UART commands and changes the LED behavior on `GPIO13`.

Current command set:

- `I`: connection check, LED stays on, monitor prints `CONNECTED`
- `E`: easy mode, LED blinks slowly, monitor prints `EASY`
- `H`: hard mode, LED blinks faster, monitor prints `HARD`
- `S`: stop the active blink mode and return to idle, monitor prints `IDLE`
- `X`: exit the program until reset/reflash, monitor prints `EXITED`

Important note:

- Commands are uppercase characters. In the monitor, that means `Shift+I`, `Shift+E`, `Shift+H`, `Shift+S`, and `Shift+X`.

## Current Firmware Behavior

- The LED is connected to `GPIO13`.
- The UART command interface is currently configured to use `UART0`.
- The current UART pins from `sdkconfig` are:
  - `TX = GPIO1`
  - `RX = GPIO3`
  - `Baud rate = 115200`
- Because this project is currently using `UART0`, the same ESP-IDF monitor window used for flashing can also be used to send commands.
- The current blink timing in code is:
  - Easy mode: `500 ms` between toggles
  - Hard mode: `100 ms` between toggles

## Project Structure

- `main/uart_echo_example_main.c`: main application logic, UART command handling, LED state machine
- `main/Kconfig.projbuild`: project menuconfig options for UART selection, baud rate, and task stack
- `sdkconfig`: current resolved build configuration for this local project
- `CMakeLists.txt` and `main/CMakeLists.txt`: ESP-IDF build setup

## Setup Requirements

Each teammate needs:

- ESP-IDF installed and sourced in the terminal
- An ESP32 development board
- A USB cable connected to the board

If `idf.py` is not recognized, the ESP-IDF environment has not been loaded in that terminal yet.

## How To Run

### 1. Open the project

Open this repository in your editor or terminal.

### 2. Load the ESP-IDF environment

Use your local ESP-IDF setup command before building. This step depends on where ESP-IDF is installed on your machine.

### 3. Build and flash

Run:

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with your board port, for example `/dev/cu.usbserial-0001` or similar on macOS.

This command:

- builds the firmware
- flashes it to the ESP32
- opens the serial monitor

To exit the monitor, press `Ctrl-]`.

## How To Use In The Monitor

Once the monitor is open:

1. Press `Shift+I` to verify communication and turn the LED on.
2. Press `Shift+E` to enter easy mode and blink slowly.
3. Press `Shift+H` to enter hard mode and blink quickly.
4. Press `Shift+S` to stop blinking and return to idle.
5. Press `Shift+X` to exit the firmware logic until the board is reset or reflashed.

Expected monitor responses:

- `I` -> `CONNECTED`
- `E` -> `EASY`
- `H` -> `HARD`
- `S` -> `IDLE`
- `X` -> `EXITED`

## Important Workflow Note

Changing the source code in the editor does not change the ESP32 by itself.

After every code change, you must rebuild and reflash:

```bash
idf.py -p PORT flash monitor
```

If you forget to reflash, the board will keep running the old firmware.

## What We Changed From The Original Example

This is no longer a plain UART echo example.

We changed it to:

- use a command-driven LED control flow
- add explicit firmware states for idle, connected, easy, hard, and exited behavior
- replace the old incremental blink-speed logic with fixed easy/hard modes
- use monitor-visible UART status messages for each command
- use the current `UART0` console configuration so commands can be typed directly in the ESP-IDF monitor

## Troubleshooting

### `idf.py: command not found`

Your ESP-IDF environment is not loaded in the current terminal. Source ESP-IDF first, then rerun the flash command.

### LED behavior does not match your latest code

You likely edited the file but did not rebuild and reflash. Flash again.

### Nothing happens when typing commands

Check:

- you are typing uppercase letters
- the board is still running and not already in `EXITED`
- the correct serial port is being used
- the firmware was flashed successfully

### `X` was pressed and now no commands work

That is expected. `X` is terminal in the current lab design. Reset the board or flash again to continue testing.
