#!/usr/bin/env python3
"""
FSR Force Sensor Test Script
Reads live JSON telemetry from the ESP32 and displays
force readings for all three FSR 402 sensors in real time.

Usage:
    python tests/fsr_test.py                      # auto-detect port
    python tests/fsr_test.py /dev/cu.usbserial-X  # explicit port
    python tests/fsr_test.py --log                # also save to fsr_log.csv
    python tests/fsr_test.py --baud 921600        # non-default baud rate
    python tests/fsr_test.py --raw               # print raw JSON lines too
"""

import argparse
import csv
import json
import os
import sys
import time
from collections import deque
from datetime import datetime

import serial
import serial.tools.list_ports

# ── Terminal colour helpers ────────────────────────────────────────────────
RESET  = "\033[0m"
BOLD   = "\033[1m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
CYAN   = "\033[96m"
GREY   = "\033[90m"
CLEAR_LINE = "\033[2K\r"

BAR_WIDTH = 30  # characters per force bar

# ── Force thresholds (Horeman et al. 2010, surgical literature) ───────────
# Expert mean force: 0.9 N | Novice mean: 2.1 N | Novice max: 4.7 N
CONTACT_ON_N   = 0.30  # N — contact detected above this (unchanged)
WARN_FINGER_N  = 0.8   # N — per-finger warning
ERR_FINGER_N   = 1.5   # N — per-finger error
WARN_N         = 2.0   # N — F_sum warning (above expert mean)
ERR_N          = 4.0   # N — F_sum error   (near novice maximum)
MAX_DISPLAY_N  = 6.0   # N — full bar width (operating range)


def finger_colour(f: float) -> str:
    """Colour for a single-finger numeric value."""
    if f >= ERR_FINGER_N:
        return RED
    if f >= WARN_FINGER_N:
        return YELLOW
    if f >= CONTACT_ON_N:
        return GREEN
    return GREY


def sum_colour(f: float) -> str:
    """Colour for the F_sum row."""
    if f >= ERR_N:
        return RED
    if f >= WARN_N:
        return YELLOW
    if f >= CONTACT_ON_N:
        return GREEN
    return GREY


def bar(f: float, width: int = BAR_WIDTH) -> str:
    """Per-finger bar with character encoding by force level."""
    filled = int(min(f / MAX_DISPLAY_N, 1.0) * width)
    empty  = width - filled
    if f >= ERR_FINGER_N:
        fill = RED    + "█" * filled
    elif f >= WARN_FINGER_N:
        fill = YELLOW + "▒" * filled
    elif f >= CONTACT_ON_N:
        fill = GREEN  + "░" * filled
    else:
        fill = GREY   + "░" * filled
    return fill + GREY + " " * empty + RESET


def detect_port() -> str | None:
    ports = list(serial.tools.list_ports.comports())

    # Priority 1: USB-serial adapters (CP210x / CH340 / FTDI) — the ESP32
    for p in ports:
        if any(k in p.device for k in ("usbserial", "usbmodem", "ttyUSB", "ttyACM")):
            return p.device

    # Priority 2: any cu.* that isn't a macOS system port
    SKIP = ("wlan-debug", "Bluetooth", "BTLE", "debug")
    for p in ports:
        if "cu." in p.device and not any(s in p.device for s in SKIP):
            return p.device

    return None


def moving_stats(window: deque) -> tuple[float, float, float]:
    """Return (mean, std, peak) for the values in window."""
    if not window:
        return 0.0, 0.0, 0.0
    lst = list(window)
    mean = sum(lst) / len(lst)
    variance = sum((x - mean) ** 2 for x in lst) / len(lst)
    std = variance ** 0.5
    peak = max(lst)
    return mean, std, peak


def run(port: str, baud: int, log: bool, raw: bool) -> None:
    finger_labels = ["Thumb (GPIO34)", "Index (GPIO39)", "Middle (GPIO32)"]
    json_keys     = ["f0", "f1", "f2"]

    # Rolling 1-second window at ~20 Hz JSON output
    windows: list[deque] = [deque(maxlen=20) for _ in range(3)]

    sample_count  = 0
    parse_errors  = 0
    start_time    = time.monotonic()
    log_file      = None
    csv_writer    = None

    if log:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = f"fsr_log_{ts}.csv"
        log_file = open(log_path, "w", newline="")
        csv_writer = csv.writer(log_file)
        csv_writer.writerow(["timestamp_ms", "f0_N", "f1_N", "f2_N",
                             "f_sum_N", "contact0", "contact1", "contact2", "state"])
        print(f"{CYAN}Logging to {log_path}{RESET}")

    all_ports = [p.device for p in serial.tools.list_ports.comports()]
    print(f"Available ports: {all_ports}")
    print(f"\n{BOLD}FSR Force Sensor Test{RESET}  |  port={port}  baud={baud}")
    print(f"Thresholds: contact>{CONTACT_ON_N}N  warn>{WARN_N}N  err>{ERR_N}N\n")

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"{RED}Cannot open {port}: {e}{RESET}")
        sys.exit(1)

    print(f"{GREEN}Connected to {port}{RESET}  (Ctrl-C to quit)\n")

    # Flush any startup banner lines
    time.sleep(0.3)
    ser.reset_input_buffer()

    try:
        while True:
            line = ser.readline()
            if not line:
                continue

            try:
                text = line.decode("ascii", errors="replace").strip()
            except Exception:
                continue

            if raw and text:
                print(f"{GREY}{text}{RESET}")

            # Only parse JSON lines
            if not text.startswith("{"):
                continue

            try:
                data = json.loads(text)
            except json.JSONDecodeError:
                parse_errors += 1
                continue

            sample_count += 1

            f = [float(data.get(k, 0.0)) for k in json_keys]
            f_sum     = float(data.get("f_sum", sum(f)))
            contact   = data.get("contact", [0, 0, 0])
            state     = data.get("state", "?")
            t_ms      = int(data.get("t", 0))
            actual_hz = float(data.get("actual_hz", 0.0))

            for i, v in enumerate(f):
                windows[i].append(v)

            # ── CSV logging ──────────────────────────────────────────────
            if csv_writer:
                csv_writer.writerow([
                    t_ms,
                    f"{f[0]:.4f}", f"{f[1]:.4f}", f"{f[2]:.4f}",
                    f"{f_sum:.4f}",
                    int(contact[0]), int(contact[1]), int(contact[2]),
                    state,
                ])

            # ── Terminal display (refresh in place) ──────────────────────
            elapsed = time.monotonic() - start_time

            # Move cursor up to overwrite previous block (after first draw)
            if sample_count > 1:
                lines_up = 10
                sys.stdout.write(f"\033[{lines_up}A")

            # Header row
            sys.stdout.write(CLEAR_LINE)
            sys.stdout.write(
                f"{BOLD}FSR Live  {RESET}"
                f"t={t_ms/1000:.1f}s  "
                f"acq={actual_hz:.0f}Hz  "
                f"samples={sample_count}  "
                f"errs={parse_errors}  "
                f"elapsed={elapsed:.0f}s\n"
            )

            # Per-finger rows
            for i in range(3):
                mean, std, peak = moving_stats(windows[i])
                contact_icon = f"{GREEN}●{RESET}" if contact[i] else f"{GREY}○{RESET}"

                sys.stdout.write(CLEAR_LINE)
                sys.stdout.write(
                    f"  {finger_labels[i]:<18} {contact_icon} "
                    f"{finger_colour(f[i])}{f[i]:5.3f} N{RESET}  "
                    f"{bar(f[i])}  "
                    f"peak={peak:.3f}  σ={std:.3f}\n"
                )

            # Sum row
            sys.stdout.write(CLEAR_LINE)
            sys.stdout.write(
                f"  {'Total (F_sum)':<18}   "
                f"{sum_colour(f_sum)}{f_sum:5.3f} N{RESET}  "
                f"{bar(f_sum)}\n"
            )

            # State row
            state_colour = {
                "HOLD":   CYAN,
                "ACTIVE": YELLOW,
                "IDLE":   GREY,
                "EXITED": RED,
            }.get(state, GREY)

            sys.stdout.write(CLEAR_LINE)
            sys.stdout.write(
                f"  State: {state_colour}{BOLD}{state:<8}{RESET}  "
                f"contact: [{int(contact[0])},{int(contact[1])},{int(contact[2])}]\n"
            )

            # Legend row
            sys.stdout.write(CLEAR_LINE)
            sys.stdout.write(
                f"  {GREY}Scale: 0 N"
                f"{'─' * (BAR_WIDTH // 2 - 5)}"
                f"  {MAX_DISPLAY_N / 2:.0f} N"
                f"{'─' * (BAR_WIDTH // 2 - 5)}"
                f"  {MAX_DISPLAY_N:.0f} N{RESET}\n"
            )

            # Threshold legend
            sys.stdout.write(CLEAR_LINE)
            sys.stdout.write(
                f"  {GREY}finger: <{WARN_FINGER_N}N {GREEN}░{RESET}  "
                f"{WARN_FINGER_N}-{ERR_FINGER_N}N {YELLOW}▒{RESET}  "
                f">{ERR_FINGER_N}N {RED}█{RESET}  "
                f"  sum warn>{WARN_N}N  err>{ERR_N}N\n"
            )

            # Blank separator
            sys.stdout.write(CLEAR_LINE + "\n")
            sys.stdout.write(CLEAR_LINE + "\n")

            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if log_file:
            log_file.close()
            print(f"\n{CYAN}Log saved.{RESET}")

    elapsed = time.monotonic() - start_time
    rate = sample_count / elapsed if elapsed > 0 else 0.0
    print(f"\n{BOLD}Session summary{RESET}")
    print(f"  Samples parsed : {sample_count}")
    print(f"  Parse errors   : {parse_errors}")
    print(f"  Elapsed        : {elapsed:.1f} s")
    print(f"  Effective rate : {rate:.1f} samples/s")


def main() -> None:
    parser = argparse.ArgumentParser(description="FSR force sensor live test")
    parser.add_argument("port", nargs="?", default=None,
                        help="Serial port (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--log", action="store_true",
                        help="Save readings to fsr_log_<timestamp>.csv")
    parser.add_argument("--list-ports", action="store_true",
                        help="List all detected serial ports and exit")
    parser.add_argument("--raw", action="store_true",
                        help="Also print raw serial lines (for debugging)")
    args = parser.parse_args()

    if args.list_ports:
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("No serial ports found.")
        for p in ports:
            print(f"  {p.device:35s} {p.description}")
        sys.exit(0)

    port = args.port or detect_port()
    if port is None:
        print(f"{RED}No serial port found. Plug in the ESP32 or pass a port explicitly.{RESET}")
        sys.exit(1)

    run(port, args.baud, args.log, args.raw)


if __name__ == "__main__":
    main()
