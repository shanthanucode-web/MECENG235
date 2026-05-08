#!/usr/bin/env python3
"""
IMU Test Script — BNO085 via CEVA sh2
Reads live JSON telemetry from the ESP32 and displays
gyroscope, accelerometer, and Euler angle data in real time.

This validates the IMU portion of the firmware telemetry path. It does not
talk to the IMU directly; it checks what the ESP32 is already decoding and
publishing over JSON.

Usage:
    python tests/imu_test.py                      # auto-detect port
    python tests/imu_test.py /dev/cu.usbserial-X  # explicit port
    python tests/imu_test.py --log                # also save to imu_log.csv
    python tests/imu_test.py --raw                # print raw JSON lines too
"""

import argparse
import csv
import json
import math
import sys
import time
from collections import deque
from datetime import datetime

import serial
import serial.tools.list_ports

# ── ANSI helpers ───────────────────────────────────────────────────────────
RESET      = "\033[0m"
BOLD       = "\033[1m"
GREEN      = "\033[92m"
YELLOW     = "\033[93m"
RED        = "\033[91m"
CYAN       = "\033[96m"
BLUE       = "\033[94m"
GREY       = "\033[90m"
WHITE      = "\033[97m"
CLEAR_LINE = "\033[2K\r"

BAR_WIDTH  = 28
MAX_GYRO   = 200.0   # deg/s — full bar
MAX_ACCEL  = 2.0     # g — full bar


# ── Port detection (same logic as fsr_test.py) ─────────────────────────────
def detect_port() -> str | None:
    """Match the same USB-serial style devices used across repo tools."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if any(k in p.device for k in ("usbserial", "usbmodem", "ttyUSB", "ttyACM")):
            return p.device
    SKIP = ("wlan-debug", "Bluetooth", "BTLE", "debug")
    for p in ports:
        if "cu." in p.device and not any(s in p.device for s in SKIP):
            return p.device
    return None


# ── Bar helpers ────────────────────────────────────────────────────────────
def signed_bar(v: float, max_v: float, width: int = BAR_WIDTH) -> str:
    """Centred bar: negative left, positive right."""
    half  = width // 2
    ratio = max(-1.0, min(1.0, v / max_v))
    if ratio >= 0:
        filled = int(ratio * half)
        return (GREY  + "─" * half +
                GREEN + "█" * filled +
                GREY  + "─" * (half - filled) + RESET)
    else:
        filled = int(-ratio * half)
        return (GREY  + "─" * (half - filled) +
                YELLOW + "█" * filled +
                GREY  + "─" * half + RESET)


def angle_bar(deg: float, width: int = BAR_WIDTH) -> str:
    """Bar showing angle in -180..+180 range."""
    return signed_bar(deg, 180.0, width)


# ── IMU alive check ────────────────────────────────────────────────────────
def imu_alive(pkt: dict) -> bool:
    return any(abs(pkt.get(k, 0.0)) > 0.001
               for k in ("gx", "gy", "gz", "ax", "ay", "az"))


# ── Main display loop ──────────────────────────────────────────────────────
def run(port: str, baud: int, log: bool, raw: bool) -> None:
    # Rolling window for omega stats
    omega_win: deque = deque(maxlen=100)   # 1 second at 100 Hz

    sample_count = 0
    parse_errors = 0
    start_time   = time.monotonic()
    log_file     = None
    csv_writer   = None

    if log:
        ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = f"imu_log_{ts}.csv"
        log_file = open(log_path, "w", newline="")
        csv_writer = csv.writer(log_file)
        csv_writer.writerow([
            "timestamp_ms",
            "gx_dps", "gy_dps", "gz_dps",
            "ax_g",   "ay_g",   "az_g",
            "roll_deg", "pitch_deg", "yaw_deg",
            "omega_norm_dps",
        ])
        print(f"{CYAN}Logging to {log_path}{RESET}")

    all_ports = [p.device for p in serial.tools.list_ports.comports()]
    print(f"Available ports: {all_ports}")
    print(f"\n{BOLD}IMU Test — BNO085{RESET}  |  port={port}  baud={baud}")
    print(f"Move the board to verify live data.  Ctrl-C to quit.\n")

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"{RED}Cannot open {port}: {e}{RESET}")
        sys.exit(1)

    print(f"{GREEN}Connected to {port}{RESET}\n")
    time.sleep(0.3)
    ser.reset_input_buffer()

    DISPLAY_LINES = 18   # lines in one dashboard frame

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

            if not text.startswith("{"):
                continue

            try:
                pkt = json.loads(text)
            except json.JSONDecodeError:
                parse_errors += 1
                continue

            sample_count += 1

            gx  = float(pkt.get("gx",    0.0))
            gy  = float(pkt.get("gy",    0.0))
            gz  = float(pkt.get("gz",    0.0))
            ax  = float(pkt.get("ax",    0.0))
            ay  = float(pkt.get("ay",    0.0))
            az  = float(pkt.get("az",    0.0))
            rol = float(pkt.get("roll",  0.0))
            pit = float(pkt.get("pitch", 0.0))
            yaw = float(pkt.get("yaw",   0.0))
            t_ms       = int(pkt.get("t", 0))
            actual_hz  = float(pkt.get("actual_hz", 0.0))

            omega_norm = math.sqrt(gx*gx + gy*gy + gz*gz)
            omega_win.append(omega_norm)
            omega_rms = math.sqrt(sum(v*v for v in omega_win) / len(omega_win))

            if csv_writer:
                csv_writer.writerow([
                    t_ms,
                    f"{gx:.4f}", f"{gy:.4f}", f"{gz:.4f}",
                    f"{ax:.4f}", f"{ay:.4f}", f"{az:.4f}",
                    f"{rol:.3f}", f"{pit:.3f}", f"{yaw:.3f}",
                    f"{omega_norm:.4f}",
                ])

            # ── Redraw dashboard in-place ──────────────────────────────
            if sample_count > 1:
                sys.stdout.write(f"\033[{DISPLAY_LINES}A")

            elapsed = time.monotonic() - start_time

            alive       = imu_alive(pkt)
            alive_str   = f"{GREEN}● ACTIVE{RESET}" if alive else f"{RED}○ NO DATA{RESET}"
            omega_colour = RED if omega_rms > 30 else (YELLOW if omega_rms > 10 else GREEN)

            def line_out(s: str = "") -> None:
                sys.stdout.write(CLEAR_LINE + s + "\n")

            line_out(f"{BOLD}IMU Live{RESET}  "
                     f"t={t_ms/1000:.1f}s  acq={actual_hz:.0f}Hz  "
                     f"samples={sample_count}  errs={parse_errors}  "
                     f"elapsed={elapsed:.0f}s  {alive_str}")
            line_out()

            # ── Gyroscope ──────────────────────────────────────────────
            line_out(f"{BOLD}{CYAN}Gyroscope (deg/s){RESET}      "
                     f"[neg ◄ {'─'*10} 0 {'─'*10} ► pos]"
                     f"  range ±{MAX_GYRO:.0f}")
            line_out(f"  Gx {gx:+8.2f}  {signed_bar(gx, MAX_GYRO)}  {GREY}roll rate{RESET}")
            line_out(f"  Gy {gy:+8.2f}  {signed_bar(gy, MAX_GYRO)}  {GREY}pitch rate{RESET}")
            line_out(f"  Gz {gz:+8.2f}  {signed_bar(gz, MAX_GYRO)}  {GREY}yaw rate{RESET}")
            line_out(f"  ‖ω‖ {omega_norm:7.2f}  "
                     f"RMS(1s): {omega_colour}{omega_rms:.2f} deg/s{RESET}")
            line_out()

            # ── Accelerometer ──────────────────────────────────────────
            accel_norm = math.sqrt(ax*ax + ay*ay + az*az)
            line_out(f"{BOLD}{BLUE}Accelerometer (g){RESET}      "
                     f"[neg ◄ {'─'*10} 0 {'─'*10} ► pos]"
                     f"  range ±{MAX_ACCEL:.0f}")
            line_out(f"  Ax {ax:+8.3f}  {signed_bar(ax, MAX_ACCEL)}  {GREY}lateral{RESET}")
            line_out(f"  Ay {ay:+8.3f}  {signed_bar(ay, MAX_ACCEL)}  {GREY}forward{RESET}")
            line_out(f"  Az {az:+8.3f}  {signed_bar(az, MAX_ACCEL)}  {GREY}vertical{RESET}")
            line_out(f"  ‖a‖ {accel_norm:7.3f} g")
            line_out()

            # ── Euler angles ───────────────────────────────────────────
            line_out(f"{BOLD}{YELLOW}Euler Angles (deg){RESET}     "
                     f"[─180 ◄ {'─'*10} 0 {'─'*10} ► +180]")
            line_out(f"  Roll  {rol:+8.2f}°  {angle_bar(rol)}")
            line_out(f"  Pitch {pit:+8.2f}°  {angle_bar(pit)}")
            line_out(f"  Yaw   {yaw:+8.2f}°  {angle_bar(yaw)}")
            line_out()

            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if log_file:
            log_file.close()
            print(f"\n{CYAN}Log saved.{RESET}")

    elapsed = time.monotonic() - start_time
    rate    = sample_count / elapsed if elapsed > 0 else 0.0
    print(f"\n{BOLD}Session summary{RESET}")
    print(f"  Samples : {sample_count}")
    print(f"  Errors  : {parse_errors}")
    print(f"  Rate    : {rate:.1f} samples/s")


def main() -> None:
    parser = argparse.ArgumentParser(description="BNO085 IMU live test")
    parser.add_argument("port", nargs="?", default=None,
                        help="Serial port (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--log",  action="store_true",
                        help="Save to imu_log_<timestamp>.csv")
    parser.add_argument("--raw",  action="store_true",
                        help="Also print raw JSON lines")
    args = parser.parse_args()

    port = args.port or detect_port()
    if port is None:
        print(f"{RED}No serial port found. Pass one explicitly.{RESET}")
        sys.exit(1)

    run(port, args.baud, args.log, args.raw)


if __name__ == "__main__":
    main()
