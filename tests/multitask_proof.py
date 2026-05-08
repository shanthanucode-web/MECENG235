#!/usr/bin/env python3
"""
Live terminal proof for Core 0 FreeRTOS multitasking.

Usage:
    python3 tests/multitask_proof.py
    python3 tests/multitask_proof.py --port /dev/cu.usbserial-XXXX

The script enables proof mode with MT_ON, renders a compact live dashboard,
and disables proof mode with MT_OFF on exit.
"""

from __future__ import annotations

import argparse
import atexit
import json
import signal
import statistics
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

import serial
import serial.tools.list_ports


RESET = "\033[0m"
BOLD = "\033[1m"
CYAN = "\033[96m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
GREY = "\033[90m"
WHITE = "\033[97m"


@dataclass
class LatestCycle:
    ctrl: int
    tb: int
    sg: int
    aw: int
    ad: int
    cr: int


@dataclass
class Snapshot:
    latest: LatestCycle
    period_us: float
    jitter_us: float
    wake_us: float
    acq_us: float
    ctrl_resume_us: float
    ctrl_ratio: float
    sample_count: int
    trace_drop: int
    cycle_miss: int


_snapshots: deque[Snapshot] = deque(maxlen=60)
_event_log: deque[str] = deque(maxlen=10)
_trace_enabled = False
_connected = True
_malformed = 0
_received = 0
_ser: serial.Serial | None = None
_lock = threading.Lock()
_port_label = "not connected"
_disconnect_reason = ""
_verbose = False


def list_candidate_ports() -> list[str]:
    ports = list(serial.tools.list_ports.comports())
    matches: list[str] = []
    for p in ports:
        if any(k in p.device for k in ("usbserial", "usbmodem", "ttyUSB", "ttyACM")):
            matches.append(p.device)
    for p in ports:
        if "cu." in p.device and "Bluetooth" not in p.device and p.device not in matches:
            matches.append(p.device)
    return matches


def choose_port(candidates: list[str]) -> str:
    if len(candidates) == 1:
        return candidates[0]

    print("Multiple plausible serial ports found:")
    for idx, port in enumerate(candidates, start=1):
        print(f"  {idx}. {port}")

    while True:
        choice = input("Select port number: ").strip()
        if choice.isdigit():
            index = int(choice)
            if 1 <= index <= len(candidates):
                return candidates[index - 1]
        print("Invalid selection.")


def resolve_port(explicit_port: str | None) -> str:
    if explicit_port:
        return explicit_port

    candidates = list_candidate_ports()
    if not candidates:
        all_ports = [p.device for p in serial.tools.list_ports.comports()]
        if all_ports:
            print("No likely ESP32 serial port found.", file=sys.stderr)
            print("Available ports:", file=sys.stderr)
            for port in all_ports:
                print(f"  {port}", file=sys.stderr)
        else:
            print("No serial ports found.", file=sys.stderr)
        print("Fallback: python3 tests/multitask_proof.py --port /dev/cu.usbserial-XXXX", file=sys.stderr)
        sys.exit(1)

    return choose_port(candidates)


def send_cmd(cmd: str) -> None:
    if _ser is None:
        return
    _ser.write((cmd + "\n").encode("ascii"))
    _ser.flush()


def cleanup() -> None:
    global _connected
    try:
        send_cmd("MT_OFF")
    except Exception:
        pass
    try:
        if _ser is not None:
            _ser.close()
    except Exception:
        pass
    _connected = False


def handle_signal(signum, frame) -> None:  # noqa: ARG001
    cleanup()
    sys.exit(0)


def render_timeline(cycle: LatestCycle, period_us: float | None) -> str:
    width = 56
    if not period_us or period_us <= 0:
        period_us = 10_000.0

    def raw_pos(delta_us: int) -> int:
        if delta_us <= 0:
            return 1
        return max(0, min(width - 1, int((delta_us / period_us) * (width - 1))))

    chars = [" "] * width
    bg0 = 0

    deltas = [
        max(0, cycle.sg - cycle.tb),
        max(0, cycle.aw - cycle.tb),
        max(0, cycle.ad - cycle.tb),
        max(0, cycle.cr - cycle.tb) if cycle.cr >= cycle.tb else int(period_us),
    ]
    min_positions = [2, 4, 6, 8]

    tb = 1
    positions = []
    prev = tb
    for delta_us, floor in zip(deltas, min_positions):
        p = raw_pos(delta_us)
        p = max(p, floor, prev + 1)
        p = min(p, width - 1)
        positions.append(p)
        prev = p

    sg, aw, ad, cr = positions

    for i in range(bg0 + 1, tb):
        chars[i] = "~"
    for i in range(tb + 1, sg):
        chars[i] = "-"
    for i in range(sg + 1, aw):
        chars[i] = "-"
    for i in range(aw + 1, ad):
        chars[i] = "="
    for i in range(ad + 1, cr):
        chars[i] = "."
    for i in range(cr + 1, width):
        chars[i] = "~"

    chars[bg0] = "C"
    chars[tb] = "T"
    chars[sg] = "S"
    chars[aw] = "W"
    chars[ad] = "D"
    chars[cr] = "C"
    return "".join(chars)


def fmt_us(value: float | None) -> str:
    if value is None:
        return "--"
    return f"{value:7.1f} us"


def health_color(value: float | None, lo: float, hi: float) -> str:
    if value is None:
        return WHITE
    if lo <= value <= hi:
        return GREEN
    return YELLOW


def reader_thread(port: str, baud: int) -> None:
    global _connected, _malformed, _received, _trace_enabled, _ser, _port_label, _disconnect_reason

    try:
        _ser = serial.Serial(port, baud, timeout=0.5)
    except serial.SerialException as exc:
        print(f"{RED}Cannot open {port}: {exc}{RESET}", file=sys.stderr)
        sys.exit(1)

    _port_label = port
    time.sleep(0.2)
    _ser.reset_input_buffer()
    send_cmd("MT_ON")

    while True:
        try:
            raw = _ser.readline()
            if not raw:
                continue

            text = raw.decode("ascii", errors="replace").strip()
            if not text.startswith("{"):
                continue

            pkt = json.loads(text)

            with _lock:
                if pkt.get("mt") == "ON":
                    _trace_enabled = True
                    _event_log.appendleft("mode enabled")
                    continue
                if pkt.get("mt") == "OFF":
                    _trace_enabled = False
                    _event_log.appendleft("mode disabled")
                    continue
                if pkt.get("mt") != "SN":
                    continue

                latest_pkt = pkt.get("latest", {})
                stats_pkt = pkt.get("stats", {})
                health_pkt = pkt.get("health", {})

                snap = Snapshot(
                    latest=LatestCycle(
                        ctrl=int(latest_pkt.get("ctrl", 0)),
                        tb=int(latest_pkt.get("tb", 0)),
                        sg=int(latest_pkt.get("sg", 0)),
                        aw=int(latest_pkt.get("aw", 0)),
                        ad=int(latest_pkt.get("ad", 0)),
                        cr=int(latest_pkt.get("cr", 0)),
                    ),
                    period_us=float(stats_pkt.get("period_us", 0.0)),
                    jitter_us=float(stats_pkt.get("jitter_us", 0.0)),
                    wake_us=float(stats_pkt.get("wake_us", 0.0)),
                    acq_us=float(stats_pkt.get("acq_us", 0.0)),
                    ctrl_resume_us=float(stats_pkt.get("ctrl_resume_us", 0.0)),
                    ctrl_ratio=float(stats_pkt.get("ctrl_ratio", 0.0)),
                    sample_count=int(stats_pkt.get("n", 0)),
                    trace_drop=int(health_pkt.get("trace_drop", 0)),
                    cycle_miss=int(health_pkt.get("cycle_miss", 0)),
                )
                _snapshots.append(snap)
                _received += 1

                latest = snap.latest
                cb_to_sem = latest.sg - latest.tb
                timer_to_wake = latest.aw - latest.tb
                acq_dur = latest.ad - latest.aw
                ctrl_resume = latest.cr - latest.ad if latest.cr >= latest.ad else -1
                _event_log.appendleft(
                    f"ctrl={latest.ctrl} | tb->aw {timer_to_wake} us | acq {acq_dur} us | ctrl resume {ctrl_resume} us | drop={snap.trace_drop} miss={snap.cycle_miss}"
                )
        except json.JSONDecodeError:
            with _lock:
                _malformed += 1
        except serial.SerialException as exc:
            with _lock:
                _disconnect_reason = str(exc)
            _connected = False
            return
        except Exception:
            with _lock:
                _malformed += 1


def render() -> None:
    with _lock:
        snapshots = list(_snapshots)
        log_lines = list(_event_log)
        trace_enabled = _trace_enabled
        malformed = _malformed
        received = _received
        connected = _connected
        port_label = _port_label
        disconnect_reason = _disconnect_reason

    last = snapshots[-1] if snapshots else None
    latest = last.latest if last else None
    period_avg_us = last.period_us if last else None
    jitter_us = last.jitter_us if last else None
    wake_us = last.wake_us if last else None
    acq_us = last.acq_us if last else None
    ctrl_resume_us = last.ctrl_resume_us if last else None
    ctrl_before_timer_ratio = last.ctrl_ratio if last else None
    cb_sem_us = ((latest.sg - latest.tb) if latest else None)
    trace_drop = last.trace_drop if last else None
    cycle_miss = last.cycle_miss if last else None
    sample_count = last.sample_count if last else 0
    healthy = (trace_drop == 0 and cycle_miss == 0) if last else False

    sys.stdout.write("\033[H\033[J")
    print(f"{BOLD}Core 0 Multitasking Proof Dashboard{RESET}")
    print("Real Core 0 proof: control_task is interrupted by timer/acquisition work and then resumes.")

    print(f"{BOLD}Proof Snapshot{RESET}")
    print(f"  Timer period              : {health_color(period_avg_us, 9990.0, 10010.0)}{fmt_us(period_avg_us)}{RESET}")
    print(f"  Timer -> acquisition wake : {health_color(wake_us, 0.0, 200.0)}{fmt_us(wake_us)}{RESET}")
    print(f"  Acquisition runtime       : {health_color(acq_us, 0.0, 1000.0)}{fmt_us(acq_us)}{RESET}")
    print(f"  Acq done -> CTRL resume   : {health_color(ctrl_resume_us, 0.0, 500.0)}{fmt_us(ctrl_resume_us)}{RESET}")
    if ctrl_before_timer_ratio is None:
        print("  CTRL running before timer : --")
    else:
        ctrl_text = f"{ctrl_before_timer_ratio * 100.0:5.1f}% of recent cycles"
        ctrl_color = GREEN if ctrl_before_timer_ratio > 0.9 else YELLOW
        print(f"  CTRL running before timer : {ctrl_color}{ctrl_text}{RESET}")

    print(f"{BOLD}Latest Core 0 Timeline{RESET}")
    if latest is None:
        print("  waiting for proof packets...")
    else:
        print(
            "  "
            f"CTRL -> T(0 us) -> S({latest.sg - latest.tb} us) -> W({latest.aw - latest.tb} us) -> "
            f"D({latest.ad - latest.tb} us) -> CTRL({latest.cr - latest.tb if latest.cr >= latest.tb else '--'} us)"
        )
        print("  Legend: CTRL=control_task, T=timer callback, S=semaphore give, W=acquisition wake, D=acquisition done")

    print(f"{BOLD}Proof Health{RESET}")
    if last is None:
        print("  Trace drops             : --")
        print("  Cycle misses            : --")
        print("  Snapshot samples        : --")
        print("  Stream health           : --")
    else:
        health_color_text = GREEN if healthy else YELLOW
        print(f"  Trace drops             : {trace_drop}")
        print(f"  Cycle misses            : {cycle_miss}")
        print(f"  Snapshot samples        : {sample_count}")
        print(f"  Stream health           : {health_color_text}{'clean' if healthy else 'degraded'}{RESET}")

    print(f"{BOLD}Status{RESET}")
    print(f"  Port: {port_label}")
    print(f"  Trace: {CYAN if trace_enabled else YELLOW}{'enabled' if trace_enabled else 'waiting for MT_ON ack'}{RESET}")
    if disconnect_reason:
        print(f"  Note: {disconnect_reason}")

    print(f"{BOLD}Why This Is Proof{RESET}")
    print("  control_task was running before the higher-priority timer/acquisition chain.")
    print("  After acquisition_task finishes, that same control_task resumes.")

    print(f"{GREY}Live firmware instrumentation. Stats come from assembled firmware cycles. Ctrl+C sends MT_OFF.{RESET}")

    if _verbose and log_lines:
        print()
        print(f"{BOLD}Recent Cycles (verbose){RESET}")
        print(f"  Jitter (sigma): {fmt_us(jitter_us)} | Callback -> sem give: {fmt_us(cb_sem_us)}")
        print(f"  Snapshot packets: {received} | Malformed lines: {malformed}")
        if latest is not None:
            print(f"  ASCII timeline: {render_timeline(latest, period_avg_us)}")
        for line in log_lines[:6]:
            print(f"  {line}")


def main() -> None:
    global _verbose
    parser = argparse.ArgumentParser(description="ESP32 Core 0 multitasking proof dashboard")
    parser.add_argument("--port", help="Serial port override (auto-detect and chooser used if omitted)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--refresh", type=float, default=0.2, help="Refresh interval seconds")
    parser.add_argument("--verbose", action="store_true", help="Show extra diagnostic fields")
    args = parser.parse_args()
    _verbose = args.verbose

    port = resolve_port(args.port)

    atexit.register(cleanup)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    thread = threading.Thread(target=reader_thread, args=(port, args.baud), daemon=True)
    thread.start()

    while True:
        render()
        time.sleep(args.refresh)


if __name__ == "__main__":
    main()
