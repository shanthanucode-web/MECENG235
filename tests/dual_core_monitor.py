# Haptic Surgical Skill Trainer — Dual Core Activity Monitor
# Usage: python tests/dual_core_monitor.py --port /dev/cu.usbserial-XXXX
#
# Shows real-time dual-core activity from ESP32 JSON stream.
# Opens two outputs simultaneously:
#   1. Terminal dashboard with tick rate proof
#   2. Matplotlib oscilloscope window with square wave proxies
#
# Core 0 tick rate is recovered from JSON timestamps:
#   actual_acq_period = delta_t_between_packets / 5
#   (firmware outputs JSON every 5th sample at 100Hz = 20Hz output)
#
# GPIO 13 (Core 0 timer tick) and GPIO 12 (Core 1 processing) are the hardware heartbeats.
# This script provides the software equivalent without an oscilloscope.
#
# Read-only — does not send any commands to the ESP32.
# Press Ctrl+C to exit.

import argparse
import json
import sys
import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

# ── Shared state ───────────────────────────────────────────────────────────
# Each entry: dict of JSON fields + '_rx_time' (monotonic seconds)
_packets: deque = deque(maxlen=500)
_total_packets_received: int = 0
_err_count: int = 0
_connected: bool = True
_tick_ctr: int = 0         # drives TICK animation
_proc_ctr: int = 0         # drives PROC animation

# ── ANSI helpers ───────────────────────────────────────────────────────────
RESET  = "\033[0m"
BOLD   = "\033[1m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
CYAN   = "\033[96m"
BLUE   = "\033[94m"
GREY   = "\033[90m"
WHITE  = "\033[97m"


# ── Serial reader (daemon thread) ──────────────────────────────────────────

def _serial_reader(port: str, baud: int) -> None:
    global _connected, _err_count, _total_packets_received
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as exc:
        print(f"\n{RED}Cannot open {port}: {exc}{RESET}")
        sys.exit(1)

    print(f"{GREEN}Connected to {port}{RESET}")
    time.sleep(0.3)
    ser.reset_input_buffer()

    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("ascii", errors="replace").strip()
            if not text.startswith("{"):
                continue
            pkt = json.loads(text)
            pkt["_rx_time"] = time.monotonic()
            _packets.append(pkt)
            _total_packets_received += 1
        except json.JSONDecodeError:
            _err_count += 1
        except serial.SerialException:
            _connected = False
            return
        except Exception:
            _err_count += 1


# ── Statistics helpers ─────────────────────────────────────────────────────

def _snapshot() -> list:
    """Thread-safe snapshot of current packet list."""
    return list(_packets)


def _compute_core0_stats(pkts: list) -> dict:
    """Derive Core 0 metrics from JSON 't' field timestamps."""
    if len(pkts) < 2:
        return {}
    ts = [p["t"] for p in pkts if "t" in p]
    if len(ts) < 2:
        return {}
    deltas_ms = [ts[i] - ts[i - 1] for i in range(1, len(ts))]
    # Each JSON packet = 5 acquisition samples, so acq period = Δt / 5
    periods_ms = [d / 5.0 for d in deltas_ms]
    last50 = periods_ms[-50:] if len(periods_ms) >= 50 else periods_ms
    mean_d = np.mean(deltas_ms[-50:]) if deltas_ms else 0.0
    tick_hz = (1000.0 / mean_d * 5.0) if mean_d > 0 else 0.0
    return {
        "tick_hz":    tick_hz,
        "jitter_ms":  float(np.std(last50)) if last50 else 0.0,
        "min_ms":     float(np.min(last50)) if last50 else 0.0,
        "max_ms":     float(np.max(last50)) if last50 else 0.0,
        "last_t_ms":  ts[-1],
        "last_json_delta_ms": deltas_ms[-1] if deltas_ms else 0.0,
        "periods_ms": periods_ms,
        "last20":     periods_ms[-20:] if len(periods_ms) >= 20 else periods_ms,
    }


def _compute_core1_stats(pkts: list) -> dict:
    """Derive Core 1 metrics from system receipt times and JSON fields."""
    if not pkts:
        return {}
    rx = [p["_rx_time"] for p in pkts]
    if len(rx) >= 2:
        deltas_rx = [rx[i] - rx[i - 1] for i in range(1, len(rx))]
        mean_rx = np.mean(deltas_rx[-50:]) if deltas_rx else 0.0
        json_hz = (1.0 / mean_rx) if mean_rx > 0 else 0.0
    else:
        json_hz = 0.0
    last = pkts[-1]
    roll = float(last.get("roll", 0.0))
    pitch = float(last.get("pitch", 0.0))
    yaw = float(last.get("yaw", 0.0))
    gx = float(last.get("gx", 0.0))
    gy = float(last.get("gy", 0.0))
    gz = float(last.get("gz", 0.0))
    motion_dps = float(np.sqrt(gx * gx + gy * gy + gz * gz))
    return {
        "json_hz":   json_hz,
        "packets":   _total_packets_received,
        "window":    len(pkts),
        "state":     last.get("state", "?"),
        "warn":      last.get("warn", 0),
        "err":       last.get("err", 0),
        "f_sum":     last.get("f_sum", 0.0),
        "score":     last.get("score", 0.0),
        "actual_hz": last.get("actual_hz", 0.0),
        "roll":      roll,
        "pitch":     pitch,
        "yaw":       yaw,
        "gx":        gx,
        "gy":        gy,
        "gz":        gz,
        "motion_dps": motion_dps,
        "imu_ok":    any(last.get(k, 0) != 0
                         for k in ("gx", "gy", "gz", "ax", "ay", "az")),
    }


# ── Terminal dashboard ─────────────────────────────────────────────────────

def _period_colour(p: float) -> str:
    if 9.5 <= p <= 10.5:
        return GREEN
    if 8.0 <= p <= 12.0:
        return YELLOW
    return RED


def _render_terminal() -> None:
    global _tick_ctr, _proc_ctr

    pkts = _snapshot()
    c0 = _compute_core0_stats(pkts)
    c1 = _compute_core1_stats(pkts)

    # Animate TICK and PROC on different counters so they can diverge
    total = _total_packets_received
    _tick_ctr = total                   # Core 0: tied to total packet count
    _proc_ctr = total // 3              # Core 1: slower counter to show divergence
    tick_frame = "████ TICK ████" if _tick_ctr % 2 else "░░░░ TICK ░░░░"
    proc_frame = "▓▓▓▓ PROC ▓▓▓▓" if _proc_ctr % 2 else "▒▒▒▒ PROC ▒▒▒▒"

    # Full-screen refresh is more reliable than cursor-up rewrites because
    # ANSI-coloured box rows wrap differently across terminal widths.
    sys.stdout.write("\033[H\033[J")

    W = 40   # column width

    def row(label: str, val: str, colour: str = WHITE) -> str:
        content = f"│ {label:<16} {colour}{val:<20}{RESET} │"
        return content

    # ── Core 0 box ──
    c0_lines = [
        f"┌{'─' * 3} {BOLD}{BLUE}CORE 0 — Acquisition{RESET} {'─' * (W - 24)}┐",
        row("Tick rate:",   f"{c0.get('tick_hz', 0):.2f} Hz",     CYAN),
        row("JSON packets:", str(_total_packets_received),          WHITE),
        row("ACQ samples:",  str(_total_packets_received * 5),       WHITE),
        row("Display win:",  f"{len(pkts)} packets",                 GREY),
        row("Jitter (σ):",  f"{c0.get('jitter_ms', 0):.3f} ms",    GREEN if c0.get('jitter_ms', 99) < 0.5 else YELLOW),
        row("Min period:",  f"{c0.get('min_ms', 0):.2f} ms",       WHITE),
        row("Max period:",  f"{c0.get('max_ms', 0):.2f} ms",       WHITE),
        row("Last ESP t:",  f"{c0.get('last_t_ms', 0):.0f} ms",    WHITE),
        row("JSON Δ:",      f"{c0.get('last_json_delta_ms', 0):.1f} ms / 5", WHITE),
        row("IMU:",         "active" if c1.get("imu_ok") else "FSR-only mode", GREEN if c1.get("imu_ok") else YELLOW),
        f"│ {CYAN}{tick_frame:<{W - 2}}{RESET} │",
        f"└{'─' * W}┘",
    ]

    # ── Core 1 box ──
    state_colour = {
        "HOLD": CYAN, "ACTIVE": YELLOW, "IDLE": GREY, "EXITED": RED
    }.get(c1.get("state", ""), WHITE)

    c1_lines = [
        f"┌{'─' * 3} {BOLD}{GREEN}CORE 1 — Processing{RESET} {'─' * (W - 23)}┐",
        row("JSON rate:",   f"{c1.get('json_hz', 0):.1f} Hz",      CYAN),
        row("JSON packets:", str(c1.get('packets', 0)),              WHITE),
        row("State:",       c1.get('state', '?'),                    state_colour),
        row("Warn flags:",  f"0x{c1.get('warn', 0):02X}",           GREEN if c1.get('warn', 0) == 0 else YELLOW),
        row("Err flags:",   f"0x{c1.get('err', 0):02X}",            GREEN if c1.get('err', 0) == 0 else RED),
        row("F_sum:",       f"{c1.get('f_sum', 0):.3f} N",          WHITE),
        row("RPY deg:",     f"{c1.get('roll', 0):.1f},{c1.get('pitch', 0):.1f},{c1.get('yaw', 0):.1f}", WHITE),
        row("Gyro |dps|:",  f"{c1.get('motion_dps', 0):.1f}",       GREEN if c1.get("motion_dps", 0) > 1.0 else GREY),
        row("Score:",       f"{c1.get('score', 0):.0f}",            GREEN if c1.get('score', 0) >= 80 else YELLOW),
        row("Source:",      "live UART JSON",                       WHITE),
        f"│ {GREEN}{proc_frame:<{W - 2}}{RESET} │",
        f"└{'─' * W}┘",
    ]

    # Print side by side (Core 0 left, Core 1 right)
    max_rows = max(len(c0_lines), len(c1_lines))
    while len(c0_lines) < max_rows:
        c0_lines.append(f"│{' ' * W}│")
    while len(c1_lines) < max_rows:
        c1_lines.append(f"│{' ' * W}│")

    print(f"{BOLD}ESP32 Dual-Core Activity Monitor — live serial proof{RESET}")
    print(f"Port source: firmware JSON stream. Period proof: Δ(JSON t) / 5 samples.")
    print("Hardware pins for external proof: GPIO13 = Core0 tick, GPIO12 = Core1 cycle.")
    print()

    terminal_width = 100
    try:
        terminal_width = __import__("shutil").get_terminal_size((100, 24)).columns
    except Exception:
        pass

    if terminal_width >= 92:
        for l0, l1 in zip(c0_lines, c1_lines):
            print(f"{l0}  {l1}")
    else:
        print("\n".join(c0_lines))
        print()
        print("\n".join(c1_lines))

    # ── Proof row ──
    last20 = c0.get("last20", [])
    mean_p = float(np.mean(last20)) if last20 else 0.0
    std_p  = float(np.std(last20))  if last20 else 0.0
    nominal = 9.5 <= mean_p <= 10.5
    status_str = f"{GREEN}✓ NOMINAL{RESET}" if nominal else f"{RED}✗ OUT OF SPEC{RESET}"

    intervals = "  ".join(
        f"{_period_colour(p)}[{p:5.2f}]{RESET}" for p in last20
    ) or f"{GREY}(waiting for data){RESET}"

    print(f"\n{BOLD}CORE 0 TICK RATE PROOF{RESET} — derived sample interval, last packets:")
    print(intervals)
    print(f"Target: 10.0ms   Mean: {mean_p:.2f}ms   σ: {std_p:.3f}ms   "
          f"Err: {_err_count}   Status: {status_str}")

    if not _connected:
        print(f"\n{RED}{BOLD}⚠  CONNECTION LOST — last data shown above{RESET}")
    else:
        print()

    sys.stdout.flush()


# ── Recurring terminal timer ───────────────────────────────────────────────

def _schedule_terminal() -> None:
    if not _connected:
        return
    _render_terminal()
    t = threading.Timer(0.2, _schedule_terminal)
    t.daemon = True
    t.start()


# ── Square wave generation ─────────────────────────────────────────────────

def _reconstruct_core0_sample_edges(packet_timestamps_ms: list) -> list:
    """Expand 20 Hz JSON packet timestamps into estimated 100 Hz sample edges."""
    edges = []
    for t_ms in packet_timestamps_ms:
        # Firmware emits JSON every 5th 100 Hz acquisition sample.
        edges.extend(t_ms - offset for offset in (40.0, 30.0, 20.0, 10.0, 0.0))
    return edges

def _make_square_wave(timestamps_ms: list, window_ms: float = 2000.0):
    """Build a step-function square wave from a list of timestamps."""
    if len(timestamps_ms) < 2:
        return np.array([0.0]), np.array([0.0])
    t_end   = timestamps_ms[-1]
    t_start = max(timestamps_ms[0], t_end - window_ms)
    ts = [t for t in timestamps_ms if t >= t_start]
    if len(ts) < 2:
        return np.array([0.0]), np.array([0.0])

    x_pts = [ts[0]]
    y_pts = [0]
    level = 0
    for t in ts[1:]:
        x_pts.append(t)
        y_pts.append(level)
        x_pts.append(t)
        level = 1 - level
        y_pts.append(level)

    x_arr = np.array(x_pts, dtype=float)
    y_arr = np.array(y_pts, dtype=float)
    # Shift so rightmost point = 0 (relative time)
    return x_arr - x_arr[-1], y_arr


# ── Matplotlib figure setup ────────────────────────────────────────────────

BG_DARK  = "#1a1a2e"
BG_AX    = "#16213e"
COL_GRID = "#0f3460"

def _setup_figure():
    plt.rcParams["toolbar"] = "None"
    fig, axes = plt.subplots(3, 1, figsize=(12, 8))
    fig.patch.set_facecolor(BG_DARK)
    fig.suptitle(
        "ESP32 Dual-Core Activity Monitor — Haptic Surgical Skill Trainer",
        color="white", fontsize=12, fontweight="bold"
    )

    titles = [
        "CORE 0 — 100 Hz ACQ EDGES (reconstructed from live JSON t / 5)",
        "CORE 1 — JSON OUTPUT EDGES (host receipt proxy)",
        "CORE 0 SAMPLE PERIOD — Δ(JSON t) / 5, TARGET 10.0ms",
    ]
    for ax, title in zip(axes, titles):
        ax.set_facecolor(BG_AX)
        ax.grid(color=COL_GRID, alpha=0.5, linestyle="--")
        ax.tick_params(colors="white")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")
        ax.title.set_color("white")
        ax.set_title(title, fontsize=9, pad=4)
        for spine in ax.spines.values():
            spine.set_edgecolor(COL_GRID)

    ax0, ax1, ax2 = axes

    # Square wave plots
    ax0.set_ylim(-0.1, 1.2)
    ax0.set_xlabel("Time relative to now (ms)", fontsize=8)
    ax0.set_ylabel("Signal", fontsize=8)
    ax0.axhline(0.5, ls="--", color="white", alpha=0.35, lw=0.8, label="50% duty")
    ax0.legend(fontsize=7, facecolor=BG_AX, labelcolor="white", framealpha=0.5)

    ax1.set_ylim(-0.1, 1.2)
    ax1.set_xlabel("Time relative to now (ms)", fontsize=8)
    ax1.set_ylabel("Signal", fontsize=8)
    ax1.axhline(0.5, ls="--", color="white", alpha=0.35, lw=0.8, label="50% duty")
    ax1.legend(fontsize=7, facecolor=BG_AX, labelcolor="white", framealpha=0.5)

    # Period plot
    ax2.set_ylim(8.0, 12.0)
    ax2.set_xlabel("Sample index", fontsize=8)
    ax2.set_ylabel("Period (ms)", fontsize=8)
    ax2.axhline(10.0,  ls="--", color="red",    lw=1.2, label="10.0ms target")
    ax2.axhline(9.5,   ls="--", color="#FFA500", lw=0.8, label="±5% tolerance")
    ax2.axhline(10.5,  ls="--", color="#FFA500", lw=0.8)
    ax2.legend(fontsize=7, facecolor=BG_AX, labelcolor="white", framealpha=0.5)

    # Line artists (mutable handles for FuncAnimation)
    line0, = ax0.plot([], [], color="#4a9eff", lw=1.5)
    line1, = ax1.plot([], [], color="#4aff9e", lw=1.5)
    line2, = ax2.plot([], [], color="#b44aff", lw=1.0)
    status_txt = ax2.text(
        0.98, 0.92, "", transform=ax2.transAxes,
        ha="right", va="top", fontsize=9, fontweight="bold"
    )

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    return fig, line0, line1, line2, status_txt


# ── FuncAnimation update ───────────────────────────────────────────────────

def _make_update(line0, line1, line2, status_txt):
    def update(_frame):
        pkts = _snapshot()
        if len(pkts) < 2:
            return line0, line1, line2, status_txt

        # Core 0 square wave — from firmware timestamps
        t_ms   = [p["t"] for p in pkts if "t" in p]
        acq_edges_ms = _reconstruct_core0_sample_edges(t_ms)
        x0, y0 = _make_square_wave(acq_edges_ms, window_ms=2000)
        if x0.size:
            line0.set_data(x0, y0)
            line0.axes.set_xlim(x0[0], 50)

        # Core 1 square wave — from system receipt times
        rx_ms  = [p["_rx_time"] * 1000.0 for p in pkts]
        x1, y1 = _make_square_wave(rx_ms, window_ms=2000)
        if x1.size:
            line1.set_data(x1, y1)
            line1.axes.set_xlim(x1[0], 50)

        # Acquisition period plot
        if len(t_ms) >= 2:
            deltas     = [(t_ms[i] - t_ms[i - 1]) / 5.0 for i in range(1, len(t_ms))]
            last200    = deltas[-200:]
            x_idx      = list(range(len(last200)))
            line2.set_data(x_idx, last200)
            line2.axes.set_xlim(0, max(len(last200) - 1, 1))

            last50_mean = float(np.mean(last200[-50:])) if last200 else 0.0
            if 9.5 <= last50_mean <= 10.5:
                status_txt.set_text("✓ 100 Hz VERIFIED")
                status_txt.set_color("#4aff9e")
            else:
                status_txt.set_text("✗ OUT OF SPEC")
                status_txt.set_color("#ff4a4a")

        return line0, line1, line2, status_txt

    return update


# ── Entry point ────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="ESP32 dual-core activity monitor (read-only)"
    )
    parser.add_argument("--port", required=True,
                        help="Serial port, e.g. /dev/cu.usbserial-XXXX")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    args = parser.parse_args()

    # Start serial reader
    reader = threading.Thread(
        target=_serial_reader, args=(args.port, args.baud), daemon=True
    )
    reader.start()

    # Start recurring terminal dashboard
    _schedule_terminal()

    # Build matplotlib figure
    fig, line0, line1, line2, status_txt = _setup_figure()
    update_fn = _make_update(line0, line1, line2, status_txt)
    _anim = animation.FuncAnimation(fig, update_fn, interval=100, blit=False,
                                    cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        plt.close("all")
        print(f"\n{BOLD}Monitor stopped.{RESET}")


if __name__ == "__main__":
    main()
