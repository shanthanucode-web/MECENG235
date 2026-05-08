"""
Haptic Surgical Skill Trainer

Clinical desktop dashboard for the ESP32 glove firmware. The GUI keeps the
existing UART command protocol and turns live JSON telemetry into force,
motion, coaching, calibration, and optional 3D orientation/contact views.

REDESIGN: _build_ui() and STYLESHEET updated for three-zone layout.
All data classes, workers, signal/slot connections, and serial logic unchanged.

How to read this file:
1. Start with TelemetryStore — it explains how live packets are cached.
2. Read SerialWorker — it owns the background UART thread.
3. Read MainWindow.__init__ and _build_ui — they show the top-level GUI shape.
4. Read _on_data_received, _handle_telemetry, and _handle_text_response —
   that is the main host-side control/data flow.
5. Read the calibration and summary dialogs last; they are consumers of the
   same runtime state, not the source of it.
"""

from __future__ import annotations

import json
import math
import queue
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from pathlib import Path

import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PySide6.QtCore import QPointF, QRectF, QSize, Qt, QThread, QTimer, Signal, Slot
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QFrame,
    QGraphicsDropShadowEffect,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)


# ── Constants ────────────────────────────────────────────────────────────────

BAUD_RATE = 115200
HISTORY_POINTS = 900
PLOT_WINDOW_S = 30.0
MAX_FEED_CARDS = 18
HUD_RECOVERY_DELAY_S = 1.4
HUD_CALM_STREAK_S = 0.45
HUD_SAME_LEVEL_SWAP_S = 0.85
# ── Color palette (unchanged) ────────────────────────────────────────────────

NAVY      = "#0f2d52"
BLUE      = "#1e88e5"
BLUE_DARK = "#155fa0"
BLUE_LIGHT = "#e8f3ff"
ICE       = "#f6fbff"
PANEL     = "#ffffff"
PANEL_ALT = "#f0f7ff"
BORDER    = "#cfe1f5"
GRID      = "#d9e9f8"
TEXT      = "#10263f"
MUTED     = "#b7b7b7"
GREEN     = "#1f9d55"
AMBER     = "#d98a00"
RED       = "#c62828"
VIOLET    = "#6f5bd6"
GRAY      = "#9aaec3"

WARN_TECH_LABELS = {
    1 << 0: "Hold instability",
    1 << 1: "Tremor",
    1 << 2: "Smoothness",
    1 << 3: "Swing rate",
    1 << 4: "Force opening",
    1 << 5: "Force variability",
    1 << 6: "Force spike",
}

ERR_TECH_LABELS = {
    1 << 0: "Hold instability",
    1 << 1: "Tremor",
    1 << 2: "Force opening",
    1 << 3: "Sustained compression",
}

WARN_COACH_LABELS = {
    1 << 0: "Hold steady",
    1 << 1: "Steady hand",
    1 << 2: "Move more smoothly",
    1 << 3: "Slow the motion",
    1 << 4: "Ease grip",
    1 << 5: "Keep pressure steady",
    1 << 6: "Avoid sudden force",
}

ERR_COACH_LABELS = {
    1 << 0: "Hold steady",
    1 << 1: "Steady hand",
    1 << 2: "Ease grip",
    1 << 3: "Release pressure",
}

CALIBRATION_STEPS = {
    "C1": {
        "step": 1,
        "nav": "Still Hand",
        "title": "Still Hand Baseline",
        "purpose": "Measures the glove while your hand is completely still so the trainer can separate normal motion from tremor.",
        "instruction": "Rest your hand on the table, keep your fingers relaxed, and avoid pressing the glove until the step finishes.",
        "running": "Measuring stillness",
        "success": "Still-hand baseline saved.",
        "failure": "The glove moved too much during the baseline. Rest your hand and run the step again.",
    },
    "C2": {
        "step": 2,
        "nav": "Finger Pressure",
        "title": "Finger Pressure Baseline",
        "purpose": "Learns the no-pressure sensor baseline so the glove can tell the difference between resting fingers and real instrument contact.",
        "instruction": "Keep your fingers off the pressure pads and let the glove sit naturally until the step finishes.",
        "running": "Measuring finger pressure baseline",
        "success": "Finger pressure baseline saved.",
        "failure": "The glove detected too much finger pressure. Relax your hand and run the step again.",
    },
    "C3": {
        "step": 3,
        "nav": "Light Grip",
        "title": "Normal Light Grip",
        "purpose": "Captures your typical light training grip so force warnings are scaled to normal use instead of a hardcoded assumption.",
        "instruction": "Hold a steady normal light grip until the glove confirms the force reference was captured.",
        "running": "Capturing normal light grip",
        "success": "Normal light grip saved.",
        "failure": "The glove could not capture a clean light-grip reference. Hold a steady light grip and run the step again.",
    },
    "C4": {
        "step": 4,
        "nav": "Hand Motion",
        "title": "Normal Hand Motion",
        "purpose": "Captures the way you naturally move your hand so the trainer can separate intended motion from actual tremor.",
        "instruction": "Keep a light grip and move your hand naturally for 10 seconds when prompted.",
        "running": "Capturing normal hand motion",
        "success": "Normal hand motion saved.",
        "failure": "The glove could not capture enough clean normal hand motion. Move naturally and run the step again.",
    },
    "Z": {
        "title": "Erase Saved Calibration",
        "purpose": "Clears the glove's saved calibration values so you can start over from the beginning.",
        "instruction": "Use this only when you need to reset calibration for a new user, a new glove fit, or obviously incorrect saved values.",
        "running": "Erasing saved calibration",
        "success": "Saved calibration erased.",
        "failure": "The saved calibration could not be erased. Try again while connected to the glove.",
    },
}

CORE_CALIBRATION_FLOW = ("C1", "C2", "C3", "C4")

CALIBRATION_STAGE_UI = {
    "grip_hold": {
        "title": "Hold light grip",
        "instruction": "Hold your normal light training grip steady until the first check appears.",
    },
    "hand_motion": {
        "title": "Move hand naturally",
        "instruction": "Keep a light grip and move your hand naturally for 10 seconds so the glove can learn what intended motion looks like for you.",
    },
}

CALIBRATION_STAGE_ORDER = tuple(CALIBRATION_STAGE_UI.keys())

CALIBRATION_REASON_TEXT = {
    "not_still": "The glove moved too much during the still-hand capture.",
    "pressure_present": "Finger pressure was still on the pads when the baseline was measured.",
    "unstable_pressure": "The pressure sensors were too noisy to capture a clean baseline.",
    "drift": "The pressure baseline drifted during capture. Let the glove settle and try again.",
    "grip_unstable": "The light grip was not steady enough to use as your normal force reference.",
    "range_too_small": "The motion window did not contain enough regular hand movement. Move a little more naturally and try again.",
    "returned_too_little": "The prior return-to-center check is no longer part of calibration.",
    "motion_too_short": "The motion window ended too early. Keep moving naturally until the capture finishes.",
    "motion_too_noisy": "The motion was too jittery to use as a clean reference.",
    "pressure_absent": "A steady light grip was not detected during the grip-capture phase.",
    "stage_failed": "One part of the calibration capture did not complete cleanly.",
    "save_failed": "The glove could not save the completed calibration stage.",
    "timeout": "The glove stopped waiting for data before the stage could finish.",
    "deprecated": "This calibration action is no longer part of the guided workflow.",
}


def calibration_reason_text(reason: str, fallback: str) -> str:
    return CALIBRATION_REASON_TEXT.get(reason, fallback)


class TrainerState(Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTED    = "CONNECTED"
    IDLE         = "IDLE"
    HOLD         = "HOLD"
    ACTIVE       = "ACTIVE"
    EXITED       = "EXITED"


STATE_COLORS = {
    TrainerState.DISCONNECTED: GRAY,
    TrainerState.CONNECTED:    BLUE,
    TrainerState.IDLE:         MUTED,
    TrainerState.HOLD:         GREEN,
    TrainerState.ACTIVE:       AMBER,
    TrainerState.EXITED:       RED,
}

COMMAND_RESPONSES = {
    "ESP32_TRAINER": ("CONNECTED", "Board identified"),
    "EASY":          ("EASY",         "Easy training mode"),
    "INTERMEDIATE":  ("INTERMEDIATE", "Intermediate training mode"),
    "HARD":          ("HARD",         "Hard training mode"),
    "STOPPED":       ("IDLE",         "Training stopped"),
    "EXITED":        ("EXITED",       "Session exited"),
}

MODE_LABELS = {
    "EASY":         "Easy",
    "INTERMEDIATE": "Intermediate",
    "HARD":         "Hard",
    "UNSELECTED":   "No Mode",
}

MODE_COLORS = {
    "EASY":         GREEN,
    "INTERMEDIATE": BLUE,
    "HARD":         RED,
    "UNSELECTED":   MUTED,
}

MODE_BY_COMMAND = {
    "E": "EASY",
    "M": "INTERMEDIATE",
    "H": "HARD",
}

COMMAND_BY_MODE = {mode: cmd for cmd, mode in MODE_BY_COMMAND.items()}


# ── Data classes (unchanged) ─────────────────────────────────────────────────

@dataclass
class TelemetryStore:
    """
    Rolling telemetry cache for plots and end-of-session summaries.

    The firmware already sends structured JSON; this class does two GUI-side
    jobs:
    - keep a bounded history window for live plotting
    - remember enough recent samples to build a session summary when the user
      stops a run

    The store keeps time as "seconds since session start" so the plots can use
    a stable x-axis even if ESP32 timestamps continue increasing across runs.
    """
    maxlen: int = HISTORY_POINTS
    packet_count: int = 0
    start_ms: float | None = None
    latest: dict = field(default_factory=dict)
    series: dict[str, deque] = field(default_factory=dict)

    def __post_init__(self) -> None:
        keys = (
            "t", "f0", "f1", "f2", "f_sum",
            "roll", "pitch", "yaw",
            "tremor", "cv_f", "swing", "f95",
            "score", "warn", "err", "contact_any", "engaged",
        )
        self.series = {key: deque(maxlen=self.maxlen) for key in keys}

    def reset(self) -> None:
        self.packet_count = 0
        self.start_ms = None
        self.latest = {}
        for values in self.series.values():
            values.clear()

    def append(self, packet: dict) -> None:
        # Convert absolute firmware time into a per-session relative timeline.
        t_ms = float(packet.get("t", 0.0))
        if self.start_ms is None:
            self.start_ms = t_ms
        rel_t = max(0.0, (t_ms - self.start_ms) / 1000.0)

        self.packet_count += 1
        self.latest = packet
        self.series["t"].append(rel_t)
        for key in self.series:
            if key == "t":
                continue
            # Some plot channels are derived on the host for convenience rather
            # than sent explicitly by the firmware.
            if key == "contact_any":
                contact = packet.get("contact", [0, 0, 0])
                if isinstance(contact, list) and len(contact) >= 3:
                    value = 1.0 if any(int(v) for v in contact[:3]) else 0.0
                else:
                    value = 1.0 if float(packet.get("f_sum", 0.0)) > 0.05 else 0.0
                self.series[key].append(value)
                continue
            if key == "engaged":
                self.series[key].append(1.0 if int(packet.get("engaged", 0)) else 0.0)
                continue
            self.series[key].append(float(packet.get(key, 0.0)))

    def x(self) -> list[float]:
        return list(self.series["t"])

    def y(self, key: str) -> list[float]:
        return list(self.series[key])


# ── Serial worker (unchanged) ────────────────────────────────────────────────

class SerialWorker(QThread):
    """
    Background serial thread.

    Qt widgets must stay on the main thread, so UART I/O lives here instead.
    The worker only does transport work:
    - drain the outbound command queue
    - read bytes from the ESP32
    - split them into lines
    - emit each line back to the GUI thread

    Higher-level meaning (telemetry vs status vs command response) is handled
    by MainWindow after the line crosses the thread boundary.
    """
    data_received = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, port: str, baud: int, cmd_queue: queue.Queue) -> None:
        super().__init__()
        self._port = port
        self._baud = baud
        self._cmd_queue = cmd_queue
        self._running = False

    def run(self) -> None:
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.1)
        except serial.SerialException as exc:
            self.error_occurred.emit(str(exc))
            return

        self._running = True
        # We read arbitrary byte chunks, not guaranteed full lines, so keep a
        # small reassembly buffer until '\n' arrives.
        rx_buf = bytearray()
        partial_since: float | None = None
        try:
            while self._running:
                # Outbound commands are queued by button clicks / dialogs in the
                # GUI thread and written here so the UI never blocks on serial.
                while not self._cmd_queue.empty():
                    try:
                        ser.write(self._cmd_queue.get_nowait())
                    except queue.Empty:
                        break
                raw = ser.read(max(1, ser.in_waiting or 0))
                if raw:
                    rx_buf.extend(raw)
                    partial_since = time.monotonic()
                    while b"\n" in rx_buf:
                        raw_line, _, rx_buf = rx_buf.partition(b"\n")
                        line = raw_line.decode("ascii", errors="replace").strip(" \t\r\n\0")
                        if line:
                            self.data_received.emit(line)
                    continue

                # If the ESP32 stops mid-line, surface the fragment instead of
                # silently discarding it. This is useful when debugging resets
                # or malformed output.
                if rx_buf and partial_since is not None and (time.monotonic() - partial_since) > 0.25:
                    fragment = rx_buf.decode("ascii", errors="replace").strip(" \t\r\n\0")
                    if fragment:
                        self.data_received.emit(f"<partial serial fragment> {fragment}")
                    rx_buf.clear()
                    partial_since = None
        except serial.SerialException as exc:
            self.error_occurred.emit(str(exc))
        finally:
            ser.close()

    def stop(self) -> None:
        self._running = False


# ── ClinicalValue widget (unchanged class; accent from sidebar spec) ──────────

class ClinicalValue(QFrame):
    def __init__(self, title: str, unit: str = "", accent: str = BLUE) -> None:
        super().__init__()
        self.setObjectName("clinical_value")
        self.setMinimumHeight(56)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(1)

        title_label = QLabel(title)
        title_label.setObjectName("metric_title")
        self._value = QLabel("--")
        self._value.setObjectName("metric_value")
        self._unit = QLabel(unit)
        self._unit.setObjectName("metric_unit")
        layout.addWidget(title_label)
        layout.addWidget(self._value)
        layout.addWidget(self._unit)
        self.set_accent(accent)

    def set_value(self, value: str) -> None:
        self._value.setText(value)

    def set_accent(self, color: str) -> None:
        self.setStyleSheet(
            "QFrame#clinical_value {"
            f"border-left: 4px solid {color};"
            "}"
        )


# ── KPI card (new widget for the Train tab summary row) ─────────────────────

class KpiCard(QFrame):
    """
    # DESIGN: KpiCard replaces SummaryCard — uses border-left accent instead of
    # border-top, giving a consistent left-aligned visual rhythm across all cards.
    # border-top accents felt like category tabs; border-left reads as data rows,
    # which matches the clinical value hierarchy of the sidebar metric cards.
    """
    def __init__(self, title: str, unit: str, accent: str) -> None:
        super().__init__()
        self.setObjectName("kpi_card")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(2)
        title_label = QLabel(title)
        title_label.setObjectName("kpi_label")
        self._value = QLabel("--")
        self._value.setObjectName("kpi_value")
        unit_label = QLabel(unit)
        unit_label.setObjectName("kpi_unit")
        layout.addWidget(title_label)
        layout.addWidget(self._value)
        layout.addWidget(unit_label)
        self.set_accent(accent)
        self._apply_shadow()

    def _apply_shadow(self) -> None:
        # DESIGN: Drop shadow applied in Python (QSS cannot do box-shadow) —
        # blurRadius=14 gives depth without looking like a popup; color has low
        # alpha (25) to stay subtle against the ICE background.
        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(14)
        shadow.setOffset(0, 2)
        shadow.setColor(QColor(15, 45, 82, 25))
        self.setGraphicsEffect(shadow)

    def set_value(self, value: str) -> None:
        self._value.setText(value)

    def set_accent(self, color: str) -> None:
        self.setStyleSheet(
            "QFrame#kpi_card {"
            f"border-left: 4px solid {color};"
            "}"
        )


# ── SummaryCard (kept for SessionSummaryDialog compatibility) ─────────────────

class SummaryCard(QFrame):
    def __init__(self, title: str, unit: str, accent: str) -> None:
        super().__init__()
        self.setObjectName("summary_card")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 8, 14, 8)
        layout.setSpacing(2)
        title_label = QLabel(title)
        title_label.setObjectName("summary_title")
        self._value = QLabel("--")
        self._value.setObjectName("summary_value")
        unit_label = QLabel(unit)
        unit_label.setObjectName("summary_unit")
        layout.addWidget(title_label)
        layout.addWidget(self._value)
        layout.addWidget(unit_label)
        self.set_accent(accent)

    def set_value(self, value: str) -> None:
        self._value.setText(value)

    def set_accent(self, color: str) -> None:
        self.setStyleSheet(
            "QFrame#summary_card {"
            f"border-top: 4px solid {color};"
            "}"
        )


# ── VitalSignWidget (unchanged painting logic) ───────────────────────────────

class VitalSignWidget(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("vital_sign")
        # DESIGN: Fixed height 80px — keeps the banner row compact; taller heights
        # ate into chart real estate without adding legibility to the heartbeat line.
        self.setFixedHeight(80)
        self.setMinimumWidth(180)
        self._active = False
        self._phase  = 0.0
        self._color  = MUTED
        self._timer  = QTimer(self)
        self._timer.timeout.connect(self._tick)

    def set_status(
        self,
        active: bool,
        status: str = "",
        detail: str = "",
        color: str = MUTED,
        pulse: bool = False,
    ) -> None:
        self._active = active
        self._color = color
        if pulse:
            if not self._timer.isActive():
                self._timer.start(70)
        else:
            self._timer.stop()
            self._phase = 0.0
        self.update()

    def set_countdown(self, text: str) -> None:
        self._active = False
        self._color  = BLUE
        self._timer.stop()
        self._phase  = 0.0
        self.update()

    def _tick(self) -> None:
        self._phase = (self._phase + 0.22) % (math.pi * 2.0)
        self.update()

    def paintEvent(self, event) -> None:
        _ = event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        rect = QRectF(self.rect()).adjusted(1.0, 1.0, -1.0, -1.0)
        painter.setPen(Qt.NoPen)
        painter.setBrush(Qt.NoBrush)

        left  = 8.0
        right = max(left + 24.0, rect.right() - 4.0)
        base  = rect.center().y()
        amp   = 14.0 if self._active else 3.0
        path  = QPainterPath()
        for i in range(96):
            ratio = i / 95.0
            x = left + ratio * (right - left)
            y = base + math.sin((ratio * 6.0 * math.pi) + self._phase) * amp * 0.10
            if self._active:
                beat = (ratio * 3.0 + self._phase / (math.pi * 2.0)) % 1.0
                if 0.11 <= beat <= 0.15:
                    y += amp * 0.55
                elif 0.15 < beat <= 0.19:
                    y -= amp * 1.15
                elif 0.19 < beat <= 0.25:
                    y += amp * 0.35
            if i == 0:
                path.moveTo(x, y)
            else:
                path.lineTo(x, y)

        painter.setPen(QPen(QColor(self._color), 2.2))
        painter.drawPath(path)


class AppIconWidget(QFrame):
    """Small vector-style scissors mark drawn for crisp rendering at 36x36."""

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("app_icon")
        self.setFixedSize(36, 36)

    def paintEvent(self, event) -> None:  # type: ignore[override]
        _ = event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        pen = QPen(QColor(NAVY), 1.7, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin)
        painter.setPen(pen)

        # Finger rings
        painter.drawEllipse(QRectF(8.0, 23.0, 8.5, 8.5))
        painter.drawEllipse(QRectF(19.5, 23.0, 8.5, 8.5))

        # Shanks converging into the hinge
        painter.drawLine(QPointF(16.1, 25.0), QPointF(17.8, 18.2))
        painter.drawLine(QPointF(20.0, 25.0), QPointF(18.3, 18.2))

        # Hinge
        painter.setBrush(QColor(NAVY))
        painter.drawEllipse(QRectF(17.0, 16.7, 2.2, 2.2))
        painter.setBrush(Qt.BrushStyle.NoBrush)

        # Blade body: one straight edge, one gently curved edge
        left_blade = QPainterPath(QPointF(17.7, 18.0))
        left_blade.lineTo(16.9, 6.7)
        left_blade.lineTo(18.4, 4.6)
        left_blade.lineTo(18.5, 18.0)
        painter.drawPath(left_blade)

        right_blade = QPainterPath(QPointF(18.4, 18.0))
        right_blade.cubicTo(QPointF(20.6, 14.8), QPointF(22.0, 10.2), QPointF(21.9, 5.0))
        painter.drawPath(right_blade)

        # Open tip
        painter.drawLine(QPointF(16.8, 6.7), QPointF(15.7, 5.0))
        painter.drawLine(QPointF(18.4, 4.7), QPointF(20.1, 3.2))


# ── Calibration UI ───────────────────────────────────────────────────────────

class CalibrationProgressBar(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._value = 0.0
        self.setFixedHeight(12)

    def set_value(self, value: float) -> None:
        self._value = max(0.0, min(1.0, float(value)))
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(0, 0, -1, -1)
        painter.setPen(QPen(QColor(BORDER), 1))
        painter.setBrush(QColor(ICE))
        painter.drawRoundedRect(rect, 6, 6)
        fill = QRectF(rect)
        fill.setWidth(max(8.0, rect.width() * self._value))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(BLUE if self._value < 1.0 else GREEN))
        painter.drawRoundedRect(fill, 6, 6)
        painter.end()


class CalibrationWizard(QDialog):
    command_requested = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Guided Calibration")
        self.setMinimumSize(QSize(860, 620))
        self._selected = "C1"
        self._phase = "idle"
        self._current_stage = ""
        self._progress = 0.0
        self._spinner_index = 0
        self._latest_packet: dict[str, object] = {}
        self._status_base = "Ready to begin"
        self._status_detail = "Review the instructions, then start the selected calibration step."
        self._completed_steps: set[str] = set()
        self._stage_events_seen = 0

        self._spinner_timer = QTimer(self)
        self._spinner_timer.timeout.connect(self._advance_spinner)
        self._stage_guard_timer = QTimer(self)
        self._stage_guard_timer.setSingleShot(True)
        self._stage_guard_timer.timeout.connect(self._handle_stage_guard_timeout)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(12)

        title = QLabel("Guided Calibration")
        title.setObjectName("dialog_title")
        subtitle = QLabel(
            "Work through the three setup steps in order. The trainer will guide wrist-motion capture automatically and tell you when a retry is needed."
        )
        subtitle.setWordWrap(True)
        subtitle.setObjectName("muted_label")
        layout.addWidget(title)
        layout.addWidget(subtitle)

        body = QHBoxLayout()
        body.setSpacing(12)

        step_col = QVBoxLayout()
        step_col.setSpacing(8)
        step_header = QLabel("Calibration Steps")
        step_header.setObjectName("panel_caption")
        step_col.addWidget(step_header)
        self._step_buttons: dict[str, QPushButton] = {}
        for cmd in CORE_CALIBRATION_FLOW:
            data = CALIBRATION_STEPS[cmd]
            btn = QPushButton(f"Step {data['step']}  {data['nav']}")
            btn.clicked.connect(lambda _checked=False, value=cmd: self._select_step(value))
            self._step_buttons[cmd] = btn
            step_col.addWidget(btn)
        step_col.addStretch()
        self._erase_btn = QPushButton("Erase Saved Calibration")
        self._erase_btn.setObjectName("danger_button")
        self._erase_btn.clicked.connect(lambda: self.command_requested.emit("Z"))
        step_col.addWidget(self._erase_btn)
        body.addLayout(step_col, 1)

        info = QFrame()
        info.setObjectName("dialog_panel")
        info_layout = QVBoxLayout(info)
        info_layout.setContentsMargins(18, 16, 18, 16)
        info_layout.setSpacing(10)

        self._step_kicker = QLabel("")
        self._step_kicker.setObjectName("summary_title")
        self._step_title = QLabel("")
        self._step_title.setObjectName("panel_title")
        self._stage_label = QLabel("")
        self._stage_label.setObjectName("summary_title")
        self._purpose = QLabel("")
        self._purpose.setWordWrap(True)
        self._purpose.setObjectName("instruction_text")
        self._instruction = QLabel("")
        self._instruction.setWordWrap(True)
        self._instruction.setObjectName("instruction_text")
        self._check_target = QLabel("")
        self._check_target.setObjectName("instruction_text")
        self._check_return = QLabel("")
        self._check_return.setObjectName("instruction_text")
        self._progress_bar = QProgressBar()
        self._progress_bar.setObjectName("calibration_progress")
        self._progress_bar.setTextVisible(False)
        self._status_chip = QLabel("")
        self._status_chip.setObjectName("command_chip")
        self._status_detail_label = QLabel("")
        self._status_detail_label.setWordWrap(True)
        self._status_detail_label.setObjectName("muted_label")

        self._primary_btn = QPushButton("Start Calibration")
        self._primary_btn.setObjectName("command_button")
        self._primary_btn.clicked.connect(self._run_selected)
        self._next_btn = QPushButton("Next Step")
        self._next_btn.setObjectName("outline_button")
        self._next_btn.clicked.connect(self._go_to_next_step)
        self._finish_btn = QPushButton("Finish")
        self._finish_btn.setObjectName("outline_button")
        self._finish_btn.clicked.connect(self.close)

        action_row = QHBoxLayout()
        action_row.setSpacing(8)
        action_row.addWidget(self._primary_btn, 1)
        action_row.addWidget(self._next_btn)
        action_row.addWidget(self._finish_btn)

        info_layout.addWidget(self._step_kicker)
        info_layout.addWidget(self._step_title)
        info_layout.addWidget(self._stage_label)
        info_layout.addWidget(self._purpose)
        info_layout.addWidget(self._instruction)
        info_layout.addWidget(self._check_target)
        info_layout.addWidget(self._check_return)
        info_layout.addWidget(self._progress_bar)
        info_layout.addWidget(self._status_chip)
        info_layout.addWidget(self._status_detail_label)
        info_layout.addLayout(action_row)
        body.addWidget(info, 2)

        layout.addLayout(body)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn, alignment=Qt.AlignmentFlag.AlignRight)
        self._select_step("C1")

    def update_live_packet(self, packet: dict) -> None:
        self._latest_packet = packet

    def _set_stage_label(self, stage: str) -> None:
        if stage in CALIBRATION_STAGE_ORDER:
            idx = CALIBRATION_STAGE_ORDER.index(stage) + 1
            self._stage_label.setText(f"Stage {idx} of {len(CALIBRATION_STAGE_ORDER)}")
            self._stage_label.show()
        else:
            self._stage_label.clear()
            self._stage_label.hide()

    def _update_progress_ui(self) -> None:
        if self._selected in ("C3", "C4"):
            self._progress_bar.show()
            self._progress_bar.setRange(0, 100)
            self._progress_bar.setValue(int(round(self._progress * 100.0)))
            return

        self._stage_label.hide()
        if self._phase == "running":
            self._progress_bar.show()
            self._progress_bar.setRange(0, 0)
        else:
            self._progress_bar.hide()
            self._progress_bar.setRange(0, 100)
            self._progress_bar.setValue(0)

    def _set_direction_checklist(self, stage: str, phase: str = "", checkpoint: str = "") -> None:
        if self._selected not in ("C3", "C4"):
            self._check_target.hide()
            self._check_return.hide()
            return

        if self._selected == "C3":
            target_text, return_text = ("Capture normal light grip", "")
            target_done = (stage == "grip_hold" and phase == "pass") or checkpoint == "grip"
            return_done = False
        else:
            target_text, return_text = ("Capture natural hand motion", "")
            target_done = (stage == "hand_motion" and phase == "pass") or checkpoint == "motion"
            return_done = False
        target_prefix = "✓" if target_done else "○"
        target_color = GREEN if target_done else MUTED
        self._check_target.setText(f"{target_prefix} {target_text}")
        self._check_target.setStyleSheet(f"color: {target_color}; font-size: 13px;")
        self._check_target.show()
        self._check_return.hide()

    def _clear_direction_checklist(self) -> None:
        self._check_target.clear()
        self._check_return.clear()
        self._check_target.hide()
        self._check_return.hide()

    def _set_status(self, phase: str, base: str, detail: str, progress: float | None = None) -> None:
        self._phase = phase
        self._status_base = base
        self._status_detail = detail
        if progress is not None:
            self._progress = max(0.0, min(1.0, progress))

        color = NAVY
        bg = BLUE_LIGHT
        if phase == "running":
            color = BLUE
            self._spinner_index = 0
            if not self._spinner_timer.isActive():
                self._spinner_timer.start(420)
        else:
            self._spinner_timer.stop()
            if phase == "passed":
                color = GREEN
                bg = PANEL_ALT
            elif phase == "failed":
                color = RED
                bg = "#fff2f2"
        self._status_chip.setText(base if phase != "running" else f"{base}.")
        self._status_chip.setStyleSheet(
            f"color: {color}; background-color: {bg};"
            f"border: 1px solid {BORDER}; border-radius: 8px; padding: 6px 8px; font-weight: 700;"
        )
        self._status_detail_label.setText(detail)
        self._purpose.setVisible(not (self._selected in ("C3", "C4") and self._phase != "idle"))
        self._update_progress_ui()
        self._update_action_buttons()

    def _start_stage_guard(self) -> None:
        self._stage_guard_timer.stop()
        self._stage_guard_timer.start(3500)

    def _stop_stage_guard(self) -> None:
        self._stage_guard_timer.stop()

    def _handle_stage_guard_timeout(self) -> None:
        if self._selected not in ("C3", "C4") or self._phase != "running" or self._stage_events_seen > 0:
            return
        step_title = CALIBRATION_STEPS[self._selected]["title"]
        self._set_status(
            "failed",
            f"{step_title} did not begin",
            f"Step {CALIBRATION_STEPS[self._selected]['step']} started, but the glove rebooted or stopped sending stage events. Check the engineering log for {self._selected}_STAGE lines and confirm the flashed firmware matches proto=cal-v3.",
            self._progress,
        )

    def _advance_spinner(self) -> None:
        if self._phase != "running":
            self._spinner_timer.stop()
            return
        self._spinner_index = (self._spinner_index + 1) % 4
        self._status_chip.setText(f"{self._status_base}{'.' * (self._spinner_index + 1)}")

    def _select_step(self, cmd: str) -> None:
        if self._phase == "running" and cmd != self._selected:
            self._set_status(
                "running",
                self._status_base,
                "Calibration is still running. Wait for it to finish before moving to another step.",
                self._progress,
            )
            return
        self._selected = cmd
        self._current_stage = ""
        self._progress = 0.0
        self._stage_events_seen = 0
        self._stop_stage_guard()
        data = CALIBRATION_STEPS[cmd]
        self._step_kicker.setText(f"Step {data['step']} of {len(CORE_CALIBRATION_FLOW)}")
        self._step_title.setText(data["title"])
        self._set_stage_label("")
        self._clear_direction_checklist()
        self._purpose.setText(data["purpose"])
        self._instruction.setText(data["instruction"])
        for key, btn in self._step_buttons.items():
            btn.setProperty("selected", "true" if key == cmd else "false")
            btn.style().unpolish(btn)
            btn.style().polish(btn)
        self._set_status("idle", "Ready to begin", "Review the instructions, then start this step.", 0.0)

    def _run_selected(self) -> None:
        if self._phase == "running":
            return
        data = CALIBRATION_STEPS[self._selected]
        self._current_stage = ""
        self._stage_events_seen = 0
        self._set_status(
            "running",
            data["running"],
            "Calibration is in progress. Keep the glove in the requested position until the trainer confirms this step.",
            0.0,
        )
        if self._selected in ("C3", "C4"):
            self._start_stage_guard()
        self.command_requested.emit(self._selected)

    def _go_to_next_step(self) -> None:
        idx = CORE_CALIBRATION_FLOW.index(self._selected)
        if idx + 1 < len(CORE_CALIBRATION_FLOW):
            self._select_step(CORE_CALIBRATION_FLOW[idx + 1])

    def _update_action_buttons(self) -> None:
        next_available = (
            self._phase == "passed"
            and self._selected in CORE_CALIBRATION_FLOW
            and CORE_CALIBRATION_FLOW.index(self._selected) < len(CORE_CALIBRATION_FLOW) - 1
        )
        self._next_btn.setVisible(next_available)
        self._next_btn.setEnabled(next_available)

        finish_available = self._phase == "passed" and self._selected == "C4"
        self._finish_btn.setVisible(finish_available)
        self._finish_btn.setEnabled(finish_available)

        self._primary_btn.setEnabled(self._phase != "running")
        if self._phase == "failed":
            self._primary_btn.setText("Retry Stage")
        elif self._phase == "passed" and self._selected in ("C3", "C4"):
            self._primary_btn.setText("Run Again")
        else:
            self._primary_btn.setText("Start Calibration")

        for key, btn in self._step_buttons.items():
            btn.setEnabled(self._phase != "running" or key == self._selected)
        self._erase_btn.setEnabled(self._phase != "running")

    def _update_motion_stage(
        self,
        stage: str,
        phase: str,
        progress: float,
        reason: str,
        checkpoint: str,
        axis: str,
    ) -> None:
        step_key = "C4" if stage == "hand_motion" else "C3"
        data = CALIBRATION_STAGE_UI.get(stage, {"title": "Guided capture", "instruction": "Follow the on-screen calibration cue."})
        self._current_stage = stage
        self._set_stage_label(stage)
        self._step_title.setText(data["title"])
        self._instruction.setText(data["instruction"])
        self._stop_stage_guard()
        self._set_direction_checklist(stage, phase, checkpoint)

        if phase == "prompt":
            if stage == "grip_hold":
                base = "Hold your normal light grip"
                detail = "Keep a steady light grip until the glove captures your typical force."
            else:
                base = "Move your hand naturally"
                detail = "Keep a light grip and move your hand naturally for 10 seconds so the glove can record intended motion."
            self._set_status(
                "running",
                base,
                detail,
                progress,
            )
        elif phase == "capturing":
            if stage == "grip_hold":
                base = "Capturing normal light grip"
                detail = "Keep the grip steady. The glove is measuring your typical force."
            else:
                base = "Capturing natural hand motion"
                detail = "Keep moving naturally until the motion capture finishes."
            self._set_status(
                "running",
                base,
                detail,
                progress,
            )
        elif phase == "pass":
            if stage == "grip_hold":
                base = "Normal grip captured"
                detail = "The glove captured your normal light-grip reference. Finalizing calibration now."
            else:
                base = "Natural hand motion captured"
                detail = "The glove captured a normal-motion reference. Finalizing calibration now."
            self._set_status(
                "running",
                base,
                detail,
                progress,
            )
        elif phase == "fail":
            self._set_status(
                "failed",
                "Needs retry",
                calibration_reason_text(reason, CALIBRATION_STEPS[step_key]["failure"]),
                progress,
            )

    def record_response(self, data: object) -> None:
        if isinstance(data, dict):
            self._record_payload(data)
        else:
            self._record_text(str(data))

    def mark_transport_restart(self, details: str) -> bool:
        if self._phase != "running":
            return False
        self._stop_stage_guard()
        if self._selected in ("C3", "C4"):
            self._set_status(
                "failed",
                "Calibration interrupted",
                f"The glove restarted during Step {CALIBRATION_STEPS[self._selected]['step']} before the capture finished. Connected firmware: {details or 'unknown'}. Re-run calibration after the device is stable.",
                self._progress,
            )
        else:
            self._set_status(
                "failed",
                "Calibration interrupted",
                f"The glove restarted while calibration was running. Connected firmware: {details or 'unknown'}. Re-run the current step.",
                self._progress,
            )
        return True

    def _record_payload(self, payload: dict) -> None:
        if "nvs" in payload:
            self._stop_stage_guard()
            self._set_status("passed", CALIBRATION_STEPS["Z"]["success"], "Saved calibration was cleared from the device.", 1.0)
            return
        if "err" in payload:
            self._stop_stage_guard()
            self._set_status("failed", "Calibration could not continue", f"Device reported an error: {payload['err']}.", self._progress)
            return

        cal = str(payload.get("cal", ""))
        status = str(payload.get("status", ""))
        reason = str(payload.get("reason", ""))

        if cal in ("C3_STAGE", "C4_STAGE"):
            self._stage_events_seen += 1
            self._selected = "C4" if cal.startswith("C4") else "C3"
            self._current_stage = str(payload.get("stage", ""))
            checkpoint = str(payload.get("checkpoint", ""))
            axis = str(payload.get("axis", ""))
            parent = self.parent()
            if parent is not None and hasattr(parent, "_log"):
                getattr(parent, "_log")(f"{cal} payload: {json.dumps(payload, separators=(',', ':'))}", "sys")
            self._update_motion_stage(
                self._current_stage,
                str(payload.get("phase", "live")),
                float(payload.get("progress", 0.0)),
                reason,
                checkpoint,
                axis,
            )
            return

        if cal in ("C3_START", "C4_START"):
            self._selected = "C4" if cal.startswith("C4") else "C3"
            self._current_stage = ""
            self._step_title.setText(CALIBRATION_STEPS[self._selected]["title"])
            stages = payload.get("stages", [])
            first_stage = stages[0] if isinstance(stages, list) and stages else ""
            if isinstance(first_stage, str) and first_stage in CALIBRATION_STAGE_UI:
                self._current_stage = first_stage
                self._set_stage_label(first_stage)
                self._set_direction_checklist(first_stage, "prompt", "")
                self._step_title.setText(CALIBRATION_STAGE_UI[first_stage]["title"])
                self._instruction.setText(CALIBRATION_STAGE_UI[first_stage]["instruction"])
                status_base = "Waiting for capture"
                detail = CALIBRATION_STAGE_UI[first_stage]["instruction"]
            else:
                self._set_stage_label("")
                self._clear_direction_checklist()
                self._instruction.setText(CALIBRATION_STEPS[self._selected]["instruction"])
                status_base = f"Preparing {CALIBRATION_STEPS[self._selected]['title'].lower()}"
                detail = CALIBRATION_STEPS[self._selected]["instruction"]
            self._stage_events_seen = 0
            self._start_stage_guard()
            self._set_status(
                "running",
                status_base,
                detail,
                0.0,
            )
            return

        if cal.endswith("_START"):
            base_cmd = cal.split("_")[0]
            if base_cmd in CALIBRATION_STEPS:
                self._selected = base_cmd
                self._set_stage_label("")
                self._set_status(
                    "running",
                    CALIBRATION_STEPS[base_cmd]["running"],
                    "Calibration is running. Keep the glove in the requested position until the trainer confirms this step.",
                    0.0,
                )
            return

        if cal.endswith("_LIVE"):
            base_cmd = cal.split("_")[0]
            if base_cmd == "C1":
                detail = f"Stillness is being measured now. Current motion level: {float(payload.get('omega', 0.0)):.2f}."
            else:
                detail = "Finger pressure baseline is being measured. Keep your fingers relaxed and off the pads."
            self._selected = base_cmd
            self._set_stage_label("")
            self._set_status("running", CALIBRATION_STEPS[base_cmd]["running"], detail, self._progress)
            return

        if cal == "COMPLETE":
            self._stop_stage_guard()
            active_step = self._selected if self._selected in ("C3", "C4") else "C3"
            if status == "PASS":
                self._completed_steps.add(active_step)
                self._set_stage_label("")
                self._clear_direction_checklist()
                self._step_title.setText(CALIBRATION_STEPS[active_step]["title"])
                self._instruction.setText(CALIBRATION_STEPS[active_step]["instruction"])
                detail = "All calibration steps are complete and the results were saved." if active_step == "C4" else "The light-grip reference was saved to the glove."
                self._set_status("passed", "Calibration saved to the glove", detail, 1.0)
            else:
                self._set_status("failed", "Calibration could not be saved", "Capture completed, but the device could not save the calibration. Run the step again.", self._progress)
            return

        if cal in CALIBRATION_STEPS:
            self._selected = cal
            data = CALIBRATION_STEPS[cal]
            if status == "PASS":
                self._stop_stage_guard()
                self._completed_steps.add(cal)
                detail = data["success"]
                if cal in ("C3", "C4"):
                    self._set_stage_label("")
                    self._clear_direction_checklist()
                    self._step_title.setText(data["title"])
                    self._instruction.setText(data["instruction"])
                    if cal == "C3":
                        detail = "Light-grip capture finished. The glove is saving the measured force reference now."
                    else:
                        detail = "Normal hand motion capture finished. The glove is saving the measured motion references now."
                self._set_status("passed", data["success"], detail, 1.0 if cal in ("C3", "C4") else self._progress)
                return
            if status == "FAIL" and cal in ("C3", "C4") and reason == "stage_failed" and self._phase == "failed":
                return
            if status == "FAIL":
                self._stop_stage_guard()
                self._set_status(
                    "failed",
                    "Calibration needs another try",
                    calibration_reason_text(reason, data["failure"]),
                    self._progress,
                )
                return

        if cal in ("C3_REP", "C4_CYCLE"):
            self._set_status("failed", "Legacy calibration action", calibration_reason_text(reason, "This calibration action is no longer part of the guided workflow."), self._progress)
            return

        self._record_text(f"Calibration update received for {cal}.")

    def _record_text(self, text: str) -> None:
        clean = text.strip()
        if not clean:
            return
        if "Not connected" in clean:
            self._set_status("failed", "Connect the glove first", "Calibration could not start because the glove is not connected.", self._progress)
            return
        if clean.startswith("<partial serial fragment>"):
            self._set_status("failed", "Serial line was incomplete", "The glove sent a partial calibration message. Retry the step and check the connection if this continues.", self._progress)
        self._status_detail_label.setText(clean)


# ── SessionSummaryDialog (unchanged) ─────────────────────────────────────────

class SessionSummaryDialog(QDialog):
    def __init__(self, summary: dict, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Session Summary")
        self.setMinimumSize(QSize(560, 500))

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 18, 20, 18)
        layout.setSpacing(12)

        title = QLabel("Session Summary")
        title.setObjectName("dialog_title")
        subtitle = QLabel(
            f"{summary['mode']} training | {summary['duration_s']:.1f} s | "
            f"{summary['samples']} telemetry packets"
        )
        subtitle.setObjectName("muted_label")
        layout.addWidget(title)
        layout.addWidget(subtitle)

        score_panel = QFrame()
        score_panel.setObjectName("dialog_panel")
        score_layout = QVBoxLayout(score_panel)
        score_layout.setContentsMargins(16, 14, 16, 14)
        score_layout.setSpacing(4)
        score = QLabel(f"{summary['final_score']:.1f}")
        score.setObjectName("summary_score_label")
        grade = QLabel(summary["grade"])
        grade.setObjectName("summary_grade_label")
        grade.setStyleSheet(f"color: {summary['grade_color']};")
        score_layout.addWidget(QLabel("Final Skill Score"))
        score_layout.addWidget(score)
        score_layout.addWidget(grade)
        layout.addWidget(score_panel)

        grid_panel = QFrame()
        grid_panel.setObjectName("dialog_panel")
        grid = QGridLayout(grid_panel)
        grid.setContentsMargins(12, 12, 12, 12)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        metrics = [
            ("Avg Force",    f"{summary['avg_force']:.3f} N"),
            ("Peak Force",   f"{summary['peak_force']:.3f} N"),
            ("Peak Finger",  f"{summary['peak_finger']:.3f} N"),
            ("Engaged Time", f"{summary['engaged_pct']:.0f}%"),
            ("Warnings",     str(summary["warning_events"])),
            ("Errors",       str(summary["error_events"])),
            ("Tremor Avg",   f"{summary['avg_tremor']:.3f}"),
            ("Smoothness f95", f"{summary['peak_f95']:.2f} Hz"),
        ]
        for idx, (name, value) in enumerate(metrics):
            item = ClinicalValue(name, "")
            item.set_value(value)
            item.set_accent(
                RED   if name == "Errors"   and summary["error_events"]   else
                AMBER if name == "Warnings" and summary["warning_events"] else
                BLUE
            )
            grid.addWidget(item, idx // 2, idx % 2)
        layout.addWidget(grid_panel)

        coaching = QLabel(summary["coaching"])
        coaching.setObjectName("instruction_text")
        coaching.setWordWrap(True)
        layout.addWidget(coaching)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn, alignment=Qt.AlignmentFlag.AlignRight)


# ── MainWindow ───────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        # Runtime state for the host application.
        #
        # Keep the mental model simple:
        # - SerialWorker owns the transport thread.
        # - _cmd_queue is the only outbound path to the board.
        # - _telemetry is the rolling cache for plots and summaries.
        # - everything else is UI/session state derived from firmware output.
        self._worker: SerialWorker | None = None
        self._cmd_queue: queue.Queue = queue.Queue()
        self._connected = False
        self._firmware_info = "Firmware: unknown"
        self._last_packet_at: float | None = None
        self._last_warn = 0
        self._last_err  = 0
        self._last_state = ""
        self._mode = "UNSELECTED"
        self._latest_packet: dict | None = None
        self._hand_process: subprocess.Popen | None = None
        self._calibration_wizard: CalibrationWizard | None = None
        self._session_summary: SessionSummaryDialog | None = None
        self._session_mode   = "UNSELECTED"
        self._session_active = False
        self._armed_mode     = "UNSELECTED"
        self._awaiting_arm_mode: str | None = None
        self._pending_live_mode: str | None = None
        self._countdown_value = 0
        self._last_score: float | None = None
        self._hud_level = "idle"
        self._hud_headline = ""
        self._hud_detail = ""
        self._hud_color = MUTED
        self._hud_hold_until = 0.0
        self._hud_calm_since: float | None = None
        self._hud_last_change_at = 0.0
        self._telemetry   = TelemetryStore()
        self._curves: dict[str, pg.PlotDataItem] = {}
        self._plot_widgets: list[pg.PlotWidget] = []
        self._metric_values: dict[str, ClinicalValue] = {}
        self._cmd_button_by_cmd: dict[str, QPushButton] = {}
        self._mode_buttons: dict[str, QPushButton] = {}

        self._countdown_timer = QTimer(self)
        self._countdown_timer.timeout.connect(self._advance_countdown)

        self.setWindowTitle("Haptic Surgical Skill Trainer")
        self.resize(1280, 800)
        self.setMinimumSize(QSize(1180, 740))
        pg.setConfigOptions(antialias=True, foreground=TEXT, background="#0a1929")

        # DESIGN: Single central widget with no margins — the three-zone layout
        # owns all spacing internally so the window chrome stays flush.
        page = QWidget()
        page.setObjectName("central")
        self.setCentralWidget(page)

        root = QVBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self._build_ui(root)

        # ── Init state (unchanged signal calls) ──────────────────────────────
        self._populate_ports()
        self._set_commands_enabled(False)
        self._set_state(TrainerState.DISCONNECTED)
        self._set_mode("UNSELECTED")
        self._refresh_non_session_banner()
        self._update_start_button()
        self._add_activity("System ready. Connect glove to begin.", BLUE)

        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._refresh_plots)
        self._plot_timer.start(100)

        self._stale_timer = QTimer(self)
        self._stale_timer.timeout.connect(self._check_stale)
        self._stale_timer.start(500)

    # ── NEW: Three-zone build entry point ─────────────────────────────────────

    def _build_ui(self, root: QVBoxLayout) -> None:
        """Build the window in layers: app bar, main content, engineering log."""
        self._build_app_bar(root)       # Zone 1 — fixed 64 px top bar
        self._build_content_area(root)  # Zone 2 — expands to fill remaining height
        self._build_engineering_log(root)  # collapsible; stays outside the split

    # ── ZONE 1: Application Bar ───────────────────────────────────────────────

    def _build_app_bar(self, root: QVBoxLayout) -> None:
        """
        # DESIGN: App bar is exactly 64 px and uses a 3 px left NAVY accent border
        # instead of a bottom highlight — this differentiates the brand edge from
        # the content separator (1 px BORDER bottom). The left bar echoes the
        # left-accent pattern used on KPI cards and metric cards throughout.
        # Considered a full NAVY background bar but rejected: dark headers conflict
        # with the clinical-white ICE aesthetic and obscure text on small displays.
        """
        bar = QFrame()
        bar.setObjectName("app_bar")
        bar.setFixedHeight(64)
        row = QHBoxLayout(bar)
        row.setContentsMargins(16, 0, 16, 0)
        row.setSpacing(12)

        # Brand: icon + title + subtitle
        brand_row = QHBoxLayout()
        brand_row.setSpacing(8)

        icon_box = AppIconWidget()
        brand_row.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        brand_text = QVBoxLayout()
        brand_text.setSpacing(0)
        brand_text.setContentsMargins(0, 1, 0, 0)
        title = QLabel("Haptic Surgical Skill Trainer")
        title.setObjectName("app_title")
        subtitle = QLabel("Clinical Training Console")
        subtitle.setObjectName("app_subtitle")
        brand_text.addWidget(title)
        brand_text.addWidget(subtitle)

        brand_row.addWidget(icon_box)
        brand_row.addLayout(brand_text)
        row.addLayout(brand_row, 1)

        # Center: status pill + packet rate
        center_row = QHBoxLayout()
        center_row.setSpacing(8)

        self._status_pill = QLabel("⬤  Disconnected")
        self._status_pill.setObjectName("status_pill")
        # DESIGN: status_pill uses border-radius:14px and dynamic background set
        # from Python so connected=green vs disconnected=gray reads in <1 second.
        # The filled circle char (⬤) is the indicator dot — avoids a custom widget.
        self._set_status_pill_disconnected()

        self._rate_label = QLabel("-- Hz")
        self._rate_label.setObjectName("rate_chip")

        center_row.addWidget(self._status_pill)
        center_row.addWidget(self._rate_label)
        row.addLayout(center_row)

        # Right: port combo + Refresh + Connect
        controls_row = QHBoxLayout()
        controls_row.setSpacing(8)

        self._port_combo = QComboBox()
        self._port_combo.setObjectName("port_combo")
        self._port_combo.setFixedWidth(180)

        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._populate_ports)

        self._btn_connect = QPushButton("Connect")
        self._btn_connect.setObjectName("connect_button")
        self._btn_connect.clicked.connect(self._on_connect_clicked)

        controls_row.addWidget(self._port_combo)
        controls_row.addWidget(refresh_btn)
        controls_row.addWidget(self._btn_connect)
        row.addLayout(controls_row)

        root.addWidget(bar)

    def _set_status_pill_disconnected(self) -> None:
        self._status_pill.setText("⬤  Disconnected")
        self._status_pill.setStyleSheet(
            f"color: {MUTED}; background: #f4f7fa;"
            f"border: 1.5px solid {BORDER};"
            "border-radius: 14px; padding: 4px 12px;"
            "font-size: 11px; font-weight: 800; letter-spacing: 0.04em;"
        )

    def _set_status_pill_connected(self) -> None:
        self._status_pill.setText("⬤  Connected")
        self._status_pill.setStyleSheet(
            f"color: {GREEN}; background: #edf8f2;"
            f"border: 1.5px solid {GREEN};"
            "border-radius: 14px; padding: 4px 12px;"
            "font-size: 11px; font-weight: 800; letter-spacing: 0.04em;"
        )

    # ── ZONE 2: Main Content Area ─────────────────────────────────────────────

    def _build_content_area(self, root: QVBoxLayout) -> None:
        """
        # DESIGN: Content area splits 70/30 (left column / sidebar) using a
        # QHBoxLayout with stretch factors 7 and 3. The sidebar has a fixed
        # max-width=310 so it doesn't balloon on ultra-wide displays. Considered
        # QSplitter but rejected — users shouldn't resize the sidebar; the sidebar
        # width was tuned so all buttons are fully readable without truncation.
        """
        split = QHBoxLayout()
        split.setContentsMargins(0, 0, 0, 0)
        split.setSpacing(0)

        self._build_left_column(split)
        self._build_sidebar(split)

        root.addLayout(split, 1)

    # ── LEFT COLUMN ──────────────────────────────────────────────────────────

    def _build_left_column(self, split: QHBoxLayout) -> None:
        left_widget = QWidget()
        left_widget.setObjectName("left_column")
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(12, 10, 10, 10)
        left_layout.setSpacing(8)

        # Pill tab bar
        self._build_tab_bar(left_layout)

        # Tab content container (stacked manually via show/hide)
        self._build_train_tab(left_layout)
        self._build_metrics_tab_widget(left_layout)
        self._build_review_tab_widget(left_layout)

        # Default: Train tab visible
        self._show_tab("Train")

        split.addWidget(left_widget, 7)

    def _build_tab_bar(self, root: QVBoxLayout) -> None:
        """
        # DESIGN: Pill-button tab bar replaces QTabWidget for the left column.
        # QPushButton with border-radius:20px and checkable=False (state managed
        # manually via objectName property) avoids QTabWidget's pane border which
        # bled a 1px BORDER line across the full width — looked like a bug.
        # Considered custom QTabBar subclass but QPushButton + property QSS is
        # simpler to maintain and easier for teammates to read.
        """
        tab_row = QHBoxLayout()
        tab_row.setSpacing(6)
        tab_row.setContentsMargins(0, 0, 0, 2)
        self._tab_buttons: dict[str, QPushButton] = {}
        for label in ("Train", "Metrics", "Review"):
            btn = QPushButton(label)
            btn.setObjectName("tab_pill")
            btn.setCheckable(False)
            btn.clicked.connect(lambda _checked=False, t=label: self._show_tab(t))
            self._tab_buttons[label] = btn
            tab_row.addWidget(btn)
        tab_row.addStretch()
        root.addLayout(tab_row)

    def _show_tab(self, tab: str) -> None:
        """Show the selected tab content; update pill active states."""
        for name, btn in self._tab_buttons.items():
            active = name == tab
            btn.setProperty("tab_active", "true" if active else "false")
            btn.style().unpolish(btn)
            btn.style().polish(btn)
        self._train_container.setVisible(tab == "Train")
        self._metrics_container.setVisible(tab == "Metrics")
        self._review_container.setVisible(tab == "Review")

    # ── TRAIN TAB ─────────────────────────────────────────────────────────────

    def _build_train_tab(self, root: QVBoxLayout) -> None:
        self._train_container = QWidget()
        self._train_container.setObjectName("train_container")
        layout = QVBoxLayout(self._train_container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        self._build_session_banner(layout)
        self._build_kpi_grid(layout)
        self._build_force_chart(layout)

        root.addWidget(self._train_container, 1)

    def _build_session_banner(self, root: QVBoxLayout) -> None:
        """
        # DESIGN: Session banner is a full-width card with a 4 px colored left
        # border that changes state color. Left-border state feedback chosen over
        # background tinting — tinted backgrounds interact poorly with the white
        # KPI cards below (creates apparent color bleed at close proximity).
        # The VitalSignWidget is embedded in the RIGHT side of the banner so the
        # heartbeat waveform is visible without scrolling from the Train default view.
        """
        banner = QFrame()
        banner.setObjectName("session_banner")
        self._session_banner = banner
        row = QHBoxLayout(banner)
        row.setContentsMargins(16, 12, 16, 12)
        row.setSpacing(16)

        # Left: state text + mode badge + score
        left = QVBoxLayout()
        left.setSpacing(4)

        self._state_label = QLabel("Connect glove")
        self._state_label.setObjectName("state_label")

        meta_row = QHBoxLayout()
        meta_row.setSpacing(8)
        self._mode_label = QLabel("No difficulty selected")
        self._mode_label.setObjectName("mode_badge")
        self._score_inline = QLabel("Score: --")
        self._score_inline.setObjectName("score_inline")
        meta_row.addWidget(self._mode_label)
        meta_row.addWidget(self._score_inline)
        meta_row.addStretch()

        # Status items below — packet / rate / contact for quick reference
        status_row = QHBoxLayout()
        status_row.setSpacing(16)
        self._packet_label = QLabel("Packets: 0")
        self._packet_label.setObjectName("banner_stat")
        self._contact_label = QLabel("Contact: --/--/--")
        self._contact_label.setObjectName("banner_stat")
        self._gate_label = QLabel("Gate: NONE")
        self._gate_label.setObjectName("banner_stat")
        self._stale_label = QLabel("Telemetry: waiting")
        self._stale_label.setObjectName("banner_stat")
        status_row.addWidget(self._packet_label)
        status_row.addWidget(self._contact_label)
        status_row.addWidget(self._gate_label)
        status_row.addWidget(self._stale_label)
        status_row.addStretch()

        left.addWidget(self._state_label)
        left.addLayout(meta_row)
        left.addLayout(status_row)

        row.addLayout(left, 1)
        self._vital_sign = VitalSignWidget()
        row.addWidget(self._vital_sign)
        root.addWidget(banner)

        # Helper refs for coaching / alerts (used by unchanged _update_coaching)
        self._coaching_label       = self._state_label  # reuse state label slot
        self._session_hint_label   = self._mode_label   # keep compat
        self._score_delta_label    = self._score_inline
        self._alert_label          = self._stale_label

    def _build_kpi_grid(self, root: QVBoxLayout) -> None:
        """
        # DESIGN: 4-column KPI grid with individual accent colors chosen by data
        # domain — GREEN=score (clinical performance), BLUE=force (primary sensor),
        # VIOLET=packets (data pipeline), AMBER=rate (frequency domain). This
        # color-to-domain mapping is consistent with the metric card accents below.
        # Considered 2×2 grid but 1×4 row keeps all KPIs scannable without vertical
        # eye movement — critical when the user is also watching the glove.
        """
        grid_row = QHBoxLayout()
        grid_row.setSpacing(8)

        self._score_card = KpiCard("Session Score", "resets every mode start", GREEN)
        self._force_card = KpiCard("Total Force",   "N",                       BLUE)
        self._packet_kpi = KpiCard("Packet Count",  "telemetry packets",       VIOLET)
        self._rate_kpi   = KpiCard("Telemetry Rate", "Hz",                      AMBER)

        for card in (self._score_card, self._force_card, self._packet_kpi, self._rate_kpi):
            grid_row.addWidget(card, 1)

        root.addLayout(grid_row)

    def _build_force_chart(self, root: QVBoxLayout) -> None:
        """
        # DESIGN: Chart container is a white outer card holding an inner dark-bg
        # pyqtgraph plot. The 2px NAVY border on the chart itself signals deliberate
        # contrast — without it the dark plot appeared to be a rendering error
        # against the surrounding white card. Rounded container (12px) gives the
        # chart a frame that matches the KPI cards above.
        # Sub-tab buttons (Force/Motion/Skill/Score) are outlined chips — not QTabWidget
        # — so the tab bar doesn't add a pane border that competes with the chart border.
        """
        chart_card = QFrame()
        chart_card.setObjectName("chart_card")
        chart_layout = QVBoxLayout(chart_card)
        chart_layout.setContentsMargins(0, 0, 0, 0)
        chart_layout.setSpacing(0)

        # Header row: title + sub-tabs
        header = QHBoxLayout()
        header.setContentsMargins(14, 10, 10, 8)
        header.setSpacing(8)
        chart_title = QLabel("Force Profile")
        chart_title.setObjectName("chart_title")
        header.addWidget(chart_title)
        header.addStretch()

        self._chart_sub_tabs: dict[str, QPushButton] = {}
        for label in ("Force", "Motion", "Skill", "Score"):
            btn = QPushButton(label)
            btn.setObjectName("chart_tab_chip")
            btn.clicked.connect(lambda _checked=False, t=label: self._switch_chart_tab(t))
            self._chart_sub_tabs[label] = btn
            header.addWidget(btn)
        chart_layout.addLayout(header)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setObjectName("chart_sep")
        chart_layout.addWidget(sep)

        # Plot widget stack (reuse existing _build_plots logic but in one container)
        self._plots_stack = QWidget()
        stack_layout = QVBoxLayout(self._plots_stack)
        stack_layout.setContentsMargins(0, 0, 0, 0)
        stack_layout.setSpacing(0)
        self._build_plots_in(stack_layout)
        chart_layout.addWidget(self._plots_stack, 1)

        root.addWidget(chart_card, 1)
        self._switch_chart_tab("Force")

    def _build_plots_in(self, root: QVBoxLayout) -> None:
        """Build the four pyqtgraph plot widgets, stacked, only one visible at a time."""
        self._plot_frames: dict[str, pg.PlotWidget] = {}

        force = self._make_plot("Force profile", "Time (s)", "Force (N)")
        force.setYRange(0, 5)
        self._curves["f0"]    = self._plot_curve(force, "Thumb",  "#2196f3")
        self._curves["f1"]    = self._plot_curve(force, "Index",  "#4caf50")
        self._curves["f2"]    = self._plot_curve(force, "Middle", "#9e9e9e")
        self._curves["f_sum"] = self._plot_curve(force, "Total",  "#ff9800", 3)
        self._plot_frames["Force"] = force
        root.addWidget(force)

        motion = self._make_plot("Orientation", "Time (s)", "Angle (deg)")
        motion.setYRange(-180, 180)
        self._curves["roll"]  = self._plot_curve(motion, "Roll",  BLUE)
        self._curves["pitch"] = self._plot_curve(motion, "Pitch", GREEN)
        self._curves["yaw"]   = self._plot_curve(motion, "Yaw",   AMBER)
        self._plot_frames["Motion"] = motion
        root.addWidget(motion)

        skill = self._make_plot("Skill stability", "Time (s)", "Scaled metric")
        skill.setYRange(0, 20)
        self._curves["tremor"] = self._plot_curve(skill, "Tremor x10", RED)
        self._curves["cv_f"]   = self._plot_curve(skill, "Force CV x10", VIOLET)
        self._curves["swing"]  = self._plot_curve(skill, "Swing",        BLUE)
        self._curves["f95"]    = self._plot_curve(skill, "F95",          AMBER)
        self._plot_frames["Skill"] = skill
        root.addWidget(skill)

        score = self._make_plot("Performance score", "Time (s)", "Score / event")
        score.setYRange(0, 100)
        self._curves["score"] = self._plot_curve(score, "Score",         GREEN, 3)
        self._curves["warn"]  = self._plot_curve(score, "Warning event", AMBER)
        self._curves["err"]   = self._plot_curve(score, "Error event",   RED)
        self._plot_frames["Score"] = score
        root.addWidget(score)

        # Hide all initially; _switch_chart_tab will show the default.
        for w in self._plot_frames.values():
            w.setVisible(False)

    def _switch_chart_tab(self, tab: str) -> None:
        for name, widget in self._plot_frames.items():
            widget.setVisible(name == tab)
        for name, btn in self._chart_sub_tabs.items():
            active = name == tab
            btn.setProperty("chip_active", "true" if active else "false")
            btn.style().unpolish(btn)
            btn.style().polish(btn)

    def _make_plot(self, title: str, x_label: str, y_label: str) -> pg.PlotWidget:
        """
        # DESIGN: Dark background (#0a1929) on all plot widgets — creates deliberate
        # medical-monitor contrast against the surrounding white/ICE panels. Grid
        # color #1a3050 is dark enough to be invisible at a glance but visible on
        # inspection, mimicking ECG paper. Axes text uses MUTED so numeric ticks
        # don't compete with the waveform colors.
        """
        plot = pg.PlotWidget(background="#0a1929")
        plot.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        plot.setTitle(title, color="#8ab4d4", size="11pt")
        plot.setLabel("bottom", x_label, color=MUTED)
        plot.setLabel("left",   y_label, color=MUTED)
        plot.showGrid(x=True, y=True, alpha=0.35)
        plot.addLegend(offset=(12, 12), labelTextColor="#8ab4d4")
        plot.getAxis("bottom").setPen("#1a3050")
        plot.getAxis("left").setPen("#1a3050")
        plot.getAxis("bottom").setTextPen(MUTED)
        plot.getAxis("left").setTextPen(MUTED)
        plot.setMenuEnabled(False)
        plot.setMouseEnabled(x=False, y=False)
        self._plot_widgets.append(plot)
        return plot

    def _plot_curve(self, plot: pg.PlotWidget, name: str,
                    color: str, width: int = 2) -> pg.PlotDataItem:
        return plot.plot([], [], name=name, pen=pg.mkPen(color=color, width=width))

    # ── METRICS TAB ──────────────────────────────────────────────────────────

    def _build_metrics_tab_widget(self, root: QVBoxLayout) -> None:
        """
        # DESIGN: Metrics tab eliminates the large dead whitespace that existed above
        # the 'Live Metrics' label in the original layout (caused by the QTabWidget
        # pane adding ~40px of empty padding before content). The new layout starts
        # the 3-column grid at y=0 within its container, with only an 8px top margin.
        """
        self._metrics_container = QWidget()
        layout = QVBoxLayout(self._metrics_container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setObjectName("metrics_scroll")

        inner = QWidget()
        inner_layout = QVBoxLayout(inner)
        inner_layout.setContentsMargins(0, 4, 0, 8)
        inner_layout.setSpacing(6)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)

        # DESIGN: 3-column grid (was 2-column) — matches the wider left column
        # (70% of window vs. the old right panel at ~330px). 3 columns gives each
        # metric card more compact proportions — feels like a clinical data table
        # rather than a list of tiles.
        specs = [
            ("f0",      "Thumb",    "N",       BLUE),
            ("f1",      "Index",    "N",       BLUE),
            ("f2",      "Middle",   "N",       BLUE),
            ("roll",    "Roll",     "deg",     BLUE_DARK),
            ("pitch",   "Pitch",    "deg",     BLUE_DARK),
            ("yaw",     "Yaw",      "deg",     BLUE_DARK),
            ("tremor",  "Tremor",   "ratio",   VIOLET),
            ("cv_f",    "Force CV", "ratio",   VIOLET),
            ("swing",   "Swing",    "Hz",      VIOLET),
            ("f95",     "F95",      "Hz",      VIOLET),
            ("contact", "Contact",  "T/I/M",   GREEN),
            ("warn_err","Warn/Err", "bitmask", AMBER),
        ]
        for idx, (key, title, unit, color) in enumerate(specs):
            item = ClinicalValue(title, unit, color)
            self._metric_values[key] = item
            grid.addWidget(item, idx // 3, idx % 3)

        inner_layout.addLayout(grid)
        inner_layout.addStretch()
        scroll.setWidget(inner)
        layout.addWidget(scroll)
        root.addWidget(self._metrics_container, 1)

    # ── REVIEW TAB ───────────────────────────────────────────────────────────

    def _build_review_tab_widget(self, root: QVBoxLayout) -> None:
        """
        # DESIGN: Review tab is a full-width scrollable activity feed. Each card
        # uses a 4 px left border (same color system as metric cards) + an icon
        # glyph (●/▲/✕/✓) so the severity is readable at peripheral vision without
        # reading the message text. Message text is 14px semibold; timestamp is
        # 10px MUTED mono — clear typographic hierarchy between 'what' and 'when'.
        """
        self._review_container = QWidget()
        layout = QVBoxLayout(self._review_container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        scroll = QScrollArea()
        scroll.setObjectName("review_scroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        container = QWidget()
        self._feed_layout = QVBoxLayout(container)
        self._feed_layout.setContentsMargins(0, 4, 0, 8)
        self._feed_layout.setSpacing(8)
        self._feed_layout.addStretch()
        scroll.setWidget(container)
        layout.addWidget(scroll)

        root.addWidget(self._review_container, 1)

    # ── RIGHT SIDEBAR ─────────────────────────────────────────────────────────

    def _build_sidebar(self, split: QHBoxLayout) -> None:
        """
        # DESIGN: Sidebar is a single white QFrame (not a QTabWidget) — all sections
        # are separated by 1px BORDER horizontal rules, not by tab panes. This gives
        # a unified-card feel that reads as a control panel, not a navigation system.
        # A single QVBoxLayout with internal separators + a bottom spacer is simpler
        # to maintain than nested tab widgets and avoids the extra pane border overhead.
        # Max-width 310px prevents the sidebar from becoming a second content column
        # on >1440px displays.
        """
        sidebar_card = QFrame()
        sidebar_card.setObjectName("sidebar_card")
        sidebar_card.setMaximumWidth(310)
        sidebar_card.setMinimumWidth(260)

        # DESIGN: Drop shadow on the sidebar card — distinguishes the control panel
        # from the content area without a heavy border. blurRadius=20 chosen to spread
        # across the gap between sidebar and content; alpha=28 keeps it subtle.
        sidebar_shadow = QGraphicsDropShadowEffect(sidebar_card)
        sidebar_shadow.setBlurRadius(20)
        sidebar_shadow.setOffset(-2, 0)
        sidebar_shadow.setColor(QColor(15, 45, 82, 28))
        sidebar_card.setGraphicsEffect(sidebar_shadow)

        outer = QVBoxLayout(sidebar_card)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        scroll = QScrollArea()
        scroll.setObjectName("sidebar_scroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

        inner_widget = QWidget()
        self._sidebar_layout = QVBoxLayout(inner_widget)
        self._sidebar_layout.setContentsMargins(14, 14, 14, 14)
        self._sidebar_layout.setSpacing(0)

        self._build_training_controls_section()
        self._sidebar_sep()
        self._build_calibration_section()
        self._sidebar_sep()
        self._build_visualization_section()

        # DESIGN: QSpacerItem pushes Exit Session to the bottom of the sidebar.
        # Adjacent placement of the danger action next to normal controls risks
        # accidental session termination — physical separation is the safest UX
        # pattern for irreversible actions on clinical devices.
        self._sidebar_layout.addItem(
            QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        )

        scroll.setWidget(inner_widget)
        outer.addWidget(scroll, 1)

        # Exit Session pinned at the very bottom, outside the scroll area
        self._build_exit_section(outer)

        split.addWidget(sidebar_card)

    def _sidebar_sep(self) -> None:
        """Insert a 1px horizontal rule between sidebar sections."""
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setObjectName("sidebar_sep")
        self._sidebar_layout.addWidget(sep)

    def _sidebar_section_header(self, text: str) -> QLabel:
        """
        # DESIGN: Section headers are 9px all-caps MUTED with letter-spacing 0.08em.
        # All-caps + letter-spacing differentiates structural labels from data labels
        # without adding a new typeface or weight to the hierarchy. The thin bottom
        # border (BORDER color) provides a subtle grouping underline without a heavy
        # visual separator that would compete with the data cards.
        """
        lbl = QLabel(text)
        lbl.setObjectName("section_header")
        self._sidebar_layout.addWidget(lbl)
        return lbl

    def _build_training_controls_section(self) -> None:
        self._sidebar_layout.addSpacing(4)
        self._sidebar_section_header("TRAINING CONTROLS")
        self._sidebar_layout.addSpacing(8)

        # Mode buttons stacked full-width
        # DESIGN: Mode buttons are full-width stacked — considered 3 equal columns
        # but "Intermediate" truncates at any sidebar width below 290px; stacked
        # layout scales correctly at all sizes and groups the buttons as a mutually
        # exclusive radio set, which is visually clearer than a grid.
        self._cmd_buttons: list[QPushButton] = []

        easy_btn = QPushButton("Easy")
        easy_btn.setObjectName("mode_easy")
        easy_btn.clicked.connect(lambda _=False: self._select_mode("E"))
        self._cmd_buttons.append(easy_btn)
        self._cmd_button_by_cmd["E"] = easy_btn
        self._mode_buttons["EASY"] = easy_btn
        self._sidebar_layout.addWidget(easy_btn)
        self._sidebar_layout.addSpacing(6)

        int_btn = QPushButton("Intermediate")
        int_btn.setObjectName("mode_intermediate")
        int_btn.clicked.connect(lambda _=False: self._select_mode("M"))
        self._cmd_buttons.append(int_btn)
        self._cmd_button_by_cmd["M"] = int_btn
        self._mode_buttons["INTERMEDIATE"] = int_btn
        self._sidebar_layout.addWidget(int_btn)
        self._sidebar_layout.addSpacing(6)

        hard_btn = QPushButton("Hard")
        hard_btn.setObjectName("mode_hard")
        hard_btn.clicked.connect(lambda _=False: self._select_mode("H"))
        self._cmd_buttons.append(hard_btn)
        self._cmd_button_by_cmd["H"] = hard_btn
        self._mode_buttons["HARD"] = hard_btn
        self._sidebar_layout.addWidget(hard_btn)
        self._sidebar_layout.addSpacing(10)

        # Start button — primary action
        self._btn_start = QPushButton("▶  Start")
        self._btn_start.setObjectName("start_button")
        self._btn_start.clicked.connect(self._start_countdown)
        self._btn_start.setEnabled(False)
        # DESIGN: 48px min-height on Start button — primary action must be
        # thumb-sized and impossible to miss. 36px felt weak against the sidebar's
        # other elements; 48px matches the touch target recommendation for clinical
        # interfaces where gloves may be worn.
        self._btn_start.setMinimumHeight(48)
        self._sidebar_layout.addWidget(self._btn_start)
        self._sidebar_layout.addSpacing(6)

        # Stop button (secondary)
        stop_btn = QPushButton("■  Stop")
        stop_btn.setObjectName("command_button")
        stop_btn.setToolTip("Send S — stop session")
        stop_btn.clicked.connect(lambda _=False: self._send_command("S"))
        self._cmd_buttons.append(stop_btn)
        self._cmd_button_by_cmd["S"] = stop_btn
        self._sidebar_layout.addWidget(stop_btn)

        self._countdown_label = QLabel("Pick a difficulty, then press Start.")
        self._countdown_label.setObjectName("countdown_label")
        self._countdown_label.setWordWrap(True)
        self._sidebar_layout.addSpacing(6)
        self._sidebar_layout.addWidget(self._countdown_label)
        self._sidebar_layout.addSpacing(14)

    def _build_calibration_section(self) -> None:
        self._sidebar_layout.addSpacing(12)
        self._sidebar_section_header("CALIBRATION")
        self._sidebar_layout.addSpacing(8)

        calibrate = QPushButton("Calibrate…")
        calibrate.setObjectName("outline_button")
        calibrate.clicked.connect(self._open_calibration_wizard)
        self._sidebar_layout.addWidget(calibrate)

        self._cal_status = QLabel("No calibration yet.")
        self._cal_status.setObjectName("muted_label")
        self._fw_info_label = QLabel(self._firmware_info)
        self._fw_info_label.setObjectName("muted_label")
        self._fw_info_label.setWordWrap(True)
        self._sidebar_layout.addSpacing(4)
        self._sidebar_layout.addWidget(self._cal_status)
        self._sidebar_layout.addSpacing(4)
        self._sidebar_layout.addWidget(self._fw_info_label)
        self._sidebar_layout.addSpacing(14)

    def _build_visualization_section(self) -> None:
        self._sidebar_layout.addSpacing(12)
        self._sidebar_section_header("3D VISUALIZATION")
        self._sidebar_layout.addSpacing(8)

        open_3d = QPushButton("Open 3D Hand View")
        open_3d.setObjectName("outline_button")
        open_3d.clicked.connect(self._open_hand_window)
        self._sidebar_layout.addWidget(open_3d)

        self._hand_status = QLabel("3D window closed")
        self._hand_status.setObjectName("muted_label")
        self._hand_status.setWordWrap(True)
        self._sidebar_layout.addSpacing(4)
        self._sidebar_layout.addWidget(self._hand_status)
        self._sidebar_layout.addSpacing(14)

    def _build_exit_section(self, outer: QVBoxLayout) -> None:
        """
        # DESIGN: Exit Session sits outside the scroll area in a pinned bottom strip
        # separated by a BORDER top-rule. Using a separate QWidget (not inside the
        # scroll) guarantees it is always visible at the bottom regardless of how
        # much content is in the scroll — the danger action must never be hidden.
        """
        exit_strip = QFrame()
        exit_strip.setObjectName("exit_strip")
        exit_layout = QVBoxLayout(exit_strip)
        exit_layout.setContentsMargins(14, 10, 14, 14)
        exit_layout.setSpacing(6)

        exit_btn = QPushButton("Exit Session")
        exit_btn.setObjectName("danger_button")
        exit_btn.setToolTip("Send X — exit session")
        exit_btn.clicked.connect(lambda _=False: self._send_command("X"))
        self._cmd_buttons.append(exit_btn)
        self._cmd_button_by_cmd["X"] = exit_btn
        exit_layout.addWidget(exit_btn)

        outer.addWidget(exit_strip)

    # ── Engineering log (unchanged) ───────────────────────────────────────────

    def _build_engineering_log(self, root: QVBoxLayout) -> None:
        self._btn_terminal = QPushButton("Show Engineering Log")
        self._btn_terminal.setObjectName("terminal_button")
        self._btn_terminal.clicked.connect(self._toggle_terminal)
        root.addWidget(self._btn_terminal)

        self._terminal_container = QWidget()
        term_layout = QVBoxLayout(self._terminal_container)
        term_layout.setContentsMargins(0, 0, 0, 0)
        term_layout.setSpacing(0)
        self._log_view = QPlainTextEdit()
        self._log_view.setObjectName("log")
        self._log_view.setReadOnly(True)
        self._log_view.setMinimumHeight(170)
        term_layout.addWidget(self._log_view)
        self._terminal_container.setVisible(False)
        root.addWidget(self._terminal_container)

    # ════════════════════════════════════════════════════════════════════════
    # All methods below this line are UNCHANGED from original — signal/slot
    # connections, serial logic, data processing, calibration, session state.
    # ════════════════════════════════════════════════════════════════════════

    @Slot()
    def _toggle_terminal(self) -> None:
        visible = not self._terminal_container.isVisible()
        self._terminal_container.setVisible(visible)
        self._btn_terminal.setText(
            "Hide Engineering Log" if visible else "Show Engineering Log"
        )
        if visible:
            bar = self._log_view.verticalScrollBar()
            bar.setValue(bar.maximum())

    @Slot()
    def _populate_ports(self) -> None:
        self._port_combo.clear()
        ports = serial.tools.list_ports.comports()
        preferred = [p for p in ports if "cu." in p.device or "ttyUSB" in p.device]
        others = [p for p in ports if p not in preferred]
        for port in preferred + others:
            label = port.device
            if port.description and port.description != "n/a":
                label += f" - {port.description}"
            self._port_combo.addItem(label, userData=port.device)
        if not ports:
            self._port_combo.addItem("No serial ports found", userData=None)

    def _select_mode(self, cmd: str) -> None:
        if self._session_active:
            self._add_activity("Stop the current session before changing modes.", AMBER)
            return
        if cmd not in MODE_BY_COMMAND:
            return
        mode = MODE_BY_COMMAND[cmd]
        if self._countdown_timer.isActive():
            self._cancel_countdown("Countdown cancelled. Pick a difficulty and press Start.")
        self._pending_live_mode   = None
        self._awaiting_arm_mode   = mode
        self._armed_mode          = mode
        self._set_mode(mode)
        self._countdown_label.setText("Setting difficulty on the glove…")
        self._update_start_button()
        self._send_command(cmd)

    def _arm_training_mode(self, mode: str) -> None:
        self._armed_mode         = mode
        self._session_mode       = mode
        self._mode               = mode
        self._session_active     = False
        self._awaiting_arm_mode  = None
        self._pending_live_mode  = None
        self._set_mode(mode)
        self._score_card.set_value("--")
        self._score_card.set_accent(GRAY)
        self._score_delta_label.setText("Score: --")
        self._score_delta_label.setStyleSheet(f"color: {MUTED};")
        self._countdown_label.setText("Difficulty selected. Press Start when you're ready.")
        self._refresh_non_session_banner()
        self._update_start_button()

    def _start_countdown(self) -> None:
        if not self._connected:
            self._add_activity("Connect to the glove before starting.", AMBER)
            return
        if self._armed_mode not in COMMAND_BY_MODE:
            self._add_activity("Select Easy, Intermediate, or Hard before starting.", AMBER)
            return
        if self._session_active:
            self._add_activity("A session is already live.", AMBER)
            return
        self._pending_live_mode  = None
        self._countdown_value    = 3
        self._set_mode(self._armed_mode, countdown=True)
        self._set_countdown_text("3")
        self._btn_start.setEnabled(False)
        self._countdown_timer.start(1000)
        self._update_start_button()
        self._add_activity(f"{MODE_LABELS[self._armed_mode]} countdown started", BLUE)

    def _advance_countdown(self) -> None:
        if self._countdown_value > 1:
            self._countdown_value -= 1
            self._set_countdown_text(str(self._countdown_value))
            return
        self._countdown_timer.stop()
        self._countdown_value = 0
        self._set_countdown_text("Begin")
        QTimer.singleShot(350, self._finish_countdown)

    def _finish_countdown(self) -> None:
        if self._session_active or self._armed_mode not in COMMAND_BY_MODE:
            return
        if not self._connected:
            self._cancel_countdown("Connection lost before start.")
            return
        self._pending_live_mode  = self._armed_mode
        self._countdown_label.setText("Starting session…")
        self._send_command(COMMAND_BY_MODE[self._armed_mode])

    def _set_countdown_text(self, text: str) -> None:
        label = f"Starting in {text}" if text != "Begin" else "Begin"
        self._countdown_label.setText(label)
        self._vital_sign.set_countdown(text)
        self._apply_player_banner("Get ready", "", BLUE, active=False, pulse=False)

    def _cancel_countdown(self, message: str) -> None:
        self._countdown_timer.stop()
        self._countdown_value    = 0
        self._awaiting_arm_mode  = None
        self._pending_live_mode  = None
        self._countdown_label.setText(message)
        self._refresh_non_session_banner()
        self._update_start_button()

    def _update_start_button(self) -> None:
        button = getattr(self, "_btn_start", None)
        if button is None:
            return
        can_start = (
            self._connected and
            self._armed_mode in COMMAND_BY_MODE and
            not self._session_active and
            not self._countdown_timer.isActive() and
            self._awaiting_arm_mode is None and
            self._pending_live_mode is None
        )
        button.setEnabled(can_start)
        if self._countdown_timer.isActive():
            button.setText("Starting…")
        elif self._awaiting_arm_mode is not None:
            button.setText("Selecting…")
        elif self._session_active:
            button.setText("▶  Live")
        else:
            button.setText("▶  Start")

    @Slot()
    def _on_connect_clicked(self) -> None:
        if self._connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        # Host-side connection setup is intentionally lightweight: spawn the
        # serial worker, reset GUI session state, and immediately identify the
        # board with the existing 'I' command.
        port = self._port_combo.currentData()
        if not port:
            self._log("No port selected", "err")
            return
        self._cmd_queue = queue.Queue()
        self._worker    = SerialWorker(port, BAUD_RATE, self._cmd_queue)
        self._worker.data_received.connect(self._on_data_received)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.finished.connect(self._on_worker_finished)
        self._worker.start()
        self._connected      = True
        self._last_packet_at = None
        self._reset_session_display(clear_mode=True)
        self._btn_connect.setText("Disconnect")
        self._btn_connect.setProperty("connected", "true")
        self._repolish(self._btn_connect)
        self._set_status_pill_connected()
        self._set_commands_enabled(True)
        self._update_start_button()
        self._set_state(TrainerState.CONNECTED)
        self._stale_label.setText("Telemetry: waiting")
        self._stale_label.setStyleSheet(f"color: {AMBER};")
        self._log(f"Opening {port} at {BAUD_RATE}", "sys")
        self._add_activity(f"Connected to {port.split('/')[-1]}", BLUE)
        self._send_command("I")

    def _disconnect(self) -> None:
        if self._worker:
            self._worker.stop()
            self._worker.wait()
            self._worker = None
        self._connected      = False
        self._firmware_info  = "Firmware: unknown"
        self._last_packet_at = None
        self._reset_session_display(clear_mode=True)
        self._btn_connect.setText("Connect")
        self._btn_connect.setProperty("connected", "false")
        self._repolish(self._btn_connect)
        self._set_status_pill_disconnected()
        self._set_commands_enabled(False)
        self._update_start_button()
        self._set_state(TrainerState.DISCONNECTED)
        self._stale_label.setText("Telemetry: disconnected")
        self._stale_label.setStyleSheet(f"color: {MUTED};")
        if hasattr(self, "_fw_info_label"):
            self._fw_info_label.setText(self._firmware_info)
        self._log("Disconnected", "sys")
        self._add_activity("Disconnected", MUTED)

    @Slot(str)
    def _on_data_received(self, line: str) -> None:
        self._log(line, "rx")
        # All serial input enters here as plain text. The first split is:
        # - JSON: structured telemetry or structured status/calibration updates
        # - plain text: simple command responses and READY banners
        if line.startswith("{"):
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                self._add_activity("Malformed JSON packet", RED)
                return
            if "t" in payload and "f_sum" in payload:
                self._handle_telemetry(payload)
            else:
                self._handle_status_json(payload)
            return
        self._handle_text_response(line)

    def _handle_telemetry(self, packet: dict) -> None:
        # This path is for the high-rate runtime JSON stream: force, IMU,
        # warnings, score, state, contact, and so on.
        self._latest_packet  = packet
        self._last_packet_at = time.monotonic()
        self._stale_label.setText("Telemetry: live")
        self._stale_label.setStyleSheet(f"color: {GREEN};")
        if self._calibration_wizard is not None and self._calibration_wizard.isVisible():
            self._calibration_wizard.update_live_packet(packet)
        if self._session_active:
            # Only store and score packets against a user session once Start
            # has actually armed a mode. This prevents idle packets from
            # polluting plots or summaries.
            self._telemetry.append(packet)
            self._packet_label.setText(f"Packets: {self._telemetry.packet_count}")
            self._packet_kpi.set_value(str(self._telemetry.packet_count))
            if len(self._telemetry.series["t"]) >= 2:
                times = list(self._telemetry.series["t"])
                dt    = times[-1] - times[-2]
                rate  = (1.0 / dt) if dt > 0 else 0.0
                self._rate_label.setText(f"{rate:.1f} Hz")
                self._rate_kpi.set_value(f"{rate:.1f}")
        else:
            self._rate_label.setText("live")
        # The firmware is the source of truth for surgical state (IDLE/HOLD/
        # ACTIVE/EXITED), so the GUI mirrors that instead of inventing its own.
        state = str(packet.get("state", "CONNECTED"))
        self._set_state(self._state_from_text(state))
        self._update_metrics(packet, scored=self._session_active)
        if self._session_active:
            self._update_coaching(packet)
        self._send_to_hand_visualizer(packet)

    def _handle_status_json(self, payload: dict) -> None:
        # Non-telemetry JSON packets are mostly calibration/proof/NVS status.
        # They still matter, but they update activity/coaching state rather
        # than the live numeric plots.
        if "nvs" in payload:
            text = f"Calibration storage: {payload['nvs']}"
            self._add_activity(text, AMBER)
            self._record_calibration_response(payload)
        elif "cal" in payload:
            text = f"Calibration: {payload['cal']}"
            self._add_activity(text, BLUE)
            self._record_calibration_response(payload)
        elif "err" in payload:
            text = f"Firmware error: {payload['err']}"
            self._add_activity(text, RED)
            self._record_calibration_response(payload)
        else:
            self._add_activity("Status JSON received", MUTED)

    def _handle_text_response(self, line: str) -> None:
        # Plain-text responses are the compact command protocol: READY banners,
        # mode changes, STOPPED, EXITED, and identification responses.
        clean = line.strip()
        response = COMMAND_RESPONSES.get(clean)
        if response is None:
            if clean.startswith("READY:"):
                self._handle_ready_line(clean)
            return
        key, message = response
        if key in ("EASY", "INTERMEDIATE", "HARD"):
            # The GUI distinguishes between:
            # - selecting/arming a mode
            # - starting a live training session in that mode
            #
            # That is why there are "awaiting arm" and "pending live" fields
            # instead of flipping directly into session-active on every mode
            # response.
            self._mode = key
            if self._pending_live_mode == key:
                self._pending_live_mode = None
                self._begin_training_session(key)
            elif self._awaiting_arm_mode is not None and self._awaiting_arm_mode != key:
                self._add_activity(f"Ignored stale {MODE_LABELS[key]} mode response", MUTED)
                return
            else:
                self._awaiting_arm_mode = None
                self._arm_training_mode(key)
                self._add_activity(
                    f"{MODE_LABELS[key]} difficulty selected. Press Start when you're ready.",
                    MODE_COLORS[key],
                )
        elif key == "IDLE":
            was_live = self._session_active
            self._set_state(TrainerState.IDLE)
            self._session_active    = False
            self._awaiting_arm_mode = None
            self._pending_live_mode = None
            self._armed_mode        = "UNSELECTED"
            if self._countdown_timer.isActive():
                self._cancel_countdown("Stopped before the session started.")
            self._set_mode(self._session_mode, active=False, complete=True)
            self._refresh_non_session_banner(
                headline="Session complete" if clean == "STOPPED" and was_live else None
            )
            self._update_start_button()
            self._add_activity("Session stopped" if clean == "STOPPED" else message, MUTED)
            if clean == "STOPPED" and was_live:
                self._show_session_summary()
        elif key == "EXITED":
            self._set_state(TrainerState.EXITED)
            self._armed_mode        = "UNSELECTED"
            self._session_active    = False
            self._awaiting_arm_mode = None
            self._pending_live_mode = None
            self._apply_player_banner("Session ended", "", RED, active=False, pulse=False)
            self._set_commands_enabled(False)
            self._update_start_button()
            self._add_activity("Session ended", RED)
        else:
            self._set_state(TrainerState.CONNECTED)
            self._add_activity(message, BLUE)

    def _state_from_text(self, value: str) -> TrainerState:
        try:
            return TrainerState(value)
        except ValueError:
            return TrainerState.CONNECTED

    def _begin_training_session(self, mode: str) -> None:
        # This is the true session reset point from the GUI's perspective.
        # All plots, counters, and score deltas are cleared here so the next
        # run starts with a clean slate.
        self._session_mode       = mode
        self._mode               = mode
        self._armed_mode         = mode
        self._session_active     = True
        self._last_score         = 100.0
        self._last_warn          = 0
        self._last_err           = 0
        self._last_state         = ""
        self._hud_level          = "stable"
        self._hud_hold_until     = 0.0
        self._hud_calm_since     = None
        self._hud_last_change_at = 0.0
        self._countdown_timer.stop()
        self._countdown_value    = 0
        self._telemetry.reset()
        self._clear_plots()
        self._set_mode(mode, active=True)
        self._score_card.set_value("100.0")
        self._score_card.set_accent(GREEN)
        self._score_inline.setText("Score: 100.0")
        self._force_card.set_value("0.000")
        self._force_card.set_accent(BLUE)
        self._packet_label.setText("Packets: 0")
        self._packet_kpi.set_value("0")
        self._rate_label.setText("-- Hz")
        self._rate_kpi.set_value("--")
        self._contact_label.setText("Contact: --/--/--")
        self._contact_label.setStyleSheet(f"color: {MUTED};")
        self._gate_label.setText("Gate: NONE")
        self._gate_label.setStyleSheet(f"color: {MUTED};")
        self._score_delta_label.setText(f"Score: 100.0")
        self._score_delta_label.setStyleSheet(f"color: {GREEN};")
        self._countdown_label.setText("Session live. Press Stop when finished.")
        self._apply_player_banner("Nice control", "", GREEN, active=True, pulse=True)
        self._update_start_button()
        self._add_activity(
            f"{MODE_LABELS[mode]} session started. Score reset to 100.", BLUE
        )

    def _reset_session_display(self, clear_mode: bool = False) -> None:
        # Used when disconnecting, stopping, or cancelling before a live run
        # truly starts. It clears the GUI-side session artifacts without
        # touching the board directly.
        self._countdown_timer.stop()
        self._countdown_value   = 0
        self._pending_live_mode = None
        self._session_active    = False
        self._last_score        = None
        self._last_warn         = 0
        self._last_err          = 0
        self._last_state        = ""
        self._hud_level         = "idle"
        self._hud_headline      = ""
        self._hud_detail        = ""
        self._hud_color         = MUTED
        self._hud_hold_until    = 0.0
        self._hud_calm_since    = None
        self._hud_last_change_at = 0.0
        self._telemetry.reset()
        self._clear_plots()
        self._score_card.set_value("--")
        self._score_card.set_accent(GRAY)
        self._force_card.set_value("--")
        self._force_card.set_accent(GRAY)
        self._packet_label.setText("Packets: 0")
        self._packet_kpi.set_value("0")
        self._rate_label.setText("-- Hz")
        self._rate_kpi.set_value("--")
        self._contact_label.setText("Contact: --/--/--")
        self._contact_label.setStyleSheet(f"color: {MUTED};")
        self._gate_label.setText("Gate: NONE")
        self._gate_label.setStyleSheet(f"color: {MUTED};")
        self._score_delta_label.setText("Score: --")
        self._score_delta_label.setStyleSheet(f"color: {MUTED};")
        self._refresh_non_session_banner()
        if hasattr(self, "_countdown_label"):
            self._countdown_label.setText("Pick a difficulty, then press Start.")
        for metric in self._metric_values.values():
            metric.set_value("--")
            metric.set_accent(BLUE)
        if clear_mode:
            self._session_mode = "UNSELECTED"
            self._mode         = "UNSELECTED"
            self._armed_mode   = "UNSELECTED"
            self._set_mode("UNSELECTED")
        self._update_start_button()

    def _set_mode(self, mode: str, active: bool = False,
                  complete: bool = False, countdown: bool = False) -> None:
        label = MODE_LABELS.get(mode, mode.title())
        color = MODE_COLORS.get(mode, MUTED)
        if mode == "UNSELECTED":
            text = "No difficulty selected"
        elif complete:
            text = f"Last run: {label}"
        elif active:
            text = f"{label} live"
        elif countdown:
            text = f"{label} starting"
        else:
            text = f"{label} selected"

        self._mode_label.setText(text)
        self._mode_label.setStyleSheet(
            f"color: {color};"
            f"background-color: {BLUE_LIGHT if color != RED else '#fff2f2'};"
            f"border: 1px solid {color};"
            "border-radius: 10px;"
            "padding: 3px 10px;"
            "font-weight: 900; font-size: 11px;"
        )

        if not self._session_active and not countdown:
            self._refresh_non_session_banner()

        # Update mode button visual states
        active_cmd = {"EASY": "E", "INTERMEDIATE": "M", "HARD": "H"}.get(mode)
        for cmd in ("E", "M", "H"):
            button = self._cmd_button_by_cmd.get(cmd)
            if button is None:
                continue
            selected = cmd == active_cmd and not complete
            button.setProperty("activeMode", "true" if selected else "false")
            self._repolish(button)

    def _show_session_summary(self) -> None:
        summary = self._build_session_summary()
        self._session_summary = SessionSummaryDialog(summary, self)
        self._session_summary.show()
        self._session_summary.raise_()
        self._session_summary.activateWindow()
        self._add_activity(
            f"Session complete: score {summary['final_score']:.1f} ({summary['grade']})",
            summary["grade_color"],
        )

    def _build_session_summary(self) -> dict:
        # Summaries are computed from the GUI's telemetry cache, not re-asked
        # from the firmware. That keeps the board protocol simple and makes the
        # summary reproducible from exactly what the user saw live.
        def values(key: str) -> list[float]:
            return self._telemetry.y(key)
        times    = self._telemetry.x()
        samples  = len(times)
        duration_s = times[-1] - times[0] if len(times) >= 2 else 0.0
        scores   = values("score")
        forces   = values("f_sum")
        f0, f1, f2 = values("f0"), values("f1"), values("f2")
        warns    = values("warn")
        errs     = values("err")
        engaged  = values("engaged")
        tremor   = values("tremor")
        f95      = values("f95")
        final_score     = scores[-1] if scores else 0.0
        warning_events  = sum(1 for v in warns if v > 0.0)
        error_events    = sum(1 for v in errs if v > 0.0)
        engaged_pct     = (100.0 * sum(engaged) / len(engaged)) if engaged else 0.0
        peak_finger     = max(f0 + f1 + f2) if (f0 or f1 or f2) else 0.0
        avg_force       = (sum(forces) / len(forces)) if forces else 0.0
        peak_force      = max(forces) if forces else 0.0
        avg_tremor      = (sum(tremor) / len(tremor)) if tremor else 0.0
        peak_f95        = max(f95) if f95 else 0.0
        if final_score >= 90 and error_events == 0:
            grade, grade_color = "Excellent control", GREEN
            coaching = "Strong run. Your grip stayed controlled and the trainer saw no major correction events."
        elif final_score >= 75 and error_events == 0:
            grade, grade_color = "Solid control", BLUE
            coaching = "Good run. Keep smoothing out the warning moments and keep the grip light."
        elif final_score >= 50:
            grade, grade_color = "Building consistency", AMBER
            coaching = "You completed the run. Focus next on steadier hand motion and gentler force changes."
        else:
            grade, grade_color = "Keep practicing", RED
            coaching = "This run was tough. Try an easier difficulty and focus on gentle contact and smoother motion."
        if samples == 0:
            coaching = "No live telemetry was captured for this session. Start a mode, interact with the glove, then press Stop."
        return {
            "mode": self._session_mode, "samples": samples, "duration_s": duration_s,
            "final_score": final_score, "grade": grade, "grade_color": grade_color,
            "avg_force": avg_force, "peak_force": peak_force, "peak_finger": peak_finger,
            "engaged_pct": engaged_pct, "warning_events": warning_events,
            "error_events": error_events, "avg_tremor": avg_tremor, "peak_f95": peak_f95,
            "coaching": coaching,
        }

    def _severity_rank(self, level: str) -> int:
        return {"idle": -1, "stable": 0, "warn": 1, "err": 2}.get(level, -1)

    def _set_banner_accent(self, color: str) -> None:
        if hasattr(self, "_session_banner"):
            self._session_banner.setStyleSheet(
                "QFrame#session_banner {"
                f"background-color: {PANEL};"
                f"border: 1px solid {BORDER};"
                f"border-left: 4px solid {color};"
                "border-radius: 12px;"
                "}"
            )

    def _apply_player_banner(
        self,
        headline: str,
        detail: str,
        color: str,
        *,
        active: bool,
        pulse: bool,
    ) -> None:
        self._state_label.setText(headline)
        self._state_label.setStyleSheet(f"color: {color};")
        self._set_banner_accent(color)
        self._vital_sign.set_status(active, headline, detail, color, pulse)
        self._hud_headline = headline
        self._hud_detail = detail
        self._hud_color = color

    def _refresh_non_session_banner(
        self,
        headline: str | None = None,
        *,
        state: TrainerState | None = None,
        color: str | None = None,
    ) -> None:
        if self._session_active or self._countdown_timer.isActive():
            return
        current_state = state or self._state_from_text(self._last_state or "CONNECTED")
        if not self._connected or current_state == TrainerState.DISCONNECTED:
            headline = headline or "Connect glove"
            banner_color = MUTED
        elif current_state == TrainerState.EXITED:
            headline = headline or "Session ended"
            banner_color = RED
        elif headline == "Session complete":
            banner_color = BLUE
        elif self._mode == "UNSELECTED":
            headline = headline or "Choose difficulty"
            banner_color = BLUE
        else:
            headline = headline or "Ready"
            banner_color = color or MODE_COLORS.get(self._mode, BLUE)
        self._hud_level = "idle"
        self._hud_hold_until = 0.0
        self._hud_calm_since = None
        self._apply_player_banner(headline, "", banner_color, active=False, pulse=False)

    def _coaching_signal(self, warn: int, err: int, state: str) -> tuple[str, str, str, str]:
        motion_warn = warn & ((1 << 0) | (1 << 1) | (1 << 3))
        force_warn = warn & ((1 << 2) | (1 << 4) | (1 << 5) | (1 << 6))
        finger_warn = warn & ((1 << 4) | (1 << 5) | (1 << 6))
        motion_err = err & ((1 << 0) | (1 << 1))
        force_err = err & ((1 << 2) | (1 << 3))

        if err:
            if motion_err and force_err:
                return ("err", "Reset control", "", RED)
            if force_err:
                return ("err", "Too much force", "", RED)
            if motion_err:
                return ("err", "Steady hand", "", RED)
            return ("err", "Reset control", "", RED)

        if warn:
            if motion_warn and force_warn:
                return ("warn", "Reset gently", "", AMBER)
            if finger_warn:
                return ("warn", "Relax fingers", "", AMBER)
            if force_warn:
                return ("warn", "Ease grip", "", AMBER)
            if motion_warn:
                return ("warn", "Steady hand", "", AMBER)
            return ("warn", "Small correction", "", AMBER)

        if state in ("HOLD", "ACTIVE"):
            return ("stable", "Nice control", "", GREEN)
        return ("stable", "Ready", "", GREEN)

    def _apply_sticky_coaching(self, level: str, text: str, detail: str, color: str) -> None:
        now = time.monotonic()
        current_rank = self._severity_rank(self._hud_level)
        target_rank = self._severity_rank(level)

        def commit(target_level: str, target_text: str, target_detail: str, target_color: str) -> None:
            self._hud_level = target_level
            self._hud_hold_until = now + HUD_RECOVERY_DELAY_S if self._severity_rank(target_level) > 0 else 0.0
            self._hud_calm_since = None
            self._hud_last_change_at = now
            self._apply_player_banner(
                target_text,
                target_detail,
                target_color,
                active=self._session_active and self._severity_rank(target_level) >= 0,
                pulse=self._severity_rank(target_level) >= 0,
            )

        if target_rank > current_rank:
            commit(level, text, detail, color)
            return

        if target_rank == current_rank and target_rank > 0:
            if text == self._hud_headline:
                self._hud_hold_until = now + HUD_RECOVERY_DELAY_S
                self._hud_calm_since = None
            elif now - self._hud_last_change_at >= HUD_SAME_LEVEL_SWAP_S:
                commit(level, text, detail, color)
            return

        if 0 < target_rank < current_rank:
            if self._hud_calm_since is None:
                self._hud_calm_since = now
            if now >= self._hud_hold_until and (now - self._hud_calm_since) >= HUD_CALM_STREAK_S:
                commit(level, text, detail, color)
            return

        if target_rank == 0 and current_rank > 0:
            if self._hud_calm_since is None:
                self._hud_calm_since = now
            if now >= self._hud_hold_until and (now - self._hud_calm_since) >= HUD_CALM_STREAK_S:
                commit(level, text, detail, color)
            return

        if target_rank == 0 and (
            self._hud_level != "stable"
            or self._hud_headline != text
            or self._hud_detail != detail
        ):
            commit(level, text, detail, color)

    def _update_metrics(self, packet: dict, scored: bool = True) -> None:
        # This is the main "packet -> visible numbers" adapter. Most widgets do
        # not parse raw JSON directly; they are fed normalized display strings
        # and accent colors here.
        def f(key: str, default: float = 0.0) -> float:
            return float(packet.get(key, default))
        score       = f("score")
        total_force = f("f_sum")
        self._force_card.set_value(f"{total_force:.3f}")
        self._force_card.set_accent(RED if total_force >= 4.0 else AMBER if total_force >= 2.0 else BLUE)
        if scored:
            self._score_card.set_value(f"{score:05.1f}")
            self._score_card.set_accent(GREEN if score >= 80 else AMBER if score >= 50 else RED)
            self._score_inline.setText(f"Score: {score:05.1f}")
            if self._last_score is None:
                self._score_delta_label.setText(f"Score: {score:05.1f}")
                self._score_delta_label.setStyleSheet(f"color: {MUTED};")
            else:
                delta = score - self._last_score
                color = GREEN if delta >= 0 else AMBER
                self._score_delta_label.setText(f"Score: {score:05.1f} ({delta:+.1f})")
                self._score_delta_label.setStyleSheet(f"color: {color};")
            self._last_score = score
        for key in ("f0", "f1", "f2"):
            value = f(key)
            self._metric_values[key].set_value(f"{value:.3f}")
            self._metric_values[key].set_accent(RED if value >= 1.5 else AMBER if value >= 0.8 else BLUE)
        for key in ("roll", "pitch", "yaw"):
            self._metric_values[key].set_value(f"{f(key):.2f}")
        warn   = int(f("warn"))
        err    = int(f("err"))
        tremor = f("tremor")
        self._metric_values["tremor"].set_value(f"{tremor:.3f}")
        self._metric_values["tremor"].set_accent(RED if err & (1 << 1) else AMBER if warn & (1 << 1) else VIOLET)
        self._metric_values["cv_f"].set_value(f"{f('cv_f'):.3f}")
        self._metric_values["swing"].set_value(f"{f('swing'):.2f}")
        self._metric_values["f95"].set_value(f"{f('f95'):.2f}")
        self._metric_values["warn_err"].set_value(f"0x{warn:02X}/0x{err:02X}")
        self._metric_values["warn_err"].set_accent(RED if err else AMBER if warn else GREEN)
        contact = packet.get("contact", [0, 0, 0])
        if isinstance(contact, list) and len(contact) >= 3:
            text = "/".join("ON" if int(v) else "--" for v in contact[:3])
        else:
            text = "--/--/--"
        self._metric_values["contact"].set_value(text)
        self._contact_label.setText(f"Contact: {text}")
        self._contact_label.setStyleSheet(f"color: {GREEN if 'ON' in text else MUTED};")
        gate = str(packet.get("gate", "NONE"))
        gate_color = GREEN if gate == "FSR" else BLUE if gate == "IMU" else MUTED
        self._gate_label.setText(f"Gate: {gate}")
        self._gate_label.setStyleSheet(f"color: {gate_color};")

    def _update_coaching(self, packet: dict) -> None:
        # The firmware decides when a warning/error exists; the GUI's job is to
        # translate that into coaching language and smooth rapid packet-to-packet
        # flips so the banner reads like a training HUD rather than a debug feed.
        warn  = int(packet.get("warn", 0))
        err   = int(packet.get("err", 0))
        state = str(packet.get("state", ""))
        self._alert_label.setText("Telemetry: live")
        self._alert_label.setStyleSheet(f"color: {GREEN};")
        level, text, detail, color = self._coaching_signal(warn, err, state)
        self._apply_sticky_coaching(level, text, detail, color)
        if err and err != self._last_err:
            self._add_activity(text, RED)
        elif warn and warn != self._last_warn:
            self._add_activity(text, AMBER)
        if state and state != self._last_state:
            self._add_activity(
                f"Training state changed to {state}",
                STATE_COLORS.get(self._state_from_text(state), MUTED),
            )
        self._last_warn  = warn
        self._last_err   = err
        self._last_state = state

    def _flag_messages(self, flags: int, labels: dict[int, str] | None = None) -> list[str]:
        labels = labels or WARN_COACH_LABELS
        messages = [label for bit, label in labels.items() if flags & bit]
        return messages or [f"0x{flags:02X}"]

    def _refresh_plots(self) -> None:
        if not self._telemetry.latest:
            return
        x = self._telemetry.x()
        for key, curve in self._curves.items():
            curve.setData(x, self._plot_y(key))
        right = x[-1] if x else 0.0
        left  = max(0.0, right - PLOT_WINDOW_S)
        for plot in self._plot_widgets:
            plot.setXRange(left, max(PLOT_WINDOW_S, right), padding=0.02)

    def _plot_y(self, key: str) -> list[float]:
        values = self._telemetry.y(key)
        if key in ("tremor", "cv_f"):
            return [min(v * 10.0, 20.0) for v in values]
        if key in ("swing", "f95"):
            return [min(v, 20.0) for v in values]
        if key in ("warn", "err"):
            return [100.0 if v else 0.0 for v in values]
        return values

    def _clear_plots(self) -> None:
        for curve in self._curves.values():
            curve.setData([], [])

    def _check_stale(self) -> None:
        self._check_hand_process()
        if not self._connected:
            return
        if self._last_packet_at is None:
            self._stale_label.setText("Telemetry: waiting")
            self._stale_label.setStyleSheet(f"color: {AMBER};")
            return
        age = time.monotonic() - self._last_packet_at
        if age > 2.0:
            self._stale_label.setText(f"Telemetry: stale {age:.1f}s")
            self._stale_label.setStyleSheet(f"color: {RED};")
            if self._session_active:
                self._apply_player_banner(
                    "Waiting",
                    "",
                    RED,
                    active=False,
                    pulse=False,
                )

    def _check_hand_process(self) -> None:
        if self._hand_process is None:
            return
        code = self._hand_process.poll()
        if code is None:
            return
        self._hand_process = None
        self._hand_status.setText("3D window closed")
        self._log(f"3D hand visualizer exited with code {code}", "sys")

    def _open_calibration_wizard(self) -> None:
        if self._calibration_wizard is None:
            self._calibration_wizard = CalibrationWizard(self)
            self._calibration_wizard.command_requested.connect(self._send_command)
        if self._latest_packet is not None:
            self._calibration_wizard.update_live_packet(self._latest_packet)
        self._calibration_wizard.show()
        self._calibration_wizard.raise_()
        self._calibration_wizard.activateWindow()

    def _record_calibration_response(self, data: object) -> None:
        should_forward_to_wizard = False
        if isinstance(data, dict):
            should_forward_to_wizard = any(key in data for key in ("cal", "nvs", "err"))
        else:
            text = str(data).strip()
            should_forward_to_wizard = (
                "Not connected" in text or text.startswith("<partial serial fragment>")
            )

        if should_forward_to_wizard and self._calibration_wizard is not None and self._calibration_wizard.isVisible():
            self._calibration_wizard.record_response(data)
        if hasattr(self, "_cal_status"):
            self._cal_status.setText(self._summarize_calibration_status(data))

    def _summarize_calibration_status(self, data: object) -> str:
        if isinstance(data, dict):
            if "nvs" in data:
                return "Saved calibration erased."
            if "err" in data:
                return f"Calibration error: {str(data['err'])[:32]}"
            cal = str(data.get("cal", ""))
            if not cal:
                return "Calibration update received."
            if cal == "COMPLETE":
                return "Calibration complete and saved." if str(data.get("status", "")) == "PASS" else "Calibration save failed."
            if cal in ("C3_STAGE", "C4_STAGE"):
                stage = str(data.get("stage", ""))
                phase = str(data.get("phase", "live"))
                title = CALIBRATION_STAGE_UI.get(stage, {}).get("title", "Guided motion")
                if phase == "fail":
                    return calibration_reason_text(str(data.get("reason", "")), f"{title} needs another try.")
                if phase == "pass":
                    return f"{title} captured."
                if phase == "prompt":
                    return f"Prepare: {title}."
                if phase == "capturing":
                    return f"Capturing {title}…"
                return f"Capturing {title}…"
            status = str(data.get("status", ""))
            if cal.endswith("_START"):
                base = cal.split("_")[0]
                return f"Running {CALIBRATION_STEPS.get(base, {}).get('title', base)}…"
            if cal.endswith("_LIVE"):
                base = cal.split("_")[0]
                return f"{CALIBRATION_STEPS.get(base, {}).get('title', base)} is in progress…"
            title = CALIBRATION_STEPS.get(cal, {}).get("title", cal)
            if status in ("PASS", "COMPLETE"):
                return f"{title} complete."
            if status == "FAIL":
                return calibration_reason_text(str(data.get("reason", "")), f"{title} needs another try.")
            return f"{title} updated."
        text = str(data).strip()
        if not text:
            return "No calibration yet."
        if "Not connected" in text:
            return "Connect the glove before calibrating."
        if text.startswith("<partial serial fragment>"):
            return "Serial fragment received during calibration."
        return text[:56]

    def _handle_ready_line(self, line: str) -> None:
        self._set_state(TrainerState.CONNECTED)
        details = line[len("READY:"):].strip() if line.startswith("READY:") else line
        restart_during_calibration = (
            self._calibration_wizard is not None
            and self._calibration_wizard.isVisible()
        )
        if details:
            self._firmware_info = f"Firmware: {details}"
            if hasattr(self, "_fw_info_label"):
                self._fw_info_label.setText(self._firmware_info)
            self._add_activity(f"Firmware ready ({details})", BLUE)
        else:
            self._firmware_info = "Firmware: ready"
            if hasattr(self, "_fw_info_label"):
                self._fw_info_label.setText(self._firmware_info)
            self._add_activity("Firmware ready", BLUE)
        if restart_during_calibration and self._calibration_wizard is not None:
            interrupted = self._calibration_wizard.mark_transport_restart(details)
            if interrupted and hasattr(self, "_cal_status"):
                self._cal_status.setText("Calibration interrupted by glove reboot.")

    def _open_hand_window(self) -> None:
        if self._hand_process is not None and self._hand_process.poll() is None:
            self._hand_status.setText("3D hand view running")
            if self._latest_packet is not None:
                self._send_to_hand_visualizer(self._latest_packet)
            return
        script = Path(__file__).with_name("hand_visualizer.py")
        try:
            self._hand_process = subprocess.Popen(
                [sys.executable, str(script)],
                stdin=subprocess.PIPE, text=True, cwd=str(script.parent),
            )
        except OSError as exc:
            message = f"Could not open 3D hand view: {exc}"
            self._hand_status.setText(message)
            self._add_activity(message, RED)
            self._log(message, "err")
            return
        self._hand_status.setText("3D hand view running")
        self._add_activity("3D hand visualizer opened", BLUE)
        self._log(f"Started {script}", "sys")
        if self._latest_packet is not None:
            self._send_to_hand_visualizer(self._latest_packet)

    def _send_to_hand_visualizer(self, packet: dict) -> None:
        proc = self._hand_process
        if proc is None:
            return
        if proc.poll() is not None:
            self._hand_process = None
            self._hand_status.setText("3D window closed")
            return
        if proc.stdin is None:
            return
        try:
            proc.stdin.write(json.dumps(packet, separators=(",", ":")) + "\n")
            proc.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            self._log(f"3D hand stream closed: {exc}", "sys")
            self._hand_process = None
            self._hand_status.setText("3D window closed")

    @Slot(str)
    def _on_error(self, msg: str) -> None:
        self._log(f"Serial error: {msg}", "err")
        self._add_activity(f"Serial error: {msg[:52]}", RED)
        self._disconnect()

    @Slot()
    def _on_worker_finished(self) -> None:
        if self._connected:
            self._log("Connection lost", "err")
            self._add_activity("Connection lost", RED)
            self._disconnect()

    def _send_command(self, cmd: str) -> None:
        # Commands are always queued, never written directly from the GUI
        # thread. That keeps button clicks responsive even if the serial driver
        # stalls or the USB link hiccups.
        if not self._connected:
            self._add_activity("Connect to the glove before sending commands.", AMBER)
            if self._calibration_wizard is not None and self._calibration_wizard.isVisible():
                self._calibration_wizard.record_response("Not connected. Command was not sent.")
            return
        if cmd in ("S", "X") and self._countdown_timer.isActive():
            self._cancel_countdown("Stopped before the session started.")
        if cmd in ("S", "X"):
            self._awaiting_arm_mode = None
            self._pending_live_mode = None
        self._cmd_queue.put(cmd.encode("ascii"))
        self._log(cmd, "tx")
        self._add_activity(f"Command sent: {cmd}", BLUE)

    def _set_state(self, state: TrainerState) -> None:
        # This updates only the host-side visual state; it does not command the
        # board. The board remains the source of truth for runtime state.
        color = STATE_COLORS[state]
        if state == TrainerState.EXITED:
            self._set_commands_enabled(False)
        if not self._session_active and not self._countdown_timer.isActive():
            self._refresh_non_session_banner(state=state, color=color)

    def _set_commands_enabled(self, enabled: bool) -> None:
        for button in getattr(self, "_cmd_buttons", []):
            button.setEnabled(enabled)
        self._update_start_button()

    def _add_activity(self, message: str, color: str) -> None:
        """
        # DESIGN: Activity feed cards use a glyph icon (●/▲/✕/✓) left of the
        # message. Icon glyph chosen over an SVG icon widget — lower overhead, no
        # resource dependency, and readable at 14px in the existing QLabel flow.
        # Timestamp is 10px IBM Plex Mono MUTED — clearly secondary to the 14px
        # semibold message text without needing a separate label color.
        """
        ts = datetime.now().strftime("%H:%M:%S")
        icon_map = {
            BLUE:   "●",
            GREEN:  "✓",
            AMBER:  "▲",
            RED:    "✕",
            MUTED:  "–",
        }
        icon = icon_map.get(color, "●")

        card = QFrame()
        card.setObjectName("activity_card")
        card.setStyleSheet(
            "QFrame#activity_card {"
            f"background-color: {PANEL_ALT};"
            f"border: 1px solid {BORDER};"
            f"border-left: 4px solid {color};"
            "border-radius: 8px;"
            "}"
        )
        row_layout = QHBoxLayout(card)
        row_layout.setContentsMargins(8, 8, 10, 8)
        row_layout.setSpacing(8)

        icon_lbl = QLabel(icon)
        icon_lbl.setStyleSheet(f"color: {color}; font-size: 14px; border: none; background: transparent;")
        icon_lbl.setFixedWidth(14)

        body = QVBoxLayout()
        body.setSpacing(2)
        msg = QLabel(message)
        msg.setWordWrap(True)
        msg.setStyleSheet(
            f"color: {TEXT}; font-weight: 600; font-size: 13px;"
            "border: none; background: transparent;"
        )
        stamp = QLabel(ts)
        stamp.setObjectName("timestamp")
        body.addWidget(msg)
        body.addWidget(stamp)

        row_layout.addWidget(icon_lbl, 0, Qt.AlignmentFlag.AlignTop)
        row_layout.addLayout(body, 1)

        self._feed_layout.insertWidget(0, card)
        while self._feed_layout.count() > MAX_FEED_CARDS + 1:
            item = self._feed_layout.takeAt(self._feed_layout.count() - 2)
            if item and item.widget():
                item.widget().deleteLater()

    def _log(self, text: str, direction: str) -> None:
        ts    = datetime.now().strftime("%H:%M:%S")
        tag = {"tx": "TX", "rx": "RX", "err": "ERR", "sys": "SYS"}.get(direction, "SYS")
        visible = text
        if not visible or not any(ch.isprintable() for ch in visible):
            visible = "<non-printable serial bytes>"
        elif any((not ch.isprintable()) and ch not in "\t" for ch in visible):
            visible = visible.encode("unicode_escape", errors="replace").decode("ascii")
        self._log_view.appendPlainText(f"[{ts}] {tag} {visible}")
        bar = self._log_view.verticalScrollBar()
        bar.setValue(bar.maximum())

    def _repolish(self, widget: QWidget) -> None:
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def closeEvent(self, event) -> None:
        self._countdown_timer.stop()
        if self._worker:
            self._worker.stop()
            self._worker.wait()
        if self._hand_process is not None and self._hand_process.poll() is None:
            try:
                if self._hand_process.stdin is not None:
                    self._hand_process.stdin.close()
            except OSError:
                pass
            self._hand_process.terminate()
        super().closeEvent(event)


# ── STYLESHEET ────────────────────────────────────────────────────────────────

STYLESHEET = f"""
/*
DESIGN REGISTRY — new objectNames introduced in this redesign:
  app_bar           — fixed 64px top application bar (Zone 1)
  app_title         — "Haptic Surgical Skill Trainer" bold NAVY label in bar
  app_subtitle      — "Clinical Training Console" muted subtitle in bar
  app_icon          — fixed 36px navy rounded-square logo using the scissors PNG
  status_pill       — connection state badge (DISCONNECTED/CONNECTED) in bar
  rate_chip         — packet rate monospace label in bar
  port_combo        — port selector QComboBox in bar
  tab_pill          — pill-style navigation tab buttons in the left content column
  tab_active=true   — selected state for tab_pill (via QSS property selector)
  session_banner    — full-width rounded card at top of Train tab
  banner_stat       — small stat labels (Packets, Contact, Telemetry) in banner
  score_inline      — inline Score: XX.X label in banner meta row
  kpi_card          — summary metric card with left-border accent (replaces summary_card in Train)
  kpi_label         — all-caps MUTED label in kpi_card
  kpi_value         — large monospace value in kpi_card
  kpi_unit          — small muted unit label in kpi_card
  chart_card        — outer white container for the force profile chart
  chart_title       — "Force Profile" label in chart header
  chart_sep         — 1px QFrame separator between chart header and plot
  chart_tab_chip    — small outlined chip buttons above the chart (Force/Motion/Skill/Score)
  chip_active=true  — selected state for chart_tab_chip
  metrics_scroll    — QScrollArea containing the 3-column metrics grid (Metrics tab)
  review_scroll     — QScrollArea containing the activity feed (Review tab)
  sidebar_card      — right sidebar white card (~300px wide)
  sidebar_scroll    — QScrollArea for the sidebar interior
  sidebar_sep       — 1px horizontal rule between sidebar sections
  section_header    — all-caps MUTED 9px section label with bottom border
  mode_easy         — Easy mode toggle button (green selected state)
  mode_intermediate — Intermediate mode toggle button (blue selected state)
  mode_hard         — Hard mode toggle button (red selected state)
  start_button      — large 48px primary Start action button (GREEN filled)
  outline_button    — secondary outlined button (Calibrate, Open 3D)
  exit_strip        — pinned bottom strip containing the Exit Session button
  danger_button     — Exit Session button (RED outlined)
  left_column       — left 70% content area widget
  train_container   — Train tab content wrapper
  countdown_label   — hint text below Start button
*/

/* ── Base ──────────────────────────────────────────────────────────────── */
QMainWindow, QWidget#central, QWidget#left_column, QWidget#train_container {{
    background-color: {ICE};
}}

/* ── App Bar ───────────────────────────────────────────────────────────── */
QFrame#app_bar {{
    background-color: {PANEL};
    border-bottom: 1px solid {BORDER};
    border-left: 3px solid {NAVY};
}}
QLabel#app_title {{
    color: {NAVY};
    font-size: 16px;
    font-weight: 900;
}}
QLabel#app_subtitle {{
    color: {TEXT};
    font-size: 10px;
    font-weight: 600;
}}
QLabel#rate_chip {{
    color: {MUTED};
    background-color: {PANEL_ALT};
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 3px 8px;
    font-family: Menlo, Consolas, monospace;
    font-size: 11px;
}}
QComboBox#port_combo {{
    background-color: {PANEL};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 5px 8px;
    font-family: Menlo, Consolas, monospace;
    font-size: 12px;
}}
QComboBox#port_combo QAbstractItemView {{
    background-color: {PANEL};
    color: {TEXT};
    selection-background-color: {BLUE_LIGHT};
}}

/* ── Pill Tab Bar ──────────────────────────────────────────────────────── */
QPushButton#tab_pill {{
    background-color: transparent;
    color: {MUTED};
    border: none;
    border-radius: 20px;
    padding: 7px 20px;
    font-size: 12px;
    font-weight: 700;
}}
QPushButton#tab_pill:hover {{
    background-color: {PANEL_ALT};
    color: {NAVY};
}}
QPushButton#tab_pill[tab_active="true"] {{
    background-color: {BLUE_LIGHT};
    color: {NAVY};
}}

/* ── Session Banner ────────────────────────────────────────────────────── */
QFrame#session_banner {{
    background-color: {PANEL};
    border: 1px solid {BORDER};
    border-left: 4px solid {MUTED};
    border-radius: 12px;
}}
QLabel#state_label {{
    font-size: 26px;
    font-weight: 800;
    color: {NAVY};
}}
QLabel#banner_stat {{
    color: {MUTED};
    font-size: 11px;
    font-weight: 600;
}}
QLabel#vital_header {{
    color: {MUTED};
    font-size: 9px;
    font-weight: 800;
    letter-spacing: 0.08em;
    text-transform: uppercase;
}}
QLabel#score_inline {{
    color: {MUTED};
    font-size: 13px;
    font-family: Menlo, Consolas, monospace;
    font-weight: 700;
}}

/* ── KPI Cards ─────────────────────────────────────────────────────────── */
QFrame#kpi_card {{
    background-color: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 12px;
}}
QLabel#kpi_label {{
    color: {MUTED};
    font-size: 9px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.08em;
}}
QLabel#kpi_value {{
    color: {NAVY};
    font-family: Menlo, Consolas, monospace;
    font-size: 22px;
    font-weight: 800;
}}
QLabel#kpi_unit {{
    color: {MUTED};
    font-size: 10px;
}}

/* ── Chart Container ───────────────────────────────────────────────────── */
QFrame#chart_card {{
    background-color: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 12px;
}}
QLabel#chart_title {{
    color: {NAVY};
    font-size: 13px;
    font-weight: 700;
}}
QFrame#chart_sep {{
    color: {BORDER};
    background-color: {BORDER};
    border: none;
    max-height: 1px;
}}
QPushButton#chart_tab_chip {{
    background-color: transparent;
    color: {MUTED};
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 4px 10px;
    font-size: 11px;
    font-weight: 600;
}}
QPushButton#chart_tab_chip:hover {{
    background-color: {PANEL_ALT};
    color: {NAVY};
}}
QPushButton#chart_tab_chip[chip_active="true"] {{
    background-color: {NAVY};
    color: {PANEL};
    border-color: {NAVY};
}}

/* ── Sidebar Card ──────────────────────────────────────────────────────── */
QFrame#sidebar_card {{
    background-color: {PANEL};
    border-left: 1px solid {BORDER};
    border-radius: 0;
}}
QScrollArea#sidebar_scroll {{
    border: none;
    background: transparent;
}}
QFrame#sidebar_sep {{
    color: {BORDER};
    background-color: {BORDER};
    border: none;
    max-height: 1px;
    margin: 2px 0;
}}
QFrame#exit_strip {{
    background-color: {PANEL};
    border-top: 1px solid {BORDER};
    border-radius: 0;
}}

/* ── Section Headers ───────────────────────────────────────────────────── */
QLabel#section_header {{
    color: {MUTED};
    font-size: 9px;
    font-weight: 800;
    letter-spacing: 0.10em;
    text-transform: uppercase;
    border-bottom: 1px solid {BORDER};
    padding-bottom: 5px;
}}

/* ── Mode Buttons ──────────────────────────────────────────────────────── */
QPushButton#mode_easy, QPushButton#mode_intermediate, QPushButton#mode_hard {{
    background-color: {PANEL};
    color: {MUTED};
    border: 1.5px solid {BORDER};
    border-radius: 8px;
    padding: 9px 14px;
    font-size: 12px;
    font-weight: 700;
    text-align: left;
}}
QPushButton#mode_easy:hover,
QPushButton#mode_intermediate:hover,
QPushButton#mode_hard:hover {{
    background-color: {PANEL_ALT};
    color: {TEXT};
}}
QPushButton#mode_easy:disabled,
QPushButton#mode_intermediate:disabled,
QPushButton#mode_hard:disabled {{
    color: {GRAY};
    background-color: #f6f8fa;
    border-color: #dde6ef;
}}
QPushButton#mode_easy[activeMode="true"] {{
    background-color: {GREEN};
    color: {PANEL};
    border-color: #177a42;
}}
QPushButton#mode_intermediate[activeMode="true"] {{
    background-color: {BLUE};
    color: {PANEL};
    border-color: {BLUE_DARK};
}}
QPushButton#mode_hard[activeMode="true"] {{
    background-color: {RED};
    color: {PANEL};
    border-color: #a02020;
}}

/* ── Start Button ──────────────────────────────────────────────────────── */
QPushButton#start_button {{
    background-color: {GREEN};
    color: {PANEL};
    border: none;
    border-radius: 12px;
    padding: 13px 14px;
    min-height: 48px;
    font-size: 14px;
    font-weight: 800;
    letter-spacing: 0.01em;
}}
QPushButton#start_button:hover {{
    background-color: #177a42;
}}
QPushButton#start_button:disabled {{
    background-color: #c8d8cc;
    color: #8aab95;
}}

/* ── Outline Buttons ───────────────────────────────────────────────────── */
QPushButton#outline_button {{
    background-color: {PANEL};
    color: {BLUE};
    border: 1.5px solid {BLUE};
    border-radius: 8px;
    padding: 9px 14px;
    font-size: 12px;
    font-weight: 700;
}}
QPushButton#outline_button:hover {{
    background-color: {BLUE_LIGHT};
}}
QPushButton#outline_button:disabled {{
    color: {GRAY};
    border-color: {BORDER};
    background-color: #f6f8fa;
}}

/* ── Danger Button ─────────────────────────────────────────────────────── */
QPushButton#danger_button {{
    background-color: #fff8f8;
    color: {RED};
    border: 1.5px solid #efb4b4;
    border-radius: 8px;
    padding: 9px 14px;
    font-size: 12px;
    font-weight: 700;
    width: 100%;
}}
QPushButton#danger_button:hover {{
    background-color: #fee;
    border-color: {RED};
}}

/* ── Generic Command Button ────────────────────────────────────────────── */
QPushButton#command_button {{
    background-color: {PANEL};
    color: {NAVY};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 8px 10px;
    min-height: 30px;
    font-weight: 700;
    font-size: 12px;
}}
QPushButton#command_button:hover {{
    background-color: {BLUE_LIGHT};
    border-color: {BLUE};
}}
QPushButton#command_button:disabled {{
    color: {GRAY};
    background-color: #f6f8fa;
    border-color: #dde6ef;
}}

/* ── Connect Button ────────────────────────────────────────────────────── */
QPushButton#connect_button {{
    background-color: {BLUE};
    color: {PANEL};
    border: 1px solid {BLUE_DARK};
    border-radius: 8px;
    padding: 6px 14px;
    font-weight: 700;
    font-size: 12px;
    min-width: 90px;
}}
QPushButton#connect_button:hover {{
    background-color: {BLUE_DARK};
}}
QPushButton#connect_button[connected="true"] {{
    background-color: {GREEN};
    border-color: #177a42;
}}
QPushButton#connect_button[connected="true"]:hover {{
    background-color: #177a42;
}}

/* ── Generic QPushButton fallback ──────────────────────────────────────── */
QPushButton {{
    background-color: {PANEL};
    color: {NAVY};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 7px 12px;
    min-height: 26px;
    font-weight: 700;
    font-size: 12px;
}}
QPushButton:hover {{
    background-color: {BLUE_LIGHT};
    border-color: {BLUE};
}}
QPushButton:pressed {{
    background-color: #d7ebff;
}}
QPushButton:disabled {{
    color: {GRAY};
    background-color: #f4f7fa;
    border-color: #dde6ef;
}}
QPushButton[selected="true"] {{
    background-color: {BLUE_LIGHT};
    border-color: {BLUE};
    color: {NAVY};
}}
QPushButton#terminal_button {{
    color: {MUTED};
    background: transparent;
    border: none;
    font-size: 12px;
}}

/* ── Labels ────────────────────────────────────────────────────────────── */
QLabel {{
    color: {TEXT};
    font-size: 13px;
}}
QLabel#muted_label, QLabel#countdown_label, QLabel#timestamp {{
    color: {MUTED};
    font-size: 11px;
}}
QLabel#countdown_label {{
    font-style: italic;
}}
QLabel#timestamp {{
    font-family: Menlo, Consolas, monospace;
    font-size: 10px;
}}
QLabel#panel_title {{
    color: {NAVY};
    font-size: 15px;
    font-weight: 800;
}}
QLabel#dialog_title {{
    color: {NAVY};
    font-size: 21px;
    font-weight: 900;
}}
QLabel#mode_badge {{
    font-size: 11px;
    font-weight: 900;
    border-radius: 10px;
    padding: 3px 10px;
}}

/* ── Metric Cards (ClinicalValue) ──────────────────────────────────────── */
QFrame#clinical_value {{
    background-color: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QLabel#metric_title {{
    color: {MUTED};
    font-size: 9px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.07em;
}}
QLabel#metric_value {{
    color: {TEXT};
    font-family: Menlo, Consolas, monospace;
    font-size: 16px;
    font-weight: 800;
}}
QLabel#metric_unit {{
    color: {MUTED};
    font-size: 10px;
}}

/* ── Summary Dialog ────────────────────────────────────────────────────── */
QFrame#summary_card, QFrame#dialog_panel {{
    background-color: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QLabel#summary_score_label {{
    color: {NAVY};
    font-family: Menlo, Consolas, monospace;
    font-size: 48px;
    font-weight: 900;
}}
QLabel#summary_grade_label {{
    font-size: 18px;
    font-weight: 900;
}}
QLabel#summary_title {{
    color: {MUTED};
    font-size: 10px;
    font-weight: 800;
    text-transform: uppercase;
}}
QLabel#summary_value {{
    color: {NAVY};
    font-family: Menlo, Consolas, monospace;
    font-size: 28px;
    font-weight: 900;
}}
QLabel#summary_unit {{
    color: {MUTED};
    font-size: 10px;
}}

/* ── Activity Feed ─────────────────────────────────────────────────────── */
QFrame#activity_card {{
    background-color: {PANEL_ALT};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QScrollArea#review_scroll, QScrollArea#metrics_scroll {{
    border: none;
    background: transparent;
}}

/* ── Engineering Log ───────────────────────────────────────────────────── */
QPlainTextEdit#log {{
    font-family: Menlo, Consolas, monospace;
    font-size: 12px;
    background-color: {NAVY};
    color: {ICE};
    border: none;
}}

/* ── Dialogs / Calibration ─────────────────────────────────────────────── */
QLabel#instruction_text {{
    color: {TEXT};
    font-size: 13px;
    line-height: 150%;
}}
QLabel#command_chip {{
    color: {NAVY};
    background-color: {BLUE_LIGHT};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 6px 8px;
    font-size: 12px;
    font-weight: 700;
}}
QLabel#panel_caption {{
    color: {MUTED};
    font-size: 12px;
}}
QLabel#info_mode {{
    font-size: 14px;
    font-weight: 900;
}}
QLabel#info_body {{
    color: {TEXT};
    font-size: 12px;
}}
QFrame#info_card {{
    background-color: {PANEL_ALT};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QProgressBar#calibration_progress {{
    min-height: 10px;
    max-height: 10px;
    border: 1px solid {BORDER};
    border-radius: 5px;
    background-color: #dfeaf7;
}}
QProgressBar#calibration_progress::chunk {{
    background-color: {BLUE};
    border-radius: 5px;
}}

/* ── QComboBox ─────────────────────────────────────────────────────────── */
QComboBox {{
    background-color: {PANEL};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 7px;
}}
QComboBox QAbstractItemView {{
    background-color: {PANEL};
    color: {TEXT};
    selection-background-color: {BLUE_LIGHT};
}}
"""


if __name__ == "__main__":
    app = QApplication(sys.argv)
    # DESIGN: IBM Plex Sans chosen over Arial — same neutral sans-serif role but
    # with a slightly technical character appropriate for an instrument interface.
    # Falls back to the system sans-serif if IBM Plex Sans is not installed.
    app.setFont(QFont("IBM Plex Sans", 12))
    app.setStyleSheet(STYLESHEET)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
