"""
Haptic Surgical Skill Trainer

Clinical desktop dashboard for the ESP32 glove firmware. The GUI keeps the
existing UART command protocol and turns live JSON telemetry into force,
motion, coaching, calibration, and optional 3D orientation/contact views.
"""

from __future__ import annotations

import html
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
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTextEdit,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


BAUD_RATE = 115200
HISTORY_POINTS = 900
PLOT_WINDOW_S = 30.0
MAX_FEED_CARDS = 18

NAVY = "#0f2d52"
BLUE = "#1e88e5"
BLUE_DARK = "#155fa0"
BLUE_LIGHT = "#e8f3ff"
ICE = "#f6fbff"
PANEL = "#ffffff"
PANEL_ALT = "#f0f7ff"
BORDER = "#cfe1f5"
GRID = "#d9e9f8"
TEXT = "#10263f"
MUTED = "#63758a"
GREEN = "#1f9d55"
AMBER = "#d98a00"
RED = "#c62828"
VIOLET = "#6f5bd6"
GRAY = "#9aaec3"

WARN_LABELS = {
    1 << 0: "Hold instability",
    1 << 1: "Tremor",
    1 << 2: "Smoothness",
    1 << 3: "Swing rate",
    1 << 4: "Force opening",
    1 << 5: "Force variability",
    1 << 6: "Force spike",
}

ERR_LABELS = {
    1 << 0: "Hold instability",
    1 << 1: "Tremor",
    1 << 2: "Force opening",
    1 << 3: "Sustained compression",
}

CALIBRATION_STEPS = {
    "C1": {
        "title": "C1 Baseline",
        "instruction": (
            "Keep the glove relaxed with no finger pressure. Keep the hand still "
            "on the table. Send C1 once the force sensors read near zero."
        ),
    },
    "C2": {
        "title": "C2 Force Reference",
        "instruction": (
            "Apply a steady reference grip using the training posture. Hold the "
            "pressure constant while sending C2."
        ),
    },
    "C3": {
        "title": "C3 Stable Hold",
        "instruction": (
            "Hold the glove in the intended neutral surgical posture. Avoid "
            "shaking or changing grip pressure while sending C3."
        ),
    },
    "C4": {
        "title": "C4 Controlled Motion",
        "instruction": (
            "Perform a smooth controlled motion with the glove. Keep the force "
            "reasonable and avoid abrupt jerks while sending C4."
        ),
    },
    "C3_REP": {
        "title": "Repeat C3 Hold",
        "instruction": (
            "Repeat the stable hold capture. Use this if the previous hold sample "
            "looked unstable or the subject moved too early."
        ),
    },
    "C4_CYCLE": {
        "title": "Cycle C4 Motion",
        "instruction": (
            "Run another controlled motion cycle. Use the same motion pattern as "
            "the prior C4 sample so the reference remains consistent."
        ),
    },
    "Z": {
        "title": "Erase Stored Calibration",
        "instruction": (
            "This clears saved calibration values from NVS. Use only when starting "
            "over with a new glove setup or bad calibration data."
        ),
    },
}


class TrainerState(Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    IDLE = "IDLE"
    HOLD = "HOLD"
    ACTIVE = "ACTIVE"
    EXITED = "EXITED"


STATE_COLORS = {
    TrainerState.DISCONNECTED: GRAY,
    TrainerState.CONNECTED: BLUE,
    TrainerState.IDLE: MUTED,
    TrainerState.HOLD: GREEN,
    TrainerState.ACTIVE: AMBER,
    TrainerState.EXITED: RED,
}

COMMAND_RESPONSES = {
    "ESP32_TRAINER": ("CONNECTED", "Board identified"),
    "EASY": ("EASY", "Easy training mode"),
    "INTERMEDIATE": ("INTERMEDIATE", "Intermediate training mode"),
    "HARD": ("HARD", "Hard training mode"),
    "STOPPED": ("IDLE", "Training stopped"),
    "EXITED": ("EXITED", "Session exited"),
}

MODE_LABELS = {
    "EASY": "Easy",
    "INTERMEDIATE": "Intermediate",
    "HARD": "Hard",
    "UNSELECTED": "No Mode",
}

MODE_COLORS = {
    "EASY": GREEN,
    "INTERMEDIATE": BLUE,
    "HARD": RED,
    "UNSELECTED": MUTED,
}

MODE_BY_COMMAND = {
    "E": "EASY",
    "M": "INTERMEDIATE",
    "H": "HARD",
}

COMMAND_BY_MODE = {mode: cmd for cmd, mode in MODE_BY_COMMAND.items()}


@dataclass
class TelemetryStore:
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
            "score", "warn", "err", "contact_any",
        )
        self.series = {key: deque(maxlen=self.maxlen) for key in keys}

    def reset(self) -> None:
        self.packet_count = 0
        self.start_ms = None
        self.latest = {}
        for values in self.series.values():
            values.clear()

    def append(self, packet: dict) -> None:
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
            if key == "contact_any":
                contact = packet.get("contact", [0, 0, 0])
                if isinstance(contact, list) and len(contact) >= 3:
                    value = 1.0 if any(int(v) for v in contact[:3]) else 0.0
                else:
                    value = 1.0 if float(packet.get("f_sum", 0.0)) > 0.05 else 0.0
                self.series[key].append(value)
                continue
            self.series[key].append(float(packet.get(key, 0.0)))

    def x(self) -> list[float]:
        return list(self.series["t"])

    def y(self, key: str) -> list[float]:
        return list(self.series[key])


class SerialWorker(QThread):
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
        try:
            while self._running:
                while not self._cmd_queue.empty():
                    try:
                        ser.write(self._cmd_queue.get_nowait())
                    except queue.Empty:
                        break
                raw = ser.readline()
                if raw:
                    line = raw.decode("ascii", errors="replace").strip(" \t\r\n\0")
                    if line and any(ch.isprintable() for ch in line):
                        self.data_received.emit(line)
        except serial.SerialException as exc:
            self.error_occurred.emit(str(exc))
        finally:
            ser.close()

    def stop(self) -> None:
        self._running = False


class ClinicalValue(QFrame):
    def __init__(self, title: str, unit: str = "", accent: str = BLUE) -> None:
        super().__init__()
        self.setObjectName("clinical_value")
        self.setMinimumHeight(42)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 5, 8, 5)
        layout.setSpacing(0)

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


class VitalSignWidget(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("vital_sign")
        self.setMinimumHeight(82)
        self._active = False
        self._phase = 0.0
        self._color = MUTED
        self._status = "Session idle"
        self._detail = "Select a mode and press Start"
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)

    def set_status(self, active: bool, warn: int = 0, err: int = 0) -> None:
        self._active = active
        if not active:
            self._color = MUTED
            self._status = "Session idle"
            self._detail = "Vital sign starts with live scoring"
            self._timer.stop()
            self._phase = 0.0
        elif err:
            self._color = RED
            self._status = "Danger threshold"
            self._detail = "Error flag active"
            if not self._timer.isActive():
                self._timer.start(70)
        elif warn:
            self._color = AMBER
            self._status = "Warning threshold"
            self._detail = "Technique nearing limit"
            if not self._timer.isActive():
                self._timer.start(70)
        else:
            self._color = GREEN
            self._status = "Stable technique"
            self._detail = "No active thresholds"
            if not self._timer.isActive():
                self._timer.start(70)
        self.update()

    def set_countdown(self, text: str) -> None:
        self._active = False
        self._color = BLUE
        self._status = f"Starting in {text}" if text != "Begin" else "Begin"
        self._detail = "Get ready"
        self._timer.stop()
        self._phase = 0.0
        self.update()

    def _tick(self) -> None:
        self._phase = (self._phase + 0.22) % (math.pi * 2.0)
        self.update()

    def paintEvent(self, event) -> None:
        _ = event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        rect = QRectF(self.rect()).adjusted(1.0, 1.0, -1.0, -1.0)
        painter.setPen(QPen(QColor(BORDER), 1.0))
        painter.setBrush(QColor(PANEL))
        painter.drawRoundedRect(rect, 8.0, 8.0)

        painter.setPen(QColor(MUTED))
        small = QFont(self.font())
        small.setPointSize(9)
        small.setBold(True)
        painter.setFont(small)
        painter.drawText(QPointF(14, 18), "LIVE VITAL")

        painter.setPen(QColor(self._color))
        title = QFont(self.font())
        title.setPointSize(14)
        title.setBold(True)
        painter.setFont(title)
        painter.drawText(QPointF(14, 39), self._status)

        painter.setPen(QColor(MUTED))
        detail = QFont(self.font())
        detail.setPointSize(9)
        painter.setFont(detail)
        painter.drawText(QPointF(14, 57), self._detail)

        left = 126.0
        right = max(left + 24.0, rect.right() - 12.0)
        base = rect.center().y() + 8.0
        amp = 16.0 if self._active else 4.0
        path = QPainterPath()
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

        painter.setPen(QPen(QColor(self._color), 2.4))
        painter.drawPath(path)



class CalibrationWizard(QDialog):
    command_requested = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Guided Calibration")
        self.setMinimumSize(QSize(560, 430))
        self._selected = "C1"

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(12)

        title = QLabel("Guided Calibration")
        title.setObjectName("dialog_title")
        subtitle = QLabel("Follow each instruction before sending the calibration command.")
        subtitle.setObjectName("muted_label")
        layout.addWidget(title)
        layout.addWidget(subtitle)

        body = QHBoxLayout()
        body.setSpacing(12)

        step_col = QVBoxLayout()
        step_col.setSpacing(7)
        self._step_buttons: dict[str, QPushButton] = {}
        for cmd, data in CALIBRATION_STEPS.items():
            btn = QPushButton(data["title"])
            btn.clicked.connect(lambda _checked=False, value=cmd: self._select_step(value))
            self._step_buttons[cmd] = btn
            step_col.addWidget(btn)
        step_col.addStretch()
        body.addLayout(step_col, 1)

        info = QFrame()
        info.setObjectName("dialog_panel")
        info_layout = QVBoxLayout(info)
        info_layout.setContentsMargins(14, 12, 14, 12)
        info_layout.setSpacing(10)
        self._step_title = QLabel("")
        self._step_title.setObjectName("panel_title")
        self._instruction = QLabel("")
        self._instruction.setWordWrap(True)
        self._instruction.setObjectName("instruction_text")
        self._command_label = QLabel("")
        self._command_label.setObjectName("command_chip")
        self._status = QLabel("No calibration command sent yet.")
        self._status.setWordWrap(True)
        self._status.setObjectName("muted_label")
        self._send_btn = QPushButton("Send selected step")
        self._send_btn.clicked.connect(self._send_selected)
        info_layout.addWidget(self._step_title)
        info_layout.addWidget(self._instruction)
        info_layout.addWidget(self._command_label)
        info_layout.addStretch()
        info_layout.addWidget(self._status)
        info_layout.addWidget(self._send_btn)
        body.addWidget(info, 2)

        layout.addLayout(body)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn, alignment=Qt.AlignmentFlag.AlignRight)
        self._select_step("C1")

    def _select_step(self, cmd: str) -> None:
        self._selected = cmd
        data = CALIBRATION_STEPS[cmd]
        self._step_title.setText(data["title"])
        self._instruction.setText(data["instruction"])
        self._command_label.setText(f"Command to send: {cmd}")
        self._send_btn.setText("Erase calibration" if cmd == "Z" else f"Send {cmd}")
        for key, btn in self._step_buttons.items():
            btn.setProperty("selected", "true" if key == cmd else "false")
            btn.style().unpolish(btn)
            btn.style().polish(btn)

    def _send_selected(self) -> None:
        self.command_requested.emit(self._selected)
        self._status.setText(f"Sent {self._selected}. Waiting for firmware response...")

    def record_response(self, text: str) -> None:
        self._status.setText(f"Firmware response: {text}")


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
            ("Avg Force", f"{summary['avg_force']:.3f} N"),
            ("Peak Force", f"{summary['peak_force']:.3f} N"),
            ("Peak Finger", f"{summary['peak_finger']:.3f} N"),
            ("Contact Time", f"{summary['contact_pct']:.0f}%"),
            ("Warnings", str(summary["warning_events"])),
            ("Errors", str(summary["error_events"])),
            ("Tremor Avg", f"{summary['avg_tremor']:.3f}"),
            ("Smoothness f95", f"{summary['peak_f95']:.2f} Hz"),
        ]
        for idx, (name, value) in enumerate(metrics):
            item = ClinicalValue(name, "")
            item.set_value(value)
            item.set_accent(
                RED if name == "Errors" and summary["error_events"] else
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


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self._worker: SerialWorker | None = None
        self._cmd_queue: queue.Queue = queue.Queue()
        self._connected = False
        self._last_packet_at: float | None = None
        self._last_warn = 0
        self._last_err = 0
        self._last_state = ""
        self._mode = "UNSELECTED"
        self._latest_packet: dict | None = None
        self._hand_process: subprocess.Popen | None = None
        self._calibration_wizard: CalibrationWizard | None = None
        self._session_summary: SessionSummaryDialog | None = None
        self._session_mode = "UNSELECTED"
        self._session_active = False
        self._armed_mode = "UNSELECTED"
        self._awaiting_arm_mode: str | None = None
        self._pending_live_mode: str | None = None
        self._countdown_value = 0
        self._last_score: float | None = None
        self._telemetry = TelemetryStore()
        self._curves: dict[str, pg.PlotDataItem] = {}
        self._plot_widgets: list[pg.PlotWidget] = []
        self._metric_values: dict[str, ClinicalValue] = {}
        self._cmd_button_by_cmd: dict[str, QPushButton] = {}

        self._countdown_timer = QTimer(self)
        self._countdown_timer.timeout.connect(self._advance_countdown)

        self.setWindowTitle("Haptic Surgical Skill Trainer")
        self.resize(1280, 800)
        self.setMinimumSize(QSize(1180, 740))
        pg.setConfigOptions(antialias=True, foreground=TEXT, background=ICE)

        page = QWidget()
        page.setObjectName("central")
        self.setCentralWidget(page)

        root = QVBoxLayout(page)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        self._build_header(root)
        self._build_summary(root)
        self._build_main_content(root)
        self._build_engineering_log(root)

        self._populate_ports()
        self._set_commands_enabled(False)
        self._set_state(TrainerState.DISCONNECTED)
        self._set_mode("UNSELECTED")
        self._vital_sign.set_status(False)
        self._update_start_button()
        self._add_activity("System ready. Connect glove to begin.", BLUE)

        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._refresh_plots)
        self._plot_timer.start(100)

        self._stale_timer = QTimer(self)
        self._stale_timer.timeout.connect(self._check_stale)
        self._stale_timer.start(500)

    def _build_header(self, root: QVBoxLayout) -> None:
        header = QFrame()
        header.setObjectName("header_panel")
        row = QHBoxLayout(header)
        row.setContentsMargins(14, 8, 14, 8)
        row.setSpacing(8)

        title_col = QVBoxLayout()
        title_col.setSpacing(2)
        title = QLabel("Haptic Surgical Skill Trainer")
        title.setObjectName("title")
        subtitle = QLabel("Clinical training console | live force, motion, and skill telemetry")
        subtitle.setObjectName("subtitle")
        title_col.addWidget(title)
        title_col.addWidget(subtitle)
        row.addLayout(title_col, 1)

        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(260)
        row.addWidget(self._port_combo)

        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._populate_ports)
        row.addWidget(refresh_btn)

        self._btn_connect = QPushButton("Connect")
        self._btn_connect.setObjectName("connect_button")
        self._btn_connect.clicked.connect(self._on_connect_clicked)
        row.addWidget(self._btn_connect)
        root.addWidget(header)

    def _build_summary(self, root: QVBoxLayout) -> None:
        row = QHBoxLayout()
        row.setSpacing(8)

        self._score_card = SummaryCard("Session Score", "resets every mode start", GREEN)
        self._force_card = SummaryCard("Total Force", "N", BLUE)
        self._vital_sign = VitalSignWidget()
        row.addWidget(self._score_card, 1)
        row.addWidget(self._force_card, 1)
        row.addWidget(self._vital_sign, 2)

        status = QFrame()
        status.setObjectName("status_panel")
        layout = QGridLayout(status)
        layout.setContentsMargins(14, 8, 14, 8)
        layout.setHorizontalSpacing(14)
        layout.setVerticalSpacing(3)

        self._state_label = QLabel("DISCONNECTED")
        self._state_label.setObjectName("state_label")
        self._mode_label = QLabel("No mode selected")
        self._mode_label.setObjectName("mode_badge")
        self._session_hint_label = QLabel("Choose a difficulty mode to start a scored session.")
        self._session_hint_label.setObjectName("session_hint")
        self._packet_label = QLabel("Packets: 0")
        self._packet_label.setObjectName("muted_label")
        self._rate_label = QLabel("JSON: -- Hz")
        self._rate_label.setObjectName("muted_label")
        self._stale_label = QLabel("Telemetry: waiting")
        self._stale_label.setObjectName("status_badge")
        self._contact_label = QLabel("Contact: --/--/--")
        self._contact_label.setObjectName("status_badge")
        self._score_delta_label = QLabel("Score change: --")
        self._score_delta_label.setObjectName("status_badge")
        self._coaching_label = QLabel("Awaiting live glove data")
        self._coaching_label.setObjectName("coach_label")
        self._alert_label = QLabel("No active warnings")
        self._alert_label.setObjectName("alert_label")

        layout.addWidget(QLabel("Session State"), 0, 0)
        layout.addWidget(self._state_label, 1, 0)
        layout.addWidget(self._mode_label, 2, 0)
        layout.addWidget(self._packet_label, 0, 1)
        layout.addWidget(self._rate_label, 1, 1)
        layout.addWidget(self._stale_label, 2, 1)
        layout.addWidget(self._coaching_label, 0, 2, 2, 1)
        layout.addWidget(self._session_hint_label, 2, 2)
        layout.addWidget(self._contact_label, 3, 0)
        layout.addWidget(self._score_delta_label, 3, 1)
        layout.addWidget(self._alert_label, 3, 2)
        layout.setColumnStretch(2, 1)
        row.addWidget(status, 3)
        root.addLayout(row)

    def _build_main_content(self, root: QVBoxLayout) -> None:
        split = QHBoxLayout()
        split.setSpacing(10)

        left = QVBoxLayout()
        left.setSpacing(0)
        self._build_plots(left)
        split.addLayout(left, 1)

        right_panel = QWidget()
        right_panel.setFixedWidth(330)
        right = QVBoxLayout()
        right_panel.setLayout(right)
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(8)
        self._build_side_tabs(right)
        split.addWidget(right_panel)

        root.addLayout(split, 1)

    def _build_side_tabs(self, root: QVBoxLayout) -> None:
        tabs = QTabWidget()
        tabs.setObjectName("side_tabs")

        session = QWidget()
        session_layout = QVBoxLayout(session)
        session_layout.setContentsMargins(0, 0, 0, 0)
        session_layout.setSpacing(8)
        self._build_command_panel(session_layout)
        self._build_visualization_panel(session_layout)
        self._build_activity_feed(session_layout)
        tabs.addTab(session, "Session")

        metrics = QWidget()
        metrics_layout = QVBoxLayout(metrics)
        metrics_layout.setContentsMargins(0, 0, 0, 0)
        metrics_layout.setSpacing(0)
        self._build_metrics_strip(metrics_layout)
        tabs.addTab(metrics, "Metrics")

        info = QWidget()
        info_layout = QVBoxLayout(info)
        info_layout.setContentsMargins(0, 0, 0, 0)
        info_layout.setSpacing(8)
        self._build_info_tab(info_layout)
        tabs.addTab(info, "Info")

        root.addWidget(tabs, 1)

    def _build_info_tab(self, root: QVBoxLayout) -> None:
        panel = QFrame()
        panel.setObjectName("side_panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(8)

        title = QLabel("Training Modes")
        title.setObjectName("panel_title")
        intro = QLabel("Select a mode to arm the trainer, then press Start for the countdown and scored run.")
        intro.setObjectName("panel_caption")
        intro.setWordWrap(True)
        layout.addWidget(title)
        layout.addWidget(intro)

        for name, color, body in [
            ("Easy", GREEN, "Widest force margin. Use for onboarding, sensor checks, and early practice."),
            ("Intermediate", BLUE, "Normal training mode. Use once the user can maintain stable contact."),
            ("Hard", RED, "Narrowest force margin. Use for advanced control and assessment runs."),
        ]:
            card = QFrame()
            card.setObjectName("info_card")
            card.setStyleSheet(
                "QFrame#info_card {"
                f"background-color: {PANEL_ALT};"
                f"border: 1px solid {BORDER};"
                f"border-left: 4px solid {color};"
                "border-radius: 8px;"
                "}"
            )
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(10, 8, 10, 8)
            card_layout.setSpacing(3)
            heading = QLabel(name)
            heading.setObjectName("info_mode")
            heading.setStyleSheet(f"color: {color};")
            text = QLabel(body)
            text.setObjectName("info_body")
            text.setWordWrap(True)
            card_layout.addWidget(heading)
            card_layout.addWidget(text)
            layout.addWidget(card)

        layout.addStretch()
        root.addWidget(panel)

    def _build_metrics_strip(self, root: QVBoxLayout) -> None:
        panel = QFrame()
        panel.setObjectName("metric_strip")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(6)
        title = QLabel("Live Metrics")
        title.setObjectName("panel_title")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)
        layout.addLayout(grid)

        specs = [
            ("f0", "Thumb", "N", BLUE),
            ("f1", "Index", "N", BLUE),
            ("f2", "Middle", "N", BLUE),
            ("roll", "Roll", "deg", BLUE_DARK),
            ("pitch", "Pitch", "deg", BLUE_DARK),
            ("yaw", "Yaw", "deg", BLUE_DARK),
            ("tremor", "Tremor", "ratio", VIOLET),
            ("cv_f", "Force CV", "ratio", VIOLET),
            ("swing", "Swing", "Hz", VIOLET),
            ("f95", "F95", "Hz", VIOLET),
            ("contact", "Contact", "T/I/M", GREEN),
            ("warn_err", "Warn / Err", "bitmask", AMBER),
        ]
        for idx, (key, title, unit, color) in enumerate(specs):
            item = ClinicalValue(title, unit, color)
            self._metric_values[key] = item
            grid.addWidget(item, idx // 2, idx % 2)
        root.addWidget(panel)

    def _build_plots(self, root: QVBoxLayout) -> None:
        self._plots = QTabWidget()
        self._plots.setObjectName("plot_tabs")
        self._plots.setMinimumHeight(500)
        self._plots.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        force = self._make_plot("Force profile", "Time (s)", "Force (N)")
        force.setYRange(0, 5)
        self._curves["f0"] = self._plot_curve(force, "Thumb", BLUE)
        self._curves["f1"] = self._plot_curve(force, "Index", GREEN)
        self._curves["f2"] = self._plot_curve(force, "Middle", VIOLET)
        self._curves["f_sum"] = self._plot_curve(force, "Total", AMBER, 3)
        self._plots.addTab(force, "Force")

        motion = self._make_plot("Orientation", "Time (s)", "Angle (deg)")
        motion.setYRange(-180, 180)
        self._curves["roll"] = self._plot_curve(motion, "Roll", BLUE)
        self._curves["pitch"] = self._plot_curve(motion, "Pitch", GREEN)
        self._curves["yaw"] = self._plot_curve(motion, "Yaw", AMBER)
        self._plots.addTab(motion, "Motion")

        skill = self._make_plot("Skill stability", "Time (s)", "Scaled metric")
        skill.setYRange(0, 20)
        self._curves["tremor"] = self._plot_curve(skill, "Tremor x10", RED)
        self._curves["cv_f"] = self._plot_curve(skill, "Force CV x10", VIOLET)
        self._curves["swing"] = self._plot_curve(skill, "Swing", BLUE)
        self._curves["f95"] = self._plot_curve(skill, "F95", AMBER)
        self._plots.addTab(skill, "Skill")

        score = self._make_plot("Performance score", "Time (s)", "Score / event")
        score.setYRange(0, 100)
        self._curves["score"] = self._plot_curve(score, "Score", GREEN, 3)
        self._curves["warn"] = self._plot_curve(score, "Warning event", AMBER)
        self._curves["err"] = self._plot_curve(score, "Error event", RED)
        self._plots.addTab(score, "Score")
        root.addWidget(self._plots, 1)

    def _make_plot(self, title: str, x_label: str, y_label: str) -> pg.PlotWidget:
        plot = pg.PlotWidget(background=PANEL)
        plot.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        plot.setTitle(title, color=NAVY, size="12pt")
        plot.setLabel("bottom", x_label, color=MUTED)
        plot.setLabel("left", y_label, color=MUTED)
        plot.showGrid(x=True, y=True, alpha=0.28)
        plot.addLegend(offset=(12, 12), labelTextColor=TEXT)
        plot.getAxis("bottom").setPen(GRID)
        plot.getAxis("left").setPen(GRID)
        plot.getAxis("bottom").setTextPen(MUTED)
        plot.getAxis("left").setTextPen(MUTED)
        plot.setMenuEnabled(False)
        plot.setMouseEnabled(x=False, y=False)
        self._plot_widgets.append(plot)
        return plot

    def _plot_curve(self, plot: pg.PlotWidget, name: str,
                    color: str, width: int = 2) -> pg.PlotDataItem:
        return plot.plot([], [], name=name, pen=pg.mkPen(color=color, width=width))

    def _build_visualization_panel(self, root: QVBoxLayout) -> None:
        panel = QFrame()
        panel.setObjectName("side_panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(6)
        title = QLabel("3D Visualization")
        title.setObjectName("panel_title")
        caption = QLabel("Standalone skeletal suturing pose. IMU rotates the view; force highlights contact.")
        caption.setObjectName("panel_caption")
        open_3d = QPushButton("Open 3D Hand View")
        open_3d.clicked.connect(self._open_hand_window)
        self._hand_status = QLabel("3D window closed")
        self._hand_status.setObjectName("muted_label")
        self._hand_status.setWordWrap(True)
        layout.addWidget(title)
        layout.addWidget(caption)
        layout.addWidget(open_3d)
        layout.addWidget(self._hand_status)
        root.addWidget(panel)

    def _build_command_panel(self, root: QVBoxLayout) -> None:
        panel = QFrame()
        panel.setObjectName("side_panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(6)
        title = QLabel("Training Controls")
        title.setObjectName("panel_title")
        layout.addWidget(title)

        self._cmd_buttons: list[QPushButton] = []
        grid = QGridLayout()
        grid.setHorizontalSpacing(7)
        grid.setVerticalSpacing(7)
        for idx, (cmd, label) in enumerate([
            ("I", "Identify"),
            ("S", "Stop"),
            ("E", "Easy"),
            ("M", "Intermediate"),
            ("H", "Hard"),
        ]):
            if cmd in MODE_BY_COMMAND:
                button = self._mode_button(cmd, label)
            else:
                button = self._command_button(cmd, label)
            grid.addWidget(button, idx // 2, idx % 2)
        layout.addLayout(grid)

        self._btn_start = QPushButton("Start")
        self._btn_start.setObjectName("start_button")
        self._btn_start.clicked.connect(self._start_countdown)
        self._btn_start.setEnabled(False)
        self._countdown_label = QLabel("Select a mode, then press Start.")
        self._countdown_label.setObjectName("countdown_label")
        self._countdown_label.setWordWrap(True)
        layout.addWidget(self._btn_start)
        layout.addWidget(self._countdown_label)

        calibrate = QPushButton("Calibrate...")
        calibrate.clicked.connect(self._open_calibration_wizard)
        layout.addWidget(calibrate)
        exit_btn = self._command_button("X", "Exit Session", "danger_button")
        layout.addWidget(exit_btn)
        root.addWidget(panel)

    def _command_button(self, cmd: str, label: str,
                        obj_name: str = "command_button") -> QPushButton:
        btn = QPushButton(label)
        btn.setObjectName(obj_name)
        btn.setToolTip(f"Send {cmd}")
        btn.clicked.connect(lambda _checked=False, value=cmd: self._send_command(value))
        self._cmd_buttons.append(btn)
        self._cmd_button_by_cmd[cmd] = btn
        return btn

    def _mode_button(self, cmd: str, label: str) -> QPushButton:
        btn = QPushButton(label)
        btn.setObjectName("command_button")
        btn.setToolTip(f"Select {label} mode")
        btn.clicked.connect(lambda _checked=False, value=cmd: self._select_mode(value))
        self._cmd_buttons.append(btn)
        self._cmd_button_by_cmd[cmd] = btn
        return btn

    def _build_activity_feed(self, root: QVBoxLayout) -> None:
        panel = QFrame()
        panel.setObjectName("side_panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(6)
        title = QLabel("Clinical Coaching")
        title.setObjectName("panel_title")
        layout.addWidget(title)

        scroll = QScrollArea()
        scroll.setObjectName("feed_scroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setMinimumHeight(120)
        container = QWidget()
        self._feed_layout = QVBoxLayout(container)
        self._feed_layout.setContentsMargins(0, 0, 0, 0)
        self._feed_layout.setSpacing(8)
        self._feed_layout.addStretch()
        scroll.setWidget(container)
        layout.addWidget(scroll, 1)
        root.addWidget(panel, 1)

    def _build_engineering_log(self, root: QVBoxLayout) -> None:
        self._btn_terminal = QPushButton("Show Engineering Log")
        self._btn_terminal.setObjectName("terminal_button")
        self._btn_terminal.clicked.connect(self._toggle_terminal)
        root.addWidget(self._btn_terminal)

        self._terminal_container = QWidget()
        term_layout = QVBoxLayout(self._terminal_container)
        term_layout.setContentsMargins(0, 0, 0, 0)
        term_layout.setSpacing(0)
        self._log_view = QTextEdit()
        self._log_view.setObjectName("log")
        self._log_view.setReadOnly(True)
        self._log_view.setMinimumHeight(170)
        term_layout.addWidget(self._log_view)
        self._terminal_container.setVisible(False)
        root.addWidget(self._terminal_container)

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
            self._cancel_countdown("Countdown cancelled. Select a mode and press Start.")
        self._pending_live_mode = None
        self._awaiting_arm_mode = mode
        self._armed_mode = mode
        self._set_mode(mode)
        self._countdown_label.setText("Selecting mode on the glove...")
        self._coaching_label.setText(f"Selecting {MODE_LABELS[mode]} mode...")
        self._coaching_label.setStyleSheet(f"color: {MODE_COLORS[mode]};")
        self._update_start_button()
        self._send_command(cmd)

    def _arm_training_mode(self, mode: str) -> None:
        self._armed_mode = mode
        self._session_mode = mode
        self._mode = mode
        self._session_active = False
        self._awaiting_arm_mode = None
        self._pending_live_mode = None
        self._set_mode(mode)
        self._score_card.set_value("--")
        self._score_card.set_accent(GRAY)
        self._score_delta_label.setText("Score change: --")
        self._score_delta_label.setStyleSheet(f"color: {MUTED};")
        self._coaching_label.setText(f"{MODE_LABELS[mode]} mode selected. Press Start.")
        self._coaching_label.setStyleSheet(f"color: {MODE_COLORS[mode]};")
        self._countdown_label.setText("Mode selected. Press Start for a 3-second countdown.")
        self._vital_sign.set_status(False)
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

        self._pending_live_mode = None
        self._countdown_value = 3
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

        self._pending_live_mode = self._armed_mode
        self._countdown_label.setText("Starting scored session...")
        self._send_command(COMMAND_BY_MODE[self._armed_mode])

    def _set_countdown_text(self, text: str) -> None:
        label = f"Starting in {text}" if text != "Begin" else "Begin"
        self._countdown_label.setText(label)
        self._coaching_label.setText(label)
        self._coaching_label.setStyleSheet(f"color: {MODE_COLORS.get(self._armed_mode, BLUE)};")
        self._vital_sign.set_countdown(text)

    def _cancel_countdown(self, message: str) -> None:
        self._countdown_timer.stop()
        self._countdown_value = 0
        self._awaiting_arm_mode = None
        self._pending_live_mode = None
        self._countdown_label.setText(message)
        self._vital_sign.set_status(False)
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
            button.setText("Starting...")
        elif self._awaiting_arm_mode is not None:
            button.setText("Selecting...")
        elif self._session_active:
            button.setText("Live")
        else:
            button.setText("Start")

    @Slot()
    def _on_connect_clicked(self) -> None:
        if self._connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self._port_combo.currentData()
        if not port:
            self._log("No port selected", "err")
            return

        self._cmd_queue = queue.Queue()
        self._worker = SerialWorker(port, BAUD_RATE, self._cmd_queue)
        self._worker.data_received.connect(self._on_data_received)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.finished.connect(self._on_worker_finished)
        self._worker.start()

        self._connected = True
        self._last_packet_at = None
        self._reset_session_display(clear_mode=True)
        self._btn_connect.setText("Disconnect")
        self._btn_connect.setProperty("connected", "true")
        self._repolish(self._btn_connect)
        self._set_commands_enabled(True)
        self._update_start_button()
        self._set_state(TrainerState.CONNECTED)
        self._stale_label.setText("Telemetry: waiting")
        self._stale_label.setStyleSheet(f"color: {AMBER};")
        self._log(f"Opening {port} at {BAUD_RATE}", "sys")
        self._add_activity(f"Connected to {port.split('/')[-1]}", BLUE)

    def _disconnect(self) -> None:
        if self._worker:
            self._worker.stop()
            self._worker.wait()
            self._worker = None
        self._connected = False
        self._last_packet_at = None
        self._reset_session_display(clear_mode=True)
        self._btn_connect.setText("Connect")
        self._btn_connect.setProperty("connected", "false")
        self._repolish(self._btn_connect)
        self._set_commands_enabled(False)
        self._update_start_button()
        self._set_state(TrainerState.DISCONNECTED)
        self._stale_label.setText("Telemetry: disconnected")
        self._stale_label.setStyleSheet(f"color: {MUTED};")
        self._log("Disconnected", "sys")
        self._add_activity("Disconnected", MUTED)

    @Slot(str)
    def _on_data_received(self, line: str) -> None:
        self._log(line, "rx")
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
        self._latest_packet = packet
        self._last_packet_at = time.monotonic()
        self._stale_label.setText("Telemetry: live")
        self._stale_label.setStyleSheet(f"color: {GREEN};")

        if self._session_active:
            self._telemetry.append(packet)
            self._packet_label.setText(f"Packets: {self._telemetry.packet_count}")
            if len(self._telemetry.series["t"]) >= 2:
                times = list(self._telemetry.series["t"])
                dt = times[-1] - times[-2]
                rate = (1.0 / dt) if dt > 0 else 0.0
                self._rate_label.setText(f"JSON: {rate:.1f} Hz")
        else:
            self._rate_label.setText("JSON: live")

        state = str(packet.get("state", "CONNECTED"))
        self._set_state(self._state_from_text(state))
        self._update_metrics(packet, scored=self._session_active)
        if self._session_active:
            self._update_coaching(packet)
        self._send_to_hand_visualizer(packet)

    def _handle_status_json(self, payload: dict) -> None:
        if "nvs" in payload:
            text = f"Calibration storage: {payload['nvs']}"
            self._add_activity(text, AMBER)
            self._record_calibration_response(text)
        elif "cal" in payload:
            text = f"Calibration: {payload['cal']}"
            self._add_activity(text, BLUE)
            self._record_calibration_response(text)
        elif "err" in payload:
            text = f"Firmware error: {payload['err']}"
            self._add_activity(text, RED)
            self._record_calibration_response(text)
        else:
            self._add_activity("Status JSON received", MUTED)

    def _handle_text_response(self, line: str) -> None:
        clean = line.strip()
        self._record_calibration_response(clean)
        response = COMMAND_RESPONSES.get(clean)
        if response is None:
            if clean.startswith("READY:"):
                self._set_state(TrainerState.CONNECTED)
                self._add_activity("Firmware ready", BLUE)
            return

        key, message = response
        if key in ("EASY", "INTERMEDIATE", "HARD"):
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
                    f"{MODE_LABELS[key]} mode selected - press Start when ready",
                    MODE_COLORS[key],
                )
        elif key == "IDLE":
            was_live = self._session_active
            self._set_state(TrainerState.IDLE)
            self._session_active = False
            self._awaiting_arm_mode = None
            self._pending_live_mode = None
            self._armed_mode = "UNSELECTED"
            if self._countdown_timer.isActive():
                self._cancel_countdown("Stopped before the session started.")
            self._set_mode(self._session_mode, active=False, complete=True)
            self._vital_sign.set_status(False)
            self._update_start_button()
            self._add_activity(message, MUTED)
            if clean == "STOPPED" and was_live:
                self._show_session_summary()
        elif key == "EXITED":
            self._set_state(TrainerState.EXITED)
            self._armed_mode = "UNSELECTED"
            self._session_active = False
            self._awaiting_arm_mode = None
            self._pending_live_mode = None
            self._vital_sign.set_status(False)
            self._set_commands_enabled(False)
            self._update_start_button()
            self._add_activity(message, RED)
        else:
            self._set_state(TrainerState.CONNECTED)
            self._add_activity(message, BLUE)

    def _state_from_text(self, value: str) -> TrainerState:
        try:
            return TrainerState(value)
        except ValueError:
            return TrainerState.CONNECTED

    def _begin_training_session(self, mode: str) -> None:
        self._session_mode = mode
        self._mode = mode
        self._armed_mode = mode
        self._session_active = True
        self._last_score = 100.0
        self._last_warn = 0
        self._last_err = 0
        self._last_state = ""
        self._countdown_timer.stop()
        self._countdown_value = 0
        self._telemetry.reset()
        self._clear_plots()
        self._set_mode(mode, active=True)
        self._score_card.set_value("100.0")
        self._score_card.set_accent(GREEN)
        self._force_card.set_value("0.000")
        self._force_card.set_accent(BLUE)
        self._packet_label.setText("Packets: 0")
        self._rate_label.setText("JSON: -- Hz")
        self._contact_label.setText("Contact: --/--/--")
        self._contact_label.setStyleSheet(f"color: {MUTED};")
        self._score_delta_label.setText("Score change: reset")
        self._score_delta_label.setStyleSheet(f"color: {GREEN};")
        self._coaching_label.setText(f"{MODE_LABELS[mode]} session live - make contact to begin")
        self._coaching_label.setStyleSheet(f"color: {MODE_COLORS[mode]};")
        self._countdown_label.setText("Live session running. Press Stop to finish.")
        self._vital_sign.set_status(True, 0, 0)
        self._update_start_button()
        self._add_activity(f"New {MODE_LABELS[mode].lower()} session started - score reset to 100", BLUE)

    def _reset_session_display(self, clear_mode: bool = False) -> None:
        self._countdown_timer.stop()
        self._countdown_value = 0
        self._pending_live_mode = None
        self._session_active = False
        self._last_score = None
        self._last_warn = 0
        self._last_err = 0
        self._last_state = ""
        self._telemetry.reset()
        self._clear_plots()
        self._score_card.set_value("--")
        self._score_card.set_accent(GRAY)
        self._force_card.set_value("--")
        self._force_card.set_accent(GRAY)
        self._packet_label.setText("Packets: 0")
        self._rate_label.setText("JSON: -- Hz")
        self._contact_label.setText("Contact: --/--/--")
        self._contact_label.setStyleSheet(f"color: {MUTED};")
        self._score_delta_label.setText("Score change: --")
        self._score_delta_label.setStyleSheet(f"color: {MUTED};")
        self._alert_label.setText("No active warnings")
        self._alert_label.setStyleSheet(f"color: {GREEN};")
        self._vital_sign.set_status(False)
        if hasattr(self, "_countdown_label"):
            self._countdown_label.setText("Select a mode, then press Start.")
        for metric in self._metric_values.values():
            metric.set_value("--")
            metric.set_accent(BLUE)
        if clear_mode:
            self._session_mode = "UNSELECTED"
            self._mode = "UNSELECTED"
            self._armed_mode = "UNSELECTED"
            self._set_mode("UNSELECTED")
        self._update_start_button()

    def _set_mode(self, mode: str, active: bool = False,
                  complete: bool = False, countdown: bool = False) -> None:
        label = MODE_LABELS.get(mode, mode.title())
        color = MODE_COLORS.get(mode, MUTED)
        if mode == "UNSELECTED":
            text = "No mode selected"
            hint = "Choose Easy, Intermediate, or Hard to start a scored session."
        elif complete:
            text = f"Last Mode: {label}"
            hint = "Session stopped. Review the summary or choose a mode to start a fresh score."
        elif active:
            text = f"Active Mode: {label}"
            hint = "Live scoring is running from a fresh 100-point session."
        elif countdown:
            text = f"Starting: {label}"
            hint = "Countdown is running. Scoring starts after Begin."
        else:
            text = f"Selected Mode: {label}"
            hint = "Mode selected. Press Start when ready."

        self._mode_label.setText(text)
        self._mode_label.setStyleSheet(
            f"color: {color};"
            f"background-color: {BLUE_LIGHT if color != RED else '#fff2f2'};"
            f"border: 1px solid {color};"
            "border-radius: 8px;"
            "padding: 4px 8px;"
            "font-weight: 900;"
        )
        self._session_hint_label.setText(hint)

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
        def values(key: str) -> list[float]:
            return self._telemetry.y(key)

        times = self._telemetry.x()
        samples = len(times)
        duration_s = times[-1] - times[0] if len(times) >= 2 else 0.0
        scores = values("score")
        forces = values("f_sum")
        f0 = values("f0")
        f1 = values("f1")
        f2 = values("f2")
        warns = values("warn")
        errs = values("err")
        contacts = values("contact_any")
        tremor = values("tremor")
        f95 = values("f95")

        final_score = scores[-1] if scores else 0.0
        warning_events = sum(1 for v in warns if v > 0.0)
        error_events = sum(1 for v in errs if v > 0.0)
        contact_pct = (100.0 * sum(contacts) / len(contacts)) if contacts else 0.0
        peak_finger = max(f0 + f1 + f2) if (f0 or f1 or f2) else 0.0
        avg_force = (sum(forces) / len(forces)) if forces else 0.0
        peak_force = max(forces) if forces else 0.0
        avg_tremor = (sum(tremor) / len(tremor)) if tremor else 0.0
        peak_f95 = max(f95) if f95 else 0.0

        if final_score >= 90 and error_events == 0:
            grade = "Expert control"
            grade_color = GREEN
            coaching = "Clean session. Force stayed controlled and no major error events were recorded."
        elif final_score >= 75 and error_events == 0:
            grade = "Good technique"
            grade_color = BLUE
            coaching = "Good run. Review warning moments and keep force changes smooth."
        elif final_score >= 50:
            grade = "Needs refinement"
            grade_color = AMBER
            coaching = "Usable session, but force or motion stability needs more consistency."
        else:
            grade = "High-risk handling"
            grade_color = RED
            coaching = "High event count or low score. Repeat at an easier mode and focus on gentle contact."

        if samples == 0:
            coaching = "No live telemetry was captured for this session. Start a mode, interact with the glove, then press Stop."

        return {
            "mode": self._session_mode,
            "samples": samples,
            "duration_s": duration_s,
            "final_score": final_score,
            "grade": grade,
            "grade_color": grade_color,
            "avg_force": avg_force,
            "peak_force": peak_force,
            "peak_finger": peak_finger,
            "contact_pct": contact_pct,
            "warning_events": warning_events,
            "error_events": error_events,
            "avg_tremor": avg_tremor,
            "peak_f95": peak_f95,
            "coaching": coaching,
        }

    def _update_metrics(self, packet: dict, scored: bool = True) -> None:
        def f(key: str, default: float = 0.0) -> float:
            return float(packet.get(key, default))

        score = f("score")
        total_force = f("f_sum")
        self._force_card.set_value(f"{total_force:.3f}")
        self._force_card.set_accent(RED if total_force >= 4.0 else AMBER if total_force >= 2.0 else BLUE)

        if scored:
            self._score_card.set_value(f"{score:05.1f}")
            self._score_card.set_accent(GREEN if score >= 80 else AMBER if score >= 50 else RED)
            if self._last_score is None:
                self._score_delta_label.setText("Score change: --")
                self._score_delta_label.setStyleSheet(f"color: {MUTED};")
            else:
                delta = score - self._last_score
                if abs(delta) < 0.05:
                    self._score_delta_label.setText("Score change: steady")
                    self._score_delta_label.setStyleSheet(f"color: {GREEN};")
                else:
                    self._score_delta_label.setText(f"Score change: {delta:+.1f}")
                    self._score_delta_label.setStyleSheet(f"color: {GREEN if delta > 0 else AMBER};")
            self._last_score = score

        for key in ("f0", "f1", "f2"):
            value = f(key)
            self._metric_values[key].set_value(f"{value:.3f}")
            self._metric_values[key].set_accent(RED if value >= 1.5 else AMBER if value >= 0.8 else BLUE)
        for key in ("roll", "pitch", "yaw"):
            self._metric_values[key].set_value(f"{f(key):.2f}")

        warn = int(f("warn"))
        err = int(f("err"))
        tremor = f("tremor")
        self._metric_values["tremor"].set_value(f"{tremor:.3f}")
        self._metric_values["tremor"].set_accent(RED if err & (1 << 1) else AMBER if warn & (1 << 1) else VIOLET)
        self._metric_values["cv_f"].set_value(f"{f('cv_f'):.3f}")
        self._metric_values["swing"].set_value(f"{f('swing'):.2f}")
        self._metric_values["f95"].set_value(f"{f('f95'):.2f}")
        self._metric_values["warn_err"].set_value(f"0x{warn:02X} / 0x{err:02X}")
        self._metric_values["warn_err"].set_accent(RED if err else AMBER if warn else GREEN)

        contact = packet.get("contact", [0, 0, 0])
        if isinstance(contact, list) and len(contact) >= 3:
            text = "/".join("ON" if int(v) else "--" for v in contact[:3])
        else:
            text = "--/--/--"
        self._metric_values["contact"].set_value(text)
        self._contact_label.setText(f"Contact: {text}")
        self._contact_label.setStyleSheet(f"color: {GREEN if 'ON' in text else MUTED};")

    def _update_coaching(self, packet: dict) -> None:
        warn = int(packet.get("warn", 0))
        err = int(packet.get("err", 0))
        state = str(packet.get("state", ""))

        if err:
            messages = self._flag_messages(err, ERR_LABELS)
            text = "Error: " + ", ".join(messages)
            color = RED
            if err != self._last_err:
                self._add_activity(text, RED)
        elif warn:
            messages = self._flag_messages(warn, WARN_LABELS)
            text = "Warning: " + ", ".join(messages)
            color = AMBER
            if warn != self._last_warn:
                self._add_activity(text, AMBER)
        else:
            text = "Stable technique" if state in ("HOLD", "ACTIVE") else "Ready for contact"
            color = GREEN

        self._coaching_label.setText(text)
        self._coaching_label.setStyleSheet(f"color: {color};")
        self._alert_label.setText(f"Warn 0x{warn:02X} | Err 0x{err:02X}")
        self._alert_label.setStyleSheet(f"color: {RED if err else AMBER if warn else GREEN};")
        self._vital_sign.set_status(self._session_active, warn, err)

        if state and state != self._last_state:
            self._add_activity(
                f"State changed to {state}",
                STATE_COLORS.get(self._state_from_text(state), MUTED),
            )
        self._last_warn = warn
        self._last_err = err
        self._last_state = state

    def _flag_messages(self, flags: int, labels: dict[int, str]) -> list[str]:
        messages = [label for bit, label in labels.items() if flags & bit]
        return messages or [f"0x{flags:02X}"]

    def _refresh_plots(self) -> None:
        if not self._telemetry.latest:
            return
        x = self._telemetry.x()
        for key, curve in self._curves.items():
            curve.setData(x, self._plot_y(key))

        right = x[-1] if x else 0.0
        left = max(0.0, right - PLOT_WINDOW_S)
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
            self._coaching_label.setText("Telemetry paused - waiting for live packets")
            self._coaching_label.setStyleSheet(f"color: {RED};")

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
        self._calibration_wizard.show()
        self._calibration_wizard.raise_()
        self._calibration_wizard.activateWindow()

    def _record_calibration_response(self, text: str) -> None:
        if self._calibration_wizard is not None and self._calibration_wizard.isVisible():
            self._calibration_wizard.record_response(text)

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
                stdin=subprocess.PIPE,
                text=True,
                cwd=str(script.parent),
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
        color = STATE_COLORS[state]
        self._state_label.setText(state.value)
        self._state_label.setStyleSheet(f"color: {color};")
        if state == TrainerState.EXITED:
            self._set_commands_enabled(False)

    def _set_commands_enabled(self, enabled: bool) -> None:
        for button in getattr(self, "_cmd_buttons", []):
            button.setEnabled(enabled)
        self._update_start_button()

    def _add_activity(self, message: str, color: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
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
        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)
        msg = QLabel(message)
        msg.setWordWrap(True)
        msg.setStyleSheet(f"color: {TEXT}; font-weight: 700;")
        stamp = QLabel(ts)
        stamp.setObjectName("timestamp")
        layout.addWidget(msg)
        layout.addWidget(stamp)
        self._feed_layout.insertWidget(0, card)
        while self._feed_layout.count() > MAX_FEED_CARDS + 1:
            item = self._feed_layout.takeAt(self._feed_layout.count() - 2)
            if item and item.widget():
                item.widget().deleteLater()

    def _log(self, text: str, direction: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        color = {
            "tx": BLUE,
            "rx": GREEN,
            "err": RED,
            "sys": MUTED,
        }.get(direction, MUTED)
        prefix = {
            "tx": ">>",
            "rx": "<<",
            "err": "!!",
            "sys": "--",
        }.get(direction, "--")
        visible = text
        if not visible or not any(ch.isprintable() for ch in visible):
            visible = "<non-printable serial bytes>"
        elif any((not ch.isprintable()) and ch not in "\t" for ch in visible):
            visible = visible.encode("unicode_escape", errors="replace").decode("ascii")
        visible = html.escape(visible)
        self._log_view.append(
            f'<span style="color:{color};">[{ts}] {prefix} {visible}</span>'
        )
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


STYLESHEET = f"""
QMainWindow, QWidget#central, QScrollArea#main_scroll {{
    background-color: {ICE};
}}
QScrollArea#main_scroll {{
    border: none;
}}
QFrame#header_panel, QFrame#status_panel, QFrame#summary_card,
QFrame#metric_strip, QFrame#side_panel, QFrame#dialog_panel,
QFrame#hand_canvas_holder {{
    background-color: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QLabel {{
    color: {TEXT};
    font-size: 13px;
}}
QLabel#title {{
    color: {NAVY};
    font-size: 22px;
    font-weight: 900;
}}
QLabel#dialog_title {{
    color: {NAVY};
    font-size: 21px;
    font-weight: 900;
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
QLabel#subtitle, QLabel#muted_label, QLabel#panel_caption, QLabel#timestamp,
QLabel#session_hint {{
    color: {MUTED};
    font-size: 12px;
}}
QLabel#panel_title {{
    color: {NAVY};
    font-size: 15px;
    font-weight: 800;
}}
QLabel#state_label {{
    font-size: 25px;
    font-weight: 900;
}}
QLabel#coach_label {{
    font-size: 20px;
    font-weight: 900;
}}
QLabel#alert_label, QLabel#status_badge {{
    font-size: 12px;
    font-weight: 800;
}}
QLabel#countdown_label {{
    color: {MUTED};
    font-size: 12px;
    font-weight: 800;
}}
QLabel#mode_badge {{
    font-size: 12px;
    font-weight: 900;
}}
QLabel#summary_title, QLabel#metric_title {{
    color: {MUTED};
    font-size: 10px;
    font-weight: 800;
    text-transform: uppercase;
}}
QLabel#summary_value {{
    color: {NAVY};
    font-family: Menlo, Consolas, monospace;
    font-size: 30px;
    font-weight: 900;
}}
QLabel#metric_value {{
    color: {TEXT};
    font-family: Menlo, Consolas, monospace;
    font-size: 17px;
    font-weight: 900;
}}
QLabel#summary_unit, QLabel#metric_unit {{
    color: {MUTED};
    font-size: 10px;
}}
QLabel#info_mode {{
    font-size: 14px;
    font-weight: 900;
}}
QLabel#info_body {{
    color: {TEXT};
    font-size: 12px;
}}
QLabel#instruction_text {{
    color: {TEXT};
    font-size: 14px;
    line-height: 150%;
}}
QLabel#command_chip {{
    color: {NAVY};
    background-color: {BLUE_LIGHT};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 8px;
    font-family: Menlo, Consolas, monospace;
    font-weight: 800;
}}
QLabel#fallback_label {{
    color: {MUTED};
    background-color: {PANEL_ALT};
    border: 1px dashed {BORDER};
    border-radius: 8px;
    padding: 12px;
}}
QFrame#clinical_value {{
    background-color: {PANEL_ALT};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QFrame#activity_card {{
    background-color: {PANEL_ALT};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QFrame#info_card {{
    background-color: {PANEL_ALT};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QComboBox, QTextEdit {{
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
QTextEdit#log {{
    font-family: Menlo, Consolas, monospace;
    font-size: 12px;
}}
QPushButton {{
    background-color: {PANEL};
    color: {NAVY};
    border: 1px solid {BORDER};
    border-radius: 8px;
    padding: 8px 10px;
    min-height: 24px;
    font-weight: 800;
}}
QPushButton:hover {{
    background-color: {BLUE_LIGHT};
    border-color: {BLUE};
}}
QPushButton:pressed {{
    background-color: #d7ebff;
}}
QPushButton:disabled {{
    color: #a9b6c4;
    background-color: #f1f6fb;
    border-color: #d8e4ef;
}}
QPushButton#connect_button[connected="true"] {{
    color: {GREEN};
    border-color: {GREEN};
}}
QPushButton#danger_button {{
    color: {RED};
    border-color: #efb4b4;
}}
QPushButton#start_button {{
    color: {GREEN};
    border-color: {GREEN};
    background-color: #f1fbf5;
}}
QPushButton#start_button:disabled {{
    color: #9fb8aa;
    border-color: #d5e6dd;
    background-color: #f4f8f6;
}}
QPushButton[activeMode="true"] {{
    color: {PANEL};
    background-color: {BLUE};
    border-color: {BLUE_DARK};
}}
QPushButton#terminal_button {{
    color: {MUTED};
    background-color: transparent;
}}
QPushButton[selected="true"] {{
    background-color: {BLUE_LIGHT};
    border-color: {BLUE};
    color: {NAVY};
}}
QTabWidget::pane {{
    background-color: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QTabBar::tab {{
    background: {PANEL_ALT};
    color: {MUTED};
    border: 1px solid {BORDER};
    padding: 8px 16px;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
}}
QTabBar::tab:selected {{
    color: {NAVY};
    background: {PANEL};
    border-bottom-color: {PANEL};
}}
QScrollArea#feed_scroll {{
    border: none;
    background: transparent;
}}
"""


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Arial", 12))
    app.setStyleSheet(STYLESHEET)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
