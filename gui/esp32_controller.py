"""
Haptic Surgical Skill Trainer — Lab 4 Dev Interface
PySide6 host-side application for the uart_echo_VitalSignsLab4 project.

Communicates with the ESP32 over USB-serial (115200 baud).
Supported commands: I (Identify), E (Easy), H (Hard), S (Stop), X (Exit)
"""

import queue
import sys
from datetime import datetime
from enum import Enum

import serial
import serial.tools.list_ports
from PySide6.QtCore import (
    QPauseAnimation,
    QPropertyAnimation,
    QSequentialAnimationGroup,
    QThread,
    QEasingCurve,
    Signal,
    Slot,
    Qt,
    QSize,
    Property,
)
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QRadialGradient
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


# ---------------------------------------------------------------------------
# Board state
# ---------------------------------------------------------------------------

class BoardState(Enum):
    IDLE      = "IDLE"
    CONNECTED = "CONNECTED"
    EASY      = "EASY"
    HARD      = "HARD"
    EXITED    = "EXITED"


STATE_COLORS: dict[BoardState, str] = {
    BoardState.IDLE:      "#3D4455",
    BoardState.CONNECTED: "#00BCD4",
    BoardState.EASY:      "#FF8F00",
    BoardState.HARD:      "#D32F2F",
    BoardState.EXITED:    "#5C1010",
}

STATE_LABELS: dict[BoardState, str] = {
    BoardState.IDLE:      "STANDBY",
    BoardState.CONNECTED: "TRAINER ONLINE",
    BoardState.EASY:      "EASY MODE",
    BoardState.HARD:      "HARD MODE",
    BoardState.EXITED:    "SESSION ENDED",
}

RESPONSE_TO_STATE: dict[str, BoardState] = {
    "CONNECTED":     BoardState.CONNECTED,
    "EASY":          BoardState.EASY,
    "HARD":          BoardState.HARD,
    "IDLE":          BoardState.IDLE,
    "READY":         BoardState.CONNECTED,
    "ALREADY IDLE":  BoardState.CONNECTED,
    "ALREADY READY": BoardState.CONNECTED,
    "EXITED":        BoardState.EXITED,
}

ACTIVITY_MESSAGES: dict[BoardState, tuple[str, str]] = {
    BoardState.CONNECTED: ("TRAINER ONLINE · LED active",     "#00BCD4"),
    BoardState.EASY:      ("EASY MODE · 500 ms blink",        "#FF8F00"),
    BoardState.HARD:      ("HARD MODE · 100 ms blink",        "#D32F2F"),
    BoardState.IDLE:      ("STANDBY · mode stopped",          "#3D4455"),
    BoardState.EXITED:    ("SESSION ENDED · reset to resume", "#5C1010"),
}

MAX_FEED_CARDS = 20


# ---------------------------------------------------------------------------
# Vitals-style indicator widget
# ---------------------------------------------------------------------------

class LedIndicator(QWidget):
    """Heartbeat-rhythm indicator. Pulses like a vitals monitor in EASY/HARD."""

    SIZE = 28

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFixedSize(self.SIZE, self.SIZE)
        self._state = BoardState.IDLE
        self._glow  = 1.0

        self._beat_rise = QPropertyAnimation(self, b"glow_level")
        self._beat_rise.setEasingCurve(QEasingCurve.Type.OutQuad)
        self._beat_rise.setStartValue(0.05)
        self._beat_rise.setEndValue(1.0)

        self._beat_fall = QPropertyAnimation(self, b"glow_level")
        self._beat_fall.setEasingCurve(QEasingCurve.Type.InQuad)
        self._beat_fall.setStartValue(1.0)
        self._beat_fall.setEndValue(0.05)

        self._beat_pause = QPauseAnimation()

        self._anim_group = QSequentialAnimationGroup(self)
        self._anim_group.setLoopCount(-1)
        self._anim_group.addAnimation(self._beat_rise)
        self._anim_group.addAnimation(self._beat_fall)
        self._anim_group.addAnimation(self._beat_pause)

    def _get_glow(self) -> float:
        return self._glow

    def _set_glow(self, value: float) -> None:
        self._glow = value
        self.update()

    glow_level = Property(float, _get_glow, _set_glow)

    def set_state(self, state: BoardState) -> None:
        self._state = state
        self._anim_group.stop()
        if state == BoardState.EASY:
            self._beat_rise.setDuration(150)
            self._beat_fall.setDuration(250)
            self._beat_pause.setDuration(600)
            self._anim_group.start()
        elif state == BoardState.HARD:
            self._beat_rise.setDuration(80)
            self._beat_fall.setDuration(150)
            self._beat_pause.setDuration(200)
            self._anim_group.start()
        else:
            self._glow = 1.0
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        color = QColor(STATE_COLORS[self._state])
        cx, cy, r = self.SIZE / 2, self.SIZE / 2, self.SIZE / 2 - 3

        if self._state in (BoardState.EASY, BoardState.HARD):
            halo = QRadialGradient(cx, cy, r * 2.0)
            glow_color = QColor(color)
            glow_color.setAlphaF(self._glow * 0.40)
            halo.setColorAt(0.0, glow_color)
            halo.setColorAt(1.0, QColor(0, 0, 0, 0))
            painter.setBrush(halo)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(int(cx - r * 2.0), int(cy - r * 2.0),
                                int(r * 4.0), int(r * 4.0))

        ring_color = QColor(color)
        ring_color.setAlphaF(0.55)
        painter.setPen(QPen(ring_color, 1.5))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(int(cx - r - 2), int(cy - r - 2),
                            int((r + 2) * 2), int((r + 2) * 2))

        painter.setPen(QPen(color.darker(160), 1))
        fill_color = QColor(color)
        if self._state in (BoardState.EASY, BoardState.HARD):
            fill_color.setAlphaF(0.35 + self._glow * 0.65)
        painter.setBrush(fill_color)
        painter.drawEllipse(int(cx - r), int(cy - r), int(r * 2), int(r * 2))

        painter.setBrush(QColor(255, 255, 255, 60))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(int(cx - r * 0.40), int(cy - r * 0.50),
                            int(r * 0.45), int(r * 0.35))
        painter.end()


# ---------------------------------------------------------------------------
# Serial worker thread  (unchanged)
# ---------------------------------------------------------------------------

class SerialWorker(QThread):
    data_received  = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, port: str, baud: int, cmd_queue: queue.Queue) -> None:
        super().__init__()
        self._port      = port
        self._baud      = baud
        self._cmd_queue = cmd_queue
        self._running   = False

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
                    line = raw.decode(errors="replace").strip()
                    if line:
                        self.data_received.emit(line)
        except serial.SerialException as exc:
            self.error_occurred.emit(str(exc))
        finally:
            ser.close()

    def stop(self) -> None:
        self._running = False


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _hline() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setStyleSheet("color: #121E30;")
    return line


def _section_label(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setObjectName("section_header")
    return lbl


# ---------------------------------------------------------------------------
# Stylesheet
# ---------------------------------------------------------------------------

STYLESHEET = """
QMainWindow, QWidget#central {
    background-color: #080C14;
}

QWidget#sensor_panel {
    background-color: #0B1220;
    border: 1px solid #1A2E48;
}

QScrollArea#feed_scroll {
    background-color: #080C14;
    border: 1px solid #0E1E2C;
}

QWidget#feed_container {
    background-color: #080C14;
}

QLabel {
    color: #B0C8DC;
    font-size: 13px;
}

QLabel#title {
    color: #D8EEFF;
    font-size: 17px;
    font-weight: bold;
    letter-spacing: 3px;
}

QLabel#section_header {
    color: #1E5E7A;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 2.5px;
}

QLabel#state_label {
    color: #D8EEFF;
    font-size: 20px;
    font-weight: bold;
    letter-spacing: 3px;
}

QLabel#badge {
    border-radius: 2px;
    padding: 2px 9px;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 2px;
}

QLabel#sensor_key {
    color: #2A5870;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 1.5px;
}

QLabel#sensor_val {
    color: #00ACC1;
    font-size: 16px;
    font-weight: bold;
    font-family: "Courier New", "Menlo", monospace;
}

QLabel#sensor_unit {
    color: #1E5070;
    font-size: 10px;
    letter-spacing: 1px;
}

QComboBox {
    background-color: #0B1828;
    color: #90B8CC;
    border: 1px solid #1A3450;
    border-radius: 2px;
    padding: 4px 10px;
    font-size: 12px;
    min-width: 220px;
}
QComboBox QAbstractItemView {
    background-color: #0B1828;
    color: #90B8CC;
    selection-background-color: #14385C;
    border: 1px solid #1A3450;
}
QComboBox::drop-down { border: none; width: 20px; }

QPushButton {
    background-color: #0B1828;
    color: #6898B4;
    border: 1px solid #1A3450;
    border-radius: 2px;
    padding: 5px 12px;
    font-size: 12px;
}
QPushButton:hover {
    background-color: #102235;
    border-color: #2A6080;
    color: #A0C8E0;
}
QPushButton:pressed { background-color: #060C18; }
QPushButton:disabled {
    color: #162030;
    border-color: #0E1C2C;
    background-color: #080C14;
}

QPushButton#btn_connect {
    background-color: #071814;
    color: #00BFA5;
    border: 1px solid #007A6A;
    border-radius: 2px;
    font-weight: bold;
    letter-spacing: 1px;
    min-width: 115px;
}
QPushButton#btn_connect:hover {
    background-color: #0A2820;
    border-color: #00E5CC;
    color: #1DE9B6;
}
QPushButton#btn_connect[connected="true"] {
    background-color: #0A2820;
    color: #1DE9B6;
    border-color: #00BFA5;
}

QPushButton#btn_cmd {
    background-color: #0A1828;
    color: #6898B4;
    border: 1px solid #1A4060;
    border-radius: 2px;
    font-size: 12px;
    font-weight: bold;
    letter-spacing: 1px;
    min-height: 52px;
    min-width: 92px;
}
QPushButton#btn_cmd:hover {
    background-color: #0E2840;
    border-color: #00BCD4;
    color: #00E5FF;
}
QPushButton#btn_cmd:pressed { background-color: #060E1A; }

QPushButton#btn_exit {
    background-color: #180808;
    color: #B05060;
    border: 1px solid #6A1414;
    border-radius: 2px;
    font-size: 12px;
    font-weight: bold;
    letter-spacing: 1px;
    min-height: 52px;
    min-width: 92px;
}
QPushButton#btn_exit:hover {
    background-color: #280A0A;
    border-color: #C62828;
    color: #EF5350;
}
QPushButton#btn_exit:pressed { background-color: #0E0404; }

QPushButton#btn_terminal {
    background-color: #080C14;
    color: #1E4A62;
    border: none;
    border-top: 1px solid #0E1E2C;
    border-radius: 0px;
    padding: 7px;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 2px;
    text-align: center;
}
QPushButton#btn_terminal:hover {
    background-color: #0B1220;
    color: #2E6A8A;
}

QTextEdit#log {
    background-color: #040810;
    color: #4A7890;
    border: none;
    border-top: 1px solid #0E1E30;
    border-radius: 0px;
    padding: 8px;
    font-family: "Courier New", "Menlo", monospace;
    font-size: 12px;
}

QScrollBar:vertical {
    background: #060A12;
    width: 5px;
    border: none;
    margin: 0px;
}
QScrollBar::handle:vertical {
    background: #1A3450;
    border-radius: 2px;
    min-height: 16px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
"""


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self._worker: SerialWorker | None = None
        self._cmd_queue: queue.Queue = queue.Queue()
        self._connected = False

        self.setWindowTitle("Haptic Surgical Skill Trainer — Lab 4 Dev Interface")
        self.setMinimumSize(QSize(580, 680))

        central = QWidget()
        central.setObjectName("central")
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(24, 18, 24, 0)
        root.setSpacing(0)

        self._build_header(root)
        root.addWidget(_hline())
        root.addSpacing(14)
        self._build_connection_bar(root)
        root.addSpacing(14)
        root.addWidget(_hline())
        root.addSpacing(14)
        self._build_state_panel(root)
        root.addSpacing(12)
        self._build_activity_feed(root)
        root.addSpacing(12)
        self._build_sensor_data(root)
        root.addSpacing(14)
        root.addWidget(_hline())
        root.addSpacing(14)
        self._build_command_buttons(root)
        root.addSpacing(14)
        self._build_terminal(root)

        self._populate_ports()
        self._set_commands_enabled(False)
        self._add_activity("INTERFACE INITIALIZED · CONNECT DEVICE", "#1E5E7A")

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_header(self, root: QVBoxLayout) -> None:
        title = QLabel("HAPTIC SURGICAL SKILL TRAINER")
        title.setObjectName("title")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.addWidget(title)
        root.addSpacing(5)
        sub = QLabel("ESP32 HUZZAH32  ·  UART Echo  ·  115200 baud")
        sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sub.setStyleSheet("color: #1E4A62; font-size: 11px; letter-spacing: 1px;")
        root.addWidget(sub)
        root.addSpacing(14)

    def _build_connection_bar(self, root: QVBoxLayout) -> None:
        row = QHBoxLayout()
        row.setSpacing(10)

        row.addWidget(QLabel("Port:"))

        self._port_combo = QComboBox()
        row.addWidget(self._port_combo)

        refresh_btn = QPushButton("⟳")
        refresh_btn.setToolTip("Refresh port list")
        refresh_btn.setFixedWidth(36)
        refresh_btn.clicked.connect(self._populate_ports)
        row.addWidget(refresh_btn)

        baud_lbl = QLabel("115200")
        baud_lbl.setStyleSheet("color: #1E4A62; font-size: 11px; padding: 0 4px;")
        row.addWidget(baud_lbl)

        row.addStretch()

        self._btn_connect = QPushButton("Connect")
        self._btn_connect.setObjectName("btn_connect")
        self._btn_connect.setCheckable(False)
        self._btn_connect.clicked.connect(self._on_connect_clicked)
        row.addWidget(self._btn_connect)

        root.addLayout(row)

    def _build_state_panel(self, root: QVBoxLayout) -> None:
        root.addWidget(_section_label("TRAINER STATE"))
        root.addSpacing(8)

        row = QHBoxLayout()
        row.setAlignment(Qt.AlignmentFlag.AlignCenter)
        row.setSpacing(16)

        self._led_indicator = LedIndicator()
        row.addWidget(self._led_indicator)

        self._state_label = QLabel("STANDBY")
        self._state_label.setObjectName("state_label")
        row.addWidget(self._state_label)

        self._badge = QLabel("IDLE")
        self._badge.setObjectName("badge")
        self._badge.setStyleSheet(
            f"background-color: #0B1220; color: {STATE_COLORS[BoardState.IDLE]};"
            " border: 1px solid #1A2E48; border-radius: 2px;"
            " padding: 2px 9px; font-size: 10px; font-weight: bold; letter-spacing: 2px;"
        )
        row.addWidget(self._badge)

        root.addLayout(row)

    def _build_activity_feed(self, root: QVBoxLayout) -> None:
        root.addWidget(_section_label("RECENT ACTIVITY"))
        root.addSpacing(6)

        scroll = QScrollArea()
        scroll.setObjectName("feed_scroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setFixedHeight(172)

        feed_container = QWidget()
        feed_container.setObjectName("feed_container")

        self._feed_layout = QVBoxLayout(feed_container)
        self._feed_layout.setContentsMargins(0, 0, 0, 0)
        self._feed_layout.setSpacing(0)
        self._feed_layout.addStretch()

        scroll.setWidget(feed_container)
        root.addWidget(scroll)

    def _make_activity_card(self, message: str, timestamp: str, color: str) -> QFrame:
        card = QFrame()
        card.setObjectName("activity_card")
        card.setFixedHeight(38)
        card.setStyleSheet(f"""
            QFrame#activity_card {{
                background-color: #0B1220;
                border-left: 3px solid {color};
                border-bottom: 1px solid #0A1828;
                border-top: none;
                border-right: none;
            }}
        """)

        layout = QHBoxLayout(card)
        layout.setContentsMargins(12, 0, 14, 0)
        layout.setSpacing(10)

        dot = QLabel("●")
        dot.setStyleSheet(f"color: {color}; font-size: 9px; border: none; background: transparent;")
        dot.setFixedWidth(14)

        msg_lbl = QLabel(message)
        msg_lbl.setStyleSheet(
            "color: #7AAFC8; font-size: 11px; font-weight: bold;"
            " letter-spacing: 0.5px; border: none; background: transparent;"
        )

        ts_lbl = QLabel(timestamp)
        ts_lbl.setStyleSheet(
            "color: #1E4A62; font-size: 10px;"
            " font-family: 'Courier New', monospace; border: none; background: transparent;"
        )
        ts_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        layout.addWidget(dot)
        layout.addWidget(msg_lbl)
        layout.addStretch()
        layout.addWidget(ts_lbl)

        return card

    def _add_activity(self, message: str, color: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        card = self._make_activity_card(message, ts, color)
        self._feed_layout.insertWidget(0, card)
        # Trim oldest entries beyond limit
        while self._feed_layout.count() > MAX_FEED_CARDS + 1:
            item = self._feed_layout.takeAt(self._feed_layout.count() - 2)
            if item and item.widget():
                item.widget().deleteLater()

    def _build_sensor_data(self, root: QVBoxLayout) -> None:
        panel = QWidget()
        panel.setObjectName("sensor_panel")
        panel.setFixedHeight(74)

        inner = QHBoxLayout(panel)
        inner.setContentsMargins(16, 10, 16, 10)
        inner.setSpacing(0)

        for metric, value, unit in [
            ("FORCE L", "--", "N"),
            ("FORCE R", "--", "N"),
            ("TREMOR",  "--", "mm/s"),
        ]:
            col = QVBoxLayout()
            col.setSpacing(3)
            col.setAlignment(Qt.AlignmentFlag.AlignCenter)

            key_lbl = QLabel(metric)
            key_lbl.setObjectName("sensor_key")
            key_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

            val_row = QHBoxLayout()
            val_row.setSpacing(3)
            val_row.setAlignment(Qt.AlignmentFlag.AlignCenter)

            val_lbl = QLabel(value)
            val_lbl.setObjectName("sensor_val")
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

            unit_lbl = QLabel(unit)
            unit_lbl.setObjectName("sensor_unit")
            unit_lbl.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)

            val_row.addWidget(val_lbl)
            val_row.addWidget(unit_lbl)
            col.addWidget(key_lbl)
            col.addLayout(val_row)
            inner.addLayout(col)
            inner.addStretch()

        inner.takeAt(inner.count() - 1)
        root.addWidget(panel)

    def _build_command_buttons(self, root: QVBoxLayout) -> None:
        root.addWidget(_section_label("TRAINER COMMANDS"))
        root.addSpacing(10)

        row = QHBoxLayout()
        row.setSpacing(8)
        row.setAlignment(Qt.AlignmentFlag.AlignCenter)

        def _cmd_btn(key: str, label: str, tooltip: str, obj_name: str = "btn_cmd") -> QPushButton:
            btn = QPushButton(f"{key}\n{label}")
            btn.setObjectName(obj_name)
            btn.setToolTip(tooltip)
            btn.clicked.connect(lambda: self._send_command(key))
            btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
            return btn

        self._cmd_buttons: list[QPushButton] = []

        for key, label, tip in [
            ("I", "IDENTIFY", "Confirm connection · LED on"),
            ("E", "EASY",     "Easy training mode · slow blink · 500 ms"),
            ("H", "HARD",     "Hard training mode · fast blink · 100 ms"),
            ("S", "STOP",     "Stop active mode · return to idle"),
        ]:
            btn = _cmd_btn(key, label, tip)
            self._cmd_buttons.append(btn)
            row.addWidget(btn)

        exit_btn = _cmd_btn("X", "EXIT", "Terminate session · requires reset to resume", "btn_exit")
        self._cmd_buttons.append(exit_btn)
        row.addWidget(exit_btn)

        root.addLayout(row)

    def _build_terminal(self, root: QVBoxLayout) -> None:
        self._btn_terminal = QPushButton("▼   SHOW TERMINAL")
        self._btn_terminal.setObjectName("btn_terminal")
        self._btn_terminal.clicked.connect(self._toggle_terminal)
        root.addWidget(self._btn_terminal)

        self._terminal_container = QWidget()
        term_layout = QVBoxLayout(self._terminal_container)
        term_layout.setContentsMargins(0, 0, 0, 0)
        term_layout.setSpacing(0)

        self._log_view = QTextEdit()
        self._log_view.setObjectName("log")
        self._log_view.setReadOnly(True)
        self._log_view.setMinimumHeight(200)
        self._log_view.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        term_layout.addWidget(self._log_view)

        self._terminal_container.setVisible(False)
        root.addWidget(self._terminal_container)

    @Slot()
    def _toggle_terminal(self) -> None:
        visible = not self._terminal_container.isVisible()
        self._terminal_container.setVisible(visible)
        if visible:
            self._btn_terminal.setText("▲   HIDE TERMINAL")
            self.setMinimumHeight(900)
            if self.height() < 900:
                self.resize(self.width(), 900)
            sb = self._log_view.verticalScrollBar()
            sb.setValue(sb.maximum())
        else:
            self._btn_terminal.setText("▼   SHOW TERMINAL")
            self.setMinimumHeight(680)

    # ------------------------------------------------------------------
    # Port management  (unchanged)
    # ------------------------------------------------------------------

    @Slot()
    def _populate_ports(self) -> None:
        self._port_combo.clear()
        ports = serial.tools.list_ports.comports()
        preferred = [p for p in ports if "cu." in p.device]
        others    = [p for p in ports if "cu." not in p.device]
        for p in preferred + others:
            label = f"{p.device}"
            if p.description and p.description != "n/a":
                label += f"  —  {p.description}"
            self._port_combo.addItem(label, userData=p.device)
        if not ports:
            self._port_combo.addItem("No ports found", userData=None)

    # ------------------------------------------------------------------
    # Connection  (unchanged logic, added _add_activity calls)
    # ------------------------------------------------------------------

    @Slot()
    def _on_connect_clicked(self) -> None:
        if self._connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self._port_combo.currentData()
        if not port:
            self._log("No port selected.", "err")
            return

        self._cmd_queue = queue.Queue()
        self._worker = SerialWorker(port, 115200, self._cmd_queue)
        self._worker.data_received.connect(self._on_data_received)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.finished.connect(self._on_worker_finished)
        self._worker.start()

        self._connected = True
        self._btn_connect.setText("Disconnect")
        self._btn_connect.setProperty("connected", "true")
        self._btn_connect.style().unpolish(self._btn_connect)
        self._btn_connect.style().polish(self._btn_connect)
        self._set_commands_enabled(True)
        self._log(f"Opening {port} at 115200…", "sys")
        self._add_activity(f"CONNECTED · {port.split('/')[-1]}", "#00BCD4")

    def _disconnect(self) -> None:
        if self._worker:
            self._worker.stop()
            self._worker.wait()
            self._worker = None
        self._connected = False
        self._btn_connect.setText("Connect")
        self._btn_connect.setProperty("connected", "false")
        self._btn_connect.style().unpolish(self._btn_connect)
        self._btn_connect.style().polish(self._btn_connect)
        self._set_commands_enabled(False)
        self._update_state(BoardState.IDLE)
        self._log("Disconnected.", "sys")
        self._add_activity("DISCONNECTED · session closed", "#3D4455")

    # ------------------------------------------------------------------
    # Slots  (unchanged logic, added _add_activity calls)
    # ------------------------------------------------------------------

    @Slot(str)
    def _on_data_received(self, line: str) -> None:
        self._log(line, "rx")
        state = RESPONSE_TO_STATE.get(line)
        if state is not None:
            self._update_state(state)
            msg, color = ACTIVITY_MESSAGES[state]
            self._add_activity(msg, color)
        elif line.startswith("READY:"):
            self._update_state(BoardState.IDLE)
            self._add_activity("SYSTEM READY · awaiting commands", "#1E5E7A")

    @Slot(str)
    def _on_error(self, msg: str) -> None:
        self._log(f"Serial error: {msg}", "err")
        self._add_activity(f"ERROR · {msg[:50]}", "#EF5350")
        self._disconnect()

    @Slot()
    def _on_worker_finished(self) -> None:
        if self._connected:
            self._log("Connection lost.", "err")
            self._add_activity("CONNECTION LOST", "#EF5350")
            self._disconnect()

    # ------------------------------------------------------------------
    # Commands  (unchanged)
    # ------------------------------------------------------------------

    def _send_command(self, cmd: str) -> None:
        if not self._connected:
            return
        self._cmd_queue.put(cmd.encode())
        self._log(cmd, "tx")

    # ------------------------------------------------------------------
    # State display  (unchanged)
    # ------------------------------------------------------------------

    def _update_state(self, state: BoardState) -> None:
        self._led_indicator.set_state(state)
        self._state_label.setText(STATE_LABELS[state])
        color = STATE_COLORS[state]
        self._badge.setText(state.value)
        self._badge.setStyleSheet(
            f"background-color: #0B1220; color: {color};"
            f" border: 1px solid {color}40;"
            " border-radius: 2px; padding: 2px 9px;"
            " font-size: 10px; font-weight: bold; letter-spacing: 2px;"
        )
        if state == BoardState.EXITED:
            self._set_commands_enabled(False)

    def _set_commands_enabled(self, enabled: bool) -> None:
        for btn in self._cmd_buttons:
            btn.setEnabled(enabled)

    # ------------------------------------------------------------------
    # Log  (unchanged)
    # ------------------------------------------------------------------

    def _log(self, text: str, direction: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        if direction == "tx":
            line = f'<span style="color:#4DD0E1;">[{ts}]  &gt;&gt;  {text}</span>'
        elif direction == "rx":
            line = f'<span style="color:#26A69A;">[{ts}]  &lt;&lt;  {text}</span>'
        elif direction == "err":
            line = f'<span style="color:#EF5350;">[{ts}]  !!  {text}</span>'
        else:
            line = f'<span style="color:#263B4A;">[{ts}]  --  {text}</span>'
        self._log_view.append(line)
        sb = self._log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ------------------------------------------------------------------
    # Cleanup  (unchanged)
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        if self._worker:
            self._worker.stop()
            self._worker.wait()
        super().closeEvent(event)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLESHEET)
    font = QFont("Inter", 12)
    font.setStyleHint(QFont.StyleHint.SansSerif)
    app.setFont(font)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
