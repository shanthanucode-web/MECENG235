"""
ESP32 LED Controller GUI
PySide6 host-side application for the uart_echo_VitalSignsLab4 project.

Communicates with the ESP32 over USB-serial (115200 baud).
Supported commands: I (Init), E (Easy), H (Hard), S (Stop), X (Exit)
"""

import queue
import sys
from datetime import datetime
from enum import Enum

import serial
import serial.tools.list_ports
from PySide6.QtCore import (
    QPropertyAnimation,
    QThread,
    QEasingCurve,
    Signal,
    Slot,
    Qt,
    QSize,
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
    BoardState.IDLE:      "#555566",
    BoardState.CONNECTED: "#00E676",
    BoardState.EASY:      "#C6FF00",
    BoardState.HARD:      "#FF6D00",
    BoardState.EXITED:    "#F44336",
}

STATE_LABELS: dict[BoardState, str] = {
    BoardState.IDLE:      "IDLE",
    BoardState.CONNECTED: "CONNECTED",
    BoardState.EASY:      "EASY MODE",
    BoardState.HARD:      "HARD MODE",
    BoardState.EXITED:    "EXITED",
}

RESPONSE_TO_STATE: dict[str, BoardState] = {
    "CONNECTED":    BoardState.CONNECTED,
    "EASY":         BoardState.EASY,
    "HARD":         BoardState.HARD,
    "IDLE":         BoardState.IDLE,
    "ALREADY IDLE": BoardState.CONNECTED,
    "EXITED":       BoardState.EXITED,
}


# ---------------------------------------------------------------------------
# LED indicator widget
# ---------------------------------------------------------------------------

class LedIndicator(QWidget):
    """Painted circular indicator that pulses in EASY / HARD mode."""

    SIZE = 22

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFixedSize(self.SIZE, self.SIZE)
        self._state = BoardState.IDLE
        self._glow = 0.0           # 0.0 – 1.0, animated in EASY/HARD

        self._anim = QPropertyAnimation(self, b"glow_level", self)
        self._anim.setEasingCurve(QEasingCurve.Type.SineCurve)
        self._anim.setStartValue(0.15)
        self._anim.setEndValue(1.0)
        self._anim.setLoopCount(-1)

    # Qt property for animation
    def _get_glow(self) -> float:
        return self._glow

    def _set_glow(self, value: float) -> None:
        self._glow = value
        self.update()

    glow_level = property(_get_glow, _set_glow)

    # Make it a Qt property so QPropertyAnimation works
    from PySide6.QtCore import Property as _Prop
    glow_level = _Prop(float, _get_glow, _set_glow)

    def set_state(self, state: BoardState) -> None:
        self._state = state
        self._anim.stop()
        if state == BoardState.EASY:
            self._anim.setDuration(900)
            self._anim.start()
        elif state == BoardState.HARD:
            self._anim.setDuration(200)
            self._anim.start()
        else:
            self._glow = 1.0
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        color = QColor(STATE_COLORS[self._state])
        cx, cy, r = self.SIZE / 2, self.SIZE / 2, self.SIZE / 2 - 2

        # Glow halo
        if self._state in (BoardState.EASY, BoardState.HARD):
            halo = QRadialGradient(cx, cy, r * 1.8)
            glow_color = QColor(color)
            glow_color.setAlphaF(self._glow * 0.35)
            halo.setColorAt(0.0, glow_color)
            halo.setColorAt(1.0, QColor(0, 0, 0, 0))
            painter.setBrush(halo)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(int(cx - r * 1.8), int(cy - r * 1.8),
                                int(r * 3.6), int(r * 3.6))

        # Main circle
        painter.setPen(QPen(color.darker(150), 1))
        fill_color = QColor(color)
        if self._state in (BoardState.EASY, BoardState.HARD):
            fill_color.setAlphaF(0.4 + self._glow * 0.6)
        painter.setBrush(fill_color)
        painter.drawEllipse(int(cx - r), int(cy - r), int(r * 2), int(r * 2))

        # Specular highlight
        highlight = QColor(255, 255, 255, 80)
        painter.setBrush(highlight)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(int(cx - r * 0.45), int(cy - r * 0.55),
                            int(r * 0.5), int(r * 0.4))
        painter.end()


# ---------------------------------------------------------------------------
# Serial worker thread
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
                # Drain outgoing commands first
                while not self._cmd_queue.empty():
                    try:
                        ser.write(self._cmd_queue.get_nowait())
                    except queue.Empty:
                        break

                # Read one line (returns b'' on timeout)
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
# Separator helper
# ---------------------------------------------------------------------------

def _hline() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setStyleSheet("color: #2A2A3F;")
    return line


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

STYLESHEET = """
QMainWindow, QWidget#central {
    background-color: #1E1E2E;
}

QLabel {
    color: #CDD6F4;
    font-size: 13px;
}

QLabel#title {
    color: #CDD6F4;
    font-size: 18px;
    font-weight: bold;
    letter-spacing: 1px;
}

QLabel#state_label {
    color: #CDD6F4;
    font-size: 22px;
    font-weight: bold;
    letter-spacing: 2px;
}

QLabel#badge {
    border-radius: 6px;
    padding: 2px 10px;
    font-size: 11px;
    font-weight: bold;
    letter-spacing: 1px;
}

QComboBox {
    background-color: #2D2D44;
    color: #CDD6F4;
    border: 1px solid #45475A;
    border-radius: 6px;
    padding: 4px 10px;
    font-size: 13px;
    min-width: 220px;
}
QComboBox QAbstractItemView {
    background-color: #2D2D44;
    color: #CDD6F4;
    selection-background-color: #5C6BC0;
}
QComboBox::drop-down {
    border: none;
    width: 20px;
}

QPushButton {
    background-color: #2D2D44;
    color: #CDD6F4;
    border: 1px solid #45475A;
    border-radius: 8px;
    padding: 6px 14px;
    font-size: 13px;
}
QPushButton:hover {
    background-color: #3D3D60;
    border-color: #5C6BC0;
}
QPushButton:pressed {
    background-color: #1A1A30;
}
QPushButton:disabled {
    color: #585B70;
    border-color: #313244;
    background-color: #1E1E2E;
}

QPushButton#btn_connect {
    background-color: #2D2D44;
    color: #A6E3A1;
    border-color: #A6E3A1;
    font-weight: bold;
    min-width: 110px;
}
QPushButton#btn_connect:hover {
    background-color: #1B5E20;
    border-color: #A6E3A1;
}
QPushButton#btn_connect[connected="true"] {
    background-color: #1B5E20;
    color: #A6E3A1;
}

QPushButton#btn_cmd {
    background-color: #31314D;
    color: #CDD6F4;
    border-color: #5C6BC0;
    font-size: 14px;
    font-weight: bold;
    min-height: 44px;
    min-width: 90px;
    border-radius: 10px;
}
QPushButton#btn_cmd:hover {
    background-color: #5C6BC0;
    color: #FFFFFF;
}

QPushButton#btn_exit {
    background-color: #2D1B1B;
    color: #F38BA8;
    border-color: #B71C1C;
    font-size: 14px;
    font-weight: bold;
    min-height: 44px;
    min-width: 90px;
    border-radius: 10px;
}
QPushButton#btn_exit:hover {
    background-color: #B71C1C;
    color: #FFFFFF;
}

QTextEdit#log {
    background-color: #12121C;
    color: #BAC2DE;
    border: 1px solid #2A2A3F;
    border-radius: 8px;
    padding: 8px;
    font-family: "Menlo", "Courier New", monospace;
    font-size: 12px;
}
"""


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self._worker: SerialWorker | None = None
        self._cmd_queue: queue.Queue = queue.Queue()
        self._connected = False

        self.setWindowTitle("ESP32 LED Controller")
        self.setMinimumSize(QSize(560, 620))

        central = QWidget()
        central.setObjectName("central")
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(0)

        self._build_header(root)
        root.addWidget(_hline())
        root.addSpacing(14)
        self._build_connection_bar(root)
        root.addSpacing(14)
        root.addWidget(_hline())
        root.addSpacing(16)
        self._build_state_panel(root)
        root.addSpacing(16)
        root.addWidget(_hline())
        root.addSpacing(16)
        self._build_command_buttons(root)
        root.addSpacing(16)
        root.addWidget(_hline())
        root.addSpacing(10)
        self._build_log(root)

        self._populate_ports()
        self._set_commands_enabled(False)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_header(self, root: QVBoxLayout) -> None:
        title = QLabel("ESP32  LED  CONTROLLER")
        title.setObjectName("title")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.addWidget(title)
        root.addSpacing(4)
        sub = QLabel("uart_echo_VitalSignsLab4  ·  Core 0: UART  ·  Core 1: LED")
        sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sub.setStyleSheet("color: #585B70; font-size: 11px; letter-spacing: 0.5px;")
        root.addWidget(sub)
        root.addSpacing(14)

    def _build_connection_bar(self, root: QVBoxLayout) -> None:
        row = QHBoxLayout()
        row.setSpacing(10)

        port_lbl = QLabel("Port:")
        row.addWidget(port_lbl)

        self._port_combo = QComboBox()
        row.addWidget(self._port_combo)

        refresh_btn = QPushButton("⟳")
        refresh_btn.setToolTip("Refresh port list")
        refresh_btn.setFixedWidth(36)
        refresh_btn.clicked.connect(self._populate_ports)
        row.addWidget(refresh_btn)

        baud_lbl = QLabel("115200")
        baud_lbl.setStyleSheet("color: #585B70; font-size: 12px; padding: 0 4px;")
        row.addWidget(baud_lbl)

        row.addStretch()

        self._btn_connect = QPushButton("Connect")
        self._btn_connect.setObjectName("btn_connect")
        self._btn_connect.setCheckable(False)
        self._btn_connect.clicked.connect(self._on_connect_clicked)
        row.addWidget(self._btn_connect)

        root.addLayout(row)

    def _build_state_panel(self, root: QVBoxLayout) -> None:
        row = QHBoxLayout()
        row.setAlignment(Qt.AlignmentFlag.AlignCenter)
        row.setSpacing(14)

        self._led_indicator = LedIndicator()
        row.addWidget(self._led_indicator)

        self._state_label = QLabel("IDLE")
        self._state_label.setObjectName("state_label")
        row.addWidget(self._state_label)

        self._badge = QLabel("IDLE")
        self._badge.setObjectName("badge")
        self._badge.setStyleSheet(
            f"background-color: #2D2D44; color: {STATE_COLORS[BoardState.IDLE]};"
            " border-radius: 6px; padding: 2px 10px; font-size: 11px; font-weight: bold;"
        )
        row.addWidget(self._badge)

        root.addLayout(row)

    def _build_command_buttons(self, root: QVBoxLayout) -> None:
        lbl = QLabel("COMMANDS")
        lbl.setStyleSheet("color: #585B70; font-size: 11px; letter-spacing: 1px;")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.addWidget(lbl)
        root.addSpacing(10)

        row = QHBoxLayout()
        row.setSpacing(10)
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
            ("I", "INIT",  "Confirm connection · LED on"),
            ("E", "EASY",  "Slow blink · 500 ms"),
            ("H", "HARD",  "Fast blink · 100 ms"),
            ("S", "STOP",  "Stop blinking · return to idle"),
        ]:
            btn = _cmd_btn(key, label, tip)
            self._cmd_buttons.append(btn)
            row.addWidget(btn)

        exit_btn = _cmd_btn("X", "EXIT", "Terminate firmware · requires reset", "btn_exit")
        self._cmd_buttons.append(exit_btn)
        row.addWidget(exit_btn)

        root.addLayout(row)

    def _build_log(self, root: QVBoxLayout) -> None:
        lbl = QLabel("SERIAL LOG")
        lbl.setStyleSheet("color: #585B70; font-size: 11px; letter-spacing: 1px;")
        root.addWidget(lbl)
        root.addSpacing(6)

        self._log_view = QTextEdit()
        self._log_view.setObjectName("log")
        self._log_view.setReadOnly(True)
        self._log_view.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        root.addWidget(self._log_view)

    # ------------------------------------------------------------------
    # Port management
    # ------------------------------------------------------------------

    @Slot()
    def _populate_ports(self) -> None:
        self._port_combo.clear()
        ports = serial.tools.list_ports.comports()
        # On macOS prefer cu.* over tty.*
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
    # Connection
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

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------

    @Slot(str)
    def _on_data_received(self, line: str) -> None:
        self._log(line, "rx")
        state = RESPONSE_TO_STATE.get(line)
        if state is not None:
            self._update_state(state)
        elif line.startswith("READY:"):
            self._update_state(BoardState.IDLE)

    @Slot(str)
    def _on_error(self, msg: str) -> None:
        self._log(f"Serial error: {msg}", "err")
        self._disconnect()

    @Slot()
    def _on_worker_finished(self) -> None:
        if self._connected:
            self._log("Connection lost.", "err")
            self._disconnect()

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def _send_command(self, cmd: str) -> None:
        if not self._connected:
            return
        self._cmd_queue.put(cmd.encode())
        self._log(cmd, "tx")

    # ------------------------------------------------------------------
    # State display
    # ------------------------------------------------------------------

    def _update_state(self, state: BoardState) -> None:
        self._led_indicator.set_state(state)
        self._state_label.setText(STATE_LABELS[state])
        color = STATE_COLORS[state]
        self._badge.setText(state.value)
        self._badge.setStyleSheet(
            f"background-color: #2D2D44; color: {color};"
            " border-radius: 6px; padding: 2px 10px;"
            " font-size: 11px; font-weight: bold; letter-spacing: 1px;"
        )
        if state == BoardState.EXITED:
            self._set_commands_enabled(False)

    def _set_commands_enabled(self, enabled: bool) -> None:
        for btn in self._cmd_buttons:
            btn.setEnabled(enabled)

    # ------------------------------------------------------------------
    # Log
    # ------------------------------------------------------------------

    def _log(self, text: str, direction: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        if direction == "tx":
            line = f'<span style="color:#89DCEB;">[{ts}]  &gt;&gt;  {text}</span>'
        elif direction == "rx":
            line = f'<span style="color:#A6E3A1;">[{ts}]  &lt;&lt;  {text}</span>'
        elif direction == "err":
            line = f'<span style="color:#F38BA8;">[{ts}]  !!  {text}</span>'
        else:
            line = f'<span style="color:#585B70;">[{ts}]  --  {text}</span>'
        self._log_view.append(line)
        sb = self._log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ------------------------------------------------------------------
    # Cleanup
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
    font = QFont("Inter", 13)
    font.setStyleHint(QFont.StyleHint.SansSerif)
    app.setFont(font)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
