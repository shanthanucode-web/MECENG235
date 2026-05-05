#!/usr/bin/env python3
"""
Standalone skeletal orientation/contact view for the haptic trainer.

The main GUI owns the ESP32 serial port and streams firmware JSON packets here
over stdin. This process only renders a live proof view: IMU orientation rotates
the skeletal pose, and the three instrumented fingertips light up from force or
contact. The fingers do not bend from pressure.
"""

from __future__ import annotations

import json
import sys
import threading
import time
from collections import deque

import numpy as np
from vispy import app, scene


WIN_W, WIN_H = 1180, 760
PANEL_W = 280
TIMER_INTERVAL = 1.0 / 30.0
FORCE_WARN_N = 0.8
FORCE_ERR_N = 1.5

BG_COLOR = "#f6fbff"
NAVY = "#0f2d52"
MUTED = "#5f7288"
GRID = (0.78, 0.87, 0.96, 1.0)
SKELETON = (0.20, 0.38, 0.58, 1.0)
PALM = (0.10, 0.26, 0.44, 1.0)
INACTIVE = (0.62, 0.72, 0.82, 1.0)
BLUE = (0.12, 0.50, 0.88, 1.0)
GREEN = (0.12, 0.62, 0.34, 1.0)
AMBER = (0.92, 0.56, 0.06, 1.0)
RED = (0.82, 0.10, 0.10, 1.0)

BAR_LABELS = ("Thumb", "Index", "Middle")
BAR_X0 = 46.0
BAR_X1 = PANEL_W - 44.0
BAR_W = BAR_X1 - BAR_X0
BAR_H = 18.0
BAR_Y = (582.0, 536.0, 490.0)


_BASE_LINES = {
    # Palm/wrist frame.
    "palm": np.asarray([
        [-0.36, -0.34, 0.00],
        [0.24, -0.34, 0.00],
        [0.35, -0.12, 0.02],
        [0.28, 0.16, 0.04],
        [-0.30, 0.12, 0.02],
        [-0.42, -0.08, 0.00],
        [-0.36, -0.34, 0.00],
    ], dtype=np.float32),
    "wrist": np.asarray([
        [-0.21, -0.34, 0.00],
        [-0.19, -0.58, -0.02],
        [0.17, -0.58, -0.02],
        [0.19, -0.34, 0.00],
    ], dtype=np.float32),
    # Static suturing-ready pose: thumb/index/middle gathered near a pinch area,
    # ring and pinky relaxed behind them. These coordinates never change from FSR.
    "thumb": np.asarray([
        [-0.35, -0.05, 0.00],
        [-0.56, 0.08, 0.08],
        [-0.40, 0.23, 0.16],
        [-0.18, 0.34, 0.18],
    ], dtype=np.float32),
    "index": np.asarray([
        [-0.16, 0.10, 0.02],
        [-0.14, 0.34, 0.08],
        [-0.08, 0.54, 0.14],
        [0.03, 0.68, 0.18],
    ], dtype=np.float32),
    "middle": np.asarray([
        [0.02, 0.12, 0.02],
        [0.05, 0.38, 0.07],
        [0.08, 0.60, 0.12],
        [0.12, 0.78, 0.15],
    ], dtype=np.float32),
    "ring": np.asarray([
        [0.18, 0.08, 0.00],
        [0.25, 0.34, -0.02],
        [0.31, 0.57, -0.04],
        [0.36, 0.75, -0.05],
    ], dtype=np.float32),
    "pinky": np.asarray([
        [0.30, 0.02, -0.01],
        [0.39, 0.24, -0.05],
        [0.45, 0.43, -0.08],
        [0.49, 0.58, -0.10],
    ], dtype=np.float32),
}
_FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")

_state = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "force": [0.0, 0.0, 0.0],
    "contact": [0, 0, 0],
    "warn": 0,
    "err": 0,
    "score": 0.0,
    "trainer_state": "IDLE",
    "times": deque(maxlen=40),
}
_lock = threading.Lock()
_stop = threading.Event()
_neutral = np.asarray([0.0, 0.0, 0.0], dtype=np.float32)


def _stdin_reader() -> None:
    for line in sys.stdin:
        if _stop.is_set():
            return
        try:
            packet = json.loads(line.strip())
        except json.JSONDecodeError:
            continue

        contact = packet.get("contact", [0, 0, 0])
        if not isinstance(contact, list):
            contact = [0, 0, 0]
        force = [
            float(packet.get("f0", 0.0)),
            float(packet.get("f1", 0.0)),
            float(packet.get("f2", 0.0)),
        ]
        with _lock:
            _state["roll"] = float(packet.get("roll", 0.0))
            _state["pitch"] = float(packet.get("pitch", 0.0))
            _state["yaw"] = float(packet.get("yaw", 0.0))
            _state["force"] = force
            _state["contact"] = [int(v) for v in (contact + [0, 0, 0])[:3]]
            _state["warn"] = int(packet.get("warn", 0))
            _state["err"] = int(packet.get("err", 0))
            _state["score"] = float(packet.get("score", 0.0))
            _state["trainer_state"] = str(packet.get("state", "IDLE"))
            _state["times"].append(time.monotonic())


def _rx(deg: float) -> np.ndarray:
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float32)


def _ry(deg: float) -> np.ndarray:
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float32)


def _rz(deg: float) -> np.ndarray:
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)


def _rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    # Same principle as the working reference: calculate a matrix and apply it
    # to geometry each frame. The neutral offset is only for visual centering.
    return _rz(yaw) @ _rx(pitch) @ _ry(roll)


def _transform(points: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    return (rotation @ points.T).T


def _force_color(force: float, contact: int, warn: int, err: int) -> tuple[float, float, float, float]:
    if err or force >= FORCE_ERR_N:
        return RED
    if warn or force >= FORCE_WARN_N:
        return AMBER
    if contact or force > 0.04:
        return BLUE
    return INACTIVE


def _bar_quad(y: float, value: float, full_width: bool = False) -> np.ndarray:
    width = BAR_W if full_width else max(BAR_W * float(np.clip(value, 0.0, 1.0)), 1.0)
    return np.array([
        [BAR_X0, y, 0.0],
        [BAR_X0 + width, y, 0.0],
        [BAR_X0 + width, y + BAR_H, 0.0],
        [BAR_X0, y + BAR_H, 0.0],
    ], dtype=np.float32)


def _bars_mesh(values: list[float], tracks: bool = False):
    vs, fs, cs = [], [], []
    for idx, value in enumerate(values):
        verts = _bar_quad(BAR_Y[idx], value, tracks)
        base = len(vs)
        vs.extend(verts)
        fs.extend([[base, base + 1, base + 2], [base, base + 2, base + 3]])
        color = GRID if tracks else (BLUE if idx != 2 else GREEN)
        cs.extend([color] * 4)
    return np.asarray(vs, dtype=np.float32), np.asarray(fs, dtype=np.uint32), np.asarray(cs, dtype=np.float32)


threading.Thread(target=_stdin_reader, daemon=True).start()

app.use_app("pyside6")
canvas = scene.SceneCanvas(
    title="Haptic Trainer - Skeletal Contact View",
    keys="interactive",
    size=(WIN_W, WIN_H),
    bgcolor=BG_COLOR,
    show=True,
    dpi=96,
    px_scale=1,
)
grid = canvas.central_widget.add_grid(margin=0)
view3d = grid.add_view(row=0, col=0, col_span=3)
view2d = grid.add_view(row=0, col=3, col_span=1)

view3d.camera = scene.TurntableCamera(fov=38, distance=2.0, elevation=62, azimuth=-25)
view3d.camera.center = (0.02, 0.03, 0.04)
view2d.camera = scene.PanZoomCamera(rect=(0, 0, PANEL_W, WIN_H), aspect=None)

palm_line = scene.visuals.Line(pos=_BASE_LINES["palm"], color=PALM, width=5, method="gl", parent=view3d.scene)
wrist_line = scene.visuals.Line(pos=_BASE_LINES["wrist"], color=PALM, width=4, method="gl", parent=view3d.scene)
finger_lines = {
    name: scene.visuals.Line(pos=_BASE_LINES[name], color=SKELETON, width=6 if name in ("thumb", "index", "middle") else 4, method="gl", parent=view3d.scene)
    for name in _FINGER_NAMES
}
joint_markers = scene.visuals.Markers(parent=view3d.scene)
tip_markers = scene.visuals.Markers(parent=view3d.scene)
scene.visuals.XYZAxis(parent=view3d.scene)

tv, tf, tc = _bars_mesh([1.0, 1.0, 1.0], tracks=True)
bar_tracks = scene.visuals.Mesh(vertices=tv, faces=tf, vertex_colors=tc, parent=view2d.scene)
fv, ff, fc = _bars_mesh([0.0, 0.0, 0.0])
bar_fills = scene.visuals.Mesh(vertices=fv, faces=ff, vertex_colors=fc, parent=view2d.scene)

scene.visuals.Text("CONTACT FORCE", pos=(PANEL_W / 2.0, 642), color=NAVY, font_size=13, bold=True, anchor_x="center", anchor_y="top", parent=view2d.scene)
value_texts = []
for idx, label in enumerate(BAR_LABELS):
    scene.visuals.Text(label, pos=(BAR_X0, BAR_Y[idx] + BAR_H + 12), color=MUTED, font_size=9, anchor_x="left", anchor_y="bottom", parent=view2d.scene)
    txt = scene.visuals.Text("0.000 N", pos=(BAR_X1, BAR_Y[idx] + BAR_H + 12), color=NAVY, font_size=9, bold=True, anchor_x="right", anchor_y="bottom", parent=view2d.scene)
    value_texts.append(txt)

scene.visuals.Text(
    "Skeletal suturing pose\nIMU orientation + fingertip contact\nNo pressure-driven finger curl",
    pos=(PANEL_W / 2.0, 390),
    color=MUTED,
    font_size=9,
    anchor_x="center",
    anchor_y="top",
    parent=view2d.scene,
)
scene.visuals.Line(pos=np.array([[1.0, 0.0, 0.0], [1.0, WIN_H, 0.0]], dtype=np.float32), color=GRID, width=1, parent=view2d.scene)

hud_title = scene.visuals.Text("HAPTIC TRAINER SKELETAL VIEW", pos=(18, 18), color=NAVY, font_size=14, bold=True, anchor_x="left", anchor_y="top", parent=canvas.scene)
hud_pose = scene.visuals.Text("roll --  pitch --  yaw --", pos=(18, 44), color=MUTED, font_size=9, anchor_x="left", anchor_y="top", parent=canvas.scene)
hud_status = scene.visuals.Text("Waiting for GUI telemetry", pos=(18, 64), color=MUTED, font_size=9, anchor_x="left", anchor_y="top", parent=canvas.scene)
hud_help = scene.visuals.Text("Left-drag orbit | scroll zoom | C visual-neutral", pos=(18, 84), color=MUTED, font_size=8, anchor_x="left", anchor_y="top", parent=canvas.scene)
hud_rate = scene.visuals.Text("-- packets/sec", pos=(18, WIN_H - 18), color=MUTED, font_size=9, anchor_x="left", anchor_y="bottom", parent=canvas.scene)


def on_timer(_ev) -> None:
    with _lock:
        roll = float(_state["roll"])
        pitch = float(_state["pitch"])
        yaw = float(_state["yaw"])
        force = list(_state["force"])
        contact = list(_state["contact"])
        warn = int(_state["warn"])
        err = int(_state["err"])
        score = float(_state["score"])
        trainer_state = str(_state["trainer_state"])
        times = list(_state["times"])

    adjusted = np.asarray([roll, pitch, yaw], dtype=np.float32) - _neutral
    rotation = _rotation(float(adjusted[0]), float(adjusted[1]), float(adjusted[2]))

    palm_line.set_data(pos=_transform(_BASE_LINES["palm"], rotation), color=PALM)
    wrist_line.set_data(pos=_transform(_BASE_LINES["wrist"], rotation), color=PALM)
    joint_positions = []
    tip_positions = []
    tip_colors = []
    for idx, name in enumerate(_FINGER_NAMES):
        points = _transform(_BASE_LINES[name], rotation)
        if idx < 3:
            color = _force_color(force[idx], contact[idx], warn, err)
            tip_colors.append(color)
        else:
            color = INACTIVE
        finger_lines[name].set_data(pos=points, color=color if idx < 3 else SKELETON)
        joint_positions.extend(points[:-1])
        tip_positions.append(points[-1])

    joint_markers.set_data(np.asarray(joint_positions, dtype=np.float32), face_color=BG_COLOR, edge_color=SKELETON, size=7, edge_width=1.5)
    tip_markers.set_data(np.asarray(tip_positions[:3], dtype=np.float32), face_color=np.asarray(tip_colors, dtype=np.float32), edge_color=NAVY, size=[13 + min(v / FORCE_ERR_N, 1.0) * 12 for v in force], edge_width=2.0)

    normalized = [min(v / FORCE_ERR_N, 1.0) for v in force]
    fv, ff, fc = _bars_mesh(normalized)
    bar_fills.set_data(vertices=fv, faces=ff, vertex_colors=fc)
    for idx, txt in enumerate(value_texts):
        txt.text = f"{force[idx]:.3f} N"

    hud_pose.text = f"roll {roll:+.1f}   pitch {pitch:+.1f}   yaw {yaw:+.1f}"
    hud_status.text = f"{trainer_state}   score {score:.1f}   warn 0x{warn:02X}   err 0x{err:02X}"
    if len(times) >= 2:
        dt = times[-1] - times[0]
        if dt > 0:
            hud_rate.text = f"{(len(times) - 1) / dt:.1f} packets/sec"
    canvas.update()


def on_key_press(ev) -> None:
    global _neutral
    if ev.key.name.lower() == "c":
        with _lock:
            _neutral = np.asarray([_state["roll"], _state["pitch"], _state["yaw"]], dtype=np.float32)
        hud_help.text = "Visual neutral captured | C recalibrates view only"


canvas.events.key_press.connect(on_key_press)
timer = app.Timer(interval=TIMER_INTERVAL, connect=on_timer, start=True)

print("[hand_visualizer] open - skeletal pose, no pressure-driven curl")
try:
    app.run()
finally:
    _stop.set()
