#!/usr/bin/env python3
"""Operator console (read-only) for the Parrot + Husky ice inspection mission.

PySide6 GUI. ROS runs in a background executor thread and only fills a
lock-protected state; the Qt timer reads snapshots at ~10 Hz. This node never
publishes and never reads ground truth.

Subscribes:
  /fde/debug_image                 Parrot detector debug video
  /husky1/ice_confirm/debug_image  Husky confirm video
  /fde/detection_status            none | candidate | confirmed
  /fde/detection                   PoseStamped of the accepted ice pose
  /husky1/ice_confirm/result       JSON Husky confirmation result
  /fde/ice_targets                 JSON target queue + patrol/Husky phase
  /parrot1/odometry, /husky1/odometry   telemetry and map

Buttons other than Snapshot are placeholders (disabled) for future wiring.
Install:  pip install PySide6     Run:  ros2 run 41068_ignition_bringup operator_console.py
"""
from __future__ import annotations

import ctypes
import json
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def _ensure_immortal_none() -> None:
    """Compensate for PySide6 6.12.0 bug which decrements None refcount on void/enum Qt methods."""
    try:
        if sys.getrefcount(None) < 500000:
            for _ in range(100000):
                ctypes.pythonapi.Py_IncRef(ctypes.py_object(None))
    except Exception:
        pass

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError:
    try:
        from PyQt5 import QtCore, QtGui, QtWidgets
    except ImportError:
        from PySide2 import QtCore, QtGui, QtWidgets

# ------------------------------------------------------------------ palette
BG = '#181a1e'
PANEL = '#22252b'
TITLE = '#2d3138'
TEXT = '#eceff1'
MUTED = '#8d949e'
GREEN = '#50c850'
AMBER = '#ffbe00'
RED = '#e63c3c'
BLUE = '#3ca0e6'
ORANGE = '#ff8c3c'
GREY = '#6b727c'

STATE_COLOURS = {
    'confirmed': GREEN, 'candidate': AMBER, 'inconclusive': AMBER,
    'not_confirmed': RED, 'none': RED, 'active': BLUE, 'ready': AMBER,
    'tracking': GREY, 'missed': GREY,
}


def state_colour(s: str) -> str:
    return STATE_COLOURS.get((s or '').strip().lower(), GREY)


STYLE = """
QWidget { background: %(BG)s; color: %(TEXT)s; font-family: 'DejaVu Sans'; font-size: 12px; }
QFrame#card, QFrame#panel { background: %(PANEL)s; border: 1px solid #343941; }
QLabel#h1 { font-size: 18px; font-weight: bold; }
QLabel#h2 { font-size: 13px; font-weight: bold; color: %(MUTED)s; }
QPushButton { background: #2f343c; border: 1px solid #454b55; padding: 7px; font-weight: bold; border-radius: 4px; }
QPushButton:hover { background: #3a414b; }
QPushButton:disabled { color: #666c75; background: #25282e; }

QPushButton#btn_start {
    background: #1b3a24;
    border: 2px solid #2e7d32;
    color: #81c784;
    padding: 8px;
    font-size: 13px;
    font-weight: bold;
}
QPushButton#btn_start:hover {
    background: #2e7d32;
    color: #ffffff;
}

QPushButton#btn_rtb {
    background: #3e2712;
    border: 2px solid #e65100;
    color: #ffb74d;
    padding: 8px;
    font-size: 13px;
    font-weight: bold;
}
QPushButton#btn_rtb:hover {
    background: #e65100;
    color: #ffffff;
}

QPushButton#btn_estop {
    background: #4a1515;
    border: 2px solid #c62828;
    color: #ef9a9a;
    padding: 8px;
    font-size: 13px;
    font-weight: bold;
}
QPushButton#btn_estop:hover {
    background: #c62828;
    color: #ffffff;
}

QPushButton#btn_snapshot {
    background: #272b33;
    border: 1px solid #454b55;
    color: %(TEXT)s;
    padding: 6px;
    font-size: 12px;
}
QPushButton#btn_snapshot:hover {
    background: #37474f;
    color: #ffffff;
}

QTableWidget { background: %(PANEL)s; gridline-color: #343941; border: none; }
QHeaderView::section { background: %(TITLE)s; color: %(MUTED)s; border: none; padding: 4px; }
""" % dict(BG=BG, TEXT=TEXT, PANEL=PANEL, MUTED=MUTED, TITLE=TITLE)


# --------------------------------------------------------------- ROS thread
class RosState(Node):
    def __init__(self) -> None:
        super().__init__('operator_console')
        self.declare_parameter('parrot_image_topic', '/fde/debug_image')
        self.declare_parameter('husky_image_topic', '/husky1/ice_confirm/debug_image')
        self.declare_parameter('snapshot_dir', str(Path.home() / '.ros' / 'operator_console'))
        self.declare_parameter('stale_seconds', 3.0)
        g = lambda n: str(self.get_parameter(n).value)  # noqa: E731

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames: Dict[str, Optional[np.ndarray]] = {'parrot': None, 'husky': None}
        self.frame_stamp: Dict[str, Optional[float]] = {'parrot': None, 'husky': None}
        self.frame_seq = {'parrot': 0, 'husky': 0}
        self.det_status = ''
        self.det_pose: Optional[Tuple[float, float, float]] = None
        self.det_frame = ''
        self.husky_result: Optional[dict] = None
        self.targets: dict = {}
        self.odom: Dict[str, Optional[dict]] = {'parrot': None, 'husky': None}
        self.last_rx = 0.0
        self.events: deque = deque(maxlen=200)
        self._last_parrot: Optional[str] = None
        self._last_husky: Optional[tuple] = None
        self._last_states: Dict[str, str] = {}
        self._last_phase: Optional[str] = None

        self.cmd_pub = self.create_publisher(String, '/fde/mission_command', 10)
        self.husky_cmd = self.create_publisher(Twist, '/husky1/cmd_vel', 10)
        self.parrot_cmd = self.create_publisher(Twist, '/parrot1/cmd_vel', 10)
        self.last_debug_time: Dict[str, float] = {'parrot': 0.0, 'husky': 0.0}

        # Annotated debug streams (detector & confirm)
        self.create_subscription(Image, g('parrot_image_topic'),
                                 lambda m: self._on_debug_image('parrot', m), qos_profile_sensor_data)
        self.create_subscription(Image, g('husky_image_topic'),
                                 lambda m: self._on_debug_image('husky', m), qos_profile_sensor_data)

        # Raw camera live fallbacks
        self.create_subscription(Image, '/parrot1/camera/image',
                                 lambda m: self._on_raw_image('parrot', m), qos_profile_sensor_data)
        self.create_subscription(Image, '/husky1/camera/image',
                                 lambda m: self._on_raw_image('husky', m), qos_profile_sensor_data)

        self.create_subscription(String, '/fde/detection_status', self._on_status, 10)
        self.create_subscription(PoseStamped, '/fde/detection', self._on_detection, 10)
        self.create_subscription(String, '/husky1/ice_confirm/result', self._on_husky_result, 10)
        self.create_subscription(String, '/fde/ice_targets', self._on_targets, 10)
        self.create_subscription(Odometry, '/parrot1/odometry',
                                 lambda m: self._on_odom('parrot', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/husky1/odometry',
                                 lambda m: self._on_odom('husky', m), qos_profile_sensor_data)
        self.get_logger().info('Operator console ready with active mission controls.')

    def _log(self, source: str, text: str, colour: str) -> None:
        """Call with self.lock held."""
        self.events.append((time.strftime('%H:%M:%S'), source, text, colour))

    def log_event(self, source: str, text: str, colour: str) -> None:
        with self.lock:
            self._log(source, text, colour)

    def send_mission_command(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info('Dispatched mission command: %s' % cmd)

    def emergency_halt(self) -> None:
        stop = Twist()
        for _ in range(5):
            self.husky_cmd.publish(stop)
            self.parrot_cmd.publish(stop)
        self.send_mission_command('estop')

    def _on_debug_image(self, key: str, msg: Image) -> None:
        self.last_debug_time[key] = time.monotonic()
        self._on_image(key, msg)

    def _on_raw_image(self, key: str, msg: Image) -> None:
        if time.monotonic() - self.last_debug_time[key] > 1.5:
            self._on_image(key, msg)

    def _on_image(self, key: str, msg: Image) -> None:
        try:
            if not msg or not msg.data or msg.width <= 0 or msg.height <= 0:
                return
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            enc = str(getattr(msg, 'encoding', '') or '').lower()
            if 'rgb' in enc:
                rgb_arr = raw.reshape((msg.height, msg.width, 3))
                frame = np.ascontiguousarray(rgb_arr[:, :, ::-1])
            elif 'bgr' in enc:
                frame = raw.reshape((msg.height, msg.width, 3))
            elif self.bridge is not None:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                frame = raw.reshape((msg.height, msg.width, 3))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('Bad %s frame: %s' % (key, exc), throttle_duration_sec=5.0)
            return
        with self.lock:
            self.frames[key] = frame
            self.frame_stamp[key] = time.monotonic()
            self.frame_seq[key] += 1
            self.last_rx = time.monotonic()

    def _on_status(self, msg: String) -> None:
        s = msg.data.strip()
        with self.lock:
            self.det_status = s
            self.last_rx = time.monotonic()
            if s != self._last_parrot:
                self._last_parrot = s
                self._log('PARROT', '-> %s' % (s.upper() or 'WAITING'), state_colour(s))

    def _on_detection(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        with self.lock:
            self.det_pose = (p.x, p.y, p.z)
            self.det_frame = msg.header.frame_id

    def _on_husky_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        with self.lock:
            self.husky_result = data
            self.last_rx = time.monotonic()
            key = (data.get('request_id'), data.get('status'), data.get('priority'))
            if key != self._last_husky:
                self._last_husky = key
                st = str(data.get('status', '?'))
                priority = data.get('priority')
                if st == 'CONFIRMED' and priority:
                    self._log('HUSKY', 'DETECTED reported ice %s: %s' % (
                        str(priority).upper(), data.get('priority_reason') or ''),
                        '#ff4444' if str(priority).upper() == 'HIGH' else state_colour(st))
                else:
                    self._log('HUSKY', '-> %s  %s  (%s/%s frames)' % (
                        st.upper(), data.get('request_id', ''),
                        data.get('positive_frames', '?'), data.get('valid_frames', '?')),
                        state_colour(st))

    def _on_targets(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        with self.lock:
            self.targets = data
            self.last_rx = time.monotonic()
            phase = str((data.get('husky') or {}).get('phase', ''))
            if phase and phase != self._last_phase:
                self._last_phase = phase
                self._log('HUSKY', 'phase %s' % phase, MUTED)
            for t in data.get('targets', []):
                tid, st = str(t.get('id', '?')), str(t.get('state', ''))
                if self._last_states.get(tid) != st:
                    if tid not in self._last_states:
                        self._log('TARGET', '%s new (%s)' % (tid, st), state_colour(st))
                    else:
                        self._log('TARGET', '%s -> %s' % (tid, st.upper()), state_colour(st))
                    self._last_states[tid] = st

    def _on_odom(self, key: str, msg: Odometry) -> None:
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        with self.lock:
            self.odom[key] = {
                'x': p.x, 'y': p.y, 'z': p.z,
                'speed': math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2),
            }
            self.last_rx = time.monotonic()

    def snapshot(self) -> dict:
        with self.lock:
            return {
                'frames': dict(self.frames), 'stamps': dict(self.frame_stamp),
                'seq': dict(self.frame_seq), 'det_status': self.det_status,
                'det_pose': self.det_pose, 'det_frame': self.det_frame,
                'husky_result': dict(self.husky_result) if self.husky_result else None,
                'targets': dict(self.targets), 'odom': dict(self.odom),
                'events': list(self.events), 'last_rx': self.last_rx,
            }


# ------------------------------------------------------------------ widgets
def badge_css(colour: str) -> str:
    return ('background:%s; color:#141414; font-weight:bold; padding:2px 8px;'
            'border:none;' % colour)


class ImagePane(QtWidgets.QFrame):
    def __init__(self, title: str, accent: str) -> None:
        super().__init__()
        self.setObjectName('panel')
        self._qimg: Optional[QtGui.QImage] = None
        self._overlay = 'NO SIGNAL'
        self._dim = False
        self._seq = -1
        bar = QtWidgets.QFrame()
        bar.setStyleSheet('background:%s; border:none; border-left:5px solid %s;' % (TITLE, accent))
        bl = QtWidgets.QHBoxLayout(bar)
        bl.setContentsMargins(10, 4, 8, 4)
        t = QtWidgets.QLabel(title)
        t.setStyleSheet('font-weight:bold; font-size:13px; border:none;')
        self.tag = QtWidgets.QLabel('NO SIGNAL')
        bl.addWidget(t)
        bl.addStretch(1)
        bl.addWidget(self.tag)
        self.view = QtWidgets.QLabel()
        self.view.setAlignment(QtCore.Qt.AlignCenter)
        self.view.setMinimumSize(1, 1)
        self.view.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.view.setStyleSheet('background:%s; border:none;' % BG)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(bar)
        lay.addWidget(self.view, 1)

    def update_state(self, frame: Optional[np.ndarray], seq: int,
                     age: Optional[float], stale_s: float) -> None:
        if frame is None or age is None:
            self.tag.setText('NO SIGNAL')
            self.tag.setStyleSheet(badge_css(GREY))
            self._qimg, self._overlay, self._dim = None, 'NO SIGNAL', False
            self._render()
            return
        stale = age > stale_s
        self.tag.setText('STALE %.0fs' % age if stale else 'LIVE')
        self.tag.setStyleSheet(badge_css(AMBER if stale else GREEN))
        self._overlay = 'STALE  %.0fs' % age if stale else ''
        changed = seq != self._seq or stale != self._dim
        self._dim = stale
        if seq != self._seq:
            self._seq = seq
            rgb = np.ascontiguousarray(frame[:, :, ::-1])
            h, w = rgb.shape[:2]
            self._qimg = QtGui.QImage(rgb.data, w, h, 3 * w,
                                      QtGui.QImage.Format_RGB888).copy()
        if changed:
            self._render()

    def _render(self) -> None:
        size = self.view.size()
        if self._qimg is None:
            self.view.setPixmap(QtGui.QPixmap())
            self.view.setText(self._overlay)
            return
        pm = QtGui.QPixmap.fromImage(self._qimg).scaled(
            size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        if self._dim:
            p = QtGui.QPainter(pm)
            p.fillRect(pm.rect(), QtGui.QColor(0, 0, 0, 140))
            p.setPen(QtGui.QColor(AMBER))
            f = p.font()
            f.setPointSize(18)
            f.setBold(True)
            p.setFont(f)
            p.drawText(pm.rect(), QtCore.Qt.AlignCenter, self._overlay)
            p.end()
        self.view.setText('')
        self.view.setPixmap(pm)

    def resizeEvent(self, e: QtGui.QResizeEvent) -> None:  # noqa: N802
        super().resizeEvent(e)
        self._render()


class Card(QtWidgets.QFrame):
    def __init__(self, title: str, accent: str) -> None:
        super().__init__()
        self.setObjectName('card')
        self.setFixedHeight(72)
        self._title = QtWidgets.QLabel(title)
        self._title.setStyleSheet('font-weight:bold; font-size:13px; color:%s; border:none;' % accent)
        self._badge = QtWidgets.QLabel('STANDBY')
        self._detail = QtWidgets.QLabel('')
        self._detail.setStyleSheet('color:%s; border:none;' % MUTED)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(self._title)
        top.addStretch(1)
        top.addWidget(self._badge)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(14, 8, 10, 8)
        lay.addLayout(top)
        lay.addWidget(self._detail)
        self.set('STANDBY', GREY, '')

    def set(self, badge: str, colour: str, detail: str) -> None:
        self._badge.setText(badge)
        self._badge.setStyleSheet(badge_css(colour))
        self._detail.setText(detail)
        self.setStyleSheet('QFrame#card { background:%s; border:1px solid #343941;'
                           'border-left:6px solid %s; }' % (PANEL, colour))


class MapWidget(QtWidgets.QWidget):
    X0, X1, Y0, Y1 = -30.0, 90.0, -40.0, 10.0   # world extent in metres

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(190)
        self.parrot: Optional[dict] = None
        self.husky: Optional[dict] = None
        self.targets: List[dict] = []

    def set_data(self, parrot, husky, targets) -> None:
        self.parrot, self.husky, self.targets = parrot, husky, targets
        self.update()

    def paintEvent(self, _e) -> None:  # noqa: N802
        p = QtGui.QPainter(self)
        try:
            p.setRenderHint(QtGui.QPainter.Antialiasing)
            w, h = self.width(), self.height()
            p.fillRect(self.rect(), QtGui.QColor('#14161a'))
            
            # 1. Swap aspect ratio: width maps Y (50m span), height maps X (120m span)
            s = min(w / (self.Y1 - self.Y0), h / (self.X1 - self.X0))
            ox = (w - s * (self.Y1 - self.Y0)) / 2
            oy = (h - s * (self.X1 - self.X0)) / 2

            # 2. Vertical projection:
            #    Screen X = lateral offset (Y1 - y) -> Lane on left, Cable on right
            #    Screen Y = longitudinal progress (X) -> Bottom (-30) to Top (+90)
            def px(x: float, y: float) -> QtCore.QPointF:
                return QtCore.QPointF(
                    ox + (self.Y1 - y) * s,
                    h - oy - (x - self.X0) * s
                )

            # 3. Grid lines
            p.setPen(QtGui.QPen(QtGui.QColor('#262a31'), 1))
            for gx in range(int(self.X0), int(self.X1) + 1, 10):
                p.drawLine(px(gx, self.Y0), px(gx, self.Y1))   # horizontal grid rungs
            for gy in range(int(self.Y0), int(self.Y1) + 1, 10):
                p.drawLine(px(self.X0, gy), px(self.X1, gy))   # vertical grid tracks

            # 4. Vertical lanes and cables (Line 488)
            dash = QtGui.QPen(QtGui.QColor(MUTED), 1, QtCore.Qt.DashLine)
            p.setPen(dash)
            p.drawLine(px(20, -30), px(80, -30))               # Vertical cable line
            p.drawLine(px(-30, 0), px(90, 0))                   # Vertical lane y=0
            p.drawLine(px(-30, -10), px(90, -10))               # Vertical lane y=-10

            p.setPen(QtGui.QColor(MUTED))
            # Text labels placed alongside the vertical lines
            pt_cable = px(50, -30)
            p.drawText(int(pt_cable.x() + 6), int(pt_cable.y()), 'cable')
            pt_lane0 = px(10, 0)
            p.drawText(int(pt_lane0.x() - 60), int(pt_lane0.y()), 'lane y=0')
            pt_lane10 = px(10, -10)
            p.drawText(int(pt_lane10.x() + 6), int(pt_lane10.y()), 'lane y=-10')

            # 5. Targets and robot markers
            for t in self.targets:
                try:
                    pt = px(float(t['x']), float(t['y']))
                except (KeyError, TypeError, ValueError):
                    continue
                c = QtGui.QColor(state_colour(str(t.get('state', ''))))
                p.setBrush(c)
                p.setPen(QtCore.Qt.NoPen)
                p.drawEllipse(pt, 5, 5)

            if self.husky:
                pt = px(self.husky['x'], self.husky['y'])
                p.setBrush(QtGui.QColor(ORANGE))
                p.setPen(QtCore.Qt.NoPen)
                p.drawRect(QtCore.QRectF(pt.x() - 6, pt.y() - 6, 12, 12))

            if self.parrot:
                pt = px(self.parrot['x'], self.parrot['y'])
                # Triangle points forward (+X / UP on screen)
                tri = QtGui.QPolygonF([
                    QtCore.QPointF(pt.x(), pt.y() - 8),
                    QtCore.QPointF(pt.x() - 7, pt.y() + 6),
                    QtCore.QPointF(pt.x() + 7, pt.y() + 6)
                ])
                p.setBrush(QtGui.QColor(BLUE))
                p.setPen(QtCore.Qt.NoPen)
                p.drawPolygon(tri)
        finally:
            p.end()


def section(title: str) -> QtWidgets.QLabel:
    lab = QtWidgets.QLabel(title)
    lab.setObjectName('h2')
    return lab


# --------------------------------------------------------------- main window
class Console(QtWidgets.QMainWindow):
    def __init__(self, node: RosState) -> None:
        super().__init__()
        self.node = node
        self.stale_s = float(node.get_parameter('stale_seconds').value)
        self._event_count = -1
        self.setWindowTitle('FDE Operator Console')
        self.resize(1500, 900)

        # header
        self.mission = QtWidgets.QLabel('Mission: waiting for data')
        self.link = QtWidgets.QLabel('NO DATA')
        self.clock = QtWidgets.QLabel('')
        title = QtWidgets.QLabel('FDE OS')
        title.setObjectName('h1')
        head = QtWidgets.QFrame()
        head.setObjectName('panel')
        hl = QtWidgets.QHBoxLayout(head)
        hl.addWidget(title)
        hl.addSpacing(24)
        hl.addWidget(self.mission)
        hl.addStretch(1)
        hl.addWidget(self.link)
        hl.addSpacing(12)
        hl.addWidget(self.clock)

        # sidebar: map, telemetry, controls
        self.map = MapWidget()
        self.tele = {k: QtWidgets.QLabel('-') for k in ('parrot', 'husky')}
        for k, lab in self.tele.items():
            lab.setStyleSheet('border:none;')
        mp = QtWidgets.QFrame()
        mp.setObjectName('panel')
        ml = QtWidgets.QVBoxLayout(mp)
        ml.addWidget(section('2D Overview'))
        ml.addWidget(self.map, 1)
        tp = QtWidgets.QFrame()
        tp.setObjectName('panel')
        tl = QtWidgets.QVBoxLayout(tp)
        tl.addWidget(section('Device Telementry'))
        for k, c, n in (('parrot', BLUE, 'UAV'), ('husky', ORANGE, 'UGV')):
            lab = QtWidgets.QLabel(n)
            lab.setStyleSheet('color:%s; font-weight:bold; border:none;' % c)
            tl.addWidget(lab)
            tl.addWidget(self.tele[k])
        cp = QtWidgets.QFrame()
        cp.setObjectName('panel')
        cl = QtWidgets.QVBoxLayout(cp)
        cl.addWidget(section('MISSION CONTROLS'))
        self.btn_start = QtWidgets.QPushButton('Start Mission')
        self.btn_start.setObjectName('btn_start')
        self.btn_start.setCursor(QtCore.Qt.PointingHandCursor)
        self.btn_start.clicked.connect(self.start_mission)

        self.btn_rtb = QtWidgets.QPushButton('Return Home')
        self.btn_rtb.setObjectName('btn_rtb')
        self.btn_rtb.setCursor(QtCore.Qt.PointingHandCursor)
        self.btn_rtb.clicked.connect(self.return_home)

        self.btn_estop = QtWidgets.QPushButton('Emergency Stop')
        self.btn_estop.setObjectName('btn_estop')
        self.btn_estop.setCursor(QtCore.Qt.PointingHandCursor)
        self.btn_estop.clicked.connect(self.estop)

        self.btn_snapshot = QtWidgets.QPushButton('Snapshot')
        self.btn_snapshot.setObjectName('btn_snapshot')
        self.btn_snapshot.setCursor(QtCore.Qt.PointingHandCursor)
        self.btn_snapshot.clicked.connect(self.save_snapshot)

        cl.addWidget(self.btn_start)
        cl.addWidget(self.btn_rtb)
        cl.addWidget(self.btn_estop)
        cl.addWidget(self.btn_snapshot)
        side = QtWidgets.QVBoxLayout()
        side.addWidget(mp, 3)
        side.addWidget(tp, 0)
        side.addWidget(cp, 0)
        side_w = QtWidgets.QWidget()
        side_w.setLayout(side)
        side_w.setFixedWidth(340)

        # main column
        self.p_parrot = ImagePane('Ariel Inspection', BLUE)
        self.p_husky = ImagePane('Ground-level view and Identification', ORANGE)
        panes = QtWidgets.QHBoxLayout()
        panes.addWidget(self.p_parrot, 1)
        panes.addWidget(self.p_husky, 1)
        self.c_parrot = Card('Air', BLUE)
        self.c_husky = Card('Ground', ORANGE)
        cards = QtWidgets.QHBoxLayout()
        cards.addWidget(self.c_parrot, 1)
        cards.addWidget(self.c_husky, 1)

        self.queue = QtWidgets.QTableWidget(0, 6)
        self.queue.setHorizontalHeaderLabels(['Target', 'Span', 'State', 'Hits', 'Dist (m)', 'Position'])
        self.log = QtWidgets.QTableWidget(0, 3)
        self.log.setHorizontalHeaderLabels(['Time', 'Source', 'Event'])
        for tbl in (self.queue, self.log):
            tbl.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            tbl.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
            tbl.verticalHeader().setVisible(False)
            tbl.horizontalHeader().setStretchLastSection(True)
        qf = QtWidgets.QFrame()
        qf.setObjectName('panel')
        ql = QtWidgets.QVBoxLayout(qf)
        ql.addWidget(section('TARGET QUEUE'))
        ql.addWidget(self.queue)
        lf = QtWidgets.QFrame()
        lf.setObjectName('panel')
        ll = QtWidgets.QVBoxLayout(lf)
        ll.addWidget(section('EVENT LOG'))
        ll.addWidget(self.log)
        bottom = QtWidgets.QHBoxLayout()
        bottom.addWidget(qf, 1)
        bottom.addWidget(lf, 1)

        main = QtWidgets.QVBoxLayout()
        main.addLayout(panes, 5)
        main.addLayout(cards, 0)
        main.addLayout(bottom, 3)

        body = QtWidgets.QHBoxLayout()
        body.addWidget(side_w)
        body.addLayout(main, 1)
        root = QtWidgets.QVBoxLayout()
        root.addWidget(head)
        root.addLayout(body, 1)
        cw = QtWidgets.QWidget()
        cw.setLayout(root)
        self.setCentralWidget(cw)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def save_snapshot(self) -> None:
        out = Path(str(self.node.get_parameter('snapshot_dir').value)).expanduser()
        try:
            out.mkdir(parents=True, exist_ok=True)
            path = out / ('console_%s.png' % time.strftime('%Y%m%d_%H%M%S'))
            ok = self.grab().save(str(path))
            self.node.get_logger().info('Snapshot %s: %s' % ('saved' if ok else 'FAILED', path))
        except OSError as exc:
            self.node.get_logger().error('Snapshot failed: %s' % exc)

    def start_mission(self) -> None:
        self.node.send_mission_command('start')
        self.node.log_event('CONSOLE', 'Operator commanded: START MISSION', GREEN)

    def return_home(self) -> None:
        self.node.send_mission_command('return_home')
        self.node.log_event('CONSOLE', 'Operator commanded: RETURN HOME', AMBER)

    def estop(self) -> None:
        self.node.emergency_halt()
        self.node.log_event('CONSOLE', 'EMERGENCY STOP ACTIVATED', RED)

    @staticmethod
    def _item(text: str, colour: Optional[str] = None) -> QtWidgets.QTableWidgetItem:
        it = QtWidgets.QTableWidgetItem(text)
        if colour:
            it.setForeground(QtGui.QColor(colour))
        return it

    def refresh(self) -> None:
        _ensure_immortal_none()
        s = self.node.snapshot()
        now = time.monotonic()
        self.clock.setText(time.strftime('%H:%M:%S'))
        online = s['last_rx'] > 0 and now - s['last_rx'] < self.stale_s
        self.link.setText('ONLINE' if online else 'NO DATA')
        self.link.setStyleSheet(badge_css(GREEN if online else GREY))

        for key, pane in (('parrot', self.p_parrot), ('husky', self.p_husky)):
            t = s['stamps'][key]
            pane.update_state(s['frames'][key], s['seq'][key],
                              None if t is None else now - t, self.stale_s)

        # status cards
        st = s['det_status']
        detail = 'Waiting for detection'
        if s['det_pose'] is not None:
            detail = 'Pose  x %.1f   y %.1f   z %.1f' % s['det_pose']
            if s['det_frame']:
                detail += '   [%s]' % s['det_frame']
        self.c_parrot.set((st or 'WAITING').upper(), state_colour(st), detail)
        r = s['husky_result']
        if not r:
            self.c_husky.set('STANDBY', GREY, 'No request yet')
        else:
            status = str(r.get('status', '?'))
            priority = str(r.get('priority') or '')
            d = '%s   %s/%s positive frames' % (
                r.get('request_id', ''), r.get('positive_frames', '?'), r.get('valid_frames', '?'))
            if priority:
                d = '%s  %s' % (priority.upper(), r.get('priority_reason') or d)
                self.c_husky.set(priority.upper(),
                                 '#ff4444' if priority.upper() == 'HIGH' else state_colour(status),
                                 d)
            else:
                if r.get('reason'):
                    d += '   (%s)' % r['reason']
                self.c_husky.set(status.upper(), state_colour(status), d)

        # header mission line, telemetry, map
        tg = s['targets']
        parrot_info, husky_info = tg.get('parrot') or {}, tg.get('husky') or {}
        if tg:
            self.mission.setText('Husky: %s   |   Parrot patrol %s/%s%s' % (
                husky_info.get('phase', '?'), parrot_info.get('patrol_index', '?'),
                parrot_info.get('patrol_total', '?'),
                '  (done)' if parrot_info.get('patrol_done') else ''))
        o = s['odom']
        for k in ('parrot', 'husky'):
            v = o[k]
            self.tele[k].setText('-' if v is None else
                                 'x %.1f  y %.1f  Alt %.1f m   Speed %.1f m/s' % (
                                     v['x'], v['y'], v['z'], v['speed']))
        targets = tg.get('targets', []) if tg else []
        self.map.set_data(o['parrot'], o['husky'], targets)

        # target queue
        self.queue.setRowCount(len(targets))
        for i, t in enumerate(targets):
            state = str(t.get('state', ''))
            if t.get('priority'):
                state = '%s %s' % (state, str(t.get('priority')).upper())
            hits = t.get('n_hits', t.get('hits', '-'))
            dist = t.get('dist_m', t.get('dist', t.get('distance')))
            pos = '(%.1f, %.1f, %.1f)' % (t.get('x', 0.0), t.get('y', 0.0), t.get('z', 0.0)) \
                if all(k in t for k in ('x', 'y', 'z')) else '-'
            vals = [str(t.get('id', '?')), str(t.get('span', '-')), state.upper(), str(hits),
                    '-' if dist is None else '%.1f' % float(dist), pos]
            for c, v in enumerate(vals):
                self.queue.setItem(i, c, self._item(v, state_colour(state) if c == 2 else None))

        # event log (newest first), rebuild only when it changes
        ev = s['events']
        if len(ev) != self._event_count:
            self._event_count = len(ev)
            shown = ev[::-1][:100]
            self.log.setRowCount(len(shown))
            for i, (ts, src, txt, col) in enumerate(shown):
                self.log.setItem(i, 0, self._item(ts, MUTED))
                self.log.setItem(i, 1, self._item(src))
                self.log.setItem(i, 2, self._item(txt, col))


def main(args: Optional[List[str]] = None) -> None:
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('operator_console: no DISPLAY/WAYLAND_DISPLAY - cannot open a window. '
              'Run inside WSLg or set DISPLAY.', file=sys.stderr)
        sys.exit(1)

    _ensure_immortal_none()
    rclpy.init(args=args)
    node = RosState()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def _spin() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    win = Console(node)
    win.show()
    try:
        if hasattr(app, 'exec'):
            app.exec()
        else:
            app.exec_()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
