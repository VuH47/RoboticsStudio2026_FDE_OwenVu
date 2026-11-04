#!/usr/bin/env python3
"""Combined Parrot + Husky detection pop-up window (read-only viewer).

One OpenCV window, both robots side by side, status cards and an event log
underneath. It only subscribes; it never publishes and never reads ground
truth. RViz is left untouched and is not part of the client demo, so this is
the view an operator console can later embed or replace.

Subscribes:
  /fde/debug_image                 Parrot HSV blobs + depth check (ice_rgbd_detector)
  /husky1/ice_confirm/debug_image  Husky live feed + ROI/contours (husky_ice_visual_confirm)
  /fde/detection_status            none | candidate | confirmed
  /fde/detection                   PoseStamped of the accepted ice pose
  /husky1/ice_confirm/result       JSON result of the Husky RGB confirmation

Keys in the window:  q / ESC close the viewer,  s save a snapshot.
Needs a GUI-capable OpenCV and a display (WSLg). Without DISPLAY it exits with
a clear message instead of crashing.
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))
from detection_viewer_core import (  # noqa: E402
    AMBER, DashboardConfig, compose_dashboard, status_colour,
)


class DetectionViewer(Node):
    def __init__(self) -> None:
        super().__init__('detection_viewer')
        self.declare_parameter('parrot_image_topic', '/fde/debug_image')
        self.declare_parameter('husky_image_topic', '/husky1/ice_confirm/debug_image')
        self.declare_parameter('detection_status_topic', '/fde/detection_status')
        self.declare_parameter('detection_topic', '/fde/detection')
        self.declare_parameter('husky_result_topic', '/husky1/ice_confirm/result')
        self.declare_parameter('targets_topic', '/fde/ice_targets')
        self.declare_parameter('window_name', 'FDE Ice Detection - Parrot + Husky')
        self.declare_parameter('display_hz', 10.0)
        self.declare_parameter('pane_width', 480)
        self.declare_parameter('pane_height', 360)
        self.declare_parameter('stale_seconds', 3.0)
        self.declare_parameter('snapshot_dir', str(Path.home() / '.ros' / 'detection_viewer'))

        self._cfg = DashboardConfig(
            pane_w=int(self.get_parameter('pane_width').value),
            pane_h=int(self.get_parameter('pane_height').value),
            stale_s=float(self.get_parameter('stale_seconds').value),
        )
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        # Wall-clock receipt time (not sim time): staleness is about the viewer.
        self._frames: Dict[str, Optional[np.ndarray]] = {'parrot': None, 'husky': None}
        self._stamps: Dict[str, Optional[float]] = {'parrot': None, 'husky': None}
        self._det_status = ''
        self._det_pose: Optional[Tuple[float, float, float]] = None
        self._det_frame = ''
        self._husky_result: Optional[dict] = None
        self._targets_data: Optional[dict] = None
        self._targets_stamp: Optional[float] = None
        self._last_target_states: Dict[str, str] = {}

        # Event log (newest last); only status changes are recorded.
        self._events: deque = deque(maxlen=50)
        self._last_parrot: Optional[str] = None
        self._last_husky: Optional[tuple] = None
        self._last_corridor: Optional[str] = None

        self.create_subscription(
            Image, str(self.get_parameter('parrot_image_topic').value),
            lambda m: self._on_image('parrot', m), qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter('husky_image_topic').value),
            lambda m: self._on_image('husky', m), qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter('detection_status_topic').value),
            self._on_status, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('detection_topic').value),
            self._on_detection, 10)
        self.create_subscription(
            String, str(self.get_parameter('husky_result_topic').value),
            self._on_husky_result, 10)
        self.create_subscription(
            String, str(self.get_parameter('targets_topic').value),
            self._on_targets, 10)
        self.get_logger().info('Detection viewer ready (read-only; q/ESC closes, s snapshot).')

    def _log(self, msg: str, colour) -> None:
        """Append an event. Call with self._lock held."""
        self._events.append((time.strftime('%H:%M:%S'), msg, colour))

    def _on_image(self, key: str, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  #never kill the viewer on a bad frame
            self.get_logger().warn('Bad %s frame: %s' % (key, exc), throttle_duration_sec=5.0)
            return
        with self._lock:
            self._frames[key] = frame
            self._stamps[key] = time.monotonic()

    def _on_status(self, msg: String) -> None:
        s = msg.data.strip()
        with self._lock:
            self._det_status = s
            if s != self._last_parrot:
                self._last_parrot = s
                self._log('PARROT -> %s' % (s.upper() or 'WAITING'), status_colour(s))

    def _on_detection(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        with self._lock:
            self._det_pose = (p.x, p.y, p.z)
            self._det_frame = msg.header.frame_id

    def _on_husky_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            with self._lock:
                self._husky_result = data
                key = (data.get('request_id'), data.get('status'),
                       data.get('priority'))
                if key != self._last_husky:
                    self._last_husky = key
                    st = str(data.get('status', '?'))
                    priority = data.get('priority')
                    if priority:
                        self._log('HUSKY %s  %s' % (
                            str(priority).upper(),
                            data.get('priority_reason') or ''),
                            status_colour(str(priority)))
                    else:
                        self._log('HUSKY -> %s  %s  (%s/%s frames)' % (
                            st.upper(), data.get('request_id', ''),
                            data.get('positive_frames', '?'),
                            data.get('valid_frames', '?')), status_colour(st))

    def _on_targets(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        now = time.monotonic()
        with self._lock:
            self._targets_data = data
            self._targets_stamp = now
            notice = (data.get('husky') or {}).get('corridor')
            if notice and notice != self._last_corridor:
                self._last_corridor = notice
                self._log(str(notice), AMBER)
            for t in data.get('targets', []):
                tid = t.get('id')
                state = t.get('state')
                if tid and state and self._last_target_states.get(tid) != state:
                    self._last_target_states[tid] = state
                    span = t.get('span') or ''
                    txt = f"{tid} ({span}) -> {state.upper()}" if span else f"{tid} -> {state.upper()}"
                    self._log(txt, status_colour(state))

    def render(self) -> np.ndarray:
        now = time.monotonic()
        with self._lock:
            parrot = self._frames['parrot']
            husky = self._frames['husky']
            ages = {k: (None if t is None else now - t) for k, t in self._stamps.items()}
            det_status = self._det_status
            det_pose = self._det_pose
            det_frame = self._det_frame
            husky_result = None if self._husky_result is None else dict(self._husky_result)
            events = list(self._events)
            targets = None
            if self._targets_data is not None:
                targets = dict(self._targets_data)
                age = (now - self._targets_stamp) if self._targets_stamp else 999.0
                targets['stale'] = age > self._cfg.stale_s
        return compose_dashboard(
            parrot, husky, ages['parrot'], ages['husky'],
            parrot_status=det_status,
            parrot_pose=det_pose,
            parrot_frame_id=det_frame,
            husky_result=husky_result,
            events=events,
            targets=targets,
            cfg=self._cfg,
        )

    def save_snapshot(self, image: np.ndarray) -> Optional[Path]:
        out_dir = Path(str(self.get_parameter('snapshot_dir').value)).expanduser()
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
            path = out_dir / ('detection_%s.png' % time.strftime('%Y%m%d_%H%M%S'))
            return path if cv2.imwrite(str(path), image) else None
        except OSError as exc:
            self.get_logger().error('Snapshot failed: %s' % exc)
            return None


def _window_closed(name: str) -> bool:
    """True once the user closes the window with the X button.

    The GTK OpenCV build under WSLg returns -1 for WND_PROP_VISIBLE even while
    the window is open, so use ASPECT_RATIO: positive while open, NaN once gone.
    """
    try:
        ratio = cv2.getWindowProperty(name, cv2.WND_PROP_ASPECT_RATIO)
    except cv2.error:
        return True
    return not (ratio > 0)


def main(args: Optional[List[str]] = None) -> None:
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('detection_viewer: no DISPLAY/WAYLAND_DISPLAY - cannot open a window. '
              'Run inside WSLg or set DISPLAY.', file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=args)
    node = DetectionViewer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def _spin() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass  # Ctrl-C / launch shutdown: normal stop

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    window = str(node.get_parameter('window_name').value)
    period_ms = max(1, int(1000.0 / max(1.0, float(node.get_parameter('display_hz').value))))
    try:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        first = node.render()
        cv2.resizeWindow(window, first.shape[1], first.shape[0])
        while rclpy.ok():
            frame = node.render()
            cv2.imshow(window, frame)
            key = cv2.waitKey(period_ms) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                saved = node.save_snapshot(frame)
                node.get_logger().info('Snapshot saved: %s' % saved)
            # Window closed with the X button.
            if _window_closed(window):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
