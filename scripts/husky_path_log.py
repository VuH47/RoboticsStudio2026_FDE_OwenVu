#!/usr/bin/env python3
"""Save the Husky camera at the look for each ice the Parrot reported.

No path drawing. One PNG when the Husky reaches the viewing pose and is
asked to look, and one PNG when that look finishes. The file name is the
hazard and the result.
"""

import os
import re
from pathlib import Path
from typing import Optional

_TRACK = re.compile(r'(T\d+)', re.IGNORECASE)


def track_from_request(request_id: str) -> str:
    match = _TRACK.search(str(request_id or ''))
    if match:
        return match.group(1).upper()
    text = str(request_id or '').strip()
    return text or 'hazard'


def look_filename(request_id: str) -> str:
    """Name for the frame at the viewing pose, before the look finishes."""
    return '%s_under_determination.png' % track_from_request(request_id)


def result_filename(request_id: str, status: str) -> str:
    """Name for the frame when the look finishes."""
    track = track_from_request(request_id)
    if str(status or '').strip().upper() == 'CONFIRMED':
        return '%s_verified.png' % track
    return '%s_cannot_verify.png' % track


def _default_output_root() -> str:
    """Save inside the package's image/path_log directory portably."""
    file_path = globals().get('__file__')
    if file_path:
        curr = Path(file_path).resolve()
        # 1. Walk up parents to find 'image' (works when running directly from source)
        for p in curr.parents:
            cand = p / 'image'
            if cand.is_dir():
                return str(cand / 'path_log')
            # If running from install/ or build/, locate the source workspace
            if p.name in ('install', 'build'):
                ws = p.parent
                for pkg_name in ('MinhandOwen', '41068_ignition_bringup'):
                    src_cand = ws / 'src' / pkg_name / 'image'
                    if src_cand.is_dir():
                        return str(src_cand / 'path_log')

    # 2. Check current working directory and subdirectories
    cwd = Path.cwd()
    if (cwd / 'image').is_dir():
        return str(cwd / 'image' / 'path_log')
    if (cwd / 'src' / 'MinhandOwen' / 'image').is_dir():
        return str(cwd / 'src' / 'MinhandOwen' / 'image' / 'path_log')

    # 3. Known workspace locations on this system
    for known in (
        Path('/home/vu_pc1/41068_ws/src/MinhandOwen/image'),
        Path('/home/vu_pc1/41068_ws/src/41068_ignition_bringup/image'),
    ):
        if known.is_dir():
            return str(known / 'path_log')

    return os.path.join(os.getcwd(), 'image', 'path_log')


def _default_output_root() -> str:
    """Save inside the package's image/path_log directory portably."""
    file_path = globals().get('__file__')
    if file_path:
        curr = Path(file_path).resolve()
        # 1. Walk up parents to find 'image' (works when running directly from source)
        for p in curr.parents:
            cand = p / 'image'
            if cand.is_dir():
                return str(cand / 'path_log')
            # If running from install/ or build/, locate the source workspace
            if p.name in ('install', 'build'):
                ws = p.parent
                for pkg_name in ('MinhandOwen', '41068_ignition_bringup'):
                    src_cand = ws / 'src' / pkg_name / 'image'
                    if src_cand.is_dir():
                        return str(src_cand / 'path_log')

    # 2. Check current working directory and subdirectories
    cwd = Path.cwd()
    if (cwd / 'image').is_dir():
        return str(cwd / 'image' / 'path_log')
    if (cwd / 'src' / 'MinhandOwen' / 'image').is_dir():
        return str(cwd / 'src' / 'MinhandOwen' / 'image' / 'path_log')

    # 3. Known workspace locations on this system
    for known in (
        Path('/home/vu_pc1/41068_ws/src/MinhandOwen/image'),
        Path('/home/vu_pc1/41068_ws/src/41068_ignition_bringup/image'),
    ):
        if known.is_dir():
            return str(known / 'path_log')

    return os.path.join(os.getcwd(), 'image', 'path_log')


import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover
    CvBridge = None


class HuskyPathLog(Node):
    def __init__(self) -> None:
        super().__init__('husky_path_log')
        self.declare_parameter('output_dir', '')
        self._bridge = CvBridge() if CvBridge is not None else None
        self._image = None
        self._pending_look: Optional[str] = None
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        root = str(self.get_parameter('output_dir').value).strip() or _default_output_root()
        self._dir = os.path.join(root, stamp)
        os.makedirs(self._dir, exist_ok=True)
        self.create_subscription(
            Image, '/husky1/camera/image', self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            String, '/husky1/ice_confirm/request', self._on_request, 10)
        self.create_subscription(
            String, '/husky1/ice_confirm/result', self._on_result, 10)
        self.get_logger().info('Husky look images: %s' % self._dir)

    def _on_image(self, msg: Image) -> None:
        self._image = msg
        if self._pending_look is not None:
            name = self._pending_look
            self._pending_look = None
            self._save(name)

    def _on_request(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        filename = look_filename(str(data.get('request_id') or ''))
        if self._image is None:
            self._pending_look = filename
            return
        self._save(filename)

    def _on_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        filename = result_filename(
            str(data.get('request_id') or ''), str(data.get('status') or ''))
        if self._image is None:
            self._pending_look = filename
            return
        self._save(filename)

    def _save(self, filename: str) -> None:
        if self._bridge is None or self._image is None:
            self.get_logger().warn('No Husky camera frame for %s' % filename)
            return
        import cv2
        frame = self._bridge.imgmsg_to_cv2(self._image, desired_encoding='bgr8')
        path = os.path.join(self._dir, filename)
        cv2.imwrite(path, frame)
        self.get_logger().info('Saved %s' % path)


def main() -> None:
    rclpy.init()
    node = HuskyPathLog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
