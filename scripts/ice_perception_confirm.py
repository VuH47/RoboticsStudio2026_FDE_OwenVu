#!/usr/bin/env python3
"""Same patrol and Husky confirm as ice_random_confirm, without the placement file.

Ice positions come from /fde/detection. That pose is published only after the
RGB-D node has a pale-cyan blob and a depth range that lands on a cable.
~/.ros/ice_ground_truth.json is not read.

  ros2 launch 41068_ignition_bringup ice_unified_perception.launch.py
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped

from ice_random_confirm import IceRandomConfirm, Phase


class IcePerceptionConfirm(IceRandomConfirm):
    """Random-confirm mission whose detections are perception poses."""

    def __init__(self) -> None:
        super().__init__()
        self._hits: List[Tuple[float, float, float, float]] = []
        self._warmup_start: Optional[float] = None
        self.create_subscription(
            PoseStamped, '/fde/detection', self._on_detection, 10)
        self.get_logger().info(
            'Perception confirm: /fde/detection only. '
            'ice_ground_truth.json is not read.')

    def _on_detection(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._hits.append((time.monotonic(), float(p.x), float(p.y), float(p.z)))
        if len(self._hits) > 40:
            self._hits = self._hits[-40:]

    def _already_seen(self, x: float, y: float, z: float) -> bool:
        for found in self._seen.values():
            fx, fy, fz = found['position']
            if math.sqrt((x - fx) ** 2 + (y - fy) ** 2 + (z - fz) ** 2) < 3.0:
                return True
        return False

    def _nearest_new_block(self) -> Optional[Dict[str, Any]]:
        pose = self._parrot_pose()
        if pose is None:
            return None
        px, py, pz, _yaw = pose
        rng = float(self.get_parameter('detect_range_m').value)
        now = time.monotonic()
        best: Optional[Dict[str, Any]] = None
        best_d = rng
        for stamp, x, y, z in self._hits:
            if now - stamp > 2.0 or self._already_seen(x, y, z):
                continue
            d = math.sqrt((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2)
            if d < best_d:
                best_d = d
                best = {
                    'id': 'rgb_%d_%d_%d' % (round(x), round(y), round(z)),
                    'position': [x, y, z],
                }
        return best

    def _tick(self) -> None:
        if self._phase == Phase.WAIT_TRUTH:
            if self._parrot_pose() is None:
                return
            if self._warmup_start is None:
                self._warmup_start = time.monotonic()
                self.get_logger().info('Parrot odometry is up. Warming up.')
            if time.monotonic() - self._warmup_start < 5.0:
                return
            self._phase = Phase.PATROL
            return
        super()._tick()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = IcePerceptionConfirm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
