#!/usr/bin/env python3
"""Same patrol and Husky confirm as ice_random_confirm, without the placement file.

Ice positions come from /fde/detection. That pose is published only after the
RGB-D node has a pale-cyan blob and a depth range that lands on a cable.
~/.ros/ice_ground_truth.json is not read.

Concurrent mode (default): the Parrot keeps patrolling while the Husky works.
Detections are merged into averaged targets (ice_target_tracker), queued
nearest-first along the Husky lane, and published as JSON on /fde/ice_targets
for the viewer. concurrent_patrol:=false restores the old serial behaviour.

  ros2 launch 41068_ignition_bringup ice_unified_perception.launch.py
"""

from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from ice_random_confirm import IceRandomConfirm, Phase
from ice_target_tracker import (
    ACTIVE, CONFIRMED, INCONCLUSIVE, MISSED, NOT_CONFIRMED, TargetTracker)


class IcePerceptionConfirm(IceRandomConfirm):
    """Random-confirm mission whose detections are perception poses."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('concurrent_patrol', True)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('merge_radius_m', 3.0)
        self.declare_parameter('hit_max_range_m', 10.0)
        self.declare_parameter('targets_topic', '/fde/ice_targets')
        self._concurrent = bool(self.get_parameter('concurrent_patrol').value)
        self._tracker = TargetTracker(
            merge_radius=float(self.get_parameter('merge_radius_m').value),
            min_hits=int(self.get_parameter('min_hits').value))
        self._hit_max_range = float(self.get_parameter('hit_max_range_m').value)
        self._hits: List[Tuple[float, float, float, float]] = []
        self._warmup_start: Optional[float] = None
        self._targets_pub = self.create_publisher(
            String, str(self.get_parameter('targets_topic').value), 10)
        self.create_timer(0.5, self._publish_targets)
        self.create_subscription(
            PoseStamped, '/fde/detection', self._on_detection, 10)
        self.get_logger().info(
            'Perception confirm: /fde/detection only (%s). '
            'ice_ground_truth.json is not read.'
            % ('concurrent patrol, queued targets' if self._concurrent
               else 'serial patrol'))

    # ---- detections -------------------------------------------------------
    def _on_detection(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        x, y, z = float(p.x), float(p.y), float(p.z)
        now = time.monotonic()
        self._hits.append((now, x, y, z))
        if len(self._hits) > 40:
            self._hits = self._hits[-40:]
        if not self._concurrent:
            return
        pose = self._parrot_pose()
        if pose is None:
            return
        d = math.sqrt((pose[0] - x) ** 2 + (pose[1] - y) ** 2
                      + (pose[2] - z) ** 2)
        if d > self._hit_max_range:
            return
        before = len(self._tracker.tracks)
        tr = self._tracker.add_hit(now, x, y, z)
        if len(self._tracker.tracks) > before:
            self.get_logger().info(
                'New ice target %s near (%.1f, %.1f, %.1f)' % (tr.id, x, y, z))

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

    # ---- concurrent mission ----------------------------------------------
    def _hold_parrot(self) -> None:
        if not self._concurrent:
            super()._hold_parrot()

    def _patrol_done(self) -> bool:
        return self._patrol_index >= len(self._patrol)

    def _lane_y(self) -> float:
        pose = self._husky_pose()
        hy = pose[1] if pose is not None else 0.0
        return self._approach_y if hy < -2.0 else 0.0

    def _husky_xy(self) -> Tuple[float, float]:
        pose = self._husky_pose()
        return (pose[0], pose[1]) if pose is not None else (0.0, 0.0)

    def _patrol_step(self) -> None:
        if self._patrol_done():
            self._stop_parrot()
            return
        label, tx, ty, tz, tyaw = self._patrol[self._patrol_index]
        if self._fly_toward(tx, ty, tz, tyaw):
            self.get_logger().info('Patrol reached %s.' % label)
            self._patrol_index += 1

    def _dispatch_next(self) -> bool:
        tr = self._tracker.pick_next(
            self._husky_xy(), self._lane_y(), final=self._patrol_done())
        if tr is None:
            return False
        x, y, z = tr.position
        block = {
            'id': 'rgb_%d_%d_%d' % (round(x), round(y), round(z)),
            'position': [x, y, z],
        }
        self._active = self._note_detection(block)
        self._active['track'] = tr.id
        self._tracker.set_state(tr.id, ACTIVE)
        self.get_logger().info(
            'Husky target %s (%d hits, spread %.1f m, %d more queued).'
            % (tr.id, tr.n_hits, tr.spread,
               len(self._tracker.queue_order(
                   self._husky_xy(), self._lane_y(), True))))
        self._hover = None
        self._goals = self._goals_for(x)
        self._saw_in_camera = False
        self._goal_rejects = 0
        self._phase = Phase.CONFIRM
        self._send_next_goal()
        return True

    def _finish_confirm(self, outcome: Optional[str] = None) -> None:
        active = self._active
        super()._finish_confirm(outcome)
        if not self._concurrent or active is None or self._active is not None:
            return
        tid = active.get('track')
        if tid is None:
            return
        state = {
            'confirmed': CONFIRMED,
            'not_confirmed': NOT_CONFIRMED,
            'inconclusive': INCONCLUSIVE,
        }.get(active.get('state'), MISSED)
        self._tracker.set_state(tid, state, active.get('state'))

    def _mission_summary(self) -> None:
        counts: Dict[str, int] = {}
        for tr in self._tracker.tracks:
            counts[tr.state] = counts.get(tr.state, 0) + 1
        self.get_logger().info(
            'Mission finished. targets=%d %s'
            % (len(self._tracker.tracks), counts))

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
        if not self._concurrent or self._phase == Phase.DONE:
            super()._tick()
            return
        self._patrol_step()
        if self._phase == Phase.PATROL:
            if not self._dispatch_next() and self._patrol_done() \
                    and self._tracker.unfinished_count() == 0:
                self._mission_summary()
                self._phase = Phase.DONE
            return
        super()._tick()

    # ---- monitoring feed --------------------------------------------------
    def _publish_targets(self) -> None:
        active = self._active.get('track') if self._active else None
        data = {
            'concurrent': self._concurrent,
            'parrot': {
                'patrol_index': min(self._patrol_index, len(self._patrol)),
                'patrol_total': len(self._patrol),
                'patrol_done': self._patrol_done(),
            },
            'husky': {'phase': self._phase.name, 'active': active},
            'targets': [],
        }
        for t in self._tracker.snapshot(self._husky_xy(), self._lane_y()):
            t['span'] = self._span_of(t['x'], t['y'], t['z'])
            data['targets'].append(t)
        self._targets_pub.publish(String(data=json.dumps(data)))


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
