#!/usr/bin/env python3
"""Husky : Nav2 short cable loop on ski corridor (sim already up).

Prereq:
  ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \\
    world:=snow_forest rviz:=true slam:=true nav2:=true

Then:
  ros2 launch 41068_ignition_bringup husky_nav2_cable_tour.launch.py

Ground-safe XY only (inspect at y=0). Do not run husky_ice_inspect at the same time.
Pairs with Parrot Phase 2 parrot_nav2_cable_tour (aerial offset vs corridor inspect).

Clearance / corridor checks use husky1_map → base_link TF (same frame as Nav2
goals). Gazebo /odometry is logged for diagnostics only — SLAM map can drift
from world, so odom must not gate pass/fail (ISSUE 16).
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from ice_cable_geometry import (  # noqa: E402
    corridor_violation,
    husky_nav2_tour_xy,
    min_cable_clearance,
    min_cable_clearance_xy,
)

Waypoint = Tuple[str, float, float, float]  # label, x, y, yaw


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class HuskyNav2CableTour(Node):
    def __init__(self) -> None:
        super().__init__('husky_nav2_cable_tour')

        self.declare_parameter('robot_name', 'husky1')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('odom_topic', 'odometry')
        self.declare_parameter('goal_pause_seconds', 1.5)
        self.declare_parameter('goal_timeout_seconds', 180.0)

        robot = self.get_parameter('robot_name').get_parameter_value().string_value
        self.map_frame = f'{robot}_map'
        self.base_frame = f'{robot}_base_link'
        self.waypoints: List[Waypoint] = husky_nav2_tour_xy()
        self.index = 0
        self.goal_active = False
        self._done = False
        self.next_goal_time = 0.0
        self.last_feedback_log = 0.0
        self.results: List[str] = []
        self._view_clearance_xy_m: Optional[float] = None
        self._view_clearance_3d_m: Optional[float] = None
        self._corridor_ok: Optional[bool] = None
        self._map_xy: Optional[Tuple[float, float]] = None
        self._odom: Optional[Odometry] = None
        self._goal_sent_at = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter('navigate_action').value,
        )
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            'Husky Phase 2 Nav2 cable tour ready in %s (pose check via TF %s→%s). '
            'Waypoints: %s (corridor inspect; no alpine y).'
            % (
                self.map_frame,
                self.map_frame,
                self.base_frame,
                [w[0] for w in self.waypoints],
            )
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _lookup_map_xy(self) -> Optional[Tuple[float, float, float]]:
        """Robot x,y,z in map frame (Nav2 goal frame)."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except Exception as exc:
            self.get_logger().warn('TF %s→%s failed: %s' % (
                self.map_frame, self.base_frame, exc))
            return None
        t = transform.transform.translation
        return (t.x, t.y, t.z)

    def _tick(self) -> None:
        if self._done or self.goal_active:
            return
        if time.monotonic() < self.next_goal_time:
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().info(
                'Waiting for Nav2 navigate_to_pose...',
                throttle_duration_sec=5.0,
            )
            return
        if self.index >= len(self.waypoints):
            self._finish()
            return
        self._send_waypoint(self.waypoints[self.index])

    def _send_waypoint(self, wp: Waypoint) -> None:
        label, x, y, yaw = wp
        qx, qy, qz, qw = yaw_to_quat(yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.goal_active = True
        self._goal_sent_at = time.monotonic()
        self.get_logger().info(
            '[%d/%d] Navigating to %s (%.1f, %.1f) yaw=%.2f in %s'
            % (self.index + 1, len(self.waypoints), label, x, y, yaw, self.map_frame)
        )
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected goal — will retry.')
            self.goal_active = False
            self.next_goal_time = time.monotonic() + 3.0
            return
        self.get_logger().info('Goal accepted.')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result)

    def _feedback(self, feedback_msg) -> None:
        now = time.monotonic()
        timeout = float(self.get_parameter('goal_timeout_seconds').value)
        if now - self._goal_sent_at > timeout:
            self.get_logger().warn(
                'Goal timeout (%.0fs) — still waiting on Nav2 result.'
                % timeout
            )
            self._goal_sent_at = now
        if now - self.last_feedback_log < 5.0:
            return
        self.last_feedback_log = now
        dist = getattr(feedback_msg.feedback, 'distance_remaining', None)
        if dist is not None:
            self.get_logger().info('  … %.1f m remaining' % dist)

    def _goal_result(self, future) -> None:
        status = future.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        label = self.waypoints[self.index][0]
        if ok:
            self.results.append('WP %s: PASS' % label)
            self.get_logger().info('Reached %s.' % label)
            if label == 'span01_view':
                self._record_view_metrics()
            self.index += 1
        else:
            self.results.append('WP %s: FAIL status=%s' % (label, status))
            self.get_logger().warn(
                'Failed %s (status=%s). Retrying same waypoint.'
                % (label, status)
            )
        self.goal_active = False
        self.next_goal_time = time.monotonic() + float(
            self.get_parameter('goal_pause_seconds').value
        )

    def _record_view_metrics(self) -> None:
        """Clearance + corridor from map TF; odom logged for drift compare."""
        map_pose = self._lookup_map_xy()
        odom_xy = None
        if self._odom is not None:
            op = self._odom.pose.pose.position
            odom_xy = (op.x, op.y, op.z)

        if map_pose is None and odom_xy is not None:
            self.get_logger().warn(
                'No map TF — falling back to Gazebo odometry for metrics '
                '(may disagree with Nav2 map; see ISSUE 16).'
            )
            x, y, z = odom_xy
            source = 'odometry'
        elif map_pose is not None:
            x, y, z = map_pose
            source = 'tf_map'
        else:
            self.get_logger().error('No map TF and no odometry — cannot score view.')
            self._corridor_ok = False
            return

        self._map_xy = (x, y)
        d_xy, span, cable = min_cable_clearance_xy(x, y)
        d_3d, _, _ = min_cable_clearance(x, y, z)
        self._view_clearance_xy_m = d_xy
        self._view_clearance_3d_m = d_3d
        self._corridor_ok = not corridor_violation(y)

        odom_note = ''
        if odom_xy is not None and source == 'tf_map':
            odom_note = (
                ' odom_xy=(%.2f, %.2f) map_vs_odom_dy=%.2f'
                % (odom_xy[0], odom_xy[1], y - odom_xy[1])
            )

        self.get_logger().info(
            'span01_view clearance_xy=%.2f m clearance_3d=%.2f m (%s/%s) '
            'corridor_ok=%s pose_source=%s map_xy=(%.2f, %.2f)%s'
            % (
                d_xy, d_3d, span, cable, self._corridor_ok, source, x, y, odom_note,
            )
        )

    def _finish(self) -> None:
        expected = [w[0] for w in self.waypoints]
        passed_labels = set()
        for r in self.results:
            if r.endswith('PASS'):
                # 'WP span01_approach: PASS'
                passed_labels.add(r[3:].rsplit(':', 1)[0].strip())
        all_wp_ok = all(lab in passed_labels for lab in expected)
        summary = {
            'phase': 2,
            'robot': 'husky',
            'mode': 'nav2_corridor',
            'waypoints': self.results,
            'view_clearance_xy_m': self._view_clearance_xy_m,
            'view_clearance_3d_m': self._view_clearance_3d_m,
            'corridor_ok': self._corridor_ok,
            'map_xy_at_view': self._map_xy,
            'pass': all_wp_ok and self._corridor_ok is not False,
        }
        self.get_logger().info('TOUR SUMMARY %s' % json.dumps(summary))
        self._done = True


def main() -> None:
    rclpy.init()
    node = HuskyNav2CableTour()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
