#!/usr/bin/env python3
"""Parrot Phase 2: Nav2 short cable loop + altitude-follow (sim already up).

Prereq:
  ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py \\
    world:=snow_forest rviz:=true slam:=true nav2:=true

Then:
  ros2 launch 41068_ignition_bringup parrot_nav2_cable_tour.launch.py

Do not run Phase 1 parrot_cable_tour at the same time.
Height is terrain hover only (not roof/cable z).
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from ice_cable_geometry import (  # noqa: E402
    min_cable_clearance_xy,
    parrot_nav2_tour_xy,
)

Waypoint = Tuple[str, float, float, float]  # label, x, y, yaw


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class ParrotNav2CableTour(Node):
    def __init__(self) -> None:
        super().__init__('parrot_nav2_cable_tour')

        self.declare_parameter('robot_name', 'parrot1')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('odom_topic', 'odometry')
        self.declare_parameter('goal_pause_seconds', 1.5)
        self.declare_parameter('goal_timeout_seconds', 180.0)

        robot = self.get_parameter('robot_name').get_parameter_value().string_value
        self.map_frame = f'{robot}_map'
        self.waypoints: List[Waypoint] = parrot_nav2_tour_xy()
        self.index = 0
        self.goal_active = False
        self._done = False
        self.next_goal_time = 0.0
        self.last_feedback_log = 0.0
        self.results: List[str] = []
        self._view_clearance_xy_m: Optional[float] = None
        self._odom: Optional[Odometry] = None
        self._goal_sent_at = 0.0

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
            'Phase 2 Nav2 cable tour ready in %s. Waypoints: %s '
            '(z = terrain hover only; not roof/cable height).'
            % (self.map_frame, [w[0] for w in self.waypoints])
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

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
            self._goal_sent_at = now  # avoid spam every tick
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
            if label == 'span01_view' and self._odom is not None:
                p = self._odom.pose.pose.position
                d, span, cable = min_cable_clearance_xy(p.x, p.y)
                self._view_clearance_xy_m = d
                self.get_logger().info(
                    'span01_view horizontal clearance=%.2f m (%s/%s) '
                    '(Phase 2: xy only; roof/cable z out of scope)'
                    % (d, span, cable)
                )
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

    def _finish(self) -> None:
        expected = [w[0] for w in self.waypoints]
        passed = [r for r in self.results if r.endswith('PASS')]
        summary = {
            'phase': 2,
            'mode': 'nav2_altitude_follow',
            'waypoints': self.results,
            'view_clearance_xy_m': self._view_clearance_xy_m,
            'height_note': 'terrain hover only; not roof/cable z',
            'pass': len(passed) == len(expected),
        }
        self.get_logger().info('TOUR SUMMARY %s' % json.dumps(summary))
        self._done = True


def main() -> None:
    rclpy.init()
    node = ParrotNav2CableTour()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
