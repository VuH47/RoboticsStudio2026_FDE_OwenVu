#!/usr/bin/env python3
"""Husky world-verification mission.

Purpose
  1) Prove free-space Nav2 works on the ski corridor (no false walls).
  2) Prove the Husky can climb the trimmed ramp in short steps.
  3) Prove sensors see world objects (trees / thick alpine cables) via LaserScan.

Prereq (sim already running):
  ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \\
    world:=snow_forest slam:=true nav2:=true rviz:=true

Then:
  ros2 launch 41068_ignition_bringup verify_world_mission.launch.py
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


Waypoint = Tuple[str, float, float, float]  # label, x, y, yaw


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class VerifyWorldMission(Node):
    """Staged free-space climb + lidar object detection check."""

    def __init__(self):
        super().__init__('verify_world_mission')

        self.declare_parameter('robot_name', 'husky1')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('scan_topic', 'scan_filtered')
        self.declare_parameter('goal_pause_seconds', 1.5)
        # Right-side sector toward alpine (world y≈-30 when facing +X).
        self.declare_parameter('detect_yaw', -math.pi / 2.0)
        self.declare_parameter('detect_half_angle', 0.45)
        self.declare_parameter('detect_min_range', 12.0)
        self.declare_parameter('detect_max_range', 45.0)
        self.declare_parameter('detect_min_hits', 8)

        robot = self.get_parameter('robot_name').get_parameter_value().string_value
        self.map_frame = f'{robot}_map'
        self.pause_s = float(self.get_parameter('goal_pause_seconds').value)
        self.detect_yaw = float(self.get_parameter('detect_yaw').value)
        self.detect_half = float(self.get_parameter('detect_half_angle').value)
        self.detect_rmin = float(self.get_parameter('detect_min_range').value)
        self.detect_rmax = float(self.get_parameter('detect_max_range').value)
        self.detect_min_hits = int(self.get_parameter('detect_min_hits').value)

        # Free corridor only (y=0). Short steps so SLAM/costmap stay happy.
        self.waypoints: List[Waypoint] = [
            ('flat_open', 8.0, 0.0, 0.0),
            ('pre_ramp', 18.0, 0.0, 0.0),
            ('ramp_entry', 25.0, 0.0, 0.0),
            ('ramp_mid', 40.0, 0.0, 0.0),
            # Face right (−Y) so lidar looks at alpine cables / trees.
            ('view_alpine', 45.0, 0.0, self.detect_yaw),
            ('ramp_upper', 60.0, 0.0, 0.0),
            ('near_rr4', 72.0, 0.0, 0.0),
        ]

        self.index = 0
        self.goal_active = False
        self.mission_done = False
        self.next_goal_time = 0.0
        self._last_scan: Optional[LaserScan] = None
        self.results: List[str] = []

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter('navigate_action').value,
        )
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f'Verify-world mission ready in {self.map_frame}. '
            f'Waypoints: {[w[0] for w in self.waypoints]}'
        )
        self.get_logger().info(
            f'Object check uses /{robot}/{scan_topic} '
            f'right-sector hits in [{self.detect_rmin:.0f},{self.detect_rmax:.0f}] m'
        )

    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg

    def _count_object_hits(self) -> int:
        scan = self._last_scan
        if scan is None or not scan.ranges:
            return 0
        hits = 0
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < self.detect_rmin or r > self.detect_rmax:
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            ang = scan.angle_min + i * scan.angle_increment
            # Wrap angle difference into [-pi, pi]
            d = (ang - self.detect_yaw + math.pi) % (2.0 * math.pi) - math.pi
            if abs(d) <= self.detect_half:
                hits += 1
        return hits

    def _run_detection_check(self) -> None:
        # Allow a few scans after arriving / rotating.
        time.sleep(1.0)
        hits = self._count_object_hits()
        ok = hits >= self.detect_min_hits
        msg = (
            f'DETECT alpine/trees: hits={hits} '
            f'(need >={self.detect_min_hits}) → {"PASS" if ok else "FAIL"}'
        )
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(msg)
        self.results.append(msg)

    def _tick(self) -> None:
        if self.mission_done or self.goal_active:
            return
        if time.monotonic() < self.next_goal_time:
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().info(
                'Waiting for Nav2 navigate_to_pose...', throttle_duration_sec=5.0)
            return
        if self.index >= len(self.waypoints):
            self.mission_done = True
            self.get_logger().info('======== VERIFY SUMMARY ========')
            for line in self.results:
                self.get_logger().info(line)
            fails = sum(1 for r in self.results if 'FAIL' in r)
            if fails == 0 and self.results:
                self.get_logger().info(
                    'WORLD VERIFY PASS — free navigation + object detection OK')
            else:
                self.get_logger().warn(
                    f'WORLD VERIFY finished with {fails} FAIL item(s). '
                    'Check costmap / scan_filtered / alpine pose.')
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
        self.get_logger().info(
            f'[{self.index + 1}/{len(self.waypoints)}] NAV {label} '
            f'→ ({x:.1f}, {y:.1f}) yaw={yaw:.2f}'
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            label = self.waypoints[self.index][0]
            self.get_logger().error(f'Goal rejected: {label}')
            self.results.append(f'NAV {label}: FAIL (rejected)')
            self.goal_active = False
            self.mission_done = True
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result)

    def _goal_result(self, future) -> None:
        result = future.result()
        status = result.status
        label = self.waypoints[self.index][0]
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'NAV {label}: PASS')
            self.results.append(f'NAV {label}: PASS')
            if label == 'view_alpine':
                self._run_detection_check()
            self.index += 1
            self.next_goal_time = time.monotonic() + self.pause_s
        else:
            self.get_logger().error(f'NAV {label}: FAIL (status={status})')
            self.results.append(f'NAV {label}: FAIL (status={status})')
            self.mission_done = True
        self.goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = VerifyWorldMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
