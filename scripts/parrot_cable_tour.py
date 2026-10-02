#!/usr/bin/env python3
"""Parrot short cable tour: spawn climb → deploy roof → span_01 view → roof.

Prereq (sim running, NO slam/nav2 so altitude-follow is off):
  ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py \\
    world:=snow_forest rviz:=true
  # rabbits default False on parrot launch (avoids Gazebo Host unreachable).

Then:
  ros2 launch 41068_ignition_bringup parrot_cable_tour.launch.py

Pose must come from bridged /parrot1/odometry with OdometryPublisher
dimensions=3 (planar default zeros z and climb never completes).
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from ice_cable_geometry import (  # noqa: E402
    min_cable_clearance,
    parrot_cable_tour_waypoints,
)

Waypoint = Tuple[str, float, float, float, float]  # label, x, y, z, yaw


def yaw_error(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


class ParrotCableTour(Node):
    def __init__(self) -> None:
        super().__init__('parrot_cable_tour')

        self.declare_parameter('odom_topic', 'odometry')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('kp_xy', 0.6)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('max_v_xy', 1.2)
        self.declare_parameter('max_vz', 0.8)
        self.declare_parameter('max_wz', 1.0)
        self.declare_parameter('pos_tol', 0.8)
        self.declare_parameter('yaw_tol', 0.35)
        self.declare_parameter('hold_seconds', 1.0)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('log_period_seconds', 1.0)

        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.max_v_xy = float(self.get_parameter('max_v_xy').value)
        self.max_vz = float(self.get_parameter('max_vz').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.pos_tol = float(self.get_parameter('pos_tol').value)
        self.yaw_tol = float(self.get_parameter('yaw_tol').value)
        self.hold_s = float(self.get_parameter('hold_seconds').value)
        self.log_period = float(self.get_parameter('log_period_seconds').value)

        self._odom: Optional[Odometry] = None
        self._hold_start: Optional[float] = None
        self._last_log = 0.0
        self.index = 0
        self.results: List[str] = []
        self._view_clearance_m: Optional[float] = None
        self._done = False

        self.waypoints: List[Waypoint] = parrot_cable_tour_waypoints()

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        hz = float(self.get_parameter('control_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)

        self.get_logger().info(
            'Parrot cable tour ready (pose from "%s"). Waypoints: %s'
            % (odom_topic, [w[0] for w in self.waypoints])
        )

    def _on_odom(self, msg: Odometry) -> None:
        if self._odom is None:
            p = msg.pose.pose.position
            self.waypoints = parrot_cable_tour_waypoints(spawn_xy=(p.x, p.y))
            self.get_logger().info(
                'Climb start from spawn xy=(%.2f, %.2f)' % (p.x, p.y)
            )
        self._odom = msg

    def _pose(self) -> Optional[Tuple[float, float, float, float]]:
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        return (p.x, p.y, p.z, yaw_from_quat(q.z, q.w))

    def _stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _tick(self) -> None:
        if self._done:
            return
        pose = self._pose()
        if pose is None:
            self.get_logger().info(
                'Waiting for odom...', throttle_duration_sec=2.0)
            return
        if self.index >= len(self.waypoints):
            self._finish(True)
            return

        label, tx, ty, tz, tyaw = self.waypoints[self.index]
        x, y, z, yaw = pose
        ex, ey, ez = tx - x, ty - y, tz - z
        dist = math.sqrt(ex * ex + ey * ey + ez * ez)
        eyaw = yaw_error(tyaw, yaw)

        # Body-frame horizontal velocity from world-frame error (yaw about z).
        c, s = math.cos(yaw), math.sin(yaw)
        ex_b = c * ex + s * ey
        ey_b = -s * ex + c * ey

        cmd = Twist()
        cmd.linear.x = max(-self.max_v_xy, min(self.max_v_xy, self.kp_xy * ex_b))
        cmd.linear.y = max(-self.max_v_xy, min(self.max_v_xy, self.kp_xy * ey_b))
        cmd.linear.z = max(-self.max_vz, min(self.max_vz, self.kp_z * ez))
        cmd.angular.z = max(-self.max_wz, min(self.max_wz, self.kp_yaw * eyaw))

        if dist < self.pos_tol and abs(eyaw) < self.yaw_tol:
            if self._hold_start is None:
                self._hold_start = time.monotonic()
                self.get_logger().info(
                    '[%d/%d] HOLD %s dist=%.2f'
                    % (self.index + 1, len(self.waypoints), label, dist)
                )
            elif time.monotonic() - self._hold_start >= self.hold_s:
                self.results.append('WP %s: PASS' % label)
                if label == 'span01_view':
                    d, span, cable = min_cable_clearance(x, y, z)
                    self._view_clearance_m = d
                    self.get_logger().info(
                        'span01_view clearance=%.2f m (%s/%s)'
                        % (d, span, cable)
                    )
                self.index += 1
                self._hold_start = None
                self._stop()
                return
            self._stop()
            return

        self._hold_start = None
        now = time.monotonic()
        if now - self._last_log >= self.log_period:
            self._last_log = now
            self.get_logger().info(
                '[%d/%d] %s pose=(%.2f,%.2f,%.2f) tgt=(%.2f,%.2f,%.2f) '
                'dist=%.2f vz_cmd=%.2f'
                % (
                    self.index + 1, len(self.waypoints), label,
                    x, y, z, tx, ty, tz, dist, cmd.linear.z,
                )
            )
        self._cmd_pub.publish(cmd)

    def _finish(self, ok: bool) -> None:
        self._stop()
        clearance_ok = (
            self._view_clearance_m is not None and self._view_clearance_m >= 2.0
        )
        summary = {
            'waypoints': self.results,
            'view_clearance_m': self._view_clearance_m,
            'clearance_ok': clearance_ok,
            'pass': ok and clearance_ok and len(self.results) == len(self.waypoints),
        }
        self.get_logger().info('TOUR SUMMARY %s' % json.dumps(summary))
        self._done = True


def main() -> None:
    rclpy.init()
    node = ParrotCableTour()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
