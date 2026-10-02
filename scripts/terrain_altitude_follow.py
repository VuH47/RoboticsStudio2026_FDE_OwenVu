#!/usr/bin/env python3
"""Follow snow_ramp height while Nav2 only commands planar Twist.

Both Husky and Parrot use Ignition VelocityControl. Nav2 is 2D (vx/vy/yaw,
vz=0), so the robot holds spawn height while the 4° ramp surface rises:

  z_surface(x) = (x - 20) * tan(0.07)

Past ~x=31 m the surface is above spawn height → the robot tunnels into the
open-bottom wedge ("inside the ramp"). This node merges Nav2 planar cmd_vel_nav
with a P-controller on z so the robot climbs with the terrain.

Params (set per robot in launch):
  Husky  — flat_hover_z≈0.4, hover_height≈0.35 (chassis above surface)
  Parrot — flat_hover_z≈0.8, hover_height≈1.5, lookahead so it rises
           before driving into the slope face
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TerrainAltitudeFollow(Node):
    """Merge planar Nav2 Twist with ramp altitude tracking → cmd_vel."""

    def __init__(self):
        super().__init__('terrain_altitude_follow')

        self.declare_parameter('ramp_start_x', 20.0)
        self.declare_parameter('ramp_pitch', 0.07)
        self.declare_parameter('ramp_length', 60.0)
        self.declare_parameter('hover_height', 0.35)
        self.declare_parameter('flat_hover_z', 0.4)
        self.declare_parameter('kp_z', 1.5)
        self.declare_parameter('max_vz', 1.5)
        # Climb using surface height a few metres ahead so we clear the face.
        self.declare_parameter('lookahead_x', 0.0)
        # Husky: True → vz=0 on flat snow so it does not hover/fly at spawn.
        self.declare_parameter('follow_only_on_ramp', False)
        # Parrot: use Gazebo "odometry" (XYZ). EKF "odom" often does not fuse Z
        # → measured z stuck at 0 → this node commands +vz forever (fly to sky).
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('log_period_seconds', 2.0)

        self.ramp_start_x = float(self.get_parameter('ramp_start_x').value)
        self.ramp_pitch = float(self.get_parameter('ramp_pitch').value)
        self.ramp_length = float(self.get_parameter('ramp_length').value)
        self.hover_height = float(self.get_parameter('hover_height').value)
        self.flat_hover_z = float(self.get_parameter('flat_hover_z').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.max_vz = float(self.get_parameter('max_vz').value)
        self.lookahead_x = float(self.get_parameter('lookahead_x').value)
        raw_gate = self.get_parameter('follow_only_on_ramp').value
        if isinstance(raw_gate, str):
            self.follow_only_on_ramp = raw_gate.strip().lower() in ('1', 'true', 'yes')
        else:
            self.follow_only_on_ramp = bool(raw_gate)
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.log_period = float(self.get_parameter('log_period_seconds').value)

        self._odom: Optional[Odometry] = None
        self._planar = Twist()
        self._last_log = 0.0

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self._on_nav, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_timer(0.05, self._tick)  # 20 Hz

        self.get_logger().info(
            'Terrain altitude follow: hover=%.2fm above ramp, flat_z=%.2fm, '
            'kp_z=%.2f, lookahead=%.1fm, odom=%s'
            % (
                self.hover_height, self.flat_hover_z, self.kp_z,
                self.lookahead_x, odom_topic,
            )
        )

    def _surface_z(self, x: float) -> float:
        if x < self.ramp_start_x:
            return 0.0
        along = min(x - self.ramp_start_x, self.ramp_length)
        return along * math.tan(self.ramp_pitch)

    def _desired_z(self, x: float) -> float:
        x_ref = x + max(0.0, self.lookahead_x)
        return max(self.flat_hover_z, self._surface_z(x_ref) + self.hover_height)

    def _on_nav(self, msg: Twist):
        self._planar = msg

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _tick(self):
        out = Twist()
        out.linear.x = self._planar.linear.x
        out.linear.y = self._planar.linear.y
        out.angular.z = self._planar.angular.z

        # Allow teleop / manual altitude commands to adjust the hover height
        if abs(self._planar.linear.z) > 0.01:
            self.flat_hover_z = max(0.4, min(20.0, self.flat_hover_z + self._planar.linear.z * 0.05))

        if self._odom is not None:
            x = self._odom.pose.pose.position.x
            z = self._odom.pose.pose.position.z
            if self.follow_only_on_ramp and x < self.ramp_start_x:
                out.linear.z = 0.0
            else:
                z_des = self._desired_z(x)
                err = z_des - z
                out.linear.z = max(-self.max_vz, min(self.max_vz, self.kp_z * err))
                now = time.monotonic()
                if now - self._last_log >= self.log_period:
                    self._last_log = now
                    self.get_logger().info(
                        'alt x=%.1f z=%.2f z_des=%.2f vz=%.2f'
                        % (x, z, z_des, out.linear.z)
                    )

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainAltitudeFollow()
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
