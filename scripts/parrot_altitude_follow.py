#!/usr/bin/env python3
"""Keep Parrot above the snow_ramp while Nav2 only commands planar Twist.

Nav2 is 2D: it publishes vx/vy/yaw and leaves vz=0. The drone then holds its
spawn height (~0.8 m) while the 4° ramp surface rises:

  z_surface(x) = (x - 20) * tan(0.07)

Past x ≈ 31 m the surface is above the drone → it tunnels into the open-bottom
wedge collision (looks "fully inside the ramp"). This node merges Nav2 planar
cmd_vel with a P-controller on z so the drone climbs with the terrain.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ParrotAltitudeFollow(Node):
    """Merge planar Nav2 Twist with ramp altitude tracking → cmd_vel."""

    def __init__(self):
        super().__init__('parrot_altitude_follow')

        self.declare_parameter('ramp_start_x', 20.0)
        self.declare_parameter('ramp_pitch', 0.07)
        self.declare_parameter('ramp_length', 175.0)
        self.declare_parameter('hover_height', 1.0)
        self.declare_parameter('flat_hover_z', 0.8)
        self.declare_parameter('kp_z', 1.5)
        self.declare_parameter('max_vz', 1.5)

        self.ramp_start_x = float(self.get_parameter('ramp_start_x').value)
        self.ramp_pitch = float(self.get_parameter('ramp_pitch').value)
        self.ramp_length = float(self.get_parameter('ramp_length').value)
        self.hover_height = float(self.get_parameter('hover_height').value)
        self.flat_hover_z = float(self.get_parameter('flat_hover_z').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.max_vz = float(self.get_parameter('max_vz').value)

        self._odom: Optional[Odometry] = None
        self._planar = Twist()

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self._on_nav, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)
        self.create_timer(0.05, self._tick)  # 20 Hz

        self.get_logger().info(
            f'Parrot altitude follow: hover={self.hover_height}m above ramp, '
            f'flat_z={self.flat_hover_z}m, kp_z={self.kp_z}'
        )

    def _surface_z(self, x: float) -> float:
        if x < self.ramp_start_x:
            return 0.0
        along = min(x - self.ramp_start_x, self.ramp_length)
        return along * math.tan(self.ramp_pitch)

    def _desired_z(self, x: float) -> float:
        return max(self.flat_hover_z, self._surface_z(x) + self.hover_height)

    def _on_nav(self, msg: Twist):
        self._planar = msg

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _tick(self):
        out = Twist()
        out.linear.x = self._planar.linear.x
        out.linear.y = self._planar.linear.y
        out.angular.z = self._planar.angular.z

        if self._odom is not None:
            x = self._odom.pose.pose.position.x
            z = self._odom.pose.pose.position.z
            err = self._desired_z(x) - z
            vz = max(-self.max_vz, min(self.max_vz, self.kp_z * err))
            out.linear.z = vz

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ParrotAltitudeFollow()
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
