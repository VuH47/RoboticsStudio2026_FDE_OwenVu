#!/usr/bin/env python3
"""Drop lidar beams that match a shallow slope / ground plane.

Leaves vertical hits (trees, alpine towers, cable cars) so Nav2 still
avoids them, while the 4 deg snow_ramp is no longer painted as a wall.
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _pitch_from_quat(q) -> float:
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        return math.copysign(math.pi / 2.0, sinp)
    return math.asin(sinp)


def _rpy_rotate(roll, pitch, yaw, vx, vy, vz):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x1, y1, z1 = vx, cr * vy - sr * vz, sr * vy + cr * vz
    x2, y2, z2 = cp * x1 + sp * z1, y1, -sp * x1 + cp * z1
    return cy * x2 - sy * y2, sy * x2 + cy * y2, z2


class ScanGroundFilter(Node):
    """Publish scan_filtered with ground/ramp beams cleared."""

    def __init__(self):
        super().__init__('scan_ground_filter')

        self.declare_parameter('road_gradient', 0.12)
        self.declare_parameter('height_tolerance', 0.25)
        # Hits higher than this above wheel contact stay occupied (trees, towers).
        # A widening slope cone alone treats distant lidar-height trunks as 3% grade.
        self.declare_parameter('max_ground_height', 0.35)
        # Match husky.urdf.xacro base_scan joint (pitched up ~7 deg).
        self.declare_parameter('lidar_xyz', [0.0, 0.0, 0.85])
        self.declare_parameter('lidar_rpy', [0.0, -0.12, 0.0])
        # Wheel contact in base_link: joint z 0.03282 - radius 0.1651.
        self.declare_parameter('ground_z', -0.132)

        self.road_gradient = float(self.get_parameter('road_gradient').value)
        self.height_tolerance = float(self.get_parameter('height_tolerance').value)
        self.max_ground_height = float(self.get_parameter('max_ground_height').value)
        self.lidar_xyz = [float(v) for v in self.get_parameter('lidar_xyz').value]
        self.lidar_rpy = [float(v) for v in self.get_parameter('lidar_rpy').value]
        self.ground_z = float(self.get_parameter('ground_z').value)
        self._pitch = 0.0

        self._pub = self.create_publisher(LaserScan, 'scan_filtered', qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)

        self.get_logger().info(
            f'Scan ground filter: road_gradient={self.road_gradient:.3f} '
            f'height_tol={self.height_tolerance:.2f}m '
            f'max_ground_height={self.max_ground_height:.2f}m'
        )

    def _on_odom(self, msg: Odometry):
        self._pitch = _pitch_from_quat(msg.pose.pose.orientation)

    def _is_ground(self, range_m, angle):
        lx = range_m * math.cos(angle)
        ly = range_m * math.sin(angle)
        mx, my, mz = _rpy_rotate(
            self.lidar_rpy[0], self.lidar_rpy[1], self.lidar_rpy[2], lx, ly, 0.0)
        x = self.lidar_xyz[0] + mx
        y = self.lidar_xyz[1] + my
        z = self.lidar_xyz[2] + mz
        horizontal = math.hypot(x, y)
        # Tilt ground plane with robot pitch so ramp hits stay "ground" while climbing.
        expected_ground_z = self.ground_z + mx * math.tan(self._pitch)
        height_above = z - expected_ground_z
        if horizontal < 0.05:
            return abs(height_above) <= self.height_tolerance

        # Pitched-up lidar hits trees/towers well above the wheel plane; keep them.
        if height_above > self.max_ground_height:
            return False

        # Near-plane hit whose implied grade is at most road_gradient (4 deg ramp ~0.07).
        if abs(height_above) <= self.road_gradient * horizontal + self.height_tolerance:
            return True

        # Rear of the 7 deg scan plane: range matching the ground intersection.
        if mz < -1e-3:
            expected = (self.lidar_xyz[2] - self.ground_z) / max(-mz / range_m, 1e-3)
            if expected > 0.0 and abs(range_m - expected) <= 0.25 * max(expected, 1.0):
                return True
        return False

    def _on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        inf = float('inf')
        filtered = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                filtered.append(r)
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if self._is_ground(r, angle):
                filtered.append(inf)
            else:
                filtered.append(r)
        out.ranges = filtered
        if msg.intensities:
            out.intensities = list(msg.intensities)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanGroundFilter()
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
