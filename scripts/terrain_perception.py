#!/usr/bin/env python3
"""PFMS-style terrain perception from a downward sonar LaserScan + odom.

Builds a rolling elevation grid and publishes an OccupancyGrid where cells
steeper than road_gradient are occupied (100) and shallower cells are free (0).
Reimplements the METHOD of PFMS TerrainAnalysis in Python (not a C++ port).
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


Cell = Tuple[int, int]


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_rotate(q, vx: float, vy: float, vz: float):
    """Rotate a vector by a geometry_msgs quaternion (x, y, z, w)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    # Hamilton product q * (0, v) * q_conj
    ix = w * vx + y * vz - z * vy
    iy = w * vy + z * vx - x * vz
    iz = w * vz + x * vy - y * vx
    iw = -x * vx - y * vy - z * vz
    return (
        ix * w + iw * -x + iy * -z - iz * -y,
        iy * w + iw * -y + iz * -x - ix * -z,
        iz * w + iw * -z + ix * -y - iy * -x,
    )


def _rpy_rotate(roll: float, pitch: float, yaw: float, vx: float, vy: float, vz: float):
    """Apply URDF RPY (Rz * Ry * Rx) to a vector."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Rx
    x1, y1, z1 = vx, cr * vy - sr * vz, sr * vy + cr * vz
    # Ry
    x2, y2, z2 = cp * x1 + sp * z1, y1, -sp * x1 + cp * z1
    # Rz
    return cy * x2 - sy * y2, sy * x2 + cy * y2, z2


class TerrainPerception(Node):
    """Elevation grid + gradient traversability from downward sonar."""

    def __init__(self):
        super().__init__('terrain_perception')

        ns = self.get_namespace().strip('/')
        default_frame = f'{ns}_odom' if ns else 'husky1_odom'

        self.declare_parameter('road_gradient', 0.10)
        self.declare_parameter('resolution', 0.25)
        self.declare_parameter('map_length', 80.0)
        self.declare_parameter('map_frame', default_frame)
        self.declare_parameter('lookahead_m', 10.0)
        self.declare_parameter('sonar_xyz', [0.15, 0.0, 1.0])
        self.declare_parameter('sonar_rpy', [0.0, 0.785398, 0.0])
        self.declare_parameter('publish_rate', 2.0)

        self.road_gradient = float(self.get_parameter('road_gradient').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.map_length = float(self.get_parameter('map_length').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.lookahead_m = float(self.get_parameter('lookahead_m').value)
        self.sonar_xyz = [float(v) for v in self.get_parameter('sonar_xyz').value]
        self.sonar_rpy = [float(v) for v in self.get_parameter('sonar_rpy').value]
        publish_rate = float(self.get_parameter('publish_rate').value)

        self._elev: Dict[Cell, float] = {}
        self._odom: Optional[Odometry] = None

        self._map_pub = self.create_publisher(OccupancyGrid, 'terrain_map', 10)
        self._grad_pub = self.create_publisher(Float32, 'terrain/worst_gradient', 10)

        self.create_subscription(
            LaserScan, 'sonar', self._on_sonar, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)

        period = 1.0 / max(publish_rate, 0.1)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Terrain perception: road_gradient={self.road_gradient:.3f} '
            f'res={self.resolution}m map={self.map_length}m frame={self.map_frame}'
        )

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _on_sonar(self, msg: LaserScan):
        if self._odom is None:
            return

        odom = self._odom
        ox = odom.pose.pose.position.x
        oy = odom.pose.pose.position.y
        oz = odom.pose.pose.position.z
        oq = odom.pose.pose.orientation
        sx, sy, sz = self.sonar_xyz
        sr, sp, syaw = self.sonar_rpy

        have_valid = False
        min_range = float('inf')
        min_angle = 0.0

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            have_valid = True
            angle = msg.angle_min + i * msg.angle_increment
            if r < min_range:
                min_range = r
                min_angle = angle
            self._insert_hit(r, angle, ox, oy, oz, oq, sx, sy, sz, sr, sp, syaw)

        # Conservative min-range sample (PFMS TerrainAnalysis does this too)
        if have_valid:
            self._insert_hit(
                min_range, min_angle, ox, oy, oz, oq, sx, sy, sz, sr, sp, syaw)

    def _insert_hit(
        self,
        range_m: float,
        angle: float,
        ox: float,
        oy: float,
        oz: float,
        oq,
        sx: float,
        sy: float,
        sz: float,
        sr: float,
        sp: float,
        syaw: float,
    ):
        # Polar in sonar_link (X forward, Y left), then mount RPY into base_link.
        lx = range_m * math.cos(angle)
        ly = range_m * math.sin(angle)
        mx, my, mz = _rpy_rotate(sr, sp, syaw, lx, ly, 0.0)
        bx, by, bz = sx + mx, sy + my, sz + mz
        wx, wy, wz = _quat_rotate(oq, bx, by, bz)
        hit_x = ox + wx
        hit_y = oy + wy
        hit_z = oz + wz

        ix = int(math.floor(hit_x / self.resolution))
        iy = int(math.floor(hit_y / self.resolution))
        cell = (ix, iy)
        prev = self._elev.get(cell)
        if prev is None:
            self._elev[cell] = hit_z
        else:
            # Low-pass like PFMS (0.7 old + 0.3 new)
            self._elev[cell] = 0.7 * prev + 0.3 * hit_z

    def _height_at(self, x: float, y: float) -> float:
        ix = int(math.floor(x / self.resolution))
        iy = int(math.floor(y / self.resolution))
        return self._elev.get((ix, iy), float('nan'))

    def _cell_gradient(self, ix: int, iy: int) -> float:
        h = self._elev.get((ix, iy))
        if h is None:
            return float('nan')
        worst = 0.0
        known = False
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            hn = self._elev.get((ix + dx, iy + dy))
            if hn is None:
                continue
            ds = self.resolution * math.hypot(dx, dy)
            if ds < 1e-6:
                continue
            known = True
            g = abs(hn - h) / ds
            if g > worst:
                worst = g
        return worst if known else float('nan')

    def _on_timer(self):
        if self._odom is None:
            return
        self._publish_map()
        self._check_forward_path()

    def _publish_map(self):
        odom = self._odom
        cx = odom.pose.pose.position.x
        cy = odom.pose.pose.position.y
        n = max(4, int(round(self.map_length / self.resolution)))
        origin_x = cx - 0.5 * n * self.resolution
        origin_y = cy - 0.5 * n * self.resolution
        ix0 = int(math.floor(origin_x / self.resolution))
        iy0 = int(math.floor(origin_y / self.resolution))

        data = np.full(n * n, -1, dtype=np.int8)
        for j in range(n):
            for i in range(n):
                ix, iy = ix0 + i, iy0 + j
                if (ix, iy) not in self._elev:
                    continue
                g = self._cell_gradient(ix, iy)
                idx = j * n + i
                if not math.isfinite(g):
                    data[idx] = 0  # known height, no neighbour yet: treat as free
                elif g > self.road_gradient + 1e-6:
                    data[idx] = 100
                else:
                    data[idx] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = n
        msg.info.height = n
        msg.info.origin.position.x = ix0 * self.resolution
        msg.info.origin.position.y = iy0 * self.resolution
        msg.info.origin.orientation.w = 1.0
        msg.data = data.astype(np.int8).tolist()
        self._map_pub.publish(msg)

    def _check_forward_path(self):
        odom = self._odom
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        yaw = _yaw_from_quat(odom.pose.pose.orientation)
        dist = self.lookahead_m
        samples = max(3, int(dist / self.resolution))
        prev_h = self._height_at(px, py)
        have_prev = math.isfinite(prev_h)
        worst = 0.0
        blocked = False

        for i in range(1, samples + 1):
            t = i / samples
            x = px + t * dist * math.cos(yaw)
            y = py + t * dist * math.sin(yaw)
            h = self._height_at(x, y)
            if not math.isfinite(h):
                have_prev = False
                continue
            if have_prev:
                ds = dist / samples
                g = abs(h - prev_h) / ds
                if g > worst:
                    worst = g
                if g > self.road_gradient + 1e-6:
                    blocked = True
            prev_h = h
            have_prev = True

        self._grad_pub.publish(Float32(data=float(worst)))
        ok = not blocked
        self.get_logger().info(
            f'Forward {dist:.0f} m path: '
            f'{"TRAVERSABLE" if ok else "BLOCKED"} '
            f'(worst_gradient={worst:.3f}, limit={self.road_gradient:.3f})',
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TerrainPerception()
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
