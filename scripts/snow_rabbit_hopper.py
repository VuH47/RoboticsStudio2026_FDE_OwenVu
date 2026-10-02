#!/usr/bin/env python3
"""Hop snow_rabbit models in snow_forest so lidar/camera can track them.

Rabbits are pre-included in worlds/snow_forest.sdf. This node only calls
Gazebo /world/<world>/set_pose (same pattern as dynamic_world_demo.py).
They stay static in SDF so hops do not fight the physics engine; collision
volumes still register on gpu_lidar.

Also publishes:
  /wildlife/rabbit_poses   geometry_msgs/PoseArray   (sim world frame)
  /wildlife/rabbit_markers visualization_msgs/MarkerArray
"""

from __future__ import annotations

import math
import random
import shutil
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

RAMP_START = 20.0
RAMP_TAN = math.tan(0.07)
CORRIDOR_Y = 12.0


def ground_z(x: float) -> float:
    if x < RAMP_START:
        return 0.0
    return RAMP_TAN * (x - RAMP_START)


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


@dataclass
class Rabbit:
    name: str
    x: float
    y: float
    yaw: float
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    rng: random.Random
    hopping: bool = False
    t: float = 0.0
    duration: float = 0.45
    rest: float = 0.6
    x0: float = 0.0
    y0: float = 0.0
    x1: float = 0.0
    y1: float = 0.0
    hop_h: float = 0.35

    def z(self) -> float:
        base = ground_z(self.x)
        if not self.hopping:
            return base
        s = max(0.0, min(1.0, self.t / self.duration))
        return base + 4.0 * self.hop_h * s * (1.0 - s)

    def _clip(self, x: float, y: float) -> Tuple[float, float]:
        x = min(self.x_max, max(self.x_min, x))
        y = min(self.y_max, max(self.y_min, y))
        # Keep the Nav2 ski lane clear.
        if abs(y) < CORRIDOR_Y:
            y = math.copysign(CORRIDOR_Y + 0.8, y if y != 0.0 else self.y)
        return x, y

    def start_hop(self) -> None:
        dist = self.rng.uniform(0.45, 0.95)
        # Prefer heading along current yaw, with some wander.
        yaw = self.yaw + self.rng.uniform(-0.7, 0.7)
        x1 = self.x + dist * math.cos(yaw)
        y1 = self.y + dist * math.sin(yaw)
        x1, y1 = self._clip(x1, y1)
        self.x0, self.y0 = self.x, self.y
        self.x1, self.y1 = x1, y1
        self.yaw = math.atan2(y1 - self.y, x1 - self.x) if hypot_ok(x1 - self.x, y1 - self.y) else self.yaw
        self.hop_h = self.rng.uniform(0.28, 0.48)
        self.duration = self.rng.uniform(0.32, 0.50)
        self.t = 0.0
        self.hopping = True

    def step(self, dt: float) -> None:
        if not self.hopping:
            self.rest -= dt
            if self.rest <= 0.0:
                self.start_hop()
            return
        self.t += dt
        s = max(0.0, min(1.0, self.t / self.duration))
        # Ease-in/out so the landing is not a teleport.
        e = s * s * (3.0 - 2.0 * s)
        self.x = self.x0 + e * (self.x1 - self.x0)
        self.y = self.y0 + e * (self.y1 - self.y0)
        if s >= 1.0:
            self.x, self.y = self._clip(self.x1, self.y1)
            self.hopping = False
            self.rest = self.rng.uniform(0.25, 0.90)


def hypot_ok(dx: float, dy: float) -> bool:
    return (dx * dx + dy * dy) > 1e-6


class SnowRabbitHopper(Node):
    def __init__(self) -> None:
        super().__init__('snow_rabbit_hopper')
        self.declare_parameter('world_name', 'snow_forest')
        self.declare_parameter('update_period', 0.10)
        self.declare_parameter(
            'rabbit_names',
            [
                'snow_rabbit_flat_L',
                'snow_rabbit_flat_R',
                'snow_rabbit_ramp_L',
                'snow_rabbit_ramp_R',
                'snow_rabbit_rr1',
            ],
        )
        self.world_name = str(self.get_parameter('world_name').value)
        self.update_period = float(self.get_parameter('update_period').value)
        names: Sequence[str] = list(self.get_parameter('rabbit_names').value)

        # Home patches match worlds/snow_forest.sdf spawn poses (slightly
        # closer to the corridor than the first static placement).
        homes = {
            'snow_rabbit_flat_L': (6.0, 14.5, 0.40, 1.0, 14.0, 13.5, 22.0),
            'snow_rabbit_flat_R': (8.0, -14.5, -1.20, 2.0, 16.0, -22.0, -13.5),
            'snow_rabbit_ramp_L': (28.0, 16.5, 0.20, 22.0, 40.0, 13.5, 24.0),
            'snow_rabbit_ramp_R': (36.0, -16.5, 2.40, 28.0, 48.0, -24.0, -13.5),
            'snow_rabbit_rr1': (31.5, -37.0, -0.50, 26.0, 40.0, -44.0, -34.0),
        }

        self.rabbits: List[Rabbit] = []
        for i, name in enumerate(names):
            hx, hy, hyaw, xmin, xmax, ymin, ymax = homes.get(
                name, (0.0, 16.0, 0.0, -5.0, 20.0, 13.5, 25.0)
            )
            r = Rabbit(
                name=name,
                x=hx,
                y=hy,
                yaw=hyaw,
                x_min=xmin,
                x_max=xmax,
                y_min=ymin,
                y_max=ymax,
                rng=random.Random(17 + 11 * i),
            )
            r.rest = r.rng.uniform(0.0, 0.8)
            self.rabbits.append(r)

        self.gz_command = 'ign' if shutil.which('ign') else 'gz'
        self.msg_prefix = 'ignition.msgs' if self.gz_command == 'ign' else 'gz.msgs'
        self.pose_pub = self.create_publisher(PoseArray, '/wildlife/rabbit_poses', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/wildlife/rabbit_markers', 10)
        self.ready = False
        self.timer = self.create_timer(self.update_period, self._tick)
        self.get_logger().info(
            'Hopping %d rabbits in world [%s] (lidar collision + camera mesh).'
            % (len(self.rabbits), self.world_name)
        )

    def _run(self, command: List[str]) -> bool:
        try:
            result = subprocess_run(command, timeout=1.5)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('set_pose helper failed: %s' % exc)
            return False
        return result == 0

    def set_pose(self, r: Rabbit) -> bool:
        qx, qy, qz, qw = yaw_to_quat(r.yaw)
        request = (
            'name: "%s" '
            'position { x: %.5f y: %.5f z: %.5f } '
            'orientation { x: %.5f y: %.5f z: %.5f w: %.5f }'
            % (r.name, r.x, r.y, r.z(), qx, qy, qz, qw)
        )
        return self._run(
            [
                self.gz_command,
                'service',
                '-s',
                '/world/%s/set_pose' % self.world_name,
                '--reqtype',
                '%s.Pose' % self.msg_prefix,
                '--reptype',
                '%s.Boolean' % self.msg_prefix,
                '--timeout',
                '800',
                '--req',
                request,
            ]
        )

    def _publish_ros(self) -> None:
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'husky1_odom'
        markers = MarkerArray()
        now = arr.header.stamp
        for i, r in enumerate(self.rabbits):
            p = Pose()
            p.position.x = r.x
            p.position.y = r.y
            p.position.z = r.z()
            qx, qy, qz, qw = yaw_to_quat(r.yaw)
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            arr.poses.append(p)

            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'husky1_odom'
            m.ns = 'rabbits'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = p
            m.scale.x = 0.45
            m.scale.y = 0.35
            m.scale.z = 0.40
            m.color.r = 0.75
            m.color.g = 0.55
            m.color.b = 0.35
            m.color.a = 0.85
            markers.markers.append(m)
        self.pose_pub.publish(arr)
        self.marker_pub.publish(markers)

    def _tick(self) -> None:
        if not self.ready:
            ok = True
            for r in self.rabbits:
                ok = self.set_pose(r) and ok
            self.ready = ok
            if ok:
                self.get_logger().info('Rabbits live — hopping.')
            return
        dt = self.update_period
        for r in self.rabbits:
            was_hopping = r.hopping
            r.step(dt)
            if r.hopping or was_hopping:
                self.set_pose(r)
        self._publish_ros()


def subprocess_run(command: List[str], timeout: float) -> int:
    import subprocess

    try:
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return 1
    return result.returncode


def main() -> None:
    rclpy.init()
    node = SnowRabbitHopper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
