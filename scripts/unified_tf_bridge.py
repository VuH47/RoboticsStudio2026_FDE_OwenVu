#!/usr/bin/env python3
"""Merge namespaced robot TF onto /tf for the one unified RViz.

Husky and Parrot publish /husky1/tf and /parrot1/tf. Unified RViz listens on
/tf, so without this node the Parrot model has no transforms.

The Parrot EKF does not fuse x/y, so parrot1_odom -> parrot1_base_link starts
at the origin while Gazebo spawned the drone at (-8, 0, 4). That edge is
dropped. /parrot1/odometry is the Gazebo world pose, and this node republishes
it as husky1_odom -> parrot1_base_link. Husky TF is forwarded unchanged so
Nav2 and the costmap stay on the same tree as the Husky mesh.
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from tf2_msgs.msg import TFMessage


def _frame(name: str) -> str:
    return str(name).lstrip('/')


def _tf_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _tf_static_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class UnifiedTfBridge(Node):
    def __init__(self) -> None:
        super().__init__('unified_tf_bridge')
        self.declare_parameter('husky_odom_frame', 'husky1_odom')
        self.declare_parameter('parrot_base_frame', 'parrot1_base_link')
        self.declare_parameter('husky_tf_topic', '/husky1/tf')
        self.declare_parameter('husky_tf_static_topic', '/husky1/tf_static')
        self.declare_parameter('parrot_tf_topic', '/parrot1/tf')
        self.declare_parameter('parrot_tf_static_topic', '/parrot1/tf_static')
        self.declare_parameter('parrot_odom_topic', '/parrot1/odometry')

        self._odom_frame = str(self.get_parameter('husky_odom_frame').value)
        self._parrot_base = str(self.get_parameter('parrot_base_frame').value)
        self._saw_husky = False
        self._saw_parrot = False

        self._tf_pub = self.create_publisher(TFMessage, '/tf', _tf_qos())
        self._static_pub = self.create_publisher(
            TFMessage, '/tf_static', _tf_static_qos())

        for topic, static in (
            (str(self.get_parameter('husky_tf_topic').value), False),
            (str(self.get_parameter('parrot_tf_topic').value), False),
            (str(self.get_parameter('husky_tf_static_topic').value), True),
            (str(self.get_parameter('parrot_tf_static_topic').value), True),
        ):
            self.create_subscription(
                TFMessage,
                topic,
                lambda msg, is_static=static: self._relay(msg, is_static),
                _tf_static_qos() if static else _tf_qos(),
            )

        self.create_subscription(
            Odometry,
            str(self.get_parameter('parrot_odom_topic').value),
            self._on_parrot_odom,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            'Unified TF bridge: relay /husky1/tf + /parrot1/tf onto /tf; '
            'Parrot base_link follows Gazebo /parrot1/odometry in %s'
            % self._odom_frame
        )

    def _drop_edge(self, transform: TransformStamped) -> bool:
        # We own parrot base_link. Forwarding the EKF edge as well gives the
        # link two parents and RViz drops the model.
        return _frame(transform.child_frame_id) == self._parrot_base

    def _relay(self, msg: TFMessage, static: bool) -> None:
        kept: List[TransformStamped] = [
            t for t in msg.transforms if not self._drop_edge(t)
        ]
        if not kept:
            return
        if not static and not self._saw_husky:
            parents = {_frame(t.header.frame_id) for t in kept}
            if self._odom_frame in parents or any(
                _frame(t.child_frame_id).startswith('husky1_') for t in kept
            ):
                self._saw_husky = True
                self.get_logger().info('Husky TF is reaching /tf')
        out = TFMessage()
        out.transforms = kept
        (self._static_pub if static else self._tf_pub).publish(out)

    def _on_parrot_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        stamped = TransformStamped()
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        stamped.header.stamp = stamp
        stamped.header.frame_id = self._odom_frame
        stamped.child_frame_id = self._parrot_base
        stamped.transform.translation.x = float(pose.position.x)
        stamped.transform.translation.y = float(pose.position.y)
        stamped.transform.translation.z = float(pose.position.z)
        q = pose.orientation
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm < 1e-6:
            stamped.transform.rotation.w = 1.0
        else:
            stamped.transform.rotation = q
        out = TFMessage()
        out.transforms = [stamped]
        self._tf_pub.publish(out)
        if not self._saw_parrot:
            self._saw_parrot = True
            self.get_logger().info(
                'Parrot model anchored at Gazebo (%.2f, %.2f, %.2f) in %s'
                % (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    self._odom_frame,
                )
            )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = UnifiedTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
