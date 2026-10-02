#!/usr/bin/env python3
"""Publish one fake alpine hazard so the operator UI can be demoed now.

Perception (ice detector / span localiser) does not need to be finished
for Assignment-3 UI work. This stub pretends the drone already classified
a severe icy patch on the snow corridor and hands that target to the
mission gate.

Disable it in the launch file once the real detector publishes
``/mission/target``.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class DemoHazardStub(Node):
    def __init__(self) -> None:
        super().__init__('demo_hazard_stub')
        self.declare_parameter('robot_name', 'husky1')
        self.declare_parameter('hazard_x', 45.0)
        self.declare_parameter('hazard_y', 0.0)
        self.declare_parameter('hazard_z', 0.0)
        self.declare_parameter('hazard_label', 'severe_ice_patch')
        self.declare_parameter('period', 2.0)

        robot = str(self.get_parameter('robot_name').value).strip().strip('/') or 'husky1'
        self.map_frame = f'{robot}_map'

        self.target_pub = self.create_publisher(PoseStamped, '/mission/target', 10)
        self.label_pub = self.create_publisher(String, '/mission/hazard_label', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission/hazard_markers', 10)
        self.create_timer(float(self.get_parameter('period').value), self._tick)
        self.get_logger().info(
            'Demo hazard stub publishing a severe ice patch. '
            'Turn this off when the real detector is online.'
        )

    def _tick(self) -> None:
        x = float(self.get_parameter('hazard_x').value)
        y = float(self.get_parameter('hazard_y').value)
        z = float(self.get_parameter('hazard_z').value)
        label = str(self.get_parameter('hazard_label').value)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.target_pub.publish(pose)
        self.label_pub.publish(String(data=label))

        marker = Marker()
        marker.header = pose.header
        marker.ns = 'hazards'
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.2
        marker.scale.y = 2.2
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 0.23
        marker.color.b = 0.31
        marker.color.a = 0.45
        marker.lifetime.sec = 3

        flag = Marker()
        flag.header = pose.header
        flag.ns = 'hazards'
        flag.id = 2
        flag.type = Marker.TEXT_VIEW_FACING
        flag.action = Marker.ADD
        flag.pose.position.x = x
        flag.pose.position.y = y
        flag.pose.position.z = z + 1.6
        flag.pose.orientation.w = 1.0
        flag.scale.z = 0.5
        flag.color.r = 1.0
        flag.color.g = 0.35
        flag.color.b = 0.35
        flag.color.a = 1.0
        flag.text = f'HAZARD  {label.replace("_", " ").upper()}'
        flag.lifetime.sec = 3

        arrow = Marker()
        arrow.header = pose.header
        arrow.ns = 'hazards'
        arrow.id = 3
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = z + 1.2
        arrow.pose.orientation.y = math.sin(-math.pi / 4)
        arrow.pose.orientation.w = math.cos(-math.pi / 4)
        arrow.scale.x = 0.9
        arrow.scale.y = 0.12
        arrow.scale.z = 0.12
        arrow.color.r = 1.0
        arrow.color.g = 0.23
        arrow.color.b = 0.31
        arrow.color.a = 0.95
        arrow.lifetime.sec = 3

        self.marker_pub.publish(MarkerArray(markers=[marker, flag, arrow]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoHazardStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
