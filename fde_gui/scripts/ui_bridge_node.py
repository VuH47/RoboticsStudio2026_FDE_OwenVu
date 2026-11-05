#!/usr/bin/env python3
"""Operator UI bridge.

This node publishes markers and echoes mission state.

Operator commands (from a second terminal):

  ros2 topic pub --once /fde/mission_command std_msgs/msg/String "{data: start}"
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class UiBridgeNode(Node):
    def __init__(self):
        super().__init__('fde_ui')
        self.declare_parameter('robot_name', 'parrot1')
        self.robot = self.get_parameter('robot_name').get_parameter_value().string_value
        self.mission_state = 'IDLE'
        self.detection = None

        self.pub_markers = self.create_publisher(MarkerArray, '/fde/markers', 10)
        self.create_subscription(String, '/fde/mission_state', self.state_callback, 10)
        self.create_subscription(PoseStamped, '/fde/detection', self.detection_callback, 10)
        self.create_timer(0.5, self.publish_markers)

        self.get_logger().info(
            'UI bridge running. Add MarkerArray display on /fde/markers in RViz. '
            'Publish start/pause/abort on /fde/mission_command.'
        )

    def state_callback(self, msg: String):
        if msg.data != self.mission_state:
            self.get_logger().info(f'Mission state: {msg.data}')
        self.mission_state = msg.data

    def detection_callback(self, msg: PoseStamped):
        self.detection = msg

    def publish_markers(self):
        markers = MarkerArray()
        text = Marker()
        text.header.frame_id = f'{self.robot}_map'
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = 'fde'
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.z = 2.0
        text.pose.orientation.w = 1.0
        text.scale.z = 0.4
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f'FDE mission: {self.mission_state}'
        markers.markers.append(text)

        if self.detection is not None:
            sphere = Marker()
            sphere.header = self.detection.header
            sphere.ns = 'fde'
            sphere.id = 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = self.detection.pose
            sphere.scale.x = 0.5
            sphere.scale.y = 0.5
            sphere.scale.z = 0.5
            sphere.color.r = 1.0
            sphere.color.a = 0.8
            markers.markers.append(sphere)

        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = UiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
