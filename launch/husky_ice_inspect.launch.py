#!/usr/bin/env python3
"""Husky corridor inspection facing the ski-lift (sim + Nav2 already up)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="41068_ignition_bringup",
            executable="husky_ice_inspect.py",
            name="husky_ice_inspect",
            namespace="husky1",
            output="screen",
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
            parameters=[{
                "robot_name": "husky1",
                "camera_topic": "camera/image",
                "odom_topic": "odom",
                "goal_pause_seconds": 2.0,
            }],
        ),
    ])
