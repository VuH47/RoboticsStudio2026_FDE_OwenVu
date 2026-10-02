#!/usr/bin/env python3
"""Launch Husky world-verification mission (sim + Nav2 must already be running)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='41068_ignition_bringup',
            executable='verify_world_mission.py',
            name='verify_world_mission',
            namespace='husky1',
            output='screen',
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
            parameters=[{
                'robot_name': 'husky1',
                'scan_topic': 'scan_filtered',
                'detect_min_hits': 8,
                'detect_min_range': 12.0,
                'detect_max_range': 45.0,
            }],
        ),
    ])
