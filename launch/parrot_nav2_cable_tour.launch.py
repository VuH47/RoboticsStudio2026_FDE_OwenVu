#!/usr/bin/env python3
"""Launch Parrot Phase 2 Nav2 cable tour (sim already running with slam+nav2)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='41068_ignition_bringup',
            executable='parrot_nav2_cable_tour.py',
            name='parrot_nav2_cable_tour',
            namespace='parrot1',
            output='screen',
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
            parameters=[{
                'robot_name': 'parrot1',
                'odom_topic': 'odometry',
                'goal_pause_seconds': 1.5,
                'goal_timeout_seconds': 180.0,
            }],
        ),
    ])
