#!/usr/bin/env python3
"""Launch Parrot short cable tour (sim already running, no Nav2)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='41068_ignition_bringup',
            executable='parrot_full_cable_tour.py',
            name='parrot_full_cable_tour',
            namespace='parrot1',
            output='screen',
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
            parameters=[{
                # Raw Gazebo odometry (full XYZ). Filtered /odom from EKF does
                # not fuse Z, so using it makes the drone climb forever.
                'odom_topic': 'odometry',
                'cmd_vel_topic': 'cmd_vel',
                'pos_tol': 0.8,
                'hold_seconds': 1.0,
            }],
        ),
    ])
