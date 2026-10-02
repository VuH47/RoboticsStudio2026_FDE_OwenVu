#!/usr/bin/env python3
"""Launch the Husky deploy-station patrol mission (Nav2 must already be running)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    loop = LaunchConfiguration('loop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='If true, keep repeating summit <-> deploy',
        ),
        Node(
            package='41068_ignition_bringup',
            executable='deploy_patrol_mission.py',
            name='deploy_patrol_mission',
            namespace='husky1',
            output='screen',
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
            parameters=[{
                'robot_name': 'husky1',
                # Summit near top of snow_ramp, center ski corridor
                'summit_x': 75.0,
                'summit_y': 0.0,
                'summit_yaw': 0.0,
                # Just outside deploy_station door
                'deploy_x': -15.0,
                'deploy_y': 10.0,
                'deploy_yaw': 3.14159,
                'loop': loop,
            }],
        ),
    ])
