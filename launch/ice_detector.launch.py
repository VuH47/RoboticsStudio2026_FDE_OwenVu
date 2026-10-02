#!/usr/bin/env python3
"""Launch the RGB-D alpine ice perception node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'parrot_namespace',
            default_value='parrot1',
            description='Parrot namespace',
        ),
        DeclareLaunchArgument(
            'husky_namespace',
            default_value='husky1',
            description='Husky namespace',
        ),
        DeclareLaunchArgument(
            'min_bulge_radius_m',
            default_value='0.20',
            description='Radial threshold (m) above bare cable radius (0.17m) to confirm ice',
        ),
        DeclareLaunchArgument(
            'min_ice_area_px',
            default_value='35',
            description='Minimum pixel contour area for candidate',
        ),
        Node(
            package='41068_ignition_bringup',
            executable='ice_rgbd_detector.py',
            name='ice_rgbd_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'parrot_namespace': LaunchConfiguration('parrot_namespace'),
                'husky_namespace': LaunchConfiguration('husky_namespace'),
                'min_bulge_radius_m': LaunchConfiguration('min_bulge_radius_m'),
                'min_ice_area_px': LaunchConfiguration('min_ice_area_px'),
            }],
        ),
    ])
