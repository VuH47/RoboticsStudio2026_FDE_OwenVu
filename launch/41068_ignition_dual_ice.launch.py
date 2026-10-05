#!/usr/bin/env python3
"""Husky on corridor spawn + Parrot on deploy roof; Husky Nav2 only (Parrot cmd_vel)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('nav2', default_value='true'),
        DeclareLaunchArgument('world', default_value='snow_forest'),
        DeclareLaunchArgument('terrain', default_value='False'),
        DeclareLaunchArgument('sonar', default_value='false'),
        DeclareLaunchArgument('parrot_scan', default_value='true'),
        DeclareLaunchArgument('road_gradient', default_value='0.10'),
        DeclareLaunchArgument('rabbits', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('41068_ignition_bringup'),
                    'launch',
                    '41068_ignition.launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rviz': LaunchConfiguration('rviz'),
                'slam': LaunchConfiguration('slam'),
                'nav2': LaunchConfiguration('nav2'),
                'world': LaunchConfiguration('world'),
                'terrain': LaunchConfiguration('terrain'),
                'sonar': LaunchConfiguration('sonar'),
                'parrot_scan': LaunchConfiguration('parrot_scan'),
                'road_gradient': LaunchConfiguration('road_gradient'),
                'rabbits': LaunchConfiguration('rabbits'),
                'husky': 'True',
                'parrot': 'True',
                'parrot_nav2': 'False',
                'parrot_x': '-8.0',
                'parrot_y': '0.0',
                'parrot_z': '4.0',
            }.items(),
        ),
    ])
