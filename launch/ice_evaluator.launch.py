#!/usr/bin/env python3
"""Score ice detections against ~/.ros/ice_ground_truth.json."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("truth_path", default_value=""),
        DeclareLaunchArgument(
            "detections_path",
            default_value="",
            description="Optional JSON from the perception teammate.",
        ),
        Node(
            package="41068_ignition_bringup",
            executable="ice_evaluator.py",
            name="ice_evaluator",
            output="screen",
            parameters=[{
                "truth_path": LaunchConfiguration("truth_path"),
                "detections_path": LaunchConfiguration("detections_path"),
                "run_once": True,
            }],
        ),
    ])
