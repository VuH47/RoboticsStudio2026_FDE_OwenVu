#!/usr/bin/env python3
"""Place randomly seeded ice blocks on alpine cables. Sim must already be running."""

import random
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    block_count = LaunchConfiguration("block_count")
    world = LaunchConfiguration("world")
    
    # Generate a completely random seed for each launch
    random_seed = random.randint(0, 2147483647)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "world",
            default_value="snow_forest",
            description="Gazebo world name (must match the running sim).",
        ),
        DeclareLaunchArgument("block_count", default_value="8"),
        DeclareLaunchArgument(
            "one_per_span",
            default_value="false",
            description="Place blocks purely randomly across all spans.",
        ),
        DeclareLaunchArgument(
            "span_ids",
            default_value="span_01,span_02,span_03",
            description="Comma-separated spans for ice placement (e.g. span_01).",
        ),
        Node(
            package="41068_ignition_bringup",
            executable="ice_scenario_manager.py",
            name="ice_scenario_manager",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "world_name": world,
                "seed": random_seed,
                "block_count": ParameterValue(block_count, value_type=int),
                "one_per_span": ParameterValue(
                    LaunchConfiguration("one_per_span"), value_type=bool
                ),
                "span_ids": LaunchConfiguration("span_ids"),
            }],
        ),
    ])
