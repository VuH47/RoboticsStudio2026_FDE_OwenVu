#!/usr/bin/env python3
"""Place ice on cables, then run Parrot patrol → Husky Nav2 handoff."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    seed = LaunchConfiguration('seed')
    block_count = LaunchConfiguration('block_count')
    world = LaunchConfiguration('world')
    handoff_delay = LaunchConfiguration('handoff_delay_seconds')

    ice_scenario = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('41068_ignition_bringup'),
                'launch',
                'ice_scenario.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'world': world,
            'seed': seed,
            'block_count': block_count,
            'one_per_span': 'true',
            # Match Parrot span01_view patrol — avoid ice on span_02/03 out of range.
            'span_ids': LaunchConfiguration('span_ids'),
        }.items(),
    )

    handoff = Node(
        package='41068_ignition_bringup',
        executable='ice_aerial_husky_handoff.py',
        name='ice_aerial_husky_handoff',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'truth_wait_seconds': 90.0,
            'detect_range_m': 25.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='snow_forest'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('block_count', default_value='1'),
        DeclareLaunchArgument(
            'span_ids',
            default_value='span_01',
            description='Ice placement spans (default span_01 for aerial handoff).',
        ),
        DeclareLaunchArgument(
            'handoff_delay_seconds',
            default_value='8.0',
            description='Wait for ice_scenario set_pose before handoff node starts.',
        ),
        ice_scenario,
        TimerAction(
            period=handoff_delay,
            actions=[handoff],
        ),
    ])
