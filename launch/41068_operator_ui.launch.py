#!/usr/bin/env python3
"""Operator guidance console add-on for Owen 

Example:
  # Terminal 1 — sim + SLAM + Nav2
  ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \\
    world:=snow_forest slam:=true nav2:=true rviz:=true

  # Terminal 2 — operator UI
  ros2 launch 41068_ignition_bringup 41068_operator_ui.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    demo_hazard = LaunchConfiguration('demo_hazard')
    console_port = LaunchConfiguration('console_port')
    hazard_x = LaunchConfiguration('hazard_x')
    hazard_y = LaunchConfiguration('hazard_y')

    tf_remap = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='husky1',
            description='Robot namespace (must match the running sim).',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'demo_hazard',
            default_value='true',
            description='Publish a fake icy hazard for UI demo. Set false when the real detector runs.',
        ),
        DeclareLaunchArgument(
            'console_port',
            default_value='8080',
            description='HTTP port for the operator web console.',
        ),
        DeclareLaunchArgument(
            'hazard_x',
            default_value='45.0',
            description='Demo hazard map x (near alpine cable view on trimmed snow_forest).',
        ),
        DeclareLaunchArgument(
            'hazard_y',
            default_value='0.0',
            description='Demo hazard map y.',
        ),

        # Global gate + console (absolute /operator/* and /mission/* topics).
        Node(
            package='41068_ignition_bringup',
            executable='mission_gate.py',
            name='mission_gate',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_name': robot,
                'navigate_action': [
                    TextSubstitution(text='/'),
                    robot,
                    TextSubstitution(text='/navigate_to_pose'),
                ],
            }],
        ),
        Node(
            package='41068_ignition_bringup',
            executable='operator_console.py',
            name='operator_console',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'port': console_port,
            }],
        ),
        Node(
            package='41068_ignition_bringup',
            executable='demo_hazard_stub.py',
            name='demo_hazard_stub',
            output='screen',
            condition=IfCondition(PythonExpression(
                ["'", demo_hazard, "'.lower() in ['true', '1', 'yes', 'on']"]
            )),
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_name': robot,
                'hazard_x': hazard_x,
                'hazard_y': hazard_y,
            }],
        ),

        # Per-robot viz nodes (relative Nav2 + camera topics).
        Node(
            package='41068_ignition_bringup',
            executable='path_preview_viz.py',
            namespace=robot,
            name='path_preview_viz',
            output='screen',
            remappings=tf_remap,
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_name': robot,
            }],
        ),
        Node(
            package='41068_ignition_bringup',
            executable='camera_hud_overlay.py',
            namespace=robot,
            name='camera_hud_overlay',
            output='screen',
            remappings=tf_remap,
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_name': robot,
            }],
        ),
    ])
