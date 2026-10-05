import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('41068_ignition_bringup')

    # Same world, robots, sensors, and unified RViz as ice_unified_mission.
    # This file does not start that mission's handoff.
    dual_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg, 'launch', '41068_ignition_dual_ice.launch.py')),
        launch_arguments={
            'rviz': 'false',
            'sonar': 'true',
            'parrot_scan': 'false',
        }.items()
    )

    # Fresh seed every launch. Blocks may land on any span; nothing is pinned
    # to span_01. block_count 3 keeps a test short — pass block_count:=8 for
    # the full pool.
    ice_scenario = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg, 'launch', 'ice_scenario_random.launch.py')),
                launch_arguments={
                    'use_sim_time': 'true',
                    'world': 'snow_forest',
                    'block_count': '3',
                    'one_per_span': 'false',
                }.items()
            )
        ]
    )

    tf_bridge = Node(
        package='41068_ignition_bringup',
        executable='unified_tf_bridge.py',
        name='unified_tf_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    confirm = Node(
        package='41068_ignition_bringup',
        executable='ice_random_confirm.py',
        name='ice_random_confirm',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'truth_wait_seconds': 180.0,
            'approach_y': -10.0,
        }],
    )

    ice_detector = Node(
        package='41068_ignition_bringup',
        executable='ice_rgbd_detector.py',
        name='ice_rgbd_detector',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'target_frame': 'husky1_odom'},
        ],
    )

    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                namespace='husky1',
                name='rviz2',
                arguments=['-d', os.path.join(pkg, 'config', '41068_unified.rviz')],
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('/tf', '/tf'),
                    ('tf', '/tf'),
                    ('/tf_static', '/tf_static'),
                    ('tf_static', '/tf_static'),
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        dual_sim,
        ice_scenario,
        tf_bridge,
        confirm,
        ice_detector,
        rviz,
    ])
