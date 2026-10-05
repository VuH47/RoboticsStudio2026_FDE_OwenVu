import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('41068_ignition_bringup')

    # Same world, robots, Nav2, and unified RViz as the random-confirm launch.
    # The placer still writes ice_ground_truth.json. This mission does not read it.
    dual_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg, 'launch', '41068_ignition_dual_ice.launch.py')),
        launch_arguments={
            'rviz': 'false',
            'sonar': 'true',
            'parrot_scan': 'false',
        }.items()
    )

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
        executable='ice_perception_confirm.py',
        name='ice_perception_confirm',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'approach_y': -10.0,
        }],
    )

    # Colour blob plus aligned depth. A pose is published on /fde/detection
    # only when the depth point lands on a cable and stands proud of it.
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
