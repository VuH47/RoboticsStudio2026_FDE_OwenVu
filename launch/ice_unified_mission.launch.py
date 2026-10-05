import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('41068_ignition_bringup')

    # One Gazebo, both robots, Husky Nav2, no per-robot RViz.
    dual_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', '41068_ignition_dual_ice.launch.py')),
        launch_arguments={
            'rviz': 'false',
            'sonar': 'true',
            'parrot_scan': 'false',
        }.items()
    )

    # World service is not up at t=0. The manager retries, and this delay
    # keeps the first set_pose calls from landing before Gazebo exists.
    ice_scenario = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'ice_scenario.launch.py')),
                launch_arguments={
                    'use_sim_time': 'true',
                    'world': 'snow_forest',
                    'seed': '42',
                    'block_count': '1',
                    'one_per_span': 'true',
                }.items()
            )
        ]
    )

    # Copy /husky1/tf and /parrot1/tf onto /tf, and pin the Parrot mesh to
    # the Gazebo pose. Do not publish husky1_odom -> parrot1_odom at 0:
    # that draws the drone 8 m away from the roof.
    tf_bridge = Node(
        package='41068_ignition_bringup',
        executable='unified_tf_bridge.py',
        name='unified_tf_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    handoff = Node(
        package='41068_ignition_bringup',
        executable='ice_unified_handoff.py',
        name='ice_unified_handoff',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'truth_wait_seconds': 180.0,
        }],
    )

    ice_detector = Node(
        package='41068_ignition_bringup',
        executable='ice_rgbd_detector.py',
        name='ice_rgbd_detector',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            # Detector xyz is Gazebo world metres. husky1_odom is that frame
            # for this launch (Husky spawn is the world origin).
            {'target_frame': 'husky1_odom'},
        ],
    )

    # Namespace so the Nav2 panel binds to /husky1/navigate_to_pose.
    # TF stays on global /tf (the bridge), not /husky1/tf.
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
        handoff,
        ice_detector,
        rviz,
    ])
