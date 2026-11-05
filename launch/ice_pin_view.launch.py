import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('41068_ignition_bringup')

    # Same world, both robots, Nav2, and the unified RViz. The old crumb
    # mission is not started. The Parrot only pins ice. The Husky plans
    # its own path to a viewing pose.
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
                    'block_count': LaunchConfiguration('block_count'),
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

    pin_view = Node(
        package='41068_ignition_bringup',
        executable='husky_pin_view.py',
        name='husky_pin_view',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    husky_visual_confirm = Node(
        package='41068_ignition_bringup',
        executable='husky_ice_visual_confirm.py',
        name='husky_ice_visual_confirm',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'optical_frame': 'husky1_camera_rgb_optical_frame',
            'frames_required': 5,
            'positives_required': 3,
            'timeout_seconds': 8.0,
            'roi_half_width_px': 75,
            'roi_half_height_px': 56,
            'min_ice_area_px': 14.0,
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

    path_log = Node(
        package='41068_ignition_bringup',
        executable='husky_path_log.py',
        name='husky_path_log',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    viewer = Node(
        package='41068_ignition_bringup',
        executable='detection_viewer.py',
        name='detection_viewer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_viewer')),
    )

    console = Node(
        package='41068_ignition_bringup',
        executable='operator_gui.py',
        name='fde_operator_gui',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_console')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'block_count', default_value='4',
            description='How many ice blocks the scenario places.'),
        DeclareLaunchArgument(
            'enable_viewer', default_value='true',
            description='Open the combined OpenCV detection window.'),
        DeclareLaunchArgument(
            'enable_console', default_value='false',
            description='Open the Qt operator console window.'),
        dual_sim,
        ice_scenario,
        tf_bridge,
        pin_view,
        husky_visual_confirm,
        ice_detector,
        path_log,
        viewer,
        console,
        rviz,
    ])
