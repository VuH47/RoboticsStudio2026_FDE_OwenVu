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
            'require_husky_visual_confirmation': True,
            'visual_confirm_timeout_s': 12.0,
        }],
    )

    # Mission 3 requires independently observed Husky RGB pixels before 'confirmed'.
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
            # Pixel thresholds scaled 0.75x for the 480x360 Husky camera
            # (defaults 100 / 75 / 25 were tuned at 640x480).
            'roi_half_width_px': 75,
            'roi_half_height_px': 56,
            'min_ice_area_px': 14.0,
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

    # One combined pop-up (Parrot + Husky). RViz is unchanged.
    # Disable with:  enable_viewer:=false
    viewer = Node(
        package='41068_ignition_bringup',
        executable='detection_viewer.py',
        name='detection_viewer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_viewer')),
    )

    # Operator console GUI. Read-only.
    # Enable with:  enable_console:=true
    console = Node(
        package='41068_ignition_bringup',
        executable='operator_console.py',
        name='operator_console',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_console')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_viewer', default_value='true',
            description='Open the combined OpenCV detection window.'),
        DeclareLaunchArgument(
            'enable_console', default_value='false',
            description='Open the Qt operator console window.'),
        dual_sim,
        ice_scenario,
        tf_bridge,
        confirm,
        husky_visual_confirm,
        ice_detector,
        viewer,
        console,
        rviz,
    ])
