from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_TRUE_STRINGS = "['true', '1', 'yes', 'on']"


def _is_true_expression(name):
    return ["'", LaunchConfiguration(name), "'.lower() in ", _TRUE_STRINGS]


def _if_true(name):
    return IfCondition(PythonExpression(_is_true_expression(name)))


def _if_all_true(*names):
    pieces = []
    for i, name in enumerate(names):
        if i > 0:
            pieces.append(' and ')
        pieces.extend(_is_true_expression(name))
    return IfCondition(PythonExpression(pieces))


def _if_slam_or_nav2():
    return IfCondition(PythonExpression(
        _is_true_expression('slam') + [' or '] + _is_true_expression('nav2')
    ))


def _if_husky_slam_or_nav2():
    return IfCondition(PythonExpression(
        _is_true_expression('husky') + [' and (']
        + _is_true_expression('slam') + [' or '] + _is_true_expression('nav2') + [')']
    ))


def _if_parrot_slam_or_nav2():
    return IfCondition(PythonExpression(
        _is_true_expression('parrot') + [' and (']
        + _is_true_expression('slam') + [' or '] + _is_true_expression('nav2') + [')']
    ))


def _if_parrot_nav2_stack():
    """Parrot SLAM/Nav2/altitude-follow (off when parrot_nav2:=false for cmd_vel missions)."""
    return IfCondition(PythonExpression(
        _is_true_expression('parrot') + [' and ']
        + _is_true_expression('parrot_nav2') + [' and (']
        + _is_true_expression('slam') + [' or '] + _is_true_expression('nav2') + [')']
    ))


def add_robot(
    ld,
    *,
    pkg_path,
    config_path,
    use_sim_time,
    enabled_arg,
    namespace,
    frame_prefix,
    gz_model_name,
    xacro_parts,
    bridge_config,
    localization_config,
    x,
    y,
    z,
    yaw='0.0',
    spawn_delay=0.0,
    extra_xacro=None,
):
    """Add one robot instance to the launch description.

    Convention used here:
      - ROS nodes/topics are under /<namespace>/...
      - Gazebo entity/model is named <gz_model_name>
      - Gazebo plugin topics are under /model/<gz_model_name>/...
      - TF topics are under /<namespace>/tf and /<namespace>/tf_static
      - TF frame ids use <frame_prefix>, e.g. husky1_base_link
    """

    xacro_cmd = [
        'xacro ',
        PathJoinSubstitution([pkg_path] + xacro_parts),
        ' ',
        'prefix:=', frame_prefix,
        ' ',
        'gz_model_name:=', gz_model_name,
    ]
    if extra_xacro:
        xacro_cmd.extend(extra_xacro)

    robot_description_content = ParameterValue(
        Command(xacro_cmd),
        value_type=str,
    )

    robot_enabled = _if_true(enabled_arg)

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        condition=robot_enabled,
    ))

    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=namespace,
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, localization_config]),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odom'),
        ],
        condition=robot_enabled,
    ))

    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_' + namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-topic', '/' + namespace + '/robot_description',
            '-name', gz_model_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
    )

    # Start Gazebo first, then insert robot entities in a predictable order.
    ld.add_action(TimerAction(
        period=spawn_delay,
        actions=[robot_spawner],
        condition=robot_enabled,
    ))

    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='gazebo_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, bridge_config]),
            'use_sim_time': use_sim_time,
        }],
        condition=robot_enabled,
    ))


def add_navigation_instance(
    ld,
    *,
    pkg_path,
    use_sim_time,
    robot_arg,
    robot_namespace,
    start_delay,
    terrain='False',
    cmd_vel_out='cmd_vel',
):
    nav_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_path,
                'launch',
                '41068_navigation.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': robot_namespace,
            'config_filename_suffix': '_' + robot_namespace,
            'slam': LaunchConfiguration('slam'),
            'nav2': LaunchConfiguration('nav2'),
            'terrain': terrain,
            'cmd_vel_out': cmd_vel_out,
        }.items(),
    )

    # Scope is important: without it, repeated IncludeLaunchDescription calls can
    # leak/overwrite launch configurations such as namespace and config suffix.
    nav_condition = _if_true(robot_arg)
    if robot_arg == 'parrot':
        nav_condition = _if_parrot_nav2_stack()

    ld.add_action(TimerAction(
        period=start_delay,
        actions=[GroupAction(
            scoped=True,
            actions=[nav_include],
        )],
        condition=nav_condition,
    ))


def add_rviz_instance(
    ld,
    *,
    config_path,
    use_sim_time,
    robot_arg,
    robot_namespace,
    rviz_config,
    start_delay,
):
    # Each RViz process is itself placed in the robot namespace so the Nav2 RViz
    # panel resolves relative action/service names such as navigate_to_pose and
    # lifecycle_manager_navigation to the correct robot.
    ld.add_action(TimerAction(
        period=start_delay,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            namespace=robot_namespace,
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', PathJoinSubstitution([config_path, rviz_config])],
            remappings=[
                ('/tf', '/' + robot_namespace + '/tf'),
                ('/tf_static', '/' + robot_namespace + '/tf_static'),
            ],
        )],
        condition=_if_all_true('rviz', robot_arg),
    ))


def generate_launch_description():

    ld = LaunchDescription()

    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz',
    )
    ld.add_action(rviz_launch_arg)

    slam_launch_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Flag to launch SLAM Toolbox for each enabled robot',
    )
    ld.add_action(slam_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='False',
        description='Flag to launch Nav2 for each enabled robot. Nav2 also starts SLAM.',
    )
    ld.add_action(nav2_launch_arg)

    husky_launch_arg = DeclareLaunchArgument(
        'husky',
        default_value='True',
        description='Launch the Husky UGV instance in namespace /husky1',
    )
    ld.add_action(husky_launch_arg)

    parrot_launch_arg = DeclareLaunchArgument(
        'parrot',
        default_value='False',
        description='Launch the Parrot drone instance in namespace /parrot1',
    )
    ld.add_action(parrot_launch_arg)
    ld.add_action(DeclareLaunchArgument(
        'parrot_nav2',
        default_value='True',
        description='Start Parrot SLAM/Nav2/altitude-follow. Set false for cmd_vel aerial missions.',
    ))
    ld.add_action(DeclareLaunchArgument('parrot_x', default_value='2.0'))
    ld.add_action(DeclareLaunchArgument('parrot_y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('parrot_z', default_value='0.8'))

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        # Teacher note: keep snow_forest because worlds/snow_forest.sdf was
        # added in this branch. After editing launch choices, verify with:
        # ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py world:=snow_forest
        choices=['simple_trees', 'large_demo', 'snow_forest', 'snow_ramp_test', 'snow_ice_cable_test', 'snow_cable_baseline'],
    )
    ld.add_action(world_launch_arg)

    terrain_launch_arg = DeclareLaunchArgument(
        'terrain',
        default_value='False',
        description='Optional downward sonar + terrain_map for husky1. Climbing uses scan_filtered even when this is false.',
    )
    ld.add_action(terrain_launch_arg)
    # Sonar rays without the altitude follower. terrain:=true still enables both.
    ld.add_action(DeclareLaunchArgument(
        'sonar',
        default_value='False',
        description='Spawn the Husky front sonar lidar. Does not start altitude follow.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'parrot_scan',
        default_value='True',
        description='Parrot 360 lidar. Off for the unified demo; that view uses the camera and the downward beam.',
    ))
    road_gradient_launch_arg = DeclareLaunchArgument(
        'road_gradient',
        default_value='0.10',
        description='Max traversable grade. 0.10 for the 4 degree snow_ramp (~7 percent).',
    )
    ld.add_action(road_gradient_launch_arg)

    rabbits_launch_arg = DeclareLaunchArgument(
        'rabbits',
        default_value='True',
        description='Hop snow_rabbit models in snow_forest (set False for Parrot cable tour).',
    )
    ld.add_action(rabbits_launch_arg)

    # Load common Gazebo server systems from a shared config file. This keeps
    # required systems such as Sensors out of individual robot and world files,
    # so custom student worlds do not need to copy plugin blocks.
    server_config_file = PathJoinSubstitution([
        config_path,
        'ignition_server.config',
    ])
    ld.add_action(SetEnvironmentVariable(
        name='IGN_GAZEBO_SERVER_CONFIG_PATH',
        value=server_config_file,
    ))
    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_SERVER_CONFIG_PATH',
        value=server_config_file,
    ))

    # model://snow_ramp/... resolves against this path. Prepend package models
    # so new meshes are found after colcon install without a hand-set bashrc.
    pkg_models = PathJoinSubstitution([pkg_path, 'models'])
    existing_ign_path = EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value='')
    existing_gz_path = EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')
    ld.add_action(SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[pkg_models, ':', existing_ign_path],
    ))
    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_models, ':', existing_gz_path],
    ))

    # Start Gazebo once.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'ign_args': [
                PathJoinSubstitution([
                    pkg_path,
                    'worlds',
                    [LaunchConfiguration('world'), '.sdf'],
                ]),
                ' -r',
            ]
        }.items(),
    ))

    # Bridge the global simulation clock once.
    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_clock',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_clock.yaml']),
            'use_sim_time': use_sim_time,
        }],
    ))

    add_robot(
        ld,
        pkg_path=pkg_path,
        config_path=config_path,
        use_sim_time=use_sim_time,
        enabled_arg='husky',
        namespace='husky1',
        frame_prefix='husky1_',
        gz_model_name='husky1',
        xacro_parts=['urdf_husky', 'husky.urdf.xacro'],
        bridge_config='gazebo_bridge_husky1.yaml',
        localization_config='robot_localization_husky1.yaml',
        x='0.0',
        y='0.0',
        z='0.4',
        spawn_delay=3.0,
        extra_xacro=[
            ' ',
            'enable_sonar:=',
            PythonExpression(
                _is_true_expression('sonar') + [' or '] + _is_true_expression('terrain')
            ),
        ],
    )

    add_robot(
        ld,
        pkg_path=pkg_path,
        config_path=config_path,
        use_sim_time=use_sim_time,
        enabled_arg='parrot',
        namespace='parrot1',
        frame_prefix='parrot1_',
        gz_model_name='parrot1',
        xacro_parts=['urdf_parrot', 'parrot.urdf.xacro'],
        bridge_config='gazebo_bridge_parrot1.yaml',
        localization_config='robot_localization_parrot1.yaml',
        x=LaunchConfiguration('parrot_x'),
        y=LaunchConfiguration('parrot_y'),
        z=LaunchConfiguration('parrot_z'),
        spawn_delay=6.0,
        extra_xacro=[
            ' ',
            'enable_scan:=', LaunchConfiguration('parrot_scan'),
        ],
    )

    # Ground/ramp lidar filter must run before SLAM/Nav2 (7 s). Without it the
    # 4 deg snow_ramp reads as a vertical wall at x≈20 and the robot stops there.
    ld.add_action(TimerAction(
        period=6.0,
        actions=[Node(
            package='41068_ignition_bringup',
            executable='scan_ground_filter.py',
            namespace='husky1',
            name='scan_ground_filter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'road_gradient': ParameterValue(
                    LaunchConfiguration('road_gradient'), value_type=float),
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )],
        condition=_if_husky_slam_or_nav2(),
    ))

    # Parrot: horizontal lidar at ~1 m AGL still paints snow/ramp as obstacles.
    ld.add_action(TimerAction(
        period=9.0,
        actions=[Node(
            package='41068_ignition_bringup',
            executable='scan_ground_filter.py',
            namespace='parrot1',
            name='scan_ground_filter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'road_gradient': ParameterValue(
                    LaunchConfiguration('road_gradient'), value_type=float),
                'lidar_xyz': [0.0, 0.0, 0.2],
                'lidar_rpy': [0.0, 0.0, 0.0],
                # Spawn z=0.8 m; filter ground hits relative to hover height.
                'ground_z': -0.8,
                'max_ground_height': 0.5,
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )],
        condition=_if_parrot_nav2_stack(),
    ))

    add_navigation_instance(
        ld,
        pkg_path=pkg_path,
        use_sim_time=use_sim_time,
        robot_arg='husky',
        robot_namespace='husky1',
        start_delay=7.0,
        terrain=LaunchConfiguration('terrain'),
        # Never command vz on Husky — VelocityControl treats linear.z as flight
        # and lifts the robot into the sky. Climb = wheels + closed ramp mesh.
        cmd_vel_out='cmd_vel',
    )

    add_navigation_instance(
        ld,
        pkg_path=pkg_path,
        use_sim_time=use_sim_time,
        robot_arg='parrot',
        robot_namespace='parrot1',
        start_delay=10.0,
        terrain='False',
        cmd_vel_out='cmd_vel_nav',
    )

    # Parrot: fly ~1 m above ramp surface (same VelocityControl + 2D Nav2 issue).
    ld.add_action(TimerAction(
        period=9.5,
        actions=[Node(
            package='41068_ignition_bringup',
            executable='terrain_altitude_follow.py',
            namespace='parrot1',
            name='terrain_altitude_follow',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'ramp_start_x': 20.0,
                'ramp_pitch': 0.07,
                # Clear ramp face; lookahead starts climb before x=20.
                'hover_height': 1.5,
                'flat_hover_z': 0.8,
                'kp_z': 2.0,
                'max_vz': 2.0,
                'lookahead_x': 5.0,
                # Raw Gazebo XYZ — EKF /odom does not fuse Z (fly-forever bug).
                'odom_topic': 'odometry',
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )],
        condition=_if_parrot_nav2_stack(),
    ))

    # Optional downward sonar + terrain_map (PFMS a3_terrain style). Scan filter
    # is started above when slam/nav2 are on; do not duplicate it here.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_path,
                'launch',
                '41068_terrain_perception.launch.py',
            ])
        ),
        launch_arguments={
            'robot': 'husky1',
            'use_sim_time': use_sim_time,
            'road_gradient': LaunchConfiguration('road_gradient'),
            'start_filter': 'False',
        }.items(),
        condition=_if_all_true('terrain', 'husky'),
    ))

    # Hop winter hares in snow_forest after Gazebo is up. Collision is in the
    # SDF; this node only set_pose hops so lidar/camera see a moving animal.
    # Disable with rabbits:=false — set_pose spam can freeze Gazebo (Host unreachable).
    ld.add_action(TimerAction(
        period=8.0,
        actions=[Node(
            package='41068_ignition_bringup',
            executable='snow_rabbit_hopper.py',
            name='snow_rabbit_hopper',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'world_name': LaunchConfiguration('world'),
            }],
        )],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('world'), "' == 'snow_forest' and '",
            LaunchConfiguration('rabbits'), "' == 'True'",
        ])),
    ))

    add_rviz_instance(
        ld,
        config_path=config_path,
        use_sim_time=use_sim_time,
        robot_arg='husky',
        robot_namespace='husky1',
        rviz_config='41068_husky1.rviz',
        start_delay=11.0,
    )

    add_rviz_instance(
        ld,
        config_path=config_path,
        use_sim_time=use_sim_time,
        robot_arg='parrot',
        robot_namespace='parrot1',
        rviz_config='41068_parrot1.rviz',
        start_delay=13.0,
    )

    return ld
