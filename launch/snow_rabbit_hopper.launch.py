from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='snow_forest',
        description='Gazebo world name the rabbits were spawned in.',
    ))
    ld.add_action(Node(
        package='41068_ignition_bringup',
        executable='snow_rabbit_hopper.py',
        name='snow_rabbit_hopper',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world_name': LaunchConfiguration('world'),
        }],
    ))
    return ld
