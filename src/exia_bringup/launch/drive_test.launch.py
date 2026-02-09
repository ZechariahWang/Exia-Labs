#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    drive_speed_arg = DeclareLaunchArgument(
        'drive_speed',
        default_value='0.5',
        description='Forward speed (m/s)'
    )

    drive_duration_arg = DeclareLaunchArgument(
        'drive_duration',
        default_value='5.0',
        description='Drive duration (s)'
    )

    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='1.5',
        description='Stop distance for obstacles (m)'
    )

    front_sector_angle_arg = DeclareLaunchArgument(
        'front_sector_angle',
        default_value='1.05',
        description='Front detection cone angle (radians, ~60 degrees)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    drive_forward_test = Node(
        package='exia_control',
        executable='drive_forward_test_node',
        name='drive_forward_test',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'drive_speed': LaunchConfiguration('drive_speed'),
            'drive_duration': LaunchConfiguration('drive_duration'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'front_sector_angle': LaunchConfiguration('front_sector_angle'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        drive_speed_arg,
        drive_duration_arg,
        obstacle_threshold_arg,
        front_sector_angle_arg,
        drive_forward_test,
    ])
