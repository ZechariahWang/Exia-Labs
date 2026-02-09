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

    target_heading_arg = DeclareLaunchArgument(
        'target_heading_deg',
        default_value='90.0',
        description='Target heading (degrees)'
    )

    turn_speed_arg = DeclareLaunchArgument(
        'turn_speed',
        default_value='0.5',
        description='Forward speed while turning (m/s)'
    )

    kp_heading_arg = DeclareLaunchArgument(
        'kp_heading',
        default_value='1.5',
        description='Proportional gain for heading control'
    )

    heading_tolerance_arg = DeclareLaunchArgument(
        'heading_tolerance_deg',
        default_value='5.0',
        description='Heading tolerance (degrees)'
    )

    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='30.0',
        description='Maximum time for turn (s)'
    )

    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='1.5',
        description='Stop distance for obstacles (m)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    turn_to_heading_test = Node(
        package='exia_control',
        executable='turn_to_heading_test_node',
        name='turn_to_heading_test',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_heading_deg': LaunchConfiguration('target_heading_deg'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'kp_heading': LaunchConfiguration('kp_heading'),
            'heading_tolerance_deg': LaunchConfiguration('heading_tolerance_deg'),
            'timeout': LaunchConfiguration('timeout'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        target_heading_arg,
        turn_speed_arg,
        kp_heading_arg,
        heading_tolerance_arg,
        timeout_arg,
        obstacle_threshold_arg,
        turn_to_heading_test,
    ])
