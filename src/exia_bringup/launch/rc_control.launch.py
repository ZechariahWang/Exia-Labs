#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/arduino_control',
        description='Serial port for Arduino communication'
    )

    rc_driver_node = Node(
        package='exia_driver',
        executable='rc_driver_node',
        name='rc_driver_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'serial_port': LaunchConfiguration('serial_port'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        serial_port_arg,
        rc_driver_node,
    ])
