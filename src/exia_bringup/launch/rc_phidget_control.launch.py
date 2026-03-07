#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/arduino_control',
        description='Serial port for Arduino communication'
    )

    hub_port_arg = DeclareLaunchArgument(
        'hub_port',
        default_value='0',
        description='Phidget VINT Hub port for steering motor'
    )

    autonomous_arg = DeclareLaunchArgument(
        'autonomous',
        default_value='false',
        description='Start in autonomous mode'
    )

    rc_phidget_driver_node = Node(
        package='exia_driver',
        executable='rc_phidget_driver_node',
        name='rc_phidget_driver_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'phidgets_hub_port': LaunchConfiguration('hub_port'),
            'autonomous_mode': LaunchConfiguration('autonomous'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        hub_port_arg,
        autonomous_arg,
        rc_phidget_driver_node,
    ])
