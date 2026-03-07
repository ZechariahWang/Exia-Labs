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

    skip_motor_arg = DeclareLaunchArgument(
        'skip_motor',
        default_value='false',
        description='Skip motor/serial/Arduino init (TAK-only mode)'
    )

    enable_tak_udp_arg = DeclareLaunchArgument(
        'enable_tak_udp',
        default_value='false',
        description='Enable TAK UDP output from rc_phidget_driver_node'
    )

    tak_gps_topic_arg = DeclareLaunchArgument(
        'tak_gps_topic',
        default_value='/navsatfix',
        description='GPS topic for TAK UDP messages'
    )

    tak_host_arg = DeclareLaunchArgument(
        'tak_host',
        default_value='192.168.1.69',
        description='TAK UDP destination host'
    )

    tak_port_arg = DeclareLaunchArgument(
        'tak_port',
        default_value='4242',
        description='TAK UDP destination port'
    )

    tak_uid_arg = DeclareLaunchArgument(
        'tak_uid',
        default_value='exialabs-argus-1',
        description='TAK CoT UID'
    )

    tak_callsign_arg = DeclareLaunchArgument(
        'tak_callsign',
        default_value='exialabs-argus-1',
        description='TAK callsign'
    )

    tak_type_arg = DeclareLaunchArgument(
        'tak_type',
        default_value='a-f-G-E-V-U',
        description='TAK CoT type'
    )

    tak_rate_hz_arg = DeclareLaunchArgument(
        'tak_rate_hz',
        default_value='0.2',
        description='TAK UDP send rate (Hz)'
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
            'skip_motor': LaunchConfiguration('skip_motor'),
            'tak_udp_enabled': LaunchConfiguration('enable_tak_udp'),
            'tak_gps_topic': LaunchConfiguration('tak_gps_topic'),
            'tak_host': LaunchConfiguration('tak_host'),
            'tak_port': LaunchConfiguration('tak_port'),
            'tak_uid': LaunchConfiguration('tak_uid'),
            'tak_callsign': LaunchConfiguration('tak_callsign'),
            'tak_type': LaunchConfiguration('tak_type'),
            'tak_rate_hz': LaunchConfiguration('tak_rate_hz'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        hub_port_arg,
        autonomous_arg,
        skip_motor_arg,
        enable_tak_udp_arg,
        tak_gps_topic_arg,
        tak_host_arg,
        tak_port_arg,
        tak_uid_arg,
        tak_callsign_arg,
        tak_type_arg,
        tak_rate_hz_arg,
        rc_phidget_driver_node,
    ])
