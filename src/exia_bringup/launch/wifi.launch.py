#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('exia_bringup')
    wifi_params_file = os.path.join(bringup_dir, 'config', 'wifi_params.yaml')

    role_arg = DeclareLaunchArgument(
        'role',
        default_value='robot',
        description='Role: base or robot',
    )

    peer_ip_arg = DeclareLaunchArgument(
        'peer_ip',
        default_value='192.168.1.100',
        description='IP address of the peer machine',
    )

    dds_config_arg = DeclareLaunchArgument(
        'dds_config',
        default_value='',
        description='Path to CycloneDDS XML config (auto-selected if empty)',
    )

    return LaunchDescription([
        role_arg,
        peer_ip_arg,
        dds_config_arg,
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
    ])
