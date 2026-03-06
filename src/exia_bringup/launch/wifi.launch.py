#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('exia_bringup')
    wifi_params_file = os.path.join(bringup_dir, 'config', 'wifi_params.yaml')

    role_arg = DeclareLaunchArgument(
        'role', default_value='robot',
        description='Bridge role: base or robot')

    peer_ip_arg = DeclareLaunchArgument(
        'peer_ip', default_value='192.168.1.122',
        description='IP address of the peer machine')

    wifi_bridge_node = Node(
        package='exia_driver',
        executable='wifi_bridge_node',
        name='wifi_bridge',
        output='screen',
        parameters=[
            wifi_params_file,
            {
                'role': LaunchConfiguration('role'),
                'peer_ip': LaunchConfiguration('peer_ip'),
            },
        ],
    )

    return LaunchDescription([
        role_arg,
        peer_ip_arg,
        wifi_bridge_node,
    ])
