#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('exia_bringup')
    radio_params_file = os.path.join(bringup_dir, 'config', 'radio_params.yaml')

    role_arg = DeclareLaunchArgument(
        'role',
        default_value='robot',
        description='Bridge role: base or robot',
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RFD900x radio',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    radio_bridge_node = Node(
        package='exia_driver',
        executable='radio_bridge_node',
        name='radio_bridge',
        output='screen',
        parameters=[
            radio_params_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'role': LaunchConfiguration('role'),
                'serial_port': LaunchConfiguration('serial_port'),
            },
        ],
    )

    return LaunchDescription([
        role_arg,
        serial_port_arg,
        use_sim_time_arg,
        radio_bridge_node,
    ])
