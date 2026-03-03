#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('exia_bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_hardware_gps_arg = DeclareLaunchArgument(
        'use_hardware_gps',
        default_value='false',
        description='Use Septentrio hardware GPS driver'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_hardware_gps = LaunchConfiguration('use_hardware_gps')

    gps_params_file = os.path.join(bringup_dir, 'config', 'gps_params.yaml')
    ekf_params_file = os.path.join(bringup_dir, 'config', 'ekf_params.yaml')
    septentrio_params_file = os.path.join(bringup_dir, 'config', 'septentrio_rover.yaml')

    gps_transform_node = Node(
        package='exia_driver',
        executable='gps_transform_node',
        name='gps_transform_node',
        output='screen',
        parameters=[gps_params_file, {'use_sim_time': use_sim_time}],
    )

    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_hardware_gps_arg,
        gps_transform_node,
        ekf_localization_node,
    ])
