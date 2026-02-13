#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('exia_bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start nav2 stack'
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='false',
        description='Use EKF sensor fusion with GPS'
    )

    use_gps_arg = DeclareLaunchArgument(
        'use_gps',
        default_value='true',
        description='Use GPS lat/lon target (true) or local x/y target (false)'
    )

    target_x_arg = DeclareLaunchArgument(
        'target_x',
        default_value='22.0',
        description='Target X coordinate in meters (local frame)'
    )

    target_y_arg = DeclareLaunchArgument(
        'target_y',
        default_value='24.0',
        description='Target Y coordinate in meters (local frame)'
    )

    target_lat_arg = DeclareLaunchArgument(
        'target_lat',
        default_value='49.666667',
        description='Target latitude (decimal degrees)'
    )

    target_lon_arg = DeclareLaunchArgument(
        'target_lon',
        default_value='11.841389',
        description='Target longitude (decimal degrees)'
    )

    origin_lat_arg = DeclareLaunchArgument(
        'origin_lat',
        default_value='49.666400',
        description='GPS origin latitude (decimal degrees)'
    )

    origin_lon_arg = DeclareLaunchArgument(
        'origin_lon',
        default_value='11.841100',
        description='GPS origin longitude (decimal degrees)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_ekf = LaunchConfiguration('use_ekf')
    use_gps = LaunchConfiguration('use_gps')
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_lat = LaunchConfiguration('target_lat')
    target_lon = LaunchConfiguration('target_lon')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')

    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')
    gps_params_file = os.path.join(bringup_dir, 'config', 'gps_params.yaml')
    ekf_params_file = os.path.join(bringup_dir, 'config', 'ekf_params.yaml')
    pointcloud_to_laserscan_config = os.path.join(
        bringup_dir, 'config', 'pointcloud_to_laserscan.yaml'
    )

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    configured_slam_params = RewrittenYaml(
        source_file=slam_params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[pointcloud_to_laserscan_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', '/points'),
            ('scan', '/scan'),
        ],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[configured_slam_params],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_nav2_params],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['planner_server'],
            'bond_timeout': 20.0,
        }],
    )

    delayed_lifecycle_manager = TimerAction(
        period=5.0,
        actions=[lifecycle_manager_node],
    )

    dynamic_navigator_node = Node(
        package='exia_control',
        executable='dynamic_navigator_node',
        name='dynamic_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_start': False,
            'use_gps_waypoint': use_gps,
            'target_x': target_x,
            'target_y': target_y,
            'target_lat': target_lat,
            'target_lon': target_lon,
            'origin_lat': origin_lat,
            'origin_lon': origin_lon,
        }],
    )

    delayed_navigator = TimerAction(
        period=20.0,
        actions=[dynamic_navigator_node],
    )

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
        condition=IfCondition(use_ekf),
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        use_ekf_arg,
        use_gps_arg,
        target_x_arg,
        target_y_arg,
        target_lat_arg,
        target_lon_arg,
        origin_lat_arg,
        origin_lon_arg,
        pointcloud_to_laserscan,
        slam_toolbox_node,
        planner_server_node,
        delayed_lifecycle_manager,
        gps_transform_node,
        ekf_localization_node,
        delayed_navigator,
    ])
