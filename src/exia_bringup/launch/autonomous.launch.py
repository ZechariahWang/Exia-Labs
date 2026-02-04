#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')

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
            'auto_start': True,
        }],
    )

    delayed_navigator = TimerAction(
        period=20.0,
        actions=[dynamic_navigator_node],
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        slam_toolbox_node,
        planner_server_node,
        delayed_lifecycle_manager,
        delayed_navigator,
    ])
