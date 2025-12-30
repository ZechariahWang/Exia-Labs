"""
Exia Ground Robot - Navigation Launch File

Launches the Nav2 navigation stack for autonomous navigation.
Uses Smac Hybrid-A* planner for Ackermann-specific path planning.

Prerequisites:
    - Simulation or hardware running (exia_ground_sim.launch.py)
    - SLAM or map server running

Usage:
    ros2 launch exia_ground_description navigation.launch.py

Author: Zechariah Wang
Date: December 2025
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('exia_ground_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Path to Nav2 params file'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start navigation stack'
    )

    # Nav2 bringup (includes costmaps, planner, controller, etc.)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    # Our custom navigation node (Pure Pursuit with replanning)
    navigation_node = Node(
        package='exia_ground_description',
        executable='navigation_node.py',
        name='navigation_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'lookahead_distance': 2.0},
            {'goal_tolerance': 0.5},
            {'max_speed': 1.5},
            {'replan_period': 10.0},
            {'replan_distance': 5.0},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        nav2_bringup,
        navigation_node,
    ])
