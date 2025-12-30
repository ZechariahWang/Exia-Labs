"""
Exia Ground Robot - SLAM + Navigation Launch File

Launches both slam_toolbox and Nav2 navigation for simultaneous
mapping and navigation (explore while building map).

Prerequisites:
    - Simulation running (exia_ground_sim.launch.py)

Usage:
    # Terminal 1: Start simulation
    ros2 launch exia_ground_description exia_ground_sim.launch.py

    # Terminal 2: Start SLAM + Navigation
    ros2 launch exia_ground_description slam_nav.launch.py

    # Terminal 3: RViz for visualization and goal setting
    rviz2 -d ~/exia_ws/src/exia_ground_description/rviz/exia_slam_nav.rviz

Author: Zechariah Wang
Date: December 2025
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('exia_ground_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    autostart = LaunchConfiguration('autostart')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml'),
        description='Path to slam_toolbox params file'
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Path to Nav2 params file'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start navigation stack'
    )

    # slam_toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # Nav2 bringup (delayed to let SLAM start first)
    nav2_bringup = TimerAction(
        period=3.0,  # Wait 3 seconds for SLAM to start
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                    'autostart': autostart,
                }.items()
            )
        ]
    )

    # Our custom navigation node (delayed even more)
    navigation_node = TimerAction(
        period=8.0,  # Wait for Nav2 to fully start
        actions=[
            Node(
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
                    {'max_path_deviation': 2.0},
                ],
            )
        ]
    )

    # Robot localization EKF (optional - for sensor fusion)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'robot_localization.yaml'),
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        nav2_params_file_arg,
        autostart_arg,

        # Start SLAM first
        slam_toolbox_node,

        # Then Nav2 (with delay)
        nav2_bringup,

        # Then our navigation node (with delay)
        navigation_node,

        # Robot localization for sensor fusion
        # robot_localization_node,  # Uncomment if needed
    ])
