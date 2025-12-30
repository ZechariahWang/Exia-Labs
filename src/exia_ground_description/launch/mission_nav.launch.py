"""
Exia Ground Robot - Mission Navigation Launch File

Simplified navigation for predefined path following with obstacle avoidance.
NO SLAM - uses only local costmap for obstacle detection.

Architecture:
- Planner server with internal costmap (30m rolling window in odom frame)
- Nav2 Smac Hybrid-A* planner for detour planning around obstacles
- Mission navigator node for path following with Pure Pursuit

Usage:
    # Terminal 1: Start simulation
    ros2 launch exia_ground_description exia_ground_sim.launch.py

    # Terminal 2: Start mission navigation
    ros2 launch exia_ground_description mission_nav.launch.py

Author: Zechariah Wang
Date: December 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('exia_ground_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    path_type = LaunchConfiguration('path_type')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Path to Nav2 params file'
    )

    path_type_arg = DeclareLaunchArgument(
        'path_type',
        default_value='figure_eight',
        description='Predefined path type: line, square, circle, figure_eight, slalom'
    )

    # ==================== Planner Server ====================
    # A* planner with its own internal global_costmap (in odom frame)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    # ==================== Lifecycle Manager ====================
    # Manages lifecycle of planner_server (and its internal costmap)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['planner_server']},
        ],
    )

    # ==================== Mission Navigator ====================
    # Our simplified path following node with obstacle avoidance
    mission_navigator_node = TimerAction(
        period=5.0,  # Wait for planner to be active
        actions=[
            Node(
                package='exia_ground_description',
                executable='mission_navigator_node.py',
                name='mission_navigator',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'path_type': path_type},
                    {'path_scale': 3.0},
                    {'target_speed': 1.0},
                    {'lookahead_distance': 2.0},
                    {'goal_tolerance': 1.0},
                    {'obstacle_lookahead': 5.0},
                    {'detour_clearance': 3.0},
                    {'auto_start': True},
                ],
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_params_file_arg,
        path_type_arg,

        # Start planner server (creates its own internal costmap)
        planner_server_node,

        # Lifecycle manager (activates planner + its internal costmap)
        TimerAction(period=2.0, actions=[lifecycle_manager]),

        # Start mission navigator (after lifecycle activation)
        mission_navigator_node,
    ])
