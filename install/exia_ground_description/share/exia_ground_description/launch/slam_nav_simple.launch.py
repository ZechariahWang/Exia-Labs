"""
Exia Ground Robot - Simplified SLAM + Navigation Launch File

Launches SLAM and navigation WITHOUT the full Nav2 behavior tree.
Uses only the essential components:
- slam_toolbox for mapping
- Nav2 costmaps for obstacle detection
- Nav2 planner server for A* planning
- Our custom navigation node for path execution

This avoids the bt_navigator which can have library compatibility issues.

Usage:
    # Terminal 1: Start simulation
    ros2 launch exia_ground_description exia_ground_sim.launch.py

    # Terminal 2: Start SLAM + Navigation (simplified)
    ros2 launch exia_ground_description slam_nav_simple.launch.py

Author: Zechariah Wang
Date: December 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('exia_ground_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

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

    # ==================== SLAM ====================
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

    # ==================== Planner Server ====================
    # Note: planner_server creates its own global_costmap internally
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    # ==================== Lifecycle Manager ====================
    # Manages the lifecycle of Nav2 nodes
    # Note: planner_server has its own internal global_costmap that it manages
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

    # ==================== Our Navigation Node ====================
    navigation_node = TimerAction(
        period=8.0,  # Wait for all Nav2 nodes to be active
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

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        nav2_params_file_arg,

        # Start SLAM first
        slam_toolbox_node,

        # Start planner (has its own internal costmap)
        TimerAction(period=3.0, actions=[planner_server_node]),

        # Lifecycle manager (activates planner + its internal costmap)
        TimerAction(period=4.0, actions=[lifecycle_manager]),

        # Start our navigation node (after lifecycle activation)
        navigation_node,
    ])
