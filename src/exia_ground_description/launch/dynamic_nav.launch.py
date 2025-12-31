"""
Exia Ground Robot - Dynamic Navigation Launch File

Goal-based navigation with continuous A* replanning.
Uses SLAM for localization and Nav2 SmacPlannerHybrid for path planning.

Architecture:
- slam_toolbox: Provides map->odom TF via lidar+IMU localization
- planner_server: Nav2 Smac Hybrid-A* planner (Ackermann-aware)
- dynamic_navigator: Goal-based navigation with 500ms replanning

Usage:
    # Terminal 1: Start simulation
    ros2 launch exia_ground_description exia_ground_sim.launch.py

    # Terminal 2: Start dynamic navigation
    ros2 launch exia_ground_description dynamic_nav.launch.py

    # Terminal 3: Send goal
    ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'map'},
      pose: {position: {x: 10.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}
    }" -1

    # Or use RViz "2D Goal Pose" button

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
    slam_params_file = LaunchConfiguration('slam_params_file')

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

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml'),
        description='Path to SLAM params file'
    )

    # ==================== SLAM Toolbox ====================
    # Provides map->odom TF via lidar scan matching
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ==================== Planner Server ====================
    # Nav2 A* planner with its own internal global_costmap
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

    # ==================== Dynamic Navigator ====================
    # Our goal-based navigation node with continuous replanning
    dynamic_navigator_node = TimerAction(
        period=5.0,  # Wait for SLAM and planner to be active
        actions=[
            Node(
                package='exia_ground_description',
                executable='dynamic_navigator_node.py',
                name='dynamic_navigator',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'replan_period': 0.5},          # 500ms replanning
                    {'min_replan_interval': 0.2},    # Rate limit
                    {'goal_tolerance': 1.0},         # meters
                    {'lookahead_distance': 2.5},     # Pure Pursuit
                    {'target_speed': 1.0},           # m/s
                    {'path_switch_threshold': 2.0},  # Switch path if 2m shorter
                    {'obstacle_lookahead': 8.0},     # Obstacle detection range
                ],
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_params_file_arg,
        slam_params_file_arg,

        # Start SLAM (provides map->odom TF)
        slam_node,

        # Start planner server (creates its own internal costmap)
        TimerAction(period=2.0, actions=[planner_server_node]),

        # Lifecycle manager (activates planner + its internal costmap)
        TimerAction(period=3.0, actions=[lifecycle_manager]),

        # Start dynamic navigator (after lifecycle activation)
        dynamic_navigator_node,
    ])
