# Exia Ground - Dynamic Navigation Launch (SLAM + A* + Navigator)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('exia_ground_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # SLAM for map->odom TF
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    # Nav2 A* planner
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    # Lifecycle manager for planner
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

    # Dynamic navigator node
    dynamic_navigator_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='exia_ground_description',
                executable='dynamic_navigator_node.py',
                name='dynamic_navigator',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'replan_period': 0.5},
                    {'min_replan_interval': 0.2},
                    {'goal_tolerance': 1.0},
                    {'lookahead_distance': 2.5},
                    {'target_speed': 1.0},
                    {'path_switch_threshold': 2.0},
                    {'obstacle_lookahead': 8.0},
                ],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav2_params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')),
        slam_node,
        TimerAction(period=2.0, actions=[planner_server_node]),
        TimerAction(period=3.0, actions=[lifecycle_manager]),
        dynamic_navigator_node,
    ])
