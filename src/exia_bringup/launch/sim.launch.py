#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = get_package_share_directory('exia_bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='exia_world.sdf',
        description='World file to load'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    urdf_file = os.path.join(bringup_dir, 'urdf', 'exia_ground.urdf.xacro')
    controllers_file = os.path.join(bringup_dir, 'config', 'ackermann_controllers.yaml')
    world_path = PathJoinSubstitution([bringup_dir, 'worlds', world])

    robot_description = Command(['xacro ', urdf_file, ' config_path:=', controllers_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'exia_ground',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller'],
        output='screen',
    )

    throttle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['throttle_controller'],
        output='screen',
    )

    brake_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['brake_controller'],
        output='screen',
    )

    delay_steering_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[steering_controller_spawner],
        )
    )

    delay_throttle_after_steering = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=steering_controller_spawner,
            on_exit=[throttle_controller_spawner],
        )
    )

    delay_brake_after_throttle = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=throttle_controller_spawner,
            on_exit=[brake_controller_spawner],
        )
    )

    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar_3d/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
        ],
        remappings=[
            ('/lidar_3d/points', '/points'),
            ('/imu', '/imu/data'),
            ('/navsat', '/gps/fix'),
        ],
        output='screen',
    )

    pointcloud_to_laserscan_config = os.path.join(
        bringup_dir, 'config', 'pointcloud_to_laserscan.yaml'
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

    ackermann_drive = Node(
        package='exia_control',
        executable='ackermann_drive_node',
        name='ackermann_drive',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_tf': True,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        joint_state_broadcaster_spawner,
        delay_steering_after_jsb,
        delay_throttle_after_steering,
        delay_brake_after_throttle,
        gz_ros_bridge,
        pointcloud_to_laserscan,
        ackermann_drive,
    ])
