#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
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
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
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
        ],
    )

    unpause_sim = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ign', 'service', '-s', '/world/exia_forest/control',
                     '--reqtype', 'ignition.msgs.WorldControl',
                     '--reptype', 'ignition.msgs.Boolean',
                     '--timeout', '3000',
                     '--req', 'pause: false'],
                output='screen',
            )
        ],
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ],
    )

    steering_controller_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['steering_controller'],
                output='screen',
            )
        ],
    )

    throttle_controller_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['throttle_controller'],
                output='screen',
            )
        ],
    )

    brake_controller_spawner = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['brake_controller'],
                output='screen',
            )
        ],
    )

    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar_3d/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        remappings=[
            ('/lidar_3d/points', '/points'),
            ('/imu', '/imu/data'),
            ('/navsat', '/gps/fix'),
            ('/depth_camera/points', '/depth/points'),
            ('/depth_camera/image', '/camera/color/image_raw'),
            ('/depth_camera/depth_image', '/camera/depth/image_raw'),
        ],
        output='screen',
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
        unpause_sim,
        joint_state_broadcaster_spawner,
        steering_controller_spawner,
        throttle_controller_spawner,
        brake_controller_spawner,
        gz_ros_bridge,
        ackermann_drive,
    ])
