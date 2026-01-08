# Exia Ground - Gazebo Fortress Simulation Launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('exia_ground_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_path = os.path.join(pkg_share, 'urdf', 'exia_ground.urdf.xacro')
    hardware_config = os.path.join(pkg_share, 'config', 'hardware_config.yaml')
    controller_config = os.path.join(pkg_share, 'config', 'ackermann_controllers.yaml')

    robot_description = xacro.process_file(
        xacro_path, mappings={'config_path': hardware_config}).toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ackermann_drive = LaunchConfiguration('use_ackermann_drive')

    # Gazebo with custom world
    world_file = os.path.join(pkg_share, 'worlds', 'exia_world.sdf')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'exia_ground', '-topic', 'robot_description', '-z', '0.15'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/imu', '/imu/data'),
            ('/lidar', '/scan'),
            ('/lidar_3d/points', '/points'),
        ],
        output='screen'
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30', '--param-file', controller_config])

    steering_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['steering_controller', '--controller-manager-timeout', '30', '--param-file', controller_config])

    throttle_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['throttle_controller', '--controller-manager-timeout', '30', '--param-file', controller_config])

    brake_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['brake_controller', '--controller-manager-timeout', '30', '--param-file', controller_config])

    # Ackermann drive node
    ackermann_drive_node = Node(
        package='exia_ground_description',
        executable='ackermann_drive_node.py',
        name='ackermann_drive_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheelbase': 1.3, 'track_width': 1.1, 'wheel_radius': 0.3,
            'max_steering_angle': 0.6, 'max_speed': 5.0,
            'max_acceleration': 2.0, 'max_deceleration': 5.0,
            'steering_rate_limit': 2.0, 'hal_type': 'simulation',
            'cmd_timeout': 0.5, 'auto_brake_on_stop': True,
        }],
        output='screen',
        condition=IfCondition(use_ackermann_drive),
    )

    # Fallback odometry node
    odometry_node = Node(
        package='exia_ground_description',
        executable='ackermann_odometry.py',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", use_ackermann_drive, "' == 'false'"])),
    )

    # Controller spawning sequence with delays
    delayed_controller_spawner = TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])

    delay_steering = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[steering_controller_spawner]))
    delay_throttle = RegisterEventHandler(
        OnProcessExit(target_action=steering_controller_spawner, on_exit=[throttle_controller_spawner]))
    delay_brake = RegisterEventHandler(
        OnProcessExit(target_action=throttle_controller_spawner, on_exit=[brake_controller_spawner]))
    delay_ackermann = RegisterEventHandler(
        OnProcessExit(target_action=brake_controller_spawner, on_exit=[ackermann_drive_node, odometry_node]))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_ackermann_drive', default_value='true'),
        gz_sim, robot_state_publisher, spawn_entity, bridge,
        delayed_controller_spawner, delay_steering, delay_throttle, delay_brake, delay_ackermann,
    ])
