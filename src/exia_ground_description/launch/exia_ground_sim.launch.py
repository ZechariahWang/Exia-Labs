"""
Exia Ground Robot - Gazebo Fortress Simulation Launch File

This launch file starts the complete Ackermann drive simulation:
  - Gazebo Fortress with empty world
  - Robot spawn and state publisher
  - ros2_control with three controllers (steering, throttle, brake)
  - Ackermann drive node with HAL
  - Odometry publisher

Three-Motor Architecture:
  1. Steering Controller - Front wheel position control
  2. Throttle Controller - Rear wheel velocity control
  3. Brake Controller    - Brake actuator position control

Usage:
  ros2 launch exia_ground_description exia_ground_sim.launch.py
  ros2 launch exia_ground_description exia_ground_sim.launch.py use_ackermann_drive:=true

Author: Zechariah Wang
Date: December 2025
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('exia_ground_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_path = os.path.join(pkg_share, 'urdf', 'exia_ground.urdf.xacro')
    # Minimal config for gz_ros2_control hardware interface
    hardware_config = os.path.join(pkg_share, 'config', 'hardware_config.yaml')
    # Full controller config for spawners
    controller_config = os.path.join(pkg_share, 'config', 'ackermann_controllers.yaml')

    robot_description = xacro.process_file(
        xacro_path,
        mappings={'config_path': hardware_config}
    ).toxml()

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ackermann_drive = LaunchConfiguration('use_ackermann_drive')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    use_ackermann_drive_arg = DeclareLaunchArgument(
        'use_ackermann_drive',
        default_value='true',
        description='Use unified Ackermann drive node instead of simple odometry'
    )

    # ==================== Gazebo Fortress ====================
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # ==================== Robot State Publisher ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # ==================== Spawn Robot ====================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'exia_ground',
            '-topic', 'robot_description',
            '-z', '0.15'
        ],
        output='screen'
    )

    # ==================== Gazebo-ROS Bridge ====================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            ('/imu', '/imu/data'),
        ],
        output='screen'
    )

    # ==================== Controller Spawners ====================
    # gz_ros2_control only gets minimal hardware config.
    # Spawners load and configure controllers from full config file.

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '30',
            '--param-file', controller_config,
        ],
    )

    # Steering Controller
    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'steering_controller',
            '--controller-manager-timeout', '30',
            '--param-file', controller_config,
        ],
    )

    # Throttle Controller
    throttle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'throttle_controller',
            '--controller-manager-timeout', '30',
            '--param-file', controller_config,
        ],
    )

    # Brake Controller
    brake_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'brake_controller',
            '--controller-manager-timeout', '30',
            '--param-file', controller_config,
        ],
    )

    # ==================== Ackermann Drive Node ====================
    # Unified controller that handles cmd_vel -> steering/throttle/brake
    ackermann_drive_node = Node(
        package='exia_ground_description',
        executable='ackermann_drive_node.py',
        name='ackermann_drive_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheelbase': 0.4,
            'track_width': 0.45,
            'wheel_radius': 0.1,
            'max_steering_angle': 0.6,
            'max_speed': 5.0,
            'max_acceleration': 2.0,
            'max_deceleration': 5.0,
            'steering_rate_limit': 2.0,
            'hal_type': 'simulation',
            'cmd_timeout': 0.5,
            'auto_brake_on_stop': True,
        }],
        output='screen',
        condition=IfCondition(use_ackermann_drive),
    )

    # ==================== Simple Odometry Node (fallback) ====================
    # Used when use_ackermann_drive is false
    odometry_node = Node(
        package='exia_ground_description',
        executable='ackermann_odometry.py',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", use_ackermann_drive, "' == 'false'"])),
    )

    # ==================== Controller Spawning Sequence ====================
    # Use TimerAction to give Gazebo time to fully initialize before spawning controllers.
    # gz_ros2_control needs the simulation to be running before it can configure hardware.

    # Wait 3 seconds after spawn for Gazebo to fully initialize, then spawn controllers
    delayed_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
        ]
    )

    # Chain remaining controllers after joint_state_broadcaster
    delay_steering = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[steering_controller_spawner]
        )
    )

    delay_throttle = RegisterEventHandler(
        OnProcessExit(
            target_action=steering_controller_spawner,
            on_exit=[throttle_controller_spawner]
        )
    )

    delay_brake = RegisterEventHandler(
        OnProcessExit(
            target_action=throttle_controller_spawner,
            on_exit=[brake_controller_spawner]
        )
    )

    # Start Ackermann drive node after all controllers are loaded
    delay_ackermann = RegisterEventHandler(
        OnProcessExit(
            target_action=brake_controller_spawner,
            on_exit=[ackermann_drive_node, odometry_node]
        )
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        use_ackermann_drive_arg,

        # Simulation
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,

        # Controller spawning sequence (with initial delay for Gazebo initialization)
        delayed_controller_spawner,
        delay_steering,
        delay_throttle,
        delay_brake,
        delay_ackermann,
    ])
