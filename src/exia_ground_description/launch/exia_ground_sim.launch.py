from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('exia_ground_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_path = os.path.join(pkg_share, 'urdf', 'exia_ground.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'ackermann_controllers.yaml')

    robot_description = xacro.process_file(
        xacro_path,
        mappings={'config_path': controller_config}
    ).toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Gazebo Fortress
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn robot in Gazebo
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

    # Bridge for clock and IMU
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

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file', controller_config,
        ],
    )

    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'steering_controller',
            '--param-file', controller_config,
        ],
    )

    throttle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'throttle_controller',
            '--param-file', controller_config,
        ],
    )

    # Odometry node
    odometry_node = Node(
        package='exia_ground_description',
        executable='ackermann_odometry.py',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Chain controller spawning after robot is spawned
    delay_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )
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
    delay_odom = RegisterEventHandler(
        OnProcessExit(
            target_action=throttle_controller_spawner,
            on_exit=[odometry_node]
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        delay_jsb,
        delay_steering,
        delay_throttle,
        delay_odom,
    ])
