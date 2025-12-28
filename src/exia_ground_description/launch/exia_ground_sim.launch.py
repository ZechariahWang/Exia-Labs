from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for Exia Ground robot simulation with IMU sensor.

    This launch file supports both simulation and hardware modes:
    - Simulation mode: Uses Gazebo IMU plugin (default)
    - Hardware mode: Designed for future Yahboom IMU hardware integration

    Launch arguments:
        use_sim_time (bool): Use simulation time from Gazebo (default: true)
        use_hardware_imu (bool): Use hardware IMU instead of simulated (default: false)
        imu_frame_id (str): Frame ID for IMU data (default: imu_link)
    """

    exia_share = get_package_share_directory('exia_ground_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(exia_share, 'urdf', 'exia_ground.urdf')

    # Read URDF file
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # ==================== LAUNCH ARGUMENTS ====================

    # sim imu
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Hardware IMU flag, i think we will use when real imu is connected
    use_hardware_imu_arg = DeclareLaunchArgument(
        'use_hardware_imu',
        default_value='false',
        description='Use hardware IMU (Yahboom) instead of simulated IMU'
    )

    # IMU frame ID - should match URDF imu_link
    imu_frame_id_arg = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame ID for IMU sensor data'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Note: use_hardware_imu and imu_frame_id will be used when hardware IMU support is added

    # ==================== GAZEBO SIMULATION ====================

    # Launch Gazebo using the gazebo_ros launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true'
        }.items()
    )

    # ==================== ROBOT STATE PUBLISHER ====================

    # Robot state publisher - publishes TF transforms from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # ==================== SPAWN ROBOT IN GAZEBO ====================

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'exia_ground',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.05'
        ],
        output='screen'
    )

    # ==================== BUILD LAUNCH DESCRIPTION ====================

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        use_hardware_imu_arg,
        imu_frame_id_arg,

        # Gazebo and robot
        gazebo,
        robot_state_publisher,
        spawn_entity,

        # Note dec 28, 2025: im pretty sure IMU data is published by the Gazebo plugin configured in the URDF

        # Topic: /imu/data (sensor_msgs/Imu)
        # Frame: imu_link
        #
        # For hardware deployment with Yahboom IMU, i have to make sure:
        # 1. Set use_hardware_imu:=true
        # 2. Add Yahboom IMU driver node (e.g., wit_ros2_imu or custom serial driver)
        # 3. Ensure driver publishes to /imu/data with frame_id=imu_link
    ])
