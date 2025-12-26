from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    exia_share = get_package_share_directory('exia_ground_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(exia_share, 'urdf', 'exia_ground.urdf')

    # Read URDF file
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # Launch Gazebo using the gazebo_ros launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true'
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

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

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
