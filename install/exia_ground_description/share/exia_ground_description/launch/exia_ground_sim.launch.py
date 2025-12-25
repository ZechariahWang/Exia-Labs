from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    exia_share = get_package_share_directory('exia_ground_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(exia_share, 'urdf', 'exia_ground.urdf')
    world_path = os.path.join(gazebo_ros_share, 'worlds', 'empty.world')

    return LaunchDescription([

        # Launch Gazebo SERVER ONLY (no GUI, no gzclient)
        ExecuteProcess(
            cmd=[
                'gzserver',
                world_path,
                '-slibgazebo_ros_init.so',
                '-slibgazebo_ros_factory.so',
                '-slibgazebo_ros_force_system.so'
            ],
            output='screen'
        ),

        # Spawn Exia robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'exia_ground',
                '-file', urdf_path,
                '-x', '0',
                '-y', '0',
                '-z', '0.2'
            ],
            output='screen'
        )
    ])
