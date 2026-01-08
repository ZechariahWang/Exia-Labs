from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('exia_ground_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    octomap_params_file = os.path.join(pkg_share, 'config', 'octomap_params.yaml')

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            octomap_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cloud_in', '/points'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        octomap_server_node,
    ])
