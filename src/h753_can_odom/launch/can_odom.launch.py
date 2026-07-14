import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('h753_can_odom')
    default_params = os.path.join(package_share, 'config', 'h753_can_odom.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the h753_can_odom parameter file.',
        ),
        Node(
            package='h753_can_odom',
            executable='can_odom_node',
            name='h753_can_odom',
            output='screen',
            parameters=[params_file],
        ),
    ])
