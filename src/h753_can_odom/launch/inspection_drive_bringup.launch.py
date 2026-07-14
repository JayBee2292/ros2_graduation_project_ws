from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    h753_share = Path(get_package_share_directory('h753_can_odom'))

    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_camera = LaunchConfiguration('launch_camera')
    enable_imu = LaunchConfiguration('enable_imu')
    launch_odom = LaunchConfiguration('launch_odom')
    launch_uart_bridge = LaunchConfiguration('launch_uart_bridge')
    launch_rviz = LaunchConfiguration('launch_rviz')
    collision_monitor_params = LaunchConfiguration('collision_monitor_params')
    uart_bridge_params = LaunchConfiguration('uart_bridge_params')
    uart_port = LaunchConfiguration('uart_port')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_camera', default_value='false'),
        DeclareLaunchArgument('enable_imu', default_value='false'),
        DeclareLaunchArgument('launch_odom', default_value='true'),
        DeclareLaunchArgument('launch_uart_bridge', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument(
            'collision_monitor_params',
            default_value=str(h753_share / 'config' / 'h753_collision_monitor_modes.yaml'),
        ),
        DeclareLaunchArgument(
            'uart_bridge_params',
            default_value=str(h753_share / 'config' / 'h753_cmd_vel_uart_bridge.yaml'),
        ),
        DeclareLaunchArgument(
            'uart_port',
            default_value='',
            description='STM ST-LINK VCP UART path. Empty selects the ST-LINK /dev/serial/by-id path.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(h753_share / 'rviz' / 'calibration.rviz'),
        ),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(h753_share / 'launch' / 'slam_bringup.launch.py')
                    ),
                    launch_arguments={
                        'launch_lidar': launch_lidar,
                        'launch_camera': launch_camera,
                        'enable_imu': enable_imu,
                        'launch_odom': launch_odom,
                        'launch_imu_odom': 'false',
                        'launch_slam': 'false',
                        'launch_map_odom': 'false',
                        'launch_rviz': 'false',
                    }.items(),
                ),
            ],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[collision_monitor_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['collision_monitor'],
            }],
        ),
        Node(
            package='h753_can_odom',
            executable='cmd_vel_uart_bridge_node',
            name='h753_cmd_vel_uart_bridge',
            output='screen',
            parameters=[uart_bridge_params, {'uart_port': uart_port}],
            condition=IfCondition(launch_uart_bridge),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
    ])
