from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    h753_share = Path(get_package_share_directory('h753_can_odom'))

    map_yaml = LaunchConfiguration('map')
    amcl_params = LaunchConfiguration('amcl_params')
    odom_params = LaunchConfiguration('odom_params')
    lidar_params = LaunchConfiguration('lidar_params')
    manual_drive_params = LaunchConfiguration('manual_drive_params')
    collision_monitor_params = LaunchConfiguration('collision_monitor_params')
    uart_bridge_params = LaunchConfiguration('uart_bridge_params')
    uart_port = LaunchConfiguration('uart_port')
    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_odom = LaunchConfiguration('launch_odom')
    launch_joy = LaunchConfiguration('launch_joy')
    launch_navigation = LaunchConfiguration('launch_navigation')
    launch_uart_bridge = LaunchConfiguration('launch_uart_bridge')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    nav2_bond_timeout = LaunchConfiguration('nav2_bond_timeout')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    navigation_start_delay_s = LaunchConfiguration(
        'navigation_start_delay_s'
    )
    safety_start_delay_s = LaunchConfiguration('safety_start_delay_s')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=(
                '/home/jyl1015/ros2_graduation_project_ws/'
                'maps/go2/go2_map.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'amcl_params',
            default_value=str(h753_share / 'config' / 'h753_go2_amcl.yaml'),
        ),
        DeclareLaunchArgument(
            'odom_params',
            default_value=str(h753_share / 'config' / 'h753_can_odom.yaml'),
        ),
        DeclareLaunchArgument(
            'lidar_params',
            default_value=str(
                Path(get_package_share_directory('ydlidar_ros2_driver'))
                / 'params'
                / 'Tmini.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'manual_drive_params',
            default_value=str(
                h753_share / 'config' / 'h753_go2_manual_drive.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=str(h753_share / 'config' / 'h753_nav2.yaml'),
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=str(
                h753_share
                / 'behavior_trees'
                / 'navigate_to_pose_w_replanning_0_5hz.xml'
            ),
            description='NavigateToPose BT with reduced global replanning load.',
        ),
        DeclareLaunchArgument(
            'collision_monitor_params',
            default_value=str(
                h753_share
                / 'config'
                / 'h753_go2_manual_collision_monitor.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'uart_bridge_params',
            default_value=str(
                h753_share / 'config' / 'h753_cmd_vel_uart_bridge.yaml'
            ),
        ),
        DeclareLaunchArgument('uart_port', default_value=''),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_odom', default_value='true'),
        DeclareLaunchArgument('launch_joy', default_value='true'),
        DeclareLaunchArgument('launch_navigation', default_value='false'),
        DeclareLaunchArgument('launch_uart_bridge', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('nav2_bond_timeout', default_value='15.0'),
        DeclareLaunchArgument(
            'navigation_start_delay_s',
            default_value='5.0',
            description=(
                'Let map_server, AMCL and safety activate before Nav2.'
            ),
        ),
        DeclareLaunchArgument(
            'safety_start_delay_s',
            default_value='1.5',
            description='Create collision_monitor before its manager.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(
                Path(get_package_share_directory('nav2_bringup'))
                / 'rviz'
                / 'nav2_default_view.rviz'
            ),
        ),

        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(
                            h753_share
                            / 'launch'
                            / 'go2_amcl_test_bringup.launch.py'
                        )
                    ),
                    launch_arguments={
                        'map': map_yaml,
                        'amcl_params': amcl_params,
                        'odom_params': odom_params,
                        'lidar_params': lidar_params,
                        'launch_lidar': launch_lidar,
                        'launch_odom': launch_odom,
                        'launch_rviz': launch_rviz,
                        'rviz_config': rviz_config,
                    }.items(),
                ),
            ],
        ),
        TimerAction(
            condition=IfCondition(launch_navigation),
            period=navigation_start_delay_s,
            actions=[
                GroupAction(
                    scoped=True,
                    actions=[
                        SetParameter(
                            name='bond_timeout',
                            value=ParameterValue(
                                nav2_bond_timeout,
                                value_type=float,
                            ),
                        ),
                        SetParameter(
                            name='default_nav_to_pose_bt_xml',
                            value=ParameterValue(
                                nav_to_pose_bt_xml,
                                value_type=str,
                            ),
                        ),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                str(
                                    Path(
                                        get_package_share_directory(
                                            'nav2_bringup'
                                        )
                                    )
                                    / 'launch'
                                    / 'navigation_launch.py'
                                )
                            ),
                            launch_arguments={
                                'use_sim_time': 'false',
                                'autostart': 'true',
                                'params_file': nav2_params,
                                'use_composition': 'False',
                                'use_respawn': 'False',
                            }.items(),
                        ),
                    ],
                ),
            ],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.15,
                'autorepeat_rate': 20.0,
            }],
            condition=IfCondition(launch_joy),
        ),
        Node(
            package='h753_can_odom',
            executable='go2_manual_drive_node',
            name='h753_go2_manual_drive',
            output='screen',
            parameters=[
                manual_drive_params,
                {
                    'navigation_enabled': ParameterValue(
                        launch_navigation,
                        value_type=bool,
                    ),
                },
            ],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[collision_monitor_params],
        ),
        TimerAction(
            period=safety_start_delay_s,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_go2_collision_monitor',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': ['collision_monitor'],
                    }],
                ),
            ],
        ),
        Node(
            package='h753_can_odom',
            executable='cmd_vel_uart_bridge_node',
            name='h753_cmd_vel_uart_bridge',
            output='screen',
            parameters=[uart_bridge_params, {'uart_port': uart_port}],
            condition=IfCondition(launch_uart_bridge),
        ),
    ])
