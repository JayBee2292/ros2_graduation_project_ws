from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    h753_share = Path(get_package_share_directory('h753_can_odom'))
    perception_share = Path(get_package_share_directory('h753_perception'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))

    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_camera = LaunchConfiguration('launch_camera')
    launch_vlm_gateway = LaunchConfiguration('launch_vlm_gateway')
    launch_yolo_perception = LaunchConfiguration('launch_yolo_perception')
    yolo_python_executable = LaunchConfiguration('yolo_python_executable')
    yolo_params = LaunchConfiguration('yolo_params')
    yolo_show_window = LaunchConfiguration('yolo_show_window')
    yolo_tuning_mode = LaunchConfiguration('yolo_tuning_mode')
    realsense_params = LaunchConfiguration('realsense_params')
    enable_imu = LaunchConfiguration('enable_imu')
    launch_odom = LaunchConfiguration('launch_odom')
    launch_imu_odom = LaunchConfiguration('launch_imu_odom')
    launch_map_odom = LaunchConfiguration('launch_map_odom')
    launch_uart_bridge = LaunchConfiguration('launch_uart_bridge')
    launch_rviz = LaunchConfiguration('launch_rviz')
    posegraph_file = LaunchConfiguration('posegraph_file')
    localization_params = LaunchConfiguration('localization_params')
    nav2_params = LaunchConfiguration('nav2_params')
    collision_monitor_params = LaunchConfiguration('collision_monitor_params')
    uart_bridge_params = LaunchConfiguration('uart_bridge_params')
    vlm_gateway_params = LaunchConfiguration('vlm_gateway_params')
    uart_port = LaunchConfiguration('uart_port')
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_bond_timeout = LaunchConfiguration('nav2_bond_timeout')

    return LaunchDescription([
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument('launch_vlm_gateway', default_value='true'),
        DeclareLaunchArgument('launch_yolo_perception', default_value='false'),
        DeclareLaunchArgument(
            'yolo_python_executable',
            default_value='/home/jyl1015/yolo_project/venv_gpu/bin/python3',
        ),
        DeclareLaunchArgument(
            'yolo_params',
            default_value=str(
                perception_share / 'config' / 'h753_yolo_perception.yaml'
            ),
        ),
        DeclareLaunchArgument('yolo_show_window', default_value='false'),
        DeclareLaunchArgument('yolo_tuning_mode', default_value='false'),
        DeclareLaunchArgument(
            'realsense_params',
            default_value=str(h753_share / 'config' / 'h753_realsense.yaml'),
            description='RealSense profile selected by the mode manager.',
        ),
        DeclareLaunchArgument('enable_imu', default_value='false'),
        DeclareLaunchArgument('launch_odom', default_value='true'),
        DeclareLaunchArgument('launch_imu_odom', default_value='false'),
        DeclareLaunchArgument('launch_map_odom', default_value='false'),
        DeclareLaunchArgument(
            'launch_uart_bridge',
            default_value='true',
            description='Forward collision-monitored Nav2 /cmd_vel_safe to STM UART.',
        ),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument(
            'nav2_bond_timeout',
            default_value='15.0',
            description='Lifecycle heartbeat timeout for a loaded Jetson.',
        ),
        DeclareLaunchArgument(
            'posegraph_file',
            default_value='/home/jyl1015/ros2_graduation_project_ws/maps/h753_map',
            description='Serialized slam_toolbox posegraph base path without extension.',
        ),
        DeclareLaunchArgument(
            'localization_params',
            default_value=str(h753_share / 'config' / 'h753_slam_toolbox_localization.yaml'),
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=str(h753_share / 'config' / 'h753_nav2.yaml'),
        ),
        DeclareLaunchArgument(
            'collision_monitor_params',
            default_value=str(h753_share / 'config' / 'h753_collision_monitor.yaml'),
        ),
        DeclareLaunchArgument(
            'uart_bridge_params',
            default_value=str(h753_share / 'config' / 'h753_cmd_vel_uart_bridge.yaml'),
        ),
        DeclareLaunchArgument(
            'vlm_gateway_params',
            default_value=str(h753_share / 'config' / 'h753_vlm_gateway.yaml'),
        ),
        DeclareLaunchArgument(
            'uart_port',
            default_value='',
            description='STM ST-LINK VCP UART path. Empty selects the ST-LINK /dev/serial/by-id path.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(h753_share / 'rviz' / 'navigation.rviz'),
        ),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(h753_share / 'launch' / 'localization_bringup.launch.py')
                    ),
                    launch_arguments={
                        'launch_lidar': launch_lidar,
                        'launch_camera': launch_camera,
                        'realsense_params': realsense_params,
                        'enable_imu': enable_imu,
                        'launch_odom': launch_odom,
                        'launch_imu_odom': launch_imu_odom,
                        'launch_map_odom': launch_map_odom,
                        'launch_rviz': 'false',
                        'localization_params': localization_params,
                        'posegraph_file': posegraph_file,
                    }.items(),
                ),
            ],
        ),
        GroupAction(
            scoped=True,
            actions=[
                SetParameter(
                    name='bond_timeout',
                    value=ParameterValue(nav2_bond_timeout, value_type=float),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(nav2_share / 'launch' / 'navigation_launch.py')
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
            executable='vlm_gateway_node',
            name='h753_vlm_gateway',
            output='screen',
            parameters=[vlm_gateway_params],
            condition=IfCondition(PythonExpression([
                "'", launch_vlm_gateway, "' == 'true' and '",
                launch_camera, "' == 'true'",
            ])),
        ),
        ExecuteProcess(
            cmd=[
                yolo_python_executable,
                '-m',
                'h753_perception.yolo_perception_node',
                '--ros-args',
                '--params-file',
                yolo_params,
                '-p',
                ['show_window:=', yolo_show_window],
                '-p',
                ['tuning_mode:=', yolo_tuning_mode],
            ],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", launch_yolo_perception, "' == 'true' and '",
                launch_camera, "' == 'true'",
            ])),
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
