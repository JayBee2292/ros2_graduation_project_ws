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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    h753_share = Path(get_package_share_directory('h753_can_odom'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))

    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_camera = LaunchConfiguration('launch_camera')
    realsense_params = LaunchConfiguration('realsense_params')
    launch_vlm_gateway = LaunchConfiguration('launch_vlm_gateway')
    enable_imu = LaunchConfiguration('enable_imu')
    launch_odom = LaunchConfiguration('launch_odom')
    launch_imu_odom = LaunchConfiguration('launch_imu_odom')
    launch_map_odom = LaunchConfiguration('launch_map_odom')
    launch_uart_bridge = LaunchConfiguration('launch_uart_bridge')
    launch_frontier_explorer = LaunchConfiguration('launch_frontier_explorer')
    auto_explore = LaunchConfiguration('auto_explore')
    launch_rviz = LaunchConfiguration('launch_rviz')
    slam_params = LaunchConfiguration('slam_params')
    continue_mapping = LaunchConfiguration('continue_mapping')
    posegraph_file = LaunchConfiguration('posegraph_file')
    nav2_params = LaunchConfiguration('nav2_params')
    collision_monitor_params = LaunchConfiguration('collision_monitor_params')
    uart_bridge_params = LaunchConfiguration('uart_bridge_params')
    vlm_gateway_params = LaunchConfiguration('vlm_gateway_params')
    frontier_explorer_params = LaunchConfiguration('frontier_explorer_params')
    uart_port = LaunchConfiguration('uart_port')
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_bond_timeout = LaunchConfiguration('nav2_bond_timeout')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    collision_monitor_start_delay_s = LaunchConfiguration(
        'collision_monitor_start_delay_s'
    )

    return LaunchDescription([
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument(
            'realsense_params',
            default_value=str(h753_share / 'config' / 'h753_realsense_imu.yaml'),
            description='RealSense profile; mapping uses lightweight RGB plus IMU.',
        ),
        DeclareLaunchArgument('launch_vlm_gateway', default_value='false'),
        DeclareLaunchArgument('enable_imu', default_value='false'),
        DeclareLaunchArgument('launch_odom', default_value='true'),
        DeclareLaunchArgument('launch_imu_odom', default_value='false'),
        DeclareLaunchArgument('launch_map_odom', default_value='false'),
        DeclareLaunchArgument(
            'launch_uart_bridge',
            default_value='true',
            description='Forward collision-monitored Nav2 /cmd_vel_safe to STM UART.',
        ),
        DeclareLaunchArgument(
            'launch_frontier_explorer',
            default_value='false',
            description='Start the frontier goal selection node.',
        ),
        DeclareLaunchArgument(
            'auto_explore',
            default_value='false',
            description='Allow the frontier explorer to submit Nav2 goals immediately.',
        ),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument(
            'nav2_bond_timeout',
            default_value='15.0',
            description='Lifecycle heartbeat timeout for a loaded Jetson.',
        ),
        DeclareLaunchArgument(
            'collision_monitor_start_delay_s',
            default_value='1.5',
            description=(
                'Delay the collision lifecycle manager until its node services '
                'are ready.'
            ),
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value=str(h753_share / 'config' / 'h753_slam_toolbox.yaml'),
        ),
        DeclareLaunchArgument(
            'continue_mapping',
            default_value='false',
            description='Load a serialized posegraph and continue live mapping.',
        ),
        DeclareLaunchArgument(
            'posegraph_file',
            default_value='/home/jyl1015/ros2_graduation_project_ws/maps/h753_map',
            description='Serialized slam_toolbox posegraph base path without extension.',
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
            'frontier_explorer_params',
            default_value=str(h753_share / 'config' / 'h753_frontier_explorer.yaml'),
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
                        str(h753_share / 'launch' / 'slam_bringup.launch.py')
                    ),
                    launch_arguments={
                        'launch_lidar': launch_lidar,
                        'launch_camera': launch_camera,
                        'realsense_params': realsense_params,
                        'enable_imu': enable_imu,
                        'launch_odom': launch_odom,
                        'launch_imu_odom': launch_imu_odom,
                        'launch_slam': 'true',
                        'launch_map_odom': launch_map_odom,
                        'launch_rviz': 'false',
                        'slam_params': slam_params,
                        'continue_mapping': continue_mapping,
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
                SetParameter(
                    name='default_nav_to_pose_bt_xml',
                    value=ParameterValue(nav_to_pose_bt_xml, value_type=str),
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
        TimerAction(
            # Keep the lifecycle client alive until collision_monitor has
            # finished creating its transition services on a loaded Jetson.
            period=collision_monitor_start_delay_s,
            actions=[
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
            ],
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
        Node(
            package='h753_can_odom',
            executable='cmd_vel_uart_bridge_node',
            name='h753_cmd_vel_uart_bridge',
            output='screen',
            parameters=[uart_bridge_params, {'uart_port': uart_port}],
            condition=IfCondition(launch_uart_bridge),
        ),
        Node(
            package='h753_can_odom',
            executable='frontier_explorer_node',
            name='h753_frontier_explorer',
            output='screen',
            parameters=[
                frontier_explorer_params,
                {'start_enabled': ParameterValue(auto_explore, value_type=bool)},
            ],
            condition=IfCondition(launch_frontier_explorer),
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
