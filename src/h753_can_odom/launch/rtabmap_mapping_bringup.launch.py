from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def load_params(path: Path, node_name: str) -> dict:
    try:
        with path.open('r', encoding='utf-8') as file:
            data = yaml.safe_load(file) or {}
    except OSError:
        return {}
    return data.get(node_name, {}).get('ros__parameters', {})


def generate_launch_description():
    h753_share = Path(get_package_share_directory('h753_can_odom'))
    ydlidar_share = Path(get_package_share_directory('ydlidar_ros2_driver'))
    sensor_tf = load_params(h753_share / 'config' / 'h753_sensor_tf.yaml', 'sensor_tf')

    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_camera = LaunchConfiguration('launch_camera')
    launch_odom = LaunchConfiguration('launch_odom')
    launch_imu_odom = LaunchConfiguration('launch_imu_odom')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rtabmap_params = LaunchConfiguration('rtabmap_params')
    odom_params = LaunchConfiguration('odom_params')
    imu_odom_params = LaunchConfiguration('imu_odom_params')
    rviz_config = LaunchConfiguration('rviz_config')

    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')

    imu_odom_condition = IfCondition(PythonExpression([
        "'", launch_odom, "' == 'true' and '", launch_imu_odom, "' == 'true'",
    ]))
    normal_odom_condition = IfCondition(PythonExpression([
        "'", launch_odom, "' == 'true' and '", launch_imu_odom, "' != 'true'",
    ]))

    return LaunchDescription([
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument('launch_odom', default_value='true'),
        DeclareLaunchArgument('launch_imu_odom', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('launch_uart_bridge', default_value='true'),
        DeclareLaunchArgument(
            'uart_bridge_params',
            default_value=str(h753_share / 'config' / 'h753_cmd_vel_uart_bridge.yaml'),
        ),
        DeclareLaunchArgument(
            'rtabmap_params',
            default_value=str(h753_share / 'config' / 'h753_rtabmap.yaml'),
        ),
        DeclareLaunchArgument(
            'odom_params',
            default_value=str(h753_share / 'config' / 'h753_can_odom.yaml'),
        ),
        DeclareLaunchArgument(
            'imu_odom_params',
            default_value=str(h753_share / 'config' / 'h753_imu_odom_fusion.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(h753_share / 'rviz' / 'mapping.rviz'),
        ),
        DeclareLaunchArgument('base_frame', default_value=str(sensor_tf.get('base_frame', 'base_link'))),
        DeclareLaunchArgument('laser_frame', default_value=str(sensor_tf.get('laser_frame', 'laser_frame'))),
        DeclareLaunchArgument('laser_x', default_value=str(sensor_tf.get('laser_x', 0.0))),
        DeclareLaunchArgument('laser_y', default_value=str(sensor_tf.get('laser_y', 0.0))),
        DeclareLaunchArgument('laser_z', default_value=str(sensor_tf.get('laser_z', 0.02))),
        DeclareLaunchArgument('laser_roll', default_value=str(sensor_tf.get('laser_roll', 0.0))),
        DeclareLaunchArgument('laser_pitch', default_value=str(sensor_tf.get('laser_pitch', 0.0))),
        DeclareLaunchArgument('laser_yaw', default_value=str(sensor_tf.get('laser_yaw', 0.0))),

        # Sensors bringup (LiDAR + RealSense)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(h753_share / 'launch' / 'sensors_bringup.launch.py')
            ),
            launch_arguments={
                'launch_lidar': launch_lidar,
                'launch_camera': launch_camera,
                'enable_imu': 'true',
                'publish_laser_tf': 'true',
                'lidar_params': str(ydlidar_share / 'params' / 'Tmini.yaml'),
                'base_frame': base_frame,
                'laser_frame': laser_frame,
                'laser_x': laser_x,
                'laser_y': laser_y,
                'laser_z': laser_z,
                'laser_roll': laser_roll,
                'laser_pitch': laser_pitch,
                'laser_yaw': laser_yaw,
            }.items(),
        ),

        # Encoder odom (normal mode — no IMU fusion)
        Node(
            package='h753_can_odom',
            executable='can_odom_node',
            name='h753_can_odom',
            output='screen',
            parameters=[odom_params],
            condition=normal_odom_condition,
        ),

        # Encoder odom (IMU fusion mode — publish to wheel/odom, TF off)
        Node(
            package='h753_can_odom',
            executable='can_odom_node',
            name='h753_can_odom',
            output='screen',
            parameters=[odom_params, {'publish_tf': False}],
            remappings=[
                ('odom', 'wheel/odom'),
                ('odom_vel', 'wheel/odom_vel'),
            ],
            condition=imu_odom_condition,
        ),

        # IMU + encoder fusion
        Node(
            package='h753_can_odom',
            executable='imu_odom_fusion_node',
            name='h753_imu_odom_fusion',
            output='screen',
            parameters=[imu_odom_params],
            condition=imu_odom_condition,
        ),

        # RTAB-Map (replaces slam_toolbox)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_params],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('scan', '/scan'),
                ('odom', '/odom'),
            ],
            arguments=['--delete_db_on_start'],
        ),

        # UART bridge (forward /cmd_vel_safe to STM32)
        Node(
            package='h753_can_odom',
            executable='cmd_vel_uart_bridge_node',
            name='h753_cmd_vel_uart_bridge',
            output='screen',
            parameters=[LaunchConfiguration('uart_bridge_params'), {
                'cmd_vel_topic': '/cmd_vel_selected',
                'require_scan': False,
            }],
            condition=IfCondition(LaunchConfiguration('launch_uart_bridge')),
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
    ])
