from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    realsense_params = LaunchConfiguration('realsense_params')
    enable_imu = LaunchConfiguration('enable_imu')
    launch_odom = LaunchConfiguration('launch_odom')
    launch_imu_odom = LaunchConfiguration('launch_imu_odom')
    launch_map_odom = LaunchConfiguration('launch_map_odom')
    launch_rviz = LaunchConfiguration('launch_rviz')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')
    lidar_params = LaunchConfiguration('lidar_params')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    gyro_fps = LaunchConfiguration('gyro_fps')
    accel_fps = LaunchConfiguration('accel_fps')
    odom_params = LaunchConfiguration('odom_params')
    imu_odom_params = LaunchConfiguration('imu_odom_params')
    localization_params = LaunchConfiguration('localization_params')
    posegraph_file = LaunchConfiguration('posegraph_file')
    localization_backend = LaunchConfiguration('localization_backend')
    static_map_yaml = LaunchConfiguration('static_map_yaml')
    amcl_params = LaunchConfiguration('amcl_params')
    rviz_config = LaunchConfiguration('rviz_config')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')

    sensor_pose_args = {
        'base_frame': base_frame,
        'laser_frame': laser_frame,
        'laser_x': laser_x,
        'laser_y': laser_y,
        'laser_z': laser_z,
        'laser_roll': laser_roll,
        'laser_pitch': laser_pitch,
        'laser_yaw': laser_yaw,
    }

    return LaunchDescription([
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument(
            'realsense_params',
            default_value=str(h753_share / 'config' / 'h753_realsense.yaml'),
            description='RealSense profile forwarded to sensors_bringup.',
        ),
        DeclareLaunchArgument('enable_imu', default_value='false'),
        DeclareLaunchArgument('launch_odom', default_value='true'),
        DeclareLaunchArgument('launch_imu_odom', default_value='false'),
        DeclareLaunchArgument('launch_map_odom', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('publish_laser_tf', default_value='true'),
        DeclareLaunchArgument(
            'lidar_params',
            default_value=str(ydlidar_share / 'params' / 'Tmini.yaml'),
        ),
        DeclareLaunchArgument('unite_imu_method', default_value='2'),
        DeclareLaunchArgument('gyro_fps', default_value='200'),
        DeclareLaunchArgument('accel_fps', default_value='100'),
        DeclareLaunchArgument(
            'odom_params',
            default_value=str(h753_share / 'config' / 'h753_can_odom.yaml'),
        ),
        DeclareLaunchArgument(
            'imu_odom_params',
            default_value=str(h753_share / 'config' / 'h753_imu_odom_fusion.yaml'),
        ),
        DeclareLaunchArgument(
            'localization_params',
            default_value=str(h753_share / 'config' / 'h753_slam_toolbox_localization.yaml'),
        ),
        DeclareLaunchArgument(
            'posegraph_file',
            default_value='/home/jyl1015/ros2_graduation_project_ws/maps/h753_map',
            description='Serialized slam_toolbox posegraph base path without extension.',
        ),
        DeclareLaunchArgument(
            'localization_backend',
            default_value='slam_toolbox',
            choices=['slam_toolbox', 'amcl'],
            description='Saved-map localization implementation.',
        ),
        DeclareLaunchArgument(
            'static_map_yaml',
            default_value=(
                '/home/jyl1015/ros2_graduation_project_ws/'
                'maps/go2/go2_map.yaml'
            ),
            description='Occupancy-map YAML loaded by map_server for AMCL.',
        ),
        DeclareLaunchArgument(
            'amcl_params',
            default_value=str(h753_share / 'config' / 'h753_go2_amcl.yaml'),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(h753_share / 'launch' / 'sensors_bringup.launch.py')
            ),
            launch_arguments={
                'launch_lidar': launch_lidar,
                'launch_camera': launch_camera,
                'realsense_params': realsense_params,
                'enable_imu': enable_imu,
                'publish_laser_tf': publish_laser_tf,
                'lidar_params': lidar_params,
                'unite_imu_method': unite_imu_method,
                'gyro_fps': gyro_fps,
                'accel_fps': accel_fps,
                **sensor_pose_args,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(h753_share / 'launch' / 'mapping_bringup.launch.py')
            ),
            launch_arguments={
                'launch_lidar': 'false',
                'launch_camera': 'false',
                'enable_imu': enable_imu,
                'publish_laser_tf': 'false',
                'launch_odom': launch_odom,
                'launch_imu_odom': launch_imu_odom,
                'launch_slam': 'false',
                'launch_map_odom': 'false',
                'launch_rviz': 'false',
                'lidar_params': lidar_params,
                'unite_imu_method': unite_imu_method,
                'gyro_fps': gyro_fps,
                'accel_fps': accel_fps,
                'odom_params': odom_params,
                'imu_odom_params': imu_odom_params,
                **sensor_pose_args,
            }.items(),
        ),
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                localization_params,
                {'map_file_name': posegraph_file},
            ],
            condition=IfCondition(PythonExpression([
                "'", localization_backend, "' == 'slam_toolbox'",
            ])),
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                amcl_params,
                {
                    'yaml_filename': ParameterValue(
                        static_map_yaml,
                        value_type=str,
                    ),
                },
            ],
            condition=IfCondition(PythonExpression([
                "'", localization_backend, "' == 'amcl'",
            ])),
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params],
            condition=IfCondition(PythonExpression([
                "'", localization_backend, "' == 'amcl'",
            ])),
        ),
        TimerAction(
            # Sensors, map_server and AMCL are inside a scoped parent launch.
            # A literal delay remains valid after that parent scope exits.
            period=1.5,
            condition=IfCondition(PythonExpression([
                "'", localization_backend, "' == 'amcl'",
            ])),
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'bond_timeout': 15.0,
                        'node_names': ['map_server', 'amcl'],
                    }],
                ),
            ],
        ),
        Node(
            package='h753_can_odom',
            executable='map_odom_publisher_node',
            name='h753_map_odom_publisher',
            output='screen',
            parameters=[{
                'map_frame_id': 'map',
                'base_frame_id': 'base_link',
                'odom_topic': 'map_odom',
                'twist_topic': 'odom_vel',
                'publish_rate_hz': 5.0,
            }],
            condition=IfCondition(launch_map_odom),
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
