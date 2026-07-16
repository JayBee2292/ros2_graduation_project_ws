from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    odom_config_params = load_params(h753_share / 'config' / 'h753_can_odom.yaml', 'h753_can_odom')
    sensor_tf = load_params(h753_share / 'config' / 'h753_sensor_tf.yaml', 'sensor_tf')

    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_camera = LaunchConfiguration('launch_camera')
    enable_imu = LaunchConfiguration('enable_imu')
    launch_odom = LaunchConfiguration('launch_odom')
    launch_rviz = LaunchConfiguration('launch_rviz')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')
    lidar_params = LaunchConfiguration('lidar_params')
    realsense_params = LaunchConfiguration('realsense_params')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    gyro_fps = LaunchConfiguration('gyro_fps')
    accel_fps = LaunchConfiguration('accel_fps')
    odom_params = LaunchConfiguration('odom_params')
    rviz_config = LaunchConfiguration('rviz_config')
    odom_linear_sign = LaunchConfiguration('odom_linear_sign')
    odom_angular_sign = LaunchConfiguration('odom_angular_sign')
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
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description='Start YDLIDAR for scan direction calibration.',
        ),
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Start D435i for IMU yaw-axis and sign calibration.',
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='true',
            description='Enable D435i IMU for integrated SLAM/Nav2 odom calibration.',
        ),
        DeclareLaunchArgument(
            'launch_odom',
            default_value='true',
            description='Start raw wheel odom without IMU fusion.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Start RViz for odom arrow and scan inspection.',
        ),
        DeclareLaunchArgument(
            'publish_laser_tf',
            default_value='true',
            description='Publish base_link -> laser_frame static transform.',
        ),
        DeclareLaunchArgument(
            'lidar_params',
            default_value=str(ydlidar_share / 'params' / 'Tmini.yaml'),
            description='Path to YDLIDAR parameter file.',
        ),
        DeclareLaunchArgument(
            'realsense_params',
            default_value=str(h753_share / 'config' / 'h753_realsense_imu.yaml'),
            description='Lightweight RGB plus IMU RealSense profile for calibration.',
        ),
        DeclareLaunchArgument(
            'unite_imu_method',
            default_value='2',
            description='Forwarded only when launch_camera and enable_imu are true.',
        ),
        DeclareLaunchArgument('gyro_fps', default_value='200'),
        DeclareLaunchArgument('accel_fps', default_value='100'),
        DeclareLaunchArgument(
            'odom_params',
            default_value=str(h753_share / 'config' / 'h753_can_odom.yaml'),
            description='Path to h753_can_odom parameter file.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(h753_share / 'rviz' / 'calibration.rviz'),
            description='Path to RViz config file.',
        ),
        DeclareLaunchArgument(
            'odom_linear_sign',
            default_value=str(odom_config_params.get('odom_linear_sign', 1.0)),
            description='Use -1.0 if forward tape motion decreases /odom x.',
        ),
        DeclareLaunchArgument(
            'odom_angular_sign',
            default_value=str(odom_config_params.get('odom_angular_sign', 1.0)),
            description='Use -1.0 if counter-clockwise rotation gives negative yaw.',
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
                'enable_imu': enable_imu,
                'publish_laser_tf': publish_laser_tf,
                'lidar_params': lidar_params,
                'realsense_params': realsense_params,
                'unite_imu_method': unite_imu_method,
                'gyro_fps': gyro_fps,
                'accel_fps': accel_fps,
                **sensor_pose_args,
            }.items(),
        ),
        Node(
            package='h753_can_odom',
            executable='can_odom_node',
            name='h753_can_odom',
            output='screen',
            parameters=[
                odom_params,
                {
                    'publish_tf': True,
                    'odom_linear_sign': ParameterValue(odom_linear_sign, value_type=float),
                    'odom_angular_sign': ParameterValue(odom_angular_sign, value_type=float),
                },
            ],
            condition=IfCondition(launch_odom),
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
