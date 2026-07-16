from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
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
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')
    lidar_params = LaunchConfiguration('lidar_params')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    gyro_fps = LaunchConfiguration('gyro_fps')
    accel_fps = LaunchConfiguration('accel_fps')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')
    camera_frame = LaunchConfiguration('camera_frame')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description='Start the YDLIDAR driver and publish /scan.',
        ),
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Start the Intel RealSense camera under /camera.',
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='false',
            description='Enable RealSense gyro/accel streams and /camera/camera/imu.',
        ),
        DeclareLaunchArgument(
            'realsense_params',
            default_value=str(h753_share / 'config' / 'h753_realsense.yaml'),
            description='RealSense parameter file (VLM or explicit RGB-D profile).',
        ),
        DeclareLaunchArgument(
            'publish_laser_tf',
            default_value='true',
            description='Publish static base_frame -> laser_frame transform.',
        ),
        DeclareLaunchArgument(
            'lidar_params',
            default_value=str(ydlidar_share / 'params' / 'Tmini.yaml'),
            description='Path to YDLIDAR parameter file.',
        ),
        DeclareLaunchArgument(
            'unite_imu_method',
            default_value='2',
            description='RealSense IMU unite method: 0=none, 1=copy, 2=linear interpolation.',
        ),
        DeclareLaunchArgument('gyro_fps', default_value='200'),
        DeclareLaunchArgument('accel_fps', default_value='100'),
        DeclareLaunchArgument('base_frame', default_value=str(sensor_tf.get('base_frame', 'base_link'))),
        DeclareLaunchArgument('laser_frame', default_value=str(sensor_tf.get('laser_frame', 'laser_frame'))),
        DeclareLaunchArgument('laser_x', default_value=str(sensor_tf.get('laser_x', 0.0))),
        DeclareLaunchArgument('laser_y', default_value=str(sensor_tf.get('laser_y', 0.0))),
        DeclareLaunchArgument('laser_z', default_value=str(sensor_tf.get('laser_z', 0.02))),
        DeclareLaunchArgument('laser_roll', default_value=str(sensor_tf.get('laser_roll', 0.0))),
        DeclareLaunchArgument('laser_pitch', default_value=str(sensor_tf.get('laser_pitch', 0.0))),
        DeclareLaunchArgument('laser_yaw', default_value=str(sensor_tf.get('laser_yaw', 0.0))),
        DeclareLaunchArgument('publish_camera_tf', default_value='true'),
        DeclareLaunchArgument('camera_frame', default_value=str(sensor_tf.get('camera_frame', 'camera_link'))),
        DeclareLaunchArgument('camera_x', default_value=str(sensor_tf.get('camera_x', 0.0))),
        DeclareLaunchArgument('camera_y', default_value=str(sensor_tf.get('camera_y', 0.0))),
        DeclareLaunchArgument('camera_z', default_value=str(sensor_tf.get('camera_z', 0.15))),
        DeclareLaunchArgument('camera_roll', default_value=str(sensor_tf.get('camera_roll', 0.0))),
        DeclareLaunchArgument('camera_pitch', default_value=str(sensor_tf.get('camera_pitch', 0.0))),
        DeclareLaunchArgument('camera_yaw', default_value=str(sensor_tf.get('camera_yaw', 0.0))),
        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='/',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_params],
            condition=IfCondition(launch_lidar),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            output='screen',
            arguments=[
                '--x', laser_x,
                '--y', laser_y,
                '--z', laser_z,
                '--roll', laser_roll,
                '--pitch', laser_pitch,
                '--yaw', laser_yaw,
                '--frame-id', base_frame,
                '--child-frame-id', laser_frame,
            ],
            condition=IfCondition(publish_laser_tf),
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            parameters=[realsense_params, {
                'enable_gyro': ParameterValue(enable_imu, value_type=bool),
                'enable_accel': ParameterValue(enable_imu, value_type=bool),
                'unite_imu_method': ParameterValue(unite_imu_method, value_type=int),
                'gyro_fps': ParameterValue(gyro_fps, value_type=int),
                'accel_fps': ParameterValue(accel_fps, value_type=int),
            }],
            condition=IfCondition(launch_camera),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            output='screen',
            arguments=[
                '--x', camera_x,
                '--y', camera_y,
                '--z', camera_z,
                '--roll', camera_roll,
                '--pitch', camera_pitch,
                '--yaw', camera_yaw,
                '--frame-id', base_frame,
                '--child-frame-id', camera_frame,
            ],
            condition=IfCondition(publish_camera_tf),
        ),
    ])
