from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    launch_slam = LaunchConfiguration('launch_slam')
    launch_map_odom = LaunchConfiguration('launch_map_odom')
    launch_rviz = LaunchConfiguration('launch_rviz')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')
    lidar_params = LaunchConfiguration('lidar_params')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    gyro_fps = LaunchConfiguration('gyro_fps')
    accel_fps = LaunchConfiguration('accel_fps')
    odom_params = LaunchConfiguration('odom_params')
    imu_odom_params = LaunchConfiguration('imu_odom_params')
    slam_params = LaunchConfiguration('slam_params')
    continue_mapping = LaunchConfiguration('continue_mapping')
    posegraph_file = LaunchConfiguration('posegraph_file')
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
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description='Start the YDLIDAR driver in sensors_bringup.',
        ),
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Start the Intel RealSense camera in sensors_bringup.',
        ),
        DeclareLaunchArgument(
            'realsense_params',
            default_value=str(h753_share / 'config' / 'h753_realsense_imu.yaml'),
            description='RealSense parameter profile forwarded to sensors_bringup.',
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='false',
            description='Enable RealSense gyro/accel streams and /camera/camera/imu.',
        ),
        DeclareLaunchArgument(
            'launch_odom',
            default_value='true',
            description='Start h753_can_odom.',
        ),
        DeclareLaunchArgument(
            'launch_imu_odom',
            default_value='false',
            description='Fuse wheel odom with D435i IMU yaw-rate and publish final /odom TF.',
        ),
        DeclareLaunchArgument(
            'launch_slam',
            default_value='true',
            description='Start slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'launch_map_odom',
            default_value='false',
            description='Publish map-corrected /map_odom from slam_toolbox TF.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Start rviz2 with the mapping display config.',
        ),
        DeclareLaunchArgument(
            'publish_laser_tf',
            default_value='true',
            description='Publish static base_frame -> laser_frame transform in sensors_bringup.',
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
        DeclareLaunchArgument(
            'odom_params',
            default_value=str(h753_share / 'config' / 'h753_can_odom.yaml'),
            description='Path to h753_can_odom parameter file.',
        ),
        DeclareLaunchArgument(
            'imu_odom_params',
            default_value=str(h753_share / 'config' / 'h753_imu_odom_fusion.yaml'),
            description='Path to IMU odom fusion parameter file.',
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value=str(h753_share / 'config' / 'h753_slam_toolbox.yaml'),
            description='Path to slam_toolbox parameter file.',
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
            'rviz_config',
            default_value=str(h753_share / 'rviz' / 'mapping.rviz'),
            description='Path to RViz config file.',
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
                'realsense_params': realsense_params,
                'publish_laser_tf': 'false',
                'launch_odom': launch_odom,
                'launch_imu_odom': launch_imu_odom,
                'launch_slam': launch_slam,
                'launch_map_odom': launch_map_odom,
                'launch_rviz': launch_rviz,
                'lidar_params': lidar_params,
                'unite_imu_method': unite_imu_method,
                'gyro_fps': gyro_fps,
                'accel_fps': accel_fps,
                'odom_params': odom_params,
                'imu_odom_params': imu_odom_params,
                'slam_params': slam_params,
                'continue_mapping': continue_mapping,
                'posegraph_file': posegraph_file,
                'rviz_config': rviz_config,
                **sensor_pose_args,
            }.items(),
        ),
    ])
