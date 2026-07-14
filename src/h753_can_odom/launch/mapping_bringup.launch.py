from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    realsense_params = load_params(h753_share / 'config' / 'h753_realsense.yaml', 'realsense_camera')
    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_odom = LaunchConfiguration('launch_odom')
    launch_imu_odom = LaunchConfiguration('launch_imu_odom')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_slam = LaunchConfiguration('launch_slam')
    launch_map_odom = LaunchConfiguration('launch_map_odom')
    launch_camera = LaunchConfiguration('launch_camera')
    enable_imu = LaunchConfiguration('enable_imu')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')
    lidar_params = LaunchConfiguration('lidar_params')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    gyro_fps = LaunchConfiguration('gyro_fps')
    accel_fps = LaunchConfiguration('accel_fps')
    odom_params = LaunchConfiguration('odom_params')
    imu_odom_params = LaunchConfiguration('imu_odom_params')
    rviz_config = LaunchConfiguration('rviz_config')
    slam_params = LaunchConfiguration('slam_params')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    normal_odom_condition = IfCondition(PythonExpression([
        "'", launch_odom, "' == 'true' and '", launch_imu_odom, "' != 'true'",
    ]))
    imu_odom_condition = IfCondition(PythonExpression([
        "'", launch_odom, "' == 'true' and '", launch_imu_odom, "' == 'true'",
    ]))
    map_odom_condition = IfCondition(PythonExpression([
        "'", launch_slam, "' == 'true' and '", launch_map_odom, "' == 'true'",
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='false',
            description='Start ydlidar_ros2_driver. Keep false when sensors_bringup is already running.',
        ),
        DeclareLaunchArgument(
            'launch_odom',
            default_value='true',
            description='Start h753_can_odom. Set false if it is already running.',
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
            'launch_camera',
            default_value='false',
            description='Start Intel RealSense camera. Keep false when sensors_bringup is already running.',
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='false',
            description='Enable RealSense gyro/accel streams and /camera/camera/imu.',
        ),
        DeclareLaunchArgument(
            'publish_laser_tf',
            default_value='false',
            description='Publish static base_frame -> laser_frame transform. Keep false when sensors_bringup is already running.',
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
        DeclareLaunchArgument('accel_fps', default_value='63'),
        DeclareLaunchArgument(
            'odom_params',
            default_value=str(h753_share / 'config' / 'h753_can_odom.yaml'),
            description='Path to h753_can_odom parameter file.',
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value=str(h753_share / 'config' / 'h753_slam_toolbox.yaml'),
            description='Path to slam_toolbox parameter file.',
        ),
        DeclareLaunchArgument(
            'imu_odom_params',
            default_value=str(h753_share / 'config' / 'h753_imu_odom_fusion.yaml'),
            description='Path to IMU odom fusion parameter file.',
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
            package='h753_can_odom',
            executable='can_odom_node',
            name='h753_can_odom',
            output='screen',
            parameters=[odom_params],
            condition=normal_odom_condition,
        ),
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
        Node(
            package='h753_can_odom',
            executable='imu_odom_fusion_node',
            name='h753_imu_odom_fusion',
            output='screen',
            parameters=[imu_odom_params],
            condition=imu_odom_condition,
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
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
            condition=IfCondition(launch_slam),
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
            condition=map_odom_condition,
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
