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
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    ydlidar_share = Path(get_package_share_directory('ydlidar_ros2_driver'))
    sensor_tf = load_params(
        h753_share / 'config' / 'h753_sensor_tf.yaml',
        'sensor_tf',
    )

    map_yaml = LaunchConfiguration('map')
    amcl_params = LaunchConfiguration('amcl_params')
    odom_params = LaunchConfiguration('odom_params')
    lidar_params = LaunchConfiguration('lidar_params')
    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_odom = LaunchConfiguration('launch_odom')
    launch_rviz = LaunchConfiguration('launch_rviz')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')
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
            'map',
            default_value=(
                '/home/jyl1015/ros2_graduation_project_ws/'
                'maps/go2/go2_map.yaml'
            ),
            description='Imported Go2 occupancy-map YAML file.',
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
            default_value=str(ydlidar_share / 'params' / 'Tmini.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            # Includes map, scan, AMCL particles and 2D Pose Estimate.
            default_value=str(nav2_share / 'rviz' / 'nav2_default_view.rviz'),
        ),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_odom', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('publish_laser_tf', default_value='true'),
        DeclareLaunchArgument(
            'base_frame',
            default_value=str(sensor_tf.get('base_frame', 'base_link')),
        ),
        DeclareLaunchArgument(
            'laser_frame',
            default_value=str(sensor_tf.get('laser_frame', 'laser_frame')),
        ),
        DeclareLaunchArgument(
            'laser_x',
            default_value=str(sensor_tf.get('laser_x', 0.0)),
        ),
        DeclareLaunchArgument(
            'laser_y',
            default_value=str(sensor_tf.get('laser_y', 0.0)),
        ),
        DeclareLaunchArgument(
            'laser_z',
            default_value=str(sensor_tf.get('laser_z', 0.02)),
        ),
        DeclareLaunchArgument(
            'laser_roll',
            default_value=str(sensor_tf.get('laser_roll', 0.0)),
        ),
        DeclareLaunchArgument(
            'laser_pitch',
            default_value=str(sensor_tf.get('laser_pitch', 0.0)),
        ),
        DeclareLaunchArgument(
            'laser_yaw',
            default_value=str(sensor_tf.get('laser_yaw', 0.0)),
        ),

        # Evaluate the public launch_rviz option before mapping_bringup's
        # private launch_rviz:=false argument is introduced below.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
        # Sensors only: no camera or motor-command node.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(h753_share / 'launch' / 'sensors_bringup.launch.py')
            ),
            launch_arguments={
                'launch_lidar': launch_lidar,
                'launch_camera': 'false',
                'enable_imu': 'false',
                'publish_laser_tf': publish_laser_tf,
                'publish_camera_tf': 'false',
                'lidar_params': lidar_params,
                **sensor_pose_args,
            }.items(),
        ),
        # Reuse verified CAN odometry with SLAM and RViz disabled here.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(h753_share / 'launch' / 'mapping_bringup.launch.py')
            ),
            launch_arguments={
                'launch_lidar': 'false',
                'launch_camera': 'false',
                'launch_odom': launch_odom,
                'launch_imu_odom': 'false',
                'launch_slam': 'false',
                'launch_map_odom': 'false',
                'launch_rviz': 'false',
                'publish_laser_tf': 'false',
                'odom_params': odom_params,
                **sensor_pose_args,
            }.items(),
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
                        map_yaml,
                        value_type=str,
                    ),
                },
            ],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params],
        ),
        TimerAction(
            # This launch is included inside a scoped group. Keep the delay
            # literal so it remains valid after that scoped context exits.
            period=1.5,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': ['map_server', 'amcl'],
                    }],
                ),
            ],
        ),
    ])
