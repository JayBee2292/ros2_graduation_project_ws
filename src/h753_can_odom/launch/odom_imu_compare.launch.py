from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    imu_params = load_params(
        h753_share / 'config' / 'h753_imu_odom_fusion.yaml',
        'h753_imu_odom_fusion',
    )
    compare_params = h753_share / 'config' / 'h753_odom_imu_compare.yaml'
    imu_yaw_axis = str(imu_params.get('imu_yaw_axis', 'z'))
    imu_yaw_axis_index = {'x': 0, 'y': 1, 'z': 2}.get(imu_yaw_axis, 2)

    wheel_odom_topic = LaunchConfiguration('wheel_odom_topic')
    csv_path = LaunchConfiguration('csv_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'wheel_odom_topic',
            default_value='wheel/odom',
            description='Encoder-only odom topic. Use /odom with run_calibration.sh.',
        ),
        DeclareLaunchArgument(
            'csv_path',
            default_value='',
            description='Optional CSV output path.',
        ),
        Node(
            package='h753_can_odom',
            executable='odom_imu_compare_node',
            name='h753_odom_imu_compare',
            output='screen',
            parameters=[
                compare_params,
                {
                    'wheel_odom_topic': wheel_odom_topic,
                    'csv_path': csv_path,
                    'imu_yaw_axis_index': imu_yaw_axis_index,
                    'imu_yaw_sign': float(imu_params.get('imu_yaw_sign', 1.0)),
                    'gyro_bias_rad_s': float(imu_params.get('gyro_bias_rad_s', 0.0)),
                    'gyro_deadband_rad_s': float(imu_params.get('gyro_deadband_rad_s', 0.003)),
                },
            ],
        ),
    ])
