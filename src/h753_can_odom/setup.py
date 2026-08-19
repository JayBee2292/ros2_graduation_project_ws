from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'h753_can_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (
            os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml'),
        ),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jyl1015',
    maintainer_email='jyl1015@example.com',
    description='CAN FD telemetry to ROS 2 odometry bridge for the STM32H753 robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_odom_node = h753_can_odom.can_odom_node:main',
            'imu_odom_fusion_node = h753_can_odom.imu_odom_fusion_node:main',
            'odom_imu_compare_node = h753_can_odom.odom_imu_compare_node:main',
            'imu_vibration_analysis = h753_can_odom.imu_vibration_analysis:main',
            'interactive_calibration = h753_can_odom.interactive_calibration:main',
            'map_odom_publisher_node = h753_can_odom.map_odom_publisher_node:main',
            'cmd_vel_uart_bridge_node = h753_can_odom.cmd_vel_uart_bridge_node:main',
            'frontier_explorer_node = h753_can_odom.frontier_explorer_node:main',
            'go2_manual_drive_node = h753_can_odom.go2_manual_drive_node:main',
            'robot_mode_manager_node = h753_can_odom.robot_mode_manager_node:main',
            'vlm_gateway_node = h753_can_odom.vlm_gateway_node:main',
        ],
    },
)
