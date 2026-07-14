#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 - <<'PY'
from __future__ import annotations

import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan


class SensorRateProbe(Node):
    def __init__(self) -> None:
        super().__init__('h753_sensor_rate_probe')
        self.arrivals: dict[str, list[float]] = {
            '/scan': [],
            '/wheel/odom': [],
            '/odom': [],
            '/camera/camera/color/image_raw': [],
            '/camera/camera/depth/image_rect_raw': [],
            '/camera/camera/imu': [],
        }
        self.create_subscription(LaserScan, '/scan', self.callback('/scan'), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wheel/odom', self.callback('/wheel/odom'), 20)
        self.create_subscription(Odometry, '/odom', self.callback('/odom'), 20)
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.callback('/camera/camera/color/image_raw'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.callback('/camera/camera/depth/image_rect_raw'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.callback('/camera/camera/imu'),
            qos_profile_sensor_data,
        )

    def callback(self, topic: str):
        def record(_message) -> None:
            self.arrivals[topic].append(time.monotonic())

        return record

    def print_report(self) -> None:
        print('[H753 sensor-rate probe: 8 second low-overhead sample]')
        for topic, arrivals in self.arrivals.items():
            if len(arrivals) < 2:
                publishers = len(self.get_publishers_info_by_topic(topic))
                print(f'{topic}: missing ({publishers} publisher(s), {len(arrivals)} sample(s))')
                continue
            intervals = [
                current - previous
                for previous, current in zip(arrivals, arrivals[1:])
            ]
            rate_hz = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
            print(
                f'{topic}: {rate_hz:.2f} Hz, samples={len(arrivals)}, '
                f'period min/avg/max='
                f'{min(intervals):.3f}/{statistics.mean(intervals):.3f}/{max(intervals):.3f} s'
            )


rclpy.init()
node = SensorRateProbe()
deadline = time.monotonic() + 8.0
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.print_report()
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
