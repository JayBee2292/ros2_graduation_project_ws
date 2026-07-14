#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class OdomImuCompareNode(Node):
    def __init__(self) -> None:
        super().__init__('h753_odom_imu_compare')

        self.declare_parameter('wheel_odom_topic', 'wheel/odom')
        self.declare_parameter('imu_topic', '/camera/camera/imu')
        self.declare_parameter('imu_yaw_axis', 'y')
        self.declare_parameter('imu_yaw_axis_index', -1)
        self.declare_parameter('imu_yaw_sign', -1.0)
        self.declare_parameter('gyro_bias_rad_s', 0.0)
        self.declare_parameter('gyro_deadband_rad_s', 0.003)
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('max_imu_dt_s', 0.1)
        self.declare_parameter('report_period_s', 1.0)
        self.declare_parameter('csv_path', '')

        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.imu_yaw_axis = str(self.get_parameter('imu_yaw_axis').value)
        imu_yaw_axis_index = int(self.get_parameter('imu_yaw_axis_index').value)
        if 0 <= imu_yaw_axis_index <= 2:
            self.imu_yaw_axis = ('x', 'y', 'z')[imu_yaw_axis_index]
        self.imu_yaw_sign = float(self.get_parameter('imu_yaw_sign').value)
        self.gyro_bias_rad_s = float(self.get_parameter('gyro_bias_rad_s').value)
        self.gyro_deadband_rad_s = float(self.get_parameter('gyro_deadband_rad_s').value)
        self.imu_timeout_s = float(self.get_parameter('imu_timeout_s').value)
        self.max_imu_dt_s = float(self.get_parameter('max_imu_dt_s').value)

        if self.imu_yaw_axis not in ('x', 'y', 'z'):
            raise ValueError('imu_yaw_axis must be x, y, or z')
        if self.imu_yaw_sign not in (-1.0, 1.0):
            raise ValueError('imu_yaw_sign must be exactly -1.0 or 1.0')

        self.wheel_distance_m = 0.0
        self.wheel_yaw_rad = 0.0
        self.wheel_linear_mps = 0.0
        self.wheel_angular_radps = 0.0
        self.wheel_last_stamp_s: float | None = None
        self.wheel_last_yaw_rad: float | None = None
        self.wheel_received_at_s: float | None = None

        self.imu_yaw_rad = 0.0
        self.imu_yaw_rate_radps = 0.0
        self.imu_last_stamp_s: float | None = None
        self.imu_received_at_s: float | None = None

        self.csv_file = None
        self.csv_writer = None
        csv_path = str(self.get_parameter('csv_path').value).strip()
        if csv_path:
            path = Path(csv_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = path.open('w', newline='', encoding='utf-8', buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'ros_time_s',
                'wheel_distance_m',
                'wheel_yaw_deg',
                'wheel_linear_mps',
                'wheel_angular_radps',
                'imu_yaw_deg',
                'imu_yaw_rate_radps',
                'yaw_error_deg',
                'imu_to_wheel_yaw_ratio',
                'imu_status',
            ])

        self.create_subscription(Odometry, self.wheel_odom_topic, self._wheel_callback, 20)
        self.create_subscription(Imu, self.imu_topic, self._imu_callback, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset', self._reset_callback)
        self.create_timer(float(self.get_parameter('report_period_s').value), self._report)

        self.get_logger().info(
            'Odom/IMU comparison ready: wheel=%s imu=%s yaw_axis=%s yaw_sign=%.1f'
            % (
                self.wheel_odom_topic,
                self.imu_topic,
                self.imu_yaw_axis,
                self.imu_yaw_sign,
            )
        )
        if csv_path:
            self.get_logger().info(f'CSV log: {csv_path}')

    def destroy_node(self) -> bool:
        if self.csv_file is not None:
            self.csv_file.close()
        return super().destroy_node()

    def _wheel_callback(self, msg: Odometry) -> None:
        stamp_s = stamp_to_seconds(msg.header.stamp)
        yaw_rad = yaw_from_odom(msg)
        self.wheel_received_at_s = time.monotonic()
        self.wheel_linear_mps = msg.twist.twist.linear.x
        self.wheel_angular_radps = msg.twist.twist.angular.z

        if self.wheel_last_stamp_s is not None and self.wheel_last_yaw_rad is not None:
            dt = stamp_s - self.wheel_last_stamp_s
            if 0.0 < dt <= 0.5:
                self.wheel_distance_m += self.wheel_linear_mps * dt
                self.wheel_yaw_rad += normalize_angle(yaw_rad - self.wheel_last_yaw_rad)

        self.wheel_last_stamp_s = stamp_s
        self.wheel_last_yaw_rad = yaw_rad

    def _imu_callback(self, msg: Imu) -> None:
        stamp_s = stamp_to_seconds(msg.header.stamp)
        angular_rate = self._select_imu_axis(msg)
        angular_rate = self.imu_yaw_sign * (angular_rate - self.gyro_bias_rad_s)
        if abs(angular_rate) < self.gyro_deadband_rad_s:
            angular_rate = 0.0

        if self.imu_last_stamp_s is not None:
            dt = stamp_s - self.imu_last_stamp_s
            if 0.0 < dt <= self.max_imu_dt_s:
                self.imu_yaw_rad += angular_rate * dt

        self.imu_last_stamp_s = stamp_s
        self.imu_received_at_s = time.monotonic()
        self.imu_yaw_rate_radps = angular_rate

    def _select_imu_axis(self, msg: Imu) -> float:
        if self.imu_yaw_axis == 'x':
            return msg.angular_velocity.x
        if self.imu_yaw_axis == 'y':
            return msg.angular_velocity.y
        return msg.angular_velocity.z

    def _reset_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.wheel_distance_m = 0.0
        self.wheel_yaw_rad = 0.0
        self.wheel_linear_mps = 0.0
        self.wheel_angular_radps = 0.0
        self.wheel_last_stamp_s = None
        self.wheel_last_yaw_rad = None
        self.imu_yaw_rad = 0.0
        self.imu_yaw_rate_radps = 0.0
        self.imu_last_stamp_s = None
        response.success = True
        response.message = 'Encoder and IMU integrators reset'
        self.get_logger().info(response.message)
        return response

    def _report(self) -> None:
        now_monotonic_s = time.monotonic()
        wheel_status = self._source_status(self.wheel_received_at_s, now_monotonic_s, 1.0)
        imu_status = self._source_status(self.imu_received_at_s, now_monotonic_s, self.imu_timeout_s)
        yaw_error_rad = self.imu_yaw_rad - self.wheel_yaw_rad
        ratio = (
            self.imu_yaw_rad / self.wheel_yaw_rad
            if abs(self.wheel_yaw_rad) >= math.radians(5.0)
            else math.nan
        )

        self.get_logger().info(
            'wheel[%s]: distance=%+.3fm yaw=%+.1fdeg v=%+.3fm/s w=%+.3frad/s | '
            'imu[%s]: yaw=%+.1fdeg w=%+.3frad/s | error=%+.1fdeg ratio=%s'
            % (
                wheel_status,
                self.wheel_distance_m,
                math.degrees(self.wheel_yaw_rad),
                self.wheel_linear_mps,
                self.wheel_angular_radps,
                imu_status,
                math.degrees(self.imu_yaw_rad),
                self.imu_yaw_rate_radps,
                math.degrees(yaw_error_rad),
                'n/a' if math.isnan(ratio) else f'{ratio:.3f}',
            )
        )

        if self.csv_writer is not None:
            self.csv_writer.writerow([
                f'{self.get_clock().now().nanoseconds / 1e9:.9f}',
                f'{self.wheel_distance_m:.9f}',
                f'{math.degrees(self.wheel_yaw_rad):.9f}',
                f'{self.wheel_linear_mps:.9f}',
                f'{self.wheel_angular_radps:.9f}',
                f'{math.degrees(self.imu_yaw_rad):.9f}',
                f'{self.imu_yaw_rate_radps:.9f}',
                f'{math.degrees(yaw_error_rad):.9f}',
                '' if math.isnan(ratio) else f'{ratio:.9f}',
                imu_status,
            ])

    @staticmethod
    def _source_status(received_at_s: float | None, now_s: float, timeout_s: float) -> str:
        if received_at_s is None:
            return 'missing'
        return 'fresh' if now_s - received_at_s <= timeout_s else 'stale'


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomImuCompareNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
