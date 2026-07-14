#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ImuOdomFusionNode(Node):
    def __init__(self) -> None:
        super().__init__('h753_imu_odom_fusion')

        self.declare_parameter('wheel_odom_topic', 'wheel/odom')
        self.declare_parameter('imu_topic', '/camera/camera/imu')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('twist_topic', 'odom_vel')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_imu_yaw_rate', True)
        self.declare_parameter('imu_yaw_axis', 'z')
        self.declare_parameter('imu_yaw_sign', 1.0)
        self.declare_parameter('gyro_bias_rad_s', 0.0)
        self.declare_parameter('gyro_deadband_rad_s', 0.003)
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('max_imu_dt_s', 0.1)

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.use_imu_yaw_rate = bool(self.get_parameter('use_imu_yaw_rate').value)
        self.imu_yaw_axis = self.get_parameter('imu_yaw_axis').value
        self.imu_yaw_sign = float(self.get_parameter('imu_yaw_sign').value)
        self.gyro_bias_rad_s = float(self.get_parameter('gyro_bias_rad_s').value)
        self.gyro_deadband_rad_s = float(self.get_parameter('gyro_deadband_rad_s').value)
        self.imu_timeout_s = float(self.get_parameter('imu_timeout_s').value)
        self.max_imu_dt_s = float(self.get_parameter('max_imu_dt_s').value)

        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            20,
        )
        self.twist_pub = self.create_publisher(
            Twist,
            self.get_parameter('twist_topic').value,
            20,
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(
            Odometry,
            self.get_parameter('wheel_odom_topic').value,
            self._wheel_odom_callback,
            20,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self._imu_callback,
            qos_profile_sensor_data,
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_wheel_odom: Odometry | None = None
        self.last_imu_time_s: float | None = None
        self.last_imu_receive_time_s: float | None = None
        self.last_imu_angular_z = 0.0
        self.warned_no_imu = False

        self.get_logger().info(
            'IMU odom fusion active: wheel_odom=%s imu=%s yaw_axis=%s yaw_sign=%.1f'
            % (
                self.get_parameter('wheel_odom_topic').value,
                self.get_parameter('imu_topic').value,
                self.imu_yaw_axis,
                self.imu_yaw_sign,
            )
        )

    def _imu_callback(self, msg: Imu) -> None:
        if not self.use_imu_yaw_rate:
            return

        now_s = self._stamp_to_seconds(msg.header.stamp)
        if now_s <= 0.0:
            now_s = self.get_clock().now().nanoseconds / 1e9

        angular_z = self._select_imu_axis(msg)
        angular_z = self.imu_yaw_sign * (angular_z - self.gyro_bias_rad_s)
        if abs(angular_z) < self.gyro_deadband_rad_s:
            angular_z = 0.0

        if self.last_imu_time_s is not None:
            dt = now_s - self.last_imu_time_s
            if 0.0 < dt <= self.max_imu_dt_s:
                self.yaw = normalize_angle(self.yaw + angular_z * dt)

        self.last_imu_time_s = now_s
        self.last_imu_receive_time_s = self.get_clock().now().nanoseconds / 1e9
        self.last_imu_angular_z = angular_z
        self.warned_no_imu = False

    def _wheel_odom_callback(self, msg: Odometry) -> None:
        wheel_yaw = yaw_from_quaternion(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        if self.last_wheel_odom is None:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.yaw = wheel_yaw
            self.last_wheel_odom = msg
            self._publish(msg, msg.twist.twist.linear.x, msg.twist.twist.angular.z, imu_fresh=False)
            return

        dt = self._stamp_to_seconds(msg.header.stamp) - self._stamp_to_seconds(
            self.last_wheel_odom.header.stamp
        )
        if dt <= 0.0 or dt > 0.5:
            self.get_logger().warn(f'Wheel odom dt out of range ({dt:.3f}s); resetting fusion baseline')
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.yaw = wheel_yaw
            self.last_wheel_odom = msg
            self._publish(msg, msg.twist.twist.linear.x, msg.twist.twist.angular.z, imu_fresh=False)
            return

        imu_fresh = self._imu_is_fresh()
        if not imu_fresh:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.yaw = wheel_yaw
            self.last_imu_angular_z = msg.twist.twist.angular.z
            if not self.warned_no_imu:
                self.get_logger().warn('No fresh IMU data; publishing wheel odom fallback')
                self.warned_no_imu = True
        else:
            linear_v = msg.twist.twist.linear.x
            delta_s = linear_v * dt
            yaw_mid = self.yaw - (self.last_imu_angular_z * dt / 2.0)
            self.x += delta_s * math.cos(yaw_mid)
            self.y += delta_s * math.sin(yaw_mid)

        linear_v = msg.twist.twist.linear.x
        self.last_wheel_odom = msg
        self._publish(msg, linear_v, self.last_imu_angular_z, imu_fresh=imu_fresh)

    def _select_imu_axis(self, msg: Imu) -> float:
        if self.imu_yaw_axis == 'x':
            return msg.angular_velocity.x
        if self.imu_yaw_axis == 'y':
            return msg.angular_velocity.y
        return msg.angular_velocity.z

    def _imu_is_fresh(self) -> bool:
        if self.last_imu_receive_time_s is None:
            return False
        now_s = self.get_clock().now().nanoseconds / 1e9
        return now_s - self.last_imu_receive_time_s <= self.imu_timeout_s

    def _publish(self, wheel_msg: Odometry, linear_v: float, angular_z: float, imu_fresh: bool) -> None:
        odom = Odometry()
        odom.header.stamp = wheel_msg.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = linear_v
        odom.twist.twist.angular.z = angular_z
        odom.pose.covariance = list(wheel_msg.pose.covariance)
        odom.twist.covariance = list(wheel_msg.twist.covariance)
        odom.pose.covariance[35] = 0.02 if imu_fresh else wheel_msg.pose.covariance[35]
        odom.twist.covariance[35] = 0.03 if imu_fresh else wheel_msg.twist.covariance[35]
        self.odom_pub.publish(odom)

        twist = Twist()
        twist.linear.x = linear_v
        twist.angular.z = angular_z
        self.twist_pub.publish(twist)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = odom.header.stamp
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = odom.pose.pose.orientation.z
            transform.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuOdomFusionNode()
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
