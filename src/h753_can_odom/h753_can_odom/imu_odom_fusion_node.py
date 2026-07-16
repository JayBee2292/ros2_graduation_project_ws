#!/usr/bin/env python3
from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from h753_can_odom.imu_filtering import (
    SecondOrderButterworthLowPass,
    adaptive_imu_weight,
    fuse_yaw_rates,
    hampel_filter_sample,
    population_variance,
)
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def mean_gyro_bias(samples: list[float]) -> float:
    if not samples:
        raise ValueError('at least one gyro sample is required')
    return sum(samples) / len(samples)


def integrate_planar_pose(
    x: float,
    y: float,
    yaw: float,
    delta_distance_m: float,
    delta_yaw_rad: float,
) -> tuple[float, float, float]:
    """Apply a local wheel-motion increment without changing pose origin."""
    # Pseudocode:
    #   heading_mid = current_heading + wheel_heading_change / 2
    #   position += wheel_distance * unit_vector(heading_mid)
    #   heading = normalize(current_heading + wheel_heading_change)
    yaw_mid = normalize_angle(yaw + delta_yaw_rad / 2.0)
    return (
        x + delta_distance_m * math.cos(yaw_mid),
        y + delta_distance_m * math.sin(yaw_mid),
        normalize_angle(yaw + delta_yaw_rad),
    )


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
        self.declare_parameter('auto_gyro_bias_samples', 200)
        self.declare_parameter('gyro_deadband_rad_s', 0.01)
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('max_imu_dt_s', 0.1)
        self.declare_parameter('enable_vibration_filter', False)
        self.declare_parameter('enable_adaptive_yaw_fusion', False)
        self.declare_parameter('enable_online_bias_update', False)
        self.declare_parameter('imu_sample_rate_hz', 200.0)
        self.declare_parameter('hampel_window_samples', 5)
        self.declare_parameter('hampel_threshold_sigma', 3.0)
        self.declare_parameter('hampel_min_threshold_rad_s', 0.10)
        self.declare_parameter('gyro_lpf_cutoff_hz', 10.0)
        self.declare_parameter('vibration_window_s', 0.50)
        self.declare_parameter('vibration_variance_low', 0.0001)
        self.declare_parameter('vibration_variance_high', 0.0025)
        self.declare_parameter('imu_weight_min', 0.35)
        self.declare_parameter('imu_weight_max', 0.90)
        self.declare_parameter('innovation_gate_rad_s', 0.35)
        self.declare_parameter('wheel_fresh_timeout_s', 0.20)
        self.declare_parameter('online_bias_alpha', 0.001)
        self.declare_parameter('bias_hold_time_s', 1.0)
        self.declare_parameter('stationary_linear_threshold_mps', 0.01)
        self.declare_parameter('stationary_angular_threshold_rad_s', 0.02)
        self.declare_parameter('raw_yaw_rate_topic', '/imu_filter/yaw_rate_raw')
        self.declare_parameter(
            'filtered_yaw_rate_topic',
            '/imu_filter/yaw_rate_filtered',
        )
        self.declare_parameter(
            'vibration_variance_topic',
            '/imu_filter/vibration_variance',
        )
        self.declare_parameter('imu_weight_topic', '/imu_filter/imu_weight')

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.use_imu_yaw_rate = bool(self.get_parameter('use_imu_yaw_rate').value)
        self.imu_yaw_axis = str(self.get_parameter('imu_yaw_axis').value).lower()
        if self.imu_yaw_axis not in {'x', 'y', 'z'}:
            raise ValueError('imu_yaw_axis must be one of: x, y, z')
        self.imu_yaw_sign = float(self.get_parameter('imu_yaw_sign').value)
        self.gyro_bias_rad_s = float(self.get_parameter('gyro_bias_rad_s').value)
        self.auto_gyro_bias_samples = max(
            0,
            int(self.get_parameter('auto_gyro_bias_samples').value),
        )
        self.gyro_deadband_rad_s = float(self.get_parameter('gyro_deadband_rad_s').value)
        self.imu_timeout_s = float(self.get_parameter('imu_timeout_s').value)
        self.max_imu_dt_s = float(self.get_parameter('max_imu_dt_s').value)
        self.enable_vibration_filter = bool(
            self.get_parameter('enable_vibration_filter').value
        )
        self.enable_adaptive_yaw_fusion = bool(
            self.get_parameter('enable_adaptive_yaw_fusion').value
        )
        self.enable_online_bias_update = bool(
            self.get_parameter('enable_online_bias_update').value
        )
        self.imu_sample_rate_hz = float(
            self.get_parameter('imu_sample_rate_hz').value
        )
        self.hampel_window_samples = int(
            self.get_parameter('hampel_window_samples').value
        )
        if self.hampel_window_samples < 3 or self.hampel_window_samples % 2 == 0:
            raise ValueError('hampel_window_samples must be an odd integer >= 3')
        self.hampel_threshold_sigma = float(
            self.get_parameter('hampel_threshold_sigma').value
        )
        self.hampel_min_threshold_rad_s = float(
            self.get_parameter('hampel_min_threshold_rad_s').value
        )
        self.gyro_lpf_cutoff_hz = float(
            self.get_parameter('gyro_lpf_cutoff_hz').value
        )
        self.vibration_window_s = float(
            self.get_parameter('vibration_window_s').value
        )
        self.vibration_variance_low = float(
            self.get_parameter('vibration_variance_low').value
        )
        self.vibration_variance_high = float(
            self.get_parameter('vibration_variance_high').value
        )
        self.imu_weight_min = float(self.get_parameter('imu_weight_min').value)
        self.imu_weight_max = float(self.get_parameter('imu_weight_max').value)
        self.innovation_gate_rad_s = float(
            self.get_parameter('innovation_gate_rad_s').value
        )
        self.wheel_fresh_timeout_s = float(
            self.get_parameter('wheel_fresh_timeout_s').value
        )
        self.online_bias_alpha = float(
            self.get_parameter('online_bias_alpha').value
        )
        self.bias_hold_time_s = float(
            self.get_parameter('bias_hold_time_s').value
        )
        self.stationary_linear_threshold_mps = float(
            self.get_parameter('stationary_linear_threshold_mps').value
        )
        self.stationary_angular_threshold_rad_s = float(
            self.get_parameter('stationary_angular_threshold_rad_s').value
        )
        if self.vibration_window_s <= 0.0:
            raise ValueError('vibration_window_s must be positive')
        if self.wheel_fresh_timeout_s <= 0.0:
            raise ValueError('wheel_fresh_timeout_s must be positive')
        if not 0.0 < self.online_bias_alpha <= 1.0:
            raise ValueError('online_bias_alpha must be in (0, 1]')
        if self.bias_hold_time_s < 0.0:
            raise ValueError('bias_hold_time_s must be non-negative')
        if (
            self.stationary_linear_threshold_mps < 0.0
            or self.stationary_angular_threshold_rad_s < 0.0
        ):
            raise ValueError('stationary thresholds must be non-negative')

        hampel_filter_sample(
            [0.0],
            0.0,
            self.hampel_threshold_sigma,
            self.hampel_min_threshold_rad_s,
        )

        self.gyro_low_pass = SecondOrderButterworthLowPass(
            self.imu_sample_rate_hz,
            self.gyro_lpf_cutoff_hz,
        )
        # Validate adaptive bounds at startup even while the feature is disabled.
        adaptive_imu_weight(
            self.vibration_variance_low,
            self.vibration_variance_low,
            self.vibration_variance_high,
            self.imu_weight_min,
            self.imu_weight_max,
            0.0,
            self.innovation_gate_rad_s,
            False,
        )

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
        self.raw_yaw_rate_pub = self.create_publisher(
            Float64,
            str(self.get_parameter('raw_yaw_rate_topic').value),
            qos_profile_sensor_data,
        )
        self.filtered_yaw_rate_pub = self.create_publisher(
            Float64,
            str(self.get_parameter('filtered_yaw_rate_topic').value),
            qos_profile_sensor_data,
        )
        self.vibration_variance_pub = self.create_publisher(
            Float64,
            str(self.get_parameter('vibration_variance_topic').value),
            qos_profile_sensor_data,
        )
        self.imu_weight_pub = self.create_publisher(
            Float64,
            str(self.get_parameter('imu_weight_topic').value),
            qos_profile_sensor_data,
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
        self.last_wheel_receive_time_s: float | None = None
        self.last_wheel_linear_mps = 0.0
        self.last_wheel_angular_rad_s = 0.0
        self.hampel_samples: deque[float] = deque(
            maxlen=self.hampel_window_samples
        )
        vibration_window_samples = max(
            2,
            int(round(self.vibration_window_s * self.imu_sample_rate_hz)),
        )
        self.vibration_residuals: deque[float] = deque(
            maxlen=vibration_window_samples
        )
        self.vibration_variance = 0.0
        self.current_imu_weight = 1.0
        self.hampel_rejected_samples = 0
        self.stationary_since_s: float | None = None
        self.warned_invalid_imu = False
        self.gyro_bias_samples: list[float] = []
        self.gyro_bias_ready = self.auto_gyro_bias_samples == 0
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
        if not self.gyro_bias_ready:
            self.get_logger().info(
                'Keep robot still while collecting %d startup gyro-bias samples'
                % self.auto_gyro_bias_samples
            )
        self.get_logger().info(
            'IMU vibration pipeline: filter=%s adaptive_fusion=%s online_bias=%s '
            'LPF=%.1fHz Hampel=%d samples diagnostics=/imu_filter/*'
            % (
                self.enable_vibration_filter,
                self.enable_adaptive_yaw_fusion,
                self.enable_online_bias_update,
                self.gyro_lpf_cutoff_hz,
                self.hampel_window_samples,
            )
        )

    def _imu_callback(self, msg: Imu) -> None:
        # Pseudocode:
        #   if startup bias is not ready:
        #       collect stationary gyro samples
        #       bias = mean(samples); keep wheel-odom fallback active
        #   else:
        #       reject impulse -> low-pass -> estimate vibration variance
        #       optionally blend filtered IMU rate with fresh encoder yaw rate
        #       optionally update bias only after verified stationary hold
        #       yaw += selected_yaw_rate * imu_dt
        if not self.use_imu_yaw_rate:
            return

        now_s = self._stamp_to_seconds(msg.header.stamp)
        if now_s <= 0.0:
            now_s = self.get_clock().now().nanoseconds / 1e9
        receive_now_s = self.get_clock().now().nanoseconds / 1e9

        raw_angular_z = self._select_imu_axis(msg)
        if not math.isfinite(raw_angular_z):
            if not self.warned_invalid_imu:
                self.get_logger().warn(
                    'Ignoring non-finite IMU yaw-rate sample until valid data returns'
                )
                self.warned_invalid_imu = True
            return
        self.warned_invalid_imu = False

        if not self.gyro_bias_ready:
            self.gyro_bias_samples.append(raw_angular_z)
            self.last_imu_time_s = now_s
            if len(self.gyro_bias_samples) >= self.auto_gyro_bias_samples:
                self.gyro_bias_rad_s = mean_gyro_bias(self.gyro_bias_samples)
                self.gyro_bias_samples.clear()
                self.gyro_bias_ready = True
                self.last_imu_receive_time_s = receive_now_s
                self._reset_filter_history()
                self.get_logger().info(
                    'Startup gyro bias ready: %.8f rad/s'
                    % self.gyro_bias_rad_s
                )
            return

        dt = None if self.last_imu_time_s is None else now_s - self.last_imu_time_s
        if dt is not None and (dt <= 0.0 or dt > self.max_imu_dt_s):
            # A timestamp reset or USB gap invalidates causal filter history.
            # Reset it so stale pre-gap values cannot create a recovery transient.
            self._reset_filter_history()

        corrected_rate = self.imu_yaw_sign * (
            raw_angular_z - self.gyro_bias_rad_s
        )
        self.hampel_samples.append(corrected_rate)
        robust_rate = corrected_rate
        if len(self.hampel_samples) == self.hampel_samples.maxlen:
            robust_rate, rejected = hampel_filter_sample(
                tuple(self.hampel_samples),
                corrected_rate,
                self.hampel_threshold_sigma,
                self.hampel_min_threshold_rad_s,
            )
            if rejected:
                self.hampel_rejected_samples += 1

        filtered_rate = self.gyro_low_pass.update(robust_rate)
        self.vibration_residuals.append(robust_rate - filtered_rate)
        self.vibration_variance = population_variance(
            tuple(self.vibration_residuals)
        )

        imu_rate_for_pose = (
            filtered_rate if self.enable_vibration_filter else corrected_rate
        )
        if abs(imu_rate_for_pose) < self.gyro_deadband_rad_s:
            imu_rate_for_pose = 0.0

        self.current_imu_weight = 1.0
        angular_z = imu_rate_for_pose
        if self.enable_adaptive_yaw_fusion and self._wheel_is_fresh():
            innovation = imu_rate_for_pose - self.last_wheel_angular_rad_s
            robot_is_turning = (
                abs(imu_rate_for_pose) > self.stationary_angular_threshold_rad_s
                or abs(self.last_wheel_angular_rad_s)
                > self.stationary_angular_threshold_rad_s
            )
            self.current_imu_weight = adaptive_imu_weight(
                self.vibration_variance,
                self.vibration_variance_low,
                self.vibration_variance_high,
                self.imu_weight_min,
                self.imu_weight_max,
                innovation,
                self.innovation_gate_rad_s,
                robot_is_turning,
            )
            angular_z = fuse_yaw_rates(
                imu_rate_for_pose,
                self.last_wheel_angular_rad_s,
                self.current_imu_weight,
            )
            if abs(angular_z) < self.gyro_deadband_rad_s:
                angular_z = 0.0

        self._update_online_bias(
            raw_angular_z,
            receive_now_s,
        )
        self._publish_filter_diagnostics(
            corrected_rate,
            filtered_rate,
            self.vibration_variance,
            self.current_imu_weight,
        )

        if dt is not None and 0.0 < dt <= self.max_imu_dt_s:
            self.yaw = normalize_angle(self.yaw + angular_z * dt)

        self.last_imu_time_s = now_s
        self.last_imu_receive_time_s = receive_now_s
        self.last_imu_angular_z = angular_z
        self.warned_no_imu = False

    def _wheel_odom_callback(self, msg: Odometry) -> None:
        # Pseudocode:
        #   if first wheel sample: initialize the common pose origin
        #   elif wheel timestamp is invalid: keep pose, reset only baseline
        #   elif IMU is fresh: integrate distance along IMU heading
        #   else: integrate relative wheel distance/yaw from current fused pose
        # The fallback deliberately never copies absolute wheel x/y/yaw after
        # startup; doing so caused a multi-meter map jump on D435i disconnect.
        wheel_yaw = yaw_from_quaternion(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.last_wheel_receive_time_s = self.get_clock().now().nanoseconds / 1e9
        self.last_wheel_linear_mps = msg.twist.twist.linear.x
        self.last_wheel_angular_rad_s = msg.twist.twist.angular.z

        if self.last_wheel_odom is None:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.yaw = wheel_yaw
            self.last_wheel_odom = msg
            self._publish(
                msg,
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z,
                imu_fresh=False,
            )
            return

        dt = self._stamp_to_seconds(msg.header.stamp) - self._stamp_to_seconds(
            self.last_wheel_odom.header.stamp
        )
        if dt <= 0.0 or dt > 0.5:
            self.get_logger().warn(
                f'Wheel odom dt out of range ({dt:.3f}s); '
                'keeping fused pose and resetting only the wheel baseline'
            )
            self.last_wheel_odom = msg
            self._publish(msg, 0.0, 0.0, imu_fresh=False)
            return

        imu_fresh = self._imu_is_fresh()
        linear_v = msg.twist.twist.linear.x
        if not imu_fresh:
            previous_wheel_yaw = yaw_from_quaternion(
                self.last_wheel_odom.pose.pose.orientation.z,
                self.last_wheel_odom.pose.pose.orientation.w,
            )
            wheel_delta_yaw = normalize_angle(wheel_yaw - previous_wheel_yaw)
            self.x, self.y, self.yaw = integrate_planar_pose(
                self.x,
                self.y,
                self.yaw,
                linear_v * dt,
                wheel_delta_yaw,
            )
            self.last_imu_angular_z = msg.twist.twist.angular.z
            if not self.warned_no_imu:
                self.get_logger().warn(
                    'No fresh IMU data; continuing from the current fused pose '
                    'with relative wheel odom'
                )
                self.warned_no_imu = True
        else:
            delta_s = linear_v * dt
            yaw_mid = self.yaw - (self.last_imu_angular_z * dt / 2.0)
            self.x += delta_s * math.cos(yaw_mid)
            self.y += delta_s * math.sin(yaw_mid)

        self.last_wheel_odom = msg
        self._publish(msg, linear_v, self.last_imu_angular_z, imu_fresh=imu_fresh)

    def _select_imu_axis(self, msg: Imu) -> float:
        if self.imu_yaw_axis == 'x':
            return msg.angular_velocity.x
        if self.imu_yaw_axis == 'y':
            return msg.angular_velocity.y
        return msg.angular_velocity.z

    def _imu_is_fresh(self) -> bool:
        if not self.gyro_bias_ready or self.last_imu_receive_time_s is None:
            return False
        now_s = self.get_clock().now().nanoseconds / 1e9
        return now_s - self.last_imu_receive_time_s <= self.imu_timeout_s

    def _wheel_is_fresh(self) -> bool:
        if self.last_wheel_receive_time_s is None:
            return False
        now_s = self.get_clock().now().nanoseconds / 1e9
        return (
            now_s - self.last_wheel_receive_time_s
            <= self.wheel_fresh_timeout_s
        )

    def _reset_filter_history(self) -> None:
        self.hampel_samples.clear()
        self.vibration_residuals.clear()
        self.gyro_low_pass.reset()
        self.vibration_variance = 0.0
        self.current_imu_weight = 1.0

    def _update_online_bias(
        self,
        raw_angular_z: float,
        receive_now_s: float,
    ) -> None:
        # Pseudocode:
        #   only while wheel odom is fresh, motion is near zero, and vibration
        #   remains low for a hold period: slowly move raw-axis bias to gyro raw
        # This prevents real turns or encoder dropouts from being learned as bias.
        if not self.enable_online_bias_update or not self._wheel_is_fresh():
            self.stationary_since_s = None
            return

        stationary = (
            abs(self.last_wheel_linear_mps)
            <= self.stationary_linear_threshold_mps
            and abs(self.last_wheel_angular_rad_s)
            <= self.stationary_angular_threshold_rad_s
            and self.vibration_variance <= self.vibration_variance_low
        )
        if not stationary:
            self.stationary_since_s = None
            return
        if self.stationary_since_s is None:
            self.stationary_since_s = receive_now_s
            return
        if receive_now_s - self.stationary_since_s < self.bias_hold_time_s:
            return

        self.gyro_bias_rad_s += self.online_bias_alpha * (
            raw_angular_z - self.gyro_bias_rad_s
        )

    def _publish_filter_diagnostics(
        self,
        raw_rate: float,
        filtered_rate: float,
        vibration_variance: float,
        imu_weight: float,
    ) -> None:
        self.raw_yaw_rate_pub.publish(Float64(data=float(raw_rate)))
        self.filtered_yaw_rate_pub.publish(Float64(data=float(filtered_rate)))
        self.vibration_variance_pub.publish(
            Float64(data=float(vibration_variance))
        )
        self.imu_weight_pub.publish(Float64(data=float(imu_weight)))

    def _publish(
        self,
        wheel_msg: Odometry,
        linear_v: float,
        angular_z: float,
        imu_fresh: bool,
    ) -> None:
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
