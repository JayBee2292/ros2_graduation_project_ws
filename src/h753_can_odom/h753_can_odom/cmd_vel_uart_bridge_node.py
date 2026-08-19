#!/usr/bin/env python3
from __future__ import annotations

import fcntl
import glob
import math
import os
import time
from pathlib import Path
from typing import TextIO

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

try:
    import serial
    from serial import SerialException
except ImportError as exc:
    raise SystemExit('python3-serial is required for h753_can_odom') from exc


FRAME_HEADER_0 = 0xA5
FRAME_HEADER_1 = 0x5A
PACKET_TWIST = 0x10
STM_UART_HINTS = ('stmicroelectronics', 'stlink')
EXCLUDED_UART_HINTS = (
    'canable',
    'openlight',
    'elmuesoft',
    'netcult',
    'slcan',
    'silicon_labs',
    'cp210',
)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def clamp_i16(value: int) -> int:
    return max(-32768, min(32767, value))


def limit_twist(
    linear_mps: float,
    angular_radps: float,
    max_linear_mps: float,
    max_angular_radps: float,
    track_gauge_m: float,
    max_track_speed_mps: float,
) -> tuple[float, float]:
    linear_mps = clamp(linear_mps, -max_linear_mps, max_linear_mps)
    angular_radps = clamp(angular_radps, -max_angular_radps, max_angular_radps)
    half_gauge = max(0.0, track_gauge_m) / 2.0
    left_mps = linear_mps - angular_radps * half_gauge
    right_mps = linear_mps + angular_radps * half_gauge
    peak_track_speed = max(abs(left_mps), abs(right_mps))

    if peak_track_speed > max_track_speed_mps:
        scale = max_track_speed_mps / peak_track_speed
        linear_mps *= scale
        angular_radps *= scale

    return linear_mps, angular_radps


def apply_track_stiction_floor(
    linear_mps: float,
    angular_radps: float,
    track_gauge_m: float,
    max_track_speed_mps: float,
    min_track_pwm_percent: float,
    track_zero_deadband_mps: float,
    min_in_place_turn_pwm_percent: float | None = None,
    in_place_turn_linear_threshold_mps: float = 0.0,
) -> tuple[float, float]:
    """Raise each moving track above the STM open-loop PWM dead zone.

    The STM feedforward is linear: ``track_mps / max_track_speed_mps * 100``.
    Applying the floor after converting body twist to left/right track speeds
    also covers curved motion where only the inside track is below breakaway.
    Both moving tracks are scaled together whenever possible so the requested
    steering ratio is retained.
    """
    effective_min_pwm_percent = min_track_pwm_percent
    if (
        min_in_place_turn_pwm_percent is not None
        and abs(linear_mps) <= in_place_turn_linear_threshold_mps
        and angular_radps != 0.0
    ):
        effective_min_pwm_percent = max(
            effective_min_pwm_percent,
            min_in_place_turn_pwm_percent,
        )

    if effective_min_pwm_percent <= 0.0:
        return linear_mps, angular_radps

    half_gauge = track_gauge_m / 2.0
    left_mps = linear_mps - angular_radps * half_gauge
    right_mps = linear_mps + angular_radps * half_gauge
    min_track_speed_mps = (
        max_track_speed_mps * effective_min_pwm_percent / 100.0
    )

    if abs(left_mps) <= track_zero_deadband_mps:
        left_mps = 0.0
    if abs(right_mps) <= track_zero_deadband_mps:
        right_mps = 0.0

    moving_speeds = [
        abs(track_mps)
        for track_mps in (left_mps, right_mps)
        if track_mps != 0.0
    ]
    if not moving_speeds:
        return 0.0, 0.0

    slowest_mps = min(moving_speeds)
    if slowest_mps < min_track_speed_mps:
        scale = min_track_speed_mps / slowest_mps
        scaled_left_mps = left_mps * scale
        scaled_right_mps = right_mps * scale
        peak_scaled_mps = max(abs(scaled_left_mps), abs(scaled_right_mps))

        if peak_scaled_mps <= max_track_speed_mps:
            left_mps = scaled_left_mps
            right_mps = scaled_right_mps
        else:
            # A ratio below min_pwm/100 cannot simultaneously retain its
            # curvature and keep both tracks inside 0..100% PWM. Clamp to the
            # nearest physically achievable track pair in that edge case.
            def clamp_moving_track(track_mps: float) -> float:
                if track_mps == 0.0:
                    return 0.0
                magnitude = clamp(
                    abs(track_mps),
                    min_track_speed_mps,
                    max_track_speed_mps,
                )
                return math.copysign(magnitude, track_mps)

            left_mps = clamp_moving_track(left_mps)
            right_mps = clamp_moving_track(right_mps)
    linear_mps = (left_mps + right_mps) / 2.0
    angular_radps = (right_mps - left_mps) / track_gauge_m
    return linear_mps, angular_radps


def encode_twist_frame(linear_mps: float, angular_radps: float) -> bytes:
    linear_mmps = clamp_i16(int(round(linear_mps * 1000.0)))
    angular_mradps = clamp_i16(int(round(angular_radps * 1000.0)))
    payload = bytearray([
        FRAME_HEADER_0,
        FRAME_HEADER_1,
        PACKET_TWIST,
        linear_mmps & 0xFF,
        (linear_mmps >> 8) & 0xFF,
        angular_mradps & 0xFF,
        (angular_mradps >> 8) & 0xFF,
    ])
    checksum = 0
    for value in payload:
        checksum ^= value
    payload.append(checksum)
    return bytes(payload)


class InjuryStopLatch:
    """Keep the latest VLM stop state until the server explicitly changes it."""

    def __init__(self) -> None:
        self.active = False

    def update(self, value: int) -> bool:
        requested = value != 0
        changed = requested != self.active
        self.active = requested
        return changed


def serial_alias_names(path: str) -> list[str]:
    resolved_path = os.path.realpath(path)
    return [
        os.path.basename(alias).lower()
        for alias in glob.glob('/dev/serial/by-id/*')
        if os.path.realpath(alias) == resolved_path
    ]


def has_any_hint(name: str, hints: tuple[str, ...]) -> bool:
    return any(hint in name for hint in hints)


def resolve_stm_uart_port(preferred: str) -> str:
    if preferred:
        path = str(Path(preferred).expanduser())
        names = [os.path.basename(path).lower(), *serial_alias_names(path)]
        if any(has_any_hint(name, EXCLUDED_UART_HINTS) for name in names):
            raise RuntimeError(f'Configured UART port resolves to a non-STM device: {path}')
        if not any(has_any_hint(name, STM_UART_HINTS) for name in names):
            raise RuntimeError(f'Configured UART port is not identifiable as ST-LINK: {path}')
        return path

    for path in sorted(glob.glob('/dev/serial/by-id/*')):
        name = os.path.basename(path).lower()
        if any(hint in name for hint in STM_UART_HINTS):
            return path

    raise RuntimeError('No STM ST-LINK /dev/serial/by-id port found')


class UartControlLock:
    def __init__(self, path: str) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream: TextIO = self.path.open('a+', encoding='ascii')
        try:
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.stream.seek(0)
            active_pid = self.stream.read().strip() or 'unknown'
            self.stream.close()
            raise RuntimeError(
                f'UART control is already active (pid={active_pid}, lock={self.path})'
            ) from exc

        self.stream.seek(0)
        self.stream.truncate()
        self.stream.write(f'{os.getpid()}\n')
        self.stream.flush()

    def close(self) -> None:
        if self.stream.closed:
            return
        fcntl.flock(self.stream.fileno(), fcntl.LOCK_UN)
        self.stream.close()


class CmdVelUartBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('h753_cmd_vel_uart_bridge')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')
        self.declare_parameter('injury_stop_topic', '/safety/vlm_stop')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('require_scan', True)
        self.declare_parameter('scan_timeout_s', 0.50)
        self.declare_parameter('uart_port', '')
        self.declare_parameter('uart_baud', 921600)
        self.declare_parameter(
            'lock_file',
            '~/ros2_graduation_project_ws/h753_ros_humble/tools/logs/xbox_uart_control.lock',
        )
        self.declare_parameter('deadman_timeout_s', 0.30)
        # 20 Hz steady cadence, close to the legacy 60 ms drive loop, well under
        # the firmware's 800 ms command-hold timeout.
        self.declare_parameter('transmit_period_s', 0.05)
        self.declare_parameter('max_linear_mps', 0.30)
        self.declare_parameter('max_angular_radps', 0.80)
        self.declare_parameter('linear_command_sign', 1.0)
        self.declare_parameter('angular_command_sign', 1.0)
        # Keep this equal to STM EFFECTIVE_TRACK_WIDTH_M. The bridge uses it
        # both for track-speed limiting and for PWM breakaway compensation.
        self.declare_parameter('track_gauge_m', 0.45)
        self.declare_parameter('max_track_speed_mps', 0.60)
        self.declare_parameter('min_track_pwm_percent', 0.0)
        self.declare_parameter('min_in_place_turn_pwm_percent', 0.0)
        self.declare_parameter('in_place_turn_linear_threshold_mps', 0.02)
        self.declare_parameter('track_zero_deadband_mps', 0.01)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.injury_stop_topic = str(self.get_parameter('injury_stop_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.require_scan = bool(self.get_parameter('require_scan').value)
        self.scan_timeout_s = float(self.get_parameter('scan_timeout_s').value)
        self.requested_uart_port = str(self.get_parameter('uart_port').value)
        self.uart_baud = int(self.get_parameter('uart_baud').value)
        self.deadman_timeout_s = float(self.get_parameter('deadman_timeout_s').value)
        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.max_angular_radps = float(self.get_parameter('max_angular_radps').value)
        self.linear_command_sign = float(
            self.get_parameter('linear_command_sign').value
        )
        self.angular_command_sign = float(
            self.get_parameter('angular_command_sign').value
        )
        self.track_gauge_m = float(self.get_parameter('track_gauge_m').value)
        self.max_track_speed_mps = float(self.get_parameter('max_track_speed_mps').value)
        self.min_track_pwm_percent = float(
            self.get_parameter('min_track_pwm_percent').value
        )
        self.min_in_place_turn_pwm_percent = float(
            self.get_parameter('min_in_place_turn_pwm_percent').value
        )
        self.in_place_turn_linear_threshold_mps = float(
            self.get_parameter('in_place_turn_linear_threshold_mps').value
        )
        self.track_zero_deadband_mps = float(
            self.get_parameter('track_zero_deadband_mps').value
        )

        if self.track_gauge_m <= 0.0:
            raise ValueError('track_gauge_m must be positive')
        if self.max_track_speed_mps <= 0.0:
            raise ValueError('max_track_speed_mps must be positive')
        if not 0.0 <= self.min_track_pwm_percent <= 100.0:
            raise ValueError('min_track_pwm_percent must be between 0 and 100')
        if not 0.0 <= self.min_in_place_turn_pwm_percent <= 100.0:
            raise ValueError(
                'min_in_place_turn_pwm_percent must be between 0 and 100'
            )
        if self.in_place_turn_linear_threshold_mps < 0.0:
            raise ValueError(
                'in_place_turn_linear_threshold_mps must be non-negative'
            )
        if not 0.0 <= self.track_zero_deadband_mps < self.max_track_speed_mps:
            raise ValueError(
                'track_zero_deadband_mps must be non-negative and below max track speed'
            )
        if self.deadman_timeout_s <= 0.0:
            raise ValueError('deadman_timeout_s must be positive')
        if self.scan_timeout_s <= 0.0:
            raise ValueError('scan_timeout_s must be positive')
        if self.linear_command_sign not in (-1.0, 1.0):
            raise ValueError('linear_command_sign must be -1.0 or 1.0')
        if self.angular_command_sign not in (-1.0, 1.0):
            raise ValueError('angular_command_sign must be -1.0 or 1.0')

        self.control_lock = UartControlLock(str(self.get_parameter('lock_file').value))
        self.serial_port: serial.Serial | None = None
        self.active_linear_mps = 0.0
        self.active_angular_radps = 0.0
        self.last_cmd_time: float | None = None
        self.last_scan_time: float | None = None
        self.last_reconnect_attempt = 0.0
        self.deadman_active = True
        self.scan_stale_active = False
        self.injury_stop = InjuryStopLatch()

        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10)
        injury_stop_qos = QoSProfile(depth=10)
        injury_stop_qos.reliability = ReliabilityPolicy.RELIABLE
        injury_stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Int32,
            self.injury_stop_topic,
            self._injury_stop_callback,
            injury_stop_qos,
        )
        if self.require_scan:
            self.create_subscription(
                LaserScan,
                self.scan_topic,
                self._scan_callback,
                qos_profile_sensor_data,
            )
        self.create_timer(
            float(self.get_parameter('transmit_period_s').value),
            self._timer_callback,
        )
        self._try_open()
        self.get_logger().info(
            f'UART bridge ready: {self.cmd_vel_topic} -> STM CMD_TWIST, '
            f'limits=({self.max_linear_mps:.2f} m/s, {self.max_angular_radps:.2f} rad/s), '
            f'signs=({self.linear_command_sign:+.0f}, {self.angular_command_sign:+.0f}), '
            f'PWM floor={self.min_track_pwm_percent:.1f}% '
            f'({self.max_track_speed_mps * self.min_track_pwm_percent / 100.0:.3f} m/s/track), '
            f'in-place turn floor={self.min_in_place_turn_pwm_percent:.1f}%, '
            f'scan_guard={self.require_scan}, injury_stop={self.injury_stop_topic}'
        )

    def destroy_node(self) -> bool:
        self._close_serial(send_stop=True)
        self.control_lock.close()
        return super().destroy_node()

    def _cmd_vel_callback(self, msg: Twist) -> None:
        linear_mps, angular_radps = limit_twist(
            msg.linear.x * self.linear_command_sign,
            msg.angular.z * self.angular_command_sign,
            self.max_linear_mps,
            self.max_angular_radps,
            self.track_gauge_m,
            self.max_track_speed_mps,
        )
        if not math.isfinite(linear_mps) or not math.isfinite(angular_radps):
            self.get_logger().error('Rejected non-finite cmd_vel')
            return

        self.active_linear_mps = linear_mps
        self.active_angular_radps = angular_radps
        self.last_cmd_time = time.monotonic()
        self.deadman_active = False
        # Do not transmit here. The legacy xbox_uart_can_telemetry.py drive loop
        # sends the latest command on a fixed cadence regardless of input timing;
        # mirroring that keeps STM frames evenly spaced. Transmitting on every
        # irregular /cmd_vel_safe callback (collision_monitor output) plus the
        # timer produced jittery spacing that the motors showed as stutter.

    def _scan_callback(self, _msg: LaserScan) -> None:
        self.last_scan_time = time.monotonic()

    def _injury_stop_callback(self, msg: Int32) -> None:
        value = int(msg.data)
        changed = self.injury_stop.update(value)

        if value not in (0, 1):
            self.get_logger().error(
                f'Invalid injury stop value {value}; treating non-zero as stop'
            )

        if changed:
            if self.injury_stop.active:
                self.get_logger().warn(
                    'VLM injury stop asserted; holding STM stop until explicit value 0'
                )
            else:
                self.get_logger().info(
                    'VLM injury stop cleared; UART drive commands enabled'
                )

        # Send the changed safety state immediately instead of waiting for the
        # next 20 Hz timer tick. The normal timer keeps repeating the stop frame.
        self._send_active_command()

    def _timer_callback(self) -> None:
        self._try_open()
        self._drain_input()

        now = time.monotonic()
        if self.last_cmd_time is None or now - self.last_cmd_time > self.deadman_timeout_s:
            self.active_linear_mps = 0.0
            self.active_angular_radps = 0.0
            if not self.deadman_active:
                self.get_logger().warn('cmd_vel timeout; sending stop')
            self.deadman_active = True

        scan_stale = not self._scan_is_fresh()
        if scan_stale and not self.scan_stale_active:
            self.get_logger().warn('scan timeout; holding STM stop command')
        elif not scan_stale and self.scan_stale_active:
            self.get_logger().info('scan stream recovered; UART drive commands enabled')
        self.scan_stale_active = scan_stale

        self._send_active_command()

    def _try_open(self) -> None:
        if self.serial_port is not None and self.serial_port.is_open:
            return

        now = time.monotonic()
        if now - self.last_reconnect_attempt < 1.0:
            return
        self.last_reconnect_attempt = now
        try:
            port = resolve_stm_uart_port(self.requested_uart_port)
        except RuntimeError as exc:
            self.get_logger().warn(str(exc))
            return

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.uart_baud,
                timeout=0,
                write_timeout=0.1,
                exclusive=True,
            )
        except (SerialException, OSError) as exc:
            self.serial_port = None
            self.get_logger().warn(f'Waiting for STM UART {port}: {exc}')
            return

        self.get_logger().info(f'STM UART open: {port} @ {self.uart_baud}')
        self._send_active_command()

    def _apply_stiction_floor(self, linear_mps: float, angular_radps: float) -> tuple[float, float]:
        return apply_track_stiction_floor(
            linear_mps,
            angular_radps,
            self.track_gauge_m,
            self.max_track_speed_mps,
            self.min_track_pwm_percent,
            self.track_zero_deadband_mps,
            self.min_in_place_turn_pwm_percent,
            self.in_place_turn_linear_threshold_mps,
        )

    def _send_active_command(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            return
        linear_mps = self.active_linear_mps
        angular_radps = self.active_angular_radps
        if self.injury_stop.active or not self._scan_is_fresh():
            linear_mps = 0.0
            angular_radps = 0.0
        linear_mps, angular_radps = self._apply_stiction_floor(linear_mps, angular_radps)
        try:
            self.serial_port.write(
                encode_twist_frame(linear_mps, angular_radps)
            )
            # Match the legacy drive loop, which flushes after every frame so
            # the STM receives each command promptly instead of in bursts.
            self.serial_port.flush()
        except (SerialException, OSError) as exc:
            self.get_logger().warn(f'STM UART write failed, reconnecting: {exc}')
            self._close_serial(send_stop=False)

    def _scan_is_fresh(self) -> bool:
        if not self.require_scan:
            return True
        if self.last_scan_time is None:
            return False
        return time.monotonic() - self.last_scan_time <= self.scan_timeout_s

    def _drain_input(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting:
                self.serial_port.read(waiting)
        except (SerialException, OSError) as exc:
            self.get_logger().warn(f'STM UART read failed, reconnecting: {exc}')
            self._close_serial(send_stop=False)

    def _close_serial(self, send_stop: bool) -> None:
        if self.serial_port is None:
            return
        try:
            if self.serial_port.is_open and send_stop:
                stop_frame = encode_twist_frame(0.0, 0.0)
                for _ in range(3):
                    self.serial_port.write(stop_frame)
                    time.sleep(0.02)
            if self.serial_port.is_open:
                self.serial_port.close()
        except (SerialException, OSError):
            pass
        finally:
            self.serial_port = None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: CmdVelUartBridgeNode | None = None
    try:
        node = CmdVelUartBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
