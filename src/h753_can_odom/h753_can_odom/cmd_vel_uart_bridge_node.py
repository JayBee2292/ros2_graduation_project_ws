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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

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
        self.declare_parameter('track_gauge_m', 0.45)
        self.declare_parameter('max_track_speed_mps', 0.60)
        self.declare_parameter('min_linear_mps', 0.0)
        self.declare_parameter('min_angular_radps', 0.0)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
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
        self.min_linear_mps = float(self.get_parameter('min_linear_mps').value)
        self.min_angular_radps = float(self.get_parameter('min_angular_radps').value)

        if self.max_track_speed_mps <= 0.0:
            raise ValueError('max_track_speed_mps must be positive')
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

        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10)
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
            f'scan_guard={self.require_scan}'
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
        if self.min_linear_mps > 0.0 and 0.0 < abs(linear_mps) < self.min_linear_mps:
            linear_mps = math.copysign(self.min_linear_mps, linear_mps)
        if self.min_angular_radps > 0.0 and 0.0 < abs(angular_radps) < self.min_angular_radps:
            angular_radps = math.copysign(self.min_angular_radps, angular_radps)
        return linear_mps, angular_radps

    def _send_active_command(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            return
        linear_mps = self.active_linear_mps
        angular_radps = self.active_angular_radps
        if not self._scan_is_fresh():
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
