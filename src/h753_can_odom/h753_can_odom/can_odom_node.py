#!/usr/bin/env python3
from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

try:
    import serial
    from serial import SerialException
except ImportError as exc:
    raise SystemExit('python3-serial is required for h753_can_odom') from exc


CAN_STATE_ID = 0x100
CAN_ENCODER_ID = 0x101
CAN_COMMAND_ID = 0x1FF
CAN_ENABLE_REFRESH_S = 1.0
DEFAULT_CAN_PORT = (
    '/dev/serial/by-id/'
    'usb-ElmueSoft__netcult.ch_elmue__Slcan_2.5_-_Multiboard_209F338336305011-if00'
)
FD_DLC_TO_LENGTH = {
    **{str(i): i for i in range(9)},
    '9': 12,
    'A': 16,
    'B': 20,
    'C': 24,
    'D': 32,
    'E': 48,
    'F': 64,
}
FD_LENGTH_TO_DLC = {value: key for key, value in FD_DLC_TO_LENGTH.items()}
STATE_PAYLOAD = struct.Struct('<IIffffffhhhhBBIBBBBBBBBBII')
ENCODER_PAYLOAD = struct.Struct('<IIqqqqIIIIBB')
FDCAN_ERROR_NAMES = {
    0: 'none',
    1: 'stuff',
    2: 'form',
    3: 'ack',
    4: 'bit1',
    5: 'bit0',
    6: 'crc',
    7: 'no-change',
}


@dataclass
class StateTelemetry:
    seq: int
    timestamp_ms: int
    robot_linear_v: float
    robot_angular_w: float
    flags: int
    bus_off_count: int
    last_error_code: int
    data_last_error_code: int
    tx_error_count: int
    rx_error_count: int
    error_logging_count: int
    error_passive: int
    error_warning: int
    protocol_exception: int
    tdc_value: int
    error_event_count: int
    last_error_status_its: int


@dataclass
class EncoderTelemetry:
    seq: int
    timestamp_ms: int
    ticks: tuple[int, int, int, int]
    delta_abs: tuple[int, int, int, int]
    valid_mask: int
    fault_mask: int


class CanableSlcanReader:
    def __init__(
        self,
        port: str,
        baud: int,
        nominal_command: str,
        data_command: str,
    ) -> None:
        self.port_name = port
        self.baud = baud
        self.nominal_command = nominal_command
        self.data_command = data_command
        self.port: serial.Serial | None = None
        self.buffer = bytearray()

    def is_open(self) -> bool:
        return self.port is not None and self.port.is_open

    def open(self) -> None:
        self.close()
        self.port = serial.Serial(
            self.port_name,
            self.baud,
            timeout=0,
            exclusive=True,
        )
        self._send_command('C')
        self._drain(0.05)
        self._send_command(self.nominal_command)
        self._send_command(self.data_command)
        self._send_command('M0')
        self._send_command('A0')
        self._send_command('O')
        self._drain(0.10)
        self.send_enable(True)

    def close(self) -> None:
        if self.port is None:
            return

        try:
            if self.port.is_open:
                self.send_enable(False)
                time.sleep(0.02)
                self._send_command('C')
                self.port.close()
        except (SerialException, OSError):
            pass
        finally:
            self.port = None
            self.buffer.clear()

    def _send_command(self, command: str) -> None:
        if self.port is None:
            return
        self.port.write(command.encode('ascii') + b'\r')
        self.port.flush()

    def _drain(self, duration_s: float) -> None:
        if self.port is None:
            return
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            waiting = self.port.in_waiting
            if waiting:
                self.port.read(waiting)
            time.sleep(0.01)

    def send_frame(self, can_id: int, payload: bytes, brs: bool = True) -> None:
        if len(payload) not in FD_LENGTH_TO_DLC:
            raise ValueError(f'Unsupported CAN FD payload length: {len(payload)}')
        frame_type = 'b' if brs else 'd'
        dlc = FD_LENGTH_TO_DLC[len(payload)]
        self._send_command(f'{frame_type}{can_id:03X}{dlc}{payload.hex().upper()}')

    def send_enable(self, enabled: bool) -> None:
        payload = bytes([1, 1 if enabled else 0]).ljust(8, b'\x00')
        self.send_frame(CAN_COMMAND_ID, payload, brs=False)

    def read_frames(self) -> list[tuple[int, bytes]]:
        if self.port is None:
            return []

        waiting = self.port.in_waiting
        if waiting:
            self.buffer.extend(self.port.read(waiting))
            self.buffer = bytearray(self.buffer.replace(b'\n', b'\r'))

        frames: list[tuple[int, bytes]] = []
        while b'\r' in self.buffer:
            raw_line, _, remainder = self.buffer.partition(b'\r')
            self.buffer = bytearray(remainder)
            frame = self._decode_line(raw_line.decode('ascii', errors='replace').strip())
            if frame is not None:
                frames.append(frame)

        return frames

    @staticmethod
    def _decode_line(line: str) -> tuple[int, bytes] | None:
        if not line:
            return None

        frame_type = line[0]
        try:
            if frame_type in {'d', 'b'} and len(line) >= 5:
                can_id = int(line[1:4], 16)
                dlc_char = line[4].upper()
                data_start = 5
            elif frame_type in {'D', 'B'} and len(line) >= 10:
                can_id = int(line[1:9], 16)
                dlc_char = line[9].upper()
                data_start = 10
            else:
                return None
        except ValueError:
            return None

        length = FD_DLC_TO_LENGTH.get(dlc_char)
        if length is None:
            return None

        data_hex = line[data_start:data_start + length * 2]
        try:
            return can_id, bytes.fromhex(data_hex)
        except ValueError:
            return None


class H753CanOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('h753_can_odom')

        self.declare_parameter('can_port', DEFAULT_CAN_PORT)
        self.declare_parameter('can_baud', 921600)
        self.declare_parameter('can_nominal_command', 'S8')
        self.declare_parameter('can_data_command', 'Y5')
        self.declare_parameter('poll_period_s', 0.01)
        self.declare_parameter('ppr', 548776)
        self.declare_parameter('wheel_diameter_m', 0.21)
        self.declare_parameter('track_width_m', 0.50)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_wheel_speed_mps', 3.5)
        self.declare_parameter('max_dt_s', 0.5)
        self.declare_parameter('delta_margin_ticks', 2048)
        self.declare_parameter('odom_linear_sign', 1.0)
        self.declare_parameter('odom_angular_sign', 1.0)

        self.can_reader = CanableSlcanReader(
            self.get_parameter('can_port').value,
            int(self.get_parameter('can_baud').value),
            self.get_parameter('can_nominal_command').value,
            self.get_parameter('can_data_command').value,
        )
        self.ppr = int(self.get_parameter('ppr').value)
        self.wheel_diameter_m = float(self.get_parameter('wheel_diameter_m').value)
        self.track_width_m = float(self.get_parameter('track_width_m').value)
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.max_wheel_speed_mps = float(self.get_parameter('max_wheel_speed_mps').value)
        self.max_dt_s = float(self.get_parameter('max_dt_s').value)
        self.delta_margin_ticks = int(self.get_parameter('delta_margin_ticks').value)
        self.odom_linear_sign = float(self.get_parameter('odom_linear_sign').value)
        self.odom_angular_sign = float(self.get_parameter('odom_angular_sign').value)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.twist_pub = self.create_publisher(Twist, 'odom_vel', 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.last_encoder: EncoderTelemetry | None = None
        self.last_state: StateTelemetry | None = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_reconnect_attempt = 0.0
        self.last_enable_sent_at = 0.0

        self.timer = self.create_timer(
            float(self.get_parameter('poll_period_s').value),
            self._timer_callback,
        )
        self._try_open()

    def destroy_node(self) -> bool:
        self.can_reader.close()
        return super().destroy_node()

    def _try_open(self) -> None:
        now = time.monotonic()
        if now - self.last_reconnect_attempt < 1.0:
            return
        self.last_reconnect_attempt = now
        try:
            self.can_reader.open()
        except (SerialException, OSError) as exc:
            self.get_logger().warn(f'Waiting for CANable port {self.can_reader.port_name}: {exc}')
            return
        self.get_logger().info(
            f'CANable open: {self.can_reader.port_name} '
            f'{self.can_reader.nominal_command}/{self.can_reader.data_command}'
        )
        self.last_enable_sent_at = time.monotonic()

    def _timer_callback(self) -> None:
        if not self.can_reader.is_open():
            self._try_open()
            return

        try:
            now = time.monotonic()
            if now - self.last_enable_sent_at >= CAN_ENABLE_REFRESH_S:
                self.can_reader.send_enable(True)
                self.last_enable_sent_at = now
            frames = self.can_reader.read_frames()
        except (SerialException, OSError) as exc:
            self.get_logger().warn(f'CANable read failed, reconnecting: {exc}')
            self.can_reader.close()
            return

        for can_id, payload in frames:
            if can_id == CAN_STATE_ID:
                state = self._decode_state(payload)
                if state is not None:
                    bus_off_changed = (
                        state.bus_off_count > 0 and
                        (self.last_state is None or
                         state.bus_off_count != self.last_state.bus_off_count)
                    )
                    error_event_changed = (
                        state.error_event_count > 0 and
                        (self.last_state is None or
                         state.error_event_count != self.last_state.error_event_count)
                    )
                    if bus_off_changed or error_event_changed:
                        self.get_logger().warn(
                            'STM32 FDCAN diagnostic '
                            f'count={state.bus_off_count} '
                            f'events={state.error_event_count} '
                            f'ITs=0x{state.last_error_status_its:08X} '
                            f'LEC={state.last_error_code}'
                            f'({FDCAN_ERROR_NAMES.get(state.last_error_code, "unknown")}) '
                            f'DLEC={state.data_last_error_code}'
                            f'({FDCAN_ERROR_NAMES.get(state.data_last_error_code, "unknown")}) '
                            f'TEC={state.tx_error_count} REC={state.rx_error_count} '
                            f'CEL={state.error_logging_count} '
                            f'EP={state.error_passive} EW={state.error_warning} '
                            f'PXE={state.protocol_exception} TDCV={state.tdc_value}'
                        )
                    self.last_state = state
            elif can_id == CAN_ENCODER_ID:
                encoder = self._decode_encoder(payload)
                if encoder is not None:
                    self._handle_encoder(encoder)

    @staticmethod
    def _decode_state(payload: bytes) -> StateTelemetry | None:
        if len(payload) < STATE_PAYLOAD.size:
            return None
        values = STATE_PAYLOAD.unpack_from(payload)
        return StateTelemetry(
            seq=values[0],
            timestamp_ms=values[1],
            robot_linear_v=values[2],
            robot_angular_w=values[3],
            flags=values[13],
            bus_off_count=values[14],
            last_error_code=values[15],
            data_last_error_code=values[16],
            tx_error_count=values[17],
            rx_error_count=values[18],
            error_logging_count=values[19],
            error_passive=values[20],
            error_warning=values[21],
            protocol_exception=values[22],
            tdc_value=values[23],
            error_event_count=values[24],
            last_error_status_its=values[25],
        )

    @staticmethod
    def _decode_encoder(payload: bytes) -> EncoderTelemetry | None:
        if len(payload) < ENCODER_PAYLOAD.size:
            return None
        values = ENCODER_PAYLOAD.unpack_from(payload)
        return EncoderTelemetry(
            seq=values[0],
            timestamp_ms=values[1],
            ticks=(values[2], values[3], values[4], values[5]),
            delta_abs=(values[6], values[7], values[8], values[9]),
            valid_mask=values[10],
            fault_mask=values[11],
        )

    def _handle_encoder(self, encoder: EncoderTelemetry) -> None:
        if encoder.valid_mask != 0x0F or encoder.fault_mask != 0:
            self.get_logger().warn(
                f'Encoder sample rejected by STM: valid=0x{encoder.valid_mask:X} '
                f'fault=0x{encoder.fault_mask:X}'
            )
            self.last_encoder = encoder
            return

        if self.last_encoder is None:
            self.last_encoder = encoder
            self.get_logger().info('Encoder baseline received; odom integration starts on next frame')
            return

        dt_ms = (encoder.timestamp_ms - self.last_encoder.timestamp_ms) & 0xFFFFFFFF
        dt = dt_ms / 1000.0
        if dt <= 0.0 or dt > self.max_dt_s:
            self.get_logger().warn(f'Encoder dt out of range ({dt:.3f}s); baseline reset')
            self.last_encoder = encoder
            return

        delta_ticks = tuple(
            current - previous
            for current, previous in zip(encoder.ticks, self.last_encoder.ticks)
        )
        if not self._delta_ticks_are_plausible(delta_ticks, dt):
            self.get_logger().warn(f'Encoder delta out of range {delta_ticks}; baseline reset')
            self.last_encoder = encoder
            return

        self.last_encoder = encoder
        self._integrate_and_publish(delta_ticks, dt)

    def _delta_ticks_are_plausible(self, delta_ticks: tuple[int, int, int, int], dt: float) -> bool:
        circumference_m = math.pi * self.wheel_diameter_m
        max_ticks = int(
            (self.max_wheel_speed_mps * dt / circumference_m) * self.ppr
            + self.delta_margin_ticks
        )
        return all(abs(delta) <= max_ticks for delta in delta_ticks)

    def _integrate_and_publish(self, delta_ticks: tuple[int, int, int, int], dt: float) -> None:
        fl, fr, rl, rr = delta_ticks
        meters_per_tick = math.pi * self.wheel_diameter_m / self.ppr
        left_delta_m = (fl + rl) / 2.0 * meters_per_tick
        right_delta_m = (fr + rr) / 2.0 * meters_per_tick
        delta_s = (left_delta_m + right_delta_m) / 2.0
        delta_yaw = (right_delta_m - left_delta_m) / self.track_width_m
        delta_s *= self.odom_linear_sign
        delta_yaw *= self.odom_angular_sign
        yaw_mid = self.yaw + delta_yaw / 2.0

        self.x += delta_s * math.cos(yaw_mid)
        self.y += delta_s * math.sin(yaw_mid)
        self.yaw = self._normalize_angle(self.yaw + delta_yaw)

        linear_v = delta_s / dt
        angular_w = delta_yaw / dt
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = linear_v
        odom.twist.twist.angular.z = angular_w
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.10
        self.odom_pub.publish(odom)

        twist = Twist()
        twist.linear.x = linear_v
        twist.angular.z = angular_w
        self.twist_pub.publish(twist)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = odom.pose.pose.orientation.z
            transform.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = H753CanOdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
