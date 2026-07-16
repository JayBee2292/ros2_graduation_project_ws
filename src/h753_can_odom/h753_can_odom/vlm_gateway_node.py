#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String, UInt8


DEFAULT_ENABLED_MODES = (3, 4, 5)


@dataclass(frozen=True)
class StopUpdate:
    accepted: bool
    value: int
    changed: bool
    valid: bool


class VlmSafetyGate:
    """Mode-aware state machine between an external VLM and robot safety."""

    def __init__(self, enabled_modes: tuple[int, ...] = DEFAULT_ENABLED_MODES) -> None:
        self.enabled_modes = frozenset(enabled_modes)
        self.mode: int | None = None
        self.stop_active = False

    @property
    def mode_enabled(self) -> bool:
        return self.mode in self.enabled_modes

    @property
    def communication_enabled(self) -> bool:
        # Once stop is asserted, keep sending images and accepting VLM updates
        # until the server explicitly publishes zero. This lets the server keep
        # observing the person whose detection caused the stop.
        return self.mode_enabled or self.stop_active

    def update_mode(self, mode: int) -> bool:
        was_enabled = self.communication_enabled
        self.mode = mode
        return was_enabled != self.communication_enabled

    def update_stop(self, value: int) -> StopUpdate:
        if not self.communication_enabled:
            return StopUpdate(False, int(self.stop_active), False, value in (0, 1))

        valid = value in (0, 1)
        normalized = 0 if value == 0 else 1
        changed = bool(normalized) != self.stop_active
        self.stop_active = bool(normalized)
        return StopUpdate(True, normalized, changed, valid)


class VlmGatewayNode(Node):
    """Own the robot side of the DDS contract with the remote VLM server."""

    def __init__(self) -> None:
        super().__init__('h753_vlm_gateway')

        self.declare_parameter(
            'camera_input_topic',
            '/camera/camera/color/image_raw/compressed',
        )
        self.declare_parameter(
            'server_image_topic',
            '/vlm/request/image/compressed',
        )
        # These three defaults preserve the currently deployed server contract.
        # The server can later remap them under /vlm/response without changing
        # the robot's internal /safety/vlm_stop interface.
        self.declare_parameter('server_injury_stop_topic', '/vlm/injury_stop')
        self.declare_parameter('server_result_topic', '/vlm/result')
        self.declare_parameter('server_result_detail_topic', '/vlm/result_detail')
        self.declare_parameter('safety_stop_topic', '/safety/vlm_stop')
        self.declare_parameter('result_output_topic', '/vlm/gateway/result')
        self.declare_parameter(
            'result_detail_output_topic',
            '/vlm/gateway/result_detail',
        )
        self.declare_parameter('status_topic', '/vlm/status')
        self.declare_parameter('mode_topic', '/robot_mode')
        self.declare_parameter('enabled_modes', list(DEFAULT_ENABLED_MODES))
        self.declare_parameter('response_timeout_s', 3.0)
        self.declare_parameter('status_period_s', 1.0)
        self.declare_parameter('max_result_chars', 20000)

        self.camera_input_topic = str(
            self.get_parameter('camera_input_topic').value
        )
        self.server_image_topic = str(
            self.get_parameter('server_image_topic').value
        )
        self.server_injury_stop_topic = str(
            self.get_parameter('server_injury_stop_topic').value
        )
        self.server_result_topic = str(
            self.get_parameter('server_result_topic').value
        )
        self.server_result_detail_topic = str(
            self.get_parameter('server_result_detail_topic').value
        )
        self.safety_stop_topic = str(
            self.get_parameter('safety_stop_topic').value
        )
        self.result_output_topic = str(
            self.get_parameter('result_output_topic').value
        )
        self.result_detail_output_topic = str(
            self.get_parameter('result_detail_output_topic').value
        )
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.mode_topic = str(self.get_parameter('mode_topic').value)
        enabled_modes = tuple(
            int(mode) for mode in self.get_parameter('enabled_modes').value
        )
        self.response_timeout_s = float(
            self.get_parameter('response_timeout_s').value
        )
        self.status_period_s = float(self.get_parameter('status_period_s').value)
        self.max_result_chars = int(self.get_parameter('max_result_chars').value)

        if self.camera_input_topic == self.server_image_topic:
            raise ValueError('camera_input_topic and server_image_topic must differ')
        if self.server_injury_stop_topic == self.safety_stop_topic:
            raise ValueError(
                'server_injury_stop_topic and safety_stop_topic must differ'
            )
        if self.response_timeout_s <= 0.0 or not math.isfinite(
            self.response_timeout_s
        ):
            raise ValueError('response_timeout_s must be finite and positive')
        if self.status_period_s <= 0.0 or not math.isfinite(self.status_period_s):
            raise ValueError('status_period_s must be finite and positive')
        if self.max_result_chars <= 0:
            raise ValueError('max_result_chars must be positive')

        self.gate = VlmSafetyGate(enabled_modes)
        self.last_response_at: float | None = None
        self.forwarded_images = 0
        self.received_responses = 0
        self.last_status_state: str | None = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.image_pub = self.create_publisher(
            CompressedImage,
            self.server_image_topic,
            sensor_qos,
        )
        self.safety_pub = self.create_publisher(
            Int32,
            self.safety_stop_topic,
            latched_qos,
        )
        self.result_pub = self.create_publisher(
            String,
            self.result_output_topic,
            reliable_qos,
        )
        self.result_detail_pub = self.create_publisher(
            String,
            self.result_detail_output_topic,
            reliable_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            latched_qos,
        )

        self.create_subscription(
            CompressedImage,
            self.camera_input_topic,
            self._image_callback,
            sensor_qos,
        )
        self.create_subscription(
            Int32,
            self.server_injury_stop_topic,
            self._injury_stop_callback,
            reliable_qos,
        )
        self.create_subscription(
            String,
            self.server_result_topic,
            self._result_callback,
            reliable_qos,
        )
        self.create_subscription(
            String,
            self.server_result_detail_topic,
            self._result_detail_callback,
            reliable_qos,
        )
        self.create_subscription(
            UInt8,
            self.mode_topic,
            self._mode_callback,
            latched_qos,
        )
        self.create_timer(self.status_period_s, self._status_timer_callback)

        # Absence of the optional server does not block driving because the UART
        # latch defaults inactive. Do not publish a synthetic zero here: after a
        # gateway restart it could incorrectly clear a previously asserted stop.
        self._publish_status()
        self.get_logger().info(
            'VLM gateway ready: '
            f'{self.camera_input_topic} -> {self.server_image_topic}, '
            f'{self.server_injury_stop_topic} -> {self.safety_stop_topic}, '
            f'enabled_modes={sorted(self.gate.enabled_modes)}'
        )

    def _image_callback(self, msg: CompressedImage) -> None:
        if not self.gate.communication_enabled:
            return
        self.image_pub.publish(msg)
        self.forwarded_images += 1

    def _mode_callback(self, msg: UInt8) -> None:
        communication_changed = self.gate.update_mode(int(msg.data))
        if communication_changed:
            state = 'enabled' if self.gate.communication_enabled else 'disabled'
            self.get_logger().info(
                f'VLM communication {state} for robot mode {self.gate.mode}'
            )
        self._publish_status()

    def _injury_stop_callback(self, msg: Int32) -> None:
        value = int(msg.data)
        self._record_response()
        update = self.gate.update_stop(value)
        if not update.accepted:
            return
        if not update.valid:
            self.get_logger().error(
                f'Invalid server injury_stop={value}; normalized to fail-safe stop=1'
            )
        self._publish_safety_stop(update.value)
        if update.changed:
            if update.value:
                self.get_logger().warn(
                    'Validated VLM stop asserted; image forwarding remains active '
                    'until an explicit zero is received'
                )
            else:
                self.get_logger().info('Validated VLM stop cleared')
        self._publish_status()

    def _result_callback(self, msg: String) -> None:
        self._record_response()
        if not self.gate.communication_enabled:
            return
        self.result_pub.publish(String(data=self._bounded_text(msg.data)))

    def _result_detail_callback(self, msg: String) -> None:
        self._record_response()
        if not self.gate.communication_enabled:
            return
        self.result_detail_pub.publish(
            String(data=self._bounded_text(msg.data))
        )

    def _bounded_text(self, value: str) -> str:
        if len(value) <= self.max_result_chars:
            return value
        self.get_logger().warn(
            f'VLM result exceeded {self.max_result_chars} characters; truncating'
        )
        return value[: self.max_result_chars]

    def _record_response(self) -> None:
        self.last_response_at = time.monotonic()
        self.received_responses += 1

    def _publish_safety_stop(self, value: int) -> None:
        self.safety_pub.publish(Int32(data=1 if value else 0))

    def _status_timer_callback(self) -> None:
        self._publish_status()

    def _publish_status(self) -> None:
        now = time.monotonic()
        age = (
            None
            if self.last_response_at is None
            else max(0.0, now - self.last_response_at)
        )
        publisher_counts = {
            'injury_stop': self.count_publishers(self.server_injury_stop_topic),
            'result': self.count_publishers(self.server_result_topic),
            'result_detail': self.count_publishers(
                self.server_result_detail_topic
            ),
        }
        discovered = any(count > 0 for count in publisher_counts.values())
        stale = age is not None and age > self.response_timeout_s

        if not self.gate.communication_enabled:
            state = 'disabled'
        elif self.gate.stop_active:
            state = 'stop_latched'
        elif not discovered:
            state = 'waiting_for_server'
        elif age is None:
            state = 'waiting_for_first_response'
        elif stale:
            state = 'response_stale'
        else:
            state = 'online'

        status = {
            'state': state,
            'mode': self.gate.mode,
            'mode_enabled': self.gate.mode_enabled,
            'communication_enabled': self.gate.communication_enabled,
            'injury_stop': int(self.gate.stop_active),
            'server_discovered': discovered,
            'response_age_s': None if age is None else round(age, 3),
            'response_timeout_s': self.response_timeout_s,
            'publisher_counts': publisher_counts,
            'forwarded_images': self.forwarded_images,
            'received_responses': self.received_responses,
        }
        self.status_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False, sort_keys=True))
        )
        if state != self.last_status_state:
            if state in (
                'waiting_for_server',
                'response_stale',
                'stop_latched',
            ):
                self.get_logger().warn(f'VLM gateway state: {state}')
            else:
                self.get_logger().info(f'VLM gateway state: {state}')
            self.last_status_state = state


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: VlmGatewayNode | None = None
    try:
        node = VlmGatewayNode()
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
