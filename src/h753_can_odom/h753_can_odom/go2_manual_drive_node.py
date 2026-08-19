#!/usr/bin/env python3
from __future__ import annotations

import time
from collections.abc import Sequence

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

from h753_can_odom.robot_mode_manager_node import (
    apply_deadzone,
    joystick_to_drive_twist,
    limit_planar_twist,
    select_autonomous_drive_twist,
)


def indexed_axis(axes: Sequence[float], index: int) -> float:
    return float(axes[index]) if 0 <= index < len(axes) else 0.0


def indexed_button(buttons: Sequence[int], index: int) -> bool:
    return bool(buttons[index]) if 0 <= index < len(buttons) else False


def joystick_axes_to_drive_twist(
    axes: Sequence[float],
    left_stick_y_axis: int,
    right_stick_x_axis: int,
    deadzone: float,
    max_linear_mps: float,
    max_angular_radps: float,
    track_gauge_m: float,
    moving_inner_ratio: float,
) -> tuple[float, float]:
    """Convert Xbox axes with the same signs and turn mix as robot modes."""
    throttle = -apply_deadzone(indexed_axis(axes, left_stick_y_axis), deadzone)
    steer = apply_deadzone(indexed_axis(axes, right_stick_x_axis), deadzone)
    return joystick_to_drive_twist(
        throttle,
        steer,
        max_linear_mps,
        max_angular_radps,
        track_gauge_m,
        moving_inner_ratio,
    )


def manual_drive_enabled(
    joy_is_fresh: bool,
    deadman_held: bool,
    rearm_required: bool,
    emergency_stop_latched: bool,
) -> bool:
    return (
        joy_is_fresh
        and deadman_held
        and not rearm_required
        and not emergency_stop_latched
    )


def select_go2_drive_twist(
    navigation_enabled: bool,
    emergency_stop_latched: bool,
    manual_override_held: bool,
    manual_fresh: bool,
    manual_twist: Twist,
    nav_fresh: bool,
    nav_twist: Twist,
    limits: tuple[float, float],
) -> Twist:
    if emergency_stop_latched:
        return Twist()
    selected = select_autonomous_drive_twist(
        True,
        manual_override_held,
        manual_fresh,
        manual_twist,
        navigation_enabled and nav_fresh,
        nav_twist,
    )
    return limit_planar_twist(selected, limits)


class Go2ManualDriveNode(Node):
    def __init__(self) -> None:
        super().__init__('h753_go2_manual_drive')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_selected')
        self.declare_parameter('navigation_enabled', False)
        self.declare_parameter('nav_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_period_s', 0.05)
        self.declare_parameter('joy_timeout_s', 0.50)
        self.declare_parameter('nav_timeout_s', 0.50)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('max_linear_mps', 0.60)
        self.declare_parameter('max_angular_radps', 2.67)
        self.declare_parameter('track_gauge_m', 0.45)
        self.declare_parameter('moving_inner_ratio', 0.75)
        self.declare_parameter('left_stick_y_axis', 1)
        self.declare_parameter('right_stick_x_axis', 3)
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('emergency_stop_button', 1)
        self.declare_parameter('emergency_reset_button', 0)

        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.navigation_enabled = bool(
            self.get_parameter('navigation_enabled').value
        )
        self.nav_cmd_vel_topic = str(
            self.get_parameter('nav_cmd_vel_topic').value
        )
        self.publish_period_s = float(
            self.get_parameter('publish_period_s').value
        )
        self.joy_timeout_s = float(self.get_parameter('joy_timeout_s').value)
        self.nav_timeout_s = float(self.get_parameter('nav_timeout_s').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.max_linear_mps = float(
            self.get_parameter('max_linear_mps').value
        )
        self.max_angular_radps = float(
            self.get_parameter('max_angular_radps').value
        )
        self.track_gauge_m = float(self.get_parameter('track_gauge_m').value)
        self.moving_inner_ratio = float(
            self.get_parameter('moving_inner_ratio').value
        )
        self.left_stick_y_axis = int(
            self.get_parameter('left_stick_y_axis').value
        )
        self.right_stick_x_axis = int(
            self.get_parameter('right_stick_x_axis').value
        )
        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.emergency_stop_button = int(
            self.get_parameter('emergency_stop_button').value
        )
        self.emergency_reset_button = int(
            self.get_parameter('emergency_reset_button').value
        )

        if self.publish_period_s <= 0.0:
            raise ValueError('publish_period_s must be positive')
        if self.joy_timeout_s <= 0.0:
            raise ValueError('joy_timeout_s must be positive')
        if self.nav_timeout_s <= 0.0:
            raise ValueError('nav_timeout_s must be positive')
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError('deadzone must be in [0.0, 1.0)')
        if self.max_linear_mps <= 0.0 or self.max_angular_radps <= 0.0:
            raise ValueError('drive limits must be positive')
        if self.track_gauge_m <= 0.0:
            raise ValueError('track_gauge_m must be positive')
        if not 0.0 <= self.moving_inner_ratio <= 1.0:
            raise ValueError('moving_inner_ratio must be in [0.0, 1.0]')

        self.requested_linear_mps = 0.0
        self.requested_angular_radps = 0.0
        self.last_joy_at: float | None = None
        self.nav_twist = Twist()
        self.last_nav_at: float | None = None
        self.deadman_held = False
        self.rearm_required = False
        self.emergency_stop_latched = False
        self.previous_buttons: list[int] = []
        self.active_source = 'stop'
        self.joy_timeout_reported = False
        self.cancel_pending = False
        self.last_cancel_attempt_at: float | None = None

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Joy, self.joy_topic, self._joy_callback, 10)
        if self.navigation_enabled:
            self.create_subscription(
                Twist,
                self.nav_cmd_vel_topic,
                self._nav_cmd_vel_callback,
                10,
            )
        self.cancel_nav_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
        )
        self.cancel_nav_through_client = self.create_client(
            CancelGoal,
            '/navigate_through_poses/_action/cancel_goal',
        )
        self.create_timer(self.publish_period_s, self._publish_callback)

        self.get_logger().info(
            f'Go2 drive selector ready: navigation={self.navigation_enabled}; '
            'hold Xbox LB for manual drive; '
            'B=latch stop, release LB then A=clear stop; '
            f'limits=({self.max_linear_mps:.2f} m/s, '
            f'{self.max_angular_radps:.2f} rad/s), '
            f'moving turn ratio={self.moving_inner_ratio:.2f}:1.00'
        )

    def destroy_node(self) -> bool:
        if rclpy.ok():
            self.publisher.publish(Twist())
            self._try_cancel_nav_goals()
        return super().destroy_node()

    def _joy_callback(self, msg: Joy) -> None:
        now = time.monotonic()
        stop_pressed = self._button_edge(msg, self.emergency_stop_button)
        reset_pressed = self._button_edge(msg, self.emergency_reset_button)
        deadman_held = indexed_button(msg.buttons, self.deadman_button)

        if stop_pressed:
            self.emergency_stop_latched = True
            self.cancel_pending = self.navigation_enabled
            self._try_cancel_nav_goals()
            if self.navigation_enabled:
                self.get_logger().error(
                    'Xbox B stop latched and Nav2 goal cancel requested; '
                    'release LB then press A to clear'
                )
            else:
                self.get_logger().error(
                    'Xbox B stop latched; release LB then press A to clear'
                )
        elif reset_pressed:
            if deadman_held:
                self.get_logger().warn(
                    'Stop reset rejected: release Xbox LB first'
                )
            elif self.emergency_stop_latched:
                self.emergency_stop_latched = False
                self.cancel_pending = False
                self.get_logger().info(
                    'Xbox stop latch cleared; send a new goal or hold LB '
                    'to drive'
                )

        if self.rearm_required and not deadman_held:
            self.rearm_required = False
            self.get_logger().info('Xbox deadman re-armed after LB release')

        self.requested_linear_mps, self.requested_angular_radps = (
            joystick_axes_to_drive_twist(
                msg.axes,
                self.left_stick_y_axis,
                self.right_stick_x_axis,
                self.deadzone,
                self.max_linear_mps,
                self.max_angular_radps,
                self.track_gauge_m,
                self.moving_inner_ratio,
            )
        )
        self.deadman_held = deadman_held
        self.last_joy_at = now
        self.previous_buttons = list(msg.buttons)
        self.joy_timeout_reported = False

    def _nav_cmd_vel_callback(self, msg: Twist) -> None:
        self.nav_twist = msg
        self.last_nav_at = time.monotonic()

    def _publish_callback(self) -> None:
        now = time.monotonic()
        joy_is_fresh = (
            self.last_joy_at is not None
            and now - self.last_joy_at <= self.joy_timeout_s
        )

        timed_out_while_held = not joy_is_fresh and self.deadman_held
        if timed_out_while_held:
            self.rearm_required = True
        if self.last_joy_at is not None and not joy_is_fresh:
            if not self.joy_timeout_reported:
                if timed_out_while_held:
                    self.get_logger().warn(
                        'Xbox Joy timeout while LB held; stopped and LB '
                        'release is required before restart'
                    )
                else:
                    self.get_logger().warn(
                        'Xbox Joy timeout; publishing stop'
                    )
                self.joy_timeout_reported = True

        manual_enabled = manual_drive_enabled(
            joy_is_fresh,
            self.deadman_held,
            self.rearm_required,
            self.emergency_stop_latched,
        )
        nav_is_fresh = (
            self.navigation_enabled
            and self.last_nav_at is not None
            and now - self.last_nav_at <= self.nav_timeout_s
        )

        joy_twist = Twist()
        joy_twist.linear.x = self.requested_linear_mps
        joy_twist.angular.z = self.requested_angular_radps

        twist = select_go2_drive_twist(
            self.navigation_enabled,
            self.emergency_stop_latched,
            self.deadman_held,
            manual_enabled,
            joy_twist,
            nav_is_fresh,
            self.nav_twist,
            (self.max_linear_mps, self.max_angular_radps),
        )

        source = 'stop'
        if not self.emergency_stop_latched:
            if self.deadman_held and manual_enabled:
                source = 'manual'
            elif not self.deadman_held and nav_is_fresh:
                source = 'navigation'
        self._report_source_transition(source)

        if self.cancel_pending:
            if (
                self.last_cancel_attempt_at is None
                or now - self.last_cancel_attempt_at >= 0.50
            ):
                self._try_cancel_nav_goals()
        self.publisher.publish(twist)

    def _report_source_transition(self, source: str) -> None:
        if source == self.active_source:
            return
        self.active_source = source
        if source == 'manual':
            self.get_logger().warn(
                'Xbox LB manual override engaged; joystick has drive priority'
            )
        elif source == 'navigation':
            self.get_logger().info('Fresh Nav2 command selected')
        else:
            self.get_logger().info('Drive source stopped; publishing zero')

    def _try_cancel_nav_goals(self) -> None:
        if not self.navigation_enabled:
            return
        self.last_cancel_attempt_at = time.monotonic()
        request = CancelGoal.Request()
        sent = False
        for client in (
            self.cancel_nav_client,
            self.cancel_nav_through_client,
        ):
            if client.service_is_ready():
                client.call_async(request)
                sent = True
        if sent:
            self.cancel_pending = False
            self.get_logger().warn('Active Nav2 goal cancellation sent')

    def _button_edge(self, msg: Joy, index: int) -> bool:
        pressed = indexed_button(msg.buttons, index)
        previously_pressed = indexed_button(self.previous_buttons, index)
        return pressed and not previously_pressed


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2ManualDriveNode()
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
