#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import select
import signal
import subprocess
import sys
import termios
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

import rclpy
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, UInt8


MODE_STOP = 0
MODE_MANUAL_MAPPING = 1
MODE_AUTO_MAPPING = 2
MODE_MANUAL_LOCALIZATION = 3
MODE_GOAL_NAVIGATION = 4
MODE_INSPECTION_DRIVE = 5
VALID_MODES = tuple(range(6))

MODE_NAMES = {
    MODE_STOP: 'STOP',
    MODE_MANUAL_MAPPING: 'MANUAL_MAPPING',
    MODE_AUTO_MAPPING: 'AUTO_MAPPING',
    MODE_MANUAL_LOCALIZATION: 'MANUAL_LOCALIZATION',
    MODE_GOAL_NAVIGATION: 'GOAL_NAVIGATION',
    MODE_INSPECTION_DRIVE: 'INSPECTION_DRIVE',
}

MODE_DESCRIPTIONS = {
    MODE_STOP: '정지: 현재 런타임을 유지하고 모터 명령을 차단합니다.',
    MODE_MANUAL_MAPPING: '수동 매핑: Xbox/키보드로 주행하며 실시간 지도를 만듭니다.',
    MODE_AUTO_MAPPING: '자동 매핑: frontier 목표를 따라 이동하며 실시간 지도를 만듭니다.',
    MODE_MANUAL_LOCALIZATION: '수동 위치 확인: 저장 맵에서 위치를 추정하며 수동 주행합니다.',
    MODE_GOAL_NAVIGATION: '목적지 자율주행: 저장 맵에서 RViz Nav2 Goal로 이동합니다.',
    MODE_INSPECTION_DRIVE: '점검 주행: SLAM 없이 수동 주행으로 방향을 점검합니다.',
}

TOPOLOGY_MAPPING = 'mapping'
TOPOLOGY_LOCALIZATION = 'localization'
TOPOLOGY_INSPECTION = 'inspection'

MODE_TOPOLOGY = {
    MODE_MANUAL_MAPPING: TOPOLOGY_MAPPING,
    MODE_AUTO_MAPPING: TOPOLOGY_MAPPING,
    MODE_MANUAL_LOCALIZATION: TOPOLOGY_LOCALIZATION,
    MODE_GOAL_NAVIGATION: TOPOLOGY_LOCALIZATION,
    MODE_INSPECTION_DRIVE: TOPOLOGY_INSPECTION,
}

CAN_DEVICE_HINTS = ('canable', 'openlight', 'normaldotcom')
UART_DEVICE_HINTS = ('stmicroelectronics', 'stlink')


@dataclass
class RuntimeProcess:
    topology: str
    process: subprocess.Popen
    log_stream: TextIO
    log_path: Path


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) < deadzone else value


def joystick_to_drive_twist(
    throttle: float,
    steer: float,
    max_linear_mps: float,
    max_angular_radps: float,
    track_gauge_m: float,
    moving_inner_ratio: float,
) -> tuple[float, float]:
    throttle = clamp(throttle, -1.0, 1.0)
    steer = clamp(steer, -1.0, 1.0)
    if throttle == 0.0:
        return 0.0, -steer * max_angular_radps

    outer_mps = throttle * max_linear_mps
    inner_scale = 1.0 - ((1.0 - clamp(moving_inner_ratio, 0.0, 1.0)) * abs(steer))
    inner_mps = outer_mps * inner_scale
    outer_is_left = (steer > 0.0) == (outer_mps > 0.0)
    left_mps, right_mps = (
        (outer_mps, inner_mps) if outer_is_left else (inner_mps, outer_mps)
    )
    linear_mps = (left_mps + right_mps) / 2.0
    angular_radps = (
        (right_mps - left_mps) / track_gauge_m if track_gauge_m > 0.0 else 0.0
    )
    return linear_mps, angular_radps


def bool_text(value: bool) -> str:
    return 'true' if value else 'false'


class RobotModeManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('h753_robot_mode_manager')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('nav_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('selected_cmd_vel_topic', '/cmd_vel_selected')
        self.declare_parameter('mode_request_topic', '/robot_mode/request')
        self.declare_parameter('mode_topic', '/robot_mode')
        self.declare_parameter('frontier_enabled_topic', '/frontier_explorer/enabled')
        self.declare_parameter(
            'posegraph_file',
            '/home/jyl1015/ros2_graduation_project_ws/maps/h753_map',
        )
        self.declare_parameter('runtime_log_directory', '~/.ros/log/h753_robot_modes')
        self.declare_parameter('launch_lidar', True)
        self.declare_parameter('launch_odom', True)
        self.declare_parameter('launch_uart_bridge', True)
        self.declare_parameter('launch_rviz', True)
        self.declare_parameter('launch_camera', True)
        self.declare_parameter('enable_imu', False)
        self.declare_parameter('launch_imu_odom', False)
        self.declare_parameter('launch_map_odom', False)
        self.declare_parameter('lidar_device_path', '/dev/ttyUSB0')
        self.declare_parameter(
            'can_device_path',
            '/dev/serial/by-id/'
            'usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_'
            'canable2.git_209F33833630-if00',
        )
        self.declare_parameter(
            'uart_device_path',
            '/dev/serial/by-id/'
            'usb-STMicroelectronics_STLINK-V3_0036002C3235511837333439-if02',
        )
        self.declare_parameter('collision_monitor_node', '/collision_monitor')
        self.declare_parameter('joy_timeout_s', 0.50)
        self.declare_parameter('joy_connection_warning_s', 3.0)
        self.declare_parameter('nav_timeout_s', 0.50)
        self.declare_parameter('keyboard_hold_s', 0.50)
        self.declare_parameter('publish_period_s', 0.05)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('manual_max_linear_mps', 0.60)
        self.declare_parameter('manual_max_angular_radps', 2.67)
        self.declare_parameter('inspection_max_linear_mps', 0.60)
        self.declare_parameter('inspection_max_angular_radps', 2.67)
        self.declare_parameter('track_gauge_m', 0.45)
        self.declare_parameter('moving_inner_ratio', 0.40)
        self.declare_parameter('left_stick_y_axis', 1)
        self.declare_parameter('right_stick_x_axis', 3)
        self.declare_parameter('dpad_y_axis', 7)
        self.declare_parameter('dpad_up_value', 1.0)
        self.declare_parameter('button_a', 0)
        self.declare_parameter('button_b', 1)
        self.declare_parameter('button_menu', 7)
        self.declare_parameter('menu_hold_stop_s', 2.0)

        self.posegraph_file = Path(str(self.get_parameter('posegraph_file').value)).expanduser()
        self.runtime_log_directory = Path(
            str(self.get_parameter('runtime_log_directory').value)
        ).expanduser()
        self.runtime_log_directory.mkdir(parents=True, exist_ok=True)
        self.launch_lidar = bool(self.get_parameter('launch_lidar').value)
        self.launch_odom = bool(self.get_parameter('launch_odom').value)
        self.launch_uart_bridge = bool(self.get_parameter('launch_uart_bridge').value)
        self.launch_rviz = bool(self.get_parameter('launch_rviz').value)
        self.launch_camera = bool(self.get_parameter('launch_camera').value)
        self.enable_imu = bool(self.get_parameter('enable_imu').value)
        self.launch_imu_odom = bool(self.get_parameter('launch_imu_odom').value)
        self.launch_map_odom = bool(self.get_parameter('launch_map_odom').value)
        self.lidar_device_path = Path(str(self.get_parameter('lidar_device_path').value))
        self.can_device_path = Path(str(self.get_parameter('can_device_path').value))
        self.uart_device_path = Path(str(self.get_parameter('uart_device_path').value))
        self.collision_monitor_node = str(
            self.get_parameter('collision_monitor_node').value
        ).rstrip('/')
        self.joy_timeout_s = float(self.get_parameter('joy_timeout_s').value)
        self.joy_connection_warning_s = float(
            self.get_parameter('joy_connection_warning_s').value
        )
        self.nav_timeout_s = float(self.get_parameter('nav_timeout_s').value)
        self.keyboard_hold_s = float(self.get_parameter('keyboard_hold_s').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.manual_max_linear_mps = float(
            self.get_parameter('manual_max_linear_mps').value
        )
        self.manual_max_angular_radps = float(
            self.get_parameter('manual_max_angular_radps').value
        )
        self.inspection_max_linear_mps = float(
            self.get_parameter('inspection_max_linear_mps').value
        )
        self.inspection_max_angular_radps = float(
            self.get_parameter('inspection_max_angular_radps').value
        )
        self.track_gauge_m = float(self.get_parameter('track_gauge_m').value)
        self.moving_inner_ratio = float(self.get_parameter('moving_inner_ratio').value)
        self.left_stick_y_axis = int(self.get_parameter('left_stick_y_axis').value)
        self.right_stick_x_axis = int(self.get_parameter('right_stick_x_axis').value)
        self.dpad_y_axis = int(self.get_parameter('dpad_y_axis').value)
        self.dpad_up_value = float(self.get_parameter('dpad_up_value').value)
        self.button_a = int(self.get_parameter('button_a').value)
        self.button_b = int(self.get_parameter('button_b').value)
        self.button_menu = int(self.get_parameter('button_menu').value)
        self.menu_hold_stop_s = float(self.get_parameter('menu_hold_stop_s').value)

        self.mode = MODE_STOP
        self.menu_open = False
        self.menu_selection = MODE_STOP
        self.runtime: RuntimeProcess | None = None
        self.frontier_runtime: RuntimeProcess | None = None
        self.joy_twist = Twist()
        self.nav_twist = Twist()
        self.keyboard_twist = Twist()
        self.last_joy_at: float | None = None
        self.started_at = time.monotonic()
        self.joy_missing_warned = False
        self.last_nav_at: float | None = None
        self.last_keyboard_at: float | None = None
        self.previous_buttons: list[int] = []
        self.previous_dpad_y = 0.0
        self.menu_pressed_at: float | None = None
        self.menu_hold_stop_fired = False
        self.stdin_attributes = None
        self.collision_safety_applied: bool | None = None
        self.collision_state_request_pending = False
        self.collision_safety_request_pending = False
        self.collision_safety_request_id = 0
        self.last_collision_safety_warning_at: float | None = None

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        self.selected_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('selected_cmd_vel_topic').value),
            10,
        )
        self.mode_pub = self.create_publisher(
            UInt8,
            str(self.get_parameter('mode_topic').value),
            latched_qos,
        )
        self.frontier_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('frontier_enabled_topic').value),
            latched_qos,
        )
        self.create_subscription(
            Joy,
            str(self.get_parameter('joy_topic').value),
            self._joy_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('nav_cmd_vel_topic').value),
            self._nav_cmd_vel_callback,
            10,
        )
        self.create_subscription(
            UInt8,
            str(self.get_parameter('mode_request_topic').value),
            self._mode_request_callback,
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
        self.collision_parameter_client = self.create_client(
            SetParameters,
            f'{self.collision_monitor_node}/set_parameters',
        )
        self.collision_state_client = self.create_client(
            GetState,
            f'{self.collision_monitor_node}/get_state',
        )
        self.create_timer(
            float(self.get_parameter('publish_period_s').value),
            self._publish_timer_callback,
        )
        self.create_timer(0.05, self._keyboard_timer_callback)
        self.create_timer(0.50, self._runtime_monitor_callback)
        self.create_timer(0.50, self._sync_collision_safety)
        self.create_timer(1.0, self._joy_monitor_callback)

        self._enable_terminal_input()
        self._publish_mode()
        self._publish_frontier_enabled()
        self._print_help()

    def destroy_node(self) -> bool:
        try:
            self._publish_stop()
            self._publish_frontier_enabled(False)
            self._cancel_nav_goals()
        finally:
            self._stop_runtime()
            self._restore_terminal_input()
        try:
            return super().destroy_node()
        except Exception:  # ROS context may already be closed by SIGINT.
            return False

    def _joy_callback(self, msg: Joy) -> None:
        now = time.monotonic()
        self.last_joy_at = now
        if self.joy_missing_warned:
            self.joy_missing_warned = False
            self.get_logger().info('Xbox /joy input received')
        self._handle_joy_buttons(msg, now)
        self.joy_twist = self._joy_to_twist(msg)

    def _joy_monitor_callback(self) -> None:
        now = time.monotonic()
        reference = self.started_at if self.last_joy_at is None else self.last_joy_at
        if now - reference < self.joy_connection_warning_s:
            return
        if self.joy_missing_warned:
            return
        self.joy_missing_warned = True
        self.get_logger().warn(
            'No /joy input received. Xbox dongle presence alone is not enough: '
            'power on and pair the controller, then check `ros2 run joy '
            'joy_enumerate_devices`.'
        )

    def _nav_cmd_vel_callback(self, msg: Twist) -> None:
        self.nav_twist = msg
        self.last_nav_at = time.monotonic()

    def _mode_request_callback(self, msg: UInt8) -> None:
        self.request_mode(int(msg.data), source='ROS topic')

    def request_mode(self, requested_mode: int, source: str) -> None:
        if requested_mode not in VALID_MODES:
            self.get_logger().warn(f'Ignored unsupported mode {requested_mode}')
            return

        self._publish_stop()
        self._publish_frontier_enabled(False)
        self._cancel_nav_goals()
        if requested_mode != MODE_AUTO_MAPPING:
            self._stop_frontier_explorer()

        if requested_mode == MODE_STOP:
            self.mode = MODE_STOP
            self.menu_selection = MODE_STOP
            self.menu_open = False
            self._publish_mode()
            self._sync_collision_safety()
            self.get_logger().warn(f'Mode 0 STOP selected from {source}')
            return

        if requested_mode in (MODE_MANUAL_LOCALIZATION, MODE_GOAL_NAVIGATION):
            if not self._posegraph_exists():
                self.mode = MODE_STOP
                self.menu_selection = MODE_STOP
                self._publish_mode()
                self.get_logger().error(
                    f'Mode {requested_mode} requires saved map files: '
                    f'{self.posegraph_file}.posegraph and {self.posegraph_file}.data'
                )
                return

        requested_topology = MODE_TOPOLOGY[requested_mode]
        if self.runtime is None or self.runtime.topology != requested_topology:
            self._stop_runtime()
            if not self._start_runtime(requested_topology):
                self.mode = MODE_STOP
                self.menu_selection = MODE_STOP
                self._publish_mode()
                return

        if requested_mode == MODE_AUTO_MAPPING and not self._start_frontier_explorer():
            self.mode = MODE_STOP
            self.menu_selection = MODE_STOP
            self._publish_mode()
            return

        self.mode = requested_mode
        self.menu_selection = requested_mode
        self.menu_open = False
        self._publish_mode()
        self._publish_frontier_enabled()
        self._sync_collision_safety()
        self.get_logger().info(
            f'Mode {requested_mode} {MODE_NAMES[requested_mode]} selected from {source}'
        )

    def _publish_timer_callback(self) -> None:
        if self.menu_pressed_at is not None and not self.menu_hold_stop_fired:
            if time.monotonic() - self.menu_pressed_at >= self.menu_hold_stop_s:
                self.menu_hold_stop_fired = True
                self.request_mode(MODE_STOP, source='Xbox Menu long hold')

        try:
            self.selected_pub.publish(self._selected_twist())
        except Exception:  # ROS context may already be closed during shutdown.
            return
        self._publish_frontier_enabled()

    def _selected_twist(self) -> Twist:
        if self.menu_open:
            return Twist()
        if self.mode in (
            MODE_MANUAL_MAPPING,
            MODE_MANUAL_LOCALIZATION,
            MODE_INSPECTION_DRIVE,
        ):
            if self._is_fresh(self.last_keyboard_at, self.keyboard_hold_s):
                return self.keyboard_twist
            if self._is_fresh(self.last_joy_at, self.joy_timeout_s):
                return self.joy_twist
            return Twist()
        if self.mode in (MODE_AUTO_MAPPING, MODE_GOAL_NAVIGATION):
            if self.collision_safety_applied is not True:
                return Twist()
            if self._is_fresh(self.last_nav_at, self.nav_timeout_s):
                # Nav2 /cmd_vel is negated here because the UART bridge applies
                # linear_command_sign / angular_command_sign = -1.0 to correct
                # the physical motor polarity. Manual joystick achieves the same
                # sign by negating the stick Y axis in _joy_to_twist. Autonomous
                # Nav2 output has no such built-in negation, so we apply it here.
                inverted_twist = Twist()
                inverted_twist.linear.x = -self.nav_twist.linear.x
                inverted_twist.angular.z = -self.nav_twist.angular.z
                return inverted_twist
            return Twist()
        return Twist()

    def _joy_to_twist(self, msg: Joy) -> Twist:
        max_linear, max_angular = self._manual_limits()
        throttle = -apply_deadzone(
            self._axis(msg, self.left_stick_y_axis),
            self.deadzone,
        )
        steer = apply_deadzone(
            self._axis(msg, self.right_stick_x_axis),
            self.deadzone,
        )
        linear_mps, angular_radps = joystick_to_drive_twist(
            throttle,
            steer,
            max_linear,
            max_angular,
            self.track_gauge_m,
            self.moving_inner_ratio,
        )
        twist = Twist()
        twist.linear.x = linear_mps
        twist.angular.z = angular_radps
        return twist

    def _handle_joy_buttons(self, msg: Joy, now: float) -> None:
        menu_pressed = self._button(msg, self.button_menu)
        menu_was_pressed = self._previous_button(self.button_menu)
        if menu_pressed and not menu_was_pressed:
            self.menu_pressed_at = now
            self.menu_hold_stop_fired = False
        elif not menu_pressed and menu_was_pressed:
            if not self.menu_hold_stop_fired:
                self.menu_open = not self.menu_open
                self.menu_selection = self.mode
                self._print_menu()
            self.menu_pressed_at = None
            self.menu_hold_stop_fired = False

        if self._button_edge(msg, self.button_b):
            self.request_mode(MODE_STOP, source='Xbox B')
        if self.menu_open and self._button_edge(msg, self.button_a):
            self.request_mode(self.menu_selection, source='Xbox A')

        dpad_y = self._axis(msg, self.dpad_y_axis)
        if self.menu_open and abs(dpad_y) > 0.5 and abs(self.previous_dpad_y) <= 0.5:
            direction = -1 if dpad_y * self.dpad_up_value > 0.0 else 1
            self._move_menu_selection(direction)
        self.previous_dpad_y = dpad_y
        self.previous_buttons = list(msg.buttons)

    def _keyboard_timer_callback(self) -> None:
        if self.stdin_attributes is None:
            return
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return
        data = os.read(sys.stdin.fileno(), 32).decode('utf-8', errors='ignore')
        while '\x1b[A' in data:
            data = data.replace('\x1b[A', '', 1)
            if self.menu_open:
                self._move_menu_selection(-1)
        while '\x1b[B' in data:
            data = data.replace('\x1b[B', '', 1)
            if self.menu_open:
                self._move_menu_selection(1)
        for key in data:
            self._handle_keyboard_key(key)

    def _handle_keyboard_key(self, key: str) -> None:
        if key in '012345':
            self.request_mode(int(key), source='keyboard')
            return
        if key in ('m', 'M'):
            self.menu_open = not self.menu_open
            self.menu_selection = self.mode
            self._print_menu()
            return
        if self.menu_open:
            if key in ('w', 'W', 'k', 'K'):
                self._move_menu_selection(-1)
            elif key in ('s', 'S', 'j', 'J'):
                self._move_menu_selection(1)
            elif key in ('\r', '\n', 'a', 'A'):
                self.request_mode(self.menu_selection, source='keyboard menu')
            elif key in ('b', 'B', '\x1b'):
                self.request_mode(MODE_STOP, source='keyboard menu cancel')
            return
        if key in ('h', 'H', '?'):
            self._print_help()
            return
        if key in ('q', 'Q'):
            self.request_mode(MODE_STOP, source='keyboard quit')
            self.get_logger().info('Press Ctrl+C to close run_robot_modes.sh')
            return
        self._set_keyboard_drive(key)

    def _set_keyboard_drive(self, key: str) -> None:
        if self.mode not in (
            MODE_MANUAL_MAPPING,
            MODE_MANUAL_LOCALIZATION,
            MODE_INSPECTION_DRIVE,
        ):
            return
        max_linear, max_angular = self._manual_limits()
        linear_mps = 0.0
        angular_radps = 0.0
        if key in ('w', 'W'):
            linear_mps = -max_linear
        elif key in ('s', 'S'):
            linear_mps = 0.70 * max_linear
        elif key in ('a', 'A'):
            angular_radps = -max_angular
        elif key in ('d', 'D'):
            angular_radps = max_angular
        elif key not in ('x', 'X', ' '):
            return
        self.keyboard_twist = Twist()
        self.keyboard_twist.linear.x = linear_mps
        self.keyboard_twist.angular.z = angular_radps
        self.last_keyboard_at = time.monotonic()

    def _start_runtime(self, topology: str) -> bool:
        busy_devices = self._busy_exclusive_devices()
        if busy_devices:
            self.get_logger().error(
                'Cannot start runtime while devices are already in use: '
                + ', '.join(busy_devices)
            )
            return False

        h753_share = Path(get_package_share_directory('h753_can_odom'))
        collision_params = h753_share / 'config' / 'h753_collision_monitor_modes.yaml'
        common_args = [
            f'launch_lidar:={bool_text(self.launch_lidar)}',
            f'launch_odom:={bool_text(self.launch_odom)}',
            f'launch_rviz:={bool_text(self.launch_rviz)}',
            f'enable_imu:={bool_text(self.enable_imu)}',
            f'launch_imu_odom:={bool_text(self.launch_imu_odom)}',
            f'launch_map_odom:={bool_text(self.launch_map_odom)}',
            f'launch_uart_bridge:={bool_text(self.launch_uart_bridge)}',
            f'collision_monitor_params:={collision_params}',
        ]
        # Live SLAM mapping is latency-sensitive: slam_toolbox's TF message
        # filter drops every scan if laser_frame->map TF isn't ready within
        # transform_timeout, and the D435i camera stream adds enough Jetson
        # CPU load to push that lookup past the deadline. Force it off for
        # mapping regardless of the launch_camera param; other topologies
        # keep the configured value.
        camera_on = self.launch_camera and topology != TOPOLOGY_MAPPING
        common_args.append(f'launch_camera:={bool_text(camera_on)}')

        if topology == TOPOLOGY_MAPPING:
            launch_file = 'slam_navigation_bringup.launch.py'
            launch_args = common_args + [
                'launch_frontier_explorer:=false',
                'auto_explore:=false',
            ]
        elif topology == TOPOLOGY_LOCALIZATION:
            launch_file = 'navigation_bringup.launch.py'
            launch_args = common_args + [f'posegraph_file:={self.posegraph_file}']
        elif topology == TOPOLOGY_INSPECTION:
            launch_file = 'inspection_drive_bringup.launch.py'
            launch_args = common_args
        else:
            self.get_logger().error(f'Unknown runtime topology: {topology}')
            return False

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        log_path = self.runtime_log_directory / f'{timestamp}_{topology}.log'
        log_stream = log_path.open('w', encoding='utf-8', buffering=1)
        command = ['ros2', 'launch', 'h753_can_odom', launch_file, *launch_args]
        child_env = os.environ.copy()
        # A CLion snap terminal exports SNAP_COMMON. The system slam_toolbox
        # package interprets that as its own snap runtime and rewrites map paths.
        child_env.pop('SNAP_COMMON', None)
        try:
            process = subprocess.Popen(
                command,
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
                env=child_env,
            )
        except OSError as exc:
            log_stream.close()
            self.get_logger().error(f'Failed to start {topology} runtime: {exc}')
            return False
        self.runtime = RuntimeProcess(topology, process, log_stream, log_path)
        self.get_logger().info(
            f'Started {topology} runtime pid={process.pid}; log={log_path}'
        )
        return True

    def _stop_runtime(self) -> None:
        self._stop_frontier_explorer()
        if self.runtime is None:
            return
        runtime = self.runtime
        self.runtime = None
        self._reset_collision_safety_state()
        if rclpy.ok():
            self.get_logger().info(
                f'Stopping {runtime.topology} runtime pid={runtime.process.pid}'
            )
        self._terminate_process(runtime)

    @staticmethod
    def _terminate_process(runtime: RuntimeProcess) -> None:
        try:
            os.killpg(runtime.process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            runtime.process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(runtime.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                runtime.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(runtime.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                runtime.process.wait(timeout=2.0)
        runtime.log_stream.close()

    def _runtime_monitor_callback(self) -> None:
        if self.frontier_runtime is not None:
            exit_code = self.frontier_runtime.process.poll()
            if exit_code is not None:
                runtime = self.frontier_runtime
                self.frontier_runtime = None
                runtime.log_stream.close()
                if self.mode == MODE_AUTO_MAPPING:
                    self.mode = MODE_STOP
                    self.menu_selection = MODE_STOP
                    self._publish_stop()
                    self._publish_mode()
                    self._publish_frontier_enabled(False)
                    self.get_logger().error(
                        f'Frontier explorer exited with code {exit_code}; '
                        f'forced mode 0 STOP. Check {runtime.log_path}'
                    )
        if self.runtime is None:
            return
        exit_code = self.runtime.process.poll()
        if exit_code is None:
            return
        runtime = self.runtime
        self.runtime = None
        runtime.log_stream.close()
        self.mode = MODE_STOP
        self.menu_selection = MODE_STOP
        self._publish_stop()
        self._publish_mode()
        self._publish_frontier_enabled(False)
        self._reset_collision_safety_state()
        self.get_logger().error(
            f'{runtime.topology} runtime exited with code {exit_code}; '
            f'forced mode 0 STOP. Check {runtime.log_path}'
        )

    def _start_frontier_explorer(self) -> bool:
        if self.frontier_runtime is not None:
            return self.frontier_runtime.process.poll() is None

        h753_share = Path(get_package_share_directory('h753_can_odom'))
        params_file = h753_share / 'config' / 'h753_frontier_explorer.yaml'
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        log_path = self.runtime_log_directory / f'{timestamp}_frontier.log'
        log_stream = log_path.open('w', encoding='utf-8', buffering=1)
        command = [
            'ros2',
            'run',
            'h753_can_odom',
            'frontier_explorer_node',
            '--ros-args',
            '--params-file',
            str(params_file),
            '-p',
            'start_enabled:=true',
        ]
        child_env = os.environ.copy()
        child_env.pop('SNAP_COMMON', None)
        try:
            process = subprocess.Popen(
                command,
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
                env=child_env,
            )
        except OSError as exc:
            log_stream.close()
            self.get_logger().error(f'Failed to start frontier explorer: {exc}')
            return False
        self.frontier_runtime = RuntimeProcess('frontier', process, log_stream, log_path)
        self.get_logger().info(
            f'Started frontier explorer pid={process.pid}; log={log_path}'
        )
        return True

    def _stop_frontier_explorer(self) -> None:
        if self.frontier_runtime is None:
            return
        runtime = self.frontier_runtime
        self.frontier_runtime = None
        if rclpy.ok():
            self.get_logger().info(
                f'Stopping frontier explorer pid={runtime.process.pid}'
            )
        self._terminate_process(runtime)

    def _publish_stop(self) -> None:
        try:
            self.selected_pub.publish(Twist())
        except Exception:  # ROS context may already be closed during shutdown.
            pass

    def _publish_mode(self) -> None:
        message = UInt8()
        message.data = self.mode
        try:
            self.mode_pub.publish(message)
        except Exception:  # ROS context may already be closed during shutdown.
            pass

    def _publish_frontier_enabled(self, enabled: bool | None = None) -> None:
        message = Bool()
        message.data = self.mode == MODE_AUTO_MAPPING if enabled is None else enabled
        try:
            self.frontier_pub.publish(message)
        except Exception:  # ROS context may already be closed during shutdown.
            pass

    def _cancel_nav_goals(self) -> None:
        request = CancelGoal.Request()
        for client in (self.cancel_nav_client, self.cancel_nav_through_client):
            try:
                if client.service_is_ready():
                    client.call_async(request)
            except Exception:  # ROS context may already be closed during shutdown.
                pass

    def _sync_collision_safety(self) -> None:
        if self.runtime is None:
            return
        enabled = self.mode in (MODE_AUTO_MAPPING, MODE_GOAL_NAVIGATION)
        if self.collision_safety_applied == enabled:
            return
        if self.collision_state_request_pending:
            return
        if self.collision_safety_request_pending:
            return
        if not self.collision_state_client.service_is_ready():
            if enabled:
                self._warn_collision_safety_unavailable()
            return

        self.collision_state_request_pending = True
        self.collision_safety_request_id += 1
        request_id = self.collision_safety_request_id
        future = self.collision_state_client.call_async(GetState.Request())
        future.add_done_callback(
            lambda completed: self._collision_state_done_callback(
                completed,
                request_id,
                enabled,
            )
        )

    def _collision_state_done_callback(self, future, request_id: int, enabled: bool) -> None:
        if request_id != self.collision_safety_request_id:
            return
        self.collision_state_request_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to query collision safety state: {exc}')
            return
        if response.current_state.id != State.PRIMARY_STATE_ACTIVE:
            if enabled:
                self._warn_collision_safety_unavailable()
            return
        if self.collision_safety_applied == enabled:
            return
        if not self.collision_parameter_client.service_is_ready():
            if enabled:
                self._warn_collision_safety_unavailable()
            return

        request = SetParameters.Request()
        request.parameters = [
            # PolygonStop disabled: Nav2 costmap + inflation handles forward
            # obstacle avoidance. Re-enable after navigation is verified.
            self._bool_parameter('PolygonStop.enabled', False),
            self._bool_parameter('PolygonSlow.enabled', enabled),
        ]
        self.collision_safety_request_pending = True
        future = self.collision_parameter_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._collision_safety_done_callback(
                completed,
                request_id,
                enabled,
            )
        )

    def _collision_safety_done_callback(self, future, request_id: int, enabled: bool) -> None:
        if request_id != self.collision_safety_request_id:
            return
        self.collision_safety_request_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to update collision safety: {exc}')
            return
        failures = [result.reason for result in response.results if not result.successful]
        if failures:
            self.get_logger().error(
                'Collision safety parameter update rejected: ' + '; '.join(failures)
            )
            return
        self.collision_safety_applied = enabled
        self.get_logger().info(
            f'Collision polygons enabled={enabled} for mode {self.mode}'
        )
        self._sync_collision_safety()

    def _warn_collision_safety_unavailable(self) -> None:
        now = time.monotonic()
        if (
            self.last_collision_safety_warning_at is not None
            and now - self.last_collision_safety_warning_at < 3.0
        ):
            return
        self.last_collision_safety_warning_at = now
        self.get_logger().warn(
            'Autonomous command held at zero until collision_monitor is active '
            'and safety parameters are applied'
        )

    def _reset_collision_safety_state(self) -> None:
        self.collision_safety_request_id += 1
        self.collision_state_request_pending = False
        self.collision_safety_request_pending = False
        self.collision_safety_applied = None
        self.last_collision_safety_warning_at = None

    @staticmethod
    def _bool_parameter(name: str, value: bool) -> ParameterMessage:
        return ParameterMessage(
            name=name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=value,
            ),
        )

    def _busy_exclusive_devices(self) -> list[str]:
        candidates: list[Path] = []
        if self.launch_lidar:
            candidates.append(self.lidar_device_path)
        if self.launch_odom:
            candidates.append(
                self._resolve_serial_device(self.can_device_path, CAN_DEVICE_HINTS)
            )
        if self.launch_uart_bridge:
            candidates.append(
                self._resolve_serial_device(self.uart_device_path, UART_DEVICE_HINTS)
            )

        busy_devices: list[str] = []
        resolved_paths: set[Path] = set()
        for configured_path in candidates:
            if not configured_path.exists():
                continue
            resolved_path = configured_path.resolve()
            if resolved_path in resolved_paths:
                continue
            resolved_paths.add(resolved_path)
            try:
                result = subprocess.run(
                    ['fuser', str(resolved_path)],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=2.0,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.get_logger().warn(
                    f'Unable to check device owner for {configured_path}: {exc}'
                )
                continue
            if result.returncode == 0:
                owners = result.stdout.strip() or result.stderr.strip()
                busy_devices.append(f'{configured_path} [{owners}]')
        return busy_devices

    @staticmethod
    def _resolve_serial_device(configured_path: Path, hints: tuple[str, ...]) -> Path:
        if configured_path.exists():
            return configured_path

        by_id_directory = Path('/dev/serial/by-id')
        if not by_id_directory.is_dir():
            return configured_path

        for candidate in sorted(by_id_directory.iterdir()):
            name = candidate.name.lower()
            if any(hint in name for hint in hints):
                return candidate

        return configured_path

    def _posegraph_exists(self) -> bool:
        return (
            Path(f'{self.posegraph_file}.posegraph').is_file()
            and Path(f'{self.posegraph_file}.data').is_file()
        )

    def _manual_limits(self) -> tuple[float, float]:
        if self.mode == MODE_INSPECTION_DRIVE:
            return self.inspection_max_linear_mps, self.inspection_max_angular_radps
        return self.manual_max_linear_mps, self.manual_max_angular_radps

    @staticmethod
    def _is_fresh(timestamp: float | None, timeout_s: float) -> bool:
        return timestamp is not None and time.monotonic() - timestamp <= timeout_s

    @staticmethod
    def _axis(msg: Joy, index: int) -> float:
        return float(msg.axes[index]) if 0 <= index < len(msg.axes) else 0.0

    @staticmethod
    def _button(msg: Joy, index: int) -> bool:
        return bool(msg.buttons[index]) if 0 <= index < len(msg.buttons) else False

    def _previous_button(self, index: int) -> bool:
        return bool(self.previous_buttons[index]) if 0 <= index < len(self.previous_buttons) else False

    def _button_edge(self, msg: Joy, index: int) -> bool:
        return self._button(msg, index) and not self._previous_button(index)

    def _move_menu_selection(self, direction: int) -> None:
        current_index = VALID_MODES.index(self.menu_selection)
        self.menu_selection = VALID_MODES[(current_index + direction) % len(VALID_MODES)]
        self._print_menu()

    def _enable_terminal_input(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warn(
                'stdin is not a TTY; use Xbox Menu or publish /robot_mode/request'
            )
            return
        self.stdin_attributes = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

    def _restore_terminal_input(self) -> None:
        if self.stdin_attributes is None:
            return
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.stdin_attributes)
        self.stdin_attributes = None

    def _print_menu(self) -> None:
        if not self.menu_open:
            print('\n[Robot mode menu closed]', flush=True)
            return
        print('\n[Robot mode menu] 숫자 또는 Xbox D-pad 위/아래 + A', flush=True)
        for mode in VALID_MODES:
            marker = '>' if mode == self.menu_selection else ' '
            print(f'{marker} {mode}. {MODE_DESCRIPTIONS[mode]}', flush=True)

    def _print_help(self) -> None:
        print('\n[H753 robot mode manager]', flush=True)
        for mode in VALID_MODES:
            print(f'  {mode}. {MODE_DESCRIPTIONS[mode]}', flush=True)
        print('Keyboard: 0-5 mode, m menu, w/s/a/d drive, x or Space stop, h help', flush=True)
        print('Xbox: Menu menu, D-pad up/down select, A apply, B stop, hold Menu 2s stop', flush=True)
        print('Start state: mode 0 STOP', flush=True)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: RobotModeManagerNode | None = None
    try:
        node = RobotModeManagerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
