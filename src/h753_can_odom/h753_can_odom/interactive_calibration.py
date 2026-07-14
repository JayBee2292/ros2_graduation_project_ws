#!/usr/bin/env python3
from __future__ import annotations

import copy
import math
import shutil
import threading
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def calibrated_track_width(
    current_track_width_m: float,
    measured_odom_yaw_rad: float,
    actual_yaw_rad: float,
) -> float:
    if current_track_width_m <= 0.0:
        raise ValueError('current_track_width_m must be positive')
    if abs(measured_odom_yaw_rad) <= 0.0:
        raise ValueError('measured_odom_yaw_rad must be non-zero')
    if abs(actual_yaw_rad) <= 0.0:
        raise ValueError('actual_yaw_rad must be non-zero')
    return current_track_width_m * abs(measured_odom_yaw_rad) / abs(actual_yaw_rad)


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def stamp_suffix() -> str:
    return time.strftime('%Y%m%d_%H%M%S')


class InteractiveCalibration(Node):
    def __init__(self) -> None:
        super().__init__('h753_interactive_calibration')

        share = Path(get_package_share_directory('h753_can_odom'))
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/camera/camera/imu')
        self.declare_parameter('odom_config_path', str(share / 'config' / 'h753_can_odom.yaml'))
        self.declare_parameter('imu_config_path', str(share / 'config' / 'h753_imu_odom_fusion.yaml'))
        self.declare_parameter('sensor_tf_config_path', str(share / 'config' / 'h753_sensor_tf.yaml'))
        self.declare_parameter('min_straight_distance_m', 0.25)
        self.declare_parameter('min_rotation_rad', 0.35)
        self.declare_parameter('max_imu_dt_s', 0.1)

        self.odom_config_path = Path(self.get_parameter('odom_config_path').value).expanduser()
        self.imu_config_path = Path(self.get_parameter('imu_config_path').value).expanduser()
        self.sensor_tf_config_path = Path(self.get_parameter('sensor_tf_config_path').value).expanduser()
        self.min_straight_distance_m = float(self.get_parameter('min_straight_distance_m').value)
        self.min_rotation_rad = float(self.get_parameter('min_rotation_rad').value)
        self.max_imu_dt_s = float(self.get_parameter('max_imu_dt_s').value)

        self.lock = threading.Lock()
        self.latest_odom: Odometry | None = None
        self.latest_scan: LaserScan | None = None
        self.latest_imu: Imu | None = None
        self.imu_integral = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_imu_stamp_s: float | None = None
        self.odom_yaw_integral = 0.0
        self.last_odom_yaw: float | None = None

        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._odom_callback,
            20,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self._imu_callback,
            qos_profile_sensor_data,
        )

    def _odom_callback(self, msg: Odometry) -> None:
        yaw = yaw_from_odom(msg)
        with self.lock:
            if self.last_odom_yaw is not None:
                self.odom_yaw_integral += normalize_angle(yaw - self.last_odom_yaw)
            self.last_odom_yaw = yaw
            self.latest_odom = copy.deepcopy(msg)

    def _scan_callback(self, msg: LaserScan) -> None:
        with self.lock:
            self.latest_scan = copy.deepcopy(msg)

    def _imu_callback(self, msg: Imu) -> None:
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        with self.lock:
            if self.last_imu_stamp_s is not None:
                dt = stamp_s - self.last_imu_stamp_s
                if 0.0 < dt <= self.max_imu_dt_s:
                    self.imu_integral['x'] += msg.angular_velocity.x * dt
                    self.imu_integral['y'] += msg.angular_velocity.y * dt
                    self.imu_integral['z'] += msg.angular_velocity.z * dt
            self.last_imu_stamp_s = stamp_s
            self.latest_imu = copy.deepcopy(msg)

    def menu_loop(self) -> None:
        while rclpy.ok():
            print()
            print('=== H753 tape calibration ===')
            print('1. 직진 측정 -> odom_linear_sign 자동 저장')
            print('2. 제자리 반시계 회전 측정 -> odom_angular_sign + track_width_m 자동 저장')
            print('3. IMU 반시계 회전 측정 -> imu_yaw_axis/sign 자동 저장')
            print('4. 라이다 앞 물체 측정 -> laser_yaw 자동 저장')
            print('5. 현재 저장값 보기')
            print('q. 종료')
            try:
                choice = input('선택: ').strip().lower()
            except EOFError:
                print()
                return

            if choice == '1':
                self.measure_straight()
            elif choice == '2':
                self.measure_rotation()
            elif choice == '3':
                self.measure_imu_rotation()
            elif choice == '4':
                self.measure_laser_yaw()
            elif choice == '5':
                self.print_saved_values()
            elif choice in {'q', 'quit', 'exit'}:
                return
            else:
                print('알 수 없는 선택입니다.')

    def measure_straight(self) -> None:
        print()
        print('[직진 측정]')
        print('로봇을 테이프 시작선에 놓고, 실제 앞 방향이 테이프 진행 방향을 보게 맞추세요.')
        input('준비되면 Enter를 누르세요. 그 다음 0.5~1m 직진 후 다시 Enter를 누릅니다.')
        start = self.wait_for_odom()
        if start is None:
            print('오류: /odom을 받지 못했습니다.')
            return

        input('직진을 끝냈으면 Enter를 누르세요.')
        end = self.wait_for_odom()
        if end is None:
            print('오류: /odom을 받지 못했습니다.')
            return

        start_yaw = yaw_from_odom(start)
        dx = end.pose.pose.position.x - start.pose.pose.position.x
        dy = end.pose.pose.position.y - start.pose.pose.position.y
        forward_delta = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
        distance = math.hypot(dx, dy)

        print(f'측정 이동거리: {distance:.3f} m, 시작방향 기준 전진량: {forward_delta:.3f} m')
        if abs(forward_delta) < self.min_straight_distance_m:
            print(f'전진량이 너무 작습니다. 최소 {self.min_straight_distance_m:.2f} m 이상 움직여서 다시 측정하세요.')
            return

        measured_sign = 1.0 if forward_delta > 0.0 else -1.0
        current_sign = self.load_odom_sign('odom_linear_sign')
        sign = current_sign * measured_sign
        print(f'현재 odom_linear_sign: {current_sign:.1f}')
        print(f'추천 odom_linear_sign: {sign:.1f}')
        if self.confirm_save():
            self.save_odom_param('odom_linear_sign', sign)
            print('저장 완료. 실행 중인 odom 노드에는 바로 반영되지 않으니 캘리브레이션을 재시작하면 적용됩니다.')

    def measure_rotation(self) -> None:
        print()
        print('[제자리 회전 측정]')
        print('바닥 테이프로 시작 방향과 목표 방향을 표시하세요.')
        print('로봇 중심은 고정하고, 반시계 방향으로 목표 각도만큼 천천히 회전시키세요.')
        actual_rotation_deg = self.prompt_actual_rotation_degrees()
        if actual_rotation_deg is None:
            return
        input('준비되면 Enter를 누르세요. 그 다음 반시계 회전 후 다시 Enter를 누릅니다.')
        start = self.wait_for_odom_yaw_integral()
        if start is None:
            print('오류: /odom을 받지 못했습니다.')
            return

        input('반시계 회전을 끝냈으면 Enter를 누르세요.')
        end = self.wait_for_odom_yaw_integral()
        if end is None:
            print('오류: /odom을 받지 못했습니다.')
            return

        delta_yaw = end - start
        print(f'측정 yaw 변화량: {math.degrees(delta_yaw):.1f} deg ({delta_yaw:.3f} rad)')
        if abs(delta_yaw) < self.min_rotation_rad:
            print(f'회전량이 너무 작습니다. 최소 {math.degrees(self.min_rotation_rad):.0f}도 이상 회전해서 다시 측정하세요.')
            return

        measured_sign = 1.0 if delta_yaw > 0.0 else -1.0
        current_sign = self.load_odom_sign('odom_angular_sign')
        sign = current_sign * measured_sign
        current_track_width_m = self.load_odom_float('track_width_m', 0.45)
        track_width_m = calibrated_track_width(
            current_track_width_m,
            delta_yaw,
            math.radians(actual_rotation_deg),
        )
        print(f'현재 odom_angular_sign: {current_sign:.1f}')
        print(f'추천 odom_angular_sign: {sign:.1f}')
        print(f'실제 반시계 회전량: {actual_rotation_deg:.1f} deg')
        print(f'현재 track_width_m: {current_track_width_m:.4f} m')
        print(f'추천 track_width_m: {track_width_m:.4f} m')
        print('스키드 스티어 슬립을 포함한 ROS 오도메트리용 유효 차폭입니다.')
        if not 0.20 <= track_width_m <= 2.00:
            print('추천 차폭이 허용 범위 0.20~2.00 m 밖입니다. 테이프 각도와 회전 방향을 확인하고 다시 측정하세요.')
            return
        if self.confirm_save():
            self.save_odom_params({
                'odom_angular_sign': sign,
                'track_width_m': track_width_m,
            })
            print('저장 완료. 실행 중인 odom 노드에는 바로 반영되지 않으니 캘리브레이션을 재시작하면 적용됩니다.')

    def measure_laser_yaw(self) -> None:
        print()
        print('[라이다 방향 측정]')
        print('로봇 정면 0.5~1m 지점에 박스 같은 물체를 두세요.')
        print('가까운 다른 물체가 있으면 오검출될 수 있으니 주변을 비워두는 게 좋습니다.')
        input('준비되면 Enter를 누르세요.')
        scan = self.wait_for_scan()
        if scan is None:
            print('오류: /scan을 받지 못했습니다.')
            return

        result = self.find_front_object_angle(scan)
        if result is None:
            print('유효한 /scan 거리값을 찾지 못했습니다.')
            return

        angle, distance = result
        laser_yaw = normalize_angle(-angle)
        print(f'가장 가까운 물체: 거리 {distance:.3f} m, scan 각도 {math.degrees(angle):.1f} deg')
        print(f'추천 laser_yaw: {laser_yaw:.6f} rad ({math.degrees(laser_yaw):.1f} deg)')
        print('물체를 로봇 정면에 뒀다는 전제에서 계산한 값입니다.')
        if self.confirm_save():
            self.save_sensor_tf_param('laser_yaw', laser_yaw)
            print('저장 완료. static TF는 재시작해야 적용됩니다.')

    def measure_imu_rotation(self) -> None:
        print()
        print('[IMU 제자리 회전 측정]')
        print('로봇을 제자리에 두고, 반시계 방향으로 45~90도 정도 회전시키세요.')
        print('D435i IMU의 yaw 축과 부호를 자동으로 선택합니다.')
        input('준비되면 Enter를 누르세요. 그 다음 반시계 회전 후 다시 Enter를 누릅니다.')
        start = self.wait_for_imu_integral()
        if start is None:
            print('오류: /camera/camera/imu를 받지 못했습니다.')
            print('카메라가 연결됐는지 확인하고 launch_camera:=true enable_imu:=true로 다시 실행하세요.')
            return

        input('반시계 회전을 끝냈으면 Enter를 누르세요.')
        end = self.wait_for_imu_integral()
        if end is None:
            print('오류: /camera/camera/imu를 받지 못했습니다.')
            return

        delta = {axis: end[axis] - start[axis] for axis in ('x', 'y', 'z')}
        axis = max(delta, key=lambda candidate: abs(delta[candidate]))
        angle = delta[axis]
        print(
            'IMU 적분 회전량: '
            f"x={math.degrees(delta['x']):.1f} deg, "
            f"y={math.degrees(delta['y']):.1f} deg, "
            f"z={math.degrees(delta['z']):.1f} deg"
        )
        if abs(angle) < self.min_rotation_rad:
            print(f'회전량이 너무 작습니다. 최소 {math.degrees(self.min_rotation_rad):.0f}도 이상 회전해서 다시 측정하세요.')
            return

        sign = 1.0 if angle > 0.0 else -1.0
        print(f'추천 imu_yaw_axis: {axis}')
        print(f'추천 imu_yaw_sign: {sign:.1f}')
        if self.confirm_save():
            self.save_imu_param('imu_yaw_axis', axis)
            self.save_imu_param('imu_yaw_sign', sign)
            print('저장 완료. 통합 SLAM/Nav2 런타임을 재시작하면 적용됩니다.')

    def wait_for_odom(self, timeout_s: float = 5.0) -> Odometry | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            with self.lock:
                if self.latest_odom is not None:
                    return copy.deepcopy(self.latest_odom)
            time.sleep(0.05)
        return None

    def wait_for_odom_yaw_integral(self, timeout_s: float = 5.0) -> float | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            with self.lock:
                if self.latest_odom is not None:
                    return float(self.odom_yaw_integral)
            time.sleep(0.05)
        return None

    def wait_for_scan(self, timeout_s: float = 5.0) -> LaserScan | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            with self.lock:
                if self.latest_scan is not None:
                    return copy.deepcopy(self.latest_scan)
            time.sleep(0.05)
        return None

    def wait_for_imu_integral(self, timeout_s: float = 5.0) -> dict[str, float] | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            with self.lock:
                if self.latest_imu is not None:
                    return dict(self.imu_integral)
            time.sleep(0.05)
        return None

    @staticmethod
    def find_front_object_angle(scan: LaserScan) -> tuple[float, float] | None:
        candidates: list[tuple[float, float]] = []
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value <= max(scan.range_min, 0.10) or value >= scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            candidates.append((value, angle))

        if not candidates:
            return None

        min_range = min(distance for distance, _ in candidates)
        cluster = [
            (distance, angle)
            for distance, angle in candidates
            if abs(distance - min_range) <= 0.15
        ]
        if not cluster:
            return None

        weight_sum = 0.0
        sin_sum = 0.0
        cos_sum = 0.0
        for distance, angle in cluster:
            weight = 1.0 / max(distance, 0.05)
            weight_sum += weight
            sin_sum += weight * math.sin(angle)
            cos_sum += weight * math.cos(angle)

        return math.atan2(sin_sum / weight_sum, cos_sum / weight_sum), min_range

    @staticmethod
    def confirm_save() -> bool:
        answer = input('이 값을 저장할까요? [y/N]: ').strip().lower()
        return answer in {'y', 'yes', '1', 's', 'save'}

    @staticmethod
    def prompt_actual_rotation_degrees() -> float | None:
        raw = input('테이프로 표시한 실제 반시계 회전각 [기본 90 deg]: ').strip()
        try:
            value = float(raw) if raw else 90.0
        except ValueError:
            print('회전각은 숫자로 입력하세요.')
            return None
        if not 20.0 <= value <= 180.0:
            print('회전각은 20~180 deg 범위로 입력하세요.')
            return None
        return value

    def print_saved_values(self) -> None:
        odom_data = self.load_yaml(self.odom_config_path)
        odom_params = odom_data.get('h753_can_odom', {}).get('ros__parameters', {})
        imu_data = self.load_yaml(self.imu_config_path)
        imu_params = imu_data.get('h753_imu_odom_fusion', {}).get('ros__parameters', {})
        sensor_data = self.load_yaml(self.sensor_tf_config_path)
        sensor_params = sensor_data.get('sensor_tf', {}).get('ros__parameters', {})
        print()
        print('[현재 저장값]')
        print(f'odom config: {self.odom_config_path}')
        print(f"  odom_linear_sign: {odom_params.get('odom_linear_sign')}")
        print(f"  odom_angular_sign: {odom_params.get('odom_angular_sign')}")
        print(f"  track_width_m: {odom_params.get('track_width_m')}")
        print(f'imu config: {self.imu_config_path}')
        print(f"  imu_yaw_axis: {imu_params.get('imu_yaw_axis')}")
        print(f"  imu_yaw_sign: {imu_params.get('imu_yaw_sign')}")
        print(f'sensor tf config: {self.sensor_tf_config_path}')
        print(f"  laser_yaw: {sensor_params.get('laser_yaw')}")
        print(f"  laser_x/y/z: {sensor_params.get('laser_x')}, {sensor_params.get('laser_y')}, {sensor_params.get('laser_z')}")

    def save_odom_param(self, key: str, value: float) -> None:
        self.save_odom_params({key: value})

    def save_odom_params(self, values: dict[str, float]) -> None:
        data = self.load_yaml(self.odom_config_path)
        params = data.setdefault('h753_can_odom', {}).setdefault('ros__parameters', {})
        for key, value in values.items():
            params[key] = float(value)
        self.write_yaml(self.odom_config_path, data)

    def load_odom_float(self, key: str, default: float) -> float:
        data = self.load_yaml(self.odom_config_path)
        params = data.get('h753_can_odom', {}).get('ros__parameters', {})
        return float(params.get(key, default))

    def load_odom_sign(self, key: str) -> float:
        data = self.load_yaml(self.odom_config_path)
        params = data.get('h753_can_odom', {}).get('ros__parameters', {})
        value = float(params.get(key, 1.0))
        return -1.0 if value < 0.0 else 1.0

    def save_sensor_tf_param(self, key: str, value: float) -> None:
        data = self.load_yaml(self.sensor_tf_config_path)
        params = data.setdefault('sensor_tf', {}).setdefault('ros__parameters', {})
        params[key] = float(value)
        self.write_yaml(self.sensor_tf_config_path, data)

    def save_imu_param(self, key: str, value) -> None:
        data = self.load_yaml(self.imu_config_path)
        params = data.setdefault('h753_imu_odom_fusion', {}).setdefault('ros__parameters', {})
        params[key] = value
        self.write_yaml(self.imu_config_path, data)

    @staticmethod
    def load_yaml(path: Path) -> dict:
        if not path.exists():
            return {}
        with path.open('r', encoding='utf-8') as file:
            return yaml.safe_load(file) or {}

    @staticmethod
    def write_yaml(path: Path, data: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            backup = path.with_suffix(path.suffix + f'.bak_{stamp_suffix()}')
            shutil.copy2(path, backup)
        with path.open('w', encoding='utf-8') as file:
            yaml.safe_dump(data, file, sort_keys=False, allow_unicode=False)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InteractiveCalibration()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.menu_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
