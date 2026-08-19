import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
BT_DIR = Path(__file__).resolve().parents[1] / 'behavior_trees'
LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
WORKSPACE_DIR = Path(__file__).resolve().parents[3]
GO2_MAP_DIR = WORKSPACE_DIR / 'maps' / 'go2'
TMINI_CONFIG = (
    WORKSPACE_DIR / 'src' / 'ydlidar_ros2_driver' / 'params' / 'Tmini.yaml'
)
REALSENSE_CONFIG_DIR = WORKSPACE_DIR / 'src' / 'h753_can_odom' / 'config'


def load_config(name: str) -> dict:
    with (CONFIG_DIR / name).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_nav2_uses_rotation_shim_before_dwb() -> None:
    config = load_config('h753_nav2.yaml')
    follow_path = config['controller_server']['ros__parameters']['FollowPath']

    assert follow_path['plugin'] == (
        'nav2_rotation_shim_controller::RotationShimController'
    )
    assert follow_path['primary_controller'] == 'dwb_core::DWBLocalPlanner'
    assert follow_path['angular_dist_threshold'] == 0.35
    assert follow_path['angular_disengage_threshold'] == 0.29
    assert follow_path['rotate_to_heading_angular_vel'] == 2.67
    assert follow_path['max_angular_accel'] == 3.00
    assert follow_path['rotate_to_goal_heading'] is True
    assert follow_path['min_vel_x'] == 0.0


def test_nav2_progress_checker_counts_in_place_rotation() -> None:
    config = load_config('h753_nav2.yaml')
    params = config['controller_server']['ros__parameters']
    progress_checker = params['progress_checker']

    assert progress_checker['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert progress_checker['required_movement_radius'] == 0.15
    assert progress_checker['required_movement_angle'] == 0.20
    assert progress_checker['movement_time_allowance'] == 15.0


def test_nav_to_pose_bt_replans_at_half_hz() -> None:
    bt_path = BT_DIR / 'navigate_to_pose_w_replanning_0_5hz.xml'
    root = ET.parse(bt_path).getroot()
    rate_controller = root.find('.//RateController')

    assert rate_controller is not None
    assert float(rate_controller.attrib['hz']) == 0.5

    config = load_config('h753_nav2.yaml')
    planner_params = config['planner_server']['ros__parameters']
    assert planner_params['expected_planner_frequency'] == 0.5


def test_tmini_scan_frequency_uses_stable_10_hz_profile() -> None:
    config = load_config_from_path(TMINI_CONFIG)
    params = config['ydlidar_ros2_driver_node']['ros__parameters']

    assert params['frequency'] == 10.0
    assert params['sample_rate'] == 4


def test_vlm_gateway_rearm_matches_server_duplicate_suppression_policy() -> None:
    config = load_config('h753_vlm_gateway.yaml')
    params = config['h753_vlm_gateway']['ros__parameters']

    assert params['server_status_topic'] == '/vlm/status'
    assert params['status_topic'] == '/vlm/gateway/status'
    assert params['local_rearm_cooldown_s'] == 15.0
    assert params['local_rearm_clear_s'] == 2.0
    assert params['local_rearm_check_period_s'] == 0.2


def test_realsense_profiles_target_fresh_15_hz_frames() -> None:
    for config_name in (
        'h753_realsense.yaml',
        'h753_realsense_imu.yaml',
        'h753_realsense_rgbd.yaml',
    ):
        config = load_config_from_path(REALSENSE_CONFIG_DIR / config_name)
        params = config['/camera/camera']['ros__parameters']

        assert params['rgb_camera.color_profile'] == '640x480x15'
        assert params['color_qos'] == 'SENSOR_DATA'
        assert params['rgb_camera.frames_queue_size'] == 2

    rgbd = load_config_from_path(
        REALSENSE_CONFIG_DIR / 'h753_realsense_rgbd.yaml'
    )['/camera/camera']['ros__parameters']
    assert rgbd['depth_module.depth_profile'] == '640x480x15'
    assert rgbd['depth_qos'] == 'SENSOR_DATA'
    assert rgbd['depth_module.frames_queue_size'] == 2
    assert rgbd['align_depth.frames_queue_size'] == 2


def test_uart_bridge_uses_field_verified_breakaway_pwm() -> None:
    config = load_config('h753_cmd_vel_uart_bridge.yaml')
    params = config['h753_cmd_vel_uart_bridge']['ros__parameters']

    assert params['min_track_pwm_percent'] == 50.0
    assert params['min_in_place_turn_pwm_percent'] == 60.0
    assert params['in_place_turn_linear_threshold_mps'] == 0.02


def test_nav2_costmaps_use_reduced_30_cm_inflation_radius() -> None:
    config = load_config('h753_nav2.yaml')
    local_params = config['local_costmap']['local_costmap']['ros__parameters']
    global_params = config['global_costmap']['global_costmap']['ros__parameters']

    assert local_params['inflation_layer']['inflation_radius'] == 0.30
    assert global_params['inflation_layer']['inflation_radius'] == 0.30


def test_nav2_translation_is_80_moving_turn_is_95_and_pivot_is_100() -> None:
    config = load_config('h753_nav2.yaml')
    params = config['controller_server']['ros__parameters']
    follow_path = params['FollowPath']
    behavior = config['behavior_server']['ros__parameters']
    smoother = config['velocity_smoother']['ros__parameters']
    uart = load_config('h753_cmd_vel_uart_bridge.yaml')
    uart_params = uart['h753_cmd_vel_uart_bridge']['ros__parameters']
    manager = load_config('h753_robot_mode_manager.yaml')
    manager_params = manager['h753_robot_mode_manager']['ros__parameters']
    go2_drive = load_config('h753_go2_manual_drive.yaml')
    go2_drive_params = go2_drive['h753_go2_manual_drive']['ros__parameters']

    assert follow_path['max_vel_x'] == 0.48
    assert follow_path['max_speed_xy'] == 0.48
    assert follow_path['max_vel_theta'] == 2.54
    assert follow_path['acc_lim_theta'] == 2.80
    assert follow_path['decel_lim_theta'] == -2.80
    assert follow_path['rotate_to_heading_angular_vel'] == 2.67
    assert behavior['max_rotational_vel'] == 2.67
    assert behavior['min_rotational_vel'] == 0.20
    assert behavior['rotational_acc_lim'] == 1.50
    assert smoother['max_velocity'] == [0.48, 0.0, 2.67]
    assert smoother['min_velocity'] == [-0.48, 0.0, -2.67]
    assert smoother['max_accel'] == [0.80, 0.0, 3.00]
    assert smoother['max_decel'] == [-1.00, 0.0, -3.00]
    assert 'autonomous_max_track_speed_mps' not in manager_params
    assert 'autonomous_max_track_speed_mps' not in go2_drive_params
    assert uart_params['max_linear_mps'] == 0.60
    assert uart_params['max_angular_radps'] == 2.67


def test_integrated_collision_lifecycle_manager_starts_after_its_node() -> None:
    for launch_name in (
        'navigation_bringup.launch.py',
        'slam_navigation_bringup.launch.py',
        'inspection_drive_bringup.launch.py',
    ):
        source = (LAUNCH_DIR / launch_name).read_text(encoding='utf-8')
        collision_node_at = source.index("executable='collision_monitor'")
        manager_at = source.index("name='lifecycle_manager_collision_monitor'")
        timer_at = source.rfind('TimerAction(', collision_node_at, manager_at)
        delayed_manager_block = source[timer_at:manager_at]

        assert collision_node_at < timer_at < manager_at
        assert 'period=collision_monitor_start_delay_s' in delayed_manager_block
        assert "'collision_monitor_start_delay_s'" in source
        assert "default_value='1.5'" in source


def test_go2_slow_polygon_preserves_stop_zone() -> None:
    config = load_config('h753_go2_manual_collision_monitor.yaml')
    params = config['collision_monitor']['ros__parameters']
    stop = params['PolygonStop']['points']
    slow = params['PolygonSlow']['points']

    assert stop == [0.43, 0.38, 0.43, -0.38, -0.33, -0.38, -0.33, 0.38]
    assert params['PolygonSlow']['slowdown_ratio'] == 0.80

    stop_x, stop_y = stop[0::2], stop[1::2]
    slow_x, slow_y = slow[0::2], slow[1::2]
    assert min(slow_x) <= min(stop_x)
    assert max(slow_x) >= max(stop_x)
    assert min(slow_y) <= min(stop_y)
    assert max(slow_y) >= max(stop_y)


def test_integrated_mode_collision_policy_matches_mode5_pass_through() -> None:
    config = load_config('h753_collision_monitor_modes.yaml')
    params = config['collision_monitor']['ros__parameters']
    manager_source = (
        Path(__file__).resolve().parents[1]
        / 'h753_can_odom'
        / 'robot_mode_manager_node.py'
    ).read_text(encoding='utf-8')

    assert params['PolygonStop']['enabled'] is False
    assert params['PolygonSlow']['enabled'] is False
    assert "h753_share / 'config' / 'h753_collision_monitor_modes.yaml'" in (
        manager_source
    )


def test_mode_manager_defaults_to_workspace_go2_amcl_map() -> None:
    config = load_config('h753_robot_mode_manager.yaml')
    params = config['h753_robot_mode_manager']['ros__parameters']

    assert params['localization_backend'] == 'amcl'
    assert params['static_map_yaml'].endswith('/maps/go2/go2_map.yaml')


def test_workspace_go2_map_matches_validated_source_asset() -> None:
    map_yaml = load_config_from_path(GO2_MAP_DIR / 'go2_map.yaml')
    image_path = GO2_MAP_DIR / map_yaml['image']

    assert image_path.is_file()
    assert hashlib.sha256(image_path.read_bytes()).hexdigest() == (
        '7ac42b120bc5029425f7fcf9cfb823d868197e34af063afc3b7114acd5a24b04'
    )


def load_config_from_path(path: Path) -> dict:
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)
