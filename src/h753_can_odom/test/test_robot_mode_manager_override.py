from geometry_msgs.msg import Twist

from h753_can_odom.robot_mode_manager_node import (
    LOCALIZATION_BACKEND_AMCL,
    LOCALIZATION_BACKEND_SLAM_TOOLBOX,
    MODE_AUTO_MAPPING,
    MODE_DISASTER_MAPPING,
    MODE_GOAL_NAVIGATION,
    MODE_INSPECTION_DRIVE,
    MODE_MANUAL_MAPPING,
    MODE_MANUAL_LOCALIZATION,
    TOPOLOGY_LOCALIZATION,
    drive_limits_for_mode,
    integrated_collision_polygon_enablement,
    joystick_to_drive_twist,
    limit_planar_twist,
    mapping_runtime_flavor_changed,
    normalize_localization_backend,
    occupancy_map_image_path,
    person_detection_transition,
    perception_runtime_flavor_changed,
    localization_allows_autonomous_drive,
    saved_map_available_for_mode,
    select_autonomous_drive_twist,
    should_launch_perception,
    should_resume_saved_map,
    should_reseed_amcl_pose,
    static_map_exists,
)


def make_twist(linear_x, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    return twist


def test_nav_command_is_selected_and_inverted_without_override():
    selected = select_autonomous_drive_twist(
        True,
        False,
        True,
        make_twist(0.4, 0.8),
        True,
        make_twist(0.3, -0.6),
    )

    assert selected.linear.x == -0.3
    assert selected.angular.z == 0.6


def test_lb_override_selects_fresh_joystick_without_canceling_nav():
    selected = select_autonomous_drive_twist(
        True,
        True,
        True,
        make_twist(-0.4, 0.8),
        True,
        make_twist(0.3, -0.6),
    )

    assert selected.linear.x == -0.4
    assert selected.angular.z == 0.8


def test_stale_joystick_while_lb_held_stops_instead_of_resuming_nav():
    selected = select_autonomous_drive_twist(
        True,
        True,
        False,
        make_twist(-0.4, 0.8),
        True,
        make_twist(0.3, -0.6),
    )

    assert selected.linear.x == 0.0
    assert selected.angular.z == 0.0


def test_releasing_lb_resumes_fresh_nav_command():
    selected = select_autonomous_drive_twist(
        True,
        False,
        True,
        make_twist(-0.4, 0.8),
        True,
        make_twist(0.3, -0.6),
    )

    assert selected.linear.x == -0.3
    assert selected.angular.z == 0.6


def test_manual_override_remains_available_while_auto_safety_sync_is_pending():
    selected = select_autonomous_drive_twist(
        False,
        True,
        True,
        make_twist(-0.4, 0.8),
        True,
        make_twist(0.3, -0.6),
    )

    assert selected.linear.x == -0.4
    assert selected.angular.z == 0.8


def test_collision_safety_gate_still_blocks_nav_command():
    selected = select_autonomous_drive_twist(
        False,
        False,
        True,
        make_twist(-0.4, 0.8),
        True,
        make_twist(0.3, -0.6),
    )

    assert selected.linear.x == 0.0
    assert selected.angular.z == 0.0


def test_integrated_mode4_keeps_collision_polygons_disabled_like_mode5():
    assert integrated_collision_polygon_enablement(
        MODE_GOAL_NAVIGATION
    ) == (False, False)
    assert integrated_collision_polygon_enablement(
        MODE_INSPECTION_DRIVE
    ) == (False, False)


def test_mapping_modes_use_restored_normal_speed_policy():
    mapping_modes = (
        MODE_MANUAL_MAPPING,
        MODE_AUTO_MAPPING,
        MODE_DISASTER_MAPPING,
    )
    for mode in mapping_modes:
        assert drive_limits_for_mode(
            mode,
            (0.60, 2.67),
            (0.60, 2.67),
        ) == (0.60, 2.67)


def test_non_mapping_modes_keep_normal_speed_policy():
    normal_modes = (
        MODE_MANUAL_LOCALIZATION,
        MODE_GOAL_NAVIGATION,
        MODE_INSPECTION_DRIVE,
    )
    for mode in normal_modes:
        assert drive_limits_for_mode(
            mode,
            (0.60, 2.67),
            (0.25, 0.35),
        ) == (0.60, 2.67)


def test_planar_limit_scales_twist_without_changing_curvature():
    limited = limit_planar_twist(make_twist(0.50, 0.50), (0.25, 0.35))

    assert limited.linear.x == 0.25
    assert limited.angular.z == 0.25


def test_planar_limit_caps_in_place_rotation():
    limited = limit_planar_twist(make_twist(0.0, -2.67), (0.25, 0.35))

    assert limited.linear.x == 0.0
    assert limited.angular.z == -0.35


def test_full_joystick_steer_uses_75_to_100_track_ratio():
    linear_mps, angular_radps = joystick_to_drive_twist(
        throttle=1.0,
        steer=1.0,
        max_linear_mps=0.60,
        max_angular_radps=2.67,
        track_gauge_m=0.45,
        moving_inner_ratio=0.75,
    )
    left_mps = linear_mps - angular_radps * 0.45 / 2.0
    right_mps = linear_mps + angular_radps * 0.45 / 2.0

    assert abs(left_mps - 0.60) < 1e-9
    assert abs(right_mps - 0.45) < 1e-9


def test_half_joystick_steer_uses_87_5_to_100_track_ratio():
    linear_mps, angular_radps = joystick_to_drive_twist(
        throttle=1.0,
        steer=0.5,
        max_linear_mps=0.60,
        max_angular_radps=2.67,
        track_gauge_m=0.45,
        moving_inner_ratio=0.75,
    )
    left_mps = linear_mps - angular_radps * 0.45 / 2.0
    right_mps = linear_mps + angular_radps * 0.45 / 2.0

    assert abs(left_mps - 0.60) < 1e-9
    assert abs(right_mps - 0.525) < 1e-9


def test_mode_2_always_starts_a_new_map_even_when_posegraph_exists():
    assert not should_resume_saved_map(MODE_AUTO_MAPPING, True)


def test_mode_6_resumes_existing_posegraph():
    assert should_resume_saved_map(MODE_DISASTER_MAPPING, True)


def test_mode_6_cannot_resume_when_posegraph_is_missing():
    assert not should_resume_saved_map(MODE_DISASTER_MAPPING, False)


def test_localization_backend_is_normalized_and_validated():
    assert normalize_localization_backend(' AMCL ') == LOCALIZATION_BACKEND_AMCL
    assert normalize_localization_backend('SLAM_TOOLBOX') == (
        LOCALIZATION_BACKEND_SLAM_TOOLBOX
    )


def test_localization_backend_rejects_unknown_value():
    try:
        normalize_localization_backend('cartographer')
    except ValueError as exc:
        assert 'localization_backend' in str(exc)
    else:
        raise AssertionError('unknown localization backend was accepted')


def test_static_map_resolves_relative_image(tmp_path):
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n1 1\n255\n\x00')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\nresolution: 0.05\n')

    assert occupancy_map_image_path(map_yaml) == image.resolve()
    assert static_map_exists(map_yaml)


def test_static_map_requires_existing_image(tmp_path):
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: missing.pgm\nresolution: 0.05\n')

    assert not static_map_exists(map_yaml)


def test_mode_3_and_4_use_backend_specific_saved_map():
    for mode in (MODE_MANUAL_LOCALIZATION, MODE_GOAL_NAVIGATION):
        assert saved_map_available_for_mode(
            mode,
            LOCALIZATION_BACKEND_AMCL,
            posegraph_exists=False,
            occupancy_map_exists=True,
        )
        assert saved_map_available_for_mode(
            mode,
            LOCALIZATION_BACKEND_SLAM_TOOLBOX,
            posegraph_exists=True,
            occupancy_map_exists=False,
        )


def test_mode_6_still_requires_posegraph_with_amcl_localization():
    assert not saved_map_available_for_mode(
        MODE_DISASTER_MAPPING,
        LOCALIZATION_BACKEND_AMCL,
        posegraph_exists=False,
        occupancy_map_exists=True,
    )


def test_mode_4_blocks_nav_until_amcl_pose_but_not_other_modes():
    assert not localization_allows_autonomous_drive(
        MODE_GOAL_NAVIGATION,
        LOCALIZATION_BACKEND_AMCL,
        amcl_pose_received=False,
    )
    assert localization_allows_autonomous_drive(
        MODE_GOAL_NAVIGATION,
        LOCALIZATION_BACKEND_AMCL,
        amcl_pose_received=True,
    )
    assert localization_allows_autonomous_drive(
        MODE_AUTO_MAPPING,
        LOCALIZATION_BACKEND_AMCL,
        amcl_pose_received=False,
    )


def test_amcl_pose_is_reseeded_only_across_localization_restart():
    assert should_reseed_amcl_pose(
        MODE_MANUAL_LOCALIZATION,
        MODE_GOAL_NAVIGATION,
        TOPOLOGY_LOCALIZATION,
        TOPOLOGY_LOCALIZATION,
        LOCALIZATION_BACKEND_AMCL,
        pose_available=True,
    )
    assert not should_reseed_amcl_pose(
        MODE_MANUAL_MAPPING,
        MODE_GOAL_NAVIGATION,
        'mapping',
        TOPOLOGY_LOCALIZATION,
        LOCALIZATION_BACKEND_AMCL,
        pose_available=True,
    )
    assert not should_reseed_amcl_pose(
        0,
        MODE_GOAL_NAVIGATION,
        TOPOLOGY_LOCALIZATION,
        TOPOLOGY_LOCALIZATION,
        LOCALIZATION_BACKEND_AMCL,
        pose_available=True,
    )


def test_switching_from_mode_6_to_mode_2_restarts_with_a_fresh_graph():
    assert mapping_runtime_flavor_changed('mapping', 'mapping', True, False)


def test_switching_between_fresh_mapping_modes_keeps_the_runtime():
    assert not mapping_runtime_flavor_changed('mapping', 'mapping', False, False)


def test_mode_3_to_mode_4_restarts_shared_localization_runtime_for_yolo():
    assert perception_runtime_flavor_changed(False, True, False, False)


def test_mode_4_to_mode_3_restarts_to_remove_yolo():
    assert perception_runtime_flavor_changed(True, False, False, False)


def test_unchanged_mode_4_perception_keeps_runtime():
    assert not perception_runtime_flavor_changed(True, True, False, False)


def test_mode_5_tuning_policy_change_restarts_perception():
    assert perception_runtime_flavor_changed(True, True, False, True)


def test_yolo_is_enabled_only_for_modes_4_and_5():
    enabled_modes = {MODE_GOAL_NAVIGATION, MODE_INSPECTION_DRIVE}

    for mode in range(7):
        expected = mode in enabled_modes
        actual = should_launch_perception(mode, True, True, enabled_modes)
        assert actual == expected


def test_yolo_requires_both_feature_flag_and_camera():
    assert not should_launch_perception(
        MODE_GOAL_NAVIGATION,
        False,
        True,
        {4, 5},
    )
    assert not should_launch_perception(
        MODE_GOAL_NAVIGATION,
        True,
        False,
        {4, 5},
    )


def test_person_detection_logs_only_on_transitions_in_yolo_modes():
    enabled_modes = {MODE_GOAL_NAVIGATION, MODE_INSPECTION_DRIVE}

    state, event = person_detection_transition(
        MODE_GOAL_NAVIGATION, enabled_modes, None, False
    )
    assert state is False
    assert event is None

    state, event = person_detection_transition(
        MODE_GOAL_NAVIGATION, enabled_modes, state, True
    )
    assert state is True
    assert event == 'detected'

    state, event = person_detection_transition(
        MODE_GOAL_NAVIGATION, enabled_modes, state, True
    )
    assert state is True
    assert event is None

    state, event = person_detection_transition(
        MODE_GOAL_NAVIGATION, enabled_modes, state, False
    )
    assert state is False
    assert event == 'cleared'


def test_person_detection_is_silent_outside_yolo_modes():
    state, event = person_detection_transition(
        MODE_MANUAL_LOCALIZATION,
        {MODE_GOAL_NAVIGATION, MODE_INSPECTION_DRIVE},
        True,
        True,
    )

    assert state is None
    assert event is None
