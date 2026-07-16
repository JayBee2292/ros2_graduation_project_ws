from geometry_msgs.msg import Twist

from h753_can_odom.robot_mode_manager_node import (
    MODE_AUTO_MAPPING,
    MODE_DISASTER_MAPPING,
    MODE_GOAL_NAVIGATION,
    MODE_INSPECTION_DRIVE,
    MODE_MANUAL_MAPPING,
    MODE_MANUAL_LOCALIZATION,
    drive_limits_for_mode,
    mapping_runtime_flavor_changed,
    person_detection_transition,
    perception_runtime_flavor_changed,
    select_autonomous_drive_twist,
    should_launch_perception,
    should_resume_saved_map,
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


def test_all_drive_modes_use_one_speed_policy():
    drive_modes = (
        MODE_MANUAL_MAPPING,
        MODE_AUTO_MAPPING,
        MODE_MANUAL_LOCALIZATION,
        MODE_GOAL_NAVIGATION,
        MODE_INSPECTION_DRIVE,
        MODE_DISASTER_MAPPING,
    )

    for mode in drive_modes:
        assert drive_limits_for_mode(mode, (0.60, 2.67)) == (0.60, 2.67)


def test_mode_2_always_starts_a_new_map_even_when_posegraph_exists():
    assert not should_resume_saved_map(MODE_AUTO_MAPPING, True)


def test_mode_6_resumes_existing_posegraph():
    assert should_resume_saved_map(MODE_DISASTER_MAPPING, True)


def test_mode_6_cannot_resume_when_posegraph_is_missing():
    assert not should_resume_saved_map(MODE_DISASTER_MAPPING, False)


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
