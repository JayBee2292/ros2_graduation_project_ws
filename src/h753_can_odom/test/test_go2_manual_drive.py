from geometry_msgs.msg import Twist

from h753_can_odom.go2_manual_drive_node import (
    joystick_axes_to_drive_twist,
    manual_drive_enabled,
    select_go2_drive_twist,
)


def make_twist(linear_x, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    return twist


def test_manual_drive_requires_fresh_deadman_without_latches():
    assert manual_drive_enabled(True, True, False, False)
    assert not manual_drive_enabled(False, True, False, False)
    assert not manual_drive_enabled(True, False, False, False)
    assert not manual_drive_enabled(True, True, True, False)
    assert not manual_drive_enabled(True, True, False, True)


def test_go2_manual_drive_uses_existing_75_to_100_turn_mix():
    linear_mps, angular_radps = joystick_axes_to_drive_twist(
        axes=[0.0, -1.0, 0.0, 1.0],
        left_stick_y_axis=1,
        right_stick_x_axis=3,
        deadzone=0.15,
        max_linear_mps=0.60,
        max_angular_radps=2.67,
        track_gauge_m=0.45,
        moving_inner_ratio=0.75,
    )
    left_mps = linear_mps - angular_radps * 0.45 / 2.0
    right_mps = linear_mps + angular_radps * 0.45 / 2.0

    assert abs(left_mps - 0.60) < 1e-9
    assert abs(right_mps - 0.45) < 1e-9


def test_missing_axes_produce_stop_instead_of_index_error():
    linear_mps, angular_radps = joystick_axes_to_drive_twist(
        axes=[],
        left_stick_y_axis=1,
        right_stick_x_axis=3,
        deadzone=0.15,
        max_linear_mps=0.60,
        max_angular_radps=2.67,
        track_gauge_m=0.45,
        moving_inner_ratio=0.75,
    )

    assert linear_mps == 0.0
    assert angular_radps == 0.0


def test_go2_navigation_command_uses_uart_coordinate_signs():
    selected = select_go2_drive_twist(
        True,
        False,
        False,
        False,
        make_twist(0.0, 0.0),
        True,
        make_twist(0.4, -0.6),
        (0.60, 2.67),
    )

    assert selected.linear.x == -0.4
    assert selected.angular.z == 0.6


def test_go2_navigation_keeps_nav2_component_limited_curve():
    selected = select_go2_drive_twist(
        True,
        False,
        False,
        False,
        make_twist(0.0, 0.0),
        True,
        make_twist(-0.54, -1.0),
        (0.60, 2.67),
    )

    assert selected.linear.x == 0.54
    assert selected.angular.z == 1.0


def test_go2_lb_override_has_priority_over_navigation():
    selected = select_go2_drive_twist(
        True,
        False,
        True,
        True,
        make_twist(-0.5, 0.2),
        True,
        make_twist(0.4, -0.6),
        (0.60, 2.67),
    )

    assert selected.linear.x == -0.5
    assert selected.angular.z == 0.2


def test_go2_stale_lb_override_does_not_fall_back_to_navigation():
    selected = select_go2_drive_twist(
        True,
        False,
        True,
        False,
        make_twist(-0.5, 0.2),
        True,
        make_twist(0.4, -0.6),
        (0.60, 2.67),
    )

    assert selected.linear.x == 0.0
    assert selected.angular.z == 0.0


def test_go2_stop_latch_blocks_manual_and_navigation():
    selected = select_go2_drive_twist(
        True,
        True,
        True,
        True,
        make_twist(-0.5, 0.2),
        True,
        make_twist(0.4, -0.6),
        (0.60, 2.67),
    )

    assert selected.linear.x == 0.0
    assert selected.angular.z == 0.0
