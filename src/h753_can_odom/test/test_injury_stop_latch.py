from h753_can_odom.cmd_vel_uart_bridge_node import (
    CmdVelUartBridgeNode,
    InjuryStopLatch,
    apply_track_stiction_floor,
    encode_twist_frame,
)


class FakeSerial:
    def __init__(self):
        self.is_open = True
        self.frames = []

    def write(self, frame):
        self.frames.append(frame)

    def flush(self):
        pass


def test_injury_stop_latches_until_explicit_zero():
    latch = InjuryStopLatch()

    assert latch.active is False
    assert latch.update(1) is True
    assert latch.active is True
    assert latch.update(1) is False
    assert latch.active is True
    assert latch.update(0) is True
    assert latch.active is False


def test_nonzero_value_fails_safe_to_stop():
    latch = InjuryStopLatch()

    assert latch.update(2) is True
    assert latch.active is True


def test_missing_injury_stop_message_does_not_block_drive():
    bridge = CmdVelUartBridgeNode.__new__(CmdVelUartBridgeNode)
    bridge.serial_port = FakeSerial()
    bridge.active_linear_mps = 0.5
    bridge.active_angular_radps = 0.0
    bridge.track_gauge_m = 0.45
    bridge.max_track_speed_mps = 0.60
    bridge.min_track_pwm_percent = 50.0
    bridge.min_in_place_turn_pwm_percent = 60.0
    bridge.in_place_turn_linear_threshold_mps = 0.02
    bridge.track_zero_deadband_mps = 0.01
    bridge.injury_stop = InjuryStopLatch()
    bridge._scan_is_fresh = lambda: True

    bridge._send_active_command()

    assert bridge.serial_port.frames == [encode_twist_frame(0.5, 0.0)]


def test_active_injury_stop_overrides_nonzero_motor_command():
    bridge = CmdVelUartBridgeNode.__new__(CmdVelUartBridgeNode)
    bridge.serial_port = FakeSerial()
    bridge.active_linear_mps = 0.5
    bridge.active_angular_radps = 0.0
    bridge.track_gauge_m = 0.45
    bridge.max_track_speed_mps = 0.60
    bridge.min_track_pwm_percent = 50.0
    bridge.min_in_place_turn_pwm_percent = 60.0
    bridge.in_place_turn_linear_threshold_mps = 0.02
    bridge.track_zero_deadband_mps = 0.01
    bridge.injury_stop = InjuryStopLatch()
    bridge.injury_stop.update(1)
    bridge._scan_is_fresh = lambda: True

    bridge._send_active_command()

    assert bridge.serial_port.frames == [encode_twist_frame(0.0, 0.0)]


def test_straight_command_is_raised_to_50_percent_pwm():
    linear_mps, angular_radps = apply_track_stiction_floor(
        0.08,
        0.0,
        track_gauge_m=0.45,
        max_track_speed_mps=0.60,
        min_track_pwm_percent=50.0,
        track_zero_deadband_mps=0.01,
    )

    assert abs(linear_mps - 0.30) < 1e-9
    assert angular_radps == 0.0


def test_in_place_turn_raises_both_tracks_to_50_percent_pwm():
    linear_mps, angular_radps = apply_track_stiction_floor(
        0.0,
        0.30,
        track_gauge_m=0.45,
        max_track_speed_mps=0.60,
        min_track_pwm_percent=50.0,
        track_zero_deadband_mps=0.01,
    )

    assert linear_mps == 0.0
    assert abs(angular_radps - (2.0 * 0.30 / 0.45)) < 1e-9


def test_in_place_turn_uses_dedicated_60_percent_pwm_floor():
    linear_mps, angular_radps = apply_track_stiction_floor(
        0.0,
        0.30,
        track_gauge_m=0.45,
        max_track_speed_mps=0.60,
        min_track_pwm_percent=50.0,
        track_zero_deadband_mps=0.01,
        min_in_place_turn_pwm_percent=60.0,
        in_place_turn_linear_threshold_mps=0.02,
    )

    assert linear_mps == 0.0
    assert abs(angular_radps - (2.0 * 0.36 / 0.45)) < 1e-9


def test_straight_command_keeps_50_percent_floor_with_turn_floor_enabled():
    linear_mps, angular_radps = apply_track_stiction_floor(
        0.08,
        0.0,
        track_gauge_m=0.45,
        max_track_speed_mps=0.60,
        min_track_pwm_percent=50.0,
        track_zero_deadband_mps=0.01,
        min_in_place_turn_pwm_percent=60.0,
        in_place_turn_linear_threshold_mps=0.02,
    )

    assert abs(linear_mps - 0.30) < 1e-9
    assert angular_radps == 0.0


def test_curve_floor_preserves_track_ratio():
    linear_mps, angular_radps = apply_track_stiction_floor(
        0.25,
        0.35,
        track_gauge_m=0.45,
        max_track_speed_mps=0.60,
        min_track_pwm_percent=50.0,
        track_zero_deadband_mps=0.01,
    )
    left_mps = linear_mps - angular_radps * 0.45 / 2.0
    right_mps = linear_mps + angular_radps * 0.45 / 2.0

    assert abs(left_mps - 0.30) < 1e-9
    assert abs((left_mps / right_mps) - (0.17125 / 0.32875)) < 1e-9
    assert right_mps <= 0.60


def test_exact_stop_and_stationary_inner_track_remain_zero():
    assert apply_track_stiction_floor(
        0.0,
        0.0,
        track_gauge_m=0.45,
        max_track_speed_mps=0.60,
        min_track_pwm_percent=50.0,
        track_zero_deadband_mps=0.01,
    ) == (0.0, 0.0)

    linear_mps, angular_radps = apply_track_stiction_floor(
        0.123,
        2.0 * 0.123 / 0.45,
        track_gauge_m=0.45,
        max_track_speed_mps=0.60,
        min_track_pwm_percent=50.0,
        track_zero_deadband_mps=0.01,
    )
    left_mps = linear_mps - angular_radps * 0.45 / 2.0
    right_mps = linear_mps + angular_radps * 0.45 / 2.0

    assert abs(left_mps) < 1e-9
    assert abs(right_mps - 0.30) < 1e-9
