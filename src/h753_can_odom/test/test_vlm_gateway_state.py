from h753_can_odom.vlm_gateway_node import (
    LocalDetectionRearmGate,
    VlmSafetyGate,
)


def test_mapping_modes_do_not_accept_server_stop():
    gate = VlmSafetyGate((3, 4, 5))

    gate.update_mode(2)
    update = gate.update_stop(1)

    assert gate.communication_enabled is False
    assert update.accepted is False
    assert gate.stop_active is False


def test_enabled_mode_normalizes_invalid_value_to_stop():
    gate = VlmSafetyGate((3, 4, 5))

    gate.update_mode(4)
    update = gate.update_stop(7)

    assert update.accepted is True
    assert update.valid is False
    assert update.value == 1
    assert gate.stop_active is True


def test_stop_keeps_communication_enabled_until_explicit_zero():
    gate = VlmSafetyGate((3, 4, 5))

    gate.update_mode(3)
    gate.update_stop(1)
    gate.update_mode(0)

    assert gate.mode_enabled is False
    assert gate.communication_enabled is True

    update = gate.update_stop(0)

    assert update.accepted is True
    assert gate.stop_active is False
    assert gate.communication_enabled is False


def test_zero_in_enabled_mode_allows_drive():
    gate = VlmSafetyGate((3, 4, 5))

    gate.update_mode(5)
    update = gate.update_stop(0)

    assert update.accepted is True
    assert update.value == 0
    assert gate.stop_active is False


def test_local_stop_latches_without_server_round_trip():
    gate = VlmSafetyGate((3, 4, 5))

    gate.update_mode(4)
    changed = gate.assert_local_stop()

    assert changed is True
    assert gate.stop_active is True

    # A second on-board trigger while already stopped is a no-op, not a
    # duplicate log-worthy change.
    assert gate.assert_local_stop() is False


def test_local_stop_ignored_outside_enabled_modes():
    gate = VlmSafetyGate((3, 4, 5))

    gate.update_mode(2)
    changed = gate.assert_local_stop()

    assert changed is False
    assert gate.stop_active is False


def test_local_stop_only_clears_on_explicit_server_zero():
    gate = VlmSafetyGate((3, 4, 5))

    gate.update_mode(4)
    gate.assert_local_stop()
    gate.update_mode(0)

    assert gate.communication_enabled is True
    assert gate.stop_active is True

    update = gate.update_stop(0)

    assert update.accepted is True
    assert gate.stop_active is False


def make_local_rearm_gate() -> LocalDetectionRearmGate:
    return LocalDetectionRearmGate(
        ('/yolo/person_found', '/yolo/blue_person'),
        cooldown_s=15.0,
        clear_hold_s=2.0,
    )


def test_local_detection_first_rising_edge_requests_immediate_stop():
    rearm = make_local_rearm_gate()

    assert rearm.update_gate('/yolo/person_found', 1, now=0.0) is True
    assert rearm.update_gate('/yolo/person_found', 1, now=0.1) is False


def test_validated_clear_blocks_same_person_restop():
    rearm = make_local_rearm_gate()
    assert rearm.update_gate('/yolo/person_found', 1, now=0.0) is True

    rearm.block_after_validated_clear(now=1.0)
    rearm.update_gate('/yolo/person_found', 0, now=2.0)

    assert rearm.update_gate('/yolo/person_found', 1, now=2.5) is False
    assert rearm.detection_armed is False
    assert rearm.clear_since is None


def test_short_clear_does_not_rearm_local_stop():
    rearm = make_local_rearm_gate()
    rearm.update_gate('/yolo/person_found', 1, now=0.0)
    rearm.block_after_validated_clear(now=1.0)
    rearm.update_gate('/yolo/person_found', 0, now=14.5)

    assert rearm.tick(now=15.5) is False
    assert rearm.update_gate('/yolo/person_found', 1, now=15.6) is False
    assert rearm.detection_armed is False


def test_clear_hold_alone_does_not_bypass_cooldown():
    rearm = make_local_rearm_gate()
    rearm.update_gate('/yolo/person_found', 1, now=0.0)
    rearm.block_after_validated_clear(now=1.0)
    rearm.update_gate('/yolo/person_found', 0, now=2.0)

    assert rearm.tick(now=4.1) is False
    assert rearm.detection_armed is False


def test_cooldown_and_continuous_clear_rearm_next_person():
    rearm = make_local_rearm_gate()
    rearm.update_gate('/yolo/person_found', 1, now=0.0)
    rearm.block_after_validated_clear(now=1.0)
    rearm.update_gate('/yolo/person_found', 0, now=13.0)

    assert rearm.tick(now=16.0) is True
    assert rearm.detection_armed is True
    assert rearm.update_gate('/yolo/person_found', 1, now=17.0) is True


def test_any_active_yolo_gate_cancels_shared_clear_timer():
    rearm = make_local_rearm_gate()
    rearm.update_gate('/yolo/person_found', 1, now=0.0)
    rearm.block_after_validated_clear(now=1.0)
    rearm.update_gate('/yolo/person_found', 0, now=13.0)
    rearm.update_gate('/yolo/blue_person', 1, now=14.5)

    assert rearm.tick(now=16.0) is False
    assert rearm.clear_since is None


def test_repeated_server_clear_can_be_ignored_without_resetting_gate():
    rearm = make_local_rearm_gate()
    rearm.update_gate('/yolo/person_found', 1, now=0.0)
    rearm.block_after_validated_clear(now=1.0)
    rearm.update_gate('/yolo/person_found', 0, now=13.0)

    # The node calls block_after_validated_clear only for a real stop 1->0
    # transition. Repeated zero keepalives therefore leave this state intact.
    assert rearm.tick(now=16.0) is True


def test_fresh_server_disarmed_state_blocks_local_rearm_race():
    rearm = make_local_rearm_gate()
    rearm.update_gate('/yolo/person_found', 1, now=0.0)
    rearm.block_after_validated_clear(now=1.0)
    rearm.update_gate('/yolo/person_found', 0, now=13.0)

    assert rearm.tick(now=16.0, server_armed=False) is False
    assert rearm.detection_armed is False
    assert rearm.tick(now=16.2, server_armed=True) is True


def test_missing_server_status_keeps_backward_compatible_local_rearm():
    rearm = make_local_rearm_gate()
    rearm.update_gate('/yolo/person_found', 1, now=0.0)
    rearm.block_after_validated_clear(now=1.0)
    rearm.update_gate('/yolo/person_found', 0, now=13.0)

    assert rearm.tick(now=16.0, server_armed=None) is True
