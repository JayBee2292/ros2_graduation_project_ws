from h753_can_odom.vlm_gateway_node import VlmSafetyGate


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
