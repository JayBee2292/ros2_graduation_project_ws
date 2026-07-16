from h753_can_odom.cmd_vel_uart_bridge_node import (
    CmdVelUartBridgeNode,
    InjuryStopLatch,
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
    bridge.active_angular_radps = 1.0
    bridge.min_linear_mps = 0.08
    bridge.min_angular_radps = 0.30
    bridge.injury_stop = InjuryStopLatch()
    bridge._scan_is_fresh = lambda: True

    bridge._send_active_command()

    assert bridge.serial_port.frames == [encode_twist_frame(0.5, 1.0)]


def test_active_injury_stop_overrides_nonzero_motor_command():
    bridge = CmdVelUartBridgeNode.__new__(CmdVelUartBridgeNode)
    bridge.serial_port = FakeSerial()
    bridge.active_linear_mps = 0.5
    bridge.active_angular_radps = 1.0
    bridge.min_linear_mps = 0.08
    bridge.min_angular_radps = 0.30
    bridge.injury_stop = InjuryStopLatch()
    bridge.injury_stop.update(1)
    bridge._scan_is_fresh = lambda: True

    bridge._send_active_command()

    assert bridge.serial_port.frames == [encode_twist_frame(0.0, 0.0)]
