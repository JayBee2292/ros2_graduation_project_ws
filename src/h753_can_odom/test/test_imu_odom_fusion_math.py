import pytest

import math

from h753_can_odom.imu_odom_fusion_node import (
    integrate_planar_pose,
    mean_gyro_bias,
)


def test_mean_gyro_bias_tracks_stationary_sensor_offset():
    samples = [0.0034, 0.0035, 0.0036]

    assert mean_gyro_bias(samples) == pytest.approx(0.0035)


def test_mean_gyro_bias_rejects_empty_calibration():
    with pytest.raises(ValueError):
        mean_gyro_bias([])


def test_relative_wheel_fallback_preserves_existing_fused_pose_origin():
    x, y, yaw = integrate_planar_pose(
        10.0,
        -4.0,
        math.radians(30.0),
        0.20,
        math.radians(10.0),
    )

    assert x == pytest.approx(10.0 + 0.20 * math.cos(math.radians(35.0)))
    assert y == pytest.approx(-4.0 + 0.20 * math.sin(math.radians(35.0)))
    assert yaw == pytest.approx(math.radians(40.0))


def test_relative_wheel_fallback_handles_rotation_without_translation_jump():
    x, y, yaw = integrate_planar_pose(3.5, 2.0, 0.2, 0.0, -0.4)

    assert x == pytest.approx(3.5)
    assert y == pytest.approx(2.0)
    assert yaw == pytest.approx(-0.2)
