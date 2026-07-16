import math

import pytest

from h753_can_odom.imu_filtering import (
    SecondOrderButterworthLowPass,
    adaptive_imu_weight,
    fuse_yaw_rates,
    hampel_filter_sample,
    population_variance,
    variance_to_weight,
)


def test_hampel_replaces_large_impulsive_spike_with_window_median():
    value, rejected = hampel_filter_sample(
        [-0.01, 0.0, 0.01, 0.0, 2.0],
        2.0,
        threshold_sigma=3.0,
        minimum_threshold=0.05,
    )

    assert rejected
    assert value == pytest.approx(0.0)


def test_hampel_threshold_floor_preserves_small_real_rate_change():
    value, rejected = hampel_filter_sample(
        [0.0, 0.0, 0.0, 0.0, 0.05],
        0.05,
        threshold_sigma=3.0,
        minimum_threshold=0.10,
    )

    assert not rejected
    assert value == pytest.approx(0.05)


def test_population_variance_matches_known_values():
    assert population_variance([]) == 0.0
    assert population_variance([3.0]) == 0.0
    assert population_variance([1.0, 3.0]) == pytest.approx(1.0)


def test_variance_to_weight_decreases_between_configured_bounds():
    assert variance_to_weight(0.0, 0.1, 0.5, 0.2, 0.9) == pytest.approx(0.9)
    assert variance_to_weight(0.3, 0.1, 0.5, 0.2, 0.9) == pytest.approx(0.55)
    assert variance_to_weight(1.0, 0.1, 0.5, 0.2, 0.9) == pytest.approx(0.2)


def test_fuse_yaw_rates_obeys_imu_confidence():
    assert fuse_yaw_rates(0.5, 0.1, 1.0) == pytest.approx(0.5)
    assert fuse_yaw_rates(0.5, 0.1, 0.0) == pytest.approx(0.1)
    assert fuse_yaw_rates(0.5, 0.1, 0.75) == pytest.approx(0.4)


def test_adaptive_weight_prefers_imu_for_quiet_tracked_turn():
    weight = adaptive_imu_weight(
        vibration_variance=0.00001,
        variance_low=0.0001,
        variance_high=0.0025,
        weight_min=0.35,
        weight_max=0.90,
        innovation_rad_s=0.4,
        innovation_gate_rad_s=0.35,
        robot_is_turning=True,
    )

    assert weight == pytest.approx(0.90)


def test_adaptive_weight_reduces_noisy_disagreeing_imu():
    weight = adaptive_imu_weight(
        vibration_variance=0.003,
        variance_low=0.0001,
        variance_high=0.0025,
        weight_min=0.35,
        weight_max=0.90,
        innovation_rad_s=0.4,
        innovation_gate_rad_s=0.35,
        robot_is_turning=True,
    )

    assert weight == pytest.approx(0.35)


def test_butterworth_has_no_false_dc_startup_transient():
    low_pass = SecondOrderButterworthLowPass(200.0, 10.0)

    assert [low_pass.update(0.25) for _ in range(20)] == pytest.approx([0.25] * 20)


def test_butterworth_preserves_slow_yaw_and_attenuates_fast_vibration():
    sample_rate_hz = 200.0
    slow_filter = SecondOrderButterworthLowPass(sample_rate_hz, 10.0)
    fast_filter = SecondOrderButterworthLowPass(sample_rate_hz, 10.0)
    slow_outputs = []
    fast_outputs = []

    for index in range(800):
        time_s = index / sample_rate_hz
        slow_outputs.append(slow_filter.update(math.sin(2.0 * math.pi * 2.0 * time_s)))
        fast_outputs.append(fast_filter.update(math.sin(2.0 * math.pi * 50.0 * time_s)))

    def rms(values):
        steady = values[200:]
        return math.sqrt(sum(value * value for value in steady) / len(steady))

    assert rms(slow_outputs) > 0.65
    assert rms(fast_outputs) < 0.05


@pytest.mark.parametrize(
    'arguments',
    [
        (200.0, 0.0),
        (200.0, 100.0),
        (0.0, 10.0),
    ],
)
def test_butterworth_rejects_invalid_frequency_configuration(arguments):
    with pytest.raises(ValueError):
        SecondOrderButterworthLowPass(*arguments)
