import math

import pytest

from h753_can_odom.imu_vibration_analysis import (
    allan_deviation,
    dominant_psd_peaks,
    estimate_sample_rate_hz,
)


def test_estimate_sample_rate_uses_median_interval():
    assert estimate_sample_rate_hz([0.0, 0.005, 0.010, 0.016]) == pytest.approx(200.0)


def test_allan_deviation_is_zero_for_constant_rate():
    result = allan_deviation([0.25] * 128, 200.0)

    assert result
    assert all(deviation == pytest.approx(0.0) for _tau, deviation in result)


def test_psd_finds_known_vibration_frequency():
    sample_rate_hz = 200.0
    values = [
        math.sin(2.0 * math.pi * 37.0 * index / sample_rate_hz)
        for index in range(2000)
    ]

    _frequencies, _density, peaks = dominant_psd_peaks(
        values,
        sample_rate_hz,
        count=1,
    )

    assert peaks[0][0] == pytest.approx(37.0, abs=0.25)
