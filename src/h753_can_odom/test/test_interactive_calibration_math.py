import math
from pathlib import Path

import pytest

from h753_can_odom.interactive_calibration import (
    InteractiveCalibration,
    calibrated_track_width,
)


def test_track_width_uses_physical_baseline_ratio():
    result = calibrated_track_width(0.50, math.radians(450.0), math.radians(360.0))
    assert result == pytest.approx(0.625)


def test_repeated_results_remain_independent_of_previous_saved_result():
    baseline = 0.50
    first = calibrated_track_width(baseline, math.radians(400.0), math.radians(360.0))
    second = calibrated_track_width(baseline, math.radians(380.0), math.radians(360.0))

    assert first == pytest.approx(0.5555555556)
    assert second == pytest.approx(0.5277777778)
    assert second != pytest.approx(
        calibrated_track_width(first, math.radians(380.0), math.radians(360.0))
    )


def test_default_rotation_calibration_accepts_full_turn(monkeypatch):
    monkeypatch.setattr('builtins.input', lambda _prompt: '')

    assert InteractiveCalibration.prompt_actual_rotation_degrees() == 360.0


def test_rotation_calibration_accepts_two_full_turns(monkeypatch):
    monkeypatch.setattr('builtins.input', lambda _prompt: '720')

    assert InteractiveCalibration.prompt_actual_rotation_degrees() == 720.0


def test_calibration_yaml_always_quotes_ros_ambiguous_y_axis(tmp_path: Path):
    path = tmp_path / 'imu.yaml'
    data = {
        'h753_imu_odom_fusion': {
            'ros__parameters': {
                'imu_yaw_axis': 'y',
                'imu_yaw_sign': -1.0,
            },
        },
    }

    InteractiveCalibration.write_yaml(path, data)

    assert 'imu_yaw_axis: "y"' in path.read_text(encoding='utf-8')
