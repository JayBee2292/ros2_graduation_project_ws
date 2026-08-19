from pathlib import Path

import cv2
import numpy as np
import yaml

from h753_perception.perception_core import (
    blue_clothing_ratio,
    DetectionClearHold,
    median_depth_m,
)


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "h753_yolo_perception.yaml"
)


def test_center_roi_depth_ignores_zero_samples_and_uses_median():
    depth = np.zeros((100, 100), dtype=np.uint16)
    depth[45:55, 45:55] = 2000
    depth[50, 50] = 9000

    distance = median_depth_m(depth, (25, 25, 75, 75), 0.20, 0.001)

    assert distance == 2.0


def test_center_roi_depth_returns_none_without_valid_samples():
    depth = np.zeros((20, 20), dtype=np.uint16)

    assert median_depth_m(depth, (0, 0, 20, 20)) is None


def test_blue_ratio_only_uses_torso_slice():
    frame = np.zeros((100, 50, 3), dtype=np.uint8)
    blue_bgr = cv2.cvtColor(
        np.uint8([[[115, 255, 255]]]),
        cv2.COLOR_HSV2BGR,
    )[0, 0]
    frame[15:60, :] = blue_bgr

    ratio, mask, torso_box = blue_clothing_ratio(
        frame,
        (0, 0, 50, 100),
        100,
        130,
        80,
        50,
        0.15,
        0.60,
    )

    assert ratio == 1.0
    assert mask is not None
    assert torso_box == (0, 15, 50, 60)


def test_yolo_annotated_stream_targets_camera_rate_without_raising_inference_load():
    with CONFIG_PATH.open(encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["h753_yolo_perception"]["ros__parameters"]

    assert params["output_rate_hz"] == 15.0
    assert params["inference_rate_hz"] == 5.0
    assert params["detection_clear_hold_s"] == 2.0


def test_detection_clear_hold_asserts_immediately():
    detection = DetectionClearHold(clear_hold_s=2.0)

    assert detection.update(True, now=1.0) is True


def test_detection_clear_hold_ignores_short_dropout():
    detection = DetectionClearHold(clear_hold_s=2.0)
    detection.update(True, now=0.0)

    assert detection.update(False, now=1.0) is True
    assert detection.update(False, now=2.9) is True
    assert detection.update(True, now=3.0) is True
    assert detection.clear_since is None


def test_detection_clear_hold_clears_after_continuous_absence():
    detection = DetectionClearHold(clear_hold_s=2.0)
    detection.update(True, now=0.0)
    detection.update(False, now=1.0)

    assert detection.update(False, now=3.0) is False
    assert detection.active is False


def test_detection_clear_hold_can_be_disabled_with_zero():
    detection = DetectionClearHold(clear_hold_s=0.0)
    detection.update(True, now=0.0)

    assert detection.update(False, now=0.1) is False


def test_yolo_preview_is_pumped_from_executor_main_loop():
    source_path = (
        Path(__file__).resolve().parents[1]
        / "h753_perception"
        / "yolo_perception_node.py"
    )
    source = source_path.read_text(encoding="utf-8")

    assert "executor.spin_once(timeout_sec=0.02)" in source
    assert "node._pump_windows()" in source
    show_method = source[source.index("    def _show_images("):]
    show_method = show_method[: show_method.index("    def _pump_windows(")]
    assert "cv2.imshow" not in show_method
