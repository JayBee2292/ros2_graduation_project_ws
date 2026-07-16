import cv2
import numpy as np

from h753_perception.perception_core import blue_clothing_ratio, median_depth_m


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
