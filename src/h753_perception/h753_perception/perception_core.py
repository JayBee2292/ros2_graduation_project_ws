from __future__ import annotations

from collections.abc import Sequence

import cv2
import numpy as np

Box = tuple[int, int, int, int]


def clip_box(box: Sequence[int], width: int, height: int) -> Box:
    """Clip an xyxy detection box to image bounds."""
    x1, y1, x2, y2 = (int(value) for value in box)
    x1 = max(0, min(width, x1))
    x2 = max(0, min(width, x2))
    y1 = max(0, min(height, y1))
    y2 = max(0, min(height, y2))
    return x1, y1, x2, y2


def median_depth_m(
    depth_image: np.ndarray | None,
    box: Box,
    roi_fraction: float = 0.20,
    depth_scale_m: float = 0.001,
) -> float | None:
    """Return median valid depth from a small ROI at the bbox center."""
    if depth_image is None or depth_image.ndim < 2:
        return None

    height, width = depth_image.shape[:2]
    x1, y1, x2, y2 = clip_box(box, width, height)
    if x2 <= x1 or y2 <= y1:
        return None

    # Pseudocode:
    #   crop a small center patch instead of sampling one noisy depth pixel
    #   discard zero/invalid RealSense samples
    #   use the median so one foreground/background outlier cannot dominate
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    roi_width = max(2, int((x2 - x1) * roi_fraction))
    roi_height = max(2, int((y2 - y1) * roi_fraction))
    roi_x1 = max(0, center_x - roi_width // 2)
    roi_x2 = min(width, center_x + max(1, roi_width // 2))
    roi_y1 = max(0, center_y - roi_height // 2)
    roi_y2 = min(height, center_y + max(1, roi_height // 2))
    depth_roi = depth_image[roi_y1:roi_y2, roi_x1:roi_x2]
    valid_depths = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0)]
    if valid_depths.size == 0:
        return None
    return float(np.median(valid_depths)) * depth_scale_m


def blue_clothing_ratio(
    frame: np.ndarray,
    box: Box,
    h_min: int,
    h_max: int,
    s_min: int,
    v_min: int,
    torso_top_ratio: float = 0.15,
    torso_bottom_ratio: float = 0.60,
) -> tuple[float, np.ndarray | None, Box | None]:
    """Measure blue pixels in the torso portion of a person bbox."""
    if frame is None or frame.ndim != 3:
        return 0.0, None, None
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = clip_box(box, width, height)
    if x2 <= x1 or y2 <= y1:
        return 0.0, None, None

    # Pseudocode:
    #   ignore head and lower body to reduce hair/jeans false positives
    #   threshold the remaining torso pixels in HSV color space
    #   classify blue clothing from the blue-pixel fraction
    box_height = y2 - y1
    top = max(y1, min(y2, y1 + int(box_height * torso_top_ratio)))
    bottom = max(top, min(y2, y1 + int(box_height * torso_bottom_ratio)))
    roi = frame[top:bottom, x1:x2]
    if roi.size == 0:
        return 0.0, None, None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        (int(h_min), int(s_min), int(v_min)),
        (int(h_max), 255, 255),
    )
    ratio = cv2.countNonZero(mask) / float(mask.shape[0] * mask.shape[1])
    return ratio, mask, (x1, top, x2, bottom)
