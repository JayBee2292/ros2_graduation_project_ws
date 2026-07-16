from __future__ import annotations

import math
import statistics
from collections.abc import Sequence


def hampel_filter_sample(
    samples: Sequence[float],
    current_sample: float,
    threshold_sigma: float,
    minimum_threshold: float,
) -> tuple[float, bool]:
    """Replace an impulsive sample using a median/MAD robust threshold."""
    if not samples:
        return current_sample, False
    if threshold_sigma <= 0.0:
        raise ValueError('threshold_sigma must be positive')
    if minimum_threshold < 0.0:
        raise ValueError('minimum_threshold must be non-negative')

    # Pseudocode:
    #   center = median(recent samples)
    #   robust_sigma = 1.4826 * median(abs(sample - center))
    #   threshold = max(configured floor, sigma multiplier * robust_sigma)
    #   if current sample is outside the threshold: replace it with center
    center = float(statistics.median(samples))
    mad = float(statistics.median(abs(value - center) for value in samples))
    robust_sigma = 1.4826 * mad
    threshold = max(minimum_threshold, threshold_sigma * robust_sigma)
    rejected = abs(current_sample - center) > threshold
    return (center, True) if rejected else (current_sample, False)


def population_variance(samples: Sequence[float]) -> float:
    """Return population variance without requiring NumPy at runtime."""
    if len(samples) < 2:
        return 0.0
    mean = sum(samples) / len(samples)
    return sum((value - mean) ** 2 for value in samples) / len(samples)


def variance_to_weight(
    variance: float,
    variance_low: float,
    variance_high: float,
    weight_min: float,
    weight_max: float,
) -> float:
    """Map vibration variance to a bounded, decreasing IMU weight."""
    if variance_low < 0.0:
        raise ValueError('variance_low must be non-negative')
    if variance_high <= variance_low:
        raise ValueError('variance_high must be greater than variance_low')
    if not 0.0 <= weight_min <= weight_max <= 1.0:
        raise ValueError('weights must satisfy 0 <= min <= max <= 1')

    # Pseudocode:
    #   quiet vibration range -> maximum IMU trust
    #   severe vibration range -> minimum IMU trust
    #   between thresholds -> linearly interpolate the trust
    if variance <= variance_low:
        return weight_max
    if variance >= variance_high:
        return weight_min
    ratio = (variance - variance_low) / (variance_high - variance_low)
    return weight_max + ratio * (weight_min - weight_max)


def fuse_yaw_rates(
    imu_yaw_rate: float,
    encoder_yaw_rate: float,
    imu_weight: float,
) -> float:
    """Blend two yaw-rate observations using the IMU confidence."""
    if not 0.0 <= imu_weight <= 1.0:
        raise ValueError('imu_weight must be between 0 and 1')
    return (
        imu_weight * imu_yaw_rate
        + (1.0 - imu_weight) * encoder_yaw_rate
    )


def adaptive_imu_weight(
    vibration_variance: float,
    variance_low: float,
    variance_high: float,
    weight_min: float,
    weight_max: float,
    innovation_rad_s: float,
    innovation_gate_rad_s: float,
    robot_is_turning: bool,
) -> float:
    """Choose IMU trust from vibration, disagreement, and turning state."""
    if innovation_gate_rad_s < 0.0:
        raise ValueError('innovation_gate_rad_s must be non-negative')

    # Pseudocode:
    #   start with a weight that falls as vibration variance rises
    #   severe vibration + sensor disagreement -> minimum IMU trust
    #   quiet tracked-vehicle turning -> maximum IMU trust because tracks slip
    weight = variance_to_weight(
        vibration_variance,
        variance_low,
        variance_high,
        weight_min,
        weight_max,
    )
    if (
        vibration_variance >= variance_high
        and abs(innovation_rad_s) >= innovation_gate_rad_s
    ):
        return weight_min
    if robot_is_turning and vibration_variance <= variance_low:
        return weight_max
    return weight


class SecondOrderButterworthLowPass:
    """Stateful second-order Butterworth low-pass biquad."""

    def __init__(self, sample_rate_hz: float, cutoff_hz: float) -> None:
        if sample_rate_hz <= 0.0:
            raise ValueError('sample_rate_hz must be positive')
        if not 0.0 < cutoff_hz < sample_rate_hz / 2.0:
            raise ValueError('cutoff_hz must be between 0 and Nyquist frequency')

        # Bilinear-transform coefficients for a Q=1/sqrt(2) Butterworth filter.
        k = math.tan(math.pi * cutoff_hz / sample_rate_hz)
        norm = 1.0 / (1.0 + math.sqrt(2.0) * k + k * k)
        self.b0 = k * k * norm
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (k * k - 1.0) * norm
        self.a2 = (1.0 - math.sqrt(2.0) * k + k * k) * norm
        self._initialized = False
        self._x1 = 0.0
        self._x2 = 0.0
        self._y1 = 0.0
        self._y2 = 0.0

    def reset(self) -> None:
        self._initialized = False
        self._x1 = 0.0
        self._x2 = 0.0
        self._y1 = 0.0
        self._y2 = 0.0

    def update(self, sample: float) -> float:
        if not math.isfinite(sample):
            raise ValueError('sample must be finite')
        if not self._initialized:
            # Initialize all delay elements at the first DC value to avoid a
            # false startup transient immediately after gyro-bias collection.
            self._initialized = True
            self._x1 = sample
            self._x2 = sample
            self._y1 = sample
            self._y2 = sample
            return sample

        # Pseudocode:
        #   y[n] = feedforward(current and two previous inputs)
        #          - feedback(two previous outputs)
        #   shift the two-sample input/output history
        output = (
            self.b0 * sample
            + self.b1 * self._x1
            + self.b2 * self._x2
            - self.a1 * self._y1
            - self.a2 * self._y2
        )
        self._x2 = self._x1
        self._x1 = sample
        self._y2 = self._y1
        self._y1 = output
        return output
