#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import welch


@dataclass
class TimeSeries:
    timestamps_s: list[float]
    values: list[float]


ANALYSIS_TOPICS = {
    '/imu_filter/yaw_rate_raw': lambda msg: msg.data,
    '/imu_filter/yaw_rate_filtered': lambda msg: msg.data,
    '/imu_filter/vibration_variance': lambda msg: msg.data,
    '/imu_filter/imu_weight': lambda msg: msg.data,
    '/wheel/odom': lambda msg: msg.twist.twist.angular.z,
    '/cmd_vel_selected': lambda msg: msg.angular.z,
}


def estimate_sample_rate_hz(timestamps_s: list[float]) -> float:
    if len(timestamps_s) < 2:
        raise ValueError('at least two timestamps are required')
    intervals = np.diff(np.asarray(timestamps_s, dtype=float))
    intervals = intervals[intervals > 0.0]
    if len(intervals) == 0:
        raise ValueError('timestamps must contain a positive interval')
    return float(1.0 / np.median(intervals))


def allan_deviation(
    values: list[float],
    sample_rate_hz: float,
) -> list[tuple[float, float]]:
    """Calculate non-overlapping Allan deviation for powers-of-two clusters."""
    samples = np.asarray(values, dtype=float)
    results: list[tuple[float, float]] = []
    cluster_size = 1
    while cluster_size * 4 <= len(samples):
        cluster_count = len(samples) // cluster_size
        trimmed = samples[:cluster_count * cluster_size]
        cluster_means = trimmed.reshape(cluster_count, cluster_size).mean(axis=1)
        differences = np.diff(cluster_means)
        deviation = math.sqrt(0.5 * float(np.mean(differences * differences)))
        results.append((cluster_size / sample_rate_hz, deviation))
        cluster_size *= 2
    return results


def dominant_psd_peaks(
    values: list[float],
    sample_rate_hz: float,
    minimum_frequency_hz: float = 1.0,
    count: int = 5,
) -> tuple[np.ndarray, np.ndarray, list[tuple[float, float]]]:
    samples = np.asarray(values, dtype=float)
    if len(samples) < 8:
        raise ValueError('at least eight samples are required for PSD')
    frequencies, density = welch(
        samples - samples.mean(),
        fs=sample_rate_hz,
        nperseg=min(1024, len(samples)),
    )
    candidate_indices = np.where(frequencies >= minimum_frequency_hz)[0]
    ranked_indices = candidate_indices[np.argsort(density[candidate_indices])[::-1]]
    peaks = [
        (float(frequencies[index]), float(density[index]))
        for index in ranked_indices[:count]
    ]
    return frequencies, density, peaks


def read_bag_series(bag_path: Path) -> dict[str, TimeSeries]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }
    series = {
        topic: TimeSeries([], [])
        for topic in ANALYSIS_TOPICS
        if topic in topic_types
    }
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in series
    }

    # Pseudocode:
    #   stream each rosbag message once
    #   keep only yaw/filter topics needed for PSD and Allan analysis
    #   use the bag receive timestamp for headerless Float64 diagnostics
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in series:
            continue
        message = deserialize_message(serialized, message_types[topic])
        series[topic].timestamps_s.append(timestamp_ns / 1e9)
        series[topic].values.append(float(ANALYSIS_TOPICS[topic](message)))
    return series


def write_analysis(
    bag_path: Path,
    series: dict[str, TimeSeries],
) -> Path:
    output_directory = bag_path / 'imu_analysis'
    output_directory.mkdir(parents=True, exist_ok=True)
    summary_path = output_directory / 'summary.csv'
    psd_path = output_directory / 'raw_yaw_rate_psd.csv'
    allan_path = output_directory / 'raw_yaw_rate_allan.csv'

    with summary_path.open('w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow([
            'topic',
            'samples',
            'sample_rate_hz',
            'mean',
            'stddev',
            'rms',
        ])
        for topic, values in series.items():
            if len(values.values) < 2:
                continue
            data = np.asarray(values.values, dtype=float)
            sample_rate_hz = estimate_sample_rate_hz(values.timestamps_s)
            writer.writerow([
                topic,
                len(data),
                f'{sample_rate_hz:.9f}',
                f'{data.mean():.12g}',
                f'{data.std():.12g}',
                f'{math.sqrt(float(np.mean(data * data))):.12g}',
            ])

    raw = series.get('/imu_filter/yaw_rate_raw')
    if raw is None or len(raw.values) < 8:
        raise RuntimeError(
            'bag does not contain enough /imu_filter/yaw_rate_raw samples'
        )
    sample_rate_hz = estimate_sample_rate_hz(raw.timestamps_s)
    frequencies, density, peaks = dominant_psd_peaks(
        raw.values,
        sample_rate_hz,
    )
    with psd_path.open('w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['frequency_hz', 'power_density'])
        writer.writerows(zip(frequencies, density))
    with allan_path.open('w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['cluster_time_s', 'allan_deviation_rad_s'])
        writer.writerows(allan_deviation(raw.values, sample_rate_hz))

    print(f'Analysis directory: {output_directory}')
    print(f'Raw gyro sample rate: {sample_rate_hz:.3f} Hz')
    print('Dominant PSD peaks above 1 Hz:')
    for frequency_hz, power_density in peaks:
        print(f'  {frequency_hz:8.3f} Hz  {power_density:.6g} (rad/s)^2/Hz')
    return output_directory


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Compute PSD and Allan deviation from an H753 IMU rosbag.',
    )
    parser.add_argument('bag', type=Path, help='rosbag2 directory')
    args = parser.parse_args()
    bag_path = args.bag.expanduser().resolve()
    if not (bag_path / 'metadata.yaml').is_file():
        raise SystemExit(f'not a rosbag2 directory: {bag_path}')
    write_analysis(bag_path, read_bag_series(bag_path))


if __name__ == '__main__':
    main()
