#!/usr/bin/env python3
"""Summarize terminal integrity state from a recorded ROS 2 bag.

This is an offline diagnostics tool.  It never publishes ROS messages and is
therefore safe to use on a frozen experiment artifact.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = (
    "/integrity/directional",
    "/integrity/exploration_decision",
    "/integrity/exploration_debug",
    "/integrity/information_map",
    "/localization/odom",
    "/xq/p12/flight_status",
)


def stamp_s(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def finite_list(values) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=float).reshape(-1)]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--tail-seconds", type=float, default=30.0)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = [topic for topic in TOPICS if topic in topic_types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    message_types = {topic: get_message(topic_types[topic]) for topic in selected}

    records: list[tuple[str, int, object]] = []
    final_bag_ns = 0
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        final_bag_ns = max(final_bag_ns, int(timestamp_ns))
        records.append((topic, int(timestamp_ns), deserialize_message(data, message_types[topic])))

    cutoff_ns = final_bag_ns - int(max(args.tail_seconds, 0.0) * 1.0e9)
    tail = [record for record in records if record[1] >= cutoff_ns]
    summary: dict[str, object] = {
        "bag": str(args.bag.resolve()),
        "selected_topics": selected,
        "tail_seconds": args.tail_seconds,
        "tail_message_count": len(tail),
    }

    directional = [record for record in tail if record[0] == "/integrity/directional"]
    if directional:
        samples = []
        for _, timestamp_ns, message in directional:
            covariance = np.asarray(message.integrity_covariance, dtype=float).reshape(3, 3)
            covariance = 0.5 * (covariance + covariance.T)
            values, vectors = np.linalg.eigh(covariance)
            samples.append(
                {
                    "bag_time_s": timestamp_ns * 1.0e-9,
                    "stamp_s": stamp_s(message),
                    "covariance": np.asarray(covariance).tolist(),
                    "eigenvalues_m2": finite_list(values),
                    "maximum_variance_direction": finite_list(vectors[:, -1]),
                    "reported_weak_direction": finite_list(message.weak_direction_map),
                }
            )
        summary["directional_terminal"] = samples[-1]
        summary["directional_tail_max_eigenvalue_m2"] = max(
            sample["eigenvalues_m2"][-1] for sample in samples
        )

    decisions = [record for record in tail if record[0] == "/integrity/exploration_decision"]
    if decisions:
        latest_by_batch = {}
        for _, timestamp_ns, message in decisions:
            latest_by_batch[int(message.batch_id)] = {
                "bag_time_s": timestamp_ns * 1.0e-9,
                "batch_id": int(message.batch_id),
                "candidate_names": list(message.candidate_names),
                "predicted_minimum_margins_m": finite_list(
                    message.predicted_minimum_margins
                ),
                "integrity_feasible": list(message.integrity_feasible),
                "unconstrained_selected_name": message.unconstrained_selected_name,
                "selected_name": message.selected_name,
                "reason": message.reason,
            }
        summary["terminal_decisions"] = [latest_by_batch[key] for key in sorted(latest_by_batch)]

    debug = [record for record in tail if record[0] == "/integrity/exploration_debug"]
    parsed_debug = []
    for _, timestamp_ns, message in debug:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            continue
        payload["bag_time_s"] = timestamp_ns * 1.0e-9
        parsed_debug.append(payload)
    if parsed_debug:
        summary["terminal_exploration_debug"] = parsed_debug[-6:]

    maps = [record for record in tail if record[0] == "/integrity/information_map"]
    if maps:
        _, timestamp_ns, message = maps[-1]
        normals = np.asarray(
            [(normal.x, normal.y, normal.z) for normal in message.normals], dtype=float
        )
        information = np.zeros((3, 3), dtype=float)
        if len(normals):
            static = np.asarray(message.static_confidence, dtype=float)
            quality = np.asarray(message.geometry_quality, dtype=float)
            weights = static * quality
            information = np.einsum("n,ni,nj->ij", weights, normals, normals)
        values, vectors = np.linalg.eigh(0.5 * (information + information.T))
        summary["information_map_terminal"] = {
            "bag_time_s": timestamp_ns * 1.0e-9,
            "surfel_count": int(len(normals)),
            "normal_information_eigenvalues": finite_list(values),
            "weak_normal_direction": finite_list(vectors[:, 0]),
            "axis_absolute_normal_sums": finite_list(np.abs(normals).sum(axis=0))
            if len(normals)
            else [0.0, 0.0, 0.0],
        }

    odometry = [record for record in tail if record[0] == "/localization/odom"]
    if odometry:
        _, timestamp_ns, message = odometry[-1]
        position = message.pose.pose.position
        summary["odometry_terminal"] = {
            "bag_time_s": timestamp_ns * 1.0e-9,
            "stamp_s": stamp_s(message),
            "position_m": [float(position.x), float(position.y), float(position.z)],
        }

    statuses = [record for record in tail if record[0] == "/xq/p12/flight_status"]
    parsed_statuses = []
    for _, timestamp_ns, message in statuses:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            continue
        payload["bag_time_s"] = timestamp_ns * 1.0e-9
        parsed_statuses.append(payload)
    if parsed_statuses:
        summary["flight_status_terminal"] = parsed_statuses[-1]

    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
