#!/usr/bin/env python3
"""Scan every P6 message in a rosbag and verify directional equations."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(arguments.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = ("/localization/geometry", "/integrity/directional")
    for topic in required:
        if topic not in topic_types:
            raise RuntimeError(f"required P6 topic missing: {topic}")
    geometry_type = get_message(topic_types[required[0]])
    integrity_type = get_message(topic_types[required[1]])

    geometry_count = 0
    integrity_count = 0
    effective_points: list[int] = []
    condition_numbers: list[float] = []
    weak_pl_values: list[float] = []
    axis_pl_values: list[np.ndarray] = []
    weak_axis_counts = np.zeros(3, dtype=int)
    minimum_covariance_eigenvalue = float("inf")
    maximum_formula_error = 0.0

    while reader.has_next():
        topic, serialized, _timestamp = reader.read_next()
        if topic == required[0]:
            message = deserialize_message(serialized, geometry_type)
            geometry_count += 1
            effective_points.append(int(message.effective_points))
        elif topic == required[1]:
            message = deserialize_message(serialized, integrity_type)
            integrity_count += 1
            condition_numbers.append(float(message.condition_number))
            weak_pl_values.append(float(message.weak_direction_protection_level))
            axis_pl_values.append(np.asarray(message.protection_level_axes, dtype=float))
            covariance = np.asarray(message.integrity_covariance, dtype=float).reshape(3, 3)
            weak = np.asarray(message.weak_direction_map, dtype=float)
            minimum_covariance_eigenvalue = min(
                minimum_covariance_eigenvalue, float(np.linalg.eigvalsh(covariance)[0])
            )
            expected = float(message.k_alpha * np.sqrt(max(weak @ covariance @ weak, 0.0)))
            maximum_formula_error = max(
                maximum_formula_error,
                abs(expected - float(message.weak_direction_protection_level)),
            )
            weak_axis_counts[int(np.argmax(np.abs(weak)))] += 1

    axes = np.asarray(axis_pl_values, dtype=float)
    all_finite = bool(
        condition_numbers
        and np.all(np.isfinite(condition_numbers))
        and np.all(np.isfinite(weak_pl_values))
        and np.all(np.isfinite(axes))
    )
    checks = {
        "geometry_stream_full_run": geometry_count >= 1000,
        "integrity_stream_full_run": integrity_count >= 1000,
        "all_outputs_finite": all_finite,
        "integrity_covariance_positive_definite": minimum_covariance_eigenvalue > 0.0,
        "protection_level_equation_exact": maximum_formula_error <= 1.0e-9,
        "effective_constraints_present": min(effective_points, default=0) > 0,
        "directional_response_observed": (
            max(weak_pl_values, default=0.0) > min(weak_pl_values, default=0.0) * 1.05
        ),
    }
    result = {
        "schema_version": 1,
        "analysis": "P6_FULL_ROSBAG_DIRECTIONAL_INTEGRITY",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "counts": {"geometry": geometry_count, "integrity": integrity_count},
        "metrics": {
            "condition_number_min": min(condition_numbers, default=None),
            "condition_number_max": max(condition_numbers, default=None),
            "weak_pl_min_m": min(weak_pl_values, default=None),
            "weak_pl_max_m": max(weak_pl_values, default=None),
            "axis_pl_min_m": axes.min(axis=0).tolist() if len(axes) else None,
            "axis_pl_max_m": axes.max(axis=0).tolist() if len(axes) else None,
            "effective_points_min": min(effective_points, default=None),
            "effective_points_max": max(effective_points, default=None),
            "weak_axis_dominance_counts_xyz": weak_axis_counts.tolist(),
            "minimum_integrity_covariance_eigenvalue": minimum_covariance_eigenvalue,
            "maximum_pl_formula_error": maximum_formula_error,
        },
    }
    arguments.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

