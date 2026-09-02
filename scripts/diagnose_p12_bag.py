#!/usr/bin/env python3
"""Read a P12 rosbag directly and diagnose dynamic-cloud/path alignment."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def cloud_xyz(message):
    offsets = {field.name: field.offset for field in message.fields}
    dtype = np.dtype({
        "names": ("x", "y", "z"),
        "formats": ("<f4", "<f4", "<f4"),
        "offsets": (offsets["x"], offsets["y"], offsets["z"]),
        "itemsize": message.point_step,
    })
    values = np.frombuffer(message.data, dtype=dtype, count=message.width * message.height)
    return np.column_stack((values["x"], values["y"], values["z"])).astype(float)


def main():
    bag = Path(sys.argv[1]).resolve()
    reader_type = getattr(rosbag2_py, "SequentialCompressionReader", rosbag2_py.SequentialReader)
    reader = reader_type()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {
        "/localization/odom", "/mapping/p12/dynamic_voxels",
        "/mapping/p12/static_voxels", "/livox/lidar",
        "/xq/eval/p12/obstacle_state",
    }
    classes = {topic: get_message(types[topic]) for topic in wanted}
    odom = None
    obstacle = {}
    best = None
    active_samples = []
    layer_counts = {"dynamic": 0, "static": 0, "raw_lidar": 0}
    raw_sweep_best = None
    raw_obstacle_best = None
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in wanted:
            continue
        message = deserialize_message(raw, classes[topic])
        if topic == "/localization/odom":
            p = message.pose.pose.position
            odom = np.asarray((p.x, p.y, p.z), dtype=float)
        elif topic == "/xq/eval/p12/obstacle_state":
            obstacle = json.loads(message.data)
        elif odom is not None:
            points = cloud_xyz(message)
            if not len(points):
                continue
            if obstacle.get("state") == "BLOCKING":
                # In this world, initial vehicle x=-12, obstacle x=-4.5 and
                # initial local odom is zero, so obstacle center is x=7.5.
                expected = np.asarray((7.5, 0.0, -0.2))
                compare_points = points + odom if topic == "/livox/lidar" else points
                count = int(np.count_nonzero(np.linalg.norm(compare_points - expected, axis=1) <= 1.25))
                layer = (
                    "raw_lidar" if topic == "/livox/lidar"
                    else "static" if topic.endswith("static_voxels") else "dynamic"
                )
                layer_counts[layer] = max(layer_counts[layer], count)
                if topic == "/livox/lidar":
                    along = points[:, 0]
                    lateral = np.linalg.norm(points[:, 1:3], axis=1)
                    mask = (along >= 0.2) & (along <= 4.0)
                    if np.any(mask):
                        index = np.flatnonzero(mask)[int(np.argmin(lateral[mask]))]
                        candidate = {
                            "stamp_s": obstacle.get("stamp_s"),
                            "point_sensor": points[index].tolist(),
                            "lateral_m": float(lateral[index]),
                            "along_m": float(along[index]),
                        }
                        if raw_sweep_best is None or candidate["lateral_m"] < raw_sweep_best["lateral_m"]:
                            raw_sweep_best = candidate
                    expected_sensor = expected - odom
                    distances = np.linalg.norm(points - expected_sensor, axis=1)
                    nearest_index = int(np.argmin(distances))
                    obstacle_candidate = {
                        "stamp_s": obstacle.get("stamp_s"),
                        "odom": odom.tolist(),
                        "expected_sensor": expected_sensor.tolist(),
                        "point_sensor": points[nearest_index].tolist(),
                        "error_m": float(distances[nearest_index]),
                        "along_m": float(points[nearest_index, 0]),
                        "lateral_m": float(np.linalg.norm(points[nearest_index, 1:3])),
                    }
                    if (raw_obstacle_best is None
                            or obstacle_candidate["error_m"] < raw_obstacle_best["error_m"]):
                        raw_obstacle_best = obstacle_candidate
            if topic.endswith("static_voxels") or topic == "/livox/lidar":
                continue
            relative = points - odom
            along = relative[:, 0]
            lateral3 = np.linalg.norm(relative[:, 1:3], axis=1)
            mask = (along >= 0.2) & (along <= 6.0)
            candidate = None
            if np.any(mask):
                indices = np.flatnonzero(mask)
                local = indices[int(np.argmin(lateral3[mask]))]
                candidate = {
                    "stamp_s": message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec,
                    "odom": odom.tolist(),
                    "point": points[local].tolist(),
                    "along_m": float(along[local]),
                    "lateral3_m": float(lateral3[local]),
                    "count": len(points),
                    "obstacle": obstacle,
                }
                if best is None or candidate["lateral3_m"] < best["lateral3_m"]:
                    best = candidate
            if obstacle.get("passage_occupied") is True and candidate is not None:
                active_samples.append(candidate)
    summary = {
        "best_all": best,
        "active_sample_count": len(active_samples),
        "best_while_passage_occupied": min(
            active_samples, key=lambda item: item["lateral3_m"], default=None
        ),
        "first_active": active_samples[0] if active_samples else None,
        "last_active": active_samples[-1] if active_samples else None,
        "obstacle_local_layer_peak_counts": layer_counts,
        "raw_sweep_best_while_blocking": raw_sweep_best,
        "raw_obstacle_best_while_blocking": raw_obstacle_best,
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
