#!/usr/bin/env python3
"""Read-only diagnostics for a P5 rosbag after a failed/successful run."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def cloud_xyz(message):
    fields = {field.name: field for field in message.fields}
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({
        "names": ("x", "y", "z"),
        "formats": (endian + "f4", endian + "f4", endian + "f4"),
        "offsets": tuple(fields[name].offset for name in ("x", "y", "z")),
        "itemsize": message.point_step,
    })
    data = np.frombuffer(message.data, dtype=dtype, count=message.width * message.height)
    points = np.column_stack((data["x"], data["y"], data["z"]))
    return points[np.isfinite(points).all(axis=1)]


def make_p4_evaluation_map(resolution=0.05, vehicle_radius=0.35):
    """Recreate the evaluator map used by the recorded P4-room run."""
    origin = -9.0
    size = int(round(18.0 / resolution))
    centers = origin + (np.arange(size) + 0.5) * resolution
    x, y = np.meshgrid(centers, centers, indexing="ij")
    occupied = (np.abs(x) >= 8.9) | (np.abs(y) >= 8.9)
    occupied |= (x + 5.0) ** 2 + (y - 5.0) ** 2 <= 0.35**2
    occupied |= (x - 5.0) ** 2 + (y + 5.0) ** 2 <= 0.28**2
    yaw = 0.25
    c, s = math.cos(yaw), math.sin(yaw)
    local_x = c * (x - 5.8) + s * (y - 4.4)
    local_y = -s * (x - 5.8) + c * (y - 4.4)
    occupied |= (np.abs(local_x) <= 0.6) & (np.abs(local_y) <= 1.0)
    inflated = occupied.copy()
    radius = int(math.ceil(vehicle_radius / resolution))
    indices = np.argwhere(occupied)
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            shifted = indices + np.array((dx, dy))
            valid = (
                (shifted[:, 0] >= 0)
                & (shifted[:, 0] < size)
                & (shifted[:, 1] >= 0)
                & (shifted[:, 1] < size)
            )
            inflated[shifted[valid, 0], shifted[valid, 1]] = True
    return origin, resolution, occupied, inflated


def diagnose_truth(truth_samples):
    if not truth_samples:
        return
    origin, resolution, occupied, inflated = make_p4_evaluation_map()
    obstacle_indices = np.argwhere(occupied)
    rows = []
    for stamp_ns, xyz in truth_samples:
        ix = int(math.floor((xyz[0] - origin) / resolution))
        iy = int(math.floor((xyz[1] - origin) / resolution))
        in_bounds = 0 <= ix < inflated.shape[0] and 0 <= iy < inflated.shape[1]
        collision = not in_bounds or bool(inflated[ix, iy])
        clearance = None
        if in_bounds:
            delta = (obstacle_indices - np.array((ix, iy))) * resolution
            clearance = float(np.sqrt(np.min(np.sum(delta * delta, axis=1)))) - 0.35
        rows.append((stamp_ns, xyz, collision, clearance))
    collisions = [row for row in rows if row[2]]
    print("truth_airborne_samples", len(rows))
    print("truth_start_xyz", rows[0][1].tolist())
    print("truth_end_xyz", rows[-1][1].tolist())
    print("truth_xy_bounds", [
        float(min(row[1][0] for row in rows)), float(max(row[1][0] for row in rows)),
        float(min(row[1][1] for row in rows)), float(max(row[1][1] for row in rows)),
    ])
    print("truth_collision_samples", len(collisions))
    if collisions:
        print("truth_first_collision_xyz", collisions[0][1].tolist())
        print("truth_last_collision_xyz", collisions[-1][1].tolist())
        print("truth_collision_xy_bounds", [
            float(min(row[1][0] for row in collisions)), float(max(row[1][0] for row in collisions)),
            float(min(row[1][1] for row in collisions)), float(max(row[1][1] for row in collisions)),
        ])
        # Print a sparse set of points across the collision interval.
        sample_indices = np.linspace(0, len(collisions) - 1, min(12, len(collisions)), dtype=int)
        print("truth_collision_trace")
        for index in sample_indices:
            stamp_ns, xyz, _, clearance = collisions[index]
            print(stamp_ns, xyz.tolist(), clearance)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    first_cloud = None
    last_grid = None
    first_odom = None
    armed = False
    truth_samples = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/livox/lidar" and first_cloud is None:
            first_cloud = deserialize_message(data, get_message(types[topic]))
        elif topic == "/localization/odom" and first_odom is None:
            first_odom = deserialize_message(data, get_message(types[topic]))
        elif topic == "/xq/p5/navigation_map":
            last_grid = deserialize_message(data, get_message(types[topic]))
        elif topic == "/uav1/mavros/state":
            armed = bool(deserialize_message(data, get_message(types[topic])).armed)
        elif topic == "/xq/eval/p4/ground_truth":
            truth = deserialize_message(data, get_message(types[topic]))
            p = truth.pose.pose.position
            xyz = np.array((p.x, p.y, p.z), dtype=np.float64)
            if armed and xyz[2] >= 0.5 and np.isfinite(xyz).all():
                truth_samples.append((_, xyz))
    if first_cloud is not None:
        points = cloud_xyz(first_cloud)
        ranges = np.linalg.norm(points, axis=1)
        print("cloud_frame", first_cloud.header.frame_id)
        print("cloud_points", len(points))
        print("range_percentiles_m", np.percentile(ranges, (0, 1, 5, 25, 50, 95, 100)).tolist())
        print("xyz_min", np.min(points, axis=0).tolist())
        print("xyz_max", np.max(points, axis=0).tolist())
    if first_odom is not None:
        p = first_odom.pose.pose.position
        q = first_odom.pose.pose.orientation
        print("odom_frame", first_odom.header.frame_id)
        print("odom_xyz", [p.x, p.y, p.z])
        print("odom_q_wxyz", [q.w, q.x, q.y, q.z])
    if last_grid is not None:
        values = np.asarray(last_grid.data, dtype=np.int16).reshape(
            (last_grid.info.height, last_grid.info.width)
        ).T
        free = values == 0
        occupied = values == 100
        origin = last_grid.info.origin.position
        resolution = last_grid.info.resolution
        sx = int(math.floor((0.0 - origin.x) / resolution))
        sy = int(math.floor((0.0 - origin.y) / resolution))
        radius = 12
        local = values[sx - radius:sx + radius + 1, sy - radius:sy + radius + 1]
        print("grid_counts", {"unknown": int(np.count_nonzero(values < 0)), "free": int(np.count_nonzero(free)), "occupied": int(np.count_nonzero(occupied))})
        print("grid_origin_index", [sx, sy])
        print("grid_local_occupied", int(np.count_nonzero(local == 100)))
        print("grid_local_free", int(np.count_nonzero(local == 0)))
        for row in local.T[::-1]:
            print("".join("#" if value == 100 else "." if value == 0 else "?" for value in row))
    diagnose_truth(truth_samples)


if __name__ == "__main__":
    main()
