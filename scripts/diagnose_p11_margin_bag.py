#!/usr/bin/env python3
"""Offline decomposition of P11 Alert Limit minus Protection Level samples."""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
from pathlib import Path

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from xq_autonomy.p10_active_perception_node import _cloud_xyz
from xq_sim_interfaces.msg import DirectionalIntegrity


def stamp(message) -> float:
    return float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("database", type=Path)
    parser.add_argument("calibration", type=Path)
    args = parser.parse_args()
    calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
    k_alpha = max(float(value["k95"]) for value in calibration["directional"].values())
    connection = sqlite3.connect(str(args.database))
    topics = {
        name: topic_id
        for topic_id, name in connection.execute("SELECT id,name FROM topics")
    }
    required = {
        topics["/localization/odom"]: Odometry,
        topics["/integrity/directional"]: DirectionalIntegrity,
        topics["/xq/p5/cloud_map"]: PointCloud2,
    }
    odom = None
    integrity = None
    samples = []
    query = "SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp"
    for topic_id, _, raw in connection.execute(query):
        message_type = required.get(topic_id)
        if message_type is None:
            continue
        message = deserialize_message(raw, message_type)
        if message_type is Odometry:
            odom = message
            continue
        if message_type is DirectionalIntegrity:
            integrity = message
            continue
        if odom is None or integrity is None:
            continue
        points = _cloud_xyz(message)
        if len(points) == 0:
            continue
        position = np.asarray(
            (
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ),
            dtype=float,
        )
        deltas = points - position
        distance2 = np.einsum("ij,ij->i", deltas, deltas)
        index = int(np.argmin(distance2))
        clearance = math.sqrt(max(float(distance2[index]), 0.0))
        if clearance <= 1.0e-6:
            continue
        direction = deltas[index] / clearance
        covariance = np.asarray(integrity.integrity_covariance, dtype=float).reshape(3, 3)
        protection = k_alpha * math.sqrt(max(float(direction @ covariance @ direction), 0.0))
        velocity = odom.twist.twist.linear
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        alert = clearance - (0.35 + 0.10 + 0.10 + speed * 0.10 + 0.5 * 0.10 ** 2)
        samples.append(
            {
                "stamp_s": stamp(message),
                "estimate_xyz_m": position.tolist(),
                "nearest_point_xyz_m": points[index].tolist(),
                "nearest_direction": direction.tolist(),
                "clearance_m": clearance,
                "alert_limit_m": alert,
                "protection_level_m": protection,
                "margin_m": alert - protection,
                "integrity_covariance": covariance.reshape(-1).tolist(),
                "geometry_eigenvalues": list(integrity.geometry_eigenvalues),
            }
        )
    connection.close()
    ordered = sorted(samples, key=lambda item: item["margin_m"])
    print(json.dumps({"count": len(samples), "minimum_samples": ordered[:10]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
