#!/usr/bin/env python3
"""Diagnose whether a flight stall is commanded, localized, or physical."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
    }
    odom = []
    truth = []
    commands = []
    phases = []
    map_status = []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in topic_types:
            continue
        stamp_s = 1.0e-9 * stamp_ns
        if topic in {
            "/localization/odom",
            "/xq/eval/agent_01/ground_truth",
        }:
            message = deserialize_message(data, topic_types[topic])
            p = message.pose.pose.position
            target = odom if topic == "/localization/odom" else truth
            target.append((stamp_s, float(p.x), float(p.y), float(p.z)))
        elif topic == "/xq/p3/cmd_vel":
            message = deserialize_message(data, topic_types[topic])
            commands.append(
                (
                    stamp_s,
                    float(message.linear.x),
                    float(message.linear.y),
                    float(message.linear.z),
                )
            )
        elif topic in {"/xq/p12/flight_status", "/mapping/p12/status"}:
            message = deserialize_message(data, topic_types[topic])
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                continue
            if topic == "/xq/p12/flight_status":
                phases.append((stamp_s, payload))
            else:
                map_status.append((stamp_s, payload))

    def rows(values):
        return np.asarray(values, dtype=float) if values else np.empty((0, 4))

    odom_array = rows(odom)
    truth_array = rows(truth)
    command_array = rows(commands)
    fail_stamp = next(
        (
            stamp
            for stamp, payload in phases
            if str(payload.get("phase", "")).startswith("FAIL_CLOSED")
        ),
        (odom_array[-1, 0] if len(odom_array) else math.nan),
    )

    def state_at(array: np.ndarray, stamp: float):
        if not len(array) or not math.isfinite(stamp):
            return None
        row = array[int(np.argmin(np.abs(array[:, 0] - stamp)))]
        return {"stamp_s": row[0], "xyz": row[1:].tolist()}

    windows = {}
    for seconds in (90.0, 30.0, 10.0):
        lower = fail_stamp - seconds
        position_window = odom_array[
            (odom_array[:, 0] >= lower) & (odom_array[:, 0] <= fail_stamp)
        ]
        command_window = command_array[
            (command_array[:, 0] >= lower) & (command_array[:, 0] <= fail_stamp)
        ]
        command_norm = (
            np.linalg.norm(command_window[:, 1:], axis=1)
            if len(command_window)
            else np.empty(0)
        )
        windows[str(int(seconds))] = {
            "position_delta_m": (
                float(np.linalg.norm(position_window[-1, 1:] - position_window[0, 1:]))
                if len(position_window) >= 2
                else None
            ),
            "forward_delta_m": (
                float(position_window[-1, 1] - position_window[0, 1])
                if len(position_window) >= 2
                else None
            ),
            "command_sample_count": int(len(command_norm)),
            "command_speed_mean_mps": (
                float(np.mean(command_norm)) if len(command_norm) else None
            ),
            "command_speed_max_mps": (
                float(np.max(command_norm)) if len(command_norm) else None
            ),
        }

    result = {
        "schema": "impact.p15.stall-diagnostic.v1",
        "bag": str(args.bag.resolve()),
        "fail_stamp_s": fail_stamp,
        "phases": [
            {
                "bag_stamp_s": stamp,
                "phase": payload.get("phase"),
                "elapsed_s": payload.get("elapsed_s"),
                "current_position_x_m": payload.get("current_position_x_m"),
            }
            for stamp, payload in phases
        ],
        "odom_at_fail": state_at(odom_array, fail_stamp),
        "ground_truth_at_fail": state_at(truth_array, fail_stamp),
        "windows_before_fail_s": windows,
        "map_blocked_fraction": (
            float(
                np.mean(
                    [
                        payload.get("forward_path_blocked") is True
                        for _, payload in map_status
                    ]
                )
            )
            if map_status
            else None
        ),
        "raw_blocked_fraction": (
            float(
                np.mean(
                    [payload.get("raw_path_blocked") is True for _, payload in map_status]
                )
            )
            if map_status
            else None
        ),
    }
    encoded = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")


if __name__ == "__main__":
    main()
