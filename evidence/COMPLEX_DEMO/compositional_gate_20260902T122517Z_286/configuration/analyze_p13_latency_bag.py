#!/usr/bin/env python3
"""Decompose recorded P13 latency traces without replaying the bag."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def nearest_rank(values: list[float], percentile: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value) and value >= 0.0)
    if not finite:
        return math.nan
    index = max(0, math.ceil(percentile * len(finite)) - 1)
    return finite[index]


def summary(values: list[float]) -> dict[str, float | int]:
    finite = [value for value in values if math.isfinite(value) and value >= 0.0]
    return {
        "count": len(finite),
        "p50_ms": nearest_rank(finite, 0.50),
        "p95_ms": nearest_rank(finite, 0.95),
        "p99_ms": nearest_rank(finite, 0.99),
        "maximum_ms": max(finite, default=math.nan),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    arguments = parser.parse_args()

    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(arguments.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    samples: dict[str, list[float]] = {
        "sensor_age_at_receive": [],
        "receive_to_localization": [],
        "localization_to_map": [],
        "map_to_planner_trigger": [],
        "planner_processing": [],
        "planner_to_trajectory_certification": [],
        "certification_to_command": [],
        "receive_to_command": [],
        "end_to_end": [],
    }
    topic = "/integrity/p13/latency_trace"
    while reader.has_next():
        current_topic, data, _ = reader.read_next()
        if current_topic != topic:
            continue
        trace = json.loads(deserialize_message(data, String).data)
        receive = int(trace["receive_timestamp_ns"])
        localization = int(trace["localization_done_ns"])
        map_done = int(trace["map_done_ns"])
        planner_trigger = int(trace["planner_trigger_ns"])
        planner_done = int(trace["planner_done_ns"])
        certified = int(trace["trajectory_certified_ns"])
        command = int(trace["command_sent_ns"])
        samples["sensor_age_at_receive"].append(
            float(trace["sensor_age_at_receive_ms"])
        )
        samples["receive_to_localization"].append(1.0e-6 * (localization - receive))
        samples["localization_to_map"].append(1.0e-6 * (map_done - localization))
        samples["map_to_planner_trigger"].append(1.0e-6 * (planner_trigger - map_done))
        samples["planner_processing"].append(float(trace["planner_processing_ms"]))
        samples["planner_to_trajectory_certification"].append(
            1.0e-6 * (certified - planner_done)
        )
        samples["certification_to_command"].append(1.0e-6 * (command - certified))
        samples["receive_to_command"].append(float(trace["receive_to_command_ms"]))
        samples["end_to_end"].append(float(trace["end_to_end_latency_ms"]))

    print(json.dumps({name: summary(values) for name, values in samples.items()}, indent=2))


if __name__ == "__main__":
    main()
