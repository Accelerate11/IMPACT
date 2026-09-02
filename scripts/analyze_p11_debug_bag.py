#!/usr/bin/env python3
"""Extract P11 candidate clearance diagnostics directly from a rosbag."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    arguments = parser.parse_args()
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(arguments.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/integrity/exploration_debug":
            continue
        payload = json.loads(deserialize_message(data, String).data)
        rows.append(
            {
                "batch_id": payload.get("batch_id"),
                "selected": payload.get("selected"),
                "predicted_minimum_margins": dict(
                    zip(
                        payload.get("candidate_names", []),
                        payload.get("predicted_minimum_margins", []),
                    )
                ),
                "alert_diagnostics": payload.get("alert_diagnostics", []),
            }
        )
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
