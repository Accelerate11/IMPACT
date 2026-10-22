#!/usr/bin/env python3
"""Index immutable R2A sensors, then replay only their original acquisition data.

The index command exports truth for a SEPARATE offline evaluator. The publisher
reads only the sensor index and never loads/publishes truth, old odometry, TF,
commands or any AP message. Acquisition headers are not changed. A derived
monotone /clock follows those headers, independently of storage wall time.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time


SENSORS = {"/livox/imu": ("imu", "sensor_msgs/msg/Imu"),
           "/livox/lidar": ("lidar", "sensor_msgs/msg/PointCloud2"),
           "/impact/r2/front/image_raw": ("front", "sensor_msgs/msg/Image")}
TRUTH = "/xq/eval/p4/ground_truth"
INFO = "/impact/r2/front/camera_info"


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def connect(path):
    return sqlite3.connect(path.resolve().as_uri() + "?mode=ro&immutable=1", uri=True)


def prepare(bag, output, keep_every):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    if output.exists():
        raise ValueError("index output must be new")
    if type(keep_every) is not int or keep_every < 1:
        raise ValueError("positive camera decimation required")
    output.mkdir(parents=True)
    entries, truths, info, counts, hashes = [], [], None, Counter(), []
    classes = {name: get_message(kind) for name, (_, kind) in SENSORS.items()}
    classes[TRUTH] = get_message("nav_msgs/msg/Odometry")
    classes[INFO] = get_message("sensor_msgs/msg/CameraInfo")
    for file in sorted(bag.glob("*.db3")):
        hashes.append({"file": str(file), "size_bytes": file.stat().st_size, "sha256": digest(file)})
        with connect(file) as database:
            topics = {row[0]: (row[1], row[2]) for row in database.execute("select id,name,type from topics") if row[1] in classes}
            placeholders = ",".join("?" for _ in topics)
            for rowid, topic_id, blob in database.execute("select id,topic_id,data from messages where topic_id in (" + placeholders + ") order by timestamp", tuple(topics)):
                name, kind = topics[topic_id]
                message = deserialize_message(blob, classes[name])
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                if stamp <= 0:
                    raise ValueError("invalid original acquisition timestamp")
                counts[name] += 1
                if name == TRUTH:
                    p, q = message.pose.pose.position, message.pose.pose.orientation
                    truths.append({"stamp_s": stamp * 1e-9, "position": [p.x, p.y, p.z],
                                   "quaternion_xyzw": [q.x, q.y, q.z, q.w],
                                   "frame_id": message.header.frame_id, "child_frame_id": message.child_frame_id})
                    continue
                if name == INFO:
                    current = {"width": message.width, "height": message.height,
                               "k": list(message.k), "d": list(message.d), "distortion_model": message.distortion_model}
                    if info is not None and current != info:
                        raise ValueError("changing camera calibration in frozen replay")
                    info = current
                    continue
                if name == "/impact/r2/front/image_raw":
                    if (message.width, message.height, message.encoding, message.step, len(message.data)) != (320, 240, "rgb8", 960, 230400):
                        raise ValueError("unexpected camera pixel contract")
                    if (counts[name] - 1) % keep_every:
                        continue
                if name == "/livox/lidar":
                    fields = {field.name for field in message.fields}
                    if not {"x", "y", "z", "intensity"} <= fields or {"time", "t", "timestamp", "offset_time"} & fields:
                        raise ValueError("snapshot field model mismatch")
                entries.append({"file": str(file), "rowid": rowid, "sensor": SENSORS[name][0],
                                "stamp_ns": stamp, "frame_id": message.header.frame_id, "type": kind,
                                "payload_sha256": hashlib.sha256(blob).hexdigest()})
    priority = {"imu": 0, "lidar": 1, "front": 2}
    entries.sort(key=lambda row: (row["stamp_ns"], priority[row["sensor"]]))
    truths.sort(key=lambda row: row["stamp_s"])
    if not entries or not truths or info is None:
        raise ValueError("missing sensors, truth or camera model")
    expected_k = [277.128129211, 0, 159.5, 0, 277.128129211, 119.5, 0, 0, 1]
    if (info["width"], info["height"]) != (320, 240) or any(abs(a - b) > 1e-8 for a, b in zip(info["k"], expected_k)) or any(info["d"]):
        raise ValueError("camera model differs from frozen simulation contract")
    stamps = {label + "_stamps_s": [row["stamp_ns"] * 1e-9 for row in entries if row["sensor"] == key]
              for key, label in (("front", "front_camera"), ("lidar", "lidar"), ("imu", "imu"))}
    for key, times in stamps.items():
        if not times or any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError("nonmonotone or missing " + key)
    source = {"schema_version": 1, "input_origin": "R2A_simulation_replay", "bag": str(bag),
              "camera_keep_every_n": keep_every, "source_counts": dict(counts),
              "published_sensor_counts": dict(Counter(row["sensor"] for row in entries)),
              "camera_info": info, "raw_files": hashes, "entries": entries,
              "clock_policy": "derived simulation /clock from original acquisition headers; no wall-time substitution"}
    write_json(output / "sensor-index.json", source)
    write_json(output / "source-stamps.json", stamps)
    with (output / "truth.jsonl").open("w") as stream:
        for row in truths:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    return {"status": "PASS", "counts": source["published_sensor_counts"], "truth_samples": len(truths)}


def publish(index_path, output, rate):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message
    if not math.isfinite(rate) or not 0 < rate <= 1:
        raise ValueError("replay rate must be in (0,1]")
    source = json.loads(index_path.read_text())
    bag = Path(source["bag"]).resolve()
    allowed_files = {str(Path(row["file"]).resolve()) for row in source["raw_files"]}
    if not allowed_files or any(Path(file).parent != bag for file in allowed_files):
        raise ValueError("source index files must be the hashed bag databases")
    for row in source["entries"]:
        # Each indexed row must use one of the six exact canonical paths above.
        # Resolving that same path for every row adds thousands of filesystem
        # lookups before the ROS node exists, delaying the READY handshake.
        if row["file"] not in allowed_files:
            raise ValueError("indexed row is outside hashed source databases")
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("parameter_blackboard")
    camera = {"cam_model": "Pinhole", "cam_width": 320, "cam_height": 240, "scale": 1.0,
              "cam_fx": 277.128129211, "cam_fy": 277.128129211, "cam_cx": 159.5, "cam_cy": 119.5,
              "cam_d0": 0.0, "cam_d1": 0.0, "cam_d2": 0.0, "cam_d3": 0.0}
    for key, value in camera.items():
        node.declare_parameter(key, value)
    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
    publishers = {key: node.create_publisher(get_message(kind), "/impact/r2b2/input/" + key, qos)
                  for _, (key, kind) in SENSORS.items()}
    clock = node.create_publisher(Clock, "/clock", 100)
    write_json(output / "parameter-ready.json", {"status": "READY", "camera": camera})
    databases = {}
    def graph_snapshot():
        snapshot = {"nodes": node.get_node_names(), "topics": node.get_topic_names_and_types(),
                    "input_endpoints": {}, "estimator_subscriptions":
                    node.get_subscriber_names_and_types_by_node("impact_livo_replay", "/")}
        allowed_subscriptions = {"/clock", "/parameter_events"} | {
            "/impact/r2b2/input/" + key for key in publishers}
        if any(name not in allowed_subscriptions for name, _ in snapshot["estimator_subscriptions"]):
            raise RuntimeError("estimator subscribes outside sensor/clock/parameter whitelist")
        for key in publishers:
            topic = "/impact/r2b2/input/" + key
            endpoints = {}
            for label, function in (("publishers", node.get_publishers_info_by_topic),
                                    ("subscribers", node.get_subscriptions_info_by_topic)):
                endpoints[label] = [{"node_name": item.node_name, "node_namespace": item.node_namespace,
                                     "topic_type": item.topic_type,
                                     "qos_reliability": item.qos_profile.reliability.name,
                                     "qos_depth": item.qos_profile.depth} for item in function(topic)]
            if ([item["node_name"] for item in endpoints["publishers"]] != ["parameter_blackboard"] or
                [item["node_name"] for item in endpoints["subscribers"]] != ["impact_livo_replay"]):
                raise RuntimeError("unexpected sensor publisher/subscriber endpoints: " + key)
            if any(item['qos_reliability'] != 'RELIABLE'
                   for label in ('publishers', 'subscribers') for item in endpoints[label]):
                raise RuntimeError('offline replay requires reliable delivery at both endpoints')
            snapshot["input_endpoints"][key] = endpoints
        if any("mavros" in name.split("/") or name in {"/tf", "/tf_static", TRUTH, "/localization/odom"}
               for name, _ in snapshot["topics"]):
            raise RuntimeError("shadow graph contains an AP, TF, truth or old-state topic")
        return snapshot
    try:
        deadline = time.monotonic() + 90
        while not all(pub.get_subscription_count() >= 1 for pub in publishers.values()):
            if time.monotonic() >= deadline:
                raise TimeoutError("LIVO did not subscribe to the three inputs")
            rclpy.spin_once(node, timeout_sec=0.05)
        graph = graph_snapshot()
        write_json(output / "input-graph.json", graph)
        start, first_stamp = time.monotonic(), source["entries"][0]["stamp_ns"]
        counts, maximum_lag = Counter(), 0.0
        for row in source["entries"]:
            target = start + (row["stamp_ns"] - first_stamp) * 1e-9 / rate
            while True:
                remaining = target - time.monotonic()
                if remaining <= 0:
                    break
                rclpy.spin_once(node, timeout_sec=min(0.01, remaining))
            maximum_lag = max(maximum_lag, time.monotonic() - target)
            file = row["file"]
            if file not in databases:
                databases[file] = connect(Path(file))
            actual = databases[file].execute("select m.data,t.name,t.type from messages m join topics t on m.topic_id=t.id where m.id=?", (row["rowid"],)).fetchone()
            blob, name, kind = actual
            if name not in SENSORS or SENSORS[name] != (row["sensor"], row["type"]) or kind != row["type"]:
                raise ValueError("indexed row does not identify the declared sensor")
            if hashlib.sha256(blob).hexdigest() != row["payload_sha256"]:
                raise ValueError("indexed payload hash changed")
            header = deserialize_message(blob, get_message(kind)).header
            if header.stamp.sec * 1_000_000_000 + header.stamp.nanosec != row["stamp_ns"]:
                raise ValueError("indexed acquisition header changed")
            clock.publish(Clock(clock=Time(nanoseconds=row["stamp_ns"]).to_msg()))
            publishers[row["sensor"]].publish(bytes(blob))
            counts[row["sensor"]] += 1
            rclpy.spin_once(node, timeout_sec=0)
        # Let the estimator consume the final queue without advancing simulation
        # beyond the last original sensor sample.
        end = time.monotonic() + 2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        write_json(output / "input-graph.after.json", graph_snapshot())
        result = {"status": "PASS", "published_counts": dict(counts), "replay_rate": rate,
                  "maximum_wall_schedule_lag_s": maximum_lag, "source_index_sha256": digest(index_path),
                  "input_origin": source["input_origin"], "hardware_timing_claim": False,
                  "original_acquisition_headers_preserved": True, "truth_or_old_state_published": False}
        write_json(output / "publisher-result.json", result)
        return result
    finally:
        for database in databases.values():
            database.close()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("prepare", "publish"))
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--index", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--camera-keep-every", type=int, default=3)
    parser.add_argument("--rate", type=float, default=0.5)
    args = parser.parse_args()
    result = prepare(args.bag, args.output, args.camera_keep_every) if args.mode == "prepare" else publish(args.index, args.output, args.rate)
    print(json.dumps(result))


if __name__ == "__main__":
    main()
