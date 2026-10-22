#!/usr/bin/env python3
"""Read-only source, receipt-count and shadow-graph audit after LIVO replay."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy

from audit_livo_processing import audit_processing


def read(path):
    return json.loads(path.read_text())


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def audit(run):
    source = read(run / "sensor-index.json")
    before = read(run / "source-bag.before.json")
    after = [{"path": item["path"], "bytes": Path(item["path"]).stat().st_size,
              "sha256": sha(Path(item["path"]))} for item in before]
    checks = {"raw_bag_unchanged_after_both_arms": before == after}
    protocol = read(run / 'configuration/research_livo_replay_gate.json')
    checks['input_matches_frozen_protocol'] = (
        source['bag'] == protocol['source_bag'] and
        source['camera_keep_every_n'] == protocol['camera_keep_every_n'])
    source_stamps = read(run / 'source-stamps.json')
    checks['source_stamps_derived_from_index'] = source_stamps == {
        label + '_stamps_s': [row['stamp_ns'] * 1e-9 for row in source['entries'] if row['sensor'] == sensor]
        for sensor, label in (('front', 'front_camera'), ('lidar', 'lidar'), ('imu', 'imu'))}
    # Reconstruct evaluator truth directly from the immutable frozen databases;
    # neither a modified truth sidecar nor a stale evaluator PASS can certify ATE.
    from replay_livo_inputs import connect
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    original_truth = []
    for file in source['raw_files']:
        with connect(Path(file['file'])) as database:
            for (blob,) in database.execute("select m.data from messages m join topics t on m.topic_id=t.id "
                    "where t.name='/xq/eval/p4/ground_truth' order by m.timestamp"):
                message = deserialize_message(blob, Odometry)
                p, q = message.pose.pose.position, message.pose.pose.orientation
                original_truth.append({'stamp_s': (message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec) * 1e-9,
                    'position': [p.x, p.y, p.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w],
                    'frame_id': message.header.frame_id, 'child_frame_id': message.child_frame_id})
    original_truth.sort(key=lambda row: row['stamp_s'])
    checks['evaluator_truth_equals_original_bag'] = original_truth == [json.loads(line) for line in (run / 'truth.jsonl').read_text().splitlines()]
    arms = {}
    for arm in ("arm_livo", "arm_no_visual"):
        publisher = read(run / arm / "publisher-result.json")
        ingress = read(run / arm / "ingress-result.json")
        graph = read(run / arm / "input-graph.json")
        graph_after = read(run / arm / "input-graph.after.json")
        counts = publisher["published_counts"]
        checks[arm + ".all_frozen_inputs_published"] = counts == source["published_sensor_counts"]
        checks[arm + ".publisher_succeeded"] = publisher["status"] == "PASS"
        checks[arm + '.replay_rate_matches_frozen_protocol'] = publisher['replay_rate'] == protocol['replay_rate']
        checks[arm + ".snapshot_model_explicit"] = ingress["source_lidar_type"] == 100 and ingress["snapshot_zero_within_cloud_offsets"] is True
        for stream, key in (("image", "front"), ("lidar", "lidar"), ("imu", "imu")):
            items = [row for row in source["entries"] if row["sensor"] == key]
            received = ingress["sensors"][stream]
            checks[arm + "." + key + "_complete_receipt_count"] = received["received"] == counts[key] > 0
            checks[arm + "." + key + "_header_endpoints"] = (
                abs(received["first_header_stamp_s"] - items[0]["stamp_ns"] * 1e-9) < 1e-9 and
                abs(received["last_header_stamp_s"] - items[-1]["stamp_ns"] * 1e-9) < 1e-9)
        forbidden = {"/tf", "/tf_static", "/xq/eval/p4/ground_truth", "/localization/odom"}
        checks[arm + ".isolated_shadow_graph"] = not any(
            'mavros' in name.split('/') or name in forbidden for name, _ in graph["topics"] + graph_after["topics"])
        for epoch, snapshot in (("before", graph), ("after", graph_after)):
            checks[arm + '.reliable_input_transport_' + epoch] = all(
                item['qos_reliability'] == 'RELIABLE' for endpoints in snapshot['input_endpoints'].values()
                for label in ('publishers', 'subscribers') for item in endpoints[label])
            allowed_subscriptions = {'/clock', '/parameter_events', '/impact/r2b2/input/front',
                                     '/impact/r2b2/input/lidar', '/impact/r2b2/input/imu'}
            checks[arm + '.estimator_subscription_whitelist_' + epoch] = all(
                name in allowed_subscriptions for name, _ in snapshot['estimator_subscriptions'])
            checks[arm + ".input_endpoint_whitelist_" + epoch] = all(
                [item["node_name"] for item in endpoints["publishers"]] == ["parameter_blackboard"] and
                [item["node_name"] for item in endpoints["subscribers"]] == ["impact_livo_replay"]
                for endpoints in snapshot["input_endpoints"].values()) and len(snapshot["input_endpoints"]) == 3
        arms[arm] = {"published_counts": counts, "received": ingress["sensors"],
                     "source_index_sha256": publisher["source_index_sha256"]}
        processing = audit_processing(run / arm, source)
        checks[arm + '.all_images_accounted_and_processing_complete'] = processing['status'] == 'PASS'
        arms[arm]['processing'] = processing
    checks["same_input_index_in_both_arms"] = arms["arm_livo"]["source_index_sha256"] == arms["arm_no_visual"]["source_index_sha256"] == sha(run / "sensor-index.json")
    helper = Path("/mnt/d/CODEX/impact/evidence/RESEARCH_R2/r2_suite_20261018T165613Z_320/archive_research_r2.py")
    baseline = Path("/mnt/d/CODEX/impact/runs/research_bootstrap_20261018/protected.before.json")
    api = runpy.run_path(str(helper))
    current, protected = api["compare_protected"](api["read_object"](baseline))
    checks["original_433_file_content_and_set_preserved"] = protected["status"] == "PASS"
    result = {"schema_version": 1, "status": "PASS" if all(checks.values()) else "FAIL",
              "checks": checks, "arms": arms, "protected_external_audit": protected,
              "raw_files_after": after,
              "scope": "Original bag hashes, complete ROS receipt counts, sampled shadow graph and original workspace files; not hardware timing"}
    (run / "artifact-audit.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run_dir)
    print(json.dumps({"status": result["status"], "checks": len(result["checks"])}))
    raise SystemExit(0 if result["status"] == "PASS" else 1)
