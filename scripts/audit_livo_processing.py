#!/usr/bin/env python3
"""Check complete image processing against the original replay acquisition index.

Callback receipt counts alone do not establish that queued VIO work completed.
This audit accounts for every source image and joins each processed epoch to both
flushed diagnostic streams. It never starts ROS or changes replay input files.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import yaml


CLASSES = ("processed_postVIO", "startup_discard", "imu_initialization", "terminal_uncovered")
LABELS = {"lidar": "lidar", "imu": "imu", "image": "front"}


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _integer(value, *, minimum=0):
    return type(value) is int and value >= minimum


def _read(path, evidence, *, lines=False):
    before = path.stat()
    data = path.read_bytes()
    after = path.stat()
    if (before.st_ino, before.st_mtime_ns, before.st_size) != (after.st_ino, after.st_mtime_ns, after.st_size):
        raise ValueError("input changed during audit: " + str(path))
    evidence[str(path)] = {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}

    def number(value):
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError("non-finite JSON number")
        return parsed

    def constant(value):
        raise ValueError("non-finite JSON constant: " + value)

    def decode(text):
        return json.loads(text, parse_float=number, parse_constant=constant)

    values = [decode(row) for row in data.decode().splitlines() if row.strip()] if lines else decode(data.decode())
    if lines and any(not isinstance(row, dict) for row in values):
        raise ValueError("JSONL records must be objects")
    return values


def _near_values(actual, expected):
    return (isinstance(actual, (list, tuple)) and len(actual) == len(expected)
            and all(_finite(a) and abs(a - b) <= 1e-12 for a, b in zip(actual, expected)))


def _ns(row):
    stamp = row.get("stamp_s")
    if not _finite(stamp) or stamp <= 0:
        raise ValueError("diagnostic acquisition stamp is invalid")
    return int(round(stamp * 1_000_000_000))


def audit_processing(arm_dir, source_index):
    """Pure read-only API used by audit_livo_replay_artifacts.audit."""
    arm_dir = Path(arm_dir)
    checks, errors, evidence = {}, [], {}
    result = {"schema_version": 1, "status": "FAIL", "arm": arm_dir.name,
              "checks": checks, "errors": errors, "input_evidence": evidence,
              "scope": "Complete original image accounting and frozen simulation geometry; not hardware timing or accuracy superiority"}

    def check(key, condition):
        checks[key] = bool(condition)
        if not condition:
            errors.append(key)

    try:
        if arm_dir.name not in ("arm_livo", "arm_no_visual"):
            raise ValueError("unexpected replay arm")
        if not isinstance(source_index, dict) or source_index.get("schema_version") != 1 or source_index.get("input_origin") != "R2A_simulation_replay":
            raise ValueError("wrong source acquisition index schema/origin")
        entries = source_index.get("entries")
        if not isinstance(entries, list) or not entries:
            raise ValueError("missing source acquisition entries")
        streams = {label: [] for label in LABELS.values()}
        previous_key = None
        priority = {"imu": 0, "lidar": 1, "front": 2}
        for row in entries:
            if not isinstance(row, dict) or row.get("sensor") not in streams or not _integer(row.get("stamp_ns"), minimum=1):
                raise ValueError("invalid source acquisition entry")
            key = (row["stamp_ns"], priority[row["sensor"]])
            if previous_key is not None and key <= previous_key:
                raise ValueError("source acquisition sequence is not strictly sorted")
            previous_key = key
            streams[row["sensor"]].append(row["stamp_ns"])
        if any(not times or any(b <= a for a, b in zip(times, times[1:])) for times in streams.values()):
            raise ValueError("missing, repeated or backward per-sensor acquisition stamps")
        expected_counts = {name: len(streams[key]) for name, key in LABELS.items()}
        check("source_counts_match_entries", source_index.get("published_sensor_counts") ==
              {key: len(values) for key, values in streams.items()})
        protocol = _read(arm_dir.parent / "configuration/research_livo_replay_gate.json", evidence)
        check("frozen_source_and_decimation", protocol.get("input_origin") == source_index["input_origin"]
              and protocol.get("camera_keep_every_n") == source_index.get("camera_keep_every_n")
              and Path(protocol.get("source_bag", "")).resolve() == Path(source_index.get("bag", ".")).resolve())
        check("frozen_body_reference", protocol.get("body_frame") == "livox_imu"
              and protocol.get("truth_body_frame") == "xq_base_link"
              and _near_values(protocol.get("truth_body_from_estimator_body", {}).get("rotation_xyzw"), [0, 0, 0, 1])
              and _near_values(protocol.get("truth_body_from_estimator_body", {}).get("translation_m"), [0, 0, 0]))
        params_path = arm_dir.parent / "configuration/r2_snapshot.yaml"
        yaml_bytes = params_path.read_bytes()
        evidence[str(params_path)] = {"bytes": len(yaml_bytes), "sha256": hashlib.sha256(yaml_bytes).hexdigest()}
        params = yaml.safe_load(yaml_bytes)["/**"]["ros__parameters"]
        extrinsics = params.get("extrin_calib", {})
        check("frozen_imu_from_lidar", _near_values(extrinsics.get("extrinsic_T"), [0.04, 0, 0.12])
              and _near_values(extrinsics.get("extrinsic_R"), [1, 0, 0, 0, 1, 0, 0, 0, 1]))
        # R_camera_lidar = R_body_camera.T; t_camera_lidar =
        # R_body_camera.T * ([.04,0,.12] - [.12,0,.03]).
        check("frozen_camera_from_lidar", _near_values(extrinsics.get("Rcl"), [0, -1, 0, 0, 0, -1, 1, 0, 0])
              and _near_values(extrinsics.get("Pcl"), [0, -0.09, -0.08]))
        offsets = params.get("time_offset", {})
        check("no_acquisition_restamping_offsets", all(_finite(offsets.get(name)) and offsets[name] == 0.0
              for name in ("img_time_offset", "imu_time_offset", "lidar_time_offset", "exposure_time_init")))
        check("frozen_sensor_input_topics", all(params.get("common", {}).get(key) == topic for key, topic in (
            ("img_topic", "/impact/r2b2/input/front"), ("lid_topic", "/impact/r2b2/input/lidar"),
            ("imu_topic", "/impact/r2b2/input/imu"))))
        check("snapshot_mode_and_body_frame", params.get("preprocess", {}).get("lidar_type") == 100
              and params.get("research", {}).get("body_frame") == "livox_imu"
              and params.get("research", {}).get("shadow_only") is True
              and params.get("common", {}).get("ros_driver_bug_fix") is False)
        camera = source_index.get("camera_info", {})
        check("frozen_camera_model", camera.get("width") == 320 and camera.get("height") == 240
              and camera.get("distortion_model") == "plumb_bob"
              and _near_values(camera.get("k"), [277.128129211, 0, 159.5, 0, 277.128129211, 119.5, 0, 0, 1])
              and isinstance(camera.get("d"), list) and len(camera["d"]) >= 4
              and all(_finite(value) and value == 0.0 for value in camera["d"]))

        ledger = _read(arm_dir / "image-epochs.jsonl", evidence, lines=True)
        completion = _read(arm_dir / "processing-result.json", evidence)
        diagnostics = _read(arm_dir / "visual-updates.jsonl", evidence, lines=True)
        odometry = _read(arm_dir / "odometry.jsonl", evidence, lines=True)
        image_order = {stamp: index + 1 for index, stamp in enumerate(streams["front"])}
        first_lidar = streams["lidar"][0]
        last_coverage = min(streams["lidar"][-1], streams["imu"][-1])
        eligible = [stamp for stamp in streams["front"] if stamp >= first_lidar + 10_000 and stamp <= last_coverage]
        if not eligible:
            raise ValueError("no image has the frozen LIVO LiDAR/IMU coverage")
        target = eligible[-1]
        by_stamp, processed, classifications = {}, {}, Counter()
        for row in ledger:
            stamp = row.get("header_stamp_ns")
            if row.get("schema_version") != 1 or row.get("event") != "image_epoch" or not _integer(stamp, minimum=1) or stamp not in image_order or stamp in by_stamp:
                raise ValueError("duplicate, foreign or malformed image ledger acquisition stamp")
            if row.get("image_sequence_id") != image_order[stamp] or type(row.get("image_sequence_id")) is not int:
                raise ValueError("image callback sequence does not identify the original source image")
            classification = row.get("classification")
            if classification not in CLASSES or not _integer(row.get("lio_points")) or type(row.get("odom_published")) is not bool or not isinstance(row.get("reason"), str):
                raise ValueError("invalid image classification/point count/publication flags")
            if classification == "processed_postVIO":
                if stamp not in eligible or not _integer(row.get("epoch_index"), minimum=1) or row["odom_published"] is not True or row["lio_points"] <= 0:
                    raise ValueError("processed image lacks a covered VIO epoch, LiDAR points or odometry")
                processed[stamp] = row
            else:
                if row.get("epoch_index") is not None or row["odom_published"] is not False or not row["reason"]:
                    raise ValueError("unprocessed image invented an epoch/odometry or lacks a reason")
                if classification == "startup_discard" and stamp >= first_lidar + 10_000:
                    raise ValueError("startup discard conceals an eligible image")
                if classification == "imu_initialization" and stamp not in eligible:
                    raise ValueError("IMU initialization classification is outside covered image range")
                if classification == "terminal_uncovered" and stamp <= last_coverage:
                    raise ValueError("terminal classification conceals a covered image")
            classifications[classification] += 1
            by_stamp[stamp] = row
        check("every_source_image_accounted_once", set(by_stamp) == set(image_order) and len(ledger) == len(image_order))
        check("last_eligible_image_processed", target in processed and processed and max(processed) == target)
        post_started = False
        for stamp in eligible:
            row = by_stamp.get(stamp)
            if row is None:
                continue
            if row["classification"] == "processed_postVIO":
                post_started = True
            elif row["classification"] == "imu_initialization" and post_started:
                raise ValueError("initialization restarted after post-VIO output and concealed a covered image")
        ordered_processed = sorted(processed)
        check("post_vio_epochs_are_contiguous", [processed[stamp]["epoch_index"] for stamp in ordered_processed] == list(range(1, len(processed) + 1)))

        def match_rows(rows, event):
            seen = set()
            for row in rows:
                stamp = _ns(row)
                if row.get("schema_version") != 1 or row.get("event") != event or stamp not in processed or stamp in seen or row.get("epoch_index") != processed[stamp]["epoch_index"]:
                    raise ValueError("orphan, duplicate or mismatched " + event + " record")
                if row.get("visual_update_enabled") is not (arm_dir.name == "arm_livo"):
                    raise ValueError("diagnostic record has the wrong visual arm")
                if event == "visual_update":
                    if not _finite(row.get("image_stamp_s")) or int(round(row["image_stamp_s"] * 1e9)) != stamp or row.get("imu_initialized") is not True or row.get("accepted") is not True:
                        raise ValueError("processed diagnostic is not an accepted initialized source-image epoch")
                elif row.get("post_vio_epoch") is not True:
                    raise ValueError("odometry did not originate after VIO processing")
                seen.add(stamp)
            return seen == set(processed)

        check("all_processed_images_have_exact_diagnostics", match_rows(diagnostics, "visual_update"))
        check("all_processed_images_have_exact_odometry", match_rows(odometry, "odometry"))
        check("completion_status", completion.get("schema_version") == 1 and completion.get("status") == "COMPLETE" and completion.get("complete") is True)
        check("completion_all_flushed_and_accounted", all(completion.get(key) is True for key in (
            "diagnostics_flushed", "odometry_flushed", "ledger_flushed", "all_inputs_received", "all_image_messages_accounted")))
        check("completion_expected_and_received_counts", completion.get("expected_messages") == expected_counts and completion.get("received_messages") == expected_counts)
        check("completion_original_header_endpoints", completion.get("first_header_stamp_ns") ==
              {name: streams[key][0] for name, key in LABELS.items()} and completion.get("last_header_stamp_ns") ==
              {name: streams[key][-1] for name, key in LABELS.items()})
        check("completion_target_and_final_processed_image", completion.get("last_eligible_image_stamp_ns") == target
              and completion.get("last_completed_post_vio_header_stamp_ns") == target)
        check("completion_ledger_and_output_counts", completion.get("image_ledger_rows") == len(ledger)
              and completion.get("visual_diagnostic_rows") == len(diagnostics)
              and completion.get("odometry_rows") == len(odometry)
              and completion.get("eligible_image_rows") == len(eligible))
        check("completion_classification_counts", completion.get("image_classification_counts") ==
              {key: classifications[key] for key in CLASSES})
        result["counts"] = {"source_images": len(image_order), "eligible_images": len(eligible),
            "processed_images": len(processed), "visual_diagnostic_rows": len(diagnostics), "odometry_rows": len(odometry),
            "classifications": {key: classifications[key] for key in CLASSES}}
        result["last_eligible_image_stamp_ns"] = target
        result["last_original_lidar_stamp_ns"] = streams["lidar"][-1]
        result["last_original_imu_stamp_ns"] = streams["imu"][-1]
        result["status"] = "PASS" if checks and all(checks.values()) and not errors else "FAIL"
    except (OSError, ValueError, TypeError, KeyError, OverflowError, yaml.YAMLError) as exc:
        errors.append(type(exc).__name__ + ": " + str(exc))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-dir", type=Path, required=True)
    parser.add_argument("--index", type=Path)
    args = parser.parse_args(argv)
    try:
        index = _read(args.index or args.arm_dir.parent / "sensor-index.json", {})
        result = audit_processing(args.arm_dir, index)
    except (OSError, ValueError, TypeError) as exc:
        result = {"status": "FAIL", "errors": [str(exc)]}
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
