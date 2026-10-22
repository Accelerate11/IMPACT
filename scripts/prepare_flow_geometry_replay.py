#!/usr/bin/env python3
"""Re-extract recorded simulation pixels; audit geometry while blocking fusion.

Only five recorded topics are decoded. Front images contribute their original
integer CDR headers, never pixels. No ROS context, estimator or updater starts.
"""
from __future__ import annotations

import argparse
import bisect
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time

import cv2
import numpy as np

from xq_autonomy import downward_flow, flow_geometry
from xq_autonomy.downward_flow import CameraModel, GyroBuffer
from xq_autonomy.flow_geometry import (
    compensated_noise_statistics, extract_raw_tracks, rotation_compensation,
)


TOPICS = {
    "/impact/r2/down/image_raw": ("down_images", "sensor_msgs/msg/Image"),
    "/impact/r2/down/camera_info": ("down_camera_info", "sensor_msgs/msg/CameraInfo"),
    "/impact/r2/front/image_raw": ("front_images", "sensor_msgs/msg/Image"),
    "/livox/imu": ("imu", "sensor_msgs/msg/Imu"),
    "/impact/r2/flow/measurement": ("old_flow", "std_msgs/msg/String"),
}
ORIGIN = "recorded_R2A_simulation_raw_pixels_and_livox_imu"
BLOCKS = [
    "PIXEL_COVARIANCE_UNKNOWN", "GYRO_INTEGRAL_COVARIANCE_UNKNOWN",
    "STATE_GYRO_CROSS_COVARIANCE_UNKNOWN", "CROSS_TIME_COVARIANCE_UNKNOWN",
    "METRIC_DEPTH_UNKNOWN", "PIXEL_STATE_AND_GYRO_INDEPENDENCE_UNPROVEN",
]


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def decode_json(text):
    def reject(value):
        raise ValueError("nonfinite JSON constant: " + value)
    return json.loads(text, parse_constant=reject)


def save(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def header_from_cdr(blob):
    """Read only the original standard Header prefix in ROS2 CDR little endian."""
    if len(blob) < 17 or blob[:4] != b"\x00\x01\x00\x00":
        raise ValueError("unsupported or truncated original CDR encapsulation")
    sec, nanosec, length = struct.unpack_from("<iII", blob, 4)
    if sec < 0 or nanosec >= 1_000_000_000 or length < 1 or 16 + length > len(blob):
        raise ValueError("invalid original integer Header")
    if blob[15 + length] != 0 or b"\x00" in blob[16:15 + length]:
        raise ValueError("invalid original Header frame string")
    frame = blob[16:15 + length].decode("utf-8")
    stamp_ns = sec * 1_000_000_000 + nanosec
    if stamp_ns <= 0 or not frame:
        raise ValueError("zero acquisition timestamp or missing frame")
    return stamp_ns, frame


def protocol_config(path):
    data = decode_json(Path(path).read_text())
    if data.get("schema_version") != 1 or data.get("phase") != "R2B3A_FLOW_GEOMETRY_AND_CORRELATED_UPDATE_PRECONDITIONS":
        raise ValueError("unsupported frozen protocol")
    tracker, checks = data["raw_tracker"], data["bag_checks"]
    for key in ("minimum_tracks", "maximum_tracks"):
        if type(tracker[key]) is not int or tracker[key] <= 0:
            raise ValueError("invalid tracker " + key)
    if tracker["minimum_tracks"] > tracker["maximum_tracks"]:
        raise ValueError("minimum_tracks exceeds maximum_tracks")
    if tracker["selection_uses_imu"] is not False or tracker["maximum_interval_s"] != .2:
        raise ValueError("raw selection must remain pixel-only with the supported 0.2s interval")
    if tracker["camera_profile"] != "R2A_zero_distortion_320x240":
        raise ValueError("unknown camera profile")
    if isinstance(tracker["forward_backward_threshold_px"], bool) or not math.isfinite(tracker["forward_backward_threshold_px"]) or tracker["forward_backward_threshold_px"] <= 0:
        raise ValueError("invalid forward/backward threshold")
    if type(data["front_camera_keep_every_n"]) is not int or data["front_camera_keep_every_n"] != 3:
        raise ValueError("this frozen alignment protocol requires every third original front image")
    for key in ("minimum_original_down_images", "minimum_native_valid_track_windows", "minimum_aligned_valid_track_windows"):
        if type(checks[key]) is not int or checks[key] <= 0:
            raise ValueError("missing positive acceptance minimum: " + key)
    for key in ("require_all_original_images_accounted", "require_original_payload_and_header_identity",
                "require_complete_gyro_coverage_for_compensated_window", "unknown_covariance_must_block_fusion",
                "unknown_metric_depth_must_block_fusion"):
        if checks[key] is not True:
            raise ValueError("required precondition cannot be disabled: " + key)
    if type(checks["actual_flow_state_updates_must_equal"]) is not int or checks["actual_flow_state_updates_must_equal"] != 0:
        raise ValueError("recorded data must perform zero flow state updates")
    for key in ("maximum_gyro_integral_reproduction_error_rad", "maximum_analytic_numeric_rotation_jacobian_error"):
        if isinstance(checks[key], bool) or not math.isfinite(checks[key]) or checks[key] <= 0:
            raise ValueError("invalid numerical acceptance bound: " + key)
    return data


def source_snapshot(bag):
    """Hash the complete closed seven-file set, including metadata."""
    if not bag.is_dir() or bag.is_symlink():
        raise ValueError("source bag must be a real closed directory")
    result = {}
    for path in sorted(bag.iterdir()):
        if not path.is_file() or path.is_symlink():
            raise ValueError("source bag contains a non-regular entry")
        before = path.stat()
        item = {"bytes": before.st_size, "sha256": digest(path)}
        after = path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError("source file changed while hashing: " + path.name)
        result[path.name] = item
    return result


def strict_stream(records, name):
    stamps = [item["stamp_ns"] for item in records]
    if not stamps or any(type(x) is not int or x <= 0 for x in stamps) or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("non-increasing/missing original acquisition stream: " + name)


def camera_record(message):
    k, d = [float(x) for x in message.k], [float(x) for x in message.d]
    expected = [277.128129211, 0., 159.5, 0., 277.128129211, 119.5, 0., 0., 1.]
    if (message.width, message.height) != (320, 240) or len(k) != 9 or any(
            not math.isfinite(a) or abs(a-b) > 1e-8 for a, b in zip(k, expected)) or any(
            not math.isfinite(x) or x != 0. for x in d):
        raise ValueError("CameraInfo differs from frozen zero-distortion R2A calibration")
    if message.distortion_model != "plumb_bob":
        raise ValueError("unexpected CameraInfo distortion model")
    r, p = [float(x) for x in message.r], [float(x) for x in message.p]
    if not all(math.isfinite(x) for x in r+p):
        raise ValueError("nonfinite original CameraInfo projection metadata")
    roi = message.roi
    return {"width": int(message.width), "height": int(message.height), "k": k, "d": d,
            "r": r, "p": p, "binning_x": int(message.binning_x), "binning_y": int(message.binning_y),
            "roi": {"x_offset": int(roi.x_offset), "y_offset": int(roi.y_offset), "width": int(roi.width),
                    "height": int(roi.height), "do_rectify": bool(roi.do_rectify)},
            "distortion_model": message.distortion_model}


def read_sensors(bag):
    # Imports deserialize CDR offline; there is deliberately no rclpy.init().
    import sqlite3
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from std_msgs.msg import String
    classes = {"down_images": Image, "down_camera_info": CameraInfo, "imu": Imu, "old_flow": String}
    streams = {key: [] for key in ("down_images", "down_camera_info", "front_images", "imu", "old_flow")}
    topic_counts = Counter()
    for file in sorted(bag.glob("*.db3")):
        database = sqlite3.connect(file.resolve().as_uri() + "?mode=ro&immutable=1", uri=True)
        try:
            database.execute("PRAGMA query_only=ON")
            topics = {}
            for topicid, topic, kind in database.execute("select id,name,type from topics"):
                if topic in TOPICS:
                    key, expected = TOPICS[topic]
                    if kind != expected:
                        raise ValueError("recorded topic type mismatch: " + topic)
                    topics[topicid] = (topic, kind, key)
            sql = "select id,topic_id,data from messages where topic_id in (" + ",".join("?" for _ in topics) + ") order by timestamp,id"
            for rowid, topicid, blob in database.execute(sql, tuple(topics)):
                topic, kind, key = topics[topicid]
                topic_counts[topic] += 1
                ref = {"file": str(file), "rowid": rowid, "topic": topic, "type": kind,
                       "sha256": hashlib.sha256(blob).hexdigest(), "serialized_size_bytes": len(blob)}
                if key == "old_flow":
                    value = decode_json(deserialize_message(blob, String).data)
                    streams[key].append({"measurement": value, "ref": ref})
                    continue
                stamp_ns, frame = header_from_cdr(blob)
                item = {"stamp_ns": stamp_ns, "frame_id": frame, "ref": ref}
                if key == "front_images":
                    if frame != "impact_front_optical":
                        raise ValueError("unexpected original front optical frame")
                    # Do not deserialize this Image or construct its pixel array.
                    streams[key].append(item)
                    continue
                message = deserialize_message(blob, classes[key])
                if (message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec, message.header.frame_id) != (stamp_ns, frame):
                    raise ValueError("manual/original CDR Header disagreement")
                if key in ("down_images", "down_camera_info") and frame != "impact_down_optical":
                    raise ValueError("unexpected down optical frame")
                if key == "down_images":
                    if (message.width, message.height, message.encoding, message.step, len(message.data), message.is_bigendian) != (320, 240, "rgb8", 960, 230400, 0):
                        raise ValueError("unexpected original down pixel layout")
                    item["pixels"] = np.frombuffer(message.data, dtype=np.uint8).reshape(240, 320, 3).copy()
                elif key == "down_camera_info":
                    item["camera"] = camera_record(message)
                else:
                    if frame != "livox_imu":
                        raise ValueError("unexpected original gyro body frame")
                    omega = message.angular_velocity
                    item["angular_velocity_body_rad_s"] = [float(omega.x), float(omega.y), float(omega.z)]
                    item["angular_velocity_covariance_raw"] = [float(x) for x in message.angular_velocity_covariance]
                    if not all(math.isfinite(x) for x in item["angular_velocity_body_rad_s"] + item["angular_velocity_covariance_raw"]):
                        raise ValueError("nonfinite original gyro")
                streams[key].append(item)
        finally:
            database.close()
    for key in ("down_images", "down_camera_info", "front_images", "imu"):
        strict_stream(streams[key], key)
    if not streams["old_flow"]:
        raise ValueError("missing original angular-flow records")
    return streams, dict(topic_counts)


def rotation_numeric_oracle(xy, gamma, step=2e-6):
    """Independent cv2 Rodrigues projection, central difference in optical gamma."""
    xy, gamma = np.asarray(xy, dtype=float), np.asarray(gamma, dtype=float)
    ray = np.r_[xy, 1.]

    def projected(value):
        rotation, _ = cv2.Rodrigues(-value)
        point = rotation @ ray
        if point[2] <= 1e-8:
            raise ValueError("oracle ray projects behind camera")
        return point[:2] / point[2] - xy

    columns = []
    for axis in range(3):
        delta = np.zeros(3)
        delta[axis] = step
        columns.append((projected(gamma + delta) - projected(gamma - delta)) / (2 * step))
    return projected(gamma), np.column_stack(columns)


def extract_window(kind, index, before, after, camera, gyro, tracker, imu_records=None, imu_stamps=None):
    row = extract_raw_tracks(
        before["pixels"], after["pixels"], camera, before["stamp_ns"], after["stamp_ns"],
        minimum_tracks=tracker["minimum_tracks"], maximum_tracks=tracker["maximum_tracks"],
        fb_threshold_px=tracker["forward_backward_threshold_px"])
    row.update(window_kind=kind, window_index=index, source_images=[before["ref"], after["ref"]],
               source_camera_info=[before["camera_info_ref"], after["camera_info_ref"]])
    gamma, reason = gyro.integrate(before["stamp_ns"], after["stamp_ns"])
    row["gyro"] = {"covered": gamma is not None, "reason": reason,
                   "integral_optical_rad": gamma.tolist() if gamma is not None else None,
                   "source": "livox_imu", "bias_subtracted": False,
                   "integral_covariance": None, "state_gyro_cross_covariance": None}
    row["gyro"].update(sample_index_range_in_source_index=None, first_sample_ref=None,
                       last_sample_ref=None, bracketing_sample_count=0)
    if gamma is not None and imu_records is not None and imu_stamps is not None:
        left = bisect.bisect_right(imu_stamps, before["stamp_ns"])-1
        right = bisect.bisect_left(imu_stamps, after["stamp_ns"])
        row["gyro"].update(sample_index_range_in_source_index=[left,right],
                           first_sample_ref=imu_records[left]["ref"], last_sample_ref=imu_records[right]["ref"],
                           bracketing_sample_count=right-left+1)
    oracle_error = displacement_error = None
    jacobians = []
    for track in row["tracks"]:
        track["c_gamma"] = track["J_gamma"] = None
        if gamma is None:
            continue
        xy = track["old_normalized_ray"][:2]
        geometry = rotation_compensation(xy, gamma)
        track["c_gamma"] = geometry["normalized_displacement"]
        track["J_gamma"] = geometry["jacobian_gamma"]
        track["compensated_normalized_displacement"] = (
            np.asarray(track["raw_normalized_displacement"]) - np.asarray(track["c_gamma"])).tolist()
        jacobians.append(track["J_gamma"])
        if oracle_error is None:
            value, jacobian = rotation_numeric_oracle(xy, gamma)
            oracle_error = float(np.max(np.abs(np.asarray(track["J_gamma"]) - jacobian)))
            displacement_error = float(np.max(np.abs(np.asarray(track["c_gamma"]) - value)))
    row["rotation_oracle"] = {"tested_tracks": int(oracle_error is not None),
                              "maximum_jacobian_absolute_error": oracle_error,
                              "maximum_displacement_absolute_error": displacement_error,
                              "central_difference_step_rad": 2e-6, "method": "cv2.Rodrigues"}
    row["noise_statistics"] = (compensated_noise_statistics(
        np.vstack(jacobians), None, None, None, None, pixel_noise_independent=False) if jacobians else
        {"status": "UNAVAILABLE", "reason": "NO_COVERED_TRACK_JACOBIANS", "R_effective": None,
         "C_effective": None, "uncertainty_known": False})
    row["depth_m"] = None
    row["cross_time_covariance"] = None
    row["fusion_update_performed"] = False
    row["blocked_reasons"] = BLOCKS + ([] if gamma is not None else [reason]) + ([] if row["valid"] else [row["reason"]])
    return row


def original_flow_summary(records, gyro, retained):
    reasons = Counter()
    maximum_error, positive, covered = 0., 0, 0
    both, ends, valid = 0, 0, 0
    last_end = -1
    for item in records:
        row = item["measurement"]
        if row.get("schema_version") != 1 or row.get("frame_id") != "impact_down_optical":
            raise ValueError("unknown original flow schema/frame")
        a, b = row["acquisition_start_ns"], row["acquisition_end_ns"]
        if any(type(value) is not int or value <= 0 for value in (a, b)) or b < a:
            raise ValueError("invalid original flow integer interval")
        if b <= last_end:
            raise ValueError("non-increasing original flow acquisition stream")
        last_end = b
        if row["source_image_stamps_ns"] != [a, b] or abs(row["dt_s"] - (b-a)*1e-9) > 1e-12:
            raise ValueError("original flow interval/header contract mismatch")
        if type(row["valid"]) is not bool or type(row["gyro_valid"]) is not bool:
            raise ValueError("invalid original flow validity markers")
        if row["ground_truth_used"] is not False or row["dependencies"] != ["down_camera", "livox_imu"] or row["source_model"] != "gazebo_pixels_lk":
            raise ValueError("unknown original flow provenance")
        for field, size in (("raw_integrated_flow_rad", 2), ("compensated_integrated_flow_rad", 2), ("gyro_integral_optical_rad", 3)):
            if len(row[field]) != size or not all(isinstance(x, (float,int)) and not isinstance(x,bool) and math.isfinite(x) for x in row[field]):
                raise ValueError("invalid original angular-flow vector")
        reasons[row["reason"]] += 1
        valid += row["valid"]
        both += row["valid"] and a in retained and b in retained
        ends += row["valid"] and b in retained
        if b > a:
            positive += 1
            gamma, _ = gyro.integrate(a, b)
            covered += gamma is not None
            if row["gyro_valid"]:
                if gamma is None:
                    raise ValueError("original gyro-valid record has no raw IMU coverage")
                maximum_error = max(maximum_error, float(np.linalg.norm(gamma - np.asarray(row["gyro_integral_optical_rad"]))))
    return {"count": len(records), "valid": valid, "invalid": len(records)-valid, "reason_counts": dict(reasons),
            "positive_windows": positive, "covered_positive_windows": covered,
            "max_gyro_reproduction_error_rad": maximum_error,
            "metric_velocity_valid_count": sum(item["measurement"]["metric_velocity_valid"] is not False for item in records),
            "metric_velocity_nonnull_count": sum(item["measurement"]["metric_velocity_mps"] is not None for item in records),
            "native_both_endpoints_match_retained_front": both, "native_valid_end_matches_retained_front": ends}


def prepare(protocol, protocol_path, output, canonical, state):
    checks, counts = state["checks"], state["counts"]
    bag = Path(protocol["source_bag"]).resolve()
    state["bag"] = str(bag)
    if output.resolve().is_relative_to(bag):
        raise ValueError("output must not be inside original bag")
    frozen = Path(protocol["source_evidence"])
    frozen = frozen if frozen.is_absolute() else canonical / frozen
    audit = decode_json(frozen.read_text())
    if audit.get("status") != "PASS":
        raise ValueError("frozen bag audit did not pass")
    expected = audit["original_files_before"]
    if len(expected) != 7 or set(expected) != {"metadata.yaml", *(f"rosbag_{i}.db3" for i in range(6))}:
        raise ValueError("unexpected frozen source bag file set")
    state["source_binding"] = {"frozen_audit": str(frozen), "frozen_audit_sha256": digest(frozen)}
    before = source_snapshot(bag)
    state["source_binding"]["files_before"] = before
    save(output / "raw-hashes.before.json", state["source_binding"])
    checks["frozen_raw_file_set_and_all_sha_before"] = before == expected
    if before != expected:
        raise ValueError("source bag differs from frozen original SHA/size set")
    streams, topic_counts = read_sensors(bag)
    images, infos, front, imus = (streams[key] for key in ("down_images", "down_camera_info", "front_images", "imu"))
    info_by_stamp = {item["stamp_ns"]: item for item in infos}
    if set(info_by_stamp) != {item["stamp_ns"] for item in images}:
        raise ValueError("down Image/CameraInfo acquisition stamps do not match exactly")
    models = {json.dumps(item["camera"], sort_keys=True) for item in infos}
    if len(models) != 1:
        raise ValueError("changing original CameraInfo")
    down_by_stamp = {item["stamp_ns"]: item for item in images}
    if any(item["stamp_ns"] not in down_by_stamp for item in front):
        raise ValueError("original front epoch has no exact down image counterpart")
    for item in images:
        item["camera_info_ref"] = info_by_stamp[item["stamp_ns"]]["ref"]
    kept = front[::protocol["front_camera_keep_every_n"]]
    aligned = [down_by_stamp[item["stamp_ns"]] for item in kept]
    retained = {item["stamp_ns"] for item in kept}
    gyro = GyroBuffer(maximum_samples=max(20000, len(imus)+16))
    for item in imus:
        if not gyro.append(item["stamp_ns"], item["angular_velocity_body_rad_s"]):
            raise ValueError("original IMU cannot enter strict covered-window buffer")
    camera = CameraModel(320, 240, 277.128129211, 277.128129211, 159.5, 119.5)
    counts.update(down_images=len(images), down_camera_info=len(infos), front_images=len(front),
                  imu_messages=len(imus), original_flow_messages=len(streams["old_flow"]), image_ledger_rows=0,
                  native_windows=0, native_valid_windows=0, native_tracks=0,
                  aligned_windows=0, aligned_valid_windows=0, aligned_tracks=0, rotation_oracle_windows=0)
    state["old_flow"] = original_flow_summary(streams["old_flow"], gyro, retained)
    if any(row["measurement"]["acquisition_start_ns"] not in down_by_stamp or
           row["measurement"]["acquisition_end_ns"] not in down_by_stamp for row in streams["old_flow"]):
        raise ValueError("original flow endpoints are not original down images")
    source_index = {
        "schema_version": 1, "input_origin": ORIGIN, "source_bag": str(bag), "protocol_sha256": digest(protocol_path),
        "source_binding": state["source_binding"], "topic_counts": topic_counts,
        "decoder_policy": "Only down Image/Info, front Header prefix, Livox IMU, original flow String; no truth/odom/TF; no ROS context",
        "messages": {key: [{k:v for k,v in item.items() if k != "pixels"} for item in records] for key,records in streams.items()},
        "retained_front_stamps_ns": [item["stamp_ns"] for item in kept],
        "front_camera_keep_every_n": protocol["front_camera_keep_every_n"],
        "gyro_buffer_capacity": gyro.samples.maxlen,
        "module_sources": {module.__name__: {"file": str(Path(module.__file__).resolve()), "sha256": digest(module.__file__)}
                           for module in (downward_flow, flow_geometry)},
        "runtime": {"opencv": cv2.__version__, "numpy": np.__version__, "opencv_threads": cv2.getNumThreads()},
    }
    save(output / "source-index.json", source_index)
    index_path = output / "source-index.json"
    state["source_index"] = {"path": str(index_path), "bytes": index_path.stat().st_size, "sha256": digest(index_path)}
    aligned_positions = {item["stamp_ns"]: i for i,item in enumerate(aligned)}
    with (output / "image-ledger.jsonl").open("w") as stream:
        for i,item in enumerate(images):
            j = aligned_positions.get(item["stamp_ns"])
            row = {"schema_version": 1, "image_sequence_id": i+1, "header_stamp_ns": item["stamp_ns"],
                   "classification": "ACCOUNTED_ORIGINAL_DOWN_IMAGE", "source_image": item["ref"],
                   "source_camera_info": item["camera_info_ref"], "retained_front_epoch": j is not None,
                   "native_window_indices": [x for x in (i-1,i) if 0 <= x < len(images)-1],
                   "aligned_window_indices": [x for x in (j-1,j) if 0 <= x < len(aligned)-1] if j is not None else []}
            stream.write(json.dumps(row, allow_nan=False) + "\n")
            counts["image_ledger_rows"] += 1
    maximum_oracle = maximum_displacement = 0.
    imu_stamps = [item["stamp_ns"] for item in imus]
    statistics_unavailable, valid_covered, all_blocked, aligned_endpoints = True, True, True, True
    for kind, frames in (("native", images), ("aligned", aligned)):
        reasons = Counter()
        with (output / (kind + "-tracks.jsonl")).open("w") as stream:
            for index,(before,after) in enumerate(zip(frames, frames[1:])):
                row = extract_window(kind, index, before, after, camera, gyro, protocol["raw_tracker"],
                                     imus, imu_stamps)
                if kind == "aligned":
                    row["both_endpoints_are_retained_front_epochs"] = before["stamp_ns"] in retained and after["stamp_ns"] in retained
                    aligned_endpoints &= row["both_endpoints_are_retained_front_epochs"]
                counts[kind+"_windows"] += 1
                counts[kind+"_valid_windows"] += int(row["valid"])
                counts[kind+"_tracks"] += row["track_count"]
                reasons[row["reason"]] += 1
                statistics_unavailable &= row["noise_statistics"]["status"] == "UNAVAILABLE"
                valid_covered &= not row["valid"] or row["gyro"]["covered"]
                all_blocked &= row["fusion_update_performed"] is False and bool(row["blocked_reasons"])
                oracle = row["rotation_oracle"]
                if oracle["tested_tracks"]:
                    counts["rotation_oracle_windows"] += 1
                    maximum_oracle = max(maximum_oracle, oracle["maximum_jacobian_absolute_error"])
                    maximum_displacement = max(maximum_displacement, oracle["maximum_displacement_absolute_error"])
                stream.write(json.dumps(row, allow_nan=False) + "\n")
        state[kind+"_reasons"] = dict(reasons)
    state["geometry"] = {"maximum_rotation_jacobian_oracle_error": maximum_oracle,
                         "maximum_rotation_displacement_oracle_error": maximum_displacement}
    gate = protocol["bag_checks"]
    checks.update(
        complete_original_image_ledger=counts["image_ledger_rows"] == len(images),
        minimum_original_down_images=len(images) >= gate["minimum_original_down_images"],
        native_all_adjacent_windows=counts["native_windows"] == len(images)-1,
        aligned_all_retained_front_windows=counts["aligned_windows"] == len(kept)-1,
        aligned_both_endpoints_match_retained_front=aligned_endpoints,
        minimum_native_valid_windows=counts["native_valid_windows"] >= gate["minimum_native_valid_track_windows"],
        minimum_aligned_valid_windows=counts["aligned_valid_windows"] >= gate["minimum_aligned_valid_track_windows"],
        valid_track_windows_have_covered_gyro=valid_covered,
        original_gyro_reproduced=state["old_flow"]["max_gyro_reproduction_error_rad"] <= gate["maximum_gyro_integral_reproduction_error_rad"],
        original_metric_velocity_unavailable=state["old_flow"]["metric_velocity_valid_count"] == 0 and state["old_flow"]["metric_velocity_nonnull_count"] == 0,
        original_all_positive_windows_gyro_covered=state["old_flow"]["positive_windows"] == state["old_flow"]["covered_positive_windows"],
        analytic_rotation_jacobian_matches_independent_oracle=counts["rotation_oracle_windows"] > 0 and maximum_oracle <= gate["maximum_analytic_numeric_rotation_jacobian_error"],
        analytic_rotation_displacement_matches_independent_oracle=counts["rotation_oracle_windows"] > 0 and maximum_displacement <= gate["maximum_analytic_numeric_rotation_jacobian_error"],
        unknown_statistics_unavailable=statistics_unavailable,
        every_window_blocks_actual_fusion=all_blocked,
        actual_flow_state_updates_zero=state["boundaries"]["actual_flow_state_updates"] == gate["actual_flow_state_updates_must_equal"],
        strict_original_headers_and_camera_info=True, only_allowed_sensor_topics_decoded=True, ros_context_not_initialized=True)
    state["checks"] = {key:bool(value) for key,value in checks.items()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--canonical-root", type=Path, default=Path("/mnt/d/CODEX/impact"))
    args = parser.parse_args(argv)
    if args.output.exists():
        print("Refuse to overwrite an existing output directory", file=sys.stderr)
        return 2
    # Resolve this guard before creating any directory: even a failed request
    # must never create an output entry inside the protected original bag.
    try:
        source_value = decode_json(args.protocol.read_text()).get("source_bag")
        if isinstance(source_value, str) and args.output.resolve().is_relative_to(Path(source_value).resolve()):
            print("Refuse to create output inside original bag", file=sys.stderr)
            return 2
    except (OSError, ValueError, TypeError):
        pass  # The normal failure artifact below reports malformed protocols.
    args.output.mkdir(parents=True)
    cv2.setNumThreads(1)
    start = time.monotonic()
    state = {"schema_version": 1, "status": "FAIL", "input_origin": ORIGIN, "checks": {}, "counts": {},
             "errors": [], "script_sha256": digest(__file__), "protocol": str(args.protocol),
             "boundaries": {"fusion_update_performed": False, "actual_flow_state_updates": 0,
                "independent_range_available": False, "metric_depth_known": False,
                "pixel_noise_independent": False, "actual_pixel_covariance": None,
                "actual_gyro_integral_covariance": None, "actual_state_gyro_cross_covariance": None,
                "actual_cross_time_covariance": None, "hardware_readiness": "NOT_READY",
                "blocked_reasons": BLOCKS}}
    try:
        protocol = protocol_config(args.protocol)
        save(args.output / "protocol.json", protocol)
        prepare(protocol, args.protocol, args.output, args.canonical_root.resolve(), state)
        state["checks"]["execution_completed"] = True
    except Exception as error:
        state["errors"].append(type(error).__name__ + ": " + str(error))
        state["checks"]["execution_completed"] = False
    finally:
        binding = state.get("source_binding", {})
        if state.get("source_index"):
            try:
                index_path = args.output / "source-index.json"
                record = state["source_index"]
                state["checks"]["source_index_content_unchanged"] = (
                    index_path.is_file() and index_path.stat().st_size == record["bytes"] and digest(index_path) == record["sha256"])
            except OSError as error:
                state["checks"]["source_index_content_unchanged"] = False
                state["errors"].append("source-index after-check: " + str(error))
        if binding.get("files_before"):
            try:
                after = source_snapshot(Path(state["bag"]))
                binding["files_after"] = after
                state["checks"]["frozen_raw_file_set_and_all_sha_after"] = after == binding["files_before"]
                save(args.output / "raw-hashes.after.json", binding)
            except Exception as error:
                state["checks"]["frozen_raw_file_set_and_all_sha_after"] = False
                state["errors"].append("source after-check: " + str(error))
        state["checks"] = {key:bool(value) for key,value in state["checks"].items()}
        state["elapsed_wall_s"] = time.monotonic()-start
        state["status"] = "PASS" if state["checks"] and all(state["checks"].values()) and not state["errors"] else "FAIL"
        save(args.output / "replay-result.json", state)
    print(json.dumps({"status": state["status"], "output": str(args.output),
                      "counts": state["counts"], "errors": state["errors"]}, allow_nan=False))
    return 0 if state["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
