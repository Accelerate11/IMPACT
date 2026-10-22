#!/usr/bin/env python3
"""Independent recorded-pixel/gyro geometry audit; never runs a state updater.

This reader does not import the production writer, tracker or geometry module.
Original CDR is checked offline through read-only SQLite and a small CDR reader.
It independently rechecks LK on every reported point, Rodrigues projections,
finite-difference derivatives, window schedules and the missing-noise/depth gate.
The writer's before/after whole-bag hashes are bound to frozen evidence; this
audit verifies every allowed message payload instead of rehashing other topics.
"""
from __future__ import annotations

import argparse
import bisect
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import time

import cv2
import numpy as np


ORIGIN = "recorded_R2A_simulation_raw_pixels_and_livox_imu"
TOPICS = {
    "/impact/r2/down/image_raw": ("down_images", "sensor_msgs/msg/Image"),
    "/impact/r2/down/camera_info": ("down_camera_info", "sensor_msgs/msg/CameraInfo"),
    "/impact/r2/front/image_raw": ("front_images", "sensor_msgs/msg/Image"),
    "/livox/imu": ("imu", "sensor_msgs/msg/Imu"),
    "/impact/r2/flow/measurement": ("old_flow", "std_msgs/msg/String"),
}
BLOCKS = ["PIXEL_COVARIANCE_UNKNOWN", "GYRO_INTEGRAL_COVARIANCE_UNKNOWN",
          "STATE_GYRO_CROSS_COVARIANCE_UNKNOWN", "CROSS_TIME_COVARIANCE_UNKNOWN",
          "METRIC_DEPTH_UNKNOWN", "PIXEL_STATE_AND_GYRO_INDEPENDENCE_UNPROVEN"]
FOCAL = np.array([277.128129211, 277.128129211])
PRINCIPAL = np.array([159.5, 119.5])
ROTATION_BC = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
INPUTS = {"protocol.json", "raw-hashes.before.json", "raw-hashes.after.json", "source-index.json",
          "image-ledger.jsonl", "native-tracks.jsonl", "aligned-tracks.jsonl", "replay-result.json"}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def _pairs(items):
    output = {}
    for key, value in items:
        require(key not in output, "duplicate JSON key: " + key)
        output[key] = value
    return output


def _float(value):
    result = float(value)
    require(math.isfinite(result), "nonfinite JSON number")
    return result


def loads(data):
    def bad(value):
        raise ValueError("nonfinite JSON constant: " + value)
    return json.loads(data, object_pairs_hook=_pairs, parse_float=_float, parse_constant=bad)


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def read(path):
    return loads(Path(path).read_text())


def array(value, shape, label):
    data = np.asarray(value)
    require(data.dtype.kind in "fiu" and data.shape == shape, label + " numeric shape")
    data = data.astype(float)
    require(np.isfinite(data).all(), label + " finite")
    return data


def close(value, expected, label, tolerance=1e-12):
    expected = np.asarray(expected, dtype=float)
    actual = array(value, expected.shape, label)
    require(np.max(np.abs(actual - expected), initial=0.) <= tolerance, label + " mismatch")


def frozen_protocol(value):
    require(value.get("schema_version") == 1 and value.get("phase") ==
            "R2B3A_FLOW_GEOMETRY_AND_CORRELATED_UPDATE_PRECONDITIONS", "protocol identity")
    require(type(value.get("front_camera_keep_every_n")) is int and value["front_camera_keep_every_n"] == 3,
            "frozen every-third alignment")
    require(value.get("raw_tracker") == {
        "minimum_tracks": 12, "maximum_tracks": 150, "forward_backward_threshold_px": 1.0,
        "maximum_interval_s": .2, "selection_uses_imu": False,
        "camera_profile": "R2A_zero_distortion_320x240"}, "frozen raw tracker")
    require(value.get("bag_checks") == {
        "minimum_original_down_images": 1000, "minimum_native_valid_track_windows": 100,
        "minimum_aligned_valid_track_windows": 100, "require_all_original_images_accounted": True,
        "require_original_payload_and_header_identity": True,
        "require_complete_gyro_coverage_for_compensated_window": True,
        "unknown_covariance_must_block_fusion": True, "unknown_metric_depth_must_block_fusion": True,
        "maximum_gyro_integral_reproduction_error_rad": 1e-12,
        "maximum_analytic_numeric_rotation_jacobian_error": 1e-7,
        "actual_flow_state_updates_must_equal": 0}, "frozen acceptance bounds")
    require(value.get("hardware_readiness") == "NOT_READY" and
            value.get("full_common_state_flow_fusion_validated") is False, "protocol scope")
    return value


class CDR:
    """Little-endian ROS2 CDR, alignment relative to the encapsulated body."""
    def __init__(self, blob):
        require(blob[:4] == b"\x00\x01\x00\x00", "unsupported original CDR")
        self.blob, self.pos = blob, 4

    def numbers(self, kind, count=1):
        size = struct.calcsize("<" + kind)
        self.pos += (-(self.pos - 4)) % size
        length = size * count
        require(self.pos + length <= len(self.blob), "truncated CDR numeric")
        values = struct.unpack_from("<" + kind * count, self.blob, self.pos)
        self.pos += length
        return values[0] if count == 1 else list(values)

    def string(self):
        length = self.numbers("I")
        require(length >= 1 and self.pos + length <= len(self.blob), "truncated CDR string")
        raw = self.blob[self.pos:self.pos + length]
        self.pos += length
        require(raw[-1:] == b"\0" and b"\0" not in raw[:-1], "invalid CDR string")
        return raw[:-1].decode("utf-8")

    def header(self):
        sec, ns = self.numbers("i"), self.numbers("I")
        frame = self.string()
        require(sec >= 0 and ns < 1_000_000_000 and bool(frame), "original Header invalid")
        stamp = sec * 1_000_000_000 + ns
        require(stamp > 0, "zero acquisition stamp")
        return stamp, frame

    def done(self):
        # Recorded DDS payloads can include terminal submessage alignment,
        # whereas rclpy.serialize_message returns only the encoded fields.
        # Accept an empty suffix or exactly the ZERO bytes needed for the next
        # 4-byte boundary. No other suffix length or content is permitted.
        # Original full payload SHA/size is still checked before any decoding.
        suffix = self.blob[self.pos:]
        require(not suffix or (len(suffix) == (-self.pos) % 4 and suffix == bytes(len(suffix))),
                "unexpected CDR trailing content")


def decode_payload(blob, key):
    reader = CDR(blob)
    if key == "old_flow":
        value = {"measurement": loads(reader.string())}
        reader.done()
        return value
    stamp, frame = reader.header()
    result = {"stamp_ns": stamp, "frame_id": frame}
    if key == "front_images":
        require(frame == "impact_front_optical", "front optical frame")
        return result  # Only the Header prefix is decoded.
    if key == "down_images":
        h, w, encoding = reader.numbers("I"), reader.numbers("I"), reader.string()
        big, step, length = reader.numbers("B"), reader.numbers("I"), reader.numbers("I")
        require((h, w, encoding, big, step, length) == (240, 320, "rgb8", 0, 960, 230400), "down pixel layout")
        require(reader.pos + length == len(blob), "down pixel array length")
        result["pixels"] = np.frombuffer(blob, dtype=np.uint8, offset=reader.pos, count=length).reshape(h, w, 3)
        reader.pos += length
    elif key == "imu":
        reader.numbers("d", 4)  # Orientation metadata is not a state estimate input.
        reader.numbers("d", 9)
        result["angular_velocity_body_rad_s"] = reader.numbers("d", 3)
        result["angular_velocity_covariance_raw"] = reader.numbers("d", 9)
        reader.numbers("d", 3)
        reader.numbers("d", 9)
        require(frame == "livox_imu", "gyro body frame")
    else:
        h, w, distortion = reader.numbers("I"), reader.numbers("I"), reader.string()
        d = reader.numbers("d", reader.numbers("I"))
        if not isinstance(d, list):
            d = [d]
        k, r, p = reader.numbers("d", 9), reader.numbers("d", 9), reader.numbers("d", 12)
        bx, by = reader.numbers("I"), reader.numbers("I")
        roi = dict(zip(("x_offset", "y_offset", "height", "width"), reader.numbers("I", 4)))
        rectify = reader.numbers("B")
        require(rectify in (0, 1), "CameraInfo ROI bool")
        roi["do_rectify"] = bool(rectify)
        result["camera"] = {"width": w, "height": h, "distortion_model": distortion,
                            "k": k, "d": d, "r": r, "p": p, "binning_x": bx, "binning_y": by, "roi": roi}
        require((w, h, distortion) == (320, 240, "plumb_bob") and all(x == 0. for x in d), "frozen CameraInfo")
        close(k, [FOCAL[0], 0., PRINCIPAL[0], 0., FOCAL[1], PRINCIPAL[1], 0., 0., 1.], "CameraInfo K", 1e-8)
    reader.done()
    if key in ("down_images", "down_camera_info"):
        require(frame == "impact_down_optical", "down optical frame")
    return result


def check_sources(index, bag):
    """Match all indexed allowed records to all original allowed SQL rows."""
    require(set(index["messages"]) == {x[0] for x in TOPICS.values()}, "indexed topic whitelist")
    expected, pixels = {}, {}
    for key, records in index["messages"].items():
        last = -1
        for record in records:
            ref = record["ref"]
            path = Path(ref["file"]).resolve()
            require(path.parent == bag and path.suffix == ".db3", "source ref outside frozen bag")
            identity = (str(path), ref["rowid"])
            require(type(ref["rowid"]) is int and ref["rowid"] > 0 and identity not in expected, "duplicate/invalid source row")
            require(TOPICS.get(ref["topic"]) == (key, ref["type"]), "source ref topic/type")
            if key != "old_flow":
                stamp = record["stamp_ns"]
                require(type(stamp) is int and stamp > last, "source acquisition ordering")
                last = stamp
            expected[identity] = (key, record)
    counts, seen = Counter(), set()
    for path in sorted(bag.glob("*.db3")):
        connection = sqlite3.connect(path.as_uri() + "?mode=ro&immutable=1", uri=True)
        try:
            connection.execute("PRAGMA query_only=ON")
            topics = {}
            for tid, topic, kind in connection.execute("select id,name,type from topics"):
                if topic in TOPICS:
                    require(kind == TOPICS[topic][1], "original topic type")
                    topics[tid] = (topic, kind, TOPICS[topic][0])
            if not topics:
                continue
            query = "select id,topic_id,data from messages where topic_id in (" + ",".join("?" for _ in topics) + ") order by timestamp,id"
            for rowid, tid, blob in connection.execute(query, tuple(topics)):
                identity = (str(path), rowid)
                require(identity in expected and identity not in seen, "original payload omitted or duplicated in index")
                key, record = expected[identity]
                topic, kind, original_key = topics[tid]
                ref = record["ref"]
                require((key, ref["topic"], ref["type"]) == (original_key, topic, kind), "source row topic association")
                require(type(ref["serialized_size_bytes"]) is int and ref["serialized_size_bytes"] == len(blob) and
                        ref["sha256"] == hashlib.sha256(blob).hexdigest(), "original CDR payload SHA/size")
                decoded = decode_payload(blob, key)
                if key == "down_images":
                    pixels[decoded["stamp_ns"]] = cv2.cvtColor(decoded.pop("pixels"), cv2.COLOR_RGB2GRAY)
                require(decoded == {k: v for k, v in record.items() if k not in ("ref", "camera_info_ref")}, "indexed values differ from original CDR")
                counts[topic] += 1
                seen.add(identity)
        finally:
            connection.close()
    require(seen == set(expected), "indexed source refs missing from original bag")
    require(dict(counts) == index["topic_counts"], "original topic counts")
    return pixels, len(seen)


def integrate_gyro(records, stamps, start, end):
    if end <= start:
        return None, "INVALID_INTEGRATION_INTERVAL", None
    if len(stamps) < 2 or start < stamps[0] or end > stamps[-1]:
        return None, "GYRO_WINDOW_UNCOVERED", None
    left, right = bisect.bisect_right(stamps, start) - 1, bisect.bisect_left(stamps, end)
    if any(b - a > 20_000_000 for a, b in zip(stamps[left:right], stamps[left+1:right+1])):
        return None, "GYRO_GAP_EXCEEDED", None
    # Independent piecewise linear integration, clipping each IMU segment.
    total = np.zeros(3)
    for i in range(left, right):
        t0, t1 = stamps[i:i+2]
        a, b = max(start, t0), min(end, t1)
        if a >= b:
            continue
        w0 = np.asarray(records[i]["angular_velocity_body_rad_s"], dtype=float)
        w1 = np.asarray(records[i+1]["angular_velocity_body_rad_s"], dtype=float)
        wa = w0 + (w1 - w0) * ((a - t0) / (t1 - t0))
        wb = w0 + (w1 - w0) * ((b - t0) / (t1 - t0))
        total += .5 * (wa + wb) * ((b - a) * 1e-9)
    return ROTATION_BC.T @ total, "OK", (left, right)


def rotation_oracle(xy, gamma, step=2e-6):
    ray = np.r_[xy, 1.]
    def displacement(g):
        matrix, _ = cv2.Rodrigues(np.asarray(-g, dtype=float))
        projected = matrix @ ray
        require(projected[2] > 1e-8, "oracle projection behind camera")
        return projected[:2] / projected[2] - xy
    jacobian = np.column_stack([(displacement(gamma + np.eye(3)[i]*step) -
                                 displacement(gamma - np.eye(3)[i]*step))/(2*step) for i in range(3)])
    return displacement(gamma), jacobian


def check_lk(tracks, previous, current, detected_count):
    features = cv2.goodFeaturesToTrack(previous, maxCorners=150, qualityLevel=.01, minDistance=5., blockSize=5)
    if features is None:
        require(detected_count == 0 and not tracks, "independent no-feature window")
        return "INSUFFICIENT_FEATURES"
    require(len(features) == detected_count, "independent feature count")
    ids = [track["track_id"] for track in tracks]
    options = dict(winSize=(21, 21), maxLevel=3,
                   criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
    # Re-run all original detected points to preserve the original LK batch.
    forward, status, _ = cv2.calcOpticalFlowPyrLK(previous, current, features, None, **options)
    if forward is None or status is None:
        require(not tracks, "independent failed forward LK has tracks")
        return "LK_FORWARD_FAILED"
    old, new = features.reshape(-1, 2), forward.reshape(-1, 2)
    usable = status.reshape(-1).astype(bool) & np.isfinite(new).all(axis=1) & (new[:, 0] >= 0) & (new[:, 0] < 320) & (new[:, 1] >= 0) & (new[:, 1] < 240)
    identifiers = np.flatnonzero(usable)
    if not len(identifiers):
        require(not tracks, "independent no usable LK tracks")
        return "INSUFFICIENT_TRACKS"
    returned, status, _ = cv2.calcOpticalFlowPyrLK(current, previous, new[identifiers].reshape(-1, 1, 2), None, **options)
    if returned is None or status is None:
        require(not tracks, "independent failed backward LK has tracks")
        return "LK_BACKWARD_FAILED"
    error = np.linalg.norm(returned.reshape(-1, 2) - old[identifiers], axis=1)
    good = status.reshape(-1).astype(bool) & np.isfinite(returned.reshape(-1, 2)).all(axis=1) & (error <= 1.)
    require(ids == identifiers[good].tolist(), "reported LK accepted point set")
    mapping = dict(zip(identifiers.tolist(), error.tolist()))
    for track in tracks:
        i = track["track_id"]
        close(track["old_pixel"], old[i], "original detected pixel", 1e-6)
        close(track["new_pixel"], new[i], "independent tracked pixel", 1e-5)
        require(abs(track["forward_backward_error_px"] - mapping[i]) <= 1e-5, "independent forward-backward error")
    return "OK_RAW_PIXEL_TRACKS_NO_SCALE_OR_COVARIANCE" if len(ids) >= 12 else "INSUFFICIENT_TRACKS"


def check_window(row, kind, index, before, after, imus, stamps, pixels, tolerance):
    start, end = before["stamp_ns"], after["stamp_ns"]
    require(row["schema_version"] == 1 and row["window_kind"] == kind and type(row["window_index"]) is int and row["window_index"] == index, "window identity")
    require(type(row["acquisition_start_ns"]) is int and type(row["acquisition_end_ns"]) is int and
            [row["acquisition_start_ns"], row["acquisition_end_ns"]] == [start, end] and
            row["source_image_stamps_ns"] == [start, end], "window integer schedule")
    require(abs(row["dt_s"] - (end-start)*1e-9) <= 1e-15 and 0 < end-start < 200_000_000, "window interval")
    require(row["source_images"] == [before["ref"], after["ref"]] and
            row["source_camera_info"] == [before["camera_info_ref"], after["camera_info_ref"]], "window source image/CameraInfo refs")
    require(row["frame_id"] == "impact_down_optical" and row["measurement_space"] == "normalized_image_plane" and
            row["track_id_scope"] == "image_pair" and row["selection_dependencies"] == ["down_camera"] and
            row["gyro_used"] is False and row["ground_truth_used"] is False, "pixel-only tracking provenance")
    require(row["covariance"] is None and row["uncertainty_known"] is False and row["metric_velocity_valid"] is False and
            row["metric_velocity_mps"] is None and row["scale_source"] == "unavailable" and row["depth_m"] is None and
            row["cross_time_covariance"] is None and row["fusion_update_performed"] is False, "unknown-noise/depth fusion block")
    require(type(row["detected_track_count"]) is int and type(row["track_count"]) is int and
            0 <= row["track_count"] == len(row["tracks"]) <= row["detected_track_count"] <= 150, "track counts")
    require(type(row["valid"]) is bool and row["valid"] == (len(row["tracks"]) >= 12), "valid track threshold")
    if row["valid"]:
        require(row["reason"] == "OK_RAW_PIXEL_TRACKS_NO_SCALE_OR_COVARIANCE", "valid window reason")
    else:
        require(row["reason"] in ("INSUFFICIENT_FEATURES", "INSUFFICIENT_TRACKS", "LK_FORWARD_FAILED", "LK_BACKWARD_FAILED"), "invalid tracking reason")
    gamma, reason, bounds = integrate_gyro(imus, stamps, start, end)
    gyro = row["gyro"]
    require(type(gyro["covered"]) is bool and gyro["covered"] == (gamma is not None) and gyro["reason"] == reason and
            gyro["source"] == "livox_imu" and gyro["bias_subtracted"] is False and gyro["integral_covariance"] is None and
            gyro["state_gyro_cross_covariance"] is None, "covered-window gyro provenance")
    require(not row["valid"] or gamma is not None, "valid window gyro uncovered")
    if gamma is None:
        require(gyro["integral_optical_rad"] is None and gyro["sample_index_range_in_source_index"] is None and
                gyro["first_sample_ref"] is None and gyro["last_sample_ref"] is None and gyro["bracketing_sample_count"] == 0, "uncovered gyro bounds")
    else:
        close(gyro["integral_optical_rad"], gamma, "independent gyro integral")
        left, right = bounds
        require(gyro["sample_index_range_in_source_index"] == [left, right] and
                gyro["first_sample_ref"] == imus[left]["ref"] and gyro["last_sample_ref"] == imus[right]["ref"] and
                gyro["bracketing_sample_count"] == right-left+1, "gyro raw sample bracket refs")
    expected_blocks = BLOCKS + ([] if gamma is not None else [reason]) + ([] if row["valid"] else [row["reason"]])
    require(row["blocked_reasons"] == expected_blocks, "missing actual fusion block")
    noise = row["noise_statistics"]
    require(noise["status"] == "UNAVAILABLE" and noise["uncertainty_known"] is False and
            noise["R_effective"] is None and noise["C_effective"] is None, "unknown covariance treated as known")
    require(noise["reason"] == ("COVARIANCE_UNKNOWN" if gamma is not None and row["tracks"] else "NO_COVERED_TRACK_JACOBIANS"), "noise availability reason")
    identifiers, largest_j, largest_c = [], 0., 0.
    for track in row["tracks"]:
        tid = track["track_id"]
        require(type(tid) is int and 0 <= tid < row["detected_track_count"] and tid not in identifiers, "track identifier")
        identifiers.append(tid)
        old, new = array(track["old_pixel"], (2,), "old pixel"), array(track["new_pixel"], (2,), "new pixel")
        require(np.all(old >= 0) and old[0] < 320 and old[1] < 240 and np.all(new >= 0) and new[0] < 320 and new[1] < 240, "pixel bounds")
        x, y = (old-PRINCIPAL)/FOCAL
        new_xy = (new-PRINCIPAL)/FOCAL
        close(track["old_normalized_ray"], [x, y, 1.], "old calibrated ray")
        close(track["new_normalized_ray"], [*new_xy, 1.], "new calibrated ray")
        raw = new_xy - [x, y]
        close(track["raw_normalized_displacement"], raw, "raw pixel displacement")
        close(track["jacobian_gamma_at_zero"], [[x*y, -(1+x*x), y], [1+y*y, -x*y, -x]], "zero-rotation Jacobian")
        require(type(track["forward_backward_error_px"]) in (int, float) and 0 <= track["forward_backward_error_px"] <= 1. and
                track["covariance"] is None and track["uncertainty_known"] is False, "track FB gate/covariance")
        if gamma is None:
            require(track["c_gamma"] is None and track["J_gamma"] is None, "uncovered track rotation")
        else:
            # Independent projection for every track; derivative oracle at every
            # track is inexpensive compared with LK and prevents interior tampering.
            value, jacobian = rotation_oracle(np.array([x, y]), gamma)
            c, j = array(track["c_gamma"], (2,), "rotation displacement"), array(track["J_gamma"], (2,3), "rotation Jacobian")
            largest_j = max(largest_j, float(np.max(np.abs(j-jacobian))))
            largest_c = max(largest_c, float(np.max(np.abs(c-value))))
            close(c, value, "independent rotation displacement", tolerance)
            close(j, jacobian, "independent rotation Jacobian", tolerance)
            close(track["compensated_normalized_displacement"], raw-value, "compensated displacement", tolerance)
    require(identifiers == sorted(identifiers), "reported feature ordering")
    require(check_lk(row["tracks"], pixels[start], pixels[end], row["detected_track_count"]) == row["reason"], "independent tracking reason")
    oracle = row["rotation_oracle"]
    tested = int(gamma is not None and bool(row["tracks"]))
    require(type(oracle["tested_tracks"]) is int and oracle["tested_tracks"] == tested and
            oracle["method"] == "cv2.Rodrigues" and oracle["central_difference_step_rad"] == 2e-6, "declared rotation oracle")
    if tested:
        require(type(oracle["maximum_jacobian_absolute_error"]) in (int,float) and
                0 <= oracle["maximum_jacobian_absolute_error"] <= tolerance and
                type(oracle["maximum_displacement_absolute_error"]) in (int,float) and
                0 <= oracle["maximum_displacement_absolute_error"] <= tolerance, "writer oracle bound")
    else:
        require(oracle["maximum_jacobian_absolute_error"] is None and oracle["maximum_displacement_absolute_error"] is None, "empty oracle")
    return {"tracks": len(row["tracks"]), "valid": int(row["valid"]), "oracle_window": tested,
            "max_jacobian_error": largest_j, "max_displacement_error": largest_c}


def check_ledger(rows, images, aligned):
    positions = {row["stamp_ns"]: i for i, row in enumerate(aligned)}
    require(len(rows) == len(images), "every original image ledger row")
    for i, (row, image) in enumerate(zip(rows, images)):
        j = positions.get(image["stamp_ns"])
        expected = {"schema_version": 1, "image_sequence_id": i+1, "header_stamp_ns": image["stamp_ns"],
                    "classification": "ACCOUNTED_ORIGINAL_DOWN_IMAGE", "source_image": image["ref"],
                    "source_camera_info": image["camera_info_ref"], "retained_front_epoch": j is not None,
                    "native_window_indices": [v for v in (i-1,i) if 0 <= v < len(images)-1],
                    "aligned_window_indices": [v for v in (j-1,j) if 0 <= v < len(aligned)-1] if j is not None else []}
        require(row == expected, "image ledger identity/schedule at image " + str(i))


def json_lines(path):
    with Path(path).open() as stream:
        for line in stream:
            require(bool(line.strip()), "blank JSONL row")
            yield loads(line)


def audit(replay_dir, protocol_path):
    replay_dir, protocol_path = Path(replay_dir).resolve(), Path(protocol_path).resolve()
    state = {"schema_version": 1, "status": "FAIL", "checks": {}, "counts": {}, "errors": [],
             "scope": "Independent recorded-pixel geometry/provenance audit; zero actual flow state updates; no metric depth/noise calibration or hardware validation",
             "input_files": {}, "script_sha256": digest(__file__)}
    start_time = time.monotonic()
    def checked(name, condition):
        state["checks"][name] = bool(condition)
        require(condition, name)
    try:
        cv2.setNumThreads(1)
        checked("closed_replay_input_file_set", {p.name for p in replay_dir.iterdir()} in (INPUTS, INPUTS | {"audit-result.json"}) and
                all((replay_dir/name).is_file() and not (replay_dir/name).is_symlink() for name in INPUTS))
        for name in sorted(INPUTS):
            path = replay_dir/name
            state["input_files"][name] = {"bytes": path.stat().st_size, "sha256": digest(path)}
        protocol = frozen_protocol(read(protocol_path))
        checked("saved_protocol_exact_identity", read(replay_dir/"protocol.json") == protocol)
        result, index = read(replay_dir/"replay-result.json"), read(replay_dir/"source-index.json")
        checked("writer_pass_and_all_checks", result["status"] == "PASS" and not result["errors"] and result["checks"] and
                all(v is True for v in result["checks"].values()))
        bag = Path(protocol["source_bag"]).resolve()
        checked("source_index_origin_and_protocol", index["schema_version"] == 1 and index["input_origin"] == ORIGIN and
                index["source_bag"] == str(bag) and index["protocol_sha256"] == digest(protocol_path) and result["input_origin"] == ORIGIN)
        binding = index["source_binding"]
        frozen = Path(binding["frozen_audit"])
        original = read(frozen)
        checked("frozen_audit_and_raw_hash_binding", original["status"] == "PASS" and digest(frozen) == binding["frozen_audit_sha256"] and
                binding["files_before"] == original["original_files_before"] and
                read(replay_dir/"raw-hashes.before.json") == binding)
        after = read(replay_dir/"raw-hashes.after.json")
        checked("raw_before_after_binding", after == {**binding, "files_after": binding["files_before"]} and result["source_binding"] == after)
        expected_files = {"metadata.yaml", *("rosbag_"+str(i)+".db3" for i in range(6))}
        checked("original_seven_file_set_and_sizes", set(binding["files_before"]) == expected_files and
                {p.name for p in bag.iterdir()} == expected_files and all(
                    (bag/name).is_file() and not (bag/name).is_symlink() and
                    (bag/name).stat().st_size == item["bytes"] for name, item in binding["files_before"].items()))
        anchor = result["source_index"]
        checked("source_index_anchored_sha", Path(anchor["path"]).resolve() == replay_dir/"source-index.json" and
                anchor["bytes"] == state["input_files"]["source-index.json"]["bytes"] and
                anchor["sha256"] == state["input_files"]["source-index.json"]["sha256"])
        pixels, cdr_count = check_sources(index, bag)
        state["counts"]["original_allowed_cdr_payloads_checked"] = cdr_count
        checked("all_original_allowed_cdr_payloads_and_headers", cdr_count > 0)
        messages = index["messages"]
        images, infos, front, imus = (messages[key] for key in ("down_images", "down_camera_info", "front_images", "imu"))
        info_by_stamp = {v["stamp_ns"]: v for v in infos}
        down_by_stamp = {v["stamp_ns"]: v for v in images}
        checked("one_camera_info_per_original_image", len(info_by_stamp) == len(infos) == len(images) and set(info_by_stamp) == set(down_by_stamp) and
                all(v["camera_info_ref"] == info_by_stamp[v["stamp_ns"]]["ref"] for v in images))
        checked("constant_calibration", len({json.dumps(v["camera"],sort_keys=True) for v in infos}) == 1)
        kept = front[::3]
        checked("retained_front_epoch_schedule", index["front_camera_keep_every_n"] == 3 and
                index["retained_front_stamps_ns"] == [v["stamp_ns"] for v in kept] and all(v["stamp_ns"] in down_by_stamp for v in kept))
        aligned = [down_by_stamp[v["stamp_ns"]] for v in kept]
        check_ledger(list(json_lines(replay_dir/"image-ledger.jsonl")), images, aligned)
        checked("every_original_image_ledger_and_window_memberships", True)
        stamps = [v["stamp_ns"] for v in imus]
        checked("raw_zero_gyro_covariance_is_unknown", all(v["angular_velocity_covariance_raw"] == [0.]*9 for v in imus) and index["gyro_buffer_capacity"] >= len(imus))
        counts = {"down_images": len(images), "down_camera_info": len(infos), "front_images": len(front),
                  "imu_messages": len(imus), "original_flow_messages": len(messages["old_flow"]), "image_ledger_rows": len(images),
                  "rotation_oracle_windows": 0}
        largest_j = largest_c = 0.
        for kind, frames in (("native", images), ("aligned", aligned)):
            n, valid, tracks, reasons = 0, 0, 0, Counter()
            for i, row in enumerate(json_lines(replay_dir/(kind+"-tracks.jsonl"))):
                require(i < len(frames)-1, "extra " + kind + " window")
                if kind == "aligned":
                    require(row["both_endpoints_are_retained_front_epochs"] is True, "aligned endpoint assertion")
                value = check_window(row, kind, i, frames[i], frames[i+1], imus, stamps, pixels,
                                     protocol["bag_checks"]["maximum_analytic_numeric_rotation_jacobian_error"])
                n += 1; valid += value["valid"]; tracks += value["tracks"]
                counts["rotation_oracle_windows"] += value["oracle_window"]
                largest_j = max(largest_j, value["max_jacobian_error"])
                largest_c = max(largest_c, value["max_displacement_error"])
                reasons[row["reason"]] += 1
            checked(kind+"_all_window_geometry_pixel_LK_gyro_and_no_fusion", n == len(frames)-1 and dict(reasons) == result[kind+"_reasons"])
            counts.update({kind+"_windows": n, kind+"_valid_windows": valid, kind+"_tracks": tracks})
            checked(kind+"_minimum_valid_track_windows", valid >= protocol["bag_checks"]["minimum_"+kind+"_valid_track_windows"])
        checked("all_reported_counts_independently_reproduced", counts == result["counts"] and len(images) >= 1000)
        state["counts"].update(counts)
        state["geometry"] = {"maximum_independent_jacobian_error_all_tracks": largest_j,
                             "maximum_independent_displacement_error_all_tracks": largest_c,
                             "independent_LK_pixel_tolerance": 1e-5}
        old_reasons, old_valid, positive, covered, both, ends, last = Counter(), 0, 0, 0, 0, 0, -1
        gyro_error = 0.
        retained = set(index["retained_front_stamps_ns"])
        for item in messages["old_flow"]:
            row = item["measurement"]
            a, b = row["acquisition_start_ns"], row["acquisition_end_ns"]
            require(type(a) is int and type(b) is int and a in down_by_stamp and b in down_by_stamp and b >= a and b > last and
                    row["source_image_stamps_ns"] == [a,b] and abs(row["dt_s"]-(b-a)*1e-9) <= 1e-12, "original flow integer windows")
            require(row["schema_version"] == 1 and row["frame_id"] == "impact_down_optical" and row["ground_truth_used"] is False and
                    row["dependencies"] == ["down_camera", "livox_imu"] and row["source_model"] == "gazebo_pixels_lk" and
                    row["metric_velocity_valid"] is False and row["metric_velocity_mps"] is None and row["scale_source"] == "unavailable" and
                    type(row["valid"]) is bool and type(row["gyro_valid"]) is bool, "original flow provenance/scope")
            last = b; old_reasons[row["reason"]] += 1; old_valid += row["valid"]
            both += row["valid"] and a in retained and b in retained
            ends += row["valid"] and b in retained
            if b > a:
                positive += 1
                gamma, _, _ = integrate_gyro(imus, stamps, a, b)
                covered += gamma is not None
                if row["gyro_valid"]:
                    require(gamma is not None, "old flow uncovered valid gyro")
                    gyro_error = max(gyro_error, float(np.linalg.norm(gamma-array(row["gyro_integral_optical_rad"], (3,), "old gyro"))))
        old_expected = {"count": len(messages["old_flow"]), "valid": old_valid, "invalid": len(messages["old_flow"])-old_valid,
                        "reason_counts": dict(old_reasons), "positive_windows": positive, "covered_positive_windows": covered,
                        "metric_velocity_valid_count": 0, "metric_velocity_nonnull_count": 0,
                        "native_both_endpoints_match_retained_front": both, "native_valid_end_matches_retained_front": ends}
        checked("original_flow_counts_timing_provenance_and_gyro_reproduced", positive == covered and gyro_error <= 1e-12 and
                all(result["old_flow"][k] == v for k,v in old_expected.items()) and
                abs(result["old_flow"]["max_gyro_reproduction_error_rad"]-gyro_error) <= 1e-12)
        boundaries = {"fusion_update_performed": False, "actual_flow_state_updates": 0, "independent_range_available": False,
                      "metric_depth_known": False, "pixel_noise_independent": False, "actual_pixel_covariance": None,
                      "actual_gyro_integral_covariance": None, "actual_state_gyro_cross_covariance": None,
                      "actual_cross_time_covariance": None, "hardware_readiness": "NOT_READY", "blocked_reasons": BLOCKS}
        checked("all_actual_noise_depth_and_cross_time_preconditions_block_fusion", result["boundaries"] == boundaries)
        checked("inputs_unchanged_after_audit", all((replay_dir/name).stat().st_size == record["bytes"] and digest(replay_dir/name) == record["sha256"]
                for name, record in state["input_files"].items()))
        checked("execution_completed", True)
    except Exception as error:
        state["errors"].append(type(error).__name__ + ": " + str(error))
        state["checks"]["execution_completed"] = False
    state["check_count"] = len(state["checks"])
    state["elapsed_wall_s"] = time.monotonic()-start_time
    state["status"] = "PASS" if state["checks"] and all(state["checks"].values()) and not state["errors"] else "FAIL"
    return state


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay-dir", type=Path, required=True)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    output = args.output or args.replay_dir/"audit-result.json"
    # Never overwrite an input artifact or create an entry in the protected bag.
    resolved = output.resolve()
    if resolved in {(args.replay_dir/name).resolve() for name in INPUTS} or resolved == args.protocol.resolve():
        parser.error("audit output must not replace an input")
    try:
        bag = Path(read(args.protocol)["source_bag"]).resolve()
        if resolved.is_relative_to(bag):
            parser.error("audit output must be outside the original bag")
    except (ValueError, KeyError, OSError, TypeError):
        pass  # Malformed input is recorded as a FAIL below.
    result = audit(args.replay_dir, args.protocol)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name+".tmp")
    temporary.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    temporary.replace(output)
    print(json.dumps({"status": result["status"], "check_count": result["check_count"], "counts": result["counts"], "errors": result["errors"]}))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
