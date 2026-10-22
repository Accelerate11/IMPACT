#!/usr/bin/env python3
"""Offline evidence gate for the two shared-state LIVO replay arms.

This program never subscribes or publishes. Truth is read only by this evaluator.
An initial SE(3) alignment fixes the coordinate gauge once; neither scale fitting
nor repeated alignment can conceal later drift. Passing this gate establishes
numerical usability and observed visual corrections on one simulation, not
hardware calibration, accuracy superiority, a protection level, or AP ingestion.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np


ARMS = ("arm_livo", "arm_no_visual")


def _reject_constant(value):
    raise ValueError("non-finite JSON constant: " + value)


def _parse_float(value):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("non-finite JSON number: " + value)
    return parsed


def _decode(text):
    return json.loads(text, parse_constant=_reject_constant, parse_float=_parse_float)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _array(value, shape, name):
    raw = np.asarray(value, dtype=object)
    if not all(_finite(element) for element in raw.flat):
        raise ValueError(name + " must contain finite numeric values")
    result = np.asarray(value, dtype=float)
    if result.size != math.prod(shape) or result.shape not in (shape, (math.prod(shape),)):
        raise ValueError(name + " has the wrong shape")
    result = result.reshape(shape)
    if not np.isfinite(result).all():
        raise ValueError(name + " contains non-finite values")
    return result


def _quaternion(row):
    a, b = row.get("quaternion_xyzw"), row.get("orientation_xyzw")
    if a is not None and b is not None and a != b:
        raise ValueError("conflicting quaternion aliases")
    q = _array(a if a is not None else b, (4,), "quaternion_xyzw")
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("quaternion must have a finite nonzero norm")
    return q / norm


def rotation_from_xyzw(q):
    x, y, z, w = _quaternion({"quaternion_xyzw": q})
    return np.array((
        (1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
        (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
        (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)),
    ))


def _pose(row):
    return _array(row.get("position"), (3,), "position"), _quaternion(row)


def _rotation_distance(a, b):
    # q_after * conjugate(q_before); atan2 retains very small corrections that
    # acos(trace(R)) rounds to zero. q and -q describe the same rotation.
    v = a[3] * (-b[:3]) + b[3] * a[:3] + np.cross(a[:3], -b[:3])
    w = a[3] * b[3] + float(a[:3] @ b[:3])
    return 2.0 * math.atan2(float(np.linalg.norm(v)), abs(w))


def _norm(values):
    values = np.asarray(values, dtype=float)
    scale = float(np.max(np.abs(values)))
    return 0.0 if scale == 0.0 else float(np.linalg.norm(values / scale)) * scale


def _rms(values):
    scale = max(values)
    return 0.0 if scale == 0.0 else float(np.sqrt(np.mean(np.square(np.asarray(values) / scale)))) * scale


def check_covariance(value, dimension, tolerance, *, require_nonzero=False):
    """Check raw covariance after diagonal scaling to correlation coordinates.

    Positive variance axes are scaled by their own standard deviation. A zero
    variance axis must have an exactly zero row/column. Negative variances fail.
    After checking scaled symmetry, a temporary symmetric matrix is used only
    for the eigenvalue calculation; the supplied covariance is never repaired.
    """
    matrix = _array(value, (dimension, dimension), "covariance")
    diagonal = np.diag(matrix)
    if np.any(diagonal < 0.0):
        raise ValueError("covariance has a negative variance")
    for axis in np.flatnonzero(diagonal == 0.0):
        if np.any(matrix[axis, :] != 0.0) or np.any(matrix[:, axis] != 0.0):
            raise ValueError("zero variance has nonzero cross covariance")
    if require_nonzero and not np.any(matrix != 0.0):
        raise ValueError("covariance is the default all-zero covariance")
    scales = np.sqrt(np.where(diagonal > 0.0, diagonal, 1.0))
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        balanced = matrix / scales[:, None] / scales[None, :]
    if not np.isfinite(balanced).all():
        raise ValueError("correlation-scaled covariance is non-finite")
    asymmetry = float(np.max(np.abs(balanced - balanced.T)))
    if asymmetry > tolerance:
        raise ValueError("covariance is not symmetric in correlation coordinates")
    minimum_eigenvalue = float(np.linalg.eigvalsh(0.5 * balanced + 0.5 * balanced.T)[0])
    if minimum_eigenvalue < -tolerance:
        raise ValueError("covariance is not PSD in correlation coordinates")
    return matrix, {"scaled_asymmetry_max": asymmetry,
                    "scaled_minimum_eigenvalue": minimum_eigenvalue,
                    "nonzero": bool(np.any(matrix != 0.0))}


def world_pose_covariance(covariance19, quaternion_xyzw):
    """[position_world, rotation_world] from [rotation_body, position_world, ...]."""
    projection = np.zeros((6, 19))
    projection[:3, 3:6] = np.eye(3)
    projection[3:, :3] = rotation_from_xyzw(quaternion_xyzw)
    return projection @ covariance19 @ projection.T


def _covariance_mapping_error(actual, expected):
    diagonal = np.maximum(np.abs(np.diag(actual)), np.abs(np.diag(expected)))
    scales = np.sqrt(np.where(diagonal > 0, diagonal, 1.0))
    return float(np.max(np.abs(actual - expected) / scales[:, None] / scales[None, :]))


def _stamp(row):
    stamp = row.get("stamp_s")
    if not _finite(stamp) or stamp <= 0.0:
        raise ValueError("stamp_s must be a finite positive acquisition time")
    return float(stamp)


def _stamp_ns(stamp):
    return int(round(stamp * 1_000_000_000))


def _nearest(stamps, value, tolerance):
    index = bisect.bisect_left(stamps, value)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
    if not candidates:
        return None
    closest = min(candidates, key=lambda i: (abs(stamps[i] - value), i))
    return closest if abs(stamps[closest] - value) <= tolerance else None


def align_and_measure(odometry, truth, *, tolerance_s, body_transform):
    """Nearest acquisition-time pairs and one initial SE(3) gauge alignment."""
    translation = _array(body_transform["translation_m"], (3,), "body translation")
    rotation = rotation_from_xyzw(body_transform["rotation_xyzw"])
    transformed = []
    for row in truth:
        position, quaternion = _pose(row)
        orientation = rotation_from_xyzw(quaternion)
        transformed.append((_stamp(row), position + orientation @ translation, orientation @ rotation))
    stamps = [row[0] for row in transformed]
    pairs = []
    for row in odometry:
        stamp = _stamp(row)
        index = _nearest(stamps, stamp, tolerance_s)
        if index is not None:
            pairs.append((row, transformed[index]))
    if not pairs:
        raise ValueError("no odometry/truth acquisition-time pairs")
    first_position, first_quaternion = _pose(pairs[0][0])
    first_truth = pairs[0][1]
    alignment_rotation = rotation_from_xyzw(first_quaternion) @ first_truth[2].T
    alignment_translation = first_position - alignment_rotation @ first_truth[1]
    errors, offsets, truth_frame_trajectory = [], [], []
    for row, reference in pairs:
        position, _ = _pose(row)
        error = position - (alignment_rotation @ reference[1] + alignment_translation)
        error_norm = _norm(error)
        if not math.isfinite(error_norm):
            raise ValueError("non-finite derived trajectory error")
        errors.append(error_norm)
        offsets.append(abs(_stamp(row) - reference[0]))
        truth_frame_trajectory.append({"stamp_s": _stamp(row),
            "position": (alignment_rotation.T @ (position - alignment_translation)).tolist()})
    return {
        "pair_count": len(pairs), "unmatched_count": len(odometry) - len(pairs),
        "ate_rms_m": _rms(errors),
        "ate_max_m": max(errors), "pair_time_offset_max_s": max(offsets),
        "alignment": {"method": "first_pair_SE3_no_scale",
            "first_odometry_stamp_s": _stamp(pairs[0][0]), "first_truth_stamp_s": first_truth[0],
            "rotation_truth_to_estimator": alignment_rotation.tolist(),
            "translation_truth_to_estimator_m": alignment_translation.tolist()},
    }, truth_frame_trajectory


def _protocol(protocol):
    if not isinstance(protocol, dict) or protocol.get("schema_version") != 1:
        raise ValueError("missing schema_version=1 protocol")
    if protocol.get("input_origin") != "R2A_simulation_replay" or protocol.get("arm_names") != list(ARMS):
        raise ValueError("wrong input origin or replay arms")
    if protocol.get("require_livo_accuracy_improvement") is not False:
        raise ValueError("protocol must not require LIVO accuracy superiority")
    for key in ("minimum_odometry_samples", "minimum_visual_updates", "minimum_selected_points"):
        if type(protocol.get(key)) is not int or protocol[key] <= 0:
            raise ValueError("invalid frozen integer threshold: " + key)
    for key in ("truth_match_tolerance_s", "maximum_ate_rms_m", "maximum_ate_max_m",
                "minimum_pose_delta", "minimum_covariance_delta", "covariance_correlation_tolerance"):
        if not _finite(protocol.get(key)) or protocol[key] <= 0:
            raise ValueError("invalid frozen threshold: " + key)
    for key in ("body_frame", "truth_body_frame"):
        if not isinstance(protocol.get(key), str) or not protocol[key]:
            raise ValueError("missing frozen body frame: " + key)
    transform = protocol.get("truth_body_from_estimator_body")
    if not isinstance(transform, dict):
        raise ValueError("missing explicit truth/estimator body transform")
    _array(transform.get("translation_m"), (3,), "body translation")
    rotation_from_xyzw(transform.get("rotation_xyzw"))


def analyze_records(arms, truth, source_stamps, protocol):
    """Pure evaluator entry point; all containers are already decoded records."""
    result = {"schema_version": 1, "gate": "R2B2_COMMON_STATE_LIVO_REPLAY", "status": "FAIL",
        "input_origin": "R2A_simulation_replay", "hardware_readiness": "NOT_READY",
        "truth_consumed_by": "offline_evaluator", "accuracy_superiority_required": False,
        "estimator_truth_isolation_boundary": "Sensor-only ingress is audited by the replay publisher/runner; this evaluator has no ROS publishers or subscribers",
        "scope": "Offline shared-state visual corrections and numerical usability; covariance is not a protection level",
        "checks": {}, "errors": [], "arms": {}}
    checks, errors = result["checks"], result["errors"]
    try:
        _protocol(protocol)
        result["protocol"] = protocol
        checks["frozen_protocol_valid"] = True
    except (ValueError, TypeError, OverflowError) as exc:
        errors.append("protocol: " + str(exc))
        return result

    def check(name, condition, detail=""):
        checks[name] = bool(condition)
        if not condition:
            errors.append(name + (": " + detail if detail else ""))

    tolerance = protocol["covariance_correlation_tolerance"]
    camera_ns = set()
    try:
        if not isinstance(source_stamps, dict):
            raise ValueError("source-stamps must be an object")
        for key in ("front_camera_stamps_s", "lidar_stamps_s", "imu_stamps_s"):
            values = source_stamps.get(key)
            if not isinstance(values, list) or not values or not all(_finite(v) and v > 0 for v in values):
                raise ValueError("missing finite positive acquisition index: " + key)
            if any(b <= a for a, b in zip(values, values[1:])):
                raise ValueError("acquisition index must be strictly increasing: " + key)
        camera_ns = {_stamp_ns(s) for s in source_stamps["front_camera_stamps_s"]}
        check("source_acquisition_index_valid", True)
    except (ValueError, TypeError, OverflowError) as exc:
        check("source_acquisition_index_valid", False, str(exc))

    good_truth = []
    previous = None
    for index, row in enumerate(truth):
        try:
            stamp = _stamp(row)
            _pose(row)
            if previous is not None and stamp <= previous:
                raise ValueError("truth acquisition stamps must be strictly increasing")
            previous = stamp
            good_truth.append(row)
        except (ValueError, TypeError, OverflowError, KeyError) as exc:
            errors.append(f"truth row {index}: {exc}")
    check("truth_valid", bool(good_truth) and len(good_truth) == len(truth))
    trajectories = {}

    for arm_name in ARMS:
        enabled = arm_name == "arm_livo"
        arm_result = {"odometry_count": 0, "visual_epoch_count": 0, "qualifying_visual_updates": 0,
                      "same_state_addresses": [], "maximum_pose_covariance_mapping_error": 0.0}
        result["arms"][arm_name] = arm_result
        arm = arms.get(arm_name, {}) if isinstance(arms, dict) else {}
        odometry, updates = arm.get("odometry", []), arm.get("visual_updates", [])
        good_odometry, good_updates, addresses = [], {}, set()
        previous_stamp = previous_epoch = None
        for index, row in enumerate(updates):
            try:
                if row.get("schema_version") != 1 or row.get("event") != "visual_update":
                    raise ValueError("wrong visual diagnostic schema/event")
                stamp, image_stamp = _stamp(row), row.get("image_stamp_s")
                if not _finite(image_stamp) or _stamp_ns(stamp) != _stamp_ns(image_stamp) or _stamp_ns(stamp) not in camera_ns:
                    raise ValueError("visual stamp is not an original kept camera acquisition stamp")
                epoch = row.get("epoch_index")
                if type(epoch) is not int or epoch <= 0 or (previous_epoch is not None and epoch <= previous_epoch):
                    raise ValueError("visual epoch indices must be strictly increasing")
                if previous_stamp is not None and stamp <= previous_stamp:
                    raise ValueError("visual acquisition stamps must be strictly increasing")
                previous_stamp, previous_epoch = stamp, epoch
                if row.get("visual_update_enabled") is not enabled or row.get("same_state_object") is not True:
                    raise ValueError("wrong visual arm or shared-state identity")
                before_address, after_address = row.get("state_address_before"), row.get("state_address_after")
                if not isinstance(before_address, str) or before_address != after_address or int(before_address, 16) <= 0:
                    raise ValueError("visual before/after state addresses differ or are invalid")
                addresses.add(before_address)
                provenance = row.get("provenance", {})
                if provenance.get("shared_state_id") != "fast_livo2_19d" or provenance.get("visual_uses_lidar_map") is not True or provenance.get("imu_state_propagation_shared") is not True:
                    raise ValueError("missing common-state/LiDAR-map provenance")
                if row.get("hardware_validated") is not False:
                    raise ValueError("simulation diagnostic cannot claim hardware validation")
                selected = row.get("selected_points")
                if type(selected) is not int or selected < 0:
                    raise ValueError("selected_points must be a nonnegative integer")
                p0, q0 = _pose(row["pose_before"])
                p1, q1 = _pose(row["pose_after"])
                covariance0, _ = check_covariance(row.get("covariance_before"), 19, tolerance, require_nonzero=True)
                covariance1, _ = check_covariance(row.get("covariance_after"), 19, tolerance, require_nonzero=True)
                dp = _norm(p1 - p0)
                dr = _rotation_distance(q1, q0)
                dc = _norm(covariance1 - covariance0)
                for key, actual in (("delta_position_m", dp), ("delta_rotation_rad", dr), ("covariance_delta_frobenius", dc)):
                    reported = row.get(key)
                    if not _finite(actual) or not _finite(reported) or reported < 0 or not math.isclose(reported, actual, rel_tol=tolerance, abs_tol=min(protocol["minimum_pose_delta"], protocol["minimum_covariance_delta"])):
                        raise ValueError("reported correction delta does not match before/after evidence: " + key)
                pose_changed = dp > protocol["minimum_pose_delta"] or dr > protocol["minimum_pose_delta"]
                covariance_changed = dc > protocol["minimum_covariance_delta"]
                if type(row.get("correction_applied")) is not bool or type(row.get("accepted")) is not bool or type(row.get("imu_initialized")) is not bool:
                    raise ValueError("correction/accepted/IMU flags must be explicit booleans")
                if not enabled and (row["correction_applied"] or pose_changed or covariance_changed):
                    raise ValueError("disabled visual arm applied a visual state/covariance correction")
                if enabled and row["correction_applied"] and selected < protocol["minimum_selected_points"]:
                    raise ValueError("visual correction used fewer than the frozen minimum selected points")
                if enabled and not row["correction_applied"] and (pose_changed or covariance_changed):
                    raise ValueError("unreported visual correction")
                qualifies = (enabled and row["accepted"] and row["imu_initialized"] and row["correction_applied"]
                    and selected >= protocol["minimum_selected_points"] and pose_changed and covariance_changed)
                arm_result["qualifying_visual_updates"] += int(qualifies)
                good_updates[epoch] = (stamp, row, covariance1, p1, q1)
            except (ValueError, TypeError, OverflowError, KeyError, np.linalg.LinAlgError) as exc:
                errors.append(f"{arm_name} visual row {index}: {exc}")
        arm_result["visual_epoch_count"] = len(updates)
        arm_result["same_state_addresses"] = sorted(addresses)
        check(arm_name + ".visual_diagnostics_valid", bool(updates) and len(good_updates) == len(updates))
        previous_stamp = previous_epoch = None
        for index, row in enumerate(odometry):
            try:
                stamp = _stamp(row)
                if previous_stamp is not None and stamp <= previous_stamp:
                    raise ValueError("odometry acquisition stamps must be strictly increasing")
                previous_stamp = stamp
                if _stamp_ns(stamp) not in camera_ns:
                    raise ValueError("odometry stamp is not an original kept camera acquisition stamp")
                epoch = row.get("epoch_index")
                if type(epoch) is not int or epoch <= 0 or (previous_epoch is not None and epoch <= previous_epoch):
                    raise ValueError("odometry epoch indices must be strictly increasing")
                previous_epoch = epoch
                if row.get("visual_update_enabled") is not enabled or epoch not in good_updates:
                    raise ValueError("odometry has no matching valid visual epoch/arm")
                update_stamp, update, update_covariance, update_position, update_quaternion = good_updates[epoch]
                if _stamp_ns(stamp) != _stamp_ns(update_stamp) or not update["accepted"] or not update["imu_initialized"]:
                    raise ValueError("odometry is not an accepted post-VIO epoch")
                position, quaternion = _pose(row)
                if "velocity_world_mps" in row:
                    _array(row["velocity_world_mps"], (3,), "velocity_world_mps")
                covariance, _ = check_covariance(row.get("covariance"), 19, tolerance, require_nonzero=True)
                pose_covariance, _ = check_covariance(row.get("pose_covariance"), 6, tolerance, require_nonzero=True)
                mapping_error = _covariance_mapping_error(pose_covariance, world_pose_covariance(covariance, quaternion))
                if not math.isfinite(mapping_error) or mapping_error > tolerance:
                    raise ValueError("pose covariance does not map full 19-state body rotation tangent to world tangent")
                arm_result["maximum_pose_covariance_mapping_error"] = max(arm_result["maximum_pose_covariance_mapping_error"], mapping_error)
                if _covariance_mapping_error(covariance, update_covariance) > tolerance or _norm(position - update_position) > protocol["minimum_pose_delta"] or _rotation_distance(quaternion, update_quaternion) > protocol["minimum_pose_delta"]:
                    raise ValueError("published odometry differs from the corrected shared state")
                good_odometry.append(row)
            except (ValueError, TypeError, OverflowError, KeyError, np.linalg.LinAlgError) as exc:
                errors.append(f"{arm_name} odometry row {index}: {exc}")
        arm_result["odometry_count"] = len(odometry)
        arm_result["valid_post_vio_odometry_count"] = len(good_odometry)
        check(arm_name + ".odometry_valid", bool(odometry) and len(good_odometry) == len(odometry))
        check(arm_name + ".minimum_odometry", len(good_odometry) >= protocol["minimum_odometry_samples"])
        check(arm_name + ".visual_update_count", arm_result["qualifying_visual_updates"] >= protocol["minimum_visual_updates"] if enabled else arm_result["qualifying_visual_updates"] == 0)
        try:
            metrics, trajectory = align_and_measure(good_odometry, good_truth,
                tolerance_s=protocol["truth_match_tolerance_s"], body_transform=protocol["truth_body_from_estimator_body"])
            arm_result.update(metrics)
            trajectories[arm_name] = trajectory
            check(arm_name + ".truth_pairing_complete", metrics["unmatched_count"] == 0 and metrics["pair_count"] >= protocol["minimum_odometry_samples"])
            check(arm_name + ".ate_rms_bound", metrics["ate_rms_m"] <= protocol["maximum_ate_rms_m"])
            check(arm_name + ".ate_max_bound", metrics["ate_max_m"] <= protocol["maximum_ate_max_m"])
        except (ValueError, TypeError, OverflowError, KeyError, np.linalg.LinAlgError) as exc:
            check(arm_name + ".truth_alignment", False, str(exc))

    if all(name in trajectories for name in ARMS):
        reference, compared = trajectories[ARMS[0]], trajectories[ARMS[1]]
        reference_stamps = [r["stamp_s"] for r in reference]
        differences, offsets = [], []
        for row in compared:
            index = _nearest(reference_stamps, row["stamp_s"], protocol["truth_match_tolerance_s"])
            if index is not None:
                differences.append(_norm(np.array(row["position"]) - reference[index]["position"]))
                offsets.append(abs(row["stamp_s"] - reference[index]["stamp_s"]))
        result["arm_difference"] = {"frame": "truth_world_after_each_initial_SE3",
            "pair_count": len(differences), "position_difference_rms_m": _rms(differences) if differences else None,
            "position_difference_max_m": max(differences) if differences else None,
            "pair_time_offset_max_s": max(offsets) if offsets else None,
            "livo_minus_no_visual_ate_rms_m": result["arms"][ARMS[0]]["ate_rms_m"] - result["arms"][ARMS[1]]["ate_rms_m"]}
        check("two_arm_pair_count", len(differences) >= protocol["minimum_odometry_samples"])
    else:
        check("two_arm_pair_count", False, "both trajectories are required")
    result["status"] = "PASS" if checks and all(checks.values()) and not errors else "FAIL"
    return result


def _read(path, *, lines=False):
    before = path.stat()
    data = path.read_bytes()
    after = path.stat()
    if (before.st_size, before.st_mtime_ns, before.st_ino) != (after.st_size, after.st_mtime_ns, after.st_ino):
        raise ValueError("input changed while reading: " + str(path))
    text = data.decode("utf-8")
    value = [_decode(line) for line in text.splitlines() if line.strip()] if lines else _decode(text)
    if lines and any(not isinstance(row, dict) for row in value):
        raise ValueError("JSONL records must be objects: " + str(path))
    return value, {"path": str(path), "size_bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def analyze_run(run_dir, protocol):
    run_dir = Path(run_dir)
    evidence, arms = {}, {}
    truth, evidence["truth"] = _read(run_dir / "truth.jsonl", lines=True)
    source, evidence["source_stamps"] = _read(run_dir / "source-stamps.json")
    for arm in ARMS:
        odometry, evidence[arm + ".odometry"] = _read(run_dir / arm / "odometry.jsonl", lines=True)
        updates, evidence[arm + ".visual_updates"] = _read(run_dir / arm / "visual-updates.jsonl", lines=True)
        arms[arm] = {"odometry": odometry, "visual_updates": updates}
    result = analyze_records(arms, truth, source, protocol)
    result["run_dir"] = str(run_dir.resolve())
    result["input_evidence"] = evidence
    result["acquisition_index_boundary"] = "Original kept sensor header index; byte/SQLite row provenance and original bag hashes are verified by the replay publisher/runner"
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--protocol", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    output = args.output or args.run_dir / "livo-replay-result.json"
    protocol_path = args.protocol or args.run_dir / "configuration/research_livo_replay_gate.json"
    try:
        protocol, evidence = _read(protocol_path)
        result = analyze_run(args.run_dir, protocol)
        result["protocol_evidence"] = evidence
    except (OSError, ValueError, TypeError, OverflowError, KeyError, np.linalg.LinAlgError) as exc:
        result = {"schema_version": 1, "gate": "R2B2_COMMON_STATE_LIVO_REPLAY", "status": "FAIL",
            "hardware_readiness": "NOT_READY", "errors": [type(exc).__name__ + ": " + str(exc)], "checks": {"inputs_readable": False}}
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(output)
    print(json.dumps({"status": result["status"], "output": str(output), "errors": result["errors"]}, ensure_ascii=False, allow_nan=False))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
