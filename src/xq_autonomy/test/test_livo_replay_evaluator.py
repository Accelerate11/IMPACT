"""Synthetic geometry/evidence checks; no estimator, bag replay, or hardware."""

from copy import deepcopy
import json
import math
from pathlib import Path
import runpy

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[3]
API = runpy.run_path(str(ROOT / "scripts/analyze_livo_replay.py"))
ANALYZE = API["analyze_records"]
POSE_COVARIANCE = API["world_pose_covariance"]


def yaw_quaternion(yaw):
    return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]


def fixture():
    protocol = json.loads((ROOT / "config/research_livo_replay_gate.json").read_text())
    truth, arms, stamps = [], {}, []
    for index in range(protocol["minimum_odometry_samples"] + 2):
        stamp = 1.0 + index * 0.2
        stamps.append(stamp)
        truth.append({"stamp_s": stamp, "position": [index * 0.02, 0.0, 1.0],
                      "orientation_xyzw": yaw_quaternion(index * 0.001)})
    global_rotation = API["rotation_from_xyzw"](yaw_quaternion(0.4))
    covariance = np.diag(np.linspace(0.001, 0.019, 19))
    covariance[0, 3] = covariance[3, 0] = 0.0001
    for name, enabled in (("arm_livo", True), ("arm_no_visual", False)):
        odometry, updates = [], []
        for index, reference in enumerate(truth):
            position = global_rotation @ np.array(reference["position"]) + [2.0, -1.0, 0.3]
            # Both arms are usable, while LIVO is slightly LESS accurate. This
            # must pass: the contract does not promise accuracy superiority.
            if enabled:
                position[0] += 0.01 * math.sin(index * 0.1)
            quaternion = yaw_quaternion(0.4 + index * 0.001)
            after = {"position": position.tolist(), "quaternion_xyzw": quaternion}
            before = deepcopy(after)
            if enabled:
                before["position"][0] -= 0.0001
            covariance_before = covariance * (1.01 if enabled else 1.0)
            dp = float(np.linalg.norm(position - before["position"]))
            dc = float(np.linalg.norm(covariance - covariance_before))
            updates.append({"schema_version": 1, "event": "visual_update", "epoch_index": index + 1,
                "first_post_vio_epoch": index == 0, "stamp_s": reference["stamp_s"],
                "image_stamp_s": reference["stamp_s"], "visual_update_enabled": enabled,
                "selected_points": 12, "correction_applied": enabled,
                "same_state_object": True, "state_address_before": "0x1234", "state_address_after": "0x1234",
                "pose_before": before, "pose_after": after,
                "covariance_before": covariance_before.tolist(), "covariance_after": covariance.tolist(),
                "delta_position_m": dp, "delta_rotation_rad": 0.0, "covariance_delta_frobenius": dc,
                "imu_initialized": True, "accepted": True, "hardware_validated": False,
                "provenance": {"shared_state_id": "fast_livo2_19d", "visual_uses_lidar_map": True,
                               "imu_state_propagation_shared": True}})
            odometry.append({"event": "odometry", "epoch_index": index + 1, "stamp_s": reference["stamp_s"],
                **after, "covariance": covariance.tolist(), "visual_update_enabled": enabled,
                "pose_covariance": POSE_COVARIANCE(covariance, quaternion).tolist()})
        arms[name] = {"odometry": odometry, "visual_updates": updates}
    return arms, truth, {name: list(stamps) for name in
        ("front_camera_stamps_s", "lidar_stamps_s", "imu_stamps_s")}, protocol


def test_one_initial_se3_alignment_recovers_gauge_without_scale_fitting():
    truth = [{"stamp_s": i + 1.0, "position": [i * 0.1, (i % 3) * 0.2, i * 0.03],
              "orientation_xyzw": yaw_quaternion(0.02 * i)} for i in range(20)]
    rotation = API["rotation_from_xyzw"](yaw_quaternion(0.7))
    odometry = [{"stamp_s": row["stamp_s"], "position": (rotation @ row["position"] + [5.0, -3.0, 2.0]).tolist(),
                 "quaternion_xyzw": yaw_quaternion(0.7 + 0.02 * i)} for i, row in enumerate(truth)]
    metrics, _ = API["align_and_measure"](odometry, truth, tolerance_s=0.03,
        body_transform={"translation_m": [0, 0, 0], "rotation_xyzw": [0, 0, 0, 1]})
    assert metrics["ate_max_m"] < 1e-12
    assert metrics["pair_count"] == 20
    odometry[-1]["position"][0] += 0.5
    drift, _ = API["align_and_measure"](odometry, truth, tolerance_s=0.03,
        body_transform={"translation_m": [0, 0, 0], "rotation_xyzw": [0, 0, 0, 1]})
    assert drift["ate_max_m"] == pytest.approx(0.5)


def test_covariance_projection_rotates_body_tangent_and_all_cross_blocks():
    covariance = np.eye(19)
    covariance[:6, :6] = np.diag([1, 4, 9, 16, 25, 36])
    covariance[3, 0] = covariance[0, 3] = 0.2
    projected = POSE_COVARIANCE(covariance, yaw_quaternion(math.pi / 2))
    assert np.diag(projected) == pytest.approx([16, 25, 36, 4, 1, 9])
    assert projected[0, 4] == projected[4, 0] == pytest.approx(0.2)
    assert projected[0, 3] == pytest.approx(0.0, abs=1e-14)


def test_valid_shared_state_evidence_passes_without_livo_accuracy_advantage():
    result = ANALYZE(*fixture())
    assert result["status"] == "PASS", result["errors"]
    assert result["arms"]["arm_livo"]["qualifying_visual_updates"] >= 100
    assert result["arms"]["arm_livo"]["ate_rms_m"] > result["arms"]["arm_no_visual"]["ate_rms_m"]
    assert result["accuracy_superiority_required"] is False


@pytest.mark.parametrize("problem", ["duplicate", "backward", "restamped"])
def test_bad_odometry_acquisition_time_fails_even_when_truth_is_nearby(problem):
    arms, truth, source, protocol = fixture()
    rows = arms["arm_livo"]["odometry"]
    rows[5]["stamp_s"] = rows[4]["stamp_s"] + {"duplicate": 0.0, "backward": -0.01, "restamped": 0.001}[problem]
    result = ANALYZE(arms, truth, source, protocol)
    assert result["status"] == "FAIL"
    assert not result["checks"]["arm_livo.odometry_valid"]


@pytest.mark.parametrize("problem", ["zero_pose", "indefinite19", "asymmetric19", "wrong_world_tangent"])
def test_default_or_invalid_or_wrong_frame_covariance_cannot_pass(problem):
    arms, truth, source, protocol = fixture()
    if problem == "zero_pose":
        arms["arm_livo"]["odometry"][5]["pose_covariance"] = [0.0] * 36
    elif problem == "wrong_world_tangent":
        covariance = np.array(arms["arm_livo"]["odometry"][5]["covariance"])
        indices = [3, 4, 5, 0, 1, 2]
        arms["arm_livo"]["odometry"][5]["pose_covariance"] = covariance[np.ix_(indices, indices)].tolist()
    else:
        matrix = arms["arm_livo"]["visual_updates"][5]["covariance_before"]
        matrix[0][1] = 0.1
        if problem == "indefinite19":
            matrix[1][0] = 0.1
    result = ANALYZE(arms, truth, source, protocol)
    assert result["status"] == "FAIL"
    assert any("covariance" in error for error in result["errors"])


def test_tiny_roundoff_negative_eigenvalue_is_checked_in_correlation_coordinates():
    covariance = np.array([[1e-12, (1.0 + 1e-10) * 1e-3],
                           [(1.0 + 1e-10) * 1e-3, 1e6]])
    _, report = API["check_covariance"](covariance, 2, 1e-9)
    assert -1e-9 <= report["scaled_minimum_eigenvalue"] < 0
    covariance[0, 1] = covariance[1, 0] = (1 + 1e-5) * 1e-3
    with pytest.raises(ValueError, match="PSD"):
        API["check_covariance"](covariance, 2, 1e-9)


def test_false_disabled_arm_correction_and_forged_state_identity_fail():
    arms, truth, source, protocol = fixture()
    arms["arm_no_visual"]["visual_updates"][1]["correction_applied"] = True
    arms["arm_livo"]["visual_updates"][1]["state_address_after"] = "0x5678"
    result = ANALYZE(arms, truth, source, protocol)
    assert result["status"] == "FAIL"
    assert any("disabled visual arm" in error for error in result["errors"])
    assert any("state addresses" in error for error in result["errors"])


def test_missing_input_still_writes_a_failure_result(tmp_path):
    result_path = tmp_path / "result.json"
    status = API["main"](["--run-dir", str(tmp_path), "--protocol",
        str(ROOT / "config/research_livo_replay_gate.json"), "--output", str(result_path)])
    assert status == 1
    result = json.loads(result_path.read_text())
    assert result["status"] == "FAIL"
    assert result["checks"]["inputs_readable"] is False


@pytest.mark.parametrize("invalid_number", ["NaN", "Infinity", "1e999"])
def test_nonfinite_json_input_cannot_prevent_a_failure_artifact(tmp_path, invalid_number):
    protocol = tmp_path / "invalid-protocol.json"
    protocol.write_text('{"schema_version":1,"hidden_number":' + invalid_number + '}')
    result_path = tmp_path / "result.json"
    status = API["main"](["--run-dir", str(tmp_path), "--protocol", str(protocol),
                          "--output", str(result_path)])
    assert status == 1
    assert json.loads(result_path.read_text())["status"] == "FAIL"
