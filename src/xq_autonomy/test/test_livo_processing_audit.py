"""Independent synthetic completion ledger tests, without ROS or real replay."""

from copy import deepcopy
import json
from pathlib import Path
import runpy
import shutil

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
AUDIT = runpy.run_path(str(ROOT / "scripts/audit_livo_processing.py"))["audit_processing"]


def write(path, value, *, lines=False):
    path.write_text("".join(json.dumps(row, allow_nan=False) + "\n" for row in value)
                    if lines else json.dumps(value, allow_nan=False) + "\n")


def fixture(tmp_path):
    run = tmp_path / "synthetic_processing"
    arm = run / "arm_livo"
    arm.mkdir(parents=True)
    (run / "configuration").mkdir()
    protocol = json.loads((ROOT / "config/research_livo_replay_gate.json").read_text())
    protocol["source_bag"] = "/synthetic/r2a/rosbag"
    write(run / "configuration/research_livo_replay_gate.json", protocol)
    shutil.copyfile(ROOT / "src/xq_fast_livo2/config/r2_snapshot.yaml", run / "configuration/r2_snapshot.yaml")
    times = {"front": [1_000_000_000 + i * 100_000_000 for i in range(6)],
             "imu": [1_000_000_000 + i * 100_000_000 for i in range(7)],
             "lidar": [1_100_000_000, 1_300_000_000, 1_450_000_000]}
    entries = [{"sensor": key, "stamp_ns": stamp} for key, values in times.items() for stamp in values]
    entries.sort(key=lambda row: (row["stamp_ns"], {"imu": 0, "lidar": 1, "front": 2}[row["sensor"]]))
    source = {"schema_version": 1, "input_origin": "R2A_simulation_replay", "bag": protocol["source_bag"],
              "camera_keep_every_n": 3, "entries": entries,
              "published_sensor_counts": {key: len(values) for key, values in times.items()},
              "camera_info": {"width": 320, "height": 240, "distortion_model": "plumb_bob",
                  "k": [277.128129211, 0, 159.5, 0, 277.128129211, 119.5, 0, 0, 1], "d": [0] * 5}}
    kinds = ["startup_discard", "startup_discard", "imu_initialization",
             "processed_postVIO", "processed_postVIO", "terminal_uncovered"]
    ledger, diagnostics, odometry = [], [], []
    for index, (stamp, kind) in enumerate(zip(times["front"], kinds)):
        processed = kind == "processed_postVIO"
        epoch = index - 2 if processed else None
        ledger.append({"schema_version": 1, "event": "image_epoch", "image_sequence_id": index + 1,
            "header_stamp_ns": stamp, "classification": kind, "epoch_index": epoch,
            "lio_points": 50 if processed else 0, "odom_published": processed,
            "reason": "synthetic coverage/init reason"})
        if processed:
            diagnostics.append({"schema_version": 1, "event": "visual_update", "epoch_index": epoch,
                "stamp_s": stamp * 1e-9, "image_stamp_s": stamp * 1e-9,
                "visual_update_enabled": True, "imu_initialized": True, "accepted": True})
            odometry.append({"schema_version": 1, "event": "odometry", "epoch_index": epoch,
                "stamp_s": stamp * 1e-9, "visual_update_enabled": True, "post_vio_epoch": True})
    expected = {"image": 6, "lidar": 3, "imu": 7}
    completion = {"schema_version": 1, "status": "COMPLETE", "complete": True,
        "expected_messages": expected, "received_messages": deepcopy(expected),
        "first_header_stamp_ns": {name: times[key][0] for name, key in (("image", "front"), ("lidar", "lidar"), ("imu", "imu"))},
        "last_header_stamp_ns": {name: times[key][-1] for name, key in (("image", "front"), ("lidar", "lidar"), ("imu", "imu"))},
        "last_eligible_image_stamp_ns": 1_400_000_000, "last_completed_post_vio_header_stamp_ns": 1_400_000_000,
        "image_classification_counts": {kind: kinds.count(kind) for kind in set(kinds)},
        "image_ledger_rows": 6, "visual_diagnostic_rows": 2, "odometry_rows": 2, "eligible_image_rows": 3,
        "diagnostics_flushed": True, "odometry_flushed": True, "ledger_flushed": True,
        "all_inputs_received": True, "all_image_messages_accounted": True}
    write(arm / "image-epochs.jsonl", ledger, lines=True)
    write(arm / "visual-updates.jsonl", diagnostics, lines=True)
    write(arm / "odometry.jsonl", odometry, lines=True)
    write(arm / "processing-result.json", completion)
    return arm, source, ledger, diagnostics, odometry, completion


def test_complete_images_include_explicit_startup_initialization_and_uncovered_tail(tmp_path):
    arm, source, *_ = fixture(tmp_path)
    result = AUDIT(arm, source)
    assert result["status"] == "PASS", result["errors"]
    assert result["counts"]["source_images"] == 6
    assert result["counts"]["processed_images"] == 2
    assert result["last_eligible_image_stamp_ns"] == 1_400_000_000


def test_ledger_completion_order_need_not_equal_callback_order(tmp_path):
    arm, source, ledger, *_ = fixture(tmp_path)
    write(arm / "image-epochs.jsonl", list(reversed(ledger)), lines=True)
    assert AUDIT(arm, source)["status"] == "PASS"


@pytest.mark.parametrize("problem", ["missing_image", "duplicate_image", "missing_diag", "missing_odom", "wrong_epoch"])
def test_received_counts_and_complete_flag_cannot_hide_partial_processing(tmp_path, problem):
    arm, source, ledger, diagnostics, odometry, _ = fixture(tmp_path)
    if problem == "missing_image":
        write(arm / "image-epochs.jsonl", ledger[:-1], lines=True)
    elif problem == "duplicate_image":
        write(arm / "image-epochs.jsonl", ledger + [ledger[-1]], lines=True)
    elif problem == "missing_diag":
        write(arm / "visual-updates.jsonl", diagnostics[:-1], lines=True)
    elif problem == "missing_odom":
        write(arm / "odometry.jsonl", odometry[:-1], lines=True)
    elif problem == "wrong_epoch":
        odometry[-1]["epoch_index"] = 99
        write(arm / "odometry.jsonl", odometry, lines=True)
    assert AUDIT(arm, source)["status"] == "FAIL"


@pytest.mark.parametrize("problem", ["covered_terminal", "late_initialization", "uncovered_processed", "invented_odometry"])
def test_exclusions_cannot_conceal_covered_work_or_invent_uncovered_state(tmp_path, problem):
    arm, source, ledger, *_ = fixture(tmp_path)
    row = ledger[4]
    if problem == "covered_terminal":
        row.update(classification="terminal_uncovered", epoch_index=None, odom_published=False)
    elif problem == "late_initialization":
        row.update(classification="imu_initialization", epoch_index=None, odom_published=False)
    elif problem == "uncovered_processed":
        ledger[-1].update(classification="processed_postVIO", epoch_index=3, odom_published=True, lio_points=50)
    elif problem == "invented_odometry":
        ledger[0]["odom_published"] = True
    write(arm / "image-epochs.jsonl", ledger, lines=True)
    assert AUDIT(arm, source)["status"] == "FAIL"


@pytest.mark.parametrize("key,value", [("last_completed_post_vio_header_stamp_ns", 1_300_000_000),
                                       ("diagnostics_flushed", False), ("eligible_image_rows", 99)])
def test_completion_claims_must_match_actual_target_counts_and_flush(tmp_path, key, value):
    arm, source, _, _, _, completion = fixture(tmp_path)
    completion[key] = value
    write(arm / "processing-result.json", completion)
    assert AUDIT(arm, source)["status"] == "FAIL"


def test_changed_camera_extrinsic_cannot_reuse_frozen_simulation_geometry(tmp_path):
    arm, source, *_ = fixture(tmp_path)
    path = arm.parent / "configuration/r2_snapshot.yaml"
    values = yaml.safe_load(path.read_text())
    values["/**"]["ros__parameters"]["extrin_calib"]["Pcl"][2] *= -1
    path.write_text(yaml.safe_dump(values))
    result = AUDIT(arm, source)
    assert result["status"] == "FAIL"
    assert result["checks"]["frozen_camera_from_lidar"] is False


def test_missing_completion_record_returns_fail_without_starting_ros(tmp_path):
    arm, source, *_ = fixture(tmp_path)
    (arm / "processing-result.json").unlink()
    result = AUDIT(arm, source)
    assert result["status"] == "FAIL"
    assert result["errors"]
