"""Small synthetic checks; never read the recorded bag or start ROS."""
import importlib.util
import json
from pathlib import Path
import struct

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("prepare_flow_geometry_replay", ROOT / "scripts/prepare_flow_geometry_replay.py")
writer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(writer)


def test_header_prefix_preserves_epoch_nanoseconds_without_front_pixel_decode():
    frame = b"impact_front_optical\x00"
    blob = b"\x00\x01\x00\x00" + struct.pack("<iII", 1_700_000_000, 123456789, len(frame)) + frame
    assert writer.header_from_cdr(blob) == (1_700_000_000_123456789, "impact_front_optical")
    # The parser accepts a header-only fixture and never needs front RGB bytes.
    with pytest.raises(ValueError):
        writer.header_from_cdr(b"\x00\x00\x00\x00" + blob[4:])
    with pytest.raises(ValueError):
        writer.header_from_cdr(blob[:-1])


def test_output_guard_cannot_create_an_entry_inside_original_bag(tmp_path):
    protocol = json.loads((ROOT / "config/research_flow_precondition_gate.json").read_text())
    bag = tmp_path / "original"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("unchanged")
    protocol["source_bag"] = str(bag)
    config = tmp_path / "protocol.json"
    config.write_text(json.dumps(protocol))
    target = bag / "accidental-output"
    assert writer.main(["--protocol", str(config), "--output", str(target)]) == 2
    assert not target.exists()
    assert (bag / "metadata.yaml").read_text() == "unchanged"


@pytest.mark.parametrize("field,value", [
    ("actual_flow_state_updates_must_equal", 1),
    ("unknown_covariance_must_block_fusion", False),
    ("require_original_payload_and_header_identity", False),
])
def test_fusion_or_provenance_preconditions_cannot_be_disabled(tmp_path, field, value):
    protocol = json.loads((ROOT / "config/research_flow_precondition_gate.json").read_text())
    protocol["bag_checks"][field] = value
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(protocol))
    with pytest.raises(ValueError):
        writer.protocol_config(path)


def test_real_lk_window_retains_pixel_tracks_gyro_brackets_and_unknown_statistics():
    writer.cv2.setNumThreads(1)
    camera = writer.CameraModel(320, 240, 277.128129211, 277.128129211, 159.5, 119.5)
    gray = np.zeros((240, 320), np.uint8)
    for y in range(20, 220, 24):
        for x in range(20, 300, 24):
            cv2.rectangle(gray, (x, y), (x+7, y+7), 255, -1)
    image = np.repeat(gray[:, :, None], 3, axis=2)
    shifted = cv2.warpAffine(image, np.array([[1., 0., 2.], [0., 1., -1.]]), (320, 240))
    start, end = 1_000_000_000, 1_066_000_000
    gyro = writer.GyroBuffer(maximum_samples=20000)
    imus = []
    for i in range(15):
        stamp = start + i*5_000_000
        ref = {"synthetic_imu_index": i}
        imus.append({"stamp_ns": stamp, "ref": ref})
        assert gyro.append(stamp, [0.01, -0.02, 0.003])
    a = {"stamp_ns": start, "pixels": image, "ref": {"synthetic_image": 0}, "camera_info_ref": {"synthetic_info": 0}}
    b = {"stamp_ns": end, "pixels": shifted, "ref": {"synthetic_image": 1}, "camera_info_ref": {"synthetic_info": 1}}
    row = writer.extract_window("native", 0, a, b, camera, gyro,
        {"minimum_tracks": 12, "maximum_tracks": 150, "forward_backward_threshold_px": 1.0},
        imus, [item["stamp_ns"] for item in imus])
    assert row["valid"] is True and row["track_count"] >= 12
    assert row["gyro"]["sample_index_range_in_source_index"] == [0, 14]
    assert row["gyro"]["first_sample_ref"] == imus[0]["ref"]
    assert row["gyro"]["last_sample_ref"] == imus[14]["ref"]
    assert all(track["J_gamma"] is not None and track["c_gamma"] is not None for track in row["tracks"])
    assert row["rotation_oracle"]["tested_tracks"] == 1
    assert row["rotation_oracle"]["maximum_jacobian_absolute_error"] < 1e-7
    assert row["noise_statistics"]["status"] == "UNAVAILABLE"
    assert row["noise_statistics"]["R_effective"] is None
    assert row["noise_statistics"]["C_effective"] is None
    assert row["depth_m"] is None and row["fusion_update_performed"] is False
    assert "CROSS_TIME_COVARIANCE_UNKNOWN" in row["blocked_reasons"]
