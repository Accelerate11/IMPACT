"""Narrow integration wiring/build-identity guards; never replay acceptance.

Numerical behavior is tested by C++ fixtures and actual native replay. These
source checks only prevent accidental bypass/removal of safety-critical hooks.
"""
from pathlib import Path
import importlib.util
import json
import pytest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "src/xq_fast_livo2"


def function(source, signature):
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[brace + 1:end - 1]


def test_native_hooks_precede_joint_and_odometry():
    source = (PACKAGE / "src/LIVMapper.cpp").read_text()
    imu = function(source, "void LIVMapper::processImu()")
    assert imu.index("p_imu->Process2(") < imu.index("research_joint->propagated(") < imu.index("state_propagat = _state")
    vio = function(source, "void LIVMapper::handleVIO()")
    assert vio.index("processFrame(") < vio.index("native_update(") < vio.index("processJointEpoch(") < vio.index("publish_odometry(")


def test_exact_acquisition_lookup_and_complete_five_source_tail():
    source = (PACKAGE / "src/LIVMapper.cpp").read_text()
    epoch = function(source, "void LIVMapper::processJointEpoch(")
    assert "joint_down_images.find(stamp_ns)" in epoch
    assert "joint_ranges.find(stamp_ns)" in epoch
    assert "lower_bound" not in epoch
    complete = function(source, "void LIVMapper::tryFinalizeProcessing()")
    for name in ("joint_down_received", "joint_range_received", "joint_last_down_ns", "joint_last_range_ns", "joint_last_barrier_ns", "joint_last_barrier_counts"):
        assert name in complete
    assert complete.index("joint_last_barrier_counts") < complete.index('"processing-result.json"')


def test_seed_range_not_reused_as_independent_update_and_no_truth_height():
    source = (PACKAGE / "src/research_joint_integration.cpp").read_text()
    assert source.count(".initialize_from_range(") == 1
    assert ".update_range(" not in source
    assert "world_plane_height_from_truth\\\":false" in source
    assert "engineering_pixel_covariance_not_empirically_calibrated" in source
    assert "gyro_precompensated\\\":false" in source


def test_first_gyro_seed_must_be_unpropagated_initialization():
    source = (PACKAGE / "src/research_joint_integration.cpp").read_text()
    body = function(source, "void JointIntegration::propagated(")
    assert body.index("q.norm()") < body.index("p.seed_gyro(state)")
    assert "qgyro.norm()" in body and "f-NativeCov::Identity()" in body
    assert "initial_gyro_component" in body


def test_original_native_control_is_explicitly_separate_from_joint_no_flow():
    source = (PACKAGE / "src/research_joint_integration.cpp").read_text()
    native = function(source, "void JointIntegration::native_update(")
    assert native.index('p.mode=="native"') < native.index("adopt_native_update(")
    epoch = function(source, "void JointIntegration::epoch(")
    assert epoch.index('p.mode=="joint_no_flow"') < epoch.index("update_tracks(")


@pytest.fixture
def manifest_module():
    path = ROOT / "scripts/check_joint_livo_build_manifest.py"
    spec = importlib.util.spec_from_file_location("joint_build_manifest_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_tree_identity_detects_content_and_file_set_changes(tmp_path, manifest_module):
    item = tmp_path / "native.cpp"
    item.write_bytes(b"original")
    original = manifest_module.tree(tmp_path)
    item.write_bytes(b"modified")
    assert manifest_module.tree(tmp_path) != original
    item.write_bytes(b"original")
    (tmp_path / "hidden.hpp").write_bytes(b"extra")
    assert manifest_module.tree(tmp_path) != original


def test_build_audit_rejects_source_mutation_and_records_both_sides(tmp_path, monkeypatch, manifest_module):
    attempt = tmp_path / "attempt"
    attempt.mkdir()
    before = {"source_files": {"native.cpp": {"sha256": "a"}}}
    after = {"source_files": {"native.cpp": {"sha256": "b"}}}
    protected = {"livo_install": {"binary": "unchanged"}, "livo_build": {}}
    (attempt / "inputs.before.json").write_text(json.dumps(before))
    (attempt / "protected.before.json").write_text(json.dumps(protected))
    monkeypatch.setattr(manifest_module, "inputs", lambda _: after)
    monkeypatch.setattr(manifest_module, "protected", lambda _: protected)
    with pytest.raises(ValueError, match="changed during compilation"):
        manifest_module.main(["--workspace-root", str(tmp_path), "--mode", "audit", "--record-dir", str(attempt)])
    assert json.loads((attempt / "inputs.after.json").read_text()) == after
    assert not (tmp_path / "joint_livo_install/.impact-joint-build-manifest.json").exists()


def test_build_audit_rejects_protected_prefix_mutation(tmp_path, monkeypatch, manifest_module):
    attempt = tmp_path / "attempt"
    attempt.mkdir()
    inputs = {"source_files": {"native.cpp": {"sha256": "a"}}}
    (attempt / "inputs.before.json").write_text(json.dumps(inputs))
    (attempt / "protected.before.json").write_text(json.dumps({"livo_install": {"binary": "old"}}))
    monkeypatch.setattr(manifest_module, "inputs", lambda _: inputs)
    monkeypatch.setattr(manifest_module, "protected", lambda _: {"livo_install": {"binary": "mutated"}})
    with pytest.raises(ValueError, match="old livo_install/livo_build changed"):
        manifest_module.main(["--workspace-root", str(tmp_path), "--mode", "audit", "--record-dir", str(attempt)])


def test_isolated_build_prefixes_never_target_old_install():
    source = (ROOT / "scripts/build_joint_livo_isolated.sh").read_text()
    assert '--build-base "${workspace_root}/joint_livo_build"' in source
    assert '--install-base "${workspace_root}/joint_livo_install"' in source
    assert '--install-base "${workspace_root}/livo_install"' not in source
    assert source.index("--mode audit") < source.index('if [[ "${build_status}" -ne 0 ]]')
