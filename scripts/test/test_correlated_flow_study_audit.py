"""Small arithmetic and hostile-artifact checks, never a reduced formal study."""

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import zipfile

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "audit_correlated_flow_study.py"
PROTOCOL = SCRIPT.parent.parent / "config/research_flow_precondition_gate.json"
SPEC = importlib.util.spec_from_file_location("independent_flow_study_audit", SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def _protocol(tmp_path, draws=40000):
    protocol = json.loads(PROTOCOL.read_text())
    protocol["synthetic_study"]["draws_per_seed_and_case"] = draws
    path = tmp_path / "protocol.json"
    path.write_text(json.dumps(protocol))
    return path


def _early_artifacts(tmp_path):
    study = tmp_path / "study"
    study.mkdir()
    archive = study / "samples.npz"
    archive.write_bytes(b"not a valid archive")
    arrays = {}
    for case in range(3):
        for seed in (17019, 31183, 92401):
            for field in AUDIT.FIELDS:
                arrays[f"case{case}_seed{seed}_{field}"] = {
                    "shape": [40000, 6 if field == "latent" else 2], "dtype": "float64"}
    index = {"path": str(archive.resolve()), "bytes": archive.stat().st_size,
             "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(), "arrays": arrays}
    result = {"schema_version": 1, "status": "PASS", "input_origin": AUDIT.ORIGIN,
              "independent_trials": 360000, "raw_arrays": copy.deepcopy(index)}
    _write_metadata(study, index, result)
    return study, index, result


def _write_metadata(study, index, result):
    result["raw_arrays"] = copy.deepcopy(index)
    (study / "raw-array-index.json").write_text(json.dumps(index))
    (study / "study-result.json").write_text(json.dumps(result))


def test_primitive_rng_literal_fixture_and_all_six_coordinates():
    # Synthetic PCG64 fixture, not sensor captures. Pins seed/case identity.
    literal = np.array([
        [1.5802681085935282, 1.4036115657508108, -0.18085859597706797,
         -0.46676769462983636, 0.07135195987467874, 0.5325139579813746],
        [1.0639946046814839, -0.0668898261604823, 0.1959801171540661,
         1.3529220830227795, 0.5794969039645591, 0.29544841260598],
    ])
    latent, error, noise, innovation = AUDIT._generated_block(17019, 0, 0.0, 2, 0.2)
    np.testing.assert_array_equal(latent, literal)
    np.testing.assert_array_equal(error, 0.2 * literal[:, 2:4])
    np.testing.assert_array_equal(noise, 0.2 * literal[:, 4:6])
    np.testing.assert_array_equal(innovation, error + noise)
    other = AUDIT._generated_block(17019, 1, 0.0, 2, 0.2)[0]
    assert not np.array_equal(other, latent)


def test_statistics_matches_hand_computed_covariance_and_count():
    errors = np.array([[0.1, 0], [-0.1, 0], [0.3, 0], [-0.3, 0]])
    result = AUDIT._statistics(errors, np.eye(2) * 0.01, np.array([1., 0.]), AUDIT.QUANTILE)
    np.testing.assert_allclose(result["empirical_covariance_xy_mps2"], [[0.05, 0], [0, 0]])
    assert result["covered_trials"] == 2
    assert result["coverage"] == 0.5
    assert result["predicted_directional_variance_mps2"] == 0.01
    np.testing.assert_allclose(result["relative_covariance_frobenius_error"], np.sqrt(17.0 / 2.0))


@pytest.mark.parametrize("successes", [0, 95, 100])
def test_wilson_score_endpoints_against_closed_form_boundaries(successes):
    interval = AUDIT._score_interval(successes, 100)
    z2 = AUDIT.QUANTILE ** 2
    expected = ([0., z2 / (100 + z2)] if successes == 0 else
                [100 / (100 + z2), 1.] if successes == 100 else
                [0.8882495307680809, 0.978456320845632])
    np.testing.assert_allclose(interval, expected, rtol=0, atol=2e-16)


def test_twenty_draw_development_protocol_cannot_receive_formal_pass(tmp_path):
    protocol = _protocol(tmp_path, draws=20)
    study = tmp_path / "development_twenty_draws"
    study.mkdir()
    result = AUDIT.audit(study, protocol)
    assert result["status"] == "FAIL"
    assert result["checks"]["exact_360000_frozen_trials"] is False
    assert not (study / "audit-result.json").exists()  # pure API


def test_relaxed_thresholds_are_rejected_as_not_the_frozen_gate(tmp_path):
    protocol = _protocol(tmp_path)
    data = json.loads(protocol.read_text())
    data["synthetic_study"]["checks"]["correct_coverage_interval"] = [0., 1.]
    protocol.write_text(json.dumps(data))
    study = tmp_path / "study"
    study.mkdir()
    result = AUDIT.audit(study, protocol)
    assert result["status"] == "FAIL"
    assert result["checks"]["frozen_numeric_limits"] is False


@pytest.mark.parametrize("text", ['{"a":1,"a":2}', '{"a":NaN}', '{"a":1e999}'])
def test_bad_json_produces_fail_without_importing_production(tmp_path, text):
    protocol = tmp_path / "protocol.json"
    protocol.write_text(text)
    result = AUDIT.audit(tmp_path, protocol)
    assert result["status"] == "FAIL"
    assert result["errors"]
    assert result["production_update_imported"] is False


def test_unexpected_file_cannot_be_ignored(tmp_path):
    protocol = _protocol(tmp_path)
    study, _, _ = _early_artifacts(tmp_path)
    (study / "unindexed.npy").write_bytes(b"extra")
    result = AUDIT.audit(study, protocol)
    assert result["status"] == "FAIL"
    assert result["checks"]["study_directory_fileset"] is False


def test_hash_mismatch_rejects_even_self_consistent_summary_and_index(tmp_path):
    protocol = _protocol(tmp_path)
    study, index, result = _early_artifacts(tmp_path)
    index["sha256"] = "0" * 64
    _write_metadata(study, index, result)
    audited = AUDIT.audit(study, protocol)
    assert audited["status"] == "FAIL"
    assert audited["checks"]["embedded_and_separate_array_index_identical"] is True
    assert audited["checks"]["archive_hash_and_bytes"] is False


def test_index_missing_one_of_the_54_arrays_is_rejected(tmp_path):
    protocol = _protocol(tmp_path)
    study, index, result = _early_artifacts(tmp_path)
    del index["arrays"][next(iter(index["arrays"]))]
    _write_metadata(study, index, result)
    audited = AUDIT.audit(study, protocol)
    assert audited["status"] == "FAIL"
    assert audited["checks"]["index_exactly_54_arrays"] is False


def test_duplicate_npz_member_is_rejected_even_after_rehash(tmp_path):
    protocol = _protocol(tmp_path)
    study, index, result = _early_artifacts(tmp_path)
    archive = study / "samples.npz"
    member = next(iter(index["arrays"])) + ".npy"
    with zipfile.ZipFile(archive, "w") as zipped:
        zipped.writestr(member, b"first")
        with pytest.warns(UserWarning, match="Duplicate name"):
            zipped.writestr(member, b"second")
    index["bytes"] = archive.stat().st_size
    index["sha256"] = hashlib.sha256(archive.read_bytes()).hexdigest()
    _write_metadata(study, index, result)
    audited = AUDIT.audit(study, protocol)
    assert audited["status"] == "FAIL"
    assert audited["checks"]["archive_hash_and_bytes"] is True
    assert audited["checks"]["npz_exact_member_fileset_no_duplicates"] is False


def test_cli_writes_failure_artifact_for_small_development_run(tmp_path):
    protocol = _protocol(tmp_path, draws=20)
    study = tmp_path / "development_twenty_draws"
    study.mkdir()
    completed = subprocess.run([sys.executable, "-B", str(SCRIPT), "--study-dir", str(study),
                                "--protocol", str(protocol)], capture_output=True, text=True)
    assert completed.returncode == 1, completed.stderr
    result = json.loads((study / "audit-result.json").read_text())
    assert result["status"] == "FAIL"
    assert result["checks"]["exact_360000_frozen_trials"] is False


def test_cli_cannot_overwrite_protocol(tmp_path):
    protocol = _protocol(tmp_path, draws=20)
    before = protocol.read_bytes()
    study = tmp_path / "study"
    study.mkdir()
    completed = subprocess.run([sys.executable, "-B", str(SCRIPT), "--study-dir", str(study),
                                "--protocol", str(protocol), "--output", str(protocol)],
                               capture_output=True, text=True)
    assert completed.returncode == 2
    assert protocol.read_bytes() == before
