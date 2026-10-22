#!/usr/bin/env python3
"""Independently reconstruct the frozen 360000-trial correlation fixture.

No fusion implementation is imported.  Passing establishes arithmetic and
artifact consistency for declared synthetic Gaussian draws, not a sensor noise
model, a multi-window filter, live flow fusion or a protection level.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np


METHODS = ("keep_C", "force_C_zero")
FIELDS = ("latent", "prior_error_xy", "measurement_noise", "innovation",
          "keep_C_posterior_error_xy", "force_C_zero_posterior_error_xy")
INPUT_FILES = {"samples.npz", "raw-array-index.json", "study-result.json"}
ORIGIN = "declared_linear_gaussian_fixture_not_sensor_calibration"
QUANTILE = 1.959963984540054


def _reject_constant(value):
    raise ValueError("non-finite JSON constant: " + value)


def _float(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("non-finite JSON number")
    return number


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError("duplicate JSON key: " + key)
        result[key] = value
    return result


def _hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read(path, evidence):
    before = path.stat()
    data = path.read_bytes()
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError("input changed during read: " + str(path))
    evidence[str(path)] = {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
    return json.loads(data, parse_constant=_reject_constant, parse_float=_float, object_pairs_hook=_pairs)


def _integer(value):
    return type(value) is int


def _close(actual, expected, tolerance=1.0e-12):
    try:
        raw = np.asarray(actual)
        expected = np.asarray(expected)
        return bool(raw.dtype.kind in "iuf" and raw.shape == expected.shape
                    and np.isfinite(raw).all() and np.max(np.abs(raw - expected)) <= tolerance)
    except (ValueError, TypeError, OverflowError):
        return False


def _score_interval(successes, total):
    """Wilson endpoints as roots of the binomial score inequality."""
    z2 = QUANTILE * QUANTILE
    proportion = successes / total
    denominator = total + z2
    centre = (successes + z2 / 2.0) / denominator
    half_width = math.sqrt(z2 * (total * proportion * (1.0 - proportion) + z2 / 4.0)) / denominator
    return [centre - half_width, centre + half_width]


def _statistics(errors, predicted, direction, quantile):
    """Direct centred second moments, independently of numpy.cov/study helpers."""
    count = len(errors)
    centred = errors - np.sum(errors, axis=0) / count
    empirical = centred.T @ centred / count
    predicted_variance = float(direction @ predicted @ direction)
    covered = int(np.count_nonzero(np.abs(errors @ direction) <= quantile * math.sqrt(predicted_variance)))
    return {
        "covered_trials": covered,
        "coverage": covered / count,
        "wilson_95": _score_interval(covered, count),
        "predicted_covariance_xy_mps2": predicted.tolist(),
        "empirical_covariance_xy_mps2": empirical.tolist(),
        "predicted_directional_variance_mps2": predicted_variance,
        "empirical_directional_variance_mps2": float(direction @ empirical @ direction),
        "relative_covariance_frobenius_error": float(np.linalg.norm(empirical - predicted) / np.linalg.norm(predicted)),
    }


def _generated_block(seed, case_index, rho, count, sigma):
    # Recreate the SIX independent primitive coordinates, before either method.
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed, case_index])))
    latent = rng.standard_normal((count, 6))
    shared, private_x, private_v = latent[:, :2], latent[:, 2:4], latent[:, 4:]
    error = sigma * (math.sqrt(abs(rho)) * shared + math.sqrt(1.0 - abs(rho)) * private_x)
    noise = sigma * (np.sign(rho) * math.sqrt(abs(rho)) * shared + math.sqrt(1.0 - abs(rho)) * private_v)
    return latent, error, noise, error + noise


def audit(study_dir, protocol_path):
    """Read-only API.  The CLI alone writes its separate audit-result.json."""
    study_dir, protocol_path = Path(study_dir), Path(protocol_path)
    checks, errors, evidence, recomputed, arrays_read = {}, [], {}, [], {}
    result = {
        "schema_version": 1, "status": "FAIL", "checks": checks, "errors": errors,
        "study_dir": str(study_dir), "protocol_path": str(protocol_path),
        "input_evidence": evidence, "recomputed_cases": recomputed, "arrays_read": arrays_read,
        "production_update_imported": False,
        "unknown_C_runtime_retest": "not repeated; covered by separate kernel tests",
        "scope": "Independent single-window synthetic Gaussian arithmetic/artifact audit; no hardware calibration, actual flow state update, statistical integrity or protection level",
    }

    def check(key, condition):
        checks[key] = bool(condition)
        if not condition:
            errors.append(key)
        return bool(condition)

    def require(key, condition):
        if not check(key, condition):
            raise ValueError("required contract failed: " + key)

    try:
        protocol = _read(protocol_path, evidence)
        cfg = protocol["synthetic_study"]
        limits, coverage = cfg["checks"], cfg["coverage"]
        require("frozen_protocol_schema_and_scope", _integer(protocol["schema_version"]) and protocol["schema_version"] == 1
                and protocol["hardware_readiness"] == "NOT_READY"
                and protocol["full_common_state_flow_fusion_validated"] is False
                and cfg["input_origin"] == ORIGIN)
        require("frozen_fixture_dimensions_and_units", cfg["state_dimension"] == 19
                and cfg["observed_state_indices"] == [7, 8]
                and cfg["prior_velocity_xy_mps"] == [1.0, 0.6]
                and cfg["observed_prior_sigma_mps"] == 0.2
                and cfg["measurement_noise_sigma_mps"] == 0.2
                and cfg["state_noise_correlations"] == [-0.9, 0.0, 0.9])
        require("exact_360000_frozen_trials", cfg["seeds"] == [17019, 31183, 92401]
                and all(_integer(seed) for seed in cfg["seeds"])
                and _integer(cfg["draws_per_seed_and_case"])
                and cfg["draws_per_seed_and_case"] == 40000
                and 3 * len(cfg["seeds"]) * cfg["draws_per_seed_and_case"] == 360000)
        require("frozen_coverage_definition", coverage["kind"] == "directional_normal_interval_not_protection_level"
                and coverage["nominal"] == 0.95 and coverage["normal_quantile"] == QUANTILE
                and coverage["direction"] == "unit vector along prior_velocity_xy_mps"
                and coverage["confidence_interval"] == "Wilson 95% over independent synthetic trials only")
        frozen_limits = {
            "correct_coverage_interval": [0.93, 0.97],
            "maximum_relative_posterior_covariance_error": 0.04,
            "minimum_negative_correlation_coverage_gap": 0.07,
            "minimum_positive_correlation_independence_coverage": 0.99,
            "maximum_zero_correlation_method_difference": 1e-12,
            "maximum_posterior_mean_method_difference": 1e-12,
            "maximum_analytic_posterior_covariance_error": 1e-12,
        }
        require("frozen_numeric_limits", limits == frozen_limits)
        require("study_directory_fileset", {p.name for p in study_dir.iterdir()} - {"audit-result.json"} == INPUT_FILES)
        declared = _read(study_dir / "study-result.json", evidence)
        index = _read(study_dir / "raw-array-index.json", evidence)
        require("declared_complete_synthetic_study", _integer(declared["schema_version"]) and declared["schema_version"] == 1
                and declared["status"] == "PASS" and declared["input_origin"] == ORIGIN
                and _integer(declared["independent_trials"]) and declared["independent_trials"] == 360000)
        require("embedded_and_separate_array_index_identical", declared["raw_arrays"] == index)
        require("index_archive_basename", isinstance(index["path"], str)
                and index["path"].replace("\\", "/").rsplit("/", 1)[-1] == "samples.npz")
        expected = {}
        count = cfg["draws_per_seed_and_case"]
        for case in range(3):
            for seed in cfg["seeds"]:
                for field in FIELDS:
                    expected[f"case{case}_seed{seed}_{field}"] = (count, 6 if field == "latent" else 2)
        require("index_exactly_54_arrays", isinstance(index["arrays"], dict)
                and len(index["arrays"]) == 54 and set(index["arrays"]) == set(expected))
        archive_path = study_dir / "samples.npz"
        archive_before = {"bytes": archive_path.stat().st_size, "sha256": _hash(archive_path)}
        evidence[str(archive_path)] = archive_before
        require("archive_hash_and_bytes", _integer(index["bytes"]) and index["bytes"] == archive_before["bytes"]
                and index["sha256"] == archive_before["sha256"])
        with zipfile.ZipFile(archive_path) as zipped:
            members = zipped.infolist()
            require("npz_exact_member_fileset_no_duplicates", len(members) == 54
                    and {member.filename for member in members} == {key + ".npy" for key in expected})
            require("bounded_npz_members", all(
                not member.is_dir() and np.prod(expected[member.filename[:-4]]) * 8 <= member.file_size
                <= np.prod(expected[member.filename[:-4]]) * 8 + 4096 for member in members))
        require("three_declared_cases", isinstance(declared["cases"], list) and len(declared["cases"]) == 3)
        direction = np.asarray(cfg["prior_velocity_xy_mps"], dtype=float)
        direction = direction / math.sqrt(float(direction @ direction))
        expected_study_checks = {"unknown_cross_covariance_rejected": True,
                                 "all_raw_arrays_read_back_identically": True,
                                 "sample_archive_unchanged_after_read": True}
        with np.load(archive_path, allow_pickle=False) as archive:
            require("npz_loader_exact_54_arrays", len(archive.files) == 54 and set(archive.files) == set(expected))
            for case_index, rho in enumerate(cfg["state_noise_correlations"]):
                row = declared["cases"][case_index]
                require(f"case{case_index}.identity", row["case_index"] == case_index
                        and _integer(row["case_index"]) and row["rho"] == rho
                        and _integer(row["trials"]) and row["trials"] == 120000
                        and set(row["methods"]) == set(METHODS))
                parts = {method: [] for method in METHODS}
                maximum_difference = 0.0
                for seed in cfg["seeds"]:
                    prefix = f"case{case_index}_seed{seed}_"
                    block = {}
                    for field in FIELDS:
                        key = prefix + field
                        array = archive[key]
                        metadata = index["arrays"][key]
                        require(key + ".shape_dtype_finite", array.shape == expected[key]
                                and array.dtype == np.dtype("float64") and np.isfinite(array).all()
                                and metadata["shape"] == list(expected[key]) and all(_integer(v) for v in metadata["shape"])
                                and metadata["dtype"] == "float64")
                        arrays_read[key] = {"shape": list(array.shape), "dtype": str(array.dtype),
                                            "sha256_c_order_values": hashlib.sha256(array.tobytes(order="C")).hexdigest()}
                        block[field] = array
                    latent, error, noise, residual = _generated_block(seed, case_index, rho, count, 0.2)
                    check(prefix + "rng_seed_case_identity", np.array_equal(block["latent"], latent))
                    check(prefix + "primitive_error_noise_innovation", np.array_equal(block["prior_error_xy"], error)
                          and np.array_equal(block["measurement_noise"], noise)
                          and np.array_equal(block["innovation"], residual))
                    # Both methods MUST have K_obs = 0.5 I in this symmetric fixture.
                    posterior_oracle = error - 0.5 * residual
                    for method in METHODS:
                        actual = block[method + "_posterior_error_xy"]
                        check(prefix + method + ".half_gain_point_estimate", _close(actual, posterior_oracle))
                        parts[method].append(actual)
                    maximum_difference = max(maximum_difference, float(np.max(np.abs(
                        block["keep_C_posterior_error_xy"] - block["force_C_zero_posterior_error_xy"]))))
                report = {"case_index": case_index, "rho": rho, "trials": 120000,
                          "maximum_point_estimate_difference_mps": maximum_difference, "methods": {}}
                check(f"case{case_index}.reported_mean_difference", _close(row["maximum_point_estimate_difference_mps"], maximum_difference))
                for method in METHODS:
                    variance = 0.04 * (1.0 - rho) / 2.0 if method == "keep_C" else 0.02
                    statistics = _statistics(np.concatenate(parts[method]), np.eye(2) * variance,
                                             direction, QUANTILE)
                    report["methods"][method] = statistics
                    reported = row["methods"][method]
                    require(f"case{case_index}.{method}.metric_fields", set(reported) == set(statistics))
                    check(f"case{case_index}.{method}.covered_count", _integer(reported["covered_trials"])
                          and reported["covered_trials"] == statistics["covered_trials"])
                    for key, value in statistics.items():
                        if key != "covered_trials":
                            check(f"case{case_index}.{method}.{key}", _close(reported[key], value))
                correct, wrong = report["methods"]["keep_C"], report["methods"]["force_C_zero"]
                gates = {
                    f"case{case_index}.analytic_posterior": _close(row["methods"]["keep_C"]["predicted_covariance_xy_mps2"], np.eye(2) * (0.04 * (1-rho) / 2)),
                    f"case{case_index}.same_point_estimates": maximum_difference <= limits["maximum_posterior_mean_method_difference"],
                    f"case{case_index}.correct_coverage": limits["correct_coverage_interval"][0] <= correct["coverage"] <= limits["correct_coverage_interval"][1],
                    f"case{case_index}.correct_covariance": correct["relative_covariance_frobenius_error"] <= limits["maximum_relative_posterior_covariance_error"],
                }
                if rho < 0:
                    gates[f"case{case_index}.negative_correlation_undercoverage"] = correct["coverage"]-wrong["coverage"] >= limits["minimum_negative_correlation_coverage_gap"]
                elif rho > 0:
                    gates[f"case{case_index}.positive_correlation_overcoverage"] = wrong["coverage"] >= limits["minimum_positive_correlation_independence_coverage"]
                else:
                    gates[f"case{case_index}.zero_correlation_control"] = correct["covered_trials"] == wrong["covered_trials"] and _close(correct["predicted_covariance_xy_mps2"], wrong["predicted_covariance_xy_mps2"], limits["maximum_zero_correlation_method_difference"])
                for key, passed in gates.items():
                    check("independent_gate." + key, passed)
                    expected_study_checks[key] = bool(passed)
                recomputed.append(report)
        check("every_one_of_54_arrays_read", len(arrays_read) == 54 and set(arrays_read) == set(expected))
        check("declared_checks_match_independent_results", declared["checks"] == expected_study_checks
              and all(type(value) is bool for value in declared["checks"].values()))
        check("archive_unchanged_during_audit", archive_before == {"bytes": archive_path.stat().st_size, "sha256": _hash(archive_path)})
        for path_string, recorded in tuple(evidence.items()):
            path = Path(path_string)
            check("unchanged:" + path_string, path.stat().st_size == recorded["bytes"] and _hash(path) == recorded["sha256"])
        result["independent_trials"] = sum(row["trials"] for row in recomputed)
        check("exact_360000_reconstructed_trials", result["independent_trials"] == 360000)
        result["status"] = "PASS" if checks and all(checks.values()) and not errors else "FAIL"
    except (OSError, ValueError, KeyError, TypeError, OverflowError, RuntimeError, NotImplementedError,
            zipfile.BadZipFile, EOFError) as exc:
        errors.append(type(exc).__name__ + ": " + str(exc))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study-dir", type=Path, required=True)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or args.study_dir / "audit-result.json"
    protected = {args.protocol.resolve()} | {(args.study_dir / name).resolve() for name in INPUT_FILES}
    if output.resolve() in protected:
        parser.error("audit output must not overwrite a study input or protocol")
    result = audit(args.study_dir, args.protocol)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".tmp")
    temporary.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    temporary.replace(output)
    print(json.dumps({"status": result["status"], "checks": len(result["checks"]),
                      "arrays_read": len(result["arrays_read"]), "errors": result["errors"], "output": str(output)}))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
