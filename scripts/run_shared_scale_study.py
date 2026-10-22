#!/usr/bin/env python3
"""Frozen synthetic verification of repeated optical flow sharing one scale.

Point estimates are identical between methods. Only the covariance assumption
changes. Rates are already calibrated abstract variables here, NOT decoded
MTF-01P data or R2A image-angle measurements.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from xq_autonomy.scale_uncertainty import scale_velocity_moments


def _wilson(hits, count, z=1.959963984540054):
    p, zz = hits / count, z * z
    centre = (p + zz / (2 * count)) / (1 + zz / count)
    half = z * math.sqrt(p * (1 - p) / count + zz / (4 * count * count)) / (1 + zz / count)
    return [centre - half, centre + half]


def _write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def run_study(config_path, output):
    config_bytes = config_path.read_bytes()
    config = json.loads(config_bytes)
    # This entry point is a frozen experiment, not a generic parameter sweep.
    if config.get("schema_version") != 1 or config.get("input_origin") != "synthetic_joint_gaussian_not_hardware_or_gazebo":
        raise ValueError("unsupported study configuration")
    if output.exists():
        raise ValueError("output must be a new directory; earlier attempts are preserved")
    output.mkdir(parents=True)
    (output / "configuration.json").write_bytes(config_bytes)
    d = float(config["true_distance_m"])
    velocity = np.asarray(config["true_velocity_xy_mps"], dtype=float)
    q = velocity / d
    single_a = np.diag(np.square(config["single_frame_rate_sigma_per_s"]))
    direction = velocity / np.linalg.norm(velocity)
    quantile = config["coverage"]["normal_quantile"]
    draws = config["draws_per_seed_and_case"]
    limits = config["predeclared_checks"]
    rows, raw, checks = [], {}, {}
    for case_index, case in enumerate(config["cases"]):
        s = case["shared_range_sigma_m"] ** 2
        single_cov = np.zeros((3, 3))
        single_cov[:2, :2], single_cov[2, 2] = single_a, s
        single_moments = scale_velocity_moments(q, d, single_cov)
        for n in config["frames_per_window"]:
            joint_cov = single_cov.copy()
            joint_cov[:2, :2] /= n
            moments = scale_velocity_moments(q, d, joint_cov)
            correct = np.asarray(moments["covariance_mps2"])
            naive = np.asarray(single_moments["covariance_mps2"]) / n
            all_errors, hits_correct, hits_naive, negative_ranges = [], 0, 0, 0
            per_seed = []
            for seed in config["seeds"]:
                # SeedSequence separates declared cases/windows without relying
                # on iteration order or Python's randomized hash().
                rng = np.random.default_rng(np.random.SeedSequence([seed, case_index, n]))
                sampled_d = d + rng.normal(0, math.sqrt(s), draws)
                sampled_q = q + rng.normal(size=(draws, 2)) * np.sqrt(np.diag(single_a) / n)
                estimate = sampled_d[:, None] * sampled_q
                error = estimate - velocity
                scalar_error = error @ direction
                correct_hits = int(np.count_nonzero(np.abs(scalar_error) <= quantile * math.sqrt(direction @ correct @ direction)))
                naive_hits = int(np.count_nonzero(np.abs(scalar_error) <= quantile * math.sqrt(direction @ naive @ direction)))
                negative_ranges += int(np.count_nonzero(sampled_d <= 0))
                hits_correct += correct_hits
                hits_naive += naive_hits
                all_errors.append(error)
                key = f"{case['name']}_n{n}_seed{seed}"
                raw[key + "_rate_xy"] = sampled_q
                raw[key + "_range_m"] = sampled_d
                per_seed.append({"seed": seed, "draws": draws,
                                 "correct_hits": correct_hits, "independent_frame_hits": naive_hits})
            errors = np.concatenate(all_errors)
            count = len(errors)
            empirical = np.cov(errors, rowvar=False, ddof=1)
            relative_error = float(np.linalg.norm(empirical - correct) / np.linalg.norm(correct))
            row = {
                "case": case["name"], "frames_per_window": n, "independent_trials": count,
                "range_sigma_m": math.sqrt(s), "per_seed": per_seed,
                "point_estimate_equal_between_methods": True,
                "bias_xy_mps": np.mean(errors, axis=0).tolist(),
                "rmse_vector_mps": float(np.sqrt(np.mean(np.sum(errors * errors, axis=1)))),
                "predicted_covariance_mps2": correct.tolist(),
                "independent_frame_covariance_mps2": naive.tolist(),
                "empirical_covariance_mps2": empirical.tolist(),
                "relative_covariance_frobenius_error": relative_error,
                "correct_directional_coverage": hits_correct / count,
                "independent_frame_directional_coverage": hits_naive / count,
                "correct_coverage_wilson_95": _wilson(hits_correct, count),
                "independent_frame_coverage_wilson_95": _wilson(hits_naive, count),
                "negative_sampled_ranges": negative_ranges,
            }
            rows.append(row)
            label = f"{case['name']}.n{n}"
            checks[label + ".covariance_matches_samples"] = relative_error <= limits["maximum_relative_covariance_frobenius_error"]
            low, high = limits["correct_directional_coverage_interval"]
            checks[label + ".correct_empirical_coverage"] = low <= hits_correct / count <= high
            checks[label + ".positive_sampled_ranges"] = negative_ranges == 0
            if s == 0 or n == 1:
                checks[label + ".models_equal_control"] = bool(np.allclose(correct, naive, rtol=0, atol=limits["control_covariance_absolute_tolerance"]))
            if s > 0 and n == max(config["frames_per_window"]):
                checks[label + ".independence_failure_exposed"] = (hits_correct - hits_naive) / count >= limits["minimum_coverage_gap_at_largest_window"]
    sample_path = output / "samples.npz"
    np.savez(sample_path, **raw)
    result = {
        "schema_version": 1, "status": "PASS" if all(checks.values()) else "FAIL",
        "phase": config["phase"], "input_origin": config["input_origin"],
        "configuration_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "numpy_version": np.__version__, "checks": checks, "results": rows,
        "samples": {"file": sample_path.name, "size_bytes": sample_path.stat().st_size,
                    "sha256": hashlib.sha256(sample_path.read_bytes()).hexdigest()},
        "hardware_readiness": "NOT_READY", "multisensor_state_fusion_tested": False,
        "statistical_integrity_tested": False, "scope": config["scope"],
    }
    _write(output / "result.json", result)
    return result


def plot_result(result, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), constrained_layout=True)
    labels = {"known_range_control": "Known range control", "shared_range_stress": "Shared scale stress (synthetic)"}
    for ax, case in zip(axes, labels):
        rows = [row for row in result["results"] if row["case"] == case]
        x = [row["frames_per_window"] for row in rows]
        for field, title, marker in (("correct_directional_coverage", "Shared scale retained", "o"),
                                      ("independent_frame_directional_coverage", "Independent frames assumption", "s")):
            ax.plot(x, [row[field] for row in rows], marker=marker, label=title)
        ax.axhline(0.95, color="gray", linestyle="--", linewidth=1)
        ax.set(title=labels[case], xlabel="Rate samples sharing one range", ylabel="Directional interval coverage", ylim=(0, 1.02))
        ax.set_xticks(x)
        ax.grid(alpha=0.2)
    axes[0].legend(loc="lower left", fontsize=8)
    fig.suptitle("Same point estimate; different uncertainty assumption. No hardware data.", fontsize=10)
    fig.savefig(output / "coverage.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run_study(args.config, args.output)
    plot_result(result, args.output)
    print(json.dumps({"status": result["status"], "checks": len(result["checks"]), "output": str(args.output)}))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
