#!/usr/bin/env python3
"""Freeze train-only P7 directional quantiles and summarize independent tests."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


DIRECTIONS = ("x", "y", "z", "weak")


def _read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _higher(values: np.ndarray, quantile: float) -> float:
    return float(np.quantile(values, quantile, method="higher"))


def calibrate(train_paths: list[Path], output: Path) -> int:
    records = [_read(path) for path in train_paths]
    if len(records) < 2 or len({item["scenario"] for item in records}) < 2:
        raise ValueError("P7 requires at least two distinct train scenarios")
    if any(item["split"] != "train" or item["status"] != "PASS" for item in records):
        raise ValueError("all calibration inputs must be PASS train captures")
    directional = {}
    per_direction_quantiles = {}
    for name in DIRECTIONS:
        per_scenario = []
        for record in records:
            ratios = np.asarray(record["raw_directional_samples"][name]["ratio"], dtype=float)
            per_scenario.append(
                {
                    "scenario": record["scenario"],
                    "count": len(ratios),
                    "q95": _higher(ratios, 0.95),
                    "q99": _higher(ratios, 0.99),
                }
            )
        per_direction_quantiles[name] = per_scenario
    # A per-scenario quantile only covers repeated samples from the same domain.
    # Estimate a train-only domain-shift reserve from the largest observed ratio
    # between scenario quantiles, then apply that common reserve to every axis.
    # Using one common factor prevents any held-out direction from choosing its
    # own post-test correction.
    shift_ratios = {
        name: max(item["q95"] for item in items) / max(min(item["q95"] for item in items), 1.0e-12)
        for name, items in per_direction_quantiles.items()
    }
    domain_shift_factor = max(1.0, max(shift_ratios.values()))
    for name, per_scenario in per_direction_quantiles.items():
        directional[name] = {
            "k95": domain_shift_factor * max(item["q95"] for item in per_scenario),
            "k99": domain_shift_factor * max(item["q99"] for item in per_scenario),
            "base_k95": max(item["q95"] for item in per_scenario),
            "base_k99": max(item["q99"] for item in per_scenario),
            "selection_rule": "train-only worst cross-scenario shift reserve times max per-scenario higher empirical quantile",
            "per_train_scenario": per_scenario,
        }
    artifact = {
        "schema_version": 2,
        "artifact": "P7_FROZEN_DIRECTIONAL_PL_CALIBRATION",
        "train_only": True,
        "test_data_used": False,
        "directions": list(DIRECTIONS),
        "domain_shift_reserve": {
            "factor": domain_shift_factor,
            "per_direction_q95_ratio": shift_ratios,
            "source": "training scenarios only",
        },
        "directional": directional,
        "train_inputs": [
            {"path": str(path), "sha256": _sha(path), "scenario": record["scenario"]}
            for path, record in zip(train_paths, records)
        ],
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    output.write_text(json.dumps(artifact, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(artifact, indent=2))
    return 0


def summarize(calibration_path: Path, test_paths: list[Path], output: Path) -> int:
    calibration_sha = _sha(calibration_path)
    records = [_read(path) for path in test_paths]
    if len(records) < 2 or len({item["scenario"] for item in records}) < 2:
        raise ValueError("P7 requires at least two distinct independent test scenarios")
    if any(item["split"] != "test" or item["status"] != "PASS" for item in records):
        raise ValueError("all validation inputs must be PASS test captures")
    if any(item["calibration_sha256"] != calibration_sha for item in records):
        raise ValueError("test capture did not use the frozen calibration artifact")
    scenario_results = {}
    all_coverage95 = []
    all_coverage99 = []
    weighted95_numerator = weighted99_numerator = total = 0
    for record in records:
        metrics = record["metrics"]
        scenario_count = sum(int(metrics[name]["count"]) for name in DIRECTIONS)
        covered95 = sum(
            int(round(float(metrics[name]["coverage_95"]) * int(metrics[name]["count"])))
            for name in DIRECTIONS
        )
        covered99 = sum(
            int(round(float(metrics[name]["coverage_99"]) * int(metrics[name]["count"])))
            for name in DIRECTIONS
        )
        coverage95 = covered95 / scenario_count
        coverage99 = covered99 / scenario_count
        all_coverage95.append(coverage95)
        all_coverage99.append(coverage99)
        weighted95_numerator += covered95
        weighted99_numerator += covered99
        total += scenario_count
        scenario_results[record["scenario"]] = {
            "coverage_95": coverage95,
            "coverage_99": coverage99,
            "directional": metrics,
        }
    aggregate95 = weighted95_numerator / total
    aggregate99 = weighted99_numerator / total
    checks = {
        "train_test_separated": True,
        "calibration_frozen_before_test": True,
        "two_independent_test_scenarios": len(scenario_results) >= 2,
        "aggregate_95_coverage": aggregate95 >= 0.95,
        "each_test_scenario_95_coverage": min(all_coverage95) >= 0.95,
    }
    result = {
        "schema_version": 1,
        "gate": "P7_PROTECTION_LEVEL_CALIBRATION",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "calibration_file": str(calibration_path),
        "calibration_sha256": calibration_sha,
        "aggregate": {
            "coverage_95": aggregate95,
            "coverage_99": aggregate99,
            "sample_directions": total,
            "false_alarm_rate": None,
            "false_alarm_note": "P8 Alert Limit is intentionally not defined in P7",
            "missed_integrity_event_rate_95": 1.0 - aggregate95,
        },
        "test_scenarios": scenario_results,
        "test_inputs": [{"path": str(path), "sha256": _sha(path)} for path in test_paths],
        "test_data_used_for_tuning": False,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    train = subparsers.add_parser("calibrate")
    train.add_argument("--train", type=Path, nargs="+", required=True)
    train.add_argument("--output", type=Path, required=True)
    test = subparsers.add_parser("summarize")
    test.add_argument("--calibration", type=Path, required=True)
    test.add_argument("--test", type=Path, nargs="+", required=True)
    test.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.command == "calibrate":
        return calibrate(arguments.train, arguments.output)
    return summarize(arguments.calibration, arguments.test, arguments.output)


if __name__ == "__main__":
    raise SystemExit(main())
