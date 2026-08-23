#!/usr/bin/env python3
"""Aggregate the frozen three-arm P10 long-corridor Gate."""

from __future__ import annotations

import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path


VARIANTS = ("baseline", "yaw_only", "minimum_excitation")


def _prediction(arm: dict, name: str) -> float:
    decision = arm["decision"]
    return float(decision["predicted_minimum_margins"][decision["candidate_names"].index(name)])


def main() -> int:
    if len(sys.argv) != 3:
        raise SystemExit("Usage: analyze_p10_gate.py RESULT_DIR THRESHOLDS_JSON")
    root = Path(sys.argv[1]).resolve()
    thresholds_path = Path(sys.argv[2]).resolve()
    thresholds = json.loads(thresholds_path.read_text(encoding="utf-8"))
    arms = {
        variant: json.loads((root / variant / "flight-result.json").read_text(encoding="utf-8"))
        for variant in VARIANTS
    }
    baseline = arms["baseline"]
    yaw = arms["yaw_only"]
    recovery = arms["minimum_excitation"]
    reserve = float(thresholds["margin_reserve_m"])
    actual = {
        variant: float(arm["metrics"]["actual_minimum_integrity_margin_m"])
        for variant, arm in arms.items()
    }
    path = {
        variant: float(arm["metrics"]["ground_truth_path_length_m"])
        for variant, arm in arms.items()
    }
    mission_time = {
        variant: float(arm["metrics"]["mission_time_s"])
        for variant, arm in arms.items()
    }
    ate = {variant: float(arm["metrics"]["ate_rms_m"]) for variant, arm in arms.items()}
    weak = {
        variant: float(arm["metrics"]["weak_direction_error_rms_m"])
        for variant, arm in arms.items()
    }
    # Candidate forecasts must be compared within one common map/covariance
    # snapshot.  The independent comparator flights have their own scheduling
    # and map realization, so mixing their forecasts is not a paired test.
    predictions = {
        "baseline": _prediction(recovery, "baseline"),
        "yaw_only": _prediction(recovery, "yaw_only"),
        "minimum_excitation": _prediction(
            recovery, str(recovery["decision"]["selected_name"])
        ),
    }
    extra_path = path["minimum_excitation"] - path["baseline"]
    time_overhead = mission_time["minimum_excitation"] - mission_time["baseline"]
    reference_best_ate = min(ate["baseline"], ate["yaw_only"])
    reference_best_weak = min(weak["baseline"], weak["yaw_only"])
    checks = {
        "all_flight_arms_pass": all(arm["status"] == "PASS" for arm in arms.values()),
        "baseline_predicted_insufficient": predictions["baseline"] < reserve,
        "yaw_only_predicted_insufficient": predictions["yaw_only"] < reserve,
        "minimum_excitation_predicted_reserved": predictions["minimum_excitation"] >= reserve,
        "required_recovery_selected": recovery["decision"]["selected_name"]
        == thresholds["required_recovery_candidate"],
        "minimum_excitation_actually_reserved": actual["minimum_excitation"] >= reserve,
        "baseline_actually_insufficient": actual["baseline"] < reserve,
        "yaw_only_actually_insufficient": actual["yaw_only"] < reserve,
        "yaw_only_does_not_restore_margin": actual["yaw_only"]
        <= actual["baseline"] + float(thresholds["maximum_yaw_only_margin_gain_m"]),
        "actual_margin_gain": actual["minimum_excitation"]
        - max(actual["baseline"], actual["yaw_only"])
        >= float(thresholds["minimum_recovery_margin_gain_m"]),
        "bounded_extra_path": 0.0 <= extra_path <= float(thresholds["maximum_extra_path_m"]),
        "bounded_mission_time": time_overhead
        <= float(thresholds["maximum_mission_time_overhead_s"]),
        "ate_not_materially_degraded": ate["minimum_excitation"]
        <= reference_best_ate + float(thresholds["maximum_ate_degradation_m"]),
        "weak_error_not_materially_degraded": weak["minimum_excitation"]
        <= reference_best_weak + float(thresholds["maximum_weak_error_degradation_m"]),
        "same_frozen_calibration": len(
            {arm["calibration_sha256"] for arm in arms.values()}
        ) == 1,
        "finite_comparison_metrics": all(
            math.isfinite(value)
            for group in (actual, path, mission_time, ate, weak, predictions)
            for value in group.values()
        ),
    }
    summary = {
        "schema_version": 1,
        "gate": "P10_MINIMUM_EXCITATION_ACTIVE_PERCEPTION",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "scenario": "xq_p10_long_corridor",
        "variants": list(VARIANTS),
        "predicted_minimum_margins_m": predictions,
        "actual_minimum_integrity_margins_m": actual,
        "ate_rms_m": ate,
        "weak_direction_error_rms_m": weak,
        "ground_truth_path_length_m": path,
        "mission_time_s": mission_time,
        "minimum_excitation_extra_path_m": extra_path,
        "minimum_excitation_time_overhead_s": time_overhead,
        "selected_recovery": recovery["decision"]["selected_name"],
        "prediction_comparison_source": "minimum_excitation_common_decision_snapshot",
        "checks": checks,
        "thresholds": thresholds,
        "ground_truth_policy": "evaluation_and_logging_only",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    output = root / "summary.json"
    temporary = output.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    temporary.replace(output)
    print(json.dumps(summary, indent=2))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
