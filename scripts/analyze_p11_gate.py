#!/usr/bin/env python3
"""Aggregate the frozen two-arm P11 Gazebo flight Gate."""

from __future__ import annotations

import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path


VARIANTS = ("information_only", "integrity_constrained")


def main() -> int:
    if len(sys.argv) != 3:
        raise SystemExit("Usage: analyze_p11_gate.py RESULT_DIR THRESHOLDS_JSON")
    root = Path(sys.argv[1]).resolve()
    thresholds = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    arms = {
        variant: json.loads((root / variant / "flight-result.json").read_text(encoding="utf-8"))
        for variant in VARIANTS
    }
    unconstrained = arms["information_only"]
    constrained = arms["integrity_constrained"]
    constrained_decisions = list(constrained.get("decisions", []))
    if not constrained_decisions:
        constrained_decisions = [constrained["decision"]]
    direct_name = thresholds["required_unconstrained_candidate"]
    safe_name = thresholds["required_integrity_candidate"]
    critical = [
        item
        for item in constrained_decisions
        if item["unconstrained_selected_name"] == direct_name
        and item["selected_name"] == safe_name
        and direct_name in item["candidate_names"]
        and safe_name in item["candidate_names"]
    ]
    if not critical:
        raise ValueError("no rolling decision contains the required integrity intervention")
    decision = critical[0]
    names = list(decision["candidate_names"])
    direct_index = names.index(direct_name)
    safe_index = names.index(safe_name)
    reserve = float(thresholds["margin_reserve_m"])
    predicted = {
        direct_name: float(decision["predicted_minimum_margins"][direct_index]),
        safe_name: float(decision["predicted_minimum_margins"][safe_index]),
    }
    utilities = {
        direct_name: float(decision["utilities"][direct_index]),
        safe_name: float(decision["utilities"][safe_index]),
    }
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
    information_total = {
        variant: float(arm["metrics"]["executed_information_gain"])
        for variant, arm in arms.items()
    }
    progress = {
        variant: float(arm["metrics"]["forward_progress_m"])
        for variant, arm in arms.items()
    }
    final_x = {
        variant: float(arm["metrics"]["truth_final_x_m"])
        for variant, arm in arms.items()
    }
    decision_batches = {
        variant: len(arm.get("decisions", [arm["decision"]]))
        for variant, arm in arms.items()
    }
    information = {
        variant: information_total[variant] / decision_batches[variant]
        for variant in VARIANTS
    }
    extra_path = path["integrity_constrained"] - path["information_only"]
    time_overhead = mission_time["integrity_constrained"] - mission_time["information_only"]
    information_loss = information["information_only"] - information["integrity_constrained"]
    checks = {
        "both_flight_arms_pass": all(arm["status"] == "PASS" for arm in arms.values()),
        "both_arms_reach_corridor_goal": all(
            value >= float(thresholds["minimum_truth_final_x_m"])
            for value in final_x.values()
        ),
        "both_arms_cover_full_corridor": all(
            value >= float(thresholds["minimum_forward_progress_m"])
            for value in progress.values()
        ),
        "rolling_decision_batches_complete": all(
            value >= int(thresholds["minimum_decision_batches"])
            for value in decision_batches.values()
        ),
        "common_snapshot_has_required_candidates": direct_name in names and safe_name in names,
        "unconstrained_candidate_required": decision["unconstrained_selected_name"] == direct_name,
        "integrity_candidate_required": decision["selected_name"] == safe_name,
        "direct_task_utility_higher": utilities[direct_name] > utilities[safe_name],
        "direct_predicted_insufficient": predicted[direct_name] < reserve,
        "safe_predicted_reserved": predicted[safe_name] >= reserve,
        "direct_integrity_hard_rejected": not decision["integrity_feasible"][direct_index],
        "safe_all_hard_feasible": decision["feasible"][safe_index],
        "collision_constraint_satisfied": all(
            all(item["collision_feasible"]) for item in constrained_decisions
        ),
        "return_energy_constraint_satisfied": all(
            all(item["energy_feasible"]) for item in constrained_decisions
        ),
        "information_only_actually_insufficient": actual["information_only"] < reserve,
        "integrity_constrained_actually_reserved": actual["integrity_constrained"] >= reserve,
        "minimum_actual_margin_gain": actual["integrity_constrained"]
        - actual["information_only"]
        >= float(thresholds["minimum_actual_margin_gain_m"]),
        "bounded_information_loss": 0.0 <= information_loss
        <= float(thresholds["maximum_information_gain_loss"]),
        "bounded_extra_path": 0.0 <= extra_path
        <= float(thresholds["maximum_extra_path_m"]),
        "bounded_mission_time": time_overhead
        <= float(thresholds["maximum_mission_time_overhead_s"]),
        "ate_not_materially_degraded": ate["integrity_constrained"]
        <= ate["information_only"] + float(thresholds["maximum_ate_degradation_m"]),
        "same_frozen_calibration": len({arm["calibration_sha256"] for arm in arms.values()}) == 1,
        "algorithm_ground_truth_absent": all(
            arm.get("algorithm_ground_truth_subscribed") is False for arm in arms.values()
        ),
        "margin_not_in_utility": all(
            arm["decision"]["margin_in_utility"] is False for arm in arms.values()
        ),
        "finite_comparison_metrics": all(
            math.isfinite(value)
            for group in (
                predicted, utilities, actual, path, mission_time, ate, information,
                progress, final_x,
            )
            for value in group.values()
        ),
    }
    summary = {
        "schema_version": 1,
        "gate": "P11_INTEGRITY_CONSTRAINED_EXPLORATION",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "scenario": "xq_p11_integrity_exploration_open_top",
        "variants": list(VARIANTS),
        "predicted_minimum_margins_m": predicted,
        "utilities": utilities,
        "actual_minimum_integrity_margins_m": actual,
        "executed_information_gain_total": information_total,
        "executed_information_gain_mean_per_batch": information,
        "ground_truth_path_length_m": path,
        "mission_time_s": mission_time,
        "ate_rms_m": ate,
        "forward_progress_m": progress,
        "truth_final_x_m": final_x,
        "decision_batches": decision_batches,
        "selected_sequences": {
            variant: [
                (
                    item["unconstrained_selected_name"]
                    if variant == "information_only"
                    else item["selected_name"]
                )
                for item in arm.get("decisions", [arm["decision"]])
            ]
            for variant, arm in arms.items()
        },
        "actual_margin_gain_m": actual["integrity_constrained"] - actual["information_only"],
        "mean_information_gain_loss_per_batch": information_loss,
        "extra_path_m": extra_path,
        "mission_time_overhead_s": time_overhead,
        "critical_batch_id": decision.get("batch_id"),
        "prediction_comparison_source": "integrity_constrained_critical_rolling_snapshot",
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
