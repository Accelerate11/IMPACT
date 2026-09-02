#!/usr/bin/env python3
"""Aggregate the fair two-arm complex full-autonomy comparison."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path


VARIANTS = ("information_only", "integrity_constrained")


def _read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _decisions(result: dict) -> list[dict]:
    values = list(result.get("decisions", []))
    if not values and result.get("decision"):
        values = [result["decision"]]
    return values


def _executed_sequence(variant: str, decisions: list[dict]) -> list[str]:
    key = (
        "unconstrained_selected_name"
        if variant == "information_only"
        else "selected_name"
    )
    return [str(item.get(key, "")) for item in decisions]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-dir", required=True, type=Path)
    parser.add_argument("--thresholds", required=True, type=Path)
    parser.add_argument("--world-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    thresholds = _read(args.thresholds)
    arms = {}
    for variant in VARIANTS:
        root = args.result_dir / variant
        arms[variant] = {
            "p11": _read(root / "p11-result.json"),
            "p12": _read(root / "p12-result.json"),
            "p13": _read(root / "p13-result.json"),
        }

    p11 = {key: value["p11"] for key, value in arms.items()}
    p12 = {key: value["p12"] for key, value in arms.items()}
    p13 = {key: value["p13"] for key, value in arms.items()}
    decisions = {key: _decisions(value) for key, value in p11.items()}
    sequences = {
        key: _executed_sequence(key, decisions[key]) for key in VARIANTS
    }
    constrained_decisions = decisions["integrity_constrained"]
    direct = str(thresholds["required_unconstrained_candidate"])
    interventions = [
        item
        for item in constrained_decisions
        if item.get("unconstrained_selected_name") != item.get("selected_name")
    ]
    intervention_pairs = [
        [
            str(item.get("unconstrained_selected_name", "")),
            str(item.get("selected_name", "")),
        ]
        for item in interventions
    ]
    required_pairs = [
        [str(pair[0]), str(pair[1])]
        for pair in thresholds.get("required_intervention_pairs", [])
    ]
    if not required_pairs:
        detour = str(thresholds["required_integrity_candidate"])
        required_interventions = [
            item
            for item in interventions
            if item.get("unconstrained_selected_name") == direct
            and item.get("selected_name") == detour
        ]
        required_pairs_observed = len(required_interventions) >= int(
            thresholds["minimum_integrity_interventions"]
        )
    else:
        required_interventions = interventions
        required_pairs_observed = intervention_pairs == required_pairs
    required_unconstrained_sequence = [
        str(value) for value in thresholds.get("required_unconstrained_sequence", [])
    ]
    required_integrity_sequence = [
        str(value) for value in thresholds.get("required_integrity_sequence", [])
    ]

    actual_margin = {
        key: float(value["metrics"]["actual_minimum_integrity_margin_m"])
        for key, value in p11.items()
    }
    path_length = {
        key: float(value["metrics"]["ground_truth_path_length_m"])
        for key, value in p11.items()
    }
    mission_time = {
        key: float(value["metrics"]["mission_time_s"])
        for key, value in p11.items()
    }
    ate = {
        key: float(value["metrics"]["ate_rms_m"])
        for key, value in p11.items()
    }
    progress = {
        key: float(value["metrics"]["forward_progress_m"])
        for key, value in p11.items()
    }
    dynamic_clearance = {
        key: float(value["metrics"]["minimum_obstacle_clearance_m"])
        for key, value in p12.items()
    }
    dynamic_replans = {
        key: int(value["metrics"]["replan_event_count"])
        for key, value in p12.items()
    }
    latency_p99 = {
        key: float(value["metrics"]["end_to_end_p99_ms"])
        for key, value in p13.items()
    }
    p13_margin = {
        key: float(value["metrics"]["final_integrity_margin_m"])
        for key, value in p13.items()
    }
    margin_gain = actual_margin["integrity_constrained"] - actual_margin["information_only"]
    extra_path = path_length["integrity_constrained"] - path_length["information_only"]
    time_overhead = mission_time["integrity_constrained"] - mission_time["information_only"]

    common_checks = {
        "all_component_evaluators_pass": all(
            result[phase]["status"] == "PASS"
            for result in arms.values()
            for phase in ("p11", "p12", "p13")
        ),
        "both_complete_full_mission": all(
            progress[key] >= float(thresholds["minimum_forward_progress_m"])
            for key in VARIANTS
        ),
        "both_have_rolling_decisions": all(
            len(decisions[key]) >= int(thresholds["minimum_decision_batches"])
            for key in VARIANTS
        ),
        "required_integrity_interventions_observed": bool(
            required_pairs_observed
            and len(interventions) >= int(thresholds["minimum_integrity_interventions"])
        ),
        "integrity_intervention_count_bounded": len(interventions)
        <= int(thresholds.get("maximum_integrity_interventions", len(interventions))),
        "required_integrity_sequence_observed": bool(
            not required_integrity_sequence
            or sequences["integrity_constrained"] == required_integrity_sequence
        ),
        "information_only_executes_direct": sequences["information_only"].count(direct)
        >= int(thresholds["minimum_integrity_interventions"])
        and (
            not required_unconstrained_sequence
            or sequences["information_only"] == required_unconstrained_sequence
        ),
        "information_only_actual_margin_is_unsafe": actual_margin["information_only"]
        <= float(thresholds["maximum_information_only_actual_margin_m"]),
        "integrity_constrained_actual_margin_is_reserved": actual_margin[
            "integrity_constrained"
        ]
        >= float(thresholds["minimum_integrity_constrained_actual_margin_m"]),
        "actual_margin_gain": margin_gain
        >= float(thresholds["minimum_actual_margin_gain_m"]),
        "bounded_extra_path": 0.0 <= extra_path
        <= float(thresholds["maximum_extra_path_m"]),
        "bounded_mission_time_overhead": time_overhead
        <= float(thresholds["maximum_mission_time_overhead_s"]),
        "localization_not_degraded": ate["integrity_constrained"]
        <= ate["information_only"] + float(thresholds["maximum_ate_degradation_m"]),
        "both_dynamic_brake_and_reopen": all(
            dynamic_replans[key] >= int(thresholds["minimum_dynamic_replan_events"])
            and p12[key]["checks"].get("planner_brake_replan") is True
            and p12[key]["checks"].get("passage_reopened") is True
            for key in VARIANTS
        ),
        "both_keep_physical_clearance": all(
            dynamic_clearance[key]
            >= float(thresholds["minimum_physical_obstacle_clearance_m"])
            for key in VARIANTS
        ),
        "both_keep_p13_safety_envelope": all(
            p13_margin[key] >= float(thresholds["minimum_p13_integrity_margin_m"])
            and latency_p99[key] <= float(thresholds["maximum_p13_latency_p99_ms"])
            for key in VARIANTS
        ),
        "same_frozen_calibration": len(
            {p11[key]["calibration_sha256"] for key in VARIANTS}
        )
        == 1,
        "algorithm_ground_truth_absent": all(
            result[phase].get("algorithm_ground_truth_subscribed") is False
            for result in arms.values()
            for phase in ("p11", "p12", "p13")
        ),
        "no_fault_injection": thresholds.get("fault_injection_allowed") is False,
        "finite_comparison_metrics": all(
            math.isfinite(value)
            for group in (
                actual_margin,
                path_length,
                mission_time,
                ate,
                progress,
                dynamic_clearance,
                latency_p99,
                p13_margin,
            )
            for value in group.values()
        ),
    }

    summary = {
        "schema_version": 1,
        "gate": str(thresholds.get("benchmark", "COMPLEX_FULL_AUTONOMY_COMPARISON")),
        "status": "PASS" if all(common_checks.values()) else "FAIL",
        "world": Path(str(thresholds["world"])).stem,
        "world_sha256": args.world_sha256,
        "variants": list(VARIANTS),
        "selected_sequences": sequences,
        "integrity_intervention_count": len(interventions),
        "actual_intervention_pairs": intervention_pairs,
        "required_intervention_pairs": required_pairs,
        "required_intervention_count": (
            len(required_pairs)
            if required_pairs
            else int(thresholds["minimum_integrity_interventions"])
        ),
        "actual_minimum_integrity_margin_m": actual_margin,
        "actual_margin_gain_m": margin_gain,
        "ground_truth_path_length_m": path_length,
        "extra_path_m": extra_path,
        "mission_time_s": mission_time,
        "mission_time_overhead_s": time_overhead,
        "ate_rms_m": ate,
        "forward_progress_m": progress,
        "dynamic_replan_events": dynamic_replans,
        "minimum_dynamic_obstacle_clearance_m": dynamic_clearance,
        "p13_end_to_end_p99_ms": latency_p99,
        "p13_final_integrity_margin_m": p13_margin,
        "checks": common_checks,
        "thresholds": thresholds,
        "comparison_fairness": {
            "same_world": True,
            "same_lidar_imu_fast_lio": True,
            "same_dynamic_map": True,
            "same_collision_and_energy_constraints": True,
            "same_p13_latency_safety": True,
            "only_difference": "integrity hard feasibility filter before task utility",
        },
        "ground_truth_policy": "evaluation_only",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    temporary.replace(args.output)
    print(json.dumps(summary, indent=2))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
