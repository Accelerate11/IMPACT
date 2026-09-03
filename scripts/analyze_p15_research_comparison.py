#!/usr/bin/env python3
"""Aggregate the P15 map-derived, integrity-only causal comparison."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path


VARIANTS = ("information_only", "integrity_constrained")


def _read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _ratio(numerator: float, denominator: float) -> float:
    return float(numerator / max(abs(denominator), 1.0e-12))


def _read_env(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.lstrip().startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-dir", required=True, type=Path)
    parser.add_argument("--thresholds", required=True, type=Path)
    parser.add_argument("--world-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    thresholds = _read(args.thresholds)
    arms = {
        variant: {
            phase: _read(args.result_dir / variant / f"{phase}-result.json")
            for phase in ("p11", "p12", "p13")
        }
        for variant in VARIANTS
    }
    p11 = {variant: arms[variant]["p11"] for variant in VARIANTS}
    run_configuration = {
        variant: _read_env(args.result_dir / variant / "run.env")
        for variant in VARIANTS
    }
    observed_runtime_configuration = {
        variant: {
            "controller_dynamic_path_query_mode": arms[variant]["p11"]
            .get("flight_status", {})
            .get("dynamic_path_query_mode"),
            "controller_minimum_dynamic_cluster_points": arms[variant]["p11"]
            .get("flight_status", {})
            .get("minimum_dynamic_cluster_points"),
            "controller_dynamic_cluster_radius_m": arms[variant]["p11"]
            .get("flight_status", {})
            .get("dynamic_cluster_radius_m"),
            "map_dynamic_path_query_mode": arms[variant]["p12"]
            .get("map_status", {})
            .get("dynamic_path_query_mode"),
            "map_minimum_dynamic_cluster_points": arms[variant]["p12"]
            .get("map_status", {})
            .get("minimum_dynamic_cluster_points"),
            "map_dynamic_cluster_radius_m": arms[variant]["p12"]
            .get("map_status", {})
            .get("dynamic_cluster_radius_m"),
            "runtime_integrity_guard_mode": arms[variant]["p13"]
            .get("flight_status", {})
            .get("runtime_integrity_guard_mode"),
        }
        for variant in VARIANTS
    }
    metrics = {variant: p11[variant]["metrics"] for variant in VARIANTS}
    baseline = metrics["information_only"]
    constrained = metrics["integrity_constrained"]
    decisions = {
        variant: list(p11[variant].get("decisions", [])) for variant in VARIANTS
    }
    sequences = {
        "information_only": [
            item.get("unconstrained_selected_name", "")
            for item in decisions["information_only"]
        ],
        "integrity_constrained": [
            item.get("selected_name", "")
            for item in decisions["integrity_constrained"]
        ],
    }
    interventions = [
        {
            "batch_id": int(item.get("batch_id", -1)),
            "task_selected": item.get("unconstrained_selected_name", ""),
            "integrity_selected": item.get("selected_name", ""),
        }
        for item in decisions["integrity_constrained"]
        if item.get("unconstrained_selected_name") != item.get("selected_name")
    ]
    utility_band_applications = [
        int(item.get("batch_id", -1))
        for item in decisions["integrity_constrained"]
        if item.get("minimum_intervention_applied") is True
    ]

    realized_margin_gain = float(
        constrained["gt_minimum_realized_margin_m"]
        - baseline["gt_minimum_realized_margin_m"]
    )
    predicted_margin_gain = float(
        constrained["actual_minimum_integrity_margin_m"]
        - baseline["actual_minimum_integrity_margin_m"]
    )
    availability_gain = float(
        constrained["availability_rate"] - baseline["availability_rate"]
    )
    path_overhead = _ratio(
        constrained["ground_truth_path_length_m"]
        - baseline["ground_truth_path_length_m"],
        baseline["ground_truth_path_length_m"],
    )
    time_overhead = _ratio(
        constrained["mission_time_s"] - baseline["mission_time_s"],
        baseline["mission_time_s"],
    )
    ate_change = _ratio(
        constrained["ate_rms_m"] - baseline["ate_rms_m"],
        baseline["ate_rms_m"],
    )
    ate_change_m = float(constrained["ate_rms_m"] - baseline["ate_rms_m"])
    candidate_modes = {
        str(item.get("candidate_generation_mode", ""))
        for values in decisions.values()
        for item in values
    }
    metric_sources = {
        str(item.get("metric_source", ""))
        for values in decisions.values()
        for item in values
    }
    checks = {
        "all_phase_gates_pass": all(
            arms[variant][phase].get("status") == "PASS"
            for variant in VARIANTS
            for phase in ("p11", "p12", "p13")
        ),
        "paired_research_candidate_mode": candidate_modes
        == {str(thresholds["required_candidate_generation_mode"])},
        "paired_online_metric_source": metric_sources
        == {str(thresholds["required_metric_source"])},
        "active_trajectory_dynamic_query": all(
            run_configuration[variant].get("dynamic_path_query_mode")
            == str(thresholds["required_dynamic_path_query_mode"])
            for variant in VARIANTS
        ),
        "dynamic_cluster_support_predeclared": all(
            int(
                run_configuration[variant].get(
                    "minimum_dynamic_cluster_points", "0"
                )
            )
            >= int(thresholds["required_minimum_dynamic_cluster_points"])
            and math.isclose(
                float(
                    run_configuration[variant].get(
                        "dynamic_cluster_radius_m", "nan"
                    )
                ),
                float(thresholds["required_dynamic_cluster_radius_m"]),
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            for variant in VARIANTS
        ),
        "runtime_integrity_guard_enabled": all(
            run_configuration[variant].get("runtime_integrity_guard_mode")
            == str(thresholds["required_runtime_integrity_guard_mode"])
            for variant in VARIANTS
        ),
        "runtime_configuration_observed": all(
            observed_runtime_configuration[variant].get(
                "controller_dynamic_path_query_mode"
            )
            == str(thresholds["required_dynamic_path_query_mode"])
            and observed_runtime_configuration[variant].get(
                "map_dynamic_path_query_mode"
            )
            == str(thresholds["required_dynamic_path_query_mode"])
            and int(
                observed_runtime_configuration[variant].get(
                    "controller_minimum_dynamic_cluster_points", 0
                ) or 0
            )
            >= int(thresholds["required_minimum_dynamic_cluster_points"])
            and int(
                observed_runtime_configuration[variant].get(
                    "map_minimum_dynamic_cluster_points", 0
                ) or 0
            )
            >= int(thresholds["required_minimum_dynamic_cluster_points"])
            and math.isclose(
                float(
                    observed_runtime_configuration[variant].get(
                        "controller_dynamic_cluster_radius_m", math.nan
                    ) or math.nan
                ),
                float(thresholds["required_dynamic_cluster_radius_m"]),
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            and math.isclose(
                float(
                    observed_runtime_configuration[variant].get(
                        "map_dynamic_cluster_radius_m", math.nan
                    ) or math.nan
                ),
                float(thresholds["required_dynamic_cluster_radius_m"]),
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            and observed_runtime_configuration[variant].get(
                "runtime_integrity_guard_mode"
            )
            == str(thresholds["required_runtime_integrity_guard_mode"])
            for variant in VARIANTS
        ),
        "independent_gt_samples": all(
            int(metrics[variant].get("gt_integrity_matched_samples", 0))
            >= int(thresholds["minimum_gt_integrity_samples"])
            for variant in VARIANTS
        ),
        "full_mission_progress": all(
            float(metrics[variant].get("forward_progress_m", -math.inf))
            >= float(thresholds["minimum_forward_progress_m"])
            for variant in VARIANTS
        ),
        "pl_coverage_retained": all(
            float(metrics[variant].get("pl_empirical_coverage_rate", 0.0))
            >= float(thresholds["minimum_pl_coverage_rate"])
            for variant in VARIANTS
        ),
        "constrained_no_realized_safety_violation": int(
            constrained.get("gt_safety_violation_count", -1)
        ) <= int(thresholds["maximum_constrained_safety_violations"]),
        "baseline_exposes_integrity_unavailability": float(
            baseline.get("availability_rate", 1.0)
        ) <= float(thresholds["maximum_baseline_availability_rate"]),
        "constrained_availability": float(constrained.get("availability_rate", 0.0))
        >= float(thresholds["minimum_constrained_availability_rate"]),
        "availability_gain": availability_gain
        >= float(thresholds["minimum_availability_gain"]),
        "realized_margin_gain": realized_margin_gain
        >= float(thresholds["minimum_realized_margin_gain_m"]),
        "bounded_path_overhead": path_overhead
        <= float(thresholds["maximum_path_overhead_fraction"]),
        "bounded_time_overhead": time_overhead
        <= float(thresholds["maximum_time_overhead_fraction"]),
        "bounded_absolute_ate_change": ate_change_m
        <= float(thresholds["maximum_ate_degradation_m"]),
        "hard_gate_intervened": len(interventions)
        >= int(thresholds["minimum_integrity_interventions"]),
        "minimum_intervention_policy_exercised": len(utility_band_applications)
        >= int(thresholds["minimum_utility_band_applications"]),
        "zero_hmi_in_both_arms": all(
            int(metrics[variant].get("hmi_count", -1)) == 0
            for variant in VARIANTS
        ),
        "ground_truth_isolation": all(
            p11[variant].get("algorithm_ground_truth_subscribed") is False
            and p11[variant].get("ground_truth_consumer")
            == "xq_p11_flight_evaluator_only"
            for variant in VARIANTS
        ),
    }
    output = {
        "schema_version": 1,
        "gate": "P15_MAP_DERIVED_INTEGRITY_RESEARCH_COMPARISON",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "world_sha256": args.world_sha256,
        "candidate_generation_mode": sorted(candidate_modes),
        "metric_source": sorted(metric_sources),
        "run_configuration": run_configuration,
        "observed_runtime_configuration": observed_runtime_configuration,
        "executed_sequences": sequences,
        "integrity_interventions": interventions,
        "utility_band_application_batches": utility_band_applications,
        "metrics": {
            variant: {
                key: metrics[variant].get(key)
                for key in (
                    "ate_rms_m",
                    "weak_direction_error_rms_m",
                    "actual_minimum_integrity_margin_m",
                    "gt_minimum_realized_margin_m",
                    "gt_safety_violation_count",
                    "hmi_count",
                    "availability_rate",
                    "false_alarm_rate",
                    "pl_empirical_coverage_rate",
                    "pl_tightness_mean_m",
                    "ground_truth_path_length_m",
                    "mission_time_s",
                    "forward_progress_m",
                )
            }
            for variant in VARIANTS
        },
        "effects": {
            "predicted_margin_gain_m": predicted_margin_gain,
            "realized_margin_gain_m": realized_margin_gain,
            "availability_gain": availability_gain,
            "path_overhead_fraction": path_overhead,
            "mission_time_overhead_fraction": time_overhead,
            "ate_change_fraction": ate_change,
            "ate_change_m": ate_change_m,
        },
        "checks": checks,
        "thresholds": thresholds,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(output, indent=2))
    return 0 if output["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
