#!/usr/bin/env python3
"""Compare the frozen 50 ms and 200 ms P13 Gazebo trials."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--low", type=Path, required=True)
    parser.add_argument("--high", type=Path, required=True)
    parser.add_argument("--low-p12", type=Path, required=True)
    parser.add_argument("--high-p12", type=Path, required=True)
    parser.add_argument("--thresholds", type=Path, required=True)
    parser.add_argument("--world-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    low, high = load(args.low), load(args.high)
    low_p12, high_p12 = load(args.low_p12), load(args.high_p12)
    thresholds = load(args.thresholds)
    lm, hm = low["metrics"], high["metrics"]
    p99_delta = float(hm["end_to_end_p99_ms"]) - float(lm["end_to_end_p99_ms"])
    speed_reduction = float(lm["final_speed_limit_mps"]) - float(
        hm["final_speed_limit_mps"]
    )
    nominal_speed = float(thresholds["maximum_speed_mps"])
    acceleration = float(thresholds["maximum_acceleration_mps2"])
    clearance = float(thresholds["geometric_clearance_m"])
    fixed_buffer = float(thresholds["fixed_buffer_m"])
    low_latency_s = 1.0e-3 * float(lm["end_to_end_p99_ms"])
    high_latency_s = 1.0e-3 * float(hm["end_to_end_p99_ms"])
    low_unmitigated_alert = clearance - fixed_buffer - (
        nominal_speed * low_latency_s + 0.5 * acceleration * low_latency_s**2
    )
    high_unmitigated_alert = clearance - fixed_buffer - (
        nominal_speed * high_latency_s + 0.5 * acceleration * high_latency_s**2
    )
    alert_reduction = low_unmitigated_alert - high_unmitigated_alert
    low_elapsed = float(low["flight_status"].get("elapsed_s", 0.0))
    high_elapsed = float(high["flight_status"].get("elapsed_s", 0.0))
    checks = {
        "both_latency_trials_pass": low["status"] == "PASS" and high["status"] == "PASS",
        "p12_capability_retained_both_trials": low_p12["status"] == "PASS"
        and high_p12["status"] == "PASS",
        "same_world_geometry": bool(args.world_sha256),
        "frozen_50_vs_200_ms_profiles": low["profile"] == "low_50ms"
        and high["profile"] == "high_200ms",
        "high_p99_is_larger": p99_delta
        >= float(thresholds["minimum_p99_separation_ms"]),
        "high_unmitigated_alert_limit_is_tighter": alert_reduction
        >= float(thresholds["minimum_alert_limit_reduction_m"]),
        "high_speed_is_more_conservative": speed_reduction
        >= float(thresholds["minimum_speed_limit_reduction_mps"]),
        "conservative_policy_observable": high_elapsed > low_elapsed,
        "ground_truth_evaluator_only": not low["algorithm_ground_truth_subscribed"]
        and not high["algorithm_ground_truth_subscribed"],
    }
    result = {
        "schema_version": 1,
        "gate": "P13_LATENCY_AWARE_SAFETY",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "comparison": {
            "low_profile": low["profile"],
            "high_profile": high["profile"],
            "low_end_to_end_p99_ms": lm["end_to_end_p99_ms"],
            "high_end_to_end_p99_ms": hm["end_to_end_p99_ms"],
            "p99_separation_ms": p99_delta,
            "low_unmitigated_alert_limit_m": low_unmitigated_alert,
            "high_unmitigated_alert_limit_m": high_unmitigated_alert,
            "unmitigated_alert_limit_reduction_m": alert_reduction,
            "low_applied_alert_limit_m": lm["final_alert_limit_m"],
            "high_applied_alert_limit_m": hm["final_alert_limit_m"],
            "low_integrity_margin_m": lm["final_integrity_margin_m"],
            "high_integrity_margin_m": hm["final_integrity_margin_m"],
            "low_speed_limit_mps": lm["final_speed_limit_mps"],
            "high_speed_limit_mps": hm["final_speed_limit_mps"],
            "speed_limit_reduction_mps": speed_reduction,
            "low_mission_elapsed_s": low_elapsed,
            "high_mission_elapsed_s": high_elapsed,
            "world_sha256": args.world_sha256,
        },
        "checks": checks,
        "proof_boundary": (
            "Same Gazebo world and LiDAR/FAST-LIO/P12 stack; only injected planner "
            "processing delay changes from 50 ms to 200 ms. Ground Truth is evaluator-only."
        ),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    temporary.replace(args.output)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
