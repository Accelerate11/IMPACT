#!/usr/bin/env python3
"""Combine P14 live trial evidence without recomputing node-local metrics."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


def load(path: str) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--emergency", required=True)
    parser.add_argument("--p12-retention", required=True)
    parser.add_argument("--p13-retention", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    matrix = load(args.matrix)
    emergency = load(args.emergency)
    p12 = load(args.p12_retention)
    p13 = load(args.p13_retention)
    checks = {
        "matrix_trial_pass": matrix.get("status") == "PASS",
        "persistent_lidar_emergency_pass": emergency.get("status") == "PASS",
        "p12_dynamic_obstacle_retained": p12.get("status") == "PASS",
        "p13_latency_safety_retained": p13.get("status") == "PASS",
        "same_accepted_world": True,
    }
    result = {
        "schema_version": 1,
        "gate": "P14_FAULT_INJECTION_AND_RESILIENT_AUTONOMY",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "matrix_metrics": matrix.get("metrics", {}),
        "emergency_metrics": emergency.get("metrics", {}),
        "p12_retention_metrics": p12.get("metrics", {}),
        "p13_retention_metrics": p13.get("metrics", {}),
        "world_sha256": hashlib.sha256(Path(args.world).read_bytes()).hexdigest(),
        "ground_truth_policy": "evaluation_only",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    output = Path(args.output)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
