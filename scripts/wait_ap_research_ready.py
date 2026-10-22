#!/usr/bin/env python3
"""Wait for AP hover or fresh IMPACT mapping inputs with a wall-time deadline."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from xq_sim_interfaces.msg import DirectionalIntegrity, InformationMap


def _number(value) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) else None


def _count(value) -> int:
    return value if isinstance(value, int) and not isinstance(value, bool) and value >= 0 else -1


def _fresh(record: dict | None, ros_now_s: float, wall_now_s: float, *, stamp_key="stamp_s") -> bool:
    if record is None or ros_now_s <= 0.0:
        return False
    stamp, receipt = record.get(stamp_key), record.get("received_wall_s")
    if stamp is None or receipt is None:
        return False
    return bool(0.0 <= ros_now_s - stamp <= 1.0 and 0.0 <= wall_now_s - receipt <= 2.0)


def mapping_checks(inputs: dict, ros_now_s: float, wall_now_s: float) -> dict[str, bool]:
    integrity = inputs.get("integrity")
    information = inputs.get("information_map")
    dynamic = inputs.get("dynamic_map")
    return {
        "ros_clock_started": ros_now_s > 0.0,
        "integrity_fresh": _fresh(integrity, ros_now_s, wall_now_s),
        "integrity_valid": bool(integrity and integrity.get("valid") and integrity.get("finite")),
        "integrity_effective_points": bool(integrity and integrity.get("effective_points", 0) > 0),
        "information_map_fresh": _fresh(information, ros_now_s, wall_now_s),
        "information_map_valid": bool(information and information.get("valid")),
        "information_map_surfels": bool(information and information.get("surfel_count", 0) >= 12),
        "information_map_aligned_finite": bool(information and information.get("aligned_finite")),
        "information_map_truth_free": bool(information and information.get("ground_truth_used") is False),
        "dynamic_map_fresh": _fresh(dynamic, ros_now_s, wall_now_s),
        "dynamic_map_sensor_fresh": _fresh(dynamic, ros_now_s, wall_now_s, stamp_key="sensor_stamp_s"),
        "static_warmup_complete": bool(dynamic and dynamic.get("static_warmup_complete") is True),
        "baseline_static_frozen": bool(dynamic and dynamic.get("baseline_static_frozen") is True),
        "static_voxels_present": bool(dynamic and dynamic.get("static_voxel_count", 0) > 0),
        "dynamic_map_truth_free": bool(dynamic and dynamic.get("ground_truth_used") is False),
    }


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


class ReadinessWaiter(Node):
    def __init__(self, stage: str, output: Path, timeout_s: float) -> None:
        super().__init__(
            "impact_ap_readiness_" + stage,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.stage, self.output, self.timeout_s = stage, output, timeout_s
        self.started_wall = time.monotonic()
        self.inputs: dict[str, dict] = {}
        self.errors: dict[str, str] = {}
        self.result: dict | None = None
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        if stage == "hover":
            self.create_subscription(String, "/impact/ap/mission_status", self._mission_cb, latched)
        else:
            self.create_subscription(DirectionalIntegrity, "/integrity/directional", self._integrity_cb, reliable)
            self.create_subscription(InformationMap, "/integrity/information_map", self._information_cb, reliable)
            self.create_subscription(String, "/mapping/p12/status", self._dynamic_cb, latched)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.10, self.check, clock=self.steady_clock)

    def _mission_cb(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict) or not isinstance(value.get("phase"), str):
                raise ValueError("mission status must contain a string phase")
        except (ValueError, TypeError) as exc:
            self.errors["mission"] = str(exc)[:200]
            return
        self.inputs["mission"] = {
            "phase": value["phase"],
            "elapsed_s": _number(value.get("elapsed_s")),
            "research_started_elapsed_s": _number(value.get("research_started_elapsed_s")),
            "execution_enabled": value.get("execution_enabled") is True,
            "received_wall_s": time.monotonic(),
        }
        self.check()

    def _integrity_cb(self, message: DirectionalIntegrity) -> None:
        finite_values = (
            list(message.integrity_covariance) + list(message.information_matrix)
            + list(message.protection_level_axes)
            + [message.weak_direction_protection_level, message.lambda_min, message.condition_number]
        )
        self.inputs["integrity"] = {
            "stamp_s": float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec),
            "received_wall_s": time.monotonic(),
            "valid": bool(getattr(message, "valid", True)),
            "finite": all(math.isfinite(value) for value in finite_values),
            "effective_points": int(message.effective_points),
            "lambda_min": _number(message.lambda_min),
            "condition_number": _number(message.condition_number),
            "weak_direction_protection_level": _number(message.weak_direction_protection_level),
        }

    def _information_cb(self, message: InformationMap) -> None:
        count = len(message.positions)
        aligned = all(
            len(values) == count
            for values in (message.normals, message.static_confidence, message.geometry_quality, message.last_seen_s)
        )
        finite_geometry = all(
            math.isfinite(value)
            for points in (message.positions, message.normals)
            for point in points
            for value in (point.x, point.y, point.z)
        )
        finite_metadata = all(
            math.isfinite(value)
            for values in (message.static_confidence, message.geometry_quality, message.last_seen_s)
            for value in values
        )
        self.inputs["information_map"] = {
            "stamp_s": float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec),
            "received_wall_s": time.monotonic(),
            "valid": bool(message.valid),
            "surfel_count": count,
            "aligned_finite": aligned and finite_geometry and finite_metadata,
            "ground_truth_used": bool(message.ground_truth_used),
            "source": str(message.source)[:256],
        }

    def _dynamic_cb(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError("P12 status must be an object")
        except (ValueError, TypeError) as exc:
            self.errors["dynamic_map"] = str(exc)[:200]
            return
        sensor_stamp = _count(value.get("source_sensor_stamp_ns"))
        self.inputs["dynamic_map"] = {
            "stamp_s": _number(value.get("stamp_s")),
            "sensor_stamp_s": sensor_stamp * 1.0e-9 if sensor_stamp > 0 else None,
            "received_wall_s": time.monotonic(),
            "static_warmup_complete": value.get("static_warmup_complete") is True,
            "baseline_static_frozen": value.get("baseline_static_frozen") is True,
            "static_voxel_count": _count(value.get("static_voxel_count")),
            "baseline_static_voxel_count": _count(value.get("baseline_static_voxel_count")),
            "ground_truth_used": value.get("ground_truth_used") is not False,
        }

    def _input_summaries(self, ros_now_s: float, wall_now_s: float) -> dict:
        summaries = {}
        for name, record in self.inputs.items():
            summary = {key: value for key, value in record.items() if key != "received_wall_s"}
            summary["received_wall_elapsed_s"] = record["received_wall_s"] - self.started_wall
            summary["wall_age_s"] = wall_now_s - record["received_wall_s"]
            if record.get("stamp_s") is not None:
                summary["ros_age_s"] = ros_now_s - record["stamp_s"]
            summaries[name] = summary
        return summaries

    def finish(self, status: str, reason: str, checks: dict | None = None) -> None:
        if self.result is not None:
            return
        now_wall = time.monotonic()
        now_ros = self.get_clock().now().nanoseconds * 1.0e-9
        self.result = {
            "schema_version": 1,
            "gate": "R1_AP_RESEARCH_READINESS",
            "stage": self.stage,
            "status": status,
            "reason": reason,
            "elapsed_wall_s": now_wall - self.started_wall,
            "timeout_s": self.timeout_s,
            "ros_now_s": now_ros,
            "checks": checks or {},
            "last_inputs": self._input_summaries(now_ros, now_wall),
            "parse_errors": self.errors,
            "ground_truth_subscribed": False,
        }
        _write_json(self.output, self.result)
        print(json.dumps(self.result, ensure_ascii=False, allow_nan=False), flush=True)

    def check(self) -> None:
        if self.result is not None:
            return
        now_wall = time.monotonic()
        now_ros = self.get_clock().now().nanoseconds * 1.0e-9
        checks = (
            mapping_checks(self.inputs, now_ros, now_wall)
            if self.stage == "mapping"
            else {"mission_in_research": self.inputs.get("mission", {}).get("phase") == "RESEARCH"}
        )
        if now_wall - self.started_wall >= self.timeout_s:
            self.finish("FAIL", "wall-time readiness timeout", checks)
        elif self.stage == "hover":
            phase = self.inputs.get("mission", {}).get("phase")
            if phase == "RESEARCH":
                self.finish("READY", "stable hover reached; research may start", checks)
            elif phase in {"FAILED", "DONE"}:
                self.finish("FAIL", "mission terminated before research startup: " + phase, checks)
        elif all(checks.values()):
            self.finish("READY", "fresh integrity and warmed static/information maps", checks)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", choices=("hover", "mapping"), required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    args, ros_args = parser.parse_known_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be positive and finite")
    node = None
    try:
        rclpy.init(args=ros_args)
        node = ReadinessWaiter(args.stage, args.output, args.timeout)
        while rclpy.ok() and node.result is None:
            node.check()
            if node.result is None:
                rclpy.spin_once(node, timeout_sec=0.2)
        if node.result is None:
            node.finish("FAIL", "ROS context stopped before readiness")
        return 0 if node.result["status"] == "READY" else 1
    except (Exception, KeyboardInterrupt) as exc:
        reason = type(exc).__name__ + ": " + str(exc)[:500]
        if node is not None:
            node.finish("FAIL", reason)
        else:
            _write_json(args.output, {"schema_version": 1, "stage": args.stage, "status": "FAIL", "reason": reason})
        print(reason, file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
