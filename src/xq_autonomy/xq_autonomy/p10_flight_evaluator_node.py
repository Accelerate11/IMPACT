"""Evaluation-only P10 flight metrics; Ground Truth never leaves this node."""

from __future__ import annotations

import bisect
import hashlib
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from xq_sim_interfaces.msg import ActivePerceptionDecision, DirectionalIntegrity

from .p10_active_perception_node import _cloud_xyz


def _stamp(message) -> float:
    return float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)


def _yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class P10FlightEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p10_flight_evaluator")
        self.declare_parameter("variant", "baseline")
        self.declare_parameter("result_file", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("margin_reserve_m", 0.10)
        self.declare_parameter("body_radius_m", 0.35)
        self.declare_parameter("base_reserve_m", 0.10)
        self.declare_parameter("tracking_reserve_m", 0.10)
        self.declare_parameter("latency_p99_s", 0.10)
        self.declare_parameter("maximum_acceleration_mps2", 1.0)
        self.variant = str(self.get_parameter("variant").value)
        self._k_alpha, self._calibration_sha = self._load_calibration()
        self.odom: list[tuple[float, np.ndarray, float]] = []
        self.truth: list[tuple[float, np.ndarray, float]] = []
        self.directional: list[tuple[float, np.ndarray, np.ndarray]] = []
        self.margin_samples: list[tuple[float, float, float, float]] = []
        self._odom_latest: Odometry | None = None
        self._integrity_latest: DirectionalIntegrity | None = None
        self._decision: ActivePerceptionDecision | None = None
        self._flight_status: dict[str, object] = {}
        self._complete_wall_s: float | None = None
        self._finalized = False

        reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, reliable)
        self.create_subscription(
            Odometry, "/xq/eval/agent_01/ground_truth", self._truth_cb, reliable
        )
        self.create_subscription(
            DirectionalIntegrity, "/integrity/directional", self._directional_cb, reliable
        )
        self.create_subscription(PointCloud2, "/xq/p5/cloud_map", self._cloud_cb, reliable)
        self.create_subscription(
            ActivePerceptionDecision,
            "/integrity/active_perception_decision",
            self._decision_cb,
            latched,
        )
        self.create_subscription(String, "/xq/p10/flight_status", self._status_cb, latched)
        self.create_timer(0.25, self._timer)
        self.get_logger().info(
            f"P10 evaluator variant={self.variant}; Ground Truth is evaluation-only"
        )

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _load_calibration(self) -> tuple[float, str]:
        path = Path(str(self.get_parameter("calibration_file").value))
        raw = path.read_bytes()
        calibration = json.loads(raw)
        if not calibration.get("train_only") or calibration.get("test_data_used", True):
            raise ValueError("P10 evaluator requires frozen train-only calibration")
        factors = [float(value["k95"]) for value in calibration["directional"].values()]
        return max(factors), hashlib.sha256(raw).hexdigest()

    @staticmethod
    def _position(message: Odometry) -> np.ndarray:
        value = message.pose.pose.position
        return np.asarray((value.x, value.y, value.z), dtype=float)

    def _odom_cb(self, message: Odometry) -> None:
        position = self._position(message)
        if np.isfinite(position).all():
            self.odom.append((_stamp(message), position, _yaw(message)))
            self._odom_latest = message

    def _truth_cb(self, message: Odometry) -> None:
        position = self._position(message)
        if np.isfinite(position).all():
            self.truth.append((_stamp(message), position, _yaw(message)))

    def _directional_cb(self, message: DirectionalIntegrity) -> None:
        covariance = np.asarray(message.integrity_covariance, dtype=float).reshape(3, 3)
        weak = np.asarray(message.weak_direction_map, dtype=float)
        if np.isfinite(covariance).all() and np.isfinite(weak).all():
            self.directional.append((_stamp(message), covariance, weak))
            self._integrity_latest = message

    def _decision_cb(self, message: ActivePerceptionDecision) -> None:
        self._decision = message

    def _status_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if payload.get("variant") != self.variant:
            return
        self._flight_status = payload
        if payload.get("finished") is True and self._complete_wall_s is None:
            self._complete_wall_s = time.monotonic()

    def _cloud_cb(self, message: PointCloud2) -> None:
        if self._odom_latest is None or self._integrity_latest is None:
            return
        points = _cloud_xyz(message)
        if len(points) == 0:
            return
        position = self._position(self._odom_latest)
        deltas = points - position
        distance2 = np.einsum("ij,ij->i", deltas, deltas)
        index = int(np.argmin(distance2))
        clearance = math.sqrt(max(float(distance2[index]), 0.0))
        if clearance <= 1.0e-6:
            return
        direction = deltas[index] / clearance
        covariance = np.asarray(
            self._integrity_latest.integrity_covariance, dtype=float
        ).reshape(3, 3)
        protection = self._k_alpha * math.sqrt(
            max(float(direction @ covariance @ direction), 0.0)
        )
        velocity = self._odom_latest.twist.twist.linear
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        latency_reserve = (
            speed * self._float("latency_p99_s")
            + 0.5
            * self._float("maximum_acceleration_mps2")
            * self._float("latency_p99_s") ** 2
        )
        alert = clearance - (
            self._float("body_radius_m")
            + self._float("base_reserve_m")
            + self._float("tracking_reserve_m")
            + latency_reserve
        )
        self.margin_samples.append((_stamp(message), alert - protection, alert, protection))

    @staticmethod
    def _interpolate(samples, stamp_s: float):
        times = [sample[0] for sample in samples]
        index = bisect.bisect_left(times, stamp_s)
        if index == 0 or index >= len(samples):
            return None
        t0, p0, y0 = samples[index - 1]
        t1, p1, y1 = samples[index]
        if t1 <= t0 or stamp_s - t0 > 0.11 or t1 - stamp_s > 0.11:
            return None
        ratio = (stamp_s - t0) / (t1 - t0)
        yaw_delta = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))
        return p0 + ratio * (p1 - p0), y0 + ratio * yaw_delta

    def _matched(self):
        matched = []
        for stamp_s, position, yaw in self.odom:
            truth = self._interpolate(self.truth, stamp_s)
            if truth is not None:
                matched.append((stamp_s, position, yaw, truth[0], truth[1]))
        return matched

    @staticmethod
    def _path_length(samples) -> float:
        if len(samples) < 2:
            return 0.0
        positions = np.stack([sample[1] for sample in samples])
        steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        return float(np.sum(steps[steps < 0.20]))

    def _metrics(self) -> dict[str, object]:
        matched = self._matched()
        if not matched:
            return {"matched_samples": 0}
        _, estimate0, yaw0, truth0, truth_yaw0 = matched[0]
        yaw_offset = truth_yaw0 - yaw0
        c, s = math.cos(yaw_offset), math.sin(yaw_offset)
        rotation = np.asarray(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
        aligned = np.stack([rotation @ (sample[1] - estimate0) + truth0 for sample in matched])
        truth = np.stack([sample[3] for sample in matched])
        errors = truth - aligned
        weak_errors = []
        direction_times = [sample[0] for sample in self.directional]
        for sample, error in zip(matched, errors):
            if not direction_times:
                break
            index = min(
                range(max(0, bisect.bisect_left(direction_times, sample[0]) - 1),
                      min(len(direction_times), bisect.bisect_left(direction_times, sample[0]) + 1)),
                key=lambda value: abs(direction_times[value] - sample[0]),
                default=None,
            )
            if index is not None and abs(direction_times[index] - sample[0]) <= 0.11:
                weak = rotation @ self.directional[index][2]
                weak /= max(float(np.linalg.norm(weak)), 1.0e-12)
                weak_errors.append(abs(float(weak @ error)))
        norm_errors = np.linalg.norm(errors, axis=1)
        truth_relative = truth - truth[0]
        margins = np.asarray([sample[1] for sample in self.margin_samples], dtype=float)
        alerts = np.asarray([sample[2] for sample in self.margin_samples], dtype=float)
        protections = np.asarray([sample[3] for sample in self.margin_samples], dtype=float)
        return {
            "matched_samples": len(matched),
            "ate_rms_m": float(np.sqrt(np.mean(norm_errors ** 2))),
            "position_error_max_m": float(np.max(norm_errors)),
            "weak_direction_error_rms_m": (
                float(np.sqrt(np.mean(np.square(weak_errors)))) if weak_errors else None
            ),
            "weak_direction_error_max_m": float(np.max(weak_errors)) if weak_errors else None,
            "actual_minimum_integrity_margin_m": float(np.min(margins)) if len(margins) else None,
            "actual_minimum_alert_limit_m": float(np.min(alerts)) if len(alerts) else None,
            "actual_maximum_protection_level_m": (
                float(np.max(protections)) if len(protections) else None
            ),
            "integrity_margin_samples": len(margins),
            "ground_truth_path_length_m": self._path_length(self.truth),
            "maximum_lateral_excursion_m": float(np.max(np.abs(truth_relative[:, 1]))),
            "mission_time_s": float(self._flight_status.get("elapsed_s", 0.0)),
        }

    def _finalize(self) -> None:
        metrics = self._metrics()
        decision = self._decision
        status = self._flight_status
        checks = {
            "matched_samples": int(metrics.get("matched_samples", 0)) >= 100,
            "integrity_samples": int(metrics.get("integrity_margin_samples", 0)) >= 20,
            "controller_complete": status.get("finished") is True,
            "decision_valid_hard": bool(
                decision is not None and decision.valid and decision.hard_constraint
            ),
            "finite_core_metrics": all(
                metrics.get(name) is not None and math.isfinite(float(metrics[name]))
                for name in (
                    "ate_rms_m",
                    "weak_direction_error_rms_m",
                    "actual_minimum_integrity_margin_m",
                    "ground_truth_path_length_m",
                    "mission_time_s",
                )
            ),
            "ground_truth_evaluator_only": True,
        }
        if self.variant == "minimum_excitation":
            checks.update(
                {
                    "baseline_insufficient": bool(decision and decision.baseline_insufficient),
                    "recovery_found": bool(decision and decision.recovery_found),
                    "selected_applied": status.get("selected_applied") is True,
                }
            )
        result = {
            "schema_version": 1,
            "gate": "P10_LONG_CORRIDOR_FLIGHT_ARM",
            "variant": self.variant,
            "status": "PASS" if all(checks.values()) else "FAIL",
            "metrics": metrics,
            "decision": None if decision is None else {
                "candidate_names": list(decision.candidate_names),
                "predicted_minimum_margins": list(decision.predicted_minimum_margins),
                "costs": list(decision.costs),
                "feasible": list(decision.feasible),
                "selected_name": decision.selected_name,
                "baseline_insufficient": decision.baseline_insufficient,
                "recovery_found": decision.recovery_found,
                "reason": decision.reason,
            },
            "flight_status": status,
            "checks": checks,
            "calibration_sha256": self._calibration_sha,
            "ground_truth_consumer": "xq_p10_flight_evaluator_only",
            "algorithm_ground_truth_subscribed": False,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        path = Path(str(self.get_parameter("result_file").value))
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
        self._finalized = True
        self.get_logger().info(f"P10 {self.variant} flight arm -> {result['status']}")

    def _timer(self) -> None:
        if (
            not self._finalized
            and self._complete_wall_s is not None
            and time.monotonic() - self._complete_wall_s >= 1.0
        ):
            self._finalize()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P10FlightEvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node._finalized and node._complete_wall_s is not None:
            node._finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
