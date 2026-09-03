"""Evaluation-only P11 metrics; Ground Truth never leaves this node."""

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
from xq_sim_interfaces.msg import DirectionalIntegrity, IntegrityExplorationDecision

from .p10_active_perception_node import _cloud_xyz
from .integrity_evaluation import evaluate_ground_truth_integrity


def _stamp(message) -> float:
    return float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)


def _yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rolling_horizon_complete(
    status: dict[str, object], minimum_batches: int, accepted_batches: int
) -> bool:
    """Validate closed planning windows, including safety-interrupted ones."""
    return bool(
        status.get("rolling_horizon") is True
        and int(status.get("planning_windows_closed", 0)) >= int(minimum_batches)
        and int(status.get("decisions_applied", 0)) == int(accepted_batches)
    )


class P11FlightEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p11_flight_evaluator")
        self.declare_parameter("variant", "information_only")
        self.declare_parameter("result_file", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("body_radius_m", 0.35)
        self.declare_parameter("base_reserve_m", 0.10)
        self.declare_parameter("tracking_reserve_m", 0.10)
        self.declare_parameter("latency_p99_s", 0.10)
        self.declare_parameter("maximum_acceleration_mps2", 1.0)
        self.declare_parameter("mission_distance_m", 24.0)
        self.declare_parameter("trajectory_distance_m", 7.5)
        self.declare_parameter("goal_tolerance_m", 0.50)
        self.declare_parameter("minimum_decision_batches", 4)
        self.variant = str(self.get_parameter("variant").value)
        self._k_alpha, self._calibration_sha = self._load_calibration()
        self.odom: list[tuple[float, np.ndarray, float]] = []
        self.truth: list[tuple[float, np.ndarray, float]] = []
        self.directional: list[tuple[float, np.ndarray, np.ndarray]] = []
        self.margin_samples: list[tuple[float, ...]] = []
        self._odom_latest: Odometry | None = None
        self._integrity_latest: DirectionalIntegrity | None = None
        self._decision: IntegrityExplorationDecision | None = None
        self._decisions: dict[int, IntegrityExplorationDecision] = {}
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
            IntegrityExplorationDecision,
            "/integrity/exploration_decision",
            self._decision_cb,
            latched,
        )
        self.create_subscription(String, "/xq/p11/flight_status", self._status_cb, latched)
        self.create_timer(0.25, self._timer)
        self.get_logger().info(
            f"P11 evaluator variant={self.variant}; Ground Truth is evaluation-only"
        )

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _load_calibration(self) -> tuple[float, str]:
        path = Path(str(self.get_parameter("calibration_file").value))
        raw = path.read_bytes()
        calibration = json.loads(raw)
        if not calibration.get("train_only") or calibration.get("test_data_used", True):
            raise ValueError("P11 evaluator requires frozen train-only calibration")
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

    def _decision_cb(self, message: IntegrityExplorationDecision) -> None:
        self._decision = message
        self._decisions[int(message.batch_id)] = message

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
        self.margin_samples.append(
            (
                _stamp(message),
                alert - protection,
                alert,
                protection,
                float(position[0]),
                float(position[1]),
                float(points[index][0]),
                float(points[index][1]),
                float(points[index][2]),
                float(position[2]),
            )
        )

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

    def _executed_information_gain(self) -> float | None:
        applied = self._applied_decisions()
        if not applied:
            return None
        total = 0.0
        for decision in applied:
            name = (
                decision.unconstrained_selected_name
                if self.variant == "information_only"
                else decision.selected_name
            )
            if name not in decision.candidate_names:
                return None
            total += float(
                decision.information_gains[list(decision.candidate_names).index(name)]
            )
        return total

    def _applied_decisions(self) -> list[IntegrityExplorationDecision]:
        """Return accepted batches; rejected immutable retries were not flown."""
        applied = []
        for key in sorted(self._decisions):
            decision = self._decisions[key]
            name = (
                decision.unconstrained_selected_name
                if self.variant == "information_only"
                else decision.selected_name
            )
            if name and name in decision.candidate_names:
                applied.append(decision)
        return applied

    @staticmethod
    def _decision_payload(decision: IntegrityExplorationDecision) -> dict[str, object]:
        return {
            "batch_id": int(decision.batch_id),
            "trajectory_ids": list(decision.trajectory_ids),
            "candidate_names": list(decision.candidate_names),
            "frontier_ids": list(decision.frontier_ids),
            "information_gains": list(decision.information_gains),
            "progress_efficiencies": list(decision.progress_efficiencies),
            "map_observation_gains": list(decision.map_observation_gains),
            "localization_information_traces": list(
                decision.localization_information_traces
            ),
            "utilities": list(decision.utilities),
            "energy_costs": list(decision.energy_costs),
            "return_energy_costs": list(decision.return_energy_costs),
            "collision_probabilities": list(decision.collision_probabilities),
            "predicted_minimum_margins": list(decision.predicted_minimum_margins),
            "integrity_feasible": list(decision.integrity_feasible),
            "collision_feasible": list(decision.collision_feasible),
            "energy_feasible": list(decision.energy_feasible),
            "feasible": list(decision.feasible),
            "unconstrained_selected_name": decision.unconstrained_selected_name,
            "selected_name": decision.selected_name,
            "hard_constraint": decision.hard_constraint,
            "margin_in_utility": decision.margin_in_utility,
            "minimum_intervention_applied": decision.minimum_intervention_applied,
            "utility_indifference_band": decision.utility_indifference_band,
            "candidate_generation_mode": decision.candidate_generation_mode,
            "metric_source": decision.metric_source,
            "reason": decision.reason,
        }

    def _metrics(self) -> dict[str, object]:
        matched = self._matched()
        if not matched:
            return {"matched_samples": 0}
        _, estimate0, yaw0, truth0, truth_yaw0 = matched[0]
        yaw_offset = truth_yaw0 - yaw0
        c, s = math.cos(yaw_offset), math.sin(yaw_offset)
        rotation = np.asarray(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
        aligned = np.stack(
            [rotation @ (sample[1] - estimate0) + truth0 for sample in matched]
        )
        truth = np.stack([sample[3] for sample in matched])
        errors = truth - aligned
        weak_errors = []
        direction_times = [sample[0] for sample in self.directional]
        for sample, error in zip(matched, errors):
            if not direction_times:
                break
            insertion = bisect.bisect_left(direction_times, sample[0])
            choices = range(max(0, insertion - 1), min(len(direction_times), insertion + 1))
            index = min(
                choices,
                key=lambda value: abs(direction_times[value] - sample[0]),
                default=None,
            )
            if index is not None and abs(direction_times[index] - sample[0]) <= 0.11:
                weak = rotation @ self.directional[index][2]
                weak /= max(float(np.linalg.norm(weak)), 1.0e-12)
                weak_errors.append(abs(float(weak @ error)))
        norm_errors = np.linalg.norm(errors, axis=1)
        truth_relative = truth - truth[0]
        matched_stamps = np.asarray([sample[0] for sample in matched], dtype=float)
        margins = np.asarray([sample[1] for sample in self.margin_samples], dtype=float)
        alerts = np.asarray([sample[2] for sample in self.margin_samples], dtype=float)
        protections = np.asarray([sample[3] for sample in self.margin_samples], dtype=float)
        minimum_margin_index = int(np.argmin(margins)) if len(margins) else None
        minimum_margin_sample = (
            self.margin_samples[minimum_margin_index]
            if minimum_margin_index is not None
            else None
        )
        segment_minimum_margins: list[float | None] = []
        segment_minimum_samples: list[dict[str, float] | None] = []
        if self.margin_samples:
            mission_start_x = float(self._flight_status.get("mission_start_x_m", 0.0))
            mission_goal_x = float(self._flight_status.get("mission_goal_x_m", mission_start_x))
            segment_count = max(int(self._flight_status.get("segments_completed", 0)), 1)
            horizon = self._float("trajectory_distance_m")
            segment_edges = [mission_start_x]
            for _ in range(segment_count):
                segment_edges.append(min(segment_edges[-1] + horizon, mission_goal_x))
            for segment_index in range(segment_count):
                lower = float(segment_edges[segment_index])
                upper = float(segment_edges[segment_index + 1])
                values = [
                    sample
                    for sample in self.margin_samples
                    if lower <= sample[4] <= upper
                ]
                critical = min(values, key=lambda sample: sample[1]) if values else None
                segment_minimum_margins.append(
                    float(critical[1]) if critical is not None else None
                )
                segment_minimum_samples.append(
                    None
                    if critical is None
                    else {
                        "stamp_s": float(critical[0]),
                        "margin_m": float(critical[1]),
                        "alert_limit_m": float(critical[2]),
                        "protection_level_m": float(critical[3]),
                        "estimate_x_m": float(critical[4]),
                        "estimate_y_m": float(critical[5]),
                        "nearest_static_x_m": float(critical[6]),
                        "nearest_static_y_m": float(critical[7]),
                        "nearest_static_z_m": float(critical[8]),
                    }
                )
        truth_start = truth[0]
        truth_final = truth[-1]
        gt_integrity = {"gt_integrity_matched_samples": 0}
        if self.margin_samples:
            directions = np.asarray(
                [
                    (
                        sample[6] - sample[4],
                        sample[7] - sample[5],
                        sample[8] - sample[9],
                    )
                    for sample in self.margin_samples
                ],
                dtype=float,
            )
            gt_integrity = evaluate_ground_truth_integrity(
                matched_stamps,
                errors,
                np.asarray([sample[0] for sample in self.margin_samples], dtype=float),
                alerts,
                protections,
                directions,
                rotation,
            )
        metrics = {
            "matched_samples": len(matched),
            "ate_rms_m": float(np.sqrt(np.mean(norm_errors ** 2))),
            "position_error_max_m": float(np.max(norm_errors)),
            "weak_direction_error_rms_m": (
                float(np.sqrt(np.mean(np.square(weak_errors)))) if weak_errors else None
            ),
            "weak_direction_error_max_m": float(np.max(weak_errors)) if weak_errors else None,
            "actual_minimum_integrity_margin_m": (
                float(np.min(margins)) if len(margins) else None
            ),
            "actual_minimum_integrity_margin_estimate_x_m": (
                float(minimum_margin_sample[4]) if minimum_margin_sample else None
            ),
            "actual_minimum_integrity_margin_estimate_y_m": (
                float(minimum_margin_sample[5]) if minimum_margin_sample else None
            ),
            "actual_minimum_integrity_margin_stamp_s": (
                float(minimum_margin_sample[0]) if minimum_margin_sample else None
            ),
            "actual_minimum_integrity_margin_alert_limit_m": (
                float(minimum_margin_sample[2]) if minimum_margin_sample else None
            ),
            "actual_minimum_integrity_margin_protection_level_m": (
                float(minimum_margin_sample[3]) if minimum_margin_sample else None
            ),
            "actual_minimum_integrity_margin_nearest_static_xyz_m": (
                [
                    float(minimum_margin_sample[6]),
                    float(minimum_margin_sample[7]),
                    float(minimum_margin_sample[8]),
                ]
                if minimum_margin_sample
                else None
            ),
            "actual_segment_minimum_integrity_margins_m": segment_minimum_margins,
            "actual_segment_minimum_integrity_samples": segment_minimum_samples,
            "actual_minimum_alert_limit_m": float(np.min(alerts)) if len(alerts) else None,
            "actual_maximum_protection_level_m": (
                float(np.max(protections)) if len(protections) else None
            ),
            "integrity_margin_samples": len(margins),
            "ground_truth_path_length_m": self._path_length(self.truth),
            "maximum_lateral_excursion_m": float(np.max(np.abs(truth_relative[:, 1]))),
            "maximum_vertical_excursion_m": float(np.max(np.abs(truth_relative[:, 2]))),
            "maximum_simultaneous_lateral_vertical_excursion_m": float(
                np.max(
                    np.minimum(
                        np.abs(truth_relative[:, 1]),
                        np.maximum(truth_relative[:, 2], 0.0),
                    )
                )
            ),
            "mission_time_s": float(self._flight_status.get("elapsed_s", 0.0)),
            "executed_information_gain": self._executed_information_gain(),
            "truth_start_x_m": float(truth_start[0]),
            "truth_final_x_m": float(truth_final[0]),
            "truth_start_z_m": float(truth_start[2]),
            "truth_final_z_m": float(truth_final[2]),
            "forward_progress_m": float(truth_final[0] - truth_start[0]),
        }
        metrics.update(gt_integrity)
        return metrics

    def _finalize(self) -> None:
        metrics = self._metrics()
        decision = self._decision
        status = self._flight_status
        applied_decisions = self._applied_decisions()
        executed_expected = (
            decision.unconstrained_selected_name
            if decision is not None and self.variant == "information_only"
            else decision.selected_name if decision is not None else ""
        )
        checks = {
            "matched_samples": int(metrics.get("matched_samples", 0)) >= 100,
            "integrity_samples": int(metrics.get("integrity_margin_samples", 0)) >= 20,
            "controller_complete": status.get("finished") is True,
            "decision_valid_hard": bool(
                decision is not None
                and len(applied_decisions)
                >= int(self.get_parameter("minimum_decision_batches").value)
                and all(item.valid and item.hard_constraint for item in self._decisions.values())
            ),
            "margin_not_in_utility": bool(
                decision is not None
                and all(not item.margin_in_utility for item in self._decisions.values())
            ),
            "selected_applied": status.get("selected_applied") is True,
            "executed_candidate_matches_decision": status.get("executed_name")
            == executed_expected,
            "rolling_horizon_complete": rolling_horizon_complete(
                status,
                int(self.get_parameter("minimum_decision_batches").value),
                len(applied_decisions),
            ),
            "corridor_goal_reached": bool(
                metrics.get("forward_progress_m") is not None
                and float(metrics["forward_progress_m"])
                >= self._float("mission_distance_m") - self._float("goal_tolerance_m")
            ),
            "finite_core_metrics": all(
                metrics.get(name) is not None and math.isfinite(float(metrics[name]))
                for name in (
                    "ate_rms_m",
                    "weak_direction_error_rms_m",
                    "actual_minimum_integrity_margin_m",
                    "ground_truth_path_length_m",
                    "mission_time_s",
                    "executed_information_gain",
                )
            ),
            "ground_truth_evaluator_only": True,
            "gt_integrity_evaluation_present": int(
                metrics.get("gt_integrity_matched_samples", 0)
            ) >= 20,
        }
        result = {
            "schema_version": 2,
            "gate": "P11_INTEGRITY_EXPLORATION_FLIGHT_ARM",
            "variant": self.variant,
            "status": "PASS" if all(checks.values()) else "FAIL",
            "metrics": metrics,
            "decision": None if decision is None else self._decision_payload(decision),
            "decisions": [
                self._decision_payload(self._decisions[key])
                for key in sorted(self._decisions)
            ],
            "flight_status": status,
            "decision_batch_counts": {
                "total": len(self._decisions),
                "applied": len(applied_decisions),
                "rejected_resamples": len(self._decisions) - len(applied_decisions),
            },
            "checks": checks,
            "calibration_sha256": self._calibration_sha,
            "ground_truth_consumer": "xq_p11_flight_evaluator_only",
            "algorithm_ground_truth_subscribed": False,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        path = Path(str(self.get_parameter("result_file").value))
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
        self._finalized = True
        self.get_logger().info(f"P11 {self.variant} flight arm -> {result['status']}")

    def _timer(self) -> None:
        if (
            not self._finalized
            and self._complete_wall_s is not None
            and time.monotonic() - self._complete_wall_s >= 1.0
        ):
            self._finalize()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P11FlightEvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble may surface an RCLError from WaitSet construction when the
        # launch context is invalidated between two executor iterations.
        # Suppress only that shutdown race; callback failures still propagate.
        if rclpy.ok():
            raise
    finally:
        if not node._finalized and node._complete_wall_s is not None:
            node._finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
