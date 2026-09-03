"""Evaluation-only P13 latency-chain and conservative-speed metrics."""

from __future__ import annotations

import bisect
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .latency_safety import summarize_latencies


def _stamp_s(message: Odometry) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


class P13FlightEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p13_flight_evaluator")
        self.declare_parameter("result_file", "")
        self.declare_parameter("thresholds_file", "")
        self.declare_parameter("expected_planner_delay_ms", 50.0)
        self.declare_parameter("latency_profile", "low_50ms")
        self.thresholds = json.loads(
            Path(str(self.get_parameter("thresholds_file").value)).read_text(encoding="utf-8")
        )
        self.odom: list[tuple[float, np.ndarray]] = []
        self.truth: list[tuple[float, np.ndarray]] = []
        self.traces: dict[int, dict[str, object]] = {}
        self.flight_status: dict[str, object] = {}
        self.complete_wall_s: float | None = None
        self.finalized = False
        reliable = QoSProfile(depth=300, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=600,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, reliable)
        self.create_subscription(
            Odometry, "/xq/eval/agent_01/ground_truth", self._truth_cb, reliable
        )
        self.create_subscription(
            String, "/integrity/p13/latency_trace", self._trace_cb, latched
        )
        self.create_subscription(String, "/xq/p13/flight_status", self._status_cb, latched)
        self.create_timer(0.25, self._timer)
        self.get_logger().info("P13 evaluator is the only P13 consumer of Ground Truth")

    @staticmethod
    def _position(message: Odometry) -> np.ndarray:
        point = message.pose.pose.position
        return np.asarray((point.x, point.y, point.z), dtype=np.float64)

    def _odom_cb(self, message: Odometry) -> None:
        point = self._position(message)
        if np.isfinite(point).all():
            self.odom.append((_stamp_s(message), point))

    def _truth_cb(self, message: Odometry) -> None:
        point = self._position(message)
        if np.isfinite(point).all():
            self.truth.append((_stamp_s(message), point))

    def _trace_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            seq = int(payload["seq"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        self.traces[seq] = payload

    def _status_cb(self, message: String) -> None:
        try:
            self.flight_status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if self.flight_status.get("finished") is True and self.complete_wall_s is None:
            self.complete_wall_s = time.monotonic()

    @staticmethod
    def _matched(estimate, truth):
        if not estimate or not truth:
            return np.empty((0, 3)), np.empty((0, 3))
        truth_times = [item[0] for item in truth]
        pairs = []
        for stamp_s, point in estimate:
            index = bisect.bisect_left(truth_times, stamp_s)
            choices = [value for value in (index - 1, index) if 0 <= value < len(truth)]
            if not choices:
                continue
            nearest = min(choices, key=lambda value: abs(truth[value][0] - stamp_s))
            if abs(truth[nearest][0] - stamp_s) <= 0.11:
                pairs.append((point, truth[nearest][1]))
        if not pairs:
            return np.empty((0, 3)), np.empty((0, 3))
        return np.stack([item[0] for item in pairs]), np.stack([item[1] for item in pairs])

    def _finalize(self) -> None:
        ordered = [self.traces[key] for key in sorted(self.traces)]
        end_to_end = [1.0e-3 * float(item["end_to_end_latency_ms"]) for item in ordered]
        planner = [1.0e-3 * float(item["planner_processing_ms"]) for item in ordered]
        stats = summarize_latencies(end_to_end)
        planner_stats = summarize_latencies(planner)
        estimate, truth = self._matched(self.odom, self.truth)
        if len(estimate):
            aligned = estimate - estimate[0] + truth[0]
            errors = np.linalg.norm(truth - aligned, axis=1)
            ate = float(np.sqrt(np.mean(errors**2)))
            progress = float(truth[-1, 0] - truth[0, 0])
        else:
            ate, progress = math.inf, -math.inf
        speed_limits = [float(item["speed_limit_mps"]) for item in ordered]
        alert_limits = [float(item["alert_limit_m"]) for item in ordered]
        margins = [float(item["integrity_margin_m"]) for item in ordered]
        ordered_stages = all(
            int(item["receive_timestamp_ns"])
            <= int(item["localization_done_ns"])
            <= int(item["map_done_ns"])
            <= int(item["planner_trigger_ns"])
            <= int(item["planner_done_ns"])
            <= int(item["trajectory_certified_ns"])
            <= int(item["command_sent_ns"])
            for item in ordered
        )
        ground_truth_clean = all(item.get("ground_truth_used") is False for item in ordered)
        expected_ms = float(self.get_parameter("expected_planner_delay_ms").value)
        planner_p50_ms = 1000.0 * planner_stats.p50_s
        minimum_samples = int(self.thresholds["minimum_latency_samples"])
        checks = {
            "latency_sample_count": stats.count >= minimum_samples,
            "planner_delay_applied": abs(planner_p50_ms - expected_ms)
            <= float(self.thresholds["maximum_planner_delay_error_ms"]),
            "all_stage_timestamps_ordered": ordered_stages,
            "nearest_rank_p99_used": bool(
                ordered and ordered[-1].get("p99_used_for_safety") is True
            ),
            "latency_margin_respected": bool(
                margins and min(margins[-minimum_samples:])
                >= float(self.thresholds["required_margin_m"]) - 0.005
            ),
            "positive_speed_envelope": bool(speed_limits and min(speed_limits) > 0.0),
            "full_corridor_complete": progress
            >= float(self.thresholds["minimum_forward_progress_m"]),
            "localization_ate": ate <= float(self.thresholds["maximum_ate_rms_m"]),
            "controller_complete": self.flight_status.get("finished") is True,
            "ground_truth_evaluator_only": ground_truth_clean
            and self.flight_status.get("ground_truth_subscribed") is False,
        }
        result = {
            "schema_version": 1,
            "gate": "P13_LATENCY_TRIAL",
            "profile": str(self.get_parameter("latency_profile").value),
            "status": "PASS" if all(checks.values()) else "FAIL",
            "metrics": {
                "matched_samples": len(estimate),
                "ate_rms_m": ate,
                "forward_progress_m": progress,
                "latency_sample_count": stats.count,
                "end_to_end_p50_ms": 1000.0 * stats.p50_s,
                "end_to_end_p95_ms": 1000.0 * stats.p95_s,
                "end_to_end_p99_ms": 1000.0 * stats.p99_s,
                "end_to_end_max_ms": 1000.0 * stats.maximum_s,
                "planner_processing_p50_ms": planner_p50_ms,
                "planner_processing_p95_ms": 1000.0 * planner_stats.p95_s,
                "planner_processing_p99_ms": 1000.0 * planner_stats.p99_s,
                "planner_processing_max_ms": 1000.0 * planner_stats.maximum_s,
                "final_speed_limit_mps": speed_limits[-1] if speed_limits else 0.0,
                "minimum_speed_limit_mps": min(speed_limits) if speed_limits else 0.0,
                "final_alert_limit_m": alert_limits[-1] if alert_limits else -math.inf,
                "minimum_alert_limit_m": min(alert_limits) if alert_limits else -math.inf,
                "final_integrity_margin_m": margins[-1] if margins else -math.inf,
                "final_unmitigated_alert_limit_m": (
                    float(ordered[-1]["unmitigated_alert_limit_m"])
                    if ordered else -math.inf
                ),
                "final_unmitigated_integrity_margin_m": (
                    float(ordered[-1]["unmitigated_integrity_margin_m"])
                    if ordered else -math.inf
                ),
            },
            "checks": checks,
            "flight_status": self.flight_status,
            "ground_truth_consumer": "xq_p13_flight_evaluator_only",
            "algorithm_ground_truth_subscribed": False,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        path = Path(str(self.get_parameter("result_file").value))
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
        self.finalized = True
        self.get_logger().info(f"P13 {result['profile']} -> {result['status']}")

    def _timer(self) -> None:
        if (
            not self.finalized
            and self.complete_wall_s is not None
            and time.monotonic() - self.complete_wall_s >= 1.0
        ):
            self._finalize()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P13FlightEvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if not node.finalized and node.complete_wall_s is not None:
            node._finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
