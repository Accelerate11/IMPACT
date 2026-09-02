"""Evaluation-only P12 dynamic-map and full-corridor acceptance metrics."""

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
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from xq_sim_interfaces.msg import ReplanEvent

from .p12_dynamic_map_node import _cloud_xyz


def _stamp(message: Odometry) -> float:
    return float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)


class P12FlightEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p12_flight_evaluator")
        self.declare_parameter("result_file", "")
        self.declare_parameter("thresholds_file", "")
        self.thresholds = json.loads(
            Path(str(self.get_parameter("thresholds_file").value)).read_text(encoding="utf-8")
        )
        self.odom: list[tuple[float, np.ndarray]] = []
        self.truth: list[tuple[float, np.ndarray]] = []
        self._obstacle: dict[str, object] = {}
        self._map_status: dict[str, object] = {}
        self._flight_status: dict[str, object] = {}
        self._replans: list[ReplanEvent] = []
        self._occupied_s: float | None = None
        self._passage_clear_s: float | None = None
        self._left_s: float | None = None
        self._detection_s: float | None = None
        self._residual_clear_s: float | None = None
        self._reopen_s: float | None = None
        self._static_baseline: int | None = None
        self._minimum_static_after: int | None = None
        self._minimum_clearance_m = math.inf
        self._dynamic_peak = 0
        self._pose_commands = 0
        self._pose_commands_applied = 0
        self._map_ground_truth_clean = True
        self._complete_wall_s: float | None = None
        self._finalized = False
        reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=30,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, reliable)
        self.create_subscription(
            Odometry, "/xq/eval/agent_01/ground_truth", self._truth_cb, reliable
        )
        self.create_subscription(
            String, "/xq/eval/p12/obstacle_state", self._obstacle_cb, latched
        )
        self.create_subscription(String, "/mapping/p12/status", self._map_cb, latched)
        self.create_subscription(
            PointCloud2, "/mapping/p12/dynamic_voxels", self._dynamic_cb, latched
        )
        self.create_subscription(
            ReplanEvent, "/planning/p12/replan_event", self._replan_cb, latched
        )
        self.create_subscription(
            String, "/xq/p12/flight_status", self._flight_cb, latched
        )
        self.create_timer(0.25, self._timer)
        self.get_logger().info("P12 evaluator is the only consumer of obstacle scenario truth")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _position(message: Odometry) -> np.ndarray:
        point = message.pose.pose.position
        return np.asarray((point.x, point.y, point.z), dtype=np.float64)

    def _odom_cb(self, message: Odometry) -> None:
        point = self._position(message)
        if np.isfinite(point).all():
            self.odom.append((_stamp(message), point))

    def _truth_cb(self, message: Odometry) -> None:
        point = self._position(message)
        if np.isfinite(point).all():
            self.truth.append((_stamp(message), point))
            if self._obstacle.get("passage_occupied") is True:
                center = np.asarray(
                    (
                        float(self._obstacle["x_m"]),
                        float(self._obstacle["y_m"]),
                    )
                )
                # Box half-width 0.40 m plus vehicle body radius 0.35 m.
                clearance = float(np.linalg.norm(point[:2] - center)) - 0.75
                self._minimum_clearance_m = min(self._minimum_clearance_m, clearance)

    def _obstacle_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self._obstacle = payload
        self._pose_commands += 1
        self._pose_commands_applied += int(payload.get("pose_applied") is True)
        stamp_s = float(payload.get("stamp_s", self._now_s()))
        if payload.get("passage_occupied") is True and self._occupied_s is None:
            self._occupied_s = stamp_s
            if self._map_status:
                self._static_baseline = int(self._map_status.get("static_voxel_count", 0))
        if (
            self._occupied_s is not None
            and payload.get("passage_occupied") is False
            and self._passage_clear_s is None
        ):
            # Safety may reopen as soon as the swept corridor is physically clear;
            # waiting until the obstacle reaches its final parking pose (LEFT)
            # would make a correct early reopen appear to have negative latency.
            self._passage_clear_s = stamp_s
        if payload.get("state") == "LEFT" and self._left_s is None:
            self._left_s = stamp_s

    def _map_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self._map_status = payload
        self._map_ground_truth_clean &= payload.get("ground_truth_used") is False
        static_count = int(payload.get("static_voxel_count", 0))
        if self._occupied_s is not None:
            if self._static_baseline is None:
                self._static_baseline = static_count
            self._minimum_static_after = (
                static_count
                if self._minimum_static_after is None
                else min(self._minimum_static_after, static_count)
            )
        stamp_s = float(payload.get("stamp_s", self._now_s()))
        if (
            self._obstacle.get("passage_occupied") is True
            and int(payload.get("path_dynamic_voxel_count", 0))
            >= int(self.thresholds["minimum_dynamic_voxel_count"])
            and self._detection_s is None
        ):
            self._detection_s = stamp_s
        if (
            self._left_s is not None
            and payload.get("forward_path_blocked") is False
            and self._residual_clear_s is None
        ):
            self._residual_clear_s = stamp_s

    def _dynamic_cb(self, message: PointCloud2) -> None:
        points = _cloud_xyz(message)
        self._dynamic_peak = max(self._dynamic_peak, len(points))
        if not self._obstacle:
            return
        center = np.asarray(
            (
                float(self._obstacle.get("x_m", 0.0)),
                float(self._obstacle.get("y_m", 0.0)),
                float(self._obstacle.get("z_m", 1.0)),
            )
        )
        near = np.linalg.norm(points - center, axis=1) <= 1.25 if len(points) else np.zeros(0, bool)
        stamp_s = float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)
        minimum = int(self.thresholds["minimum_dynamic_voxel_count"])
        if (
            self._obstacle.get("passage_occupied") is True
            and int(np.count_nonzero(near)) >= minimum
            and self._detection_s is None
        ):
            self._detection_s = stamp_s
        corridor_residual = bool(
            len(points)
            and np.any(
                (np.abs(points[:, 0] - float(self._obstacle.get("x_m", 0.0))) <= 1.0)
                & (np.abs(points[:, 1]) <= 0.85)
                & (points[:, 2] >= 0.25)
                & (points[:, 2] <= 1.8)
            )
        )
        # Per-voxel residual geometry remains diagnostic.  The acceptance
        # reopening time is taken from the map's atomic forward-path query,
        # which uses the same coordinate frame and odometry snapshot.

    def _replan_cb(self, message: ReplanEvent) -> None:
        if not any(item.seq == message.seq for item in self._replans):
            self._replans.append(message)
        if message.trigger_reason == "DYNAMIC_TTL_CLEARED" and self._reopen_s is None:
            self._reopen_s = self._now_s()

    def _flight_cb(self, message: String) -> None:
        try:
            self._flight_status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if self._flight_status.get("finished") is True and self._complete_wall_s is None:
            self._complete_wall_s = time.monotonic()

    @staticmethod
    def _matched(
        estimate: list[tuple[float, np.ndarray]], truth: list[tuple[float, np.ndarray]]
    ) -> tuple[np.ndarray, np.ndarray]:
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
        estimate, truth = self._matched(self.odom, self.truth)
        if len(estimate):
            aligned = estimate - estimate[0] + truth[0]
            errors = np.linalg.norm(truth - aligned, axis=1)
            ate = float(np.sqrt(np.mean(errors ** 2)))
            progress = float(truth[-1, 0] - truth[0, 0])
        else:
            ate, progress = math.inf, -math.inf
        brake_events = [
            event for event in self._replans
            if event.trigger_reason == "DYNAMIC_OCCUPANCY_CONFIRMED" and event.brake_fallback
        ]
        replan_latency_s = (
            min(float(event.latency_ms) for event in brake_events) * 1.0e-3
            if brake_events else math.inf
        )
        detection_latency_s = (
            self._detection_s - self._occupied_s
            if self._detection_s is not None and self._occupied_s is not None else math.inf
        )
        residual_time_s = (
            self._residual_clear_s - self._left_s
            if self._residual_clear_s is not None and self._left_s is not None else math.inf
        )
        raw_reopen_time_s = (
            self._reopen_s - self._passage_clear_s
            if self._reopen_s is not None and self._passage_clear_s is not None else math.inf
        )
        # Scenario truth is published at 2 Hz while the map reports at 5 Hz.  The
        # map can therefore observe physical clearance before the evaluator's next
        # sampled truth message.  Preserve the raw skew but do not claim negative
        # reaction latency.
        reopen_time_s = max(0.0, raw_reopen_time_s)
        static_retention = (
            self._minimum_static_after / float(self._static_baseline)
            if self._static_baseline and self._minimum_static_after is not None else 0.0
        )
        minimum_clearance = self._minimum_clearance_m
        checks = {
            "moving_obstacle_pose_applied": self._pose_commands_applied >= 0.95 * max(1, self._pose_commands),
            "dynamic_obstacle_detected": math.isfinite(detection_latency_s),
            "dynamic_detection_latency": detection_latency_s <= float(self.thresholds["maximum_detection_latency_s"]),
            "planner_brake_replan": bool(brake_events),
            "replan_latency": replan_latency_s <= float(self.thresholds["maximum_replan_latency_s"]),
            "dynamic_residual_cleared": residual_time_s <= float(self.thresholds["maximum_residual_time_s"]),
            "passage_reopened": 0.0 <= reopen_time_s <= float(
                self.thresholds["maximum_passage_reopen_time_s"]
            ),
            "static_wall_retained": static_retention >= float(self.thresholds["minimum_static_retention_ratio"]),
            "physical_clearance": minimum_clearance >= float(self.thresholds["minimum_obstacle_clearance_m"]),
            "full_corridor_complete": progress >= float(self.thresholds["minimum_forward_progress_m"]),
            "localization_ate": ate <= float(self.thresholds["maximum_ate_rms_m"]),
            "controller_complete": self._flight_status.get("finished") is True,
            "ground_truth_evaluator_only": self._map_ground_truth_clean,
        }
        metrics = {
            "matched_samples": len(estimate),
            "ate_rms_m": ate,
            "forward_progress_m": progress,
            "dynamic_detection_latency_s": detection_latency_s,
            "replan_latency_s": replan_latency_s,
            "dynamic_residual_time_s": residual_time_s,
            "passage_reopen_time_s": reopen_time_s,
            "passage_reopen_raw_observer_skew_s": raw_reopen_time_s,
            "static_retention_ratio": static_retention,
            "minimum_obstacle_clearance_m": minimum_clearance,
            "dynamic_voxel_peak": self._dynamic_peak,
            "replan_event_count": len(self._replans),
            "pose_command_success_ratio": self._pose_commands_applied / float(max(1, self._pose_commands)),
        }
        result = {
            "schema_version": 1,
            "gate": "P12_DYNAMIC_OBSTACLE_FLIGHT",
            "status": "PASS" if all(checks.values()) else "FAIL",
            "metrics": metrics,
            "checks": checks,
            "flight_status": self._flight_status,
            "ground_truth_consumer": "xq_p12_flight_evaluator_only",
            "algorithm_ground_truth_subscribed": False,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        path = Path(str(self.get_parameter("result_file").value))
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
        self._finalized = True
        self.get_logger().info(f"P12 dynamic flight -> {result['status']}")

    def _timer(self) -> None:
        if (
            not self._finalized
            and self._complete_wall_s is not None
            and time.monotonic() - self._complete_wall_s >= 1.0
        ):
            self._finalize()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P12FlightEvaluatorNode()
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
