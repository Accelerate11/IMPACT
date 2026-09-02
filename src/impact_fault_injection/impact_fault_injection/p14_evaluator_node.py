"""Ground-truth-isolated evaluator for P14 matrix and emergency trials."""

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


def _stamp_s(message: Odometry) -> float:
    return float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)


def _is_subsequence(required: list[str], observed: list[str]) -> bool:
    index = 0
    for value in observed:
        if index < len(required) and value == required[index]:
            index += 1
    return index == len(required)


class P14EvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("impact_p14_evaluator")
        self.declare_parameter("trial", "matrix")
        self.declare_parameter("result_file", "")
        self.declare_parameter("thresholds_file", "")
        self.trial = str(self.get_parameter("trial").value)
        self.thresholds = json.loads(
            Path(str(self.get_parameter("thresholds_file").value)).read_text(encoding="utf-8")
        )
        self.odom: list[tuple[float, np.ndarray]] = []
        self.truth: list[tuple[float, np.ndarray]] = []
        self.status_samples: list[dict[str, object]] = []
        self.proxy_status: dict[str, object] = {}
        self.complete_wall_s: float | None = None
        self.finalized = False
        reliable = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1000, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, reliable)
        self.create_subscription(Odometry, "/xq/eval/agent_01/ground_truth", self._truth_cb, reliable)
        self.create_subscription(String, "/impact/p14/safety_status", self._status_cb, latched)
        self.create_subscription(String, "/impact/fault_proxy_status", self._proxy_cb, latched)
        self.create_timer(0.25, self._timer)
        self.get_logger().info(f"P14 {self.trial} evaluator is the only Ground Truth consumer")

    @staticmethod
    def _point(message: Odometry) -> np.ndarray:
        p = message.pose.pose.position
        return np.asarray((p.x, p.y, p.z), dtype=np.float64)

    def _odom_cb(self, message: Odometry) -> None:
        point = self._point(message)
        if np.isfinite(point).all():
            self.odom.append((_stamp_s(message), point))

    def _truth_cb(self, message: Odometry) -> None:
        point = self._point(message)
        if np.isfinite(point).all():
            self.truth.append((_stamp_s(message), point))

    def _status_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.status_samples.append(payload)
        if payload.get("trial_complete") is True and self.complete_wall_s is None:
            self.complete_wall_s = time.monotonic()

    def _proxy_cb(self, message: String) -> None:
        try:
            self.proxy_status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def _matched(self):
        if not self.odom or not self.truth:
            return np.empty((0, 3)), np.empty((0, 3))
        times = [item[0] for item in self.truth]
        pairs = []
        for stamp, point in self.odom:
            index = bisect.bisect_left(times, stamp)
            choices = [i for i in (index - 1, index) if 0 <= i < len(times)]
            if choices:
                nearest = min(choices, key=lambda i: abs(times[i] - stamp))
                if abs(times[nearest] - stamp) <= 0.25:
                    pairs.append((point, self.truth[nearest][1]))
        if not pairs:
            return np.empty((0, 3)), np.empty((0, 3))
        return np.stack([p[0] for p in pairs]), np.stack([p[1] for p in pairs])

    def _matrix_result(self) -> tuple[dict[str, object], dict[str, object]]:
        expected = self.thresholds["expected_matrix_modes"]
        observed_events = {
            item["fault_id"]
            for sample in self.status_samples
            for item in sample.get("observed_fault_events", [])
        }
        modes_by_fault = {
            fault_id: {
                str(sample.get("mode"))
                for sample in self.status_samples
                if fault_id in sample.get("active_fault_ids", [])
            }
            for fault_id in expected
        }
        estimate, truth = self._matched()
        if len(estimate):
            aligned = estimate - estimate[0] + truth[0]
            ate = float(np.sqrt(np.mean(np.linalg.norm(truth - aligned, axis=1) ** 2)))
        else:
            ate = math.inf
        progress = float(self.truth[-1][1][0] - self.truth[0][1][0]) if self.truth else -math.inf
        counters = self.proxy_status.get("counters", {})
        windows = self.proxy_status.get("ground_fault_windows", {})
        ground_window = windows.get("F2_ground_link", {})
        final = self.status_samples[-1] if self.status_samples else {}
        checks = {
            "all_fault_events_observed": len(observed_events) >= int(self.thresholds["minimum_fault_events"])
                and set(expected).issubset(observed_events),
            "fault_to_mode_contract": all(mode in modes_by_fault[fault_id] for fault_id, mode in expected.items()),
            "lidar_dropout_applied": int(counters.get("lidar_dropped", 0)) >= 1,
            "imu_dropout_applied": int(counters.get("imu_dropped", 0)) >= 1,
            "timestamp_jitter_applied": int(counters.get("jittered_messages", 0)) >= 1,
            "odom_delay_applied": int(counters.get("odom_delayed", 0)) >= 1,
            "covariance_inflation_applied": int(counters.get("covariance_inflated", 0)) >= 1,
            "planner_delay_applied": int(final.get("actual_planner_delay_count", 0)) >= 1,
            "cpu_load_and_shedding_applied": int(final.get("cpu_work_cycles", 0)) >= 1
                and any(sample.get("essential_only") is True for sample in self.status_samples),
            "camera_geometry_fallback": any(
                "F1_camera" in sample.get("active_fault_ids", [])
                and sample.get("geometry_only") is True
                and sample.get("mission_continue") is True
                for sample in self.status_samples
            ),
            "ground_loss_seeded_20_percent": int(ground_window.get("sent", 0)) >= 20
                and 0.05 <= float(ground_window.get("loss_ratio", 0.0)) <= 0.40,
            "full_corridor_complete": progress >= float(self.thresholds["minimum_forward_progress_m"]),
            "localization_ate": ate <= float(self.thresholds["maximum_matrix_ate_rms_m"]),
            "controller_complete": final.get("trial_complete") is True and final.get("base_flight_finished") is True,
            "ground_truth_evaluator_only": final.get("ground_truth_subscribed") is False
                and self.proxy_status.get("ground_truth_used") is False,
        }
        metrics = {
            "fault_events_observed": len(observed_events),
            "modes_by_fault": {key: sorted(value) for key, value in modes_by_fault.items()},
            "forward_progress_m": progress,
            "ate_rms_m": ate,
            "proxy_counters": counters,
            "ground_fault_window": ground_window,
        }
        return checks, metrics

    def _emergency_result(self) -> tuple[dict[str, object], dict[str, object]]:
        final = self.status_samples[-1] if self.status_samples else {}
        history = final.get("state_history", [])
        observed_modes = [str(item["mode"]) for item in history]
        required = list(self.thresholds["required_emergency_state_sequence"])
        if len(self.truth) >= 2:
            initial_z = max(point[2] for _, point in self.truth[: max(2, len(self.truth) // 4)])
            final_z = float(self.truth[-1][1][2])
            descent = float(initial_z - final_z)
            recent = self.truth[-min(10, len(self.truth)):]
            if len(recent) >= 2:
                dt = recent[-1][0] - recent[0][0]
                final_speed = float(np.linalg.norm(recent[-1][1] - recent[0][1]) / dt) if dt > 0 else math.inf
            else:
                final_speed = math.inf
        else:
            initial_z = final_z = descent = -math.inf
            final_speed = math.inf
        counters = self.proxy_status.get("counters", {})
        checks = {
            "persistent_lidar_physically_dropped": int(counters.get("lidar_dropped", 0)) >= 10,
            "ordered_fail_safe_sequence": _is_subsequence(required, observed_modes),
            "land_command_completed": final.get("landed") is True and final.get("trial_complete") is True,
            "minimum_descent": descent >= float(self.thresholds["minimum_landing_descent_m"]),
            "stopped_after_landing": final_speed <= float(self.thresholds["maximum_emergency_stop_speed_mps"]),
            "ground_truth_evaluator_only": final.get("ground_truth_subscribed") is False
                and self.proxy_status.get("ground_truth_used") is False,
        }
        metrics = {
            "observed_state_sequence": observed_modes,
            "required_state_sequence": required,
            "initial_altitude_m": initial_z,
            "final_altitude_m": final_z,
            "landing_descent_m": descent,
            "final_speed_mps": final_speed,
            "proxy_counters": counters,
        }
        return checks, metrics

    def _finalize(self) -> None:
        checks, metrics = self._matrix_result() if self.trial == "matrix" else self._emergency_result()
        result = {
            "schema_version": 1,
            "gate": "P14_FAULT_MATRIX_TRIAL",
            "trial": self.trial,
            "status": "PASS" if all(checks.values()) else "FAIL",
            "checks": checks,
            "metrics": metrics,
            "final_status": self.status_samples[-1] if self.status_samples else {},
            "algorithm_ground_truth_subscribed": False,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        path = Path(str(self.get_parameter("result_file").value))
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
        self.finalized = True
        self.get_logger().info(f"P14 {self.trial} -> {result['status']}")

    def _timer(self) -> None:
        if not self.finalized and self.complete_wall_s is not None and time.monotonic() - self.complete_wall_s >= 1.5:
            self._finalize()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P14EvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.finalized and node.complete_wall_s is not None:
            node._finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
