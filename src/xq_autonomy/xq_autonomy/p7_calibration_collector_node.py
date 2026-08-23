"""Collect train/test standardized directional errors for IMPACT P7."""

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
from rclpy.qos import QoSProfile, ReliabilityPolicy
from xq_sim_interfaces.msg import DirectionalIntegrity


def _stamp(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9


def _yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class P7CalibrationCollectorNode(Node):
    DIRECTIONS = ("x", "y", "z", "weak")

    def __init__(self) -> None:
        super().__init__("xq_p7_calibration_collector")
        self.declare_parameter("scenario", "structured_room")
        self.declare_parameter("split", "train")
        self.declare_parameter("trajectory_variant", "train")
        self.declare_parameter("result_file", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("minimum_duration_s", 65.0)
        self.declare_parameter("maximum_match_gap_s", 0.11)
        self.scenario = str(self.get_parameter("scenario").value)
        self.split = str(self.get_parameter("split").value)
        if self.split not in ("train", "test"):
            raise ValueError("split must be train or test")
        self.odom: list[tuple[float, np.ndarray, float]] = []
        self.truth: list[tuple[float, np.ndarray, float]] = []
        self.integrity: list[tuple[float, np.ndarray, np.ndarray]] = []
        self.started: float | None = None
        self.finalized = False
        reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, reliable)
        self.create_subscription(Odometry, "/xq/eval/agent_01/ground_truth", self._truth_cb, reliable)
        self.create_subscription(DirectionalIntegrity, "/integrity/directional", self._integrity_cb, reliable)
        self.create_timer(1.0, self._timer)

    def _ready(self) -> None:
        if self.started is None and self.odom and self.truth and self.integrity:
            self.started = time.monotonic()

    def _odom_cb(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.odom.append((_stamp(message), np.array((p.x, p.y, p.z), dtype=float), _yaw(message)))
        self._ready()

    def _truth_cb(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.truth.append((_stamp(message), np.array((p.x, p.y, p.z), dtype=float), _yaw(message)))
        self._ready()

    def _integrity_cb(self, message: DirectionalIntegrity) -> None:
        covariance = np.asarray(message.integrity_covariance, dtype=float).reshape(3, 3)
        weak = np.asarray(message.weak_direction_map, dtype=float)
        if np.isfinite(covariance).all() and np.isfinite(weak).all():
            self.integrity.append((_stamp(message), covariance, weak))
        self._ready()

    @staticmethod
    def _nearest(samples, stamp: float, maximum_gap: float):
        times = [item[0] for item in samples]
        index = bisect.bisect_left(times, stamp)
        candidates = []
        if index < len(samples):
            candidates.append(samples[index])
        if index > 0:
            candidates.append(samples[index - 1])
        if not candidates:
            return None
        nearest = min(candidates, key=lambda item: abs(item[0] - stamp))
        return nearest if abs(nearest[0] - stamp) <= maximum_gap else None

    @staticmethod
    def _interpolate(samples, stamp: float, maximum_gap: float):
        times = [item[0] for item in samples]
        index = bisect.bisect_left(times, stamp)
        if index == 0 or index >= len(samples):
            return None
        t0, p0, y0 = samples[index - 1]
        t1, p1, y1 = samples[index]
        if t1 <= t0 or stamp - t0 > maximum_gap or t1 - stamp > maximum_gap:
            return None
        ratio = (stamp - t0) / (t1 - t0)
        yaw_delta = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))
        return p0 + ratio * (p1 - p0), y0 + ratio * yaw_delta

    def _samples(self) -> dict[str, dict[str, list[float]]]:
        maximum_gap = float(self.get_parameter("maximum_match_gap_s").value)
        matched = []
        for stamp, covariance, weak in self.integrity:
            odom = self._nearest(self.odom, stamp, maximum_gap)
            truth = self._interpolate(self.truth, stamp, maximum_gap)
            if odom is not None and truth is not None:
                matched.append((stamp, odom[1], odom[2], truth[0], truth[1], covariance, weak))
        if not matched:
            return {name: {"ratio": [], "sigma_m": [], "error_m": []} for name in self.DIRECTIONS}
        _, estimate0, estimate_yaw0, truth0, truth_yaw0, _, _ = matched[0]
        yaw_offset = truth_yaw0 - estimate_yaw0
        c, s = math.cos(yaw_offset), math.sin(yaw_offset)
        rotation = np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
        output = {name: {"ratio": [], "sigma_m": [], "error_m": []} for name in self.DIRECTIONS}
        axes = np.eye(3)
        for _, estimate, _, truth, _, covariance, weak in matched:
            aligned_estimate = rotation @ (estimate - estimate0) + truth0
            error = truth - aligned_estimate
            aligned_covariance = rotation @ covariance @ rotation.T
            directions = list(axes) + [rotation @ (weak / max(np.linalg.norm(weak), 1.0e-12))]
            for name, direction in zip(self.DIRECTIONS, directions):
                sigma = math.sqrt(max(float(direction @ aligned_covariance @ direction), 1.0e-18))
                directional_error = abs(float(direction @ error))
                output[name]["ratio"].append(directional_error / sigma)
                output[name]["sigma_m"].append(sigma)
                output[name]["error_m"].append(directional_error)
        return output

    def _calibration(self) -> tuple[dict | None, str | None]:
        path = Path(str(self.get_parameter("calibration_file").value))
        if self.split != "test":
            return None, None
        if not path.is_file():
            return None, None
        raw = path.read_bytes()
        return json.loads(raw), hashlib.sha256(raw).hexdigest()

    def _finalize(self) -> None:
        samples = self._samples()
        count = min((len(value["ratio"]) for value in samples.values()), default=0)
        calibration, calibration_sha = self._calibration()
        metrics = {}
        if calibration is not None:
            for name, values in samples.items():
                ratios = np.asarray(values["ratio"], dtype=float)
                sigma = np.asarray(values["sigma_m"], dtype=float)
                k95 = float(calibration["directional"][name]["k95"])
                k99 = float(calibration["directional"][name]["k99"])
                metrics[name] = {
                    "count": len(ratios),
                    "coverage_95": float(np.mean(ratios <= k95)),
                    "coverage_99": float(np.mean(ratios <= k99)),
                    "mean_pl95_m": float(np.mean(k95 * sigma)),
                    "missed_integrity_events_95": int(np.count_nonzero(ratios > k95)),
                    "false_alarm_rate": None,
                    "false_alarm_note": "P8 Alert Limit is not defined in P7",
                }
        result = {
            "schema_version": 1,
            "gate": "P7_PROTECTION_LEVEL_CALIBRATION_CAPTURE",
            "status": "PASS" if count >= 100 and (self.split == "train" or calibration is not None) else "FAIL",
            "scenario": self.scenario,
            "split": self.split,
            "trajectory_variant": str(self.get_parameter("trajectory_variant").value),
            "matched_samples": count,
            "calibration_sha256": calibration_sha,
            "metrics": metrics,
            "raw_directional_samples": samples if self.split == "train" else None,
            "ground_truth_consumer": "xq_p7_calibration_collector_only",
            "predictor_ground_truth_subscribed": False,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        path = Path(str(self.get_parameter("result_file").value))
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
        self.finalized = True
        self.get_logger().info(f"P7 {self.split} capture -> {result['status']} ({count} samples)")

    def _timer(self) -> None:
        if self.finalized or self.started is None:
            return
        if time.monotonic() - self.started >= float(self.get_parameter("minimum_duration_s").value):
            self._finalize()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P7CalibrationCollectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.finalized and node.started is not None:
            node._finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

