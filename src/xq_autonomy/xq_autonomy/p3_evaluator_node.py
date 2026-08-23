"""Ground-truth-isolated FAST-LIO2 evaluation for IMPACT P3."""

from __future__ import annotations

import bisect
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


def _stamp_s(message: Odometry) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class P3EvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p3_evaluator")
        self.declare_parameter("scenario", "structured_room")
        self.declare_parameter("gate_name", "P3_FAST_LIO2_BASELINE")
        self.declare_parameter("ground_truth_consumer", "xq_p3_evaluator_only")
        self.declare_parameter("result_file", "")
        self.declare_parameter("minimum_duration_s", 65.0)
        self.declare_parameter("minimum_frequency_hz", 10.0)
        self.declare_parameter("maximum_gap_s", 0.25)
        self.declare_parameter("structured_room_ate_limit_m", 0.30)
        self.declare_parameter("odom_topic", "/localization/odom")
        self.declare_parameter(
            "ground_truth_topic", "/xq/eval/agent_01/ground_truth"
        )
        self.scenario = str(self.get_parameter("scenario").value)
        self.odom: list[tuple[float, np.ndarray, float, float]] = []
        self.ground_truth: list[tuple[float, np.ndarray, float]] = []
        self.odom_monotonic_violations = 0
        self.nonfinite_violations = 0
        self.capture_started_monotonic: float | None = None
        self.last_status = "IN_PROGRESS"

        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, reliable
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("ground_truth_topic").value),
            self._ground_truth_cb,
            reliable,
        )
        self.create_timer(1.0, self._timer_cb)
        self.get_logger().info(
            f"P3 evaluator started: scenario={self.scenario}; ground truth is evaluation-only"
        )

    def _start_if_ready(self) -> None:
        if self.capture_started_monotonic is None and self.odom and self.ground_truth:
            self.capture_started_monotonic = time.monotonic()
            self.get_logger().info("P3 evaluation window started")

    def _odom_cb(self, message: Odometry) -> None:
        stamp = _stamp_s(message)
        position = message.pose.pose.position
        sample = np.array((position.x, position.y, position.z), dtype=np.float64)
        yaw = _yaw(message)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        latency = now_s - stamp
        if self.odom and stamp <= self.odom[-1][0]:
            self.odom_monotonic_violations += 1
        if not np.isfinite(sample).all() or not math.isfinite(yaw):
            self.nonfinite_violations += 1
        self.odom.append((stamp, sample, yaw, latency))
        self._start_if_ready()

    def _ground_truth_cb(self, message: Odometry) -> None:
        stamp = _stamp_s(message)
        position = message.pose.pose.position
        sample = np.array((position.x, position.y, position.z), dtype=np.float64)
        yaw = _yaw(message)
        if np.isfinite(sample).all() and math.isfinite(yaw):
            self.ground_truth.append((stamp, sample, yaw))
        self._start_if_ready()

    def _matched(self) -> list[tuple[float, np.ndarray, float, np.ndarray, float, float]]:
        if len(self.ground_truth) < 2:
            return []
        gt_times = [sample[0] for sample in self.ground_truth]
        matched = []
        for stamp, odom_position, odom_yaw, latency in self.odom:
            index = bisect.bisect_left(gt_times, stamp)
            if index == 0 or index >= len(gt_times):
                continue
            t0, p0, y0 = self.ground_truth[index - 1]
            t1, p1, y1 = self.ground_truth[index]
            if t1 <= t0 or stamp - t0 > 0.10 or t1 - stamp > 0.10:
                continue
            ratio = (stamp - t0) / (t1 - t0)
            gt_position = p0 + ratio * (p1 - p0)
            gt_yaw = _wrap(y0 + ratio * _wrap(y1 - y0))
            matched.append((stamp, odom_position, odom_yaw, gt_position, gt_yaw, latency))
        return matched

    def _metrics(self) -> dict[str, object]:
        matched = self._matched()
        result: dict[str, object] = {
            "matched_samples": len(matched),
            "ate_rms_m": None,
            "position_error_mean_m": None,
            "position_error_max_m": None,
            "position_error_final_m": None,
            "yaw_error_rms_deg": None,
            "yaw_error_max_deg": None,
            "rpe_translation_rms_m_1s": None,
            "rpe_yaw_rms_deg_1s": None,
            "processing_latency_mean_s": None,
            "processing_latency_max_s": None,
            "largest_position_step_m": None,
            "largest_speed_mps": None,
            "first_error_over_0_30_s": None,
            "error_checkpoints": [],
        }
        if not matched:
            return result

        _, lio_p0, lio_y0, gt_p0, gt_y0, _ = matched[0]
        yaw_offset = _wrap(gt_y0 - lio_y0)
        c, s = math.cos(yaw_offset), math.sin(yaw_offset)
        rotation = np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
        stamps = np.array([sample[0] for sample in matched])
        aligned = np.stack([rotation @ (sample[1] - lio_p0) + gt_p0 for sample in matched])
        truth = np.stack([sample[3] for sample in matched])
        errors = np.linalg.norm(aligned - truth, axis=1)
        yaw_errors = np.array(
            [_wrap(sample[2] + yaw_offset - sample[4]) for sample in matched]
        )
        latencies = np.array([sample[5] for sample in matched])
        steps = np.linalg.norm(np.diff(aligned, axis=0), axis=1)
        dt = np.diff(stamps)
        speeds = steps / np.maximum(dt, 1e-9)
        relative_stamps = stamps - stamps[0]
        above_limit = np.flatnonzero(errors > 0.30)
        checkpoints = []
        for target_s in np.arange(0.0, relative_stamps[-1] + 1e-9, 1.0):
            index = min(int(np.searchsorted(relative_stamps, target_s)), len(errors) - 1)
            checkpoints.append(
                {
                    "elapsed_s": float(relative_stamps[index]),
                    "position_error_m": float(errors[index]),
                    "yaw_error_deg": float(np.degrees(yaw_errors[index])),
                    "lio_xyz_m": [float(value) for value in aligned[index]],
                    "truth_xyz_m": [float(value) for value in truth[index]],
                }
            )

        rpe_translation = []
        rpe_yaw = []
        for index, stamp in enumerate(stamps):
            other = int(np.searchsorted(stamps, stamp + 1.0))
            if other >= len(stamps):
                break
            rpe_translation.append(
                np.linalg.norm((aligned[other] - aligned[index]) - (truth[other] - truth[index]))
            )
            lio_delta = _wrap(matched[other][2] - matched[index][2])
            gt_delta = _wrap(matched[other][4] - matched[index][4])
            rpe_yaw.append(_wrap(lio_delta - gt_delta))

        result.update(
            {
                "ate_rms_m": float(np.sqrt(np.mean(errors * errors))),
                "position_error_mean_m": float(np.mean(errors)),
                "position_error_max_m": float(np.max(errors)),
                "position_error_final_m": float(errors[-1]),
                "yaw_error_rms_deg": float(np.degrees(np.sqrt(np.mean(yaw_errors * yaw_errors)))),
                "yaw_error_max_deg": float(np.degrees(np.max(np.abs(yaw_errors)))),
                "rpe_translation_rms_m_1s": (
                    float(np.sqrt(np.mean(np.square(rpe_translation)))) if rpe_translation else None
                ),
                "rpe_yaw_rms_deg_1s": (
                    float(np.degrees(np.sqrt(np.mean(np.square(rpe_yaw))))) if rpe_yaw else None
                ),
                "processing_latency_mean_s": float(np.mean(latencies)),
                "processing_latency_max_s": float(np.max(latencies)),
                "largest_position_step_m": float(np.max(steps)) if len(steps) else 0.0,
                "largest_speed_mps": float(np.max(speeds)) if len(speeds) else 0.0,
                "first_error_over_0_30_s": (
                    float(relative_stamps[int(above_limit[0])]) if len(above_limit) else None
                ),
                "error_checkpoints": checkpoints,
            }
        )
        return result

    def _snapshot(self) -> dict[str, object]:
        elapsed = (
            time.monotonic() - self.capture_started_monotonic
            if self.capture_started_monotonic is not None
            else 0.0
        )
        stamps = np.array([sample[0] for sample in self.odom], dtype=np.float64)
        span = float(stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
        frequency = float((len(stamps) - 1) / span) if span > 0.0 else 0.0
        gaps = np.diff(stamps)
        max_gap = float(np.max(gaps)) if len(gaps) else None
        metrics = self._metrics()
        common_checks = {
            "duration": elapsed >= float(self.get_parameter("minimum_duration_s").value),
            # A nominal 10 Hz stream can become 9.999999999999998 after binary
            # timestamp arithmetic; this epsilon is physically negligible.
            "frequency": frequency + 1e-9 >= float(
                self.get_parameter("minimum_frequency_hz").value
            ),
            "continuous": max_gap is not None
            and max_gap <= float(self.get_parameter("maximum_gap_s").value),
            "monotonic": self.odom_monotonic_violations == 0,
            "finite": self.nonfinite_violations == 0,
            "matched_samples": int(metrics["matched_samples"]) >= 100,
            "no_obvious_jump": metrics["largest_position_step_m"] is not None
            and float(metrics["largest_position_step_m"]) <= 0.75
            and float(metrics["largest_speed_mps"]) <= 5.0,
        }
        ate = metrics["ate_rms_m"]
        common_checks["structured_room_ate"] = (
            self.scenario != "structured_room"
            or (ate is not None and float(ate) <= float(
                self.get_parameter("structured_room_ate_limit_m").value
            ))
        )
        irreversible = self.odom_monotonic_violations > 0 or self.nonfinite_violations > 0
        if irreversible:
            status = "FAIL"
        elif common_checks["duration"]:
            status = "PASS" if all(common_checks.values()) else "FAIL"
        else:
            status = "IN_PROGRESS"
        return {
            "schema_version": 1,
            "gate": str(self.get_parameter("gate_name").value),
            "scenario": self.scenario,
            "status": status,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "capture_wall_duration_s": elapsed,
            "odom": {
                "topic": str(self.get_parameter("odom_topic").value),
                "count": len(self.odom),
                "stamp_span_s": span,
                "frequency_hz": frequency,
                "max_gap_s": max_gap,
                "monotonic_violations": self.odom_monotonic_violations,
            },
            "ground_truth": {
                "topic": str(self.get_parameter("ground_truth_topic").value),
                "count": len(self.ground_truth),
                "consumer": str(self.get_parameter("ground_truth_consumer").value),
            },
            "metrics": metrics,
            "checks": common_checks,
            "nonfinite_violations": self.nonfinite_violations,
        }

    def _write(self) -> None:
        path = Path(str(self.get_parameter("result_file").value))
        if str(path) in ("", "."):
            return
        result = self._snapshot()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
        if result["status"] != self.last_status:
            self.get_logger().info(f"P3 evaluation status -> {result['status']}")
            self.last_status = str(result["status"])

    def _timer_cb(self) -> None:
        self._write()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P3EvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
