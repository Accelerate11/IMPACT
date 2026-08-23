"""P10 online minimum-excitation candidate prediction and hard selection."""

from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import (
    ActivePerceptionDecision,
    DirectionalIntegrity,
    InformationMap,
)

from .alert_limit import compute_alert_limit, sample_bspline
from .minimum_excitation import (
    CandidateForecast,
    build_information_profile,
    generate_discrete_candidates,
    select_minimum_excitation,
)


def _cloud_xyz(message: PointCloud2) -> np.ndarray:
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=float)
    if any(fields[name].datatype != PointField.FLOAT32 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=float)
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({
        "names": ("x", "y", "z"),
        "formats": (endian + "f4", endian + "f4", endian + "f4"),
        "offsets": tuple(fields[name].offset for name in ("x", "y", "z")),
        "itemsize": message.point_step,
    })
    records = np.frombuffer(message.data, dtype=dtype, count=int(message.width * message.height))
    points = np.column_stack((records["x"], records["y"], records["z"])).astype(float)
    return points[np.isfinite(points).all(axis=1)]


class P10ActivePerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p10_active_perception")
        defaults = {
            "calibration_file": "",
            "calibration_sha256": "",
            "margin_reserve_m": 0.10,
            "trajectory_sample_interval_s": 0.40,
            "lateral_offset_m": 0.40,
            "vertical_offset_m": 0.25,
            "slow_scale": 0.50,
            "hover_time_s": 1.0,
            "visibility_radius_m": 0.55,
            "age_time_constant_s": 10.0,
            "information_scale": 2500.0,
            "minimum_prediction_variance_m2": 1.0e-5,
            "lambda_energy": 0.25,
            "lambda_distance": 1.0,
            "body_radius_m": 0.35,
            "base_reserve_m": 0.10,
            "tracking_reserve_m": 0.10,
            "latency_p99_s": 0.10,
            "maximum_acceleration_mps2": 1.0,
            "maximum_input_age_s": 0.75,
            "include_yaw_only_comparator": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._k_alpha, self._calibration_sha = self._load_calibration()
        self._baseline: Bspline | None = None
        self._cloud: PointCloud2 | None = None
        self._information_map: InformationMap | None = None
        self._integrity: DirectionalIntegrity | None = None
        self._processed_ids: set[int] = set()
        self._last_decision: ActivePerceptionDecision | None = None
        self._last_selected: Bspline | None = None
        self._last_state_publish_wall = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        decision_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.decision_publisher = self.create_publisher(
            ActivePerceptionDecision, "/integrity/active_perception_decision", decision_qos
        )
        self.selected_publisher = self.create_publisher(
            Bspline, "/planning/active_perception_bspline", decision_qos
        )
        self.candidate_publisher = self.create_publisher(
            Bspline, "/planning/active_perception_candidates", 20
        )
        self.debug_publisher = self.create_publisher(
            String, "/integrity/active_perception_debug", 10
        )
        self.create_subscription(Bspline, "/planning/p10/baseline_bspline", self._baseline_cb, qos)
        self.create_subscription(PointCloud2, "/xq/p5/cloud_map", self._cloud_cb, qos)
        self.create_subscription(InformationMap, "/integrity/information_map", self._map_cb, qos)
        self.create_subscription(DirectionalIntegrity, "/integrity/directional", self._integrity_cb, qos)
        self.create_timer(0.10, self._try_process)
        self.get_logger().info(
            "P10 active perception ready: predicted Margin is a hard constraint; no Ground Truth"
        )

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    @staticmethod
    def _stamp_s(message) -> float:
        return float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)

    def _load_calibration(self) -> tuple[float, str]:
        path = Path(str(self.get_parameter("calibration_file").value))
        if not path.is_file():
            raise ValueError(f"P7 calibration file missing: {path}")
        payload = path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        expected = str(self.get_parameter("calibration_sha256").value).strip()
        if expected and digest != expected:
            raise ValueError("P7 calibration SHA-256 mismatch")
        calibration = json.loads(payload)
        if not calibration.get("train_only") or calibration.get("test_data_used", True):
            raise ValueError("P10 only accepts the frozen train-only P7 calibration")
        factors = [float(value["k95"]) for value in calibration.get("directional", {}).values()]
        if not factors or not np.isfinite(factors).all() or min(factors) <= 0.0:
            raise ValueError("P7 calibration has no valid k95 factors")
        return max(factors), digest

    def _baseline_cb(self, message: Bspline) -> None:
        self._baseline = message

    def _cloud_cb(self, message: PointCloud2) -> None:
        self._cloud = message

    def _map_cb(self, message: InformationMap) -> None:
        self._information_map = message

    def _integrity_cb(self, message: DirectionalIntegrity) -> None:
        self._integrity = message

    @staticmethod
    def _candidate_message(candidate, stamp, trajectory_id: int) -> Bspline:
        count = len(candidate.positions)
        duration = float(candidate.duration)
        message = Bspline()
        message.order = 1
        message.traj_id = int(trajectory_id)
        message.start_time = stamp
        message.pos_pts = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in candidate.positions]
        interior = [duration * index / float(count - 1) for index in range(1, count - 1)]
        message.knots = [0.0, 0.0, *interior, duration, duration]
        message.yaw_pts = candidate.yaw.tolist()
        message.yaw_dt = duration / float(max(count - 1, 1))
        return message

    def _publish_invalid(self, baseline: Bspline, reason: str) -> None:
        output = ActivePerceptionDecision()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "xq_lio_map"
        output.baseline_trajectory_id = int(baseline.traj_id)
        output.selected_index = -1
        output.selected_trajectory_id = -1
        output.baseline_insufficient = True
        output.recovery_found = False
        output.valid = False
        output.hard_constraint = True
        output.reason = reason
        self.decision_publisher.publish(output)
        self._last_decision = output
        self._last_selected = None
        self._last_state_publish_wall = time.monotonic()
        self.get_logger().error(f"P10 fail-closed decision: {reason}")

    def _try_process(self) -> None:
        if any(value is None for value in (
            self._baseline, self._cloud, self._information_map, self._integrity
        )):
            return
        baseline = self._baseline
        cloud = self._cloud
        information_map = self._information_map
        integrity = self._integrity
        assert baseline is not None and cloud is not None
        assert information_map is not None and integrity is not None
        baseline_id = int(baseline.traj_id)
        if baseline_id in self._processed_ids:
            if (
                self._last_decision is not None
                and time.monotonic() - self._last_state_publish_wall >= 0.50
            ):
                self.decision_publisher.publish(self._last_decision)
                if self._last_selected is not None:
                    self.selected_publisher.publish(self._last_selected)
                self._last_state_publish_wall = time.monotonic()
            return
        stamps = [self._stamp_s(value) for value in (cloud, information_map, integrity)]
        if max(stamps) - min(stamps) > self._float("maximum_input_age_s"):
            return
        if not information_map.valid or information_map.ground_truth_used:
            self._publish_invalid(baseline, "REJECT_INVALID_OR_GT_INFORMATION_MAP")
            self._processed_ids.add(baseline_id)
            return

        obstacles = _cloud_xyz(cloud)
        positions = np.asarray([(p.x, p.y, p.z) for p in information_map.positions], dtype=float)
        normals = np.asarray([(n.x, n.y, n.z) for n in information_map.normals], dtype=float)
        field_lengths = {
            len(positions), len(normals), len(information_map.static_confidence),
            len(information_map.geometry_quality), len(information_map.last_seen_s),
        }
        if len(obstacles) == 0 or len(field_lengths) != 1 or not positions.size:
            self._publish_invalid(baseline, "REJECT_EMPTY_OR_MISALIGNED_MAP")
            self._processed_ids.add(baseline_id)
            return
        try:
            control_points = np.asarray([(p.x, p.y, p.z) for p in baseline.pos_pts], dtype=float)
            samples = sample_bspline(
                control_points, np.asarray(baseline.knots), int(baseline.order),
                self._float("trajectory_sample_interval_s"),
            )
            duration = float(baseline.knots[-int(baseline.order) - 1] - baseline.knots[int(baseline.order)])
            candidates = generate_discrete_candidates(
                samples,
                baseline_duration=duration,
                lateral_offset=self._float("lateral_offset_m"),
                vertical_offset=self._float("vertical_offset_m"),
                slow_scale=self._float("slow_scale"),
                hover_time=self._float("hover_time_s"),
                include_yaw_only_comparator=bool(
                    self.get_parameter("include_yaw_only_comparator").value
                ),
            )
            forecasts = []
            for candidate in candidates:
                speed = float(np.linalg.norm(np.diff(candidate.positions, axis=0), axis=1).sum()) / candidate.duration
                alert = compute_alert_limit(
                    candidate.positions, obstacles,
                    speed_mps=speed,
                    latency_p99_s=self._float("latency_p99_s"),
                    maximum_acceleration_mps2=self._float("maximum_acceleration_mps2"),
                    body_radius_m=self._float("body_radius_m"),
                    base_reserve_m=self._float("base_reserve_m"),
                    tracking_reserve_m=self._float("tracking_reserve_m"),
                )
                information = build_information_profile(
                    candidate.positions, positions, normals,
                    np.asarray(information_map.static_confidence),
                    np.asarray(information_map.geometry_quality),
                    np.asarray(information_map.last_seen_s),
                    now=max(stamps),
                    visibility_radius=self._float("visibility_radius_m"),
                    age_time_constant=self._float("age_time_constant_s"),
                    information_scale=self._float("information_scale"),
                )
                forecasts.append(CandidateForecast(
                    candidate=candidate,
                    alert_limits=alert.alert_limits,
                    obstacle_directions=alert.obstacle_directions,
                    information_profile=information,
                ))
            selection = select_minimum_excitation(
                forecasts,
                np.asarray(integrity.integrity_covariance, dtype=float).reshape(3, 3),
                k_alpha=self._k_alpha,
                margin_reserve=self._float("margin_reserve_m"),
                lambda_energy=self._float("lambda_energy"),
                lambda_distance=self._float("lambda_distance"),
                minimum_prediction_variance=self._float(
                    "minimum_prediction_variance_m2"
                ),
            )
        except (ValueError, IndexError, np.linalg.LinAlgError) as error:
            self._publish_invalid(baseline, f"REJECT_INVALID_MATH_INPUT:{error}")
            self._processed_ids.add(baseline_id)
            return

        candidate_ids = [baseline_id * 100 + index for index in range(len(candidates))]
        messages = [
            self._candidate_message(candidate, baseline.start_time, trajectory_id)
            for candidate, trajectory_id in zip(candidates, candidate_ids)
        ]
        for message in messages:
            self.candidate_publisher.publish(message)

        output = ActivePerceptionDecision()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "xq_lio_map"
        output.baseline_trajectory_id = baseline_id
        output.candidate_names = [prediction.candidate.name for prediction in selection.predictions]
        output.candidate_trajectory_ids = candidate_ids
        output.predicted_minimum_margins = [prediction.minimum_margin for prediction in selection.predictions]
        output.costs = [prediction.cost for prediction in selection.predictions]
        output.information_traces = [prediction.information_trace for prediction in selection.predictions]
        output.feasible = [prediction.feasible for prediction in selection.predictions]
        output.selected_index = -1
        output.selected_trajectory_id = -1
        output.selected_name = selection.selected_name or ""
        if selection.selected_name is not None:
            output.selected_index = output.candidate_names.index(selection.selected_name)
            output.selected_trajectory_id = candidate_ids[output.selected_index]
        output.baseline_insufficient = selection.baseline_insufficient
        output.recovery_found = selection.recovery_found
        output.valid = True
        output.hard_constraint = True
        output.reason = (
            "BASELINE_ALREADY_FEASIBLE" if not selection.baseline_insufficient
            else "RECOVERY_SELECTED" if selection.recovery_found
            else "REJECT_NO_FEASIBLE_RECOVERY"
        )
        self.decision_publisher.publish(output)
        self._last_decision = output
        if output.selected_index >= 0:
            self._last_selected = messages[output.selected_index]
            self.selected_publisher.publish(self._last_selected)
        self._last_state_publish_wall = time.monotonic()
        debug = String()
        debug.data = json.dumps({
            "phase": "P10_MINIMUM_EXCITATION",
            "baseline_trajectory_id": baseline_id,
            "selected": output.selected_name,
            "reason": output.reason,
            "candidate_names": list(output.candidate_names),
            "predicted_minimum_margins": list(output.predicted_minimum_margins),
            "costs": list(output.costs),
            "feasible": list(output.feasible),
            "hard_constraint": True,
            "maximum_observability_objective": False,
            "minimum_prediction_variance_m2": self._float(
                "minimum_prediction_variance_m2"
            ),
            "ground_truth_subscribed": False,
            "calibration_sha256": self._calibration_sha,
        }, separators=(",", ":"))
        self.debug_publisher.publish(debug)
        self.get_logger().info(
            f"P10 decision baseline={baseline_id} selected={output.selected_name or 'NONE'} "
            f"reason={output.reason}"
        )
        self._processed_ids.add(baseline_id)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P10ActivePerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
