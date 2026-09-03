"""P11 ROS wrapper: hard-certify Frontier trajectories before utility ranking."""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import (
    DirectionalIntegrity,
    ExplorationCandidateSet,
    InformationMap,
    IntegrityExplorationDecision,
)

from .alert_limit import compute_alert_limit, sample_bspline
from .candidate_metrics import compute_task_gains, pointwise_collision_probability
from .integrity_exploration import (
    ExplorationForecast,
    select_integrity_constrained_exploration,
)
from .minimum_excitation import CandidateForecast, RecoveryCandidate, build_information_profile
from .p10_active_perception_node import _cloud_xyz


class P11IntegrityExplorationNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p11_integrity_exploration")
        defaults = {
            "calibration_file": "",
            "calibration_sha256": "",
            "margin_reserve_m": 0.10,
            "collision_probability_limit": 0.01,
            "energy_remaining": 20.0,
            "information_weight": 1.0,
            "travel_time_weight": 0.01,
            "energy_weight": 0.005,
            "trajectory_sample_interval_s": 0.40,
            "visibility_radius_m": 2.8,
            "age_time_constant_s": 10.0,
            "information_scale": 2500.0,
            "minimum_prediction_variance_m2": 1.0e-5,
            "body_radius_m": 0.35,
            "base_reserve_m": 0.10,
            "tracking_reserve_m": 0.10,
            "latency_p99_s": 0.10,
            "maximum_acceleration_mps2": 1.0,
            "maximum_input_age_s": 0.75,
            "task_progress_weight": 0.85,
            "task_map_age_time_constant_s": 20.0,
            "collision_tracking_sigma_multiplier": 3.0,
            "utility_indifference_band": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._k_alpha, self._calibration_sha = self._load_calibration()
        self._metadata: ExplorationCandidateSet | None = None
        self._candidate_messages: dict[int, Bspline] = {}
        self._cloud: PointCloud2 | None = None
        self._information_map: InformationMap | None = None
        self._integrity: DirectionalIntegrity | None = None
        self._processed_batches: set[int] = set()
        self._last_decision: IntegrityExplorationDecision | None = None
        self._last_selected: Bspline | None = None
        self._last_unconstrained: Bspline | None = None
        self._last_publish_wall = 0.0

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.decision_publisher = self.create_publisher(
            IntegrityExplorationDecision,
            "/integrity/exploration_decision",
            latched_qos,
        )
        self.selected_publisher = self.create_publisher(
            Bspline, "/planning/p11/selected_bspline", latched_qos
        )
        self.unconstrained_publisher = self.create_publisher(
            Bspline, "/planning/p11/unconstrained_bspline", latched_qos
        )
        self.debug_publisher = self.create_publisher(
            String, "/integrity/exploration_debug", 10
        )
        self.create_subscription(
            ExplorationCandidateSet,
            "/planning/p11/frontier_candidate_set",
            self._metadata_cb,
            latched_qos,
        )
        self.create_subscription(
            Bspline,
            "/planning/p11/frontier_candidates",
            self._candidate_cb,
            state_qos,
        )
        self.create_subscription(PointCloud2, "/xq/p5/cloud_map", self._cloud_cb, state_qos)
        self.create_subscription(
            InformationMap, "/integrity/information_map", self._map_cb, state_qos
        )
        self.create_subscription(
            DirectionalIntegrity, "/integrity/directional", self._integrity_cb, state_qos
        )
        self.create_timer(0.10, self._try_process)
        self.get_logger().info(
            "P11 ready: Margin/collision/return-energy hard filter precedes task utility; no GT"
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
            raise ValueError("P11 only accepts frozen train-only P7 calibration")
        factors = [float(value["k95"]) for value in calibration.get("directional", {}).values()]
        if not factors or not np.isfinite(factors).all() or min(factors) <= 0.0:
            raise ValueError("P7 calibration has no valid k95 factors")
        return max(factors), digest

    def _metadata_cb(self, message: ExplorationCandidateSet) -> None:
        self._metadata = message

    def _candidate_cb(self, message: Bspline) -> None:
        self._candidate_messages[int(message.traj_id)] = message

    def _cloud_cb(self, message: PointCloud2) -> None:
        self._cloud = message

    def _map_cb(self, message: InformationMap) -> None:
        self._information_map = message

    def _integrity_cb(self, message: DirectionalIntegrity) -> None:
        self._integrity = message

    def _publish_invalid(self, metadata: ExplorationCandidateSet, reason: str) -> None:
        output = IntegrityExplorationDecision()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "xq_lio_map"
        output.batch_id = int(metadata.batch_id)
        output.unconstrained_selected_index = -1
        output.unconstrained_selected_trajectory_id = -1
        output.selected_index = -1
        output.selected_trajectory_id = -1
        output.valid = False
        output.hard_constraint = True
        output.margin_in_utility = False
        output.reason = reason
        self.decision_publisher.publish(output)
        self._last_decision = output
        self._last_selected = None
        self._last_unconstrained = None
        self._last_publish_wall = time.monotonic()
        self.get_logger().error(f"P11 fail-closed: {reason}")

    @staticmethod
    def _aligned_metadata(metadata: ExplorationCandidateSet) -> bool:
        lengths = {
            len(metadata.trajectory_ids),
            len(metadata.candidate_names),
            len(metadata.frontier_ids),
            len(metadata.information_gains),
            len(metadata.travel_times_s),
            len(metadata.energy_costs),
            len(metadata.return_energy_costs),
            len(metadata.collision_probabilities),
        }
        return len(lengths) == 1 and next(iter(lengths), 0) > 0

    def _republish(self) -> None:
        if self._last_decision is None or time.monotonic() - self._last_publish_wall < 0.50:
            return
        self.decision_publisher.publish(self._last_decision)
        if self._last_selected is not None:
            self.selected_publisher.publish(self._last_selected)
        if self._last_unconstrained is not None:
            self.unconstrained_publisher.publish(self._last_unconstrained)
        self._last_publish_wall = time.monotonic()

    def _try_process(self) -> None:
        if any(
            value is None
            for value in (self._metadata, self._cloud, self._information_map, self._integrity)
        ):
            return
        metadata = self._metadata
        cloud = self._cloud
        information_map = self._information_map
        integrity = self._integrity
        assert metadata is not None and cloud is not None
        assert information_map is not None and integrity is not None
        batch_id = int(metadata.batch_id)
        if batch_id in self._processed_batches:
            self._republish()
            return
        if not self._aligned_metadata(metadata):
            self._publish_invalid(metadata, "REJECT_MISALIGNED_FRONTIER_METADATA")
            self._processed_batches.add(batch_id)
            return
        trajectory_ids = [int(value) for value in metadata.trajectory_ids]
        if any(value not in self._candidate_messages for value in trajectory_ids):
            return
        stamps = [self._stamp_s(value) for value in (metadata, cloud, information_map, integrity)]
        if max(stamps) - min(stamps) > self._float("maximum_input_age_s"):
            return
        if metadata.ground_truth_used or not information_map.valid or information_map.ground_truth_used:
            self._publish_invalid(metadata, "REJECT_GROUND_TRUTH_OR_INVALID_INFORMATION_MAP")
            self._processed_batches.add(batch_id)
            return

        obstacles = _cloud_xyz(cloud)
        positions = np.asarray([(p.x, p.y, p.z) for p in information_map.positions], dtype=float)
        normals = np.asarray([(n.x, n.y, n.z) for n in information_map.normals], dtype=float)
        surfel_lengths = {
            len(positions),
            len(normals),
            len(information_map.static_confidence),
            len(information_map.geometry_quality),
            len(information_map.last_seen_s),
        }
        if len(obstacles) == 0 or not positions.size or len(surfel_lengths) != 1:
            self._publish_invalid(metadata, "REJECT_EMPTY_OR_MISALIGNED_MAP")
            self._processed_batches.add(batch_id)
            return

        try:
            candidate_contexts = []
            alert_diagnostics = []
            for index, trajectory_id in enumerate(trajectory_ids):
                message = self._candidate_messages[trajectory_id]
                control = np.asarray([(p.x, p.y, p.z) for p in message.pos_pts], dtype=float)
                samples = sample_bspline(
                    control,
                    np.asarray(message.knots, dtype=float),
                    int(message.order),
                    self._float("trajectory_sample_interval_s"),
                )
                duration = float(
                    message.knots[-int(message.order) - 1]
                    - message.knots[int(message.order)]
                )
                speed = float(np.linalg.norm(np.diff(samples, axis=0), axis=1).sum()) / duration
                alert = compute_alert_limit(
                    samples,
                    obstacles,
                    speed_mps=speed,
                    latency_p99_s=self._float("latency_p99_s"),
                    maximum_acceleration_mps2=self._float("maximum_acceleration_mps2"),
                    body_radius_m=self._float("body_radius_m"),
                    base_reserve_m=self._float("base_reserve_m"),
                    tracking_reserve_m=self._float("tracking_reserve_m"),
                )
                predicted_information = build_information_profile(
                    samples,
                    positions,
                    normals,
                    np.asarray(information_map.static_confidence),
                    np.asarray(information_map.geometry_quality),
                    np.asarray(information_map.last_seen_s),
                    now=max(stamps),
                    visibility_radius=self._float("visibility_radius_m"),
                    age_time_constant=self._float("age_time_constant_s"),
                    information_scale=self._float("information_scale"),
                )
                alert_diagnostics.append(
                    {
                        "candidate": str(metadata.candidate_names[index]),
                        "minimum_geometric_clearance_m": float(
                            alert.geometric_clearance
                        ),
                        "minimum_alert_limit_m": float(alert.alert_limit),
                        "critical_sample_xyz": alert.critical_sample.tolist(),
                        "nearest_obstacle_xyz": alert.nearest_obstacle.tolist(),
                    }
                )
                candidate_contexts.append(
                    {
                        "index": index,
                        "trajectory_id": trajectory_id,
                        "samples": samples,
                        "duration": duration,
                        "alert": alert,
                        "predicted_information": predicted_information,
                    }
                )

            metric_source = str(metadata.metric_source or "metadata")
            if metric_source not in {"metadata", "online_map"}:
                raise ValueError(f"unsupported candidate metric source: {metric_source}")
            if metric_source == "online_map":
                task_components = compute_task_gains(
                    [item["samples"] for item in candidate_contexts],
                    positions,
                    np.asarray(information_map.static_confidence),
                    np.asarray(information_map.geometry_quality),
                    np.asarray(information_map.last_seen_s),
                    now_s=max(stamps),
                    visibility_radius_m=self._float("visibility_radius_m"),
                    age_time_constant_s=self._float(
                        "task_map_age_time_constant_s"
                    ),
                    progress_weight=self._float("task_progress_weight"),
                )
                collision_probabilities = [
                    pointwise_collision_probability(
                        item["alert"].alert_limits,
                        tracking_reserve_m=self._float("tracking_reserve_m"),
                        tracking_sigma_multiplier=self._float(
                            "collision_tracking_sigma_multiplier"
                        ),
                    )
                    for item in candidate_contexts
                ]
            else:
                task_components = None
                collision_probabilities = list(metadata.collision_probabilities)

            forecasts = []
            progress_efficiencies = []
            map_observation_gains = []
            for context_index, item in enumerate(candidate_contexts):
                index = int(item["index"])
                samples = item["samples"]
                duration = float(item["duration"])
                alert = item["alert"]
                predicted_information = item["predicted_information"]
                if task_components is None:
                    task_gain = float(metadata.information_gains[index])
                    path_length = float(
                        np.linalg.norm(np.diff(samples, axis=0), axis=1).sum()
                    )
                    progress_efficiency = float(
                        np.linalg.norm(samples[-1] - samples[0])
                        / max(path_length, 1.0e-12)
                    )
                    map_observation_gain = 0.0
                else:
                    component = task_components[context_index]
                    task_gain = component.gain
                    progress_efficiency = component.progress_efficiency
                    map_observation_gain = component.map_observation_gain
                progress_efficiencies.append(progress_efficiency)
                map_observation_gains.append(map_observation_gain)
                recovery = RecoveryCandidate(
                    name=str(metadata.candidate_names[index]),
                    positions=samples,
                    yaw=np.zeros(len(samples), dtype=float),
                    duration=duration,
                    extra_path_length=0.0,
                    extra_energy=0.0,
                )
                forecasts.append(
                    ExplorationForecast(
                        trajectory_id=int(item["trajectory_id"]),
                        frontier_id=str(metadata.frontier_ids[index]),
                        forecast=CandidateForecast(
                            candidate=recovery,
                            alert_limits=alert.alert_limits,
                            obstacle_directions=alert.obstacle_directions,
                            information_profile=predicted_information,
                        ),
                        information_gain=task_gain,
                        travel_time_s=float(metadata.travel_times_s[index]),
                        energy_cost=float(metadata.energy_costs[index]),
                        return_energy_cost=float(metadata.return_energy_costs[index]),
                        collision_probability=float(
                            collision_probabilities[context_index]
                        ),
                    )
                )
            selection = select_integrity_constrained_exploration(
                forecasts,
                np.asarray(integrity.integrity_covariance, dtype=float).reshape(3, 3),
                k_alpha=self._k_alpha,
                margin_reserve=self._float("margin_reserve_m"),
                collision_probability_limit=self._float("collision_probability_limit"),
                energy_remaining=self._float("energy_remaining"),
                information_weight=self._float("information_weight"),
                travel_time_weight=self._float("travel_time_weight"),
                energy_weight=self._float("energy_weight"),
                utility_indifference_band=self._float(
                    "utility_indifference_band"
                ),
                minimum_prediction_variance=self._float("minimum_prediction_variance_m2"),
            )
        except (ValueError, IndexError, np.linalg.LinAlgError) as error:
            self._publish_invalid(metadata, f"REJECT_INVALID_MATH_INPUT:{error}")
            self._processed_batches.add(batch_id)
            return

        output = IntegrityExplorationDecision()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "xq_lio_map"
        output.batch_id = batch_id
        output.trajectory_ids = trajectory_ids
        output.candidate_names = [item.forecast.candidate.name for item in forecasts]
        output.frontier_ids = [item.frontier_id for item in forecasts]
        output.information_gains = [item.information_gain for item in forecasts]
        output.progress_efficiencies = progress_efficiencies
        output.map_observation_gains = map_observation_gains
        output.localization_information_traces = [
            item.integrity.information_trace for item in selection.predictions
        ]
        output.travel_times_s = [item.travel_time_s for item in forecasts]
        output.energy_costs = [item.energy_cost for item in forecasts]
        output.return_energy_costs = [item.return_energy_cost for item in forecasts]
        output.collision_probabilities = [item.collision_probability for item in forecasts]
        output.utilities = [item.utility for item in selection.predictions]
        output.predicted_minimum_margins = [
            item.integrity.minimum_margin for item in selection.predictions
        ]
        output.integrity_feasible = [item.integrity_feasible for item in selection.predictions]
        output.collision_feasible = [item.collision_feasible for item in selection.predictions]
        output.energy_feasible = [item.energy_feasible for item in selection.predictions]
        output.feasible = [item.feasible for item in selection.predictions]
        output.unconstrained_selected_index = -1
        output.unconstrained_selected_trajectory_id = -1
        output.selected_index = -1
        output.selected_trajectory_id = -1
        output.unconstrained_selected_name = selection.unconstrained_selected_name or ""
        output.selected_name = selection.selected_name or ""
        if selection.unconstrained_selected_name is not None:
            output.unconstrained_selected_index = output.candidate_names.index(
                selection.unconstrained_selected_name
            )
            output.unconstrained_selected_trajectory_id = trajectory_ids[
                output.unconstrained_selected_index
            ]
        if selection.selected_name is not None:
            output.selected_index = output.candidate_names.index(selection.selected_name)
            output.selected_trajectory_id = trajectory_ids[output.selected_index]
        output.valid = True
        output.hard_constraint = True
        output.margin_in_utility = False
        output.minimum_intervention_applied = (
            selection.minimum_intervention_applied
        )
        output.utility_indifference_band = selection.utility_indifference_band
        output.candidate_generation_mode = str(
            metadata.candidate_generation_mode or "legacy"
        )
        output.metric_source = metric_source
        output.reason = (
            "INTEGRITY_CONSTRAINED_FRONTIER_SELECTED"
            if output.selected_index >= 0
            else "REJECT_NO_HARD_FEASIBLE_FRONTIER"
        )
        self.decision_publisher.publish(output)
        self._last_decision = output
        if output.unconstrained_selected_index >= 0:
            self._last_unconstrained = self._candidate_messages[
                output.unconstrained_selected_trajectory_id
            ]
            self.unconstrained_publisher.publish(self._last_unconstrained)
        if output.selected_index >= 0:
            self._last_selected = self._candidate_messages[output.selected_trajectory_id]
            self.selected_publisher.publish(self._last_selected)
        self._last_publish_wall = time.monotonic()
        debug = String()
        debug.data = json.dumps(
            {
                "phase": "P11_INTEGRITY_CONSTRAINED_EXPLORATION",
                "batch_id": batch_id,
                "candidate_names": list(output.candidate_names),
                "candidate_generation_mode": output.candidate_generation_mode,
                "metric_source": output.metric_source,
                "utilities": list(output.utilities),
                "task_information_gains": list(output.information_gains),
                "progress_efficiencies": list(output.progress_efficiencies),
                "map_observation_gains": list(output.map_observation_gains),
                "localization_information_traces": list(
                    output.localization_information_traces
                ),
                "collision_probabilities": list(output.collision_probabilities),
                "energy_costs": list(output.energy_costs),
                "return_energy_costs": list(output.return_energy_costs),
                "predicted_minimum_margins": list(output.predicted_minimum_margins),
                "alert_diagnostics": alert_diagnostics,
                "feasible": list(output.feasible),
                "unconstrained_selected": output.unconstrained_selected_name,
                "selected": output.selected_name,
                "hard_constraint": True,
                "margin_in_utility": False,
                "minimum_intervention_applied": (
                    output.minimum_intervention_applied
                ),
                "utility_indifference_band": output.utility_indifference_band,
                "ground_truth_subscribed": False,
                "calibration_sha256": self._calibration_sha,
            },
            separators=(",", ":"),
        )
        self.debug_publisher.publish(debug)
        diagnostics = "; ".join(
            f"{name}:clearance={item['minimum_geometric_clearance_m']:.3f}m,"
            f"M={margin:.3f}m,feasible={str(feasible).lower()}"
            for name, item, margin, feasible in zip(
                output.candidate_names,
                alert_diagnostics,
                output.predicted_minimum_margins,
                output.feasible,
            )
        )
        self.get_logger().info(
            f"P11 batch={batch_id} unconstrained={output.unconstrained_selected_name or 'NONE'} "
            f"selected={output.selected_name or 'NONE'} | {diagnostics}"
        )
        self._processed_batches.add(batch_id)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P11IntegrityExplorationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
