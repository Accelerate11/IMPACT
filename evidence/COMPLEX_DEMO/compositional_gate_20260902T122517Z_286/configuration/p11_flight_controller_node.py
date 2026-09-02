"""P11 two-arm flight controller and deterministic Frontier-candidate adapter."""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import (
    DirectionalIntegrity,
    ExplorationCandidateSet,
    InformationMap,
    IntegrityExplorationDecision,
)

from .alert_limit import sample_bspline
from .integrity_exploration import rolling_horizon_distances


def build_geometric_candidate_positions(
    direct: np.ndarray,
    profile: np.ndarray,
    *,
    lateral_offset_m: float,
    enable_vertical_candidate: bool = False,
    enable_diagonal_vertical_candidates: bool = False,
    vertical_offset_m: float = 0.0,
) -> tuple[tuple[str, np.ndarray], ...]:
    """Build a deterministic 3D candidate family around one direct path."""
    baseline = np.asarray(direct, dtype=float)
    window = np.asarray(profile, dtype=float).reshape(-1)
    if (
        baseline.ndim != 2
        or baseline.shape[1:] != (3,)
        or len(baseline) != len(window)
        or not np.isfinite(baseline).all()
        or not np.isfinite(window).all()
        or not math.isfinite(lateral_offset_m)
        or lateral_offset_m <= 0.0
    ):
        raise ValueError("candidate baseline, profile, and lateral offset are invalid")
    families = [("high_information_direct", baseline.copy())]
    right = baseline.copy()
    right[:, 1] -= lateral_offset_m * window
    left = baseline.copy()
    left[:, 1] += lateral_offset_m * window
    families.extend((("geometry_rich_right", right), ("geometry_rich_left", left)))
    if enable_vertical_candidate or enable_diagonal_vertical_candidates:
        if not math.isfinite(vertical_offset_m) or vertical_offset_m <= 0.0:
            raise ValueError("enabled spatial candidate requires a positive vertical offset")
    if enable_vertical_candidate:
        upward = baseline.copy()
        upward[:, 2] += vertical_offset_m * window
        families.append(("geometry_rich_up", upward))
    if enable_diagonal_vertical_candidates:
        up_right = baseline.copy()
        up_right[:, 1] -= lateral_offset_m * window
        up_right[:, 2] += vertical_offset_m * window
        up_left = baseline.copy()
        up_left[:, 1] += lateral_offset_m * window
        up_left[:, 2] += vertical_offset_m * window
        families.extend(
            (
                ("geometry_rich_up_right", up_right),
                ("geometry_rich_up_left", up_left),
            )
        )
    return tuple(families)


class P11FlightControllerNode(Node):
    VARIANTS = ("information_only", "integrity_constrained")

    def __init__(self) -> None:
        super().__init__("xq_p11_flight_controller")
        defaults = {
            "variant": "information_only",
            "initialization_s": 12.0,
            "trajectory_duration_s": 28.0,
            "trajectory_distance_m": 7.5,
            "mission_distance_m": 24.0,
            "goal_tolerance_m": 0.25,
            "maximum_segments": 8,
            "lateral_offset_m": 0.60,
            "lateral_candidate_shape": "return_to_center",
            "enable_vertical_candidate": False,
            "enable_diagonal_vertical_candidates": False,
            "vertical_offset_m": 0.70,
            "cruise_altitude_offset_m": 0.0,
            "persistent_altitude_threshold_m": 0.08,
            "persistent_altitude_limit_m": 0.40,
            "control_point_count": 21,
            "position_gain": 0.80,
            "maximum_speed_mps": 0.42,
            "segment_settle_s": 1.0,
            "final_settle_s": 3.0,
            "segment_goal_tolerance_m": 0.25,
            "maximum_segment_extension_s": 90.0,
            "direct_information_gain": 1.0,
            "safe_information_gain": 0.75,
            "collision_probability": 0.001,
            "return_energy_cost": 4.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.variant = str(self.get_parameter("variant").value)
        if self.variant not in self.VARIANTS:
            raise ValueError(f"unsupported P11 flight variant: {self.variant}")
        self._odom: Odometry | None = None
        self._integrity: DirectionalIntegrity | None = None
        self._information_map: InformationMap | None = None
        self._decision: IntegrityExplorationDecision | None = None
        self._selected: Bspline | None = None
        self._unconstrained: Bspline | None = None
        self._candidates: list[Bspline] = []
        self._metadata: ExplorationCandidateSet | None = None
        self._start_sim_s: float | None = None
        self._trajectory_start_sim_s: float | None = None
        self._last_position: np.ndarray | None = None
        self._path_length_m = 0.0
        self._planned_energy_spent = 0.0
        self._segment_index = 0
        self._segments_completed = 0
        self._decisions_applied = 0
        self._current_segment_distance_m = 0.0
        self._current_segment_duration_s = 0.0
        self._current_segment_end_x_m = float("nan")
        self._mission_start_x_m: float | None = None
        self._mission_goal_x_m: float | None = None
        self._mission_cruise_z_m: float | None = None
        self._finished = False
        self._last_phase = ""

        state_qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
        latched_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.command_publisher = self.create_publisher(Twist, "/xq/p3/cmd_vel", 20)
        self.candidate_publisher = self.create_publisher(
            Bspline, "/planning/p11/frontier_candidates", state_qos
        )
        self.metadata_publisher = self.create_publisher(
            ExplorationCandidateSet, "/planning/p11/frontier_candidate_set", latched_qos
        )
        self.status_publisher = self.create_publisher(
            String, "/xq/p11/flight_status", latched_qos
        )
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, state_qos)
        self.create_subscription(
            DirectionalIntegrity, "/integrity/directional", self._integrity_cb, state_qos
        )
        self.create_subscription(
            InformationMap, "/integrity/information_map", self._information_map_cb, state_qos
        )
        self.create_subscription(
            IntegrityExplorationDecision,
            "/integrity/exploration_decision",
            self._decision_cb,
            latched_qos,
        )
        self.create_subscription(
            Bspline, "/planning/p11/selected_bspline", self._selected_cb, latched_qos
        )
        self.create_subscription(
            Bspline,
            "/planning/p11/unconstrained_bspline",
            self._unconstrained_cb,
            latched_qos,
        )
        self.create_timer(0.05, self._timer)
        self.get_logger().info(
            f"P11 flight variant={self.variant}; Frontier adapter has no Ground Truth"
        )

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _position(message: Odometry) -> np.ndarray:
        value = message.pose.pose.position
        return np.asarray((value.x, value.y, value.z), dtype=float)

    def _odom_cb(self, message: Odometry) -> None:
        current = self._position(message)
        if self._last_position is not None and self._trajectory_start_sim_s is not None:
            step = float(np.linalg.norm(current - self._last_position))
            if math.isfinite(step) and step < 0.20:
                self._path_length_m += step
        self._last_position = current
        self._odom = message

    def _integrity_cb(self, message: DirectionalIntegrity) -> None:
        self._integrity = message

    def _information_map_cb(self, message: InformationMap) -> None:
        self._information_map = message

    def _decision_cb(self, message: IntegrityExplorationDecision) -> None:
        if self._metadata is not None and message.batch_id == self._metadata.batch_id:
            self._decision = message

    def _selected_cb(self, message: Bspline) -> None:
        if self._decision is not None and message.traj_id == self._decision.selected_trajectory_id:
            self._selected = message

    def _unconstrained_cb(self, message: Bspline) -> None:
        if (
            self._decision is not None
            and message.traj_id == self._decision.unconstrained_selected_trajectory_id
        ):
            self._unconstrained = message

    def _ready(self) -> bool:
        return bool(
            self._odom is not None
            and self._integrity is not None
            and self._information_map is not None
            and self._information_map.valid
            and not self._information_map.ground_truth_used
        )

    def _bspline(self, positions: np.ndarray, trajectory_id: int, duration: float) -> Bspline:
        count = len(positions)
        message = Bspline()
        message.order = 1
        message.traj_id = trajectory_id
        message.start_time = self.get_clock().now().to_msg()
        message.pos_pts = [
            Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            for point in positions
        ]
        interior = [duration * index / float(count - 1) for index in range(1, count - 1)]
        message.knots = [0.0, 0.0, *interior, duration, duration]
        message.yaw_pts = np.zeros(count).tolist()
        message.yaw_dt = duration / float(count - 1)
        return message

    def _make_candidates(self, now_s: float) -> None:
        assert self._odom is not None
        start = self._position(self._odom)
        count = int(self.get_parameter("control_point_count").value)
        assert self._mission_goal_x_m is not None
        assert self._mission_cruise_z_m is not None
        goal_x = self._mission_goal_x_m
        horizon = self._float("trajectory_distance_m")
        distances = rolling_horizon_distances(float(start[0]), goal_x, horizon)
        distance = float(distances[0])
        nominal_duration = self._float("trajectory_duration_s")
        duration = nominal_duration * distance / horizon
        phase = np.linspace(0.0, 1.0, count)
        direct = start + np.column_stack(
            (distance * phase, np.zeros(count), np.zeros(count))
        )
        # A bounded integrity-recovery climb is state, not tracking error.  If
        # it is large enough to be intentional, retain it across the next
        # rolling horizon instead of immediately commanding the vehicle back
        # into the geometry that caused the hard rejection.
        cruise_z = self._mission_cruise_z_m
        recovered_height = float(start[2]) - cruise_z
        target_z = cruise_z
        if recovered_height >= self._float("persistent_altitude_threshold_m"):
            target_z = min(
                float(start[2]),
                cruise_z + self._float("persistent_altitude_limit_m"),
            )
        altitude_phase = 0.5 - 0.5 * np.cos(np.pi * phase)
        direct[:, 2] += (target_z - float(start[2])) * altitude_phase
        lateral_window = self._lateral_window(
            phase, str(self.get_parameter("lateral_candidate_shape").value)
        )
        candidate_positions = build_geometric_candidate_positions(
            direct,
            lateral_window,
            lateral_offset_m=self._float("lateral_offset_m"),
            enable_vertical_candidate=bool(
                self.get_parameter("enable_vertical_candidate").value
            ),
            enable_diagonal_vertical_candidates=bool(
                self.get_parameter("enable_diagonal_vertical_candidates").value
            ),
            vertical_offset_m=self._float("vertical_offset_m"),
        )
        batch_id = 20261100 + self._segment_index
        trajectory_base = 202611000 + 10 * self._segment_index
        self._candidates = [
            self._bspline(positions, trajectory_base + index, duration)
            for index, (_, positions) in enumerate(candidate_positions, start=1)
        ]
        path_lengths = [
            float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
            for _, positions in candidate_positions
        ]
        candidate_count = len(candidate_positions)
        metadata = ExplorationCandidateSet()
        metadata.header.stamp = self.get_clock().now().to_msg()
        metadata.header.frame_id = "xq_lio_map"
        metadata.batch_id = batch_id
        metadata.trajectory_ids = [message.traj_id for message in self._candidates]
        metadata.candidate_names = [name for name, _ in candidate_positions]
        metadata.frontier_ids = ["frontier_main"] * candidate_count
        metadata.information_gains = [
            self._float(
                "direct_information_gain"
                if name == "high_information_direct"
                else "safe_information_gain"
            )
            for name, _ in candidate_positions
        ]
        metadata.travel_times_s = [duration] * candidate_count
        metadata.energy_costs = [
            self._planned_energy_spent + length for length in path_lengths
        ]
        metadata.return_energy_costs = [self._float("return_energy_cost")] * candidate_count
        metadata.collision_probabilities = [self._float("collision_probability")] * candidate_count
        metadata.ground_truth_used = False
        self._metadata = metadata
        self._trajectory_start_sim_s = None
        self._current_segment_distance_m = distance
        self._current_segment_duration_s = duration
        self._current_segment_end_x_m = float(start[0] + distance)
        self.get_logger().info(
            f"P11 rolling batch={batch_id} segment={self._segment_index + 1} "
            f"x={start[0]:.2f}->{self._current_segment_end_x_m:.2f} m"
        )

    @staticmethod
    def _lateral_window(phase: np.ndarray, shape: str) -> np.ndarray:
        """Return the lateral profile for a certified candidate family."""
        values = np.asarray(phase, dtype=float)
        if shape == "return_to_center":
            return np.sin(np.pi * values)
        if shape == "lane_shift":
            # Complete the lane change in the first 45% of the horizon, then
            # hold the certified side corridor through a portal or clutter
            # located near the segment endpoint.
            scaled = np.clip(values / 0.45, 0.0, 1.0)
            return scaled * scaled * (3.0 - 2.0 * scaled)
        if shape == "challenge_then_center":
            # Reach the side observation corridor before the integrity island,
            # hold through its full longitudinal extent plus the vehicle's
            # protected-radius runout, then complete a smooth return before
            # the horizon endpoint.  This prevents cutting across the panel's
            # trailing corner while keeping the next segment centred.
            rise_phase = np.clip(values / 0.20, 0.0, 1.0)
            rise = rise_phase * rise_phase * (3.0 - 2.0 * rise_phase)
            fall_phase = np.clip((values - 0.78) / 0.18, 0.0, 1.0)
            fall = 1.0 - fall_phase * fall_phase * (3.0 - 2.0 * fall_phase)
            return np.minimum(rise, fall)
        raise ValueError(f"unsupported lateral_candidate_shape: {shape}")

    @staticmethod
    def _trajectory_positions(message: Bspline, elapsed_s: float) -> tuple[np.ndarray, np.ndarray]:
        control = np.asarray([(p.x, p.y, p.z) for p in message.pos_pts], dtype=float)
        knots = np.asarray(message.knots, dtype=float)
        duration = float(knots[-int(message.order) - 1] - knots[int(message.order)])
        t0 = min(max(elapsed_s, 0.0), duration)
        t1 = min(t0 + 0.10, duration)
        if t0 >= duration:
            return control[-1].copy(), np.zeros(3, dtype=float)
        desired = sample_bspline(
            control, knots, int(message.order), max(duration, 1.0), minimum_parameter_s=t0
        )[0]
        ahead = (
            control[-1].copy()
            if t1 >= duration
            else sample_bspline(
                control,
                knots,
                int(message.order),
                max(duration, 1.0),
                minimum_parameter_s=t1,
            )[0]
        )
        return desired, (ahead - desired) / max(t1 - t0, 0.10)

    def _publish_status(self, phase: str, elapsed_s: float) -> None:
        if phase == self._last_phase:
            return
        executed_name = ""
        if self._decision is not None:
            executed_name = (
                self._decision.unconstrained_selected_name
                if self.variant == "information_only"
                else self._decision.selected_name
            )
        message = String()
        message.data = json.dumps(
            {
                "variant": self.variant,
                "phase": phase,
                "elapsed_s": elapsed_s,
                "path_length_m": self._path_length_m,
                "mission_start_x_m": self._mission_start_x_m,
                "mission_goal_x_m": self._mission_goal_x_m,
                "mission_cruise_z_m": self._mission_cruise_z_m,
                "mission_distance_m": self._float("mission_distance_m"),
                "current_position_x_m": (
                    float(self._position(self._odom)[0]) if self._odom is not None else None
                ),
                "current_position_z_m": (
                    float(self._position(self._odom)[2]) if self._odom is not None else None
                ),
                "segment_index": self._segment_index,
                "segments_completed": self._segments_completed,
                "decisions_applied": self._decisions_applied,
                "current_batch_id": int(self._metadata.batch_id) if self._metadata else -1,
                "current_segment_end_x_m": self._current_segment_end_x_m,
                "rolling_horizon": True,
                "candidate_set_published": self._metadata is not None,
                "decision_valid": bool(self._decision is not None and self._decision.valid),
                "unconstrained_selected_name": (
                    self._decision.unconstrained_selected_name if self._decision else ""
                ),
                "integrity_selected_name": self._decision.selected_name if self._decision else "",
                "executed_name": executed_name,
                "selected_applied": (
                    self._selected is not None
                    if self.variant == "integrity_constrained"
                    else self._unconstrained is not None
                ),
                "finished": self._finished,
                "ground_truth_subscribed": False,
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(message)
        self.get_logger().info(f"P11 flight phase={phase} t={elapsed_s:.2f}s")
        self._last_phase = phase

    def _timer(self) -> None:
        now_s = self._now_s()
        if now_s <= 0.0:
            return
        if self._start_sim_s is None:
            self._start_sim_s = now_s
        elapsed_from_node = now_s - self._start_sim_s
        command = Twist()
        if self._finished:
            self.command_publisher.publish(command)
            self._publish_status("COMPLETE", elapsed_from_node)
            return
        if elapsed_from_node < self._float("initialization_s") or not self._ready():
            self.command_publisher.publish(command)
            self._publish_status("WAIT_INITIALIZATION", elapsed_from_node)
            return
        if self._mission_start_x_m is None:
            assert self._odom is not None
            self._mission_start_x_m = float(self._position(self._odom)[0])
            self._mission_goal_x_m = (
                self._mission_start_x_m + self._float("mission_distance_m")
            )
            self._mission_cruise_z_m = (
                float(self._position(self._odom)[2])
                + self._float("cruise_altitude_offset_m")
            )
        if not self._candidates:
            assert self._odom is not None
            assert self._mission_goal_x_m is not None
            if self._position(self._odom)[0] >= (
                self._mission_goal_x_m - self._float("goal_tolerance_m")
            ):
                self._finished = True
                self.command_publisher.publish(command)
                self._publish_status("COMPLETE", elapsed_from_node)
                return
            if self._segment_index >= int(self.get_parameter("maximum_segments").value):
                self.command_publisher.publish(command)
                self._publish_status("FAIL_CLOSED_MAX_SEGMENTS", elapsed_from_node)
                return
            self._make_candidates(now_s)
        assert self._metadata is not None
        if self._decision is None or not self._decision.valid:
            # Candidate sets are immutable within one rolling horizon.  Keep
            # offering them until a hard decision is received, then stop;
            # republishing the full family at control rate only retriggers the
            # scorer and saturates DDS/recording without changing the plan.
            for candidate in self._candidates:
                self.candidate_publisher.publish(candidate)
            self._metadata.header.stamp = self.get_clock().now().to_msg()
            self.metadata_publisher.publish(self._metadata)
            self.command_publisher.publish(command)
            self._publish_status("WAIT_HARD_DECISION", elapsed_from_node)
            return
        trajectory = (
            self._unconstrained if self.variant == "information_only" else self._selected
        )
        if trajectory is None:
            self.command_publisher.publish(command)
            self._publish_status("FAIL_CLOSED_NO_TRAJECTORY", elapsed_from_node)
            return
        if self._trajectory_start_sim_s is None:
            self._trajectory_start_sim_s = now_s
        assert self._odom is not None
        elapsed = now_s - self._trajectory_start_sim_s
        duration = float(
            trajectory.knots[-int(trajectory.order) - 1]
            - trajectory.knots[int(trajectory.order)]
        )
        final_segment = self._current_segment_end_x_m >= (
            self._mission_goal_x_m - self._float("goal_tolerance_m")
        )
        settle = self._float("final_settle_s" if final_segment else "segment_settle_s")
        terminal = np.asarray(
            (
                trajectory.pos_pts[-1].x,
                trajectory.pos_pts[-1].y,
                trajectory.pos_pts[-1].z,
            ),
            dtype=float,
        )
        terminal_error = float(np.linalg.norm(terminal - self._position(self._odom)))
        terminal_reached = terminal_error <= self._float("segment_goal_tolerance_m")
        if elapsed >= duration + settle and terminal_reached:
            executed_index = (
                int(self._decision.unconstrained_selected_index)
                if self.variant == "information_only"
                else int(self._decision.selected_index)
            )
            if not 0 <= executed_index < len(self._decision.energy_costs):
                self.command_publisher.publish(command)
                self._publish_status("FAIL_CLOSED_ENERGY_INDEX", elapsed_from_node)
                return
            self._planned_energy_spent = float(
                self._decision.energy_costs[executed_index]
            )
            self._segments_completed += 1
            self._decisions_applied += 1
            if final_segment:
                self._finished = True
                self.command_publisher.publish(command)
                self._publish_status("COMPLETE", elapsed_from_node)
                return
            self._segment_index += 1
            self._candidates = []
            self._metadata = None
            self._decision = None
            self._selected = None
            self._unconstrained = None
            self._trajectory_start_sim_s = None
            self.command_publisher.publish(command)
            self._publish_status("REPLAN", elapsed_from_node)
            return
        if (
            elapsed
            >= duration + settle + self._float("maximum_segment_extension_s")
            and not terminal_reached
        ):
            self.command_publisher.publish(command)
            self._publish_status("FAIL_CLOSED_TRACKING_TIMEOUT", elapsed_from_node)
            return
        desired, feedforward = self._trajectory_positions(trajectory, elapsed)
        error = desired - self._position(self._odom)
        velocity = feedforward + self._float("position_gain") * error
        speed = float(np.linalg.norm(velocity))
        maximum = self._float("maximum_speed_mps")
        if speed > maximum:
            velocity *= maximum / speed
        command.linear.x, command.linear.y, command.linear.z = velocity.tolist()
        self.command_publisher.publish(command)
        self._publish_status(
            "TRACK_TERMINAL" if elapsed >= duration else "EXECUTE",
            elapsed_from_node,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P11FlightControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            stop = Twist()
            for _ in range(3):
                node.command_publisher.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
