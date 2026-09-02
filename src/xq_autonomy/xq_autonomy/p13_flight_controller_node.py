"""P13 controller: P12 autonomy with measured p99 latency in its safety envelope."""

from __future__ import annotations

from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
import json
import math
import time

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .latency_safety import latency_radius, safety_envelope, summarize_latencies
from .p12_dynamic_map_node import _cloud_xyz
from .p12_flight_controller_node import P12FlightControllerNode


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def online_speed_limited_parameter(
    name: str, nominal_value: float, speed_limit_mps: float
) -> float:
    """Apply the live safety envelope only to velocity, never trajectory time."""
    if name == "maximum_speed_mps":
        return min(float(nominal_value), float(speed_limit_mps))
    return float(nominal_value)


def localization_snapshot_covers_sensor(
    sensor_stamp_ns: int,
    odometry_stamp_ns: int,
    maximum_lead_ns: int = 250_000_000,
) -> bool:
    """Whether an already-delivered odometry sample covers a sensor frame."""
    return bool(
        sensor_stamp_ns <= odometry_stamp_ns
        and odometry_stamp_ns - sensor_stamp_ns <= maximum_lead_ns
    )


def ordered_map_dependency_completion(
    map_arrival_steady_ns: int,
    localization_done_steady_ns: int,
) -> tuple[int, int]:
    """Complete the ordered localization->map dependency conservatively.

    An exact-source P12 map cannot exist before P12 has an odometry snapshot.
    If P13's duplicate odometry subscription has not delivered it yet, the map
    arrival itself is therefore a conservative observable upper bound for both
    dependency stages.
    """
    localization_done = (
        int(localization_done_steady_ns)
        if localization_done_steady_ns else int(map_arrival_steady_ns)
    )
    return localization_done, max(
        localization_done, int(map_arrival_steady_ns)
    )


class P13FlightControllerNode(P12FlightControllerNode):
    def __init__(self) -> None:
        super().__init__()
        defaults = {
            "latency_profile": "low_50ms",
            "planner_delay_ms": 50.0,
            "geometric_clearance_m": 0.82,
            "fixed_buffer_m": 0.58,
            "protection_level_m": 0.10,
            "required_margin_m": 0.06,
            "maximum_acceleration_mps2": 0.8,
            "latency_window_samples": 500,
            "latency_fallback_overhead_ms": 20.0,
            "minimum_operating_speed_mps": 0.04,
            "rejected_candidate_retry_s": 2.0,
            "maximum_candidate_retries": 6,
            "integrity_recovery_speed_mps": 0.08,
            "integrity_recovery_max_offset_m": 0.35,
            "integrity_recovery_half_period_s": 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._p13_nominal_maximum_speed_mps = float(
            self.get_parameter("maximum_speed_mps").value
        )
        self._p13_speed_limit_mps = self._p13_nominal_maximum_speed_mps
        self._p13_latency_samples_s: deque[float] = deque(
            maxlen=int(self.get_parameter("latency_window_samples").value)
        )
        self._p13_sensor_seq = 0
        self._p13_processed_seq = 0
        self._p13_pipelines: dict[int, dict[str, int | float]] = {}
        self._p13_trace_seq = 0
        self._p13_envelope = self._calculate_envelope(self._fallback_latency_s())
        self._p13_last_status_wall_s = -math.inf
        self._p13_trace_ground_truth_clean = True
        self._p13_candidate_nonce = 0
        self._p13_candidate_retry_count = 0
        self._p13_rejected_since_s: float | None = None
        self._p13_integrity_recovery_active = False
        self._p13_integrity_recovery_anchor: np.ndarray | None = None
        self._p13_integrity_recovery_direction: np.ndarray | None = None
        self._p13_integrity_recovery_start_s: float | None = None
        self._p13_static_points = np.empty((0, 3), dtype=float)
        self._p13_pending_pipeline: dict[str, int | float] | None = None
        self._p13_planner_trigger_perf_ns = 0
        self._p13_planner_future: Future[int] | None = None
        self._p13_planner_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="xq_p13_planner"
        )

        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        latest_sensor = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        latched = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.p13_trace_publisher = self.create_publisher(
            String, "/integrity/p13/latency_trace", latched
        )
        self.p13_status_publisher = self.create_publisher(
            String, "/xq/p13/flight_status", latched
        )
        self.create_subscription(
            PointCloud2, "/livox/lidar", self._p13_sensor_cb, latest_sensor
        )
        self.create_subscription(
            PointCloud2,
            "/mapping/p12/static_voxels",
            self._p13_static_cloud_cb,
            reliable,
        )
        self._p13_pipeline_timer = self.create_timer(
            0.01, self._process_latency_cycle
        )
        self.get_logger().info(
            "P13 latency-aware safety: measured nearest-rank p99 drives AL and speed; "
            "Ground Truth forbidden"
        )

    def _float(self, name: str) -> float:
        value = super()._float(name)
        if hasattr(self, "_p13_speed_limit_mps"):
            value = online_speed_limited_parameter(
                name, value, self._p13_speed_limit_mps
            )
        # Keep trajectory knot times deterministic.  The measured p99 envelope
        # is applied to every velocity command above, so baking one startup
        # latency sample into the B-spline duration would freeze a transient
        # minimum speed for the entire rolling horizon.
        return value

    def _fallback_latency_s(self) -> float:
        return 1.0e-3 * (
            float(self.get_parameter("planner_delay_ms").value)
            + float(self.get_parameter("latency_fallback_overhead_ms").value)
        )

    @staticmethod
    def _complete_planner_delay(delay_s: float) -> int:
        """Model planner work without blocking the ROS subscription executor."""
        time.sleep(max(0.0, delay_s))
        return time.perf_counter_ns()

    def _calculate_envelope(self, latency_s: float):
        return safety_envelope(
            latency_s=latency_s,
            geometric_clearance_m=float(self.get_parameter("geometric_clearance_m").value),
            fixed_buffer_m=float(self.get_parameter("fixed_buffer_m").value),
            protection_level_m=float(self.get_parameter("protection_level_m").value),
            required_margin_m=float(self.get_parameter("required_margin_m").value),
            maximum_speed_mps=float(self.get_parameter("maximum_speed_mps").value),
            maximum_acceleration_mps2=float(
                self.get_parameter("maximum_acceleration_mps2").value
            ),
        )

    def _unmitigated_limits(self, latency_s: float) -> tuple[float, float]:
        radius = latency_radius(
            self._p13_nominal_maximum_speed_mps,
            latency_s,
            float(self.get_parameter("maximum_acceleration_mps2").value),
        )
        alert = (
            float(self.get_parameter("geometric_clearance_m").value)
            - float(self.get_parameter("fixed_buffer_m").value)
            - radius
        )
        return alert, alert - float(self.get_parameter("protection_level_m").value)

    def _p13_sensor_cb(self, message: PointCloud2) -> None:
        receive_perf_ns = time.perf_counter_ns()
        now_sim_ns = self.get_clock().now().nanoseconds
        sensor_ns = _stamp_ns(message.header.stamp)
        self._p13_sensor_seq += 1
        localization_already_available = bool(
            self._odom is not None
            and localization_snapshot_covers_sensor(
                sensor_ns, _stamp_ns(self._odom.header.stamp)
            )
        )
        self._p13_pipelines[sensor_ns] = {
            "seq": self._p13_sensor_seq,
            "sensor_sim_ns": sensor_ns,
            "receive_steady_ns": receive_perf_ns,
            "receive_perf_ns": receive_perf_ns,
            "sensor_age_at_receive_ns": max(0, int(now_sim_ns) - sensor_ns),
            # ROS may dispatch odometry before the LiDAR callback carrying the
            # same (or slightly earlier) simulation stamp.  In that case the
            # localization product is already available at sensor receipt;
            # waiting for the *next* odometry callback adds an artificial
            # one-period tail to the measured pipeline latency.
            "localization_done_steady_ns": (
                receive_perf_ns if localization_already_available else 0
            ),
            "map_arrival_steady_ns": 0,
            "map_done_steady_ns": 0,
        }
        if len(self._p13_pipelines) > 64:
            oldest = min(
                self._p13_pipelines,
                key=lambda stamp: int(self._p13_pipelines[stamp]["seq"]),
            )
            del self._p13_pipelines[oldest]

    def _p13_static_cloud_cb(self, message: PointCloud2) -> None:
        points = _cloud_xyz(message)
        if len(points) and np.isfinite(points).all():
            self._p13_static_points = points

    @staticmethod
    def _weak_direction_away_from_obstacle(
        position: np.ndarray,
        weak_direction: np.ndarray,
        static_points: np.ndarray,
    ) -> np.ndarray | None:
        """Resolve the weak eigenvector sign using the nearest static surface."""
        current = np.asarray(position, dtype=float).reshape(3)
        weak = np.asarray(weak_direction, dtype=float).reshape(3)
        points = np.asarray(static_points, dtype=float)
        if (
            points.ndim != 2
            or points.shape[1] != 3
            or len(points) == 0
            or not np.isfinite(current).all()
            or not np.isfinite(weak).all()
            or not np.isfinite(points).all()
        ):
            return None
        norm = float(np.linalg.norm(weak))
        if norm <= 1.0e-9:
            return None
        weak /= norm
        offsets = points - current
        nearest = offsets[int(np.argmin(np.einsum("ij,ij->i", offsets, offsets)))]
        # Eigenvectors have arbitrary sign.  Pick the sign whose projection
        # moves away from the closest mapped surface, thereby increasing the
        # directional alert limit while exciting the measured weak axis.
        if float(weak @ nearest) > 0.0:
            weak = -weak
        return weak

    def _current_integrity_recovery_direction(self) -> np.ndarray | None:
        if self._odom is None or self._integrity is None:
            return None
        return self._weak_direction_away_from_obstacle(
            self._position(self._odom),
            np.asarray(self._integrity.weak_direction_map, dtype=float),
            self._p13_static_points,
        )

    @staticmethod
    def _best_rejected_candidate_direction(
        current: np.ndarray,
        candidate_positions: list[np.ndarray],
        predicted_margins: list[float],
    ) -> np.ndarray | None:
        """Extract a bounded excitation axis without executing a rejected path."""
        if not candidate_positions or len(candidate_positions) != len(predicted_margins):
            return None
        margins = np.asarray(predicted_margins, dtype=float)
        if not np.isfinite(margins).any():
            return None
        index = int(np.nanargmax(margins))
        points = np.asarray(candidate_positions[index], dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            return None
        # The independent recovery command has no forward component and is
        # bounded separately below. Only the best rejected candidate's lateral
        # / vertical excitation direction is reused.
        direction = points[len(points) // 2] - np.asarray(current, dtype=float)
        direction[0] = 0.0
        norm = float(np.linalg.norm(direction))
        if not math.isfinite(norm) or norm <= 1.0e-9:
            return None
        return direction / norm

    def _current_candidate_recovery_direction(self) -> np.ndarray | None:
        if self._odom is None or self._decision is None:
            return None
        positions = [
            np.asarray([(p.x, p.y, p.z) for p in candidate.pos_pts], dtype=float)
            for candidate in self._candidates
        ]
        return self._best_rejected_candidate_direction(
            self._position(self._odom),
            positions,
            list(self._decision.predicted_minimum_margins),
        )

    def _make_candidates(self, now_s: float) -> None:
        """Give every safe resampling attempt a fresh immutable batch identity."""
        super()._make_candidates(now_s)
        self._p13_candidate_nonce += 1
        trajectory_offset = 1_000_000 * self._p13_candidate_nonce
        assert self._metadata is not None
        self._metadata.batch_id += 10_000 * self._p13_candidate_nonce
        for candidate in self._candidates:
            candidate.traj_id += trajectory_offset
        self._metadata.trajectory_ids = [candidate.traj_id for candidate in self._candidates]

    def _retry_rejected_candidate_set(self) -> None:
        rejected = bool(
            self.variant == "integrity_constrained"
            and self._decision is not None
            and self._decision.valid
            and int(self._decision.selected_trajectory_id) < 0
            and self._selected is None
        )
        now_s = self._now_s()
        local_recovery_clear = bool(
            not math.isfinite(self._nearest_dynamic_range_m)
            or self._nearest_dynamic_range_m > self._float("planning_lookahead_m")
        )
        recovery_permitted = bool(
            rejected
            and local_recovery_clear
            and not self._finished
            and self._odom is not None
        )
        recovery_direction = self._current_candidate_recovery_direction()
        if recovery_direction is None:
            recovery_direction = self._current_integrity_recovery_direction()
        if (
            recovery_permitted
            and recovery_direction is not None
            and not self._p13_integrity_recovery_active
        ):
            self._p13_integrity_recovery_active = True
            self._p13_integrity_recovery_anchor = self._position(self._odom).copy()
            self._p13_integrity_recovery_direction = recovery_direction
            self._p13_integrity_recovery_start_s = now_s
            self.get_logger().warning(
                "P13 hard rejection in a dynamically clear corridor; starting bounded "
                "weak-direction minimum-excitation recovery "
                f"d={recovery_direction.tolist()}"
            )
        if self._p13_integrity_recovery_active and self._selected is not None:
            self._p13_integrity_recovery_active = False
            self._p13_integrity_recovery_anchor = None
            self._p13_integrity_recovery_direction = None
            self._p13_integrity_recovery_start_s = None
            self.get_logger().info(
                "P13 minimum-excitation recovery produced an accepted trajectory"
            )
        if (
            self._p13_integrity_recovery_active
            and self._selected is None
            and local_recovery_clear
            and self._odom is not None
        ):
            current = self._position(self._odom)
            anchor = self._p13_integrity_recovery_anchor
            direction = self._p13_integrity_recovery_direction
            if anchor is not None and direction is not None:
                maximum = float(
                    self.get_parameter("integrity_recovery_max_offset_m").value
                )
                displacement = float(direction @ (current - anchor))
                recovery = Twist()
                if displacement < maximum:
                    velocity = direction * float(
                        self.get_parameter("integrity_recovery_speed_mps").value
                    )
                    recovery.linear.x, recovery.linear.y, recovery.linear.z = velocity.tolist()
                # This command is independent of the rejected trajectory. It is
                # bounded and points away from the closest static surface.  It
                # both increases AL along the weak axis and generates new LiDAR
                # constraints before the next immutable candidate batch.
                self.command_publisher.publish(recovery)
        if not rejected:
            self._p13_rejected_since_s = None
            return
        if self._p13_rejected_since_s is None:
            self._p13_rejected_since_s = now_s
            return
        if now_s - self._p13_rejected_since_s < float(
            self.get_parameter("rejected_candidate_retry_s").value
        ):
            return
        maximum = int(self.get_parameter("maximum_candidate_retries").value)
        if self._p13_candidate_retry_count >= maximum:
            return
        self._p13_candidate_retry_count += 1
        self._candidates = []
        self._metadata = None
        self._decision = None
        self._selected = None
        self._unconstrained = None
        self._trajectory_start_sim_s = None
        self._p13_rejected_since_s = None
        self.get_logger().warning(
            f"P13 hard rejection retained; hover and safely resample candidate "
            f"set ({self._p13_candidate_retry_count}/{maximum})"
        )

    @staticmethod
    def _minimum_excitation_direction(
        *,
        current_y: float,
        anchor_y: float,
        elapsed_s: float,
        maximum_offset_m: float,
        half_period_s: float,
    ) -> float:
        """Return a bounded oscillation direction without consulting Ground Truth."""
        direction = -1.0 if int(max(0.0, elapsed_s) / half_period_s) % 2 == 0 else 1.0
        if current_y <= anchor_y - maximum_offset_m:
            return 1.0
        if current_y >= anchor_y + maximum_offset_m:
            return -1.0
        return direction

    def _odom_cb(self, message) -> None:
        super()._odom_cb(message)
        odom_stamp_ns = _stamp_ns(message.header.stamp)
        completed_ns = time.perf_counter_ns()
        for sensor_stamp_ns, pipeline in self._p13_pipelines.items():
            if (
                not pipeline["localization_done_steady_ns"]
                and localization_snapshot_covers_sensor(
                    sensor_stamp_ns, odom_stamp_ns
                )
            ):
                pipeline["localization_done_steady_ns"] = completed_ns
                if pipeline["map_arrival_steady_ns"]:
                    pipeline["map_done_steady_ns"] = max(
                        completed_ns, int(pipeline["map_arrival_steady_ns"])
                    )

    def _map_status_cb(self, message: String) -> None:
        super()._map_status_cb(message)
        source_sensor_stamp_ns = int(
            self._map_status.get("source_sensor_stamp_ns", 0)
        )
        pipeline = self._p13_pipelines.get(source_sensor_stamp_ns)
        if pipeline is None or pipeline["map_arrival_steady_ns"]:
            return
        arrived_ns = time.perf_counter_ns()
        pipeline["map_arrival_steady_ns"] = arrived_ns
        localization_done_ns, map_done_ns = ordered_map_dependency_completion(
            arrived_ns, int(pipeline["localization_done_steady_ns"])
        )
        pipeline["localization_done_steady_ns"] = localization_done_ns
        pipeline["map_done_steady_ns"] = map_done_ns

    def _current_stats(self):
        return summarize_latencies(self._p13_latency_samples_s)

    def _publish_p13_status(self, phase: str, elapsed_s: float, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._p13_last_status_wall_s < 1.0:
            return
        stats = self._current_stats()
        latency_for_status = (
            stats.p99_s if math.isfinite(stats.p99_s) else self._fallback_latency_s()
        )
        unmitigated_alert, unmitigated_margin = self._unmitigated_limits(
            latency_for_status
        )
        payload = {
            "profile": str(self.get_parameter("latency_profile").value),
            "phase": phase,
            "elapsed_s": elapsed_s,
            "planner_delay_ms": float(self.get_parameter("planner_delay_ms").value),
            "latency_sample_count": stats.count,
            "latency_p50_ms": 1000.0 * stats.p50_s if math.isfinite(stats.p50_s) else None,
            "latency_p95_ms": 1000.0 * stats.p95_s if math.isfinite(stats.p95_s) else None,
            "latency_p99_ms": 1000.0 * stats.p99_s if math.isfinite(stats.p99_s) else None,
            "latency_max_ms": 1000.0 * stats.maximum_s if math.isfinite(stats.maximum_s) else None,
            "latency_radius_m": self._p13_envelope.latency_radius_m,
            "geometric_clearance_m": float(
                self.get_parameter("geometric_clearance_m").value
            ),
            "alert_limit_m": self._p13_envelope.alert_limit_m,
            "protection_level_m": float(self.get_parameter("protection_level_m").value),
            "integrity_margin_m": self._p13_envelope.integrity_margin_m,
            "unmitigated_alert_limit_m": unmitigated_alert,
            "unmitigated_integrity_margin_m": unmitigated_margin,
            "required_margin_m": float(self.get_parameter("required_margin_m").value),
            "speed_limit_mps": self._p13_speed_limit_mps,
            "nominal_speed_limit_mps": self._p13_nominal_maximum_speed_mps,
            "current_position_x_m": (
                float(self._position(self._odom)[0]) if self._odom is not None else None
            ),
            "mission_goal_x_m": self._mission_goal_x_m,
            "finished": self._finished,
            "ground_truth_subscribed": False,
            "p99_used_for_safety": True,
            "candidate_retry_count": self._p13_candidate_retry_count,
            "integrity_recovery_active": self._p13_integrity_recovery_active,
            "integrity_recovery_anchor_xyz_m": (
                self._p13_integrity_recovery_anchor.tolist()
                if self._p13_integrity_recovery_anchor is not None
                else None
            ),
            "integrity_recovery_direction": (
                self._p13_integrity_recovery_direction.tolist()
                if self._p13_integrity_recovery_direction is not None
                else None
            ),
        }
        output = String()
        output.data = json.dumps(payload, separators=(",", ":"))
        self.p13_status_publisher.publish(output)
        self._p13_last_status_wall_s = now

    def _publish_status(self, phase: str, elapsed_s: float) -> None:
        super()._publish_status(phase, elapsed_s)
        self._publish_p13_status(phase, elapsed_s, force=self._finished)

    def _process_latency_cycle(self) -> None:
        if self._p13_planner_future is None:
            ready = [
                pipeline
                for pipeline in self._p13_pipelines.values()
                if int(pipeline["seq"]) > self._p13_processed_seq
                and pipeline["localization_done_steady_ns"]
                and pipeline["map_done_steady_ns"]
            ]
            if not ready:
                return

            pipeline = dict(max(ready, key=lambda item: int(item["seq"])))
            seq = int(pipeline["seq"])
            self._p13_processed_seq = seq
            self._p13_pipelines = {
                stamp: item
                for stamp, item in self._p13_pipelines.items()
                if int(item["seq"]) > seq
            }
            self._p13_pending_pipeline = pipeline
            self._p13_planner_trigger_perf_ns = time.perf_counter_ns()
            delay_s = 1.0e-3 * float(self.get_parameter("planner_delay_ms").value)
            self._p13_planner_future = self._p13_planner_executor.submit(
                self._complete_planner_delay, delay_s
            )
            return

        if not self._p13_planner_future.done():
            return

        assert self._p13_pending_pipeline is not None
        pipeline = self._p13_pending_pipeline
        seq = int(pipeline["seq"])
        planner_trigger_perf_ns = self._p13_planner_trigger_perf_ns
        planner_done_perf_ns = self._p13_planner_future.result()
        self._p13_pending_pipeline = None
        self._p13_planner_future = None
        self._p13_planner_trigger_perf_ns = 0

        stats_before = self._current_stats()
        latency_for_safety = (
            stats_before.p99_s if math.isfinite(stats_before.p99_s) else self._fallback_latency_s()
        )
        self._p13_envelope = self._calculate_envelope(latency_for_safety)
        self._p13_speed_limit_mps = self._p13_envelope.speed_limit_mps
        trajectory_certified_steady_ns = time.perf_counter_ns()
        super()._timer()
        self._retry_rejected_candidate_set()
        command_sent_perf_ns = time.perf_counter_ns()

        sensor_age_s = 1.0e-9 * float(pipeline["sensor_age_at_receive_ns"])
        receive_to_command_s = 1.0e-9 * (
            command_sent_perf_ns - int(pipeline["receive_perf_ns"])
        )
        end_to_end_s = sensor_age_s + max(0.0, receive_to_command_s)
        self._p13_latency_samples_s.append(end_to_end_s)
        stats = self._current_stats()
        self._p13_envelope = self._calculate_envelope(stats.p99_s)
        self._p13_speed_limit_mps = self._p13_envelope.speed_limit_mps
        unmitigated_alert, unmitigated_margin = self._unmitigated_limits(stats.p99_s)
        self._p13_trace_seq += 1
        trace = {
            "seq": self._p13_trace_seq,
            "sensor_seq": seq,
            "profile": str(self.get_parameter("latency_profile").value),
            "clock_domains": {
                "sensor_timestamp": "ros_sim_time",
                "processing_timestamps": "steady_time",
                "latency_duration": "steady_time_plus_sensor_age",
            },
            "sensor_timestamp_ns": int(pipeline["sensor_sim_ns"]),
            "receive_timestamp_ns": int(pipeline["receive_steady_ns"]),
            "localization_done_ns": int(pipeline["localization_done_steady_ns"]),
            "map_done_ns": int(pipeline["map_done_steady_ns"]),
            "planner_trigger_ns": planner_trigger_perf_ns,
            "planner_done_ns": planner_done_perf_ns,
            "trajectory_certified_ns": trajectory_certified_steady_ns,
            "command_sent_ns": command_sent_perf_ns,
            "sensor_age_at_receive_ms": 1000.0 * sensor_age_s,
            "planner_processing_ms": 1.0e-6 * (
                planner_done_perf_ns - planner_trigger_perf_ns
            ),
            "receive_to_command_ms": 1000.0 * receive_to_command_s,
            "end_to_end_latency_ms": 1000.0 * end_to_end_s,
            "p50_ms": 1000.0 * stats.p50_s,
            "p95_ms": 1000.0 * stats.p95_s,
            "p99_ms": 1000.0 * stats.p99_s,
            "max_ms": 1000.0 * stats.maximum_s,
            "latency_radius_m": self._p13_envelope.latency_radius_m,
            "geometric_clearance_m": float(
                self.get_parameter("geometric_clearance_m").value
            ),
            "alert_limit_m": self._p13_envelope.alert_limit_m,
            "protection_level_m": float(self.get_parameter("protection_level_m").value),
            "integrity_margin_m": self._p13_envelope.integrity_margin_m,
            "unmitigated_alert_limit_m": unmitigated_alert,
            "unmitigated_integrity_margin_m": unmitigated_margin,
            "required_margin_m": float(self.get_parameter("required_margin_m").value),
            "speed_limit_mps": self._p13_speed_limit_mps,
            "p99_used_for_safety": True,
            "ground_truth_used": False,
        }
        message = String()
        message.data = json.dumps(trace, separators=(",", ":"))
        self.p13_trace_publisher.publish(message)

    def _timer(self) -> None:
        super()._timer()
        self._retry_rejected_candidate_set()

    def destroy_node(self):
        self._p13_planner_executor.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P13FlightControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            from geometry_msgs.msg import Twist

            for _ in range(3):
                node.command_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
