"""P12 controller: P11 rolling exploration guarded by dynamic LiDAR occupancy."""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from xq_sim_interfaces.msg import ReplanEvent

from .dynamic_planning import (
    DynamicPassageGate,
    resample_polyline,
    supported_polyline_obstruction,
)
from .p11_flight_controller_node import P11FlightControllerNode
from .p12_dynamic_map_node import _cloud_xyz


class P12FlightControllerNode(P11FlightControllerNode):
    def __init__(self) -> None:
        super().__init__()
        additions = {
            "path_clearance_radius_m": 0.70,
            "planning_lookahead_m": 4.0,
            "clear_confirmation_s": 1.0,
            # The map publishes at 5 Hz. Software-rendered Gazebo can deliver
            # callbacks in bursts, so retain seven publication periods while
            # still rejecting genuinely stale obstacle state.
            "dynamic_cloud_timeout_s": 1.50,
            "minimum_dynamic_range_m": 0.75,
            # Legacy phases retain the original +X corridor.  P15 opts into
            # the commanded 3-D B-spline query explicitly.
            "dynamic_path_query_mode": "forward_axis",
            "dynamic_path_query_max_points": 16,
            "minimum_dynamic_cluster_points": 1,
            "dynamic_cluster_radius_m": 0.45,
        }
        for name, value in additions.items():
            self.declare_parameter(name, value)
        if str(self.get_parameter("dynamic_path_query_mode").value) not in {
            "forward_axis",
            "active_trajectory",
        }:
            raise ValueError("unsupported dynamic_path_query_mode")
        self._dynamic_points = np.empty((0, 3), dtype=np.float64)
        self._dynamic_stamp_s = -math.inf
        self._dynamic_stamp_msg = Time()
        self._map_status: dict[str, object] = {}
        self._map_status_stamp_s = -math.inf
        self._passage_gate = DynamicPassageGate(self._float("clear_confirmation_s"))
        self._dynamic_confirmed = False
        self._passage_reopened = False
        self._replan_count = 0
        self._dynamic_brake_duration_s = 0.0
        self._last_guard_s: float | None = None
        self._nearest_dynamic_range_m = math.inf
        self._dynamic_support_count = 0
        self._last_p12_phase = ""
        self._dynamic_callbacks = 0
        self._last_guard_debug_s = -math.inf
        self._last_odom_message_s = -math.inf
        latched = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.p12_status_publisher = self.create_publisher(
            String, "/xq/p12/flight_status", latched
        )
        self.replan_publisher = self.create_publisher(
            ReplanEvent, "/planning/p12/replan_event", latched
        )
        self.create_subscription(
            PointCloud2,
            "/mapping/p12/dynamic_voxels",
            self._dynamic_cb,
            latched,
        )
        self.create_subscription(String, "/mapping/p12/status", self._map_status_cb, latched)
        # DDS startup occasionally left the inherited RELIABLE odometry reader
        # undiscovered while sibling readers were healthy. A BEST_EFFORT reader
        # is compatible with both publisher policies; timestamp deduplication in
        # _odom_cb guarantees that dual delivery never doubles path integration.
        fallback_odom_qos = QoSProfile(
            depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._odom_fallback_subscription = self.create_subscription(
            Odometry,
            "/localization/odom",
            self._odom_cb,
            fallback_odom_qos,
        )
        self.get_logger().info(
            "P12 controller extends P11; only dynamic voxel map may trigger replanning"
        )

    @staticmethod
    def _stamp_s(stamp: Time) -> float:
        return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)

    def _odom_cb(self, message: Odometry) -> None:
        stamp_s = self._stamp_s(message.header.stamp)
        if stamp_s <= self._last_odom_message_s + 1.0e-9:
            return
        self._last_odom_message_s = stamp_s
        super()._odom_cb(message)

    def _dynamic_cb(self, message: PointCloud2) -> None:
        self._dynamic_points = _cloud_xyz(message)
        self._dynamic_stamp_s = self._stamp_s(message.header.stamp)
        self._dynamic_stamp_msg = message.header.stamp
        self._dynamic_callbacks += 1

    def _map_status_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if payload.get("ground_truth_used") is not False:
            return
        self._map_status = payload
        self._map_status_stamp_s = float(payload.get("stamp_s", -math.inf))

    def _publish_replan(
        self,
        reason: str,
        *,
        brake: bool,
        use_dynamic_stamp: bool = True,
        outcome: str | None = None,
    ) -> None:
        now = self.get_clock().now().to_msg()
        event = ReplanEvent()
        event.header.stamp = now
        event.header.frame_id = "xq_lio_map"
        self._replan_count += 1
        event.seq = self._replan_count
        event.trigger_reason = reason
        event.trigger_stamp = (
            self._dynamic_stamp_msg if brake and use_dynamic_stamp else now
        )
        event.map_ready_stamp = now
        event.optimizer_start_stamp = now
        event.candidate_ready_stamp = now
        event.safety_pass_stamp = now
        event.accepted_stamp = now
        event.latency_ms = max(
            0.0,
            1000.0 * (self._now_s() - self._dynamic_stamp_s)
            if brake and use_dynamic_stamp
            else 0.0,
        )
        event.accepted = True
        event.brake_fallback = brake
        event.outcome = outcome or (
            "BRAKE_ACCEPTED" if brake else "CORRIDOR_TRAJECTORY_REACCEPTED"
        )
        self.replan_publisher.publish(event)

    def _dynamic_query_path(self, position: np.ndarray) -> np.ndarray:
        """Build the remaining commanded path used by the safety guard."""
        lookahead = self._float("planning_lookahead_m")
        fallback = np.stack(
            (position, position + np.asarray((lookahead, 0.0, 0.0)))
        )
        if str(self.get_parameter("dynamic_path_query_mode").value) != (
            "active_trajectory"
        ):
            return fallback
        trajectory = (
            self._unconstrained
            if self.variant == "information_only"
            else self._selected
        )
        if trajectory is None or len(trajectory.pos_pts) < 2:
            return fallback
        points = np.asarray(
            [(point.x, point.y, point.z) for point in trajectory.pos_pts],
            dtype=float,
        )
        if not np.isfinite(points).all():
            return fallback
        closest = int(np.argmin(np.linalg.norm(points - position, axis=1)))
        path = np.vstack((position, points[closest:]))
        keep = np.concatenate(
            ([True], np.linalg.norm(np.diff(path, axis=0), axis=1) > 1.0e-6)
        )
        path = path[keep]
        return (
            resample_polyline(
                path, int(self.get_parameter("dynamic_path_query_max_points").value)
            )
            if len(path) >= 2
            else fallback
        )

    def _guard(self, now_s: float) -> tuple[bool, str]:
        cloud_age_s = now_s - self._dynamic_stamp_s
        if self._odom is None or cloud_age_s > self._float(
            "dynamic_cloud_timeout_s"
        ):
            self._nearest_dynamic_range_m = math.inf
            self._dynamic_support_count = 0
            if now_s - self._last_guard_debug_s >= 2.0:
                self.get_logger().info(
                    f"P12 guard fresh=false callbacks={self._dynamic_callbacks} "
                    f"cloud_age={cloud_age_s:.3f}s points={len(self._dynamic_points)}"
                )
                self._last_guard_debug_s = now_s
            if self._dynamic_confirmed and not self._passage_reopened:
                return True, "BRAKE_DYNAMIC_MAP_STALE"
            return False, "NO_FRESH_DYNAMIC_OCCUPANCY"
        position = self._position(self._odom)
        query_path = self._dynamic_query_path(position)
        # These are already world-frame, temporally confirmed map voxels, not
        # raw LiDAR returns.  Never reapply the sensor blind-range filter here:
        # a voxel observed earlier must remain capable of braking when the
        # vehicle gets closer than the LiDAR's minimum range.
        query_points = self._dynamic_points
        blocked, distance, support_count = supported_polyline_obstruction(
            query_points,
            query_path,
            clearance_radius_m=self._float("path_clearance_radius_m"),
            lookahead_m=self._float("planning_lookahead_m"),
            minimum_support_points=int(
                self.get_parameter("minimum_dynamic_cluster_points").value
            ),
            support_radius_m=self._float("dynamic_cluster_radius_m"),
        )
        status_fresh = now_s - self._map_status_stamp_s <= self._float(
            "dynamic_cloud_timeout_s"
        )
        if status_fresh and self._map_status.get("forward_path_blocked") is True:
            blocked = True
            status_distance = self._map_status.get("nearest_forward_dynamic_range_m")
            if status_distance is not None:
                distance = min(distance, float(status_distance))
        if status_fresh:
            support_count = max(
                support_count,
                int(self._map_status.get("forward_path_support_count", 0)),
            )
        self._dynamic_support_count = support_count
        self._nearest_dynamic_range_m = distance
        if now_s - self._last_guard_debug_s >= 2.0:
            self.get_logger().info(
                f"P12 guard fresh=true callbacks={self._dynamic_callbacks} "
                f"cloud_age={cloud_age_s:.3f}s points={len(self._dynamic_points)} "
                f"map_status_fresh={str(status_fresh).lower()} "
                f"map_blocked={str(self._map_status.get('forward_path_blocked') is True).lower()} "
                f"raw_blocked={str(self._map_status.get('raw_path_blocked') is True).lower()} "
                f"support={support_count}/"
                f"{int(self.get_parameter('minimum_dynamic_cluster_points').value)} "
                f"blocked={str(blocked).lower()} nearest="
                f"{distance if math.isfinite(distance) else -1.0:.3f}m"
            )
            self._last_guard_debug_s = now_s
        decision = self._passage_gate.update(blocked, now_s)
        if decision.obstacle_confirmed and not self._dynamic_confirmed:
            self._dynamic_confirmed = True
            self._publish_replan("DYNAMIC_OCCUPANCY_CONFIRMED", brake=True)
        if decision.passage_reopened:
            self._passage_reopened = True
            self._publish_replan("DYNAMIC_TTL_CLEARED", brake=False)
        return decision.brake, decision.state

    def _publish_status(self, phase: str, elapsed_s: float) -> None:
        if phase == self._last_p12_phase:
            return
        executed_name = ""
        if self._decision is not None:
            executed_name = (
                self._decision.unconstrained_selected_name
                if self.variant == "information_only"
                else self._decision.selected_name
            )
        position = self._position(self._odom) if self._odom is not None else None
        payload = {
            "variant": self.variant,
            "phase": phase,
            "elapsed_s": elapsed_s,
            "path_length_m": self._path_length_m,
            "mission_start_x_m": self._mission_start_x_m,
            "mission_goal_x_m": self._mission_goal_x_m,
            "mission_distance_m": self._float("mission_distance_m"),
            "current_position_x_m": float(position[0]) if position is not None else None,
            "current_position_y_m": float(position[1]) if position is not None else None,
            "current_position_z_m": float(position[2]) if position is not None else None,
            "segment_index": self._segment_index,
            "segments_completed": self._segments_completed,
            "planning_windows_closed": self._planning_windows_closed,
            "interrupted_decisions": self._interrupted_decisions,
            "decisions_applied": self._decisions_applied,
            "rolling_horizon": True,
            "selected_applied": bool(
                self._selected is not None
                if self.variant == "integrity_constrained"
                else self._unconstrained is not None
            ),
            "executed_name": executed_name,
            "dynamic_obstacle_confirmed": self._dynamic_confirmed,
            "passage_reopened": self._passage_reopened,
            "replan_event_count": self._replan_count,
            "dynamic_brake_duration_s": self._dynamic_brake_duration_s,
            "nearest_dynamic_range_m": (
                self._nearest_dynamic_range_m
                if math.isfinite(self._nearest_dynamic_range_m)
                else None
            ),
            "dynamic_support_count": self._dynamic_support_count,
            "minimum_dynamic_cluster_points": int(
                self.get_parameter("minimum_dynamic_cluster_points").value
            ),
            "dynamic_cluster_radius_m": self._float("dynamic_cluster_radius_m"),
            "dynamic_path_query_mode": str(
                self.get_parameter("dynamic_path_query_mode").value
            ),
            "finished": self._finished,
            "ground_truth_subscribed": False,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.p12_status_publisher.publish(message)
        self.get_logger().info(f"P12 flight phase={phase} t={elapsed_s:.2f}s")
        self._last_p12_phase = phase

    def _timer(self) -> None:
        now_s = self._now_s()
        if now_s <= 0.0:
            return
        delta = 0.0 if self._last_guard_s is None else max(0.0, now_s - self._last_guard_s)
        self._last_guard_s = now_s
        initialization_complete = bool(
            self._start_sim_s is not None
            and now_s - self._start_sim_s >= self._float("initialization_s")
        )
        brake, guard_state = self._guard(now_s)
        if brake and initialization_complete and not self._finished:
            if self._trajectory_start_sim_s is not None:
                self._trajectory_start_sim_s += delta
            if self._last_terminal_progress_s is not None:
                self._last_terminal_progress_s += delta
            self._dynamic_brake_duration_s += delta
            self.command_publisher.publish(Twist())
            elapsed = now_s - self._start_sim_s if self._start_sim_s is not None else 0.0
            self._publish_status(guard_state, elapsed)
            return
        super()._timer()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P12FlightControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.command_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
