"""Execute the P10 three-arm benchmark with a shared trajectory contract."""

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
from xq_sim_interfaces.msg import ActivePerceptionDecision, DirectionalIntegrity, InformationMap

from .alert_limit import sample_bspline


class P10FlightControllerNode(Node):
    VARIANTS = ("baseline", "yaw_only", "minimum_excitation")

    def __init__(self) -> None:
        super().__init__("xq_p10_flight_controller")
        defaults = {
            "variant": "baseline",
            "initialization_s": 12.0,
            "trajectory_duration_s": 28.0,
            "trajectory_distance_m": 7.5,
            "control_point_count": 21,
            "position_gain": 0.80,
            "maximum_speed_mps": 0.42,
            "maximum_yaw_rate_rps": 0.25,
            "settle_s": 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.variant = str(self.get_parameter("variant").value)
        if self.variant not in self.VARIANTS:
            raise ValueError(f"unsupported P10 flight variant: {self.variant}")
        self._odom: Odometry | None = None
        self._integrity: DirectionalIntegrity | None = None
        self._information_map: InformationMap | None = None
        self._decision: ActivePerceptionDecision | None = None
        self._selected: Bspline | None = None
        self._baseline: Bspline | None = None
        self._start_sim_s: float | None = None
        self._trajectory_start_sim_s: float | None = None
        self._initial_position: np.ndarray | None = None
        self._last_position: np.ndarray | None = None
        self._path_length_m = 0.0
        self._finished = False
        self._last_phase = ""

        state_qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
        latched_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.command_publisher = self.create_publisher(Twist, "/xq/p3/cmd_vel", 20)
        self.baseline_publisher = self.create_publisher(
            Bspline, "/planning/p10/baseline_bspline", state_qos
        )
        self.status_publisher = self.create_publisher(String, "/xq/p10/flight_status", latched_qos)
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, state_qos)
        self.create_subscription(
            DirectionalIntegrity, "/integrity/directional", self._integrity_cb, state_qos
        )
        self.create_subscription(
            InformationMap, "/integrity/information_map", self._information_map_cb, state_qos
        )
        self.create_subscription(
            ActivePerceptionDecision,
            "/integrity/active_perception_decision",
            self._decision_cb,
            latched_qos,
        )
        self.create_subscription(
            Bspline, "/planning/active_perception_bspline", self._selected_cb, latched_qos
        )
        self.create_timer(0.05, self._timer)
        self.get_logger().info(
            f"P10 flight controller variant={self.variant}; Minimum-Excitation is fail-closed"
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

    def _decision_cb(self, message: ActivePerceptionDecision) -> None:
        if self._baseline is not None and message.baseline_trajectory_id == self._baseline.traj_id:
            self._decision = message

    def _selected_cb(self, message: Bspline) -> None:
        if self._decision is not None and message.traj_id == self._decision.selected_trajectory_id:
            self._selected = message

    def _ready(self) -> bool:
        return (
            self._odom is not None
            and self._integrity is not None
            and self._information_map is not None
            and self._information_map.valid
            and not self._information_map.ground_truth_used
        )

    def _make_baseline(self, now_s: float) -> Bspline:
        assert self._odom is not None
        start = self._position(self._odom)
        count = int(self.get_parameter("control_point_count").value)
        distance = self._float("trajectory_distance_m")
        duration = self._float("trajectory_duration_s")
        positions = start + np.column_stack(
            (np.linspace(0.0, distance, count), np.zeros(count), np.zeros(count))
        )
        message = Bspline()
        message.order = 1
        message.traj_id = 20261001
        message.start_time = self.get_clock().now().to_msg()
        message.pos_pts = [
            Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            for point in positions
        ]
        interior = [duration * index / float(count - 1) for index in range(1, count - 1)]
        message.knots = [0.0, 0.0, *interior, duration, duration]
        message.yaw_pts = np.zeros(count).tolist()
        message.yaw_dt = duration / float(count - 1)
        self._initial_position = start
        self._trajectory_start_sim_s = now_s
        return message

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
                control, knots, int(message.order), max(duration, 1.0), minimum_parameter_s=t1
            )[0]
        )
        feedforward = (ahead - desired) / max(t1 - t0, 0.10)
        return desired, feedforward

    def _publish_status(self, phase: str, elapsed_s: float) -> None:
        if phase == self._last_phase:
            return
        message = String()
        message.data = json.dumps(
            {
                "variant": self.variant,
                "phase": phase,
                "elapsed_s": elapsed_s,
                "path_length_m": self._path_length_m,
                "baseline_published": self._baseline is not None,
                "decision_valid": bool(self._decision is not None and self._decision.valid),
                "selected_name": self._decision.selected_name if self._decision is not None else "",
                "selected_applied": self.variant == "minimum_excitation" and self._selected is not None,
                "finished": self._finished,
                "ground_truth_subscribed": False,
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(message)
        if phase != self._last_phase:
            self.get_logger().info(f"P10 flight phase={phase} t={elapsed_s:.2f}s")
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
        if self._baseline is None:
            self._baseline = self._make_baseline(now_s)
            self.baseline_publisher.publish(self._baseline)
            self._publish_status("BASELINE_PUBLISHED", elapsed_from_node)
            return
        # Republish until the selector has produced a decision.  The trajectory
        # ID is unchanged, so the selector still evaluates it exactly once.
        if self._decision is None:
            self.baseline_publisher.publish(self._baseline)

        trajectory = self._baseline
        if self.variant == "minimum_excitation":
            if self._decision is None or not self._decision.valid:
                self.command_publisher.publish(command)
                self._publish_status("WAIT_HARD_DECISION", elapsed_from_node)
                return
            if not self._decision.recovery_found or self._decision.selected_name == "":
                self.command_publisher.publish(command)
                self._publish_status("FAIL_CLOSED_NO_RECOVERY", elapsed_from_node)
                return
            if self._selected is None:
                self.command_publisher.publish(command)
                self._publish_status("WAIT_SELECTED_TRAJECTORY", elapsed_from_node)
                return
            trajectory = self._selected

        assert self._trajectory_start_sim_s is not None and self._odom is not None
        elapsed = now_s - self._trajectory_start_sim_s
        duration = float(
            trajectory.knots[-int(trajectory.order) - 1] - trajectory.knots[int(trajectory.order)]
        )
        if elapsed >= duration + self._float("settle_s"):
            self._finished = True
            self.command_publisher.publish(command)
            self._publish_status("COMPLETE", elapsed_from_node)
            return
        desired, feedforward = self._trajectory_positions(trajectory, elapsed)
        error = desired - self._position(self._odom)
        velocity = feedforward + self._float("position_gain") * error
        speed = float(np.linalg.norm(velocity))
        maximum = self._float("maximum_speed_mps")
        if speed > maximum:
            velocity *= maximum / speed
        command.linear.x, command.linear.y, command.linear.z = velocity.tolist()
        if self.variant == "yaw_only" and 0.0 <= elapsed <= duration:
            command.angular.z = self._float("maximum_yaw_rate_rps") * math.sin(
                2.0 * math.pi * elapsed / 6.0
            )
        self.command_publisher.publish(command)
        self._publish_status("EXECUTE", elapsed_from_node)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P10FlightControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stop = Twist()
        for _ in range(3):
            node.command_publisher.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
