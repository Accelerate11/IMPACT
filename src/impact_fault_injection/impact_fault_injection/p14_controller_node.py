"""P14 resilient controller layered over the accepted P13 flight stack."""

from __future__ import annotations

import json
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from xq_sim_interfaces.msg import FaultEvent

from xq_autonomy.p13_flight_controller_node import P13FlightControllerNode

from .fault_model import ActiveFaultSet, FaultSpec
from .supervisor import ResilientSupervisor, SafetyDecision, SafetyMode


class P14ControllerNode(P13FlightControllerNode):
    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("p14_trial", "matrix")
        self.declare_parameter("cautious_speed_scale", 0.55)
        self.declare_parameter("recovery_speed_scale", 0.32)
        self.declare_parameter("return_speed_mps", 0.20)
        self.declare_parameter("landing_speed_mps", 0.30)
        self.declare_parameter("required_landing_descent_m", 0.50)
        # These recovery parameters are owned by the normal P13 controller and
        # inherited here. P14 only adds fault-supervisor policy around them.
        self.faults = ActiveFaultSet()
        self.supervisor = ResilientSupervisor()
        self._p14_decision = SafetyDecision(SafetyMode.NORMAL, "initializing")
        self._p14_manual_override = False
        self._p14_last_timer_s: float | None = None
        self._p14_last_status_s = -math.inf
        self._p14_last_mode: str | None = None
        self._p14_state_history: list[dict[str, object]] = []
        self._p14_observed_events: list[dict[str, object]] = []
        self._p14_planner_delay_ids: set[str] = set()
        self._p14_actual_planner_delays = 0
        self._p14_cpu_work_cycles = 0
        self._p14_complete = False
        self._p14_landed = False
        self._p14_commanded_landing_descent_m = 0.0
        self._p14_initial_z_m: float | None = None
        self._p14_integrity_recovery_active = False
        self._p14_integrity_recovery_anchor_y: float | None = None
        self._p14_integrity_recovery_start_s: float | None = None
        latched = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.p14_status_publisher = self.create_publisher(
            String, "/impact/p14/safety_status", latched
        )
        self.create_subscription(FaultEvent, "/impact/fault_event", self._fault_cb, latched)
        self.create_subscription(Bool, "/impact/manual_override", self._manual_cb, latched)
        self.get_logger().info(
            "P14 supervisor active: safety state has priority over exploration utility"
        )

    def _float(self, name: str) -> float:
        value = super()._float(name)
        if name == "maximum_speed_mps" and hasattr(self, "_p14_decision"):
            if self._p14_decision.mode == SafetyMode.CAUTIOUS:
                return value * float(self.get_parameter("cautious_speed_scale").value)
            if self._p14_decision.mode == SafetyMode.RECOVERY:
                return value * float(self.get_parameter("recovery_speed_scale").value)
        return value

    def _fault_cb(self, message: FaultEvent) -> None:
        now_s = self._now_s()
        spec = FaultSpec(
            fault_id=message.fault_id,
            fault_type=message.target_module,
            start_time_s=0.0,
            duration_s=float(message.duration_s),
            severity=float(message.severity),
            seed=int(message.seed),
        )
        self.faults.add(spec, now_s)
        self._p14_observed_events.append({
            "fault_id": spec.fault_id,
            "fault_type": spec.fault_type,
            "start_s": now_s,
            "duration_s": spec.duration_s,
            "severity": spec.severity,
            "seed": spec.seed,
        })
        if spec.fault_type == "planner_delay" and spec.fault_id not in self._p14_planner_delay_ids:
            self._p14_planner_delay_ids.add(spec.fault_id)
            # This is a real bounded callback stall, not a metadata-only flag.
            time.sleep(max(0.0, min(spec.severity, 0.50)))
            self._p14_actual_planner_delays += 1

    def _manual_cb(self, message: Bool) -> None:
        self._p14_manual_override = bool(message.data)

    def _pause_trajectory_clock(self, delta_s: float) -> None:
        if self._trajectory_start_sim_s is not None:
            self._trajectory_start_sim_s += max(0.0, delta_s)

    def _position_xyz(self) -> tuple[float | None, float | None, float | None]:
        if self._odom is None:
            return None, None, None
        point = self._odom.pose.pose.position
        return float(point.x), float(point.y), float(point.z)

    def _publish_p14_status(self, now_s: float, *, force: bool = False) -> None:
        if not force and now_s - self._p14_last_status_s < 0.10:
            return
        x, y, z = self._position_xyz()
        mode = self._p14_decision.mode.value
        if mode != self._p14_last_mode:
            self._p14_state_history.append({
                "stamp_s": now_s, "mode": mode, "reason": self._p14_decision.reason,
            })
            self._p14_last_mode = mode
            self.get_logger().warning(
                f"P14 safety transition -> {mode}: {self._p14_decision.reason}"
            )
        payload = {
            "trial": str(self.get_parameter("p14_trial").value),
            "stamp_s": now_s,
            "mode": mode,
            "reason": self._p14_decision.reason,
            "active_fault_ids": self.faults.ids(now_s),
            "geometry_only": self._p14_decision.geometry_only,
            "essential_only": self._p14_decision.essential_only,
            "mission_continue": self._p14_decision.mission_continue,
            "manual_override": self._p14_manual_override,
            "current_position_x_m": x,
            "current_position_y_m": y,
            "current_position_z_m": z,
            "initial_altitude_m": self._p14_initial_z_m,
            "mission_goal_x_m": self._mission_goal_x_m,
            "base_flight_finished": self._finished,
            "landed": self._p14_landed,
            "commanded_landing_descent_m": self._p14_commanded_landing_descent_m,
            "trial_complete": self._p14_complete,
            "observed_fault_events": list(self._p14_observed_events),
            "state_history": list(self._p14_state_history),
            "actual_planner_delay_count": self._p14_actual_planner_delays,
            "cpu_work_cycles": self._p14_cpu_work_cycles,
            "integrity_recovery_active": self._p14_integrity_recovery_active,
            "ground_truth_subscribed": False,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.p14_status_publisher.publish(message)
        self._p14_last_status_s = now_s

    def _timer(self) -> None:
        now_s = self._now_s()
        if now_s <= 0.0:
            return
        delta_s = 0.0 if self._p14_last_timer_s is None else max(
            0.0, now_s - self._p14_last_timer_s
        )
        self._p14_last_timer_s = now_s
        if self._odom is not None and self._p14_initial_z_m is None:
            self._p14_initial_z_m = float(self._odom.pose.pose.position.z)

        self._p14_decision = self.supervisor.evaluate(
            now_s, self.faults, manual_override=self._p14_manual_override
        )
        hard_rejected = bool(
            self._decision is not None
            and self._decision.valid
            and int(self._decision.selected_trajectory_id) < 0
            and self._selected is None
        )
        recovery_permitted = bool(
            hard_rejected
            and not self.faults.ids(now_s)
            and self._passage_reopened
            and not self._finished
        )
        if recovery_permitted:
            self._p14_integrity_recovery_active = True
            if self._p14_integrity_recovery_anchor_y is None:
                _, y, _ = self._position_xyz()
                self._p14_integrity_recovery_anchor_y = y
                self._p14_integrity_recovery_start_s = now_s
        if self._p14_integrity_recovery_active and self._selected is not None:
            self._p14_integrity_recovery_active = False
            self._p14_integrity_recovery_anchor_y = None
            self._p14_integrity_recovery_start_s = None
        if self._p14_integrity_recovery_active and self._p14_decision.mode == SafetyMode.NORMAL:
            self._p14_decision = SafetyDecision(
                SafetyMode.RECOVERY,
                "integrity_hard_rejection_minimum_excitation",
                mission_continue=False,
            )
        mode = self._p14_decision.mode
        if self._p14_complete:
            self.command_publisher.publish(Twist())
            self._publish_p14_status(now_s, force=True)
            return

        if self.faults.has(now_s, "cpu_load"):
            # Bounded deterministic computation exercises the same executor without
            # launching an uncontrolled host stress process.
            deadline = time.perf_counter() + 0.006
            accumulator = 0.0
            while time.perf_counter() < deadline:
                accumulator = math.sin(accumulator + 0.12345)
            self._p14_cpu_work_cycles += 1

        if mode in (SafetyMode.BRAKE, SafetyMode.HOVER, SafetyMode.MANUAL):
            self._pause_trajectory_clock(delta_s)
            self.command_publisher.publish(Twist())
        elif mode == SafetyMode.RETURN:
            self._pause_trajectory_clock(delta_s)
            command = Twist()
            x, _, _ = self._position_xyz()
            if x is not None and self._mission_start_x_m is not None:
                direction = -1.0 if x > self._mission_start_x_m else 1.0
                command.linear.x = direction * float(self.get_parameter("return_speed_mps").value)
            self.command_publisher.publish(command)
        elif mode == SafetyMode.LAND:
            self._pause_trajectory_clock(delta_s)
            command = Twist()
            required_descent = float(self.get_parameter("required_landing_descent_m").value)
            landing_speed = float(self.get_parameter("landing_speed_mps").value)
            # LIO odometry is intentionally unavailable during persistent LiDAR
            # dropout.  Integrate the bounded descent command (IMU propagation
            # contract) instead of consulting evaluation-only Ground Truth.
            if self._p14_commanded_landing_descent_m < required_descent:
                command.linear.z = -landing_speed
                self._p14_commanded_landing_descent_m += landing_speed * delta_s
            else:
                self._p14_landed = True
                self._p14_complete = True
            self.command_publisher.publish(command)
        else:
            super()._timer()

        local_recovery_clear = bool(
            not math.isfinite(self._nearest_dynamic_range_m)
            or self._nearest_dynamic_range_m > self._float("planning_lookahead_m")
        )
        if (
            self._p14_integrity_recovery_active
            and self._selected is None
            and local_recovery_clear
            and not self.faults.ids(now_s)
        ):
            # This is a separate, locally bounded recovery primitive; no rejected
            # exploration trajectory is executed.  Oscillation creates new
            # parallax while remaining far inside the accepted corridor walls.
            _, y, _ = self._position_xyz()
            anchor = self._p14_integrity_recovery_anchor_y
            started = self._p14_integrity_recovery_start_s
            if y is not None and anchor is not None and started is not None:
                maximum = float(self.get_parameter("integrity_recovery_max_offset_m").value)
                half_period = float(self.get_parameter("integrity_recovery_half_period_s").value)
                direction = -1.0 if int((now_s - started) / half_period) % 2 == 0 else 1.0
                if y <= anchor - maximum:
                    direction = 1.0
                elif y >= anchor + maximum:
                    direction = -1.0
                recovery = Twist()
                recovery.linear.y = direction * float(
                    self.get_parameter("integrity_recovery_speed_mps").value
                )
                self.command_publisher.publish(recovery)

        trial = str(self.get_parameter("p14_trial").value)
        if trial == "matrix" and self._finished:
            self._p14_complete = True
        self._publish_p14_status(now_s, force=self._p14_complete)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P14ControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.command_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
