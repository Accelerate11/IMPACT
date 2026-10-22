"""Stamped research velocity commands -> the ArduPilot MAVROS ENU boundary."""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy._rclpy_pybind11 import RCLError
from std_msgs.msg import Bool, String

from .ap_execution import APVelocityExecutor, ExecutionConfig, VelocityCommand


def stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_of(orientation) -> float:
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    norm = math.hypot(x, y, z, w)
    if not all(math.isfinite(value) for value in (x, y, z, w)) or norm <= 1e-12:
        raise ValueError("invalid orientation")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class APExecutionNode(Node):
    def __init__(self, *, context: Context | None = None) -> None:
        super().__init__("impact_ap_execution", context=context)
        defaults = {
            "max_speed_mps": 0.2,
            "max_acceleration_mps2": 0.3,
            "max_command_age_s": 0.35,
            "max_source_odom_age_s": 0.5,
            "max_external_nav_status_age_s": 2.0,
            "map_to_fcu_enu_yaw_rad": 0.0,
            "max_future_skew_s": 0.02,
            "maximum_frame_yaw_error_rad": 0.15,
            "freeze_after_moving_s": 0.0,
            "freeze_duration_s": 0.9,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        config = ExecutionConfig(**{
            name: float(self.get_parameter(name).value)
            for name in ExecutionConfig.__dataclass_fields__
        })
        self.velocity_executor = APVelocityExecutor(config)
        self.enabled = False
        self.state = State()
        self.state_wall = -math.inf
        self.extnav = {}
        self.extnav_wall = -math.inf
        self.lio: Odometry | None = None
        self.fcu_odom: Odometry | None = None
        self.fcu_odom_wall = -math.inf
        self.last_command: VelocityCommand | None = None
        self.last_sequence = -1
        self.accepted = self.rejected = self.published = 0
        self.stale_stops = self.replay_rejections = 0
        self.last_reason = "disabled"
        self.moving_started_wall: float | None = None
        self.freeze_started_wall: float | None = None
        self.freeze_completed = False
        self.freeze_zero_count = 0
        self.freeze_first_zero_delay_s: float | None = None
        self.disabled_publish_count = 0
        self.last_status_wall = 0.0
        self.frame_yaw_error: float | None = None
        reliable = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(
            TwistStamped, "/uav1/mavros/setpoint_velocity/cmd_vel", 20)
        self.status = self.create_publisher(String, "/impact/ap/execution_status", latched)
        self.create_subscription(Bool, "/impact/ap/execution_enabled", self._enable_cb, latched)
        self.create_subscription(String, "/impact/research/command_provenance", self._command_cb, reliable)
        self.create_subscription(State, "/uav1/mavros/state", self._state_cb, sensor)
        self.create_subscription(Odometry, "/uav1/mavros/local_position/odom", self._fcu_cb, sensor)
        self.create_subscription(Odometry, "/localization/odom", self._lio_cb, reliable)
        self.create_subscription(String, "/xq/p4/extnav/status", self._extnav_cb, reliable)
        # A simulated clock pause must not pause the wall-time watchdog.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self._tick, clock=self.steady_clock)

    def _enable_cb(self, message: Bool) -> None:
        self.enabled = bool(message.data)
        self.velocity_executor.set_enabled(self.enabled)

    def _state_cb(self, message: State) -> None:
        self.state, self.state_wall = message, time.monotonic()

    def _fcu_cb(self, message: Odometry) -> None:
        self.fcu_odom = message
        self.fcu_odom_wall = time.monotonic()

    def _lio_cb(self, message: Odometry) -> None:
        self.lio = message

    def _extnav_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                self.extnav, self.extnav_wall = payload, time.monotonic()
        except (ValueError, TypeError):
            pass

    def _ros_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _freeze_active(self, now: float) -> bool:
        if self.freeze_started_wall is None:
            return False
        if now - self.freeze_started_wall >= float(self.get_parameter("freeze_duration_s").value):
            self.freeze_completed = True
            return False
        return True

    def _accept(self, command: VelocityCommand, now: float) -> bool:
        accepted = self.velocity_executor.accept_command(
            command, now_ros_s=self._ros_s(), now_monotonic_s=now)
        if accepted:
            self.accepted += 1
            self.last_command = command
            self.last_sequence = command.sequence
        else:
            self.rejected += 1
        return accepted

    def _command_cb(self, message: String) -> None:
        now = time.monotonic()
        # The replay trial keeps the bridge/timer alive but withholds fresh
        # planner frames, repeatedly offering the last original command.
        if self._freeze_active(now):
            if self.last_command is not None and not self._accept(self.last_command, now):
                self.replay_rejections += 1
            return
        try:
            payload = json.loads(message.data)
            if payload.get("schema_version") != 1 or payload.get("frame_id") != "xq_lio_map":
                raise ValueError("unexpected command schema/frame")
            if any(abs(float(payload["angular"][axis])) > 1e-8 for axis in "xyz"):
                raise ValueError("R1 accepts translational commands with zero angular rate")
            sequence = payload["sequence_id"]
            if isinstance(sequence, bool) or not isinstance(sequence, int):
                raise ValueError("sequence must be an integer")
            command = VelocityCommand(
                sequence=sequence,
                generated_ros_s=int(payload["generation_stamp_ns"]) * 1e-9,
                source_odom_ros_s=int(payload["odometry_stamp_ns"]) * 1e-9,
                received_monotonic_s=now,
                velocity_map_mps=tuple(float(payload["linear"][axis]) for axis in "xyz"),
            )
        except (ValueError, TypeError, KeyError, OverflowError):
            self.rejected += 1
            return
        self._accept(command, now)

    def _tick(self) -> None:
        now = time.monotonic()
        ros_s = self._ros_s()
        frame_valid = self.lio is not None and self.fcu_odom is not None and now - self.fcu_odom_wall <= 1.0
        self.frame_yaw_error = None
        if frame_valid:
            try:
                difference = (yaw_of(self.fcu_odom.pose.pose.orientation)
                              - yaw_of(self.lio.pose.pose.orientation)
                              - self.velocity_executor.config.map_to_fcu_enu_yaw_rad)
                if not math.isfinite(difference):
                    raise ValueError("nonfinite heading difference")
                self.frame_yaw_error = math.remainder(difference, 2.0 * math.pi)
                frame_valid = abs(self.frame_yaw_error) <= float(
                    self.get_parameter("maximum_frame_yaw_error_rad").value)
            except ValueError:
                frame_valid = False
        decision = self.velocity_executor.step(
            now_ros_s=ros_s, now_monotonic_s=now,
            fcu_connected=bool(self.state.connected) and now - self.state_wall <= 2.0,
            fcu_armed=bool(self.state.armed), fcu_mode=self.state.mode,
            external_nav_healthy=self.extnav.get("healthy") is True and frame_valid,
            external_nav_status_monotonic_s=self.extnav_wall,
            lio_source_stamp_ros_s=stamp_s(self.lio.header.stamp) if self.lio is not None else -1.0,
        )
        speed = math.hypot(*decision.velocity_fcu_enu_mps)
        if self.enabled and decision.reason == "active" and speed > 0.03:
            if self.moving_started_wall is None:
                self.moving_started_wall = now
            delay = float(self.get_parameter("freeze_after_moving_s").value)
            if delay > 0.0 and self.freeze_started_wall is None and now - self.moving_started_wall >= delay:
                self.freeze_started_wall = now
        if self._freeze_active(now) and self.last_command is not None:
            if not self._accept(self.last_command, now):
                self.replay_rejections += 1
        if decision.allow_publish:
            command = TwistStamped()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = "map"
            command.twist.linear.x, command.twist.linear.y, command.twist.linear.z = decision.velocity_fcu_enu_mps
            self.output.publish(command)
            self.published += 1
            self.disabled_publish_count += int(not self.enabled)
            if "stale" in decision.reason:
                self.stale_stops += 1
                if self._freeze_active(now) and speed == 0.0:
                    self.freeze_zero_count += 1
                    if self.freeze_first_zero_delay_s is None:
                        self.freeze_first_zero_delay_s = now - self.freeze_started_wall
        if decision.reason != self.last_reason:
            self.get_logger().info(f"AP execution: {self.last_reason} -> {decision.reason}")
            self.last_reason = decision.reason
        if now - self.last_status_wall >= 0.2:
            payload = {
                "schema_version": 1, "enabled": self.enabled,
                "allow_publish": decision.allow_publish, "reason": decision.reason,
                "accepted_commands": self.accepted, "rejected_commands": self.rejected,
                "last_sequence": self.last_sequence, "published_commands": self.published,
                "stale_stop_count": self.stale_stops, "disabled_publish_count": self.disabled_publish_count,
                "velocity_fcu_enu_mps": list(decision.velocity_fcu_enu_mps),
                "frame_yaw_error_rad": self.frame_yaw_error,
                "freeze_test_started": self.freeze_started_wall is not None,
                "freeze_test_completed": self.freeze_completed,
                "freeze_zero_count": self.freeze_zero_count,
                "freeze_first_zero_delay_s": self.freeze_first_zero_delay_s,
                "replay_rejections": self.replay_rejections,
                "last_rejection_reason": self.velocity_executor.last_rejection_reason,
            }
            status = String()
            status.data = json.dumps(payload, allow_nan=False)
            self.status.publish(status)
            self.last_status_wall = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = APExecutionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        # SIGINT can invalidate the Humble context while spin constructs a
        # wait set. Runtime ROS errors must still propagate with a live context.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
