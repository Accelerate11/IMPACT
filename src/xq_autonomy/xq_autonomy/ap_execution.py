"""ROS-independent checks for the IMPACT -> ArduPilot velocity boundary.

All clocks are supplied by the caller. ROS time authenticates command/source
age; monotonic time runs the watchdog and normal-command acceleration limit.
The fixed yaw rotates map vectors into FCU local ENU, preserving their Z axis.
No part of this module claims that a zero setpoint guarantees a safe hover.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


Vector3 = tuple[float, float, float]
ZERO: Vector3 = (0.0, 0.0, 0.0)


def _finite(value: object) -> bool:
    if isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        return False


@dataclass(frozen=True)
class ExecutionConfig:
    max_speed_mps: float = 0.2
    max_acceleration_mps2: float = 0.3
    max_command_age_s: float = 0.35
    max_source_odom_age_s: float = 0.5
    # P4's ExternalNav status is normally published at 1 Hz.
    max_external_nav_status_age_s: float = 2.0
    map_to_fcu_enu_yaw_rad: float = 0.0
    max_future_skew_s: float = 0.0

    def __post_init__(self) -> None:
        for field in (
            "max_speed_mps", "max_acceleration_mps2", "max_command_age_s",
            "max_source_odom_age_s", "max_external_nav_status_age_s",
        ):
            value = getattr(self, field)
            if not _finite(value) or value <= 0.0:
                raise ValueError(f"{field} must be positive and finite")
        if not _finite(self.map_to_fcu_enu_yaw_rad):
            raise ValueError("map_to_fcu_enu_yaw_rad must be finite")
        if not _finite(self.max_future_skew_s) or self.max_future_skew_s < 0.0:
            raise ValueError("max_future_skew_s must be nonnegative and finite")


@dataclass(frozen=True)
class VelocityCommand:
    sequence: int
    generated_ros_s: float
    source_odom_ros_s: float
    received_monotonic_s: float
    velocity_map_mps: Vector3


@dataclass(frozen=True)
class ExecutionDecision:
    allow_publish: bool
    reason: str
    velocity_fcu_enu_mps: Vector3 = ZERO


def _limit_norm(vector: Vector3, maximum: float) -> Vector3:
    if math.hypot(*vector) <= maximum:
        return vector
    # Normalize before scaling so even extremely large finite input vectors
    # do not overflow a squared norm or lose their direction.
    largest = max(abs(value) for value in vector)
    scaled = tuple(value / largest for value in vector)
    factor = maximum / math.hypot(*scaled)
    return tuple(value * factor for value in scaled)


class APVelocityExecutor:
    """Stateful watchdog; use one instance for one monotonic/ROS clock epoch.

    A clock rollback latches a fault and invalidates the command. Recreate the
    executor for a new clock epoch rather than allowing old messages to revive.
    Enabling requires a subsequent accepted command. Disabling, losing flight
    eligibility, or stopping resets the acceleration origin to zero. Sequence
    and generation high-water marks survive disable/enable transitions.

    A healthy-localization command timeout requests immediate zero velocity;
    this safety stop overrides the normal-command slew limit. Actual braking
    remains ArduPilot's responsibility. Localization failure permits no output.
    """

    def __init__(self, config: ExecutionConfig | None = None) -> None:
        self.config = config if config is not None else ExecutionConfig()
        self._enabled = False
        self._command: VelocityCommand | None = None
        self._last_sequence = -1
        self._last_generated_ros_s = -1.0
        self._last_source_odom_ros_s = -1.0
        self._last_receipt_monotonic_s = -1.0
        self._last_ros_s: float | None = None
        self._last_monotonic_s: float | None = None
        self._clock_fault = ""
        self._last_output = ZERO
        self._last_output_monotonic_s: float | None = None
        self.last_rejection_reason = ""

    def _reset_output(self) -> None:
        self._last_output = ZERO
        self._last_output_monotonic_s = None

    def set_enabled(self, enabled: bool) -> None:
        if not isinstance(enabled, bool):
            raise ValueError("enabled must be a bool")
        if enabled != self._enabled:
            self._enabled = enabled
            self._command = None
            self._reset_output()

    def _observe_clocks(self, now_ros_s: float, now_monotonic_s: float) -> str:
        if self._clock_fault:
            return self._clock_fault
        if (
            not _finite(now_ros_s) or now_ros_s < 0.0
            or not _finite(now_monotonic_s) or now_monotonic_s < 0.0
        ):
            self._clock_fault = "invalid_clock"
        elif self._last_ros_s is not None and now_ros_s < self._last_ros_s:
            self._clock_fault = "ros_clock_rollback"
        elif self._last_monotonic_s is not None and now_monotonic_s < self._last_monotonic_s:
            self._clock_fault = "monotonic_clock_rollback"
        if self._clock_fault:
            self._command = None
            self._reset_output()
            return self._clock_fault
        self._last_ros_s = float(now_ros_s)
        self._last_monotonic_s = float(now_monotonic_s)
        return ""

    def accept_command(
        self, command: VelocityCommand, *, now_ros_s: float, now_monotonic_s: float,
    ) -> bool:
        """Reject invalid/replayed frames without replacing the accepted receipt."""
        reason = self._observe_clocks(now_ros_s, now_monotonic_s)
        if not reason:
            reason = self._command_problem(command, now_ros_s, now_monotonic_s)
        self.last_rejection_reason = reason
        if reason:
            return False
        # A delayed callback may replace an expired command before the timer
        # observes its timeout. Do not carry the old ramp across that gap.
        if (
            self._command is not None
            and self._command_age_problem(self._command, now_ros_s, now_monotonic_s)
        ) or (
            self._last_output_monotonic_s is not None
            and now_monotonic_s - self._last_output_monotonic_s > self.config.max_command_age_s
        ):
            self._reset_output()
        # Copy values so a caller-provided mutable velocity list cannot bypass
        # validation after this method returns.
        self._command = VelocityCommand(
            command.sequence, float(command.generated_ros_s), float(command.source_odom_ros_s),
            float(command.received_monotonic_s), tuple(float(v) for v in command.velocity_map_mps),
        )
        self._last_sequence = command.sequence
        self._last_generated_ros_s = command.generated_ros_s
        self._last_source_odom_ros_s = command.source_odom_ros_s
        self._last_receipt_monotonic_s = command.received_monotonic_s
        return True

    def _command_problem(
        self, command: VelocityCommand, now_ros_s: float, now_monotonic_s: float,
    ) -> str:
        if not isinstance(command, VelocityCommand):
            return "invalid_command"
        if isinstance(command.sequence, bool) or not isinstance(command.sequence, int) or command.sequence < 0:
            return "invalid_sequence"
        if command.sequence <= self._last_sequence:
            return "sequence_not_increasing"
        stamps = (command.generated_ros_s, command.source_odom_ros_s, command.received_monotonic_s)
        if any(not _finite(stamp) or stamp < 0.0 for stamp in stamps):
            return "invalid_command_stamp"
        if not isinstance(command.velocity_map_mps, (tuple, list)) or len(command.velocity_map_mps) != 3:
            return "invalid_velocity"
        if not all(_finite(value) for value in command.velocity_map_mps):
            return "invalid_velocity"
        # Several distinct sequence numbers can share one simulated ROS tick.
        # Never invent a future stamp to make those commands strictly ordered.
        if command.generated_ros_s < self._last_generated_ros_s:
            return "generation_out_of_order"
        if command.source_odom_ros_s < self._last_source_odom_ros_s:
            return "source_odom_out_of_order"
        if command.received_monotonic_s < self._last_receipt_monotonic_s:
            return "receipt_out_of_order"
        skew = self.config.max_future_skew_s
        if command.generated_ros_s > now_ros_s + skew:
            return "generation_in_future"
        if command.source_odom_ros_s > now_ros_s + skew:
            return "source_odom_in_future"
        if command.source_odom_ros_s > command.generated_ros_s + skew:
            return "source_odom_after_generation"
        if command.received_monotonic_s > now_monotonic_s:
            return "receipt_in_future"
        return self._command_age_problem(command, now_ros_s, now_monotonic_s)

    def _command_age_problem(
        self, command: VelocityCommand, now_ros_s: float, now_monotonic_s: float,
    ) -> str:
        if now_ros_s - command.generated_ros_s > self.config.max_command_age_s:
            return "command_ros_stale"
        if now_monotonic_s - command.received_monotonic_s > self.config.max_command_age_s:
            return "command_wall_stale"
        if now_ros_s - command.source_odom_ros_s > self.config.max_source_odom_age_s:
            return "command_source_odom_stale"
        return ""

    def step(
        self, *, now_ros_s: float, now_monotonic_s: float,
        fcu_connected: bool, fcu_armed: bool, fcu_mode: str,
        external_nav_healthy: bool, external_nav_status_monotonic_s: float,
        lio_source_stamp_ros_s: float,
    ) -> ExecutionDecision:
        clock_problem = self._observe_clocks(now_ros_s, now_monotonic_s)
        if not self._enabled:
            return self._blocked("disabled")
        if clock_problem:
            return self._blocked(clock_problem)
        if fcu_connected is not True:
            return self._blocked("fcu_disconnected")
        if fcu_armed is not True:
            return self._blocked("fcu_disarmed")
        if fcu_mode != "GUIDED":
            return self._blocked("fcu_not_guided")
        if external_nav_healthy is not True:
            return self._blocked("external_nav_unhealthy")
        if not _finite(external_nav_status_monotonic_s) or external_nav_status_monotonic_s < 0.0:
            return self._blocked("external_nav_status_invalid")
        status_age = now_monotonic_s - external_nav_status_monotonic_s
        if status_age < 0.0:
            return self._blocked("external_nav_status_future")
        if status_age > self.config.max_external_nav_status_age_s:
            return self._blocked("external_nav_status_stale")
        if not _finite(lio_source_stamp_ros_s) or lio_source_stamp_ros_s < 0.0:
            return self._blocked("lio_stamp_invalid")
        if lio_source_stamp_ros_s > now_ros_s + self.config.max_future_skew_s:
            return self._blocked("lio_stamp_future")
        if now_ros_s - lio_source_stamp_ros_s > self.config.max_source_odom_age_s:
            return self._blocked("lio_source_stale")

        reason = "no_command" if self._command is None else self._command_age_problem(
            self._command, now_ros_s, now_monotonic_s,
        )
        if reason:
            self._last_output = ZERO
            self._last_output_monotonic_s = now_monotonic_s
            return ExecutionDecision(True, reason)

        target = _limit_norm(self._command.velocity_map_mps, self.config.max_speed_mps)
        yaw = self.config.map_to_fcu_enu_yaw_rad
        cosine, sine = math.cos(yaw), math.sin(yaw)
        target_enu = (
            cosine * target[0] - sine * target[1],
            sine * target[0] + cosine * target[1], target[2],
        )
        # First eligible step starts at zero. Subsequent steps use actual wall
        # elapsed time, including when simulated ROS time has stopped advancing.
        elapsed = 0.0 if self._last_output_monotonic_s is None else (
            now_monotonic_s - self._last_output_monotonic_s
        )
        delta = tuple(target_enu[index] - self._last_output[index] for index in range(3))
        delta = _limit_norm(delta, self.config.max_acceleration_mps2 * elapsed)
        output = tuple(self._last_output[index] + delta[index] for index in range(3))
        self._last_output = output
        self._last_output_monotonic_s = now_monotonic_s
        return ExecutionDecision(True, "active", output)

    def _blocked(self, reason: str) -> ExecutionDecision:
        self._reset_output()
        return ExecutionDecision(False, reason)
