"""GPS-off ArduPilot mission with a bounded IMPACT research execution phase."""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from mavros_msgs.msg import Mavlink
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .p4_mission_node import P4MissionNode
from .fcu_extnav_gate import CurrentExtnavReadiness


class APResearchMissionNode(P4MissionNode):
    """Keep FCU position control and research velocity control mutually exclusive."""

    POSITION_CONTROL_PHASES = frozenset(("HOVER", "POST_RESEARCH_HOLD"))
    AP471_REQUIRED_PARAMS = {
        "GPS1_TYPE": 0, "GPS2_TYPE": 0, "SIM_GPS1_ENABLE": 0, "SIM_GPS2_ENABLE": 0,
        "VISO_TYPE": 1, "VISO_QUAL_MIN": 0, "PLND_ENABLED": 0,
        "LOG_DISARMED": 1, "LOG_BACKEND_TYPE": 1, "AHRS_EKF_TYPE": 3, "EK3_ENABLE": 1,
        "EK3_SRC1_POSXY": 6, "EK3_SRC1_VELXY": 6, "EK3_SRC1_POSZ": 6,
        "EK3_SRC1_VELZ": 6, "EK3_SRC1_YAW": 6, "ARMING_SKIPCHK": 0,
    }

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("firmware_profile", "legacy_457")
        self.declare_parameter("require_continuous_extnav_ready", False)
        self.firmware_profile = str(self.get_parameter("firmware_profile").value)
        if self.firmware_profile not in {"legacy_457", "ap471"}:
            raise ValueError("unsupported firmware_profile")
        self.require_continuous_ready = (self.firmware_profile == "ap471" or
            bool(self.get_parameter("require_continuous_extnav_ready").value))
        if self.firmware_profile == "ap471":
            self.REQUIRED_PARAMS = dict(self.AP471_REQUIRED_PARAMS)
            self.param_names = list(self.REQUIRED_PARAMS)
        self.live_extnav_gate = CurrentExtnavReadiness() if self.require_continuous_ready else None
        self.state_received_monotonic_ns = None
        self.fcu_odom_stamp_ns = 0
        self.fcu_odom_received_monotonic_ns = None
        self.fcu_odom_invalid_count = 0
        self.last_live_readiness = None
        self.live_abort_pending_future = None
        self.live_abort_pending_done_wall = None
        self.latest_actuation_future = None
        if self.require_continuous_ready:
            self.create_subscription(Mavlink, "/uas1/mavlink_source", self._raw_fcu_cb,
                QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT))
            # Parent's legacy timer uses its ROS clock. Current readiness must
            # continue to run through host wall-clock corrections or /clock loss.
            for timer in tuple(self.timers):
                self.destroy_timer(timer)
            self._live_steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(0.1, self._tick, clock=self._live_steady_clock)
        defaults = {
            "research_timeout_s": 120.0,
            "post_research_hold_s": 2.0,
            "execution_stop_timeout_s": 2.0,
            "failure_land_timeout_s": 30.0,
            "minimum_progress_m": 1.65,
            "mission_distance_m": 2.0,
            "executor_failure_dwell_s": 2.0,
            "executor_status_startup_grace_s": 5.0,
            "maximum_lio_age_s": 0.5,
            "require_freeze_test": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in defaults:
            if name != "require_freeze_test" and (
                not math.isfinite(self._parameter(name)) or self._parameter(name) <= 0.0
            ):
                raise ValueError(f"{name} must be positive and finite")
        if self._parameter("minimum_progress_m") > self._parameter("mission_distance_m"):
            raise ValueError("minimum_progress_m must not exceed mission_distance_m")

        self.research_status: dict[str, object] = {}
        self.executor_status: dict[str, object] = {}
        self.research_last_wall = 0.0
        self.executor_last_wall = 0.0
        self.lio_position: tuple[float, float, float] | None = None
        self.lio_last_wall = 0.0
        self.lio_stamp_ns = 0
        self.research_start_lio: tuple[float, float, float] | None = None
        self.research_end_lio: tuple[float, float, float] | None = None
        self.research_started_wall: float | None = None
        self.research_ended_wall: float | None = None
        self.research_stop_started_wall: float | None = None
        self.research_stop_sequence = -1
        self.research_stop_ack_wall: float | None = None
        self.research_stop_ack_sequence = -1
        self.execution_disabled_acknowledged = False
        self.takeoff_hover_confirmed = False
        self.external_nav_verified = False
        self.failure_reason: str | None = None
        self.failure_started_wall: float | None = None
        self.health_failure_started_wall: float | None = None

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.enable_publisher = self.create_publisher(
            Bool, "/impact/ap/execution_enabled", latched
        )
        self.mission_status_publisher = self.create_publisher(
            String, "/impact/ap/mission_status", latched
        )
        self.create_subscription(
            String, "/xq/p12/flight_status", self._research_cb, latched
        )
        self.create_subscription(
            String, "/impact/ap/execution_status", self._execution_cb,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            Odometry, "/localization/odom", self._lio_cb,
            QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._publish_enable()
        self._publish_mission_status()
        self._event("R1_READY", "Research nodes may start only after the RESEARCH phase")

    def _parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    @staticmethod
    def _json_status(message: String) -> dict[str, object] | None:
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return None
        return value if isinstance(value, dict) else None

    def _research_cb(self, message: String) -> None:
        value = self._json_status(message)
        if value is not None:
            self.research_status = value
            self.research_last_wall = time.monotonic()

    def _execution_cb(self, message: String) -> None:
        value = self._json_status(message)
        if value is not None:
            self.executor_status = value
            self.executor_last_wall = time.monotonic()

    def _lio_cb(self, message: Odometry) -> None:
        position = message.pose.pose.position
        xyz = (float(position.x), float(position.y), float(position.z))
        if not all(math.isfinite(value) for value in xyz):
            return
        stamp_ns = int(message.header.stamp.sec)*1_000_000_000 + int(message.header.stamp.nanosec)
        if stamp_ns <= 0 or stamp_ns <= self.lio_stamp_ns:
            return
        self.lio_position = xyz
        self.lio_last_wall = time.monotonic()
        self.lio_stamp_ns = stamp_ns

    def _state_cb(self, message):
        super()._state_cb(message)
        self.state_received_monotonic_ns = time.monotonic_ns()

    def _odom_cb(self, message):
        if getattr(self, "firmware_profile", "legacy_457") == "ap471":
            stamp = int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)
            position, orientation = message.pose.pose.position, message.pose.pose.orientation
            values = (position.x,position.y,position.z,orientation.x,orientation.y,orientation.z,orientation.w)
            quaternion_norm = math.hypot(*(float(value) for value in values[3:]))
            if (stamp <= getattr(self, "fcu_odom_stamp_ns", 0) or not all(math.isfinite(float(value)) for value in values)
                    or not .999 <= quaternion_norm <= 1.001):
                self.fcu_odom_invalid_count = getattr(self, "fcu_odom_invalid_count", 0)+1
                return
            self.fcu_odom_stamp_ns = stamp
            self.fcu_odom_received_monotonic_ns = time.monotonic_ns()
        super()._odom_cb(message)

    def _extnav_cb(self, message):
        super()._extnav_cb(message)
        gate = getattr(self, "live_extnav_gate", None)
        if gate is not None:
            gate.observe_local(self._json_status(message), time.monotonic_ns())

    def _raw_fcu_cb(self, message):
        self.live_extnav_gate.observe_sys_status(
            system_id=int(message.sysid), component_id=int(message.compid), message_id=int(message.msgid),
            framing_status=int(message.framing_status), magic=int(message.magic),
            payload_length=int(message.len), payload64=[int(value) for value in message.payload64],
            received_ns=time.monotonic_ns())

    def _current_live_ready(self):
        if not getattr(self, "require_continuous_ready", False):
            return {"ready": True, "scope": "legacy_profile_live_gate_not_enabled", "reasons": []}
        self.last_live_readiness = self.live_extnav_gate.snapshot(
            time.monotonic_ns(), connected=bool(self.have_state and self.fcu_state.connected),
            state_received_ns=self.state_received_monotonic_ns,
            parameters_valid=all(self.verified_params.get(name) == expected
                                 for name, expected in self.REQUIRED_PARAMS.items()))
        if getattr(self, "firmware_profile", "legacy_457") == "ap471":
            received = getattr(self, "fcu_odom_received_monotonic_ns", None)
            age = time.monotonic_ns()-received if type(received) is int else None
            self.last_live_readiness.update(FCU_odometry_stamp_ns=getattr(self, "fcu_odom_stamp_ns", 0),
                FCU_odometry_receive_age_ns=age,
                FCU_odometry_clock_scope="Raw MAVROS FCU odometry header ordering plus separate host monotonic receive freshness; not calibrated acquisition time")
            if age is None or not 0 <= age <= 350_000_000:
                self.last_live_readiness["reasons"].append("FCU_odometry_missing_or_stale")
                self.last_live_readiness["ready"] = False
        return self.last_live_readiness

    def _nav_ready(self):
        if getattr(self, "require_continuous_ready", False):
            return self.have_odom and self._current_live_ready()["ready"]
        return super()._nav_ready()

    def _send_command(self, kind):
        if kind in {"arm", "takeoff"} and getattr(self, "require_continuous_ready", False):
            current = self._current_live_ready()
            if self.failure_reason is not None or not current["ready"]:
                self._event("COMMAND_BLOCKED", kind + ": " + ",".join(current["reasons"]))
                self._finish("FAIL", "current ExternalNav readiness lost before " + kind)
                return
        super()._send_command(kind)
        if (getattr(self, "require_continuous_ready", False) and kind in {"arm", "takeoff"}
                and self.pending_command is not None and self.pending_command[0] == kind):
            self.latest_actuation_future = self.pending_command[1]

    def _poll_param(self):
        if getattr(self, "firmware_profile", "legacy_457") != "ap471":
            return super()._poll_param()
        if self.pending_param is not None:
            name, future = self.pending_param
            if not future.done():
                return
            self.pending_param = None
            try:
                values = future.result().values
                if len(values) != 1 or values[0].type == ParameterType.PARAMETER_NOT_SET:
                    self._event("PARAM_WAIT", name + " not pulled from FCU yet")
                    return
                parameter = values[0]
                if parameter.type == ParameterType.PARAMETER_INTEGER:
                    value = parameter.integer_value
                elif parameter.type == ParameterType.PARAMETER_DOUBLE:
                    value = parameter.double_value
                else:
                    raise ValueError("unsupported ROS parameter type")
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value != self.REQUIRED_PARAMS[name]:
                    raise ValueError(f"{name}={value!r}, expected exact {self.REQUIRED_PARAMS[name]}")
            except Exception as exc:
                self._finish("FAIL", f"AP 4.7.1 parameter {name} did not verify: {exc!r}")
                return
            self.verified_params[name] = value
            self.param_index += 1
            self._event("PARAM", f"{name}={value}")
        now = time.monotonic()
        if self.param_index < len(self.param_names) and self.param_client.service_is_ready() and now-self.last_param_request >= 1:
            name = self.param_names[self.param_index]
            request = GetParameters.Request()
            request.names = [name]
            self.pending_param = name, self.param_client.call_async(request)
            self.last_param_request = now

    def _lio_fresh(self, now: float) -> bool:
        return bool(
            self.lio_position is not None
            and now - self.lio_last_wall <= self._parameter("maximum_lio_age_s")
        )

    def _progress(self) -> float:
        endpoint = self.research_end_lio or self.lio_position
        if self.research_start_lio is None or endpoint is None:
            return 0.0
        # P11/P15's outbound mission is along +x in the LIO map. Neither end
        # of this subtraction comes from the FCU's distinct local frame.
        return float(endpoint[0] - self.research_start_lio[0])

    @staticmethod
    def _count(status: dict[str, object], name: str) -> int:
        try:
            return max(0, int(status.get(name, 0)))
        except (TypeError, ValueError, OverflowError):
            return 0

    def _research_complete(self) -> bool:
        return bool(
            self.research_status.get("phase") in {"DONE", "COMPLETE"}
            and self._count(self.research_status, "segments_completed") >= 1
            and self._count(self.research_status, "decisions_applied") >= 1
        )

    def _publish_enable(self) -> None:
        message = Bool()
        message.data = self.phase == "RESEARCH" and self.failure_reason is None
        self.enable_publisher.publish(message)

    def _execution_handoff(self) -> dict[str, object]:
        stopped, acknowledged = self.research_stop_started_wall, self.research_stop_ack_wall
        return {
            "acknowledged": self.execution_disabled_acknowledged,
            "stop_requested_elapsed_s": None if stopped is None else stopped - self.started,
            "acknowledged_elapsed_s": None if acknowledged is None else acknowledged - self.started,
            "ack_delay_s": (
                None if stopped is None or acknowledged is None else acknowledged - stopped
            ),
            "required_last_sequence": self.research_stop_sequence,
            "ack_last_sequence": self.research_stop_ack_sequence,
        }

    def _execution_stop_acknowledged(self, now: float) -> bool:
        stopped = self.research_stop_started_wall
        sequence = self.executor_status.get("last_sequence")
        if stopped is None or not isinstance(sequence, int) or isinstance(sequence, bool):
            return False
        timeout = self._parameter("execution_stop_timeout_s")
        return bool(
            stopped < self.executor_last_wall <= stopped + timeout
            and 0.0 <= now - self.executor_last_wall <= timeout
            and sequence >= max(0, self.research_stop_sequence)
            and self.executor_status.get("enabled") is False
            and self.executor_status.get("allow_publish") is False
            and self.executor_status.get("reason") == "disabled"
        )

    def _publish_mission_status(self) -> None:
        now = time.monotonic()
        start = self.research_started_wall
        message = String()
        message.data = json.dumps(
            {
                "schema_version": 1,
                "phase": self.phase,
                "elapsed_s": now - self.started,
                "research_started_elapsed_s": None if start is None else start - self.started,
                "research_elapsed_s": (
                    0.0 if start is None else (self.research_ended_wall or now) - start
                ),
                "execution_enabled": self.phase == "RESEARCH" and self.failure_reason is None,
                "lio_progress_m": self._progress(),
                "research_status": self.research_status,
                "executor_status": self.executor_status,
                "execution_handoff": self._execution_handoff(),
                "failure_reason": self.failure_reason,
                "firmware_profile": getattr(self, "firmware_profile", "legacy_457"),
                "current_extnav_readiness": getattr(self, "last_live_readiness", None),
                "ground_truth_subscribed": False,
            },
            separators=(",", ":"),
        )
        self.mission_status_publisher.publish(message)

    def _transition(self, phase: str, detail: str) -> None:
        if self.phase == "RESEARCH" and phase != "RESEARCH":
            self.research_end_lio = self.lio_position
            self.research_ended_wall = time.monotonic()
        if phase == "RESEARCH_STOP":
            self.target = None
            self.execution_disabled_acknowledged = False
            self.research_stop_started_wall = time.monotonic()
            self.research_stop_ack_wall = None
            self.research_stop_ack_sequence = -1
            sequence = self.executor_status.get("last_sequence")
            self.research_stop_sequence = (
                sequence if isinstance(sequence, int) and not isinstance(sequence, bool) else -1
            )
        super()._transition(phase, detail)
        self._publish_enable()
        self._publish_mission_status()

    def _research_health_reason(self, now: float) -> str | None:
        if not self.fcu_state.connected:
            return "FCU disconnected"
        if not self.fcu_state.armed or self.fcu_state.mode != "GUIDED":
            return "FCU left armed GUIDED execution"
        if not self._lio_fresh(now):
            return "LIO odometry stale"
        if now - self.extnav_last_wall > 2.0 or self.extnav_status.get("healthy") is not True:
            return "ExternalNav unhealthy"
        if (
            not self.executor_status
            and self.research_started_wall is not None
            and now - self.research_started_wall > self._parameter("executor_status_startup_grace_s")
        ):
            return "AP execution status missing"
        if self.executor_status:
            if now - self.executor_last_wall > 2.0:
                return "AP execution status stale"
            allow_publish = self.executor_status.get("allow_publish") is True
            reason = str(self.executor_status.get("reason", "active" if allow_publish else "unknown"))
            if not allow_publish or reason != "active":
                # The runner starts research nodes only after RESEARCH; its
                # initial no-command interval includes the map warmup period.
                waiting_for_first_command = (
                    self._count(self.executor_status, "accepted_commands") == 0
                    and reason in {"no_command", "disabled", "initializing", "waiting_for_command"}
                )
                if not waiting_for_first_command:
                    return f"AP executor blocked: {reason}"
        if str(self.research_status.get("phase", "")).startswith("FAIL"):
            return "research controller: " + str(self.research_status["phase"])
        return None

    def _checks(self) -> dict[str, bool]:
        checks = {
            "gps_disabled": all(
                self.verified_params.get(name) == 0
                for name in ("GPS_TYPE", "GPS_TYPE2")
            ) and all(
                self.verified_params.get(name) == 1
                for name in ("SIM_GPS_DISABLE", "SIM_GPS2_DISABLE")
            ),
            "external_nav": self.external_nav_verified and all(
                self.verified_params.get(name) == expected
                for name, expected in self.REQUIRED_PARAMS.items()
            ),
            "takeoff_and_hover": self.takeoff_hover_confirmed,
            "research_decision": self._research_complete(),
            "executed_progress": self._progress() >= self._parameter("minimum_progress_m"),
            "executor_forwarded": (
                self._count(self.executor_status, "accepted_commands") > 0
                and self._count(self.executor_status, "published_commands") > 0
            ),
            "execution_disabled_acknowledged": self.execution_disabled_acknowledged is True,
            "landed_and_disarmed": (
                self.have_state
                and not self.fcu_state.armed
                and abs(self.current_xyz[2] - self.origin_xyz[2]) <= 0.35
            ),
        }
        if getattr(self, "firmware_profile", "legacy_457") == "ap471":
            checks["gps_disabled"] = all(self.verified_params.get(name) == 0
                for name in ("GPS1_TYPE", "GPS2_TYPE", "SIM_GPS1_ENABLE", "SIM_GPS2_ENABLE"))
        if bool(self.get_parameter("require_freeze_test").value):
            checks["freeze_watchdog_test"] = (
                self.executor_status.get("freeze_test_started") is True
                and self.executor_status.get("freeze_test_completed") is True
                and self._count(self.executor_status, "stale_stop_count") > 0
            )
        return checks

    def _finish(self, status: str, reason: str) -> None:
        if self.finalized:
            return
        pending_actuation = self.pending_command is not None and self.pending_command[0] in {"arm", "takeoff"}
        actuation_future = (self.pending_command[1] if pending_actuation else
                            getattr(self, "latest_actuation_future", None))
        if status != "PASS" and (self.fcu_state.armed or (getattr(self, "require_continuous_ready", False) and actuation_future is not None)):
            if self.failure_reason is None:
                if actuation_future is not None:
                    # A ROS future cancellation does not cancel an already
                    # transmitted MAVLink command. Keep observing the future
                    # and fresh FCU states while issuing landing cleanup.
                    self.live_abort_pending_future = actuation_future
                    self.live_abort_pending_done_wall = None
                self.failure_reason = reason
                self.failure_started_wall = time.monotonic()
                self.target = None
                self._transition("LAND", f"failure cleanup: {reason}")
            return
        self._finalize(status, reason)

    def _finalize(self, status: str, reason: str) -> None:
        if self.finalized:
            return
        checks = self._checks()
        if self.failure_reason is not None:
            status = "FAIL"
            if not reason.startswith(self.failure_reason):
                reason = self.failure_reason
        elif status == "PASS" and not all(checks.values()):
            status = "FAIL"
            reason = "Failed R1 checks: " + ", ".join(key for key, ok in checks.items() if not ok)
        if status != "PASS" and self.failure_reason is None:
            self.failure_reason = reason
        self.finalized = True
        self.phase = "DONE" if status == "PASS" else "FAILED"
        self._publish_enable()
        self._event(status, reason)
        self._publish_mission_status()
        result = {
            "schema_version": 1,
            "gate": "R1_IMPACT_ARDUPILOT_RESEARCH_CLOSED_LOOP",
            "status": status,
            "reason": reason,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "elapsed_s": time.monotonic() - self.started,
            "research_started_elapsed_s": (
                None if self.research_started_wall is None
                else self.research_started_wall - self.started
            ),
            "research_elapsed_s": (
                0.0 if self.research_started_wall is None
                else (self.research_ended_wall or time.monotonic()) - self.research_started_wall
            ),
            "mission_distance_m": self._parameter("mission_distance_m"),
            "minimum_progress_m": self._parameter("minimum_progress_m"),
            "lio_progress_m": self._progress(),
            "research_start_lio_xyz_m": self.research_start_lio,
            "research_end_lio_xyz_m": self.research_end_lio,
            "verified_parameters": self.verified_params,
            "external_nav": self.extnav_status,
            "firmware_profile": getattr(self, "firmware_profile", "legacy_457"),
            "current_extnav_readiness": getattr(self, "last_live_readiness", None),
            "pending_actuation_at_finalization": bool(getattr(self, "live_abort_pending_future", None) is not None and
                                                       not self.live_abort_pending_future.done()),
            "failure_cleanup_state_fresh_at_finalization": bool(
                type(getattr(self, "state_received_monotonic_ns", None)) is int and
                0 <= time.monotonic_ns()-self.state_received_monotonic_ns <= 2_000_000_000 and
                self.fcu_state.connected),
            "research": self.research_status,
            "executor": self.executor_status,
            "execution_handoff": self._execution_handoff(),
            "checks": checks,
            "final": {
                "connected": bool(self.fcu_state.connected),
                "armed": bool(self.fcu_state.armed),
                "mode": self.fcu_state.mode,
                "fcu_xyz_m": self.current_xyz,
                "lio_xyz_m": self.lio_position,
            },
            "events": self.events,
            "ground_truth_subscribed": False,
        }
        path = Path(str(self.get_parameter("result_file").value))
        if str(path) in {"", "."}:
            self.get_logger().error("result_file parameter is empty")
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)

    def _tick(self) -> None:
        if self.finalized:
            return
        self._publish_origin()
        self._poll_command()
        now = time.monotonic()
        if (getattr(self, "require_continuous_ready", False) and self.failure_reason is None and
                self.phase in {"SET_GUIDED", "ARM", "TAKEOFF", "ASCEND", "HOVER", "RESEARCH", "RESEARCH_STOP", "POST_RESEARCH_HOLD"}):
            current = self._current_live_ready()
            if not current["ready"]:
                self._finish("FAIL", "current ExternalNav readiness lost: " + ",".join(current["reasons"]))
                return
        self._publish_enable()
        self._publish_mission_status()
        if self.phase in self.POSITION_CONTROL_PHASES:
            self._publish_setpoint()

        # Failure cleanup owns a separate deadline. Reapplying the ordinary
        # mission timeout here would prevent LAND/DESCEND from being serviced.
        if self.failure_reason is not None:
            pending = getattr(self, "live_abort_pending_future", None)
            if pending is not None and pending.done() and self.live_abort_pending_done_wall is None:
                self.live_abort_pending_done_wall = now
            pending_settled = (pending is None or (self.live_abort_pending_done_wall is not None and
                now-self.live_abort_pending_done_wall >= 2.0 and
                getattr(self, "state_received_monotonic_ns", 0) is not None and
                self.state_received_monotonic_ns >= int(self.live_abort_pending_done_wall*1e9) and
                0 <= time.monotonic_ns()-self.state_received_monotonic_ns <= 2_000_000_000 and
                self.fcu_state.connected))
            if getattr(self, "require_continuous_ready", False):
                received = getattr(self, "state_received_monotonic_ns", None)
                pending_settled = bool(pending_settled and type(received) is int and
                    0 <= time.monotonic_ns()-received <= 2_000_000_000 and self.fcu_state.connected)
            if not self.fcu_state.armed and pending_settled:
                self._finalize("FAIL", self.failure_reason)
            elif now - self.failure_started_wall >= self._parameter("failure_land_timeout_s"):
                self._finalize("FAIL", self.failure_reason + "; landing cleanup deadline reached")
            elif self.phase == "LAND" or (getattr(self, "require_continuous_ready", False) and self.phase == "DESCEND"):
                self._send_command("land")
            return

        if now - self.started > self._parameter("mission_timeout_s"):
            self._finish("FAIL", f"mission timeout in {self.phase}")
            return
        if self.phase != "RESEARCH" and self.have_state and not self.fcu_state.connected and self.phase != "WAIT_FCU":
            self._finish("FAIL", f"FCU disconnected in {self.phase}")
            return

        if self.phase == "WAIT_FCU":
            if self.have_state and self.fcu_state.connected:
                self._transition("SET_STREAM", "FCU heartbeat connected")
        elif self.phase == "SET_STREAM":
            self._send_command("stream")
        elif self.phase == "VERIFY_NAV":
            self._poll_param()
            if self.finalized or self.failure_reason is not None:
                return
            if self._nav_ready():
                self.external_nav_verified = True
                self.target = self.current_xyz
                self._transition("SET_GUIDED", "GPS-off ExternalNav contract verified")
            elif now - self.phase_started > 90.0:
                self._finish("FAIL", "ExternalNav or FCU parameters did not verify")
        elif self.phase == "SET_GUIDED":
            if self.fcu_state.mode == "GUIDED":
                self._transition("ARM", "GUIDED confirmed")
            else:
                self._send_command("mode")
        elif self.phase == "ARM":
            if self.fcu_state.armed:
                self.origin_xyz = self.current_xyz
                altitude = self._parameter("takeoff_altitude_m")
                self.target = (*self.origin_xyz[:2], self.origin_xyz[2] + altitude)
                self._transition("TAKEOFF", "armed with ExternalNav")
            else:
                self._send_command("arm")
        elif self.phase == "TAKEOFF":
            self._send_command("takeoff")
        elif self.phase == "ASCEND":
            if self._arrived_for(2.0):
                self._transition("HOVER", "takeoff altitude reached")
            elif now - self.phase_started > 45.0:
                self._finish("FAIL", "takeoff altitude not reached")
        elif self.phase == "HOVER":
            if self._arrived_for(self._parameter("hover_duration_s")) and self._lio_fresh(now):
                self.takeoff_hover_confirmed = True
                self.research_start_lio = self.lio_position
                self.research_started_wall = now
                self.target = None
                self._transition("RESEARCH", "stable hover; research velocity execution enabled")
        elif self.phase == "RESEARCH":
            health_reason = self._research_health_reason(now)
            if health_reason is None:
                self.health_failure_started_wall = None
            elif self.health_failure_started_wall is None:
                self.health_failure_started_wall = now
            elif now - self.health_failure_started_wall >= self._parameter("executor_failure_dwell_s"):
                self._finish("FAIL", health_reason)
                return
            if now - self.phase_started >= self._parameter("research_timeout_s"):
                self._finish("FAIL", "research phase timeout")
            elif self._research_complete():
                if health_reason is not None or self._progress() < self._parameter("minimum_progress_m"):
                    self._finish("FAIL", "research completed without healthy required LIO progress")
                    return
                self._transition(
                    "RESEARCH_STOP", "research complete; waiting for velocity executor disable acknowledgement"
                )
        elif self.phase == "RESEARCH_STOP":
            if self._execution_stop_acknowledged(now):
                self.execution_disabled_acknowledged = True
                self.research_stop_ack_wall = self.executor_last_wall
                self.research_stop_ack_sequence = self.executor_status["last_sequence"]
                # Only the acknowledged stop permits the new owner to publish.
                # Capture its FCU coordinates now, rather than a pre-stop target.
                self.target = self.current_xyz
                self._transition(
                    "POST_RESEARCH_HOLD", "velocity executor disabled; FCU position hold owns control"
                )
                self._publish_setpoint()
            elif now - self.phase_started >= self._parameter("execution_stop_timeout_s"):
                self._finish("FAIL", "AP execution disable acknowledgement timeout")
        elif self.phase == "POST_RESEARCH_HOLD":
            if now - self.phase_started >= self._parameter("post_research_hold_s"):
                self._transition("LAND", "post-research position hold complete")
        elif self.phase == "LAND":
            self._send_command("land")
        elif self.phase == "DESCEND":
            if not self.fcu_state.armed and abs(self.current_xyz[2] - self.origin_xyz[2]) <= 0.35:
                self._finish("PASS", "IMPACT research decision, AP execution and landing completed")
            elif now - self.phase_started > 60.0:
                self._finish("FAIL", "landing/disarm not confirmed")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = APResearchMissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.enable_publisher.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
