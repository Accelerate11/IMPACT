#!/usr/bin/env python3
"""Observe extnav readiness, without commanding the aircraft.

The core has no ROS dependency. Readiness is sampled backend receipt health,
not an ODOMETRY acknowledgement, EKF fusion proof, or permission to fly.

AP source evidence (also hashed in each new run's metadata):
  GCS_MAVLink/GCS.cpp:320-327: visual odometry present/enabled/healthy bits.
  AP_VisualOdom/AP_VisualOdom_Backend.cpp:32-36 and AP_VisualOdom.h:31:
    backend health means receipt within 300 ms, using AP's own boot clock.
  ArduCopter/GCS_Copter.cpp:117-124: precision landing shares bit 128;
    a frozen, successful PLND_ENABLED=0 parameter query is therefore required.
  AP_VisualOdom/AP_VisualOdom_MAV.cpp:39-77: receipt health is not quality
    acceptance or an EKF-consumption acknowledgement.
"""

from __future__ import annotations

import argparse
from collections import Counter
from decimal import Decimal, InvalidOperation
import hashlib
import json
import math
from pathlib import Path
import signal
import struct
import time
from typing import Any


NS = 1_000_000_000
SCOPE = (
    "Live sampled FCU visual-odometry backend receipt health only; not a "
    "per-frame ODOMETRY ACK, EKF fusion/quality acceptance, mission integration, "
    "flight validation, or permission to arm. Parameter target IDs are configured "
    "targets; telemetry SYS_STATUS identity is checked separately."
)
DEFAULT_CONFIG = {
    "schema_version": 1,
    "expected_system_id": 1,
    "expected_component_id": 1,
    "vision_position_mask": 128,
    "local_status_max_age_s": 0.35,
    "sys_status_max_age_s": 0.35,
    "state_max_age_s": 2.0,
    "fcu_healthy_dwell_s": 5.0,
    "maximum_sys_status_gap_s": 0.35,
    "check_rate_hz": 10.0,
    "required_parameters": {
        "VISO_TYPE": 2, "PLND_ENABLED": 0, "LOG_DISARMED": 1,
        "GPS_TYPE": 0, "EK3_SRC1_POSXY": 6, "EK3_SRC1_POSZ": 6,
        "EK3_SRC1_VELXY": 6, "EK3_SRC1_VELZ": 6, "EK3_SRC1_YAW": 6,
    },
}
BASELINE_PARAMETERS = {"VISO_TYPE": 2, "PLND_ENABLED": 0, "LOG_DISARMED": 1}
SOURCE_EVIDENCE = {
    "libraries/GCS_MAVLink/GCS.cpp": [151, 155, 320, 327],
    "libraries/AP_VisualOdom/AP_VisualOdom.cpp": [154, 169],
    "libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp": [32, 36],
    "libraries/AP_VisualOdom/AP_VisualOdom.h": [31, 31],
    "libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp": [39, 77],
    "ArduCopter/GCS_Copter.cpp": [117, 124],
}


def _integer(value: Any, minimum: int = 0, maximum: int | None = None) -> int:
    if type(value) is not int or value < minimum or (maximum is not None and value > maximum):
        raise ValueError("expected an integer in the declared range")
    return value


def _number(value: Any) -> int | float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("expected a finite numeric value, not bool")
    return value


def _finite_tree(value: Any) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_finite_tree(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return all(_finite_tree(v) for v in value)
    return True


def seconds_to_ns(value: Any) -> int:
    _number(value)
    try:
        result = Decimal(str(value)) * NS
        if result <= 0 or result != result.to_integral_value():
            raise ValueError("duration must be positive and exactly representable in ns")
        return int(result)
    except InvalidOperation as exc:
        raise ValueError("invalid duration") from exc


def validate_config(config: dict) -> dict:
    if not isinstance(config, dict) or not _finite_tree(config):
        raise ValueError("config must be a finite JSON object")
    config = config.get("live_readiness", config)
    if not isinstance(config, dict):
        raise ValueError("live_readiness must be an object")
    result = dict(DEFAULT_CONFIG)
    result.update(config)
    if type(result["schema_version"]) is not int or result["schema_version"] != 1:
        raise ValueError("unsupported schema_version")
    for key, fixed in (("expected_system_id", 1), ("expected_component_id", 1),
                       ("vision_position_mask", 128)):
        if _integer(result[key], 1, 255) != fixed:
            raise ValueError(f"this experiment requires {key}={fixed}")
    if _number(result["check_rate_hz"]) != 10:
        raise ValueError("this experiment requires a 10 Hz steady timer")
    for key in ("local_status_max_age_s", "sys_status_max_age_s", "state_max_age_s",
                "fcu_healthy_dwell_s", "maximum_sys_status_gap_s"):
        result[key.removesuffix("_s") + "_ns"] = seconds_to_ns(result[key])
    required = result["required_parameters"]
    if not isinstance(required, dict) or not required:
        raise ValueError("required_parameters must be a nonempty object")
    for key, value in required.items():
        if not isinstance(key, str) or not key:
            raise ValueError("invalid parameter name")
        _number(value)
    for key, value in BASELINE_PARAMETERS.items():
        if key not in required or required[key] != value:
            raise ValueError(f"required_parameters must retain {key}={value}")
    result["required_parameters"] = dict(required)
    return result


def validate_parameter_proof(proof: Any, config: dict) -> list[str]:
    errors = []
    if not isinstance(proof, dict) or not _finite_tree(proof):
        return ["proof_not_finite_object"]
    if type(proof.get("schema_version")) is not int or proof["schema_version"] != 1:
        errors.append("proof_schema_version")
    if proof.get("method") != "mavros_ros2_GetParameters":
        errors.append("proof_method")
    if "service" in proof and proof["service"] != "/uav1/mavros/param/get_parameters":
        errors.append("proof_service")
    for key, expected in (("target_system_id", config["expected_system_id"]),
                          ("target_component_id", config["expected_component_id"])):
        if type(proof.get(key)) is not int or proof[key] != expected:
            errors.append("proof_" + key)
    rows = proof.get("parameters")
    if not isinstance(rows, dict):
        return errors + ["proof_parameters_missing"]
    for name, expected in config["required_parameters"].items():
        row = rows.get(name)
        try:
            if not isinstance(row, dict) or row.get("success") is not True:
                raise ValueError("unsuccessful")
            ros_type = _integer(row.get("ros_type"))
            if ros_type == 2:
                actual = _integer(row.get("integer_value"), -(2**63), 2**63 - 1)
            elif ros_type == 3:
                actual = _number(row.get("double_value"))
            else:
                raise ValueError("unknown parameter type")
            if actual != expected:
                raise ValueError("value mismatch")
        except ValueError:
            errors.append("proof_parameter_" + name)
    return errors


def decode_sys_status(payload_length: Any, payload64: Any, magic: int = 253) -> dict:
    length = _integer(payload_length, 1, 31)
    if type(magic) is not int or magic not in (253, 254) or (magic == 254 and length != 31):
        raise ValueError("invalid SYS_STATUS MAVLink framing/length")
    if not isinstance(payload64, (list, tuple)) or not (0 < len(payload64) <= 33):
        raise ValueError("invalid payload64 shape")
    values = [_integer(v, 0, 2**64 - 1) for v in payload64]
    if len(values) * 8 < length:
        raise ValueError("payload64 does not cover declared length")
    raw = b"".join(struct.pack("<Q", v) for v in values)[:length]
    # MAVLink2 legally trims trailing zero bytes. This is wire decoding, not
    # filling in an absent sensor measurement. Preserve len/payload64 in events.
    present, enabled, health = struct.unpack("<III", raw.ljust(12, b"\0")[:12])
    return {"present": present, "enabled": enabled, "health_mask": health,
            "declared_payload_length": length, "trailing_zero_extension": max(0, 12 - length)}


class ReadinessCore:
    """Fail-closed, integer-ns readiness logic. No ROS/AP actions occur here."""

    def __init__(self, config: dict, parameter_proof: dict):
        self.config = validate_config(config)
        self.proof_errors = validate_parameter_proof(parameter_proof, self.config)
        self.local = None
        self.local_mono = None
        self.local_valid = False
        self.clock_fault_latched = False
        self.state = None
        self.state_mono = None
        self.state_valid = False
        self.armed_seen = False
        self.sys_mono = None
        self.sys_valid = False
        self.sys_reason = "fcu_sys_status_missing"
        self.decoded = None
        self.sys_healthy = None
        self.streak_start = None
        self.streak_last = None
        self.last_snapshot_mono = None
        self.monitor_clock_fault = False
        self.ever_ready = False
        self.counts = Counter()

    def _reset_streak(self):
        self.streak_start = None
        self.streak_last = None

    def observe_local(self, status: Any, mono_ns: int, wall_ns: int | None = None) -> dict:
        mono_ns = _integer(mono_ns)
        if wall_ns is not None:
            _integer(wall_ns)
        # Fault evidence is latched even if an event is replayed; replay must
        # never refresh receive age or clear a previously observed fault.
        if isinstance(status, dict) and status.get("clock_fault_latched") is True:
            self.clock_fault_latched = True
        if self.local_mono is not None and mono_ns <= self.local_mono:
            self.counts["local_nonnew"] += 1
            return {"accepted": False, "reason": "local_event_not_new"}
        self.local_mono = mono_ns
        self.local = status if isinstance(status, dict) else None
        fields = ("local_source_ready", "local_clock_ready", "clock_fault_latched", "healthy")
        self.local_valid = bool(isinstance(status, dict) and _finite_tree(status)
                                and all(type(status.get(k)) is bool for k in fields))
        self.counts["local_events"] += 1
        return {"accepted": True, "valid": self.local_valid}

    def observe_state(self, connected: Any, armed: Any, mono_ns: int,
                      wall_ns: int | None = None) -> dict:
        mono_ns = _integer(mono_ns)
        if wall_ns is not None:
            _integer(wall_ns)
        # Even a replayed armed report must stop this explicitly unarmed monitor.
        if armed is True:
            self.armed_seen = True
        if self.state_mono is not None and mono_ns <= self.state_mono:
            self.counts["state_nonnew"] += 1
            return {"accepted": False, "reason": "state_event_not_new"}
        self.state_mono = mono_ns
        self.state_valid = type(connected) is bool and type(armed) is bool
        self.state = {"connected": connected, "armed": armed}
        self.counts["state_events"] += 1
        return {"accepted": True, "valid": self.state_valid, "armed_seen": self.armed_seen}

    def observe_sys_status(self, msgid: Any, sysid: Any, compid: Any,
                           payload_length: Any, payload64: Any, mono_ns: int,
                           wall_ns: int | None = None, *, framing_status: Any = 1,
                           magic: Any = 253) -> dict:
        mono_ns = _integer(mono_ns)
        if wall_ns is not None:
            _integer(wall_ns)
        if type(msgid) is int and msgid != 1:
            self.counts["other_mavlink_messages"] += 1
            return {"accepted": False, "reason": "not_sys_status"}
        if self.sys_mono is not None and mono_ns <= self.sys_mono:
            self.counts["sys_nonnew"] += 1
            return {"accepted": False, "reason": "sys_event_not_new"}
        previous_health = self.sys_healthy
        previous_mono = self.sys_mono
        self.sys_mono = mono_ns
        self.sys_valid = False
        self.decoded = None
        self.sys_healthy = None
        self.counts["sys_status_events"] += 1
        reason = "fcu_sys_status_invalid"
        try:
            if type(msgid) is not int or msgid != 1:
                raise ValueError("invalid msgid")
            if (type(sysid) is not int or type(compid) is not int or
                    sysid != self.config["expected_system_id"] or
                    compid != self.config["expected_component_id"]):
                reason = "fcu_identity_mismatch"
                raise ValueError(reason)
            if type(framing_status) is not int or framing_status != 1:
                raise ValueError("MAVROS framing status not OK")
            self.decoded = decode_sys_status(payload_length, payload64, magic)
            self.sys_valid = True
            mask = self.config["vision_position_mask"]
            self.sys_healthy = all(self.decoded[k] & mask == mask
                                   for k in ("present", "enabled", "health_mask"))
            reason = "ok" if self.sys_healthy else "fcu_vision_bits_false"
        except (ValueError, TypeError, OverflowError):
            self.counts["invalid_sys_status_events"] += 1
        self.sys_reason = reason
        gap = None if previous_mono is None else mono_ns - previous_mono
        if self.sys_healthy is not True:
            self._reset_streak()
        else:
            if (self.streak_start is None or gap is None or
                    gap > self.config["maximum_sys_status_gap_ns"]):
                self.streak_start = mono_ns
                self.counts["healthy_streak_starts"] += 1
            self.streak_last = mono_ns
        transition = None
        if previous_health is not self.sys_healthy:
            transition = {"previous": previous_health, "current": self.sys_healthy}
            self.counts["fcu_health_transitions"] += 1
        return {"accepted": True, "valid": self.sys_valid, "reason": reason,
                "decoded": self.decoded, "healthy": self.sys_healthy,
                "gap_ns": gap, "health_transition": transition}

    def snapshot(self, now_mono_ns: int) -> dict:
        now = _integer(now_mono_ns)
        if self.last_snapshot_mono is not None and now < self.last_snapshot_mono:
            self.monitor_clock_fault = True
        self.last_snapshot_mono = now
        reasons = []
        if self.proof_errors:
            reasons.append("parameter_proof_invalid")
        if self.armed_seen:
            reasons.append("armed_seen")
        if self.monitor_clock_fault:
            reasons.append("monitor_monotonic_clock_fault")
        local_age = None if self.local_mono is None else now - self.local_mono
        sys_age = None if self.sys_mono is None else now - self.sys_mono
        state_age = None if self.state_mono is None else now - self.state_mono
        if local_age is None:
            reasons.append("local_status_missing")
        elif local_age < 0 or local_age > self.config["local_status_max_age_ns"]:
            reasons.append("local_status_stale")
        if not self.local_valid:
            reasons.append("local_status_invalid")
        else:
            if not self.local["local_source_ready"]:
                reasons.append("local_source_not_ready")
            if not self.local["local_clock_ready"]:
                reasons.append("local_clock_not_ready")
            if not self.local["healthy"]:
                reasons.append("local_not_healthy")
        if self.clock_fault_latched:
            reasons.append("clock_fault_latched")
        if not self.sys_valid:
            reasons.append(self.sys_reason)
        if sys_age is None or sys_age < 0 or sys_age > self.config["sys_status_max_age_ns"]:
            reasons.append("fcu_sys_status_stale")
            self._reset_streak()
        # A snapshot between SYS samples must also notice the maximum gap. It
        # never accumulates healthy dwell itself or refreshes FCU receipt age.
        if sys_age is not None and sys_age > self.config["maximum_sys_status_gap_ns"]:
            self._reset_streak()
        if self.sys_valid and self.sys_healthy is not True:
            for key, suffix in (("present", "present"), ("enabled", "enabled"),
                                ("health_mask", "healthy")):
                if not self.decoded[key] & self.config["vision_position_mask"]:
                    reasons.append("fcu_vision_not_" + suffix)
        dwell = (0 if self.streak_start is None or self.streak_last is None else
                 self.streak_last - self.streak_start)
        if self.streak_start is None or dwell < self.config["fcu_healthy_dwell_ns"]:
            reasons.append("fcu_healthy_warmup")
        if state_age is None:
            reasons.append("fcu_state_missing")
        elif state_age < 0 or state_age > self.config["state_max_age_ns"]:
            reasons.append("fcu_state_stale")
        if not self.state_valid:
            reasons.append("fcu_state_invalid")
        else:
            if not self.state["connected"]:
                reasons.append("fcu_not_connected")
            if self.state["armed"]:
                reasons.append("fcu_not_disarmed")
        ready = not reasons
        self.ever_ready = self.ever_ready or ready
        return {"schema_version": 1, "ready": ready, "reasons": reasons,
                "local_status_age_ns": local_age, "sys_status_age_ns": sys_age,
                "state_age_ns": state_age, "healthy_sample_dwell_ns": dwell,
                "healthy_streak_start_mono_ns": self.streak_start,
                "last_healthy_sample_mono_ns": self.streak_last,
                "parameter_proof_valid": not self.proof_errors,
                "parameter_proof_errors": list(self.proof_errors),
                "fcu_sys_status_valid": self.sys_valid, "fcu_masks": self.decoded,
                "clock_fault_latched": self.clock_fault_latched,
                "monitor_clock_fault": self.monitor_clock_fault,
                "armed_seen": self.armed_seen, "ever_ready": self.ever_ready,
                "scope": SCOPE}


def _safe_json(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite": repr(value)}
    if isinstance(value, dict):
        return {str(k): _safe_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe_json(v) for v in value]
    if value is None or type(value) in (str, int, float, bool):
        return value
    return repr(value)


def _load_json(path: Path) -> Any:
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--parameter-proof", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args(argv)
    duration_ns = seconds_to_ns(args.duration)
    config = _load_json(args.config)
    proof = _load_json(args.parameter_proof)
    core = ReadinessCore(config, proof)
    # Exclusive new directory/files; no existing artifact is replaced.
    args.output_dir.mkdir(exist_ok=False)
    output = args.output_dir.resolve()
    metadata = {"schema_version": 1, "scope": SCOPE, "config": config,
                "parameter_proof": proof, "configured_target_is_not_parameter_response_identity": True,
                "config_path": str(args.config.resolve()), "config_sha256": _sha(args.config),
                "parameter_proof_path": str(args.parameter_proof.resolve()),
                "parameter_proof_sha256": _sha(args.parameter_proof),
                "script_sha256": _sha(Path(__file__)), "duration_ns": duration_ns,
                "timing": "monotonic ns ages/dwell; independent original wall ns recorded; no cross-domain subtraction",
                "age_scope": "observer receive age; SYS_STATUS contains no FCU acquisition timestamp or per-ODOMETRY sequence acknowledgement",
                "source_evidence": [], "services_created": [],
                "published_topics": ["/impact/extnav/readiness"],
                "subscribed_topics": ["/uas1/mavlink_source", "/uav1/mavros/state", "/xq/p4/extnav/status"]}
    for relative, lines in SOURCE_EVIDENCE.items():
        path = Path("/home/accelerate/ardupilot") / relative
        metadata["source_evidence"].append({"path": str(path), "line_boundaries": lines,
                                            "sha256": _sha(path) if path.is_file() else None})
    with (output / "metadata.json").open("x", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2, allow_nan=False)
        handle.write("\n")
    events = (output / "events.jsonl").open("x", encoding="utf-8", buffering=1)
    event_counts = Counter()
    start_mono = time.monotonic_ns()
    stop_reason = None
    errors = []
    node = None
    rclpy = None
    previous_ready = None

    def record(kind, data=None, mono=None, wall=None):
        event_counts[kind] += 1
        events.write(json.dumps(_safe_json({"event": kind,
                     "monotonic_ns": time.monotonic_ns() if mono is None else mono,
                     "wall_ns": time.time_ns() if wall is None else wall,
                     "data": data}), allow_nan=False, separators=(",", ":")) + "\n")

    def interrupted(signum, _frame):
        nonlocal stop_reason
        stop_reason = "signal_" + str(signum)

    old_handlers = {sig: signal.signal(sig, interrupted) for sig in (signal.SIGINT, signal.SIGTERM)}
    record("start", {"parameter_proof_errors": core.proof_errors})
    try:
        # ROS is deliberately imported only in main, for dependency-free tests.
        import rclpy as ros
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from mavros_msgs.msg import Mavlink, State
        from std_msgs.msg import String
        rclpy = ros
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("impact_extnav_readiness_monitor")
        publisher = node.create_publisher(String, "/impact/extnav/readiness", 10)

        def emit():
            nonlocal previous_ready, stop_reason
            mono, wall = time.monotonic_ns(), time.time_ns()
            status = core.snapshot(mono)
            status.update({"monotonic_ns": mono, "wall_ns": wall})
            message = String()
            message.data = json.dumps(status, allow_nan=False, separators=(",", ":"))
            publisher.publish(message)
            record("readiness", status, mono, wall)
            signature = (status["ready"], tuple(status["reasons"]))
            if signature != previous_ready:
                record("readiness_transition", status, mono, wall)
                previous_ready = signature
            if core.armed_seen:
                stop_reason = "armed_seen"

        def mavlink_callback(message):
            mono, wall = time.monotonic_ns(), time.time_ns()
            raw = {name: getattr(message, name) for name in
                   ("framing_status", "magic", "len", "incompat_flags", "compat_flags", "seq",
                    "sysid", "compid", "msgid", "checksum")}
            raw.update({"payload64": list(message.payload64), "signature": list(message.signature),
                        "header": {"stamp_sec": message.header.stamp.sec,
                                   "stamp_nanosec": message.header.stamp.nanosec,
                                   "frame_id": message.header.frame_id}})
            result = core.observe_sys_status(message.msgid, message.sysid, message.compid,
                       message.len, raw["payload64"], mono, wall,
                       framing_status=message.framing_status, magic=message.magic)
            record("mavlink", {"raw": raw, "result": result}, mono, wall)
            if result.get("health_transition") is not None:
                record("fcu_health_transition", result["health_transition"], mono, wall)

        def local_callback(message):
            mono, wall = time.monotonic_ns(), time.time_ns()
            try:
                status = json.loads(message.data)
            except (ValueError, TypeError):
                status = None
            result = core.observe_local(status, mono, wall)
            record("local_status", {"raw": message.data, "result": result}, mono, wall)

        def state_callback(message):
            nonlocal stop_reason
            mono, wall = time.monotonic_ns(), time.time_ns()
            result = core.observe_state(message.connected, message.armed, mono, wall)
            record("fcu_state", {"connected": message.connected, "armed": message.armed,
                   "mode": message.mode, "system_status": message.system_status,
                   "header_stamp_sec": message.header.stamp.sec,
                   "header_stamp_nanosec": message.header.stamp.nanosec,
                   "result": result}, mono, wall)
            if core.armed_seen:
                stop_reason = "armed_seen"
                emit()

        # BEST_EFFORT accepts the FCU transport's best-effort publisher. State
        # and local health remain bounded by their own receive ages.
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions = [node.create_subscription(Mavlink, "/uas1/mavlink_source", mavlink_callback, qos),
                         node.create_subscription(State, "/uav1/mavros/state", state_callback, qos),
                         node.create_subscription(String, "/xq/p4/extnav/status", local_callback, qos)]
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        timer = node.create_timer(0.1, emit, clock=steady_clock)
        emit()
        while stop_reason is None and time.monotonic_ns() - start_mono < duration_ns:
            if not rclpy.ok():
                stop_reason = "ros_shutdown"
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        if stop_reason is None:
            stop_reason = "duration_complete"
        timer.cancel()
    except Exception as exc:
        stop_reason = "exception"
        errors.append(repr(exc))
        record("exception", {"exception": repr(exc)})
    finally:
        end_mono, end_wall = time.monotonic_ns(), time.time_ns()
        final = core.snapshot(end_mono)
        status = "PASS" if stop_reason == "duration_complete" and not core.armed_seen else "FAIL"
        summary = {"schema_version": 1, "status": status, "stop_reason": stop_reason,
                   "pass_meaning": "Observation completed safely; PASS does not mean readiness, link continuity, fusion, or flight passed.",
                   "scope": SCOPE, "start_monotonic_ns": start_mono,
                   "end_monotonic_ns": end_mono, "end_wall_ns": end_wall,
                   "observed_duration_ns": end_mono - start_mono, "current_ready": final["ready"],
                   "ever_ready": core.ever_ready, "armed_seen": core.armed_seen,
                   "parameter_proof_valid": not core.proof_errors, "final": final,
                   "core_counts": dict(core.counts), "event_counts": dict(event_counts), "errors": errors}
        record("end", summary, end_mono, end_wall)
        events.flush()
        events.close()
        with (output / "summary.json").open("x", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, allow_nan=False)
            handle.write("\n")
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
