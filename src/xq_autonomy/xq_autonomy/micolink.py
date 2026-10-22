"""Bounded Micolink 0x51 framing and an explicitly limited transport guard.

Official references (checked 2026-10-19):
https://micoair.cn/zh/docs/sensors/micolink
https://micoair.com/docs/decoding-micolink-messages-from-mtf-01/

The 20-byte layout is the published packed C struct.  This local profile uses
little-endian decoding; the published wire frame contains no protocol-version
field.  Real firmware/byte-order interoperability still needs a device capture.
No serial device, ROS publisher, flight controller, or unit conversion is used.

``accepted`` means framing/serial chronology/receipt freshness only.  The device
system-millisecond counter is NOT an exposure timestamp.  Receipt freshness does
NOT prove acquisition freshness, rotational compensation, gyro-window coverage,
metric velocity validity, calibrated uncertainty, or statistical independence.
Raw flow units are cm/s at 1 m; the fields are not measured metric velocities.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import struct


PROTOCOL_PROFILE = "micolink_0x51_20_le"
MESSAGE_ID = 0x51
PAYLOAD_LENGTH = 20
MAX_PAYLOAD_LENGTH = 64
MAX_FRAME_LENGTH = MAX_PAYLOAD_LENGTH + 7
_PAYLOAD = struct.Struct("<IIBBBBhhBBH")


@dataclass(frozen=True)
class MicolinkRangeFlow:
    device_id: int
    system_id: int
    message_id: int
    sequence: int
    payload_length: int
    time_ms: int
    distance_mm: int
    strength: int
    precision: int
    tof_status: int
    reserved1: int
    flow_vel_x: int
    flow_vel_y: int
    flow_quality: int
    flow_status: int
    reserved2: int
    raw_frame: bytes

    def to_dict(self):
        result = asdict(self)
        result["raw_frame_hex"] = result.pop("raw_frame").hex()
        result.update(protocol_profile=PROTOCOL_PROFILE,
                      protocol_version_verified=False,
                      flow_units="cm/s@1m", counter_units="system_ms")
        return result


class MicolinkParser:
    """Incremental framing with at most 71 buffered bytes.

    Device/system IDs default to the official diagram's 0x0f/0x00.  Alternative
    IDs require an explicit profile choice at construction.  Unknown message
    IDs and layouts are counted and never emitted as range/flow measurements.
    last_rejection retains one bounded raw record for diagnostic evidence.
    """

    def __init__(self, *, protocol_profile=PROTOCOL_PROFILE,
                 expected_device_id=0x0F, expected_system_id=0x00):
        if protocol_profile != PROTOCOL_PROFILE:
            raise ValueError("unsupported protocol profile; no version auto-detection")
        for value in (expected_device_id, expected_system_id):
            if type(value) is not int or not 0 <= value <= 255:
                raise ValueError("expected IDs must be explicit uint8 values")
        self.protocol_profile = protocol_profile
        self.expected_device_id = expected_device_id
        self.expected_system_id = expected_system_id
        self._buffer = bytearray()
        self.last_rejection = None
        self.counters = dict.fromkeys((
            "bytes_received", "frames_accepted", "checksum_errors",
            "invalid_lengths", "unknown_message_ids", "unexpected_device_ids",
            "unexpected_system_ids", "discarded_bytes"), 0)

    @property
    def buffered_bytes(self):
        return len(self._buffer)

    def _reject(self, counter, reason, raw):
        self.counters[counter] += 1
        self.last_rejection = {"reason": reason, "raw_frame_hex": bytes(raw).hex()}

    def feed(self, data):
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("feed requires bytes-like input")
        accepted = []
        for byte in memoryview(data).cast("B"):
            self.counters["bytes_received"] += 1
            self._buffer.append(byte)
            while self._buffer:
                if self._buffer[0] != 0xEF:
                    del self._buffer[0]
                    self.counters["discarded_bytes"] += 1
                    continue
                if len(self._buffer) < 6:
                    break
                length = self._buffer[5]
                if length > MAX_PAYLOAD_LENGTH:
                    self._reject("invalid_lengths", "payload_length_over_limit", self._buffer[:6])
                    del self._buffer[0]
                    self.counters["discarded_bytes"] += 1
                    continue
                total = length + 7
                if len(self._buffer) < total:
                    break
                frame = bytes(self._buffer[:total])
                if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                    self._reject("checksum_errors", "checksum_mismatch", frame)
                    # Search the suffix again, preserving an embedded start of
                    # the next frame after insertion/deletion/corruption.
                    del self._buffer[0]
                    self.counters["discarded_bytes"] += 1
                    continue
                del self._buffer[:total]
                if frame[3] != MESSAGE_ID:
                    self._reject("unknown_message_ids", "unknown_message_id", frame)
                elif length != PAYLOAD_LENGTH:
                    self._reject("invalid_lengths", "unsupported_0x51_payload_length", frame)
                elif frame[1] != self.expected_device_id:
                    self._reject("unexpected_device_ids", "unexpected_device_id", frame)
                elif frame[2] != self.expected_system_id:
                    self._reject("unexpected_system_ids", "unexpected_system_id", frame)
                else:
                    values = _PAYLOAD.unpack(frame[6:-1])
                    accepted.append(MicolinkRangeFlow(
                        frame[1], frame[2], frame[3], frame[4], length, *values, frame))
                    self.counters["frames_accepted"] += 1
                if len(self._buffer) > MAX_FRAME_LENGTH:
                    raise AssertionError("internal framing bound exceeded")
        return accepted


@dataclass(frozen=True)
class MicolinkGuardConfig:
    """Transport policy values, not calibrated physical sensor guarantees."""

    max_receipt_age_s: float = 0.25
    max_transport_gap_s: float = 0.5
    max_sensor_step_ms: int = 500
    max_forward_sequence_gap: int = 127

    def __post_init__(self):
        for name in ("max_receipt_age_s", "max_transport_gap_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_transport_gap_s < self.max_receipt_age_s:
            raise ValueError("max_transport_gap_s must be at least max_receipt_age_s")
        if type(self.max_sensor_step_ms) is not int or not 0 < self.max_sensor_step_ms < 2**31:
            raise ValueError("max_sensor_step_ms must be in (0, 2**31)")
        if type(self.max_forward_sequence_gap) is not int or not 1 <= self.max_forward_sequence_gap <= 127:
            raise ValueError("max_forward_sequence_gap must be in [1, 127]")


class MicolinkChronologyGuard:
    """Track modular counters without mapping them to an acquisition clock.

    new_session(unique_id) is required initially and after a latched failure.
    Forward sequence deltas 1..configured_limit (< half the uint8 range) and
    small positive uint32 time deltas can cross wrap.  A long wall gap, backward
    or ambiguous delta, or wall-clock rollback requires a new session.  This
    cannot authenticate a device or distinguish a reset exactly mimicking wrap.
    """

    def __init__(self, config=None):
        self.config = config if config is not None else MicolinkGuardConfig()
        if not isinstance(self.config, MicolinkGuardConfig):
            raise TypeError("config must be MicolinkGuardConfig")
        self.counters = dict.fromkeys((
            "observed_frames", "accepted_frames", "rejected_frames", "session_resets",
            "duplicate_sequences", "duplicate_times", "sequence_discontinuities",
            "counter_discontinuities", "long_gaps", "stale_receipts", "future_receipts",
            "receipt_rollbacks", "monotonic_rollbacks", "sequence_wraps", "counter_wraps"), 0)
        self.session_id = None
        self._last_message = None
        self._last_receipt = None
        self._last_now = None
        self._counter_elapsed_ms = 0
        self._latched_reason = "new_session_required"

    def new_session(self, session_id):
        if not isinstance(session_id, str) or not session_id.strip():
            raise ValueError("session_id must be nonempty")
        if session_id == self.session_id:
            raise ValueError("new_session requires a different explicit session_id")
        self.session_id = session_id
        self._last_message = None
        self._last_receipt = None
        self._last_now = None
        self._counter_elapsed_ms = 0
        self._latched_reason = None
        self.counters["session_resets"] += 1

    @staticmethod
    def _finite_time(value):
        return type(value) in (int, float) and math.isfinite(value) and value >= 0

    def _clock(self, now):
        if not self._finite_time(now):
            return "invalid_monotonic_time"
        if self._last_now is not None and now < self._last_now:
            self.counters["monotonic_rollbacks"] += 1
            self._latched_reason = "monotonic_rollback_requires_new_session"
        self._last_now = now
        return self._latched_reason

    def _report(self, now, *, accepted=False, reason=None):
        age = now - self._last_receipt if self._finite_time(now) and self._last_receipt is not None else None
        fresh = age is not None and 0 <= age <= self.config.max_receipt_age_s
        chronology = self._last_message is not None and self._latched_reason is None
        if self._latched_reason:
            status = "SESSION_REQUIRED"
        elif self._last_message is None:
            status = "WAITING"
        else:
            status = "READY" if fresh else "STALE"
        message = self._last_message
        range_status = message.tof_status if message else None
        flow_status = message.flow_status if message else None
        reported = lambda value: True if value == 1 else False if value == 0 else None
        return {
            "status": status, "accepted": accepted,
            "reason": reason or self._latched_reason or ("receipt_fresh" if fresh else "no_fresh_accepted_frame"),
            "session_id": self.session_id, "serial_chronology_valid": chronology,
            "transport_fresh": fresh, "receipt_age_s": age,
            "last_receipt_monotonic_s": self._last_receipt,
            "counter_elapsed_ms": self._counter_elapsed_ms,
            "last_message": message.to_dict() if message else None,
            "reported_range_usable": False if message is not None and message.distance_mm == 0 else reported(range_status),
            "reported_flow_usable": reported(flow_status),
            "quality_calibration_known": False,
            "protocol_version_verified": False,
            "acquisition_valid": False, "acquisition_timestamp_s": None,
            "timestamp_semantics": "unknown_acquisition; device_system_counter_only",
            "gyro_coverage_valid": False, "rotational_compensation_known": False,
            "measurement_valid": False, "metric_valid": False, "metric_velocity_m_s": None,
            "counters": dict(self.counters),
        }

    def _reject(self, now, reason, counter=None, *, latch=False):
        self.counters["rejected_frames"] += 1
        if counter:
            self.counters[counter] += 1
        if latch:
            self._latched_reason = reason
        return self._report(now, reason=reason)

    def observe(self, message, *, receipt_monotonic_s, now_monotonic_s):
        self.counters["observed_frames"] += 1
        clock_reason = self._clock(now_monotonic_s)
        if clock_reason:
            return self._reject(now_monotonic_s, clock_reason)
        if not isinstance(message, MicolinkRangeFlow):
            return self._reject(now_monotonic_s, "unsupported_message_object")
        if message.message_id != MESSAGE_ID or message.payload_length != PAYLOAD_LENGTH:
            return self._reject(now_monotonic_s, "unsupported_message_layout")
        if type(message.sequence) is not int or not 0 <= message.sequence <= 255 or type(message.time_ms) is not int or not 0 <= message.time_ms <= 0xFFFFFFFF:
            return self._reject(now_monotonic_s, "invalid_raw_counter")
        if not self._finite_time(receipt_monotonic_s):
            return self._reject(now_monotonic_s, "invalid_receipt_time")
        if receipt_monotonic_s > now_monotonic_s:
            return self._reject(now_monotonic_s, "future_receipt", "future_receipts")
        if now_monotonic_s - receipt_monotonic_s > self.config.max_receipt_age_s:
            return self._reject(now_monotonic_s, "stale_receipt", "stale_receipts")
        if self._last_receipt is not None:
            if receipt_monotonic_s < self._last_receipt:
                return self._reject(now_monotonic_s, "receipt_rollback_requires_new_session", "receipt_rollbacks", latch=True)
            if now_monotonic_s - self._last_receipt > self.config.max_transport_gap_s:
                return self._reject(now_monotonic_s, "transport_gap_requires_new_session", "long_gaps", latch=True)
        if self._last_message is not None:
            sequence_delta = (message.sequence - self._last_message.sequence) & 0xFF
            counter_delta = (message.time_ms - self._last_message.time_ms) & 0xFFFFFFFF
            if sequence_delta == 0:
                return self._reject(now_monotonic_s, "duplicate_sequence", "duplicate_sequences")
            if sequence_delta > self.config.max_forward_sequence_gap:
                return self._reject(now_monotonic_s, "sequence_discontinuity_requires_new_session", "sequence_discontinuities", latch=True)
            if counter_delta == 0:
                return self._reject(now_monotonic_s, "duplicate_system_counter", "duplicate_times")
            if counter_delta > self.config.max_sensor_step_ms:
                return self._reject(now_monotonic_s, "counter_discontinuity_requires_new_session", "counter_discontinuities", latch=True)
            if message.sequence < self._last_message.sequence:
                self.counters["sequence_wraps"] += 1
            if message.time_ms < self._last_message.time_ms:
                self.counters["counter_wraps"] += 1
            self._counter_elapsed_ms += counter_delta
        self._last_message = message
        self._last_receipt = receipt_monotonic_s
        self.counters["accepted_frames"] += 1
        return self._report(now_monotonic_s, accepted=True, reason="serial_counter_and_receipt_accepted")

    def status(self, *, now_monotonic_s):
        reason = self._clock(now_monotonic_s)
        if not reason and self._last_receipt is not None and now_monotonic_s - self._last_receipt > self.config.max_transport_gap_s:
            self.counters["long_gaps"] += 1
            self._latched_reason = "transport_gap_requires_new_session"
        return self._report(now_monotonic_s, reason=reason)
