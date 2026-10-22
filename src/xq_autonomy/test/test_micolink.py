"""Adversarial, synthetic Micolink streams; no hardware capture or serial device.

The literal below is hand assembled independently of the packet helper. Its
device/system IDs are explicit test labels, not claims about delivered hardware.
Passing these tests establishes parsing and chronology only, never exposure
timing, physical calibration, or navigation eligibility.
"""

from dataclasses import FrozenInstanceError
import json
import struct

import pytest

from xq_autonomy.micolink import (
    MicolinkChronologyGuard, MicolinkGuardConfig, MicolinkParser,
)


# SYNTHETIC, NOT CAPTURED: header EF/01/02/51/FE/14; time 0x01020304,
# distance 1000 mm, strength 40, precision 2, ToF status 1, reserved1 0,
# flow -123/+456 cm/s@1m, quality 200, flow status 1, reserved2 0.
# Sum of the preceding 26 bytes is 1675; its low byte is 0x8B.
SYNTHETIC_LITERAL = bytes.fromhex(
    "EF010251FE14 04030201 E8030000 28020100 85FFC801 C8010000 8B"
)


def parser():
    return MicolinkParser(expected_device_id=1, expected_system_id=2)


def packet(*, sequence=1, time_ms=1000, distance_mm=1000,
           strength=40, precision=2, tof_status=1, reserved1=0,
           flow_x=-123, flow_y=456, quality=200, flow_status=1,
           reserved2=0, device_id=1, system_id=2, message_id=0x51):
    payload = struct.pack(
        "<IIBBBBhhBBH", time_ms, distance_mm, strength, precision,
        tof_status, reserved1, flow_x, flow_y, quality, flow_status, reserved2,
    )
    frame = bytes((0xEF, device_id, system_id, message_id, sequence, len(payload))) + payload
    return frame + bytes((sum(frame) & 0xFF,))


def message(**kwargs):
    decoded = parser().feed(packet(**kwargs))
    assert len(decoded) == 1
    return decoded[0]


def new_guard(config=None):
    guard = MicolinkChronologyGuard(config)
    guard.new_session("synthetic_initial_session")
    return guard


def observe(guard, *, sequence=1, time_ms=1000, receipt=10.0, now=None,
            **kwargs):
    return guard.observe(
        message(sequence=sequence, time_ms=time_ms, **kwargs),
        receipt_monotonic_s=receipt,
        now_monotonic_s=receipt if now is None else now,
    )


def assert_no_physical_claims(result):
    assert result["acquisition_valid"] is False
    assert result["gyro_coverage_valid"] is False
    assert result["metric_valid"] is False
    assert result["measurement_valid"] is False
    assert result["quality_calibration_known"] is False
    # A system-millisecond counter and nominal output rate do not establish
    # exposure start/end, an integration window, a gyro integral, or velocity.
    for key in ("acquisition_start_ns", "acquisition_end_ns",
                "integration_time_s", "gyro_integral_rad", "metric_velocity_mps"):
        assert result.get(key) is None
    json.dumps(result, allow_nan=False)


def test_hand_assembled_literal_decodes_little_endian_signed_fields():
    assert len(SYNTHETIC_LITERAL) == 27
    assert sum(SYNTHETIC_LITERAL[:-1]) == 1675
    decoder = parser()
    [sample] = decoder.feed(SYNTHETIC_LITERAL)
    assert (sample.device_id, sample.system_id, sample.message_id) == (1, 2, 0x51)
    assert (sample.sequence, sample.payload_length) == (254, 20)
    assert sample.time_ms == 0x01020304
    assert sample.distance_mm == 1000
    assert (sample.strength, sample.precision, sample.tof_status) == (40, 2, 1)
    assert (sample.flow_vel_x, sample.flow_vel_y) == (-123, 456)
    assert (sample.flow_quality, sample.flow_status) == (200, 1)
    assert (sample.reserved1, sample.reserved2) == (0, 0)
    assert sample.raw_frame == SYNTHETIC_LITERAL
    assert sample.to_dict()["raw_frame_hex"] == SYNTHETIC_LITERAL.hex()
    assert decoder.counters["frames_accepted"] == 1
    with pytest.raises(FrozenInstanceError):
        sample.time_ms = 0


@pytest.mark.parametrize("split", range(1, 27))
def test_every_truncation_waits_for_remainder_without_output(split):
    decoder = parser()
    assert decoder.feed(SYNTHETIC_LITERAL[:split]) == []
    assert decoder.counters["frames_accepted"] == 0
    [sample] = decoder.feed(SYNTHETIC_LITERAL[split:])
    assert sample.raw_frame == SYNTHETIC_LITERAL
    assert decoder.buffered_bytes == 0


def test_single_byte_chunks_noise_and_concatenated_frames_are_invariant():
    stream = b"\x00\x11\xee" + SYNTHETIC_LITERAL + b"noise" + packet(sequence=255, time_ms=0x0102030E)
    whole = parser().feed(stream)
    decoder = parser()
    fragmented = []
    for byte in stream:
        fragmented.extend(decoder.feed(bytes((byte,))))
    assert [sample.raw_frame for sample in fragmented] == [sample.raw_frame for sample in whole]
    assert len(fragmented) == 2
    assert decoder.counters["bytes_received"] == len(stream)
    assert decoder.counters["discarded_bytes"] >= 8


@pytest.mark.parametrize("length", [0, 19, 21, 255])
def test_wrong_known_message_length_cannot_swallow_next_valid_frame(length):
    decoder = parser()
    payload = (SYNTHETIC_LITERAL[6:-1] + bytes(255))[:length]
    malformed = SYNTHETIC_LITERAL[:5] + bytes((length,)) + payload
    malformed += bytes((sum(malformed) & 0xFF,))
    decoded = decoder.feed(malformed + SYNTHETIC_LITERAL)
    assert [sample.raw_frame for sample in decoded] == [SYNTHETIC_LITERAL]
    assert decoder.counters["invalid_lengths"] >= 1


@pytest.mark.parametrize("index", range(27))
def test_one_bit_corruption_is_rejected_and_parser_recovers(index):
    malformed = bytearray(SYNTHETIC_LITERAL)
    malformed[index] ^= 1
    decoder = parser()
    decoded = decoder.feed(bytes(malformed) + SYNTHETIC_LITERAL)
    assert [sample.raw_frame for sample in decoded] == [SYNTHETIC_LITERAL]
    assert decoder.counters["frames_accepted"] == 1


def test_checksum_is_additive_and_bad_checksum_is_diagnosed():
    decoder = parser()
    bad = SYNTHETIC_LITERAL[:-1] + bytes((SYNTHETIC_LITERAL[-1] ^ 0x80,))
    assert decoder.feed(bad) == []
    assert decoder.counters["checksum_errors"] == 1
    assert len(decoder.feed(SYNTHETIC_LITERAL)) == 1


@pytest.mark.parametrize("changes,counter", [
    ({"message_id": 0x52}, "unknown_message_ids"),
    ({"device_id": 3}, "unexpected_device_ids"),
    ({"system_id": 3}, "unexpected_system_ids"),
])
def test_checksum_valid_unknown_ids_are_diagnostics_not_measurements(changes, counter):
    decoder = parser()
    bad = packet(**changes)
    assert decoder.feed(bad) == []
    assert decoder.counters[counter] >= 1
    assert decoder.last_rejection["raw_frame_hex"] == bad.hex()
    assert len(decoder.feed(SYNTHETIC_LITERAL)) == 1


def test_header_byte_inside_payload_is_data_not_resynchronization():
    raw = packet(time_ms=0xEFEFEFEF, distance_mm=0x0000EFEF,
                 reserved1=0xEF, reserved2=0xEFEF, quality=0xEF)
    [sample] = parser().feed(raw)
    assert sample.raw_frame == raw
    assert sample.time_ms == 0xEFEFEFEF
    assert sample.distance_mm == 0xEFEF
    assert (sample.reserved1, sample.reserved2, sample.flow_quality) == (0xEF, 0xEFEF, 0xEF)


def test_untrusted_noise_has_bounded_retained_buffer_and_recovers():
    decoder = parser()
    noise = b"\xef\x01\x02\x51\x00\xff" * 4096
    assert decoder.feed(noise) == []
    assert decoder.buffered_bytes < 1024
    [sample] = decoder.feed(SYNTHETIC_LITERAL)
    assert sample.raw_frame == SYNTHETIC_LITERAL
    assert decoder.buffered_bytes == 0


def test_profile_is_explicit_and_not_inferred_from_payload():
    with pytest.raises(ValueError):
        MicolinkParser(protocol_profile="unknown_firmware")
    default_decoder = MicolinkParser()
    assert default_decoder.feed(SYNTHETIC_LITERAL) == []
    [sample] = default_decoder.feed(packet(device_id=0x0F, system_id=0))
    assert (sample.device_id, sample.system_id) == (0x0F, 0)


@pytest.mark.parametrize("x,y", [(-32768, 32767), (32767, -32768), (0, 0)])
def test_signed_native_units_are_preserved_without_velocity_or_angle_invention(x, y):
    sample = message(flow_x=x, flow_y=y, distance_mm=1234)
    assert (sample.flow_vel_x, sample.flow_vel_y, sample.distance_mm) == (x, y, 1234)
    guard = new_guard()
    result = guard.observe(sample, receipt_monotonic_s=10.0, now_monotonic_s=10.0)
    assert result["accepted"] is True
    assert_no_physical_claims(result)


def test_sequence_and_uint32_system_millisecond_wrap_are_accepted_together():
    guard = new_guard()
    for sequence, stamp, receipt in (
        (254, 0xFFFFFFEC, 10.0), (255, 0xFFFFFFF6, 10.01),
        (0, 0, 10.02), (1, 10, 10.03),
    ):
        result = observe(guard, sequence=sequence, time_ms=stamp, receipt=receipt)
        assert result["accepted"] is True, result
        assert result["serial_chronology_valid"] is True
        assert_no_physical_claims(result)


def test_initial_session_is_explicit_and_same_identifier_cannot_erase_replay_history():
    guard = MicolinkChronologyGuard()
    assert observe(guard)["accepted"] is False
    guard.new_session("synthetic_explicit_session")
    assert observe(guard)["accepted"]
    with pytest.raises(ValueError):
        guard.new_session("synthetic_explicit_session")
    assert observe(guard, receipt=10.01)["accepted"] is False


@pytest.mark.parametrize("sequence,time_ms", [(1, 1000), (1, 1010), (2, 1000)])
def test_duplicate_sequence_or_system_time_does_not_refresh_health(sequence, time_ms):
    guard = new_guard()
    assert observe(guard)["accepted"]
    replay = observe(guard, sequence=sequence, time_ms=time_ms, receipt=10.2)
    assert replay["accepted"] is False
    status = guard.status(now_monotonic_s=10.26)
    assert status["transport_fresh"] is False


@pytest.mark.parametrize("sequence,time_ms", [
    (0, 990), (2, 10), (129, 1010), (2, 1600),
])
def test_backward_ambiguous_or_restart_jump_requires_explicit_new_session(sequence, time_ms):
    guard = new_guard()
    assert observe(guard)["accepted"]
    rejected = observe(guard, sequence=sequence, time_ms=time_ms, receipt=10.01)
    assert rejected["accepted"] is False
    # A following plausible packet must not silently erase a reset alarm.
    assert observe(guard, sequence=2, time_ms=1010, receipt=10.02)["accepted"] is False
    guard.new_session("synthetic_power_cycle_2")
    recovered = observe(guard, sequence=0, time_ms=0, receipt=10.03)
    assert recovered["accepted"] is True
    assert_no_physical_claims(recovered)


def test_late_receipt_cannot_refresh_or_advance_last_accepted_sample():
    guard = new_guard()
    assert observe(guard)["accepted"]
    late = observe(guard, sequence=2, time_ms=1010, receipt=10.01, now=10.27)
    assert late["accepted"] is False
    assert guard.status(now_monotonic_s=10.27)["transport_fresh"] is False
    # Retry the same sequence/time with a genuinely current receipt. If the
    # late packet poisoned chronology, this would be incorrectly called replay.
    current = observe(guard, sequence=2, time_ms=1010, receipt=10.28)
    assert current["accepted"] is True


def test_long_transport_gap_cannot_be_hidden_by_advancing_counter():
    guard = new_guard()
    assert observe(guard)["accepted"]
    result = observe(guard, sequence=2, time_ms=1010, receipt=10.6)
    assert result["accepted"] is False
    assert observe(guard, sequence=3, time_ms=1020, receipt=10.61)["accepted"] is False
    guard.new_session("synthetic_reconnected")
    assert observe(guard, sequence=3, time_ms=1020, receipt=10.62)["accepted"]


@pytest.mark.parametrize("receipt,now", [
    (float("nan"), 10.01), (10.01, float("inf")), (10.02, 10.01),
])
def test_invalid_or_future_receipt_does_not_refresh_health(receipt, now):
    guard = new_guard()
    assert observe(guard)["accepted"]
    result = observe(guard, sequence=2, time_ms=1010, receipt=receipt, now=now)
    assert result["accepted"] is False
    assert guard.status(now_monotonic_s=10.26)["transport_fresh"] is False


def test_receipt_clock_rollback_latches_until_explicit_session_reset():
    guard = new_guard()
    assert observe(guard)["accepted"]
    assert guard.status(now_monotonic_s=9.99)["serial_chronology_valid"] is False
    assert observe(guard, sequence=2, time_ms=1010, receipt=10.01)["accepted"] is False
    guard.new_session("synthetic_new_receipt_clock")
    assert observe(guard, receipt=1.0)["accepted"]


@pytest.mark.parametrize("tof_status,flow_status,distance_mm,range_usable,flow_usable", [
    (1, 1, 1000, True, True),
    (0, 1, 1000, False, True),
    (1, 0, 1000, True, False),
    (1, 1, 0, False, True),
    (2, 1, 1000, None, True),
    (1, 255, 1000, True, None),
])
def test_range_and_flow_status_are_independent_and_unknown_states_remain_unknown(
    tof_status, flow_status, distance_mm, range_usable, flow_usable,
):
    result = observe(new_guard(), tof_status=tof_status,
                     flow_status=flow_status, distance_mm=distance_mm)
    assert result["accepted"] is True
    assert result["reported_range_usable"] is range_usable
    assert result["reported_flow_usable"] is flow_usable
    assert_no_physical_claims(result)


@pytest.mark.parametrize("quality", [0, 1, 255])
def test_quality_extrema_do_not_claim_calibrated_covariance_or_integrity(quality):
    result = observe(new_guard(), quality=quality)
    assert result["accepted"] is True
    assert_no_physical_claims(result)
    assert result.get("covariance") is None
    assert result.get("integrity_probability") is None


def test_configured_chronology_limits_are_enforced_without_host_time_restamping():
    guard = new_guard(MicolinkGuardConfig(
        max_receipt_age_s=0.1, max_transport_gap_s=0.3,
        max_sensor_step_ms=20, max_forward_sequence_gap=2,
    ))
    assert observe(guard)["accepted"]
    assert observe(guard, sequence=3, time_ms=1020, receipt=10.02)["accepted"]
    assert observe(guard, sequence=6, time_ms=1030, receipt=10.03)["accepted"] is False
    assert_no_physical_claims(guard.status(now_monotonic_s=10.04))
