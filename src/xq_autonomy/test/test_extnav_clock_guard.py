"""Pure clock-contract tests: no ROS, AP, Gazebo, or system clock changes."""
from dataclasses import FrozenInstanceError
import json
import math

import pytest

from xq_autonomy.extnav_clock_guard import (
    ClockGuardPolicy,
    ClockGuardStep,
    ExtnavClockGuard,
)


NS = 1_000_000_000
SOURCE = 71 * NS
WALL = 1_790_000_000 * NS + 123
MONO = 101 * NS


def feed(guard, *, interval=100_000_000, duration=5 * NS,
         source=SOURCE, wall=WALL, mono=MONO):
    result = None
    for elapsed in range(0, duration + 1, interval):
        result = guard.observe(source + elapsed, wall + elapsed, mono + elapsed)
    return result


def test_policy_and_step_are_frozen_and_default_thresholds_are_explicit():
    policy = ClockGuardPolicy()
    assert policy.warmup_ns == 5 * NS
    assert policy.max_source_age_ns == 350_000_000
    assert policy.min_rate_hz == 4
    assert policy.max_step_residual_ns == policy.max_anchor_drift_ns == 50_000_000
    with pytest.raises(FrozenInstanceError):
        policy.warmup_ns = NS
    step = ClockGuardStep(True, True, False, ('warming_up',))
    with pytest.raises(FrozenInstanceError):
        step.clock_ready = True


@pytest.mark.parametrize('kwargs', [
    {'warmup_ns': True}, {'warmup_ns': 5e9}, {'warmup_ns': 0},
    {'max_source_age_ns': -1}, {'max_source_age_ns': 350_000_001},
    {'min_rate_hz': 3.999}, {'min_rate_hz': True}, {'min_rate_hz': math.inf},
    {'min_rate_hz': math.nan}, {'min_rate_hz': '4'},
    {'warmup_ns': 4_999_999_999}, {'max_step_residual_ns': 50_000_001},
    {'max_anchor_drift_ns': 50_000_001}, {'rate_window_ns': 2 * NS},
])
def test_policy_cannot_weaken_or_silently_convert_frozen_contract(kwargs):
    with pytest.raises(ValueError):
        ClockGuardPolicy(**kwargs)


def test_no_source_never_ready_or_forwarded():
    guard = ExtnavClockGuard()
    result = guard.tick(MONO + 10 * NS)
    assert not result.accepted and not result.forward_allowed and not result.clock_ready
    assert 'no_source' in result.reasons
    assert guard.snapshot(MONO + 10 * NS)['frame_counts'] == {
        'observed': 0, 'accepted': 0, 'rejected': 0, 'forward_allowed': 0}


def test_valid_warmup_is_forwardable_but_not_ready_before_exactly_five_seconds():
    guard = ExtnavClockGuard()
    first = guard.observe(SOURCE, WALL, MONO)
    assert first.accepted and first.forward_allowed and not first.clock_ready
    for index in range(1, 50):
        step = guard.observe(SOURCE + index * 100_000_000,
                             WALL + index * 100_000_000, MONO + index * 100_000_000)
        assert step.accepted and step.forward_allowed and not step.clock_ready
    assert not guard.tick(MONO + 5 * NS - 1).clock_ready
    final = guard.observe(SOURCE + 5 * NS, WALL + 5 * NS, MONO + 5 * NS)
    assert final == ClockGuardStep(True, True, True, ())
    snapshot = guard.snapshot(MONO + 5 * NS)
    assert snapshot['continuous_valid_duration_ns'] == 5 * NS
    assert snapshot['rate_hz'] == 10
    assert snapshot['frame_counts']['accepted'] == 51


def test_exact_four_hz_stream_remains_ready_between_ordinary_frames():
    guard = ExtnavClockGuard()
    assert feed(guard, interval=250_000_000).clock_ready
    assert guard.tick(MONO + 5 * NS + 249_999_999).clock_ready
    assert guard.snapshot(MONO + 5 * NS + 249_999_999)['rate_hz'] == 4
    assert guard.observe(SOURCE + 5_250_000_000,
                         WALL + 5_250_000_000, MONO + 5_250_000_000).clock_ready


def test_continuously_advancing_but_low_frequency_source_is_not_ready():
    guard = ExtnavClockGuard()
    step = feed(guard, interval=300_000_000, duration=6 * NS)
    assert step.accepted and step.forward_allowed and not step.clock_ready
    assert 'rate_below_minimum' in step.reasons
    snapshot = guard.snapshot(MONO + 6 * NS)
    assert not snapshot['fault_latched']
    assert snapshot['timeout_window_resets'] == 0
    assert snapshot['continuous_valid_duration_ns'] == 6 * NS


def test_source_age_350ms_is_inclusive_and_350ms_plus_one_resets_once():
    guard = ExtnavClockGuard()
    assert feed(guard).clock_ready
    assert guard.tick(MONO + 5 * NS + 350_000_000).clock_ready
    stale = guard.tick(MONO + 5 * NS + 350_000_001)
    assert not stale.clock_ready and 'source_stale' in stale.reasons
    snapshot = guard.snapshot(MONO + 6 * NS)
    assert not snapshot['fault_latched']
    assert snapshot['continuous_valid_start_monotonic_ns'] is None
    assert snapshot['timeout_window_resets'] == 1
    assert snapshot['source_age_monotonic_ns'] == NS


def test_observe_detects_timeout_without_tick_and_demands_fresh_warmup():
    guard = ExtnavClockGuard()
    assert feed(guard).clock_ready
    new_mono = MONO + 5 * NS + 350_000_001
    new_source, new_wall = SOURCE + 5 * NS + 350_000_001, WALL + 5 * NS + 350_000_001
    step = guard.observe(new_source, new_wall, new_mono)
    assert step.accepted and step.forward_allowed and not step.clock_ready
    snapshot = guard.snapshot(new_mono)
    assert snapshot['timeout_window_resets'] == 1
    assert snapshot['continuous_valid_start_monotonic_ns'] == new_mono
    assert snapshot['source_age_monotonic_ns'] == 0
    assert feed(guard, source=new_source + 100_000_000, wall=new_wall + 100_000_000,
                mono=new_mono + 100_000_000).clock_ready


def test_exact_350ms_observation_gap_does_not_reset_continuity():
    guard = ExtnavClockGuard()
    feed(guard)
    guard.observe(SOURCE + 5_350_000_000, WALL + 5_350_000_000, MONO + 5_350_000_000)
    snapshot = guard.snapshot(MONO + 5_350_000_000)
    assert snapshot['timeout_window_resets'] == 0
    assert snapshot['continuous_valid_start_monotonic_ns'] == MONO


@pytest.mark.parametrize('delta', [0, -1, -NS])
def test_duplicate_backward_and_reset_acquisition_latch_without_refresh(delta):
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    result = guard.observe(SOURCE + delta, WALL + 100_000_000, MONO + 100_000_000)
    assert not result.accepted and not result.forward_allowed and not result.clock_ready
    assert 'source_stamp_not_increasing' in result.reasons
    snapshot = guard.snapshot(MONO + NS)
    assert snapshot['last_accepted_source_stamp_ns'] == SOURCE
    assert snapshot['last_accepted_transport_stamp_ns'] == WALL
    assert snapshot['source_age_monotonic_ns'] == NS
    assert snapshot['frame_counts'] == {
        'observed': 2, 'accepted': 1, 'rejected': 1, 'forward_allowed': 1}


def test_frozen_source_cannot_refresh_age_rate_or_warmup():
    guard = ExtnavClockGuard()
    feed(guard)
    for index in range(1, 11):
        result = guard.observe(SOURCE + 5 * NS, WALL + 5 * NS + index * 100_000_000,
                               MONO + 5 * NS + index * 100_000_000)
        assert not result.forward_allowed
    snapshot = guard.snapshot(MONO + 6 * NS)
    assert snapshot['source_age_monotonic_ns'] == NS
    assert snapshot['last_accepted_source_stamp_ns'] == SOURCE + 5 * NS
    assert snapshot['frame_counts']['accepted'] == 51
    assert snapshot['frame_counts']['rejected'] == 10
    assert snapshot['rate_hz'] == 0


@pytest.mark.parametrize('delta', [0, -1, -300_000_000])
def test_transport_duplicate_or_backward_latches_and_preserves_values(delta):
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    result = guard.observe(SOURCE + 100_000_000, WALL + delta, MONO + 100_000_000)
    assert not result.accepted and 'transport_stamp_not_increasing' in result.reasons
    assert guard.snapshot(MONO + 100_000_000)['last_accepted_transport_stamp_ns'] == WALL


@pytest.mark.parametrize('delta', [0, -1, -100_000_000])
def test_observation_monotonic_must_strictly_increase(delta):
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    result = guard.observe(SOURCE + 100_000_000, WALL + 100_000_000, MONO + delta)
    assert not result.accepted
    assert 'observation_monotonic_not_increasing' in result.reasons


@pytest.mark.parametrize('offset', [-50_000_000, 50_000_000])
def test_exact_50ms_step_and_anchor_boundary_are_accepted_without_clamping(offset):
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    result = guard.observe(SOURCE + 100_000_000, WALL + 100_000_000 + offset,
                           MONO + 100_000_000)
    assert result.accepted and result.forward_allowed
    snapshot = guard.snapshot(MONO + 100_000_000)
    assert snapshot['last_accepted_transport_stamp_ns'] == WALL + 100_000_000 + offset
    assert snapshot['last_accepted_step_residual_ns'] == offset
    assert snapshot['last_accepted_anchor_drift_ns'] == offset
    assert not snapshot['timestamps_modified']


@pytest.mark.parametrize('offset', [-50_000_001, 50_000_001, 2 * NS])
def test_step_exceeding_50ms_in_either_direction_latches(offset):
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    result = guard.observe(SOURCE + 100_000_000, WALL + 100_000_000 + offset,
                           MONO + 100_000_000)
    assert not result.accepted
    assert 'wall_monotonic_step_exceeds_tolerance' in result.reasons
    assert 'wall_monotonic_anchor_drift_exceeds_tolerance' in result.reasons


@pytest.mark.parametrize('direction', [-1, 1])
def test_small_cumulative_drift_cannot_evade_anchor(direction):
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    for index in (1, 2):
        result = guard.observe(SOURCE + index * 100_000_000,
                               WALL + index * (100_000_000 + direction * 20_000_000),
                               MONO + index * 100_000_000)
        assert result.accepted
    result = guard.observe(SOURCE + 300_000_000, WALL + 300_000_000 + direction * 60_000_000,
                           MONO + 300_000_000)
    assert not result.accepted
    assert 'wall_monotonic_anchor_drift_exceeds_tolerance' in result.reasons
    assert 'wall_monotonic_step_exceeds_tolerance' not in result.reasons


def test_anchor_drift_one_ns_past_boundary_is_detected_with_large_epoch():
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    assert guard.observe(SOURCE + 100_000_000, WALL + 150_000_000,
                         MONO + 100_000_000).accepted
    result = guard.observe(SOURCE + 200_000_000, WALL + 250_000_001, MONO + 200_000_000)
    assert not result.accepted
    assert 'wall_monotonic_step_exceeds_tolerance' not in result.reasons
    assert 'wall_monotonic_anchor_drift_exceeds_tolerance' in result.reasons


def test_timeout_does_not_reanchor_and_hide_wall_jump():
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    guard.tick(MONO + NS)
    result = guard.observe(SOURCE + NS, WALL + NS + 50_000_001, MONO + NS)
    assert not result.accepted
    snapshot = guard.snapshot(MONO + NS)
    assert snapshot['anchor_transport_stamp_ns'] == WALL
    assert snapshot['anchor_monotonic_ns'] == MONO
    assert snapshot['fault_latched']


def test_all_three_clocks_may_have_unrelated_epochs_and_source_rate():
    guard = ExtnavClockGuard()
    for index in range(51):
        result = guard.observe(1 + index, 10**30 + index * 100_000_000,
                               3 * NS + index * 100_000_000)
    assert result.clock_ready
    snapshot = guard.snapshot(8 * NS)
    assert snapshot['last_accepted_source_stamp_ns'] == 51
    assert snapshot['last_accepted_transport_stamp_ns'] == 10**30 + 5 * NS
    assert snapshot['source_age_monotonic_ns'] == 0
    assert snapshot['last_accepted_anchor_drift_ns'] == 0


def test_fault_is_permanent_and_only_new_instance_can_recover():
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    guard.observe(SOURCE, WALL + 100_000_000, MONO + 100_000_000)
    first = guard.snapshot(MONO + 100_000_000)['first_fault']
    result = feed(guard, source=SOURCE + NS, wall=WALL + NS, mono=MONO + NS)
    assert not result.accepted and not result.forward_allowed and not result.clock_ready
    snapshot = guard.snapshot(MONO + 6 * NS)
    assert snapshot['first_fault'] == first
    assert snapshot['last_accepted_source_stamp_ns'] == SOURCE
    assert feed(ExtnavClockGuard()).clock_ready


@pytest.mark.parametrize('bad', [True, False, 0, -1, 1.0, math.nan, math.inf, '1', None])
@pytest.mark.parametrize('index', [0, 1, 2])
def test_observe_strictly_rejects_nonpositive_and_noninteger_inputs(index, bad):
    guard = ExtnavClockGuard()
    values = [SOURCE, WALL, MONO]
    values[index] = bad
    result = guard.observe(*values)
    assert not result.accepted and not result.forward_allowed and not result.clock_ready
    snapshot = guard.snapshot(MONO)
    assert snapshot['fault_latched']
    assert snapshot['last_accepted_source_stamp_ns'] is None
    assert snapshot['last_accepted_transport_stamp_ns'] is None
    # allow_nan=False verifies real JSON safety, including rejected NaN/Inf.
    json.dumps(snapshot, allow_nan=False)


@pytest.mark.parametrize('bad', [True, 0, -1, 1.0, math.nan, math.inf, '1', None])
def test_tick_invalid_values_latch_and_produce_json_safe_snapshot(bad):
    guard = ExtnavClockGuard()
    result = guard.tick(bad)
    assert not result.clock_ready and 'evaluation_monotonic_ns_invalid' in result.reasons
    json.dumps(guard.snapshot(MONO), allow_nan=False)


def test_tick_equal_instant_is_idempotent_but_backward_tick_latches():
    guard = ExtnavClockGuard()
    assert feed(guard).clock_ready
    before = guard.snapshot(MONO + 5 * NS)
    assert guard.tick(MONO + 5 * NS).clock_ready
    assert guard.snapshot(MONO + 5 * NS) == before
    result = guard.tick(MONO + 5 * NS - 1)
    assert not result.clock_ready and 'evaluation_monotonic_regressed' in result.reasons


def test_observation_cannot_precede_last_tick():
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    guard.tick(MONO + 200_000_000)
    result = guard.observe(SOURCE + 100_000_000, WALL + 100_000_000, MONO + 100_000_000)
    assert not result.accepted and 'observation_before_last_evaluation' in result.reasons


def test_snapshot_cannot_mutate_fault_record_or_original_transport_stamp():
    guard = ExtnavClockGuard()
    guard.observe(SOURCE, WALL, MONO)
    guard.observe(SOURCE, WALL + 100_000_000, MONO + 100_000_000)
    snapshot = guard.snapshot(MONO + 200_000_000)
    snapshot['first_fault']['reasons'].append('not_real')
    snapshot['last_accepted_transport_stamp_ns'] = -1
    final = guard.snapshot(MONO + 200_000_000)
    assert 'not_real' not in final['first_fault']['reasons']
    assert final['last_accepted_transport_stamp_ns'] == WALL
    assert final['clock_ready_is_not_FCU_health']
    assert final['timestamps_modified'] is False
