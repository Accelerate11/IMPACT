"""Clock-candidate contracts without ROS, FCU, simulation, or system-clock changes."""
from dataclasses import FrozenInstanceError
import json

import pytest

from xq_autonomy.extnav_transport_clock import (
    ExtnavTransportClock, MAX_ROS_TIME_NS, TransportClockSample,
)


EPOCH = 1_790_000_000_000_000_000
MONO = 10_000_000_000


def sample(clock, elapsed=0, wall_delta=0, span=0):
    return clock.sample(EPOCH + elapsed + wall_delta, MONO + elapsed, MONO + elapsed + span)


def test_anchor_and_mapping_use_bracket_midpoint():
    clock = ExtnavTransportClock()
    first = clock.sample(EPOCH, MONO, MONO + 4_000_000)
    second = clock.sample(EPOCH + 100_000_000, MONO + 100_000_000, MONO + 102_000_000)
    assert first == TransportClockSample(True, EPOCH, MONO + 2_000_000, ())
    assert second.mapped_ns == EPOCH + 99_000_000
    state = clock.snapshot()
    assert state['anchor_wall_ns'] == EPOCH
    assert state['anchor_monotonic_ns'] == MONO + 2_000_000
    assert state['anchor_initializations'] == 1


@pytest.mark.parametrize('wall_delta', [-500_000_000, 500_000_000, -1_000_000_000, 1_000_000_000])
def test_raw_wall_steps_recorded_without_moving_anchor_or_clamping(wall_delta):
    clock = ExtnavTransportClock()
    sample(clock)
    result = sample(clock, 100_000_000, wall_delta)
    assert result.accepted and result.mapped_ns == EPOCH + 100_000_000
    snapshot = clock.snapshot()
    assert snapshot['ever_raw_wall_anomaly']
    assert not snapshot['fault_latched']
    assert snapshot['last_sampling']['raw_wall_ns'] == EPOCH + 100_000_000 + wall_delta
    assert snapshot['last_sampling']['raw_wall_step_residual_ns'] == wall_delta
    assert snapshot['last_sampling']['raw_wall_anchor_drift_ns'] == wall_delta
    assert snapshot['anchor_wall_ns'] == EPOCH


def test_cumulative_small_wall_drift_is_visible_at_declared_budget():
    clock = ExtnavTransportClock()
    sample(clock)
    assert sample(clock, 100_000_000, 30_000_000).accepted
    assert not clock.snapshot()['ever_raw_wall_anomaly']
    assert sample(clock, 200_000_000, 50_000_000).accepted
    assert not clock.snapshot()['ever_raw_wall_anomaly']
    assert sample(clock, 300_000_000, 50_000_001).accepted
    assert clock.snapshot()['ever_raw_wall_anomaly']
    assert not clock.snapshot()['fault_latched']


def test_uncertain_first_bracket_does_not_initialize_anchor_or_latch():
    clock = ExtnavTransportClock()
    result = sample(clock, span=5_000_001)
    assert not result.accepted and result.mapped_ns is None
    assert result.reasons == ('clock_pair_sampling_uncertain',)
    snapshot = clock.snapshot()
    assert snapshot['anchor_wall_ns'] is None
    assert snapshot['last_accepted_mapped_ns'] is None
    assert not snapshot['fault_latched']
    assert sample(clock, 100_000_000).accepted
    assert clock.snapshot()['anchor_wall_ns'] == EPOCH + 100_000_000


@pytest.mark.parametrize('span,accepted', [(0, True), (5_000_000, True), (5_000_001, False)])
def test_exact_sampling_span_boundary(span, accepted):
    assert sample(ExtnavTransportClock(), span=span).accepted is accepted


def test_uncertain_later_sample_does_not_update_accepted_state_or_anchor():
    clock = ExtnavTransportClock()
    first = sample(clock)
    result = sample(clock, 100_000_000, 500_000_000, span=6_000_000)
    assert not result.accepted
    state = clock.snapshot()
    assert state['last_accepted_mapped_ns'] == first.mapped_ns
    assert state['last_accepted_monotonic_ns'] == first.monotonic_ns
    assert state['anchor_wall_ns'] == EPOCH
    assert not state['fault_latched']
    assert sample(clock, 200_000_000).mapped_ns == EPOCH + 200_000_000


def test_sampling_uncertainty_is_not_called_a_raw_clock_jump():
    clock = ExtnavTransportClock()
    sample(clock)
    sample(clock, 100_000_000, 1_000_000_000, span=6_000_000)
    state = clock.snapshot()
    assert state['last_sampling']['raw_wall_anchor_drift_exceeds_budget']
    assert not state['last_sampling']['adjacent_raw_diagnostic_sampling_trusted']
    assert not state['ever_raw_wall_anomaly']
    assert not state['fault_latched']


@pytest.mark.parametrize('delta', [0, -1, -100_000_000])
def test_nonadvancing_monotonic_latches_and_cannot_recover(delta):
    clock = ExtnavTransportClock()
    sample(clock)
    rejected = sample(clock, delta)
    assert not rejected.accepted and rejected.mapped_ns is None
    assert 'monotonic_not_increasing' in rejected.reasons
    first_fault = clock.snapshot()['first_fault']
    assert not sample(clock, 200_000_000).accepted
    assert clock.snapshot()['first_fault'] == first_fault
    assert clock.snapshot()['last_accepted_mapped_ns'] == EPOCH


def test_uncertain_sample_still_preserves_observed_monotonic_order():
    clock = ExtnavTransportClock()
    sample(clock, 100_000_000, span=6_000_000)
    assert not sample(clock, 90_000_000).accepted
    assert clock.snapshot()['fault_latched']


def test_reversed_bracket_is_invalid_not_merely_uncertain():
    clock = ExtnavTransportClock()
    result = clock.sample(EPOCH, MONO + 1, MONO)
    assert not result.accepted
    assert 'monotonic_bracket_reversed' in result.reasons
    assert clock.snapshot()['fault_latched']


@pytest.mark.parametrize('field', [0, 1, 2])
@pytest.mark.parametrize('value', [0, -1, True, 1.0, float('nan'), float('inf'), None, '1'])
def test_invalid_timestamps_are_not_coerced_and_latch(field, value):
    clock = ExtnavTransportClock()
    values = [EPOCH, MONO, MONO]
    values[field] = value
    result = clock.sample(*values)
    assert not result.accepted and result.mapped_ns is None
    assert clock.snapshot()['fault_latched']
    assert clock.snapshot()['anchor_wall_ns'] is None
    # Invalid raw evidence is retained as explicit JSON-safe values/strings.
    json.dumps(clock.snapshot(), allow_nan=False)


def test_mapping_at_ros_upper_boundary_and_overflow():
    clock = ExtnavTransportClock()
    assert clock.sample(MAX_ROS_TIME_NS - 1000, MONO, MONO).accepted
    exact = clock.sample(1, MONO + 1000, MONO + 1000)
    assert exact.accepted and exact.mapped_ns == MAX_ROS_TIME_NS
    overflow = clock.sample(1, MONO + 1001, MONO + 1001)
    assert not overflow.accepted
    assert 'mapped_timestamp_out_of_ros_range' in overflow.reasons
    assert clock.snapshot()['last_accepted_mapped_ns'] == MAX_ROS_TIME_NS


@pytest.mark.parametrize('raw', [MAX_ROS_TIME_NS + 1, 2**63, 2**128])
def test_initial_epoch_outside_ros_range_cannot_initialize(raw):
    clock = ExtnavTransportClock()
    assert not clock.sample(raw, MONO, MONO).accepted
    assert clock.snapshot()['anchor_wall_ns'] is None
    assert clock.snapshot()['fault_latched']


def test_later_positive_raw_epoch_is_only_observed_not_used_for_mapping():
    clock = ExtnavTransportClock()
    sample(clock)
    result = clock.sample(2**128, MONO + 100_000_000, MONO + 100_000_000)
    assert result.accepted and result.mapped_ns == EPOCH + 100_000_000
    assert clock.snapshot()['ever_raw_wall_anomaly']


def test_uncertain_overflow_epoch_never_becomes_an_anchor():
    clock = ExtnavTransportClock()
    result = clock.sample(MAX_ROS_TIME_NS + 1, MONO, MONO + 6_000_000)
    assert not result.accepted and not clock.snapshot()['fault_latched']
    assert sample(clock, 100_000_000).accepted


def test_snapshot_and_return_object_cannot_mutate_internal_anchor_or_fault():
    clock = ExtnavTransportClock()
    result = sample(clock)
    with pytest.raises(FrozenInstanceError):
        result.mapped_ns = 0
    sample(clock)
    state = clock.snapshot()
    state['first_fault']['reasons'].clear()
    state['last_sampling']['raw_wall_ns'] = 0
    assert clock.snapshot()['first_fault']['reasons']
    assert clock.snapshot()['last_sampling']['raw_wall_ns'] == EPOCH


def test_raw_wall_anomaly_remains_recorded_after_wall_returns_to_anchor_rate():
    clock = ExtnavTransportClock()
    sample(clock)
    sample(clock, 100_000_000, 500_000_000)
    sample(clock, 200_000_000)
    sample(clock, 300_000_000)
    assert not clock.snapshot()['last_sampling']['raw_wall_anomaly']
    assert clock.snapshot()['ever_raw_wall_anomaly']
    assert clock.snapshot()['accepted_count'] == 4
    assert clock.snapshot()['rejected_count'] == 0


def test_nanosecond_increase_but_wire_microsecond_collision_latches():
    clock = ExtnavTransportClock()
    first = sample(clock)
    next_ns = clock.sample(EPOCH + 1, MONO + 1, MONO + 1)
    assert not next_ns.accepted and next_ns.mapped_ns is None
    assert 'mapped_microsecond_not_increasing_or_nonpositive' in next_ns.reasons
    assert clock.snapshot()['fault_latched']
    assert clock.snapshot()['last_accepted_mapped_ns'] == first.mapped_ns
    assert not sample(clock, 100_000_000).accepted


@pytest.mark.parametrize('elapsed,accepted', [(999, False), (1000, True), (1001, True)])
def test_exact_microsecond_progress_boundary(elapsed, accepted):
    clock = ExtnavTransportClock()
    sample(clock)
    assert sample(clock, elapsed).accepted is accepted


def test_quantization_uses_floor_not_a_minimum_1000ns_interval():
    clock = ExtnavTransportClock()
    assert clock.sample(EPOCH + 999, MONO, MONO).accepted
    # 1 ns can cross a microsecond boundary; no artificial +1 is applied.
    result = clock.sample(EPOCH + 1000, MONO + 1, MONO + 1)
    assert result.accepted and result.mapped_ns == EPOCH + 1000
    assert clock.snapshot()['last_accepted_mapped_usec'] == (EPOCH + 1000) // 1000


def test_positive_ns_that_quantizes_to_zero_is_rejected():
    clock = ExtnavTransportClock()
    assert not clock.sample(999, MONO, MONO).accepted
    assert clock.snapshot()['fault_latched']
    assert clock.snapshot()['anchor_wall_ns'] is None


def test_initial_empty_snapshot_and_scope_are_explicit():
    clock = ExtnavTransportClock()
    state = clock.snapshot()
    assert state['last_sampling'] is None and state['first_fault'] is None
    for field in ('utc_calibrated', 'acquisition_time_calibrated', 'fcu_time_calibrated',
                  'source_freshness_validated', 'timestamp_clamping_used', 'automatic_fault_recovery'):
        assert state[field] is False
    assert state['source_acquisition_stamp_not_an_input'] is True
    assert not hasattr(clock, 'reset')


@pytest.mark.parametrize('value', [0, -1, True, 5_000_001, 1.0, None])
def test_invalid_or_loosened_sampling_budget_refused(value):
    with pytest.raises(ValueError):
        ExtnavTransportClock(value)
