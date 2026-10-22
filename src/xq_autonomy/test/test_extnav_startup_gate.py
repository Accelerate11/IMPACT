"""Pure graph fixtures; no ROS discovery, AP/Gazebo, or source-frame replay."""
from copy import deepcopy
from dataclasses import FrozenInstanceError
import json
import math

import pytest

from xq_autonomy.extnav_startup_gate import (
    ExtnavStartupGate,
    StartupGatePolicy,
    StartupGateStep,
)


NS = 1_000_000_000
START = 100*NS


def endpoint(name='odometry', namespace='/uav1/mavros'):
    return {
        'node_name': name, 'node_namespace': namespace,
        'topic_type': 'nav_msgs/msg/Odometry', 'endpoint_gid': [1]*24,
        'qos': {'reliability': 'ReliabilityPolicy.RELIABLE',
                'durability': 'DurabilityPolicy.VOLATILE'},
    }


def warm(gate, endpoints=None, start=START):
    values = [endpoint()] if endpoints is None else endpoints
    for offset in range(0, NS + 1, 100_000_000):
        result = gate.observe(values, start+offset)
    return result


def test_policy_and_step_are_frozen():
    policy = StartupGatePolicy()
    assert policy.continuous_visibility_ns == NS
    assert policy.maximum_observation_gap_ns == 250_000_000
    with pytest.raises(FrozenInstanceError):
        policy.continuous_visibility_ns = 1
    with pytest.raises(FrozenInstanceError):
        StartupGateStep(False, ()).ready = True


@pytest.mark.parametrize('value', [True, False, 0, -1, 999_999_999, float(NS), None])
def test_visibility_policy_cannot_weaken_or_convert_integer_one_second(value):
    with pytest.raises(ValueError):
        StartupGatePolicy(continuous_visibility_ns=value)


def test_initial_snapshot_is_not_ready_and_cannot_itself_advance_visibility():
    gate = ExtnavStartupGate()
    assert gate.snapshot()['ready'] is False
    assert gate.snapshot()['observations'] == 0
    gate.observe([endpoint()], START)
    first = gate.snapshot()
    assert first['continuous_visibility_duration_ns'] == 0
    assert first['ready'] is False
    for _ in range(10):
        assert gate.snapshot() == first


def test_exact_one_second_boundary_and_no_sensor_or_transport_time_inputs():
    gate = ExtnavStartupGate()
    assert gate.observe([endpoint()], START) == StartupGateStep(False, ('endpoint_visibility_warming_up',))
    for offset in range(100_000_000, NS, 100_000_000):
        assert not gate.observe([endpoint()], START + offset).ready
    assert not gate.observe([endpoint()], START+NS-1).ready
    assert gate.observe([endpoint()], START+NS) == StartupGateStep(True, ())
    result = gate.snapshot()
    assert result['continuous_visibility_duration_ns'] == NS
    assert result['matched_endpoint']['node_name'] == 'odometry'
    assert result['identity_is_discovery_only']
    assert not result['callback_or_FCU_receipt_proven']
    assert not result['TF_internal_readiness_proven']
    assert not result['source_frames_queued_or_replayed']
    assert not result['source_or_transport_stamps_modified']


def test_observer_and_recorder_subscription_counts_cannot_make_gate_ready():
    endpoints = [endpoint('impact_extnav_disarmed_probe', '/'), endpoint('rosbag2_recorder', '/')]
    gate = ExtnavStartupGate()
    assert not warm(gate, endpoints).ready
    assert gate.snapshot()['latest_endpoint_count'] == 2
    assert gate.snapshot()['latest_exact_owner_count'] == 0


def test_exact_mavros_endpoint_can_coexist_with_observer_recorder_and_unknown():
    values = [endpoint(), endpoint('impact_extnav_disarmed_probe', '/'),
              endpoint('rosbag2_recorder', '/'), endpoint('_NODE_NAME_UNKNOWN_', '_NODE_NAMESPACE_UNKNOWN_')]
    gate = ExtnavStartupGate()
    assert warm(gate, values).ready
    assert gate.snapshot()['latest_endpoint_count'] == 4
    assert gate.snapshot()['latest_exact_owner_count'] == 1


@pytest.mark.parametrize('key,value', [
    ('node_name', 'impact_extnav_disarmed_probe'), ('node_name', 'rosbag2_recorder'),
    ('node_name', '_NODE_NAME_UNKNOWN_'), ('node_namespace', '_NODE_NAMESPACE_UNKNOWN_'),
    ('node_namespace', '/'), ('node_namespace', '/uav1/mavros/'),
    ('node_namespace', '/uav2/mavros'), ('topic_type', 'std_msgs/msg/String'),
])
def test_identity_must_match_exact_owner_namespace_and_type(key, value):
    candidate = endpoint()
    candidate[key] = value
    gate = ExtnavStartupGate()
    assert not warm(gate, [candidate]).ready
    assert not gate.snapshot()['fault_latched']


@pytest.mark.parametrize('field,bad', [
    ('reliability', 'BEST_EFFORT'), ('reliability', 'ReliabilityPolicy.BEST_EFFORT'),
    ('reliability', 'ReliabilityPolicy.UNKNOWN'), ('reliability', 1),
    ('durability', 'TRANSIENT_LOCAL'), ('durability', 'DurabilityPolicy.UNKNOWN'),
    ('durability', None),
])
def test_qos_is_explicit_reliable_and_volatile(field, bad):
    candidate = endpoint()
    candidate['qos'][field] = bad
    assert not warm(ExtnavStartupGate(), [candidate]).ready


def test_canonical_qos_names_are_accepted_without_enum_dependency():
    candidate = endpoint()
    candidate['qos'] = {'reliability': 'RELIABLE', 'durability': 'VOLATILE'}
    assert warm(ExtnavStartupGate(), [candidate]).ready


@pytest.mark.parametrize('duplicate', ['same_gid', 'different_gid', 'wrong_type', 'wrong_qos'])
def test_duplicate_owner_is_ambiguous_even_if_only_one_passes_other_fields(duplicate):
    first, second = endpoint(), endpoint()
    if duplicate == 'different_gid':
        second['endpoint_gid'] = [2]*24
    elif duplicate == 'wrong_type':
        second['topic_type'] = 'std_msgs/msg/String'
    elif duplicate == 'wrong_qos':
        second['qos']['reliability'] = 'BEST_EFFORT'
    gate = ExtnavStartupGate()
    result = warm(gate, [first, second])
    assert not result.ready
    assert result.reasons == ('mavros_odometry_identity_ambiguous',)
    assert gate.snapshot()['latest_exact_owner_count'] == 2


@pytest.mark.parametrize('loss', [[], [endpoint('impact_extnav_disarmed_probe', '/')], None, [None]])
def test_disappearance_or_invalid_graph_immediately_resets_continuity(loss):
    gate = ExtnavStartupGate()
    assert warm(gate).ready
    assert not gate.observe(loss, START+NS+1).ready
    status = gate.snapshot()
    assert status['continuous_visible_since_monotonic_ns'] is None
    assert status['continuous_visibility_resets'] == 1
    assert not status['fault_latched']
    assert not gate.observe([endpoint()], START+NS+2).ready
    for offset in range(100_000_000, NS, 100_000_000):
        assert not gate.observe([endpoint()], START + NS + 2 + offset).ready
    assert not gate.observe([endpoint()], START+2*NS+1).ready
    assert gate.observe([endpoint()], START+2*NS+2).ready


def test_endpoint_restart_gid_change_demands_new_visibility_window():
    gate = ExtnavStartupGate()
    assert warm(gate).ready
    restarted = endpoint()
    restarted['endpoint_gid'] = [2]*24
    assert not gate.observe([restarted], START+NS+100).ready
    assert gate.snapshot()['continuous_visibility_resets'] == 1
    for offset in range(100_000_000, NS, 100_000_000):
        assert not gate.observe([restarted], START + NS + 100 + offset).ready
    assert not gate.observe([restarted], START+2*NS+99).ready
    assert gate.observe([restarted], START+2*NS+100).ready


@pytest.mark.parametrize('gid', [[], [True], [-1], [256], 'gid'])
def test_present_but_invalid_gid_is_not_endpoint_proof(gid):
    candidate = endpoint()
    candidate['endpoint_gid'] = gid
    assert not warm(ExtnavStartupGate(), [candidate]).ready


def test_gid_is_optional_but_its_absence_is_explicit_in_snapshot():
    candidate = endpoint()
    del candidate['endpoint_gid']
    gate = ExtnavStartupGate()
    assert warm(gate, [candidate]).ready
    assert gate.snapshot()['matched_endpoint']['endpoint_gid'] is None


@pytest.mark.parametrize('bad', [True, False, 0, -1, 1.0, math.nan, math.inf, '1', None])
def test_monotonic_clock_requires_positive_integer_ns_and_latches(bad):
    gate = ExtnavStartupGate()
    result = gate.observe([endpoint()], bad)
    assert not result.ready
    assert result.reasons == ('clock_fault_latched', 'monotonic_ns_invalid')
    first = gate.snapshot()['first_fault']
    assert not warm(gate).ready
    assert gate.snapshot()['first_fault'] == first
    json.dumps(gate.snapshot(), allow_nan=False)


def test_equal_poll_instant_is_allowed_but_backward_clock_fault_cannot_recover():
    gate = ExtnavStartupGate()
    assert warm(gate).ready
    assert gate.observe([endpoint()], START+NS).ready
    result = gate.observe([endpoint()], START+NS-1)
    assert not result.ready and 'monotonic_regressed' in result.reasons
    first = gate.snapshot()['first_fault']
    assert not warm(gate, start=START+2*NS).ready
    assert gate.snapshot()['first_fault'] == first
    assert warm(ExtnavStartupGate()).ready


def test_snapshot_and_caller_endpoint_mutation_cannot_change_stored_identity():
    gate = ExtnavStartupGate()
    candidate = endpoint()
    gate.observe([candidate], START)
    candidate['endpoint_gid'][0] = 2
    snapshot = gate.snapshot()
    assert snapshot['matched_endpoint']['endpoint_gid'][0] == 1
    changed = deepcopy(snapshot)
    changed['matched_endpoint']['endpoint_gid'][0] = 3
    changed['required_identity']['node_name'] = 'observer'
    assert gate.snapshot() == snapshot


@pytest.mark.parametrize('value', [0, -1, True, 350_000_001, 1.0, math.nan, math.inf, None])
def test_observation_gap_policy_must_be_explicit_positive_integer_and_at_most_350ms(value):
    with pytest.raises(ValueError):
        StartupGatePolicy(maximum_observation_gap_ns=value)


@pytest.mark.parametrize('gap', [NS, 10*NS, 24*3600*NS])
def test_old_two_distant_queries_same_gid_cannot_prove_visibility(gap):
    gate = ExtnavStartupGate()
    assert not gate.observe([endpoint()], START).ready
    result = gate.observe([endpoint()], START + gap)
    assert not result.ready and 'observation_gap_exceeded' in result.reasons
    state = gate.snapshot()
    assert state['continuous_visibility_duration_ns'] == 0
    assert state['continuous_visible_since_monotonic_ns'] == START + gap
    assert state['observation_gap_resets'] == 1
    assert not state['fault_latched']


@pytest.mark.parametrize('budget', [250_000_000, 350_000_000])
@pytest.mark.parametrize('extra', [0, 1])
def test_exact_maximum_gap_boundary_and_rewarm_after_gap(budget, extra):
    gate = ExtnavStartupGate(StartupGatePolicy(maximum_observation_gap_ns=budget))
    assert warm(gate).ready
    current = START + NS + budget + extra
    result = gate.observe([endpoint()], current)
    assert result.ready is (extra == 0)
    if extra:
        assert gate.snapshot()['continuous_visibility_resets'] == 1
        assert not gate.snapshot()['fault_latched']
        assert warm(gate, start=current + 100_000_000).ready


def test_quarter_second_sampled_queries_can_satisfy_default_budget():
    gate = ExtnavStartupGate()
    for offset in (0, 250_000_000, 500_000_000, 750_000_000):
        assert not gate.observe([endpoint()], START+offset).ready
    assert gate.observe([endpoint()], START+NS).ready


def test_gap_reset_keeps_monotonic_regression_fault_permanent():
    gate = ExtnavStartupGate()
    warm(gate)
    assert not gate.observe([endpoint()], START + 3*NS).ready
    assert not gate.observe([endpoint()], START + 3*NS - 1).ready
    fault = gate.snapshot()['first_fault']
    assert not warm(gate, start=START + 4*NS).ready
    assert gate.snapshot()['first_fault'] == fault
