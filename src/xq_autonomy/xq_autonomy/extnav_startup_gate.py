"""Discover one explicitly identified MAVROS odometry input before first output.

This gate consumes graph observations only. Matching an endpoint does not prove
its callback ran, that MAVROS serialized a frame, or that an FCU received it.
Call before source publication; skipped source frames must never be queued,
restamped, or replayed. TF/telemetry evidence remains separate from this gate.
Callers serialize observations and supply a host monotonic integer-ns clock.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional, Tuple


EXPECTED_IDENTITY = {
    'node_name': 'odometry', 'node_namespace': '/uav1/mavros',
    'topic_type': 'nav_msgs/msg/Odometry',
    'reliability': 'RELIABLE', 'durability': 'VOLATILE',
}


@dataclass(frozen=True)
class StartupGatePolicy:
    continuous_visibility_ns: int = 1_000_000_000
    maximum_observation_gap_ns: int = 250_000_000

    def __post_init__(self) -> None:
        if (type(self.continuous_visibility_ns) is not int
                or self.continuous_visibility_ns < 1_000_000_000):
            raise ValueError('continuous_visibility_must_be_integer_ns_at_least_1s')
        if (type(self.maximum_observation_gap_ns) is not int
                or not 0 < self.maximum_observation_gap_ns <= 350_000_000):
            raise ValueError('maximum_observation_gap_must_be_positive_integer_ns_at_most_350ms')


@dataclass(frozen=True)
class StartupGateStep:
    ready: bool
    reasons: Tuple[str, ...]


def _qos_name(value: Any, expected: str, enum_name: str) -> bool:
    return type(value) is str and value in (expected, enum_name+'.'+expected)


class ExtnavStartupGate:
    """A disappearing/ambiguous endpoint resets warmup; clock faults latch.

    ``snapshot()`` does not read clocks or promote readiness. Its state refers
    only to the latest ``observe`` call. Observe immediately before considering
    a new source publication; do not reuse this discovery result as FCU proof.
    A changed GID, when provided, starts a new visibility window even if the
    human-readable endpoint identity stays the same.
    """

    def __init__(self, policy: Optional[StartupGatePolicy] = None) -> None:
        self.policy = StartupGatePolicy() if policy is None else policy
        if not isinstance(self.policy, StartupGatePolicy):
            raise TypeError('policy_must_be_StartupGatePolicy')
        self._last_mono: Optional[int] = None
        self._visible_since: Optional[int] = None
        self._matched: Optional[Dict[str, Any]] = None
        self._first_fault: Optional[Dict[str, Any]] = None
        self._ready = False
        self._reasons: Tuple[str, ...] = ('no_graph_observation',)
        self._observations = self._matched_observations = self._resets = 0
        self._endpoint_count = self._owner_count = 0
        self._last_observation_gap_ns: Optional[int] = None
        self._last_observation_gap_exceeded = False
        self._observation_gap_resets = 0

    @property
    def ready(self) -> bool:
        return self._ready

    def _reset_visibility(self) -> None:
        if self._visible_since is not None:
            self._resets += 1
        self._visible_since = None
        self._matched = None
        self._ready = False

    def _not_ready(self, reason: str) -> StartupGateStep:
        self._reset_visibility()
        self._reasons = (reason,)
        return StartupGateStep(False, self._reasons)

    def _clock_fault(self, reason: str, value: Any) -> StartupGateStep:
        if self._first_fault is None:
            self._first_fault = {
                'reason': reason, 'observation_index': self._observations,
                'input_monotonic_ns': value if type(value) is int else repr(value),
                'last_observed_monotonic_ns': self._last_mono,
            }
        self._reset_visibility()
        self._reasons = ('clock_fault_latched', self._first_fault['reason'])
        return StartupGateStep(False, self._reasons)

    def observe(self, endpoints: list, monotonic_ns: int) -> StartupGateStep:
        self._observations += 1
        if self._first_fault is not None:
            return self._clock_fault(self._first_fault['reason'], monotonic_ns)
        if type(monotonic_ns) is not int or monotonic_ns <= 0:
            return self._clock_fault('monotonic_ns_invalid', monotonic_ns)
        if self._last_mono is not None and monotonic_ns < self._last_mono:
            return self._clock_fault('monotonic_regressed', monotonic_ns)
        self._last_observation_gap_ns = (None if self._last_mono is None else
                                         monotonic_ns - self._last_mono)
        self._last_observation_gap_exceeded = (self._last_observation_gap_ns is not None and
            self._last_observation_gap_ns > self.policy.maximum_observation_gap_ns)
        if self._last_observation_gap_exceeded:
            # Equal endpoint/GID at two distant queries cannot establish a
            # bounded-gap sampled visibility window. Start again at THIS query.
            self._reset_visibility()
            self._observation_gap_resets += 1
        self._last_mono = monotonic_ns
        if type(endpoints) is not list or any(type(item) is not dict for item in endpoints):
            self._endpoint_count = self._owner_count = 0
            return self._not_ready('endpoint_list_invalid')
        self._endpoint_count = len(endpoints)
        # Count owners BEFORE checking type/QoS: a malformed duplicate cannot
        # hide an ambiguous identity behind a second otherwise-valid endpoint.
        owners = [item for item in endpoints
                  if item.get('node_name') == EXPECTED_IDENTITY['node_name']
                  and item.get('node_namespace') == EXPECTED_IDENTITY['node_namespace']]
        self._owner_count = len(owners)
        if not owners:
            return self._not_ready('mavros_odometry_endpoint_missing')
        if len(owners) != 1:
            return self._not_ready('mavros_odometry_identity_ambiguous')
        endpoint = owners[0]
        if endpoint.get('topic_type') != EXPECTED_IDENTITY['topic_type']:
            return self._not_ready('mavros_odometry_topic_type_mismatch')
        qos = endpoint.get('qos')
        if type(qos) is not dict:
            return self._not_ready('mavros_odometry_qos_missing')
        if not _qos_name(qos.get('reliability'), 'RELIABLE', 'ReliabilityPolicy'):
            return self._not_ready('mavros_odometry_reliability_mismatch')
        if not _qos_name(qos.get('durability'), 'VOLATILE', 'DurabilityPolicy'):
            return self._not_ready('mavros_odometry_durability_mismatch')
        gid = endpoint.get('endpoint_gid')
        if gid is not None and (type(gid) is not list or not gid
                                or any(type(byte) is not int or not 0 <= byte <= 255 for byte in gid)):
            return self._not_ready('mavros_odometry_gid_invalid')
        matched = {
            'node_name': EXPECTED_IDENTITY['node_name'],
            'node_namespace': EXPECTED_IDENTITY['node_namespace'],
            'topic_type': EXPECTED_IDENTITY['topic_type'],
            'qos': {'reliability': 'RELIABLE', 'durability': 'VOLATILE'},
            'endpoint_gid': deepcopy(gid),
        }
        if self._matched is not None and matched != self._matched:
            self._reset_visibility()
        if self._visible_since is None:
            self._visible_since = monotonic_ns
        self._matched = matched
        self._matched_observations += 1
        self._ready = monotonic_ns-self._visible_since >= self.policy.continuous_visibility_ns
        self._reasons = (() if self._ready else
            ('observation_gap_exceeded', 'endpoint_visibility_warming_up')
            if self._last_observation_gap_exceeded else ('endpoint_visibility_warming_up',))
        return StartupGateStep(self._ready, self._reasons)

    def snapshot(self) -> Dict[str, Any]:
        duration = (self._last_mono-self._visible_since
                    if self._last_mono is not None and self._visible_since is not None else 0)
        return {
            'schema_version': 1, 'policy': asdict(self.policy),
            'ready': self._ready, 'reasons': list(self._reasons),
            'fault_latched': self._first_fault is not None,
            'first_fault': deepcopy(self._first_fault),
            'required_identity': dict(EXPECTED_IDENTITY),
            'matched_endpoint': deepcopy(self._matched),
            'last_observed_monotonic_ns': self._last_mono,
            'last_observation_gap_ns': self._last_observation_gap_ns,
            'last_observation_gap_exceeded': self._last_observation_gap_exceeded,
            'observation_gap_resets': self._observation_gap_resets,
            'continuous_visible_since_monotonic_ns': self._visible_since,
            'continuous_visibility_duration_ns': duration,
            'observations': self._observations, 'matched_observations': self._matched_observations,
            'latest_endpoint_count': self._endpoint_count,
            'latest_exact_owner_count': self._owner_count,
            'continuous_visibility_resets': self._resets,
            'clock_domain': 'host_monotonic',
            'identity_is_discovery_only': True, 'callback_or_FCU_receipt_proven': False,
            'TF_internal_readiness_proven': False,
            'phase': 'pre_source_publication',
            'source_frames_queued_or_replayed': False, 'source_or_transport_stamps_modified': False,
            'automatic_clock_fault_recovery': False,
            'snapshot_is_latest_observation_not_a_fresh_graph_query': True,
            'visibility_scope': 'Bounded-gap sampled endpoint visibility; existence between graph observations is not proven.',
        }
