"""Fail-closed clock contract for external-navigation transport timestamps.

All inputs are integer nanoseconds. Source timestamps belong to the acquisition
clock; transport timestamps belong to host wall time; observations and freshness
belong to host monotonic time. No arithmetic mixes source time with either host
clock. This module does not produce or modify any timestamp, pose, or covariance.

Call ``observe`` exactly once for each source frame that passed the caller's
non-clock validation. Forward only when ``forward_allowed`` is true. Warmup
frames may be forwarded; ``clock_ready`` is a separate, stricter readiness signal
and never proves FCU receipt or FCU health. Callers must serialize method calls.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import asdict, dataclass
import math
from typing import Any, Dict, Optional, Tuple


@dataclass(frozen=True)
class ClockGuardPolicy:
    warmup_ns: int = 5_000_000_000
    max_source_age_ns: int = 350_000_000
    min_rate_hz: float = 4.0
    max_step_residual_ns: int = 50_000_000
    max_anchor_drift_ns: int = 50_000_000
    rate_window_ns: int = 1_000_000_000

    def __post_init__(self) -> None:
        for field in (
            'warmup_ns', 'max_source_age_ns', 'max_step_residual_ns',
            'max_anchor_drift_ns', 'rate_window_ns',
        ):
            if not _positive_ns(getattr(self, field)):
                raise ValueError(field + '_must_be_positive_integer_nanoseconds')
        if (isinstance(self.min_rate_hz, bool)
                or not isinstance(self.min_rate_hz, (int, float))
                or not math.isfinite(self.min_rate_hz)
                or self.min_rate_hz < 4.0):
            raise ValueError('min_rate_hz_must_be_finite_and_at_least_4')
        if self.max_source_age_ns > 350_000_000:
            raise ValueError('source_age_must_not_exceed_350ms')
        if self.warmup_ns < 5_000_000_000:
            raise ValueError('warmup_must_be_at_least_5s')
        if self.max_step_residual_ns > 50_000_000 or self.max_anchor_drift_ns > 50_000_000:
            raise ValueError('clock_tolerance_must_not_exceed_50ms')
        if self.rate_window_ns != 1_000_000_000:
            raise ValueError('rate_window_is_fixed_at_1s')


@dataclass(frozen=True)
class ClockGuardStep:
    accepted: bool
    forward_allowed: bool
    clock_ready: bool
    reasons: Tuple[str, ...]


def _positive_ns(value: Any) -> bool:
    # bool and numpy integer scalars are deliberately not implicit timestamps.
    return type(value) is int and value > 0


def _audit_value(value: Any) -> Any:
    """Keep invalid inputs auditable without NaN/Inf or arbitrary JSON objects."""
    return value if type(value) in (int, str, bool) or value is None else repr(value)


class ExtnavClockGuard:
    """A clock violation latches until a NEW instance is constructed.

    An observation timeout alone is recoverable, but discards the continuous
    warmup/rate window. It never resets the original wall/monotonic anchor.
    Source/transport/observation stamps on rejected frames never replace the
    last accepted values. ``tick`` permits the same evaluation instant again;
    observation monotonic stamps must strictly increase between accepted frames.
    """

    def __init__(self, policy: Optional[ClockGuardPolicy] = None) -> None:
        self.policy = policy if policy is not None else ClockGuardPolicy()
        if not isinstance(self.policy, ClockGuardPolicy):
            raise TypeError('policy_must_be_ClockGuardPolicy')
        self._last_source_ns: Optional[int] = None
        self._last_transport_ns: Optional[int] = None
        self._last_observation_ns: Optional[int] = None
        self._last_evaluation_ns: Optional[int] = None
        self._anchor_transport_ns: Optional[int] = None
        self._anchor_monotonic_ns: Optional[int] = None
        self._continuous_start_ns: Optional[int] = None
        self._samples: deque[int] = deque()
        self._first_fault: Optional[Dict[str, Any]] = None
        self._last_step_residual_ns: Optional[int] = None
        self._last_anchor_drift_ns: Optional[int] = None
        self._observed_count = 0
        self._accepted_count = 0
        self._rejected_count = 0
        self._forward_allowed_count = 0
        self._timeout_reset_count = 0

    def _latch(self, reasons: Tuple[str, ...], kind: str, **inputs: Any) -> None:
        if self._first_fault is None:
            self._first_fault = {
                'kind': kind, 'reasons': list(reasons),
                'observation_attempt': self._observed_count,
                'inputs': {key: _audit_value(value) for key, value in inputs.items()},
                'last_accepted_source_stamp_ns': self._last_source_ns,
                'last_accepted_transport_stamp_ns': self._last_transport_ns,
                'last_accepted_monotonic_ns': self._last_observation_ns,
            }

    def _expire(self, now_ns: int) -> None:
        if (self._last_observation_ns is not None
                and now_ns-self._last_observation_ns > self.policy.max_source_age_ns
                and self._continuous_start_ns is not None):
            self._continuous_start_ns = None
            self._samples.clear()
            self._timeout_reset_count += 1
        self._prune(now_ns)

    def _prune(self, now_ns: int) -> None:
        # Half-open trailing window (now - 1s, now], so an exact 4Hz stream
        # reports 4Hz throughout its normal 250ms interarrival interval.
        lower = now_ns-self.policy.rate_window_ns
        while self._samples and self._samples[0] <= lower:
            self._samples.popleft()

    def _rate_hz(self) -> float:
        if len(self._samples) < 2:
            return 0.0
        count_rate = len(self._samples)*1_000_000_000/self.policy.rate_window_ns
        interval_rate = ((len(self._samples)-1)*1_000_000_000
                         / (self._samples[-1]-self._samples[0]))
        # Counting four frames in one window alone aliases a regular 3.33Hz
        # stream to 4Hz immediately after a frame. Require interarrival rate too.
        return min(count_rate, interval_rate)

    def _reasons(self, now_ns: Optional[int]) -> Tuple[str, ...]:
        if self._first_fault is not None:
            return ('clock_fault_latched', *self._first_fault['reasons'])
        if self._last_observation_ns is None or now_ns is None:
            return ('no_source', 'warming_up', 'rate_below_minimum')
        reasons = []
        if now_ns-self._last_observation_ns > self.policy.max_source_age_ns:
            reasons.append('source_stale')
        if (self._continuous_start_ns is None
                or now_ns-self._continuous_start_ns < self.policy.warmup_ns):
            reasons.append('warming_up')
        if self._rate_hz() < self.policy.min_rate_hz:
            reasons.append('rate_below_minimum')
        return tuple(reasons)

    def _step(self, accepted: bool, now_ns: Optional[int]) -> ClockGuardStep:
        reasons = self._reasons(now_ns)
        forward = accepted and self._first_fault is None
        if forward:
            self._forward_allowed_count += 1
        return ClockGuardStep(accepted, forward, not reasons, reasons)

    def observe(self, source_stamp_ns: int, transport_stamp_ns: int,
                monotonic_ns: int) -> ClockGuardStep:
        self._observed_count += 1
        values = {'source_stamp_ns': source_stamp_ns,
                  'transport_stamp_ns': transport_stamp_ns, 'monotonic_ns': monotonic_ns}
        if self._first_fault is not None:
            self._rejected_count += 1
            return self._step(False, self._last_evaluation_ns)
        reasons = [name+'_invalid' for name, value in values.items() if not _positive_ns(value)]
        residual = drift = None
        if not reasons:
            if self._last_source_ns is not None and source_stamp_ns <= self._last_source_ns:
                reasons.append('source_stamp_not_increasing')
            if self._last_transport_ns is not None and transport_stamp_ns <= self._last_transport_ns:
                reasons.append('transport_stamp_not_increasing')
            if self._last_observation_ns is not None and monotonic_ns <= self._last_observation_ns:
                reasons.append('observation_monotonic_not_increasing')
            if self._last_evaluation_ns is not None and monotonic_ns < self._last_evaluation_ns:
                reasons.append('observation_before_last_evaluation')
            if self._last_observation_ns is not None:
                residual = ((transport_stamp_ns-self._last_transport_ns)
                            - (monotonic_ns-self._last_observation_ns))
                if abs(residual) > self.policy.max_step_residual_ns:
                    reasons.append('wall_monotonic_step_exceeds_tolerance')
                drift = ((transport_stamp_ns-self._anchor_transport_ns)
                         - (monotonic_ns-self._anchor_monotonic_ns))
                if abs(drift) > self.policy.max_anchor_drift_ns:
                    reasons.append('wall_monotonic_anchor_drift_exceeds_tolerance')
        if reasons:
            self._rejected_count += 1
            self._latch(tuple(reasons), 'observe', **values)
            return self._step(False, self._last_evaluation_ns)
        # Observe itself enforces the timeout even if no status timer has run.
        self._expire(monotonic_ns)
        if self._anchor_monotonic_ns is None:
            self._anchor_monotonic_ns = monotonic_ns
            self._anchor_transport_ns = transport_stamp_ns
        if self._continuous_start_ns is None:
            self._continuous_start_ns = monotonic_ns
        self._last_source_ns = source_stamp_ns
        self._last_transport_ns = transport_stamp_ns
        self._last_observation_ns = monotonic_ns
        self._last_evaluation_ns = monotonic_ns
        self._last_step_residual_ns = residual
        self._last_anchor_drift_ns = drift
        self._samples.append(monotonic_ns)
        self._prune(monotonic_ns)
        self._accepted_count += 1
        return self._step(True, monotonic_ns)

    def tick(self, now_monotonic_ns: int) -> ClockGuardStep:
        if not _positive_ns(now_monotonic_ns):
            self._latch(('evaluation_monotonic_ns_invalid',), 'tick', monotonic_ns=now_monotonic_ns)
            return self._step(False, self._last_evaluation_ns)
        if self._last_evaluation_ns is not None and now_monotonic_ns < self._last_evaluation_ns:
            self._latch(('evaluation_monotonic_regressed',), 'tick', monotonic_ns=now_monotonic_ns)
            return self._step(False, self._last_evaluation_ns)
        self._last_evaluation_ns = now_monotonic_ns
        self._expire(now_monotonic_ns)
        return self._step(False, now_monotonic_ns)

    def snapshot(self, now_monotonic_ns: int) -> Dict[str, Any]:
        step = self.tick(now_monotonic_ns)
        now_ns = self._last_evaluation_ns
        age = (now_ns-self._last_observation_ns
               if now_ns is not None and self._last_observation_ns is not None else None)
        elapsed = (now_ns-self._continuous_start_ns
                   if now_ns is not None and self._continuous_start_ns is not None else 0)
        return {
            'schema_version': 1, 'policy': asdict(self.policy),
            'clock_ready': step.clock_ready, 'fault_latched': self._first_fault is not None,
            'reasons': list(step.reasons), 'first_fault': deepcopy(self._first_fault),
            'last_accepted_source_stamp_ns': self._last_source_ns,
            'last_accepted_transport_stamp_ns': self._last_transport_ns,
            'last_accepted_monotonic_ns': self._last_observation_ns,
            'last_evaluated_monotonic_ns': now_ns,
            'source_age_monotonic_ns': age,
            'continuous_valid_start_monotonic_ns': self._continuous_start_ns,
            'continuous_valid_duration_ns': elapsed,
            'rate_window_count': len(self._samples),
            'rate_hz': self._rate_hz(),
            'rate_estimator': 'minimum_of_trailing_1s_count_rate_and_observed_interarrival_rate',
            'anchor_transport_stamp_ns': self._anchor_transport_ns,
            'anchor_monotonic_ns': self._anchor_monotonic_ns,
            'last_accepted_step_residual_ns': self._last_step_residual_ns,
            'last_accepted_anchor_drift_ns': self._last_anchor_drift_ns,
            'frame_counts': {
                'observed': self._observed_count, 'accepted': self._accepted_count,
                'rejected': self._rejected_count, 'forward_allowed': self._forward_allowed_count,
            },
            'timeout_window_resets': self._timeout_reset_count,
            'source_clock_domain': 'acquisition', 'transport_clock_domain': 'host_wall',
            'freshness_clock_domain': 'host_monotonic',
            'forward_allowed_counts_are_not_actual_publication_counts': True,
            'clock_ready_is_not_FCU_health': True,
            'timestamps_modified': False, 'automatic_fault_recovery': False,
        }
