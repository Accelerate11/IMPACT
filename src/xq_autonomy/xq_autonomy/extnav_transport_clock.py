"""Explicit candidate transport clock; not acquisition/UTC/FCU calibration.

One trustworthy host wall/monotonic bracket fixes an epoch for this instance.
Later timestamps use only monotonic elapsed time. Raw wall observations and
their anomalies remain separate evidence; they never move the anchor or clamp
the candidate. This module neither publishes to AP nor accepts a source pose.

MAVROS 2.14.0 odom.cpp:316 copies the supplied header via get_time_usec;
plugin.hpp:255-257 floors ns to us, without applying UAS timesync offset.
ArduPilot b7207551 GCS_Common.cpp:3686 and 6486-6500 independently correct
remote time to boot time with JitterCorrection. That arrival-based correction
does not calibrate the source acquisition time. Accepted candidates also keep
their floored microsecond identifiers strictly increasing. Downstream code must
separately check source freshness and must explicitly opt into any
future deployment. No timestamp is automatically repaired by this candidate.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Optional, Tuple


NS = 1_000_000_000
# builtin_interfaces/Time uses signed int32 seconds; to_msg must also fit.
MAX_ROS_TIME_NS = (2**31) * NS - 1
MAXIMUM_SAMPLING_SPAN_NS = 5_000_000
RAW_WALL_ANOMALY_TOLERANCE_NS = 50_000_000


def _positive_int(value: Any) -> bool:
    return type(value) is int and value > 0


def _audit(value: Any) -> Any:
    return value if type(value) in (int, str, bool) or value is None else repr(value)


@dataclass(frozen=True)
class TransportClockSample:
    accepted: bool
    mapped_ns: Optional[int]
    monotonic_ns: Optional[int]
    reasons: Tuple[str, ...]


class ExtnavTransportClock:
    """No-reset candidate with a fixed first trusted bracket midpoint anchor.

    Sampling spans above 5 ms reject only that observation. Invalid inputs,
    reversed/nonadvancing monotonic observations, or ROS timestamp overflow
    latch a fault until a different instance is constructed. Call serially.
    Raw wall clock steps/drift are recorded but do not cause a mapping fault.
    """

    def __init__(self, maximum_sampling_span_ns: int = MAXIMUM_SAMPLING_SPAN_NS):
        if (not _positive_int(maximum_sampling_span_ns) or
                maximum_sampling_span_ns > MAXIMUM_SAMPLING_SPAN_NS):
            raise ValueError('maximum_sampling_span_ns_must_be_positive_and_at_most_5ms')
        self.maximum_sampling_span_ns = maximum_sampling_span_ns
        self._anchor_wall_ns: Optional[int] = None
        self._anchor_monotonic_ns: Optional[int] = None
        self._last_observed_monotonic_ns: Optional[int] = None
        self._last_accepted_monotonic_ns: Optional[int] = None
        self._last_accepted_mapped_ns: Optional[int] = None
        self._last_accepted_mapped_usec: Optional[int] = None
        self._last_raw_pair: Optional[dict] = None
        self._last_sampling: Optional[dict] = None
        self._first_fault: Optional[dict] = None
        self._ever_raw_wall_anomaly = False
        self._observations = self._accepted = self._uncertain = 0

    def _latch(self, reasons: Tuple[str, ...], sampling: dict):
        if self._first_fault is None:
            self._first_fault = {
                'sample_index': self._observations, 'reasons': list(reasons),
                'sampling': deepcopy(sampling),
                'last_accepted_monotonic_ns': self._last_accepted_monotonic_ns,
                'last_accepted_mapped_ns': self._last_accepted_mapped_ns,
                'last_accepted_mapped_usec': self._last_accepted_mapped_usec,
            }

    def _finish(self, sampling, accepted=False, mapped=None, reasons=()):
        sampling.update({'accepted': accepted, 'mapped_ns': mapped,
                         'mapped_usec': mapped // 1000 if mapped is not None else None,
                         'reasons': list(reasons)})
        self._last_sampling = deepcopy(sampling)
        if accepted:
            self._accepted += 1
        return TransportClockSample(accepted, mapped, sampling['monotonic_midpoint_ns'], tuple(reasons))

    def sample(self, raw_wall_ns: int, before_ns: int, after_ns: int) -> TransportClockSample:
        self._observations += 1
        sampling = {
            'raw_wall_ns': _audit(raw_wall_ns), 'monotonic_before_ns': _audit(before_ns),
            'monotonic_after_ns': _audit(after_ns), 'monotonic_midpoint_ns': None,
            'sampling_span_ns': None, 'sampling_uncertain': None,
            'raw_wall_step_ns': None, 'monotonic_step_ns': None,
            'raw_wall_step_residual_ns': None, 'raw_wall_anchor_drift_ns': None,
            'raw_wall_not_increasing': None, 'raw_wall_step_exceeds_budget': None,
            'raw_wall_anchor_drift_exceeds_budget': None,
            'adjacent_raw_diagnostic_sampling_trusted': False,
            'raw_wall_anomaly': False,
        }
        invalid = tuple(name + '_invalid' for name, value in (
            ('raw_wall_ns', raw_wall_ns), ('before_ns', before_ns), ('after_ns', after_ns))
            if not _positive_int(value))
        if invalid:
            self._latch(invalid, sampling)
        else:
            span = after_ns - before_ns
            midpoint = (before_ns + after_ns) // 2
            uncertain = span > self.maximum_sampling_span_ns
            sampling.update({'sampling_span_ns': span, 'monotonic_midpoint_ns': midpoint,
                             'sampling_uncertain': uncertain})
            if uncertain:
                self._uncertain += 1
            if span < 0:
                self._latch(('monotonic_bracket_reversed',), sampling)
            else:
                trusted = not uncertain
                previous = self._last_raw_pair
                if previous is not None:
                    wall_step = raw_wall_ns - previous['wall_ns']
                    mono_step = midpoint - previous['monotonic_ns']
                    residual = wall_step - mono_step
                    adjacent_trusted = trusted and previous['trusted']
                    sampling.update({
                        'raw_wall_step_ns': wall_step, 'monotonic_step_ns': mono_step,
                        'raw_wall_step_residual_ns': residual,
                        'adjacent_raw_diagnostic_sampling_trusted': adjacent_trusted,
                        'raw_wall_not_increasing': wall_step <= 0,
                        'raw_wall_step_exceeds_budget': abs(residual) > RAW_WALL_ANOMALY_TOLERANCE_NS,
                    })
                    if adjacent_trusted and (wall_step <= 0 or abs(residual) > RAW_WALL_ANOMALY_TOLERANCE_NS):
                        sampling['raw_wall_anomaly'] = True
                if self._anchor_wall_ns is not None:
                    drift = ((raw_wall_ns - self._anchor_wall_ns) -
                             (midpoint - self._anchor_monotonic_ns))
                    sampling.update({'raw_wall_anchor_drift_ns': drift,
                        'raw_wall_anchor_drift_exceeds_budget': abs(drift) > RAW_WALL_ANOMALY_TOLERANCE_NS})
                    if trusted and abs(drift) > RAW_WALL_ANOMALY_TOLERANCE_NS:
                        sampling['raw_wall_anomaly'] = True
                self._ever_raw_wall_anomaly |= sampling['raw_wall_anomaly']
                self._last_raw_pair = {'wall_ns': raw_wall_ns, 'monotonic_ns': midpoint, 'trusted': trusted}
                if (self._last_observed_monotonic_ns is not None and
                        midpoint <= self._last_observed_monotonic_ns):
                    self._latch(('monotonic_not_increasing',), sampling)
                # Observed ordering includes uncertain brackets. They never
                # change the accepted state or the first trusted anchor.
                self._last_observed_monotonic_ns = midpoint

        if self._first_fault is not None:
            return self._finish(sampling, reasons=(
                'transport_clock_fault_latched', *self._first_fault['reasons']))
        if sampling['sampling_uncertain']:
            return self._finish(sampling, reasons=('clock_pair_sampling_uncertain',))

        midpoint = sampling['monotonic_midpoint_ns']
        mapped = (raw_wall_ns if self._anchor_wall_ns is None else
                  self._anchor_wall_ns + midpoint - self._anchor_monotonic_ns)
        sampling['candidate_mapped_ns_before_validation'] = mapped
        if mapped <= 0 or mapped > MAX_ROS_TIME_NS:
            self._latch(('mapped_timestamp_out_of_ros_range',), sampling)
            return self._finish(sampling, reasons=(
                'transport_clock_fault_latched', *self._first_fault['reasons']))
        if self._last_accepted_mapped_ns is not None and mapped <= self._last_accepted_mapped_ns:
            self._latch(('mapped_timestamp_not_increasing',), sampling)
            return self._finish(sampling, reasons=(
                'transport_clock_fault_latched', *self._first_fault['reasons']))
        mapped_usec = mapped // 1000
        if mapped_usec <= 0 or (self._last_accepted_mapped_usec is not None and
                               mapped_usec <= self._last_accepted_mapped_usec):
            self._latch(('mapped_microsecond_not_increasing_or_nonpositive',), sampling)
            return self._finish(sampling, reasons=(
                'transport_clock_fault_latched', *self._first_fault['reasons']))
        if self._anchor_wall_ns is None:
            self._anchor_wall_ns = raw_wall_ns
            self._anchor_monotonic_ns = midpoint
            sampling['raw_wall_anchor_drift_ns'] = 0
            sampling['raw_wall_anchor_drift_exceeds_budget'] = False
        self._last_accepted_monotonic_ns = midpoint
        self._last_accepted_mapped_ns = mapped
        self._last_accepted_mapped_usec = mapped_usec
        return self._finish(sampling, accepted=True, mapped=mapped)

    def snapshot(self) -> dict:
        return {
            'schema_version': 1, 'mode': 'epoch_anchor_plus_host_monotonic_candidate',
            'anchor_wall_ns': self._anchor_wall_ns, 'anchor_monotonic_ns': self._anchor_monotonic_ns,
            'anchor_initializations': int(self._anchor_wall_ns is not None),
            'last_observed_monotonic_ns': self._last_observed_monotonic_ns,
            'last_accepted_monotonic_ns': self._last_accepted_monotonic_ns,
            'last_accepted_mapped_ns': self._last_accepted_mapped_ns,
            'last_accepted_mapped_usec': self._last_accepted_mapped_usec,
            'last_sampling': deepcopy(self._last_sampling),
            'fault_latched': self._first_fault is not None, 'first_fault': deepcopy(self._first_fault),
            'ever_raw_wall_anomaly': self._ever_raw_wall_anomaly,
            'maximum_sampling_span_ns': self.maximum_sampling_span_ns,
            'raw_wall_anomaly_tolerance_ns': RAW_WALL_ANOMALY_TOLERANCE_NS,
            'maximum_mapped_ns': MAX_ROS_TIME_NS,
            'observed_count': self._observations, 'accepted_count': self._accepted,
            'rejected_count': self._observations - self._accepted,
            'sampling_uncertain_count': self._uncertain,
            'raw_wall_anomaly_is_separate_from_mapping_fault': True,
            'utc_calibrated': False, 'acquisition_time_calibrated': False, 'fcu_time_calibrated': False,
            'source_acquisition_stamp_not_an_input': True, 'source_freshness_validated': False,
            'candidate_microsecond_uniqueness_enforced': True,
            'actual_mavros_wire_emission_observed': False,
            'timestamp_clamping_used': False, 'automatic_fault_recovery': False,
            'scope': 'Candidate companion transport/generation clock only; fixed epoch anchor is not ongoing UTC. AP arrival-based correction is not acquisition calibration. Sampling acceptance is not source freshness, FCU receipt, EKF fusion, or readiness.',
        }
