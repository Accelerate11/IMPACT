from __future__ import annotations

import re
from typing import Dict, Iterable, List, Sequence

import numpy as np

from .geometry import align_se2


def append_state_transition(
    timeline: List[Dict[str, object]],
    stamp_s: float,
    state: str,
) -> bool:
    """Append only state transitions, not a high-rate duplicate heartbeat."""
    if timeline and timeline[-1].get("state") == state:
        return False
    timeline.append({"sim_time_s": float(stamp_s), "state": str(state)})
    return True


def fault_evidence_summary(events: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """Build a compact, deterministic summary of observed test fault events."""
    fault_ids = list(dict.fromkeys(str(item.get("fault_id", "")) for item in events))
    targets = list(dict.fromkeys(str(item.get("target_module", "")) for item in events))
    return {
        "observed_count": len(events),
        "fault_ids_observed": [item for item in fault_ids if item],
        "targets_observed": [item for item in targets if item],
    }


def trajectory_metrics(estimate_xy: Iterable[Iterable[float]], truth_xy: Iterable[Iterable[float]]) -> Dict[str, float]:
    aligned, _, _ = align_se2(estimate_xy, truth_xy)
    truth = np.asarray(truth_xy, dtype=float)
    errors = np.linalg.norm(aligned - truth, axis=1)
    return {
        "samples": int(errors.size),
        "ate_rms_m": float(np.sqrt(np.mean(errors ** 2))),
        "ate_mean_m": float(np.mean(errors)),
        "ate_median_m": float(np.median(errors)),
        "ate_p95_m": float(np.percentile(errors, 95)),
        "ate_max_m": float(np.max(errors)),
    }


def frequency_metrics(stamps_s: Sequence[float], window_s: float = 1.0) -> Dict[str, float]:
    stamps = np.asarray(stamps_s, dtype=float)
    if stamps.size < 2:
        raise ValueError("at least two timestamps are required")
    if np.any(np.diff(stamps) <= 0.0):
        raise ValueError("timestamps must be strictly increasing")
    periods = np.diff(stamps)
    counts = []
    left = 0
    # Each sample pair represents one completed update interval.  Counting
    # both endpoint samples made an exact 10 Hz stream look like 11 Hz in a
    # closed one-second window.  Use interval count (right - left) and a small
    # floating-point tolerance at the lower endpoint instead.
    endpoint_tolerance = max(1.0e-12, abs(float(window_s)) * 1.0e-9)
    for right, stamp in enumerate(stamps):
        while stamp - stamps[left] > window_s + endpoint_tolerance:
            left += 1
        if stamp >= stamps[0] + window_s:
            counts.append(right - left)
    duration = stamps[-1] - stamps[0]
    return {
        "samples": int(stamps.size),
        "mean_hz": float((stamps.size - 1) / duration),
        "period_p50_s": float(np.percentile(periods, 50)),
        "period_p95_s": float(np.percentile(periods, 95)),
        "period_p99_s": float(np.percentile(periods, 99)),
        "max_gap_s": float(np.max(periods)),
        "worst_window_hz": float(min(counts) / window_s) if counts else 0.0,
    }


SIMULATED_PASS = "SIMULATED_PASS"
SIMULATED_FAIL = "SIMULATED_FAIL"
INSUFFICIENT_EVIDENCE = "INSUFFICIENT_EVIDENCE"


def _fault_code(fault_id: object) -> str | None:
    match = re.match(r"^F([1-8])(?:_|$)", str(fault_id).strip(), flags=re.IGNORECASE)
    return f"F{match.group(1)}" if match else None


def _state_observations(response: Dict[str, object]) -> List[str]:
    state_response = response.get("autonomy_state_response", {})
    observations: List[str] = []
    for state in (state_response.get("state_at_start"), state_response.get("state_at_end")):
        if state:
            observations.append(str(state))
    for item in state_response.get("transitions_in_window", []):
        state = item.get("state") if isinstance(item, dict) else None
        if state:
            observations.append(str(state))
    return list(dict.fromkeys(observations))


def _base_state(state: str) -> str:
    return str(state).split(":", 1)[0]


def _acceptance_check(
    name: str,
    expected: object,
    observed: object,
    passed: bool | None,
) -> Dict[str, object]:
    status = (
        INSUFFICIENT_EVIDENCE
        if passed is None
        else SIMULATED_PASS
        if passed
        else SIMULATED_FAIL
    )
    return {
        "name": name,
        "status": status,
        "expected": expected,
        "observed": observed,
    }


def evaluate_fault_response(response: Dict[str, object]) -> Dict[str, object]:
    """Evaluate one F1--F8 response using simulation-only observable evidence.

    The result is deliberately scoped to the Gazebo/proxy stack.  It does not
    grant hardware, real-time, production-algorithm, or flight acceptance.
    """

    fault = response.get("fault", {})
    fault_id = str(fault.get("fault_id", ""))
    code = _fault_code(fault_id)
    window = response.get("window", {})
    state_values = _state_observations(response)
    base_states = [_base_state(state) for state in state_values]
    target_health = response.get("target_health_response", {})
    critical_health = response.get("critical_health_response", {})
    streams = response.get("core_stream_continuity", {})
    network = response.get("network_stats_response", {})
    checks: List[Dict[str, object]] = []

    def add(
        name: str,
        expected: object,
        observed: object,
        passed: bool | None,
    ) -> None:
        checks.append(_acceptance_check(name, expected, observed, passed))

    def state_contains(*expected_states: str) -> None:
        expected = list(expected_states)
        passed = (
            None
            if not state_values
            else any(
                expected_state in state_values or expected_state in base_states
                for expected_state in expected
            )
        )
        add("autonomy_state_contains", expected, state_values, passed)

    def states_only(allowed_states: Sequence[str]) -> None:
        allowed = list(allowed_states)
        passed = (
            None
            if not base_states
            else all(state in allowed for state in base_states)
        )
        add("autonomy_states_remain_allowed", allowed, base_states, passed)

    def state_marker(marker: str) -> None:
        passed = None if not state_values else any(marker in state for state in state_values)
        add("autonomy_state_marker", marker, state_values, passed)

    def health_in(expected_states: Sequence[str]) -> None:
        expected = list(expected_states)
        samples = int(target_health.get("samples_in_window", 0) or 0)
        observed = list(target_health.get("states_observed", []))
        passed = None if samples <= 0 else any(state in expected for state in observed)
        add("target_health_state", expected, observed, passed)

    def critical_health_clear() -> None:
        samples = int(critical_health.get("samples_in_window", 0) or 0)
        failed = list(critical_health.get("fail_modules", []))
        passed = None if samples <= 0 else not failed
        add("critical_health_has_no_fail", [], failed, passed)

    def stream_status(stream_name: str, expected_status: str) -> None:
        stream = streams.get(stream_name, {})
        observed = stream.get("status")
        passed = (
            None
            if observed in (None, "INCOMPLETE_OBSERVATION_WINDOW")
            else observed == expected_status
        )
        add(f"{stream_name}_continuity", expected_status, observed, passed)

    window_complete = window.get("complete")
    add(
        "fault_window_complete",
        True,
        window_complete,
        True if window_complete is True else None,
    )

    expected_behavior = {
        "F1": "camera failure is visible; geometry-only navigation and core streams continue",
        "F2": "NPU failure is visible; geometry-only navigation and core streams continue",
        "F3": "planner timeout is visible; Sentinel enters BRAKE and core streams remain available",
        "F4": "LiDAR outage is visible; CAUTIOUS escalates to HOVER/LAND while proxy odom continues and map updates pause",
        "F5": "localization degradation is visible; Sentinel enters RELOCALIZE and core streams remain available",
        "F6": "20% project-relay loss is active and measured while autonomous task and core streams continue",
        "F7": "CPU load causes visible non-critical degradation while critical health and core streams remain available",
        "F8": "low battery causes RETURN or LAND while telemetry streams remain available",
    }.get(code, "unsupported or unrecognized fault ID")

    if code in ("F1", "F2"):
        health_in(("FAIL",))
        state_marker("GEOMETRY_ONLY")
        states_only(("NORMAL",))
        stream_status("proxy_odom", "CONTINUOUS")
        stream_status("map_nav", "CONTINUOUS")
    elif code == "F3":
        health_in(("FAIL",))
        state_contains("BRAKE")
        stream_status("proxy_odom", "CONTINUOUS")
        stream_status("map_nav", "CONTINUOUS")
    elif code == "F4":
        health_in(("WARN", "FAIL"))
        state_contains("CAUTIOUS")
        # An outage longer than one simulated second must escalate beyond the
        # short-outage propagation mode.
        escalation = (
            None
            if not state_values
            else any(state in base_states for state in ("HOVER", "LAND"))
        )
        add(
            "lidar_outage_escalation",
            ["HOVER", "LAND"],
            base_states,
            escalation,
        )
        stream_status("proxy_odom", "CONTINUOUS")
        stream_status("map_nav", "GAP_OBSERVED")
    elif code == "F5":
        health_in(("FAIL",))
        state_contains("RELOCALIZE")
        stream_status("proxy_odom", "CONTINUOUS")
        stream_status("map_nav", "CONTINUOUS")
    elif code == "F6":
        health_in(("FAIL",))
        states_only(("NORMAL",))
        records = int(network.get("records_in_window", 0) or 0)
        active_seen = network.get("active_seen")
        matching_id_seen = network.get("matching_fault_id_seen")
        observed_drop_rate = network.get("observed_fault_window_drop_rate")
        sent_delta = network.get("fault_window_sent_delta")
        counter_delta_drop_rate = network.get("counter_delta_drop_rate")
        add(
            "network_fault_active_seen",
            True,
            active_seen,
            None if records <= 0 or active_seen is None else bool(active_seen),
        )
        add(
            "network_fault_id_matches",
            fault_id,
            network.get("fault_ids_seen", []),
            None if records <= 0 or matching_id_seen is None else bool(matching_id_seen),
        )
        rate_passed = (
            None
            if observed_drop_rate is None
            else 0.15 <= float(observed_drop_rate) <= 0.25
        )
        add(
            "network_fault_window_drop_rate",
            {"minimum": 0.15, "maximum": 0.25},
            observed_drop_rate,
            rate_passed,
        )
        sample_count_passed = (
            None if sent_delta is None else int(sent_delta) >= 40
        )
        add(
            "network_fault_window_sample_count",
            {"minimum_sent": 40},
            sent_delta,
            sample_count_passed,
        )
        counter_rate_passed = (
            None
            if counter_delta_drop_rate is None
            else 0.15 <= float(counter_delta_drop_rate) <= 0.25
        )
        add(
            "network_counter_delta_drop_rate",
            {"minimum": 0.15, "maximum": 0.25},
            counter_delta_drop_rate,
            counter_rate_passed,
        )
        stream_status("proxy_odom", "CONTINUOUS")
        stream_status("map_nav", "CONTINUOUS")
    elif code == "F7":
        health_in(("WARN", "FAIL"))
        state_marker("ESSENTIAL_ONLY")
        states_only(("NORMAL", "CAUTIOUS"))
        critical_health_clear()
        stream_status("proxy_odom", "CONTINUOUS")
        stream_status("map_nav", "CONTINUOUS")
    elif code == "F8":
        state_contains("RETURN", "LAND")
        critical_health_clear()
        stream_status("proxy_odom", "CONTINUOUS")
        stream_status("map_nav", "CONTINUOUS")
    else:
        add(
            "recognized_fault_id",
            "F1--F8 prefix",
            fault_id,
            False if fault_id else None,
        )

    statuses = [str(check["status"]) for check in checks]
    overall_status = (
        SIMULATED_FAIL
        if SIMULATED_FAIL in statuses
        else INSUFFICIENT_EVIDENCE
        if INSUFFICIENT_EVIDENCE in statuses
        else SIMULATED_PASS
    )
    return {
        "fault_code": code,
        "status": overall_status,
        "expected_behavior": expected_behavior,
        "expected_checks": checks,
        "acceptance_scope": (
            "Gazebo SIL/proxy evidence only; this is not formal hardware, "
            "production-algorithm, real-time, or flight acceptance."
        ),
    }


def replan_metrics(latencies_s: Sequence[float], accepted: Sequence[bool]) -> Dict[str, float]:
    latency = np.asarray(latencies_s, dtype=float)
    ok = np.asarray(accepted, dtype=bool)
    if latency.size == 0 or latency.size != ok.size:
        raise ValueError("latencies and accepted must have equal non-zero length")
    return {
        "events": int(latency.size),
        "success_rate": float(np.mean(ok)),
        "mean_s": float(np.mean(latency)),
        "p50_s": float(np.percentile(latency, 50)),
        "p95_s": float(np.percentile(latency, 95)),
        "p99_s": float(np.percentile(latency, 99)),
        "max_s": float(np.max(latency)),
        "official_2s_pass": bool(np.all(latency <= 2.0) and np.all(ok)),
    }
