from __future__ import annotations

import pytest

from xq_autonomy.metrics import evaluate_fault_response, frequency_metrics


def _response(
    fault_id: str,
    target: str,
    states: list[str],
    health_states: list[str],
    *,
    odom: str = "CONTINUOUS",
    map_nav: str = "CONTINUOUS",
    critical_fail: list[str] | None = None,
    network: dict[str, object] | None = None,
    complete: bool = True,
) -> dict[str, object]:
    return {
        "fault": {
            "fault_id": fault_id,
            "target_module": target,
            "duration_s": 2.0,
        },
        "window": {"complete": complete},
        "autonomy_state_response": {
            "state_at_start": states[0] if states else None,
            "transitions_in_window": [
                {"sim_time_s": float(index), "state": state}
                for index, state in enumerate(states)
            ],
            "state_at_end": states[-1] if states else None,
        },
        "target_health_response": {
            "samples_in_window": len(health_states),
            "states_observed": health_states,
        },
        "critical_health_response": {
            "samples_in_window": 8,
            "fail_modules": critical_fail or [],
        },
        "core_stream_continuity": {
            "proxy_odom": {"status": odom},
            "map_nav": {"status": map_nav},
        },
        "network_stats_response": network
        or {
            "records_in_window": 0,
            "active_seen": False,
            "matching_fault_id_seen": False,
            "fault_ids_seen": [],
            "observed_fault_window_drop_rate": None,
        },
    }


def test_exact_10hz_worst_window_counts_intervals_not_endpoints() -> None:
    for offset in (0.0, 0.03, 1234.567):
        stamps = [offset + index * 0.1 for index in range(31)]
        result = frequency_metrics(stamps)
        assert result["mean_hz"] == pytest.approx(10.0)
        assert result["worst_window_hz"] == pytest.approx(10.0)


@pytest.mark.parametrize(
    "response",
    [
        _response(
            "F1_camera_unplug",
            "camera",
            ["NORMAL", "NORMAL:GEOMETRY_ONLY"],
            ["OK", "FAIL"],
        ),
        _response(
            "F2_npu_kill",
            "npu",
            ["NORMAL", "NORMAL:GEOMETRY_ONLY"],
            ["OK", "FAIL"],
        ),
        _response(
            "F3_planner_timeout",
            "planner",
            ["NORMAL", "BRAKE"],
            ["OK", "FAIL"],
        ),
        _response(
            "F4_lidar_pause",
            "lidar",
            ["NORMAL", "CAUTIOUS", "HOVER"],
            ["OK", "WARN", "FAIL"],
            map_nav="GAP_OBSERVED",
        ),
        _response(
            "F5_localization_covariance",
            "localization",
            ["NORMAL", "RELOCALIZE"],
            ["OK", "FAIL"],
        ),
        _response(
            "F6_ground_link_20pct",
            "ground_link",
            ["NORMAL"],
            ["OK", "FAIL"],
            network={
                "records_in_window": 51,
                "active_seen": True,
                "matching_fault_id_seen": True,
                "fault_ids_seen": ["F6_ground_link_20pct"],
                "observed_fault_window_drop_rate": 0.20,
                "fault_window_sent_delta": 50,
                "fault_window_dropped_delta": 10,
                "counter_delta_drop_rate": 0.20,
            },
        ),
        _response(
            "F7_cpu_load",
            "cpu",
            ["NORMAL", "NORMAL:ESSENTIAL_ONLY", "CAUTIOUS:ESSENTIAL_ONLY"],
            ["OK", "WARN"],
        ),
        _response(
            "F8_battery_low",
            "battery",
            ["NORMAL", "RETURN"],
            [],
        ),
    ],
)
def test_f1_through_f8_proxy_acceptance_pass_cases(response: dict[str, object]) -> None:
    result = evaluate_fault_response(response)
    assert result["status"] == "SIMULATED_PASS", result
    assert result["acceptance_scope"].startswith("Gazebo SIL/proxy")
    assert all(
        check["status"] == "SIMULATED_PASS"
        for check in result["expected_checks"]
    )


def test_f6_rejects_rate_outside_band_and_missing_active_evidence() -> None:
    response = _response(
        "F6_ground_link_20pct",
        "ground_link",
        ["NORMAL"],
        ["FAIL"],
        network={
            "records_in_window": 50,
            "active_seen": False,
            "matching_fault_id_seen": True,
            "fault_ids_seen": ["F6_ground_link_20pct"],
            "observed_fault_window_drop_rate": 0.14,
            "fault_window_sent_delta": 50,
            "fault_window_dropped_delta": 7,
            "counter_delta_drop_rate": 0.14,
        },
    )
    result = evaluate_fault_response(response)
    assert result["status"] == "SIMULATED_FAIL"
    failed_names = {
        item["name"]
        for item in result["expected_checks"]
        if item["status"] == "SIMULATED_FAIL"
    }
    assert failed_names == {
        "network_fault_active_seen",
        "network_counter_delta_drop_rate",
        "network_fault_window_drop_rate",
    }


def test_f6_rejects_too_few_fault_window_packets() -> None:
    response = _response(
        "F6_ground_link_20pct",
        "ground_link",
        ["NORMAL"],
        ["FAIL"],
        network={
            "records_in_window": 11,
            "active_seen": True,
            "matching_fault_id_seen": True,
            "fault_ids_seen": ["F6_ground_link_20pct"],
            "observed_fault_window_drop_rate": 0.20,
            "fault_window_sent_delta": 10,
            "fault_window_dropped_delta": 2,
            "counter_delta_drop_rate": 0.20,
        },
    )
    result = evaluate_fault_response(response)
    assert result["status"] == "SIMULATED_FAIL"
    sample_check = next(
        item
        for item in result["expected_checks"]
        if item["name"] == "network_fault_window_sample_count"
    )
    assert sample_check["status"] == "SIMULATED_FAIL"


def test_f7_rejects_missing_essential_only_state_marker() -> None:
    response = _response(
        "F7_cpu_load_proxy",
        "cpu",
        ["NORMAL", "CAUTIOUS"],
        ["FAIL"],
    )
    result = evaluate_fault_response(response)
    assert result["status"] == "SIMULATED_FAIL"
    marker_check = next(
        item
        for item in result["expected_checks"]
        if item["name"] == "autonomy_state_marker"
    )
    assert marker_check["expected"] == "ESSENTIAL_ONLY"
    assert marker_check["status"] == "SIMULATED_FAIL"


def test_f6_requires_counter_delta_rate_evidence() -> None:
    response = _response(
        "F6_ground_link_20pct",
        "ground_link",
        ["NORMAL"],
        ["FAIL"],
        network={
            "records_in_window": 50,
            "active_seen": True,
            "matching_fault_id_seen": True,
            "fault_ids_seen": ["F6_ground_link_20pct"],
            "observed_fault_window_drop_rate": 0.20,
            "fault_window_sent_delta": 50,
            "fault_window_dropped_delta": None,
            "counter_delta_drop_rate": None,
        },
    )
    result = evaluate_fault_response(response)
    assert result["status"] == "INSUFFICIENT_EVIDENCE"
    check = next(
        item
        for item in result["expected_checks"]
        if item["name"] == "network_counter_delta_drop_rate"
    )
    assert check["status"] == "INSUFFICIENT_EVIDENCE"


def test_incomplete_fault_window_is_insufficient_not_pass() -> None:
    response = _response(
        "F1_camera_unplug",
        "camera",
        ["NORMAL", "NORMAL:GEOMETRY_ONLY"],
        ["FAIL"],
        complete=False,
    )
    assert evaluate_fault_response(response)["status"] == "INSUFFICIENT_EVIDENCE"
