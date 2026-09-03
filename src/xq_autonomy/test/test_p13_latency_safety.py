import math

import numpy as np
import pytest

from xq_autonomy.latency_safety import (
    latency_radius,
    nearest_rank,
    safety_envelope,
    summarize_latencies,
)
from xq_autonomy.p13_flight_controller_node import (
    P13FlightControllerNode,
    localization_snapshot_covers_sensor,
    online_speed_limited_parameter,
    ordered_map_dependency_completion,
    runtime_integrity_margin,
)
from xq_autonomy.p11_flight_controller_node import (
    P11FlightControllerNode,
    build_geometric_candidate_positions,
    build_lattice_candidate_positions,
)


def test_nearest_rank_is_conservative_and_non_interpolating():
    values = [0.01 * index for index in range(1, 101)]
    assert nearest_rank(values, 0.50) == pytest.approx(0.50)
    assert nearest_rank(values, 0.95) == pytest.approx(0.95)
    assert nearest_rank(values, 0.99) == pytest.approx(0.99)
    assert nearest_rank(values, 1.00) == pytest.approx(1.00)


def test_statistics_reject_invalid_samples():
    stats = summarize_latencies([0.05, math.nan, -1.0, 0.20])
    assert stats.count == 2
    assert stats.p50_s == pytest.approx(0.05)
    assert stats.p99_s == pytest.approx(0.20)
    assert stats.maximum_s == pytest.approx(0.20)


def test_latency_radius_matches_project_formula():
    assert latency_radius(0.4, 0.2, 0.8) == pytest.approx(0.096)


def test_high_latency_reduces_speed_for_identical_geometry():
    common = dict(
        geometric_clearance_m=0.82,
        fixed_buffer_m=0.58,
        protection_level_m=0.10,
        required_margin_m=0.06,
        maximum_speed_mps=0.42,
        maximum_acceleration_mps2=0.8,
    )
    low = safety_envelope(latency_s=0.07, **common)
    high = safety_envelope(latency_s=0.22, **common)
    assert low.speed_limit_mps == pytest.approx(0.42)
    assert high.speed_limit_mps < low.speed_limit_mps
    assert high.alert_limit_m <= low.alert_limit_m
    assert high.integrity_margin_m >= common["required_margin_m"] - 1.0e-12


def test_online_envelope_limits_velocity_without_freezing_trajectory_time():
    assert online_speed_limited_parameter(
        "maximum_speed_mps", 0.42, 0.04
    ) == pytest.approx(0.04)
    assert online_speed_limited_parameter(
        "trajectory_duration_s", 28.0, 0.04
    ) == pytest.approx(28.0)


def test_runtime_integrity_margin_uses_current_map_covariance_and_latency():
    common = dict(
        position=np.zeros(3),
        velocity=np.asarray((0.4, 0.0, 0.0)),
        static_points=np.asarray(((1.0, 0.0, 0.0), (3.0, 1.0, 0.0))),
        integrity_covariance=np.diag((0.0004, 0.0001, 0.0001)),
        k_alpha=3.0,
        maximum_acceleration_mps2=0.8,
        body_radius_m=0.35,
        base_reserve_m=0.10,
        tracking_reserve_m=0.10,
    )
    low = runtime_integrity_margin(latency_p99_s=0.05, **common)
    high = runtime_integrity_margin(latency_p99_s=0.25, **common)
    assert low[0] == pytest.approx(low[1] - low[2])
    assert high[0] < low[0]
    assert low[2] == pytest.approx(0.06)


def test_existing_localization_snapshot_can_complete_sensor_stage_immediately():
    assert localization_snapshot_covers_sensor(1_000_000_000, 1_000_000_000)
    assert localization_snapshot_covers_sensor(1_000_000_000, 1_249_999_999)
    assert not localization_snapshot_covers_sensor(1_000_000_000, 999_999_999)
    assert not localization_snapshot_covers_sensor(1_000_000_000, 1_250_000_001)


def test_exact_source_map_conservatively_closes_missing_localization_delivery():
    assert ordered_map_dependency_completion(200, 0) == (200, 200)
    assert ordered_map_dependency_completion(200, 150) == (150, 200)
    # Preserve ordering even if clocks/callback ordering produce a later
    # localization observation.
    assert ordered_map_dependency_completion(200, 250) == (250, 250)


def test_impossible_budget_fails_closed_to_zero_speed():
    envelope = safety_envelope(
        latency_s=1.0,
        geometric_clearance_m=0.5,
        fixed_buffer_m=0.4,
        protection_level_m=0.1,
        required_margin_m=0.1,
        maximum_speed_mps=0.4,
        maximum_acceleration_mps2=1.0,
    )
    assert envelope.speed_limit_mps == 0.0
    assert envelope.integrity_margin_m < 0.0


def test_minimum_excitation_reverses_at_time_and_spatial_bounds():
    direction = P13FlightControllerNode._minimum_excitation_direction
    assert direction(
        current_y=0.0, anchor_y=0.0, elapsed_s=0.5,
        maximum_offset_m=0.35, half_period_s=3.0,
    ) == -1.0
    assert direction(
        current_y=0.0, anchor_y=0.0, elapsed_s=3.1,
        maximum_offset_m=0.35, half_period_s=3.0,
    ) == 1.0
    assert direction(
        current_y=-0.36, anchor_y=0.0, elapsed_s=0.5,
        maximum_offset_m=0.35, half_period_s=3.0,
    ) == 1.0
    assert direction(
        current_y=0.36, anchor_y=0.0, elapsed_s=3.1,
        maximum_offset_m=0.35, half_period_s=3.0,
    ) == -1.0


def test_weak_direction_recovery_moves_away_from_nearest_surface():
    choose = P13FlightControllerNode._weak_direction_away_from_obstacle
    position = np.asarray((2.0, 0.0, 1.2))
    ground = np.asarray(((2.0, 0.0, 0.0), (2.0, 2.0, 1.2)))
    upward = choose(position, np.asarray((0.01, 0.0, -0.999)), ground)
    assert upward is not None
    assert upward[2] > 0.99
    assert np.linalg.norm(upward) == pytest.approx(1.0)

    north_wall = np.asarray(((2.0, 1.0, 1.2), (2.0, -3.0, 1.2)))
    south = choose(position, np.asarray((0.0, 1.0, 0.0)), north_wall)
    assert south is not None
    assert south.tolist() == pytest.approx([0.0, -1.0, 0.0])

    assert choose(position, np.zeros(3), ground) is None


def test_rejected_candidate_recovery_uses_best_nonforward_excitation_only():
    choose = P13FlightControllerNode._best_rejected_candidate_direction
    current = np.asarray((4.0, 0.0, 1.0))
    direct = np.asarray(((4.0, 0.0, 1.0), (5.0, 0.0, 1.0), (6.0, 0.0, 1.0)))
    left = np.asarray(((4.0, 0.0, 1.0), (5.0, 0.7, 1.0), (6.0, 0.7, 1.0)))
    upward = np.asarray(((4.0, 0.0, 1.0), (5.0, 0.0, 1.7), (6.0, 0.0, 1.7)))
    direction = choose(current, [direct, left, upward], [-0.2, -0.05, -0.1])
    assert direction is not None
    assert direction.tolist() == pytest.approx([0.0, 1.0, 0.0])
    assert choose(current, [direct], [math.nan]) is None


def test_lateral_candidate_profiles_support_return_and_lane_shift():
    phase = np.asarray((0.0, 0.25, 0.5, 0.85, 1.0))
    returning = P11FlightControllerNode._lateral_window(
        phase, "return_to_center"
    )
    shifting = P11FlightControllerNode._lateral_window(phase, "lane_shift")
    challenge = P11FlightControllerNode._lateral_window(
        phase, "challenge_then_center"
    )
    assert returning[0] == pytest.approx(0.0)
    assert returning[-1] == pytest.approx(0.0)
    assert returning[2] == pytest.approx(1.0)
    assert shifting[0] == pytest.approx(0.0)
    assert shifting[-1] == pytest.approx(1.0)
    assert np.all(np.diff(shifting) >= 0.0)
    assert shifting[2] == pytest.approx(1.0)
    assert challenge[0] == pytest.approx(0.0)
    assert challenge[1] == pytest.approx(1.0)
    assert challenge[2] == pytest.approx(1.0)
    assert 0.0 < challenge[3] < 1.0
    assert challenge[-1] == pytest.approx(0.0)
    with pytest.raises(ValueError):
        P11FlightControllerNode._lateral_window(phase, "unsafe_unknown_shape")


def test_optional_vertical_candidate_extends_family_without_changing_defaults():
    direct = np.column_stack((np.linspace(0.0, 2.0, 5), np.zeros(5), np.ones(5)))
    profile = np.asarray((0.0, 1.0, 1.0, 0.5, 0.0))
    planar = build_geometric_candidate_positions(
        direct, profile, lateral_offset_m=0.7
    )
    spatial = build_geometric_candidate_positions(
        direct,
        profile,
        lateral_offset_m=0.7,
        enable_vertical_candidate=True,
        vertical_offset_m=0.8,
    )
    compositional = build_geometric_candidate_positions(
        direct,
        profile,
        lateral_offset_m=0.7,
        enable_vertical_candidate=True,
        enable_diagonal_vertical_candidates=True,
        vertical_offset_m=0.8,
    )
    assert [name for name, _ in planar] == [
        "high_information_direct",
        "geometry_rich_right",
        "geometry_rich_left",
    ]
    assert [name for name, _ in spatial] == [
        "high_information_direct",
        "geometry_rich_right",
        "geometry_rich_left",
        "geometry_rich_up",
    ]
    assert np.max(spatial[1][1][:, 1]) == pytest.approx(0.0)
    assert np.min(spatial[1][1][:, 1]) == pytest.approx(-0.7)
    assert np.max(spatial[2][1][:, 1]) == pytest.approx(0.7)
    assert np.max(spatial[3][1][:, 2]) == pytest.approx(1.8)
    assert np.allclose(spatial[3][1][[0, -1], 2], 1.0)
    assert [name for name, _ in compositional] == [
        "high_information_direct",
        "geometry_rich_right",
        "geometry_rich_left",
        "geometry_rich_up",
        "geometry_rich_up_right",
        "geometry_rich_up_left",
    ]
    assert np.min(compositional[4][1][:, 1]) == pytest.approx(-0.7)
    assert np.max(compositional[4][1][:, 2]) == pytest.approx(1.8)
    assert np.max(compositional[5][1][:, 1]) == pytest.approx(0.7)
    assert np.max(compositional[5][1][:, 2]) == pytest.approx(1.8)
    assert np.allclose(compositional[4][1][[0, -1]], direct[[0, -1]])
    assert np.allclose(compositional[5][1][[0, -1]], direct[[0, -1]])
    with pytest.raises(ValueError):
        build_geometric_candidate_positions(
            direct,
            profile,
            lateral_offset_m=0.7,
            enable_vertical_candidate=True,
            vertical_offset_m=0.0,
        )


def test_research_lattice_is_generic_symmetric_and_keeps_direct_first():
    direct = np.column_stack((np.linspace(0.0, 2.0, 5), np.zeros(5), np.ones(5)))
    profile = np.asarray((0.0, 1.0, 1.0, 0.5, 0.0))
    candidates = build_lattice_candidate_positions(
        direct,
        profile,
        lateral_offset_m=0.8,
        lateral_levels=5,
        vertical_offset_m=0.6,
        vertical_levels=2,
    )
    assert len(candidates) == 10
    assert candidates[0][0] == "task_efficient_direct"
    lateral_extrema = [float(np.max(path[:, 1])) for _, path in candidates]
    assert max(lateral_extrema) == pytest.approx(0.8)
    assert min(float(np.min(path[:, 1])) for _, path in candidates) == pytest.approx(-0.8)
    assert max(float(np.max(path[:, 2])) for _, path in candidates) == pytest.approx(1.6)
    assert all(np.allclose(path[[0, -1]], direct[[0, -1]]) for _, path in candidates)
    with pytest.raises(ValueError, match="lattice"):
        build_lattice_candidate_positions(
            direct, profile, lateral_offset_m=0.8, lateral_levels=4
        )
    with pytest.raises(ValueError):
        build_geometric_candidate_positions(
            direct,
            profile,
            lateral_offset_m=0.7,
            enable_diagonal_vertical_candidates=True,
            vertical_offset_m=0.0,
        )
