import numpy as np
import pytest

from xq_autonomy.minimum_excitation import (
    CandidateForecast,
    RecoveryCandidate,
    build_information_profile,
    generate_discrete_candidates,
    predict_covariance_profile,
    select_minimum_excitation,
)


def _candidate(name, *, cost_distance=0.0, duration=4.0):
    positions = np.column_stack((np.linspace(0.0, 2.0, 3), np.zeros(3), np.ones(3)))
    return RecoveryCandidate(
        name=name,
        positions=positions,
        yaw=np.zeros(3),
        duration=duration,
        extra_path_length=cost_distance,
        extra_energy=cost_distance,
    )


def _forecast(candidate, information_y):
    information = np.zeros((3, 3, 3))
    information[:, 1, 1] = information_y
    return CandidateForecast(
        candidate=candidate,
        alert_limits=np.full(3, 0.25),
        obstacle_directions=np.tile((0.0, 1.0, 0.0), (3, 1)),
        information_profile=information,
    )


def test_generates_the_frozen_discrete_action_set_and_preserves_endpoints():
    baseline = np.column_stack((np.linspace(0.0, 4.0, 9), np.zeros(9), np.ones(9)))
    candidates = generate_discrete_candidates(
        baseline,
        baseline_duration=4.0,
        previous_high_quality_pose=np.array((-0.5, 0.0, 1.0)),
    )
    assert [candidate.name for candidate in candidates] == [
        "baseline", "left_lateral", "right_lateral", "up_offset", "down_offset",
        "slow_trajectory", "short_hover", "backtrack",
    ]
    for candidate in candidates[:7]:
        assert np.allclose(candidate.positions[0], baseline[0])
        assert np.allclose(candidate.positions[-1], baseline[-1])
    assert candidates[1].positions[:, 1].max() == pytest.approx(0.4)
    assert candidates[3].positions[:, 2].max() == pytest.approx(1.25)
    assert candidates[5].duration == pytest.approx(8.0)


def test_information_map_weights_direction_confidence_quality_and_age():
    samples = np.array(((0.0, 0.0, 1.0), (1.0, 0.0, 1.0)))
    surfels = np.array(((0.0, 0.5, 1.0), (0.0, 0.0, 1.5), (9.0, 9.0, 9.0)))
    normals = np.array(((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)))
    profile = build_information_profile(
        samples, surfels, normals,
        static_confidence=np.array((1.0, 0.5, 1.0)),
        geometry_quality=np.array((1.0, 0.5, 1.0)),
        last_seen=np.array((10.0, 10.0, 10.0)),
        now=10.0, visibility_radius=1.1, age_time_constant=5.0, information_scale=4.0,
    )
    assert profile[0, 1, 1] == pytest.approx(4.0)
    assert profile[0, 2, 2] == pytest.approx(1.0)
    assert profile[:, 0, 0].sum() == 0.0


def test_information_form_update_reduces_only_observed_direction():
    prior = np.diag((0.04, 0.04, 0.04))
    information = np.zeros((2, 3, 3))
    information[:, 1, 1] = 25.0
    profile = predict_covariance_profile(prior, information)
    assert profile[0, 1, 1] == pytest.approx(0.02)
    assert profile[1, 1, 1] == pytest.approx(1.0 / 75.0)
    assert np.allclose(profile[:, 0, 0], 0.04)
    assert np.allclose(profile[:, 2, 2], 0.04)


def test_selects_minimum_cost_feasible_not_maximum_observability():
    forecasts = (
        _forecast(_candidate("baseline"), information_y=0.0),
        _forecast(_candidate("small_lateral", cost_distance=0.2), information_y=120.0),
        _forecast(_candidate("large_detour", cost_distance=2.0), information_y=1000.0),
    )
    selection = select_minimum_excitation(
        forecasts,
        np.diag((0.0001, 0.01, 0.0001)),
        k_alpha=2.0,
        margin_reserve=0.10,
        lambda_energy=0.5,
        lambda_distance=1.0,
    )
    by_name = {prediction.candidate.name: prediction for prediction in selection.predictions}
    assert not by_name["baseline"].feasible
    assert by_name["small_lateral"].feasible
    assert by_name["large_detour"].information_trace > by_name["small_lateral"].information_trace
    assert selection.selected_name == "small_lateral"


def test_returns_baseline_when_no_recovery_is_needed_and_fails_closed_if_none_recovers():
    safe_baseline = CandidateForecast(
        candidate=_candidate("baseline"),
        alert_limits=np.full(3, 0.5),
        obstacle_directions=np.tile((0.0, 1.0, 0.0), (3, 1)),
        information_profile=np.zeros((3, 3, 3)),
    )
    safe = select_minimum_excitation(
        (safe_baseline, _forecast(_candidate("detour", cost_distance=1.0), 100.0)),
        np.diag((0.0001, 0.01, 0.0001)),
        k_alpha=2.0, margin_reserve=0.10, lambda_energy=1.0, lambda_distance=1.0,
    )
    assert not safe.baseline_insufficient
    assert safe.selected_name == "baseline"

    failed = select_minimum_excitation(
        (_forecast(_candidate("baseline"), 0.0), _forecast(_candidate("weak_action"), 1.0)),
        np.diag((0.0001, 0.01, 0.0001)),
        k_alpha=2.0, margin_reserve=0.10, lambda_energy=1.0, lambda_distance=1.0,
    )
    assert failed.baseline_insufficient
    assert not failed.recovery_found
    assert failed.selected_name is None


def test_rejects_indefinite_information_increment():
    information = np.zeros((1, 3, 3))
    information[0, 0, 0] = -1.0
    with pytest.raises(ValueError, match="positive semidefinite"):
        predict_covariance_profile(np.eye(3), information)


def test_prediction_variance_floor_prevents_false_zero_uncertainty():
    forecast = _forecast(_candidate("baseline"), information_y=1.0e9)
    selection = select_minimum_excitation(
        (forecast,),
        np.diag((1.0e-4, 1.0e-2, 1.0e-4)),
        k_alpha=50.0,
        margin_reserve=0.10,
        lambda_energy=0.0,
        lambda_distance=0.0,
        minimum_prediction_variance=1.0e-5,
    )
    prediction = selection.predictions[0]
    assert prediction.protection_levels.min() == pytest.approx(50.0 * np.sqrt(1.0e-5))
    assert not prediction.feasible
