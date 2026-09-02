import numpy as np
import pytest

from xq_autonomy.integrity_exploration import (
    ExplorationForecast,
    rolling_horizon_distances,
    select_integrity_constrained_exploration,
)
from xq_autonomy.minimum_excitation import CandidateForecast, RecoveryCandidate


def _candidate(
    name,
    trajectory_id,
    *,
    information_gain,
    predicted_information_y,
    alert_limit=0.30,
    collision_probability=0.001,
    energy=7.5,
    return_energy=4.0,
    duration=28.0,
):
    positions = np.column_stack((np.linspace(0.0, 7.5, 21), np.zeros(21), np.ones(21)))
    recovery = RecoveryCandidate(
        name=name,
        positions=positions,
        yaw=np.zeros(len(positions)),
        duration=duration,
        extra_path_length=0.0,
        extra_energy=0.0,
    )
    information = np.zeros((len(positions), 3, 3))
    information[:, 1, 1] = predicted_information_y
    forecast = CandidateForecast(
        candidate=recovery,
        alert_limits=np.full(len(positions), alert_limit),
        obstacle_directions=np.tile((0.0, 1.0, 0.0), (len(positions), 1)),
        information_profile=information,
    )
    return ExplorationForecast(
        trajectory_id=trajectory_id,
        frontier_id=f"frontier_{trajectory_id}",
        forecast=forecast,
        information_gain=information_gain,
        travel_time_s=duration,
        energy_cost=energy,
        return_energy_cost=return_energy,
        collision_probability=collision_probability,
    )


def _select(candidates):
    return select_integrity_constrained_exploration(
        candidates,
        np.diag((1.0e-4, 1.0e-2, 1.0e-4)),
        k_alpha=50.0,
        margin_reserve=0.10,
        collision_probability_limit=0.01,
        energy_remaining=20.0,
        information_weight=1.0,
        travel_time_weight=0.01,
        energy_weight=0.005,
        minimum_prediction_variance=1.0e-5,
    )


def test_hard_filter_rejects_higher_utility_frontier_and_selects_safe_one():
    direct = _candidate(
        "high_information_direct", 1101,
        information_gain=1.0, predicted_information_y=0.0,
    )
    safe = _candidate(
        "geometry_rich_right", 1102,
        information_gain=0.75, predicted_information_y=1.0e8, energy=7.6,
    )
    selection = _select((direct, safe))
    by_name = {
        item.candidate.forecast.candidate.name: item for item in selection.predictions
    }
    assert by_name["high_information_direct"].utility > by_name["geometry_rich_right"].utility
    assert not by_name["high_information_direct"].integrity_feasible
    assert by_name["geometry_rich_right"].integrity_feasible
    assert selection.unconstrained_selected_name == "high_information_direct"
    assert selection.selected_name == "geometry_rich_right"
    assert selection.hard_constraint
    assert not selection.margin_in_utility


def test_margin_is_not_a_soft_reward_inside_the_feasible_set():
    high_utility = _candidate(
        "higher_utility", 1103,
        information_gain=0.85, predicted_information_y=1.0e8,
    )
    high_margin = _candidate(
        "higher_margin", 1104,
        information_gain=0.70, predicted_information_y=1.0e10,
    )
    selection = _select((high_utility, high_margin))
    by_name = {
        item.candidate.forecast.candidate.name: item for item in selection.predictions
    }
    assert all(item.feasible for item in by_name.values())
    assert by_name["higher_margin"].integrity.minimum_margin >= by_name["higher_utility"].integrity.minimum_margin
    assert selection.selected_name == "higher_utility"


def test_collision_and_return_energy_are_independent_hard_constraints():
    collision = _candidate(
        "collision_risk", 1105,
        information_gain=1.0, predicted_information_y=1.0e8,
        collision_probability=0.02,
    )
    energy = _candidate(
        "energy_risk", 1106,
        information_gain=0.95, predicted_information_y=1.0e8,
        energy=17.0, return_energy=4.0,
    )
    safe = _candidate(
        "all_constraints_safe", 1107,
        information_gain=0.60, predicted_information_y=1.0e8,
    )
    selection = _select((collision, energy, safe))
    by_name = {
        item.candidate.forecast.candidate.name: item for item in selection.predictions
    }
    assert not by_name["collision_risk"].collision_feasible
    assert not by_name["energy_risk"].energy_feasible
    assert selection.selected_name == "all_constraints_safe"


def test_fails_closed_when_no_frontier_trajectory_satisfies_all_constraints():
    failed = _select((
        _candidate("integrity_fail", 1108, information_gain=1.0, predicted_information_y=0.0),
        _candidate(
            "collision_fail", 1109,
            information_gain=0.9, predicted_information_y=1.0e8,
            collision_probability=0.5,
        ),
    ))
    assert failed.selected_name is None
    assert failed.unconstrained_selected_name == "integrity_fail"


def test_rejects_duplicate_ids_and_invalid_normalized_information():
    a = _candidate("a", 1110, information_gain=0.5, predicted_information_y=1.0e8)
    b = _candidate("b", 1110, information_gain=0.5, predicted_information_y=1.0e8)
    with pytest.raises(ValueError, match="unique"):
        _select((a, b))
    invalid = _candidate("invalid", 1111, information_gain=0.5, predicted_information_y=1.0e8)
    object.__setattr__(invalid, "information_gain", 1.1)
    with pytest.raises(ValueError, match=r"\[0, 1\]"):
        _select((invalid,))


def test_rolling_horizon_covers_full_corridor_without_overshoot():
    segments = rolling_horizon_distances(-12.0, 12.0, 7.5)
    assert segments == pytest.approx((7.5, 7.5, 7.5, 1.5))
    assert sum(segments) == pytest.approx(24.0)
    assert max(segments) <= 7.5
    with pytest.raises(ValueError, match="mission geometry"):
        rolling_horizon_distances(1.0, -1.0, 7.5)
