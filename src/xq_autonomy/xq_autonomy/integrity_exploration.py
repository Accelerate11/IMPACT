"""Pure P11 integrity-constrained exploration kernel.

Frontier extraction and local trajectory generation remain upstream.  This
module only certifies their candidate trajectories and maximizes task utility
inside the set that satisfies integrity, collision and return-energy hard
constraints.  Integrity margin is deliberately absent from the utility.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .minimum_excitation import CandidateForecast, CandidatePrediction, evaluate_candidate


def rolling_horizon_distances(
    start_x: float, goal_x: float, horizon_m: float
) -> tuple[float, ...]:
    """Partition a forward corridor mission into complete rolling horizons."""
    values = np.asarray((start_x, goal_x, horizon_m), dtype=float)
    if not np.isfinite(values).all() or horizon_m <= 0.0 or goal_x <= start_x:
        raise ValueError("rolling-horizon mission geometry is invalid")
    remaining = float(goal_x - start_x)
    segments = []
    while remaining > 1.0e-9:
        distance = min(float(horizon_m), remaining)
        segments.append(distance)
        remaining -= distance
    return tuple(segments)


@dataclass(frozen=True)
class ExplorationForecast:
    trajectory_id: int
    frontier_id: str
    forecast: CandidateForecast
    information_gain: float
    travel_time_s: float
    energy_cost: float
    return_energy_cost: float
    collision_probability: float


@dataclass(frozen=True)
class ExplorationPrediction:
    candidate: ExplorationForecast
    integrity: CandidatePrediction
    utility: float
    integrity_feasible: bool
    collision_feasible: bool
    energy_feasible: bool
    feasible: bool


@dataclass(frozen=True)
class ExplorationSelection:
    selected_name: str | None
    unconstrained_selected_name: str | None
    predictions: tuple[ExplorationPrediction, ...]
    hard_constraint: bool = True
    margin_in_utility: bool = False


def _validate_candidate(candidate: ExplorationForecast) -> None:
    scalars = np.asarray(
        (
            candidate.information_gain,
            candidate.travel_time_s,
            candidate.energy_cost,
            candidate.return_energy_cost,
            candidate.collision_probability,
        ),
        dtype=float,
    )
    if not np.isfinite(scalars).all():
        raise ValueError("exploration candidate metrics must be finite")
    if not 0.0 <= candidate.information_gain <= 1.0:
        raise ValueError("normalized information gain must lie in [0, 1]")
    if candidate.travel_time_s <= 0.0:
        raise ValueError("travel time must be positive")
    if candidate.energy_cost < 0.0 or candidate.return_energy_cost < 0.0:
        raise ValueError("energy costs must be nonnegative")
    if not 0.0 <= candidate.collision_probability <= 1.0:
        raise ValueError("collision probability must lie in [0, 1]")
    if candidate.trajectory_id < 0 or not candidate.frontier_id:
        raise ValueError("trajectory and frontier identifiers must be valid")


def select_integrity_constrained_exploration(
    forecasts: Sequence[ExplorationForecast],
    prior_covariance: np.ndarray,
    *,
    k_alpha: float,
    margin_reserve: float,
    collision_probability_limit: float,
    energy_remaining: float,
    information_weight: float,
    travel_time_weight: float,
    energy_weight: float,
    minimum_prediction_variance: float = 1.0e-12,
) -> ExplorationSelection:
    """Select maximum task utility after all hard constraints are satisfied."""
    if not forecasts:
        raise ValueError("at least one Frontier trajectory is required")
    if not np.isfinite(
        (
            k_alpha,
            margin_reserve,
            collision_probability_limit,
            energy_remaining,
            information_weight,
            travel_time_weight,
            energy_weight,
            minimum_prediction_variance,
        )
    ).all():
        raise ValueError("P11 parameters must be finite")
    if k_alpha <= 0.0 or margin_reserve < 0.0:
        raise ValueError("integrity parameters are invalid")
    if not 0.0 <= collision_probability_limit <= 1.0:
        raise ValueError("collision probability limit must lie in [0, 1]")
    if energy_remaining < 0.0:
        raise ValueError("remaining energy must be nonnegative")
    if min(information_weight, travel_time_weight, energy_weight) < 0.0:
        raise ValueError("task utility weights must be nonnegative")
    if information_weight <= 0.0:
        raise ValueError("information weight must be positive")

    names = [item.forecast.candidate.name for item in forecasts]
    identifiers = [item.trajectory_id for item in forecasts]
    if len(set(names)) != len(names) or len(set(identifiers)) != len(identifiers):
        raise ValueError("candidate names and trajectory IDs must be unique")

    predictions = []
    for candidate in forecasts:
        _validate_candidate(candidate)
        integrity = evaluate_candidate(
            candidate.forecast,
            prior_covariance,
            k_alpha=k_alpha,
            margin_reserve=margin_reserve,
            baseline_duration=candidate.forecast.candidate.duration,
            lambda_energy=0.0,
            lambda_distance=0.0,
            minimum_prediction_variance=minimum_prediction_variance,
        )
        utility = (
            information_weight * candidate.information_gain
            - travel_time_weight * candidate.travel_time_s
            - energy_weight * candidate.energy_cost
        )
        collision_feasible = (
            candidate.collision_probability <= collision_probability_limit
        )
        energy_feasible = (
            candidate.energy_cost + candidate.return_energy_cost <= energy_remaining
        )
        feasible = bool(integrity.feasible and collision_feasible and energy_feasible)
        predictions.append(
            ExplorationPrediction(
                candidate=candidate,
                integrity=integrity,
                utility=float(utility),
                integrity_feasible=bool(integrity.feasible),
                collision_feasible=bool(collision_feasible),
                energy_feasible=bool(energy_feasible),
                feasible=feasible,
            )
        )

    # The information-only comparator still obeys collision and energy safety;
    # it differs from P11 solely by omitting the integrity hard constraint.
    task_feasible = [
        item for item in predictions if item.collision_feasible and item.energy_feasible
    ]
    unconstrained = (
        max(
            task_feasible,
            key=lambda item: (
                item.utility,
                item.candidate.forecast.candidate.name,
            ),
        )
        if task_feasible
        else None
    )
    hard_feasible = [item for item in predictions if item.feasible]
    selected = (
        max(
            hard_feasible,
            key=lambda item: (
                item.utility,
                item.candidate.forecast.candidate.name,
            ),
        )
        if hard_feasible
        else None
    )
    return ExplorationSelection(
        selected_name=(selected.candidate.forecast.candidate.name if selected else None),
        unconstrained_selected_name=(
            unconstrained.candidate.forecast.candidate.name if unconstrained else None
        ),
        predictions=tuple(predictions),
    )
