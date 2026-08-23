"""Pure P10 minimum-excitation active-perception kernel.

The module deliberately has no ROS dependency.  It predicts candidate-specific
information updates, reuses the P9 directional margin equation as a hard
feasibility constraint, and only then minimizes recovery cost.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from .integrity_margin import compute_directional_protection_levels


@dataclass(frozen=True)
class RecoveryCandidate:
    name: str
    positions: np.ndarray
    yaw: np.ndarray
    duration: float
    extra_path_length: float
    extra_energy: float


@dataclass(frozen=True)
class CandidateForecast:
    candidate: RecoveryCandidate
    alert_limits: np.ndarray
    obstacle_directions: np.ndarray
    information_profile: np.ndarray


@dataclass(frozen=True)
class CandidatePrediction:
    candidate: RecoveryCandidate
    covariance_profile: np.ndarray
    protection_levels: np.ndarray
    margins: np.ndarray
    minimum_margin: float
    critical_index: int
    feasible: bool
    cost: float
    information_trace: float


@dataclass(frozen=True)
class RecoverySelection:
    baseline_insufficient: bool
    recovery_found: bool
    selected_name: str | None
    predictions: tuple[CandidatePrediction, ...]


def _positions(values: np.ndarray, *, name: str = "positions") -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3 or len(array) < 2:
        raise ValueError(f"{name} must be Nx3 with at least two samples")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must be finite")
    return array


def _path_length(positions: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


def _motion_energy(positions: np.ndarray, duration: float) -> float:
    """Deterministic velocity-squared proxy; hardware energy remains unverified."""
    if duration <= 0.0 or not np.isfinite(duration):
        raise ValueError("duration must be positive and finite")
    segment_dt = duration / float(len(positions) - 1)
    segment_lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    return float(np.sum(segment_lengths * segment_lengths / segment_dt))


def generate_discrete_candidates(
    baseline_positions: np.ndarray,
    *,
    baseline_duration: float,
    lateral_offset: float = 0.4,
    vertical_offset: float = 0.25,
    slow_scale: float = 0.5,
    hover_time: float = 1.0,
    previous_high_quality_pose: np.ndarray | None = None,
    hover_power_proxy: float = 1.0,
    include_yaw_only_comparator: bool = False,
) -> tuple[RecoveryCandidate, ...]:
    """Generate the finite P10 v1 action set without changing endpoints."""
    baseline = _positions(baseline_positions, name="baseline_positions")
    scalars = (baseline_duration, lateral_offset, vertical_offset, slow_scale, hover_time)
    if not np.isfinite(scalars).all():
        raise ValueError("candidate parameters must be finite")
    if baseline_duration <= 0.0 or lateral_offset <= 0.0 or vertical_offset <= 0.0:
        raise ValueError("duration and offsets must be positive")
    if not 0.0 < slow_scale < 1.0:
        raise ValueError("slow_scale must lie in (0, 1)")
    if hover_time <= 0.0 or hover_power_proxy < 0.0:
        raise ValueError("hover parameters are invalid")

    count = len(baseline)
    phase = np.linspace(0.0, 1.0, count)
    window = np.sin(np.pi * phase)
    tangents = np.gradient(baseline, axis=0)
    horizontal = tangents.copy()
    horizontal[:, 2] = 0.0
    norms = np.linalg.norm(horizontal, axis=1)
    if np.any(norms < 1.0e-9):
        raise ValueError("baseline needs a horizontal tangent at every sample")
    horizontal /= norms[:, None]
    left = np.column_stack((-horizontal[:, 1], horizontal[:, 0], np.zeros(count)))

    base_length = _path_length(baseline)
    base_energy = _motion_energy(baseline, baseline_duration)
    zero_yaw = np.zeros(count, dtype=float)

    def make(name: str, points: np.ndarray, duration: float, yaw: np.ndarray | None = None,
             energy_bias: float = 0.0) -> RecoveryCandidate:
        path_length = _path_length(points)
        energy = _motion_energy(points, duration)
        return RecoveryCandidate(
            name=name,
            positions=points,
            yaw=zero_yaw.copy() if yaw is None else np.asarray(yaw, dtype=float),
            duration=float(duration),
            extra_path_length=max(path_length - base_length, 0.0),
            extra_energy=max(energy - base_energy, 0.0) + float(energy_bias),
        )

    candidates = [make("baseline", baseline.copy(), baseline_duration)]
    candidates.append(make("left_lateral", baseline + lateral_offset * window[:, None] * left,
                           baseline_duration))
    candidates.append(make("right_lateral", baseline - lateral_offset * window[:, None] * left,
                           baseline_duration))
    z_offset = np.zeros_like(baseline)
    z_offset[:, 2] = vertical_offset * window
    candidates.append(make("up_offset", baseline + z_offset, baseline_duration))
    candidates.append(make("down_offset", baseline - z_offset, baseline_duration))
    candidates.append(make("slow_trajectory", baseline.copy(), baseline_duration / slow_scale))
    candidates.append(make("short_hover", baseline.copy(), baseline_duration + hover_time,
                           energy_bias=hover_power_proxy * hover_time))

    if previous_high_quality_pose is None:
        previous = baseline[0] - 0.5 * horizontal[0]
    else:
        previous = np.asarray(previous_high_quality_pose, dtype=float).reshape(-1)
        if previous.shape != (3,) or not np.isfinite(previous).all():
            raise ValueError("previous_high_quality_pose must be a finite 3-vector")
    backtrack = np.vstack((baseline[0], previous, baseline[1:]))
    backtrack_duration = baseline_duration + np.linalg.norm(previous - baseline[0])
    candidates.append(make("backtrack", backtrack, backtrack_duration))

    if include_yaw_only_comparator:
        yaw = 0.5 * np.pi * window
        candidates.append(make("yaw_only", baseline.copy(), baseline_duration, yaw=yaw,
                               energy_bias=float(np.trapz(np.gradient(yaw) ** 2, phase))))
    return tuple(candidates)


def build_information_profile(
    sample_positions: np.ndarray,
    surfel_positions: np.ndarray,
    surfel_normals: np.ndarray,
    static_confidence: np.ndarray,
    geometry_quality: np.ndarray,
    last_seen: np.ndarray,
    *,
    now: float,
    visibility_radius: float,
    age_time_constant: float,
    information_scale: float,
) -> np.ndarray:
    """Build Λ-hat from nearby, recent, static local-map surfels.

    `information_scale` carries the inverse-variance unit needed when adding
    normal outer products to the prior covariance information matrix.
    """
    samples = _positions(sample_positions, name="sample_positions")
    positions = np.asarray(surfel_positions, dtype=float)
    normals = np.asarray(surfel_normals, dtype=float)
    static = np.asarray(static_confidence, dtype=float).reshape(-1)
    quality = np.asarray(geometry_quality, dtype=float).reshape(-1)
    seen = np.asarray(last_seen, dtype=float).reshape(-1)
    if positions.ndim != 2 or positions.shape[1] != 3 or positions.shape != normals.shape:
        raise ValueError("surfel positions and normals must be aligned Nx3")
    count = len(positions)
    if count == 0 or any(len(values) != count for values in (static, quality, seen)):
        raise ValueError("surfel fields must be non-empty and aligned")
    if not all(np.isfinite(values).all() for values in (positions, normals, static, quality, seen)):
        raise ValueError("information-map inputs must be finite")
    normal_norms = np.linalg.norm(normals, axis=1)
    if np.any(np.abs(normal_norms - 1.0) > 1.0e-6):
        raise ValueError("surfel normals must be unit vectors")
    if np.any((static < 0.0) | (static > 1.0)) or np.any((quality < 0.0) | (quality > 1.0)):
        raise ValueError("surfel confidence and quality must lie in [0, 1]")
    if not np.isfinite((now, visibility_radius, age_time_constant, information_scale)).all():
        raise ValueError("information-profile parameters must be finite")
    if visibility_radius <= 0.0 or age_time_constant <= 0.0 or information_scale <= 0.0:
        raise ValueError("information-profile scales must be positive")

    age = np.maximum(float(now) - seen, 0.0)
    base_weights = static * quality * np.exp(-age / age_time_constant) * information_scale
    profile = np.zeros((len(samples), 3, 3), dtype=float)
    radius2 = visibility_radius * visibility_radius
    for index, sample in enumerate(samples):
        visible = np.sum((positions - sample) ** 2, axis=1) <= radius2
        weights = base_weights * visible
        profile[index] = np.einsum("n,ni,nj->ij", weights, normals, normals)
    return profile


def predict_covariance_profile(
    prior_covariance: np.ndarray,
    information_profile: np.ndarray,
    *,
    eigenvalue_floor: float = 1.0e-12,
) -> np.ndarray:
    """Sequentially apply P(k+1)=(P(k)^-1+Λ-hat(k))^-1."""
    prior = np.asarray(prior_covariance, dtype=float)
    information = np.asarray(information_profile, dtype=float)
    if prior.shape != (3, 3) or information.ndim != 3 or information.shape[1:] != (3, 3):
        raise ValueError("prior must be 3x3 and information profile must be Nx3x3")
    if len(information) == 0 or not np.isfinite(prior).all() or not np.isfinite(information).all():
        raise ValueError("covariance prediction inputs must be non-empty and finite")
    if eigenvalue_floor <= 0.0 or not np.isfinite(eigenvalue_floor):
        raise ValueError("eigenvalue_floor must be positive and finite")
    prior = 0.5 * (prior + prior.T)
    if np.linalg.eigvalsh(prior)[0] <= 0.0:
        raise ValueError("prior covariance must be positive definite")

    current_information = np.linalg.inv(prior)
    profile = []
    for increment in information:
        increment = 0.5 * (increment + increment.T)
        if np.linalg.eigvalsh(increment)[0] < -1.0e-10:
            raise ValueError("predicted information increments must be positive semidefinite")
        current_information += increment
        covariance = np.linalg.inv(current_information)
        values, vectors = np.linalg.eigh(0.5 * (covariance + covariance.T))
        covariance = vectors @ np.diag(np.maximum(values, eigenvalue_floor)) @ vectors.T
        profile.append(covariance)
    return np.asarray(profile)


def evaluate_candidate(
    forecast: CandidateForecast,
    prior_covariance: np.ndarray,
    *,
    k_alpha: float,
    margin_reserve: float,
    baseline_duration: float,
    lambda_energy: float,
    lambda_distance: float,
    minimum_prediction_variance: float = 1.0e-12,
) -> CandidatePrediction:
    candidate = forecast.candidate
    limits = np.asarray(forecast.alert_limits, dtype=float).reshape(-1)
    directions = np.asarray(forecast.obstacle_directions, dtype=float)
    information = np.asarray(forecast.information_profile, dtype=float)
    count = len(candidate.positions)
    if len(limits) != count or directions.shape != (count, 3) or information.shape != (count, 3, 3):
        raise ValueError("candidate AL, direction, and information profiles must align")
    if not np.isfinite(limits).all():
        raise ValueError("candidate alert limits must be finite")
    if margin_reserve < 0.0 or baseline_duration <= 0.0:
        raise ValueError("margin reserve and baseline duration are invalid")
    if lambda_energy < 0.0 or lambda_distance < 0.0:
        raise ValueError("cost weights must be nonnegative")

    covariances = predict_covariance_profile(
        prior_covariance, information, eigenvalue_floor=minimum_prediction_variance
    )
    levels = np.asarray([
        compute_directional_protection_levels(direction[None, :], covariance, k_alpha)[0]
        for direction, covariance in zip(directions, covariances)
    ])
    margins = limits - levels
    critical = int(np.argmin(margins))
    minimum = float(margins[critical])
    cost = (
        max(candidate.duration - baseline_duration, 0.0)
        + lambda_energy * candidate.extra_energy
        + lambda_distance * candidate.extra_path_length
    )
    return CandidatePrediction(
        candidate=candidate,
        covariance_profile=covariances,
        protection_levels=levels,
        margins=margins,
        minimum_margin=minimum,
        critical_index=critical,
        feasible=minimum >= margin_reserve,
        cost=float(cost),
        information_trace=float(np.trace(information, axis1=1, axis2=2).sum()),
    )


def select_minimum_excitation(
    forecasts: Sequence[CandidateForecast],
    prior_covariance: np.ndarray,
    *,
    k_alpha: float,
    margin_reserve: float,
    lambda_energy: float,
    lambda_distance: float,
    minimum_prediction_variance: float = 1.0e-12,
) -> RecoverySelection:
    """Choose minimum cost among hard-feasible candidates; fail closed otherwise."""
    if not forecasts:
        raise ValueError("at least one candidate forecast is required")
    names = [forecast.candidate.name for forecast in forecasts]
    if len(set(names)) != len(names) or "baseline" not in names:
        raise ValueError("candidate names must be unique and include baseline")
    baseline_forecast = forecasts[names.index("baseline")]
    baseline_duration = baseline_forecast.candidate.duration
    predictions = tuple(
        evaluate_candidate(
            forecast,
            prior_covariance,
            k_alpha=k_alpha,
            margin_reserve=margin_reserve,
            baseline_duration=baseline_duration,
            lambda_energy=lambda_energy,
            lambda_distance=lambda_distance,
            minimum_prediction_variance=minimum_prediction_variance,
        )
        for forecast in forecasts
    )
    baseline = predictions[names.index("baseline")]
    if baseline.feasible:
        return RecoverySelection(False, True, "baseline", predictions)

    feasible = [prediction for prediction in predictions if prediction.feasible]
    if not feasible:
        return RecoverySelection(True, False, None, predictions)
    selected = min(feasible, key=lambda item: (item.cost, item.candidate.name))
    return RecoverySelection(True, True, selected.candidate.name, predictions)
