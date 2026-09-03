"""Map- and trajectory-derived metrics for integrity-constrained exploration.

The P11 mechanism originally accepted task gain, collision probability, and
energy as upstream metadata.  That interface remains useful for controlled
ablations, but a research flight must not assign those values from candidate
names.  This module provides deterministic, Ground-Truth-free estimators that
can be audited and unit tested independently of ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class TaskGain:
    """Normalized task score and its non-integrity components."""

    gain: float
    progress_efficiency: float
    map_observation_gain: float
    raw_map_observation_gain: float


def _paths(values: Sequence[np.ndarray]) -> tuple[np.ndarray, ...]:
    output = []
    for value in values:
        path = np.asarray(value, dtype=float)
        if path.ndim != 2 or path.shape[1] != 3 or len(path) < 2:
            raise ValueError("candidate paths must be Nx3 with at least two samples")
        if not np.isfinite(path).all():
            raise ValueError("candidate paths must be finite")
        output.append(path)
    if not output:
        raise ValueError("at least one candidate path is required")
    return tuple(output)


def _surfel_weights(
    confidence: np.ndarray,
    quality: np.ndarray,
    last_seen_s: np.ndarray,
    *,
    now_s: float,
    age_time_constant_s: float,
) -> np.ndarray:
    """Score the value of observing an existing map element again.

    Low-confidence and stale surfels are worth revisiting, while geometry
    quality keeps noisy/non-planar support from dominating the task score.
    This is a mapping-observation proxy, not localization information; the
    latter remains in the hard integrity prediction.
    """
    confidence = np.asarray(confidence, dtype=float).reshape(-1)
    quality = np.asarray(quality, dtype=float).reshape(-1)
    last_seen_s = np.asarray(last_seen_s, dtype=float).reshape(-1)
    if not (len(confidence) == len(quality) == len(last_seen_s)):
        raise ValueError("surfel score fields must be aligned")
    if not all(np.isfinite(value).all() for value in (confidence, quality, last_seen_s)):
        raise ValueError("surfel score fields must be finite")
    if np.any((confidence < 0.0) | (confidence > 1.0)):
        raise ValueError("surfel confidence must lie in [0, 1]")
    if np.any((quality < 0.0) | (quality > 1.0)):
        raise ValueError("surfel quality must lie in [0, 1]")
    if not math.isfinite(now_s) or not math.isfinite(age_time_constant_s):
        raise ValueError("map observation time parameters must be finite")
    if age_time_constant_s <= 0.0:
        raise ValueError("map observation age scale must be positive")
    age = np.maximum(now_s - last_seen_s, 0.0)
    stale = 1.0 - np.exp(-age / age_time_constant_s)
    # Retain a small value for high-confidence recent surfaces so view
    # diversity is still observable, without confusing it with uncertainty.
    return quality * (0.10 + 0.45 * (1.0 - confidence) + 0.45 * stale)


def compute_task_gains(
    candidate_paths: Sequence[np.ndarray],
    surfel_positions: np.ndarray,
    static_confidence: np.ndarray,
    geometry_quality: np.ndarray,
    last_seen_s: np.ndarray,
    *,
    now_s: float,
    visibility_radius_m: float,
    age_time_constant_s: float,
    progress_weight: float = 0.85,
) -> tuple[TaskGain, ...]:
    """Estimate normalized task gain from progress and map observation value.

    Progress efficiency prevents a needless detour from winning merely by
    being longer.  The second term measures the unique, confidence/age-weighted
    surfels observable from a path.  Scores are normalized only within the
    immutable candidate batch, which makes them independent of candidate
    names and scenario-specific constants.
    """
    paths = _paths(candidate_paths)
    surfels = np.asarray(surfel_positions, dtype=float)
    if surfels.ndim != 2 or surfels.shape[1] != 3 or len(surfels) == 0:
        raise ValueError("surfel positions must be non-empty Nx3")
    if not np.isfinite(surfels).all():
        raise ValueError("surfel positions must be finite")
    if not math.isfinite(visibility_radius_m) or visibility_radius_m <= 0.0:
        raise ValueError("visibility radius must be positive and finite")
    if not math.isfinite(progress_weight) or not 0.0 <= progress_weight <= 1.0:
        raise ValueError("progress weight must lie in [0, 1]")
    weights = _surfel_weights(
        static_confidence,
        geometry_quality,
        last_seen_s,
        now_s=now_s,
        age_time_constant_s=age_time_constant_s,
    )
    if len(weights) != len(surfels):
        raise ValueError("surfel positions and weights must be aligned")

    raw_observation = []
    progress_efficiency = []
    radius2 = visibility_radius_m * visibility_radius_m
    for path in paths:
        segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        length = float(np.sum(segment_lengths))
        displacement = float(np.linalg.norm(path[-1] - path[0]))
        progress_efficiency.append(
            float(np.clip(displacement / max(length, 1.0e-12), 0.0, 1.0))
        )
        # Work in bounded chunks to avoid allocating samples x surfels for a
        # large temporal map.  Visibility is intentionally conservative: no
        # Ground Truth, semantic label, or future observation is consulted.
        visible = np.zeros(len(surfels), dtype=bool)
        for sample in path:
            delta = surfels - sample
            visible |= np.einsum("ij,ij->i", delta, delta) <= radius2
        raw_observation.append(float(np.sum(weights[visible])))

    raw = np.asarray(raw_observation, dtype=float)
    span = float(np.max(raw) - np.min(raw))
    if span <= 1.0e-12:
        normalized = np.full(len(raw), 0.5, dtype=float)
    else:
        normalized = (raw - float(np.min(raw))) / span
    progress = np.asarray(progress_efficiency, dtype=float)
    gains = progress_weight * progress + (1.0 - progress_weight) * normalized
    gains = np.clip(gains, 0.0, 1.0)
    return tuple(
        TaskGain(
            gain=float(gain),
            progress_efficiency=float(efficiency),
            map_observation_gain=float(observation),
            raw_map_observation_gain=float(raw_value),
        )
        for gain, efficiency, observation, raw_value in zip(
            gains, progress, normalized, raw
        )
    )


def pointwise_collision_probability(
    alert_limits_m: np.ndarray,
    *,
    tracking_reserve_m: float,
    tracking_sigma_multiplier: float = 3.0,
) -> float:
    """Return the worst pointwise one-sided tracking collision probability.

    ``alert_limits_m`` already subtracts the tracking reserve.  Adding it back
    yields the physical room available to the stochastic tracking residual;
    the reserve is interpreted as ``tracking_sigma_multiplier`` standard
    deviations.  The output is a transparent Gaussian proxy and is reported
    as such; it is not presented as a certified collision probability.
    """
    limits = np.asarray(alert_limits_m, dtype=float).reshape(-1)
    values = (tracking_reserve_m, tracking_sigma_multiplier)
    if len(limits) == 0 or not np.isfinite(limits).all():
        raise ValueError("alert-limit profile must be non-empty and finite")
    if not np.isfinite(values).all() or tracking_reserve_m <= 0.0:
        raise ValueError("tracking uncertainty parameters must be positive and finite")
    if tracking_sigma_multiplier <= 0.0:
        raise ValueError("tracking sigma multiplier must be positive")
    sigma = tracking_reserve_m / tracking_sigma_multiplier
    physical_room = limits + tracking_reserve_m
    probabilities = [
        0.5 * math.erfc(float(value) / (math.sqrt(2.0) * sigma))
        for value in physical_room
    ]
    return float(np.clip(max(probabilities), 0.0, 1.0))


def motion_energy_proxy(
    path: np.ndarray,
    duration_s: float,
    *,
    drag_weight: float = 0.15,
    climb_weight: float = 1.5,
    acceleration_weight: float = 0.05,
) -> float:
    """Trajectory-dependent energy proxy for ranking and reserve checks."""
    trajectory = _paths((path,))[0]
    values = np.asarray(
        (duration_s, drag_weight, climb_weight, acceleration_weight), dtype=float
    )
    if not np.isfinite(values).all() or duration_s <= 0.0:
        raise ValueError("energy parameters must be finite and duration positive")
    if min(drag_weight, climb_weight, acceleration_weight) < 0.0:
        raise ValueError("energy weights must be nonnegative")
    delta = np.diff(trajectory, axis=0)
    segment_length = np.linalg.norm(delta, axis=1)
    dt = duration_s / float(len(segment_length))
    velocity = delta / dt
    drag = float(np.sum(np.einsum("ij,ij->i", velocity, velocity)) * dt)
    climb = float(np.sum(np.maximum(delta[:, 2], 0.0)))
    acceleration = 0.0
    if len(velocity) > 1:
        acceleration_vectors = np.diff(velocity, axis=0) / dt
        acceleration = float(
            np.sum(np.einsum("ij,ij->i", acceleration_vectors, acceleration_vectors))
            * dt
        )
    return float(
        np.sum(segment_length)
        + drag_weight * drag
        + climb_weight * climb
        + acceleration_weight * acceleration
    )


def return_energy_proxy(endpoint: np.ndarray, home: np.ndarray) -> float:
    """Conservative straight-line return reserve with an ascent penalty."""
    endpoint = np.asarray(endpoint, dtype=float).reshape(-1)
    home = np.asarray(home, dtype=float).reshape(-1)
    if endpoint.shape != (3,) or home.shape != (3,):
        raise ValueError("endpoint and home must be 3-vectors")
    if not np.isfinite(endpoint).all() or not np.isfinite(home).all():
        raise ValueError("endpoint and home must be finite")
    delta = home - endpoint
    return float(np.linalg.norm(delta) + 1.5 * max(float(delta[2]), 0.0))
