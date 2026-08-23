"""Pure P8 trajectory sampling and static-obstacle Alert Limit equations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AlertLimitResult:
    critical_sample: np.ndarray
    nearest_obstacle: np.ndarray
    obstacle_direction: np.ndarray
    geometric_clearance: float
    latency_reserve: float
    alert_limit: float
    trajectory_sample_count: int
    obstacle_point_count: int


def sample_bspline(
    control_points: np.ndarray,
    knots: np.ndarray,
    degree: int,
    interval_s: float,
    minimum_parameter_s: float | None = None,
) -> np.ndarray:
    """Evaluate a clamped/non-clamped B-spline with de Boor's algorithm."""
    points = np.asarray(control_points, dtype=float)
    knot_vector = np.asarray(knots, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) <= degree:
        raise ValueError("invalid B-spline control points")
    if degree < 1 or len(knot_vector) != len(points) + degree + 1:
        raise ValueError("invalid B-spline degree or knot vector")
    if interval_s <= 0.0 or not np.isfinite(points).all() or not np.isfinite(knot_vector).all():
        raise ValueError("non-finite B-spline or invalid sampling interval")
    if np.any(np.diff(knot_vector) < 0.0):
        raise ValueError("B-spline knots must be nondecreasing")

    last_control = len(points) - 1
    start = float(knot_vector[degree])
    end = float(knot_vector[last_control + 1])
    if minimum_parameter_s is not None:
        start = max(start, float(minimum_parameter_s))
    if end <= start:
        raise ValueError("empty B-spline domain")
    sample_times = np.arange(start, end, interval_s, dtype=float)
    sample_times = np.append(sample_times, end)
    output = []
    for value in sample_times:
        if value >= end:
            span = last_control
        else:
            span = int(np.searchsorted(knot_vector, value, side="right") - 1)
            span = max(degree, min(span, last_control))
        work = [points[span - degree + index].copy() for index in range(degree + 1)]
        for recursion in range(1, degree + 1):
            for index in range(degree, recursion - 1, -1):
                knot_index = span - degree + index
                denominator = (
                    knot_vector[knot_index + degree - recursion + 1]
                    - knot_vector[knot_index]
                )
                alpha = 0.0 if denominator <= 1.0e-15 else (value - knot_vector[knot_index]) / denominator
                work[index] = (1.0 - alpha) * work[index - 1] + alpha * work[index]
        output.append(work[degree])
    sampled = np.asarray(output, dtype=float)
    if not np.isfinite(sampled).all():
        raise ValueError("B-spline evaluation produced non-finite samples")
    return sampled


def compute_alert_limit(
    trajectory_samples: np.ndarray,
    obstacle_points: np.ndarray,
    *,
    speed_mps: float,
    latency_p99_s: float,
    maximum_acceleration_mps2: float,
    body_radius_m: float,
    base_reserve_m: float,
    tracking_reserve_m: float,
    dynamic_reserve_m: float = 0.0,
) -> AlertLimitResult:
    samples = np.asarray(trajectory_samples, dtype=float)
    obstacles = np.asarray(obstacle_points, dtype=float)
    if samples.ndim != 2 or samples.shape[1] != 3 or len(samples) == 0:
        raise ValueError("trajectory samples must be non-empty Nx3")
    if obstacles.ndim != 2 or obstacles.shape[1] != 3 or len(obstacles) == 0:
        raise ValueError("obstacle points must be non-empty Nx3")
    values = np.array(
        (
            speed_mps,
            latency_p99_s,
            maximum_acceleration_mps2,
            body_radius_m,
            base_reserve_m,
            tracking_reserve_m,
            dynamic_reserve_m,
        ),
        dtype=float,
    )
    if not np.isfinite(samples).all() or not np.isfinite(obstacles).all() or not np.isfinite(values).all():
        raise ValueError("P8 inputs must be finite")
    if np.any(values < 0.0):
        raise ValueError("P8 reserve inputs must be nonnegative")

    distance_squared = (
        np.sum(samples * samples, axis=1)[:, None]
        + np.sum(obstacles * obstacles, axis=1)[None, :]
        - 2.0 * samples @ obstacles.T
    )
    np.maximum(distance_squared, 0.0, out=distance_squared)
    nearest_indices = np.argmin(distance_squared, axis=1)
    clearances = np.sqrt(distance_squared[np.arange(len(samples)), nearest_indices])
    critical_index = int(np.argmin(clearances))
    clearance = float(clearances[critical_index])
    if clearance <= 1.0e-9:
        direction = np.zeros(3, dtype=float)
    else:
        direction = (obstacles[nearest_indices[critical_index]] - samples[critical_index]) / clearance
    latency_reserve = float(
        speed_mps * latency_p99_s
        + 0.5 * maximum_acceleration_mps2 * latency_p99_s * latency_p99_s
    )
    alert_limit = float(
        clearance
        - body_radius_m
        - base_reserve_m
        - tracking_reserve_m
        - dynamic_reserve_m
        - latency_reserve
    )
    return AlertLimitResult(
        critical_sample=samples[critical_index].copy(),
        nearest_obstacle=obstacles[nearest_indices[critical_index]].copy(),
        obstacle_direction=direction,
        geometric_clearance=clearance,
        latency_reserve=latency_reserve,
        alert_limit=alert_limit,
        trajectory_sample_count=len(samples),
        obstacle_point_count=len(obstacles),
    )
