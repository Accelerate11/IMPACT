"""Pure P13 latency statistics and safety-envelope calculations."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class LatencyStatistics:
    count: int
    p50_s: float
    p95_s: float
    p99_s: float
    maximum_s: float


@dataclass(frozen=True)
class LatencySafetyEnvelope:
    latency_s: float
    latency_radius_m: float
    alert_limit_m: float
    integrity_margin_m: float
    speed_limit_mps: float


def nearest_rank(values: Iterable[float], percentile: float) -> float:
    """Return a conservative nearest-rank percentile without interpolation."""
    array = np.asarray(tuple(values), dtype=np.float64)
    array = array[np.isfinite(array) & (array >= 0.0)]
    if len(array) == 0:
        return math.inf
    if not 0.0 < percentile <= 1.0:
        raise ValueError("percentile must be in (0, 1]")
    ordered = np.sort(array)
    index = max(0, int(math.ceil(percentile * len(ordered))) - 1)
    return float(ordered[index])


def summarize_latencies(values: Iterable[float]) -> LatencyStatistics:
    samples = tuple(values)
    finite = tuple(value for value in samples if math.isfinite(value) and value >= 0.0)
    return LatencyStatistics(
        count=len(finite),
        p50_s=nearest_rank(finite, 0.50),
        p95_s=nearest_rank(finite, 0.95),
        p99_s=nearest_rank(finite, 0.99),
        maximum_s=nearest_rank(finite, 1.00),
    )


def latency_radius(speed_mps: float, latency_s: float, maximum_acceleration_mps2: float) -> float:
    if speed_mps < 0.0 or latency_s < 0.0 or maximum_acceleration_mps2 < 0.0:
        raise ValueError("speed, latency and acceleration must be non-negative")
    return speed_mps * latency_s + 0.5 * maximum_acceleration_mps2 * latency_s**2


def safety_envelope(
    *,
    latency_s: float,
    geometric_clearance_m: float,
    fixed_buffer_m: float,
    protection_level_m: float,
    required_margin_m: float,
    maximum_speed_mps: float,
    maximum_acceleration_mps2: float,
) -> LatencySafetyEnvelope:
    """Solve the largest speed whose latency-aware integrity margin is acceptable."""
    if latency_s <= 0.0:
        speed_limit = maximum_speed_mps
    else:
        linear_budget = (
            geometric_clearance_m
            - fixed_buffer_m
            - protection_level_m
            - required_margin_m
            - 0.5 * maximum_acceleration_mps2 * latency_s**2
        )
        speed_limit = min(maximum_speed_mps, max(0.0, linear_budget / latency_s))
    radius = latency_radius(speed_limit, latency_s, maximum_acceleration_mps2)
    alert_limit = geometric_clearance_m - fixed_buffer_m - radius
    margin = alert_limit - protection_level_m
    return LatencySafetyEnvelope(
        latency_s=latency_s,
        latency_radius_m=radius,
        alert_limit_m=alert_limit,
        integrity_margin_m=margin,
        speed_limit_mps=speed_limit,
    )
