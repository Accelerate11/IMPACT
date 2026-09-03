"""Pure P12 dynamic-path collision and passage reopening logic."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def resample_polyline(path_points: np.ndarray, maximum_points: int) -> np.ndarray:
    """Arc-length resample a dense path while preserving both endpoints."""
    path = np.asarray(path_points, dtype=np.float64)
    if path.ndim != 2 or path.shape[1] != 3 or len(path) < 2:
        raise ValueError("path_points must have shape (M,3), M >= 2")
    if not np.isfinite(path).all() or maximum_points < 2:
        raise ValueError("invalid polyline resampling input")
    keep = np.concatenate(
        ([True], np.linalg.norm(np.diff(path, axis=0), axis=1) > 1.0e-9)
    )
    path = path[keep]
    if len(path) < 2 or len(path) <= maximum_points:
        return path
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    targets = np.linspace(0.0, cumulative[-1], maximum_points)
    return np.column_stack(
        [np.interp(targets, cumulative, path[:, axis]) for axis in range(3)]
    )


def polyline_proximity(
    query_points: np.ndarray,
    path_points: np.ndarray,
    *,
    lookahead_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return each point's minimum distance and along-path coordinate.

    Both the temporal map and the flight guard use this primitive, so dynamic
    promotion and braking can follow the commanded 3-D trajectory instead of
    assuming that every candidate remains on the world-X axis.
    """
    points = np.asarray(query_points, dtype=np.float64)
    path = np.asarray(path_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("query_points must have shape (N,3)")
    if path.ndim != 2 or path.shape[1] != 3 or len(path) < 2:
        raise ValueError("path_points must have shape (M,3), M >= 2")
    if not np.isfinite(path).all() or not np.isfinite(points).all():
        raise ValueError("path query geometry must be finite")
    if not math.isfinite(lookahead_m) or lookahead_m <= 0.0:
        raise ValueError("lookahead must be positive and finite")
    if len(points) == 0:
        empty = np.empty(0, dtype=np.float64)
        return empty, empty

    segments = np.diff(path, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    nonzero = lengths > 1.0e-9
    starts = path[:-1][nonzero]
    segments = segments[nonzero]
    lengths = lengths[nonzero]
    if not len(lengths):
        return (
            np.full(len(points), math.inf, dtype=np.float64),
            np.full(len(points), math.inf, dtype=np.float64),
        )
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)[:-1]))
    within = cumulative < lookahead_m
    starts = starts[within]
    segments = segments[within]
    lengths = lengths[within]
    cumulative = cumulative[within]
    used_lengths = np.minimum(lengths, lookahead_m - cumulative)
    units = segments / lengths[:, None]

    # Vectorize across path segments and bound only the query-point dimension.
    # The earlier segment-by-segment Python loop allocated one Nx3 temporary
    # per segment and inflated the map latency tail.  This form keeps the same
    # exact point-to-segment geometry while making runtime scale in compiled
    # NumPy kernels; chunking caps peak memory for a dense LiDAR frame.
    best_distance = np.empty(len(points), dtype=np.float64)
    best_along = np.empty(len(points), dtype=np.float64)
    chunk_size = 4096
    for first in range(0, len(points), chunk_size):
        last = min(first + chunk_size, len(points))
        relative = points[first:last, None, :] - starts[None, :, :]
        projection = np.einsum("nsi,si->ns", relative, units)
        projection = np.clip(projection, 0.0, used_lengths[None, :])
        residual = relative - projection[:, :, None] * units[None, :, :]
        distance2 = np.einsum("nsi,nsi->ns", residual, residual)
        winner = np.argmin(distance2, axis=1)
        rows = np.arange(last - first)
        best_distance[first:last] = np.sqrt(distance2[rows, winner])
        best_along[first:last] = (
            cumulative[winner] + projection[rows, winner]
        )
    return best_distance, best_along


def polyline_obstruction(
    dynamic_points: np.ndarray,
    path_points: np.ndarray,
    *,
    clearance_radius_m: float,
    lookahead_m: float,
) -> tuple[bool, float]:
    """Test dynamic points against an actual commanded path prefix."""
    if not math.isfinite(clearance_radius_m) or clearance_radius_m <= 0.0:
        raise ValueError("clearance radius must be positive and finite")
    distance, along = polyline_proximity(
        dynamic_points, path_points, lookahead_m=lookahead_m
    )
    mask = (
        (along >= 0.20)
        & (along <= lookahead_m)
        & (distance <= clearance_radius_m)
    )
    if not np.any(mask):
        return False, math.inf
    return True, float(np.min(along[mask]))


def supported_polyline_obstruction(
    dynamic_points: np.ndarray,
    path_points: np.ndarray,
    *,
    clearance_radius_m: float,
    lookahead_m: float,
    minimum_support_points: int = 1,
    support_radius_m: float = 0.45,
) -> tuple[bool, float, int]:
    """Require spatially coherent support before declaring an obstruction.

    A temporal voxel can be real while still being insufficient evidence for
    an object: one repeatedly refreshed registration outlier otherwise keeps a
    fail-closed TTL gate latched forever.  This gate preserves the geometric
    path test and additionally requires a connected metric component.  The
    default of one exactly preserves the P1--P14 behavior.
    """
    points = np.asarray(dynamic_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("dynamic_points must have shape (N,3)")
    if (
        isinstance(minimum_support_points, bool)
        or int(minimum_support_points) != minimum_support_points
        or minimum_support_points < 1
        or not math.isfinite(support_radius_m)
        or support_radius_m <= 0.0
    ):
        raise ValueError("invalid obstruction support requirement")
    if not math.isfinite(clearance_radius_m) or clearance_radius_m <= 0.0:
        raise ValueError("clearance radius must be positive and finite")
    distance, along = polyline_proximity(
        points, path_points, lookahead_m=lookahead_m
    )
    mask = (
        (along >= 0.20)
        & (along <= lookahead_m)
        & (distance <= clearance_radius_m)
    )
    indices = np.flatnonzero(mask)
    if len(indices) == 0:
        return False, math.inf, 0

    candidates = points[indices]
    candidate_along = along[indices]
    cell_keys = np.floor(candidates / support_radius_m).astype(np.int64)
    buckets: dict[tuple[int, int, int], list[int]] = {}
    for index, key in enumerate(cell_keys):
        buckets.setdefault(tuple(int(value) for value in key), []).append(index)
    neighbor_offsets = tuple(
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
    )
    visited = np.zeros(len(candidates), dtype=bool)
    radius2 = support_radius_m * support_radius_m
    maximum_component = 0
    nearest_supported = math.inf
    for seed in range(len(candidates)):
        if visited[seed]:
            continue
        visited[seed] = True
        stack = [seed]
        component: list[int] = []
        while stack:
            current = stack.pop()
            component.append(current)
            base = cell_keys[current]
            for dx, dy, dz in neighbor_offsets:
                key = (int(base[0] + dx), int(base[1] + dy), int(base[2] + dz))
                for neighbor in buckets.get(key, ()):  # local metric search only
                    if visited[neighbor]:
                        continue
                    delta = candidates[neighbor] - candidates[current]
                    if float(delta @ delta) <= radius2 + 1.0e-12:
                        visited[neighbor] = True
                        stack.append(neighbor)
        size = len(component)
        maximum_component = max(maximum_component, size)
        if size >= minimum_support_points:
            nearest_supported = min(
                nearest_supported, float(np.min(candidate_along[component]))
            )
    return (
        math.isfinite(nearest_supported),
        nearest_supported,
        maximum_component,
    )


def path_obstruction(
    dynamic_points: np.ndarray,
    start_xyz: np.ndarray,
    goal_xyz: np.ndarray,
    *,
    clearance_radius_m: float,
    lookahead_m: float,
) -> tuple[bool, float]:
    points = np.asarray(dynamic_points, dtype=np.float64)
    start = np.asarray(start_xyz, dtype=np.float64).reshape(3)
    goal = np.asarray(goal_xyz, dtype=np.float64).reshape(3)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("dynamic_points must have shape (N,3)")
    if not np.isfinite((start, goal)).all() or clearance_radius_m <= 0.0 or lookahead_m <= 0.0:
        raise ValueError("invalid path obstruction parameter")
    direction = goal - start
    length = float(np.linalg.norm(direction))
    if length <= 1.0e-9:
        return False, math.inf
    return polyline_obstruction(
        points,
        np.stack((start, goal)),
        clearance_radius_m=clearance_radius_m,
        lookahead_m=min(length, lookahead_m),
    )


@dataclass(frozen=True)
class PassageDecision:
    state: str
    brake: bool
    obstacle_confirmed: bool
    passage_reopened: bool


class DynamicPassageGate:
    """Fail-closed state machine with debounced reopening after TTL clearance."""

    def __init__(self, clear_confirmation_s: float = 1.0) -> None:
        if not math.isfinite(clear_confirmation_s) or clear_confirmation_s < 0.0:
            raise ValueError("invalid clear confirmation time")
        self.clear_confirmation_s = float(clear_confirmation_s)
        self._blocked_once = False
        self._clear_since_s: float | None = None
        self._reopened = False

    def update(self, blocked: bool, now_s: float) -> PassageDecision:
        if not math.isfinite(now_s) or now_s < 0.0:
            raise ValueError("invalid passage timestamp")
        if blocked:
            self._blocked_once = True
            self._clear_since_s = None
            self._reopened = False
            return PassageDecision("BRAKE_DYNAMIC", True, True, False)
        if not self._blocked_once:
            return PassageDecision("CLEAR_INITIAL", False, False, False)
        if self._clear_since_s is None:
            self._clear_since_s = float(now_s)
        if now_s - self._clear_since_s < self.clear_confirmation_s:
            return PassageDecision("VERIFY_TTL_CLEAR", True, True, False)
        first_reopen = not self._reopened
        self._reopened = True
        return PassageDecision("PASSAGE_REOPENED", False, True, first_reopen)

