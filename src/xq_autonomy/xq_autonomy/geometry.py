from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Return inclusive integer grid cells on a line."""
    cells: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return cells


def path_length(path: Sequence[Tuple[float, float]]) -> float:
    if len(path) < 2:
        return 0.0
    return float(
        sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
    )


def align_se2(
    estimate_xy: Iterable[Iterable[float]],
    truth_xy: Iterable[Iterable[float]],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rigidly align estimate to truth with one global SE(2) transform.

    No scale is fitted.  Returns aligned estimate, 2x2 rotation and translation.
    """
    estimate = np.asarray(estimate_xy, dtype=float)
    truth = np.asarray(truth_xy, dtype=float)
    if estimate.shape != truth.shape or estimate.ndim != 2 or estimate.shape[1] != 2:
        raise ValueError("estimate and truth must both have shape (N, 2)")
    if estimate.shape[0] < 2:
        raise ValueError("at least two poses are required")

    e_mean = estimate.mean(axis=0)
    t_mean = truth.mean(axis=0)
    e_centered = estimate - e_mean
    t_centered = truth - t_mean
    u, _, vt = np.linalg.svd(e_centered.T @ t_centered)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = t_mean - rotation @ e_mean
    aligned = (rotation @ estimate.T).T + translation
    return aligned, rotation, translation

