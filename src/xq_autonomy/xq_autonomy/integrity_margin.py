"""Pure P9 directional Protection Level and hard trajectory certification."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class IntegrityMarginResult:
    protection_levels: np.ndarray
    margins: np.ndarray
    critical_index: int
    minimum_margin: float
    accepted: bool


def compute_directional_protection_levels(
    directions: np.ndarray,
    integrity_covariance: np.ndarray,
    k_alpha: float,
) -> np.ndarray:
    """Evaluate PL(a)=k_alpha*sqrt(a.T*P_int*a) for unit map directions."""
    unit_directions = np.asarray(directions, dtype=float)
    covariance = np.asarray(integrity_covariance, dtype=float)
    if unit_directions.ndim != 2 or unit_directions.shape[1] != 3 or len(unit_directions) == 0:
        raise ValueError("directions must be non-empty Nx3")
    if covariance.shape != (3, 3):
        raise ValueError("integrity covariance must be 3x3")
    if not np.isfinite(unit_directions).all() or not np.isfinite(covariance).all():
        raise ValueError("P9 inputs must be finite")
    if not np.isfinite(k_alpha) or k_alpha <= 0.0:
        raise ValueError("k_alpha must be positive and finite")
    norms = np.linalg.norm(unit_directions, axis=1)
    if np.any(np.abs(norms - 1.0) > 1.0e-6):
        raise ValueError("obstacle directions must be unit vectors")
    covariance = 0.5 * (covariance + covariance.T)
    eigenvalues = np.linalg.eigvalsh(covariance)
    if eigenvalues[0] < -1.0e-12:
        raise ValueError("integrity covariance must be positive semidefinite")
    variances = np.einsum("ni,ij,nj->n", unit_directions, covariance, unit_directions)
    return float(k_alpha) * np.sqrt(np.maximum(variances, 0.0))


def certify_trajectory(
    alert_limits: np.ndarray,
    directions: np.ndarray,
    integrity_covariance: np.ndarray,
    *,
    k_alpha: float,
    margin_reserve: float,
) -> IntegrityMarginResult:
    """Apply the P9 hard constraint to every remaining trajectory sample."""
    limits = np.asarray(alert_limits, dtype=float).reshape(-1)
    if len(limits) == 0 or not np.isfinite(limits).all():
        raise ValueError("alert limits must be non-empty and finite")
    if not np.isfinite(margin_reserve) or margin_reserve < 0.0:
        raise ValueError("margin_reserve must be nonnegative and finite")
    protection_levels = compute_directional_protection_levels(
        directions, integrity_covariance, k_alpha
    )
    if len(protection_levels) != len(limits):
        raise ValueError("alert-limit and obstacle-direction profiles must align")
    margins = limits - protection_levels
    critical_index = int(np.argmin(margins))
    minimum_margin = float(margins[critical_index])
    return IntegrityMarginResult(
        protection_levels=protection_levels,
        margins=margins,
        critical_index=critical_index,
        minimum_margin=minimum_margin,
        accepted=minimum_margin >= float(margin_reserve),
    )
