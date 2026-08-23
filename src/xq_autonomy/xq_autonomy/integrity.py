"""First-principles P6 directional integrity mathematics.

This module deliberately contains no ROS or Ground Truth dependency so the
estimator-facing equations can be unit-tested independently of simulation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DirectionalIntegrityResult:
    integrity_covariance: np.ndarray
    geometry_eigenvalues: np.ndarray
    weak_direction: np.ndarray
    protection_level_axes: np.ndarray
    weak_direction_protection_level: float
    lambda_min: float
    condition_number: float
    degeneracy_term: float
    kappa: float


def information_from_constraints(
    normals: np.ndarray,
    residuals: np.ndarray,
    geometry_weights: np.ndarray,
    residual_scale: float,
) -> np.ndarray:
    """Return Lambda_p = sum(w_i n_i n_i^T) for accepted FAST-LIO planes."""
    normal_array = np.asarray(normals, dtype=float)
    residual_array = np.asarray(residuals, dtype=float).reshape(-1)
    geometry_array = np.asarray(geometry_weights, dtype=float).reshape(-1)
    if normal_array.ndim != 2 or normal_array.shape[1] != 3:
        raise ValueError("normals must have shape (N, 3)")
    if len(normal_array) != len(residual_array) or len(normal_array) != len(geometry_array):
        raise ValueError("constraint arrays must have equal length")
    if residual_scale <= 0.0:
        raise ValueError("residual_scale must be positive")
    norms = np.linalg.norm(normal_array, axis=1)
    valid = norms > np.finfo(float).eps
    unit = np.zeros_like(normal_array)
    unit[valid] = normal_array[valid] / norms[valid, None]
    residual_weights = 1.0 / (1.0 + np.square(np.abs(residual_array) / residual_scale))
    weights = residual_weights * np.clip(geometry_array, 0.0, 1.0)
    information = np.einsum("n,ni,nj->ij", weights, unit, unit)
    return 0.5 * (information + information.T)


def compute_directional_integrity(
    information_matrix: np.ndarray,
    lio_position_covariance: np.ndarray,
    *,
    eta: float,
    epsilon: float,
    k_alpha: float,
    a_d: float,
    a_nnis: float,
    a_timing: float,
    a_residual: float,
    nnis_term: float = 0.0,
    timing_jitter_term: float = 0.0,
    residual_dynamics_term: float = 0.0,
    degeneracy_condition_reference: float = 100.0,
    degeneracy_cap: float = 3.0,
    covariance_floor: float = 1.0e-9,
) -> DirectionalIntegrityResult:
    """Compute P_int and PL(d) exactly as specified by IMPACT P6."""
    if eta < 0.0 or epsilon <= 0.0 or k_alpha <= 0.0:
        raise ValueError("eta must be non-negative; epsilon and k_alpha must be positive")
    if degeneracy_condition_reference <= 1.0 or degeneracy_cap <= 0.0:
        raise ValueError("degeneracy parameters are invalid")
    information = np.asarray(information_matrix, dtype=float).reshape(3, 3)
    information = 0.5 * (information + information.T)
    eigenvalues, eigenvectors = np.linalg.eigh(information)
    eigenvalues = np.maximum(eigenvalues, 0.0)
    weak_direction = eigenvectors[:, 0]
    lambda_min = float(eigenvalues[0])
    condition_number = float(eigenvalues[-1] / max(lambda_min, epsilon))
    degeneracy_term = float(
        np.clip(
            np.log(max(condition_number, 1.0)) / np.log(degeneracy_condition_reference),
            0.0,
            degeneracy_cap,
        )
    )
    kappa = float(
        1.0
        + a_d * degeneracy_term
        + a_nnis * max(float(nnis_term), 0.0)
        + a_timing * max(float(timing_jitter_term), 0.0)
        + a_residual * max(float(residual_dynamics_term), 0.0)
    )

    lio_covariance = np.asarray(lio_position_covariance, dtype=float).reshape(3, 3)
    lio_covariance = 0.5 * (lio_covariance + lio_covariance.T)
    lio_values, lio_vectors = np.linalg.eigh(lio_covariance)
    lio_covariance = lio_vectors @ np.diag(np.maximum(lio_values, covariance_floor)) @ lio_vectors.T
    geometry_covariance = np.linalg.inv(information + epsilon * np.eye(3))
    integrity_covariance = kappa * lio_covariance + eta * geometry_covariance
    integrity_covariance = 0.5 * (integrity_covariance + integrity_covariance.T)

    axis_variances = np.maximum(np.diag(integrity_covariance), 0.0)
    protection_level_axes = k_alpha * np.sqrt(axis_variances)
    weak_variance = float(weak_direction @ integrity_covariance @ weak_direction)
    weak_pl = float(k_alpha * np.sqrt(max(weak_variance, 0.0)))
    return DirectionalIntegrityResult(
        integrity_covariance=integrity_covariance,
        geometry_eigenvalues=eigenvalues,
        weak_direction=weak_direction,
        protection_level_axes=protection_level_axes,
        weak_direction_protection_level=weak_pl,
        lambda_min=lambda_min,
        condition_number=condition_number,
        degeneracy_term=degeneracy_term,
        kappa=kappa,
    )

