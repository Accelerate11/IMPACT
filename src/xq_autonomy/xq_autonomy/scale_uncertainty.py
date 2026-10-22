"""Moments of a shared uncertain scale times two normalized rates.

This is an offline mathematical building block, not a sensor adapter or fusion
filter.  q=(qx,qy) and d must already have established axes, units and semantics.
For v=d*q, supply the FULL covariance of (qx,qy,d), including cross terms.
In particular, repeated measurements sharing d must not average its variance
away.  The exact moments below assume a joint Gaussian input. They are not a
protection level, a Gaussian output claim, or evidence of hardware calibration.
"""

from __future__ import annotations

from numbers import Real

import numpy as np


def _array(value, shape, name):
    try:
        raw = np.asarray(value)
        if raw.shape != shape or raw.dtype.kind not in "iuf":
            raise ValueError
        # Reject booleans even in mixed Python containers before numpy coerces
        # them to numbers. ndarray inputs already have one declared dtype.
        def contains_bool(item):
            if isinstance(item, (bool, np.bool_)):
                return True
            return isinstance(item, (list, tuple)) and any(contains_bool(v) for v in item)
        if contains_bool(value):
            raise ValueError
        result = raw.astype(np.float64)
        if not np.all(np.isfinite(result)):
            raise ValueError
        return result
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError(f"{name} must be finite numeric data with shape {shape}") from exc


def scale_velocity_moments(normalized_rate_xy, distance_m, joint_covariance):
    """Return exact Gaussian-product and first-order moments for v=d*q.

    q has units 1/s, d metres, and covariance ordering is (qx,qy,d).  The
    distance argument is the positive distribution mean, not proof that a
    Gaussian range distribution has physically positive support. Callers must
    check its physical validity independently. Singular PSD input is allowed.
    Small symmetry/eigenvalue roundoff (1e-12 in a variance-balanced correlation
    matrix) is symmetrized/projected to zero. Balancing prevents a large variance
    in unrelated units from concealing an invalid smaller marginal.

    Vendor-normalized velocity and image-axis flow have different signs and
    semantics. This function performs no axis, rotational or timing conversion.
    Unknown covariance must not be represented as zero.
    """
    q = _array(normalized_rate_xy, (2,), "normalized_rate_xy")
    if isinstance(distance_m, (bool, np.bool_)) or not isinstance(distance_m, Real):
        raise ValueError("distance_m must be finite and positive")
    try:
        d = float(distance_m)
    except (OverflowError, ValueError) as exc:
        raise ValueError("distance_m must be finite and positive") from exc
    if not np.isfinite(d) or d <= 0:
        raise ValueError("distance_m must be finite and positive")
    covariance = _array(joint_covariance, (3, 3), "joint_covariance")
    variances = np.diag(covariance)
    if np.any(variances < 0):
        raise ValueError("joint_covariance must be positive semidefinite")
    positive = variances > 0
    if np.any(covariance[~positive, :] != 0) or np.any(covariance[:, ~positive] != 0):
        raise ValueError("zero variance requires zero cross covariance")
    if np.any(positive):
        scales = np.sqrt(variances[positive])
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise"):
                correlation = covariance[np.ix_(positive, positive)] / scales[:, None] / scales[None, :]
            if not np.all(np.isfinite(correlation)):
                raise FloatingPointError
        except FloatingPointError as exc:
            raise ValueError("invalid covariance correlation scaling") from exc
        if np.max(np.abs(correlation - correlation.T)) > 1.0e-12:
            raise ValueError("joint_covariance must be symmetric")
        correlation = 0.5 * correlation + 0.5 * correlation.T
        try:
            eigenvalues, eigenvectors = np.linalg.eigh(correlation)
        except np.linalg.LinAlgError as exc:
            raise ValueError("joint_covariance eigendecomposition failed") from exc
        if eigenvalues[0] < -1.0e-12:
            raise ValueError("joint_covariance must be positive semidefinite")
        if eigenvalues[0] < 0:
            correlation = (eigenvectors * np.maximum(eigenvalues, 0)) @ eigenvectors.T
        with np.errstate(over="raise", invalid="raise"):
            covariance[np.ix_(positive, positive)] = scales[:, None] * correlation * scales[None, :]
    a, c, s = covariance[:2, :2], covariance[:2, 2], covariance[2, 2]
    try:
        with np.errstate(over="raise", invalid="raise"):
            jacobian = np.column_stack((np.eye(2) * d, q))
            mean = d * q + c
            linear = jacobian @ covariance @ jacobian.T
            exact = linear + s * a + np.outer(c, c)
            velocity_range = d * c + s * q
            values = (mean, linear, exact, velocity_range, jacobian)
            if not all(np.all(np.isfinite(value)) for value in values):
                raise FloatingPointError
    except FloatingPointError as exc:
        raise ValueError("scale moment computation overflow") from exc
    return {
        "mean_velocity_mps": mean.tolist(),
        "covariance_mps2": (0.5 * exact + 0.5 * exact.T).tolist(),
        "linearized_covariance_mps2": (0.5 * linear + 0.5 * linear.T).tolist(),
        "covariance_velocity_range": velocity_range.tolist(),
        "linearization_jacobian": jacobian.tolist(),
        "assumption": "Joint Gaussian (qx,qy,d), v=d*q; exact first two moments only",
        "hardware_validated": False,
        "protection_level_available": False,
    }
