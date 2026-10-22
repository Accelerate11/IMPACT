"""One conditional Gaussian window with explicitly correlated observation noise.

The model is z = H x + v, with E[x] = mean, E[v] = 0 and
Cov((x - mean), v) = C.  All moments must be conditioned on the SAME prior
information.  In particular, unknown C is not zero and must not be invented
from a tracking quality score.  This pure function has no filter history,
sensor calibration, manifold retraction, protection level or ROS interface.

Keeping a shared range, image or IMU error across multiple windows requires
its posterior joint moments and conditional noise mean outside this function.
Calling this function repeatedly with the original moments is not that model.
"""

from __future__ import annotations

import math
from numbers import Real

import numpy as np


def _numeric(value, name):
    def contains_bool(item):
        if isinstance(item, (bool, np.bool_)):
            return True
        return isinstance(item, (list, tuple)) and any(contains_bool(v) for v in item)

    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf" or contains_bool(value):
            raise ValueError
        result = raw.astype(np.float64)
        if not np.all(np.isfinite(result)):
            raise ValueError
        return result
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError(name + " must contain finite real numeric values") from exc


def _scaled_joint(covariance, tolerance):
    diagonal = np.diag(covariance)
    if np.any(diagonal < 0.0):
        raise ValueError("joint covariance has a negative variance")
    zero = diagonal == 0.0
    if np.any(covariance[zero, :] != 0.0) or np.any(covariance[:, zero] != 0.0):
        raise ValueError("zero variance requires exactly zero joint cross covariance")
    scales = np.sqrt(np.where(zero, 1.0, diagonal))
    balanced = covariance / scales[:, None] / scales[None, :]
    if not np.all(np.isfinite(balanced)):
        raise ValueError("joint covariance scaling is non-finite")
    asymmetry = float(np.max(np.abs(balanced - balanced.T)))
    if asymmetry > tolerance:
        raise ValueError("joint covariance is not symmetric in correlation coordinates")
    # Only floating point asymmetry within the declared tolerance is removed.
    # No eigenvalue clipping, diagonal loading or jitter repairs input moments.
    balanced = 0.5 * balanced + 0.5 * balanced.T
    minimum = float(np.linalg.eigvalsh(balanced)[0])
    if minimum < -tolerance:
        raise ValueError("joint covariance is not positive semidefinite in correlation coordinates")
    return balanced, scales, asymmetry, minimum


def correlated_gaussian_update(
    prior_mean,
    prior_covariance,
    measurement,
    measurement_matrix,
    measurement_noise_covariance,
    state_noise_cross_covariance,
    *,
    correlation_tolerance=1.0e-12,
):
    """Condition one arbitrary-dimensional, jointly Gaussian observation.

    Arguments have shapes (n,), (n,n), (m,), (m,n), (m,m), and (n,m).
    C = state_noise_cross_covariance is mandatory, including an explicit zero
    matrix when independence is actually established.  With B = P H.T + C:

        S = H P H.T + H C + C.T H.T + R
        K = B solve(S, I)
        mean_post = mean + K (z - H mean)
        P_post = P - B solve(S, B.T)

    The implementation uses dimension-balanced matrices and Cholesky solves,
    never a matrix inverse.  Joint PSD input may be singular.  S must be SPD;
    redundant observations are rejected rather than silently removed/jittered.
    PSD/symmetry tolerances are dimensionless in correlation coordinates, and
    posterior roundoff is checked in the ORIGINAL prior-standard-deviation
    coordinates (a genuinely zero posterior need not have a usable own scale).
    Returned matrices are JSON-serializable lists.  Tiny roundoff eigenvalues
    are reported, not clipped.  The caller owns any error-state reset Jacobian.
    """
    if (isinstance(correlation_tolerance, (bool, np.bool_))
            or not isinstance(correlation_tolerance, Real)
            or not math.isfinite(correlation_tolerance)
            or not 0.0 < correlation_tolerance <= 1.0e-9):
        raise ValueError("correlation_tolerance must be positive and at most 1e-9")
    if state_noise_cross_covariance is None:
        raise ValueError("state_noise_cross_covariance is required; unknown C is not zero")
    mean = _numeric(prior_mean, "prior_mean")
    z = _numeric(measurement, "measurement")
    if mean.ndim != 1 or z.ndim != 1 or not mean.size or not z.size:
        raise ValueError("prior_mean and measurement must be nonempty vectors")
    n, m = mean.size, z.size
    matrices = (
        (prior_covariance, (n, n), "prior_covariance"),
        (measurement_matrix, (m, n), "measurement_matrix"),
        (measurement_noise_covariance, (m, m), "measurement_noise_covariance"),
        (state_noise_cross_covariance, (n, m), "state_noise_cross_covariance"),
    )
    parsed = []
    for value, shape, name in matrices:
        array = _numeric(value, name)
        if array.shape != shape:
            raise ValueError(name + " has the wrong shape")
        parsed.append(array)
    p, h, r, c = parsed
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            joint, scales, asymmetry, joint_minimum = _scaled_joint(
                np.block([[p, c], [c.T, r]]), correlation_tolerance,
            )
            state_scales, noise_scales = scales[:n], scales[n:]
            h_state_scaled = h * state_scales[None, :]
            observation_scales = np.maximum(
                np.max(np.abs(h_state_scaled), axis=1), np.sqrt(np.diag(r)),
            )
            if np.any(observation_scales <= 0.0):
                raise ValueError("innovation covariance must be positive definite")
            observation_map = np.concatenate(
                (h_state_scaled, np.diag(noise_scales)), axis=1,
            ) / observation_scales[:, None]
            s_scaled = observation_map @ joint @ observation_map.T
            s_scaled = 0.5 * s_scaled + 0.5 * s_scaled.T
            if np.any(np.diag(s_scaled) <= 0.0):
                raise ValueError("innovation covariance must be positive definite")
            innovation_scales = np.sqrt(np.diag(s_scaled))
            s_correlation = s_scaled / innovation_scales[:, None] / innovation_scales[None, :]
            s_correlation = 0.5 * s_correlation + 0.5 * s_correlation.T
            innovation_minimum = float(np.linalg.eigvalsh(s_correlation)[0])
            if innovation_minimum <= 0.0:
                raise ValueError("innovation covariance must be positive definite; no jitter is added")
            factor = np.linalg.cholesky(s_correlation)
            b_scaled = (joint[:n, :] @ observation_map.T) / innovation_scales[None, :]
            gain_scaled = np.linalg.solve(factor.T, np.linalg.solve(factor, b_scaled.T)).T
            posterior_scaled = joint[:n, :n] - gain_scaled @ b_scaled.T
            posterior_asymmetry = float(np.max(np.abs(posterior_scaled - posterior_scaled.T)))
            if posterior_asymmetry > correlation_tolerance:
                raise ValueError("posterior covariance lost symmetry")
            posterior_scaled = 0.5 * posterior_scaled + 0.5 * posterior_scaled.T
            posterior_minimum = float(np.linalg.eigvalsh(posterior_scaled)[0])
            if posterior_minimum < -correlation_tolerance:
                raise ValueError("posterior covariance is not positive semidefinite")
            innovation = z - h @ mean
            whitened_innovation = innovation / observation_scales / innovation_scales
            posterior_mean = mean + state_scales * (gain_scaled @ whitened_innovation)
            posterior_covariance = posterior_scaled * state_scales[:, None] * state_scales[None, :]
            gain = gain_scaled * state_scales[:, None] / innovation_scales[None, :] / observation_scales[None, :]
            s = s_scaled * observation_scales[:, None] * observation_scales[None, :]
            # Independent algebraic diagnostic: augmented Joseph transform,
            # which includes both signed state/noise cross terms.
            transform = np.concatenate((np.eye(n), np.zeros((n, m))), axis=1)
            transform -= (gain_scaled / innovation_scales[None, :]) @ observation_map
            joseph = transform @ joint @ transform.T
            joseph_difference = float(np.max(np.abs(joseph - posterior_scaled)))
            values = (posterior_mean, posterior_covariance, gain, innovation, s)
            if not all(np.all(np.isfinite(value)) for value in values):
                raise ValueError("conditioning produced non-finite output")
    except (FloatingPointError, np.linalg.LinAlgError) as exc:
        raise ValueError("conditioning is not numerically well defined: " + str(exc)) from exc
    return {
        "posterior_mean": posterior_mean.tolist(),
        "posterior_covariance": posterior_covariance.tolist(),
        "gain": gain.tolist(),
        "innovation": innovation.tolist(),
        "innovation_covariance": s.tolist(),
        "diagnostics": {
            "state_dimension": int(n),
            "measurement_dimension": int(m),
            "correlation_tolerance": float(correlation_tolerance),
            "joint_scaled_asymmetry_max": asymmetry,
            "joint_scaled_minimum_eigenvalue": joint_minimum,
            "innovation_correlation_minimum_eigenvalue": innovation_minimum,
            "posterior_prior_scaled_minimum_eigenvalue": posterior_minimum,
            "posterior_prior_scaled_asymmetry_max": posterior_asymmetry,
            "posterior_augmented_joseph_scaled_max_difference": joseph_difference,
            "explicit_zero_cross_covariance": bool(np.all(c == 0.0)),
            "cross_covariance_assumed": False,
            "jitter_added": False,
            "eigenvalues_clipped": False,
        },
        "scope": "single_conditional_gaussian_window",
        "assumption": "Joint Gaussian (x-mean,v), conditioned on the same prior information; E[v]=0",
        "multi_window_filter": False,
        "hardware_validated": False,
        "protection_level_available": False,
    }
