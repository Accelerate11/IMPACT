"""Independent synthetic oracles for one state/noise-correlated Gaussian window.

These checks do not calibrate sensors or establish a multi-window filter/PL.
Batch oracles condition the complete primitive nuisance vector; sequential
oracles update its conditional means and cross terms rather than reusing noise.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from xq_autonomy.correlated_update import correlated_gaussian_update


def _outputs(result):
    return tuple(np.asarray(result[key]) for key in
                 ("posterior_mean", "posterior_covariance", "gain", "innovation_covariance"))


def _batch(mean, covariance, observation, noise, measurement):
    """Condition a full augmented vector, without the correlated-update API."""
    innovation_covariance = observation @ covariance @ observation.T + noise
    cross = covariance @ observation.T
    gain = np.linalg.solve(innovation_covariance, cross.T).T
    return (mean + gain @ (measurement - observation @ mean),
            covariance - gain @ innovation_covariance @ gain.T)


@pytest.mark.parametrize("cross", [-3.0, 0.0, 3.0])
def test_scalar_closed_form(cross):
    result = correlated_gaussian_update([2.0], [[4.0]], [5.0], [[1.0]], [[9.0]], [[cross]])
    mean, covariance, gain, innovation = _outputs(result)
    expected_s = 13.0 + 2.0 * cross
    expected_k = (4.0 + cross) / expected_s
    np.testing.assert_allclose(innovation, [[expected_s]])
    np.testing.assert_allclose(gain, [[expected_k]])
    np.testing.assert_allclose(mean, [2.0 + 3.0 * expected_k])
    np.testing.assert_allclose(covariance, [[4.0 - (4.0 + cross) ** 2 / expected_s]])


def test_shared_noise_cancels_information_instead_of_shrinking_covariance():
    # e_x = n and v = -n + epsilon: z-H*mean = epsilon contains no x information.
    result = correlated_gaussian_update([7.0], [[1.0]], [20.0], [[1.0]], [[1.04]], [[-1.0]])
    mean, covariance, gain, innovation = _outputs(result)
    np.testing.assert_array_equal(mean, [7.0])
    np.testing.assert_array_equal(covariance, [[1.0]])
    np.testing.assert_array_equal(gain, [[0.0]])
    np.testing.assert_allclose(innovation, [[0.04]], rtol=1e-13)
    independent = correlated_gaussian_update([7.0], [[1.0]], [20.0], [[1.0]], [[1.04]], [[0.0]])
    assert independent["posterior_covariance"][0][0] < 0.51


def test_nineteen_state_result_matches_independent_augmented_nuisance_batch():
    rng = np.random.default_rng(1904)
    n, u, m = 19, 4, 3
    root = rng.normal(size=(n + u, n + u))
    joint = root @ root.T + np.eye(n + u) * 0.5
    mean = rng.normal(size=n + u)
    h, j = rng.normal(size=(m, n)), rng.normal(size=(m, u))
    private = np.diag([0.1, 0.3, 0.7])
    z = rng.normal(size=m)
    expected_mean, expected_covariance = _batch(mean, joint, np.hstack((h, j)), private, z)
    # Subtract the known nuisance mean to obtain the required zero-mean v model.
    r = j @ joint[n:, n:] @ j.T + private
    c = joint[:n, n:] @ j.T
    actual = correlated_gaussian_update(mean[:n], joint[:n, :n], z - j @ mean[n:], h, r, c)
    np.testing.assert_allclose(actual["posterior_mean"], expected_mean[:n], rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(actual["posterior_covariance"], expected_covariance[:n, :n], rtol=1e-12, atol=1e-12)
    assert actual["diagnostics"]["state_dimension"] == 19


def test_sequential_conditioning_updates_remaining_noise_mean_and_cross_terms():
    rng = np.random.default_rng(1903)
    n, m = 19, 3
    root = rng.normal(size=(n + m, n + m))
    joint = root @ root.T + np.eye(n + m)
    p, c, r = joint[:n, :n], joint[:n, n:], joint[n:, n:]
    mean, h, z = rng.normal(size=n), rng.normal(size=(m, n)), rng.normal(size=m)
    batch_mean, batch_cov, _, _ = _outputs(correlated_gaussian_update(mean, p, z, h, r, c))
    first = correlated_gaussian_update(mean, p, z[:1], h[:1], r[:1, :1], c[:, :1])
    first_mean, first_cov, _, s = _outputs(first)
    cross_z1_v2 = h[:1] @ c[:, 1:] + r[:1, 1:]
    cross_x_z1 = p @ h[:1].T + c[:, :1]
    v2_mean = cross_z1_v2.T @ np.linalg.solve(s, z[:1] - h[:1] @ mean)
    v2_cov = r[1:, 1:] - cross_z1_v2.T @ np.linalg.solve(s, cross_z1_v2)
    c2 = c[:, 1:] - cross_x_z1 @ np.linalg.solve(s, cross_z1_v2)
    final = correlated_gaussian_update(first_mean, first_cov, z[1:] - v2_mean, h[1:], v2_cov, c2)
    np.testing.assert_allclose(final["posterior_mean"], batch_mean, rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(final["posterior_covariance"], batch_cov, rtol=1e-12, atol=1e-12)


@pytest.mark.parametrize("rho", [-0.9, 0.0, 0.9])
def test_nineteen_state_covariance_matches_primitive_monte_carlo(rho):
    rng = np.random.default_rng(719)
    count, n = 40000, 19
    h = np.zeros((1, n)); h[0, 0] = 1.0
    c = np.zeros((n, 1)); c[0, 0] = rho
    result = correlated_gaussian_update(np.zeros(n), np.eye(n), [0.0], h, [[1.0]], c)
    _, posterior, gain, _ = _outputs(result)
    # Here all rho values have K=1/2; checking gain alone would miss the defect.
    np.testing.assert_allclose(gain[0, 0], 0.5)
    np.testing.assert_allclose(posterior[0, 0], (1.0 - rho) / 2.0)
    error = rng.normal(size=(count, n))
    noise = rho * error[:, 0] + np.sqrt(1.0 - rho * rho) * rng.normal(size=count)
    measurements = error[:, 0] + noise
    posterior_error = error - measurements[:, None] * gain[:, 0]
    variance = np.var(posterior_error, axis=0, ddof=1)
    # Six standard errors of an independent Gaussian sample-variance estimator.
    relative_error = np.abs(variance / np.diag(posterior) - 1.0)
    assert np.max(relative_error) < 6.0 * np.sqrt(2.0 / (count - 1))


def test_shared_range_error_does_not_average_away_in_a_batch():
    # Repeated observations share ONE range/scale error, rather than eight iid errors.
    n = 8
    common_variance, private_variance = 0.2, 0.03
    r = common_variance * np.ones((n, n)) + private_variance * np.eye(n)
    result = correlated_gaussian_update([0.0], [[1.0]], np.zeros(n), np.ones((n, 1)), r, np.zeros((1, n)))
    expected = 1.0 / (1.0 + n / (private_variance + n * common_variance))
    np.testing.assert_allclose(result["posterior_covariance"], [[expected]])
    naive = correlated_gaussian_update([0.0], [[1.0]], np.zeros(n), np.ones((n, 1)), np.diag(np.diag(r)), np.zeros((1, n)))
    assert naive["posterior_covariance"][0][0] < expected / 4.0


def test_explicit_zero_cross_matches_standard_kalman_control():
    p = np.diag(np.linspace(0.1, 2.0, 19))
    h = np.arange(38, dtype=float).reshape(2, 19) / 37.0
    r = np.array([[2.0, 0.3], [0.3, 1.0]])
    mean, z = np.linspace(-1, 1, 19), np.array([2.0, -1.0])
    expected_mean, expected_cov = _batch(mean, p, h, r, z)
    result = correlated_gaussian_update(mean, p, z, h, r, np.zeros((19, 2)))
    np.testing.assert_allclose(result["posterior_mean"], expected_mean, rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(result["posterior_covariance"], expected_cov, rtol=1e-12, atol=1e-12)
    assert result["diagnostics"]["explicit_zero_cross_covariance"]


def test_state_and_measurement_unit_changes_preserve_conditioning():
    p = np.array([[3.0, 0.2], [0.2, 0.4]])
    h = np.array([[0.4, -1.0], [2.0, 0.5]])
    r = np.array([[2.0, 0.1], [0.1, 0.7]])
    c = np.array([[0.2, -0.1], [0.04, 0.03]])
    mean, z = np.array([2.0, -4.0]), np.array([3.0, 7.0])
    original = _outputs(correlated_gaussian_update(mean, p, z, h, r, c))
    state_units, measurement_units = np.diag([1e6, 1e-6]), np.diag([1e-5, 1e5])
    inverse_units = np.diag([1e-6, 1e6])
    transformed = _outputs(correlated_gaussian_update(
        state_units @ mean, state_units @ p @ state_units.T, measurement_units @ z,
        measurement_units @ h @ inverse_units, measurement_units @ r @ measurement_units.T,
        state_units @ c @ measurement_units.T,
    ))
    np.testing.assert_allclose(inverse_units @ transformed[0], original[0], rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(inverse_units @ transformed[1] @ inverse_units.T, original[1], rtol=1e-12, atol=1e-12)


def test_singular_joint_prior_is_allowed_when_innovation_is_spd():
    result = correlated_gaussian_update([0.0], [[1.0]], [4.0], [[1.0]], [[1.0]], [[1.0]])
    np.testing.assert_allclose(result["posterior_mean"], [2.0])
    np.testing.assert_allclose(result["posterior_covariance"], [[0.0]], atol=1e-15)


@pytest.mark.parametrize("rho", [-1.01, 1.01])
def test_infeasible_cross_covariance_is_rejected(rho):
    with pytest.raises(ValueError, match="positive semidefinite"):
        correlated_gaussian_update([0.0], [[1.0]], [1.0], [[1.0]], [[1.0]], [[rho]])


def test_joint_psd_check_balances_units_instead_of_hiding_small_invalid_block():
    with pytest.raises(ValueError, match="positive semidefinite"):
        correlated_gaussian_update([0.0, 0.0], np.diag([1e12, 1e-12]), [0.0], [[0.0, 1.0]], [[1e-12]], [[0.0], [2e-12]])


def test_unknown_cross_covariance_is_rejected():
    with pytest.raises(ValueError, match="required"):
        correlated_gaussian_update([0.0], [[1.0]], [0.0], [[1.0]], [[1.0]], None)


def test_perfect_negative_correlation_with_zero_innovation_is_rejected():
    with pytest.raises(ValueError, match="positive definite"):
        correlated_gaussian_update([0.0], [[1.0]], [0.0], [[1.0]], [[1.0]], [[-1.0]])


def test_redundant_exact_observations_are_rejected_without_jitter():
    with pytest.raises(ValueError, match="positive definite"):
        correlated_gaussian_update([0.0], [[1.0]], [1.0, 1.0], [[1.0], [1.0]], np.zeros((2, 2)), np.zeros((1, 2)))


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), True, "1"])
def test_nonfinite_or_non_numeric_input_is_rejected(bad):
    with pytest.raises(ValueError):
        correlated_gaussian_update([0.0], [[1.0]], [bad], [[1.0]], [[1.0]], [[0.0]])


def test_negative_variance_and_nonzero_zero_variance_cross_are_rejected():
    with pytest.raises(ValueError, match="negative variance"):
        correlated_gaussian_update([0.0], [[-1e-30]], [0.0], [[1.0]], [[1.0]], [[0.0]])
    with pytest.raises(ValueError, match="zero variance"):
        correlated_gaussian_update([0.0], [[0.0]], [0.0], [[1.0]], [[1.0]], [[1e-30]])


def test_asymmetric_covariance_is_rejected():
    with pytest.raises(ValueError, match="not symmetric"):
        correlated_gaussian_update([0.0, 0.0], [[1.0, 0.1], [0.0, 1.0]], [0.0], [[1.0, 0.0]], [[1.0]], [[0.0], [0.0]])


def test_shape_errors_are_rejected():
    with pytest.raises(ValueError, match="wrong shape"):
        correlated_gaussian_update([0.0, 0.0], np.eye(2), [0.0], [[1.0, 0.0]], [[1.0]], [[0.0]])
    with pytest.raises(ValueError, match="nonempty vectors"):
        correlated_gaussian_update([], [], [0.0], [[]], [[1.0]], [])


def test_scope_and_diagnostics_cannot_claim_a_sensor_filter_or_protection_level():
    result = correlated_gaussian_update([0.0], [[1.0]], [0.0], [[1.0]], [[1.0]], [[0.0]])
    assert result["scope"] == "single_conditional_gaussian_window"
    assert result["multi_window_filter"] is False
    assert result["hardware_validated"] is False
    assert result["protection_level_available"] is False
    assert result["diagnostics"]["cross_covariance_assumed"] is False
    assert result["diagnostics"]["jitter_added"] is False
    assert result["diagnostics"]["eigenvalues_clipped"] is False
    assert result["diagnostics"]["posterior_augmented_joseph_scaled_max_difference"] < 1e-14
    json.dumps(result, allow_nan=False)
