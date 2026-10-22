"""Independent moment checks for a synthetic shared-scale observation model.

Gauss-Hermite cubature integrates the Gaussian polynomial moments directly;
it does not reproduce the implementation's closed-form covariance expression.
These tests establish arithmetic properties, not sensor calibration or PL.
"""

from __future__ import annotations

from itertools import product
import json
import math
import unittest

import numpy as np

from xq_autonomy.scale_uncertainty import scale_velocity_moments


def _arrays(result):
    return (
        np.asarray(result["mean_velocity_mps"], dtype=float),
        np.asarray(result["covariance_mps2"], dtype=float),
        np.asarray(result["linearized_covariance_mps2"], dtype=float),
        np.asarray(result["covariance_velocity_range"], dtype=float),
    )


def _gaussian_cubature(mean, covariance):
    """27 nodes exactly integrate three-dimensional polynomials to degree five."""
    nodes, weights = np.polynomial.hermite.hermgauss(3)
    root = np.linalg.cholesky(covariance)
    samples, probabilities = [], []
    for indexes in product(range(3), repeat=3):
        standard = math.sqrt(2.0) * np.asarray([nodes[index] for index in indexes])
        probability = math.prod(weights[index] for index in indexes) / math.pi ** 1.5
        samples.append(mean + root @ standard)
        probabilities.append(probability)
    samples, probabilities = np.asarray(samples), np.asarray(probabilities)
    velocities = samples[:, :2] * samples[:, 2, None]
    expected = probabilities @ velocities
    centred = velocities - expected
    velocity_covariance = np.einsum("n,ni,nj->ij", probabilities, centred, centred)
    velocity_range = np.einsum(
        "n,ni,n->i", probabilities, centred, samples[:, 2] - mean[2],
    )
    return expected, velocity_covariance, velocity_range


class ScaleVelocityMomentTests(unittest.TestCase):
    def assertClose(self, actual, expected):
        np.testing.assert_allclose(actual, expected, rtol=2.0e-11, atol=2.0e-13)

    def test_zero_uncertainty_is_a_deterministic_product(self):
        result = scale_velocity_moments([0.25, -0.4], 2.0, np.zeros((3, 3)).tolist())
        mean, covariance, linearized, cross = _arrays(result)
        self.assertClose(mean, [0.5, -0.8])
        self.assertClose(covariance, np.zeros((2, 2)))
        self.assertClose(linearized, covariance)
        self.assertClose(cross, np.zeros(2))

    def test_only_scale_uncertainty_has_rank_one_common_error(self):
        q = np.asarray([0.3, -0.7])
        scale_variance = 0.04
        joint = np.diag([0.0, 0.0, scale_variance])
        result = scale_velocity_moments(q.tolist(), 1.5, joint.tolist())
        mean, covariance, linearized, cross = _arrays(result)
        self.assertClose(mean, 1.5 * q)
        self.assertClose(covariance, scale_variance * np.outer(q, q))
        self.assertClose(linearized, covariance)
        self.assertClose(cross, scale_variance * q)
        self.assertLess(covariance[0, 1], 0.0)
        self.assertEqual(np.linalg.matrix_rank(covariance, tol=1.0e-12), 1)

    def test_constant_scale_preserves_correlated_flow_noise(self):
        a = np.asarray([[0.03, -0.012], [-0.012, 0.02]])
        joint = np.zeros((3, 3))
        joint[:2, :2] = a
        result = scale_velocity_moments([0.1, 0.2], 3.0, joint.tolist())
        _, covariance, linearized, cross = _arrays(result)
        self.assertClose(covariance, 9.0 * a)
        self.assertClose(linearized, covariance)
        self.assertClose(cross, np.zeros(2))

    def test_zero_mean_flow_still_has_a_second_order_scale_term(self):
        a = np.asarray([[0.02, 0.006], [0.006, 0.01]])
        distance, variance = 1.3, 0.09
        joint = np.zeros((3, 3))
        joint[:2, :2] = a
        joint[2, 2] = variance
        mean, covariance, linearized, cross = _arrays(
            scale_velocity_moments([0.0, 0.0], distance, joint.tolist())
        )
        self.assertClose(mean, np.zeros(2))
        self.assertClose(linearized, distance ** 2 * a)
        self.assertClose(covariance - linearized, variance * a)
        self.assertClose(cross, np.zeros(2))

    def test_general_correlated_case_matches_independent_gaussian_cubature(self):
        root = np.asarray([[0.08, 0.0, 0.0], [0.02, 0.05, 0.0], [0.03, -0.04, 0.11]])
        joint = root @ root.T
        state_mean = np.asarray([0.25, -0.4, 1.7])
        result = scale_velocity_moments(state_mean[:2].tolist(), state_mean[2], joint.tolist())
        mean, covariance, _, cross = _arrays(result)
        reference_mean, reference_covariance, reference_cross = _gaussian_cubature(state_mean, joint)
        self.assertClose(mean, reference_mean)
        self.assertClose(covariance, reference_covariance)
        self.assertClose(cross, reference_cross)
        self.assertGreater(np.linalg.norm(mean - state_mean[:2] * state_mean[2]), 0.0)

    def test_perfectly_correlated_rank_one_gaussian_has_correct_polynomial_moments(self):
        # q=a*d+b, hence v=a*d^2+b*d. This independent scalar polynomial
        # reference checks nonzero Cov(q,d), mean bias and singular PSD input.
        a, b = np.asarray([0.4, -0.7]), np.asarray([-0.1, 0.3])
        distance, variance = 2.0, 0.09
        q = a * distance + b
        factor = np.r_[a, 1.0]
        joint = variance * np.outer(factor, factor)
        mean, covariance, linearized, cross = _arrays(
            scale_velocity_moments(q.tolist(), distance, joint.tolist())
        )
        expected_mean = a * (distance ** 2 + variance) + b * distance
        expected_covariance = (
            np.outer(a, a) * (4.0 * distance ** 2 * variance + 2.0 * variance ** 2)
            + np.outer(b, b) * variance
            + (np.outer(a, b) + np.outer(b, a)) * (2.0 * distance * variance)
        )
        derivative = 2.0 * distance * a + b
        self.assertClose(mean, expected_mean)
        self.assertClose(covariance, expected_covariance)
        self.assertClose(linearized, variance * np.outer(derivative, derivative))
        self.assertClose(cross, variance * derivative)

    def test_reported_jacobian_matches_a_central_difference(self):
        mean = np.asarray([0.21, -0.38, 2.1])
        joint = np.asarray([[0.02, -0.004, 0.005], [-0.004, 0.03, -0.008], [0.005, -0.008, 0.04]])
        result = scale_velocity_moments(mean[:2].tolist(), mean[2], joint.tolist())
        jacobian = np.zeros((2, 3))
        epsilon = 1.0e-5
        for axis in range(3):
            shift = np.zeros(3)
            shift[axis] = epsilon
            plus, minus = mean + shift, mean - shift
            jacobian[:, axis] = (plus[:2] * plus[2] - minus[:2] * minus[2]) / (2.0 * epsilon)
        np.testing.assert_allclose(result["linearization_jacobian"], jacobian, rtol=1.0e-9, atol=1.0e-10)
        np.testing.assert_allclose(result["linearized_covariance_mps2"], jacobian @ joint @ jacobian.T,
                                   rtol=1.0e-9, atol=1.0e-11)

    def test_shared_range_does_not_average_away_with_independent_frame_noise(self):
        q = np.asarray([0.7, -0.45])
        distance, scale_variance = 2.0, 0.12 ** 2
        a = np.asarray([[0.0036, 0.0004], [0.0004, 0.0025]])
        joint = np.zeros((3, 3))
        joint[:2, :2], joint[2, 2] = a, scale_variance
        single = _arrays(scale_velocity_moments(q.tolist(), distance, joint.tolist()))[1]
        for count in (1, 10, 40):
            with self.subTest(count=count):
                window_joint = joint.copy()
                window_joint[:2, :2] /= count
                correct = _arrays(scale_velocity_moments(q.tolist(), distance, window_joint.tolist()))[1]
                naive = single / count
                self.assertClose(correct - naive, scale_variance * (1.0 - 1.0 / count) * np.outer(q, q))
                self.assertGreaterEqual(float(np.linalg.eigvalsh(correct - naive).min()), -1.0e-13)

    def test_zero_scale_noise_control_recovers_one_over_n_covariance(self):
        flow_covariance = np.asarray([[0.0036, -0.001], [-0.001, 0.0025]])
        # Construct the joint covariance explicitly rather than adding a fake
        # range variance to make a Cholesky decomposition possible.
        covariance = np.zeros((3, 3))
        covariance[:2, :2] = flow_covariance
        single = _arrays(scale_velocity_moments([0.7, -0.45], 2.0, covariance.tolist()))[1]
        for count in (1, 10, 40):
            with self.subTest(count=count):
                window = covariance.copy()
                window[:2, :2] /= count
                actual = _arrays(scale_velocity_moments([0.7, -0.45], 2.0, window.tolist()))[1]
                self.assertClose(actual, single / count)

    def test_api_outputs_are_json_lists_with_expected_dimensions(self):
        result = scale_velocity_moments([0.2, -0.3], 2.0, np.eye(3).tolist())
        for name, shape in (
            ("mean_velocity_mps", (2,)), ("covariance_mps2", (2, 2)),
            ("linearized_covariance_mps2", (2, 2)),
            ("covariance_velocity_range", (2,)), ("linearization_jacobian", (2, 3)),
        ):
            with self.subTest(name=name):
                self.assertIsInstance(result[name], list)
                self.assertEqual(np.asarray(result[name]).shape, shape)
                self.assertTrue(np.isfinite(np.asarray(result[name], dtype=float)).all())
        self.assertIs(result["hardware_validated"], False)
        self.assertIs(result["protection_level_available"], False)
        json.dumps(result, allow_nan=False)

    def test_overflow_rejected_as_value_error(self):
        for mean, distance in (([1.0e308, 0.2], 1.0e308), ([0.2, -0.3], 10 ** 1000)):
            with self.subTest(distance_type=type(distance).__name__):
                with self.assertRaises(ValueError):
                    scale_velocity_moments(mean, distance, np.eye(3).tolist())

    def test_covariance_validity_cannot_be_hidden_by_an_unrelated_large_variance(self):
        # Mixed physical units or a change of units must not turn a materially
        # invalid marginal into an accepted covariance. The large qx variance
        # does not make negative range variance or correlation two permissible.
        for covariance in (
            np.diag([1.0e12, 1.0, -0.5]).tolist(),
            [[1.0e12, 0.0, 0.0], [0.0, 1.0e-8, 2.0e-8], [0.0, 2.0e-8, 1.0e-8]],
        ):
            with self.subTest(covariance=covariance):
                with self.assertRaises(ValueError):
                    scale_velocity_moments([0.2, -0.3], 2.0, covariance)

    def test_invalid_distance_rejected(self):
        for distance in (0.0, -1.0, math.nan, math.inf, -math.inf, True, False, np.bool_(True)):
            with self.subTest(distance=distance):
                with self.assertRaises(ValueError):
                    scale_velocity_moments([0.2, -0.3], distance, np.eye(3).tolist())

    def test_invalid_flow_mean_rejected(self):
        for mean in ([0.1], [0.1, 0.2, 0.3], [[0.1, 0.2]], [math.nan, 0.2],
                     [0.2, math.inf], [True, 0.2], [0.2, np.bool_(False)]):
            with self.subTest(mean=mean):
                with self.assertRaises(ValueError):
                    scale_velocity_moments(mean, 2.0, np.eye(3).tolist())

    def test_invalid_covariance_rejected(self):
        invalid = (
            np.eye(2).tolist(),
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [[1.0, 0.0, 0.0], [0.0, math.nan, 0.0], [0.0, 0.0, 1.0]],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, math.inf]],
            [[1.0, 0.1, 0.0], [0.2, 1.0, 0.0], [0.0, 0.0, 1.0]],
            np.diag([-0.01, 1.0, 1.0]).tolist(),
            [[1.0, 0.0, 2.0], [0.0, 1.0, 0.0], [2.0, 0.0, 1.0]],
            [[True, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
        for covariance in invalid:
            with self.subTest(covariance=covariance):
                with self.assertRaises(ValueError):
                    scale_velocity_moments([0.2, -0.3], 2.0, covariance)


if __name__ == "__main__":
    unittest.main()
