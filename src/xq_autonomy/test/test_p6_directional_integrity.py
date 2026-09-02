import numpy as np

from xq_autonomy.integrity import (
    TemporalInformationMemory,
    compute_directional_integrity,
    information_from_constraints,
)


def test_temporal_information_memory_is_disabled_by_default():
    memory = TemporalInformationMemory()
    first = np.diag((1.0, 2.0, 3.0))
    second = np.diag((4.0, 5.0, 6.0))
    assert np.array_equal(memory.update(first, 1.0), first)
    assert np.array_equal(memory.update(second, 1.1), second)
    assert memory.equivalent_frames == 1.0


def test_temporal_information_memory_accumulates_complementary_geometry_boundedly():
    memory = TemporalInformationMemory(horizon_s=5.0, maximum_equivalent_frames=3.0)
    x_plane = np.diag((2.0, 0.0, 0.0))
    y_plane = np.diag((0.0, 2.0, 0.0))
    z_plane = np.diag((0.0, 0.0, 2.0))
    memory.update(x_plane, 0.0)
    memory.update(y_plane, 0.1)
    combined = memory.update(z_plane, 0.2)
    assert np.linalg.eigvalsh(combined)[0] > 1.8
    for index in range(20):
        combined = memory.update(np.eye(3), 0.3 + 0.1 * index)
    assert memory.equivalent_frames <= 3.0
    assert np.linalg.eigvalsh(combined)[-1] <= 3.0 + 1.0e-9


def test_balanced_room_has_isotropic_information() -> None:
    normals = np.repeat(np.eye(3), 20, axis=0)
    information = information_from_constraints(
        normals, np.zeros(len(normals)), np.ones(len(normals)), residual_scale=0.08
    )
    result = compute_directional_integrity(
        information,
        np.eye(3) * 0.002,
        eta=0.02,
        epsilon=0.01,
        k_alpha=3.0,
        a_d=0.4,
        a_nnis=0.2,
        a_timing=0.2,
        a_residual=0.2,
    )
    np.testing.assert_allclose(result.geometry_eigenvalues, [20.0, 20.0, 20.0])
    np.testing.assert_allclose(result.protection_level_axes, result.protection_level_axes[0])
    assert result.condition_number == 1.0
    assert result.degeneracy_term == 0.0


def test_corridor_reports_unbounded_axis_as_weakest_and_larger_pl() -> None:
    # Corridor walls constrain y and its floor/ceiling constrain z; translation
    # along x is absent from the point-to-plane geometry.
    normals = np.repeat(np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 30, axis=0)
    information = information_from_constraints(
        normals, np.zeros(len(normals)), np.ones(len(normals)), residual_scale=0.08
    )
    result = compute_directional_integrity(
        information,
        np.eye(3) * 0.002,
        eta=0.02,
        epsilon=0.01,
        k_alpha=3.0,
        a_d=0.4,
        a_nnis=0.2,
        a_timing=0.2,
        a_residual=0.2,
    )
    assert abs(result.weak_direction[0]) > 0.999
    assert result.lambda_min == 0.0
    assert result.protection_level_axes[0] > 20.0 * result.protection_level_axes[1]


def test_all_kappa_coefficients_are_parameterized_and_active() -> None:
    result = compute_directional_integrity(
        np.diag([1.0, 10.0, 100.0]),
        np.eye(3) * 0.01,
        eta=0.0,
        epsilon=0.01,
        k_alpha=2.0,
        a_d=0.4,
        a_nnis=0.3,
        a_timing=0.2,
        a_residual=0.1,
        nnis_term=2.0,
        timing_jitter_term=3.0,
        residual_dynamics_term=4.0,
        degeneracy_condition_reference=100.0,
    )
    # condition number == reference, hence D == 1.
    assert abs(result.kappa - (1.0 + 0.4 + 0.3 * 2.0 + 0.2 * 3.0 + 0.1 * 4.0)) < 1.0e-12
    np.testing.assert_allclose(result.integrity_covariance, np.eye(3) * 0.01 * result.kappa)


def test_protection_level_matches_directional_quadratic_form() -> None:
    result = compute_directional_integrity(
        np.diag([4.0, 9.0, 16.0]),
        np.diag([0.01, 0.02, 0.03]),
        eta=0.05,
        epsilon=0.1,
        k_alpha=2.5,
        a_d=0.0,
        a_nnis=0.0,
        a_timing=0.0,
        a_residual=0.0,
    )
    expected = 2.5 * np.sqrt(np.diag(result.integrity_covariance))
    np.testing.assert_allclose(result.protection_level_axes, expected)
    expected_weak = 2.5 * np.sqrt(
        result.weak_direction @ result.integrity_covariance @ result.weak_direction
    )
    assert abs(result.weak_direction_protection_level - expected_weak) < 1.0e-12
