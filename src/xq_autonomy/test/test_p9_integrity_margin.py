import numpy as np
import pytest

from xq_autonomy.integrity_margin import (
    certify_trajectory,
    compute_directional_protection_levels,
)


def test_directional_pl_preserves_covariance_anisotropy():
    covariance = np.diag((0.01, 0.04, 0.09))
    directions = np.eye(3)
    levels = compute_directional_protection_levels(directions, covariance, 2.0)
    assert np.allclose(levels, (0.2, 0.4, 0.6))


def test_trajectory_decision_uses_minimum_margin_not_minimum_alert_limit():
    covariance = np.diag((0.0001, 0.01, 0.0001))
    result = certify_trajectory(
        np.array((0.30, 0.35)),
        np.array(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))),
        covariance,
        k_alpha=2.0,
        margin_reserve=0.10,
    )
    assert result.critical_index == 1
    assert np.allclose(result.margins, (0.28, 0.15))
    assert result.accepted


def test_wide_accepts_and_narrow_rejects_under_identical_covariance():
    covariance = np.diag((1.6e-5, 1.6e-5, 1.6e-5))
    directions = np.array(((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)))
    wide = certify_trajectory(
        np.array((0.945, 0.945)), directions, covariance,
        k_alpha=51.234940240510646, margin_reserve=0.10,
    )
    narrow = certify_trajectory(
        np.array((0.045, 0.045)), directions, covariance,
        k_alpha=51.234940240510646, margin_reserve=0.10,
    )
    assert wide.accepted
    assert not narrow.accepted
    assert wide.protection_levels[0] == pytest.approx(narrow.protection_levels[0])


def test_rejects_nonunit_direction_and_indefinite_covariance():
    with pytest.raises(ValueError, match="unit"):
        compute_directional_protection_levels(np.array(((0.0, 2.0, 0.0),)), np.eye(3), 1.0)
    with pytest.raises(ValueError, match="semidefinite"):
        compute_directional_protection_levels(
            np.array(((1.0, 0.0, 0.0),)), np.diag((1.0, -0.1, 1.0)), 1.0
        )
