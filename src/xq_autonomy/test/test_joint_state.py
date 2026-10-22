"""Generative latent-vector checks of the offline linear joint-state fixture.

Independent batch conditioning, not repeated calls to the production update,
is the multi-window oracle.  This is not physical IMU/vision/scale validation.
"""

import json
from dataclasses import replace

import numpy as np
import pytest

from xq_autonomy.joint_state import (
    clone_pose, create_joint_state, drop_clone, linear_update, propagate_active,
)


def _initial(seed=328):
    rng = np.random.default_rng(seed)
    root = 0.15 * rng.normal(size=(20, 20)) + np.eye(20)
    mean = rng.normal(size=20)
    covariance = root @ root.T
    state = create_joint_state(mean[:19], covariance[:19, :19], mean[19:], covariance[19:, 19:],
                               covariance[:19, 19:], timestamp_ns=100)
    return state, root, mean


def _update(state, z, h, r):
    return linear_update(state, z, h, r, measurement_timestamp_ns=state.timestamp_ns,
                         independent_measurement_noise=True)


def _propagate(state, f, q, offset=None, dt=10):
    return propagate_active(state, f, q, np.zeros(19) if offset is None else offset,
                            next_timestamp_ns=state.timestamp_ns + dt,
                            independent_process_noise=True)


def _condition_latent(target_mean, target_map, observation_mean, observation_map, measurement):
    # u ~ N(0,I); target=target_mean+target_map*u; z=obs_mean+obs_map*u.
    innovation_covariance = observation_map @ observation_map.T
    cross = target_map @ observation_map.T
    gain = np.linalg.solve(innovation_covariance, cross.T).T
    mean = target_mean + gain @ (measurement - observation_mean)
    residual_map = target_map - gain @ observation_map
    # The residual loading avoids subtracting nearly equal covariance matrices.
    return mean, residual_map @ residual_map.T


def test_initial_block_indices_scope_and_immutable_moments():
    state, root, mean = _initial()
    assert state.dimension == 20
    assert state.active_indices == tuple(range(19))
    assert state.nuisance_indices == (19,)
    assert state.clone_ids == ()
    np.testing.assert_array_equal(state.mean, mean)
    np.testing.assert_allclose(state.covariance, root @ root.T, atol=1e-15)
    for array in (state.mean, state.covariance):
        with pytest.raises(ValueError):
            array.flat[0] = 0.0
        with pytest.raises(ValueError):
            array.setflags(write=True)
    encoded = json.loads(json.dumps(state.to_dict()))
    assert encoded["scope"] == "offline_discrete_linear_joint_gaussian_fixture"
    assert encoded["physical_rotation_model"] is False
    assert encoded["livo_integration"] is False
    assert encoded["hardware_validated"] is False
    assert encoded["protection_level_available"] is False


def test_direct_replacement_cannot_bypass_moment_validation_or_immutability():
    state = _initial()[0]
    with pytest.raises(ValueError):
        replace(state, _covariance=-np.eye(20))
    with pytest.raises(ValueError):
        replace(state, _mean=np.zeros(19))
    with pytest.raises(ValueError):
        replace(state, _clones=(("future", 101),))
    mutable_mean, mutable_p = state.mean.copy(), state.covariance.copy()
    independent = replace(state, _mean=mutable_mean, _covariance=mutable_p)
    mutable_mean[0] = 1e6
    mutable_p[0, 0] = 1e6
    np.testing.assert_array_equal(independent.mean, state.mean)
    np.testing.assert_array_equal(independent.covariance, state.covariance)


def test_clone_copies_the_same_latent_variables_with_every_cross_block():
    original, root, mean = _initial()
    cloned = clone_pose(original, "start")
    latent_map = np.vstack((root, root[:6]))
    np.testing.assert_allclose(cloned.covariance, latent_map @ latent_map.T, atol=1e-15)
    np.testing.assert_array_equal(cloned.mean, np.concatenate((mean, mean[:6])))
    np.testing.assert_array_equal(cloned.covariance[20:, :20], original.covariance[:6])
    np.testing.assert_array_equal(cloned.covariance[20:, 19], original.covariance[:6, 19])
    assert cloned.clone_indices("start") == tuple(range(20, 26))
    assert cloned.clone_timestamp_ns("start") == 100
    assert np.linalg.matrix_rank(cloned.covariance, tol=1e-10) == 20
    assert original.dimension == 20
    assert not cloned.to_dict()["diagnostics"]["jitter_added"]


def test_bias_integration_propagates_cross_to_old_clone_and_persistent_scale():
    initial, root, mean = _initial()
    state = clone_pose(initial, "old")
    f = np.eye(19)
    f[0, 7] = 0.3
    f[0, 13] = -0.045
    f[7, 13] = -0.3
    f[3, 14] = -0.3
    q_root = np.diag(np.linspace(0.002, 0.02, 19))
    offset = np.linspace(-0.1, 0.1, 19)
    actual = _propagate(state, f, q_root @ q_root.T, offset)
    loading = np.zeros((26, 39))
    loading[:19, :20] = f @ root[:19]
    loading[:19, 20:] = q_root
    loading[19, :20] = root[19]
    loading[20:, :20] = root[:6]
    expected_mean = np.concatenate((f @ mean[:19] + offset, mean[19:], mean[:6]))
    np.testing.assert_allclose(actual.mean, expected_mean, atol=1e-15)
    np.testing.assert_allclose(actual.covariance, loading @ loading.T, atol=1e-14)
    assert actual.covariance[7, 20] != state.covariance[7, 20]
    assert actual.covariance[0, 19] != state.covariance[0, 19]
    assert actual.timestamp_ns == 110
    assert actual.clone_timestamp_ns("old") == 100
    np.testing.assert_array_equal(state.covariance, clone_pose(initial, "old").covariance)


def test_active_only_measurement_updates_unmeasured_clone_and_nuisance():
    initial, root, mean = _initial()
    state = clone_pose(initial, "old")
    h = np.zeros((2, 26)); h[0, 0] = 1.0; h[1, 7] = 1.0
    r_root = np.diag([0.2, 0.4])
    z = np.array([2.0, -1.0])
    loading = np.pad(np.vstack((root, root[:6])), ((0, 0), (0, 2)))
    observation_map = h @ loading
    observation_map[:, -2:] = r_root
    expected_mean, expected_p = _condition_latent(state.mean, loading, h @ state.mean,
                                                  observation_map, z)
    result = _update(state, z, h, r_root @ r_root.T)
    np.testing.assert_allclose(result.state.mean, expected_mean, atol=2e-14)
    np.testing.assert_allclose(result.state.covariance, expected_p, atol=2e-14)
    assert np.max(np.abs(result.state.mean[20:] - state.mean[20:])) > 0.1
    assert result.state.mean[19] != state.mean[19]
    assert np.max(np.abs(result.state.covariance[19:, :19] - state.covariance[19:, :19])) > 0.01
    assert result.diagnostics["all_joint_blocks_updated"]


def test_marginal_drop_keeps_uncertainty_instead_of_conditioning_on_clone():
    state = clone_pose(_initial()[0], "old")
    after = drop_clone(state, "old")
    np.testing.assert_array_equal(after.mean, state.mean[:20])
    np.testing.assert_array_equal(after.covariance, state.covariance[:20, :20])
    conditional = (state.covariance[:20, :20] - state.covariance[:20, 20:]
                   @ np.linalg.solve(state.covariance[20:, 20:], state.covariance[20:, :20]))
    assert np.trace(after.covariance) - np.trace(conditional) > 1.0
    assert after.timestamp_ns == state.timestamp_ns
    assert after.clone_ids == ()


def test_multiple_clone_names_times_and_index_compaction():
    state = clone_pose(_initial()[0], "a")
    state = _propagate(state, np.eye(19), np.eye(19) * 0.001)
    state = clone_pose(state, "b")
    assert state.clone_indices("b") == tuple(range(26, 32))
    assert state.clone_timestamp_ns("a") == 100
    assert state.clone_timestamp_ns("b") == 110
    keep = list(range(20)) + list(range(26, 32))
    actual = drop_clone(state, "a")
    np.testing.assert_array_equal(actual.covariance, state.covariance[np.ix_(keep, keep)])
    assert actual.clone_ids == ("b",)
    assert actual.clone_indices("b") == tuple(range(20, 26))
    assert actual.clone_timestamp_ns("b") == 110


def test_three_windows_match_one_independent_global_latent_batch():
    rng = np.random.default_rng(371)
    state, initial_root, initial_mean = _initial(372)
    # Primitive independent variables: initial20, three process19, nine white
    # observations (two window rows + one range-like row in each window).
    primitive_dimension = 20 + 3 * 19 + 9
    loading = np.zeros((20, primitive_dimension)); loading[:, :20] = initial_root
    mean = initial_mean.copy()
    observation_maps, observation_means, measurements = [], [], []
    for window in range(3):
        state = clone_pose(state, "window" + str(window))
        loading = np.vstack((loading, loading[:6]))
        mean = np.concatenate((mean, mean[:6]))
        f = np.eye(19)
        f[0, 7] = 0.2; f[1, 8] = 0.2
        f[7, 13] = -0.2; f[8, 14] = -0.2; f[3, 13] = -0.2
        f[0, 13] = -0.02; f[1, 14] = -0.02
        q_root = np.diag(np.linspace(0.003, 0.02, 19))
        offset = rng.normal(size=19) * 0.01
        state = _propagate(state, f, q_root @ q_root.T, offset)
        loading[:19] = f @ loading[:19]
        loading[:19, 20 + window * 19:20 + (window + 1) * 19] = q_root
        mean[:19] = f @ mean[:19] + offset
        h = np.zeros((2, 26))
        h[0, 0] = 1.0; h[0, 20] = -1.0; h[0, 19] = 0.7
        h[1, 1] = 1.0; h[1, 21] = -1.0; h[1, 19] = -0.4
        h[0, 3] = 0.2; h[1, 4] = -0.2
        for sensor in range(2):
            current_h = h if sensor == 0 else np.eye(26)[19:20] + 0.15 * np.eye(26)[2:3]
            m = current_h.shape[0]
            r_root = np.diag([0.07, 0.11]) if sensor == 0 else np.array([[0.13]])
            obs_loading = current_h @ loading
            column = 20 + 3 * 19 + 3 * window + (0 if sensor == 0 else 2)
            obs_loading[:, column:column + m] = r_root
            obs_mean = current_h @ mean
            z = rng.normal(size=m)
            observation_maps.append(obs_loading)
            observation_means.append(obs_mean)
            measurements.append(z)
            state = _update(state, z, current_h, r_root @ r_root.T).state
            expected_mean, expected_p = _condition_latent(
                mean, loading, np.concatenate(observation_means),
                np.vstack(observation_maps), np.concatenate(measurements))
            np.testing.assert_allclose(state.mean, expected_mean, rtol=2e-12, atol=3e-13)
            np.testing.assert_allclose(state.covariance, expected_p, rtol=3e-11, atol=3e-13)
        state = drop_clone(state, "window" + str(window))
        loading, mean = loading[:20], mean[:20]
        expected_mean, expected_p = _condition_latent(
            mean, loading, np.concatenate(observation_means),
            np.vstack(observation_maps), np.concatenate(measurements))
        np.testing.assert_allclose(state.mean, expected_mean, rtol=2e-12, atol=3e-13)
        np.testing.assert_allclose(state.covariance, expected_p, rtol=3e-11, atol=3e-13)
    assert state.dimension == 20
    assert state.timestamp_ns == 130
    assert state.covariance[19, 19] < initial_root[19] @ initial_root[19]


def test_entire_chain_is_equivariant_to_mixed_state_and_measurement_units():
    original, _, _ = _initial(104)
    units = np.diag(np.r_[np.logspace(-6, 6, 19), 1e-3])
    inverse = np.diag(1.0 / np.diag(units))
    scaled = create_joint_state((units @ original.mean)[:19],
                                (units @ original.covariance @ units)[:19, :19],
                                (units @ original.mean)[19:],
                                (units @ original.covariance @ units)[19:, 19:],
                                (units @ original.covariance @ units)[:19, 19:], timestamp_ns=100)
    original, scaled = clone_pose(original, "c"), clone_pose(scaled, "c")
    whole_units = np.diag(np.r_[np.diag(units), np.diag(units)[:6]])
    whole_inverse = np.diag(1.0 / np.diag(whole_units))
    active_units, active_inverse = units[:19, :19], inverse[:19, :19]
    f = np.eye(19); f[0, 7] = 0.2; f[7, 13] = -0.2
    q = np.eye(19) * 0.001
    offset = np.linspace(-0.01, 0.01, 19)
    original = _propagate(original, f, q, offset)
    scaled = _propagate(scaled, active_units @ f @ active_inverse,
                        active_units @ q @ active_units, active_units @ offset)
    h = np.zeros((2, 26)); h[0, 0] = 1; h[0, 20] = -1; h[0, 19] = 0.3
    h[1, 3] = 1; h[1, 23] = -1; h[1, 19] = -0.1
    measurement_units = np.diag([1e-4, 1e4])
    r, z = np.diag([0.03, 0.06]), np.array([0.2, -0.4])
    original = _update(original, z, h, r).state
    scaled = _update(scaled, measurement_units @ z, measurement_units @ h @ whole_inverse,
                     measurement_units @ r @ measurement_units).state
    np.testing.assert_allclose(whole_inverse @ scaled.mean, original.mean, rtol=1e-11, atol=1e-12)
    np.testing.assert_allclose(whole_inverse @ scaled.covariance @ whole_inverse,
                               original.covariance, rtol=1e-10, atol=1e-12)
    original, scaled = drop_clone(original, "c"), drop_clone(scaled, "c")
    np.testing.assert_allclose(inverse @ scaled.covariance @ inverse, original.covariance,
                               rtol=1e-10, atol=1e-12)


def test_singular_zero_noise_prior_and_exact_measurement_need_no_jitter():
    state = create_joint_state(np.zeros(19), np.eye(19), [0.0], [[0.0]], np.zeros((19, 1)),
                               timestamp_ns=0)
    state = clone_pose(state, "zero")
    state = _propagate(state, np.eye(19), np.zeros((19, 19)))
    h = np.eye(26)[:1]
    state = _update(state, [2.0], h, [[0.0]]).state
    assert state.mean[0] == 2.0
    assert state.mean[20] == 2.0
    assert state.covariance[0, 0] == 0.0
    assert state.covariance[20, 20] == 0.0
    assert not state.to_dict()["diagnostics"]["jitter_added"]
    state = drop_clone(state, "zero")
    state = _propagate(state, np.eye(19), np.eye(19) * 0.01)
    assert state.covariance[0, 0] == 0.01


@pytest.mark.parametrize("field,value", [
    ("active_mean", [0.0] * 18),
    ("active_mean", [True] + [0.0] * 18),
    ("active_mean", [float("nan")] + [0.0] * 18),
    ("active_covariance", None),
    ("active_covariance", np.eye(18)),
    ("active_covariance", np.diag([-1e-30] + [1.0] * 18)),
    ("nuisance_covariance", None),
    ("nuisance_covariance", [[float("inf")]]),
    ("active_nuisance_cross_covariance", None),
    ("active_nuisance_cross_covariance", np.ones((19, 1)) * 2.0),
    ("timestamp_ns", True),
    ("timestamp_ns", 0.5),
    ("timestamp_ns", -1),
    ("correlation_tolerance", 0.0),
    ("correlation_tolerance", 1e-5),
    ("correlation_tolerance", True),
])
def test_invalid_initial_moments_or_metadata_are_rejected(field, value):
    arguments = dict(active_mean=np.zeros(19), active_covariance=np.eye(19), nuisance_mean=[0.0],
                     nuisance_covariance=[[1.0]], active_nuisance_cross_covariance=np.zeros((19, 1)),
                     timestamp_ns=0)
    arguments[field] = value
    with pytest.raises(ValueError):
        create_joint_state(**arguments)


def test_joint_psd_is_checked_after_individually_psd_blocks():
    cross = np.zeros((19, 1)); cross[0, 0] = 1.01
    with pytest.raises(ValueError, match="PSD"):
        create_joint_state(np.zeros(19), np.eye(19), [0.0], [[1.0]], cross, timestamp_ns=0)
    with pytest.raises(ValueError, match="zero variance"):
        create_joint_state(np.zeros(19), np.eye(19), [0.0], [[0.0]], cross, timestamp_ns=0)


@pytest.mark.parametrize("bad_covariance", [
    np.eye(19) + np.diag([0.1] * 18, 1),
    np.block([[np.array([[1.0, 1.0001], [1.0001, 1.0]]), np.zeros((2, 17))],
              [np.zeros((17, 2)), np.eye(17)]]),
])
def test_asymmetric_or_indefinite_process_noise_is_rejected(bad_covariance):
    with pytest.raises(ValueError):
        _propagate(_initial()[0], np.eye(19), bad_covariance)


@pytest.mark.parametrize("flag", [False, None, 1, "independent"])
def test_process_and_measurement_independence_must_be_explicit_bool_true(flag):
    state = _initial()[0]
    with pytest.raises(ValueError, match="independent_process_noise"):
        propagate_active(state, np.eye(19), np.eye(19), np.zeros(19), next_timestamp_ns=101,
                         independent_process_noise=flag)
    with pytest.raises(ValueError, match="independent_measurement_noise"):
        linear_update(state, [0.0], np.eye(20)[:1], [[1.0]], measurement_timestamp_ns=100,
                      independent_measurement_noise=flag)


def test_missing_independence_cannot_default_to_zero_cross():
    state = _initial()[0]
    with pytest.raises(TypeError):
        propagate_active(state, np.eye(19), np.eye(19), np.zeros(19), next_timestamp_ns=101)
    with pytest.raises(TypeError):
        linear_update(state, [0.0], np.eye(20)[:1], [[1.0]], measurement_timestamp_ns=100)


@pytest.mark.parametrize("stamp", [100, 99, -1, 100.5, True])
def test_propagation_requires_strict_forward_model_time(stamp):
    with pytest.raises(ValueError):
        propagate_active(_initial()[0], np.eye(19), np.eye(19), np.zeros(19),
                         next_timestamp_ns=stamp, independent_process_noise=True)


@pytest.mark.parametrize("stamp", [99, 101, 100.0, True])
def test_measurement_requires_current_integer_model_time(stamp):
    with pytest.raises(ValueError):
        linear_update(_initial()[0], [0.0], np.eye(20)[:1], [[1.0]],
                      measurement_timestamp_ns=stamp, independent_measurement_noise=True)


def test_duplicate_unknown_and_malformed_clone_ids_are_rejected():
    state = clone_pose(_initial()[0], "one")
    for name in ("one", "", " bad", 1, None):
        with pytest.raises(ValueError):
            clone_pose(state, name)
    for operation in (drop_clone, lambda s, name: s.clone_indices(name),
                      lambda s, name: s.clone_timestamp_ns(name)):
        with pytest.raises(ValueError):
            operation(state, "missing")


def test_unknown_covariance_and_invalid_observations_are_not_filled():
    state = _initial()[0]
    with pytest.raises(ValueError):
        _propagate(state, np.eye(19), None)
    with pytest.raises(ValueError):
        _propagate(state, np.eye(19), np.eye(19), offset=np.ones(18))
    for z, h, r in (([0.0], np.eye(20)[:1], None),
                    ([0.0], np.eye(19)[:1], [[1.0]]),
                    ([True], np.eye(20)[:1], [[1.0]]),
                    ([float("nan")], np.eye(20)[:1], [[1.0]]),
                    ([0.0], np.eye(20)[:1], [[-0.1]]),
                    ([], np.empty((0, 20)), np.empty((0, 0)))):
        with pytest.raises(ValueError):
            _update(state, z, h, r)


def test_redundant_zero_noise_observations_are_rejected_without_jitter():
    state = _initial()[0]
    h = np.vstack((np.eye(20)[0], np.eye(20)[0]))
    with pytest.raises(ValueError, match="positive definite"):
        _update(state, [0.0, 0.0], h, np.zeros((2, 2)))


def test_finite_inputs_that_overflow_propagation_are_rejected():
    with pytest.raises(ValueError):
        _propagate(_initial()[0], np.eye(19) * 1e308, np.zeros((19, 19)))
