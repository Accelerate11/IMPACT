"""Independent quaternion/static-world-point tests for conditional geometry."""

import math

import numpy as np
import pytest

from xq_autonomy.two_view_geometry import integrate_gyro_midpoint, predict_two_view


def quaternion(phi):
    phi = np.asarray(phi, dtype=float)
    norm = math.sqrt(float(phi @ phi))
    return np.r_[phi * (math.sin(norm / 2) / norm if norm else 0.5), math.cos(norm / 2)]


def multiply(left, right):
    return np.r_[left[3] * right[:3] + right[3] * left[:3] + np.cross(left[:3], right[:3]),
                 left[3] * right[3] - float(left[:3] @ right[:3])]


def rotate(q, vector):
    vector = np.asarray(vector)
    return vector + 2 * np.cross(q[:3], np.cross(q[:3], vector) + q[3] * vector)


def rotation(q):
    return np.column_stack([rotate(q, basis) for basis in np.eye(3)])


def physical_projection(xy, depth, q0, p0, q1, p1, qbc, lever):
    # Construct the static point and actual two camera origins in world space.
    world_camera0 = np.asarray(p0) + rotate(q0, lever)
    world_point = world_camera0 + rotate(multiply(q0, qbc), depth * np.r_[xy, 1.0])
    world_camera1 = np.asarray(p1) + rotate(q1, lever)
    world_to_camera1 = multiply(q1, qbc).copy()
    world_to_camera1[:3] *= -1
    point = rotate(world_to_camera1, world_point - world_camera1)
    assert point[2] > 0
    return point[:2] / point[2]


def integrate(stamps, rates, start, end, bias, initial=None, gap=100_000_000):
    return integrate_gyro_midpoint(
        stamps, rates, start, end, bias,
        initial_world_from_body_rotation=np.eye(3) if initial is None else initial,
        maximum_gap_ns=gap,
    )


def test_constant_axis_bias_is_removed_before_integration():
    stamps = [10, 50_000_010, 100_000_010, 150_000_010]
    true_rate, bias = np.asarray([0.3, -0.2, 0.4]), np.asarray([0.02, -0.04, 0.01])
    initial_q = quaternion([0.2, -0.1, 0.05])
    result = integrate(stamps, np.tile(true_rate + bias, (4, 1)), stamps[0], stamps[-1], bias,
                       rotation(initial_q))
    expected = multiply(initial_q, quaternion(true_rate * .15))
    np.testing.assert_allclose(result["world_from_body_rotation"], rotation(expected), atol=3e-16, rtol=0)
    np.testing.assert_allclose(result["relative_body_rotation"], rotation(quaternion(true_rate * .15)), atol=3e-16)
    assert result["duration_ns"] == 150_000_000 and result["complete_window"]
    assert result["covariance"] is None and not result["uncertainty_known"]
    assert not result["sensor_bias_calibrated"] and not result["fusion_update_performed"]


def test_clipped_piecewise_linear_midpoints_use_integer_differences():
    epoch = 1_792_425_600_123_456_789
    offsets = [0, 50_000_000, 100_000_000, 150_000_000]
    rates = np.asarray([[.4, .0, .1], [.0, .7, -.1], [-.2, .0, .9], [.1, -.3, .2]])
    bias = [.03, -.02, .04]
    zero = integrate(offsets, rates, 13_000_001, 128_000_003, bias)
    epoch_result = integrate([epoch + value for value in offsets], rates,
                             epoch + 13_000_001, epoch + 128_000_003, bias)
    np.testing.assert_array_equal(zero["world_from_body_rotation"], epoch_result["world_from_body_rotation"])
    expected_q = np.asarray([0., 0., 0., 1.])
    for index, (left, right) in enumerate(zip(offsets, offsets[1:])):
        begin, end = max(left, 13_000_001), min(right, 128_000_003)
        fraction = ((begin - left) + (end - left)) / (2 * (right - left))
        omega = rates[index] * (1 - fraction) + rates[index + 1] * fraction - bias
        expected_q = multiply(expected_q, quaternion(omega * ((end - begin) / 1_000_000_000)))
    np.testing.assert_allclose(zero["relative_body_rotation"], rotation(expected_q), atol=4e-16)
    assert [row["duration_ns"] for row in zero["segments"]] == [36_999_999, 50_000_000, 28_000_003]


def test_one_nanosecond_window_remains_visible_at_large_epoch():
    epoch = 1_792_425_600_123_456_789
    result = integrate([epoch, epoch + 1, epoch + 2], [[0, 0, 1e6]] * 3,
                       epoch, epoch + 1, [0, 0, 0], gap=1)
    np.testing.assert_allclose(result["relative_body_rotation"], rotation(quaternion([0, 0, .001])), atol=2e-16)
    assert result["duration_ns"] == 1


def test_alternating_axes_require_time_ordered_products():
    # Segment midpoints are x=2 rad/s then y=2 rad/s, for 0.1 s each.
    rates = np.asarray([[4., 0, 0], [0, 0, 0], [0, 4., 0]])
    result = integrate([0, 100_000_000, 200_000_000], rates, 0, 200_000_000,
                       [0, 0, 0])
    expected = multiply(quaternion([.2, 0, 0]), quaternion([0, .2, 0]))
    np.testing.assert_allclose(result["relative_body_rotation"], rotation(expected), atol=3e-16)
    collapsed = rotation(quaternion([.2, .2, 0]))
    assert np.max(np.abs(np.asarray(result["relative_body_rotation"]) - collapsed)) > 1e-3
    reversed_result = integrate([0, 100_000_000, 200_000_000], rates[::-1], 0, 200_000_000,
                                [0, 0, 0])
    assert np.linalg.norm(np.asarray(result["relative_body_rotation"]) - reversed_result["relative_body_rotation"]) > .03


def test_bias_jacobian_matches_independent_quaternion_perturbations():
    stamps = [0, 30_000_000, 75_000_000, 140_000_000]
    rates = np.asarray([[.3, -.1, .0], [-.2, .9, .3], [.8, .1, -.4], [-.1, .5, .7]])
    bias = np.asarray([.04, -.02, .01])
    result = integrate(stamps, rates, 0, stamps[-1], bias)

    def independent_rotation(changed_bias):
        q = np.asarray([0., 0., 0., 1.])
        for index in range(3):
            phi = ((rates[index] + rates[index + 1]) / 2 - changed_bias) * ((stamps[index + 1] - stamps[index]) / 1e9)
            q = multiply(q, quaternion(phi))
        return rotation(q)

    nominal = independent_rotation(bias)
    numerical = np.empty((3, 3))
    for axis in range(3):
        delta = np.eye(3)[axis] * 2e-6
        relative_derivative = nominal.T @ ((independent_rotation(bias + delta) - independent_rotation(bias - delta)) / 4e-6)
        numerical[:, axis] = [relative_derivative[2, 1], relative_derivative[0, 2], relative_derivative[1, 0]]
    np.testing.assert_allclose(result["rotation_jacobian_bias"], numerical, atol=2e-10, rtol=1e-8)


@pytest.mark.parametrize("stamps,start,end,gap", [
    ([1, 11], 0, 10, 10), ([0, 9], 0, 10, 10),
    ([0, 11], 2, 8, 10), ([0, 10, 10], 0, 10, 10),
    ([0, 20, 10], 0, 10, 20), ([0], 0, 1, 10),
    ([0., 10.], 0, 10, 10), ([False, 10], 0, 10, 10),
    ([0, 10], 0.0, 10, 10), ([0, 10], 0, 0, 10),
    ([0, 10], 0, 10, 0), ([0, 10], 0, 10, True),
])
def test_unknown_or_uncovered_time_series_fails_closed(stamps, start, end, gap):
    with pytest.raises(ValueError):
        integrate(stamps, [[0, 0, 0]] * len(stamps), start, end, [0, 0, 0], gap=gap)


@pytest.mark.parametrize("bias", [None, [0, 0], [True, 0, 0], [0, np.nan, 0]])
def test_unknown_bias_is_not_silently_replaced_with_zero(bias):
    with pytest.raises(ValueError):
        integrate([0, 10], [[0, 0, 0]] * 2, 0, 10, bias)


def test_outside_window_gap_does_not_hide_valid_fully_covered_window():
    result = integrate([0, 1000, 1010, 1020], [[0, 0, .2]] * 4,
                       1000, 1020, [0, 0, 0], gap=10)
    assert len(result["segments"]) == 2
    with pytest.raises(ValueError):
        integrate([0, 10], [[np.nan, 0, 0]] * 2, 0, 10, [0, 0, 0])
    with pytest.raises(ValueError):
        integrate([0, 10], [[0, 0, 0]] * 2, 0, 10, [0, 0, 0], np.diag([1., 1., -1.]))


@pytest.mark.parametrize("seed", [301, 530, 901, 17019])
def test_static_world_point_and_all_pose_depth_derivatives(seed):
    rng = np.random.default_rng(seed)
    q0, q1, qbc = [quaternion(rng.normal(size=3) * .13) for _ in range(3)]
    p0, p1 = rng.normal(size=(2, 3)) * .15
    lever = rng.normal(size=3) * .12
    xy, depth = rng.uniform(-.35, .35, 2), rng.uniform(1.1, 4.)
    result = predict_two_view(xy, depth, rotation(q0), p0, rotation(q1), p1, rotation(qbc), lever)
    expected = physical_projection(xy, depth, q0, p0, q1, p1, qbc, lever)
    np.testing.assert_allclose(result["new_normalized_ray"], [*expected, 1.0], rtol=0, atol=2e-15)
    np.testing.assert_allclose(result["raw_normalized_displacement"], expected - xy, rtol=0, atol=2e-15)

    def perturbed(delta):
        dq0, dq1 = quaternion(delta[:3]), quaternion(delta[6:9])
        return physical_projection(xy, depth + delta[12], multiply(q0, dq0), p0 + delta[3:6],
                                   multiply(q1, dq1), p1 + delta[9:12], qbc, lever)

    numerical = np.column_stack([(perturbed(np.eye(13)[axis] * 2e-6) - perturbed(-np.eye(13)[axis] * 2e-6)) / 4e-6
                                for axis in range(13)])
    analytic = np.column_stack([result["jacobian_start_pose"], result["jacobian_end_pose"], result["jacobian_depth_m"]])
    np.testing.assert_allclose(analytic, numerical, atol=3e-10, rtol=2e-8)
    numerical_xy = np.column_stack([
        (physical_projection(xy + np.eye(2)[axis] * 2e-6, depth, q0, p0, q1, p1, qbc, lever)
         - physical_projection(xy - np.eye(2)[axis] * 2e-6, depth, q0, p0, q1, p1, qbc, lever)) / 4e-6
        for axis in range(2)
    ])
    np.testing.assert_allclose(result["jacobian_old_normalized_xy"], numerical_xy, atol=3e-10, rtol=2e-8)
    np.testing.assert_allclose(result["jacobian_raw_displacement_old_normalized_xy"], numerical_xy - np.eye(2), atol=3e-10)
    assert result["covariance"] is None and not result["uncertainty_known"]
    assert not result["depth_estimated"] and not result["metric_velocity_estimated"]
    assert not result["fusion_update_performed"]


def test_geometry_is_invariant_to_rigid_world_frame_change():
    q0, q1, qbc, qworld = [quaternion(vector) for vector in ([.2, -.1, .1], [-.1, .1, .2], [.1, .2, .3], [.7, -.3, .2])]
    p0, p1, offset = np.asarray([.1, .3, -.2]), np.asarray([.2, -.1, .4]), np.asarray([10., -4., 3.])
    args = [.2, -.3], 2.0
    a = predict_two_view(*args, rotation(q0), p0, rotation(q1), p1, rotation(qbc), [.1, .0, -.04])
    b = predict_two_view(*args, rotation(multiply(qworld, q0)), rotate(qworld, p0) + offset,
                         rotation(multiply(qworld, q1)), rotate(qworld, p1) + offset, rotation(qbc), [.1, .0, -.04])
    np.testing.assert_allclose(a["new_normalized_ray"], b["new_normalized_ray"], rtol=0, atol=2e-15)
    np.testing.assert_allclose(a["jacobian_start_pose"][0][:3], b["jacobian_start_pose"][0][:3], atol=2e-15)


def test_pure_translation_and_stationary_pose_have_expected_sign_and_scale():
    identity, zero = np.eye(3), np.zeros(3)
    result = predict_two_view([.1, -.2], 2., identity, zero, identity, [.2, -.1, 0], identity, zero)
    np.testing.assert_allclose(result["raw_normalized_displacement"], [-.1, .05], atol=2e-17)
    same = predict_two_view([.1, -.2], 2., identity, zero, identity, zero, identity, [.1, -.2, .03])
    np.testing.assert_allclose(same["raw_normalized_displacement"], [0, 0], atol=2e-15)
    np.testing.assert_allclose(same["jacobian_depth_m"], [0, 0], atol=2e-15)
    np.testing.assert_allclose(np.asarray(same["jacobian_start_pose"]) + same["jacobian_end_pose"], 0, atol=2e-15)


def test_pure_rotation_changes_with_lever_arm_and_preserves_no_lever_depth_independence():
    identity, zero = np.eye(3), np.zeros(3)
    r1 = rotation(quaternion([0., 0., .2]))
    a = predict_two_view([.1, .2], 1., identity, zero, r1, zero, identity, zero)
    b = predict_two_view([.1, .2], 8., identity, zero, r1, zero, identity, zero)
    np.testing.assert_allclose(a["raw_normalized_displacement"], b["raw_normalized_displacement"], atol=1e-16)
    with_lever = predict_two_view([.1, .2], 1., identity, zero, r1, zero, identity, [.2, .0, .0])
    assert np.linalg.norm(np.asarray(a["raw_normalized_displacement"]) - with_lever["raw_normalized_displacement"]) > .03


@pytest.mark.parametrize("depth", [None, False, 0, -1, float("nan"), float("inf"), "1"])
def test_unknown_depth_fails_closed(depth):
    with pytest.raises(ValueError):
        predict_two_view([0, 0], depth, np.eye(3), [0, 0, 0], np.eye(3), [0, 0, 0], np.eye(3), [0, 0, 0])


def test_reflection_nonfinite_pose_and_point_behind_camera_are_rejected():
    args = [[0, 0], 1., np.eye(3), [0, 0, 0], np.eye(3), [0, 0, 0], np.eye(3), [0, 0, 0]]
    for index, value in [(2, np.diag([1., 1., -1.])), (3, [0, 0, np.nan]), (6, np.eye(3) * 1.1),
                         (5, [0, 0, 2]), (0, [True, 0])]:
        changed = list(args)
        changed[index] = value
        with pytest.raises(ValueError):
            predict_two_view(*changed)


def test_inputs_are_not_modified():
    stamps = np.asarray([0, 50_000_000, 100_000_000], dtype=np.int64)
    rates = np.asarray([[.2, .1, 0], [.1, .0, -.1], [.3, .1, .2]])
    bias, initial = np.asarray([.02, .0, .0]), np.eye(3)
    snapshots = [value.copy() for value in (stamps, rates, bias, initial)]
    integrate(stamps, rates, stamps[0], stamps[-1], bias, initial)
    for original, copy in zip((stamps, rates, bias, initial), snapshots):
        np.testing.assert_array_equal(original, copy)
    xy, position, lever = np.asarray([.1, -.2]), np.asarray([.2, -.1, .3]), np.asarray([.1, .0, .02])
    snapshots = [value.copy() for value in (xy, position, lever)]
    predict_two_view(xy, 2., initial, position, initial, position, initial, lever)
    for original, copy in zip((xy, position, lever), snapshots):
        np.testing.assert_array_equal(original, copy)
