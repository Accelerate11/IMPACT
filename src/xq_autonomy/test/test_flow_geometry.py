"""Independent rigid-motion/projective oracles for flow geometry contracts."""

import math
from unittest.mock import patch

import cv2
import numpy as np
import pytest

from xq_autonomy.downward_flow import BODY_FROM_DOWN_OPTICAL, CameraModel
from xq_autonomy.flow_geometry import (
    compensated_noise_statistics,
    extract_raw_tracks,
    point_interaction_matrices,
    predict_track_displacement,
    rotation_compensation,
)


def quaternion(rotation_vector):
    """Independent axis-angle quaternion used to move the physical camera."""
    vector = np.asarray(rotation_vector, dtype=float)
    angle = math.sqrt(float(vector @ vector))
    return np.r_[vector * (math.sin(angle / 2) / angle if angle else 0.5), math.cos(angle / 2)]


def rotate(q, vector):
    # Quaternion vector rotation, without the production Rodrigues/Jacobian code.
    vector = np.asarray(vector)
    return vector + 2.0 * np.cross(q[:3], np.cross(q[:3], vector) + q[3] * vector)


def camera_rotation(vector):
    q = quaternion(vector)
    return np.column_stack([rotate(q, axis) for axis in np.eye(3)])


def project_static_world_point(xy, depth, velocity, gamma, dt, rotation_bc, lever):
    """Put a point in the world, move body+camera, then project the same point."""
    world_point = np.asarray(lever) + np.asarray(rotation_bc) @ (depth * np.r_[xy, 1.0])
    body_rotation = quaternion(gamma)
    world_camera_origin = np.asarray(velocity) * dt + rotate(body_rotation, lever)
    relative_world = world_point - world_camera_origin
    inverse_rotation = np.r_[-body_rotation[:3], body_rotation[3]]
    camera_point = np.asarray(rotation_bc).T @ rotate(inverse_rotation, relative_world)
    assert camera_point[2] > 0
    return camera_point[:2] / camera_point[2] - xy


def finite_difference(function, value, step=2.0e-6):
    value = np.asarray(value, dtype=float)
    output = []
    for axis in range(len(value)):
        delta = np.zeros_like(value)
        delta[axis] = step
        output.append((function(value + delta) - function(value - delta)) / (2 * step))
    return np.column_stack(output)


@pytest.mark.parametrize("xy,depth", [([0.0, 0.0], 0.3), ([0.31, -0.24], 1.7), ([-0.4, 0.2], 8.0)])
def test_instantaneous_interaction_from_complete_static_point_projection(xy, depth):
    xy = np.asarray(xy)
    result = point_interaction_matrices(xy, depth)
    zeros, identity = np.zeros(3), np.eye(3)
    translation_oracle = lambda velocity: project_static_world_point(xy, depth, velocity, zeros, 1.0, identity, zeros)
    rotation_oracle = lambda gamma: project_static_world_point(xy, depth, zeros, gamma, 1.0, identity, zeros)
    np.testing.assert_allclose(result["A"], finite_difference(translation_oracle, zeros), rtol=1e-8, atol=2e-9)
    np.testing.assert_allclose(result["B"], finite_difference(rotation_oracle, zeros), rtol=1e-8, atol=2e-9)
    # A static point moves left for camera +x and downward for camera -y.
    assert translation_oracle([1e-3, 0, 0])[0] < 0
    assert translation_oracle([0, -1e-3, 0])[1] > 0
    if xy[0]:
        assert translation_oracle([0, 0, 1e-3])[0] * xy[0] > 0


@pytest.mark.parametrize("gamma", [[0, 0, 0], [2e-9, -3e-9, 1e-9], [0.00033, -0.00021, 0.00014], [0.23, -0.16, 0.11]])
def test_exact_rotation_and_jacobian_use_world_reprojection_oracle(gamma):
    xy = np.asarray([-0.29, 0.37])
    result = rotation_compensation(xy, gamma)
    oracle = lambda rotation: project_static_world_point(xy, 3.0, np.zeros(3), rotation, 1.0, np.eye(3), np.zeros(3))
    np.testing.assert_allclose(result["normalized_displacement"], oracle(np.asarray(gamma)), rtol=0, atol=1e-15)
    np.testing.assert_allclose(result["jacobian_gamma"], finite_difference(oracle, gamma), rtol=2e-8, atol=2e-9)
    assert result["covariance"] is None and result["uncertainty_known"] is False


def test_nonzero_rotation_compensation_is_not_a_sine_approximation():
    angle = 0.1
    # For the centre ray and a camera +y rotation, x_new = -tan(angle).
    predicted = rotation_compensation([0, 0], [0, angle, 0])["normalized_displacement"]
    assert predicted[0] == pytest.approx(-math.tan(angle), abs=2e-16)
    assert abs(predicted[0] + math.sin(angle)) > 1e-4


@pytest.mark.parametrize("rotation_bc", [np.eye(3), BODY_FROM_DOWN_OPTICAL, camera_rotation([0.2, -0.4, 0.3])])
def test_finite_motion_lever_arm_and_all_jacobians_against_physical_camera(rotation_bc):
    xy, depth, dt = np.asarray([0.27, -0.19]), 2.1, 0.066
    velocity, gamma, lever = np.asarray([0.6, -0.4, 0.18]), np.asarray([0.12, -0.08, 0.07]), np.asarray([0.17, -0.09, -0.03])
    result = predict_track_displacement(xy, depth, velocity, gamma, dt, rotation_bc, lever)
    oracle = lambda v, g, z: project_static_world_point(xy, z, v, g, dt, rotation_bc, lever)
    np.testing.assert_allclose(result["normalized_displacement"], oracle(velocity, gamma, depth), rtol=1e-12, atol=3e-16)
    np.testing.assert_allclose(result["jacobian_velocity_body"], finite_difference(lambda v: oracle(v, gamma, depth), velocity), rtol=2e-8, atol=1e-9)
    np.testing.assert_allclose(result["jacobian_rotation_increment_body"], finite_difference(lambda g: oracle(velocity, g, depth), gamma), rtol=2e-8, atol=1e-9)
    numerical_depth = (oracle(velocity, gamma, depth + 2e-6) - oracle(velocity, gamma, depth - 2e-6)) / 4e-6
    np.testing.assert_allclose(result["jacobian_depth_m"], numerical_depth, rtol=2e-8, atol=1e-9)
    no_lever = predict_track_displacement(xy, depth, velocity, gamma, dt, rotation_bc, np.zeros(3))
    assert np.linalg.norm(np.asarray(result["normalized_displacement"]) - no_lever["normalized_displacement"]) > 1e-4
    same_motion = predict_track_displacement(xy, depth, velocity / 2, gamma, dt * 2, rotation_bc, lever)
    np.testing.assert_allclose(same_motion["normalized_displacement"], result["normalized_displacement"], atol=1e-16)
    np.testing.assert_allclose(same_motion["jacobian_velocity_body"], np.asarray(result["jacobian_velocity_body"]) * 2, atol=1e-16)
    assert result["metric_velocity_estimated"] is False and result["covariance"] is None


def test_stationary_camera_and_pure_lever_arm_rotation():
    xy, depth, lever = np.asarray([0.1, 0.2]), 1.0, np.asarray([0.2, 0, 0])
    stationary = predict_track_displacement(xy, depth, [0, 0, 0], [0, 0, 0], 0.1, np.eye(3), lever)
    np.testing.assert_allclose(stationary["normalized_displacement"], [0, 0], rtol=0, atol=1e-15)
    moved = predict_track_displacement([0, 0], depth, [0, 0, 0], [0, 0, 0.1], 0.1, np.eye(3), lever)
    assert np.linalg.norm(moved["normalized_displacement"]) > 0.01
    # A rotation about optical z cannot move the centre ray without a lever arm.
    np.testing.assert_array_equal(rotation_compensation([0, 0], [0, 0, 0.1])["normalized_displacement"], [0, 0])


@pytest.mark.parametrize("depth", [None, False, 0, -1, float("nan"), float("inf"), "1"])
def test_missing_or_invalid_depth_fails_closed(depth):
    with pytest.raises(ValueError):
        point_interaction_matrices([0, 0], depth)
    with pytest.raises(ValueError):
        predict_track_displacement([0, 0], depth, [0, 0, 0], [0, 0, 0], 0.1, np.eye(3), [0, 0, 0])


def test_rotation_projection_dt_and_coordinate_boundaries():
    for xy, gamma in [([np.nan, 0], [0, 0, 0]), ([True, 0], [0, 0, 0]), ([0, 0], [math.pi, 0, 0])]:
        with pytest.raises(ValueError):
            rotation_compensation(xy, gamma)
    with pytest.raises(ValueError):
        predict_track_displacement([0, 0], 1, [0, 0, 0], [0, 0, 0], 0, np.eye(3), [0, 0, 0])
    with pytest.raises(ValueError):
        predict_track_displacement([0, 0], 1, [0, 0, 0], [0, 0, 0], 0.1, np.diag([1, 1, -1]), [0, 0, 0])


@pytest.fixture
def camera():
    return CameraModel(320, 240, 277.128129211, 277.128129211, 159.5, 119.5)


def textured_rgb(camera):
    gray = np.zeros((camera.height, camera.width), np.uint8)
    for row in range(20, camera.height - 20, 24):
        for column in range(20, camera.width - 20, 24):
            cv2.rectangle(gray, (column, row), (column + 7, row + 7), 255, -1)
    return np.repeat(gray[:, :, None], 3, axis=2)


def test_real_rgb_pair_extracts_uncompressed_raw_tracks_without_imu(camera):
    before = textured_rgb(camera)
    after = cv2.warpAffine(before, np.asarray([[1., 0., 3.], [0., 1., -2.]]), (camera.width, camera.height))
    originals = before.copy(), after.copy()
    result = extract_raw_tracks(before, after, camera, 1_000_000_000, 1_066_000_000)
    assert result["valid"] is True and result["track_count"] >= 12
    assert result["track_count"] == len(result["tracks"])
    assert result["covariance"] is None and result["uncertainty_known"] is False
    assert result["gyro_used"] is False and result["selection_dependencies"] == ["down_camera"]
    assert result["metric_velocity_valid"] is False and result["metric_velocity_mps"] is None
    assert result["acquisition_start_ns"] == 1_000_000_000 and result["acquisition_end_ns"] == 1_066_000_000
    assert len({track["track_id"] for track in result["tracks"]}) == result["track_count"]
    errors = []
    for track in result["tracks"]:
        old, new = np.asarray(track["old_pixel"]), np.asarray(track["new_pixel"])
        errors.append(np.linalg.norm(new - old - [3, -2]))
        np.testing.assert_allclose(track["raw_normalized_displacement"], (new - old) / [camera.fx, camera.fy], atol=1e-16)
        np.testing.assert_allclose(np.asarray(track["new_normalized_ray"])[:2] - np.asarray(track["old_normalized_ray"])[:2], track["raw_normalized_displacement"], atol=1e-16)
        assert track["forward_backward_error_px"] <= 1 and track["covariance"] is None
    assert np.quantile(errors, 0.95) < 0.1
    np.testing.assert_array_equal(before, originals[0])
    np.testing.assert_array_equal(after, originals[1])


def test_backward_failure_is_rejected_without_any_gyro_gate(camera):
    features = np.asarray([[[30., 30.]], [[60., 60.]], [[90., 90.]]], dtype=np.float32)
    forward = features + [2, 1]
    returned = features.copy()
    returned[1, 0, 0] += 4
    status = np.ones((3, 1), dtype=np.uint8)
    image = textured_rgb(camera)
    with patch("xq_autonomy.flow_geometry.cv2.goodFeaturesToTrack", return_value=features), patch(
        "xq_autonomy.flow_geometry.cv2.calcOpticalFlowPyrLK", side_effect=[(forward.astype(np.float32), status, None), (returned, status, None)]
    ):
        result = extract_raw_tracks(image, image, camera, 0, 66_000_000, minimum_tracks=1, maximum_tracks=3)
    assert [row["track_id"] for row in result["tracks"]] == [0, 2]
    assert result["detected_track_count"] == 3 and result["track_count"] == 2


def test_frontend_invalid_data_has_no_invented_tracks_or_covariance(camera):
    image = textured_rgb(camera)
    for before, after, start, end in [(image, image, 0, 0), (image, image, 0, 200_000_000), (image[:, :, 0], image, 0, 66_000_000), (image.astype(float), image, 0, 66_000_000)]:
        result = extract_raw_tracks(before, after, camera, start, end)
        assert result["valid"] is False and result["tracks"] == [] and result["track_count"] == 0
        assert result["covariance"] is None
    blank = np.zeros_like(image)
    assert extract_raw_tracks(blank, blank, camera, 0, 66_000_000)["reason"] == "INSUFFICIENT_FEATURES"
    with pytest.raises(ValueError):
        extract_raw_tracks(image, image, camera, True, 66_000_000)


def known_covariances():
    rng = np.random.default_rng(2731)
    loading = rng.normal(size=(7, 7)) * .02
    joint = loading @ loading.T + np.eye(7) * .001
    j = np.vstack([rotation_compensation(xy, [0.03, -0.01, .02])["jacobian_gamma"] for xy in ([0, 0], [.2, -.3], [-.1, .2])])
    pixel = np.eye(6) * .0001
    return j, pixel, joint[4:, 4:], joint[:4, 4:], joint[:4, :4]


def test_stacked_shared_gyro_noise_matches_full_joint_linear_transform():
    j, pixel, gyro, cross, prior = known_covariances()
    result = compensated_noise_statistics(j, pixel, gyro, cross, prior, pixel_noise_independent=True)
    full = np.block([[prior, cross, np.zeros((4, 6))], [cross.T, gyro, np.zeros((3, 6))], [np.zeros((6, 4)), np.zeros((6, 3)), pixel]])
    transform = np.block([[np.eye(4), np.zeros((4, 3)), np.zeros((4, 6))], [np.zeros((6, 4)), -j, np.eye(6)]])
    expected = transform @ full @ transform.T
    np.testing.assert_allclose(result["R_effective"], expected[4:, 4:], rtol=2e-15, atol=1e-17)
    np.testing.assert_allclose(result["C_effective"], expected[:4, 4:], rtol=2e-15, atol=1e-17)
    assert np.linalg.norm(np.asarray(result["R_effective"])[:2, 2:4]) > 1e-5
    assert result["hardware_validated"] is False and result["statistical_calibration_validated"] is False


def test_unknown_statistics_are_not_zero_cross_covariance():
    known = list(known_covariances())
    for index in range(1, 5):
        arguments = known.copy()
        arguments[index] = None
        result = compensated_noise_statistics(*arguments, pixel_noise_independent=True)
        assert result["status"] == "UNAVAILABLE" and result["R_effective"] is None and result["C_effective"] is None
    result = compensated_noise_statistics(*known, pixel_noise_independent=False)
    assert result["status"] == "UNAVAILABLE" and "CROSS_COVARIANCES_REQUIRED" in result["reason"]


@pytest.mark.parametrize("kind", ["invalid_joint", "nonsymmetric", "negative_variance", "zero_axis_cross", "nonfinite", "bool"])
def test_invalid_joint_noise_models_are_rejected(kind):
    j, pixel, gyro, cross, prior = known_covariances()
    if kind == "invalid_joint": cross[:] = 10
    elif kind == "nonsymmetric": gyro[0, 1] += 1e-4
    elif kind == "negative_variance": pixel[0, 0] = -1
    elif kind == "zero_axis_cross": gyro[0, 0] = 0
    elif kind == "nonfinite": j[0, 0] = np.nan
    elif kind == "bool": prior = np.eye(4, dtype=bool)
    with pytest.raises(ValueError):
        compensated_noise_statistics(j, pixel, gyro, cross, prior, pixel_noise_independent=True)
