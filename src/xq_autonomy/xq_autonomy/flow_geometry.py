"""Pixel-only tracks and conditional optical-flow geometry, without ROS.

Image displacement means (du/fx, dv/fy) on the normalized image plane. It is
dimensionless, often approximated as radians, and is not a MAVLink body-axis
flow convention. No function infers metric velocity without explicit depth.
Covariances are unknown unless supplied under an explicit joint noise model.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from .downward_flow import CameraModel


def _has_bool(value) -> bool:
    if isinstance(value, (bool, np.bool_)):
        return True
    if isinstance(value, (list, tuple)):
        return any(_has_bool(item) for item in value)
    return isinstance(value, np.ndarray) and (
        value.dtype.kind == "b" or (value.dtype.kind == "O" and any(_has_bool(item) for item in value.flat))
    )


def _array(value, shape, name):
    try:
        raw = np.asarray(value)
        if _has_bool(value) or raw.dtype.kind not in "fiu":
            raise ValueError
        result = np.asarray(raw, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be numeric and finite") from error
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must have finite shape {shape}")
    return result


def _positive(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise ValueError(f"{name} must be finite and positive")
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be finite and positive") from error
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _skew(vector):
    x, y, z = vector
    return np.asarray(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))


def _exp_and_right_jacobian(rotation_vector):
    """SO(3) Exp(phi), J_r(phi); stable series retain small-angle accuracy."""
    angle = float(np.linalg.norm(rotation_vector))
    if not math.isfinite(angle):
        raise ValueError("rotation magnitude overflows")
    cross = _skew(rotation_vector)
    if angle < 1.0e-4:
        a2 = angle * angle
        sinc = 1.0 - a2 / 6.0 + a2 * a2 / 120.0
        cosc = 0.5 - a2 / 24.0 + a2 * a2 / 720.0
        third = 1.0 / 6.0 - a2 / 120.0 + a2 * a2 / 5040.0
    else:
        sinc = math.sin(angle) / angle
        cosc = (1.0 - math.cos(angle)) / (angle * angle)
        third = (angle - math.sin(angle)) / (angle * angle * angle)
    cross2 = cross @ cross
    return np.eye(3) + sinc * cross + cosc * cross2, np.eye(3) - cosc * cross + third * cross2


def _project(point):
    if not np.isfinite(point).all() or point[2] <= 1.0e-8:
        raise ValueError("projection requires a finite point in front of the camera")
    z = point[2]
    xy = point[:2] / z
    derivative = np.asarray(((1.0, 0.0, -xy[0]), (0.0, 1.0, -xy[1]))) / z
    if not np.isfinite(xy).all() or not np.isfinite(derivative).all():
        raise ValueError("projection overflows")
    return xy, derivative


def rotation_compensation(normalized_xy, gamma_optical_rad) -> dict:
    """Exact static-ray image displacement under Exp(-gamma), and dc/dgamma.

    gamma is an explicitly supplied optical-frame rotation increment. This
    function neither reads IMU data nor removes a gyro bias. For a later noise
    model, gamma_noise means gamma_measured - gamma_true; subtracting this
    displacement from raw flow gives measurement noise -J_gamma*gamma_noise.
    """
    xy = _array(normalized_xy, (2,), "normalized_xy")
    gamma = _array(gamma_optical_rad, (3,), "gamma_optical_rad")
    ray = np.r_[xy, 1.0]
    rotation, right_jacobian = _exp_and_right_jacobian(-gamma)
    projected, derivative = _project(rotation @ ray)
    jacobian = derivative @ rotation @ _skew(ray) @ right_jacobian
    return {
        "normalized_displacement": (projected - xy).tolist(),
        "jacobian_gamma": jacobian.tolist(),
        "measurement_space": "normalized_image_plane",
        "covariance": None,
        "uncertainty_known": False,
    }


def point_interaction_matrices(normalized_xy, depth_m) -> dict:
    """Instantaneous image rate A*v_camera + B*omega_camera.

    depth_m is optical-axis Z, not slant range or world height. A has units
    1/metre, B is dimensionless, and velocities/rates are in the optical frame.
    Missing depth is rejected rather than replaced by an assumed metre.
    """
    x, y = _array(normalized_xy, (2,), "normalized_xy")
    depth = _positive(depth_m, "depth_m")
    a = np.asarray(((-1.0, 0.0, x), (0.0, -1.0, y))) / depth
    b = np.asarray(((x * y, -(1.0 + x * x), y), (1.0 + y * y, -x * y, -x)))
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("interaction matrix overflows")
    return {"A": a.tolist(), "B": b.tolist(), "depth_definition": "optical_axis_Z_metres"}


def predict_track_displacement(normalized_xy, depth_m, velocity_body_mps,
                               rotation_increment_body_rad, dt_s,
                               body_from_optical_rotation, body_from_optical_translation_m) -> dict:
    """Finite SE(3) projection of one static point, including the camera lever arm.

    The IMU/body origin translates by velocity_body_mps*dt_s expressed in the
    INITIAL body frame, while its orientation changes by Exp(gamma_body).
    Translation is held constant in that initial inertial frame, not in the
    rotating body. Extrinsics map optical coordinates/origin into the body.
    This conditional geometric prediction does not estimate depth or velocity.
    """
    xy = _array(normalized_xy, (2,), "normalized_xy")
    depth = _positive(depth_m, "depth_m")
    dt = _positive(dt_s, "dt_s")
    velocity = _array(velocity_body_mps, (3,), "velocity_body_mps")
    gamma = _array(rotation_increment_body_rad, (3,), "rotation_increment_body_rad")
    rotation_bc = _array(body_from_optical_rotation, (3, 3), "body_from_optical_rotation")
    lever = _array(body_from_optical_translation_m, (3,), "body_from_optical_translation_m")
    if not np.allclose(rotation_bc.T @ rotation_bc, np.eye(3), rtol=0.0, atol=1.0e-10) or not math.isclose(np.linalg.det(rotation_bc), 1.0, abs_tol=1.0e-10):
        raise ValueError("body_from_optical_rotation must be SO(3)")
    ray = np.r_[xy, 1.0]
    rotation, right_jacobian = _exp_and_right_jacobian(-gamma)
    initial_point_minus_translation = rotation_bc @ (depth * ray) + lever - velocity * dt
    point = rotation_bc.T @ (rotation @ initial_point_minus_translation - lever)
    projected, derivative = _project(point)
    j_velocity = -dt * derivative @ rotation_bc.T @ rotation
    j_gamma = derivative @ rotation_bc.T @ rotation @ _skew(initial_point_minus_translation) @ right_jacobian
    j_depth = derivative @ rotation_bc.T @ rotation @ rotation_bc @ ray
    if not all(np.isfinite(item).all() for item in (j_velocity, j_gamma, j_depth)):
        raise ValueError("finite-motion Jacobian overflows")
    return {
        "normalized_displacement": (projected - xy).tolist(),
        "jacobian_velocity_body": j_velocity.tolist(),
        "jacobian_rotation_increment_body": j_gamma.tolist(),
        "jacobian_depth_m": j_depth.tolist(),
        "translation_definition": "velocity_times_dt_in_initial_body_frame",
        "depth_definition": "initial_optical_axis_Z_metres",
        "covariance": None,
        "uncertainty_known": False,
        "metric_velocity_estimated": False,
    }


def extract_raw_tracks(previous_rgb, current_rgb, camera: CameraModel, start_ns: int, end_ns: int,
                       *, minimum_tracks: int = 12, maximum_tracks: int = 150,
                       fb_threshold_px: float = 1.0) -> dict:
    """Pixel-only forward/backward LK; no gyro-dependent selection or median.

    IDs identify detected points within this image pair, not persistent feature
    identities across pairs. Covariance remains unknown even when tracks pass.
    """
    if not isinstance(camera, CameraModel):
        raise ValueError("camera must be an explicitly calibrated CameraModel")
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 0 for value in (start_ns, end_ns)):
        raise ValueError("acquisition stamps must be nonnegative integer nanoseconds")
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 1 for value in (minimum_tracks, maximum_tracks)) or minimum_tracks > maximum_tracks:
        raise ValueError("track limits must be positive integers with minimum <= maximum")
    threshold = _positive(fb_threshold_px, "fb_threshold_px")
    result = {
        "schema_version": 1, "acquisition_start_ns": start_ns, "acquisition_end_ns": end_ns,
        "source_image_stamps_ns": [start_ns, end_ns], "dt_s": (end_ns - start_ns) * 1.0e-9,
        "frame_id": camera.frame_id, "valid": False, "reason": "INVALID_IMAGE_INTERVAL",
        "tracks": [], "track_count": 0, "detected_track_count": 0,
        "measurement_space": "normalized_image_plane", "track_id_scope": "image_pair",
        "selection_dependencies": ["down_camera"], "gyro_used": False, "ground_truth_used": False,
        "covariance": None, "uncertainty_known": False, "metric_velocity_valid": False,
        "metric_velocity_mps": None, "scale_source": "unavailable",
    }
    if not 0 < end_ns - start_ns < 200_000_000:
        return result
    for image in (previous_rgb, current_rgb):
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.shape != (camera.height, camera.width, 3):
            result["reason"] = "INVALID_RGB_LAYOUT"
            return result
    previous = cv2.cvtColor(previous_rgb, cv2.COLOR_RGB2GRAY)
    current = cv2.cvtColor(current_rgb, cv2.COLOR_RGB2GRAY)
    features = cv2.goodFeaturesToTrack(previous, maxCorners=maximum_tracks, qualityLevel=0.01, minDistance=5.0, blockSize=5)
    result["detected_track_count"] = 0 if features is None else len(features)
    if features is None:
        result["reason"] = "INSUFFICIENT_FEATURES"
        return result
    options = dict(winSize=(21, 21), maxLevel=3,
                   criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    forward, status, _ = cv2.calcOpticalFlowPyrLK(previous, current, features, None, **options)
    if forward is None or status is None:
        result["reason"] = "LK_FORWARD_FAILED"
        return result
    old, new = features.reshape(-1, 2), forward.reshape(-1, 2)
    usable = (status.reshape(-1).astype(bool) & np.isfinite(new).all(axis=1)
              & (new[:, 0] >= 0) & (new[:, 0] < camera.width)
              & (new[:, 1] >= 0) & (new[:, 1] < camera.height))
    identifiers = np.flatnonzero(usable)
    if not len(identifiers):
        result["reason"] = "INSUFFICIENT_TRACKS"
        return result
    old, new = old[identifiers], new[identifiers]
    backward, status, _ = cv2.calcOpticalFlowPyrLK(current, previous, new.reshape(-1, 1, 2), None, **options)
    if backward is None or status is None:
        result["reason"] = "LK_BACKWARD_FAILED"
        return result
    returned = backward.reshape(-1, 2)
    errors = np.linalg.norm(returned - old, axis=1)
    focal, principal = np.asarray((camera.fx, camera.fy)), np.asarray((camera.cx, camera.cy))
    for index in np.flatnonzero(status.reshape(-1).astype(bool) & np.isfinite(returned).all(axis=1) & (errors <= threshold)):
        old_xy, new_xy = (old[index] - principal) / focal, (new[index] - principal) / focal
        result["tracks"].append({
            "track_id": int(identifiers[index]), "old_pixel": old[index].astype(float).tolist(),
            "new_pixel": new[index].astype(float).tolist(),
            "old_normalized_ray": [*old_xy.tolist(), 1.0], "new_normalized_ray": [*new_xy.tolist(), 1.0],
            "raw_normalized_displacement": (new_xy - old_xy).tolist(),
            "forward_backward_error_px": float(errors[index]),
            "jacobian_gamma_at_zero": rotation_compensation(old_xy, (0.0, 0.0, 0.0))["jacobian_gamma"],
            "covariance": None, "uncertainty_known": False,
        })
    result["track_count"] = len(result["tracks"])
    result["valid"] = result["track_count"] >= minimum_tracks
    result["reason"] = "OK_RAW_PIXEL_TRACKS_NO_SCALE_OR_COVARIANCE" if result["valid"] else "INSUFFICIENT_TRACKS"
    return result


def _covariance(value, size, name):
    matrix = _array(value, (size, size), name)
    diagonal = np.diag(matrix)
    if np.any(diagonal < 0.0):
        raise ValueError(f"{name} has a negative variance")
    for axis in np.flatnonzero(diagonal == 0.0):
        if np.any(matrix[axis, :] != 0.0) or np.any(matrix[:, axis] != 0.0):
            raise ValueError(f"{name} has nonzero covariance on a zero-variance axis")
    scales = np.sqrt(np.where(diagonal > 0.0, diagonal, 1.0))
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        balanced = matrix / scales[:, None] / scales[None, :]
    if not np.isfinite(balanced).all() or np.max(np.abs(balanced - balanced.T)) > 1.0e-9:
        raise ValueError(f"{name} is not symmetric in correlation coordinates")
    if np.linalg.eigvalsh((balanced + balanced.T) * 0.5).min() < -1.0e-9:
        raise ValueError(f"{name} is not PSD")
    return matrix


def compensated_noise_statistics(jacobian_gamma, pixel_covariance, gyro_integral_covariance,
                                 state_gyro_cross_covariance, state_covariance,
                                 *, pixel_noise_independent: bool) -> dict:
    """Assemble noise for raw - c(gamma_measured), with shared gyro errors.

    Pxgamma = Cov(state_true-state_prior, gamma_measured-gamma_true).
    Under explicit independence of pixel noise from BOTH state and gamma,
    v = pixel_noise - Jgamma*gamma_noise, Ceff=-Pxgamma*Jgamma.T.
    The full [state,gamma] covariance must be supplied and jointly PSD; zero
    cross covariance is accepted only when the caller explicitly supplies it.
    """
    raw = np.asarray(jacobian_gamma)
    if raw.ndim != 2 or raw.shape[1] != 3 or raw.shape[0] < 1:
        raise ValueError("jacobian_gamma must have shape (m, 3), m >= 1")
    jacobian = _array(jacobian_gamma, raw.shape, "jacobian_gamma")
    if type(pixel_noise_independent) is not bool:
        raise ValueError("pixel_noise_independent must be explicit bool")
    unavailable = {
        "status": "UNAVAILABLE", "reason": "COVARIANCE_UNKNOWN", "R_effective": None, "C_effective": None,
        "uncertainty_known": False, "hardware_validated": False, "statistical_calibration_validated": False,
    }
    if any(value is None for value in (pixel_covariance, gyro_integral_covariance, state_gyro_cross_covariance, state_covariance)):
        return unavailable
    if not pixel_noise_independent:
        return {**unavailable, "reason": "PIXEL_STATE_AND_GYRO_CROSS_COVARIANCES_REQUIRED"}
    raw_p = np.asarray(state_covariance)
    if raw_p.ndim != 2 or raw_p.shape[0] < 1 or raw_p.shape[0] != raw_p.shape[1]:
        raise ValueError("state_covariance must be nonempty square")
    dimension = raw_p.shape[0]
    prior = _covariance(state_covariance, dimension, "state_covariance")
    pixel = _covariance(pixel_covariance, jacobian.shape[0], "pixel_covariance")
    gyro = _covariance(gyro_integral_covariance, 3, "gyro_integral_covariance")
    cross = _array(state_gyro_cross_covariance, (dimension, 3), "state_gyro_cross_covariance")
    _covariance(np.block([[prior, cross], [cross.T, gyro]]), dimension + 3, "joint_state_gyro_covariance")
    effective_r = pixel + jacobian @ gyro @ jacobian.T
    effective_c = -cross @ jacobian.T
    if not np.isfinite(effective_r).all() or not np.isfinite(effective_c).all():
        raise ValueError("effective covariance overflows")
    return {
        "status": "CONDITIONAL_MODEL_VALID", "reason": "EXPLICIT_JOINT_COVARIANCE_AND_PIXEL_INDEPENDENCE",
        "R_effective": effective_r.tolist(), "C_effective": effective_c.tolist(), "uncertainty_known": True,
        "pixel_noise_independent_of_state_and_gamma": True,
        "hardware_validated": False, "statistical_calibration_validated": False,
    }
