"""Conditional two-view geometry and ordered, bias-corrected SO(3) integration.

This module has no ROS, estimator, or sensor calibration side effects. Gyro bias,
poses, camera extrinsics, and initial optical-axis depth must be supplied. The
midpoint product is a declared numerical integration model, not an exact solution
for arbitrary continuously varying angular velocity. No noise covariance or
protection level is inferred from these deterministic calculations.
"""

from __future__ import annotations

import math

import numpy as np


def _contains_bool(value):
    if isinstance(value, (bool, np.bool_)):
        return True
    if isinstance(value, (list, tuple)):
        return any(_contains_bool(item) for item in value)
    if isinstance(value, np.ndarray):
        return value.dtype.kind == "b" or (
            value.dtype.kind == "O" and any(_contains_bool(item) for item in value.flat)
        )
    return False


def _array(value, shape, name):
    try:
        raw = np.asarray(value)
        if _contains_bool(value) or raw.dtype.kind not in "fiu":
            raise ValueError
        result = np.asarray(raw, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be numeric and finite") from error
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must have finite shape {shape}")
    return result


def _positive(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError(f"{name} must be explicitly finite and positive")
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be explicitly finite and positive") from error
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be explicitly finite and positive")
    return result


def _nanoseconds(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"{name} must be nonnegative integer nanoseconds")
    result = int(value)
    if result < 0:
        raise ValueError(f"{name} must be nonnegative integer nanoseconds")
    return result


def _rotation(value, name):
    result = _array(value, (3, 3), name)
    if not np.allclose(result.T @ result, np.eye(3), rtol=0.0, atol=1.0e-10) or not math.isclose(
        float(np.linalg.det(result)), 1.0, rel_tol=0.0, abs_tol=1.0e-10
    ):
        raise ValueError(f"{name} must be a proper SO(3) rotation")
    return result


def _skew(vector):
    x, y, z = vector
    return np.asarray(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))


def _exp_and_right_jacobian(vector):
    angle = float(np.linalg.norm(vector))
    if not math.isfinite(angle):
        raise ValueError("rotation magnitude overflows")
    cross = _skew(vector)
    angle2 = angle * angle
    if angle < 1.0e-4:
        a = 1.0 - angle2 / 6.0 + angle2 * angle2 / 120.0
        b = 0.5 - angle2 / 24.0 + angle2 * angle2 / 720.0
        c = 1.0 / 6.0 - angle2 / 120.0 + angle2 * angle2 / 5040.0
    else:
        a = math.sin(angle) / angle
        b = (1.0 - math.cos(angle)) / angle2
        c = (angle - math.sin(angle)) / (angle2 * angle)
    cross2 = cross @ cross
    rotation = np.eye(3) + a * cross + b * cross2
    right_jacobian = np.eye(3) - b * cross + c * cross2
    if not np.isfinite(rotation).all() or not np.isfinite(right_jacobian).all():
        raise ValueError("rotation or derivative overflows")
    return rotation, right_jacobian


def integrate_gyro_midpoint(stamps_ns, angular_velocity_body_rps, start_ns, end_ns,
                            gyro_bias_body_rps, *, initial_world_from_body_rotation,
                            maximum_gap_ns) -> dict:
    """Integrate a fully bracketed time window by ordered SO(3) products.

    Gyro samples describe a piecewise-linear angular-velocity signal in body
    coordinates. Each original sample interval is clipped to the requested
    window, evaluated at that clipped interval's midpoint, and right-multiplied:
    ``R_next = R_previous @ Exp((omega_mid - explicit_bias) * dt)``.

    All acquisition timestamps remain integers until *differences* become
    seconds. No extrapolation, missing-bias substitution, or over-gap segment is
    permitted. maximum_gap_ns applies to every original interval intersecting
    the window, even if the clipped portion of that interval is shorter.

    rotation_jacobian_bias maps an additive constant-bias perturbation to the
    final right-local body rotation perturbation, in radians per radian/second.
    It is the derivative of this midpoint product, not a covariance propagation.
    """
    start = _nanoseconds(start_ns, "start_ns")
    end = _nanoseconds(end_ns, "end_ns")
    maximum_gap = _nanoseconds(maximum_gap_ns, "maximum_gap_ns")
    if end <= start or maximum_gap <= 0:
        raise ValueError("window duration and maximum_gap_ns must be positive")
    try:
        stamps = [_nanoseconds(value, "sample timestamp") for value in stamps_ns]
    except TypeError as error:
        raise ValueError("stamps_ns must contain ordered integer nanoseconds") from error
    if len(stamps) < 2 or any(right <= left for left, right in zip(stamps, stamps[1:])):
        raise ValueError("at least two strictly increasing sample timestamps are required")
    rates = _array(angular_velocity_body_rps, (len(stamps), 3), "angular_velocity_body_rps")
    bias = _array(gyro_bias_body_rps, (3,), "gyro_bias_body_rps")
    initial = _rotation(initial_world_from_body_rotation, "initial_world_from_body_rotation")
    if stamps[0] > start or stamps[-1] < end:
        raise ValueError("gyro samples must cover both window endpoints; extrapolation forbidden")

    relative = np.eye(3)
    rotation = initial.copy()
    bias_jacobian = np.zeros((3, 3))
    segments = []
    for index, (left, right) in enumerate(zip(stamps, stamps[1:])):
        segment_start, segment_end = max(start, left), min(end, right)
        if segment_end <= segment_start:
            continue
        if right - left > maximum_gap:
            raise ValueError("gyro sample gap exceeds maximum_gap_ns inside the window")
        # Form all epoch differences as integers; do not average floating epochs.
        fraction = ((segment_start - left) + (segment_end - left)) / (2 * (right - left))
        midpoint = rates[index] + fraction * (rates[index + 1] - rates[index])
        corrected = midpoint - bias
        dt = (segment_end - segment_start) * 1.0e-9
        vector = corrected * dt
        increment, right_jacobian = _exp_and_right_jacobian(vector)
        rotation = rotation @ increment
        relative = relative @ increment
        bias_jacobian = increment.T @ bias_jacobian - dt * right_jacobian
        segments.append({
            "sample_index": index,
            "sample_start_ns": left,
            "sample_end_ns": right,
            "start_ns": segment_start,
            "end_ns": segment_end,
            "duration_ns": segment_end - segment_start,
            "dt_s": dt,
            "midpoint_angular_velocity_body_rps": midpoint.tolist(),
            "bias_corrected_midpoint_body_rps": corrected.tolist(),
            "rotation_vector_rad": vector.tolist(),
            "body_rotation_increment": increment.tolist(),
        })
    if not segments or sum(segment["duration_ns"] for segment in segments) != end - start:
        raise ValueError("gyro segments do not completely cover the requested window")
    if not all(np.isfinite(item).all() for item in (rotation, relative, bias_jacobian)):
        raise ValueError("integrated rotation or derivative overflows")
    return {
        "start_ns": start,
        "end_ns": end,
        "duration_ns": end - start,
        "dt_s": (end - start) * 1.0e-9,
        "world_from_body_rotation": rotation.tolist(),
        "relative_body_rotation": relative.tolist(),
        "rotation_jacobian_bias": bias_jacobian.tolist(),
        "gyro_bias_body_rps": bias.tolist(),
        "segments": segments,
        "integration_model": "piecewise_linear_sample_clipped_segment_midpoint_SO3_product",
        "rotation_perturbation": "R_perturbed=R_nominal_Exp(delta_theta_body)",
        "complete_window": True,
        "covariance": None,
        "uncertainty_known": False,
        "sensor_bias_calibrated": False,
        "fusion_update_performed": False,
    }


def predict_two_view(normalized_xy, depth_m, start_world_from_body_rotation,
                     start_position_world_m, end_world_from_body_rotation,
                     end_position_world_m, body_from_optical_rotation,
                     body_from_optical_translation_m) -> dict:
    """Project one static world point between explicit camera/body poses.

    depth_m is the *initial optical-axis Z* in metres. Extrinsics transform
    optical coordinates to the IMU/body frame and include the camera lever arm.
    Jacobians differentiate predicted new normalized (x, y), with pose columns
    [right-local body rotation delta_theta, additive world position delta_p].
    Raw-displacement pose/depth Jacobians are identical because the original
    normalized pixel is held fixed for those derivatives.

    This deterministic prediction supplies neither depth estimation nor a noise
    model, temporal state history, shared-IMU covariance, or a fusion update.
    """
    xy = _array(normalized_xy, (2,), "normalized_xy")
    depth = _positive(depth_m, "depth_m")
    r0 = _rotation(start_world_from_body_rotation, "start_world_from_body_rotation")
    p0 = _array(start_position_world_m, (3,), "start_position_world_m")
    r1 = _rotation(end_world_from_body_rotation, "end_world_from_body_rotation")
    p1 = _array(end_position_world_m, (3,), "end_position_world_m")
    rbc = _rotation(body_from_optical_rotation, "body_from_optical_rotation")
    tbc = _array(body_from_optical_translation_m, (3,), "body_from_optical_translation_m")

    ray = np.r_[xy, 1.0]
    body_point0 = rbc @ (depth * ray) + tbc
    world_point = r0 @ body_point0 + p0
    body_point1 = r1.T @ (world_point - p1)
    optical_point1 = rbc.T @ (body_point1 - tbc)
    if not np.isfinite(optical_point1).all() or optical_point1[2] <= 1.0e-8:
        raise ValueError("the static point must project finitely in front of the second camera")
    new_xy = optical_point1[:2] / optical_point1[2]
    projection = np.asarray(((1.0, 0.0, -new_xy[0]), (0.0, 1.0, -new_xy[1]))) / optical_point1[2]
    project_from_body1 = projection @ rbc.T
    project_from_world = project_from_body1 @ r1.T
    j_start = np.column_stack((
        -project_from_world @ r0 @ _skew(body_point0), project_from_world,
    ))
    j_end = np.column_stack((
        project_from_body1 @ _skew(body_point1), -project_from_world,
    ))
    j_depth = project_from_world @ r0 @ rbc @ ray
    j_old_xy = project_from_world @ r0 @ rbc[:, :2] * depth
    if not all(np.isfinite(item).all() for item in (new_xy, j_start, j_end, j_depth, j_old_xy)):
        raise ValueError("projection or its derivatives overflow")
    return {
        "new_normalized_ray": [*new_xy.tolist(), 1.0],
        "raw_normalized_displacement": (new_xy - xy).tolist(),
        "end_optical_axis_depth_m": float(optical_point1[2]),
        "jacobian_start_pose": j_start.tolist(),
        "jacobian_end_pose": j_end.tolist(),
        "jacobian_depth_m": j_depth.tolist(),
        "jacobian_old_normalized_xy": j_old_xy.tolist(),
        "jacobian_raw_displacement_old_normalized_xy": (j_old_xy - np.eye(2)).tolist(),
        "pose_perturbation_columns": ["body_right_rotation_x", "body_right_rotation_y", "body_right_rotation_z",
                                      "world_position_x", "world_position_y", "world_position_z"],
        "measurement_space": "normalized_image_plane",
        "depth_definition": "initial_optical_axis_Z_metres",
        "covariance": None,
        "uncertainty_known": False,
        "depth_estimated": False,
        "metric_velocity_estimated": False,
        "fusion_update_performed": False,
    }
