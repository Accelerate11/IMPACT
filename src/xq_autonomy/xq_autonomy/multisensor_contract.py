"""Pure R2 sensor contracts; no estimator, independent-fusion proof, or AP input.

Schema 1 is exemplified by config/research_multisensor_contract.json. Transforms
always apply p_body = R_body_sensor p_sensor + t_body_sensor. Numerical SO(3)
validity alone cannot distinguish a mislabeled inverse: use frame labels and an
independently supplied expected transform to check direction. Hardware readiness
and statistical independence remain separate from a passing simulation contract.
"""

from __future__ import annotations

from itertools import combinations
import math


STREAM_SEMANTICS = {
    "lidar": "snapshot_instant", "imu": "sample_instant",
    "front_camera": "exposure_instant", "down_camera": "exposure_instant",
    "flow": "integration_end",
}


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _vector(values, length):
    return isinstance(values, (tuple, list)) and len(values) == length and all(_finite(v) for v in values)


def _result(errors, **extra):
    return {"status": "FAIL" if errors else "PASS", "valid": not errors, "errors": errors, **extra}


def check_rigid_transform(
    body_from_sensor, *, body_frame, sensor_frame,
    expected_rotation=None, expected_translation_m=None, tolerance=1.0e-6,
):
    """Check labels/SO(3), optionally anchoring numbers to independent calibration.

    No matrix inversion, normalization, or direction guessing is performed.
    expected_* are independent references, not copies of the tested values.
    """
    errors = []
    if not isinstance(body_from_sensor, dict):
        return _result(["body_from_sensor must be an object"])
    if body_from_sensor.get("convention") != "body_from_sensor":
        errors.append("transform convention must be body_from_sensor")
    if body_from_sensor.get("parent_frame") != body_frame or body_from_sensor.get("child_frame") != sensor_frame:
        errors.append("transform parent/child frames do not match body_from_sensor")
    rotation = body_from_sensor.get("rotation")
    translation = body_from_sensor.get("translation_m")
    valid_rotation = isinstance(rotation, (tuple, list)) and len(rotation) == 3 and all(_vector(row, 3) for row in rotation)
    determinant = None
    orthogonality_error = None
    if not valid_rotation:
        errors.append("rotation must be a finite 3x3 matrix")
    else:
        a, b, c = rotation
        determinant = (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        )
        orthogonality_error = max(
            abs(sum(rotation[k][i] * rotation[k][j] for k in range(3)) - float(i == j))
            for i in range(3) for j in range(3)
        )
        if not _finite(determinant) or abs(determinant - 1.0) > tolerance or orthogonality_error > tolerance:
            errors.append("rotation is not SO(3): require R^T R=I and determinant +1")
        if expected_rotation is not None and rotation != expected_rotation:
            if not (isinstance(expected_rotation, (list, tuple)) and len(expected_rotation) == 3 and all(_vector(row, 3) for row in expected_rotation)) or any(
                abs(rotation[i][j] - expected_rotation[i][j]) > tolerance for i in range(3) for j in range(3)
            ):
                errors.append("rotation does not match the independent body_from_sensor reference")
    if not _vector(translation, 3):
        errors.append("translation_m must be a finite 3-vector")
    elif expected_translation_m is not None and (
        not _vector(expected_translation_m, 3)
        or any(abs(a - b) > tolerance for a, b in zip(translation, expected_translation_m))
    ):
        errors.append("translation does not match the independent body_from_sensor reference")
    return _result(
        errors, determinant=determinant if _finite(determinant) else None,
        orthogonality_error=orthogonality_error if _finite(orthogonality_error) else None,
        direction_reference_checked=expected_rotation is not None or expected_translation_m is not None,
        physical_calibration_proven=False,
    )


def map_acquisition_time(stamp_s, *, clock_domain, clock_mappings):
    """Map a positive acquisition stamp with an explicitly validated affine map.

    A mapping is {target_domain, scale, offset_s, validated,
    acquisition_not_receipt}. It is never estimated from callback receipt time.
    The caller compares the returned stamp only with that target clock domain.
    """
    if not _finite(stamp_s) or stamp_s <= 0.0:
        raise ValueError("acquisition stamp must be positive and finite; zero means not ready")
    mapping = clock_mappings.get(clock_domain) if isinstance(clock_mappings, dict) else None
    if not isinstance(mapping, dict) or mapping.get("validated") is not True or mapping.get("acquisition_not_receipt") is not True:
        raise ValueError("missing validated acquisition-not-receipt clock mapping")
    scale, offset = mapping.get("scale"), mapping.get("offset_s")
    if not _finite(scale) or scale <= 0.0 or not _finite(offset) or not isinstance(mapping.get("target_domain"), str) or not mapping["target_domain"]:
        raise ValueError("invalid clock mapping scale, offset, or target domain")
    mapped = scale * stamp_s + offset
    if not _finite(mapped) or mapped <= 0.0:
        raise ValueError("mapped acquisition stamp must be positive and finite")
    return mapped


class StreamClockGuard:
    """Use mapped acquisition time plus a monotonic receipt watchdog.

    observe returns accepted/reason/status; stale or invalid observations never
    refresh the last accepted receipt. status checks age without a new message.
    WAITING at initial clock zero is recoverable. A later clock rollback latches
    FAIL for this instance; recreate it explicitly for another clock epoch.
    """

    def __init__(self, max_age_s, future_tolerance_s=0.0):
        if not _finite(max_age_s) or max_age_s <= 0 or not _finite(future_tolerance_s) or future_tolerance_s < 0:
            raise ValueError("invalid age/future tolerance")
        self.max_age_s, self.future_tolerance_s = max_age_s, future_tolerance_s
        self.last_stamp_s = self.last_receipt_s = None
        self.last_now_source_s = self.last_now_wall_s = None
        self.clock_fault = None

    def _clock(self, now_source_s, now_monotonic_s):
        if self.clock_fault:
            return {"status": "FAIL", "valid": False, "reason": self.clock_fault, "accepted": False}
        if not _finite(now_source_s) or not _finite(now_monotonic_s) or now_monotonic_s < 0:
            return {"status": "REJECTED", "valid": False, "reason": "invalid_current_clock", "accepted": False}
        if (
            self.last_now_source_s is not None and now_source_s < self.last_now_source_s
            or self.last_now_wall_s is not None and now_monotonic_s < self.last_now_wall_s
        ):
            self.clock_fault = "clock_rollback"
            return {"status": "FAIL", "valid": False, "reason": self.clock_fault, "accepted": False}
        if now_source_s <= 0:
            return {"status": "WAITING", "valid": False, "reason": "source_clock_not_started", "accepted": False}
        self.last_now_source_s, self.last_now_wall_s = now_source_s, now_monotonic_s
        return None

    def observe(self, source_stamp_s, *, receipt_monotonic_s, now_source_s, now_monotonic_s):
        problem = self._clock(now_source_s, now_monotonic_s)
        if problem:
            return problem
        reason = None
        if not _finite(source_stamp_s) or source_stamp_s <= 0:
            reason = "invalid_acquisition_stamp"
        elif not _finite(receipt_monotonic_s) or receipt_monotonic_s < 0:
            reason = "invalid_receipt_stamp"
        elif self.last_stamp_s is not None and source_stamp_s <= self.last_stamp_s:
            reason = "duplicate_or_out_of_order_stamp"
        elif source_stamp_s > now_source_s + self.future_tolerance_s:
            reason = "future_acquisition_stamp"
        elif now_source_s - source_stamp_s > self.max_age_s:
            reason = "acquisition_stale"
        elif receipt_monotonic_s > now_monotonic_s:
            reason = "future_receipt_stamp"
        elif now_monotonic_s - receipt_monotonic_s > self.max_age_s:
            reason = "receipt_stale"
        elif self.last_receipt_s is not None and receipt_monotonic_s < self.last_receipt_s:
            reason = "receipt_out_of_order"
        if reason:
            return {"status": "REJECTED", "valid": False, "reason": reason, "accepted": False}
        self.last_stamp_s, self.last_receipt_s = source_stamp_s, receipt_monotonic_s
        return {**self.status(now_source_s=now_source_s, now_monotonic_s=now_monotonic_s), "accepted": True}

    def status(self, *, now_source_s, now_monotonic_s):
        problem = self._clock(now_source_s, now_monotonic_s)
        if problem:
            return problem
        if self.last_stamp_s is None:
            return {"status": "WAITING", "valid": False, "reason": "no_accepted_sample", "accepted": False}
        source_age, receipt_age = now_source_s - self.last_stamp_s, now_monotonic_s - self.last_receipt_s
        fresh = -self.future_tolerance_s <= source_age <= self.max_age_s and 0 <= receipt_age <= self.max_age_s
        return {
            "status": "PASS" if fresh else "STALE", "reason": "fresh" if fresh else "source_or_receipt_stale",
            "valid": fresh, "source_age_s": source_age, "receipt_age_s": receipt_age,
        }


def check_camera_model(intrinsics, *, axis_convention):
    """Pinhole calibration in ROS optical axes: X right, Y down, Z forward."""
    errors = []
    if not isinstance(intrinsics, dict):
        return _result(["intrinsics must be an object"])
    width, height = intrinsics.get("width"), intrinsics.get("height")
    shape_valid = all(isinstance(v, int) and not isinstance(v, bool) and v > 0 for v in (width, height))
    if not shape_valid:
        errors.append("image width/height must be positive integers")
    if not all(_finite(intrinsics.get(key)) for key in ("fx", "fy", "cx", "cy")):
        errors.append("camera intrinsics must be finite")
    elif intrinsics["fx"] <= 0 or intrinsics["fy"] <= 0 or shape_valid and not (
        0 <= intrinsics["cx"] < width and 0 <= intrinsics["cy"] < height
    ):
        errors.append("invalid focal length or principal point")
    if axis_convention != "ros_optical":
        errors.append("camera axis convention must be ros_optical")
    return _result(errors, axes={"x": "right", "y": "down", "z": "forward"}, hardware_calibrated=False)


def check_gyro_coverage(start_s, end_s, gyro_stamps_s, max_gap_s):
    """Require monotone IMU samples bracketing the entire integration interval."""
    if not all(_finite(v) for v in (start_s, end_s, max_gap_s)) or start_s <= 0 or end_s <= start_s or max_gap_s <= 0:
        return _result(["invalid integration interval or maximum gyro gap"], covered=False, maximum_observed_gap_s=None)
    if not isinstance(gyro_stamps_s, (tuple, list)) or len(gyro_stamps_s) < 2 or not all(_finite(v) and v > 0 for v in gyro_stamps_s):
        return _result(["need at least two finite positive gyro timestamps"], covered=False, maximum_observed_gap_s=None)
    if any(b <= a for a, b in zip(gyro_stamps_s, gyro_stamps_s[1:])):
        return _result(["gyro stamps must be strictly increasing"], covered=False, maximum_observed_gap_s=None)
    bracketed = gyro_stamps_s[0] <= start_s and gyro_stamps_s[-1] >= end_s
    gaps = [b - a for a, b in zip(gyro_stamps_s, gyro_stamps_s[1:]) if a < end_s and b > start_s]
    observed = max(gaps) if gaps else None
    errors = []
    if not bracketed:
        errors.append("gyro samples do not bracket the integration interval")
    if observed is None or observed > max_gap_s + 1.0e-12:
        errors.append("gyro integration coverage has a missing or excessive gap")
    return _result(errors, covered=not errors, maximum_observed_gap_s=observed)


def flow_metric_velocity(
    integrated_x, integrated_y, *, integration_time_s, distance_m, quality,
    gyro_integral_xy=None, metric_requested=True, input_kind="optical_compensated_angle",
):
    """Internal image-axis angular flow, NOT MAVLink OPTICAL_FLOW_RAD axes.

    For a frontoparallel stationary plane, v_optical_xy = -distance *
    compensated_image_angle_xy / dt. This is a limited model, not a calibrated
    velocity measurement or covariance. Raw optical angles require a rotational
    image-angle integral already projected into the SAME optical axes/window.
    Compensated inputs must not have gyro subtracted a second time.
    """
    errors = []
    if not all(_finite(v) for v in (integrated_x, integrated_y, integration_time_s)) or integration_time_s <= 0:
        errors.append("finite flow angles and positive integration_time_s required")
    if not isinstance(quality, int) or isinstance(quality, bool) or not 1 <= quality <= 255:
        errors.append("quality must be a valid nonzero uint8")
    if input_kind not in {"optical_compensated_angle", "raw_optical_angle"}:
        errors.append("MAVLink/body angular axes cannot be passed as internal optical image axes")
    elif input_kind == "optical_compensated_angle" and gyro_integral_xy is not None:
        errors.append("already compensated optical flow must not subtract gyro again")
    elif input_kind == "raw_optical_angle" and not _vector(gyro_integral_xy, 2):
        errors.append("raw optical flow requires same-window projected rotational optical angles")
    distance_valid = _finite(distance_m) and distance_m > 0
    if metric_requested and not distance_valid:
        errors.append("metric flow velocity unavailable without a finite positive valid range")
    velocity = None
    if not errors and metric_requested:
        rotational = gyro_integral_xy if input_kind == "raw_optical_angle" else (0.0, 0.0)
        velocity = [-(angle - gyro) * distance_m / integration_time_s for angle, gyro in zip((integrated_x, integrated_y), rotational)]
        if not all(_finite(v) for v in velocity):
            errors.append("metric velocity overflow")
            velocity = None
    return _result(
        errors, metric_velocity_available=velocity is not None, metric_velocity_optical_mps=velocity,
        range_valid=distance_valid, calibration_status="NOT_CALIBRATED", uncertainty_known=False,
    )


def check_independent_fusion(dependencies, *, assume_independent=True):
    """Reject independence shortcuts; disjoint leaves are not a proof either."""
    if not isinstance(dependencies, dict) or len(dependencies) < 2:
        return _result(["provenance requires at least two named estimates"], independent_fusion_ready=False)
    leaves = {}
    for name, value in dependencies.items():
        values = value.get("leaf_dependencies") if isinstance(value, dict) else value
        if not isinstance(values, (tuple, list)) or not values or any(not isinstance(v, str) or not v for v in values) or len(set(values)) != len(values):
            return _result([f"invalid or repeated leaf dependencies for {name}"], independent_fusion_ready=False)
        leaves[name] = set(values)
    overlaps = [
        {"estimates": [a, b], "shared_leaves": sorted(leaves[a] & leaves[b])}
        for a, b in combinations(leaves, 2) if leaves[a] & leaves[b]
    ]
    return {
        "status": "UNRESOLVED", "valid": False, "errors": [], "shared_leaf_dependencies": overlaps,
        "dependency_disjoint": not overlaps, "independent_fusion_ready": False,
        "independence_claim_accepted": False, "streaming_allowed": not assume_independent,
        "reason": "shared_inputs_require_correlation_handling" if overlaps else "disjoint_inputs_do_not_establish_statistical_independence",
    }


def validate_contract_config(config):
    """Validate schema 1; PASS permits simulation contracts, not independent fusion."""
    errors, checks = [], {}
    if not isinstance(config, dict):
        return _result(["config must be an object"], independent_fusion_ready=False)
    checks["schema_version"] = type(config.get("schema_version")) is int and config["schema_version"] == 1
    checks["profile"] = config.get("profile") in {"simulation", "hardware"}
    body = config.get("body_frame")
    checks["body_frame"] = isinstance(body, str) and bool(body)
    checks["hardware_flag_explicit"] = isinstance(config.get("hardware_calibration_validated"), bool)
    checks["independence_flag_explicit"] = isinstance(config.get("assume_independent_fusion"), bool)
    streams = config.get("streams", {})
    if not isinstance(streams, dict):
        streams = {}
    details = {}
    for name, semantics in STREAM_SEMANTICS.items():
        stream = streams.get(name)
        if not isinstance(stream, dict):
            checks[f"{name}.present"] = False
            continue
        frame = stream.get("frame_id")
        checks[f"{name}.frame"] = isinstance(frame, str) and bool(frame)
        checks[f"{name}.topic"] = isinstance(stream.get("topic"), str) and stream["topic"].startswith("/")
        if name == "lidar" and config.get("profile") == "hardware":
            checks[f"{name}.timestamp_semantics"] = stream.get("timestamp_semantics") == "scan_start" or (
                stream.get("timestamp_semantics") == "driver_documented"
                and isinstance(stream.get("timestamp_semantics_reference"), str)
                and bool(stream["timestamp_semantics_reference"])
            )
        else:
            checks[f"{name}.timestamp_semantics"] = stream.get("timestamp_semantics") == semantics
        if name == "lidar" and config.get("profile") == "simulation":
            checks["lidar.snapshot_model_explicit"] = (
                stream.get("acquisition_model") == "gazebo_instantaneous_range_snapshot"
                and stream.get("per_point_timing_available") is False
                and stream.get("real_mid360_scan_pattern") is False
            )
        checks[f"{name}.acquisition_not_receipt"] = stream.get("timestamp_source") == "acquisition"
        for field in ("nominal_rate_hz", "maximum_age_s"):
            checks[f"{name}.{field}"] = _finite(stream.get(field)) and stream[field] > 0
        try:
            map_acquisition_time(1.0, clock_domain=stream.get("clock_domain"), clock_mappings=config.get("clock_mappings"))
            mapping = config["clock_mappings"][stream["clock_domain"]]
            checks[f"{name}.clock_mapping"] = mapping["target_domain"] == config.get("reference_clock_domain")
        except (ValueError, TypeError, KeyError):
            checks[f"{name}.clock_mapping"] = False
        transform = check_rigid_transform(stream.get("body_from_sensor"), body_frame=body, sensor_frame=frame)
        checks[f"{name}.body_from_sensor"] = transform["status"] == "PASS"
        details[name] = {"transform": transform}
        leaves = stream.get("leaf_dependencies")
        checks[f"{name}.leaf_dependencies"] = isinstance(leaves, list) and bool(leaves) and all(isinstance(v, str) and v for v in leaves) and len(set(leaves)) == len(leaves)
        if name in {"front_camera", "down_camera"}:
            camera = check_camera_model(stream.get("intrinsics"), axis_convention=stream.get("axis_convention"))
            details[name]["camera"] = camera
            checks[f"{name}.intrinsics_and_axes"] = camera["status"] == "PASS"
            checks[f"{name}.exposure_semantics"] = stream.get("exposure_model") in {"instantaneous_simulation", "hardware_exposure_calibrated", "unknown"}
    flow = streams.get("flow", {})
    flow = flow if isinstance(flow, dict) else {}
    checks["flow.internal_axes"] = flow.get("input_kind") == "optical_compensated_angle"
    checks["flow.gyro_gap"] = _finite(flow.get("maximum_gyro_gap_s")) and flow["maximum_gyro_gap_s"] > 0
    checks["flow.minimum_tracks"] = type(flow.get("minimum_tracks")) is int and flow["minimum_tracks"] > 0
    checks["flow.no_invented_metric_scale"] = flow.get("metric_velocity_available") is False or flow.get("range_source") not in {None, "unknown", "none"}
    flow_leaves = flow.get("leaf_dependencies", [])
    checks["flow.gyro_provenance"] = isinstance(flow_leaves, list) and flow.get("gyro_source") in flow_leaves
    if flow.get("range_source") == "lidar":
        lidar_leaves = streams.get("lidar", {}).get("leaf_dependencies", [])
        checks["flow.range_provenance"] = isinstance(flow_leaves, list) and all(leaf in flow_leaves for leaf in lidar_leaves)
    provenance = check_independent_fusion(config.get("provenance"), assume_independent=config.get("assume_independent_fusion") is True)
    checks["provenance_structure"] = not provenance["errors"]
    provenance_flow = config.get("provenance", {}).get("flow", {}) if isinstance(config.get("provenance"), dict) else {}
    provenance_flow_leaves = provenance_flow.get("leaf_dependencies", []) if isinstance(provenance_flow, dict) else []
    checks["flow.derived_provenance_complete"] = isinstance(flow_leaves, list) and isinstance(provenance_flow_leaves, list) and all(leaf in provenance_flow_leaves for leaf in flow_leaves)
    errors.extend(name for name, valid in checks.items() if not valid)
    hardware_flags = config.get("hardware_readiness", {})
    hardware_flags = hardware_flags if isinstance(hardware_flags, dict) else {}
    missing_hardware = [name for name in (
        "clock_calibration_validated", "extrinsic_calibration_validated",
        "camera_exposure_validated", "flow_scale_calibrated",
    ) if hardware_flags.get(name) is not True]
    if config.get("hardware_calibration_validated") is not True:
        missing_hardware.append("hardware_calibration_validated")
    if any(not isinstance(stream, dict) or stream.get("clock_domain") == "simulation" for stream in streams.values()):
        missing_hardware.append("physical_clock_domains")
    if any(not isinstance(streams.get(name), dict) or streams[name].get("exposure_model") != "hardware_exposure_calibrated" for name in ("front_camera", "down_camera")):
        missing_hardware.append("physical_camera_exposure_models")
    hardware_ready = not missing_hardware
    status = "FAIL" if errors else "NOT_READY" if (
        config.get("profile") == "hardware" and not hardware_ready
        or config.get("assume_independent_fusion") is True
    ) else "PASS"
    return {
        "status": status, "valid": status == "PASS", "checks": checks, "errors": errors, "streaming_ready": not errors,
        "stream_details": details, "independent_fusion_ready": False,
        "unresolved_assumptions": [provenance["reason"]] if "reason" in provenance else provenance["errors"],
        "provenance_audit": provenance,
        "hardware_readiness": {"status": "READY" if hardware_ready else "NOT_READY", "missing": missing_hardware},
        "scope": "Sensor contracts only; no fusion covariance, calibration coverage, or integrity guarantee.",
    }
