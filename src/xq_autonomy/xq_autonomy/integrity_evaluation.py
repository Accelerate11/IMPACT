"""Independent Ground-Truth integrity metrics for evaluation nodes only."""

from __future__ import annotations

import numpy as np


def evaluate_ground_truth_integrity(
    error_stamps_s: np.ndarray,
    position_errors_truth: np.ndarray,
    sample_stamps_s: np.ndarray,
    alert_limits_m: np.ndarray,
    protection_levels_m: np.ndarray,
    obstacle_directions_map: np.ndarray,
    rotation_map_to_truth: np.ndarray,
    *,
    maximum_time_delta_s: float = 0.15,
) -> dict[str, float | int | bool | None]:
    """Compare online PL/AL decisions with independent realized errors.

    The algorithm never receives these values.  A hazardous misleading
    information (HMI) sample is an actual directional-error violation while
    the online monitor still declares ``PL <= AL``.
    """
    error_stamps = np.asarray(error_stamps_s, dtype=float).reshape(-1)
    errors = np.asarray(position_errors_truth, dtype=float)
    sample_stamps = np.asarray(sample_stamps_s, dtype=float).reshape(-1)
    alerts = np.asarray(alert_limits_m, dtype=float).reshape(-1)
    protections = np.asarray(protection_levels_m, dtype=float).reshape(-1)
    directions = np.asarray(obstacle_directions_map, dtype=float)
    rotation = np.asarray(rotation_map_to_truth, dtype=float)
    if errors.shape != (len(error_stamps), 3):
        raise ValueError("position errors and timestamps must be aligned")
    if directions.shape != (len(sample_stamps), 3):
        raise ValueError("integrity directions and timestamps must be aligned")
    if not (len(alerts) == len(protections) == len(sample_stamps)):
        raise ValueError("integrity sample arrays must be aligned")
    if rotation.shape != (3, 3):
        raise ValueError("map-to-truth rotation must be 3x3")
    arrays = (error_stamps, errors, sample_stamps, alerts, protections, directions, rotation)
    if not all(np.isfinite(value).all() for value in arrays):
        raise ValueError("Ground-Truth integrity inputs must be finite")
    if maximum_time_delta_s <= 0.0 or not np.isfinite(maximum_time_delta_s):
        raise ValueError("maximum time delta must be positive and finite")
    if len(error_stamps) == 0 or len(sample_stamps) == 0:
        return {"gt_integrity_matched_samples": 0}
    if np.any(np.diff(error_stamps) < 0.0):
        raise ValueError("error timestamps must be sorted")

    realized_errors = []
    matched_alerts = []
    matched_protections = []
    for stamp, alert, protection, direction_map in zip(
        sample_stamps, alerts, protections, directions
    ):
        insertion = int(np.searchsorted(error_stamps, stamp))
        choices = [
            index
            for index in (insertion - 1, insertion)
            if 0 <= index < len(error_stamps)
        ]
        if not choices:
            continue
        index = min(choices, key=lambda value: abs(error_stamps[value] - stamp))
        if abs(error_stamps[index] - stamp) > maximum_time_delta_s:
            continue
        norm = float(np.linalg.norm(direction_map))
        if norm <= 1.0e-9:
            continue
        direction_truth = rotation @ (direction_map / norm)
        direction_truth /= max(float(np.linalg.norm(direction_truth)), 1.0e-12)
        realized_errors.append(abs(float(direction_truth @ errors[index])))
        matched_alerts.append(float(alert))
        matched_protections.append(float(protection))

    if not realized_errors:
        return {"gt_integrity_matched_samples": 0}
    realized = np.asarray(realized_errors)
    alerts = np.asarray(matched_alerts)
    protections = np.asarray(matched_protections)
    available = protections <= alerts
    actual_violation = realized > alerts
    alarms = ~available
    hmi = actual_violation & available
    false_alarm = alarms & ~actual_violation
    coverage = realized <= protections
    tightness = protections - realized
    realized_margin = alerts - realized
    violation_count = int(np.count_nonzero(actual_violation))
    alarm_count = int(np.count_nonzero(alarms))
    hmi_count = int(np.count_nonzero(hmi))
    true_alarm_count = int(np.count_nonzero(alarms & actual_violation))
    count = len(realized)
    return {
        "gt_integrity_matched_samples": int(count),
        "gt_directional_error_rms_m": float(np.sqrt(np.mean(realized ** 2))),
        "gt_directional_error_max_m": float(np.max(realized)),
        "gt_minimum_realized_margin_m": float(np.min(realized_margin)),
        "gt_safety_violation_count": violation_count,
        "gt_safety_violation_rate": float(violation_count / count),
        "hmi_count": hmi_count,
        "hmi_rate": float(hmi_count / count),
        "trajectory_hmi": bool(hmi_count > 0),
        "availability_rate": float(np.mean(available)),
        "alarm_count": alarm_count,
        "false_alarm_count": int(np.count_nonzero(false_alarm)),
        "false_alarm_rate": float(np.mean(false_alarm)),
        "alert_recall": (
            float(true_alarm_count / violation_count) if violation_count else 1.0
        ),
        "alert_precision": (
            float(true_alarm_count / alarm_count) if alarm_count else None
        ),
        "pl_empirical_coverage_rate": float(np.mean(coverage)),
        "pl_tightness_mean_m": float(np.mean(tightness)),
        "pl_tightness_p95_m": float(np.quantile(tightness, 0.95)),
    }
