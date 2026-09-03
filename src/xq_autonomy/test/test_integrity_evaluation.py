import numpy as np
import pytest

from xq_autonomy.integrity_evaluation import evaluate_ground_truth_integrity


def test_hmi_availability_coverage_and_false_alarm_are_independent():
    result = evaluate_ground_truth_integrity(
        np.asarray((0.0, 1.0, 2.0, 3.0)),
        np.asarray(((0.02, 0.0, 0.0), (0.20, 0.0, 0.0),
                    (0.04, 0.0, 0.0), (0.20, 0.0, 0.0))),
        np.asarray((0.0, 1.0, 2.0, 3.0)),
        np.asarray((0.10, 0.10, 0.10, 0.10)),
        np.asarray((0.05, 0.05, 0.15, 0.15)),
        np.tile((1.0, 0.0, 0.0), (4, 1)),
        np.eye(3),
    )
    assert result["gt_integrity_matched_samples"] == 4
    assert result["hmi_count"] == 1
    assert result["gt_safety_violation_count"] == 2
    assert result["alarm_count"] == 2
    assert result["false_alarm_count"] == 1
    assert result["availability_rate"] == pytest.approx(0.5)
    assert result["pl_empirical_coverage_rate"] == pytest.approx(0.5)
    assert result["alert_recall"] == pytest.approx(0.5)


def test_direction_is_rotated_into_truth_frame():
    rotation = np.asarray(((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))
    result = evaluate_ground_truth_integrity(
        np.asarray((1.0,)),
        np.asarray(((0.0, 0.2, 0.0),)),
        np.asarray((1.0,)),
        np.asarray((0.3,)),
        np.asarray((0.25,)),
        np.asarray(((1.0, 0.0, 0.0),)),
        rotation,
    )
    assert result["gt_directional_error_max_m"] == pytest.approx(0.2)


def test_unmatched_samples_are_not_fabricated():
    result = evaluate_ground_truth_integrity(
        np.asarray((0.0,)), np.zeros((1, 3)), np.asarray((10.0,)),
        np.asarray((0.1,)), np.asarray((0.1,)),
        np.asarray(((1.0, 0.0, 0.0),)), np.eye(3),
    )
    assert result == {"gt_integrity_matched_samples": 0}
