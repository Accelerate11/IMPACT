import numpy as np
import pytest

from xq_autonomy.candidate_metrics import (
    compute_task_gains,
    motion_energy_proxy,
    pointwise_collision_probability,
    return_energy_proxy,
)


def _map():
    positions = np.asarray(((0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 2.0, 1.0)))
    confidence = np.asarray((1.0, 1.0, 0.2))
    quality = np.ones(3)
    last_seen = np.asarray((10.0, 0.0, 0.0))
    return positions, confidence, quality, last_seen


def test_task_gain_uses_geometry_and_progress_not_candidate_name():
    direct = np.asarray(((0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (2.0, 0.0, 1.0)))
    observing = np.asarray(((0.0, 0.0, 1.0), (1.0, 1.8, 1.0), (2.0, 0.0, 1.0)))
    positions, confidence, quality, seen = _map()
    scores = compute_task_gains(
        (direct, observing),
        positions,
        confidence,
        quality,
        seen,
        now_s=10.0,
        visibility_radius_m=0.5,
        age_time_constant_s=5.0,
        progress_weight=0.2,
    )
    assert scores[0].progress_efficiency > scores[1].progress_efficiency
    assert scores[1].map_observation_gain > scores[0].map_observation_gain
    assert scores[1].gain > scores[0].gain


def test_identical_map_support_falls_back_to_progress_efficiency():
    direct = np.asarray(((0.0, 0.0, 1.0), (2.0, 0.0, 1.0)))
    detour = np.asarray(((0.0, 0.0, 1.0), (1.0, 1.0, 1.0), (2.0, 0.0, 1.0)))
    positions, confidence, quality, seen = _map()
    scores = compute_task_gains(
        (direct, detour),
        positions,
        confidence,
        quality,
        seen,
        now_s=10.0,
        visibility_radius_m=10.0,
        age_time_constant_s=5.0,
    )
    assert scores[0].map_observation_gain == pytest.approx(0.5)
    assert scores[1].map_observation_gain == pytest.approx(0.5)
    assert scores[0].gain > scores[1].gain


def test_collision_probability_monotonically_tracks_clearance():
    safe = pointwise_collision_probability(
        np.asarray((0.20, 0.15)), tracking_reserve_m=0.10
    )
    risky = pointwise_collision_probability(
        np.asarray((0.02, -0.03)), tracking_reserve_m=0.10
    )
    assert 0.0 <= safe < risky <= 1.0
    assert safe < 0.01


def test_energy_and_return_reserve_are_trajectory_dependent():
    direct = np.asarray(((0.0, 0.0, 1.0), (2.0, 0.0, 1.0)))
    climb = np.asarray(((0.0, 0.0, 1.0), (1.0, 0.0, 2.0), (2.0, 0.0, 2.0)))
    assert motion_energy_proxy(climb, 5.0) > motion_energy_proxy(direct, 5.0)
    assert return_energy_proxy(np.asarray((4.0, 0.0, 2.0)), np.zeros(3)) > 4.0


@pytest.mark.parametrize("bad", [np.nan])
def test_invalid_metric_input_fails_closed(bad):
    with pytest.raises(ValueError):
        pointwise_collision_probability(np.asarray((bad,)), tracking_reserve_m=0.10)


def test_negative_alert_limit_is_valid_but_high_risk():
    probability = pointwise_collision_probability(
        np.asarray((-0.20,)), tracking_reserve_m=0.10
    )
    assert probability > 0.99
