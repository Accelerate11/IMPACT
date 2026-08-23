import numpy as np

from xq_autonomy.alert_limit import compute_alert_limit, sample_bspline


def test_de_boor_linear_clamped_curve_includes_endpoints():
    samples = sample_bspline(
        np.array(((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))),
        np.array((0.0, 0.0, 1.0, 1.0)),
        degree=1,
        interval_s=0.25,
    )
    assert np.allclose(samples[:, 0], (0.0, 0.25, 0.5, 0.75, 1.0))
    assert np.allclose(samples[:, 1:], 0.0)


def test_de_boor_can_drop_expired_trajectory_prefix():
    samples = sample_bspline(
        np.array(((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))),
        np.array((0.0, 0.0, 1.0, 1.0)),
        degree=1,
        interval_s=0.25,
        minimum_parameter_s=0.5,
    )
    assert np.allclose(samples[:, 0], (0.5, 0.75, 1.0))


def test_nearest_obstacle_direction_and_alert_limit_equation():
    result = compute_alert_limit(
        np.array(((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))),
        np.array(((0.0, 2.0, 0.0), (1.0, 0.8, 0.0))),
        speed_mps=0.5,
        latency_p99_s=0.1,
        maximum_acceleration_mps2=1.0,
        body_radius_m=0.35,
        base_reserve_m=0.10,
        tracking_reserve_m=0.10,
    )
    assert np.allclose(result.critical_sample, (1.0, 0.0, 0.0))
    assert np.allclose(result.nearest_obstacle, (1.0, 0.8, 0.0))
    assert np.allclose(result.obstacle_direction, (0.0, 1.0, 0.0))
    assert abs(result.latency_reserve - 0.055) < 1.0e-12
    assert abs(result.alert_limit - 0.195) < 1.0e-12
    assert result.trajectory_samples.shape == (2, 3)
    assert result.nearest_obstacles.shape == (2, 3)
    assert result.obstacle_directions.shape == (2, 3)
    assert np.allclose(result.geometric_clearances, (np.hypot(1.0, 0.8), 0.8))
    assert np.allclose(result.alert_limits, (np.hypot(1.0, 0.8) - 0.605, 0.195))


def test_alert_limit_can_be_negative_in_too_tight_environment():
    result = compute_alert_limit(
        np.array(((0.0, 0.0, 0.0),)),
        np.array(((0.4, 0.0, 0.0),)),
        speed_mps=0.0,
        latency_p99_s=0.1,
        maximum_acceleration_mps2=1.0,
        body_radius_m=0.35,
        base_reserve_m=0.10,
        tracking_reserve_m=0.10,
    )
    assert result.alert_limit < 0.0


def test_latency_reserve_monotonically_reduces_alert_limit():
    common = dict(
        trajectory_samples=np.array(((0.0, 0.0, 0.0),)),
        obstacle_points=np.array(((2.0, 0.0, 0.0),)),
        latency_p99_s=0.1,
        maximum_acceleration_mps2=1.0,
        body_radius_m=0.35,
        base_reserve_m=0.10,
        tracking_reserve_m=0.10,
    )
    slow = compute_alert_limit(speed_mps=0.0, **common)
    fast = compute_alert_limit(speed_mps=0.8, **common)
    assert fast.latency_reserve > slow.latency_reserve
    assert fast.alert_limit < slow.alert_limit
