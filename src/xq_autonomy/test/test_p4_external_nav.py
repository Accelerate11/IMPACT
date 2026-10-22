import math
import json
from types import SimpleNamespace

import numpy as np
import pytest

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rclpy.time import Time

from xq_autonomy import p4_external_nav_node as module
from xq_autonomy.p4_external_nav_node import (
    P4ExternalNavNode,
    _reported_body_velocity,
    _world_to_body,
)
from xq_autonomy.p4_mission_node import P4MissionNode
from xq_autonomy.extnav_clock_guard import ExtnavClockGuard


def test_world_velocity_is_rotated_into_body_frame() -> None:
    half = math.sqrt(0.5)
    velocity = _world_to_body((1.0, 0.0, 0.0), (half, 0.0, 0.0, half))
    assert velocity == pytest.approx((0.0, -1.0, 0.0), abs=1e-12)


def test_external_nav_covariance_never_claims_false_certainty() -> None:
    source = [0.0] * 36
    source[0] = float("nan")
    with pytest.raises(ValueError, match="pose_covariance_nonfinite"):
        P4ExternalNavNode._covariance_with_floor(source, 0.0025, 0.0012)


def test_covariance_floor_adds_only_nonnegative_diagonal_and_preserves_psd():
    source = np.eye(6) * 0.0001
    source[0, 1] = source[1, 0] = 0.00009
    result = P4ExternalNavNode._covariance_with_floor(source.reshape(-1), 0.0025, 0.0012)
    output = np.asarray(result).reshape(6, 6)
    assert output[0, 1] == output[1, 0] == pytest.approx(0.00009)
    assert np.linalg.eigvalsh(output).min() >= 0.0
    assert np.linalg.eigvalsh(output - source).min() >= 0.0
    assert [result[index] for index in (0, 7, 14)] == pytest.approx([0.0025] * 3)
    assert [result[index] for index in (21, 28, 35)] == pytest.approx([0.0012] * 3)


def test_unknown_zero_covariance_still_receives_explicit_positive_floors():
    source = [0.0] * 36
    result = P4ExternalNavNode._covariance_with_floor(source, 0.0025, 0.0012)
    assert [result[index] for index in (0, 7, 14)] == pytest.approx([0.0025] * 3)
    assert [result[index] for index in (21, 28, 35)] == pytest.approx([0.0012] * 3)
    assert all(math.isfinite(value) for value in result)


@pytest.mark.parametrize("kind, reason", [
    ("indefinite", "pose_covariance_not_psd"),
    ("negative_diagonal", "pose_covariance_negative_variance"),
    ("asymmetric", "pose_covariance_asymmetric"),
    ("nonfinite", "pose_covariance_nonfinite"),
])
def test_invalid_covariance_is_rejected_instead_of_becoming_a_floored_estimate(kind, reason):
    covariance = np.eye(6) * 0.01
    if kind == "indefinite":
        covariance[0, 1] = covariance[1, 0] = 0.02
    elif kind == "negative_diagonal":
        covariance[0, 0] = -0.001
    elif kind == "asymmetric":
        covariance[0, 1] = 0.001
    else:
        covariance[2, 2] = math.inf
    with pytest.raises(ValueError, match=reason):
        P4ExternalNavNode._covariance_with_floor(covariance.reshape(-1), 0.0025, 0.0012)


def test_only_roundoff_sized_covariance_asymmetry_is_averaged():
    source = np.eye(6) * 0.01
    source[0, 1], source[1, 0] = 1.0e-3 + 1.0e-12, 1.0e-3
    output = np.asarray(P4ExternalNavNode._covariance_with_floor(source.reshape(-1), 0.0025, 0.0012)).reshape(6, 6)
    assert output[0, 1] == output[1, 0]
    assert output[0, 1] == pytest.approx(1.0e-3 + 0.5e-12, abs=1.0e-15)


def _adapter(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(module.time, "monotonic_ns", lambda: int(round(now[0] * 1_000_000_000)))
    node = object.__new__(P4ExternalNavNode)
    output, status, warnings = [], [], []
    node.publisher = SimpleNamespace(publish=output.append, get_subscription_count=lambda: 1)
    node.status_publisher = SimpleNamespace(publish=status.append)
    node.get_logger = lambda: SimpleNamespace(warning=warnings.append)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(
        nanoseconds=1_790_000_000_000_000_000 + int(round((now[0]-100.) * 1_000_000_000))))
    node.clock_guard = ExtnavClockGuard()
    node.last_transport_clock_sample = None
    node.clock_sample_uncertain = False
    node.get_parameter = lambda _: SimpleNamespace(value=4.0)
    node.position_floor, node.orientation_floor, node.velocity_variance = 0.0025, 0.0012, 0.04
    node.alpha, node.maximum_gap = 0.35, 0.35
    node.previous = None
    node.filtered_velocity = (0.0, 0.0, 0.0)
    node.sample_times = []
    node.source_count = node.output_count = node.rejected_count = 0
    node.esekf_velocity_count = node.differenced_velocity_count = 0
    node.rejection_reasons = {}
    node.last_rejection_reason = ""
    node.last_source_wall = node.last_source_stamp = 0.0
    node.last_source_stamp_ns = 0
    return node, now, output, status


def _pose(stamp_ns):
    message = Odometry()
    message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    message.header.frame_id, message.child_frame_id = "xq_lio_map", "livox_imu"
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance = (np.eye(6) * 0.01).reshape(-1).tolist()
    message.twist.covariance[0] = 0.01
    message.twist.twist.linear.x = 0.1
    return message


def test_duplicate_and_backward_lio_do_not_refresh_health_rate_previous_or_output(monkeypatch):
    node, now, output, statuses = _adapter(monkeypatch)
    for index in range(4):
        now[0] = 100.0 + index * 0.1
        node._odom_cb(_pose(1_000_000_000 + index * 100_000_000))
    node._status_cb()
    assert json.loads(statuses[-1].data)["local_source_ready"] is True
    assert json.loads(statuses[-1].data)["healthy"] is False  # Five-second clock warmup is separate.
    original = (node.previous, list(node.sample_times), node.last_source_wall,
                node.filtered_velocity, node.source_count, node.output_count)
    now[0] = 100.8
    node._odom_cb(_pose(1_300_000_000))
    node._odom_cb(_pose(1_200_000_000))
    assert (node.previous, node.sample_times, node.last_source_wall,
            node.filtered_velocity, node.source_count, node.output_count) == original
    assert len(output) == 4
    node._status_cb()
    status = json.loads(statuses[-1].data)
    assert status["healthy"] is False
    assert status["rejection_reasons"]["source_stamp_not_increasing"] == 2
    assert status["last_accepted_acquisition_stamp_ns"] == 1_300_000_000
    assert output[-1].header.stamp.sec == 1_790_000_000


@pytest.mark.parametrize("problem", ["nonpositive", "nan_covariance", "indefinite_covariance"])
def test_source_validation_precedes_estimate_and_freshness_mutation(monkeypatch, problem):
    node, _, output, _ = _adapter(monkeypatch)
    source = _pose(0 if problem == "nonpositive" else 1_000_000_000)
    if problem == "nan_covariance":
        source.pose.covariance[0] = math.nan
    elif problem == "indefinite_covariance":
        source.pose.covariance[1] = source.pose.covariance[6] = 0.02
    node._odom_cb(source)
    assert output == []
    assert node.previous is None
    assert node.last_source_stamp_ns == node.last_source_wall == node.source_count == node.output_count == 0
    assert node.sample_times == []
    assert node.filtered_velocity == (0.0, 0.0, 0.0)
    assert node.rejected_count == 1
    node._odom_cb(_pose(1_000_000_000))
    if problem == "nonpositive":
        assert output == []
        assert node.clock_guard.snapshot(module.time.monotonic_ns())["fault_latched"] is True
    else:
        assert len(output) == 1
        assert node.last_source_stamp_ns == 1_000_000_000


def _warm_adapter(node, now):
    for index in range(51):
        now[0] = 100.0 + index * 0.1
        node._odom_cb(_pose(1_000_000_000 + index * 100_000_000))
    node._status_cb()


def test_clock_warmup_reports_local_readiness_without_claiming_fcu_receipt(monkeypatch):
    node, now, output, statuses = _adapter(monkeypatch)
    _warm_adapter(node, now)
    status = json.loads(statuses[-1].data)
    assert len(output) == 51
    assert status["local_source_ready"] is True
    assert status["local_clock_ready"] is True
    assert status["healthy"] is True
    assert status["clock_ready_is_not_FCU_health"] is True
    assert status["clock_guard"]["timestamps_modified"] is False


def test_transport_clock_backstep_latches_and_suppresses_without_clamping(monkeypatch):
    node, now, output, statuses = _adapter(monkeypatch)
    _warm_adapter(node, now)
    original = (node.previous, node.last_source_stamp_ns, node.source_count, node.output_count)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(
        nanoseconds=1_790_000_000_000_000_000 + int(round((now[0]-100.) * 1e9)) - 300_000_000))
    now[0] = 105.1
    node._odom_cb(_pose(6_100_000_000))
    assert len(output) == 51
    assert (node.previous, node.last_source_stamp_ns, node.source_count, node.output_count) == original
    revoked = json.loads(statuses[-1].data)
    assert revoked["healthy"] is False
    assert revoked["clock_fault_latched"] is True
    first_fault = revoked["clock_guard"]["first_fault"]
    assert "transport_stamp_not_increasing" in first_fault["reasons"]
    assert first_fault["inputs"]["transport_stamp_ns"] < output[-1].header.stamp.sec * 1_000_000_000 + output[-1].header.stamp.nanosec
    now[0] = 106.0
    node._odom_cb(_pose(7_000_000_000))
    assert len(output) == 51  # Returning to ordinary wall time cannot clear the latch.
    assert node.clock_guard.snapshot(module.time.monotonic_ns())["first_fault"] == first_fault


def test_duplicate_source_revokes_ready_and_cannot_refresh_liveness(monkeypatch):
    node, now, output, statuses = _adapter(monkeypatch)
    _warm_adapter(node, now)
    last_source, last_wall = node.last_source_stamp_ns, node.last_source_wall
    now[0] = 105.1
    node._odom_cb(_pose(last_source))
    assert len(output) == 51
    assert node.last_source_wall == last_wall
    assert json.loads(statuses[-1].data)["clock_fault_latched"] is True
    assert json.loads(statuses[-1].data)["healthy"] is False


def test_source_timeout_discards_warmup_then_requires_new_continuous_window(monkeypatch):
    node, now, output, statuses = _adapter(monkeypatch)
    _warm_adapter(node, now)
    now[0] = 106.0
    node._status_cb()
    stale = json.loads(statuses[-1].data)
    assert stale["healthy"] is False
    assert stale["clock_fault_latched"] is False
    for index in range(51):
        now[0] = 106.1 + index * .1
        node._odom_cb(_pose(7_100_000_000 + index * 100_000_000))
        node._status_cb()
        if index < 50:
            assert json.loads(statuses[-1].data)["local_clock_ready"] is False
    assert json.loads(statuses[-1].data)["healthy"] is True


def test_preempted_clock_sampling_is_not_misreported_as_wall_jump(monkeypatch):
    node, now, output, statuses = _adapter(monkeypatch)
    _warm_adapter(node, now)
    sampling = iter([105_100_000_000, 105_160_000_000])
    monkeypatch.setattr(module.time, "monotonic_ns", lambda: next(sampling, 105_160_000_000))
    now[0] = 105.1
    node._odom_cb(_pose(6_100_000_000))
    assert len(output) == 51
    status = json.loads(statuses[-1].data)
    assert status["clock_sample_uncertain"] is True
    assert status["local_clock_ready"] is False
    assert status["healthy"] is False
    assert status["clock_fault_latched"] is False
    assert status["transport_clock_sampling"]["sampling_span_ns"] == 60_000_000
    assert status["last_rejection_reason"] == "clock_pair_sampling_uncertain"
    monkeypatch.setattr(module.time, "monotonic_ns", lambda: int(round(now[0]*1e9)))
    now[0] = 105.2
    node._odom_cb(_pose(6_200_000_000))
    node._status_cb()
    assert len(output) == 52
    assert json.loads(statuses[-1].data)["healthy"] is True


def test_esekf_velocity_is_preferred_only_when_covariance_marks_it_available() -> None:
    message = Odometry()
    message.twist.twist.linear.x = 1.25
    assert _reported_body_velocity(message) is None
    message.twist.covariance[0] = 0.01
    assert _reported_body_velocity(message) == pytest.approx((1.25, 0.0, 0.0))


def test_p4_contract_disables_gps_and_selects_external_nav() -> None:
    params = P4MissionNode.REQUIRED_PARAMS
    assert params["GPS_TYPE"] == 0
    assert params["SIM_GPS_DISABLE"] == 1
    assert params["VISO_TYPE"] == 2
    for name in (
        "EK3_SRC1_POSXY",
        "EK3_SRC1_VELXY",
        "EK3_SRC1_POSZ",
        "EK3_SRC1_VELZ",
        "EK3_SRC1_YAW",
    ):
        assert params[name] == 6


def test_position_setpoints_cannot_cancel_guided_takeoff() -> None:
    phases = P4MissionNode.POSITION_CONTROL_PHASES
    assert "TAKEOFF" not in phases
    assert "ASCEND" not in phases
    assert phases == {"HOVER", "TRACK_SQUARE"}


def test_ros_not_set_is_distinct_from_a_real_fcu_parameter_value() -> None:
    assert ParameterType.PARAMETER_NOT_SET == 0
    assert ParameterType.PARAMETER_INTEGER != ParameterType.PARAMETER_NOT_SET
