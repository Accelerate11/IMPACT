import math

import pytest

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType

from xq_autonomy.p4_external_nav_node import (
    P4ExternalNavNode,
    _reported_body_velocity,
    _world_to_body,
)
from xq_autonomy.p4_mission_node import P4MissionNode


def test_world_velocity_is_rotated_into_body_frame() -> None:
    half = math.sqrt(0.5)
    velocity = _world_to_body((1.0, 0.0, 0.0), (half, 0.0, 0.0, half))
    assert velocity == pytest.approx((0.0, -1.0, 0.0), abs=1e-12)


def test_external_nav_covariance_never_claims_false_certainty() -> None:
    source = [0.0] * 36
    source[0] = float("nan")
    result = P4ExternalNavNode._covariance_with_floor(source, 0.0025, 0.0012)
    assert [result[index] for index in (0, 7, 14)] == pytest.approx([0.0025] * 3)
    assert [result[index] for index in (21, 28, 35)] == pytest.approx([0.0012] * 3)
    assert all(math.isfinite(value) for value in result)


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
