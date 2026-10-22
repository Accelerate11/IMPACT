"""Research/FCU handoff and bounded failure landing without a running simulator."""

import json
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from xq_autonomy import ap_research_mission_node as module
from xq_autonomy.ap_research_mission_node import APResearchMissionNode


def _mission(monkeypatch, tmp_path, *, phase="RESEARCH"):
    now = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    node = object.__new__(APResearchMissionNode)
    parameters = {
        "maximum_lio_age_s": 0.5,
        "minimum_progress_m": 1.65,
        "mission_distance_m": 2.0,
        "mission_timeout_s": 240.0,
        "research_timeout_s": 120.0,
        "post_research_hold_s": 2.0,
        "execution_stop_timeout_s": 2.0,
        "failure_land_timeout_s": 30.0,
        "executor_failure_dwell_s": 2.0,
        "executor_status_startup_grace_s": 5.0,
        "hover_duration_s": 2.0,
        "require_freeze_test": False,
        "result_file": str(tmp_path / "mission.json"),
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.get_logger = lambda: SimpleNamespace(info=lambda *_: None, error=lambda *_: None)
    node.phase = phase
    node.phase_started = 100.0
    node.started = 90.0
    node.finalized = False
    node.fcu_state = SimpleNamespace(connected=True, armed=True, mode="GUIDED")
    node.have_state = node.have_odom = True
    node.origin_xyz = (100.0, -50.0, 0.2)
    node.current_xyz = (102.0, -50.0, 2.2)
    node.current_orientation = None
    node.target = None
    node.pending_command = None
    node.last_request = 0.0
    node.arrival_started = None
    node.extnav_status = {"healthy": True, "mavros_subscribers": 1}
    node.extnav_last_wall = now[0]
    node.research_status = {}
    node.executor_status = {
        "enabled": True, "allow_publish": True, "reason": "active",
        "accepted_commands": 4, "published_commands": 4, "last_sequence": 4,
    }
    node.executor_last_wall = now[0]
    node.research_last_wall = 0.0
    node.lio_position = (2.0, 0.0, 2.0)
    node.lio_last_wall = now[0]
    node.lio_stamp_ns = 1_000_000_000
    node.research_start_lio = (0.0, 0.0, 2.0)
    node.research_end_lio = None
    node.research_started_wall = now[0]
    node.research_ended_wall = None
    node.research_stop_started_wall = None
    node.research_stop_sequence = -1
    node.research_stop_ack_wall = None
    node.research_stop_ack_sequence = -1
    node.execution_disabled_acknowledged = False
    node.takeoff_hover_confirmed = node.external_nav_verified = True
    node.verified_params = dict(node.REQUIRED_PARAMS)
    node.failure_reason = node.failure_started_wall = None
    node.health_failure_started_wall = None
    node.events = []
    node._event = lambda kind, detail: node.events.append({"phase": node.phase, "kind": kind, "detail": detail})
    node.enable_messages, node.status_messages = [], []
    node.enable_publisher = SimpleNamespace(publish=node.enable_messages.append)
    node.mission_status_publisher = SimpleNamespace(publish=node.status_messages.append)
    node._publish_origin = lambda: None
    node._poll_command = lambda: None
    node.sent_commands, node.position_targets = [], []
    node._send_command = node.sent_commands.append
    node._publish_setpoint = lambda: node.position_targets.append(node.target)
    return node, now, parameters


def test_hover_uses_lio_reference_and_research_is_the_only_enabled_phase(monkeypatch, tmp_path):
    node, _, _ = _mission(monkeypatch, tmp_path, phase="HOVER")
    node.research_start_lio = None
    node.research_started_wall = None
    node.target = node.current_xyz
    node._arrived_for = lambda _: True

    node._tick()

    assert node.phase == "RESEARCH"
    assert node.research_start_lio == (2.0, 0.0, 2.0)
    assert node.research_start_lio != node.current_xyz
    assert node._progress() == 0.0
    assert node.enable_messages[-1].data is True
    assert json.loads(node.status_messages[-1].data)["research_started_elapsed_s"] == 10.0
    node._transition("RESEARCH_STOP", "test handoff")
    assert node.enable_messages[-1].data is False


def test_completion_disables_velocity_before_fcu_frame_position_hold(monkeypatch, tmp_path):
    node, now, _ = _mission(monkeypatch, tmp_path)
    node.research_status = {"phase": "COMPLETE", "segments_completed": 1, "decisions_applied": 1}

    node._tick()

    assert node.phase == "RESEARCH_STOP"
    assert node.enable_messages[-1].data is False
    assert node.position_targets == []
    assert node.target is None
    assert node.research_stop_sequence == 4
    assert node.research_end_lio == (2.0, 0.0, 2.0)
    assert node._checks()["execution_disabled_acknowledged"] is False

    node.current_xyz = (102.1, -50.2, 2.15)
    now[0] += 0.1
    node._execution_cb(String(data=json.dumps({
        "enabled": False, "allow_publish": False, "reason": "disabled",
        "accepted_commands": 4, "published_commands": 4, "last_sequence": 4,
    })))
    node._tick()

    assert node.phase == "POST_RESEARCH_HOLD"
    assert node.enable_messages[-1].data is False
    assert node.position_targets[-1] == node.current_xyz
    assert node.position_targets[-1] != node.lio_position
    assert node.phase_started == now[0]
    assert node._checks()["execution_disabled_acknowledged"] is True
    handoff = json.loads(node.status_messages[-1].data)["execution_handoff"]
    assert handoff["acknowledged"] is True
    assert handoff["required_last_sequence"] == handoff["ack_last_sequence"] == 4
    assert handoff["ack_delay_s"] == pytest.approx(0.1)
    assert node._progress() == 2.0
    node.lio_position = (1.5, 0.0, 0.2)
    assert node._progress() == 2.0
    now[0] += 1.9
    node._tick()
    assert node.phase == "POST_RESEARCH_HOLD"
    now[0] += 0.2
    node._tick()
    assert node.phase == "LAND"


@pytest.mark.parametrize("changes, receipt_offset", [
    ({}, -0.1),
    ({}, 0.0),
    ({"last_sequence": -1}, 0.1),
    ({"last_sequence": 3}, 0.1),
    ({"last_sequence": True}, 0.1),
    ({"enabled": True}, 0.1),
    ({"allow_publish": True}, 0.1),
    ({"reason": "active"}, 0.1),
])
def test_stop_rejects_old_latched_or_incomplete_acknowledgement(
    monkeypatch, tmp_path, changes, receipt_offset,
):
    node, now, _ = _mission(monkeypatch, tmp_path)
    node.research_status = {"phase": "COMPLETE", "segments_completed": 1, "decisions_applied": 1}
    node._tick()
    node.executor_status.update(
        enabled=False, allow_publish=False, reason="disabled", last_sequence=4,
    )
    node.executor_status.update(changes)
    node.executor_last_wall = node.research_stop_started_wall + receipt_offset
    now[0] += 0.2

    node._tick()

    assert node.phase == "RESEARCH_STOP"
    assert node.position_targets == []
    assert node.execution_disabled_acknowledged is False


def test_stop_ack_timeout_disables_execution_and_performs_failure_landing(monkeypatch, tmp_path):
    node, now, parameters = _mission(monkeypatch, tmp_path)
    node.research_status = {"phase": "COMPLETE", "segments_completed": 1, "decisions_applied": 1}
    node._tick()
    node.lio_position = (2.3, 0.0, 2.0)
    now[0] += parameters["execution_stop_timeout_s"] + 0.1
    # Even a valid-shaped response arriving after the deadline cannot hand off.
    node._execution_cb(String(data=json.dumps({
        "enabled": False, "allow_publish": False, "reason": "disabled", "last_sequence": 4,
    })))

    node._tick()

    assert node.phase == "LAND"
    assert node.failure_reason == "AP execution disable acknowledgement timeout"
    assert node.enable_messages[-1].data is False
    assert node.position_targets == []
    assert node._progress() == 2.0
    assert not node.finalized
    node._tick()
    assert node.sent_commands == ["land"]
    node.fcu_state.armed = False
    node.current_xyz = node.origin_xyz
    node._tick()
    result = json.loads((tmp_path / "mission.json").read_text())
    assert result["status"] == "FAIL"
    assert result["checks"]["execution_disabled_acknowledged"] is False
    assert result["execution_handoff"]["acknowledged"] is False
    # Landing movement must not change the measured research endpoint.
    node.lio_position = (1.5, 0.0, 0.2)
    assert node._progress() == 2.0


def test_no_command_during_initialization_is_allowed_but_stale_navigation_is_not(monkeypatch, tmp_path):
    node, now, _ = _mission(monkeypatch, tmp_path)
    node.executor_status = {"reason": "no_command", "allow_publish": False, "accepted_commands": 0}
    assert node._research_health_reason(now[0]) is None
    node.lio_last_wall = now[0] - 1.0
    assert node._research_health_reason(now[0]) == "LIO odometry stale"


def test_publishing_safe_stop_is_not_a_healthy_active_executor(monkeypatch, tmp_path):
    node, now, _ = _mission(monkeypatch, tmp_path)
    node.executor_status.update(reason="command_source_odom_stale", allow_publish=True)
    assert node._research_health_reason(now[0]) == "AP executor blocked: command_source_odom_stale"


def test_missing_executor_status_has_a_bounded_startup_grace(monkeypatch, tmp_path):
    node, now, _ = _mission(monkeypatch, tmp_path)
    node.executor_status = {}
    assert node._research_health_reason(now[0]) is None
    now[0] += 5.1
    node.lio_last_wall = node.extnav_last_wall = now[0]
    assert node._research_health_reason(now[0]) == "AP execution status missing"


def test_persistent_navigation_failure_requests_landing_after_dwell(monkeypatch, tmp_path):
    node, now, _ = _mission(monkeypatch, tmp_path)
    node.lio_last_wall = 90.0
    node._tick()
    assert node.phase == "RESEARCH"
    now[0] += 2.1
    node._tick()
    assert node.phase == "LAND"
    assert not node.finalized
    assert node.enable_messages[-1].data is False


def test_expired_total_timeout_cannot_prevent_failure_landing_cleanup(monkeypatch, tmp_path):
    node, now, _ = _mission(monkeypatch, tmp_path)
    node.started = -200.0
    node._tick()
    assert node.phase == "LAND"
    node._tick()
    assert node.sent_commands == ["land"]
    assert not node.finalized
    now[0] += 30.1
    node._tick()
    result = json.loads((tmp_path / "mission.json").read_text())
    assert result["status"] == "FAIL"
    assert "deadline" in result["reason"]
    assert result["checks"]["landed_and_disarmed"] is False


@pytest.mark.parametrize("progress", [1.0, 1.8])
def test_final_gate_uses_research_checks_and_requires_disarm(monkeypatch, tmp_path, progress):
    node, _, _ = _mission(monkeypatch, tmp_path, phase="DESCEND")
    node.research_status = {"phase": "DONE", "segments_completed": 1, "decisions_applied": 1}
    node.lio_position = (progress, 0.0, 2.0)
    node.research_end_lio = node.lio_position
    node.fcu_state.armed = False
    node.current_xyz = node.origin_xyz
    node.execution_disabled_acknowledged = True
    node._finish("PASS", "landed")
    result = json.loads((tmp_path / "mission.json").read_text())
    assert result["status"] == ("PASS" if progress >= 1.65 else "FAIL")
    assert "square_and_return" not in result["checks"]
    assert result["checks"]["landed_and_disarmed"] is True


def test_lio_progress_cannot_be_faked_by_fcu_frame_offset_or_nonfinite_sample(monkeypatch, tmp_path):
    node, _, _ = _mission(monkeypatch, tmp_path)
    node.current_xyz = (10_000.0, 20_000.0, 2.0)
    node.lio_position = (0.4, 0.0, 2.0)
    bad = Odometry()
    bad.pose.pose.position.x = float("nan")
    node._lio_cb(bad)
    assert node._progress() == 0.4
    assert node._checks()["executed_progress"] is False


def test_final_gate_cannot_pass_without_disable_acknowledgement(monkeypatch, tmp_path):
    node, _, _ = _mission(monkeypatch, tmp_path, phase="DESCEND")
    node.research_status = {"phase": "DONE", "segments_completed": 1, "decisions_applied": 1}
    node.research_end_lio = node.lio_position
    node.fcu_state.armed = False
    node.current_xyz = node.origin_xyz

    node._finish("PASS", "landed without handoff")

    result = json.loads((tmp_path / "mission.json").read_text())
    assert result["status"] == "FAIL"
    assert result["checks"]["execution_disabled_acknowledged"] is False
    assert "execution_disabled_acknowledged" in result["reason"]
