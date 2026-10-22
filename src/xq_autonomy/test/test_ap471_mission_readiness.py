"""The research mission must recheck current health at actuation boundaries."""
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.time import Time

from xq_autonomy.ap_research_mission_node import APResearchMissionNode
from xq_autonomy import ap_research_mission_node as module
from test_ap_research_mission import _mission


def enabled_mission(monkeypatch, tmp_path, phase):
    node, now, parameters = _mission(monkeypatch, tmp_path, phase=phase)
    monkeypatch.setattr(module.time, 'monotonic_ns', lambda:int(now[0]*1e9))
    node.firmware_profile = 'ap471'
    node.REQUIRED_PARAMS = dict(node.AP471_REQUIRED_PARAMS)
    node.verified_params = dict(node.REQUIRED_PARAMS)
    node.require_continuous_ready = True
    node.last_live_readiness = None
    node.live_abort_pending_future = None
    node.live_abort_pending_done_wall = None
    node.state_received_monotonic_ns = int(now[0]*1e9)
    ready = {'ready':False,'reasons':['FCU_SYS_STATUS_missing_or_stale']}
    node._current_live_ready = lambda: ready
    return node, now, parameters, ready


@pytest.mark.parametrize('phase', ['SET_GUIDED','ARM','TAKEOFF','ASCEND','HOVER','RESEARCH',
    'RESEARCH_STOP','POST_RESEARCH_HOLD'])
def test_current_health_loss_aborts_without_old_two_second_research_dwell(monkeypatch, tmp_path, phase):
    node, now, parameters, ready = enabled_mission(monkeypatch,tmp_path,phase)
    node._tick()
    assert node.phase == 'LAND'
    assert node.failure_reason.startswith('current ExternalNav readiness lost:')
    assert node.enable_messages[-1].data is False
    assert node.sent_commands == []
    node._tick()
    assert node.sent_commands == ['land']


@pytest.mark.parametrize('kind', ['arm','takeoff'])
def test_dispatch_checks_current_readiness_independently_of_earlier_nav_phase(monkeypatch,tmp_path,kind):
    node, now, parameters, ready = enabled_mission(monkeypatch,tmp_path,kind.upper())
    node.fcu_state.armed = kind == 'takeoff'
    node._send_command = APResearchMissionNode._send_command.__get__(node)
    calls=[]
    client=SimpleNamespace(service_is_ready=lambda:True,call_async=lambda request:calls.append(request))
    node.arm_client=node.takeoff_client=client
    node._send_command(kind)
    assert calls == []
    assert any(event['kind']=='COMMAND_BLOCKED' for event in node.events)
    assert node.enable_messages[-1].data is False


def test_pending_arm_is_not_cancelled_or_assumed_safe_while_disarmed(monkeypatch,tmp_path):
    node, now, parameters, ready = enabled_mission(monkeypatch,tmp_path,'ARM')
    node.fcu_state.armed=False
    done=[False]
    pending=SimpleNamespace(done=lambda:done[0])
    node.pending_command=('arm',pending)
    node._tick()
    assert node.phase=='LAND'
    assert node.live_abort_pending_future is pending
    assert node.finalized is False
    node._tick()
    assert node.sent_commands==['land']
    assert node.finalized is False
    done[0]=True
    now[0]+=.1
    node._tick()
    assert node.finalized is False
    now[0]+=2.1
    node.state_received_monotonic_ns=int(now[0]*1e9)
    node._tick()
    assert node.finalized is True


def test_arm_ack_before_heartbeat_does_not_create_an_unobserved_abort_window(monkeypatch,tmp_path):
    node,now,parameters,ready=enabled_mission(monkeypatch,tmp_path,'ARM')
    node.fcu_state.armed=False
    node.latest_actuation_future=SimpleNamespace(done=lambda:True)
    node.pending_command=None  # The ordinary command poll already consumed ACK.
    node._tick()
    assert node.phase=='LAND'
    assert node.finalized is False
    assert node.live_abort_pending_future is node.latest_actuation_future


def test_cleanup_cannot_finalize_using_a_stale_disarmed_heartbeat(monkeypatch,tmp_path):
    node,now,parameters,ready=enabled_mission(monkeypatch,tmp_path,'ARM')
    node.fcu_state.armed=False
    node.latest_actuation_future=SimpleNamespace(done=lambda:True)
    node._tick();now[0]+=.1;node._tick()
    now[0]+=2.1
    node._tick()
    assert node.finalized is False
    node.state_received_monotonic_ns=int(now[0]*1e9)
    node._tick()
    assert node.finalized is True


def test_fcu_pose_replay_cannot_refresh_age_or_change_control_reference(monkeypatch,tmp_path):
    node,now,parameters,ready=enabled_mission(monkeypatch,tmp_path,'VERIFY_NAV')
    node.fcu_odom_stamp_ns=1_000_000_000
    node.fcu_odom_received_monotonic_ns=int(now[0]*1e9)
    original=node.current_xyz
    source=Odometry();source.header.stamp=Time(nanoseconds=1_000_000_000).to_msg()
    source.pose.pose.orientation.w=1.;source.pose.pose.position.x=123.
    now[0]+=.4
    node._odom_cb(source)
    assert node.current_xyz==original
    assert node.fcu_odom_received_monotonic_ns==100_000_000_000
    source.header.stamp=Time(nanoseconds=1_100_000_000).to_msg()
    node._odom_cb(source)
    assert node.current_xyz[0]==123.
    assert node.fcu_odom_received_monotonic_ns==int(now[0]*1e9)


def test_mission_current_gate_requires_fresh_fcu_pose_even_if_backend_healthy(monkeypatch,tmp_path):
    node,now,parameters,ready=enabled_mission(monkeypatch,tmp_path,'VERIFY_NAV')
    node.live_extnav_gate=SimpleNamespace(snapshot=lambda *a,**kw:{'ready':True,'reasons':[]})
    node._current_live_ready=APResearchMissionNode._current_live_ready.__get__(node)
    node.fcu_odom_received_monotonic_ns=99_500_000_000
    node.fcu_odom_stamp_ns=1_000_000_000
    assert node._current_live_ready()['ready'] is False
    node.fcu_odom_received_monotonic_ns=int(now[0]*1e9)
    assert node._current_live_ready()['ready'] is True


@pytest.mark.parametrize('value',[1.2,float('nan'),float('inf')])
def test_ap471_parameter_readback_never_rounds_or_accepts_nonfinite(monkeypatch,tmp_path,value):
    node,now,parameters,ready=enabled_mission(monkeypatch,tmp_path,'VERIFY_NAV')
    node.fcu_state.armed=False
    node.param_names=list(node.REQUIRED_PARAMS)
    node.param_index=0
    parameter=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=value)
    node.pending_param=('VISO_TYPE',SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(values=[parameter])))
    node._poll_param()
    assert node.finalized is True
    assert 'did not verify' in node.failure_reason


def test_gps_check_uses_new_names_and_skipchk_must_be_zero(monkeypatch,tmp_path):
    node,now,parameters,ready=enabled_mission(monkeypatch,tmp_path,'VERIFY_NAV')
    assert node._checks()['gps_disabled'] is True
    node.verified_params['SIM_GPS1_ENABLE']=1
    assert node._checks()['gps_disabled'] is False
    assert node.REQUIRED_PARAMS['ARMING_SKIPCHK']==0
    assert 'ARMING_CHECK' not in node.REQUIRED_PARAMS


def test_replayed_lio_stamp_cannot_refresh_progress_receipt(monkeypatch,tmp_path):
    node,now,parameters,ready=enabled_mission(monkeypatch,tmp_path,'VERIFY_NAV')
    previous=node.lio_last_wall
    source=Odometry()
    source.header.stamp=Time(nanoseconds=node.lio_stamp_ns).to_msg()
    source.pose.pose.position.x=123.
    now[0]+=.5
    node._lio_cb(source)
    assert node.lio_last_wall==previous
    assert node.lio_position[0]!=123.
