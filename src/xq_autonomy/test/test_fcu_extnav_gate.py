import struct

import pytest

from xq_autonomy.fcu_extnav_gate import CurrentExtnavReadiness


def local(now):
    return {'healthy':True,'local_source_ready':True,'local_clock_ready':True,
        'exact_mavros_startup_ready':True,'exact_mavros_startup_gate_enabled':True,
        'acquisition_clock_check_enabled':True,'last_accepted_source_monotonic_ns':now,
        'clock_fault_latched':False}


def sys_status(gate, now, *, health=128, sysid=1, framing=1):
    raw = struct.pack('<III',128,128,health)+b'\0'*19
    payload = [int.from_bytes(raw[i:i+8].ljust(8,b'\0'),'little') for i in range(0,len(raw),8)]
    gate.observe_sys_status(system_id=sysid,component_id=1,message_id=1,framing_status=framing,
        magic=253,payload_length=31,payload64=payload,received_ns=now)


def ready_gate():
    gate = CurrentExtnavReadiness()
    for index in range(51):
        now = 100_000_000_000+index*100_000_000
        gate.observe_local(local(now),now)
        sys_status(gate,now)
    return gate, now


def snap(gate, now, **kwargs):
    return gate.snapshot(now,connected=True,state_received_ns=now,parameters_valid=True,**kwargs)


def test_current_gate_has_real_sample_dwell_and_remains_meaningful_while_armed():
    gate, now = ready_gate()
    result = snap(gate,now)
    assert result['ready'] is True
    assert result['FCU_healthy_sample_dwell_ns'] == 5_000_000_000
    assert result['armed_state_is_not_itself_a_fault'] is True


@pytest.mark.parametrize('field', ['healthy','local_source_ready','local_clock_ready',
    'exact_mavros_startup_ready','exact_mavros_startup_gate_enabled','acquisition_clock_check_enabled'])
def test_current_local_flags_are_all_required(field):
    gate, now = ready_gate()
    status = local(now)
    status[field] = False
    gate.observe_local(status,now)
    assert snap(gate,now)['ready'] is False


def test_new_status_receipt_does_not_refresh_old_source():
    gate, now = ready_gate()
    gate.observe_local(local(now-400_000_000),now)
    assert 'advancing_source_missing_or_stale' in snap(gate,now)['reasons']


def test_wrong_fcu_identity_does_not_refresh_backend_receipt():
    gate, now = ready_gate()
    now += 400_000_000
    gate.observe_local(local(now),now)
    sys_status(gate,now,sysid=2)
    assert 'FCU_SYS_STATUS_missing_or_stale' in snap(gate,now)['reasons']
    assert gate.wrong_identity_count == 1


@pytest.mark.parametrize('changes', [{'health':0},{'framing':0}])
def test_backend_unhealthy_or_bad_expected_frame_revokes_immediately(changes):
    gate, now = ready_gate()
    now += 100_000_000
    gate.observe_local(local(now),now)
    sys_status(gate,now,**changes)
    assert snap(gate,now)['ready'] is False
    assert gate.healthy_start_ns is None


def test_expired_receipt_requires_a_new_complete_healthy_dwell():
    gate, now = ready_gate()
    assert snap(gate,now+400_000_000)['ready'] is False
    now += 500_000_000
    gate.observe_local(local(now),now)
    sys_status(gate,now)
    assert 'FCU_receipt_health_warming_up' in snap(gate,now)['reasons']


def test_local_or_observer_clock_fault_latches_until_new_instance():
    gate, now = ready_gate()
    status = local(now)
    status['clock_fault_latched'] = True
    gate.observe_local(status,now)
    gate.observe_local(local(now),now)
    assert snap(gate,now)['clock_fault_latched'] is True
    fresh, now = ready_gate()
    snap(fresh,now)
    assert snap(fresh,now-1)['clock_fault_latched'] is True


@pytest.mark.parametrize('length',[31,32,39,43])
def test_ap471_mavlink2_sys_status_extensions_preserve_base_health_masks(length):
    gate=CurrentExtnavReadiness()
    raw=struct.pack('<III',128,128,128)+b'\0'*19+struct.pack('<III',1,2,3)
    payload=[int.from_bytes(raw[i:i+8].ljust(8,b'\0'),'little') for i in range(0,length,8)]
    gate.observe_sys_status(system_id=1,component_id=1,message_id=1,framing_status=1,
        magic=253,payload_length=length,payload64=payload,received_ns=100_000_000_000)
    assert gate.sys_healthy is True
    assert gate.sys_masks=={'present':128,'enabled':128,'health':128}


def test_mavlink1_must_not_claim_extension_payload():
    gate=CurrentExtnavReadiness()
    gate.observe_sys_status(system_id=1,component_id=1,message_id=1,framing_status=1,
        magic=254,payload_length=43,payload64=[0]*6,received_ns=100_000_000_000)
    assert gate.sys_healthy is False
