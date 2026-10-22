#!/usr/bin/env python3
"""Observe the same current receipt core as the AP471 mission; no commands."""
import argparse
import json
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--duration',type=float,required=True)
    parser.add_argument('--parameter-proof',type=Path,required=True)
    args=parser.parse_args()
    args.output_dir.mkdir(exist_ok=False)
    proof=json.loads(args.parameter_proof.read_text())
    if proof.get('status')!='PASS':
        raise ValueError('Successful exact read-only parameter proof is required')
    import rclpy
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from std_msgs.msg import String
    from mavros_msgs.msg import Mavlink,State
    from xq_autonomy.fcu_extnav_gate import CurrentExtnavReadiness
    rclpy.init()
    node=rclpy.create_node('ap471_current_extnav_observer')
    core=CurrentExtnavReadiness()
    events=(args.output_dir/'events.jsonl').open('x',buffering=1)
    state={'connected':False,'armed':False,'received_ns':None}
    snapshots=[]
    def record(kind,value):
        events.write(json.dumps({'kind':kind,'monotonic_ns':time.monotonic_ns(),
            'wall_ns':time.time_ns(),'data':value},allow_nan=False)+'\n')
    def local(message):
        try:
            value=json.loads(message.data)
        except ValueError:
            value=None
        core.observe_local(value,time.monotonic_ns());record('local_status',value)
    def fcu(message):
        if message.msgid!=1:
            return
        value={'system_id':int(message.sysid),'component_id':int(message.compid),
            'message_id':int(message.msgid),'framing_status':int(message.framing_status),
            'magic':int(message.magic),'payload_length':int(message.len),
            'payload64':[int(value) for value in message.payload64], 'received_ns':time.monotonic_ns()}
        core.observe_sys_status(**value);record('FCU_SYS_STATUS',value)
    def state_cb(message):
        state.update(connected=bool(message.connected),armed=bool(message.armed),received_ns=time.monotonic_ns())
        record('state',dict(state))
    def snapshot():
        value=core.snapshot(time.monotonic_ns(),connected=state['connected'],
            state_received_ns=state['received_ns'],parameters_valid=True)
        snapshots.append(value);record('readiness',value)
    reliable=QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE)
    best=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(String,'/xq/p4/extnav/status',local,reliable)
    node.create_subscription(Mavlink,'/uas1/mavlink_source',fcu,best)
    node.create_subscription(State,'/uav1/mavros/state',state_cb,best)
    steady=Clock(clock_type=ClockType.STEADY_TIME)
    node.create_timer(.1,snapshot,clock=steady)
    started=time.monotonic()
    error=None
    try:
        while time.monotonic()-started<args.duration:
            rclpy.spin_once(node,timeout_sec=.05)
            if state['armed']:
                raise RuntimeError('Armed state observed during unarmed experiment')
    except BaseException as exc:
        error=repr(exc)
    finally:
        events.close();node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    result={'status':'PASS' if error is None else 'FAIL','error':error,
        'scope':'Monitor completion only; actual readiness/flow and flight assessed separately',
        'ever_ready':any(item['ready'] for item in snapshots),'current_ready':bool(snapshots and snapshots[-1]['ready']),
        'ready_snapshots':sum(item['ready'] for item in snapshots),'snapshots':len(snapshots),
        'armed_seen':state['armed'],'elapsed_monotonic_s':time.monotonic()-started}
    with (args.output_dir/'summary.json').open('x') as out:
        json.dump(result,out,indent=2);out.write('\n')
    print(json.dumps(result))
    return 0 if error is None else 1


if __name__=='__main__':
    raise SystemExit(main())
