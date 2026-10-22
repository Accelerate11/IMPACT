#!/usr/bin/env python3
"""Isolated ROS graph/source/transport test. The MAVROS endpoint is a fixture.

No real MAVROS, FCU, Gazebo, hardware, or flight commands are used. Actual ROS
publish/subscribe/graph/service paths are exercised; mock identity is disclosed.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-prefix',type=Path,required=True)
    args=parser.parse_args()
    workspace=Path('/home/accelerate/impact_research_ws')
    prefix=args.install_prefix.resolve(strict=True)
    manifest=json.loads((prefix/'.xq_build_manifest.json').read_text())
    for record in manifest['installed_modules'].values():
        if sha(record['installed_path'])!=record['sha256']:
            raise RuntimeError('Installed source has changed')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    run=workspace/'experiments/results/research_multisensor'/f'ap471_ros_transport_{stamp}_{os.getpid()}'
    run.mkdir(exist_ok=False)
    (run/'verify_ap471_ros_transport.py').write_bytes(Path(__file__).read_bytes())
    (run/'build-manifest.json').write_bytes((prefix/'.xq_build_manifest.json').read_bytes())
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from mavros_msgs.srv import CommandBool
    from xq_autonomy import p4_external_nav_node as adapter_module
    from xq_autonomy.ap_research_mission_node import APResearchMissionNode
    expected=prefix/'xq_autonomy/lib/python3.10/site-packages/xq_autonomy/p4_external_nav_node.py'
    if Path(adapter_module.__file__).resolve()!=expected.resolve():
        raise RuntimeError('ROS test imported the wrong installation')
    rclpy.init(args=['--ros-args','-p','use_sim_time:=false',
        '-p','transport_clock_mode:=epoch_anchor_plus_monotonic',
        '-p','require_exact_mavros_startup_gate:=true',
        '-p','require_acquisition_clock_freshness:=true',
        '-p','clock_ledger_path:='+str(run/'adapter-clock-ledger.jsonl'),
        '-p','firmware_profile:=ap471',
        '-p','result_file:='+str(run/'original-mission-result.json')])
    executor=SingleThreadedExecutor()
    adapter=adapter_module.P4ExternalNavNode()
    fixture=rclpy.create_node('ap471_transport_fixture')
    unrelated=rclpy.create_node('unrelated_diagnostic',namespace='/ap471_fixture')
    nodes=[adapter,fixture,unrelated]
    for node in nodes:
        executor.add_node(node)
    statuses,outputs,ledgers=[],[],[]
    reliable=QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE)
    clock_publisher=fixture.create_publisher(Clock,'/clock',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    source_publisher=fixture.create_publisher(Odometry,'/localization/odom',reliable)
    fixture.create_subscription(String,'/xq/p4/extnav/status',lambda message:statuses.append((time.monotonic_ns(),json.loads(message.data))),reliable)
    fixture.create_subscription(String,'/xq/p4/extnav/status/ledger',lambda message:ledgers.append(json.loads(message.data)),reliable)
    unrelated.create_subscription(Odometry,'/uav1/mavros/odometry/out',lambda message:None,reliable)
    synthetic={'stamp':1_000_000_000,'source':True,'advance':True}
    def emit():
        if synthetic['advance']:
            synthetic['stamp']+=50_000_000
        clock_publisher.publish(Clock(clock=Time(nanoseconds=synthetic['stamp']).to_msg()))
        if synthetic['source']:
            odom=Odometry()
            odom.header.stamp=Time(nanoseconds=synthetic['stamp']).to_msg()
            odom.header.frame_id='fixture_acquisition_map'
            odom.child_frame_id='fixture_body'
            odom.pose.pose.orientation.w=1.
            for index in (0,7,14,21,28,35):
                odom.pose.covariance[index]=.01
                odom.twist.covariance[index]=.04
            source_publisher.publish(odom)
    fixture.create_timer(.05,emit)
    markers=[]
    def spin(seconds,label):
        markers.append({'label':label,'monotonic_ns':time.monotonic_ns(),'output_count':len(outputs),'ledger_count':len(ledgers)})
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.02)
    checks={}
    original_clock=None
    arm_calls=[]
    try:
        spin(1.3,'unrelated_subscriber_only')
        checks['unrelated_subscriber_never_opens_identity_gate']=adapter.output_count==0
        mavros_fixture=rclpy.create_node('odometry',namespace='/uav1/mavros')
        nodes.append(mavros_fixture);executor.add_node(mavros_fixture)
        mavros_subscription=mavros_fixture.create_subscription(Odometry,'/uav1/mavros/odometry/out',
            lambda message:outputs.append((time.monotonic_ns(),message.header.stamp.sec*10**9+message.header.stamp.nanosec)),reliable)
        spin(6.8,'exact_fixture_endpoint_visibility_and_warmup')
        checks['exact_fixture_graph_gate_warms_and_delivers_actual_ROS_callbacks']=len(outputs)>70 and bool(statuses) and statuses[-1][1]['healthy'] is True
        original_clock=adapter.get_clock
        adapter.get_clock=lambda: type('FixtureWallClock',(),{'now':lambda _:Time(nanoseconds=original_clock().now().nanoseconds-400_000_000)})()
        spin(.5,'adapter_only_software_wall_backstep_minus_400ms')
        checks['raw_backstep_recorded_without_transport_regression']=any(item['sampling'].get('raw_wall_anomaly') for item in ledgers) and all(b[1]>a[1] for a,b in zip(outputs,outputs[1:]))
        adapter.get_clock=original_clock
        synthetic.update(source=False,advance=False)
        spin(.8,'source_paused_and_acquisition_clock_repeated')
        checks['independent_pause_revokes_local_health']=bool(statuses) and statuses[-1][1]['healthy'] is False
        synthetic.update(source=True,advance=True)
        spin(5.8,'source_resumed_new_full_warmup')
        checks['advancing_source_resume_requires_and_completes_new_warmup']=statuses[-1][1]['healthy'] is True
        mavros_fixture.destroy_subscription(mavros_subscription)
        spin(.5,'exact_fixture_endpoint_removed_unrelated_remains')
        stopped=adapter.output_count
        spin(.5,'confirm_closed_gate_does_not_replay_or_forward')
        checks['endpoint_loss_closes_even_with_unrelated_subscription']=statuses[-1][1]['healthy'] is False and adapter.output_count==stopped and adapter.publisher.get_subscription_count()>0
        # A real typed ROS service would record any mistaken ARM call. The
        # mission's phase/state setup is an explicit test fixture, not a flight.
        service_owner=rclpy.create_node('ap471_arming_service_fixture')
        nodes.append(service_owner);executor.add_node(service_owner)
        def arm(request,response):
            arm_calls.append(bool(request.value));response.success=False;return response
        service_owner.create_service(CommandBool,'/uav1/mavros/cmd/arming',arm)
        mission=APResearchMissionNode();nodes.append(mission);executor.add_node(mission)
        mission.have_state=mission.have_odom=True
        mission.fcu_state.connected=True;mission.fcu_state.armed=False;mission.fcu_state.mode='GUIDED'
        mission.verified_params=dict(mission.AP471_REQUIRED_PARAMS)
        mission.phase='ARM'
        spin(.5,'mission_forced_to_ARM_phase_without_FCU_health_must_block')
        checks['actual_ROS_arming_service_never_called_without_current_FCU_health']=not arm_calls and mission.finalized and mission.failure_reason is not None
    finally:
        if original_clock is not None:
            adapter.get_clock=original_clock
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    final_hash={name:sha(record['installed_path']) for name,record in manifest['installed_modules'].items()}
    checks['installed_modules_unchanged']=all(final_hash[name]==record['sha256'] for name,record in manifest['installed_modules'].items())
    result={'schema_version':1,'status':'PASS' if checks and all(checks.values()) else 'FAIL',
        'scope':'ROS-only transport/source/graph and blocked-mission integration; exact odometry endpoint is a disclosed fixture, not MAVROS/FCU',
        'checks':checks,'markers':markers,'actual_ROS_output_callbacks':len(outputs),'ledger_callbacks':len(ledgers),
        'clock_ledger_lines':sum(1 for _ in (run/'adapter-clock-ledger.jsonl').open()),
        'status_count':len(statuses),'arming_service_calls':arm_calls,
        'host_clock_modified':False,'software_adapter_wall_offset_injected_ns':-400_000_000,
        'firmware_flight_validated':False,'MAVROS_serialization_or_FCU_receipt_validated':False,
        'hardware_accessed':False,'installation':str(prefix),'ROS_DOMAIN_ID':os.environ.get('ROS_DOMAIN_ID')}
    for name,value in (('result.json',result),('statuses.json',statuses),('ROS-output-stamps.json',outputs)):
        with (run/name).open('x') as handle:
            json.dump(value,handle,indent=2,allow_nan=False);handle.write('\n')
    print(json.dumps({'status':result['status'],'run_dir':str(run),'checks':checks}))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
