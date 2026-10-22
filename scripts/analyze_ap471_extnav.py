#!/usr/bin/env python3
"""Closed AP471 unarmed evidence audit. Never opens a SQLite database."""
import argparse
import hashlib
import json
from pathlib import Path

from analyze_extnav_link_probe import load_fcu
from analyze_extnav_guard_probe import wire_integrity, parameter_value, parameter_name, wire_health


def read(path):return json.loads(path.read_text())
def rows(path):return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def increasing(values):return bool(values) and all(b>a for a,b in zip(values,values[1:]))
def inventory(run):
    return {str(p.relative_to(run)):{'bytes':p.stat().st_size,'sha256':sha(p)} for p in sorted(run.rglob('*'))
        if p.is_file() and p.name not in {'analysis.log','link-probe-result.json'}}


def analyze(run,protocol_path):
    before=inventory(run);checks={};report={'status':'FAIL','checks':checks}
    protocol=read(protocol_path)
    wire=rows(run/'wire/traffic.jsonl');proxy=read(run/'wire/summary.json')
    ledger=rows(run/'adapter-clock-ledger.jsonl')
    monitor=rows(run/'readiness/events.jsonl');monitored=read(run/'readiness/summary.json')
    observer=rows(run/'ros-observer/events.jsonl');observed=read(run/'ros-observer/summary.json')
    proof=read(run/'configuration/fcu-parameter-proof.json');case=read(run/'run.json')
    shutdown=read(run/'shutdown-result.json')
    bins=sorted((run/'sitl_runtime/logs').glob('*.BIN'))
    if len(bins)!=1:raise ValueError('Exactly one original FCU BIN is required')
    fcu,params,fcu_meta=load_fcu(bins[0])
    frames=[row for row in wire if row.get('kind')=='frame']
    outgoing=[row for row in frames if row.get('direction')=='to_fcu' and row.get('msgid')==331]
    keys=[int(row['time_usec']) for row in outgoing]
    forwarded=[row for row in ledger if row['decision']=='forwarded']
    mapped=[int(row['output_remote_time_us']) for row in forwarded]
    checks['pinned_4_7_1_firmware']=case['firmware_commit']==protocol['firmware_commit']
    checks['owned_process_groups_all_closed']=bool(shutdown.get('processes')) and all(row.get('group_still_exists') is False for row in shutdown['processes'])
    checks['no_mission_or_actual_flow_update']=case.get('armed_trial') is False and case.get('mission_nodes_started') is False and case.get('actual_flow_state_updates')==0
    checks['wire_proxy_byte_and_decode_integrity']=proxy.get('status')=='PASS' and wire_integrity(proxy)
    checks['unarmed_proxy_guard_without_blocked_attempt']=proxy.get('guard_unarmed') is True and proxy.get('guard_blocking_occurred') is False
    checks['never_armed']=not monitored.get('armed_seen') and not any(row.get('direction')=='from_fcu' and row.get('msgid')==0 and int(row.get('base_mode',0))&128 for row in frames) and not any(int(row.get('ArmState',0)) for row in fcu['ARM'])
    checks['observer_and_current_health_monitor_completed']=observed.get('status')=='PASS' and monitored.get('status')=='PASS'
    checks['wire_and_ledger_have_predeclared_minimum']=len(keys)>=protocol['minimum_wire_odometry_messages'] and len(mapped)>=protocol['minimum_clock_ledger_forwarded']
    checks['mapped_and_wire_microsecond_stamps_strictly_increase']=increasing(mapped) and increasing(keys)
    checks['every_forwarded_adapter_frame_has_ordered_wire_match']=bool(mapped) and mapped==keys
    for kind in ('VISP','VISV'):
        checks['every_wire_ODOMETRY_ordered_'+kind]=bool(keys) and keys==[int(row['RTimeUS']) for row in fcu[kind]]
        checks['FCU_'+kind+'_not_ignored']=bool(fcu[kind]) and all(int(row['Ign'])==0 for row in fcu[kind])
        checks['FCU_'+kind+'_quality_satisfies_threshold']=bool(fcu[kind]) and all(row['Q']>=params.get('VISO_QUAL_MIN',101) for row in fcu[kind])
    checks['ODOMETRY_LOCAL_FRD_BODY_FRD']=bool(outgoing) and all(row['frame_id']==20 and row['child_frame_id']==12 for row in outgoing)
    checks['proof_is_read_only_and_bound_to_frozen_config']=proof.get('status')=='PASS' and proof.get('read_only') is True and proof.get('parameter_writes')==0 and proof.get('flight_commands')==0 and proof.get('config_sha256')==sha(protocol_path)
    required=protocol['live_readiness']['required_parameters']
    wire_params={name:[] for name in required}
    for row in frames:
        if row.get('direction')=='from_fcu' and row.get('msgid')==22:
            fields=row.get('fields',{});name=parameter_name(fields.get('param_id'))
            if name in wire_params:wire_params[name].append((row.get('sysid'),row.get('compid'),fields.get('param_value')))
    for name,expected in required.items():
        checks['parameter_proof_wire_BIN:'+name]=(parameter_value(proof['parameters'].get(name))==expected and
            bool(wire_params[name]) and all(value==(1,1,expected) for value in wire_params[name]) and params.get(name)==expected)
    readiness=[row for row in monitor if row['kind']=='readiness']
    locals_=[row for row in monitor if row['kind']=='local_status']
    checks['current_ready_observed_from_real_FCU_health']=any(row['data']['ready'] for row in readiness)
    checks['independent_acquisition_and_exact_graph_checks_enabled']=bool(locals_) and all(row['data'].get('acquisition_clock_check_enabled') is True and row['data'].get('exact_mavros_startup_gate_enabled') is True for row in locals_)
    checks['full_raw_acquisition_wall_monotonic_mapping_ledger']=bool(ledger) and all(
        isinstance(row.get('raw_acquisition_stamp_ns'),int) and isinstance(row.get('sampling',{}).get('raw_wall_ns'),int)
        and isinstance(row['sampling'].get('monotonic_before_ns'),int) and isinstance(row['sampling'].get('monotonic_after_ns'),int)
        and row.get('source_frames_queued_or_replayed') is False for row in ledger)
    checks['no_forwarded_frame_reuses_acquisition_stamp']=increasing([row['raw_acquisition_stamp_ns'] for row in forwarded])
    health=wire_health(frames,128)
    checks['raw_FCU_SYS_STATUS_masks_valid_identity_1_1']=bool(health['samples']) and not health['errors']
    pause={row['kind']:row['monotonic_ns'] for row in observer if row.get('kind') in {'source_pause_started','source_pause_ended'}}
    if case['source']=='synthetic':
        complete=set(pause)=={'source_pause_started','source_pause_ended'}
        checks['source_pause_markers_observed']=complete
        if complete:
            start,end=pause['source_pause_started'],pause['source_pause_ended']
            checks['ready_before_pause']=any(row['data']['ready'] and row['monotonic_ns']<start for row in readiness)
            paused=[row for row in readiness if start+600_000_000<=row['monotonic_ns']<=end]
            checks['current_ready_revoked_during_pause']=bool(paused) and all(not row['data']['ready'] for row in paused)
            checks['FCU_backend_receipt_lost_during_pause']=any(not row['combined_backend_healthy'] and start<row['monotonic_ns']<end for row in health['samples'])
            checks['new_warmup_prevents_immediate_resume_ready']=not any(row['data']['ready'] and end<=row['monotonic_ns']<end+4_500_000_000 for row in readiness)
            checks['current_ready_recovered_after_new_warmup']=any(row['data']['ready'] and row['monotonic_ns']>=end+5_000_000_000 for row in readiness)
    binding=read(run/'source-binding.json')
    checks['all_frozen_helpers_match_original_binding']=all(sha(run/'sources'/name)==record['sha256'] for name,record in binding.items())
    dependencies=read(run/'runtime-dependencies.json')
    checks['runtime_dependency_SHA_unchanged']=all(sha(Path(name))==record['sha256'] for name,record in dependencies.items())
    after=inventory(run)
    checks['all_original_file_sets_and_SHA_unchanged_by_analysis']=before==after
    report.update(status='PASS' if all(checks.values()) else 'FAIL',case=case,
        counts={'source_inputs':len(ledger),'adapter_forwarded':len(mapped),'wire_ODOMETRY':len(keys),
                'FCU_VISP':len(fcu['VISP']),'FCU_VISV':len(fcu['VISV']),
                'startup_skipped':sum(row['decision']=='startup_skipped' for row in ledger),
                'source_rejected':sum(row['decision']=='rejected' for row in ledger),
                'current_ready_samples':sum(row['data']['ready'] for row in readiness),
                'raw_wall_anomaly_inputs':sum(bool(row['sampling'].get('raw_wall_anomaly')) for row in ledger)},
        source_pause=pause,FCU_health=health,FCU_parser=fcu_meta,owned_shutdown=shutdown,
        input_manifest_before=before,input_manifest_after=after,
        scope=protocol['scope'],hardware_accessed=False,actual_flow_state_updates=0,
        flight_validated=False,EKF_fusion_validated=False,
        packet_loss_rate_or_UTC_calibration_claimed=False)
    with (run/'link-probe-result.json').open('x') as out:json.dump(report,out,indent=2);out.write('\n')
    print(json.dumps({'status':report['status'],'counts':report['counts'],'failed_checks':[name for name,value in checks.items() if not value]}))
    return 0 if report['status']=='PASS' else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True);parser.add_argument('--protocol',type=Path,required=True)
    args=parser.parse_args();return analyze(args.run_dir,args.protocol)


if __name__=='__main__':raise SystemExit(main())
