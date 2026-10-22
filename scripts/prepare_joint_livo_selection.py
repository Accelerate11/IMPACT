#!/usr/bin/env python3
"""Derive explicit deterministic input subsets; never modify source CDR/hash.

Selection seeds change dropout/stride phase on the SAME flight. They are not
independent trajectories or newly sampled sensor noise. No database is opened.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys


SEEDS = (501,502,503,504,505)
PROFILES = {
    'range_dropout_2s': {'kind':'dropout','roles':['range'],'duration_ns':2_000_000_000},
    'range_dropout_10s': {'kind':'dropout','roles':['range'],'duration_ns':10_000_000_000},
    'camera_dropout_0p2s': {'kind':'dropout','roles':['front','down'],'duration_ns':200_000_000},
    'camera_dropout_1s': {'kind':'dropout','roles':['front','down'],'duration_ns':1_000_000_000},
    'lidar_keep2': {'kind':'stride','roles':['lidar'],'keep_every_n':2},
    'lidar_keep5': {'kind':'stride','roles':['lidar'],'keep_every_n':5},
}
TABLE = {'schema_version':1,'profiles':PROFILES,'seeds':list(SEEDS),
    'dropout_start_after_global_first_acquisition_ns':20_000_000_000,
    'dropout_phase_offset_ns':{str(seed):(seed-501)*20_000_000 for seed in SEEDS},
    'dropout_interval':'half-open [start,end)',
    'lidar_stride_phase':'(seed-501) modulo keep_every_n; retain zero-based source index modulo N equal to phase',
    'maximum_imu_single_sample_gap_ns':200_000_000,
    'seed_scope':'Deterministic selection/phase schedules on one original flight, NOT independent trajectories or sensor-noise realizations.',
    'unimplemented_physical_degradation':['low texture/photometric change','nonflat plane','new simulation noise or bias','real hardware faults']}


def api_module():
    path=Path(__file__).with_name('replay_joint_livo_inputs.py')
    spec=importlib.util.spec_from_file_location('selection_frozen_publisher_contract',path)
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    return module


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path,value):
    with path.open('x',encoding='utf-8') as handle:
        json.dump(value,handle,indent=2,allow_nan=False);handle.write('\n')


def transform(base,profile_name,seed,api):
    api.validate_index(base)
    if 'selection_transform' in base:
        raise ValueError('Chained selections are not declared; select directly from the original five-sensor index')
    if profile_name not in PROFILES or seed not in SEEDS:
        raise ValueError('Profile/seed must belong to the predeclared selection table')
    profile=PROFILES[profile_name]
    original=base['entries'];first=min(row['stamp_ns'] for row in original);last=max(row['stamp_ns'] for row in original)
    imu=[row['stamp_ns'] for row in original if row['sensor']=='imu']
    if max((b-a for a,b in zip(imu,imu[1:])),default=0)>TABLE['maximum_imu_single_sample_gap_ns']:
        raise ValueError('Base IMU already violates the declared 0.2s single-gap limit')
    start=first+TABLE['dropout_start_after_global_first_acquisition_ns']+TABLE['dropout_phase_offset_ns'][str(seed)]
    end=start+profile.get('duration_ns',0)
    if profile['kind']=='dropout' and (start<=first or end>=last):
        raise ValueError('Declared dropout is not strictly inside the recorded input span')
    selected,removed=[],[];role_indices={role:0 for role in api.ROLES}
    phase=(seed-501)%profile.get('keep_every_n',1)
    for source_sequence,row in enumerate(original,1):
        role=row['sensor'];index=role_indices[role];role_indices[role]+=1
        drop=(role in profile['roles'] and
            (start<=row['stamp_ns']<end if profile['kind']=='dropout' else index%profile['keep_every_n']!=phase))
        if drop:
            removed.append({'original_global_sequence':source_sequence,'original_role_index_zero_based':index,
                'source':copy.deepcopy(row),'reason':profile_name,'seed':seed})
        else:selected.append(copy.deepcopy(row))
    if not removed:
        raise ValueError('Selection schedule removed no original sensor record')
    result=copy.deepcopy(base);result['entries']=selected
    result['base_source_counts']=copy.deepcopy(base['source_counts'])
    counts={role:sum(row['sensor']==role for row in selected) for role in api.ROLES}
    result['published_sensor_counts']=counts
    for role,count in counts.items():
        result['source_counts'][base['sensor_definitions'][role]['source_topic']]=count
    result['pair_profile']=api.pair_profile(selected)
    old_cameras=[row['stamp_ns'] for row in original if row['sensor']=='front']
    old_order={stamp:index for index,stamp in enumerate(old_cameras)}
    bridged=[pair for pair in result['pair_profile']['pairs'] if old_order[pair['t1_ns']]!=old_order[pair['t0_ns']]+1]
    derived_imu=[row for row in selected if row['sensor']=='imu']
    if derived_imu!=[row for row in original if row['sensor']=='imu']:
        raise ValueError('This profile must not remove or alter IMU samples')
    result['selection_transform']={'schema_version':1,'profile':profile_name,'seed':seed,
        'kind':profile['kind'],'roles':profile['roles'],'removed_record_count':len(removed),
        'retained_record_count':len(selected),'original_record_count':len(original),
        'dropout_start_ns':start if profile['kind']=='dropout' else None,
        'dropout_end_ns':end if profile['kind']=='dropout' else None,
        'keep_every_n':profile.get('keep_every_n'),'retained_lidar_phase':phase if profile['kind']=='stride' else None,
        'source_counts_semantics':'Selected sensor rows; CameraInfo counts are inherited calibration-source counts, not published streams.',
        'camera_keep_every_n_semantics':'No periodic camera decimation; an explicit grouped dropout can remove original front/down records.',
        'next_camera_semantics':'Next retained original acquisition record after explicit selection; may skip original base frames.',
        'pairs_bridging_dropped_base_camera_frames':bridged,
        'maximum_imu_single_sample_gap_ns':max((b-a for a,b in zip(imu,imu[1:])),default=0),
        'lidar_outside_original_imu_endpoint_coverage_count':sum(
            row['sensor']=='lidar' and not(imu[0]<=row['stamp_ns']<=imu[-1]) for row in selected),
        'imu_coverage_scope':'Original IMU is fully retained; endpoint coverage and native per-scan readiness still require runtime checks. No new IMU samples are invented.',
        'imu_stream_unchanged':True,'original_payloads_headers_and_raw_family_hashes_unchanged':True,
        'intentional_dropout_is_not_unexpected_transport_loss':True,'seed_scope':TABLE['seed_scope'],
        'performance_or_robustness_validated':False}
    api.validate_index(result)
    if result['source_files']!=base['source_files'] or result['raw_files']!=base['raw_files']:
        raise ValueError('Original raw-family identities must never be relabeled')
    original_rows={(row['file'],row['rowid']):row for row in original}
    if any(original_rows[(row['file'],row['rowid'])]!=row for row in selected):
        raise ValueError('Selection altered an original payload reference/header')
    return result,removed


def output_path(base_index,bag,requested,api):
    output=api.safe_output(bag,requested)
    base_directory=base_index.resolve().parent
    if output.exists():raise ValueError('Derived index output must be new')
    if output==base_directory or output in base_directory.parents or base_directory in output.parents:
        raise ValueError('Derived output must not overlap its frozen base-index directory')
    return output


def run(args):
    api=api_module();base_bytes=args.base_index.read_bytes();base_sha=hashlib.sha256(base_bytes).hexdigest()
    base=json.loads(base_bytes);derived,removed=transform(base,args.profile,args.seed,api)
    output=output_path(args.base_index,Path(base['bag']).resolve(),args.output,api)
    output.mkdir(parents=True,exist_ok=False)
    source_paths=[Path(__file__),Path(__file__).with_name('replay_joint_livo_inputs.py')]
    sources=output/'sources';sources.mkdir()
    tool_records={}
    for source in source_paths:
        value=source.read_bytes();(sources/source.name).write_bytes(value)
        tool_records[str(source)]={'bytes':len(value),'sha256':hashlib.sha256(value).hexdigest()}
    derived['selection_transform']['base_sensor_index']={'path':str(args.base_index.resolve()),'bytes':len(base_bytes),'sha256':base_sha}
    write(output/'sensor-index.json',derived);write(output/'pair-profile.json',derived['pair_profile'])
    stamps={role:[row['stamp_ns'] for row in derived['entries'] if row['sensor']==role] for role in api.ROLES}
    write(output/'source-stamps.json',{'stamps_ns':stamps,'front_camera_stamps_s':[v*1e-9 for v in stamps['front']],
        'down_camera_stamps_s':[v*1e-9 for v in stamps['down']],'imu_stamps_s':[v*1e-9 for v in stamps['imu']],
        'lidar_stamps_s':[v*1e-9 for v in stamps['lidar']],'range_stamps_s':[v*1e-9 for v in stamps['range']]})
    write(output/'selection-table.json',TABLE)
    with (output/'removed-source-rows.jsonl').open('x') as handle:
        for row in removed:handle.write(json.dumps(row,separators=(',',':'),allow_nan=False)+'\n')
    unchanged=digest(args.base_index)==base_sha and all(digest(Path(path))==record['sha256'] for path,record in tool_records.items())
    report={'status':'PASS' if unchanged else 'FAIL','profile':args.profile,'seed':args.seed,
        'base_index_sha256_before':base_sha,'base_index_sha256_after':digest(args.base_index),
        'base_index_and_tool_sources_unchanged':unchanged,'tool_records':tool_records,
        'derived_sensor_index_sha256':digest(output/'sensor-index.json'),
        'selected_counts':derived['published_sensor_counts'],'removed_records':len(removed),
        'source_files_and_raw_files_identity_descriptors_unchanged':True,
        'imu_stream_unchanged':True,'original_databases_opened_or_rehashed':False,
        'exact_seed_opportunities':derived['pair_profile']['exact_seed_opportunities'],
        'valid_nonoverlapping_pairs':derived['pair_profile']['valid_nonoverlapping_pair_count'],
        'camera_bridging_pairs':len(derived['selection_transform']['pairs_bridging_dropped_base_camera_frames']),
        'required_evaluation':['All actually selected messages received and completed under cumulative barriers',
            'Report actual raw-flow update/rejection availability and native fallback',
            'For camera dropout report VIO odometry output gaps and adapter freshness gates; no timer republishing stale state',
            'LiDAR sparsity still requires original IMU coverage and max single-gap<=0.2s',
            'All methods receive the same exact derived index and explicit fault schedule'],
        'seed_scope':TABLE['seed_scope'],'formal_matrix_executed':False,'robustness_pass_claimed':False,
        'scope':'Preparation-only deterministic selection subset. Original CDR/header/hash descriptors retained without database I/O; the frozen publisher later independently rehashes and reads only allowed original sensor rows.'}
    write(output/'prepare-result.json',report)
    print(json.dumps({'status':report['status'],'output':str(output),'selected_counts':report['selected_counts'],
        'removed_records':len(removed),'index_sha256':report['derived_sensor_index_sha256']}))
    return 0 if report['status']=='PASS' else 1


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list-profiles',action='store_true')
    parser.add_argument('--base-index',type=Path)
    parser.add_argument('--profile',choices=tuple(PROFILES))
    parser.add_argument('--seed',type=int,choices=SEEDS)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    if args.list_profiles:print(json.dumps(TABLE,indent=2));return 0
    if any(value is None for value in (args.base_index,args.profile,args.seed,args.output)):
        parser.error('--base-index, --profile, --seed and --output are required')
    try:return run(args)
    except (OSError,ValueError,TypeError,KeyError) as exc:
        print(json.dumps({'status':'FAIL','error':type(exc).__name__+': '+str(exc)}));return 1


if __name__=='__main__':raise SystemExit(main())
