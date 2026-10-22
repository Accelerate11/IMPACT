#!/usr/bin/env python3
"""Run the frozen five-input experiment; publication is not processing proof."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import traceback


def record(path):
    value=hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda:handle.read(1048576),b''):value.update(block)
    return {'bytes':path.stat().st_size,'sha256':value.hexdigest()}


def write(path,value):
    with path.open('x') as handle:json.dump(value,handle,indent=2,allow_nan=False);handle.write('\n')


def completion(index):
    streams={role:[row['stamp_ns'] for row in index['entries'] if row['sensor']==role]
             for role in ('imu','lidar','front','down','range')}
    if any(not values or any(b<=a for a,b in zip(values,values[1:])) for values in streams.values()):
        raise ValueError('Missing or non-increasing input stream')
    if streams['front']!=streams['down']:raise ValueError('Front/down source acquisition mismatch')
    eligible=[value for value in streams['front'] if streams['lidar'][0]<value<=min(streams['lidar'][-1],streams['imu'][-1])]
    if not eligible:raise ValueError('No covered state epoch')
    counts={key:len(value) for key,value in streams.items()}
    if counts!=index['published_sensor_counts']:raise ValueError('Index count mismatch')
    return dict(expected_lidar_messages=counts['lidar'],expected_imu_messages=counts['imu'],
        expected_image_messages=counts['front'],last_eligible_image_stamp_ns=eligible[-1],
        last_lidar_stamp_ns=streams['lidar'][-1],last_imu_stamp_ns=streams['imu'][-1],
        expected_down_messages=counts['down'],expected_range_messages=counts['range'],
        last_down_stamp_ns=streams['down'][-1],last_range_stamp_ns=streams['range'][-1],
        last_barrier_stamp_ns=max(row['stamp_ns'] for row in index['entries']))


def stop(process):
    if process is None or process.poll() is not None:return {'needed':False}
    os.killpg(process.pid,signal.SIGINT)
    try:code=process.wait(timeout=10);return {'needed':True,'signal':'SIGINT','returncode':code,'forced':False}
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGKILL)
        return {'needed':True,'signal':'SIGKILL','returncode':process.wait(timeout=5),'forced':True}


def check_manifest(workspace,run,label):
    command=['python3',str(workspace/'scripts/check_joint_livo_build_manifest.py'),
             '--workspace-root',str(workspace)]
    with (run/(label+'.log')).open('x') as handle:subprocess.run(command,stdout=handle,stderr=subprocess.STDOUT,check=True)


def arm(workspace,run,index_path,mode,contract,protocol):
    directory=run/mode;directory.mkdir()
    environment=os.environ.copy();environment['ROS_LOG_DIR']=str(directory/'ros_logs')
    publisher=mapper=None;error=None;status={};commands={};start=time.monotonic()
    try:
        commands['publisher']=['python3',str(workspace/'scripts/replay_joint_livo_inputs.py'),'publish',
            '--index',str(index_path),'--output',str(directory),'--rate',str(protocol['replay_rate'])]
        with (directory/'publisher.log').open('x') as publisher_log,(directory/'mapping.log').open('x') as mapping_log:
            publisher=subprocess.Popen(commands['publisher'],env=environment,stdout=publisher_log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+90
            while not (directory/'parameter-ready.json').exists():
                if publisher.poll() is not None:raise RuntimeError('Publisher exited before parameter READY')
                if time.monotonic()>deadline:raise TimeoutError('Publisher parameter READY timeout')
                time.sleep(.05)
            args=dict(contract,joint_inputs=True,flow_mode=mode,visual_update_enabled=True,
                run_dir=str(directory),run_file=str(directory/'visual-updates.jsonl'))
            for key in ('pixel_sigma_px','range_sigma_m','local_slope_sigma','maximum_pair_s'):args[key]=protocol[key]
            commands['mapper']=[str(workspace/'joint_livo_install/xq_fast_livo2/lib/xq_fast_livo2/xq_livo_mapping'),
                '--ros-args','--params-file',str(run/'configuration/r2_snapshot.yaml'),
                '-p','common.img_topic:=/impact/joint/input/front','-p','common.lid_topic:=/impact/joint/input/lidar',
                '-p','common.imu_topic:=/impact/joint/input/imu']
            for key,value in args.items():
                commands['mapper']+=['-p','research.'+key+':='+(str(value).lower() if type(value) is bool else str(value))]
            write(directory/'commands.json',commands)
            mapper=subprocess.Popen(commands['mapper'],env=environment,stdout=mapping_log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=start+protocol['maximum_arm_wall_s']
            while publisher.poll() is None:
                if mapper.poll() is not None:raise RuntimeError('Mapper exited before publication completed: '+str(mapper.returncode))
                if time.monotonic()>deadline:raise TimeoutError('Frozen arm wall timeout')
                time.sleep(.05)
            if publisher.returncode!=0:raise RuntimeError('Publisher failed: '+str(publisher.returncode))
            published=json.loads((directory/'publisher-result.json').read_text())
            if published['status']!='PASS':raise RuntimeError('Publisher integrity/result is not PASS')
            deadline=time.monotonic()+protocol['maximum_mapper_drain_s']
            while not (directory/'processing-result.json').exists():
                if mapper.poll() is not None:raise RuntimeError('Mapper exited without COMPLETE: '+str(mapper.returncode))
                if time.monotonic()>deadline:raise TimeoutError('Five inputs published but processing not COMPLETE')
                time.sleep(.05)
            processed=json.loads((directory/'processing-result.json').read_text())
            if processed.get('status')!='COMPLETE' or not processed.get('joint_inputs',{}).get('all_five_counts_and_tails_verified'):
                raise RuntimeError('Incomplete five-input processing contract')
            status['processing']=processed
    except Exception as exc:error={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
    finally:
        status['mapper_shutdown']=stop(mapper);status['publisher_shutdown']=stop(publisher)
        status.update(mode=mode,status='PASS' if error is None else 'FAIL',error=error,
            mapper_returncode=None if mapper is None else mapper.returncode,
            publisher_returncode=None if publisher is None else publisher.returncode,
            wall_elapsed_s=time.monotonic()-start,scope='five-input native-state integration replay, not AP or hardware')
        if status['mapper_shutdown'].get('forced') or status['publisher_shutdown'].get('forced'):
            status['status']='FAIL'
        if error is None and status['mapper_returncode']!=0:status['status']='FAIL'
        write(directory/'run-result.json',status)
    return status


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root',type=Path,required=True)
    parser.add_argument('--index',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--modes',nargs='+',default=None)
    args=parser.parse_args();workspace=args.workspace_root.resolve();run=args.output.resolve()
    if run.exists():raise ValueError('Experiment output must be a new directory')
    run.mkdir(parents=True);(run/'configuration').mkdir();(run/'sources').mkdir()
    result={'status':'FAIL','arms':[],'error':None};frozen={}
    try:
        protocol=json.loads((workspace/'config/research_joint_livo_replay.json').read_text())
        modes=protocol['modes'] if args.modes is None else args.modes
        if not modes or len(set(modes))!=len(modes) or any(mode not in protocol['modes'] for mode in modes):
            raise ValueError('Invalid repeated/unregistered mode')
        shutil.copy2(workspace/'config/research_joint_livo_replay.json',run/'configuration')
        shutil.copy2(workspace/'src/xq_fast_livo2/config/r2_snapshot.yaml',run/'configuration')
        shutil.copytree(workspace/'src/xq_fast_livo2',run/'sources/xq_fast_livo2')
        for relative in ('scripts/run_joint_livo_replay.py','scripts/run_joint_livo_replay.sh',
            'scripts/replay_joint_livo_inputs.py','scripts/check_joint_livo_build_manifest.py',
            'scripts/build_joint_livo_isolated.sh','scripts/prepare_livo_transport.py'):
            path=workspace/relative;frozen[str(path)]=record(path);shutil.copy2(path,run/'sources'/path.name)
        frozen[str(args.index.resolve())]=record(args.index)
        for path in (workspace/'config/research_joint_livo_replay.json',
                     workspace/'joint_livo_install/.impact-joint-build-manifest.json'):
            frozen[str(path)]=record(path)
            if path.name.startswith('.impact-'):shutil.copy2(path,run/'configuration/joint-build-manifest.json')
        input_dir=run/'input';input_dir.mkdir();index_path=input_dir/'sensor-index.json'
        shutil.copy2(args.index,index_path)
        for name in ('source-stamps.json','pair-profile.json','prepare-result.json','excitation-audit.json'):
            path=args.index.parent/name
            if path.exists():shutil.copy2(path,input_dir/name)
        source=json.loads(index_path.read_text());contract=completion(source)
        write(run/'configuration/completion-contract.json',contract)
        write(run/'frozen-inputs.before.json',frozen)
        check_manifest(workspace,run,'build-freshness.before')
        transport=subprocess.run(['python3',str(workspace/'scripts/prepare_livo_transport.py'),
            '--output-dir',str(run/'configuration/transport')],capture_output=True,text=True)
        (run/'transport-setup.log').write_text(transport.stdout+transport.stderr)
        if transport.returncode:raise RuntimeError('DDS replay transport setup failed')
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(run/'configuration/transport/fastdds-replay.xml')
        for mode in modes:
            entry=arm(workspace,run,index_path,mode,contract,protocol);result['arms'].append(entry)
            # A failed arm is preserved; later independent arms can still provide
            # diagnostics. The suite cannot be PASS unless every requested arm is.
        check_manifest(workspace,run,'build-freshness.after')
        if any(record(Path(path))!=expected for path,expected in frozen.items()):
            raise ValueError('Run source/index changed during experiment')
        write(run/'frozen-inputs.after.json',{path:record(Path(path)) for path in frozen})
        result['status']='PASS' if all(value['status']=='PASS' for value in result['arms']) else 'FAIL'
    except Exception as exc:result['error']={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
    write(run/'suite-result.json',result);print(json.dumps(result,allow_nan=False))
    return 0 if result['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
