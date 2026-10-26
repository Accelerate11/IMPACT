#!/usr/bin/env python3
"""Run the retained mathematical studies from a relocatable source bundle."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback


def default_root():
    script=Path(__file__).resolve()
    return script.parents[2] if script.parent.parent.name=='packaging' else script.parents[1]


def positive_timeout(value):
    try:
        number=float(value)
    except ValueError:
        raise argparse.ArgumentTypeError('timeout must be a positive finite number') from None
    if not math.isfinite(number) or number<=0:
        raise argparse.ArgumentTypeError('timeout must be a positive finite number')
    return number


def live_group_members(group):
    """Inspect only our owned session; exited zombies cannot continue work."""
    if Path('/proc').is_dir():
        members=[]
        for entry in Path('/proc').iterdir():
            if not entry.name.isdecimal():
                continue
            try:
                fields=(entry/'stat').read_text().rpartition(') ')[2].split()
                if int(fields[2])==group and fields[0]!='Z':
                    members.append(int(entry.name))
            except (OSError,ValueError,IndexError):
                continue
        return members
    try:
        os.killpg(group,0)
        return [group]
    except ProcessLookupError:
        return []


def stop_owned_group(process):
    """Finish or stop this child's process group, without a global process kill."""
    previous=signal.signal(signal.SIGINT,signal.SIG_IGN)
    sent=[]
    try:
        for sig in (signal.SIGTERM,signal.SIGKILL):
            process.poll()
            if not live_group_members(process.pid):
                break
            try:
                os.killpg(process.pid,sig)
                sent.append(sig.name)
            except ProcessLookupError:
                break
            deadline=time.monotonic()+1.0
            while time.monotonic()<deadline:
                process.poll()
                if not live_group_members(process.pid):
                    break
                time.sleep(.02)
        process.poll()
        return {'signals':sent,'owned_remaining':live_group_members(process.pid)}
    finally:
        signal.signal(signal.SIGINT,previous)


def run_study(name,command,root,output,env,timeout):
    started=datetime.now(timezone.utc).isoformat()
    process=None
    code=None
    timed_out=interrupted=launch_failed=False
    error=None
    cleanup={'signals':[],'owned_remaining':[]}
    with (output/(name+'.log')).open('x') as log:
        try:
            # Keep the handle if Ctrl+C arrives between fork and assignment.
            old_mask=signal.pthread_sigmask(signal.SIG_BLOCK,{signal.SIGINT})
            try:
                process=subprocess.Popen(command,cwd=root,env=env,stdout=log,
                    stderr=subprocess.STDOUT,start_new_session=True)
            finally:
                signal.pthread_sigmask(signal.SIG_SETMASK,old_mask)
            code=process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out=True
            error='Study exceeded its timeout'
            print(error,file=log)
        except KeyboardInterrupt:
            interrupted=True
            error='Interrupted by Ctrl+C'
            print(error,file=log)
        except Exception as caught:
            launch_failed=process is None
            error=type(caught).__name__+': '+str(caught)
            traceback.print_exc(file=log)
        finally:
            if process is not None:
                cleanup=stop_owned_group(process)
                code=process.poll()
    status='INTERRUPTED' if interrupted else (
        'PASS' if code==0 and not timed_out and error is None
        and not cleanup['owned_remaining'] else 'FAIL')
    return {'study':name,'status':status,'returncode':code,
        'timed_out':timed_out,'interrupted':interrupted,'launch_failed':launch_failed,
        'error':error,'cleanup':cleanup,'command':command,'started_at_utc':started,
        'finished_at_utc':datetime.now(timezone.utc).isoformat(),
        'log':str(output/(name+'.log')),'artifacts':str(output/name)}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=default_root())
    parser.add_argument('--output',type=Path)
    parser.add_argument('--study',choices=['all','shared_scale','correlated_flow','two_view','joint_window','joint_core','serial_core'],default='all')
    parser.add_argument('--timeout',type=positive_timeout,default=900)
    args=parser.parse_args(argv)
    root=args.root.resolve()
    output=(args.output or root/'server_runtime/offline'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')).resolve()
    output.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy()
    env.update(PYTHONPATH=str(root/'src/xq_autonomy'),PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2')
    studies={
        'shared_scale':['scripts/run_shared_scale_study.py','--config','config/research_shared_scale_study.json'],
        'correlated_flow':['scripts/run_correlated_flow_study.py','--protocol','config/research_flow_precondition_gate.json'],
        'two_view':['scripts/run_two_view_geometry_study.py','--protocol','config/research_joint_window_gate.json'],
        'joint_window':['scripts/run_joint_window_study.py','--protocol','config/research_joint_window_gate.json'],
        'joint_core':['scripts/run_joint_flow_core_validation.py','--workspace-root',str(root)],
        'serial_core':['prototypes/serial_position/validate_serial_prototype.py','--joint-header',str(root/'src/xq_fast_livo2/include/research_joint_flow.h')],
    }
    planned=[]
    for name,command in studies.items():
        if args.study not in ('all',name):
            continue
        target=output/name
        output_arg='--output-dir' if name in ('joint_core','serial_core') else '--output'
        command=[sys.executable,*command,output_arg,str(target)]
        planned.append((name,command))
    reports=[]
    abort_reason=None
    interrupted_between_studies=False
    current=None
    try:
        for name,command in planned:
            current=(name,command)
            row=run_study(name,command,root,output,env,args.timeout)
            reports.append(row)
            print(json.dumps(row),flush=True)
            if row['interrupted'] or row['launch_failed']:
                abort_reason=row['error']
                break
    except KeyboardInterrupt:
        interrupted_between_studies=True
        abort_reason='Interrupted by Ctrl+C between study results'
        if current is not None and current[0] not in {row['study'] for row in reports}:
            reports.append({'study':current[0],'status':'INTERRUPTED','returncode':None,
                'command':current[1],'error':abort_reason})
    recorded={row['study'] for row in reports}
    for name,command in planned:
        if name not in recorded:
            reports.append({'study':name,'status':'NOT_RUN','returncode':None,
                'command':command,'reason':abort_reason,
                'started_at_utc':None,'finished_at_utc':None})
    interrupted=interrupted_between_studies or any(row['status']=='INTERRUPTED' for row in reports)
    status='INTERRUPTED' if interrupted else ('PASS' if all(r['status']=='PASS' for r in reports) else 'FAIL')
    summary={'status':status,'root':str(root),'studies':reports,'scope':'Standalone synthetic mechanisms, two-view geometry and C++ core checks. Not the recorded five-mode LIVO experiment, AP flight, calibrated integrity or hardware validation.'}
    previous=signal.signal(signal.SIGINT,signal.SIG_IGN)
    try:
        (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    finally:
        signal.signal(signal.SIGINT,previous)
    return 130 if interrupted else (0 if status=='PASS' else 1)


if __name__=='__main__':
    raise SystemExit(main())
