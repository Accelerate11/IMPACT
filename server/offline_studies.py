#!/usr/bin/env python3
"""Run the retained mathematical studies from a relocatable source bundle."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


def default_root():
    script=Path(__file__).resolve()
    return script.parents[2] if script.parent.parent.name=='packaging' else script.parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=default_root())
    parser.add_argument('--output',type=Path)
    parser.add_argument('--study',choices=['all','shared_scale','correlated_flow','two_view','joint_window','joint_core','serial_core'],default='all')
    parser.add_argument('--timeout',type=int,default=900)
    args=parser.parse_args()
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
    reports=[]
    for name,command in studies.items():
        if args.study not in ('all',name):
            continue
        target=output/name
        output_arg='--output-dir' if name in ('joint_core','serial_core') else '--output'
        command=[sys.executable,*command,output_arg,str(target)]
        started=datetime.now(timezone.utc).isoformat()
        timed_out=False
        with (output/(name+'.log')).open('w') as log:
            process=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                code=process.wait(timeout=args.timeout)
            except subprocess.TimeoutExpired:
                timed_out=True
                os.killpg(process.pid,signal.SIGTERM)
                try:
                    code=process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    code=process.wait()
        row={'study':name,'status':'PASS' if code==0 and not timed_out else 'FAIL','returncode':code,'timed_out':timed_out,'command':command,'started_at_utc':started,'finished_at_utc':datetime.now(timezone.utc).isoformat(),'log':str(output/(name+'.log')),'artifacts':str(target)}
        reports.append(row)
        print(json.dumps(row),flush=True)
    summary={'status':'PASS' if all(r['status']=='PASS' for r in reports) else 'FAIL','root':str(root),'studies':reports,'scope':'Standalone synthetic mechanisms, two-view geometry and C++ core checks. Not the recorded five-mode LIVO experiment, AP flight, calibrated integrity or hardware validation.'}
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    return 0 if summary['status']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
