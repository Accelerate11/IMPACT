#!/usr/bin/env python3
"""Plot a closed unarmed AP471 diagnostic with raw-file provenance."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path


def rows(path):return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args();run=args.run_dir.resolve(strict=True)
    if not (run/'runner-result.json').is_file():raise ValueError('A closed original trial is required')
    report=json.loads((run/'link-probe-result.json').read_text())
    output=args.output_dir or Path('/mnt/d/CODEX/impact/runs')/('research_ap471_visuals_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    output.mkdir(exist_ok=False)
    os.environ['MPLCONFIGDIR']=str(output/'font-cache')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    selected=[run/'adapter-clock-ledger.jsonl',run/'readiness/events.jsonl',run/'ros-observer/events.jsonl',
        run/'link-probe-result.json',run/'runner-result.json',run/'shutdown-result.json']
    before={str(path):sha(path) for path in selected}
    ledger=rows(selected[0]);events=rows(selected[1]);source=rows(selected[2])
    first=ledger[0]['sampling'];anchor_mono=first['monotonic_midpoint_ns'];anchor_wall=first['raw_wall_ns'];anchor_mapped=first['mapped_ns']
    t=np.asarray([(row['sampling']['monotonic_midpoint_ns']-anchor_mono)/1e9 for row in ledger])
    raw=np.asarray([((row['sampling']['raw_wall_ns']-anchor_wall)-(row['sampling']['monotonic_midpoint_ns']-anchor_mono))/1e6 for row in ledger])
    mapped=np.asarray([((row['sampling']['mapped_ns']-anchor_mapped)-(row['sampling']['monotonic_midpoint_ns']-anchor_mono))/1e6 for row in ledger])
    readiness=[row for row in events if row['kind']=='readiness']
    rt=np.asarray([(row['monotonic_ns']-anchor_mono)/1e9 for row in readiness])
    ready=np.asarray([row['data']['ready'] for row in readiness],dtype=float)
    health=report['FCU_health']['samples']
    ht=np.asarray([(row['monotonic_ns']-anchor_mono)/1e9 for row in health]);hv=np.asarray([row['combined_backend_healthy'] for row in health],dtype=float)
    pause={row['kind']:(row['monotonic_ns']-anchor_mono)/1e9 for row in source if row['kind'] in {'source_pause_started','source_pause_ended'}}
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    figure,axes=plt.subplots(3,1,figsize=(12,8.5),gridspec_kw={'height_ratios':[1.4,1.05,1]})
    axes[0].plot(t,raw,color='#c86531',lw=1.5,label='Raw wall minus monotonic elapsed (observed)')
    axes[0].plot(t,mapped,color='#267f80',lw=1.6,label='Mapped minus monotonic elapsed (by construction)')
    axes[0].set_ylabel('Anchor residual (ms)');axes[0].legend(loc='best',frameon=False)
    axes[0].set_title('Pinned ArduCopter 4.7.1: unarmed transport and receipt diagnostic',loc='left',weight='bold')
    axes[1].step(ht,hv*.65+.1,where='post',color='#627d39',label='FCU sampled backend health')
    axes[1].step(rt,ready*.65+1.1,where='post',color='#2a649c',label='Current local + FCU receipt gate')
    axes[1].set_yticks([.1,.75,1.1,1.75],['0','1','0','1']);axes[1].set_ylabel('Sampled state')
    axes[1].legend(loc='upper right',frameon=False,ncol=2);axes[1].set_ylim(-.02,2.15)
    for ax in axes[:2]:
        ax.set_xlim(0,max(rt));ax.grid(alpha=.18,axis='y');ax.set_xlabel('Host monotonic elapsed (s)')
        if pause:
            ax.axvspan(pause['source_pause_started'],pause['source_pause_ended'],color='#c86531',alpha=.15)
    if pause:axes[1].text((pause['source_pause_started']+pause['source_pause_ended'])/2,2.03,'Source pause',ha='center',fontsize=9,color='#9b4d25')
    counts=report['counts'];values=[counts[name] for name in ('source_inputs','adapter_forwarded','wire_ODOMETRY','FCU_VISP','FCU_VISV')]
    bars=axes[2].bar(['Source inputs','Adapter forwarded','Wire ODOMETRY','FCU VISP','FCU VISV'],values,color=['#a5a9af']+['#267f80']*4,width=.6)
    axes[2].bar_label(bars,padding=3);axes[2].set_ylabel('Observed frames');axes[2].set_ylim(0,max(values)*1.2);axes[2].grid(alpha=.18,axis='y');axes[2].set_axisbelow(True)
    figure.text(.06,.015,f"Single {report['case']['source']} trial; {counts['startup_skipped']} startup skips. Matching keys are observed transport/backend evidence, not a loss-rate, EKF fusion, UTC calibration, or flight claim.",fontsize=8.2,color='#4b5563')
    figure.tight_layout(rect=(0,.04,1,1))
    for suffix in ('png','pdf','svg'):figure.savefig(output/('ap471-transport-and-readiness.'+suffix),dpi=190)
    np.savez_compressed(output/'plot-data.npz',host_elapsed_s=t,raw_anchor_residual_ms=raw,mapped_anchor_residual_ms=mapped,
        readiness_elapsed_s=rt,current_ready=ready,FCU_health_elapsed_s=ht,FCU_backend_health=hv,frame_counts=np.asarray(values))
    after={str(path):sha(path) for path in selected}
    summary={'status':'PASS' if before==after else 'FAIL','input_sha256_before':before,'input_sha256_after':after,
        'selected_inputs_unchanged':before==after,'scope':report['scope'],'counts':counts,
        'shutdown_status':json.loads(selected[-1].read_text())['status'],
        'caption':'Mapped elapsed residual is zero by construction; this is a transport engineering diagnostic, not a calibrated sensor/UTC clock error or algorithm accuracy result.'}
    with (output/'report.json').open('x') as handle:json.dump(summary,handle,indent=2);handle.write('\n')
    (output/'plot_ap471_transport.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps({'status':summary['status'],'output':str(output)}))
    return 0 if before==after else 1


if __name__=='__main__':raise SystemExit(main())
