#!/usr/bin/env python3
"""Independent reconstruction of actual raw-pixel joint updates and trajectories.

Truth is read here only, after estimator processes have exited. This evaluator
has no ROS imports or publishers. A single input cannot establish superiority.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
from analyze_livo_replay import align_and_measure


def load(path):
    with path.open() as handle:
        for line in handle:
            yield json.loads(line)


def file_record(path):
    h=hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda:handle.read(1048576),b''):h.update(block)
    return {'bytes':path.stat().st_size,'sha256':h.hexdigest()}


def arr(value,shape=None):
    x=np.asarray(value,dtype=float)
    if shape is not None:x=x.reshape(shape)
    if not np.isfinite(x).all():raise ValueError('Nonfinite matrix')
    return x


def skew(v):
    x,y,z=v;return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def exp(v):
    a=np.linalg.norm(v);s=skew(v)
    return np.eye(3)+s+.5*s@s if a<1e-8 else np.eye(3)+np.sin(a)/a*s+(1-np.cos(a))/a**2*s@s


def reset(v):
    a=np.linalg.norm(v);s=skew(v)
    return np.eye(3)-.5*s+s@s/6 if a<1e-6 else np.eye(3)-(1-np.cos(a))/a**2*s+(a-np.sin(a))/a**3*s@s


def psd(p):
    x=arr(p)
    if x.ndim!=2 or x.shape[0]!=x.shape[1]:raise ValueError('Nonsquare covariance')
    scale=max(1.,float(np.linalg.norm(x)))
    if np.linalg.norm(x-x.T)>1e-8*scale:raise ValueError('Asymmetric covariance')
    minimum=float(np.linalg.eigvalsh((x+x.T)/2)[0])
    if minimum < -1e-9*scale:raise ValueError('Indefinite covariance')
    return minimum


def close(actual,expected,tolerance,name):
    a,b=arr(actual),arr(expected)
    error=float(np.linalg.norm(a-b))
    if error>tolerance*max(1.,float(np.linalg.norm(b))):
        raise ValueError(name+' mismatch: '+str(error))
    return error


def project(prior,pixel,protocol,delta=None):
    d=np.zeros(28) if delta is None else delta
    r=arr(prior['state']['R'],(3,3))@exp(d[:3]);p=arr(prior['state']['p'],(3,))+d[3:6]
    r0=arr(prior['clone_R'],(3,3))@exp(d[19:22]);p0=arr(prior['clone_p'],(3,))+d[22:25]
    a,b,h=arr(prior['plane'],(3,))+d[25:28]
    fx,fy,cx,cy=protocol['camera_K'];rc=arr(protocol['body_from_optical'],(3,3));t=arr(protocol['optical_origin_body_m'],(3,))
    o=p0+r0@t;direction=r0@rc@np.array([(pixel[0]-cx)/fx,(pixel[1]-cy)/fy,1.])
    n=np.array([-a,-b,1.]);depth=(h-n@o)/(n@direction)
    point=o+depth*direction;c=rc.T@(r.T@(point-p)-t)
    return np.array([fx*c[0]/c[2]+cx,fy*c[1]/c[2]+cy])


def audit_applied(row,prior,raw,protocol):
    p=arr(row['joint_covariance_before'],(28,28));h=arr(row['H']);r=arr(row['R']);s=arr(row['S']);k=arr(row['K'])
    residual=arr(row['residual'],(-1,));dx=arr(row['delta28'],(28,))
    close(p,prior['P'],1e-12,'flow prior')
    for field in ('R','p','v','bg','ba','g','cov19','inv_exposure'):
        close(row['state_before'][field],prior['state'][field],1e-12,'native mean/covariance prior '+field)
    if row.get('same_native_object') is not True or not row.get('native_state_address'):
        raise ValueError('Actual shared native object not identified')
    close(s,h@p@h.T+r,1e-10,'innovation covariance')
    close(k,np.linalg.solve(s,h@p).T,1e-8,'gain')
    close(dx,k@residual,1e-10,'state injection')
    nis=float(residual@np.linalg.solve(s,residual))
    close(row['nis'],nis,1e-8,'NIS')
    ids=row['accepted_track_ids'];tracks={entry['id']:entry for entry in raw['tracks']}
    if len(ids)!=len(set(ids)) or len(ids)*2!=len(residual) or not set(ids)<=set(tracks):
        raise ValueError('Accepted IDs not bound to the raw two-view batch')
    fd=np.zeros(h.shape);predicted=[];expected_noise=np.zeros(r.shape);eps=1e-6
    for i,identifier in enumerate(ids):
        track=tracks[identifier];u0=arr(track['u0'],(2,));u1=arr(track['u1'],(2,));pred=project(prior,u0,protocol)
        predicted.extend(u1-pred)
        for j in range(28):
            d=np.zeros(28);d[j]=eps
            fd[2*i:2*i+2,j]=(project(prior,u0,protocol,d)-project(prior,u0,protocol,-d))/(2*eps)
        jp=np.zeros((2,2));peps=1e-4
        for j in range(2):
            d=np.zeros(2);d[j]=peps
            jp[:,j]=(project(prior,u0+d,protocol)-project(prior,u0-d,protocol))/(2*peps)
        variance=protocol['pixel_sigma_px']**2
        expected_noise[2*i:2*i+2,2*i:2*i+2]=variance*(jp@jp.T+np.eye(2))
    jacobian_error=close(h,fd,2e-6,'28D raw reprojection Jacobian')
    close(residual,predicted,1e-8,'raw observed pixel residual')
    close(r,expected_noise,1e-7,'two-endpoint pixel covariance')
    l=np.eye(28)-k@h;j=np.eye(28);j[:3,:3]=reset(dx[:3]);j[19:22,19:22]=reset(dx[19:22])
    after=j@(l@p@l.T+k@r@k.T)@j.T
    close(row['joint_covariance_after'],after,1e-9,'Joseph covariance with both SO3 resets')
    pg=arr(prior['Pg'],(28,28));close(row['gyro_component_after'],j@l@pg@l.T@j.T,1e-9,'gyro component')
    before,actual=row['state_before'],row['state_after']
    close(actual['R'],arr(before['R'],(3,3))@exp(dx[:3]),1e-10,'native SO3 injection')
    for field,start in [('p',3),('v',7),('bg',10),('ba',13),('g',16)]:
        close(arr(actual[field],(3,)),arr(before[field],(3,))+dx[start:start+3],1e-10,'native '+field)
    close(actual['inv_exposure'],before['inv_exposure']+dx[6],1e-10,'exposure injection')
    close(actual['cov19'],after[:19,:19],1e-9,'actual native covariance block')
    psd(after);psd(row['gyro_component_after']);psd(after-arr(row['gyro_component_after']))
    return {'jacobian_absolute_frobenius_error':jacobian_error,'nis_per_dof':nis/len(residual),
        'native_state_delta_norm':float(np.linalg.norm(dx[:19])),
        'native_covariance_delta_norm':float(np.linalg.norm(after[:19,:19]-arr(before['cov19'])))}


def arm(directory,index,truth):
    status=json.loads((directory/'run-result.json').read_text())
    if status['status']!='PASS':raise ValueError('Arm runner did not PASS: '+directory.name)
    protocol=summary=None;prior={};raw={};counts=Counter();corrections=[];flow_reasons=Counter();seed_stamps=[];initial_gyro=0
    native_addresses=set();last_pg=None;expected_ablation_pg={};ablation_count=0
    for row in load(directory/'joint-updates.jsonl'):
        event=row['event'];counts[event]+=1
        if row.get('native_state_address'):native_addresses.add(row['native_state_address'])
        if event in ('native_LIO','native_VIO','range_seed','flow') and 'gyro_component_after' in row:
            last_pg=arr(row['gyro_component_after'],(28,28))
        if event=='protocol':protocol=row
        elif event=='initial_gyro_component':
            pg=arr(row['gyro_cov19']);psd(pg);psd(arr(row['cov19'])-pg);initial_gyro+=int(np.trace(pg)>0)
        elif event=='raw_tracks':raw[row['second_stamp_ns']]=row
        elif event=='flow_prior':
            prior[row['stamp_ns']]=row
            if row['stamp_ns'] in expected_ablation_pg:
                close(row['Pg'],expected_ablation_pg[row['stamp_ns']],1e-10,'retained ablation gyro tag')
        elif event=='cross_ablation':
            if last_pg is None:raise ValueError('Ablation has no propagated native gyro component')
            before=arr(row['covariance_before'],(28,28));after=arr(row['covariance_after'],(28,28))
            pg=last_pg.copy()
            if row['mode']=='zero_gyro':
                pg[:]=0
                for start,end in ((0,19),(19,25),(25,28)):pg[start:end,start:end]=last_pg[start:end,start:end]
                expected=before-last_pg+pg
            elif row['mode']=='zero_plane':
                expected=before.copy();expected[:25,25:]=0;expected[25:,:25]=0
                pg[:25,25:]=0;pg[25:,:25]=0
            else:raise ValueError('Unknown or mislabeled cross ablation')
            close(after,expected,1e-10,'specified cross ablation mask')
            close(np.diag(after),np.diag(before),1e-12,'ablation marginal variances')
            psd(after);psd(pg);psd(after-pg);ablation_count+=1;expected_ablation_pg[row['stamp_ns']]=pg
        elif event=='range_seed' and row['accepted']:
            seed_stamps.append(row['stamp_ns']);psd(row['joint_covariance_after'])
            psd(arr(row['joint_covariance_after'])-arr(row['gyro_component_after']))
        elif event=='flow':
            flow_reasons[row['reason']]+=1
            if row['accepted']:
                stamp=row['stamp_ns']
                if stamp not in prior or stamp not in raw:raise ValueError('Missing accepted batch input/prior')
                corrections.append(audit_applied(row,prior[stamp],raw[stamp],protocol))
        elif event=='summary':summary=row
    if protocol is None or summary is None:raise ValueError('No protocol or clean shutdown summary')
    expected=index['published_sensor_counts']
    if counts['input_down']!=expected['down'] or counts['input_range']!=expected['range']:
        raise ValueError('Down/range receipt ledger incomplete')
    source_stamps={role:{row['stamp_ns'] for row in index['entries'] if row['sensor']==role} for role in ('front','down','range')}
    if any(stamp not in source_stamps['front']&source_stamps['down']&source_stamps['range'] for stamp in seed_stamps):
        raise ValueError('Range seed is not exact simultaneous original acquisition')
    pairs=[(entry['first_stamp_ns'],entry['second_stamp_ns']) for entry in raw.values()]
    endpoint_stamps=[stamp for pair in pairs for stamp in pair]
    if len(endpoint_stamps)!=len(set(endpoint_stamps)):raise ValueError('Reused image in supposedly nonoverlapping flow pairs')
    original_camera=sorted(source_stamps['front']);allowed=set()
    for t0,t1 in zip(original_camera,original_camera[1:]):
        if t0 in source_stamps['range'] and 0<t1-t0<=300_000_000:allowed.add((t0,t1))
    if not set(pairs)<=allowed:raise ValueError('Raw pair is not an exact seed and adjacent original camera epoch')
    if len(native_addresses)>1:raise ValueError('Different native state objects across updates')
    nonzero=sum(value['native_state_delta_norm']>1e-12 and value['native_covariance_delta_norm']>1e-12 for value in corrections)
    if nonzero!=summary['nonzero_flow_state_corrections']:raise ValueError('State correction summary inconsistent')
    if directory.name in ('full','zero_gyro','zero_plane') and not nonzero:
        raise ValueError('No actual nonzero flow state and covariance correction')
    if directory.name!='native' and initial_gyro!=1:raise ValueError('Missing unique nonzero initial gyro tag')
    odometry=list(load(directory/'odometry.jsonl'))
    metrics,trajectory=align_and_measure(odometry,truth,tolerance_s=.015,
        body_transform={'translation_m':[0,0,0],'rotation_xyzw':[0,0,0,1]})
    return {'status':'PASS','events':dict(counts),'flow_reasons':dict(flow_reasons),
        'attempted_frame_pairs':summary['completed_pairs'],'pending_right_censored_pair':summary['pair_pending_at_shutdown'],
        'accepted_flow_updates':len(corrections),'actual_nonzero_state_and_covariance_updates':nonzero,
        'specified_cross_ablation_batches':ablation_count,'unique_native_state_addresses':len(native_addresses),
        'range_seed_augmentations':len(seed_stamps),'trajectory_metrics':metrics,
        'nis_per_dof':[value['nis_per_dof'] for value in corrections],
        'maximum_jacobian_frobenius_error':max((value['jacobian_absolute_frobenius_error'] for value in corrections),default=0.),
        'covariance_scope':'conditional engineering model only, not globally calibrated covariance or integrity bound'},trajectory


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--truth',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('Evaluation output must be new')
    args.output.mkdir(parents=True);index=json.loads((args.run/'input/sensor-index.json').read_text())
    truth=list(load(args.truth));suite=json.loads((args.run/'suite-result.json').read_text())
    result={'status':'PASS','arms':{},'failures':{},'scope':'single recorded-flight integration and numerical reconstruction, not multiseed superiority or AP4.7.1 flight',
        'truth_scope':'offline evaluator only; first-pair SE3 gauge, no scale fitting, nearest original truth within 15ms',
        'inputs':{'truth':file_record(args.truth),'sensor_index':file_record(args.run/'input/sensor-index.json'),
                  'evaluator':file_record(Path(__file__))}}
    for entry in suite['arms']:
        mode=entry['mode']
        try:
            report,trajectory=arm(args.run/mode,index,truth);result['arms'][mode]=report
            (args.output/(mode+'-trajectory.json')).write_text(json.dumps(trajectory,allow_nan=False)+'\n')
        except Exception as exc:result['status']='FAIL';result['failures'][mode]=type(exc).__name__+': '+str(exc)
    if len(result['arms'])!=len(suite['arms']) or suite['status']!='PASS':result['status']='FAIL'
    (args.output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,allow_nan=False));return 0 if result['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
