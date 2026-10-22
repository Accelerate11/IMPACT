#!/usr/bin/env python3
"""Known multi-window linear joint model, with a primitive-latent batch oracle."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from xq_autonomy.joint_state import (
    create_joint_state, clone_pose, propagate_active, linear_update, drop_clone,
)


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def digest(path):
    value=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            value.update(block)
    return value.hexdigest()


def wilson(successes,total,z=1.959963984540054):
    p=successes/total; den=1+z*z/total
    centre=(p+z*z/(2*total))/den
    radius=z*math.sqrt(p*(1-p)/total+z*z/(4*total*total))/den
    return [centre-radius,centre+radius]


def initial(cfg):
    mean=np.zeros(20); mean[6]=1.; mean[7:9]=cfg['nominal_velocity_xy_mps']
    mean[10:13]=[.02,-.01,.005]; mean[13:16]=[.01,-.005,0.]; mean[18]=-9.81
    factor=np.diag(cfg['initial_sigmas'])
    factor[7,19]=cfg['initial_shared_latent_loadings']['velocity_x']*cfg['initial_sigmas'][7]
    factor[1,19]=cfg['initial_shared_latent_loadings']['rotation_y']*cfg['initial_sigmas'][1]
    return mean,factor


def transition(dt,cfg):
    f=np.eye(19)
    f[3:6,7:10]=np.eye(3)*dt
    f[3:6,13:16]=-np.eye(3)*dt*dt*.5
    f[7:10,13:16]=-np.eye(3)*dt
    f[0:3,10:13]=-np.eye(3)*dt
    return f,np.diag(np.asarray(cfg['process_sigmas_per_window'])**2),np.zeros(19)


def observation_matrices(dt,cfg):
    pose=np.zeros((2,26));pose[0,3]=1.;pose[1,4]=1.
    scale=np.zeros((1,26));scale[0,19]=1.
    flow=np.zeros((2,26)); h=cfg['nominal_depth_m']
    flow[0,3]=1/h;flow[0,23]=-1/h
    flow[1,4]=1/h;flow[1,24]=-1/h
    flow[0,1]=-1.;flow[0,21]=1.
    flow[1,0]=1.;flow[1,20]=-1.
    flow[:,19]=-dt*np.asarray(cfg['nominal_velocity_xy_mps'])/h**2
    return (('pose_like_xy',pose,cfg['pose_like_noise_sigma_m']),
            ('range_like_scalar',scale,cfg['range_like_noise_sigma_m']),
            ('two_pose_flow_like_xy',flow,cfg['two_pose_flow_like_noise_sigma']))


def remake(state,mean,cov):
    # Only called at boundaries with no retained clone, to make the explicitly
    # wrong repeated-scale initialization ablation; full_joint never uses it.
    assert state.dimension==20
    return create_joint_state(mean[:19],cov[:19,:19],mean[19:20],cov[19:20,19:20],
        cov[:19,19:20],timestamp_ns=state.timestamp_ns)


def independent_clone_ablation(state,mean,cov,clone_id):
    # The declared wrong model: retain each marginal but replace every
    # active/nuisance-to-clone cross block by zero. It remains PSD, yet it is
    # not the joint law of a deterministic copy. Never used by full_joint.
    base=create_joint_state(mean[:19],cov[:19,:19],mean[19:20],cov[19:20,19:20],
        cov[:19,19:20],timestamp_ns=state.clone_timestamp_ns(clone_id))
    copied=clone_pose(base,clone_id)
    # No public kernel operation may silently break a clone's correlation.
    # This invalid-model control is implemented outside that API via its
    # existing validated constructor, retaining the same layout metadata.
    from dataclasses import replace
    from types import MappingProxyType
    scales=np.sqrt(np.diag(cov))
    if np.min(np.linalg.eigvalsh(cov/scales[:,None]/scales[None,:])) < -1e-12:
        raise ValueError('The deliberately wrong block-diagonal model must still be PSD')
    immutable=lambda x:np.frombuffer(np.asarray(x,dtype=np.float64).tobytes(),dtype=np.float64).reshape(x.shape)
    return replace(copied,_mean=immutable(mean),_covariance=immutable(cov),
        _diagnostics=MappingProxyType({'operation':'deliberately_wrong_zero_clone_cross_control',
            'jitter_added':False,'eigenvalues_clipped':False,'invalid_deterministic_clone_model':True}))


def batch_moments(mu,mapping,observations,values):
    if not observations:
        return mu,mapping@mapping.T
    rows=np.vstack([x[0] for x in observations]);centres=np.concatenate([x[1] for x in observations])
    residual=np.concatenate(values)-centres
    innovation=rows@rows.T
    gain=np.linalg.solve(innovation,rows@mapping.T).T
    return mu+gain@residual,mapping@mapping.T-gain@rows@mapping.T


def run(protocol,output):
    if output.exists():raise ValueError('Output directory must be new')
    output.mkdir(parents=True)
    cfg=protocol['joint_study'];limits=cfg['checks'];methods=cfg['methods']
    if (cfg['active_dimension'],cfg['nuisance_dimension'],len(cfg['windows_ns']),cfg['primitive_dimension'])!=(19,1,4,116):
        raise ValueError('Unsupported frozen joint model')
    if cfg['coverage_direction_velocity_xy'] != [1.0,0.0]:
        raise ValueError('This fixture reports fixed velocity-x marginal coverage only')
    mu0,factor=initial(cfg);p0=factor@factor.T;dimension=cfg['primitive_dimension']
    arrays={}; all_errors={m:[] for m in methods}; final_errors={m:[] for m in methods}
    predicted={m:[] for m in methods};checks={};max_mean=max_cov=max_sample=0.
    batch_checks=0;records=[];operation_counts={m:{} for m in methods}
    for seed_index,seed in enumerate(cfg['seeds']):
        count=cfg['draws_per_seed']
        rng=np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed,0])))
        latent=rng.standard_normal((count,dimension))
        mapping=np.zeros((20,dimension));mapping[:,:20]=factor;mu=mu0.copy()
        states={m:create_joint_state(mu0[:19],p0[:19,:19],mu0[19:],p0[19:,19:],p0[:19,19:],
                    timestamp_ns=cfg['initial_timestamp_ns']) for m in methods}
        estimates={m:np.tile(mu0,(count,1)) for m in methods}
        errors={m:[] for m in methods};sample_observations=[];history=[];history_values=[]
        cursor=20;stamp=cfg['initial_timestamp_ns']

        def record(method,operation,before,after,extra=None):
            nonlocal max_mean,max_cov,max_sample,batch_checks
            operation_counts[method][operation]=operation_counts[method].get(operation,0)+1
            max_sample=max(max_sample,float(np.max(np.abs(after.mean-estimates[method][0]))))
            if method=='full_joint':
                expected_mean,expected_cov=batch_moments(mu,mapping,history,history_values)
                max_mean=max(max_mean,float(np.max(np.abs(after.mean-expected_mean))))
                max_cov=max(max_cov,float(np.max(np.abs(after.covariance-expected_cov))))
                batch_checks+=1
            if seed_index==0:
                records.append(dict(method=method,operation=operation,window=window,seed=seed,
                    before=before.to_dict(),after=after.to_dict(),parameters=extra or {}))

        for window,dt_ns in enumerate(cfg['windows_ns']):
            dt=dt_ns*1e-9;clone_id='start_'+str(window)
            if window:
                name='reset_shared_nuisance';before=states[name];mean=before.mean.copy();cov=before.covariance.copy()
                mean[19]=mu0[19];cov[19,:]=0.;cov[:,19]=0.;cov[19,19]=p0[19,19]
                states[name]=remake(before,mean,cov);estimates[name][:,19]=mu0[19]
                record(name,'reset_shared_nuisance',before,states[name])
            mu=np.r_[mu,mu[:6]];mapping=np.vstack((mapping,mapping[:6,:]))
            for name in methods:
                before=states[name];states[name]=clone_pose(before,clone_id)
                estimates[name]=np.column_stack((estimates[name],estimates[name][:,:6]))
                record(name,'clone',before,states[name],{'clone_id':clone_id})
            name='zero_clone_cross';before=states[name];cov=before.covariance.copy()
            cov[:20,20:]=0.;cov[20:,:20]=0.
            states[name]=independent_clone_ablation(before,before.mean.copy(),cov,clone_id)
            record(name,'zero_clone_cross',before,states[name])
            f,q,offset=transition(dt,cfg);transform=np.eye(26);transform[:19,:19]=f
            mu=transform@mu;mu[:19]+=offset;mapping=transform@mapping
            mapping[:19,cursor:cursor+19]=np.diag(cfg['process_sigmas_per_window']);cursor+=19
            stamp+=dt_ns
            for name in methods:
                before=states[name]
                states[name]=propagate_active(before,f,q,offset,next_timestamp_ns=stamp,independent_process_noise=True)
                estimates[name][:,:19]=estimates[name][:,:19]@f.T+offset
                record(name,'propagate',before,states[name],{'F':f.tolist(),'Q':q.tolist(),'offset':offset.tolist(),'next_timestamp_ns':stamp})
            measurements=[]
            for label,h,sigma in observation_matrices(dt,cfg):
                n=h.shape[0];noise=np.zeros((n,dimension));noise[:,cursor:cursor+n]=np.eye(n)*sigma;cursor+=n
                model=h@mapping+noise;centre=h@mu
                z=latent@model.T+centre
                history.append((model,centre));history_values.append(z[0]);measurements.append(z)
                for name in methods:
                    before=states[name]
                    update=linear_update(before,z[0],h,np.eye(n)*sigma*sigma,
                        measurement_timestamp_ns=stamp,independent_measurement_noise=True)
                    states[name]=update.state
                    gain=np.asarray(update.gain)
                    estimates[name]+=(z-estimates[name]@h.T)@gain.T
                    record(name,'update_'+label,before,states[name],{'H':h.tolist(),'R':(np.eye(n)*sigma*sigma).tolist(),
                        'measurement':z[0].tolist(),'measurement_timestamp_ns':stamp,'gain':gain.tolist()})
            sample_observations.append(np.column_stack(measurements))
            truth=latent@mapping.T+mu
            for name in methods:
                errors[name].append(truth[:,7:9]-estimates[name][:,7:9])
                if seed_index==0:predicted[name].append(states[name].covariance[np.ix_([7,8],[7,8])].tolist())
            mu=mu[:20];mapping=mapping[:20,:]
            for name in methods:
                before=states[name];states[name]=drop_clone(before,clone_id);estimates[name]=estimates[name][:,:20]
                record(name,'drop_clone',before,states[name],{'clone_id':clone_id})
        if cursor!=dimension:raise AssertionError('Primitive graph is incomplete')
        key='seed'+str(seed)
        arrays[key+'_primitive_standard_normal']=latent
        arrays[key+'_observations']=np.stack(sample_observations,axis=1)
        truth=latent@mapping.T+mu
        for name in methods:
            value=np.stack(errors[name],axis=1);final=truth-estimates[name]
            arrays[key+'_'+name+'_velocity_error_xy']=value
            arrays[key+'_'+name+'_final_joint_error']=final
            all_errors[name].append(value);final_errors[name].append(final)
    cases=[];total=cfg['draws_per_seed']*len(cfg['seeds'])
    for window in range(len(cfg['windows_ns'])):
        row={'window':window,'independent_trajectories':total,'methods':{}}
        for name in methods:
            error=np.concatenate(all_errors[name])[:,window,:];p=np.asarray(predicted[name][window])
            empirical=np.cov(error,rowvar=False,bias=True)
            threshold=limits['normal_quantile']*math.sqrt(p[0,0]);covered=int(np.count_nonzero(np.abs(error[:,0])<=threshold))
            relative=float(np.linalg.norm(empirical-p)/np.linalg.norm(p))
            row['methods'][name]=dict(coverage=covered/total,covered_trials=covered,wilson_95=wilson(covered,total),
                predicted_covariance_xy=p.tolist(),empirical_covariance_xy=empirical.tolist(),
                relative_covariance_error=relative,mean_error_xy=error.mean(axis=0).tolist())
        correct=row['methods']['full_joint'];interval=limits['correct_directional_coverage_interval']
        checks['window'+str(window)+'.full_joint_coverage']=interval[0]<=correct['coverage']<=interval[1]
        checks['window'+str(window)+'.full_joint_covariance']=correct['relative_covariance_error']<=limits['maximum_correct_velocity_covariance_relative_error']
        cases.append(row)
    checks.update(batch_all_operations_mean=max_mean<=limits['maximum_batch_mean_absolute_error'],
        batch_all_operations_covariance=max_cov<=limits['maximum_batch_covariance_absolute_error'],
        sampled_update_matches_kernel_mean=max_sample<=limits['maximum_batch_mean_absolute_error'],
        first_window_scale_reset_control=np.max(np.abs(np.asarray(predicted['full_joint'][0])-np.asarray(predicted['reset_shared_nuisance'][0])))<=limits['maximum_first_window_reset_control_difference'])
    np.savez_compressed(output/'samples.npz',**arrays)
    index={'path':str((output/'samples.npz').resolve()),'bytes':(output/'samples.npz').stat().st_size,
        'sha256':digest(output/'samples.npz'),'arrays':{k:{'shape':list(v.shape),'dtype':str(v.dtype)} for k,v in arrays.items()}}
    save(output/'raw-array-index.json',index)
    with np.load(output/'samples.npz',allow_pickle=False) as saved:
        checks['all_arrays_read_back']=set(saved.files)==set(arrays) and all(np.array_equal(saved[k],v) for k,v in arrays.items())
    checks['sample_archive_unchanged']=index['sha256']==digest(output/'samples.npz')
    with (output/'joint-steps.jsonl').open('w') as stream:
        for row in records:stream.write(json.dumps(row,allow_nan=False)+'\n')
    result={'schema_version':1,'status':'PASS' if all(checks.values()) else 'FAIL','checks':{k:bool(v) for k,v in checks.items()},
        'independent_trajectories':total,'correlated_windows':total*len(cfg['windows_ns']),
        'raw_arrays':index,'cases':cases,'operation_counts':operation_counts,'batch_operation_checks':batch_checks,
        'maximum_batch_mean_error':max_mean,'maximum_batch_covariance_error':max_cov,'maximum_sample_kernel_mean_error':max_sample,
        'scope':cfg['scope'],'coverage_direction_velocity_xy':cfg['coverage_direction_velocity_xy'],
        'coverage_scope':cfg['coverage_scope'],'actual_recorded_flow_state_updates':0,'full_common_state_flow_fusion_validated':False,
        'joint_state_software_updates_performed':True,'hardware_readiness':'NOT_READY'}
    save(output/'study-result.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--protocol',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    result=run(json.loads(args.protocol.read_text()),args.output)
    print(json.dumps({k:result[k] for k in ('status','independent_trajectories','correlated_windows','batch_operation_checks',
        'maximum_batch_mean_error','maximum_batch_covariance_error','maximum_sample_kernel_mean_error','cases')}))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
