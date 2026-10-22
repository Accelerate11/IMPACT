#!/usr/bin/env python3
"""Independent latent/batch and sequential-Kalman audit of a frozen linear study.

No production state kernel or experiment driver is imported. The four windows
within a trajectory are correlated; only the 36000 generated trajectories are
independent. Reported intervals concern velocity_x, not 2D/trajectory coverage
or a protection level. No recorded flow update or physical estimator is tested.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np


METHODS = ('full_joint', 'reset_shared_nuisance', 'zero_clone_cross')
INPUTS = {'samples.npz', 'raw-array-index.json', 'joint-steps.jsonl', 'study-result.json'}
SEEDS = (14027, 39041, 81013)
WINDOWS = (60000000, 110000000, 80000000, 150000000)
QUANTILE = 1.959963984540054
COVERAGE_SCOPE = ('Marginal velocity_x normal interval at each window, not a 2D joint interval, '
                  'trajectory-wide guarantee, task-derived direction or protection level')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def _pairs(items):
    output = {}
    for key, value in items:
        require(key not in output, 'duplicate JSON key: '+key)
        output[key] = value
    return output


def _float(value):
    number = float(value)
    require(math.isfinite(number), 'nonfinite JSON number')
    return number


def loads(value):
    def bad(number):
        raise ValueError('nonfinite JSON constant: '+number)
    return json.loads(value, parse_float=_float, parse_constant=bad, object_pairs_hook=_pairs)


def read(path):
    return loads(Path(path).read_text())


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1048576), b''):
            value.update(chunk)
    return value.hexdigest()


def file_record(path):
    return {'bytes':Path(path).stat().st_size, 'sha256':digest(path)}


def numeric(value, shape, label):
    def has_bool(item):
        return isinstance(item, (bool, np.bool_)) or isinstance(item, (list, tuple)) and any(has_bool(x) for x in item)
    array = np.asarray(value)
    require(array.dtype.kind in 'fiu' and array.shape == shape and not has_bool(value), label+' numeric shape/type')
    array = array.astype(float)
    require(np.isfinite(array).all(), label+' finite')
    return array


def close(value, expected, label, tolerance=1e-10):
    expected = np.asarray(expected, dtype=float)
    actual = numeric(value, expected.shape, label)
    require(float(np.max(np.abs(actual-expected), initial=0.)) <= tolerance, label+' mismatch')


def frozen_protocol(protocol):
    require(type(protocol['schema_version']) is int and protocol['schema_version'] == 1 and
            protocol['phase'] == 'R2B3B0_JOINT_WINDOW_SOFTWARE' and protocol['input_origin'] ==
            'declared_linear_gaussian_and_rigid_geometry_fixtures_not_recorded_sensor_data', 'frozen protocol identity')
    cfg = protocol['joint_study']
    expected = {
        'seeds':list(SEEDS), 'draws_per_seed':12000, 'windows_ns':list(WINDOWS),
        'initial_timestamp_ns':1700000000000000000, 'active_dimension':19, 'nuisance_dimension':1,
        'clone_active_indices':list(range(6)), 'nominal_depth_m':2., 'nominal_velocity_xy_mps':[.7,.2],
        'initial_sigmas':[.04]*3+[.08]*3+[.02]+[.25]*3+[.015]*3+[.02]*3+[.03]*3+[.3],
        'initial_shared_latent_loadings':{'velocity_x':.35,'rotation_y':-.2},
        'process_sigmas_per_window':[.002]*6+[.0001]+[.02]*3+[.0005]*3+[.001]*3+[.0001]*3,
        'pose_like_noise_sigma_m':.04, 'range_like_noise_sigma_m':.12,
        'two_pose_flow_like_noise_sigma':.004, 'methods':list(METHODS),
        'rng':'numpy PCG64 with SeedSequence([seed, 0])', 'primitive_dimension':116,
        'measurement_order_per_window':['pose_like_xy','range_like_scalar','two_pose_flow_like_xy'],
        'independent_process_noise':True, 'independent_new_measurement_white_noise':True,
        'shared_image_noise_modelled':False, 'coverage_direction_velocity_xy':[1.,0.],
        'coverage_scope':COVERAGE_SCOPE,
        'checks':{'maximum_batch_mean_absolute_error':1e-10, 'maximum_batch_covariance_absolute_error':1e-10,
                  'correct_directional_coverage_interval':[.93,.97],
                  'maximum_correct_velocity_covariance_relative_error':.05,
                  'maximum_first_window_reset_control_difference':1e-12,
                  'normal_quantile':QUANTILE, 'ablation_effect_sign_is_not_an_acceptance_gate':True}}
    require(all(cfg.get(k) == v for k,v in expected.items()), 'frozen joint fixture and acceptance limits')
    for key in ('draws_per_seed','initial_timestamp_ns','active_dimension','nuisance_dimension','primitive_dimension'):
        require(type(cfg[key]) is int, 'integer protocol '+key)
    require(all(type(x) is int for x in cfg['seeds']+cfg['windows_ns']+cfg['clone_active_indices']), 'integer fixture sequences')
    require(cfg['independent_process_noise'] is True and cfg['independent_new_measurement_white_noise'] is True and
            cfg['shared_image_noise_modelled'] is False and cfg['checks']['ablation_effect_sign_is_not_an_acceptance_gate'] is True,
            'explicit independence and nongated ablation sign')
    require(type(protocol['actual_recorded_flow_state_updates']) is int and protocol['actual_recorded_flow_state_updates'] == 0 and
            protocol['full_common_state_flow_fusion_validated'] is False and protocol['hardware_readiness'] == 'NOT_READY' and
            protocol['new_ap_flight_tested'] is False and protocol['statistical_integrity_tested'] is False, 'frozen nonhardware boundary')
    return cfg


def score_interval(successes, total):
    z2 = QUANTILE**2
    centre = (successes+z2/2)/(total+z2)
    radius = math.sqrt(z2*(successes*(1-successes/total)+z2/4))/(total+z2)
    return [centre-radius, centre+radius]


def statistics(errors, covariance):
    count = len(errors)
    mean = errors.sum(axis=0)/count
    centred = errors-mean
    empirical = centred.T @ centred/count
    covered = int(np.count_nonzero(np.abs(errors[:,0]) <= QUANTILE*math.sqrt(covariance[0,0])))
    return {'coverage':covered/count, 'covered_trials':covered, 'wilson_95':score_interval(covered,count),
            'predicted_covariance_xy':covariance.tolist(), 'empirical_covariance_xy':empirical.tolist(),
            'relative_covariance_error':float(np.linalg.norm(empirical-covariance)/np.linalg.norm(covariance)),
            'mean_error_xy':mean.tolist()}


def final_statistics(errors, covariance):
    mean = errors.sum(axis=0)/len(errors)
    centred = errors-mean
    empirical = centred.T @ centred/len(errors)
    coordinate = []
    for index in range(20):
        covered = int(np.count_nonzero(np.abs(errors[:,index]) <= QUANTILE*math.sqrt(covariance[index,index])))
        coordinate.append({'coordinate':index,'covered_trajectories':covered,'coverage':covered/len(errors),
                           'wilson_95':score_interval(covered,len(errors))})
    return {'predicted_covariance_20':covariance.tolist(),'empirical_covariance_20':empirical.tolist(),
            'mean_error_20':mean.tolist(),'coordinate_marginal_95_intervals':coordinate,
            'relative_covariance_frobenius_error':float(np.linalg.norm(empirical-covariance)/np.linalg.norm(covariance)),
            'scope':'Diagnostic marginal intervals, not a 20D simultaneous region or extra acceptance gate'}


def independent_initial(cfg):
    mean = np.zeros(20)
    mean[6],mean[18] = 1.,-9.81
    mean[7:9],mean[10:13],mean[13:16] = cfg['nominal_velocity_xy_mps'],[.02,-.01,.005],[.01,-.005,0.]
    load = np.zeros((20,116))
    load[np.arange(20),np.arange(20)] = cfg['initial_sigmas']
    load[7,19] = .35*cfg['initial_sigmas'][7]
    load[1,19] = -.2*cfg['initial_sigmas'][1]
    return mean,load


def independent_transition(seconds,cfg):
    transform = np.eye(26)
    for axis in range(3):
        transform[3+axis,7+axis] = seconds
        transform[3+axis,13+axis] = -.5*seconds**2
        transform[7+axis,13+axis] = -seconds
        transform[axis,10+axis] = -seconds
    process = np.diag(np.asarray(cfg['process_sigmas_per_window'])**2)
    return transform,process


def independent_observations(seconds,cfg):
    pose = np.zeros((2,26))
    pose[0,3],pose[1,4] = 1.,1.
    scale = np.zeros((1,26));scale[0,19] = 1.
    flow = np.zeros((2,26));depth = cfg['nominal_depth_m']
    flow[0,[3,23,1,21]] = [1/depth,-1/depth,-1.,1.]
    flow[1,[4,24,0,20]] = [1/depth,-1/depth,1.,-1.]
    flow[:,19] = -seconds*np.asarray(cfg['nominal_velocity_xy_mps'])/depth**2
    return [('pose_like_xy',pose,.04),('range_like_scalar',scale,.12),('two_pose_flow_like_xy',flow,.004)]


def batch_condition(mean,load,models,centres,observations):
    """Condition the complete independent primitive vector, then map to state."""
    if not models:
        return mean.copy(),load @ load.T,None
    graph = np.vstack(models)
    covariance = graph @ graph.T
    cross = load @ graph.T
    gain = np.linalg.solve(covariance,cross.T).T
    residual = np.concatenate(observations)-np.concatenate(centres)
    posterior = mean+gain @ residual
    return posterior,load @ load.T-gain @ cross.T,gain


def reconstruct(cfg, primitive, keep_steps=False):
    """Small pure fixtures can use this oracle; audit() still enforces 36000."""
    count = len(primitive)
    mean,load = independent_initial(cfg)
    prior = load @ load.T
    filters = {name:{'x':np.repeat(mean[None,:],count,axis=0),'P':prior.copy(),
                     'stamp':cfg['initial_timestamp_ns'],'clone':None,'operation':'create_joint_state'} for name in METHODS}
    errors = {name:[] for name in METHODS};predicted = {name:[] for name in METHODS}
    models,centres,first_values,observations = [],[],[],[]
    steps=[];counts={name:Counter() for name in METHODS};max_mean=max_cov=0.
    cursor=20;stamp=cfg['initial_timestamp_ns']

    def snapshot(name):
        state=filters[name]
        return {'mean':state['x'][0].copy(),'covariance':state['P'].copy(),'timestamp_ns':state['stamp'],
                'clone':state['clone'],'operation':state['operation']}

    def record(name,operation,before,parameters=None):
        nonlocal max_mean,max_cov
        counts[name][operation]+=1
        after=snapshot(name)
        if name=='full_joint':
            expected_m,expected_p,_=batch_condition(mean,load,models,centres,first_values)
            max_mean=max(max_mean,float(np.max(np.abs(after['mean']-expected_m))))
            max_cov=max(max_cov,float(np.max(np.abs(after['covariance']-expected_p))))
        if keep_steps:
            steps.append({'method':name,'operation':operation,'window':window,'before':before,'after':after,
                          'parameters':parameters or {}})

    for window,duration in enumerate(cfg['windows_ns']):
        seconds=duration/1_000_000_000
        clone_name='start_'+str(window)
        if window:
            state=filters['reset_shared_nuisance'];before=snapshot('reset_shared_nuisance')
            state['x'][:,19]=0.;state['P'][19,:]=0.;state['P'][:,19]=0.;state['P'][19,19]=prior[19,19]
            state['operation']='create_joint_state'
            record('reset_shared_nuisance','reset_shared_nuisance',before)
        mean=np.concatenate((mean,mean[:6]));load=np.concatenate((load,load[:6,:]),axis=0)
        copy_map=np.vstack((np.eye(20),np.eye(20)[:6]))
        for name in METHODS:
            state=filters[name];before=snapshot(name)
            state['x']=state['x'] @ copy_map.T;state['P']=copy_map @ state['P'] @ copy_map.T
            state['clone']=(clone_name,state['stamp']);state['operation']='clone_pose'
            record(name,'clone',before,{'clone_id':clone_name})
        state=filters['zero_clone_cross'];before=snapshot('zero_clone_cross')
        state['P'][:20,20:]=0.;state['P'][20:,:20]=0.
        state['operation']='deliberately_wrong_zero_clone_cross_control'
        record('zero_clone_cross','zero_clone_cross',before)
        transform,process=independent_transition(seconds,cfg)
        mean=transform @ mean;load=transform @ load
        load[np.arange(19),cursor+np.arange(19)]=cfg['process_sigmas_per_window'];cursor+=19
        stamp+=duration
        for name in METHODS:
            state=filters[name];before=snapshot(name)
            state['x']=state['x'] @ transform.T
            state['P']=transform @ state['P'] @ transform.T;state['P'][:19,:19]+=process
            state['stamp']=stamp;state['operation']='propagate_active_explicit_independent_process_noise'
            record(name,'propagate',before,{'F':transform[:19,:19],'Q':process,'offset':np.zeros(19),'next_timestamp_ns':stamp})
        measurements=[]
        for label,measurement_map,sigma in independent_observations(seconds,cfg):
            dimension=len(measurement_map)
            white=np.zeros((dimension,116));white[np.arange(dimension),cursor+np.arange(dimension)]=sigma;cursor+=dimension
            model=measurement_map @ load+white;centre=measurement_map @ mean
            observed=primitive @ model.T+centre
            models.append(model);centres.append(centre);first_values.append(observed[0]);measurements.append(observed)
            noise=np.eye(dimension)*sigma**2
            for name in METHODS:
                state=filters[name];before=snapshot(name)
                covariance=state['P'];cross=covariance @ measurement_map.T
                gain=np.linalg.solve(measurement_map @ cross+noise,cross.T).T
                state['x']+=(observed-state['x'] @ measurement_map.T) @ gain.T
                joseph=np.eye(26)-gain @ measurement_map
                state['P']=joseph @ covariance @ joseph.T+gain @ noise @ gain.T
                state['P']=.5*(state['P']+state['P'].T)
                state['operation']='linear_update_explicit_independent_measurement_noise'
                record(name,'update_'+label,before,{'H':measurement_map,'R':noise,'measurement':observed[0],
                       'measurement_timestamp_ns':stamp,'gain':gain})
        observations.append(np.column_stack(measurements))
        truth=primitive @ load.T+mean
        _,_,batch_gain=batch_condition(mean,load,models,centres,first_values)
        batch_estimate=(np.concatenate(observations,axis=1)-np.concatenate(centres)) @ batch_gain.T+mean
        for name in METHODS:
            estimator=batch_estimate if name=='full_joint' else filters[name]['x']
            errors[name].append(truth[:,7:9]-estimator[:,7:9])
            predicted[name].append(filters[name]['P'][np.ix_([7,8],[7,8])].copy())
        mean=mean[:20];load=load[:20,:]
        for name in METHODS:
            state=filters[name];before=snapshot(name)
            state['x']=state['x'][:,:20];state['P']=state['P'][:20,:20]
            state['clone']=None;state['operation']='drop_clone_principal_marginal'
            record(name,'drop_clone',before,{'clone_id':clone_name})
    require(cursor==116,'independent primitive graph complete')
    truth=primitive @ load.T+mean
    _,_,batch_gain=batch_condition(mean,load,models,centres,first_values)
    full_estimate=(np.concatenate(observations,axis=1)-np.concatenate(centres)) @ batch_gain.T+mean
    final={name:truth-(full_estimate if name=='full_joint' else filters[name]['x']) for name in METHODS}
    return {'observations':np.stack(observations,axis=1),'velocity_errors':{k:np.stack(v,axis=1) for k,v in errors.items()},
            'final_errors':final,'predicted':predicted,'final_covariance':{k:v['P'] for k,v in filters.items()},
            'steps':steps,'operation_counts':{k:dict(v) for k,v in counts.items()},
            'maximum_sequential_batch_mean_error':max_mean,'maximum_sequential_batch_covariance_error':max_cov}


def check_state(value, expected, label):
    mean=expected['mean'];covariance=expected['covariance'];dimension=len(mean)
    require(type(value['dimension']) is int and value['dimension']==dimension and
            type(value['timestamp_ns']) is int and value['timestamp_ns']==expected['timestamp_ns'],label+' dimension/time')
    close(value['mean'],mean,label+' mean');close(value['covariance'],covariance,label+' covariance')
    actual=numeric(value['covariance'],(dimension,dimension),label+' covariance')
    scale=np.sqrt(np.diag(actual));require(np.all(scale>0),label+' positive variances')
    correlation=actual/scale[:,None]/scale[None,:]
    require(np.max(np.abs(correlation-correlation.T))<=1e-10 and
            np.linalg.eigvalsh(.5*(correlation+correlation.T))[0]>=-1e-10,label+' balanced PSD/symmetry')
    clone=expected['clone']
    clones=[] if clone is None else [{'clone_id':clone[0],'timestamp_ns':clone[1],'indices':list(range(20,26))}]
    require(value['active_indices']==list(range(19)) and value['nuisance_indices']==[19] and value['clones']==clones,label+' layout/clone')
    require(value['scope']=='offline_discrete_linear_joint_gaussian_fixture' and
            all(value[k] is False for k in ('physical_rotation_model','livo_integration','hardware_validated','protection_level_available')),
            label+' software boundary')
    diagnostic=value['diagnostics']
    require(diagnostic['operation']==expected['operation'] and diagnostic['jitter_added'] is False and
            diagnostic['eigenvalues_clipped'] is False,label+' operation/no numeric repair')


def check_steps(rows, expected):
    require(len(rows)==len(expected)==79,'exact 79 first-seed operation ledger rows')
    for i,(actual,wanted) in enumerate(zip(rows,expected)):
        label='ledger row '+str(i)
        require(set(actual)=={'method','operation','window','seed','before','after','parameters'},label+' keys')
        require(actual['method']==wanted['method'] and actual['operation']==wanted['operation'] and
                type(actual['window']) is int and actual['window']==wanted['window'] and
                type(actual['seed']) is int and actual['seed']==SEEDS[0],label+' identity')
        check_state(actual['before'],wanted['before'],label+' before')
        check_state(actual['after'],wanted['after'],label+' after')
        require(set(actual['parameters'])==set(wanted['parameters']),label+' parameter names')
        for key,value in wanted['parameters'].items():
            if isinstance(value,str):require(actual['parameters'][key]==value,label+' '+key)
            elif isinstance(value,int):require(type(actual['parameters'][key]) is int and actual['parameters'][key]==value,label+' '+key)
            else:close(actual['parameters'][key],value,label+' '+key)


def audit(study_dir, protocol_path):
    study_dir,protocol_path=Path(study_dir).resolve(),Path(protocol_path).resolve()
    result={'schema_version':1,'status':'FAIL','checks':{},'errors':[],'input_evidence':{},'arrays_read':{},
            'recomputed_cases':[],'final_joint_statistics':{},'production_imported':False,
            'scope':'Independent declared linear latent graph, all three sequential models, full-joint batch conditioning and artifact audit; no real flow/LIVO/AP/hardware/integrity claim'}
    def checked(name,condition):
        result['checks'][name]=bool(condition);require(condition,name)
    try:
        cfg=frozen_protocol(read(protocol_path));result['input_evidence'][str(protocol_path)]=file_record(protocol_path)
        checked('frozen_36000_trajectory_protocol_and_marginal_vx_scope',True)
        checked('closed_study_input_file_set', {p.name for p in study_dir.iterdir()} in (INPUTS,INPUTS|{'audit-result.json'})
                and all((study_dir/name).is_file() and not (study_dir/name).is_symlink() for name in INPUTS))
        for name in sorted(INPUTS):result['input_evidence'][str(study_dir/name)]=file_record(study_dir/name)
        declared,index=read(study_dir/'study-result.json'),read(study_dir/'raw-array-index.json')
        checked('declared_complete_software_study',type(declared['schema_version']) is int and declared['schema_version']==1 and
                declared['status']=='PASS' and type(declared['independent_trajectories']) is int and declared['independent_trajectories']==36000 and
                type(declared['correlated_windows']) is int and declared['correlated_windows']==144000 and
                declared['scope']==cfg['scope'] and declared['coverage_direction_velocity_xy']==[1.,0.] and declared['coverage_scope']==COVERAGE_SCOPE)
        checked('no_recorded_or_hardware_fusion_claim',type(declared['actual_recorded_flow_state_updates']) is int and
                declared['actual_recorded_flow_state_updates']==0 and declared['full_common_state_flow_fusion_validated'] is False and
                declared['joint_state_software_updates_performed'] is True and declared['hardware_readiness']=='NOT_READY')
        archive=study_dir/'samples.npz';expected={};count=12000
        for seed in SEEDS:
            prefix='seed'+str(seed)+'_';expected[prefix+'primitive_standard_normal']=(count,116)
            expected[prefix+'observations']=(count,4,5)
            for name in METHODS:
                expected[prefix+name+'_velocity_error_xy']=(count,4,2)
                expected[prefix+name+'_final_joint_error']=(count,20)
        checked('embedded_index_and_archive_size_sha',declared['raw_arrays']==index and type(index['bytes']) is int and
                index['bytes']==archive.stat().st_size and index['sha256']==digest(archive) and
                isinstance(index['path'],str) and index['path'].replace('\\','/').rsplit('/',1)[-1]=='samples.npz')
        checked('index_exact_24_raw_arrays',set(index['arrays'])==set(expected) and len(index['arrays'])==24)
        with zipfile.ZipFile(archive) as zipped:
            members=zipped.infolist()
            checked('npz_exact_members_no_duplicate_or_unbounded_payload',len(members)==24 and
                    {x.filename for x in members}=={x+'.npy' for x in expected} and all(
                        not x.is_dir() and np.prod(expected[x.filename[:-4]])*8 <= x.file_size
                        <=np.prod(expected[x.filename[:-4]])*8+4096 for x in members))
        velocity={name:[] for name in METHODS};final={name:[] for name in METHODS};oracles=[]
        with np.load(archive,allow_pickle=False) as raw:
            checked('loader_exact_24_arrays',len(raw.files)==24 and set(raw.files)==set(expected))
            for seed in SEEDS:
                prefix='seed'+str(seed)+'_';block={}
                for key,shape in expected.items():
                    if not key.startswith(prefix):continue
                    value=raw[key];meta=index['arrays'][key]
                    checked(key+'.shape_dtype_finite',value.shape==shape and value.dtype==np.dtype('float64') and
                            np.isfinite(value).all() and meta['shape']==list(shape) and
                            all(type(x) is int for x in meta['shape']) and meta['dtype']=='float64')
                    block[key[len(prefix):]]=value
                    result['arrays_read'][key]={'shape':list(shape),'dtype':'float64','sha256_c_order_values':hashlib.sha256(value.tobytes()).hexdigest()}
                generated=np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed,0]))).standard_normal((count,116))
                checked(prefix+'independent_primitive_rng_identity',np.array_equal(block['primitive_standard_normal'],generated))
                oracle=reconstruct(cfg,generated,keep_steps=seed==SEEDS[0]);oracles.append(oracle)
                close(block['observations'],oracle['observations'],prefix+'independent latent observations')
                checked(prefix+'independent_observation_graph',True)
                for name in METHODS:
                    close(block[name+'_velocity_error_xy'],oracle['velocity_errors'][name],prefix+name+' window errors')
                    close(block[name+'_final_joint_error'],oracle['final_errors'][name],prefix+name+' final20 errors')
                    checked(prefix+name+'.all_windows_and_final20_match_independent_model',True)
                    velocity[name].append(block[name+'_velocity_error_xy']);final[name].append(block[name+'_final_joint_error'])
                checked(prefix+'full_joint_sequential_batch_all_operations',
                        oracle['maximum_sequential_batch_mean_error']<=1e-10 and oracle['maximum_sequential_batch_covariance_error']<=1e-10)
        rows=[loads(line) for line in (study_dir/'joint-steps.jsonl').read_text().splitlines()]
        check_steps(rows,oracles[0]['steps'])
        checked('all_79_ledger_before_after_parameters_clone_drop_and_cross_blocks',True)
        checked('all_24_saved_arrays_read',len(result['arrays_read'])==24)
        checked('four_declared_windows',len(declared['cases'])==4)
        expected_gates={}
        for window in range(4):
            row=declared['cases'][window]
            require(type(row['window']) is int and row['window']==window and type(row['independent_trajectories']) is int and
                    row['independent_trajectories']==36000 and set(row['methods'])==set(METHODS),'declared window identity')
            report={'window':window,'independent_trajectories':36000,'methods':{}}
            for name in METHODS:
                predicted=oracles[0]['predicted'][name][window]
                for other in oracles[1:]:close(other['predicted'][name][window],predicted,'seed-independent predicted covariance')
                actual=statistics(np.concatenate(velocity[name])[:,window,:],predicted);given=row['methods'][name]
                require(set(given)==set(actual),'declared metric fields '+name)
                for key,value in actual.items():
                    if key=='covered_trials':require(type(given[key]) is int and given[key]==value,'coverage count '+name)
                    else:close(given[key],value,'window '+str(window)+' '+name+' '+key)
                report['methods'][name]=actual
                checked('window'+str(window)+'.'+name+'.all_statistics_recomputed',True)
            correct=report['methods']['full_joint']
            expected_gates['window'+str(window)+'.full_joint_coverage']=.93<=correct['coverage']<=.97
            expected_gates['window'+str(window)+'.full_joint_covariance']=correct['relative_covariance_error']<=.05
            result['recomputed_cases'].append(report)
        for name in METHODS:
            result['final_joint_statistics'][name]=final_statistics(np.concatenate(final[name]),oracles[0]['final_covariance'][name])
        checked('final20_all_models_covariance_mean_and_marginal_intervals_finite',all(
                np.isfinite(np.asarray(value['empirical_covariance_20'])).all() for value in result['final_joint_statistics'].values()))
        expected_counts={name:{op:value*3 for op,value in oracles[0]['operation_counts'][name].items()} for name in METHODS}
        checked('all_operation_counts_and_72_batch_checks',declared['operation_counts']==expected_counts and
                type(declared['batch_operation_checks']) is int and declared['batch_operation_checks']==72)
        for key in ('maximum_batch_mean_error','maximum_batch_covariance_error','maximum_sample_kernel_mean_error'):
            require(type(declared[key]) in (int,float) and math.isfinite(declared[key]) and 0<=declared[key]<=1e-10,'reported arithmetic bound '+key)
        first_difference=float(np.max(np.abs(oracles[0]['predicted']['full_joint'][0]-oracles[0]['predicted']['reset_shared_nuisance'][0])))
        expected_gates.update(batch_all_operations_mean=True,batch_all_operations_covariance=True,sampled_update_matches_kernel_mean=True,
                              first_window_scale_reset_control=first_difference<=1e-12,
                              all_arrays_read_back=True,sample_archive_unchanged=True)
        checked('declared_14_gates_reproduced_without_ablation_sign_gate',declared['checks']==expected_gates and
                all(type(x) is bool and x for x in expected_gates.values()))
        result['independent_trajectories']=36000;result['correlated_windows']=144000
        result['ledger_rows_checked']=len(rows);result['batch_operation_checks']=72
        result['maximum_independent_sequential_batch_mean_error']=max(x['maximum_sequential_batch_mean_error'] for x in oracles)
        result['maximum_independent_sequential_batch_covariance_error']=max(x['maximum_sequential_batch_covariance_error'] for x in oracles)
        checked('inputs_unchanged_during_audit',all(file_record(Path(path))==value for path,value in result['input_evidence'].items()))
        checked('execution_completed',True)
    except Exception as error:
        result['errors'].append(type(error).__name__+': '+str(error));result['checks']['execution_completed']=False
    result['status']='PASS' if result['checks'] and all(result['checks'].values()) and not result['errors'] else 'FAIL'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir',required=True,type=Path);parser.add_argument('--protocol',required=True,type=Path)
    parser.add_argument('--output',type=Path);args=parser.parse_args()
    output=args.output or args.study_dir/'audit-result.json'
    protected={(args.study_dir/name).resolve() for name in INPUTS}|{args.protocol.resolve()}
    if output.resolve() in protected:parser.error('audit output must not overwrite an input')
    result=audit(args.study_dir,args.protocol)
    output.parent.mkdir(parents=True,exist_ok=True)
    temporary=output.with_name(output.name+'.tmp');temporary.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');temporary.replace(output)
    print(json.dumps({'status':result['status'],'checks':len(result['checks']),'arrays_read':len(result['arrays_read']),
                      'ledger_rows_checked':result.get('ledger_rows_checked'),'errors':result['errors']}))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
