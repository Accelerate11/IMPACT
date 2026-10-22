"""Small independent-oracle and hostile-artifact tests; never a formal study."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


SCRIPT=Path(__file__).resolve().parents[1]/'audit_joint_window_study.py'
SPEC=importlib.util.spec_from_file_location('joint_window_audit',SCRIPT)
AUDIT=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(AUDIT)
PROTOCOL=SCRIPT.parent.parent/'config/research_joint_window_gate.json'


def protocol():return json.loads(PROTOCOL.read_text())


def state(snapshot):
    clone=snapshot['clone'];dimension=len(snapshot['mean'])
    return {'mean':snapshot['mean'].tolist(),'covariance':snapshot['covariance'].tolist(),
            'dimension':dimension,'timestamp_ns':snapshot['timestamp_ns'],'active_indices':list(range(19)),
            'nuisance_indices':[19],'clones':[] if clone is None else
            [{'clone_id':clone[0],'timestamp_ns':clone[1],'indices':list(range(20,26))}],
            'diagnostics':{'operation':snapshot['operation'],'jitter_added':False,'eigenvalues_clipped':False},
            'scope':'offline_discrete_linear_joint_gaussian_fixture','physical_rotation_model':False,
            'livo_integration':False,'hardware_validated':False,'protection_level_available':False}


@pytest.fixture(scope='module')
def small():
    # 12 trajectories only. This is NOT a reduced formal experiment or pass.
    cfg=protocol()['joint_study']
    latent=np.random.default_rng(912).standard_normal((12,116))
    return AUDIT.reconstruct(cfg,latent,keep_steps=True)


def rows(oracle):
    result=[]
    for step in oracle['steps']:
        result.append({'seed':AUDIT.SEEDS[0],**{k:v for k,v in step.items() if k not in ('before','after')},
                       'before':state(step['before']),'after':state(step['after'])})
    return json.loads(json.dumps(result,default=lambda x:x.tolist()))


@pytest.mark.parametrize('change',[('draws_per_seed',100),('primitive_dimension',115),
                                   ('coverage_direction_velocity_xy',[0.,1.]),('methods',['full_joint'])])
def test_frozen_formal_protocol_rejects_changed_fixture(change):
    value=protocol();value['joint_study'][change[0]]=change[1]
    with pytest.raises(ValueError):AUDIT.frozen_protocol(value)


def test_complete_protocol_and_missing_hardware_scope_rejected():
    value=protocol();AUDIT.frozen_protocol(value)
    value['full_common_state_flow_fusion_validated']=True
    with pytest.raises(ValueError):AUDIT.frozen_protocol(value)


def test_direct_primitive_batch_matches_two_dimensional_information_solution():
    # x=(a,u), Var(a)=1, Var(u)=4, z=(a+u+.5n1, a-u+.7n2).
    mean=np.array([.2,-.3]);load=np.array([[1.,0,0,0],[0,2.,0,0]])
    graph=[np.array([[1.,2.,.5,0]]),np.array([[1.,-2.,0,.7]])]
    h=np.array([[1.,1.],[1.,-1.]])
    z=np.array([1.2,-.4]);centres=[h[0]@mean,h[1]@mean]
    actual_mean,actual_cov,_=AUDIT.batch_condition(mean,load,graph,[np.atleast_1d(x) for x in centres],
                                                [z[:1],z[1:]])
    precision=np.diag([1.,.25])+h.T@np.diag([4.,1/.49])@h
    expected_cov=np.linalg.solve(precision,np.eye(2))
    expected_mean=expected_cov@(np.diag([1.,.25])@mean+h.T@np.diag([4.,1/.49])@z)
    np.testing.assert_allclose(actual_cov,expected_cov,atol=2e-15)
    np.testing.assert_allclose(actual_mean,expected_mean,atol=2e-15)


def test_small_multistep_oracle_matches_batch_and_retains_true_clones(small):
    assert small['maximum_sequential_batch_mean_error']<1e-12
    assert small['maximum_sequential_batch_covariance_error']<1e-12
    assert len(small['steps'])==79
    np.testing.assert_array_equal(small['predicted']['full_joint'][0],small['predicted']['reset_shared_nuisance'][0])
    clone=next(x for x in small['steps'] if x['method']=='full_joint' and x['operation']=='clone')
    np.testing.assert_array_equal(clone['after']['covariance'][:20,20:],clone['before']['covariance'][:,:6])
    drop=next(x for x in small['steps'] if x['method']=='full_joint' and x['operation']=='drop_clone')
    np.testing.assert_array_equal(drop['after']['covariance'],drop['before']['covariance'][:20,:20])
    AUDIT.check_steps(rows(small),small['steps'])


@pytest.mark.parametrize('kind',['interior_cross','mean','clone_time','ledger_order','gain','scope'])
def test_ledger_tampering_is_rejected(small,kind):
    value=rows(small)
    clone=next(x for x in value if x['method']=='full_joint' and x['operation']=='clone')
    if kind=='interior_cross':clone['after']['covariance'][19][22]+=.01
    elif kind=='mean':value[-1]['after']['mean'][7]+=.1
    elif kind=='clone_time':clone['after']['clones'][0]['timestamp_ns']+=1
    elif kind=='ledger_order':value[0],value[1]=value[1],value[0]
    elif kind=='gain':next(x for x in value if x['operation']=='update_two_pose_flow_like_xy')['parameters']['gain'][19][0]+=.1
    else:clone['after']['livo_integration']=True
    with pytest.raises(ValueError):AUDIT.check_steps(value,small['steps'])


def test_hand_statistics_and_final20_coordinate_intervals():
    errors=np.array([[-.1,0],[.1,0],[-.3,0],[.3,0]])
    value=AUDIT.statistics(errors,np.eye(2)*.01)
    assert value['covered_trials']==2 and value['coverage']==.5
    np.testing.assert_allclose(value['empirical_covariance_xy'],[[.05,0],[0,0]])
    result=AUDIT.final_statistics(np.repeat(errors[:,:1],20,axis=1),np.eye(20)*.01)
    assert all(x['covered_trajectories']==2 for x in result['coordinate_marginal_95_intervals'])
    np.testing.assert_allclose(AUDIT.score_interval(95,100),[.8882495307680809,.978456320845632])


@pytest.mark.parametrize('text',['{"a":1,"a":2}','{"x":NaN}','{"x":1e999}'])
def test_ambiguous_or_nonfinite_json_rejected(text):
    with pytest.raises(ValueError):AUDIT.loads(text)


def test_small_development_protocol_cannot_receive_formal_pass(tmp_path):
    value=protocol();value['joint_study']['draws_per_seed']=12
    path=tmp_path/'protocol.json';path.write_text(json.dumps(value))
    study=tmp_path/'study';study.mkdir()
    result=AUDIT.audit(study,path)
    assert result['status']=='FAIL' and 'frozen' in result['errors'][0]
    assert not (study/'audit-result.json').exists()


def test_audit_cli_cannot_overwrite_primitive_input_or_protocol(tmp_path):
    path=tmp_path/'protocol.json';path.write_text(json.dumps(protocol()))
    study=tmp_path/'study';study.mkdir();target=study/'samples.npz';target.write_bytes(b'keep')
    for output in (target,path):
        before=output.read_bytes()
        result=subprocess.run([sys.executable,str(SCRIPT),'--study-dir',str(study),'--protocol',str(path),
                               '--output',str(output)],capture_output=True,text=True)
        assert result.returncode==2 and output.read_bytes()==before
