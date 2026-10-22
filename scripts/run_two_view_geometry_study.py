#!/usr/bin/env python3
"""Deterministic two-view and ordered gyro fixtures; independent quaternion oracle."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from xq_autonomy.two_view_geometry import integrate_gyro_midpoint,predict_two_view


def save(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def quaternion(vector):
    angle=float(np.linalg.norm(vector))
    if angle<1e-12:return np.r_[1.,.5*np.asarray(vector)]/np.linalg.norm(np.r_[1.,.5*np.asarray(vector)])
    return np.r_[math.cos(angle/2),np.asarray(vector)*(math.sin(angle/2)/angle)]


def multiply(q,r):
    return np.r_[q[0]*r[0]-q[1:]@r[1:],q[0]*r[1:]+r[0]*q[1:]+np.cross(q[1:],r[1:])]


def rotate(q,point):
    return point+2*q[0]*np.cross(q[1:],point)+2*np.cross(q[1:],np.cross(q[1:],point))


def matrix(q):return np.column_stack([rotate(q,np.eye(3)[i]) for i in range(3)])


def log_quaternion(q):
    q=q/np.linalg.norm(q)
    if q[0]<0:q=-q
    n=float(np.linalg.norm(q[1:]))
    return 2*q[1:] if n<1e-15 else q[1:]*(2*math.atan2(n,q[0])/n)


def oracle_integrate(stamps,rates,start,end,bias,q0):
    q=q0.copy();summed=np.zeros(3);segments=[]
    for i,(left,right) in enumerate(zip(stamps,stamps[1:])):
        a=max(start,int(left));b=min(end,int(right))
        if a>=b:continue
        midpoint_fraction=((a-int(left))+(b-int(left)))/(2*(int(right)-int(left)))
        rate=rates[i]+midpoint_fraction*(rates[i+1]-rates[i])-bias
        vector=rate*((b-a)*1e-9);q=multiply(q,quaternion(vector));q/=np.linalg.norm(q)
        summed+=vector;segments.append([i,a,b])
    return q,summed,segments


def oracle_projection(xy,depth,q0,p0,q1,p1,qbc,lever):
    body0=rotate(qbc,np.r_[xy,1.]*depth)+lever
    world=rotate(q0,body0)+p0
    inverse1=np.r_[q1[0],-q1[1:]];inversebc=np.r_[qbc[0],-qbc[1:]]
    camera1=rotate(inversebc,rotate(inverse1,world-p1)-lever)
    if camera1[2]<=1e-8:raise ValueError('fixture projects behind camera')
    return camera1[:2]/camera1[2]


def run(protocol,output):
    if output.exists():raise ValueError('Output must be new')
    output.mkdir(parents=True);cfg=protocol['geometry_study'];limits=cfg['checks'];step=cfg['finite_difference_step']
    records=[];checks={};max_rotation=max_bias=max_projection=max_jacobian=0.;switched_gap=0.
    for name in cfg['profiles']:
        epoch=1700000000000000000
        stamps=epoch+np.arange(201,dtype=np.int64)*5000000
        t=(stamps-epoch)*1e-9
        bias=np.array([.02,-.03,.01]);q0=quaternion(np.array([.15,-.08,.04]))
        if name=='constant_axis':rates=np.tile([.4,-.2,.1],(201,1))+bias
        elif name=='switched_axes':
            rates=np.zeros((201,3));rates[:100,0]=1.2;rates[100:,1]=1.2;rates+=bias
        elif name=='varying_clipped_epoch':rates=np.column_stack((.3*np.cos(4*t),.5*np.sin(3*t),.2*np.cos(2*t)))+bias
        else:raise ValueError('Unsupported gyro profile')
        start,end=int(stamps[0]),int(stamps[-1])
        if name=='varying_clipped_epoch':start+=2500000;end-=2500000
        result=integrate_gyro_midpoint(stamps.tolist(),rates,start,end,bias,
            initial_world_from_body_rotation=matrix(q0),maximum_gap_ns=6000000)
        q,integral,segments=oracle_integrate(stamps,rates,start,end,bias,q0)
        rotation_error=float(np.max(np.abs(np.asarray(result['world_from_body_rotation'])-matrix(q))))
        max_rotation=max(max_rotation,rotation_error)
        numeric=[]
        for axis in range(3):
            delta=np.eye(3)[axis]*step
            plus=oracle_integrate(stamps,rates,start,end,bias+delta,q0)[0]
            minus=oracle_integrate(stamps,rates,start,end,bias-delta,q0)[0]
            inverse=np.r_[q[0],-q[1:]]
            numeric.append((log_quaternion(multiply(inverse,plus))-log_quaternion(multiply(inverse,minus)))/(2*step))
        bias_error=float(np.max(np.abs(np.asarray(result['rotation_jacobian_bias'])-np.column_stack(numeric))))
        max_bias=max(max_bias,bias_error)
        if name=='switched_axes':
            ordered_relative=multiply(np.r_[q0[0],-q0[1:]],q)
            inverse_sum=quaternion(-integral)
            switched_gap=float(np.linalg.norm(log_quaternion(multiply(inverse_sum,ordered_relative))))
        reported=[[s['sample_index'],s['start_ns'],s['end_ns']] for s in result['segments']]
        checks[name+'.all_clipped_segment_intervals']=reported==segments
        checks[name+'.unknown_covariance']=result['covariance'] is None
        records.append({'kind':'gyro','profile':name,'input':{'stamps_ns':stamps.tolist(),'rates':rates.tolist(),
            'start_ns':start,'end_ns':end,'bias':bias.tolist(),'initial_rotation':matrix(q0).tolist()},
            'result':result,'oracle_quaternion':q.tolist(),'rotation_error':rotation_error,'bias_jacobian_error':bias_error})
    rng=np.random.default_rng(cfg['random_seed']);qbc=quaternion(np.array([0.,0.,math.pi]))
    # A valid fixed extrinsic with nonzero lever arm; the controlled fixture
    # deliberately does not impersonate the R2A camera's optical convention.
    lever=np.array([.03,-.02,-.05])
    for case in range(cfg['two_view_cases']):
        xy=rng.uniform(-.4,.4,2);depth=float(rng.uniform(1.,5.))
        q0=quaternion(rng.uniform(-.2,.2,3));q1=multiply(q0,quaternion(rng.uniform(-.12,.12,3)))
        p0=rng.uniform(-2.,2.,3);p1=p0+rng.uniform(-.1,.1,3)
        result=predict_two_view(xy,depth,matrix(q0),p0,matrix(q1),p1,matrix(qbc),lever)
        expected=oracle_projection(xy,depth,q0,p0,q1,p1,qbc,lever)
        projection_error=float(np.max(np.abs(np.asarray(result['new_normalized_ray'])[:2]-expected)))
        max_projection=max(max_projection,projection_error)
        derivative=[]
        for column in range(13):
            values=[]
            for sign in (1,-1):
                a,b,c,d=q0.copy(),p0.copy(),q1.copy(),p1.copy();z=depth;amount=sign*step
                if column<3:a=multiply(a,quaternion(np.eye(3)[column]*amount))
                elif column<6:b[column-3]+=amount
                elif column<9:c=multiply(c,quaternion(np.eye(3)[column-6]*amount))
                elif column<12:d[column-9]+=amount
                else:z+=amount
                values.append(oracle_projection(xy,z,a,b,c,d,qbc,lever))
            derivative.append((values[0]-values[1])/(2*step))
        expected_j=np.column_stack(derivative)
        actual_j=np.column_stack((result['jacobian_start_pose'],result['jacobian_end_pose'],result['jacobian_depth_m']))
        error=float(np.max(np.abs(actual_j-expected_j)));max_jacobian=max(max_jacobian,error)
        old_j=np.column_stack([(oracle_projection(xy+np.eye(2)[i]*step,depth,q0,p0,q1,p1,qbc,lever)-
             oracle_projection(xy-np.eye(2)[i]*step,depth,q0,p0,q1,p1,qbc,lever))/(2*step) for i in range(2)])
        max_jacobian=max(max_jacobian,float(np.max(np.abs(np.asarray(result['jacobian_old_normalized_xy'])-old_j))))
        checks['case'+str(case)+'.unknown_covariance']=result['covariance'] is None
        records.append({'kind':'two_view','case':case,'input':{'normalized_xy':xy.tolist(),'depth_m':depth,
            'q0':q0.tolist(),'p0':p0.tolist(),'q1':q1.tolist(),'p1':p1.tolist(),'qbc':qbc.tolist(),'lever':lever.tolist()},
            'result':result,'projection_error':projection_error,'jacobian_error':error})
    checks.update(rotation_matches_independent_quaternion=max_rotation<=limits['maximum_rotation_matrix_error'],
        bias_jacobian_matches_independent_quaternion_difference=max_bias<=limits['maximum_jacobian_error'],
        two_view_matches_independent_static_world_point=max_projection<=limits['maximum_projection_error'],
        all_pose_depth_and_pixel_jacobians_match_independent_difference=max_jacobian<=limits['maximum_jacobian_error'],
        noncommuting_rotation_exposes_component_integral_approximation=switched_gap>=limits['minimum_switched_axis_component_integral_disagreement_rad'])
    with (output/'geometry-cases.jsonl').open('w') as stream:
        for row in records:stream.write(json.dumps(row,allow_nan=False)+'\n')
    result={'schema_version':1,'status':'PASS' if all(checks.values()) else 'FAIL','checks':{k:bool(v) for k,v in checks.items()},
        'gyro_profiles':len(cfg['profiles']),'two_view_cases':cfg['two_view_cases'],'maximum_rotation_error':max_rotation,
        'maximum_bias_jacobian_error':max_bias,'maximum_projection_error':max_projection,'maximum_two_view_jacobian_error':max_jacobian,
        'switched_axis_component_integral_disagreement_rad':switched_gap,
        'scope':'Deterministic declared midpoint integration and conditional geometry only; supplied bias/depth are not sensor calibration. No covariance, actual flow fusion, AP, hardware or protection level.'}
    save(output/'geometry-result.json',result);return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    result=run(json.loads(args.protocol.read_text()),args.output)
    print(json.dumps({k:v for k,v in result.items() if k!='checks'}));return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
