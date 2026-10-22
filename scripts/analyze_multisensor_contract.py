#!/usr/bin/env python3
"""R2A interface acceptance; hardware/fusion readiness are deliberately separate."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
from xq_autonomy.multisensor_contract import validate_contract_config


def evaluate(run):
    errors, hashes = {}, {}
    def read(relative):
        try:
            raw = (run/relative).read_bytes()
            hashes[relative] = hashlib.sha256(raw).hexdigest()
            value = json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
            if not isinstance(value, dict):
                raise ValueError('expected object')
            return value
        except (OSError, ValueError) as exc:
            errors[relative] = str(exc)
            return {}
    base, measured = read('gate-result.json'), read('multisensor-result.json')
    protocol = read('configuration/research_multisensor_gate.json')
    config = read('configuration/research_multisensor_contract.json')
    matrix, bag = read('multisensor-fault-matrix.json'), read('bag-audit.json')
    checks = {'inputs_readable':not errors, 'ap_dynamics_gate_pass':base.get('status')=='PASS',
              'bag_valid':bag.get('status')=='PASS' and isinstance(bag.get('message_count'),int)
                          and not isinstance(bag.get('message_count'),bool) and bag['message_count']>0}
    validation = measured.get('config_validation', {})
    actual_validation = validate_contract_config(config)
    checks.update({
        'simulation_config_valid': (validation.get('status')=='PASS' and actual_validation['status']=='PASS'
                                   and config.get('profile')=='simulation'),
        'hardware_not_claimed': validation.get('hardware_readiness',{}).get('status')=='NOT_READY'
                              and measured.get('hardware_calibration_validated') is False
                              and config.get('hardware_calibration_validated') is False,
        'independence_not_claimed': validation.get('independent_fusion_ready') is False
                                    and config.get('assume_independent_fusion') is False,
        'truth_not_consumed': measured.get('ground_truth_subscribed') is False,
        'shadow_only': measured.get('publishes_navigation_or_control') is False
                       and measured.get('metric_flow_fusion_ready') is False
                       and config.get('streams',{}).get('flow',{}).get('ap_optical_flow_injection') is False,
        'measurement_contract_errors_absent': isinstance(measured.get('errors'),dict) and not measured['errors'],
        'fault_matrix_pass': (matrix.get('status')=='PASS' and isinstance(matrix.get('cases'),dict)
            and set(matrix['cases'])==set(protocol.get('fault_cases',[]))
            and bool(matrix['cases']) and all(item.get('passed') is True for item in matrix['cases'].values())),
    })
    def number(value):
        return not isinstance(value,bool) and isinstance(value,(int,float)) and math.isfinite(value)
    streams = measured.get('streams',{})
    for name in ('lidar','imu','front_camera','down_camera','flow'):
        observed = streams.get(name,{})
        age, rate, count = observed.get('age_p99_s'), observed.get('rate_hz'), observed.get('count')
        first, last = observed.get('first_stamp_ns'), observed.get('last_stamp_ns')
        checks[name+'_count'] = number(count) and count>=protocol.get('minimum_counts',{}).get(name,math.inf)
        checks[name+'_source_rate'] = number(rate) and rate>=protocol.get('minimum_rates_hz',{}).get(name,math.inf)
        checks[name+'_source_age_p99'] = (number(age) and
            -protocol.get('maximum_future_skew_s',0)<=age<=protocol.get('maximum_age_p99_s',{}).get(name,-math.inf))
        checks[name+'_monotonic'] = observed.get('duplicate_or_backward')==0
        checks[name+'_source_span'] = number(first) and number(last) and (last-first)*1e-9>=protocol.get('minimum_source_span_s',math.inf)
    for name in ('front_camera','down_camera'):
        pixels = measured.get('pixel_stats',{}).get(name,{})
        pairing = measured.get('camera_info_stamp_match',{}).get(name,{})
        checks[name+'_rendered_nonuniform'] = (number(pixels.get('std')) and pixels['std']>=protocol.get('minimum_image_std',math.inf)
            and pixels.get('distinct_values',0)>=protocol.get('minimum_image_distinct_values',math.inf))
        checks[name+'_paired_calibration'] = (pairing.get('image_count',0)>0 and
            pairing.get('matched',0)/pairing['image_count']>=protocol.get('minimum_camera_info_match_fraction',math.inf))
    checks['all_sensor_transforms_match'] = all(
        measured.get('tf_checks',{}).get(key,{}).get('valid') is True
        for key in ('lidar','imu','front_camera','down_camera'))
    flow = measured.get('flow',{})
    valid, invalid = flow.get('valid_count',0), flow.get('invalid_count',0)
    checks['angular_flow_sufficient'] = valid>=protocol.get('minimum_valid_flow_samples',math.inf)
    checks['angular_flow_valid_fraction'] = valid+invalid>0 and valid/(valid+invalid)>=protocol.get('minimum_valid_flow_fraction',math.inf)
    checks['gyro_windows_independently_supported'] = (flow.get('gyro_bad_count')==0 and
        flow.get('gyro_verified_count',0)>=valid-protocol.get('maximum_unverified_flow_windows',-math.inf))
    checks['angular_motion_observed'] = flow.get('maximum_compensated_norm_rad',0)>=protocol.get('minimum_compensated_motion_rad',math.inf)
    topics = bag.get('topics',{})
    checks['raw_camera_and_flow_bagged'] = all(topics.get(topic,0)>0 for topic in (
        '/impact/r2/front/image_raw','/impact/r2/front/camera_info',
        '/impact/r2/down/image_raw','/impact/r2/down/camera_info','/impact/r2/flow/measurement'))
    failed = [name for name,passed in checks.items() if not passed]
    return {'schema_version':1, 'status':'PASS' if not failed else 'FAIL', 'checks':checks,
            'failed_checks':failed,'input_errors':errors,'input_sha256':hashes,
            'scope':protocol.get('scope'), 'streams':streams,'flow':flow,
            'hardware_readiness':'NOT_READY','metric_flow_fusion_ready':False,
            'ap_optical_flow_ingestion_tested':False,'statistical_integrity_tested':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    run=parser.parse_args().run_dir.resolve()
    try:
        result=evaluate(run)
    except (OSError, ValueError, KeyError, TypeError, IndexError, AttributeError, OverflowError) as exc:
        result={'schema_version':1,'status':'FAIL','checks':{},'failed_checks':['malformed_input'],
                'input_errors':{'structure':str(exc)},'hardware_readiness':'NOT_READY'}
    (run/'multisensor-gate-result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(f"{result['status']}: R2A multisensor contract ({len(result['checks'])} checks); failed={result['failed_checks']}")
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
