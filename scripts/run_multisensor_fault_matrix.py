#!/usr/bin/env python3
"""Replay frozen metadata faults against real R2 source examples, off the AP path."""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path

from xq_autonomy.multisensor_contract import (
    StreamClockGuard, check_camera_model, check_gyro_coverage, check_independent_fusion,
    check_rigid_transform, flow_metric_velocity, validate_contract_config,
)


def evaluate(config, measured):
    examples = measured['source_examples']
    sample = examples['down_camera']
    stamp, now, wall = sample['stamp_ns']*1e-9, sample['source_now_ns']*1e-9, sample['receipt_monotonic_s']
    results = {}
    def save(name, passed, observed):
        results[name] = {'passed': bool(passed), 'observed': observed}
    def observe(guard, source, source_now=now, receipt=wall):
        return guard.observe(source, receipt_monotonic_s=receipt,
                             now_source_s=source_now, now_monotonic_s=receipt)
    baseline = observe(StreamClockGuard(0.25, 0.03), stamp)
    valid_config = validate_contract_config(config)
    gyro, flow = examples['gyro_window'], examples['valid_flow']
    gyro_baseline = check_gyro_coverage(gyro['start_s'], gyro['end_s'], gyro['stamps_s'], max_gap_s=0.02)
    angular_baseline = flow_metric_velocity(*flow['compensated_integrated_flow_rad'],
        integration_time_s=flow['dt_s'],distance_m=None,quality=flow['quality'],metric_requested=False)
    baseline.update({'gyro_window':gyro_baseline,'angular_contract':angular_baseline})
    flow_times_match = (flow.get('valid') is True and
        abs(flow['acquisition_start_ns']*1e-9-gyro['start_s'])<1e-8 and
        abs(flow['acquisition_end_ns']*1e-9-gyro['end_s'])<1e-8 and
        abs(flow['dt_s']-(gyro['end_s']-gyro['start_s']))<1e-8)
    save('baseline', baseline.get('accepted') and valid_config['status']=='PASS' and
         gyro_baseline['status']=='PASS' and angular_baseline['status']=='PASS' and flow_times_match, baseline)
    guard = StreamClockGuard(0.25, 0.03)
    observe(guard, stamp)
    duplicate = observe(guard, stamp, now+0.01, wall+0.01)
    save('duplicate_acquisition_stamp', duplicate.get('accepted') is False and guard.last_receipt_s==wall, duplicate)
    stale = observe(StreamClockGuard(0.25, 0.03), stamp, max(now,stamp)+0.5, wall+0.5)
    save('stale_source_fresh_receipt', stale.get('reason')=='acquisition_stale', stale)
    future = observe(StreamClockGuard(0.25, 0.03), now+1.0)
    save('future_source_stamp', future.get('reason')=='future_acquisition_stamp', future)
    spec = config['streams']['down_camera']
    bad = deepcopy(spec['body_from_sensor'])
    bad['rotation'] = [[-row[0], *row[1:]] for row in bad['rotation']]
    reflected = check_rigid_transform(bad, body_frame=config['body_frame'], sensor_frame=spec['frame_id'])
    save('optical_axis_reflection', reflected['status']=='FAIL', reflected)
    bad = deepcopy(spec['body_from_sensor'])
    bad['convention'] = 'sensor_from_body'
    inverse = check_rigid_transform(bad, body_frame=config['body_frame'], sensor_frame=spec['frame_id'])
    save('declared_inverse_extrinsic', inverse['status']=='FAIL', inverse)
    bad = deepcopy(spec['intrinsics'])
    bad['fx'] = 0
    intrinsics = check_camera_model(bad, axis_convention='ros_optical')
    save('zero_focal_length', intrinsics['status']=='FAIL', intrinsics)
    # Remove the entire interior while retaining real bracketing endpoints.
    stamps = [gyro['stamps_s'][0], gyro['stamps_s'][-1]]
    gap = check_gyro_coverage(gyro['start_s'], gyro['end_s'], stamps, max_gap_s=0.02)
    save('missing_gyro_window', gap['status']=='FAIL', gap)
    unknown_scale = flow_metric_velocity(*flow['compensated_integrated_flow_rad'],
        integration_time_s=flow['dt_s'], distance_m=None, quality=flow['quality'])
    save('unknown_range_scale', unknown_scale['status']=='FAIL' and not unknown_scale['metric_velocity_available'], unknown_scale)
    dependence = check_independent_fusion(config['provenance'], assume_independent=True)
    shared_imu = any('livox_imu' in item['shared_leaves'] for item in dependence['shared_leaf_dependencies'])
    save('shared_imu_independence_claim', shared_imu and not dependence['independent_fusion_ready']
         and not dependence['streaming_allowed'], dependence)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    run = parser.parse_args().run_dir.resolve()
    try:
        config = json.loads((run/'configuration/research_multisensor_contract.json').read_text())
        measured = json.loads((run/'multisensor-result.json').read_text())
        thresholds = json.loads((run/'configuration/research_multisensor_gate.json').read_text())
        cases = evaluate(config, measured)
        passed = set(cases)==set(thresholds['fault_cases']) and all(item['passed'] for item in cases.values())
        result = {'status': 'PASS' if passed else 'FAIL', 'cases': cases,
                  'scope': 'Offline contract metadata fault replay using captured source examples; no live sensor degradation, fused-accuracy or integrity claim.'}
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
        result = {'status':'FAIL', 'cases':{}, 'error':str(exc)}
    (run/'multisensor-fault-matrix.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(f"{result['status']}: R2 metadata fault matrix ({len(result['cases'])} cases)")
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
