#!/usr/bin/env python3
"""Audit a closed, unarmed link probe without opening any SQLite database."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys


def read(path):
    return json.loads(path.read_text())


def sha(path):
    value = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1048576), b''):
            value.update(block)
    return value.hexdigest()


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def gaps(values, scale=1.):
    differences = [(b-a)/scale for a, b in zip(values, values[1:])]
    return {'count': len(values), 'distinct': len(set(values)),
            'span_s': (values[-1]-values[0])/scale if values else 0.,
            'maximum_gap_s': max(differences) if differences else None,
            'median_gap_s': statistics.median(differences) if differences else None,
            'negative_gaps': sum(d < 0 for d in differences),
            'gaps_at_least_300ms': sum(d >= .3 for d in differences)}


def fcu_continuity(values, start_us, end_us, protocol):
    """Include both boundary gaps; a missing boundary never establishes PASS."""
    known = start_us is not None and end_us is not None and end_us > start_us
    selected = [v for v in values if known and start_us <= int(v['TimeUS']) <= end_us]
    arrivals = [int(v['TimeUS']) for v in selected]
    output = gaps(arrivals, 1e6)
    boundary_gaps = ([arrivals[0]-start_us, end_us-arrivals[-1]] if arrivals else [])
    all_gaps = [b-a for a, b in zip(arrivals, arrivals[1:])] + boundary_gaps
    output.update({'boot_window_start_us': start_us, 'boot_window_end_us': end_us,
                   'boot_window_span_s': (end_us-start_us)/1e6 if known else None,
                   'boundaries_observed': known,
                   'head_gap_s': boundary_gaps[0]/1e6 if boundary_gaps else None,
                   'tail_gap_s': boundary_gaps[1]/1e6 if boundary_gaps else None,
                   'maximum_gap_including_boundaries_s': max(all_gaps)/1e6 if all_gaps else None,
                   'ignored_count': sum(int(v['Ign']) != 0 for v in selected),
                   'quality_histogram': dict(Counter(str(v['Q']) for v in selected)),
                   'remote_keys': [int(v['RTimeUS']) for v in selected]})
    output['continuity_pass'] = bool(known and selected and
        output['count'] >= protocol['fcu_minimum_each_vis_messages'] and
        output['span_s'] >= protocol['fcu_minimum_boot_span_s'] and
        output['negative_gaps'] == 0 and
        output['maximum_gap_including_boundaries_s'] < protocol['fcu_maximum_gap_s_exclusive'])
    return output


def compare_keys(upstream, downstream):
    a, b = set(map(int, upstream)), set(map(int, downstream))
    return {'observed_upstream_distinct': len(a), 'observed_downstream_distinct': len(b),
            'observed_in_both': len(a & b), 'upstream_not_observed_downstream': len(a-b),
            'downstream_not_observed_upstream': len(b-a),
            'first_upstream_not_observed_downstream': sorted(a-b)[:20],
            'scope': 'Observed key sets, not an end-to-end packet loss rate. Independent ROS observers can themselves miss DDS samples; window edges can be censored.'}


def boundary_coverage(boot_frames, start, end, maximum_age_s):
    first_age = (boot_frames[0]['monotonic_ns']-start)/1e9 if boot_frames else None
    last_age = (end-boot_frames[-1]['monotonic_ns'])/1e9 if boot_frames else None
    return {'host_head_uncovered_s': first_age, 'host_tail_uncovered_s': last_age,
            'maximum_allowed_age_s': maximum_age_s,
            'sufficiently_close': bool(boot_frames and len(boot_frames) >= 2 and
                0 <= first_age <= maximum_age_s and 0 <= last_age <= maximum_age_s),
            'scope': 'Continuity applies to the FCU boot subwindow delimited by these frames, not unobserved host-window edges.'}


def load_fcu(path):
    # Append only to find this already installed package. System NumPy retains priority.
    site = Path('/home/accelerate/.local/lib/python3.10/site-packages')
    if str(site) not in sys.path:
        sys.path.append(str(site))
    from pymavlink.DFReader import DFReader_binary
    import pymavlink
    parser = DFReader_binary(str(path))
    found = {'VISP': [], 'VISV': [], 'ARM': [], 'DSF': []}
    parameters = {}
    bounds = []
    while True:
        message = parser.recv_msg()
        if message is None:
            break
        kind = message.get_type()
        value = message.to_dict()
        if 'TimeUS' in value:
            bounds.append(int(value['TimeUS']))
        if kind in found:
            found[kind].append(value)
        if kind == 'PARM':
            parameters[str(value['Name'])] = float(value['Value'])
    del parser
    return found, parameters, {'first_boot_us': min(bounds) if bounds else None,
                              'last_boot_us': max(bounds) if bounds else None,
                              'pymavlink_package': pymavlink.__file__}


def analyze(run, protocol_path):
    protocol = read(protocol_path)
    observer_path = run/'ros-observer'
    wire_path = run/'wire'
    binary_paths = list((run/'sitl_runtime/logs').glob('*.BIN'))
    if len(binary_paths) != 1:
        raise ValueError('Exactly one original FCU BIN is required')
    binary = binary_paths[0]
    inputs = [protocol_path, observer_path/'events.jsonl', observer_path/'summary.json',
              observer_path/'graph.before.json', observer_path/'graph.after.json',
              wire_path/'traffic.jsonl', wire_path/'summary.json', run/'run.json', binary]
    before = {str(p): {'bytes': p.stat().st_size, 'sha256': sha(p)} for p in inputs}
    events, wire = records(observer_path/'events.jsonl'), records(wire_path/'traffic.jsonl')
    observed, proxy, case = read(observer_path/'summary.json'), read(wire_path/'summary.json'), read(run/'run.json')
    start = int(observed['started_monotonic_ns']) + int(protocol['warmup_wall_s']*1e9)
    end = int(observed['finished_monotonic_ns'])
    steady = [v for v in events if start <= int(v['monotonic_ns']) <= end]
    frames = [v for v in wire if v.get('kind') == 'frame']
    steady_frames = [v for v in frames if start <= int(v['monotonic_ns']) <= end]
    outgoing = [v for v in steady_frames if v['direction'] == 'to_fcu' and v['msgid'] == 331]
    incoming_boot = [v for v in steady_frames if v['direction'] == 'from_fcu' and
                     v['msgid'] == 2 and 'time_boot_ms' in v]
    first_boot = int(incoming_boot[0]['time_boot_ms'])*1000 if incoming_boot else None
    last_boot = int(incoming_boot[-1]['time_boot_ms'])*1000 if incoming_boot else None
    boundary = boundary_coverage(incoming_boot, start, end, protocol['maximum_boundary_wall_age_s'])
    fcu, params, fcu_meta = load_fcu(binary)
    quality_min = params.get('VISO_QUAL_MIN')
    continuity = {kind: fcu_continuity(fcu[kind], first_boot, last_boot, protocol)
                  for kind in ('VISP', 'VISV')}
    for value in continuity.values():
        value['boundary_censored'] = not boundary['sufficiently_close']
        value['continuity_pass'] = value['continuity_pass'] and boundary['sufficiently_close']
    by_kind = {name: [v for v in steady if v['kind'] == name] for name in
               ('source_published', 'source_received', 'adapter_received',
                'adapter_status', 'mavlink_sink', 'mavlink_source', 'fcu_state', 'clock')}
    adapter_keys = [int(v['stamp_ns'])//1000 for v in by_kind['adapter_received']]
    sink_keys = [int(v['time_usec']) for v in by_kind['mavlink_sink'] if v.get('msgid') == 331]
    wire_keys = [int(v['time_usec']) for v in outgoing]
    wire_all_keys = [int(v['time_usec']) for v in frames if v['direction'] == 'to_fcu' and v['msgid'] == 331]
    source_stamps = [int(v['stamp_ns']) for v in by_kind['source_received']]
    fcu_p_keys, fcu_v_keys = (continuity[k]['remote_keys'] for k in ('VISP', 'VISV'))
    commands = [v for v in frames if v['direction'] == 'to_fcu' and v['msgid'] in (75, 76)]
    forbidden_commands = [v for v in commands if v.get('command') not in protocol['allowed_command_long_int_ids']]
    control_ids = {11, 23, 69, 70, 82, 84, 86, 93, 139, 140, 161, 375}
    forbidden_control = [v for v in frames if v['direction'] == 'to_fcu' and v['msgid'] in control_ids]
    arm_records = [v for v in fcu['ARM'] if int(v.get('ArmState', 0)) != 0]
    armed_heartbeats = [v for v in frames if v['direction'] == 'from_fcu' and v['msgid'] == 0 and
                        int(v.get('base_mode', 0)) & 128]
    source_published = [v for v in events if v['kind'] == 'source_published']
    checks = {
        'observer_completed': observed['status'] == 'PASS',
        'observer_saw_connected_and_disarmed': bool(observed.get('connected_disarmed_seen')),
        'observer_never_saw_armed': not observed.get('armed_seen', True),
        'wire_proxy_completed': proxy['status'] == 'PASS',
        'wire_all_received_bytes_forwarded_unchanged': bool(proxy.get('forwarding_integrity_pass')),
        'wire_all_mavlink_frames_decoded': bool(proxy.get('all_mavlink_frames_decoded')),
        'wire_all_observed_bytes_classified': bool(proxy.get('all_observed_bytes_classified')),
        'wire_guard_enabled_and_not_triggered': bool(proxy.get('guard_unarmed')) and not proxy.get('guard_blocking_occurred', True),
        'wire_no_autonomous_or_reencoded_messages': proxy.get('autonomous_mavlink_messages') == 0 and proxy.get('wire_bytes_reencoded') is False,
        'wire_no_armed_fcu_heartbeat': not armed_heartbeats,
        'wire_no_control_or_unapproved_commands': not forbidden_commands and not forbidden_control,
        'fcu_log_no_arm': not arm_records,
        'fcu_log_disarmed_enabled': params.get('LOG_DISARMED') == protocol['required_log_disarmed'],
        'fcu_viso_type_matches': params.get('VISO_TYPE') == protocol['expected_viso_type'],
        'fcu_quality_min_recorded': quality_min is not None,
        'observation_window_long_enough': (end-start)/1e9 >= protocol['duration_wall_s']-protocol['warmup_wall_s']-1.,
        'source_observed': len(source_stamps) >= 100,
        'source_stamps_strictly_advance': bool(source_stamps) and all(b > a for a, b in zip(source_stamps, source_stamps[1:])),
        'adapter_observed': len(adapter_keys) >= 100,
        'adapter_stamps_strictly_advance': bool(adapter_keys) and all(b > a for a, b in zip(adapter_keys, adapter_keys[1:])),
        'internal_uas_sink_observed': len(sink_keys) >= 100,
        'wire_odometry_observed': len(wire_keys) >= 100,
        'wire_odometry_stamps_strictly_advance': bool(wire_keys) and all(b > a for a, b in zip(wire_keys, wire_keys[1:])),
        'wire_odometry_frames_valid': bool(outgoing) and all(
            v['frame_id'] == protocol['fcu_required_frame_id'] and
            v['child_frame_id'] == protocol['fcu_required_child_frame_id'] for v in outgoing),
        'fcu_system_time_boundaries_observed': len(incoming_boot) >= 2 and last_boot > first_boot,
        'fcu_system_time_boundaries_not_stale': boundary['sufficiently_close'],
        'fcu_visp_continuity': continuity['VISP']['continuity_pass'],
        'fcu_visv_continuity': continuity['VISV']['continuity_pass'],
        'fcu_all_logged_observations_consumed': all(c['count'] and not c['ignored_count'] for c in continuity.values()),
        'fcu_quality_satisfies_declared_minimum': quality_min is not None and all(
            c['count'] and all(float(q) >= quality_min for q in c['quality_histogram']) for c in continuity.values()),
        'fcu_visp_visv_keys_match': bool(fcu_p_keys) and fcu_p_keys == fcu_v_keys,
        'fcu_received_keys_present_on_observed_wire': bool(fcu_p_keys) and not (set(fcu_p_keys)-set(wire_all_keys)),
        'observer_source_mode_respected': bool(source_published) if case['source'] == 'synthetic' else not source_published,
        'mission_and_flight_execution_disabled': not case['mission_nodes_started'] and not case['armed_trial'],
    }
    graph_text = '\n'.join((observer_path/name).read_text() for name in ('graph.before.json', 'graph.after.json'))
    forbidden_nodes = ['impact_ap_research_mission', 'impact_ap_execution', 'xq_p13_flight_controller']
    checks['forbidden_flight_nodes_absent_from_snapshots'] = not any(name in graph_text for name in forbidden_nodes)
    after = {str(p): {'bytes': p.stat().st_size, 'sha256': sha(p)} for p in inputs}
    checks['original_inputs_unchanged'] = before == after
    report = {'status': 'PASS' if all(checks.values()) else 'FAIL',
              'stage': protocol['stage'], 'case': case, 'checks': checks,
              'failed_checks': [key for key, value in checks.items() if not value],
              'measurement_window': {'start_monotonic_ns': start, 'end_monotonic_ns': end,
                  'wall_span_s': (end-start)/1e9, 'warmup_wall_s': protocol['warmup_wall_s'],
                  'fcu_boundaries': 'First/last FCU SYSTEM_TIME messages observed inside the frozen steady wall window; FCU gaps include both boot-time boundaries.',
                  'boundary_coverage': boundary},
              'observer_counts_steady': {k: len(v) for k, v in by_kind.items()},
              'source_acquisition': gaps(source_stamps, 1e9),
              'adapter_observer_wall_interarrival': gaps([v['monotonic_ns'] for v in by_kind['adapter_received']], 1e9),
              'wire_wall_interarrival': gaps([v['monotonic_ns'] for v in outgoing], 1e9),
              'fcu_continuity': continuity,
              'host_wall_clock_backsteps': [
                  {'previous_monotonic_ns': a['monotonic_ns'], 'next_monotonic_ns': b['monotonic_ns'],
                   'stamp_delta_s': (b['stamp_ns']-a['stamp_ns'])/1e9,
                   'wall_delta_s': (b['wall_time_ns']-a['wall_time_ns'])/1e9,
                   'monotonic_delta_s': (b['monotonic_ns']-a['monotonic_ns'])/1e9}
                  for a, b in zip(by_kind['adapter_received'], by_kind['adapter_received'][1:])
                  if b['wall_time_ns'] < a['wall_time_ns']],
              'fcu_full_log': {kind: {**gaps([int(v['TimeUS']) for v in fcu[kind]], 1e6),
                  'ignored_count': sum(int(v['Ign']) != 0 for v in fcu[kind])} for kind in ('VISP', 'VISV')},
              'observed_key_comparisons': {
                  'adapter_observer_to_sink_observer': compare_keys(adapter_keys, sink_keys),
                  'sink_observer_to_wire': compare_keys(sink_keys, wire_keys),
                  'adapter_observer_to_wire': compare_keys(adapter_keys, wire_keys),
                  'wire_to_fcu_visp': compare_keys(wire_keys, fcu_p_keys),
                  'wire_to_fcu_visv': compare_keys(wire_keys, fcu_v_keys)},
              'fcu_parameters': {k: params.get(k) for k in ('LOG_DISARMED', 'VISO_TYPE', 'VISO_QUAL_MIN', 'ARMING_CHECK')},
              'fcu_log': {**fcu_meta, 'file': str(binary), 'dsf_count': len(fcu['DSF']),
                         'dsf_dropped_write_counts': sorted({v['Dp'] for v in fcu['DSF']})},
              'wire_command_counts': dict(Counter(str(v.get('command')) for v in commands)),
              'forbidden_commands': forbidden_commands, 'forbidden_control': forbidden_control,
              'original_files_before': before, 'original_files_after': after,
              'actual_flow_state_updates': 0, 'flight_regression_repaired': False,
              'scope': protocol['scope'],
              'limits': ['An observer receives its own DDS branch; it does not prove another subscriber callback executed.',
                         'This verifies communication and the VisualOdom backend, not EKF fusion, navigation health, flight safety or algorithm superiority.',
                         'No exact end-to-end latency or packet loss rate is inferred from unsynchronized clocks or unequal observation windows.']}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, required=True)
    args = parser.parse_args()
    target = args.run_dir/'link-probe-result.json'
    if target.exists():
        raise SystemExit('Refusing to overwrite a prior diagnostic')
    try:
        result = analyze(args.run_dir, args.protocol)
    except Exception as exc:
        result = {'status': 'FAIL', 'error': f'{type(exc).__name__}: {exc}',
                  'flight_regression_repaired': False, 'actual_flow_state_updates': 0}
    with target.open('x') as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
        handle.write('\n')
    print(json.dumps({'status': result['status'], 'failed_checks': result.get('failed_checks'),
                      'error': result.get('error'), 'output': str(target)}))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
