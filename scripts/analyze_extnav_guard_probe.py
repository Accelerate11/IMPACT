#!/usr/bin/env python3
"""Analyze a closed unarmed B3 guard probe; never open a SQLite database.

The helper's FCU BIN reader is reused, not its continuous-stream acceptance
gates. A latched clock fault is an expected fail-closed response. An interrupted
source-pause control is explicitly incomplete, not an isolated recovery PASS.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys


STAGE = 'R2B3B3_CLOCK_GUARD_AND_SAMPLED_FCU_READINESS'
CONTROL_IDS = {11, 23, 69, 70, 82, 84, 86, 93, 139, 140, 161, 375}
LIMITS = [
    'SYS_STATUS bit 128 is sampled VisualOdom backend receipt health with PLND disabled; not a per-frame ACK or EKF fusion proof.',
    'Ready/ever_ready are read-only observations, never flight or arming permissions. No mission integration is tested.',
    'Independent DDS observer branches can miss callbacks. FCU matching uses ordered wire RTimeUS keys, not subtraction of uncalibrated clocks.',
    'A clock fault can preempt the source-pause control. Such a control is not a recovery PASS.',
    'Original B1 INCOMPLETE and B2 FAIL results remain unchanged. Actual optical-flow state updates remain zero.',
]


def read(path):
    return json.loads(path.read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def records(path):
    with path.open() as handle:
        return [json.loads(line) for line in handle if line.strip()]


def sha(path):
    value = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1048576), b''):
            value.update(block)
    return value.hexdigest()


def fingerprint(paths):
    result = {}
    for path in paths:
        if path.is_symlink() or not path.is_file():
            raise ValueError('Regular nonsymlink input required: ' + str(path))
        result[str(path)] = {'bytes': path.stat().st_size, 'sha256': sha(path)}
    return result


def load_helper(path):
    spec = importlib.util.spec_from_file_location('frozen_guard_probe_bin_helper', path)
    module = importlib.util.module_from_spec(spec)
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    return module


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def increasing(values):
    return bool(values) and all(type(v) is int and v > 0 for v in values) and all(b > a for a, b in zip(values, values[1:]))


def runtime_identity_checks(identity, binding, dependencies, current):
    checks = {'installed_identity_exact_guard_and_node_set': set(identity) == {
        'p4_external_nav_node.py', 'extnav_clock_guard.py'}}
    for name in ('p4_external_nav_node.py', 'extnav_clock_guard.py'):
        row = identity.get(name, {})
        source = binding.get('src/xq_autonomy/xq_autonomy/' + name, {})
        path = row.get('installed_path')
        checks['installed_frozen_source_identity:' + name] = (
            row.get('matches') is True and bool(source.get('sha256'))
            and row.get('frozen_source_sha256') == source.get('sha256')
            and row.get('installed_sha256') == source.get('sha256')
            and dependencies.get(path) == source and current.get(path) == source)
    checks['all_recorded_runtime_dependencies_unchanged'] = bool(dependencies) and all(
        current.get(path) == row for path, row in dependencies.items())
    return checks


def wire_integrity(proxy):
    flows = proxy.get('flows', {})
    return (proxy.get('forwarding_integrity_pass') is True
        and proxy.get('all_mavlink_frames_decoded') is True
        and proxy.get('all_observed_bytes_classified') is True
        and set(flows) == {'to_fcu', 'from_fcu'} and all(
            row.get('forwarded_stream_identical') is True
            and type(row.get('received_bytes')) is int and row['received_bytes'] > 0
            and row.get('received_bytes') == row.get('sent_bytes')
            and isinstance(row.get('received_sha256'), str) and len(row['received_sha256']) == 64
            and row.get('received_sha256') == row.get('sent_sha256')
            and row.get('all_observed_bytes_classified') is True
            and row.get('mavlink_decode_complete') is True
            for row in flows.values()))


def parameter_value(row):
    if not isinstance(row, dict) or row.get('success') is not True:
        raise ValueError('Unsuccessful parameter response')
    if row.get('ros_type') == 2 and type(row.get('integer_value')) is int:
        return row['integer_value']
    if row.get('ros_type') == 3 and number(row.get('double_value')):
        return row['double_value']
    raise ValueError('Unsupported or nonfinite parameter response')


def parameter_name(value):
    if isinstance(value, dict) and set(value) == {'hex'}:
        value = bytes.fromhex(value['hex']).decode('ascii')
    if not isinstance(value, str):
        raise ValueError('PARAM_VALUE param_id missing')
    return value.rstrip('\0')


def parameter_checks(proof, required, wire, params, protocol_sha):
    checks = {
        'parameter_proof_complete_read_only': proof.get('status') == 'PASS' and proof.get('read_only') is True
            and proof.get('parameter_writes') == 0 and proof.get('flight_commands') == 0,
        'parameter_proof_method_and_config': proof.get('schema_version') == 1
            and proof.get('method') == 'mavros_ros2_GetParameters'
            and proof.get('service') == '/uav1/mavros/param/get_parameters'
            and proof.get('config_sha256') == protocol_sha,
        'parameter_proof_configured_target_1_1': type(proof.get('target_system_id')) is int
            and proof['target_system_id'] == 1 and type(proof.get('target_component_id')) is int
            and proof['target_component_id'] == 1,
    }
    values, errors = {}, []
    for name, expected in required.items():
        try:
            values[name] = parameter_value(proof.get('parameters', {}).get(name))
            checks['proof_parameter:' + name] = values[name] == expected
        except ValueError as exc:
            errors.append(name + ': ' + str(exc))
            checks['proof_parameter:' + name] = False
    observations = {name: [] for name in required}
    invalid = []
    for frame in wire:
        if frame.get('kind') != 'frame' or frame.get('direction') != 'from_fcu' or frame.get('msgid') != 22:
            continue
        try:
            fields = frame['fields']
            name = parameter_name(fields['param_id'])
            if name not in required:
                continue
            value = fields['param_value']
            valid = (frame.get('sysid') == 1 and frame.get('compid') == 1 and number(value))
            observations[name].append({'monotonic_ns': frame['monotonic_ns'], 'seq': frame['seq'],
                'sysid': frame['sysid'], 'compid': frame['compid'], 'value': value, 'valid': valid})
        except (KeyError, ValueError, TypeError) as exc:
            invalid.append(str(exc))
    for name, expected in required.items():
        checks['wire_parameter:' + name] = bool(observations[name]) and all(
            row['valid'] and row['value'] == expected for row in observations[name])
    checks['wire_parameter_payloads_available_and_valid'] = not invalid
    for name, expected in {'VISO_TYPE': 2, 'PLND_ENABLED': 0, 'LOG_DISARMED': 1}.items():
        checks['fcu_bin_parameter:' + name] = params.get(name) == expected
    return checks, {'proof_values': values, 'proof_errors': errors,
        'wire_parameter_observations': observations, 'wire_payload_errors': invalid,
        'target_scope': 'Configured query target, corroborated independently by every required named wire PARAM_VALUE from system/component 1/1.'}


def wire_health(frames, mask):
    samples, errors = [], []
    keys = ('onboard_control_sensors_present', 'onboard_control_sensors_enabled', 'onboard_control_sensors_health')
    for frame in frames:
        if frame.get('direction') != 'from_fcu' or frame.get('msgid') != 1:
            continue
        try:
            if frame.get('sysid') != 1 or frame.get('compid') != 1:
                raise ValueError('SYS_STATUS source is not 1/1')
            fields = frame['fields']
            values = [fields[key] for key in keys]
            if any(type(value) is not int or not 0 <= value < 2**32 for value in values):
                raise ValueError('Invalid SYS_STATUS mask')
            if any(key in frame and frame[key] != value for key, value in zip(keys, values)):
                raise ValueError('SYS_STATUS top-level/raw fields disagree')
            samples.append({'monotonic_ns': frame['monotonic_ns'], 'wall_time_ns': frame['wall_time_ns'],
                'sysid': frame['sysid'], 'compid': frame['compid'], 'seq': frame['seq'],
                'present_mask': values[0], 'enabled_mask': values[1], 'health_mask': values[2],
                'vision_present': bool(values[0] & mask), 'vision_enabled': bool(values[1] & mask),
                'raw_vision_health_bit': bool(values[2] & mask),
                'combined_backend_healthy': all(value & mask == mask for value in values)})
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(str(exc))
    transitions = [dict(monotonic_ns=b['monotonic_ns'], previous=a['combined_backend_healthy'],
                        current=b['combined_backend_healthy']) for a, b in zip(samples, samples[1:])
                   if a['combined_backend_healthy'] != b['combined_backend_healthy']]
    return {'samples': samples, 'errors': errors, 'healthy_raw_bit_count': sum(v['raw_vision_health_bit'] for v in samples),
        'unhealthy_raw_bit_count': sum(not v['raw_vision_health_bit'] for v in samples),
        'combined_healthy_count': sum(v['combined_backend_healthy'] for v in samples),
        'combined_unhealthy_count': sum(not v['combined_backend_healthy'] for v in samples), 'transitions': transitions}


def monitor_health(events, mask):
    samples, errors = [], []
    for event in events:
        if event.get('event') != 'mavlink' or event.get('data', {}).get('raw', {}).get('msgid') != 1:
            continue
        try:
            raw, result = event['data']['raw'], event['data']['result']
            if (raw['sysid'], raw['compid'], raw['framing_status']) != (1, 1, 1):
                raise ValueError('Monitor SYS_STATUS identity/framing invalid')
            length = raw['len']
            if type(length) is not int or not 1 <= length <= 31 or raw['magic'] not in (253, 254):
                raise ValueError('Invalid SYS_STATUS frame length/magic')
            if raw['magic'] == 254 and length != 31:
                raise ValueError('MAVLink1 SYS_STATUS length must be 31')
            data = b''.join(struct.pack('<Q', value) for value in raw['payload64'])
            if len(data) < length:
                raise ValueError('SYS_STATUS payload64 too short')
            present, enabled, health = struct.unpack('<III', data[:length].ljust(12, b'\0')[:12])
            expected = {'present': present, 'enabled': enabled, 'health_mask': health}
            healthy = all(value & mask == mask for value in expected.values())
            decoded = result.get('decoded', {})
            if result.get('valid') is not True or result.get('healthy') is not healthy or any(decoded.get(k) != v for k, v in expected.items()):
                raise ValueError('Monitor result disagrees with independently decoded raw masks')
            samples.append({'monotonic_ns': event['monotonic_ns'], **expected,
                            'combined_backend_healthy': healthy})
        except (KeyError, TypeError, ValueError, struct.error) as exc:
            errors.append(str(exc))
    return {'samples': samples, 'errors': errors}


def local_rows(events, monitor=False):
    result, errors = [], []
    for event in events:
        if event.get('event' if monitor else 'kind') != ('local_status' if monitor else 'adapter_status'):
            continue
        try:
            if monitor:
                value = json.loads(event['data']['raw'])
            else:
                message = event['message']
                value = message.get('parsed')
                if value is None and isinstance(message.get('data'), str):
                    value = json.loads(message['data'])
            if not isinstance(value, dict):
                raise ValueError('Local status is not an object')
            result.append({'monotonic_ns': event['monotonic_ns'], 'status': value})
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(str(exc))
    return result, errors


def fault_analysis(statuses, monitor_locals, readiness, health, outgoing, adapter_events):
    fault_rows = [row for row in statuses if row['status'].get('clock_fault_latched') is True]
    if not fault_rows:
        if any(row['status'].get('clock_fault_latched') is True for row in monitor_locals):
            return {'fault_primary_observer_evidence_present': False}, {
                'status': 'INCOMPLETE', 'scope': 'Monitor observed a node fault, but the independent observer lacks its first-fault payload.'}, None
        return {}, {'status': 'NOT_OBSERVED', 'scope': 'No natural node clock-fault rejection is claimed.'}, None
    first = fault_rows[0]
    fault = first['status']['clock_guard']['first_fault']
    fault_ns = fault['inputs']['monotonic_ns']
    last_stamp = fault['last_accepted_transport_stamp_ns']
    candidate = fault['inputs']['transport_stamp_ns']
    if not all(type(v) is int and v > 0 for v in (fault_ns, last_stamp, candidate)):
        raise ValueError('Invalid node first-fault timing evidence')
    suffix = [row for row in statuses if row['monotonic_ns'] >= first['monotonic_ns']]
    count = first['status']['output_count']
    seen = [row for row in monitor_locals if row['status'].get('clock_fault_latched') is True]
    seen_ns = seen[0]['monotonic_ns'] if seen else None
    ready_suffix = [row for row in readiness if seen_ns is not None and row['monotonic_ns'] >= seen_ns]
    post_health = [row for row in health if row['monotonic_ns'] >= fault_ns]
    keys = [row['time_usec'] for row in outgoing]
    checks = {
        'fault_suffix_observed': len(suffix) > 1,
        'first_fault_payload_permanently_unchanged': all(row['status'].get('clock_guard', {}).get('first_fault') == fault for row in suffix),
        'node_output_count_does_not_increase_after_fault': all(row['status'].get('output_count') == count for row in suffix),
        'node_clock_fault_stays_latched': all(row['status'].get('clock_fault_latched') is True and
            row['status'].get('clock_guard', {}).get('fault_latched') is True for row in suffix),
        'node_healthy_and_clock_ready_stay_false': all(row['status'].get('healthy') is False and
            row['status'].get('local_clock_ready') is False for row in suffix),
        'wire_never_exceeds_last_accepted_transport_stamp': bool(keys) and all(key <= last_stamp//1000 for key in keys),
        'rejected_candidate_not_in_adapter_observer_headers': candidate not in [row['stamp_ns'] for row in adapter_events],
        'rejected_candidate_usec_not_on_wire': candidate//1000 not in keys,
        'monitor_observes_clock_fault': seen_ns is not None,
        'monitor_ready_stays_false_after_seeing_fault': bool(ready_suffix) and all(row['data'].get('ready') is False and
            row['data'].get('clock_fault_latched') is True for row in ready_suffix),
        'final_sampled_fcu_health_is_false_after_fault': bool(post_health) and post_health[-1]['combined_backend_healthy'] is False,
    }
    return checks, {'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks,
        'node_first_fault_monotonic_ns': fault_ns, 'observer_first_fault_receipt_monotonic_ns': first['monotonic_ns'],
        'monitor_first_fault_receipt_monotonic_ns': seen_ns, 'first_fault': fault,
        'fault_suffix_status_count': len(suffix), 'fixed_node_output_count': count,
        'monitor_ready_suffix_count': len(ready_suffix), 'last_accepted_transport_stamp_ns': last_stamp,
        'rejected_candidate_transport_stamp_ns': candidate,
        'scope': 'Node clock-fault and observer/monitor receipts share host monotonic domain; wire is bounded by accepted transport stamps, allowing already-in-flight accepted frames.'}, fault_ns


def pause_analysis(case, protocol, events, statuses, readiness, health, fault_ns):
    config = next(row for row in protocol['cases'] if (row['source'], row['load']) == (case['source'], case['load']))
    if not config['source_pause_duration_s']:
        return {'status': 'NOT_APPLICABLE', 'scope': 'No source-pause control was declared for this case.'}
    starts = [row for row in events if row.get('kind') == 'source_pause_started']
    ends = [row for row in events if row.get('kind') == 'source_pause_ended']
    if len(starts) != 1 or len(ends) != 1:
        return {'status': 'PREEMPTED' if fault_ns is not None else 'INCOMPLETE',
                'reason': 'The complete declared pause start/end pair is absent', 'pause_start_count': len(starts), 'pause_end_count': len(ends)}
    start, end = starts[0]['monotonic_ns'], ends[0]['monotonic_ns']
    publications = [row for row in events if row.get('kind') == 'source_published']
    resumed = [row for row in publications if row['monotonic_ns'] >= end]
    if not resumed:
        return {'status': 'PREEMPTED' if fault_ns is not None else 'INCOMPLETE',
                'reason': 'No new synthetic source publication after the pause', 'start_monotonic_ns': start, 'end_monotonic_ns': end}
    after = [row for row in statuses if row['monotonic_ns'] >= end and
             row['status'].get('clock_guard', {}).get('last_accepted_source_stamp_ns', 0) >= resumed[0]['stamp_ns']]
    anchors = [row['status']['clock_guard'].get('continuous_valid_start_monotonic_ns') for row in after]
    anchors = [value for value in anchors if type(value) is int and value >= start]
    recovery_start = min(anchors) if anchors else None
    warmup = protocol['clock_guard']['warmup_ns']
    complete_after = (recovery_start + warmup) if recovery_start is not None else (resumed[0]['monotonic_ns'] + warmup)
    if fault_ns is not None and fault_ns <= complete_after:
        return {'status': 'PREEMPTED', 'reason': 'Natural latched clock fault prevents an isolated pause/recovery test',
            'clock_fault_monotonic_ns': fault_ns, 'start_monotonic_ns': start, 'end_monotonic_ns': end,
            'first_resumed_source_monotonic_ns': resumed[0]['monotonic_ns'],
            'recovery_is_not_claimed': True}
    stale = [row for row in statuses if start <= row['monotonic_ns'] <= end and
             row['status'].get('clock_guard', {}).get('source_age_monotonic_ns', -1) is not None and
             row['status']['clock_guard'].get('source_age_monotonic_ns', -1) > protocol['clock_guard']['max_source_age_ns']]
    unhealthy = [row for row in health if start <= row['monotonic_ns'] <= end and not row['combined_backend_healthy']]
    rewarm = [row for row in after if recovery_start is not None and
              row['status']['clock_guard'].get('last_evaluated_monotonic_ns', 0)-recovery_start < warmup]
    ready_after = [row for row in after if row['status'].get('local_clock_ready') is True and
                   row['status']['clock_guard'].get('continuous_valid_start_monotonic_ns') == recovery_start]
    monitor_rewarm = [row for row in readiness if recovery_start is not None and end <= row['monotonic_ns'] < complete_after]
    checks = {
        'pause_events_match_declared_monotonic_schedule': starts[0].get('pause_duration_ns') == int(config['source_pause_duration_s']*1e9)
            and starts[0].get('elapsed_since_first_source_ns', -1) >= int(config['source_pause_after_s']*1e9)
            and ends[0].get('elapsed_since_first_source_ns', -1) >= int((config['source_pause_after_s']+config['source_pause_duration_s'])*1e9),
        'no_source_publication_inside_pause': not any(start < row['monotonic_ns'] < end for row in publications),
        'local_receipt_freshness_expires_during_pause': bool(stale) and all(row['status'].get('local_source_ready') is False
            and row['status'].get('local_clock_ready') is False and row['status'].get('healthy') is False for row in stale),
        'fcu_sampled_health_false_during_pause': bool(unhealthy),
        'recovery_restarts_continuous_warmup': recovery_start is not None and bool(rewarm),
        'local_not_ready_during_new_warmup': bool(rewarm) and all(row['status'].get('local_clock_ready') is False for row in rewarm),
        'local_ready_recovers_after_full_warmup': bool(ready_after) and all(
            row['status']['clock_guard']['last_evaluated_monotonic_ns']-recovery_start >= warmup for row in ready_after),
        'monitor_not_ready_during_new_warmup': bool(monitor_rewarm) and all(row['data'].get('ready') is False for row in monitor_rewarm),
    }
    return {'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks,
        'start_monotonic_ns': start, 'end_monotonic_ns': end, 'observed_pause_span_s': (end-start)/1e9,
        'recovery_continuous_start_monotonic_ns': recovery_start, 'required_warmup_ns': warmup,
        'stale_local_status_count': len(stale), 'sampled_unhealthy_count': len(unhealthy),
        'scope': 'Local receive freshness and sampled FCU health, not source-to-FCU acquisition age or exact blackout duration.'}


def analyze(run, protocol_path):
    run, protocol_path = Path(run).absolute(), Path(protocol_path).absolute()
    report = {'schema_version': 1, 'status': 'FAIL', 'stage': STAGE, 'checks': {}, 'errors': [],
              'actual_flow_state_updates': 0, 'flight_regression_repaired': False, 'closed_loop_validation': False,
              'hardware_readiness': 'NOT_READY', 'limits': LIMITS}
    inputs, before = [], None
    try:
        binary_paths = sorted((run / 'sitl_runtime/logs').glob('*.BIN'))
        if len(binary_paths) != 1:
            raise ValueError('Exactly one new closed FCU BIN is required')
        binary = binary_paths[0]
        helper_path = run / 'sources/scripts/analyze_extnav_link_probe.py'
        dependencies = read(run / 'runtime-dependencies.json')
        identity = read(run / 'installed-source-identity.json')
        inputs = [protocol_path, Path(__file__).absolute(), binary, run / 'run.json', run / 'source-binding.json',
            run / 'runtime-dependencies.json', run / 'installed-source-identity.json',
            run / 'shutdown-result.json', run / 'configuration/fcu-parameter-proof.json',
            *(run / 'ros-observer' / name for name in ('events.jsonl', 'summary.json', 'metadata.json', 'graph.before.json', 'graph.after.json')),
            *(run / 'readiness' / name for name in ('events.jsonl', 'summary.json', 'metadata.json')),
            run / 'wire/traffic.jsonl', run / 'wire/summary.json']
        source_paths = sorted((run / 'sources').rglob('*'))
        if any(path.is_symlink() for path in source_paths):
            raise ValueError('Frozen source symlinks are not accepted')
        inputs += [path for path in source_paths if path.is_file()]
        inputs += [Path(path) for path in dependencies]
        inputs = sorted(set(inputs))
        before = fingerprint(inputs)
        binding = read(run / 'source-binding.json')
        expected = {str(run / 'sources' / name): row for name, row in binding.items()}
        actual = {str(path): before[str(path)] for path in inputs if run / 'sources' in path.parents}
        checks = report['checks']
        checks['frozen_source_binding_set_and_sha256'] = bool(expected) and expected == actual
        checks.update(runtime_identity_checks(identity, binding, dependencies, before))
        report['installed_source_identity'] = identity
        report['runtime_dependencies'] = dependencies
        protocol = read(protocol_path)
        if protocol.get('stage') != STAGE or protocol.get('no_arm') is not True or protocol.get('closed_loop_validation') is not False:
            raise ValueError('Unexpected B3 frozen protocol/scope')
        checks['protocol_matches_frozen_configuration'] = sha(protocol_path) == sha(run / 'sources/config/research_extnav_clock_guard.json')
        helper = load_helper(helper_path)
        fcu, params, fcu_meta = helper.load_fcu(binary)
        events, wire, monitored = records(run / 'ros-observer/events.jsonl'), records(run / 'wire/traffic.jsonl'), records(run / 'readiness/events.jsonl')
        observer, proxy, monitor = read(run / 'ros-observer/summary.json'), read(run / 'wire/summary.json'), read(run / 'readiness/summary.json')
        metadata, proof = read(run / 'readiness/metadata.json'), read(run / 'configuration/fcu-parameter-proof.json')
        case, shutdown = read(run / 'run.json'), read(run / 'shutdown-result.json')
        report['case'] = case
        checks['closed_owned_processes_recorded'] = bool(shutdown.get('processes')) and all(
            row.get('group_still_exists') is False for row in shutdown['processes'])
        checks['unarmed_case_no_mission_or_fusion'] = case.get('armed_trial') is False and case.get('mission_nodes_started') is False and case.get('actual_flow_state_updates') == 0
        checks['observer_completed_safely'] = observer.get('status') == 'PASS' and observer.get('armed_seen') is False and observer.get('connected_disarmed_seen') is True
        checks['monitor_completed_safely'] = monitor.get('status') == 'PASS' and monitor.get('stop_reason') == 'duration_complete' and monitor.get('armed_seen') is False and not monitor.get('errors')
        checks['monitor_uses_exact_frozen_proof_and_config'] = metadata.get('config_sha256') == sha(protocol_path) and metadata.get('parameter_proof_sha256') == sha(run / 'configuration/fcu-parameter-proof.json') and metadata.get('parameter_proof') == proof and metadata.get('config') == protocol
        checks['monitor_script_matches_frozen_source'] = metadata.get('script_sha256') == sha(run / 'sources/scripts/monitor_extnav_readiness.py')
        required = protocol['live_readiness']['required_parameters']
        parameter_gates, parameter_report = parameter_checks(proof, required, wire, params, sha(protocol_path))
        checks.update(parameter_gates)
        report['parameter_evidence'] = parameter_report
        frames = [row for row in wire if row.get('kind') == 'frame']
        outgoing = [row for row in frames if row.get('direction') == 'to_fcu' and row.get('msgid') == 331]
        keys = [row['time_usec'] for row in outgoing]
        commands = [row for row in frames if row.get('direction') == 'to_fcu' and row.get('msgid') in (75, 76)]
        forbidden_commands = [row for row in commands if row.get('command') not in protocol['allowed_command_long_int_ids']]
        forbidden_control = [row for row in frames if row.get('direction') == 'to_fcu' and
            (row.get('msgid') in CONTROL_IDS or str(row.get('type', '')).startswith('SET_') or row.get('type') in {'PARAM_EXT_SET', 'MISSION_SET_CURRENT', 'MISSION_ITEM', 'MISSION_ITEM_INT', 'MISSION_WRITE_PARTIAL_LIST', 'MISSION_COUNT', 'MISSION_CLEAR_ALL'})]
        checks['wire_proxy_completed'] = proxy.get('status') == 'PASS'
        checks['wire_bytes_identical_and_all_frames_classified'] = wire_integrity(proxy)
        checks['wire_guard_enabled_without_blocked_flight_attempt'] = proxy.get('guard_unarmed') is True and proxy.get('guard_blocking_occurred') is False
        checks['wire_no_autonomous_or_reencoded_messages'] = proxy.get('autonomous_mavlink_messages') == 0 and proxy.get('wire_bytes_reencoded') is False
        checks['wire_no_control_or_unapproved_commands'] = not forbidden_commands and not forbidden_control
        checks['wire_never_observed_armed_heartbeat'] = not any(row.get('direction') == 'from_fcu' and row.get('msgid') == 0 and int(row.get('base_mode', 0)) & 128 for row in frames)
        checks['fcu_bin_never_armed'] = not any(int(row.get('ArmState', 0)) for row in fcu['ARM'])
        checks['wire_odometry_present_and_strictly_increasing'] = increasing(keys)
        checks['wire_odometry_frames_match_protocol'] = bool(outgoing) and all(row.get('frame_id') == protocol['fcu_required_frame_id'] and row.get('child_frame_id') == protocol['fcu_required_child_frame_id'] for row in outgoing)
        pkeys, vkeys = ([int(row['RTimeUS']) for row in fcu[kind]] for kind in ('VISP', 'VISV'))
        checks['every_wire_odometry_matches_ordered_fcu_visp'] = bool(keys) and keys == pkeys
        checks['every_wire_odometry_matches_ordered_fcu_visv'] = bool(keys) and keys == vkeys
        quality_min = params.get('VISO_QUAL_MIN')
        checks['actual_fcu_quality_minimum_known'] = number(quality_min)
        checks['fcu_backend_did_not_ignore_any_logged_frame'] = bool(pkeys) and bool(vkeys) and all(int(row['Ign']) == 0 for kind in ('VISP', 'VISV') for row in fcu[kind])
        checks['logged_quality_satisfies_actual_fcu_threshold'] = number(quality_min) and all(number(row.get('Q')) and row['Q'] >= quality_min for kind in ('VISP', 'VISV') for row in fcu[kind])
        health_report = wire_health(frames, protocol['live_readiness']['vision_position_mask'])
        monitored_health = monitor_health(monitored, protocol['live_readiness']['vision_position_mask'])
        checks['wire_sys_status_identity_and_raw_masks_valid'] = bool(health_report['samples']) and not health_report['errors']
        checks['monitor_sys_status_raw_decode_valid_and_identity_1_1'] = bool(monitored_health['samples']) and not monitored_health['errors']
        statuses, local_errors = local_rows(events)
        monitor_locals, monitor_local_errors = local_rows(monitored, monitor=True)
        readiness = [row for row in monitored if row.get('event') == 'readiness']
        checks['local_status_payloads_and_monitor_snapshots_present'] = bool(statuses) and bool(monitor_locals) and bool(readiness) and not local_errors and not monitor_local_errors
        adapters = [row for row in events if row.get('kind') == 'adapter_received']
        fault_gates, fault_report, fault_ns = fault_analysis(statuses, monitor_locals, readiness,
            monitored_health['samples'], outgoing, adapters)
        checks.update(fault_gates)
        pause = pause_analysis(case, protocol, events, statuses, readiness, monitored_health['samples'], fault_ns)
        if pause.get('status') == 'FAIL':
            checks.update({'pause:' + key: value for key, value in pause['checks'].items()})
        graph = '\n'.join((run / 'ros-observer' / name).read_text() for name in ('graph.before.json', 'graph.after.json'))
        checks['mission_and_flight_execution_nodes_absent_from_snapshots'] = not any(name in graph for name in ('impact_ap_research_mission', 'impact_ap_execution', 'xq_p13_flight_controller'))
        report.update({'natural_clock_fault': fault_report, 'source_pause_control': pause,
            'wire_sys_status': health_report, 'monitor_sys_status': monitored_health,
            'readiness_observations': {'ever_ready': monitor.get('ever_ready'), 'current_ready': monitor.get('current_ready'),
                'ready_sample_count': sum(row['data'].get('ready') is True for row in readiness), 'sample_count': len(readiness),
                'scope': 'Read-only sampled readiness indicator, never a flight permission.'},
            'fcu_backend_correspondence': {'wire_odometry_count': len(keys), 'visp_count': len(pkeys), 'visv_count': len(vkeys),
                'wire_usec_keys': keys, 'visp_remote_usec_keys': pkeys, 'visv_remote_usec_keys': vkeys,
                'wire_interarrival_monotonic': helper.gaps([row['monotonic_ns'] for row in outgoing], 1e9),
                'quality_min_actual': quality_min,
                'quality_histograms': {kind: dict(Counter(str(row['Q']) for row in fcu[kind])) for kind in ('VISP', 'VISV')},
                'quality_scope': 'Q=0 is unknown, not calibrated confidence; Ign=0 means accepted by backend quality gate, not EKF fusion.'},
            'fcu_log': {**fcu_meta, 'path': str(binary), 'parameters': {name: params.get(name) for name in (*required, 'VISO_QUAL_MIN')},
                'dsf_dropped_write_counts': sorted({row['Dp'] for row in fcu['DSF']})},
            'wire_command_counts': dict(Counter(str(row.get('command')) for row in commands)),
            'forbidden_commands': forbidden_commands, 'forbidden_control': forbidden_control,
            'continuous_stream_300_sample_gate_applied': False})
    except Exception as exc:
        report['errors'].append(type(exc).__name__ + ': ' + str(exc))
        report['checks']['analysis_inputs_and_structures_valid'] = False
    finally:
        if before is not None:
            try:
                after = fingerprint(inputs)
                report['original_files_before'], report['original_files_after'] = before, after
                report['checks']['all_bound_original_files_sha256_unchanged'] = before == after
                report['checks']['original_fcu_bin_set_unchanged'] = binary_paths == sorted((run / 'sitl_runtime/logs').glob('*.BIN'))
            except Exception as exc:
                report['errors'].append('Post-analysis fingerprint: ' + str(exc))
                report['checks']['all_bound_original_files_sha256_unchanged'] = False
    report['failed_checks'] = [key for key, value in report['checks'].items() if value is not True]
    if report['errors'] or report['failed_checks']:
        report['status'] = 'FAIL'
    elif report.get('source_pause_control', {}).get('status') not in ('PASS', 'NOT_APPLICABLE'):
        report['status'] = 'INCOMPLETE'
    else:
        report['status'] = 'PASS'
    report['pass_meaning'] = 'Predeclared unarmed diagnostic checks satisfied; a correctly detected clock fault is expected, not a flight repair.'
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, required=True)
    args = parser.parse_args(argv)
    target = args.run_dir / 'link-probe-result.json'
    if target.exists() or target.is_symlink():
        print('Refusing to overwrite an existing diagnostic result', file=sys.stderr)
        return 2
    result = analyze(args.run_dir, args.protocol)
    with target.open('x', encoding='utf-8') as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
        handle.write('\n')
    print(json.dumps({'status': result['status'], 'failed_checks': result['failed_checks'], 'errors': result['errors'],
        'clock_fault': result.get('natural_clock_fault', {}).get('status'),
        'pause_control': result.get('source_pause_control', {}).get('status'), 'output': str(target)}))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
