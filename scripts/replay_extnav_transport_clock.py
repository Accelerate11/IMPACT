#!/usr/bin/env python3
"""Offline candidate transport arithmetic from observed, frozen B3 brackets.

Synthetic ordinal source tokens exercise the existing guard. They do not
reconstruct source acquisition time, missing callbacks, or original AP output.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys


ROOT = Path('/mnt/d/CODEX/impact')
B3 = ROOT / 'evidence/RESEARCH_R2B3B3/r2b3b3_suite_20261020T150451Z_320'
B3_MANIFEST_SHA = '251a07aef59595e2eaa278778ca8f70588ca53e8790aa40e3f1b13f886af2373'
MODULE = ROOT / 'src/xq_autonomy/xq_autonomy/extnav_transport_clock.py'
MODULE_SHA = '1caf001f2bf16527a70935724941bba5bd2c1c8780d1d6edd00f9a53ab4851d8'
GUARD = B3 / 'current-helpers/src/xq_autonomy/xq_autonomy/extnav_clock_guard.py'
GUARD_SHA = 'a272f1203388794f90bf2644b550438f9e7a194293960a7106754f5d1a878995'
PROTOCOL = ROOT / 'config/research_extnav_transport_study.json'
CASES = ('r2b3b3_synthetic_lean_20261020T145204Z_272', 'r2b3b3_lio_full_20261020T145525Z_899')
POLICY = {'warmup_ns': 5_000_000_000, 'max_source_age_ns': 350_000_000,
          'min_rate_hz': 4.0, 'max_step_residual_ns': 50_000_000,
          'max_anchor_drift_ns': 50_000_000, 'rate_window_ns': 1_000_000_000}
CONFIG = {'stage': 'R2B3B4_OFFLINE_NODE_BRACKET_TRANSPORT_CLOCK_REPLAY',
          'maximum_sampling_span_ns': 5_000_000, 'wire_resolution_ns': 1000,
          'guard_policy': POLICY, 'fixture_samples': 161, 'fixture_period_ns': 100_000_000,
          'fixture_injection_index': 70, 'fixture_wall_step_ns': 120_000_000,
          'fixture_drift_per_sample_ns': 1_000_000,
          'token_semantics': 'Deduplicated observed bracket ordinal + 1; synthetic, not source time.',
          'execution_scope': 'Offline arithmetic only; no publication, AP, ROS, clock change or SQLite.'}


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1048576), b''):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    with path.open('x', encoding='utf-8') as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write('\n')


def fingerprint(paths):
    result = {}
    for path in sorted(set(paths)):
        if any(parent.is_symlink() for parent in (path, *path.parents)) or not path.is_file():
            raise ValueError('Regular nonsymlink input required: ' + str(path))
        result[str(path)] = {'bytes': path.stat().st_size, 'sha256': sha(path)}
    return result


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    return module


def strictly_increasing(values):
    return bool(values) and all(type(v) is int and v > 0 for v in values) and all(
        b > a for a, b in zip(values, values[1:]))


def extract_samples(path):
    """Keep first-occurrence order and every full original event reference."""
    samples, lookup = [], {}
    status_count = absent_count = 0
    with path.open() as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            event = json.loads(line)
            if event.get('kind') != 'adapter_status':
                continue
            status_count += 1
            status = event.get('message', {}).get('parsed')
            if not isinstance(status, dict):
                raise ValueError('Original adapter_status parsed payload missing')
            sampling = status.get('transport_clock_sampling')
            if sampling is None:
                absent_count += 1
                continue
            names = ('transport_stamp_ns', 'monotonic_before_ns', 'monotonic_after_ns')
            values = tuple(sampling.get(name) for name in names)
            if not all(type(value) is int and value > 0 for value in values):
                raise ValueError('Invalid original sampling tuple')
            wall, before, after = values
            span, midpoint = after-before, (before+after)//2
            if (sampling.get('sampling_span_ns') != span
                    or sampling.get('monotonic_midpoint_ns') != midpoint
                    or sampling.get('maximum_sampling_span_ns') != CONFIG['maximum_sampling_span_ns']
                    or sampling.get('sampling_uncertain') is not (span > CONFIG['maximum_sampling_span_ns'])):
                raise ValueError('Original sampling fields disagree with their tuple')
            origin = {'line_number': line_number, 'event_sequence': event.get('event_sequence'),
                      'event': event}
            if values not in lookup:
                lookup[values] = len(samples)
                samples.append({'source_token': len(samples)+1, 'raw_wall_ns': wall,
                    'before_ns': before, 'after_ns': after, 'monotonic_ns': midpoint,
                    'sampling_span_ns': span, 'sampling_uncertain': sampling['sampling_uncertain'],
                    'input_kind': 'RECORDED_NODE_BRACKET_WITH_SYNTHETIC_ORDINAL_TOKEN',
                    'original_event_file': str(path), 'origins': []})
            samples[lookup[values]]['origins'].append(origin)
    if not samples:
        raise ValueError('No observed node sampling brackets: ' + str(path))
    return samples, {'adapter_status_count': status_count, 'statuses_without_sampling': absent_count,
        'deduplicated_bracket_count': len(samples), 'total_sampling_occurrences': status_count-absent_count,
        'duplicate_occurrences': status_count-absent_count-len(samples),
        'deduplication': 'Global equality of original (transport_stamp_ns,before_ns,after_ns); first occurrence order retained.'}


def replay(samples, clock_module, guard_module, ledger=None):
    clock = clock_module.ExtnavTransportClock(CONFIG['maximum_sampling_span_ns'])
    raw_guard = guard_module.ExtnavClockGuard(guard_module.ClockGuardPolicy(**POLICY))
    mapped_guard = guard_module.ExtnavClockGuard(guard_module.ClockGuardPolicy(**POLICY))
    mapped_ns, raw_forward_ns, mapped_forward_ns = [], [], []
    states, rollbacks, uncertain = [], [], []
    previous_wall = None
    for index, sample in enumerate(samples):
        wall, before, after = (sample[key] for key in ('raw_wall_ns', 'before_ns', 'after_ns'))
        token, mono = sample['source_token'], (before+after)//2
        is_uncertain = after-before > CONFIG['maximum_sampling_span_ns']
        candidate = clock.sample(wall, before, after)
        clock_snapshot = clock.snapshot()
        raw_step = (raw_guard.tick(mono) if is_uncertain else raw_guard.observe(token, wall, mono))
        mapped_step = (mapped_guard.observe(token, candidate.mapped_ns, candidate.monotonic_ns)
                       if candidate.accepted else mapped_guard.tick(mono))
        raw_snapshot, mapped_snapshot = raw_guard.snapshot(mono), mapped_guard.snapshot(mono)
        if candidate.accepted:
            mapped_ns.append(candidate.mapped_ns)
        if raw_step.forward_allowed:
            raw_forward_ns.append(wall)
        if mapped_step.forward_allowed:
            mapped_forward_ns.append(candidate.mapped_ns)
        if previous_wall is not None and wall < previous_wall:
            rollbacks.append(index)
        if is_uncertain:
            uncertain.append(index)
        state = {'index': index, 'synthetic_source_token': token,
            'original_input': {k: v for k, v in sample.items() if k != 'origins'},
            'raw_guard_step': asdict(raw_step), 'raw_guard_snapshot': raw_snapshot,
            'candidate_clock_step': asdict(candidate), 'candidate_clock_snapshot': clock_snapshot,
            'mapped_guard_step': asdict(mapped_step), 'mapped_guard_snapshot': mapped_snapshot,
            'candidate_and_guard_allow_arithmetic_only': candidate.accepted and mapped_step.forward_allowed,
            'candidate_and_guard_ready_arithmetic_only': candidate.accepted and mapped_step.clock_ready,
            'candidate_sent_to_AP': False}
        states.append(state)
        if ledger is not None:
            ledger.write(json.dumps(state, separators=(',', ':'), allow_nan=False)+'\n')
        previous_wall = wall
    def first_fault(key):
        return next((state['index'] for state in states if state[key]['fault_latched']), None)
    checks = {'candidate_accepted_nanoseconds_strictly_increasing': strictly_increasing(mapped_ns),
        'candidate_accepted_floored_microseconds_strictly_increasing': strictly_increasing([v//1000 for v in mapped_ns]),
        'mapped_guard_forward_nanoseconds_strictly_increasing': strictly_increasing(mapped_forward_ns),
        'mapped_guard_forward_floored_microseconds_strictly_increasing': strictly_increasing([v//1000 for v in mapped_forward_ns]),
        'uncertain_brackets_never_accepted_or_forwarded': all(
            not states[i]['candidate_clock_step']['accepted'] and not states[i]['raw_guard_step']['forward_allowed']
            and not states[i]['mapped_guard_step']['forward_allowed'] for i in uncertain),
        'fixed_anchor_initialized_once': clock.snapshot()['anchor_initializations'] == 1,
        'candidate_never_clamped': clock.snapshot()['timestamp_clamping_used'] is False}
    return {'sample_count': len(samples), 'checks': checks, 'states': states,
        'raw_wall_backward_indices': rollbacks, 'uncertain_bracket_indices': uncertain,
        'first_raw_guard_fault_index': first_fault('raw_guard_snapshot'),
        'first_mapped_guard_fault_index': first_fault('mapped_guard_snapshot'),
        'first_candidate_clock_fault_index': first_fault('candidate_clock_snapshot'),
        'candidate_accepted_count': len(mapped_ns), 'raw_guard_forward_allowed_count': len(raw_forward_ns),
        'mapped_guard_forward_allowed_count': len(mapped_forward_ns),
        'raw_guard_ready_count': sum(v['raw_guard_step']['clock_ready'] for v in states),
        'mapped_guard_ready_count': sum(v['candidate_and_guard_ready_arithmetic_only'] for v in states),
        'raw_guard_forward_ns_strict': strictly_increasing(raw_forward_ns),
        'raw_guard_forward_floor_us_strict': strictly_increasing([v//1000 for v in raw_forward_ns]),
        'final_candidate_snapshot': clock.snapshot(),
        'final_raw_guard_snapshot': raw_guard.snapshot((samples[-1]['before_ns']+samples[-1]['after_ns'])//2),
        'final_mapped_guard_snapshot': mapped_guard.snapshot((samples[-1]['before_ns']+samples[-1]['after_ns'])//2)}


def controlled_samples(kind):
    result = []
    for index in range(CONFIG['fixture_samples']):
        mono = 1_000_000_000 + index*CONFIG['fixture_period_ns']
        token, span = index+1, 20_000
        if kind == 'sub_microsecond_conflict':
            mono = 1_000_000_000 + index*999
            span = 0
        if kind in ('source_gap_350ms_plus_1ns', 'source_gap_exact_350ms') and index >= CONFIG['fixture_injection_index']:
            mono += 250_000_001 if kind == 'source_gap_350ms_plus_1ns' else 250_000_000
        wall = 1_700_000_000_000_000_000 + mono-1_000_000_000
        if kind == 'cumulative_small_drift':
            wall += index*CONFIG['fixture_drift_per_sample_ns']
        if index >= CONFIG['fixture_injection_index']:
            if kind == 'duplicate_source' and index == CONFIG['fixture_injection_index'] or kind == 'frozen_source':
                token = CONFIG['fixture_injection_index']
            if kind in ('forward_clock_step', 'backward_clock_step'):
                wall += CONFIG['fixture_wall_step_ns']*(1 if kind == 'forward_clock_step' else -1)
            if kind == 'uncertain_bracket' and index == CONFIG['fixture_injection_index']:
                span = CONFIG['maximum_sampling_span_ns']+1
        before = mono-span//2
        result.append({'source_token': token, 'raw_wall_ns': wall, 'before_ns': before,
            'after_ns': before+span, 'input_kind': 'CONTROLLED_SYNTHETIC_CLOCK_AND_SOURCE_TOKEN'})
    return result


def fixture_checks(kind, result):
    expected_raw = (70 if kind in ('duplicate_source', 'frozen_source', 'forward_clock_step', 'backward_clock_step')
                    else 51 if kind == 'cumulative_small_drift' else None)
    expected_mapped = 70 if kind in ('duplicate_source', 'frozen_source') else None
    expected_clock = 1 if kind == 'sub_microsecond_conflict' else None
    checks = {'raw_guard_fault_at_expected_index': result['first_raw_guard_fault_index'] == expected_raw,
              'mapped_guard_fault_at_expected_index': result['first_mapped_guard_fault_index'] == expected_mapped,
              'candidate_fault_at_expected_index': result['first_candidate_clock_fault_index'] == expected_clock}
    states = result['states']
    if kind == 'source_gap_350ms_plus_1ns':
        checks['ready_before_gap'] = states[69]['mapped_guard_step']['clock_ready'] is True
        checks['350ms_plus_1ns_restarts_five_second_warmup'] = (
            states[70]['mapped_guard_step']['clock_ready'] is False
            and states[70]['mapped_guard_snapshot']['continuous_valid_duration_ns'] == 0
            and states[119]['mapped_guard_step']['clock_ready'] is False
            and states[120]['mapped_guard_step']['clock_ready'] is True)
    if kind == 'source_gap_exact_350ms':
        checks['exact_350ms_does_not_reset_window'] = states[70]['mapped_guard_step']['clock_ready'] is True
    if kind == 'sub_microsecond_conflict':
        checks['ns_advances_but_us_collision_rejected_without_clamp'] = (
            states[0]['candidate_clock_step']['accepted'] is True
            and states[1]['original_input']['raw_wall_ns'] > states[0]['original_input']['raw_wall_ns']
            and states[1]['candidate_clock_step']['accepted'] is False
            and any('microsecond' in v for v in states[1]['candidate_clock_step']['reasons'])
            and all(not v['candidate_clock_step']['accepted'] for v in states[1:]))
        checks['original_ns_only_guard_does_not_establish_us_uniqueness'] = result['raw_guard_forward_floor_us_strict'] is False
    if expected_mapped is not None:
        checks['source_violation_still_blocks_mapped_guard_suffix'] = all(
            not v['mapped_guard_step']['forward_allowed'] and not v['mapped_guard_step']['clock_ready']
            for v in states[expected_mapped:])
    return checks


def compact(result):
    return {key: value for key, value in result.items() if key != 'states'}


def resolve_events(archive, expected, raw_index):
    resolved = []
    for name in CASES:
        relative = 'attempt/'+name+'/ros-observer/events.jsonl'
        archived = archive / relative
        if relative in expected:
            row = expected[relative]
            resolved.append({'case': name, 'path': archived, 'bytes': row['bytes'], 'sha256': row['sha256'],
                             'storage': 'COPIED_FROZEN_ARCHIVE_EVENT_FILE'})
        else:
            matches = [row for row in raw_index['files'] if row.get('intended_archive_file') == relative]
            if len(matches) != 1:
                raise ValueError('No unique frozen raw-index source for '+relative)
            row = matches[0]
            resolved.append({'case': name, 'path': Path(row['original_path']), 'bytes': row['bytes'],
                'sha256': row['sha256'], 'storage': 'ORIGINAL_EVENT_FILE_SHA_PINNED_BY_SEALED_RAW_INDEX'})
    return resolved


def archive_file_set():
    paths = list(B3.rglob('*'))
    if any(path.is_symlink() for path in (B3, *B3.parents, *paths)):
        raise ValueError('Sealed B3 aliases forbidden')
    return sorted(path.relative_to(B3).as_posix() for path in paths if path.is_file())


def run(args):
    manifest = B3 / 'archive-manifest.json'
    if sha(manifest) != B3_MANIFEST_SHA:
        raise ValueError('B3 fixed manifest identity differs')
    expected = {row['file']: row for row in read(manifest)['files']}
    original_set = archive_file_set()
    if len(original_set) != 339 or set(original_set) != set(expected) | {'archive-manifest.json'}:
        raise ValueError('B3 archive file set differs')
    raw_path = B3 / 'raw-file-index.json'
    if {'bytes': raw_path.stat().st_size, 'sha256': sha(raw_path)} != {
            key: expected['raw-file-index.json'][key] for key in ('bytes', 'sha256')}:
        raise ValueError('Frozen raw-index identity differs')
    events = resolve_events(B3, expected, read(raw_path))
    module_path, guard_path, protocol_path = args.module_path.absolute(), args.guard_module_path.absolute(), args.protocol.absolute()
    inputs = [Path(__file__).absolute(), module_path, guard_path, protocol_path, manifest, raw_path]
    inputs += [Path(__file__).parent / 'test/test_extnav_transport_clock_replay.py']
    for row in events:
        inputs.append(row['path'])
        inputs += [B3 / 'attempt' / row['case'] / name for name in ('run.json', 'runner-result.json', 'link-probe-result.json')]
    before = fingerprint(inputs)
    for path in inputs:
        if B3 in path.parents and path != manifest:
            record = expected[path.relative_to(B3).as_posix()]
            if before[str(path)] != {key: record[key] for key in ('bytes', 'sha256')}:
                raise ValueError('Frozen selected archive input changed: '+str(path))
    for row in events:
        if before[str(row['path'])] != {key: row[key] for key in ('bytes', 'sha256')}:
            raise ValueError('Recorded event bytes/SHA differ from sealed source identity')
    if before[str(module_path)]['sha256'] != MODULE_SHA or before[str(guard_path)]['sha256'] != GUARD_SHA:
        raise ValueError('Clock or original guard differs from frozen sourcefinal identity')
    protocol = read(protocol_path)
    if (protocol.get('stage') != 'R2B3B4_OFFLINE_TRANSPORT_CLOCK_AND_STARTUP_CONTRACT'
            or protocol.get('candidate_sent_to_AP') is not False
            or protocol.get('prior_seven_message_gap_repaired') is not False
            or protocol.get('actual_flow_state_updates') != 0
            or protocol.get('flight_regression_repaired') is not False
            or protocol.get('maximum_sampling_span_ns') != 5_000_000
            or protocol.get('raw_wall_anomaly_budget_ns') != 50_000_000
            or protocol.get('wire_time_resolution_ns') != 1000):
        raise ValueError('Unexpected offline protocol scope or engineering budgets')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid())
    output = (args.output_dir or ROOT / 'runs' / ('research_r2b3b4_clock_replay_'+stamp)).absolute()
    if output.exists() or any(path.is_symlink() for path in (output, *output.parents)) or output.parent.resolve() != (ROOT/'runs').resolve():
        raise ValueError('Output must be a new direct child of canonical runs')
    output.mkdir(exist_ok=False)
    frozen = output / 'sources'
    frozen.mkdir()
    snapshots = {Path(__file__).absolute(): frozen/'replay_extnav_transport_clock.py',
                 module_path: frozen/'extnav_transport_clock.py', guard_path: frozen/'extnav_clock_guard.py',
                 protocol_path: frozen/protocol_path.name,
                 Path(__file__).parent/'test/test_extnav_transport_clock_replay.py': frozen/'test_extnav_transport_clock_replay.py'}
    for source, target in snapshots.items():
        shutil.copy2(source, target)
        if sha(target) != before[str(source)]['sha256']:
            raise ValueError('Frozen source copy mismatch')
    write(output/'configuration.json', CONFIG)
    write(output/'input-fingerprint.before.json', before)
    write(output/'archive-file-set.before.json', original_set)
    clock_module = load_module(frozen/'extnav_transport_clock.py', 'b4_frozen_transport_clock')
    guard_module = load_module(frozen/'extnav_clock_guard.py', 'b4_frozen_original_clock_guard')
    if asdict(guard_module.ClockGuardPolicy(**POLICY)) != POLICY:
        raise ValueError('Original guard policy differs')
    recorded, positives, fixtures = [], [], []
    for row in events:
        samples, counts = extract_samples(row['path'])
        with (output/(row['case']+'.observed-inputs.jsonl')).open('x') as handle:
            for sample in samples:
                handle.write(json.dumps(sample, separators=(',', ':'), allow_nan=False)+'\n')
        with (output/(row['case']+'.steps.jsonl')).open('x') as handle:
            replayed = replay(samples, clock_module, guard_module, handle)
        replayed['checks'].update({'observed_raw_guard_fault_retained': replayed['first_raw_guard_fault_index'] is not None,
            'candidate_clock_has_no_mapping_fault_on_recorded_brackets': replayed['first_candidate_clock_fault_index'] is None,
            'mapped_ordinal_guard_has_no_fault': replayed['first_mapped_guard_fault_index'] is None})
        replayed['status'] = 'PASS' if all(replayed['checks'].values()) else 'FAIL'
        recorded.append({'case': row['case'], 'source': {k: str(v) if isinstance(v, Path) else v for k,v in row.items()},
            'original_runner_status': read(B3/'attempt'/row['case']/'runner-result.json')['status'],
            'original_analysis_status': read(B3/'attempt'/row['case']/'link-probe-result.json')['status'],
            'extraction': counts, **compact(replayed)})
        first_mono = (samples[0]['before_ns']+samples[0]['after_ns'])//2
        positive = [{**sample, 'raw_wall_ns': samples[0]['raw_wall_ns']+(sample['before_ns']+sample['after_ns'])//2-first_mono,
                     'input_kind': 'SYNTHETIC_NO_JUMP_WALL_WITH_ORIGINAL_BRACKETS', 'origins': []} for sample in samples]
        with (output/(row['case']+'.synthetic-no-jump.steps.jsonl')).open('x') as handle:
            result = replay(positive, clock_module, guard_module, handle)
        result['checks']['synthetic_no_jump_has_no_raw_or_mapped_fault'] = all(result[key] is None for key in (
            'first_raw_guard_fault_index', 'first_mapped_guard_fault_index', 'first_candidate_clock_fault_index'))
        result['checks']['both_accept_all_trusted_observed_brackets'] = (
            result['raw_guard_forward_allowed_count'] == result['mapped_guard_forward_allowed_count'] ==
            len(positive)-len(result['uncertain_bracket_indices']))
        result['status'] = 'PASS' if all(result['checks'].values()) else 'FAIL'
        positives.append({'case': row['case'], **compact(result)})
    kinds = ('no_jump', 'forward_clock_step', 'backward_clock_step', 'cumulative_small_drift',
             'duplicate_source', 'frozen_source', 'source_gap_exact_350ms', 'source_gap_350ms_plus_1ns',
             'uncertain_bracket', 'sub_microsecond_conflict')
    for kind in kinds:
        with (output/('fixture.'+kind+'.steps.jsonl')).open('x') as handle:
            result = replay(controlled_samples(kind), clock_module, guard_module, handle)
        result['checks'].update(fixture_checks(kind, result))
        result['status'] = 'PASS' if all(result['checks'].values()) else 'FAIL'
        fixtures.append({'kind': kind, **compact(result)})
    guard = guard_module.ExtnavClockGuard(guard_module.ClockGuardPolicy(**POLICY))
    healthy = controlled_samples('no_jump')
    for sample in healthy:
        guard.observe(sample['source_token'], sample['raw_wall_ns'], (sample['before_ns']+sample['after_ns'])//2)
    last = (healthy[-1]['before_ns']+healthy[-1]['after_ns'])//2
    initial, exact, stale = guard.snapshot(last), guard.snapshot(last+350_000_000), guard.snapshot(last+350_000_001)
    timeout = {'ready_before_timeout': initial['clock_ready'], 'exact_350ms_is_not_stale': 'source_stale' not in exact['reasons'],
               '350ms_plus_1ns_is_stale_and_not_ready': 'source_stale' in stale['reasons'] and not stale['clock_ready'],
               'timeout_does_not_manufacture_clock_fault': not stale['fault_latched']}
    write(output/'fixture.no_new_observation_timeout.json', {'checks': timeout, 'before': initial, 'exact': exact, 'stale': stale})
    after, final_set = fingerprint(inputs), archive_file_set()
    checks = {'selected_input_file_set_bytes_sha256_unchanged': before == after,
        'sealed_b3_full_file_name_set_unchanged': original_set == final_set,
        'recorded_clock_comparisons_satisfy_named_checks': all(v['status']=='PASS' for v in recorded),
        'recorded_bracket_synthetic_no_jump_controls_pass': all(v['status']=='PASS' for v in positives),
        'controlled_fixtures_pass': all(v['status']=='PASS' for v in fixtures),
        'no_new_observation_timeout_control_pass': all(timeout.values())}
    write(output/'input-fingerprint.after.json', after)
    write(output/'archive-file-set.after.json', final_set)
    summary = {'schema_version': 1, 'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks,
        'recorded_bracket_comparisons': recorded, 'synthetic_no_jump_controls': positives, 'controlled_fixtures': fixtures,
        'sealed_b3_manifest_sha256': B3_MANIFEST_SHA, 'candidate_module_sha256': MODULE_SHA,
        'original_guard_sha256': GUARD_SHA, 'script_sha256': before[str(Path(__file__).absolute())]['sha256'],
        'protocol_sha256': before[str(protocol_path)]['sha256'], 'configuration_sha256': sha(output/'configuration.json'),
        'input_preservation_scope': 'Selected input files are SHA-bound before/after; complete B3 archive file names are checked. Full B3 archive SHA preservation is a separate B4 checkpoint, not inferred here.',
        'scope': 'Offline observed node-bracket arithmetic with synthetic ordinal source tokens. Missing observations and actual source stamps are never filled in. Guard-forward/ready counts are counterfactual arithmetic, not original node outputs, AP receipt, source acquisition age or latency.',
        'raw_observation_scope': 'Original raw wall and brackets, including rollback/uncertainty, remain in the full input provenance ledger. Synthetic no-jump rows are separately labeled.',
        'source_integrity_scope': 'A mapped generation clock does not repair duplicate/frozen source tokens or prove source freshness.',
        'candidate_sent_to_AP': False, 'actual_flow_state_updates': 0, 'flight_regression_repaired': False,
        'prior_seven_message_gap_repaired': False, 'hardware_readiness': 'NOT_READY', 'sqlite_connections_opened': 0}
    write(output/'summary.json', summary)
    print(json.dumps({'status': summary['status'], 'checks': checks, 'output_directory': str(output)}))
    return 0 if summary['status']=='PASS' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--module-path', type=Path, default=MODULE)
    parser.add_argument('--guard-module-path', type=Path, default=GUARD)
    parser.add_argument('--protocol', type=Path, default=PROTOCOL)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        print(json.dumps({'status': 'FAIL', 'error': type(exc).__name__+': '+str(exc)}), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
