#!/usr/bin/env python3
"""Offline same-point observer-clock replay, not source-freshness reconstruction.

Only sealed B2 observer JSONL is read. Each adapter_received callback contributes
its own wall/monotonic pair, plus a synthetic increasing ordinal source token.
The original adapter header remains a separate diagnostic field. Neither those
tokens nor callback receipt times reconstruct a physical LIO acquisition time.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys


ROOT = Path('/mnt/d/CODEX/impact')
B2 = ROOT / 'evidence/RESEARCH_R2B3B2/r2b3b2_suite_20261020T140147Z_337'
B2_MANIFEST_SHA = '35d5008769564ed68f0b28de7a23f3918a86178775429379e83b3d5749a09ad6'
MODULE = ROOT / 'src/xq_autonomy/xq_autonomy/extnav_clock_guard.py'
CONFIG = {
    'schema_version': 1, 'stage': 'R2B3B3_OFFLINE_SAME_POINT_CLOCK_REPLAY',
    'policy': {'warmup_ns': 5_000_000_000, 'max_source_age_ns': 350_000_000,
               'min_rate_hz': 4.0, 'max_step_residual_ns': 50_000_000,
               'max_anchor_drift_ns': 50_000_000, 'rate_window_ns': 1_000_000_000},
    'controlled_fixture': {'samples': 121, 'period_ns': 100_000_000,
                           'injection_index': 70, 'clock_step_ns': 120_000_000,
                           'drift_per_sample_ns': 1_000_000},
    'source_token': 'adapter sample ordinal + 1; deliberately synthetic, not a timestamp',
    'same_point_clock_inputs': 'wall_time_ns and monotonic_ns from the same original observer callback event',
    'positive_control': 'same recorded monotonic sequence; synthetic wall0 + monotonic - monotonic0',
    'legacy_model': 'Local receipt/source-order, warmup, age and rate observation with no wall-clock consistency test; not a full reproduction of the prior node',
    'epoch_candidate': 'epoch0 + monotonic - monotonic0; only mapping monotonicity, never supplied to AP',
    'actual_flow_state_updates': 0, 'ap_tested': False, 'flight_regression_repaired': False,
}


def sha(path):
    value = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1048576), b''):
            value.update(block)
    return value.hexdigest()


def write(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write('\n')


def fingerprint(paths):
    result = {}
    for path in paths:
        if path.is_symlink() or not path.is_file():
            raise ValueError('A regular nonsymlink input is required: ' + str(path))
        result[str(path)] = {'bytes': path.stat().st_size, 'sha256': sha(path)}
    return result


def load_module(path):
    name = 'impact_frozen_clock_guard_replay'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    # Avoid creating caches beside either the frozen or current source.
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    return module


class LegacyLocalObserver:
    """Explicit clock-agnostic receipt model, not the original full estimator."""
    def __init__(self, policy):
        self.policy = policy
        self.last_token = self.last_mono = self.continuous_start = None
        self.samples = deque()

    def observe(self, token, mono):
        if self.last_mono is not None and mono < self.last_mono:
            return {'accepted': False, 'forward_allowed': False, 'local_ready': False,
                    'reason': 'receipt_monotonic_regressed'}
        if self.last_mono is not None and mono-self.last_mono > self.policy['max_source_age_ns']:
            self.samples.clear()
            self.continuous_start = None
        accepted = ((self.last_token is None or token > self.last_token)
                    and (self.last_mono is None or mono > self.last_mono))
        if accepted:
            self.last_token, self.last_mono = token, mono
            if self.continuous_start is None:
                self.continuous_start = mono
            self.samples.append(mono)
        while self.samples and self.samples[0] <= mono-self.policy['rate_window_ns']:
            self.samples.popleft()
        count_rate = len(self.samples)*1e9/self.policy['rate_window_ns']
        interval_rate = ((len(self.samples)-1)*1e9/(self.samples[-1]-self.samples[0])
                         if len(self.samples) > 1 and self.samples[-1] > self.samples[0] else 0.)
        rate = min(count_rate, interval_rate)
        age = mono-self.last_mono if self.last_mono is not None else None
        ready = (self.continuous_start is not None and mono-self.continuous_start >= self.policy['warmup_ns']
                 and age is not None and age <= self.policy['max_source_age_ns'] and rate >= self.policy['min_rate_hz'])
        return {'accepted': accepted, 'forward_allowed': accepted, 'local_ready': ready,
                'rate_hz': rate, 'receipt_age_ns': age, 'wall_clock_checked': False}


def recorded_samples(path):
    samples = []
    with path.open() as handle:
        for line_number, line in enumerate(handle, 1):
            event = json.loads(line)
            if event.get('kind') != 'adapter_received':
                continue
            for key in ('wall_time_ns', 'monotonic_ns', 'stamp_ns', 'event_sequence'):
                if type(event.get(key)) is not int or event[key] <= 0:
                    raise ValueError('Invalid recorded adapter event field ' + key)
            samples.append({'source_token': len(samples)+1,
                'observer_wall_ns': event['wall_time_ns'], 'mono_ns': event['monotonic_ns'],
                'adapter_header_stamp_ns': event['stamp_ns'], 'original_event_sequence': event['event_sequence'],
                'original_line_number': line_number, 'recorded_observer_wall_ns': event['wall_time_ns'],
                'input_kind': 'RECORDED_OBSERVER_CLOCKS_WITH_SYNTHETIC_SOURCE_TOKEN'})
    if not samples:
        raise ValueError('No adapter_received events in ' + str(path))
    return samples


def replay(name, samples, module, output):
    policy = module.ClockGuardPolicy(**CONFIG['policy'])
    guard = module.ExtnavClockGuard(policy)
    local = LegacyLocalObserver(CONFIG['policy'])
    first_wall, first_mono = samples[0]['observer_wall_ns'], samples[0]['mono_ns']
    states, mapped = [], []
    previous_wall = previous_header = previous_mapping = None
    rollbacks, header_nonincrease, mapping_nonincrease = [], [], []
    ledger = output / (name + '.steps.jsonl')
    with ledger.open('x', encoding='utf-8') as handle:
        for index, sample in enumerate(samples):
            wall, mono, token = sample['observer_wall_ns'], sample['mono_ns'], sample['source_token']
            step = guard.observe(token, wall, mono)
            snapshot = guard.snapshot(mono)
            legacy = local.observe(token, mono)
            candidate = first_wall + mono-first_mono
            mapped.append(candidate)
            if previous_wall is not None and wall < previous_wall:
                rollbacks.append(index)
            header = sample.get('adapter_header_stamp_ns')
            header_bad = header is not None and previous_header is not None and header <= previous_header
            if header_bad:
                header_nonincrease.append(index)
            if previous_mapping is not None and candidate <= previous_mapping:
                mapping_nonincrease.append(index)
            state = {'fault_latched': snapshot['fault_latched'], **asdict(step)}
            states.append(state)
            handle.write(json.dumps({'index': index, 'input': sample,
                'legacy_local_observation': legacy, 'latched_guard': {**asdict(step), 'snapshot': snapshot},
                'adapter_header_nonincrease_from_previous': header_bad,
                'epoch_plus_monotonic_mapping_candidate': {'mapped_ns': candidate,
                    'strictly_increases_from_previous': previous_mapping is None or candidate > previous_mapping,
                    'forwarded_to_AP': False, 'wall_epoch_to_AP_boot_relation_known': False}},
                separators=(',', ':'), allow_nan=False)+'\n')
            previous_wall, previous_header, previous_mapping = wall, header, candidate
    fault_indices = [i for i, state in enumerate(states) if state['fault_latched']]
    first_fault = fault_indices[0] if fault_indices else None
    checks = {
        'all_recorded_rollbacks_fault_on_affected_sample': all(states[i]['fault_latched'] and not states[i]['accepted']
            and not states[i]['forward_allowed'] and not states[i]['clock_ready'] for i in rollbacks),
        'no_ready_or_forward_after_any_rollback': all(not s['forward_allowed'] and not s['clock_ready']
            for i in rollbacks for s in states[i:]),
        'fault_remains_latched_and_blocks_suffix': first_fault is None or all(s['fault_latched'] and
            not s['accepted'] and not s['forward_allowed'] and not s['clock_ready'] for s in states[first_fault:]),
        'input_timestamps_not_modified_or_clamped': True,
    }
    # Re-evaluate only this independent legacy model for aggregate counts. Guard
    # never resets/restarts within the sequence and is not rerun for statistics.
    legacy = LegacyLocalObserver(CONFIG['policy'])
    local_states = [legacy.observe(v['source_token'], v['mono_ns']) for v in samples]
    return {'name': name, 'sample_count': len(samples), 'checks': checks,
            'status': 'PASS' if all(checks.values()) else 'FAIL',
            'recorded_wall_rollback_indices': rollbacks, 'adapter_header_nonincrease_indices': header_nonincrease,
            'first_guard_fault_index': first_fault, 'final_guard_snapshot': guard.snapshot(samples[-1]['mono_ns']),
            'guard_accepted_samples': sum(s['accepted'] for s in states),
            'guard_forward_allowed_samples': sum(s['forward_allowed'] for s in states),
            'guard_ready_samples': sum(s['clock_ready'] for s in states),
            'legacy_forward_allowed_samples': sum(s['forward_allowed'] for s in local_states),
            'legacy_local_ready_samples': sum(s['local_ready'] for s in local_states),
            'epoch_mapping_nonincrease_indices': mapping_nonincrease,
            'epoch_mapping_strictly_increasing': not mapping_nonincrease,
            'ledger_file': ledger.name, 'ledger_sha256': sha(ledger)}


def controlled_samples(kind):
    values, cfg = [], CONFIG['controlled_fixture']
    for index in range(cfg['samples']):
        token = index+1
        mono = 1_000_000_000 + index*cfg['period_ns']
        wall = 1_700_000_000_000_000_000 + index*cfg['period_ns']
        if kind == 'cumulative_small_drift':
            wall += index*cfg['drift_per_sample_ns']
        if index >= cfg['injection_index']:
            if kind == 'duplicate_source' and index == cfg['injection_index'] or kind == 'frozen_source':
                token = cfg['injection_index']
            if kind == 'monotonic_pause' and index == cfg['injection_index']:
                mono -= cfg['period_ns']
            if kind in ('forward_clock_step', 'backward_clock_step'):
                wall += cfg['clock_step_ns'] * (1 if kind == 'forward_clock_step' else -1)
        values.append({'source_token': token, 'observer_wall_ns': wall, 'mono_ns': mono,
                       'adapter_header_stamp_ns': None, 'input_kind': 'CONTROLLED_SYNTHETIC_FIXTURE'})
    return values


def run(args):
    manifest = B2 / 'archive-manifest.json'
    if sha(manifest) != B2_MANIFEST_SHA:
        raise ValueError('Sealed B2 manifest identity differs')
    expected = {row['file']: row for row in json.loads(manifest.read_text())['files']}
    event_paths = sorted((B2 / 'attempt').glob('*/ros-observer/events.jsonl'))
    if not event_paths:
        raise ValueError('No sealed B2 observer inputs')
    module_path = args.module_path.absolute()
    inputs = [Path(__file__).absolute(), module_path, manifest]
    for path in event_paths:
        inputs.extend([path, path.parent.parent / 'run.json', path.parent.parent / 'runner-result.json'])
    before = fingerprint(inputs)
    for path in inputs:
        if B2 in path.parents and path != manifest:
            record = expected[path.relative_to(B2).as_posix()]
            if before[str(path)] != {'bytes': record['bytes'], 'sha256': record['sha256']}:
                raise ValueError('Sealed replay input changed: ' + str(path))
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = args.output_dir or ROOT / 'runs' / ('research_r2b3b3_clock_replay_' + stamp)
    if output.is_symlink() or output.resolve().parent != (ROOT / 'runs').resolve() or output.exists():
        raise ValueError('Output must be a new direct child of canonical runs')
    output.mkdir(exist_ok=False)
    (output / 'inputs').mkdir()
    shutil.copy2(Path(__file__), output / 'inputs' / Path(__file__).name)
    shutil.copy2(module_path, output / 'inputs/extnav_clock_guard.py')
    if (sha(output / 'inputs/extnav_clock_guard.py') != before[str(module_path)]['sha256'] or
            sha(output / 'inputs' / Path(__file__).name) != before[str(Path(__file__).absolute())]['sha256']):
        raise ValueError('Frozen source copy differs from the declared input')
    write(output / 'configuration.json', CONFIG)
    write(output / 'input-fingerprint.before.json', before)
    module = load_module(output / 'inputs/extnav_clock_guard.py')
    if asdict(module.ClockGuardPolicy(**CONFIG['policy'])) != CONFIG['policy']:
        raise ValueError('Module policy differs from the declared configuration')
    recorded, positives, fixtures = [], [], []
    for path in event_paths:
        samples = recorded_samples(path)
        name = path.parent.parent.name
        actual = replay(name, samples, module, output)
        actual['original_events_path'] = str(path)
        actual['original_run_status'] = json.loads((path.parent.parent / 'runner-result.json').read_text())['status']
        recorded.append(actual)
        anchor_wall, anchor_mono = samples[0]['observer_wall_ns'], samples[0]['mono_ns']
        positive = [{**sample, 'observer_wall_ns': anchor_wall + sample['mono_ns']-anchor_mono,
                     'input_kind': 'SYNTHETIC_NO_JUMP_CLOCK_USING_RECORDED_MONOTONIC_ONLY'} for sample in samples]
        control = replay(name + '.synthetic_no_jump', positive, module, output)
        control['checks']['positive_accepts_and_forwards_every_synthetic_token'] = control['guard_forward_allowed_samples'] == len(positive)
        control['checks']['positive_has_no_fault'] = control['first_guard_fault_index'] is None
        control['checks']['positive_reaches_readiness_when_legacy_has_valid_window'] = (
            control['legacy_local_ready_samples'] == 0 or control['guard_ready_samples'] > 0)
        control['status'] = 'PASS' if all(control['checks'].values()) else 'FAIL'
        positives.append(control)
    for kind in ('healthy', 'duplicate_source', 'frozen_source', 'monotonic_pause',
                 'forward_clock_step', 'backward_clock_step', 'cumulative_small_drift'):
        result = replay('fixture.' + kind, controlled_samples(kind), module, output)
        expected_fault = (None if kind == 'healthy' else 51 if kind == 'cumulative_small_drift'
                          else CONFIG['controlled_fixture']['injection_index'])
        result['checks']['controlled_fault_at_declared_index'] = result['first_guard_fault_index'] == expected_fault
        if kind == 'healthy':
            result['checks']['healthy_control_becomes_ready'] = result['guard_ready_samples'] > 0
        result['status'] = 'PASS' if all(result['checks'].values()) else 'FAIL'
        fixtures.append(result)
    guard = module.ExtnavClockGuard(module.ClockGuardPolicy(**CONFIG['policy']))
    for sample in controlled_samples('healthy'):
        guard.observe(sample['source_token'], sample['observer_wall_ns'], sample['mono_ns'])
    last = controlled_samples('healthy')[-1]['mono_ns']
    prior, stale = guard.snapshot(last), guard.snapshot(last + CONFIG['policy']['max_source_age_ns'] + 1)
    timeout_checks = {'ready_before_timeout': prior['clock_ready'], 'not_ready_after_350ms_plus_1ns': not stale['clock_ready'],
                      'stale_reason_reported': 'source_stale' in stale['reasons'], 'timeout_is_not_clock_fault': not stale['fault_latched']}
    write(output / 'fixture.source_timeout.json', {'checks': timeout_checks, 'before': prior, 'after': stale})
    after = fingerprint(inputs)
    checks = {'all_original_file_sets_and_sha256_unchanged': before == after and
                  event_paths == sorted((B2 / 'attempt').glob('*/ros-observer/events.jsonl')),
              'every_recorded_wall_rollback_blocked_and_latched': all(row['status'] == 'PASS' for row in recorded),
              'recorded_wall_rollbacks_present': sum(len(row['recorded_wall_rollback_indices']) for row in recorded) > 0,
              'positive_counterfactual_controls_pass': all(row['status'] == 'PASS' for row in positives),
              'controlled_fixtures_pass': all(row['status'] == 'PASS' for row in fixtures),
              'receipt_timeout_fixture_pass': all(timeout_checks.values())}
    write(output / 'input-fingerprint.after.json', after)
    summary = {'schema_version': 1, 'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks,
        'recorded_observer_clock_replays': recorded, 'synthetic_no_jump_controls': positives, 'controlled_fixtures': fixtures,
        'configuration_sha256': sha(output / 'configuration.json'), 'script_sha256': before[str(Path(__file__).absolute())]['sha256'],
        'module_sha256': before[str(module_path)]['sha256'], 'sealed_b2_manifest_sha256': B2_MANIFEST_SHA,
        'actual_flow_state_updates': 0, 'ap_tested': False, 'flight_regression_repaired': False,
        'hardware_readiness': 'NOT_READY', 'timestamps_clamped': False,
        'scope': 'Offline observer wall/monotonic clock replay with synthetic ordinal source tokens. This does not reconstruct original LIO acquisition, actual adapter emission clocks, end-to-end freshness or a deployed transport fix.',
        'epoch_candidate_scope': 'Mapping monotonicity only; neither automatic timestamp repair nor AP validation.',
        'warmup_scope': 'The guard may allow healthy warmup frames; clock_ready separately requires the declared continuous local observation window.',
        'original_header_scope': 'Original adapter header stamps are retained only for a separate nonincrease diagnostic; not mixed with observer receipt clocks.'}
    write(output / 'summary.json', summary)
    print(json.dumps({'status': summary['status'], 'output_directory': str(output), 'checks': checks}))
    return 0 if summary['status'] == 'PASS' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--module-path', type=Path, default=MODULE)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        print(json.dumps({'status': 'FAIL', 'error': type(exc).__name__ + ': ' + str(exc)}), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
