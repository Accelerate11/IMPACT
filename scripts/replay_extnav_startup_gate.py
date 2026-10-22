#!/usr/bin/env python3
"""Offline graph-identity checks and explicitly synthetic startup schedules.

Two recorded B3 snapshots are evaluated INDEPENDENTLY. Their spacing never
becomes evidence for actual one-second endpoint visibility or a startup-gap
repair. No ROS imports/nodes, AP/Gazebo, SQLite, or source-frame sends occur.
All outputs require a brand-new directory; inputs are hashed before and after.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import sys


ROOT = Path('/mnt/d/CODEX/impact')
DEFAULT_RUN = Path('/home/accelerate/impact_research_ws/experiments/results/research_multisensor/'
                   'r2b3b3_lio_full_20261020T145525Z_899')
DEFAULT_REVIEW = ROOT/'runs/research_r2b3b3_live_review_20261020T150327Z_335/adapter-to-wire-startup-gap.json'
TOPIC = '/uav1/mavros/odometry/out'
NS = 1_000_000_000
DEFAULT_PROTOCOL = ROOT/'config/research_extnav_startup_gate.json'


def identity(path):
    data = path.read_bytes()
    return {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


def write_json(path, data):
    with path.open('x') as handle:
        json.dump(data, handle, indent=2, allow_nan=False)
        handle.write('\n')


def copy_new(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('xb') as handle:
        handle.write(source.read_bytes())


def load_module(path):
    name = '_impact_frozen_extnav_startup_gate_replay'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    # Dataclasses with deferred annotations resolve their defining module here.
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def controlled_steps(module):
    """Invented graph/time fixtures only; none are recorded flight observations."""
    observer = {
        'node_name': 'impact_extnav_disarmed_probe', 'node_namespace': '/',
        'topic_type': 'nav_msgs/msg/Odometry', 'endpoint_gid': [7]*24,
        'qos': {'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE'},
    }
    mavros = {
        'node_name': 'odometry', 'node_namespace': '/uav1/mavros',
        'topic_type': 'nav_msgs/msg/Odometry', 'endpoint_gid': [42]*24,
        'qos': {'reliability': 'RELIABLE', 'durability': 'VOLATILE'},
    }
    restarted = deepcopy(mavros)
    restarted['endpoint_gid'] = [43]*24
    schedule = [
        ('observer_only', 0, [observer], False),
        ('observer_only_still_insufficient', 100_000_000, [observer], False),
    ]
    def window(prefix, start, candidate, include_first=True):
        # Controlled 10Hz queries, plus an explicit 1ns-before-dwell sample.
        offsets = list(range(0 if include_first else 100_000_000, NS, 100_000_000)) + [NS-1, NS]
        schedule.extend((prefix+'_'+str(offset), start+offset, [observer, candidate], offset == NS)
                        for offset in offsets)
    window('first_visible', 500_000_000, mavros)
    schedule.append(('mavros_disappears', 1_600_000_000, [observer], False))
    window('reappeared_visible', 1_700_000_000, mavros)
    window('new_gid_visible', 2_800_000_000, restarted)
    schedule.append(('restarted_disappears', 3_900_000_000, [observer], False))
    window('returned_visible', 4_000_000_000, restarted)
    schedule.append(('one_second_query_gap_same_gid_resets', 6*NS, [observer, restarted], False))
    window('after_gap_visible', 6*NS, restarted, include_first=False)
    schedule.extend([
        ('old_two_query_contract_first_endpoint', 8*NS, [observer, restarted], False),
        ('old_two_query_contract_second_endpoint_not_ready', 9*NS, [observer, restarted], False),
    ])
    window('after_second_gap_visible', 9*NS, restarted, include_first=False)
    schedule.extend([
        ('synthetic_monotonic_regression', 10*NS-1, [observer, restarted], False),
        ('clock_fault_stays_latched', 11*NS, [observer, restarted], False),
    ])
    gate = module.ExtnavStartupGate()
    ledger = []
    for index, (name, offset, endpoints, expected_ready) in enumerate(schedule):
        values = deepcopy(endpoints)
        saved = deepcopy(values)
        mono = 100*NS + offset
        step = gate.observe(values, mono)
        ledger.append({
            'classification': 'SYNTHETIC_CONTROLLED_ENDPOINT_SCHEDULE_NOT_RECORDED_CONTINUITY',
            'index': index, 'phase': name, 'synthetic_monotonic_ns': mono,
            'input_endpoints': saved, 'input_endpoints_unchanged': values == saved,
            'expected_ready': expected_ready, 'ready': step.ready, 'reasons': list(step.reasons),
            'matches_expectation': step.ready is expected_ready, 'snapshot': gate.snapshot(),
            'actual_source_frames_or_AP_messages_sent': False,
        })
    return ledger


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, default=DEFAULT_RUN)
    parser.add_argument('--review-path', type=Path, default=DEFAULT_REVIEW)
    parser.add_argument('--module-path', type=Path,
                        default=ROOT/'src/xq_autonomy/xq_autonomy/extnav_startup_gate.py')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--protocol', type=Path, default=DEFAULT_PROTOCOL)
    args = parser.parse_args(argv)
    run = args.run_dir.resolve(strict=True)
    review_path = args.review_path.resolve(strict=True)
    module_path = args.module_path.resolve(strict=True)
    protocol_path = args.protocol.resolve(strict=True)
    protocol = json.loads(protocol_path.read_text())
    if (protocol.get('protocol_revision') != 2 or protocol.get('continuous_visibility_ns') != NS
            or protocol.get('maximum_observation_gap_ns') != 250_000_000
            or protocol.get('maximum_permitted_policy_gap_ns') != 350_000_000):
        raise ValueError('The corrected replay requires the explicitly declared revision2 1s/250ms protocol')
    script = Path(__file__).resolve()
    graph_paths = [run/'ros-observer/graph.before.json', run/'ros-observer/graph.after.json']
    inputs = graph_paths+[review_path, module_path, script, protocol_path]
    before = {str(path): identity(path) for path in inputs}
    graph_set_before = sorted(str(path) for path in (run/'ros-observer').glob('graph.*.json'))
    out = (args.output_dir if args.output_dir is not None else ROOT/'runs'/(
        'research_r2b3b4_startup_replay_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))).resolve()
    out.mkdir(parents=True, exist_ok=False)
    for path in graph_paths:
        copy_new(path, out/'input-snapshots/b3_full/ros-observer'/path.name)
    copy_new(review_path, out/'input-snapshots/review/adapter-to-wire-startup-gap.json')
    copy_new(module_path, out/'sources/extnav_startup_gate.py')
    copy_new(script, out/'sources/replay_extnav_startup_gate.py')
    copy_new(protocol_path, out/'configuration/research_extnav_startup_gate.json')
    module = load_module(module_path)
    actual_policy = module.StartupGatePolicy()
    if (actual_policy.continuous_visibility_ns != protocol['continuous_visibility_ns'] or
            actual_policy.maximum_observation_gap_ns != protocol['maximum_observation_gap_ns']):
        raise ValueError('Frozen module default policy differs from revision2 protocol')
    recorded = []
    for phase, path in zip(('before', 'after'), graph_paths):
        graph = json.loads(path.read_text())
        endpoints = graph['endpoints'][TOPIC]['subscribers']
        unchanged = deepcopy(endpoints)
        gate = module.ExtnavStartupGate()  # Deliberately a NEW instance per snapshot.
        step = gate.observe(endpoints, graph['monotonic_ns'])
        recorded.append({
            'classification': 'INDEPENDENT_SINGLE_RECORDED_GRAPH_SNAPSHOT',
            'phase': phase, 'input_path': str(path), 'topic': TOPIC,
            'recorded_graph_monotonic_ns': graph['monotonic_ns'],
            'subscribers_unmodified': unchanged, 'input_endpoints_unchanged': endpoints == unchanged,
            'ready': step.ready, 'reasons': list(step.reasons), 'snapshot': gate.snapshot(),
            'identity_match_valid': gate.snapshot()['matched_endpoint'] is not None,
            'actual_continuous_visibility_duration_established': False,
        })
    controlled = controlled_steps(module)
    review = json.loads(review_path.read_text())
    write_json(out/'recorded-snapshot-evaluations.json', recorded)
    with (out/'synthetic-controlled.steps.jsonl').open('x') as handle:
        for step in controlled:
            handle.write(json.dumps(step, separators=(',', ':'), allow_nan=False)+'\n')
    after = {str(path): identity(path) for path in inputs}
    graph_set_after = sorted(str(path) for path in (run/'ros-observer').glob('graph.*.json'))
    checks = {
        'recorded_before_has_only_observer_subscription': len(recorded[0]['subscribers_unmodified']) == 1
            and recorded[0]['subscribers_unmodified'][0]['node_name'] == 'impact_extnav_disarmed_probe',
        'recorded_before_missing_mavros_endpoint_and_not_ready': not recorded[0]['identity_match_valid']
            and recorded[0]['ready'] is False,
        'recorded_after_exact_mavros_identity_is_valid': recorded[1]['identity_match_valid']
            and recorded[1]['snapshot']['latest_exact_owner_count'] == 1,
        'recorded_after_single_snapshot_does_not_claim_one_second_dwell': recorded[1]['ready'] is False
            and recorded[1]['snapshot']['continuous_visibility_duration_ns'] == 0,
        'recorded_snapshots_never_interpolated_or_chained_for_visibility': all(
            step['snapshot']['continuous_visibility_duration_ns'] == 0 for step in recorded),
        'all_original_endpoint_fields_passed_unmodified': all(step['input_endpoints_unchanged'] for step in recorded),
        'synthetic_controlled_schedule_all_expectations_match': all(step['matches_expectation'] for step in controlled),
        'synthetic_schedule_inputs_not_modified': all(step['input_endpoints_unchanged'] for step in controlled),
        'synthetic_monotonic_fault_latches_without_recovery': controlled[-1]['snapshot']['fault_latched']
            and controlled[-1]['ready'] is False,
        'synthetic_long_same_gid_gaps_reset_visibility': all(not step['ready'] and
            step['snapshot']['continuous_visibility_duration_ns'] == 0 for step in controlled
            if step['phase'] in ('one_second_query_gap_same_gid_resets',
                'old_two_query_contract_first_endpoint', 'old_two_query_contract_second_endpoint_not_ready')),
        'each_synthetic_ready_window_respects_predeclared_gap_budget': all(
            step['snapshot']['last_observation_gap_ns'] <= protocol['maximum_observation_gap_ns']
            for step in controlled if step['ready']),
        'original_seven_key_gap_is_retained_as_unresolved': review['observed_adapter_prefix7_absent_from_wire'] is True
            and len(review['missing_adapter_headers']) == 7 and review['cause'] == 'UNDETERMINED',
        'all_bound_inputs_set_bytes_SHA256_unchanged': before == after and graph_set_before == graph_set_after,
    }
    report = {
        'schema_version': 1, 'status': 'PASS' if all(checks.values()) else 'FAIL',
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'classification': 'OFFLINE_GRAPH_IDENTITY_AND_SYNTHETIC_SCHEDULE_VALIDATION_ONLY',
        'protocol_revision': 2, 'protocol_path': str(protocol_path),
        'protocol_sha256': before[str(protocol_path)]['sha256'],
        'protocol_copy_sha256': identity(out/'configuration/research_extnav_startup_gate.json')['sha256'],
        'checks': checks, 'failed_checks': [key for key, value in checks.items() if not value],
        'input_identities_before': before, 'input_identities_after': after,
        'graph_input_set_before': graph_set_before, 'graph_input_set_after': graph_set_after,
        'recorded_snapshot_results': recorded,
        'controlled_step_count': len(controlled),
        'controlled_expected_ready_count': sum(step['expected_ready'] for step in controlled),
        'frozen_B3_startup_gap': {
            'node_output_count': review['node_output_count'], 'wire_odometry_count': review['wire_odometry_count'],
            'missing_prefix_count': len(review['missing_adapter_headers']), 'cause': review['cause'],
            'fixed_by_this_replay': False,
        },
        'reproduce_from_archived_inputs': [
            '/usr/bin/python3', str(out/'sources/replay_extnav_startup_gate.py'),
            '--run-dir', str(out/'input-snapshots/b3_full'),
            '--review-path', str(out/'input-snapshots/review/adapter-to-wire-startup-gap.json'),
            '--module-path', str(out/'sources/extnav_startup_gate.py'), '--output-dir', '/NEW/OUTPUT/DIRECTORY',
            '--protocol', str(out/'configuration/research_extnav_startup_gate.json'),
        ],
        'limits': [
            'A valid endpoint identity means discovery can match a compatible MAVROS subscription, not callback execution or FCU receipt.',
            'Neither real graph snapshot proves actual one-second continuous visibility; the 90s gap between them is not interpolated.',
            'All dwell/disappearance/GID-transition sequences are explicitly synthetic controlled fixtures.',
            'Revision1 two-distant-query warmup had no maximum observation gap. The original 54-test/11-check passing artifacts remain unchanged and do not validate this revised contract.',
            'Revision2 establishes only bounded-gap sampled visibility; endpoint existence between graph queries remains unproven.',
            'The frozen B3 seven-header startup gap remains unresolved; no original result is rewritten or reclassified.',
            'External TF evidence cannot prove the MAVROS internal TF buffer ready.',
            'No source or transport timestamp is changed; no source frame is queued, replayed, or sent to AP.',
            'No production node/runner behavior changes, ROS nodes, AP/Gazebo, or SQLite are involved.',
        ],
        'outputs': {name: identity(out/name) for name in ('recorded-snapshot-evaluations.json', 'synthetic-controlled.steps.jsonl')},
    }
    write_json(out/'report.json', report)
    print(json.dumps({'status': report['status'], 'output_directory': str(out), 'checks': checks,
                      'controlled_step_count': len(controlled), 'report_sha256': identity(out/'report.json')['sha256']}))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
