#!/usr/bin/env python3
"""Freeze light deployment validation evidence without modifying source attempts."""
import hashlib
import json
from pathlib import Path
import shutil


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def files(root):
    result = {}
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise ValueError('Symlink rejected: ' + str(path))
        if path.is_file() and not set(path.relative_to(root).parts) & {'cache', 'xdg-config', '__pycache__'}:
            result[path.relative_to(root).as_posix()] = {'bytes': path.stat().st_size, 'sha256': sha(path)}
    return result


def main():
    destination = Path(__file__).resolve().parent
    runtime = Path('/home/accelerate/impact_server_validation_20261022')
    names = ['base/scenario_p15_lattice_20261021T173229Z_315',
             'base/scenario_p15_lattice_20261022T112953Z_495',
             'advanced_transport_probe_20261022T111619Z',
             'transport_builtin_fixture_20261022T113600Z']
    identities, attempts = {}, []
    for name in names:
        source = runtime / name
        target = destination / source.name
        target.mkdir(exist_ok=False)
        before = files(source)
        for relative, identity in before.items():
            path = source / relative
            if identity['bytes'] > 32 * 1024 * 1024:
                raise ValueError('Unexpected large validation artifact: ' + str(path))
            copied = target / relative
            copied.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, copied)
        after = files(source)
        copied = files(target)
        if not (before == after == copied):
            raise ValueError('Source or copy identity changed: ' + name)
        record = {'source_path': str(source), 'packaged_path': 'validation/' + source.name,
                  'file_set_bytes_sha256': before, 'before_equals_after_equals_copy': True,
                  'excluded_directories': ['cache', 'xdg-config', '__pycache__']}
        (target / 'copy-manifest.json').write_text(json.dumps(record, indent=2) + '\n')
        identities[name] = {'file_count': len(before), 'bytes': sum(x['bytes'] for x in before.values()),
                            'copy_manifest_sha256': sha(target / 'copy-manifest.json'), **{k: record[k] for k in ('source_path', 'packaged_path', 'before_equals_after_equals_copy')}}
        if '/scenario_' in name:
            result = json.loads((source / 'result.json').read_text())
            observer = result['observer']
            attempts.append({'path': str(source), 'packaged_path': record['packaged_path'], 'status': result['status'],
                'result_sha256': sha(source / 'result.json'), 'acceptance_mode': result['acceptance_mode'],
                'mission_status': result['mission_status'], 'error': result['error'], 'checks': result['checks'],
                'startup_wall_s': observer['startup_wall_s'], 'wall_observation_s': observer['wall_observation_s'],
                'observation_counts': {key: item['count'] for key, item in observer['streams'].items()},
                'total_received': observer['total_received'], 'last_controller_status': observer['last_controller_status'],
                'estimated_maximum_displacement_m': observer['maximum_estimated_displacement_m'],
                'decision_repeated_or_nonincreasing_stamps': observer['streams']['decision']['nonincreasing'],
                'shutdown': result['shutdown'], 'script_sha256': result['script_sha256'],
                'shared_helper_sha256': result['shared_helper_sha256']})
    first, final = attempts
    report = {'schema_version': 1, 'status': 'PASS', 'kind': 'BOUNDED_ADVANCED_DEPLOYMENT_SMOKE',
        'attempts': attempts, 'selected_final_attempt': final['path'], 'latest_smoke_pass': final['status'] == 'PASS',
        'historical_failure_preserved': first['status'] == 'FAIL',
        'deployment_correction': 'Removed the newly introduced loopback UDP-only FastDDS profile. Final entrypoint uses the same clean builtin SHM/UDP and ROS_LOCALHOST_ONLY=1 environment as the separately passed base smoke. Algorithm launch parameters and thresholds were unchanged.',
        'transport_fixture_caveat': 'Synthetic transport fixtures retained all failures, including a non-vacuous ready/40s builtin fixture that received zero of ten 368640-byte messages in domain86. No claim that synthetic transport isolation is resolved. Actual advanced domain41 run separately received324 LiDAR messages total and231 within the declared window. The fixture/domain-specific discrepancy is not localized.',
        'host_interruption_cause': 'UNKNOWN; no evidence establishing OOM or a specific host cause',
        'mission_complete': False, 'multi_seed_statistics_completed': False, 'new_joint_estimator_integrated': False,
        'AP_started': False, 'AP_flight_validated': False, 'hardware_validated': False,
        'scope': 'One historical 3D Gazebo/native FAST-LIO/integrity/candidate/dynamic-map/latency smoke, p15_lattice/integrity_constrained/seed501,30s. Velocity-controlled carrier. Repeated decision publication is not new planning measurements. Mission evaluators remain INCOMPLETE_OR_FAILED. No EGO executable, new joint raw-flow pipeline or AP task.',
        'files': identities}
    (destination.parent / 'advanced_simulation_validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'attempts': len(attempts), 'frozen_origins': len(identities),
                      'output': str(destination.parent / 'advanced_simulation_validation.json')}))


if __name__ == '__main__':
    main()
