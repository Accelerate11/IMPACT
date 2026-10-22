#!/usr/bin/env python3
"""Compile an unchanged AP timestamp algorithm in isolation; never run AP."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path('/mnt/d/CODEX/impact')
PROTOCOL = ROOT/'config/research_extnav_transport_study.json'
HARNESS = '''#include <cstdint>
#include <iostream>
#include "JitterCorrection.h"
// Static storage mirrors the required zero initialization of algorithm state.
static JitterCorrection corrector;
int main() {
    uint64_t remote, arrival;
    while (std::cin >> remote >> arrival) {
        std::cout << corrector.correct_offboard_timestamp_usec(remote, arrival)
                  << " " << corrector.get_link_offset_usec() << "\\n";
    }
    return std::cin.eof() ? 0 : 2;
}
'''


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write('\n')


def reference(pairs, max_lag_ms=500, convergence_loops=100):
    """Independent integer rendition; state is fresh for every case."""
    offset, minimum, counter = None, 0, 0
    output = []
    for remote, arrival in pairs:
        if any(type(value) is not int or not 0 <= value < 2**63 for value in (remote, arrival)):
            raise ValueError('Only nonnegative signed-int64-compatible fixtures are supported')
        difference = arrival-remote
        offset = difference if offset is None or difference < offset else offset
        estimate = remote+offset
        if estimate+max_lag_ms*1000 < arrival:
            estimate = arrival-max_lag_ms*1000
            offset = estimate-remote
        minimum = difference if counter == 0 else min(minimum, difference)
        counter += 1
        if counter == convergence_loops:
            offset, counter = minimum, 0
        if estimate < 0:
            raise ValueError('Fixture would wrap the C++ unsigned return value')
        output.append((estimate, offset))
    return output


def cases(count=240):
    epoch, local_origin = 1_790_000_000_000_000, 10_000_000
    output = {}
    for name in ('nominal_jitter', 'wall_backstep_400ms', 'wall_forwardstep_400ms',
                 'host_fcu_rate_mismatch', 'constant_unobservable_80ms_delay',
                 'stale_origin_with_fresh_transport_stamp'):
        samples = []
        for i in range(count):
            host = i*100_000
            rate = 62_500 if name == 'host_fcu_rate_mismatch' else 100_000
            generated = local_origin+i*rate
            delay = 80_000 if name == 'constant_unobservable_80ms_delay' else 5_000
            if name == 'nominal_jitter':
                delay += (0, 13_000, 2_000, 8_000, 1_000)[i % 5]
            arrival = generated+delay
            wall_step = (-400_000 if name == 'wall_backstep_400ms' else
                         400_000 if name == 'wall_forwardstep_400ms' else 0) if i >= 60 else 0
            if name == 'stale_origin_with_fresh_transport_stamp':
                generated = local_origin  # Deliberately frozen original acquisition origin.
            samples.append({'index': i, 'raw_remote_usec': epoch+host+wall_step,
                'mapped_remote_usec': epoch+host, 'arrival_local_usec': arrival,
                'known_synthetic_acquisition_local_usec': generated,
                'input_scope': 'CONTROLLED_SYNTHETIC_FIXTURE_NOT_RECORDED_AP_OR_SENSOR_DATA'})
        output[name] = samples
    return output


def execute(output):
    protocol = json.loads(PROTOCOL.read_text())
    definition = protocol['ap_corrector_study']
    source_paths = [Path(definition['original_cpp']), Path(definition['original_header']), PROTOCOL, Path(__file__).absolute()]
    if any(path.is_symlink() or not path.is_file() for path in source_paths):
        raise ValueError('Regular source files required')
    before = {str(path): {'bytes': path.stat().st_size, 'sha256': sha(path)} for path in source_paths}
    output.mkdir(parents=True, exist_ok=False)
    inputs = output/'inputs'
    inputs.mkdir()
    for path in source_paths:
        shutil.copy2(path, inputs/path.name)
    stub = inputs/'stubs/AP_HAL/AP_HAL.h'
    stub.parent.mkdir(parents=True)
    stub.write_text('#pragma once\n#include <cstdint>\n')
    (inputs/'harness.cpp').write_text(HARNESS)
    compiler = shutil.which('g++')
    if compiler is None:
        raise RuntimeError('g++ is required for the isolated original-source comparison')
    command = [compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I'+str(inputs),
        '-I'+str(inputs/'stubs'), str(inputs/'harness.cpp'), str(inputs/'JitterCorrection.cpp'),
        '-o', str(output/'isolated-corrector')]
    with (output/'compile.log').open('x') as handle:
        subprocess.run(command, check=True, stdout=handle, stderr=subprocess.STDOUT, timeout=30)
    checks, results = {}, {}
    for name, samples in cases(definition['rows_per_case']).items():
        result = {'scope': 'Isolated unchanged C++ algorithm, not AP runtime/EKF/backend evidence', 'variants': {}}
        for variant, field in (('raw', 'raw_remote_usec'), ('mapped', 'mapped_remote_usec')):
            pairs = [(row[field], row['arrival_local_usec']) for row in samples]
            completed = subprocess.run([str(output/'isolated-corrector')],
                input=''.join(f'{remote} {arrival}\n' for remote, arrival in pairs),
                text=True, capture_output=True, check=True, timeout=15)
            actual = [tuple(map(int, line.split())) for line in completed.stdout.splitlines()]
            expected = reference(pairs, definition['max_lag_ms'], definition['convergence_loops'])
            checks[name+':'+variant+':cpp_reference_exact'] = actual == expected
            corrected = [value[0] for value in actual]
            lag = [row['arrival_local_usec']-value for row, value in zip(samples, corrected)]
            error = [value-row['known_synthetic_acquisition_local_usec'] for row, value in zip(samples, corrected)]
            checks[name+':'+variant+':output_count'] = len(corrected) == len(samples)
            checks[name+':'+variant+':structural_lag_range'] = bool(lag) and all(0 <= value <= 500_000 for value in lag)
            result['variants'][variant] = {'count': len(actual),
                'remote_nonincreasing_steps': sum(b[0] <= a[0] for a, b in zip(pairs, pairs[1:])),
                'corrected_nonincreasing_steps': sum(b <= a for a, b in zip(corrected, corrected[1:])),
                'corrected_arrival_lag_usec_min_max': [min(lag), max(lag)],
                'known_synthetic_acquisition_error_usec_min_max': [min(error), max(error)],
                'distinct_offset_count': len({value[1] for value in actual})}
            with (output/(name+'.'+variant+'.jsonl')).open('x') as handle:
                for row, (corrected_us, offset) in zip(samples, actual):
                    handle.write(json.dumps({**row, 'variant': variant, 'corrected_usec': corrected_us,
                        'offset_after_call_usec': offset}, separators=(',', ':'))+'\n')
        results[name] = result
    checks['backstep_can_regress_raw_corrected_time'] = results['wall_backstep_400ms']['variants']['raw']['corrected_nonincreasing_steps'] > 0
    checks['mapped_avoids_wallstep_corrected_regressions_in_named_fixtures'] = all(
        results[name]['variants']['mapped']['corrected_nonincreasing_steps'] == 0
        for name in ('wall_backstep_400ms', 'wall_forwardstep_400ms'))
    checks['constant_delay_hidden_despite_zero_corrected_arrival_lag'] = (
        results['constant_unobservable_80ms_delay']['variants']['mapped']['corrected_arrival_lag_usec_min_max'] == [0, 0]
        and results['constant_unobservable_80ms_delay']['variants']['mapped']['known_synthetic_acquisition_error_usec_min_max'] == [80_000, 80_000])
    checks['stale_origin_not_detected_by_timestamp_corrector'] = (
        results['stale_origin_with_fresh_transport_stamp']['variants']['mapped']['corrected_arrival_lag_usec_min_max'] == [0, 0]
        and results['stale_origin_with_fresh_transport_stamp']['variants']['mapped']['known_synthetic_acquisition_error_usec_min_max'][1] > 20_000_000)
    after = {str(path): {'bytes': path.stat().st_size, 'sha256': sha(path)} for path in source_paths}
    checks['bound_original_sources_unchanged'] = before == after
    report = {'schema_version': 1, 'status': 'PASS' if all(checks.values()) else 'FAIL',
        'checks': checks, 'failed_checks': [name for name, value in checks.items() if not value],
        'results': results, 'originals_before': before, 'originals_after': after,
        'compiler_version': subprocess.check_output([compiler, '--version'], text=True).splitlines()[0],
        'compile_command': command, 'candidate_sent_to_AP': False,
        'actual_flow_state_updates': 0, 'flight_regression_repaired': False,
        'scope': definition['interpretation'], 'synthetic_fixture_provenance': True,
        'source_copy_identity': {path.name: sha(inputs/path.name) == before[str(path)]['sha256'] for path in source_paths}}
    write(output/'summary.json', report)
    print(json.dumps({'status': report['status'], 'checks': len(checks), 'failed_checks': report['failed_checks'], 'output_directory': str(output)}))
    return 0 if report['status'] == 'PASS' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args(argv)
    output = args.output_dir or ROOT/'runs'/('research_r2b3b4_ap_time_study_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    return execute(output.absolute())


if __name__ == '__main__':
    raise SystemExit(main())
