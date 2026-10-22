#!/usr/bin/env python3
"""Own only an unarmed, pinned ArduCopter 4.7.1 preparation/smoke process.

No ROS publishers, parameter writes, mode changes, ARM, takeoff, or setpoints.
The only MAVLink sends are parameter reads, heartbeat, and REQUEST_MESSAGE 512.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

WORKSPACE = Path('/home/accelerate/impact_research_ws')
COMMIT = 'dbe792162d06cab66c3475fd5556bf7a120f119e'
REPOSITORY = Path(__file__).resolve().parent.parent


def sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def write(path, value):
    with Path(path).open('x') as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write('\n')


def number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def validate_config(config):
    if config.get('firmware', {}).get('commit') != COMMIT or config['firmware'].get('version') != [4, 7, 1]:
        raise ValueError('Only the pinned official Copter 4.7.1 is supported')
    if config.get('hardware_access') is not False or config.get('flight_commands_allowed') is not False:
        raise ValueError('Hardware and flight commands must be disabled')
    if config.get('target_system_id') != 1 or config.get('target_component_id') != 1:
        raise ValueError('The frozen experiment requires FCU identity 1/1')
    required = config.get('required_parameters')
    if not isinstance(required, dict) or not required:
        raise ValueError('Missing required parameters')
    locked = {'GPS1_TYPE': 0, 'GPS2_TYPE': 0, 'SIM_GPS1_ENABLE': 0, 'SIM_GPS2_ENABLE': 0,
              'VISO_TYPE': 1, 'PLND_ENABLED': 0, 'LOG_DISARMED': 1, 'ARMING_SKIPCHK': 0}
    if any(required.get(name) != value for name, value in locked.items()):
        raise ValueError('Required migration/safety defaults cannot be bypassed')
    if any(not isinstance(name, str) or len(name) > 16 or not number(value)
           for name, value in required.items()):
        raise ValueError('Invalid required parameter name/value')
    for key in ('startup_timeout_s', 'startup_parameter_settle_s', 'parameter_timeout_s', 'observation_s'):
        if not number(config.get(key)) or not 0 < config[key] <= 180:
            raise ValueError(f'Invalid bounded duration: {key}')


def version_matches(raw, config):
    if not isinstance(raw, int) or isinstance(raw, bool):
        return False
    version = [(raw >> 24) & 255, (raw >> 16) & 255, (raw >> 8) & 255]
    return version == config['firmware']['version'] and (raw & 255) == config['firmware']['official_version_type']


def commit_prefix_matches(raw):
    try:
        return bytes(raw).rstrip(b'\x00').decode('ascii') == COMMIT[:8]
    except (TypeError, ValueError, UnicodeDecodeError):
        return False


def param_name(value):
    if isinstance(value, bytes):
        value = value.decode('ascii', errors='strict')
    return value.split('\x00', 1)[0]


def readback_result(records, config):
    counts = {item['param_count'] for item in records}
    indices = {item['param_index'] for item in records}
    values = {item['param_id']: item['param_value'] for item in records}
    complete = len(counts) == 1 and next(iter(counts), 0) > 0 and indices == set(range(next(iter(counts), 0)))
    checks = {name: number(values.get(name)) and values[name] == expected
              for name, expected in config['required_parameters'].items()}
    return {'status': 'PASS' if complete and all(checks.values()) else 'FAIL',
            'complete_parameter_list': complete, 'announced_counts': sorted(counts),
            'unique_indices': len(indices), 'unique_parameter_names': len(values),
            'required_checks': checks,
            'required_actual_values': {name: values.get(name) for name in checks}}


def observe(run, port, config):
    # Executed with the private, recorded AP preparation Python environment.
    from pymavlink import mavutil
    target = config['target_system_id'], config['target_component_id']
    connection = None
    deadline = time.monotonic()+config['startup_timeout_s']
    while time.monotonic() < deadline:
        try:
            connection = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}',
                source_system=245, source_component=191, autoreconnect=False, dialect='ardupilotmega')
            break
        except (OSError, socket.error):
            time.sleep(.2)
    if connection is None:
        raise TimeoutError('SITL telemetry did not open before startup deadline')
    records, versions, heartbeats = [], [], []
    armed_seen = False
    requested = False
    last_heartbeat = 0.0
    first_fcu_heartbeat = None
    list_started = None
    complete_at = None
    total_deadline = time.monotonic()+config['startup_timeout_s']+config['startup_parameter_settle_s']+config['parameter_timeout_s']+config['observation_s']
    try:
        with (run/'mavlink-observations.jsonl').open('x') as output:
            while time.monotonic() < total_deadline:
                now = time.monotonic()
                if first_fcu_heartbeat is not None and list_started is None and now-first_fcu_heartbeat >= config['startup_parameter_settle_s']:
                    connection.mav.param_request_list_send(*target)
                    list_started = now
                if now-last_heartbeat > 1:
                    connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
                    last_heartbeat = now
                message = connection.recv_match(blocking=True, timeout=.2)
                if message is None:
                    continue
                identity = message.get_srcSystem(), message.get_srcComponent()
                if identity != target:
                    continue
                kind = message.get_type()
                if kind not in {'HEARTBEAT', 'AUTOPILOT_VERSION', 'PARAM_VALUE', 'SYS_STATUS', 'STATUSTEXT'}:
                    continue
                raw = message.to_dict()
                entry = {'monotonic_ns': time.monotonic_ns(), 'wall_ns': time.time_ns(),
                         'system_id': identity[0], 'component_id': identity[1], 'type': kind,
                         'packet_sha256': hashlib.sha256(bytes(message.get_msgbuf())).hexdigest(), 'message': raw,
                         'parameter_list_requested': list_started is not None}
                output.write(json.dumps(entry, allow_nan=False)+'\n')
                if kind == 'HEARTBEAT':
                    heartbeats.append(entry)
                    if first_fcu_heartbeat is None:
                        first_fcu_heartbeat = now
                    armed_seen |= bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if armed_seen:
                        raise RuntimeError('Unexpected armed heartbeat in unarmed-only experiment')
                    if not requested:
                        # Read-only MAV_CMD_REQUEST_MESSAGE=512, never mode/arming commands.
                        connection.mav.command_long_send(*target, 512, 0, 148, 0, 0, 0, 0, 0, 0)
                        requested = True
                elif kind == 'AUTOPILOT_VERSION':
                    versions.append(raw)
                elif kind == 'PARAM_VALUE' and list_started is not None:
                    records.append({'param_id': param_name(message.param_id),
                        'param_value': float(message.param_value), 'param_count': int(message.param_count),
                        'param_index': int(message.param_index), 'param_type': int(message.param_type)})
                readback = readback_result(records, config)
                if readback['complete_parameter_list'] and complete_at is None:
                    complete_at = now
                if complete_at is not None and now-complete_at >= config['observation_s']:
                    break
                if list_started is not None and complete_at is None and now-list_started > config['parameter_timeout_s']:
                    break
    finally:
        connection.close()
    readback = readback_result(records, config)
    version_ok = bool(versions) and all(version_matches(item['flight_sw_version'], config) for item in versions)
    commit_ok = bool(versions) and all(commit_prefix_matches(item['flight_custom_version']) for item in versions)
    observation_complete = complete_at is not None and time.monotonic()-complete_at >= config['observation_s']
    result = {'status': 'PASS' if readback['status'] == 'PASS' and version_ok and commit_ok and heartbeats and not armed_seen and observation_complete else 'FAIL',
        'scope': config['scope'], 'flight_validated': False, 'fusion_validated': False,
        'fcu_target': list(target), 'heartbeat_count': len(heartbeats), 'armed_seen': armed_seen,
        'autopilot_version_messages': versions, 'firmware_version_verified': version_ok,
        'firmware_commit_prefix_verified': commit_ok,
        'startup_parameter_settle_s': config['startup_parameter_settle_s'],
        'post_readback_observation_complete': observation_complete, 'parameter_readback': readback,
        'allowed_mavlink_sends': ['HEARTBEAT', 'PARAM_REQUEST_LIST', 'COMMAND_LONG(MAV_CMD_REQUEST_MESSAGE=512, AUTOPILOT_VERSION=148)']}
    write(run/'smoke-result.json', result)
    print(json.dumps({'status': result['status'], 'run_dir': str(run), 'readback': readback}))
    return 0 if result['status'] == 'PASS' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preparation', type=Path)
    parser.add_argument('--config', type=Path, default=REPOSITORY/'config/research_ap471_gate.json')
    parser.add_argument('--install-prefix', type=Path, default=Path(os.environ.get('IMPACT_INSTALL_PREFIX', str(WORKSPACE/'xq_install'))))
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--model', choices=('quad', 'JSON'), default='quad')
    parser.add_argument('--instance', type=int, default=20)
    parser.add_argument('--base-port', type=int, default=5760)
    parser.add_argument('--sim-port-in', type=int, default=9003)
    parser.add_argument('--sim-port-out', type=int, default=9002)
    parser.add_argument('--_observe-run', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--_port', type=int, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    validate_config(config)
    if args._observe_run:
        return observe(args._observe_run, args._port, config)
    if args.preparation is None:
        parser.error('--preparation is required')
    preparation = args.preparation.resolve(strict=True)
    if not preparation.is_relative_to(WORKSPACE/'experiments/results/research_multisensor'):
        raise ValueError('Preparation outside authorized research results')
    proof = json.loads((preparation/'status.json').read_text())
    if proof.get('result') != 'PASS' or proof.get('stage') != 'complete' or proof.get('commit') != COMMIT:
        raise ValueError('A complete pinned preparation PASS is required')
    source, binary = (Path(proof[name]).resolve(strict=True) for name in ('source', 'binary'))
    # Keep the venv launcher path: resolving its final symlink would select the
    # system interpreter and silently discard the private dependency environment.
    python = Path(proof['python']).absolute()
    if not source.is_relative_to(WORKSPACE/'external') or not binary.is_relative_to(WORKSPACE/'build') or not python.parent.resolve(strict=True).is_relative_to(WORKSPACE/'external') or not python.is_file():
        raise ValueError('AP sources/build/Python must remain isolated in research workspace')
    if sha(binary) != proof['binary_sha256']:
        raise ValueError('Compiled binary changed since preparation')
    actual_commit = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if actual_commit != COMMIT:
        raise ValueError('AP source no longer matches pinned commit')
    if subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain=v1', '--untracked-files=all'], text=True).strip():
        raise ValueError('AP source has changes outside the pinned commit')
    actual_submodules = subprocess.check_output(['git', '-C', str(source), 'submodule', 'status', '--recursive'], text=True)
    if actual_submodules != (preparation/'submodule-status.log').read_text() or any(line.startswith(('-', '+', 'U')) for line in actual_submodules.splitlines()):
        raise ValueError('AP submodules changed since pinned preparation')
    if not 0 <= args.instance <= 100 or not 1024 <= args.base_port <= 60000:
        raise ValueError('Invalid isolated instance/base-port')
    install = args.install_prefix.resolve(strict=True)
    if not install.is_relative_to(WORKSPACE):
        raise ValueError('Research install prefix outside workspace')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    run = WORKSPACE/'experiments/results/research_multisensor'/f'ap471_{"smoke" if args.smoke else "prepare_gate"}_{stamp}_{os.getpid()}'
    run.mkdir(exist_ok=False)
    configuration = run/'configuration'
    configuration.mkdir()
    runtime = run/'sitl_runtime'
    runtime.mkdir()
    process = None
    stage, result, error = 'freeze_provenance', 'FAIL', None
    shutdown = {}
    dependencies = [binary, source/'ArduCopter/version.h', source/'libraries/GCS_MAVLink/GCS.cpp',
        source/'ArduCopter/GCS_Copter.cpp', source/'libraries/GCS_MAVLink/GCS_Common.cpp',
        source/'libraries/AP_VisualOdom/AP_VisualOdom.cpp', source/'libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp',
        source/'libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp', source/'libraries/SITL/SITL.cpp',
        source/'libraries/SITL/SIM_GPS.cpp', source/'libraries/AP_Arming/AP_Arming.cpp',
        source/'Tools/autotest/default_params/copter.parm',
        preparation/'status.json', preparation/'submodule-status.log', Path(__file__).resolve(),
        REPOSITORY/'scripts/run_ap471_research_gate.sh', args.config.resolve(), install/'.xq_build_manifest.json']
    if args.model == 'JSON':
        dependencies.append(source/'Tools/autotest/default_params/gazebo-iris.parm')
    before = {}
    try:
        before = {str(path): sha(path) for path in dependencies}
        for i, path in enumerate(dependencies):
            if path != binary:
                (configuration/f'{i:02d}_{path.name}').write_bytes(path.read_bytes())
        params = configuration/'ap471-required.parm'
        with params.open('x') as handle:
            handle.writelines(f'{name} {value}\n' for name, value in config['required_parameters'].items())
        defaults = [source/'Tools/autotest/default_params/copter.parm']
        if args.model == 'JSON':
            defaults.append(source/'Tools/autotest/default_params/gazebo-iris.parm')
        defaults.append(params)
        port = args.base_port+10*args.instance
        command = [str(binary), '-S', '--model', args.model, '--speedup', '1', '--slave', '0', '--wipe',
            '--defaults', ','.join(map(str, defaults)), '--base-port', str(args.base_port), '-I', str(args.instance),
            '--home', '-35.363261,149.165230,584,353', '--sim-address', '127.0.0.1']
        if args.model == 'JSON':
            command += ['--sim-port-in', str(args.sim_port_in), '--sim-port-out', str(args.sim_port_out)]
        write(run/'launch-plan.json', {'command': command, 'cwd': str(runtime), 'telemetry_port': port,
            'install_prefix': str(install), 'preparation': str(preparation), 'firmware_commit': COMMIT,
            'model': args.model, 'scope': config['scope'], 'flight_commands_allowed': False,
            'JSON_requires_external_Gazebo': args.model == 'JSON',
            'JSON_plugin_ports_must_match_caller_world': args.model == 'JSON',
            'readonly_dependency_sha256': before})
        if args.smoke:
            stage = 'owned_sitl_startup'
            for candidate in (port, port+2, port+3):
                with socket.socket() as check:
                    check.bind(('127.0.0.1', candidate))
            with (run/'sitl.log').open('x') as log:
                process = subprocess.Popen(command, cwd=runtime, stdin=subprocess.DEVNULL, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
                write(run/'sitl-process.json', {'pid': process.pid, 'pgid': process.pid, 'command': command})
                stage = 'unarmed_readonly_smoke'
                with (run/'observer.log').open('x') as observer_log:
                    observed = subprocess.run([str(python), str(Path(__file__).resolve()), '--config', str(args.config.resolve()),
                        '--_observe-run', str(run), '--_port', str(port)], stdout=observer_log, stderr=subprocess.STDOUT,
                        timeout=2*config['startup_timeout_s']+config['startup_parameter_settle_s']+config['parameter_timeout_s']+config['observation_s']+15)
                if observed.returncode or process.poll() is not None:
                    raise RuntimeError(f'Unarmed smoke failed: observer={observed.returncode}, SITL={process.poll()}')
        stage, result = 'complete', 'PASS'
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        if process is not None:
            forced = []
            for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
                if process.poll() is not None:
                    break
                os.killpg(process.pid, sig)
                if sig != signal.SIGINT:
                    forced.append(signal.Signals(sig).name)
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
            shutdown = {'exit_code': process.poll(), 'forced_signals': forced}
            if forced or process.poll() not in (0, -signal.SIGINT, 130):
                result, error = 'FAIL', f'Owned SITL shutdown issue: {shutdown}; preceding_error={error}'
        after = {str(path): sha(path) if path.is_file() else None for path in dependencies}
        unchanged = bool(before) and before == after
        if not unchanged:
            result = 'FAIL'
        record = {'status': result, 'last_stage': stage, 'error': error, 'scope': config['scope'],
            'smoke_executed': args.smoke, 'model': args.model, 'firmware_commit': COMMIT,
            'flight_validated': False, 'fusion_validated': False, 'owned_shutdown': shutdown,
            'dependencies_unchanged': unchanged, 'dependency_sha256_before': before, 'dependency_sha256_after': after,
            'hardware_accessed': False, 'limits': config['limits']}
        write(run/'runner-result.json', record)
        print(json.dumps({'status': result, 'stage': stage, 'error': error, 'run_dir': str(run)}))
    return 0 if result == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
