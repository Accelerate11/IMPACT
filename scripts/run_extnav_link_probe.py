#!/usr/bin/env python3
"""Own an unarmed, isolated SITL link diagnostic; never launch flight missions."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

CANONICAL = Path('/mnt/d/CODEX/impact')


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write('\n')


def sha(path):
    value = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1048576), b''):
            value.update(chunk)
    return value.hexdigest()


class Diagnostic:
    PHASE_PREFIX = 'r2b3b2'
    PROTOCOL_RELATIVE = 'config/research_extnav_link_probe.json'
    PRESERVATION_RELATIVE = 'runs/verify_r2b3b2_preserved.py'
    ANALYZER_RELATIVE = 'scripts/analyze_extnav_link_probe.py'
    EXTRA_SITL_DEFAULTS = 'LOG_DISARMED 1\n'
    EXTRA_RUNTIME_RELATIVES = ()
    HELPER_RELATIVES = (
        'scripts/run_extnav_link_probe.py', 'scripts/probe_extnav_ros.py',
        'scripts/probe_mavlink_tcp.py', 'scripts/analyze_extnav_link_probe.py',
        'config/research_extnav_link_probe.json', 'scripts/prepare_livo_transport.py',
    )

    def __init__(self, args):
        self.args = args
        self.workspace = args.workspace_root.resolve(strict=True)
        if self.workspace != Path('/home/accelerate/impact_research_ws'):
            raise ValueError('Only the explicitly authorized research workspace is supported')
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.run = self.workspace/'experiments/results/research_multisensor'/(
            f'{self.PHASE_PREFIX}_{args.source}_{args.load}_{stamp}_{os.getpid()}')
        self.run.mkdir(parents=True, exist_ok=False)
        (self.run/'configuration').mkdir()
        (self.run/'sitl_runtime').mkdir()
        self.groups = []
        self.logs = []
        self.stage = 'preflight'
        self.env = dict(os.environ)
        self.env.update(PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
                        ROS2CLI_NO_DAEMON='1', ROS_LOCALHOST_ONLY='0',
                        RMW_IMPLEMENTATION='rmw_fastrtps_cpp', OMP_NUM_THREADS='2',
                        MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA',
                        GZ_PARTITION=f'impact_{self.PHASE_PREFIX}_{stamp}_{os.getpid()}',
                        ROS_LOG_DIR=str(self.run/'ros_logs'))
        for key in ('ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI', 'FASTDDS_DEFAULT_PROFILES_FILE',
                    'FASTRTPS_DEFAULT_PROFILES_FILE', 'LIBGL_ALWAYS_SOFTWARE',
                    'MESA_LOADER_DRIVER_OVERRIDE', 'GALLIUM_DRIVER', 'EGL_PLATFORM', 'QT_QPA_PLATFORM'):
            self.env.pop(key, None)
        resources = [self.workspace/'xq_install/xq_gz_assets/share/xq_gz_assets/models',
                     Path('/home/accelerate/ardupilot_gazebo/models'),
                     Path('/home/accelerate/ardupilot_gazebo/worlds')]
        resource_path = ':'.join(map(str, resources))
        self.env.update(GZ_SIM_RESOURCE_PATH=resource_path, IGN_GAZEBO_RESOURCE_PATH=resource_path,
                        SDF_PATH=resource_path,
                        GZ_SIM_SYSTEM_PLUGIN_PATH='/home/accelerate/ardupilot_gazebo/build')
        self.source_records = {}

    def command(self, label, command, timeout=120, check=True):
        with (self.run/(label+'.log')).open('x') as handle:
            result = subprocess.run(list(map(str, command)), env=self.env,
                                    stdout=handle, stderr=subprocess.STDOUT, timeout=timeout)
        if check and result.returncode:
            raise RuntimeError(f'{label} exited {result.returncode}')
        return result.returncode

    def start(self, label, command, cwd=None):
        handle = (self.run/(label+'.log')).open('x')
        self.logs.append(handle)
        process = subprocess.Popen(list(map(str, command)), cwd=cwd or self.workspace,
                                   env=self.env, stdin=subprocess.DEVNULL,
                                   stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.groups.append((label, process))
        write(self.run/(label+'.process.json'), {'pid': process.pid, 'pgid': process.pid,
              'command': list(map(str, command)), 'cwd': str(cwd or self.workspace)})
        return process

    def wait_log(self, label, pattern, timeout):
        deadline = time.monotonic()+timeout
        path = self.run/(label+'.log')
        while time.monotonic() < deadline:
            process = next(p for name, p in self.groups if name == label)
            if process.poll() is not None:
                raise RuntimeError(f'{label} exited before readiness ({process.returncode})')
            if pattern in path.read_text(errors='replace'):
                return
            time.sleep(.25)
        raise TimeoutError(f'{label}: {pattern!r} not observed')

    def stop(self):
        records = []
        # Keep both peers and simulation alive until the proxy has drained its
        # already observed bytes. Killing SITL first produces an artificial RST.
        late = {'proxy', 'mavros', 'gazebo', 'sitl'}
        ordered = [(n, p) for n, p in reversed(self.groups) if n not in late]
        for role in ('proxy', 'mavros', 'gazebo', 'sitl'):
            ordered += [(n, p) for n, p in self.groups if n == role]
        for label, process in ordered:
            forced = []
            for sig, wait_s in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=wait_s)
                    # A ros2 launcher can exit before all its owned children.
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    pass
                if sig != signal.SIGINT:
                    forced.append(signal.Signals(sig).name)
            code = process.poll()
            if code is None:
                try:
                    code = process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
            try:
                os.killpg(process.pid, 0)
                alive = True
            except ProcessLookupError:
                alive = False
            records.append({'label': label, 'pid': process.pid, 'exit_code': code,
                            'forced': forced, 'group_still_exists': alive})
        self.groups.clear()
        for handle in self.logs:
            handle.close()
        self.logs.clear()
        if records:
            write(self.run/'shutdown-result.json', {
                'status': 'ISSUES' if any(r['forced'] or r['group_still_exists'] or
                    r['exit_code'] not in (0, 130, -signal.SIGINT) for r in records) else 'PASS',
                'processes': records,
                'scope': 'Owned groups only; cleanup status is separate from communication acceptance'})

    def preserve(self, when):
        self.command('sealed-preservation.'+when,
                     [sys.executable, CANONICAL/self.PRESERVATION_RELATIVE])
        self.command('external-assets.'+when,
                     ['bash', self.workspace/'scripts/audit_external_assets.sh', 'snapshot',
                      self.run/f'external-assets.{when}.sha256'])
        helper = CANONICAL/'evidence/RESEARCH_R2/r2_suite_20261018T165613Z_320/archive_research_r2.py'
        baseline = CANONICAL/'runs/research_bootstrap_20261018/protected.before.json'
        # The retained helper is read-only; no operation is executed on protected files.
        import runpy
        api = runpy.run_path(str(helper))
        _, result = api['compare_protected'](api['read_object'](baseline))
        write(self.run/f'protected-audit.{when}.json', result)
        if result['status'] != 'PASS':
            raise RuntimeError('Protected workspace preservation failed')

    def prepare(self):
        print(json.dumps({'event': 'run_created', 'run_dir': str(self.run),
                          'source': self.args.source, 'load': self.args.load}), flush=True)
        declared = json.loads((CANONICAL/self.PROTOCOL_RELATIVE).read_text())
        formal_cases = {(v['source'], v['load']) for v in declared['cases']}
        pair = (self.args.source, self.args.load)
        if pair not in formal_cases:
            conditional = declared['conditional_case']
            if pair != (conditional['source'], conditional['load']) or not self.args.conditional_on:
                raise ValueError('This case requires the predeclared conditional trigger')
            dependencies = []
            for path, source, load, status in zip(self.args.conditional_on,
                    ('synthetic', 'lio'), ('lean', 'full'), ('PASS', 'FAIL')):
                path = path.resolve(strict=True)
                if path.parent != self.workspace/'experiments/results/research_multisensor':
                    raise ValueError('Conditional triggers must be original diagnostic directories')
                setup = json.loads((path/'run.json').read_text())
                result = json.loads((path/'link-probe-result.json').read_text())
                if (setup['source'], setup['load'], result['status']) != (source, load, status):
                    raise ValueError('Conditional trigger does not match the declared outcome pair')
                dependencies.append({'path': str(path), 'case': setup,
                    'result_status': result['status'], 'result_sha256': sha(path/'link-probe-result.json')})
            write(self.run/'conditional-trigger.json', dependencies)
        elif self.args.conditional_on:
            raise ValueError('Conditional triggers do not apply to this primary case')
        ports_text = subprocess.check_output(['ss', '-H', '-lntup'], text=True)
        (self.run/'ports.before.txt').write_text(ports_text)
        ports = {int(p) for p in re.findall(r':(\d+)\s', ports_text)}
        if ports & {5760, 5761, 9002}:
            raise RuntimeError('An intended SITL/proxy/Gazebo port is already occupied')
        domain = next((d for d in range(70+os.getpid()%20, 100)
                       if not any(7400+250*d <= p <= 7400+250*d+139 for p in ports)), None)
        if domain is None:
            raise RuntimeError('No isolated DDS domain is available')
        self.env['ROS_DOMAIN_ID'] = str(domain)
        self.command('build-freshness', [sys.executable, self.workspace/'scripts/check_build_manifest.py',
                                       '--workspace-root', self.workspace])
        self.preserve('before')
        self.command('transport-setup', [sys.executable, CANONICAL/'scripts/prepare_livo_transport.py',
                    '--initial-peer-range', '64', '--output-dir', self.run/'configuration'])
        self.env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(self.run/'configuration/fastdds-replay.xml')
        self.command('renderer-probe', ['glxinfo', '-B'], timeout=25)
        if 'D3D12 (NVIDIA' not in (self.run/'renderer-probe.log').read_text():
            raise RuntimeError('The requested NVIDIA renderer was not observed')
        for relative in self.HELPER_RELATIVES:
            source = CANONICAL/relative
            target = self.run/'sources'/relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source.read_bytes())
            self.source_records[relative] = {'bytes': source.stat().st_size, 'sha256': sha(source)}
        self.protocol = json.loads((self.run/'sources'/self.PROTOCOL_RELATIVE).read_text())
        if self.args.duration != self.protocol['duration_wall_s']:
            raise ValueError('The formal diagnostic uses the predeclared duration')
        write(self.run/'source-binding.json', self.source_records)
        params = self.workspace/'src/xq_autonomy/config/xq_p4_extnav.parm'
        (self.run/'configuration/unarmed.parm').write_text(params.read_text()+'\n'+self.EXTRA_SITL_DEFAULTS)
        world = self.workspace/'xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_r2b3_range.sdf'
        dependencies = [world, params, self.run/'configuration/unarmed.parm',
                        Path('/home/accelerate/ardupilot/build/sitl/bin/arducopter'),
                        Path('/home/accelerate/ardupilot_gazebo/build/libArduPilotPlugin.so'),
                        Path('/opt/ros/humble/share/mavros/launch/apm_config.yaml'),
                        Path('/opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml')]
        dependencies.extend(self.workspace/relative for relative in self.EXTRA_RUNTIME_RELATIVES)
        write(self.run/'runtime-dependencies.json', {str(p): {'bytes': p.stat().st_size,
              'sha256': sha(p)} for p in dependencies})
        write(self.run/'run.json', {'source': self.args.source, 'load': self.args.load,
              'ros_domain_id': domain, 'gz_partition': self.env['GZ_PARTITION'],
              'duration_wall_s': self.args.duration, 'armed_trial': False,
              'fcu_url': 'tcp://127.0.0.1:5761', 'transparent_target': 'tcp://127.0.0.1:5760',
              'ground_truth_input': False, 'synthetic_is_estimator_evidence': False,
              'host_boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
              'actual_flow_state_updates': 0, 'mission_nodes_started': False,
              'scope': self.protocol['scope']})
        return world

    def proxy_extra_arguments(self):
        return []

    def after_telemetry_request(self):
        pass

    def before_adapter(self):
        pass

    def observer_extra_arguments(self):
        return []

    def after_observer(self):
        pass

    def execute(self):
        world = self.prepare()
        self.stage = 'simulation_startup'
        ap = Path('/home/accelerate/ardupilot')
        defaults = ','.join(map(str, [ap/'Tools/autotest/default_params/copter.parm',
                         ap/'Tools/autotest/default_params/gazebo-iris.parm',
                         self.run/'configuration/unarmed.parm']))
        self.start('sitl', [ap/'build/sitl/bin/arducopter', '-S', '--model', 'JSON',
                   '--speedup', '1', '--slave', '0', '--wipe', '--defaults', defaults,
                   '--sim-address=127.0.0.1', '-I0'], cwd=self.run/'sitl_runtime')
        self.wait_log('sitl', 'SERIAL0 on TCP port 5760', 30)
        self.start('proxy', [sys.executable, self.run/'sources/scripts/probe_mavlink_tcp.py',
                   '--output-dir', self.run/'wire', '--guard-unarmed',
                   '--allow-fcu-startup-ascii',
                   '--max-runtime-s', str(self.args.duration+180), *self.proxy_extra_arguments()])
        self.wait_log('proxy', 'ready', 45)
        self.start('mavros', ['ros2', 'launch', 'mavros', 'apm.launch',
                   'fcu_url:=tcp://127.0.0.1:5761', 'namespace:=uav1/mavros'])
        self.wait_log('sitl', 'Loaded defaults', 45)
        self.start('gazebo', ['gz', 'sim', '-r', '-s', '-v', '3', world])
        self.wait_log('sitl', 'JSON received', 90)
        self.wait_log('mavros', 'Got HEARTBEAT', 60)
        self.stage = 'read_only_telemetry_request'
        telemetry = self.protocol['read_only_telemetry_request']
        self.command('telemetry-request', ['ros2', 'service', 'call', telemetry['service'],
            'mavros_msgs/srv/StreamRate', json.dumps({k: v for k, v in telemetry.items() if k != 'service'})], timeout=25)
        self.after_telemetry_request()
        self.start('bridge', ['ros2', 'run', 'xq_gz_bridge', 'xq_gz_bridge_node', '--ros-args',
                   '--params-file', self.workspace/'xq_install/xq_gz_bridge/share/xq_gz_bridge/config/p4_external_nav.yaml',
                   '-p', 'publish_ground_truth:=false'])
        if self.args.load == 'full':
            self.start('shadow', ['ros2', 'launch', 'xq_sim_bringup', 'xq_r2b3_range.launch.py',
                       'config_file:='+str(self.workspace/'config/research_multisensor_contract.json'),
                       'result_file:='+str(self.run/'shadow-result.json')])
            topics = ['/clock', '/livox/lidar', '/livox/imu', '/localization/odom',
                      '/uav1/mavros/odometry/out', '/uav1/mavros/state', '/xq/p4/extnav/status',
                      '/uas1/mavlink_sink', '/uas1/mavlink_source', '/tf', '/tf_static',
                      '/impact/r2/front/image_raw', '/impact/r2/front/camera_info',
                      '/impact/r2/down/image_raw', '/impact/r2/down/camera_info',
                      '/impact/r2/flow/measurement', '/impact/r2/flow/status',
                      '/impact/r2/calibration_metadata', '/impact/r2b3/down/range',
                      '/impact/r2b3/eval/range_reference', '/impact/r2b3/native/noisy_scan',
                      '/impact/r2b3/eval/reference_scan', '/impact/r2b3/native/range_source',
                      '/impact/r2b3/imu', '/impact/r2b3/noise_metadata']
            self.start('rosbag', ['ros2', 'bag', 'record', '-s', 'sqlite3', '--compression-mode',
                       'none', '--storage-preset-profile', 'resilient', '--max-bag-size', '268435456',
                       '--max-cache-size', '104857600', '-o', self.run/'rosbag', *topics])
        if self.args.source == 'lio':
            for child, parent, xyz in (
                    ('xq_base_link', 'livox_imu', ('0', '0', '0')),
                    ('livox_frame', 'xq_base_link', ('0.04', '0', '0.12'))):
                self.start('tf_'+child, ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                    '--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', parent, '--child-frame-id', child,
                    '--ros-args', '-p', 'use_sim_time:=true'])
            self.start('lio', ['ros2', 'run', 'xq_fast_lio', 'fastlio_mapping', '--ros-args',
                       '--params-file', self.workspace/'xq_install/xq_fast_lio/share/xq_fast_lio/config/xq_p4.yaml',
                       '-p', 'use_sim_time:=true', '-p', 'integrity_geometry.enable:=true',
                       '-p', 'publish.scan_publish_en:=true'])
        self.before_adapter()
        self.start('adapter', ['ros2', 'run', 'xq_autonomy', 'xq_p4_external_nav',
                              '--ros-args', '-p', 'use_sim_time:=false'])
        observer = self.start('observer', [sys.executable, self.run/'sources/scripts/probe_extnav_ros.py',
                       '--output-dir', self.run/'ros-observer', '--source', self.args.source,
                       '--duration', str(self.args.duration), *self.observer_extra_arguments()])
        self.stage = 'unarmed_observation'
        deadline = time.monotonic()+self.args.duration+20
        print(json.dumps({'event': self.stage, 'run_dir': str(self.run)}), flush=True)
        while observer.poll() is None and time.monotonic() < deadline:
            for label, process in self.groups:
                if label != 'observer' and process.poll() is not None:
                    raise RuntimeError(f'Owned {label} exited during observation: {process.returncode}')
            time.sleep(.25)
        if observer.poll() is None:
            raise TimeoutError('Observer exceeded its duration deadline')
        if observer.returncode:
            raise RuntimeError(f'Observer reported failure: {observer.returncode}')
        self.after_observer()
        self.stage = 'closed_data_analysis'
        self.stop()
        code = self.command('analysis', [sys.executable, self.run/'sources'/self.ANALYZER_RELATIVE,
                     '--run-dir', self.run, '--protocol', self.run/'sources'/self.PROTOCOL_RELATIVE],
                     timeout=120, check=False)
        self.stage = 'complete' if code == 0 else 'diagnostic_gate_failed'
        return code


def main(argv=None, diagnostic_class=Diagnostic):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root', type=Path, default=Path('/home/accelerate/impact_research_ws'))
    parser.add_argument('--source', choices=('synthetic', 'lio'), required=True)
    parser.add_argument('--load', choices=('lean', 'full'), required=True)
    parser.add_argument('--duration', type=float, default=90.)
    parser.add_argument('--conditional-on', type=Path, nargs=2, metavar=('LEAN_PASS', 'LIO_FULL_FAIL'))
    args = parser.parse_args(argv)
    diagnostic = diagnostic_class(args)
    def interrupted(signum, frame):
        raise KeyboardInterrupt(signal.Signals(signum).name)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    code, error = 1, None
    try:
        code = diagnostic.execute()
    except BaseException as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        diagnostic.stop()
        try:
            diagnostic.preserve('after')
            before = diagnostic.run/'external-assets.before.sha256'
            after = diagnostic.run/'external-assets.after.sha256'
            if before.read_bytes() != after.read_bytes():
                raise RuntimeError('External asset snapshot mismatch')
            for relative, expected in diagnostic.source_records.items():
                if sha(CANONICAL/relative) != expected['sha256'] or sha(diagnostic.run/'sources'/relative) != expected['sha256']:
                    raise RuntimeError('Diagnostic source changed during this attempt')
            if (diagnostic.run/'runtime-dependencies.json').exists():
                dependencies = json.loads((diagnostic.run/'runtime-dependencies.json').read_text())
                current = {p: {'bytes': Path(p).stat().st_size, 'sha256': sha(Path(p))} for p in dependencies}
                write(diagnostic.run/'runtime-dependencies.after.json', current)
                if current != dependencies:
                    raise RuntimeError('Runtime dependencies changed during this attempt')
            diagnostic.command('post-run-build-freshness', [sys.executable,
                diagnostic.workspace/'scripts/check_build_manifest.py', '--workspace-root', diagnostic.workspace])
        except BaseException as exc:
            code = 14
            error = (error+'; ' if error else '')+f'preservation: {exc}'
        result = {'status': 'PASS' if code == 0 and diagnostic.stage == 'complete' else 'FAIL',
                  'last_stage': diagnostic.stage, 'exit_code': code, 'error': error,
                  'run_dir': str(diagnostic.run), 'armed_trial': False,
                  'flight_regression_repaired': False, 'actual_flow_state_updates': 0}
        write(diagnostic.run/'runner-result.json', result)
        print(json.dumps(result), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
