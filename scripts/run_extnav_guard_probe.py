#!/usr/bin/env python3
"""R2B3B3 unarmed clock guard and FCU sampled-readiness experiment."""
import sys
import time

from run_extnav_link_probe import Diagnostic, main, sha, write


class GuardDiagnostic(Diagnostic):
    PHASE_PREFIX = 'r2b3b3'
    PROTOCOL_RELATIVE = 'config/research_extnav_clock_guard.json'
    PRESERVATION_RELATIVE = 'runs/verify_r2b3b3_preserved.py'
    ANALYZER_RELATIVE = 'scripts/analyze_extnav_guard_probe.py'
    EXTRA_SITL_DEFAULTS = 'LOG_DISARMED 1\nPLND_ENABLED 0\n'
    EXTRA_RUNTIME_RELATIVES = (
        'xq_install/.xq_build_manifest.json',
        'xq_install/xq_autonomy/lib/python3.10/site-packages/xq_autonomy/p4_external_nav_node.py',
        'xq_install/xq_autonomy/lib/python3.10/site-packages/xq_autonomy/extnav_clock_guard.py',
    )
    HELPER_RELATIVES = (
        'scripts/run_extnav_guard_probe.py', 'scripts/run_extnav_link_probe.py',
        'scripts/probe_extnav_ros.py', 'scripts/probe_mavlink_tcp.py',
        'scripts/monitor_extnav_readiness.py', 'scripts/query_extnav_parameter_proof.py',
        'scripts/analyze_extnav_guard_probe.py', 'scripts/analyze_extnav_link_probe.py',
        'scripts/prepare_livo_transport.py', 'config/research_extnav_clock_guard.json',
        'src/xq_autonomy/xq_autonomy/extnav_clock_guard.py',
        'src/xq_autonomy/xq_autonomy/p4_external_nav_node.py',
    )

    def prepare(self):
        world = super().prepare()
        identities = {}
        for name in ('p4_external_nav_node.py', 'extnav_clock_guard.py'):
            relative = 'src/xq_autonomy/xq_autonomy/' + name
            installed = self.workspace/'xq_install/xq_autonomy/lib/python3.10/site-packages/xq_autonomy'/name
            identities[name] = {
                'frozen_source_sha256': self.source_records[relative]['sha256'],
                'installed_sha256': sha(installed),
                'installed_path': str(installed),
            }
            identities[name]['matches'] = (
                identities[name]['frozen_source_sha256'] == identities[name]['installed_sha256'])
        write(self.run/'installed-source-identity.json', identities)
        if not all(row['matches'] for row in identities.values()):
            raise RuntimeError('Installed guard/adapter differ from the frozen canonical sources')
        return world

    def proxy_extra_arguments(self):
        return ['--allow-sitl-irlock-bootstrap-line']

    def after_telemetry_request(self):
        self.stage = 'read_only_parameter_proof'
        self.command('parameter-proof', [sys.executable,
            self.run/'sources/scripts/query_extnav_parameter_proof.py',
            '--output', self.run/'configuration/fcu-parameter-proof.json',
            '--config', self.run/'sources'/self.PROTOCOL_RELATIVE], timeout=55)

    def before_adapter(self):
        self.readiness = self.start('readiness', [sys.executable,
            self.run/'sources/scripts/monitor_extnav_readiness.py',
            '--output-dir', self.run/'readiness',
            '--duration', str(self.args.duration+self.protocol['readiness_extra_duration_s']),
            '--parameter-proof', self.run/'configuration/fcu-parameter-proof.json',
            '--config', self.run/'sources'/self.PROTOCOL_RELATIVE])

    def observer_extra_arguments(self):
        case = next(v for v in self.protocol['cases'] if
                    (v['source'], v['load']) == (self.args.source, self.args.load))
        return ['--pause-after-s', str(case['source_pause_after_s']),
                '--pause-duration-s', str(case['source_pause_duration_s'])]

    def after_observer(self):
        # Keep FCU telemetry alive while the read-only monitor completes its
        # extra window, including loss of a finished synthetic source.
        self.stage = 'readiness_tail_observation'
        deadline = time.monotonic()+30
        while self.readiness.poll() is None and time.monotonic() < deadline:
            for label, process in self.groups:
                if label not in {'observer', 'readiness'} and process.poll() is not None:
                    raise RuntimeError(f'Owned {label} exited during readiness tail: {process.returncode}')
            time.sleep(.1)
        for label, process in self.groups:
            if label not in {'observer', 'readiness'} and process.poll() is not None:
                raise RuntimeError(f'Owned {label} exited during readiness tail: {process.returncode}')
        if self.readiness.poll() is None or self.readiness.returncode != 0:
            raise RuntimeError('The read-only readiness monitor did not finish safely')


if __name__ == '__main__':
    raise SystemExit(main(diagnostic_class=GuardDiagnostic))
