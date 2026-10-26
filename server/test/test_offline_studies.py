"""Exercise runner lifecycle with tiny independent workers, never real algorithms."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch


RUNNER = Path(__file__).resolve().parents[1] / 'offline_studies.py'
SPEC = importlib.util.spec_from_file_location('impact_offline_runner', RUNNER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

WORKER = '''import os,signal,subprocess,sys,time
from pathlib import Path
child=subprocess.Popen([sys.executable,'-c',
    'import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(60)'])
Path('worker.json').write_text(__import__('json').dumps({'worker':os.getpid(),'child':child.pid}))
signal.signal(signal.SIGTERM,signal.SIG_IGN)
time.sleep(60)
'''


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux process-group lifecycle checks')
class OfflineRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='impact-offline-test-')
        self.base = Path(self.temporary.name)
        self.root = self.base / 'fixture'
        (self.root / 'scripts').mkdir(parents=True)
        self.output = self.base / 'run'
        self.process = None

    def tearDown(self):
        # Only the fixture's recorded session may be cleaned after a failed assertion.
        marker = self.root / 'worker.json'
        if marker.is_file():
            group = json.loads(marker.read_text())['worker']
            try:
                os.killpg(group, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if self.process is not None and self.process.poll() is None:
            self.process.kill()
            self.process.wait(timeout=5)
        self.temporary.cleanup()

    def command(self, *extra):
        return [sys.executable, str(RUNNER), '--root', str(self.root),
                '--output', str(self.output), *extra]

    def write_worker(self, source):
        (self.root / 'scripts/run_shared_scale_study.py').write_text(source)

    def report(self):
        return json.loads((self.output / 'summary.json').read_text())

    def assert_workers_stopped(self):
        marker = json.loads((self.root / 'worker.json').read_text())
        self.assertEqual(MODULE.live_group_members(marker['worker']), [])
        for pid in marker.values():
            stat = Path('/proc') / str(pid) / 'stat'
            if stat.exists():
                self.assertEqual(stat.read_text().rpartition(') ')[2].split()[0], 'Z')

    def test_success_retains_existing_summary_contract(self):
        self.write_worker('print("fixture success")\n')
        result = subprocess.run(self.command('--study', 'shared_scale'), capture_output=True,
                                text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = self.report()
        self.assertEqual(report['status'], 'PASS')
        self.assertEqual(len(report['studies']), 1)
        self.assertEqual(report['studies'][0]['returncode'], 0)
        self.assertIn('fixture success', (self.output / 'shared_scale.log').read_text())

    def test_nonzero_study_cannot_pass(self):
        self.write_worker('raise SystemExit(7)\n')
        result = subprocess.run(self.command('--study', 'shared_scale'), capture_output=True,
                                text=True, timeout=10)
        self.assertEqual(result.returncode, 1)
        self.assertEqual(self.report()['status'], 'FAIL')
        self.assertEqual(self.report()['studies'][0]['returncode'], 7)

    def test_timeout_stops_descendants_even_if_they_ignore_term(self):
        self.write_worker(WORKER)
        result = subprocess.run(self.command('--study', 'shared_scale', '--timeout', '.5'),
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        row = self.report()['studies'][0]
        self.assertEqual(row['status'], 'FAIL')
        self.assertTrue(row['timed_out'])
        self.assertIn('SIGKILL', row['cleanup']['signals'])
        self.assertEqual(row['cleanup']['owned_remaining'], [])
        self.assert_workers_stopped()

    def test_ctrl_c_stops_owned_group_and_marks_unstarted_studies(self):
        self.write_worker(WORKER)
        with (self.base / 'console.log').open('w') as console:
            self.process = subprocess.Popen(self.command('--study', 'all'), stdout=console,
                                            stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 5
            while not (self.root / 'worker.json').exists() and time.monotonic() < deadline:
                time.sleep(.02)
            self.assertTrue((self.root / 'worker.json').exists())
            os.kill(self.process.pid, signal.SIGINT)
            self.assertEqual(self.process.wait(timeout=10), 130)
        report = self.report()
        self.assertEqual(report['status'], 'INTERRUPTED')
        self.assertEqual([row['status'] for row in report['studies']],
                         ['INTERRUPTED'] + ['NOT_RUN'] * 5)
        self.assertEqual(report['studies'][0]['cleanup']['owned_remaining'], [])
        self.assert_workers_stopped()

    def test_launch_failure_retains_log_and_marks_remaining_not_run(self):
        with patch.object(MODULE.subprocess, 'Popen', side_effect=OSError('fixture launch failure')):
            with contextlib.redirect_stdout(io.StringIO()):
                code = MODULE.main(['--root', str(self.root), '--output', str(self.output)])
        self.assertEqual(code, 1)
        report = self.report()
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual([row['status'] for row in report['studies']], ['FAIL'] + ['NOT_RUN'] * 5)
        self.assertTrue(report['studies'][0]['launch_failed'])
        self.assertIn('fixture launch failure', (self.output / 'shared_scale.log').read_text())

    def test_ctrl_c_between_results_still_writes_summary(self):
        self.write_worker('print("fixture success")\n')
        with patch.object(MODULE, 'print', side_effect=KeyboardInterrupt, create=True):
            code = MODULE.main(['--root', str(self.root), '--output', str(self.output)])
        self.assertEqual(code, 130)
        report = self.report()
        self.assertEqual(report['status'], 'INTERRUPTED')
        self.assertEqual([row['status'] for row in report['studies']], ['PASS'] + ['NOT_RUN'] * 5)

    def test_invalid_timeouts_are_rejected_before_creating_outputs(self):
        for value in ('', '0', '-1', 'nan', 'inf', '-inf'):
            with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:
                    MODULE.main(['--output', str(self.output), '--timeout=' + value])
                self.assertEqual(caught.exception.code, 2)
                self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()
