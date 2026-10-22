"""Pure-Python checks for unarmed guard diagnostic evidence boundaries."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest


MODULE = Path(__file__).resolve().parents[1] / 'analyze_extnav_guard_probe.py'
spec = importlib.util.spec_from_file_location('guard_probe_analysis', MODULE)
api = importlib.util.module_from_spec(spec)
spec.loader.exec_module(api)
NS = 1_000_000_000


class ParameterTests(unittest.TestCase):
    def fixture(self):
        required = {'VISO_TYPE': 2, 'PLND_ENABLED': 0, 'LOG_DISARMED': 1}
        proof = {'schema_version': 1, 'status': 'PASS', 'read_only': True,
            'parameter_writes': 0, 'flight_commands': 0,
            'method': 'mavros_ros2_GetParameters', 'service': '/uav1/mavros/param/get_parameters',
            'target_system_id': 1, 'target_component_id': 1, 'config_sha256': 'frozen',
            'parameters': {name: {'success': True, 'ros_type': 2, 'integer_value': value}
                           for name, value in required.items()}}
        wire = [{'kind': 'frame', 'direction': 'from_fcu', 'msgid': 22, 'seq': i,
            'sysid': 1, 'compid': 1, 'monotonic_ns': i + 1,
            'fields': {'param_id': name, 'param_value': value}}
            for i, (name, value) in enumerate(required.items())]
        return proof, required, wire, dict(required)

    def check(self, fixture):
        return api.parameter_checks(*fixture, 'frozen')[0]

    def test_read_only_proof_wire_and_bin_parameters_match(self):
        self.assertTrue(all(self.check(self.fixture()).values()))

    def test_same_parameter_from_wrong_wire_component_fails(self):
        data = self.fixture()
        data[2][1]['compid'] = 2
        self.assertFalse(self.check(data)['wire_parameter:PLND_ENABLED'])

    def test_plnd_enabled_cannot_claim_visual_odom_health(self):
        data = self.fixture()
        data[0]['parameters']['PLND_ENABLED']['integer_value'] = 1
        data[2][1]['fields']['param_value'] = 1
        data[3]['PLND_ENABLED'] = 1
        checks = self.check(data)
        self.assertFalse(checks['proof_parameter:PLND_ENABLED'])
        self.assertFalse(checks['wire_parameter:PLND_ENABLED'])
        self.assertFalse(checks['fcu_bin_parameter:PLND_ENABLED'])

    def test_a_later_disagreeing_parameter_is_not_hidden(self):
        data = self.fixture()
        bad = copy.deepcopy(data[2][0])
        bad['fields']['param_value'] = 0
        data[2].append(bad)
        self.assertFalse(self.check(data)['wire_parameter:VISO_TYPE'])


class FaultTests(unittest.TestCase):
    def fixture(self):
        fault = {'inputs': {'monotonic_ns': 10 * NS, 'transport_stamp_ns': 19_900_000_000},
                 'last_accepted_transport_stamp_ns': 20 * NS}
        state = {'clock_fault_latched': True, 'healthy': False, 'local_clock_ready': False,
                 'output_count': 9, 'clock_guard': {'fault_latched': True, 'first_fault': fault}}
        statuses = [{'monotonic_ns': 10 * NS + i, 'status': copy.deepcopy(state)} for i in (1, 2)]
        monitor = [{'monotonic_ns': 10 * NS + 3, 'status': copy.deepcopy(state)}]
        readiness = [{'monotonic_ns': 10 * NS + 4, 'data': {'ready': False, 'clock_fault_latched': True}}]
        health = [{'monotonic_ns': 11 * NS, 'combined_backend_healthy': False}]
        outgoing = [{'time_usec': 20_000_000}]
        return statuses, monitor, readiness, health, outgoing, [{'stamp_ns': 20 * NS}]

    def test_expected_latched_fault_is_a_diagnostic_pass(self):
        checks, result, stamp = api.fault_analysis(*self.fixture())
        self.assertTrue(all(checks.values()))
        self.assertEqual(result['status'], 'PASS')
        self.assertEqual(stamp, 10 * NS)

    def test_later_output_or_first_fault_changes_fail(self):
        for field, value, check in (
                ('output_count', 10, 'node_output_count_does_not_increase_after_fault'),
                ('healthy', True, 'node_healthy_and_clock_ready_stay_false')):
            data = self.fixture()
            data[0][-1]['status'][field] = value
            self.assertFalse(api.fault_analysis(*data)[0][check])
        data = self.fixture()
        data[0][-1]['status']['clock_guard']['first_fault']['inputs']['monotonic_ns'] += 1
        self.assertFalse(api.fault_analysis(*data)[0]['first_fault_payload_permanently_unchanged'])

    def test_rejected_or_newer_transport_stamp_must_not_reach_wire(self):
        data = self.fixture()
        data[4].append({'time_usec': 20_000_001})
        self.assertFalse(api.fault_analysis(*data)[0]['wire_never_exceeds_last_accepted_transport_stamp'])
        data = self.fixture()
        data[4].append({'time_usec': 19_900_000})
        self.assertFalse(api.fault_analysis(*data)[0]['rejected_candidate_usec_not_on_wire'])

    def test_monitor_cannot_recover_and_fcu_must_end_unhealthy(self):
        data = self.fixture()
        data[2][0]['data']['ready'] = True
        data[3][-1]['combined_backend_healthy'] = True
        checks = api.fault_analysis(*data)[0]
        self.assertFalse(checks['monitor_ready_stays_false_after_seeing_fault'])
        self.assertFalse(checks['final_sampled_fcu_health_is_false_after_fault'])

    def test_no_fault_is_not_observed_and_not_a_rejection_proof(self):
        checks, result, stamp = api.fault_analysis([], [], [], [], [], [])
        self.assertEqual(checks, {})
        self.assertEqual(result['status'], 'NOT_OBSERVED')
        self.assertIsNone(stamp)

    def test_monitor_only_fault_is_incomplete_evidence(self):
        data = list(self.fixture())
        data[0] = []
        checks, result, _ = api.fault_analysis(*data)
        self.assertFalse(checks['fault_primary_observer_evidence_present'])
        self.assertEqual(result['status'], 'INCOMPLETE')


class PauseTests(unittest.TestCase):
    def fixture(self):
        case = {'source': 'synthetic', 'load': 'lean'}
        protocol = {'cases': [{**case, 'source_pause_after_s': 20., 'source_pause_duration_s': 2.}],
            'clock_guard': {'warmup_ns': 5 * NS, 'max_source_age_ns': 350_000_000}}
        events = [
            {'kind': 'source_published', 'monotonic_ns': 19 * NS, 'stamp_ns': 99},
            {'kind': 'source_pause_started', 'monotonic_ns': 20 * NS,
             'pause_duration_ns': 2 * NS, 'elapsed_since_first_source_ns': 20 * NS},
            {'kind': 'source_pause_ended', 'monotonic_ns': 22 * NS, 'elapsed_since_first_source_ns': 22 * NS},
            {'kind': 'source_published', 'monotonic_ns': 22_100_000_000, 'stamp_ns': 100}]
        def row(stamp, ready, stale=False):
            return {'monotonic_ns': stamp, 'status': {'local_source_ready': not stale,
                'local_clock_ready': ready, 'healthy': ready,
                'clock_guard': {'source_age_monotonic_ns': 500_000_000 if stale else 0,
                    'last_accepted_source_stamp_ns': 99 if stale else 100,
                    'continuous_valid_start_monotonic_ns': 22_100_000_000,
                    'last_evaluated_monotonic_ns': stamp}}}
        statuses = [row(20_500_000_000, False, True), row(22_200_000_000, False), row(27_100_000_000, True)]
        readiness = [{'monotonic_ns': 22_300_000_000, 'data': {'ready': False}}]
        health = [{'monotonic_ns': 21 * NS, 'combined_backend_healthy': False}]
        return [case, protocol, events, statuses, readiness, health, None]

    def test_isolated_pause_expires_and_restarts_full_warmup(self):
        result = api.pause_analysis(*self.fixture())
        self.assertEqual(result['status'], 'PASS', result)
        self.assertTrue(all(result['checks'].values()))

    def test_fault_before_or_during_recovery_is_preempted(self):
        for stamp in (10 * NS, 23 * NS):
            data = self.fixture()
            data[-1] = stamp
            result = api.pause_analysis(*data)
            self.assertEqual(result['status'], 'PREEMPTED')
            self.assertTrue(result['recovery_is_not_claimed'])

    def test_incomplete_pause_pair_is_not_pass(self):
        data = self.fixture()
        data[2] = [row for row in data[2] if row['kind'] != 'source_pause_ended']
        self.assertEqual(api.pause_analysis(*data)['status'], 'INCOMPLETE')

    def test_early_ready_and_no_stale_local_rejection_fail(self):
        data = self.fixture()
        data[3][1]['status']['local_clock_ready'] = True
        data[3][0]['status']['local_source_ready'] = True
        result = api.pause_analysis(*data)
        self.assertEqual(result['status'], 'FAIL')
        self.assertFalse(result['checks']['local_not_ready_during_new_warmup'])
        self.assertFalse(result['checks']['local_receipt_freshness_expires_during_pause'])


class IdentityAndWriterTests(unittest.TestCase):
    def test_installed_identity_requires_actual_runtime_hashes(self):
        source = {'bytes': 3, 'sha256': 'a' * 64}
        binding, identity, dependencies = {}, {}, {}
        for name in ('p4_external_nav_node.py', 'extnav_clock_guard.py'):
            path = '/installed/' + name
            binding['src/xq_autonomy/xq_autonomy/' + name] = dict(source)
            dependencies[path] = dict(source)
            identity[name] = {'matches': True, 'frozen_source_sha256': source['sha256'],
                             'installed_sha256': source['sha256'], 'installed_path': path}
        self.assertTrue(all(api.runtime_identity_checks(identity, binding, dependencies, dependencies).values()))
        current = copy.deepcopy(dependencies)
        current['/installed/extnav_clock_guard.py']['sha256'] = 'b' * 64
        checks = api.runtime_identity_checks(identity, binding, dependencies, current)
        self.assertFalse(checks['installed_frozen_source_identity:extnav_clock_guard.py'])
        self.assertFalse(checks['all_recorded_runtime_dependencies_unchanged'])

    def test_summary_flags_cannot_hide_stream_hash_mismatch(self):
        flow = {'received_bytes': 3, 'sent_bytes': 3, 'received_sha256': 'a' * 64,
            'sent_sha256': 'a' * 64, 'forwarded_stream_identical': True,
            'all_observed_bytes_classified': True, 'mavlink_decode_complete': True}
        proxy = {'forwarding_integrity_pass': True, 'all_mavlink_frames_decoded': True,
            'all_observed_bytes_classified': True,
            'flows': {name: dict(flow) for name in ('to_fcu', 'from_fcu')}}
        self.assertTrue(api.wire_integrity(proxy))
        proxy['flows']['to_fcu']['sent_sha256'] = 'b' * 64
        self.assertFalse(api.wire_integrity(proxy))

    def test_existing_result_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'link-probe-result.json'
            path.write_text('original failure')
            self.assertEqual(api.main(['--run-dir', directory, '--protocol', '/missing']), 2)
            self.assertEqual(path.read_text(), 'original failure')


if __name__ == '__main__':
    unittest.main()
