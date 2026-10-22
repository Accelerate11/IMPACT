"""Meaningful offline extraction, source-guard, bracket and quantization checks."""
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('transport_replay_api', ROOT/'scripts/replay_extnav_transport_clock.py')
api = importlib.util.module_from_spec(spec)
spec.loader.exec_module(api)
clock = api.load_module(ROOT/'src/xq_autonomy/xq_autonomy/extnav_transport_clock.py', 'test_replay_clock')
guard = api.load_module(ROOT/'src/xq_autonomy/xq_autonomy/extnav_clock_guard.py', 'test_replay_guard')


class ExtractionTests(unittest.TestCase):
    def event(self, seq, wall, before, after):
        return {'kind': 'adapter_status', 'event_sequence': seq, 'monotonic_ns': after+100,
            'wall_time_ns': wall+200, 'message': {'parsed': {'transport_clock_sampling': {
                'transport_stamp_ns': wall, 'monotonic_before_ns': before, 'monotonic_after_ns': after,
                'monotonic_midpoint_ns': (before+after)//2, 'sampling_span_ns': after-before,
                'maximum_sampling_span_ns': 5_000_000, 'sampling_uncertain': after-before>5_000_000}}}}

    def extract(self, events):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'events.jsonl'
            path.write_text('\n'.join(json.dumps(v) for v in events)+'\n')
            return api.extract_samples(path)

    def test_dedup_original_tuple_preserves_first_order_and_every_event(self):
        first = self.event(10, 1_700_000_000_000_000_000, 100, 120)
        second = self.event(11, 1_700_000_000_000_000_100, 200, 220)
        repeated = self.event(12, first['message']['parsed']['transport_clock_sampling']['transport_stamp_ns'], 100, 120)
        values, stats = self.extract([first, second, repeated])
        self.assertEqual([v['source_token'] for v in values], [1, 2])
        self.assertEqual([v['monotonic_ns'] for v in values], [110, 210])
        self.assertEqual([v['line_number'] for v in values[0]['origins']], [1, 3])
        self.assertEqual(values[0]['origins'][1]['event'], repeated)
        self.assertEqual(stats['duplicate_occurrences'], 1)

    def test_raw_backstep_and_uncertain_bracket_are_not_filtered(self):
        values, _ = self.extract([self.event(1, 10_000_000_000, 100, 120),
            self.event(2, 9_000_000_000, 200, 5_000_201)])
        self.assertEqual(values[1]['raw_wall_ns'], 9_000_000_000)
        self.assertTrue(values[1]['sampling_uncertain'])
        self.assertEqual(values[1]['sampling_span_ns'], 5_000_001)

    def test_missing_sampling_is_counted_and_not_filled(self):
        values, stats = self.extract([{'kind': 'adapter_status', 'message': {'parsed': {'transport_clock_sampling': None}}},
            self.event(2, 10_000_000_000, 100, 120)])
        self.assertEqual(len(values), 1)
        self.assertEqual(stats['statuses_without_sampling'], 1)
        self.assertEqual(values[0]['origins'][0]['line_number'], 2)

    def test_inconsistent_original_span_or_midpoint_is_rejected(self):
        for field in ('sampling_span_ns', 'monotonic_midpoint_ns'):
            event = self.event(1, 10_000_000_000, 100, 120)
            event['message']['parsed']['transport_clock_sampling'][field] += 1
            with self.assertRaises(ValueError):
                self.extract([event])


class ReplayTests(unittest.TestCase):
    def run_fixture(self, kind):
        value = api.replay(api.controlled_samples(kind), clock, guard)
        value['checks'].update(api.fixture_checks(kind, value))
        self.assertTrue(all(value['checks'].values()), {k:v for k,v in value['checks'].items() if not v})
        return value

    def test_no_jump_control_accepts_without_transforming_epoch(self):
        result = self.run_fixture('no_jump')
        self.assertEqual(result['candidate_accepted_count'], 161)
        self.assertEqual(result['mapped_guard_forward_allowed_count'], 161)
        self.assertGreater(result['mapped_guard_ready_count'], 0)
        self.assertEqual(result['final_candidate_snapshot']['anchor_initializations'], 1)

    def test_wall_steps_are_visible_to_raw_guard_but_not_followed_by_mapping(self):
        for kind in ('forward_clock_step', 'backward_clock_step'):
            result = self.run_fixture(kind)
            self.assertEqual(result['first_raw_guard_fault_index'], 70)
            self.assertIsNone(result['first_candidate_clock_fault_index'])
            self.assertTrue(result['final_candidate_snapshot']['ever_raw_wall_anomaly'])

    def test_cumulative_small_drift_retains_original_50ms_guard_budget(self):
        result = self.run_fixture('cumulative_small_drift')
        self.assertEqual(result['first_raw_guard_fault_index'], 51)
        self.assertFalse(result['states'][50]['raw_guard_snapshot']['fault_latched'])

    def test_candidate_clock_does_not_repair_duplicate_or_frozen_source(self):
        for kind in ('duplicate_source', 'frozen_source'):
            result = self.run_fixture(kind)
            self.assertEqual(result['first_mapped_guard_fault_index'], 70)
            self.assertIsNone(result['first_candidate_clock_fault_index'])
            self.assertTrue(all(not v['mapped_guard_step']['forward_allowed'] for v in result['states'][70:]))

    def test_exact_350ms_and_350ms_plus_1ns_have_different_warmup_behavior(self):
        exact = self.run_fixture('source_gap_exact_350ms')
        stale = self.run_fixture('source_gap_350ms_plus_1ns')
        self.assertTrue(exact['states'][70]['mapped_guard_step']['clock_ready'])
        self.assertFalse(stale['states'][70]['mapped_guard_step']['clock_ready'])
        self.assertTrue(stale['states'][120]['mapped_guard_step']['clock_ready'])

    def test_uncertain_bracket_rejects_without_reanchoring_and_later_resumes(self):
        result = self.run_fixture('uncertain_bracket')
        self.assertEqual(result['uncertain_bracket_indices'], [70])
        self.assertFalse(result['states'][70]['candidate_clock_step']['accepted'])
        self.assertTrue(result['states'][71]['candidate_clock_step']['accepted'])
        self.assertEqual(result['candidate_accepted_count'], 160)

    def test_sub_microsecond_conflict_latches_and_never_clamps(self):
        result = self.run_fixture('sub_microsecond_conflict')
        self.assertEqual(result['first_candidate_clock_fault_index'], 1)
        self.assertEqual(result['candidate_accepted_count'], 1)
        self.assertFalse(result['raw_guard_forward_floor_us_strict'])
        self.assertTrue(all(not v['candidate_clock_step']['accepted'] for v in result['states'][1:]))

    def test_ledger_keeps_raw_bracket_and_is_explicitly_not_AP_output(self):
        ledger = io.StringIO()
        samples = api.controlled_samples('no_jump')[:2]
        api.replay(samples, clock, guard, ledger)
        rows = [json.loads(v) for v in ledger.getvalue().splitlines()]
        self.assertEqual(rows[1]['original_input'], samples[1])
        self.assertIs(rows[1]['candidate_sent_to_AP'], False)

    def test_independent_monotonicity_check_catches_quantization_collision(self):
        self.assertTrue(api.strictly_increasing([1_000, 1_999]))
        self.assertFalse(api.strictly_increasing([v//1000 for v in [1_000, 1_999]]))


if __name__ == '__main__':
    unittest.main()
