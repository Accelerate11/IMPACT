"""The continuity gate must detect head/tail silence, not just middle gaps."""
import importlib.util
from pathlib import Path
import unittest

MODULE = Path(__file__).resolve().parents[1]/'analyze_extnav_link_probe.py'
spec = importlib.util.spec_from_file_location('link_analysis', MODULE)
api = importlib.util.module_from_spec(spec)
spec.loader.exec_module(api)


class ContinuityTests(unittest.TestCase):
    protocol = {'fcu_minimum_each_vis_messages': 3, 'fcu_minimum_boot_span_s': .2,
                'fcu_maximum_gap_s_exclusive': .3}

    def rows(self, stamps):
        return [{'TimeUS': stamp, 'Ign': 0, 'Q': 0, 'RTimeUS': stamp+1000000} for stamp in stamps]

    def test_regular_stream_with_observed_boundaries(self):
        result = api.fcu_continuity(self.rows([100000, 200000, 300000]), 0, 400000, self.protocol)
        self.assertTrue(result['continuity_pass'])

    def test_tail_silence_fails_despite_regular_internal_stream(self):
        result = api.fcu_continuity(self.rows([100000, 200000, 300000]), 0, 800000, self.protocol)
        self.assertFalse(result['continuity_pass'])
        self.assertEqual(result['tail_gap_s'], .5)

    def test_head_silence_fails(self):
        result = api.fcu_continuity(self.rows([500000, 600000, 700000]), 0, 800000, self.protocol)
        self.assertFalse(result['continuity_pass'])

    def test_exact_300ms_is_not_strictly_below_gate(self):
        result = api.fcu_continuity(self.rows([100000, 400000, 500000]), 0, 600000, self.protocol)
        self.assertFalse(result['continuity_pass'])

    def test_unknown_boundary_is_censored_not_pass(self):
        result = api.fcu_continuity(self.rows([100000, 200000, 300000]), None, None, self.protocol)
        self.assertFalse(result['continuity_pass'])
        self.assertFalse(result['boundaries_observed'])

    def test_burst_duplicates_do_not_hide_long_gap(self):
        result = api.fcu_continuity(self.rows([100000, 100000, 600000]), 0, 700000, self.protocol)
        self.assertFalse(result['continuity_pass'])

    def test_observed_key_comparison_retains_missing_both_directions(self):
        result = api.compare_keys([1, 2, 2, 3], [2, 3, 4])
        self.assertEqual(result['observed_in_both'], 2)
        self.assertEqual(result['upstream_not_observed_downstream'], 1)
        self.assertEqual(result['downstream_not_observed_upstream'], 1)

    def test_system_time_stops_in_middle_cannot_hide_tail(self):
        result = api.boundary_coverage([{'monotonic_ns': 1000000000},
                                       {'monotonic_ns': 40000000000}], 0, 80000000000, 1.)
        self.assertFalse(result['sufficiently_close'])
        self.assertEqual(result['host_tail_uncovered_s'], 40.)

    def test_dense_boundary_messages_expose_small_uncovered_edges(self):
        result = api.boundary_coverage([{'monotonic_ns': 50000000},
                                       {'monotonic_ns': 79950000000}], 0, 80000000000, 1.)
        self.assertTrue(result['sufficiently_close'])
        self.assertEqual(result['host_tail_uncovered_s'], .05)


if __name__ == '__main__':
    unittest.main()
