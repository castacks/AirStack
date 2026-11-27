"""Offline regression tests: clock mistakes must not become root-cause evidence."""
from decimal import Decimal
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('m00c_timing', ROOT / 'RRM/scripts/m00c_analyze_startup.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TimingEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.doc = json.loads((ROOT / 'RRM/evidence/M00-C/startup-source-records.json').read_text())
        self.prior = json.loads((ROOT / 'RRM/evidence/M00-B/startup-log-extract.json').read_text())

    def test_exact_nanosecond_arithmetic(self):
        _, result = module.analyze(self.doc, self.prior)
        self.assertEqual(result['heartbeat_after_deadline_s_exact_decimal'], '19.276162179')
        self.assertEqual(result['px4_process_after_interface_s'], '21.62')

    def test_prior_excerpts_match_originals(self):
        _, result = module.analyze(self.doc, self.prior)
        self.assertTrue(result['prior_excerpt_lines_verified_in_original_logs'])

    def test_clock_domain_mixing_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Clock-domain mismatch'):
            module.difference({'clock_domain': 'STEADY', 'timestamp_s': '1'},
                              {'clock_domain': 'UTC', 'timestamp_s': '1'})

    def test_cross_boot_ticks_rejected(self):
        self.doc['isaac_processes']['boot_id'] = 'different-boot'
        with self.assertRaisesRegex(ValueError, 'boot identities'):
            module.analyze(self.doc, self.prior)

    def test_wrong_tick_rate_rejected(self):
        self.doc['isaac_processes']['clk_tck'] = 1000
        with self.assertRaisesRegex(ValueError, 'tick frequency'):
            module.analyze(self.doc, self.prior)

    def test_missing_deadline_not_fabricated(self):
        self.doc['original_ros_logs']['logs'][0]['text'] = ''
        with self.assertRaisesRegex(ValueError, 'Missing required evidence'):
            module.analyze(self.doc, self.prior)

    def test_numeric_origin_remains_missing(self):
        rows, result = module.analyze(self.doc, self.prior)
        self.assertFalse(result['deadline_origin_numeric_observed'])
        row = next(r for r in rows if 'Guard deadline origin' in r['event'])
        self.assertEqual(row['timestamp_s'], '')
        self.assertEqual(row['clock_domain'], 'PROCESS_STEADY')

    def test_first_recorded_heartbeat_used(self):
        self.doc['original_ros_logs']['logs'][1]['text'] += '\n[INFO] [1795806000.000000000] CON: Got HEARTBEAT\n'
        _, result = module.analyze(self.doc, self.prior)
        self.assertEqual(result['heartbeat_log_epoch_s'], '1795805020.210903826')

    def test_utc_nanoseconds_preserved(self):
        a = module.utc_stamp('2026-11-27T18:43:20.937541747Z entry')
        b = module.utc_stamp('2026-11-27T18:43:20.000000000Z entry')
        self.assertEqual(a - b, Decimal('0.937541747'))

    def test_reversed_episode_rejected(self):
        p = self.doc['original_ros_logs']['logs'][1]
        p['text'] = p['text'].replace('1795805020.210903826', '1795804000.210903826')
        with self.assertRaisesRegex(ValueError, 'after deadline'):
            module.analyze(self.doc, self.prior)


if __name__ == '__main__':
    unittest.main(verbosity=2)
