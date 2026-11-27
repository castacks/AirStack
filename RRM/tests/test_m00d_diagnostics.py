"""Offline synthetic-record and historical replay checks; no ROS/Docker/runtime calls."""
from decimal import Decimal
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


analysis = load('m00d_characterize', 'RRM/scripts/m00d_characterize.py')
events = load('m00d_validate_events', 'RRM/scripts/m00d_validate_events.py')


class HistoricalAnalysis(unittest.TestCase):
    def setUp(self):
        self.doc = json.loads((ROOT / 'RRM/evidence/M00-C/startup-source-records.json').read_text())
        self.graph = json.loads((ROOT / 'RRM/evidence/M00-D/dependency-graph.json').read_text())

    def test_original_delta_independently_reproduced(self):
        metrics = analysis.characterize(self.doc)
        self.assertEqual(next(x['seconds'] for x in metrics if x['measurement_id'] == 'L11'), '19.276162179')
        self.assertEqual(next(x['seconds'] for x in metrics if x['measurement_id'] == 'L03'), '74.228771713')

    def test_missing_markers_remain_missing(self):
        self.doc['filtered_logs'][0]['records'] = [x for x in self.doc['filtered_logs'][0]['records']
                                                  if 'PX4 node at' not in x['text']]
        metrics = analysis.characterize(self.doc)
        self.assertEqual(next(x['status'] for x in metrics if x['measurement_id'] == 'L03'), 'MISSING')

    def test_different_boot_rejected(self):
        self.doc['isaac_processes']['boot_id'] = 'different'
        with self.assertRaisesRegex(ValueError, 'boot'):
            analysis.characterize(self.doc)

    def test_mixed_domains_rejected(self):
        with self.assertRaisesRegex(ValueError, 'clock domains'):
            analysis.delta({'domain': 'STEADY', 'episode': 'a', 'value': Decimal(1)},
                           {'domain': 'UTC', 'episode': 'a', 'value': Decimal(2)})

    def test_different_episode_rejected(self):
        with self.assertRaisesRegex(ValueError, 'episodes'):
            analysis.delta({'domain': 'UTC', 'episode': 'a', 'value': Decimal(1)},
                           {'domain': 'UTC', 'episode': 'b', 'value': Decimal(2)})

    def test_reversed_time_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Reversed'):
            analysis.delta({'domain': 'UTC', 'episode': 'a', 'value': Decimal(2)},
                           {'domain': 'UTC', 'episode': 'a', 'value': Decimal(1)})

    def test_dependency_graph(self):
        self.assertEqual(len(analysis.graph_order(self.graph)), 17)

    def test_unknown_edge_rejected(self):
        self.graph['edges'][0]['from'] = 'NONEXISTENT'
        with self.assertRaisesRegex(ValueError, 'Unknown'):
            analysis.graph_order(self.graph)

    def test_cycle_rejected(self):
        self.graph['edges'].append({'from': 'READY', 'to': 'GC'})
        with self.assertRaisesRegex(ValueError, 'cycle'):
            analysis.graph_order(self.graph)


class ProposedInstrumentationFormat(unittest.TestCase):
    def record(self, **kwargs):
        result = {'schema': events.VERSION, 'episode_id': 'SYNTHETIC_ONLY', 'producer_id': 'guard',
                  'process_instance_id': 'mock1', 'boot_id': 'mock-boot', 'instrumentation_id': 'I05',
                  'event': 'guard.constructed', 'sequence': 1, 'clock_domain': 'STEADY',
                  'unit': 'ns', 'native_time': 100}
        result.update(kwargs)
        return result

    def test_valid_format_does_not_qualify_coverage(self):
        result = events.validate([self.record()])
        self.assertFalse(result['coverage_qualified'])
        self.assertEqual(result['requirement_verdict'], 'NOT_ASSESSED')

    def test_boolean_timestamp_rejected(self):
        with self.assertRaisesRegex(ValueError, 'integer'):
            events.validate([self.record(native_time=True)])

    def test_wrong_native_unit_rejected(self):
        with self.assertRaisesRegex(ValueError, 'unit mismatch'):
            events.validate([self.record(unit='seconds')])

    def test_missing_identity_rejected(self):
        record = self.record()
        del record['boot_id']
        with self.assertRaisesRegex(ValueError, 'Missing'):
            events.validate([record])

    def test_duplicate_sequence_rejected(self):
        with self.assertRaisesRegex(ValueError, 'sequence'):
            events.validate([self.record(), self.record(native_time=101)])

    def test_reset_requires_new_identity(self):
        with self.assertRaisesRegex(ValueError, 'reset'):
            events.validate([self.record(), self.record(sequence=2, native_time=1)])
        self.assertEqual(events.validate([self.record(), self.record(episode_id='next', native_time=1)])['schema_valid_records'], 2)

    def test_missing_tick_rate_rejected(self):
        with self.assertRaisesRegex(ValueError, 'tick rate'):
            events.validate([self.record(clock_domain='KERNEL_BOOT', unit='ticks')])

    def test_empty_trace_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Empty'):
            events.validate([])


if __name__ == '__main__':
    unittest.main(verbosity=2)
