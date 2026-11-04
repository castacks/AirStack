"""Observable storage contract under byte pressure; no native counterfactual."""
import tempfile
from pathlib import Path
import unittest
from Scripts.Setup.capacity_retained_inputs import CapacityInputBuffer


def fixture(root, sizes):
    rows = []
    for i, pair_sizes in enumerate(sizes):
        row = {}
        for c in ['rgb', 'thermal']:
            for side in ['left', 'right']:
                name = f'{c}-{side}-{i}.bin'
                (root / name).write_bytes(bytes([i]) * pair_sizes[c])
                row[c + '_' + side] = {'path': name, 'time_ns': 100 + i}
        rows.append(row)
    return rows


class CapacityTests(unittest.TestCase):
    def test_budget_validation(self):
        for value in [-1, True, 1.0, '2']:
            with self.assertRaises(ValueError):
                CapacityInputBuffer('.', [], False, value)

    def test_zero_budget_rejects_whole_stereo(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = fixture(Path(tmp), [{'rgb': 3, 'thermal': 2}])
            b = CapacityInputBuffer(tmp, rows, False, 0)
            b.advance('rgb', 0)
            self.assertEqual(b.entries['rgb'], {})
            self.assertEqual(b.report()['ingress_bytes'], 6)
            self.assertEqual(b.report()['peak_retained_bytes'], 0)
            self.assertEqual(b.report()['rejected_pairs'], 1)

    def test_global_oldest_eviction_and_tie(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = fixture(Path(tmp), [{'rgb': 2, 'thermal': 2}] * 3)
            b = CapacityInputBuffer(tmp, rows, False, 8)
            b.advance('rgb', 0); b.advance('thermal', 0); b.advance('rgb', 1)
            self.assertEqual(b.audit[-1]['evicted'], [{'camera': 'rgb', 'source': 0}])
            b.advance('thermal', 1)
            self.assertEqual(b.audit[-1]['evicted'], [{'camera': 'thermal', 'source': 0}])
            self.assertEqual(b.resident_bytes(), 8)

    def test_oversize_does_not_evict_existing_pair(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = fixture(Path(tmp), [{'rgb': 2, 'thermal': 5}])
            b = CapacityInputBuffer(tmp, rows, False, 8)
            b.advance('rgb', 0); b.advance('thermal', 0)
            self.assertEqual(b.resident_bytes(), 4)
            self.assertIn(0, b.entries['rgb'])
            self.assertEqual(b.audit[-1]['evicted'], [])
            self.assertTrue(b.audit[-1]['rejected_oversize'])

    def test_retained_query_has_original_bytes_after_disk_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); rows = fixture(root, [{'rgb': 3, 'thermal': 2}] * 3)
            b = CapacityInputBuffer(root, rows, False, 12)
            b.advance('rgb', 0); b.advance('rgb', 1)
            (root / rows[0]['rgb_left']['path']).write_bytes(b'changed')
            self.assertEqual(b.retrieve('rgb', 0, 1)['images']['left']['data'], bytes(3))
            b.advance('rgb', 2)
            self.assertEqual(b.audit[-1]['expired_sources'], [0])
            self.assertEqual(b.audit[-1]['evicted'], [])
            with self.assertRaises(ValueError): b.retrieve('rgb', 0, 2)

    def test_outages_do_not_admit_or_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = fixture(Path(tmp), [{'rgb': 1, 'thermal': 1}] * 22)
            b = CapacityInputBuffer(tmp, rows, True, 0)
            for i in range(22): b.advance('rgb', i)
            self.assertEqual(b.rejected_pairs, 20)
            self.assertNotIn('images', b.audit[20])
            self.assertEqual(b.ingress_bytes, 40)

    def test_arrival_order_remains_strict(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = fixture(Path(tmp), [{'rgb': 1, 'thermal': 1}])
            b = CapacityInputBuffer(tmp, rows, False, 8)
            with self.assertRaises(ValueError): b.advance('rgb', 1)
            b.advance('rgb', 0)
            with self.assertRaises(ValueError): b.advance('rgb', 0)


if __name__ == '__main__': unittest.main()
