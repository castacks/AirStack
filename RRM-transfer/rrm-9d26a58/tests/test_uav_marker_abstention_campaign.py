"""Persistence and no-overwrite contracts for the offline intervention campaign."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

try:
    import numpy  # noqa: F401
    import cv2  # noqa: F401
except ImportError as error:
    raise unittest.SkipTest('optional UAV campaign: ' + str(error))

SPEC = importlib.util.spec_from_file_location(
    'uav_marker_abstention_campaign',
    Path(__file__).parents[1]/'scripts/uav_marker_abstention_campaign.py')
CAMPAIGN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CAMPAIGN)


class MarkerCampaignTests(unittest.TestCase):
    def test_saved_inputs_and_limitations_are_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'new'
            result = CAMPAIGN.run(output)
            self.assertEqual(result, json.loads((output/'report.json').read_text()))
            self.assertEqual(result['passed'], result['total'])
            self.assertEqual(result['total'], 20)
            for name, digest in (result['input_sha256'] | result['source_sha256']).items():
                self.assertEqual(hashlib.sha256((output/name).read_bytes()).hexdigest(), digest)
            records = {case['case']: case for case in result['cases']}
            for name in ('duplicate', 'occluded_split', 'occluded_full', 'depth_nan', 'depth_sparse'):
                self.assertEqual(records[name]['decision']['status'], 'ABSTAIN')
                self.assertIsNone(records[name]['decision']['delta_map_m'])
            self.assertEqual(records['occluded_partial']['decision']['status'], 'SELECTED')
            self.assertIn('NOT detected', records['occluded_partial']['limitation'])
            self.assertEqual(records['spatial_jitter']['decision']['status'], 'SELECTED')
            for name in ('spatial_disjoint', 'spatial_extremes'):
                self.assertEqual(records[name]['decision']['status'], 'ABSTAIN')
                self.assertEqual(len(set(records[name]['raw_prefixes'])), 3)
            self.assertFalse(result['physical_execution'])
            self.assertFalse(result['independent_observation_window'])
            before = (output/'report.json').read_bytes()
            with self.assertRaises(FileExistsError):
                CAMPAIGN.run(output)
            self.assertEqual((output/'report.json').read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
