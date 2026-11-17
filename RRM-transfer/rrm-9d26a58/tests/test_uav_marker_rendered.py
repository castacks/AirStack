"""Frozen renderer control that exposed the total-depth-spread regression."""
import importlib.util
from pathlib import Path
import unittest

try:
    import cv2  # noqa: F401
    import numpy  # noqa: F401
except ImportError as error:
    raise unittest.SkipTest('optional rendered UAV regression: ' + str(error))


class RenderedMarkerTests(unittest.TestCase):
    def test_saved_camera_inputs_and_development_controls(self):
        root = Path(__file__).parents[1]/'docs/uav-results/private-static-20261117'
        spec = importlib.util.spec_from_file_location('private_render_audit', root/'audit.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        report = module.audit(root)
        self.assertEqual(report['passed'], 6)
        self.assertEqual(report['frames'], 18)
        self.assertEqual(len(report['input_sha256']), 54)
        nominal = next(case for case in report['cases'] if case['case'] == 'nominal')
        self.assertTrue(all(item['status'] == 'DEPTH_INCONSISTENT'
                            for item in nominal['baseline_observations']))
        self.assertTrue(all(item['status'] == 'OBSERVED' for item in nominal['observations']))
        self.assertFalse(report['physical_execution'])
        self.assertFalse(report['held_out'])


if __name__ == '__main__':
    unittest.main()
