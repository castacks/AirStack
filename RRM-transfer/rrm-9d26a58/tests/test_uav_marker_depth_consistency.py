"""Fixture-specific depth inconsistency must not create a viewpoint goal."""
import unittest

try:
    import cv2  # noqa: F401
    import numpy as np
except ImportError as error:
    raise unittest.SkipTest('optional UAV depth consistency: ' + str(error))

from rrm.uav_marker import detect_red_surface
from rrm.uav_viewpoint import select_viewpoint


class MarkerDepthConsistencyTests(unittest.TestCase):
    def fixture(self):
        rgb = np.zeros((120, 160, 3), np.uint8)
        rgb[40:80, 60:100] = [255, 0, 0]
        return rgb, np.full((120, 160), 3.5, np.float32)

    def test_split_surface_and_single_outlier_withhold_goal(self):
        for case in ('split', 'outlier'):
            with self.subTest(case=case):
                rgb, depth = self.fixture()
                if case == 'split':
                    depth[:, 80:] = 7.
                else:
                    depth[60, 80] = 7.
                observed = detect_red_surface(rgb, depth, [100., 0., 80., 0., 100., 60., 0., 0., 1.])
                self.assertEqual(observed['status'], 'DEPTH_INCONSISTENT')
                self.assertIsNone(observed['surface_point_optical_m'])
                decision = select_viewpoint([observed]*3, [160, 120], [1., 0., 0.])
                self.assertEqual(decision['status'], 'ABSTAIN')
                self.assertIsNone(decision['delta_map_m'])

    def test_declared_limit_and_diagnostic_range(self):
        rgb, depth = self.fixture()
        k = [100., 0., 80., 0., 100., 60., 0., 0., 1.]
        depth[:, 80:] += .0625
        observed = detect_red_surface(rgb, depth, k, max_depth_spread_m=.0625)
        self.assertEqual(observed['status'], 'OBSERVED')
        self.assertEqual(observed['depth_spread_m'], .0625)
        self.assertEqual(observed['depth_min_m'], 3.5)
        self.assertEqual(observed['depth_max_m'], 3.5625)
        self.assertEqual(detect_red_surface(rgb, depth, k, max_depth_spread_m=.03125)['status'],
                         'DEPTH_INCONSISTENT')
        for value in (None, True, 0., -1., float('nan'), float('inf'), '0.1'):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    detect_red_surface(rgb, depth, k, max_depth_spread_m=value)


if __name__ == '__main__':
    unittest.main()
