"""Restricted static-fixture viewpoint windows require spatial agreement."""
import unittest

from rrm.uav_viewpoint import select_viewpoint


def window(xs):
    return [dict(status='OBSERVED', bbox_xywh=[x, 40, 30, 30],
                 surface_point_optical_m=[(x+15-80)*3.5/100, 0., 3.5],
                 surface_depth_z_m=3.5) for x in xs]


class ViewpointSpatialTests(unittest.TestCase):
    def test_disjoint_regions_cannot_form_one_stable_window(self):
        for xs in ((35, 65, 95), (95, 35, 65), (65, 95, 35)):
            with self.subTest(xs=xs):
                result = select_viewpoint(window(xs), [160, 120], [1., 0., 0.])
                self.assertEqual(result['status'], 'ABSTAIN')
                self.assertIsNone(result['delta_map_m'])

    def test_small_jitter_and_exact_boundary_are_accepted(self):
        for xs in ((63, 65, 67), (60, 65, 70)):
            with self.subTest(xs=xs):
                self.assertEqual(select_viewpoint(window(xs), [160, 120], [1., 0., 0.])['status'],
                                 'SELECTED')

    def test_extremes_must_overlap_even_if_both_match_middle(self):
        result = select_viewpoint(window((57, 65, 73)), [160, 120], [1., 0., 0.])
        self.assertEqual(result['status'], 'ABSTAIN')
        self.assertIsNone(result['delta_map_m'])


if __name__ == '__main__':
    unittest.main()
