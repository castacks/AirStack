"""Adversarial no-goal contracts for the restricted viewpoint policy."""
import unittest

from rrm.uav_viewpoint import select_viewpoint


class ViewpointBoundaryTests(unittest.TestCase):
    def nominal(self):
        return [dict(status='OBSERVED', bbox_xywh=[40, 30, 20, 20],
                     surface_point_optical_m=[0., 0., 3.5], surface_depth_z_m=3.5)
                for _ in range(3)]

    def assert_abstains(self, observations, size=(100, 80), axis=(1., 0., 0.)):
        result = select_viewpoint(observations, size, axis)
        self.assertEqual(result['status'], 'ABSTAIN')
        self.assertIsNone(result['delta_map_m'])
        self.assertNotIn('selected_observation_index', result)

    def test_malformed_envelopes(self):
        for value in (None, 3, True, 'abc', {}, [], [None]*3):
            with self.subTest(value=value):
                self.assert_abstains(value)
        for value in (None, 3, {}, 'ab', [True, 80]):
            with self.subTest(size=value):
                self.assert_abstains(self.nominal(), size=value)
        for value in (None, 3, {}, 'abc', [True, 0, 0]):
            with self.subTest(axis=value):
                self.assert_abstains(self.nominal(), axis=value)

    def test_malformed_observation_fields(self):
        for field in ('bbox_xywh', 'surface_point_optical_m', 'surface_depth_z_m'):
            for value in (None, 3, True, {}, 'abc', []):
                with self.subTest(field=field, value=value):
                    observations = self.nominal()
                    observations[1][field] = value
                    self.assert_abstains(observations)

    def test_tuple_payloads_are_consistent(self):
        observations = self.nominal()
        for observation in observations:
            observation['bbox_xywh'] = tuple(observation['bbox_xywh'])
            observation['surface_point_optical_m'] = tuple(observation['surface_point_optical_m'])
        result = select_viewpoint(tuple(observations), (100, 80), (1., 0., 0.))
        self.assertEqual(result['status'], 'SELECTED')


if __name__ == '__main__':
    unittest.main()
