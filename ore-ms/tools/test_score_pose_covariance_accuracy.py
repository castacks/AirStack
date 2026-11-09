import tempfile
import unittest
from pathlib import Path

import numpy as np

from pose_covariance_replay import exp_se3
from score_pose_covariance_accuracy import error, reference_pose, statistics, summarize


class ScoringTests(unittest.TestCase):
    def test_known_rotation_and_translation_errors(self):
        p = exp_se3([0, 0, 0, 0, 0, np.pi/2])
        p[:3, 3] = [3, 4, 0]
        d = error(p, np.eye(4))
        self.assertAlmostEqual(d['rotation_deg'], 90)
        self.assertAlmostEqual(d['translation_m'], 5)

    def test_reference_direction_and_gauge(self):
        a = exp_se3([1, .2, -.4, .1, .2, -.1])
        b = a@exp_se3([.1, .3, .2, .03, -.02, .01])
        reference = np.linalg.solve(a, b)
        np.testing.assert_allclose(list(error(reference, reference).values()), 0, atol=1e-12)
        g = exp_se3([2, 1, -.3, -.4, .1, .2])
        np.testing.assert_allclose(np.linalg.solve(g@a, g@b), reference, atol=1e-12)
        self.assertGreater(error(np.linalg.inv(reference), reference)['translation_m'], .1)

    def test_missing_estimates_not_counted_as_zero_error(self):
        rows = [{'errors': {'x': None}}, {'errors': {'x': {'rotation_deg': 2, 'translation_m': 3}}}]
        s = summarize(rows, ['x'])
        self.assertEqual(s['x']['rotation_deg']['count'], 1)
        self.assertEqual(s['x']['rotation_deg']['mean'], 2)

    def test_statistics_and_empty_support(self):
        self.assertEqual(statistics([])['mean'], None)
        self.assertAlmostEqual(statistics([0, 2])['rmse'], np.sqrt(2))

    def test_reference_qualification_matches_frozen_rigidity_rule(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'pose.txt'
            t = np.eye(4)
            t[0, 0] += 1e-7
            np.savetxt(path, t)
            p, reason = reference_pose(path)
            self.assertIsNotNone(p)
            np.testing.assert_allclose(p, np.eye(4), atol=1e-14)
            t[0, 0] = 2
            np.savetxt(path, t)
            self.assertIsNone(reference_pose(path)[0])


if __name__ == '__main__':
    unittest.main()
