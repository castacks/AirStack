import unittest
import sys
from pathlib import Path

import numpy as np

from pose_covariance_replay import exp_se3, log_se3, skew
from pose_covariance_accuracy import block_isotropic, combine, mapping, project

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'MAC-VO-main'))
from Scripts.Setup.causal_interval_model import project as old_project


class AccuracyControlsTests(unittest.TestCase):
    def test_existing_common_interval_mean_is_preserved(self):
        t = exp_se3([.2, -.1, .05, -.02, .03, .01])
        frame = exp_se3([.5, .2, -.1, .1, -.2, .3])
        native, common = [1000, 1300], [1050, 1450]
        actual = project(t, np.eye(6)*1e-4, frame, native, common)
        old = old_project(frame@t@np.linalg.inv(frame), native, common)
        self.assertTrue(actual['valid'])
        np.testing.assert_allclose(actual['transform'], old['delta_model'], atol=1e-12)

    def test_lever_arm_changes_translation_covariance(self):
        frame = np.eye(4)
        frame[:3, 3] = [1, 0, 0]
        c = np.diag([.001]*3+[.1]*3)
        r = project(np.eye(4), c, frame, [0, 10], [0, 10])
        adjoint = np.block([[np.eye(3), skew(frame[:3, 3])], [np.zeros((3, 3)), np.eye(3)]])
        np.testing.assert_allclose(r['covariance'], adjoint@c@adjoint.T, atol=1e-10)
        self.assertGreater(r['covariance'][1, 1], c[1, 1])

    def test_interval_scaling_covariance_near_identity(self):
        c = np.diag([.01]*3+[.001]*3)
        r = project(np.eye(4), c, np.eye(4), [100, 200], [100, 250])
        np.testing.assert_allclose(r['covariance'], 2.25*c, atol=1e-12)

    def test_time_horizon_rejects_not_fits(self):
        r = project(np.eye(4), np.eye(6), np.eye(4), [100, 200], [500, 600])
        self.assertFalse(r['valid'])
        self.assertIsNone(r['transform'])
        with self.assertRaises(ValueError):
            project(np.eye(4), None, np.eye(4), [100., 200.], [100, 200])

    def test_missing_covariance_preserves_equal_mean_explicitly(self):
        a, b = exp_se3([.1, 0, 0, 0, .02, 0]), exp_se3([0, .1, 0, .01, 0, 0])
        expected = combine(a, b)
        result = combine(a, b, None, np.eye(6), 'full')
        self.assertEqual(result['strategy'], 'equal_missing_covariance')
        np.testing.assert_array_equal(result['transform'], expected['transform'])

    def test_single_and_no_estimate_availability(self):
        a = exp_se3([.1, 0, 0, 0, .02, 0])
        for mode in ['equal', 'scalar', 'full']:
            np.testing.assert_array_equal(combine(None, a, mode=mode)['transform'], a)
            self.assertIsNone(combine(None, None, mode=mode)['transform'])

    def test_full_and_scalar_identical_for_isotropic_translation_case(self):
        a, b = np.eye(4), exp_se3([.1, -.2, .3, 0, 0, 0])
        ca, cb = np.eye(6)*.01, np.eye(6)*.1
        full, scalar = combine(a, b, ca, cb, 'full'), combine(a, b, ca, cb, 'scalar')
        np.testing.assert_allclose(full['transform'], scalar['transform'], atol=1e-12)

    def test_equal_midpoint_is_symmetric(self):
        a, b = exp_se3([.1, .2, .3, .03, -.02, .01]), exp_se3([.2, -.1, .2, -.01, .04, .02])
        np.testing.assert_allclose(combine(a, b)['transform'], combine(b, a)['transform'], atol=1e-12)

    def test_scalarization_keeps_block_traces_only(self):
        rng = np.random.default_rng(9)
        x = rng.normal(size=(6, 6))
        c = x@x.T+np.eye(6)
        s = block_isotropic(c)
        self.assertAlmostEqual(np.trace(s[:3, :3]), np.trace(c[:3, :3]))
        self.assertAlmostEqual(np.trace(s[3:, 3:]), np.trace(c[3:, 3:]))
        np.testing.assert_array_equal(s[:3, 3:], np.zeros((3, 3)))

    def test_nonlinear_transport_monte_carlo(self):
        rng = np.random.default_rng(32)
        t = exp_se3([.2, -.1, .3, .05, -.02, .01])
        frame = exp_se3([.4, -.2, .1, .1, -.2, .05])
        c = np.diag([1, 2, 3, .2, .3, .1])*1e-7
        output = project(t, c, frame, [1000, 1300], [1050, 1450])
        predicted = output['covariance']
        inverse = np.linalg.inv(output['transform'])
        errors = np.array([log_se3(mapping(exp_se3(x)@t, frame, 4/3)@inverse)
                           for x in rng.multivariate_normal(np.zeros(6), c, size=2000)])
        ratios = np.var(errors, axis=0, ddof=1)/predicted.diagonal()
        self.assertTrue(np.all((ratios > .85)&(ratios < 1.15)), str(ratios))


if __name__ == '__main__':
    unittest.main()
