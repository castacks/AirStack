import unittest
from unittest.mock import patch

import numpy as np

from native_information_weights import (adjoint, apply_model, fit_scales,
    information_combination, native_readout, scaled_information)
from pose_covariance_replay import exp_se3


def training_rows(name, rgb_scales, thermal_scales):
    c = np.diag([.25]*3+[.04]*3)
    experts = {}
    for camera, (a, b) in [('rgb', rgb_scales), ('thermal', thermal_scales)]:
        e = np.array([np.sqrt(3*a*.25), 0, 0, 0, np.sqrt(3*b*.04), 0])
        experts[camera] = {'valid': True, 'transform': exp_se3(-e).tolist(), 'information_inverse': c.tolist()}
    return [{'recording': name, 'experts': experts, 'reference_delta': np.eye(4).tolist()} for _ in range(50)]


class NativeWeightTests(unittest.TestCase):
    def test_native_residual_jacobian_matches_right_camera_perturbation(self):
        from Scripts.Setup.audit_optimizer_factors import residual_jacobian
        rng = np.random.default_rng(3)
        p = rng.normal(size=(12, 3))+[5, 0, 0]
        k = np.array([[400., 0, 100], [0, 390, 80], [0, 0, 1]])
        uv = rng.normal(size=(12, 2))
        disp = np.ones(12)*4
        _, j = residual_jacobian(p, uv, disp, k, .3)
        for i, e in enumerate(np.eye(6)):
            values = []
            for sign in [1, -1]:
                d = exp_se3(-sign*1e-6*e)
                pp = p@d[:3, :3].T+d[:3, 3]
                values.append(residual_jacobian(pp, uv, disp, k, .3)[0])
            np.testing.assert_allclose(j[:, :, i], (values[0]-values[1])/2e-6, atol=1e-7, rtol=1e-7)

    def test_readout_rejects_bad_covariance_and_keeps_mean_external(self):
        rng = np.random.default_rng(4)
        p = rng.normal(size=(20, 3))+[5, 0, 0]
        k = np.array([[400., 0, 100], [0, 400, 80], [0, 0, 1]])
        uv_cov = np.tile([1., 2., .2], (20, 1))
        inverse, d = native_readout(p, np.zeros((20, 2)), np.ones(20), uv_cov, np.ones(20), k, .3)
        np.testing.assert_allclose(inverse@d['information_right'], np.eye(6), atol=1e-10)
        self.assertIn('native_convergence_unknown', d['meaning'])
        with self.assertRaises(ValueError):
            native_readout(p, np.zeros((20, 2)), np.ones(20), uv_cov, -np.ones(20), k, .3)

    def test_right_to_left_covariance_transport(self):
        t = exp_se3([.4, -.2, .1, .1, -.3, .2])
        for e in np.eye(6):
            right = t@exp_se3(1e-6*e)
            left = exp_se3(1e-6*adjoint(t)@e)@t
            np.testing.assert_allclose(right, left, atol=1e-14)

    def test_fold_calibration_recovers_known_marginal_scales(self):
        a = training_rows('a', (2., 1.), (5., 2.))
        b = training_rows('b', (4., 3.), (7., 4.))
        model = fit_scales({'a': a, 'b': b}, 'c')
        for camera, expected in [('rgb', (3, 2)), ('thermal', (6, 3))]:
            self.assertAlmostEqual(model['scales'][camera]['translation'], expected[0])
            self.assertAlmostEqual(model['scales'][camera]['rotation'], expected[1])
        self.assertEqual(model['training_best_fixed'], 'rgb')
        with self.assertRaises(ValueError):
            fit_scales({'a': a, 'b': b}, 'a')
        with self.assertRaises(ValueError):
            fit_scales({'a': a, 'b': training_rows('c', (4, 3), (7, 4))}, 'c')

    def test_full_scaling_preserves_normalized_cross_terms(self):
        c = np.eye(6)
        c[0, 3] = c[3, 0] = .3
        out = scaled_information(c, 4, 9)
        self.assertAlmostEqual(out[0, 3], 1.8)
        self.assertAlmostEqual(out[0, 3]/np.sqrt(out[0, 0]*out[3, 3]), .3)
        for scale in [0, -1, np.nan]:
            with self.assertRaises(ValueError):
                scaled_information(c, scale, 1)

    def test_missing_information_uses_available_model_not_equal_bad_mean(self):
        a, b = np.eye(4), exp_se3([0, 1, 0, 0, 0, .1])
        experts = {'rgb': {'valid': True, 'transform': a, 'information_inverse': np.eye(6)},
                   'thermal': {'valid': True, 'transform': b, 'information_inverse': None}}
        np.testing.assert_array_equal(information_combination(experts, 'full')['transform'], a)
        experts['rgb']['information_inverse'] = None
        self.assertEqual(information_combination(experts, 'full')['strategy'], 'rgb_no_information')
        experts['rgb'].update(valid=False, transform=None)
        np.testing.assert_array_equal(information_combination(experts, 'full')['transform'], b)

    def test_application_keeps_native_means_and_does_not_read_labels(self):
        row = training_rows('c', (2, 1), (5, 2))[0]
        methods = {'native_'+c: {'transform': row['experts'][c]['transform'], 'strategy': c+'_only'} for c in ['rgb', 'thermal']}
        raw = {'recording': 'c', 'rows': [{'pair': 0, 'source_ids': ['000000', '000001'],
                                         'experts': row['experts'], 'methods': methods}]}
        model = fit_scales({'a': training_rows('a', (2, 1), (5, 2)),
                            'b': training_rows('b', (4, 3), (7, 4))}, 'c')
        with patch('native_information_weights.reference_rows', side_effect=AssertionError('held-out label access')):
            # Fill the remaining baseline method names as inert copies.
            raw['rows'][0]['methods'].update(native_equal=methods['native_rgb'], native_angular=methods['native_rgb'],
                                            raw_scalar=methods['native_rgb'], raw_full=methods['native_rgb'])
            output = apply_model(raw, model)
        for c in ['rgb', 'thermal']:
            np.testing.assert_array_equal(output['rows'][0]['experts'][c]['transform'], row['experts'][c]['transform'])
            np.testing.assert_array_equal(output['rows'][0]['methods']['native_'+c]['transform'], row['experts'][c]['transform'])


if __name__ == '__main__':
    unittest.main()
