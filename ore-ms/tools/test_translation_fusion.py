import copy
import unittest
import numpy as np

from test_thermal_opportunities import expert
from translation_fusion import apply, blend, fit


def fixture(recording='a', fraction=.25, difference=1.):
    a, b = expert(), expert()
    b['transform'][0][3] = difference
    truth = np.eye(4); truth[0, 3] = fraction*difference
    return {'pair': 0, 'source_ids': ['0', '1'], 'recording': recording,
            'reference_delta': truth.tolist(), 'experts': {'rgb': a, 'thermal': b}}


class TranslationTest(unittest.TestCase):
    def test_geometry_preserves_rgb_rotation_and_endpoint_translation(self):
        a = np.eye(4); a[:2, :2] = [[0, -1], [1, 0]]; a[:3, 3] = [1, 2, 3]
        b = np.eye(4); b[:3, 3] = [5, 6, 7]
        result = np.asarray(blend(a.tolist(), b.tolist(), .25))
        np.testing.assert_array_equal(result[:3, :3], a[:3, :3])
        np.testing.assert_array_equal(result[:3, 3], [2, 3, 4])
        np.testing.assert_array_equal(result[3], [0, 0, 0, 1])
        np.testing.assert_array_equal(np.asarray(blend(a, b, 1))[:3, 3], b[:3, 3])
        with self.assertRaises(ValueError):
            blend(a, b, 1.1)

    def test_equal_record_fit_and_global_alpha(self):
        data = {'a': [fixture('a', .2) for _ in range(50)],
                'b': [fixture('b', .8) for _ in range(100)]}
        m = fit(data, 'c')
        self.assertAlmostEqual(m['constant_alpha'], .5)
        self.assertAlmostEqual(m['blend_beta'][0], .5)
        np.testing.assert_allclose(m['blend_beta'][1:], 0, atol=1e-12)
        self.assertAlmostEqual(m['select_beta'][0], 0, places=12)
        self.assertAlmostEqual(apply(fixture(), m)['methods']['ridge_blend']['thermal_alpha'], .5)

    def test_known_optimum_and_zero_disagreement(self):
        data = {c: [fixture(c) for _ in range(50)] for c in ['a', 'b']}
        m = fit(data, 'c')
        self.assertAlmostEqual(m['constant_alpha'], .25)
        self.assertAlmostEqual(m['blend_beta'][0], .25)
        self.assertAlmostEqual(m['select_beta'][0], -.5)
        zero = {c: [fixture(c, difference=0) for _ in range(50)] for c in ['a', 'b']}
        m = fit(zero, 'c')
        self.assertTrue(m['all_training_disagreement_zero'])
        np.testing.assert_array_equal(m['blend_beta'], np.zeros(9))

    def test_excluded_record_identity(self):
        data = {c: [fixture(c) for _ in range(50)] for c in ['a', 'b']}
        with self.assertRaises(ValueError):
            fit(data, 'a')
        data['b'][0]['recording'] = 'c'
        with self.assertRaises(ValueError):
            fit(data, 'c')

    def test_clipping_and_label_free_application(self):
        m = {'mean': [0]*8, 'scale': [1]*8, 'blend_beta': [2]+[0]*8,
             'select_beta': [-1]+[0]*8, 'constant_alpha': .5, 'training_best_fixed': 'rgb'}
        r = fixture(); del r['reference_delta']; del r['recording']
        result = apply(r, m)
        self.assertEqual(result['methods']['ridge_blend']['thermal_alpha'], 1)
        self.assertEqual(result['methods']['ridge_select']['thermal_alpha'], 0)
        m['blend_beta'][0] = -2
        self.assertEqual(apply(r, m)['methods']['ridge_blend']['thermal_alpha'], 0)

    def test_availability_and_missing_cues(self):
        m = {'mean': [0]*8, 'scale': [1]*8, 'blend_beta': [1]*9,
             'select_beta': [1]*9, 'constant_alpha': .5, 'training_best_fixed': 'rgb'}
        r = fixture(); r['experts']['thermal']['information_inverse'] = None
        result = apply(r, m)
        self.assertEqual(result['strategy'], 'missing_cue_rgb')
        np.testing.assert_array_equal(result['methods']['ridge_blend']['transform'], r['experts']['rgb']['transform'])
        r = fixture(); r['experts']['rgb']['valid'] = False
        result = apply(r, m)
        np.testing.assert_array_equal(result['methods']['ridge_blend']['transform'], r['experts']['thermal']['transform'])
        self.assertFalse(result['rgb_rotation_preserved'])
        r['experts']['thermal']['valid'] = False
        self.assertIsNone(apply(r, m)['methods']['ridge_blend']['transform'])


if __name__ == '__main__':
    unittest.main()
