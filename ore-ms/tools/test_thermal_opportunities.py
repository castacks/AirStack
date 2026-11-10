import unittest
import numpy as np

from thermal_opportunities import FEATURES, apply, diagnose, features, fit, runs


def expert(factors=10, valid=True):
    return {'valid': valid, 'transform': np.eye(4).tolist() if valid else None,
            'information_inverse': np.eye(6).tolist(),
            'diagnostic': {'factors': factors, 'residual_mean_square_px': 1.}}


def row(pair=0, a=(2., 2.), b=(1., 1.)):
    return {'pair': pair, 'source_ids': [str(pair), str(pair+1)],
            'common_interval_ns': [pair*100000000, (pair+1)*100000000],
            'errors': {'rgb': dict(zip(['rotation_deg', 'translation_m'], a)),
                       'thermal': dict(zip(['rotation_deg', 'translation_m'], b))},
            'experts': {'rgb': expert(), 'thermal': expert()}, 'camera': 'rgb',
            'fixed_camera': 'rgb', 'strategy': 'ridge'}


class OpportunitiesTest(unittest.TestCase):
    def test_tradeoffs_oracles_and_ties(self):
        rows = [row(0), row(1, (2, 1), (1, 2)), row(2, (1, 2), (2, 1)), row(3, (1, 1), (1, 1))]
        s = diagnose(rows)
        self.assertEqual(s['categories'], {'both_thermal_better': 1, 'rotation_thermal_translation_rgb': 1,
                                          'rotation_rgb_translation_thermal': 1, 'includes_tie': 1})
        self.assertEqual(s['paired_mean_errors']['rotation_oracle'], {'rotation_deg': 1., 'translation_m': 1.5})
        self.assertEqual(s['paired_mean_errors']['translation_oracle'], {'rotation_deg': 1.5, 'translation_m': 1.})
        self.assertEqual(s['paired_mean_errors']['joint_oracle'], {'rotation_deg': 1.25, 'translation_m': 1.25})

    def test_runs_break_on_source_gap_or_losing_interval(self):
        r = [row(0), row(1), row(3), row(4, (1, 1), (2, 2)), row(5)]
        result = runs(r, lambda x: x['errors']['rgb']['rotation_deg'] > x['errors']['thermal']['rotation_deg'])
        self.assertEqual([x['intervals'] for x in result], [2, 1, 1])
        self.assertAlmostEqual(result[0]['summed_duration_s'], .2)
        r[1]['source_ids'][0] = 'broken'
        self.assertEqual(runs(r[:2], lambda _: True)[0]['intervals'], 1)

    def test_missing_estimates_keep_denominators(self):
        r = [row(0), row(1), row(2)]
        r[1]['errors']['thermal'] = None
        r[2]['errors'] = {'rgb': None, 'thermal': None}
        r[2]['camera'] = None
        s = diagnose(r)
        self.assertEqual(s['expected_intervals'], 3)
        self.assertEqual(s['paired_intervals'], 1)
        self.assertEqual(s['availability'], {'both': 1, 'rgb_only': 1, 'neither': 1})
        self.assertEqual(s['cue_own_support'], 2)

    def test_features_no_label_dependency_and_exact_ratios(self):
        r = row()
        r['experts']['thermal']['diagnostic']['factors'] = 20
        x = features(r)
        self.assertEqual(len(x), len(FEATURES))
        self.assertAlmostEqual(x[0], np.log(2))
        self.assertTrue(np.array_equal(x[1:], np.zeros(7)))
        del r['errors']
        np.testing.assert_array_equal(x, features(r))
        r['experts']['thermal']['diagnostic']['residual_mean_square_px'] = 0
        self.assertIsNone(features(r))

    def test_fit_balances_records_and_rejects_held_identity(self):
        def training(name, n, angle):
            result = []
            for i in range(n):
                r = row(i); r['recording'] = name; r['reference_delta'] = np.eye(4).tolist()
                a = np.deg2rad(angle)
                r['experts']['rgb']['transform'] = [[np.cos(a), -np.sin(a), 0, 0],
                    [np.sin(a), np.cos(a), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
                result.append(r)
            return result
        training_rows = {'a': training('a', 50, 1), 'b': training('b', 100, 3)}
        m = fit(training_rows, 'c')
        self.assertAlmostEqual(m['beta'][0], 2)
        expected = np.zeros(8); expected[6] = .5
        np.testing.assert_allclose(m['beta'][1:], expected, atol=1e-12)
        self.assertAlmostEqual(m['scale'][6], (np.log1p(np.deg2rad(3))-np.log1p(np.deg2rad(1)))/2)
        self.assertEqual(apply(row(), m)['camera'], 'thermal')
        with self.assertRaises(ValueError):
            fit(training_rows, 'a')
        training_rows['b'][0]['recording'] = 'c'
        with self.assertRaises(ValueError):
            fit(training_rows, 'c')

    def test_fallbacks_and_application_does_not_read_labels(self):
        model = {'mean': [0]*8, 'scale': [1]*8, 'beta': [0]*9}
        r = row(); del r['errors']
        self.assertEqual(apply(r, model)['camera'], 'rgb')
        r['experts']['rgb']['valid'] = False
        self.assertEqual(apply(r, model)['camera'], 'thermal')
        r['experts']['thermal']['valid'] = False
        self.assertIsNone(apply(r, model)['camera'])
        r = row(); r['experts']['thermal']['information_inverse'] = None
        self.assertEqual(apply(r, model)['strategy'], 'missing_cue_rgb')


if __name__ == '__main__':
    unittest.main()
