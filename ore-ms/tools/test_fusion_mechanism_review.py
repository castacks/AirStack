import unittest
import numpy as np
from fusion_mechanism_review import decompose


class DecompositionTest(unittest.TestCase):
    def test_opposite_constant_biases_cancel(self):
        truth = np.array([[0., 0., 1.], [0., 0., 1.]])
        a = truth+[0, 0, 1]; b = truth-[0, 0, 1]
        d = decompose(a, b, truth)
        self.assertEqual(d['same_record_oracle_constant']['alpha'], .5)
        self.assertEqual(d['same_record_oracle_constant']['mse_m2'], 0)
        self.assertEqual(d['cameras']['rgb']['bias_share_mse'], 1)
        self.assertEqual(d['centered_cross_error_m2'], 0)
        self.assertEqual(d['risk_polynomial']['B_mean_error_term_m2'], -2)

    def test_centered_cancellation_without_bias(self):
        truth = np.array([[0., 0., 1.], [0., 0., 1.]])
        noise = np.array([[1, 0, 0], [-1, 0, 0]])
        d = decompose(truth+noise, truth-noise, truth)
        self.assertEqual(d['cameras']['rgb']['bias_share_mse'], 0)
        self.assertEqual(d['cameras']['rgb']['motion_axis_share_on_nonzero_motion'], 0)
        self.assertEqual(d['centered_error_cosine_aggregate'], -1)
        self.assertEqual(d['risk_polynomial']['B_centered_term_m2'], -2)

    def test_shared_bias_cannot_be_removed_by_identical_expert(self):
        truth = np.array([[0., 0., 1.], [0., 0., 1.]])
        a = truth+[0, 0, 1]
        d = decompose(a, a, truth)
        self.assertEqual(d['same_record_oracle_constant']['alpha'], 0)
        self.assertEqual(d['per_interval_oracle']['mean_translation_error_m'], 1)

    def test_zero_motion_and_invalid_inputs(self):
        d = decompose([[1, 0, 0]], [[-1, 0, 0]], [[0, 0, 0]])
        self.assertEqual(d['nonzero_reference_motion'], 0)
        self.assertIsNone(d['cameras']['rgb']['motion_axis_mean_error_m'])
        with self.assertRaises(ValueError):
            decompose([], [], [])
        with self.assertRaises(ValueError):
            decompose([[float('nan'), 0, 0]], [[0, 0, 0]], [[0, 0, 0]])


if __name__ == '__main__':
    unittest.main()
