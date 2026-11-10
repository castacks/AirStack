import unittest
import numpy as np
from recover_depth_ablation_labels import sample


class DepthSamplingTest(unittest.TestCase):
    def test_original_pixel_rounding_and_metres(self):
        depth = np.array([[256, 512], [0, 768]], dtype=np.uint16)
        z, valid = sample(depth, [[0, 0], [.49, .49], [.5, .5], [1, 0], [0, 1]])
        np.testing.assert_array_equal(z, [1, 1, 3, 2, 0])
        np.testing.assert_array_equal(valid, [True, True, True, True, False])

    def test_no_hole_fill_or_out_of_bounds_replacement(self):
        depth = np.array([[256, 0], [512, 768]], dtype=np.uint16)
        z, valid = sample(depth, [[1, 0], [-1, 0], [2, 0], [0, float('nan')], [0, float('inf')]])
        np.testing.assert_array_equal(z, np.zeros(5))
        self.assertFalse(valid.any())

    def test_zero_maps_leave_empty_support(self):
        z, valid = sample(np.zeros((4, 4), dtype=np.uint16), [[1, 1], [2, 2]])
        self.assertFalse(valid.any())
        self.assertEqual(int(np.count_nonzero(z)), 0)


if __name__ == '__main__':
    unittest.main()
