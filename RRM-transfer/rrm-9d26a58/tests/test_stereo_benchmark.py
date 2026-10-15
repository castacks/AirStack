"""Offline metric denominators and synthetic stereo convention checks."""
import importlib.util
from pathlib import Path
import unittest


@unittest.skipUnless(importlib.util.find_spec('numpy'), 'diagnostic NumPy unavailable')
class StereoBenchmarkTests(unittest.TestCase):
    def test_positive_disparity_and_no_learned_bm_ceiling(self):
        import numpy as np
        from rrm.stereo_benchmark import disparity_depth
        value = disparity_depth(np.array([10.,128.,0.,-1.,np.nan,np.inf]),10.)
        np.testing.assert_allclose(value[:2],[1.,10/128])
        self.assertTrue(np.isnan(value[2:]).all())
        self.assertTrue(np.isnan(disparity_depth(np.array([63.]),10.,63)).all())
        with self.assertRaises(ValueError):
            disparity_depth([1],-10)

    def test_coverage_is_not_accuracy_and_outliers_stay(self):
        import numpy as np
        from rrm.stereo_benchmark import error_metrics
        value = error_metrics(np.ones(4),np.array([1.,1.,100.,np.nan]),np.ones(4,dtype=bool))
        self.assertEqual(value['teacher_pixels'],4)
        self.assertEqual(value['coverage'],.75)
        self.assertEqual(value['mae_m'],33)
        self.assertEqual(value['good_fraction_of_all_teacher'],.5)
        self.assertGreater(value['absolute_error_p50_p95_p99_m'][2],90)

    def test_common_support_equal_across_candidates(self):
        import numpy as np
        from rrm.stereo_benchmark import comparison_metrics
        value = comparison_metrics(np.ones(3),dict(sparse=np.array([1.,np.nan,np.nan]),
            dense=np.array([2.,1.,1.])),np.ones(3,dtype=bool))
        self.assertEqual(value['sparse']['own_support']['compared_pixels'],1)
        self.assertEqual(value['dense']['own_support']['compared_pixels'],3)
        self.assertEqual(value['dense']['all_candidate_common_support']['compared_pixels'],1)
        self.assertEqual(value['dense']['all_candidate_common_support']['mae_m'],1)
        self.assertEqual(value['sparse']['all_candidate_common_support']['mae_m'],0)

    def test_empty_and_invalid_area(self):
        import numpy as np
        from rrm.stereo_benchmark import error_metrics, rectangle_mask
        self.assertEqual(rectangle_mask((5,6),(1,2,3,2)).sum(),6)
        with self.assertRaises(ValueError):
            rectangle_mask((5,6),(1,2,99,2))
        value = error_metrics(np.array([np.nan,0]),np.ones(2),np.ones(2,dtype=bool))
        self.assertIsNone(value['coverage'])
        self.assertNotIn('mae_m',value)
        with self.assertRaises(ValueError):
            error_metrics(np.ones(2),np.ones(3),np.ones(2,dtype=bool))

    @unittest.skipUnless(importlib.util.find_spec('cv2'),'diagnostic OpenCV unavailable')
    def test_synthetic_shift_sign_and_units(self):
        import numpy as np
        path = Path(__file__).resolve().parents[1]/'scripts/benchmark_stereo_candidates.py'
        spec = importlib.util.spec_from_file_location('stereo_candidate_script',path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        left = np.random.default_rng(37).integers(0,256,(120,240,3),dtype=np.uint8)
        right = np.zeros_like(left)
        right[:,:-8] = left[:,8:]
        disparity = module.infer_sgbm(module.build_sgbm(64),left,right)
        region = disparity[15:-15,80:-25]
        self.assertGreater(np.mean(region>0),.9)
        self.assertAlmostEqual(float(np.median(region[region>0])),8.,delta=.2)


if __name__ == '__main__':
    unittest.main()
