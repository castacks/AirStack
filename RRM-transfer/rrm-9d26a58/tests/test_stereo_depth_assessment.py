"""Assessment math with authored truth; optional NumPy is diagnostic-only."""
from dataclasses import replace
import importlib.util
import unittest

from rrm.stereo_depth_assessment import assess_axial_depth
from rrm.stereo_localization import StereoCalibration


@unittest.skipUnless(importlib.util.find_spec('numpy'),'diagnostic NumPy dependency unavailable')
class StereoDepthAssessmentTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        self.np=np
        self.calibration=StereoCalibration(4,4,100.,100.,2.,2.,.1,0.,63.,.0625,(1,1,2,2))
        self.disparity=np.full((4,4),10.,dtype=float)
        self.truth=np.ones((4,4),dtype=float)

    def test_exact_depth_and_window(self):
        value=assess_axial_depth(self.disparity,self.truth,self.calibration)
        self.assertEqual(value['compared_pixels'],4)
        self.assertEqual(value['mae_m'],0)
        self.assertEqual(value['teacher_supported_fraction'],1)
        self.assertFalse(value['geometry_authority'])

    def test_outlier_retained_in_error(self):
        self.truth[1,1]=2.
        value=assess_axial_depth(self.disparity,self.truth,self.calibration)
        self.assertEqual(value['mae_m'],.25)
        self.assertEqual(value['rmse_m'],.5)
        self.assertEqual(value['within_10cm_fraction'],.75)

    def test_missing_and_boundary_depth_separated(self):
        self.disparity[1,1]=63.
        self.truth[2,2]=float('nan')
        value=assess_axial_depth(self.disparity,self.truth,self.calibration)
        self.assertEqual(value['teacher_pixels'],3)
        self.assertEqual(value['stereo_pixels'],3)
        self.assertEqual(value['compared_pixels'],2)
        self.assertAlmostEqual(value['teacher_supported_fraction'],2/3)

    def test_no_support_has_no_accuracy_result(self):
        self.disparity[:]=0.
        value=assess_axial_depth(self.disparity,self.truth,self.calibration)
        self.assertEqual(value['status'],'NO_COMPARABLE_PIXELS')
        self.assertNotIn('mae_m',value)

    def test_invalid_shape_and_window(self):
        with self.assertRaises(ValueError):
            assess_axial_depth(self.disparity,self.truth[:1],self.calibration)
        with self.assertRaises(ValueError):
            assess_axial_depth(self.disparity,self.truth,replace(self.calibration,valid_window=(0,0,99,99)))

    def test_near_range_does_not_hide_errors(self):
        self.truth[:]=.01
        value=assess_axial_depth(self.disparity,self.truth,self.calibration)
        self.assertEqual(value['teacher_below_stereo_minimum_fraction'],1)
        self.assertIsNone(value['within_range_teacher_supported_fraction'])
        self.assertAlmostEqual(value['mae_m'],.99)


if __name__=='__main__':unittest.main()
