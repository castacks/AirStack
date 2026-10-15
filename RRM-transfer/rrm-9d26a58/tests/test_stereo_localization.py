"""CPU-only projection and fail-closed evidence tests; no ROS or teacher inputs."""
from dataclasses import replace
import math
import unittest

from rrm.stereo_localization import (CameraMapPose, PixelRegion, SensorBinding,
                                     StereoCalibration, localize_region)


class StereoLocalizationTests(unittest.TestCase):
    def setUp(self):
        self.values = [[10.0]*20 for _ in range(20)]
        self.region = PixelRegion('operator_patch',4,4,10,10,'a'*64)
        self.calibration = StereoCalibration(20,20,100.,100.,8.5,8.5,.1,0.,63.,.0625,(0,0,20,20))
        self.binding = SensorBinding('episode','a'*64,100,100,100,100,10.)
        self.pose = CameraMapPose('map','camera_left',100,(1.,2.,3.),(0.,0.,0.,1.))

    def call(self, **changes):
        values = dict(region=self.region, calibration=self.calibration, binding=self.binding,
                      pose=self.pose, current_episode_id='episode', now_monotonic_s=10.1)
        values.update(changes)
        return localize_region(self.values,**values)

    def test_projection_and_translation(self):
        value=self.call()
        self.assertEqual(value['status'],'SURFACE_POINT_PREVIEW')
        self.assertEqual(value['camera_surface_point_m'],(0.,0.,1.))
        self.assertEqual(value['map_surface_point_m'],(1.,2.,4.))
        self.assertFalse(value['association_verified'])
        self.assertFalse(value['accuracy_validated'])
        self.assertFalse(value['execution_dispatch'])

    def test_optical_axis_rotation(self):
        pose=replace(self.pose,quaternion_xyzw=(0.,math.sqrt(.5),0.,math.sqrt(.5)))
        value=self.call(pose=pose)
        for actual,expected in zip(value['map_surface_point_m'],(2.,2.,3.)):
            self.assertAlmostEqual(actual,expected)

    def test_binding_rejections(self):
        cases=[dict(current_episode_id='other'),dict(region=replace(self.region,image_sha256='b'*64)),
               dict(binding=replace(self.binding,right_image_stamp_ns=101)),
               dict(pose=replace(self.pose,source_stamp_ns=101)),
               dict(pose=replace(self.pose,parent_frame='isaac_world')),
               dict(now_monotonic_s=12.),dict(now_monotonic_s=9.)]
        for case in cases:
            with self.subTest(case=case): self.assertEqual(self.call(**case)['status'],'HOLD')

    def test_window_and_region_rejections(self):
        for window in [(0,0,0,0),(0,0,4294967218,20),(-1,0,20,20)]:
            self.assertEqual(self.call(calibration=replace(self.calibration,valid_window=window))['status'],'HOLD')
        self.assertEqual(self.call(region=replace(self.region,x=19))['status'],'HOLD')

    def test_sparse_and_boundary_disparities_hold(self):
        for value in [0.,63.,float('nan'),float('inf')]:
            self.values=[[value]*20 for _ in range(20)]
            self.assertEqual(self.call()['reason'],'insufficient_region_depth')

    def test_incoherent_region_holds(self):
        self.values=[[5. if u<9 else 10. for u in range(20)] for _ in range(20)]
        self.assertEqual(self.call()['reason'],'incoherent_region_depth')

    def test_nonfinite_calibration_and_pose_hold(self):
        self.assertEqual(self.call(calibration=replace(self.calibration,fx=float('nan')))['status'],'HOLD')
        self.assertEqual(self.call(pose=replace(self.pose,translation_m=(float('inf'),0.,0.)))['status'],'HOLD')
        self.assertEqual(self.call(pose=replace(self.pose,quaternion_xyzw=(0.,0.,0.,2.)))['status'],'HOLD')

    def test_malformed_sample_and_window_hold(self):
        self.values[4][4]=None
        self.assertEqual(self.call()['reason'],'invalid_disparity_sample')
        self.assertEqual(self.call(calibration=replace(self.calibration,valid_window=(0,0)))['status'],'HOLD')
        self.assertEqual(self.call(calibration=replace(self.calibration,cx=100.))['status'],'HOLD')


if __name__=='__main__': unittest.main()
