"""Reject incompatible camera/image metadata before the geometry preview."""
from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace as N
import unittest


@unittest.skipUnless(importlib.util.find_spec('numpy'),'diagnostic NumPy dependency unavailable')
class StereoCaptureAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=Path(__file__).parents[1]/'scripts/airstack_stereo_depth_capture.py'
        spec=importlib.util.spec_from_file_location('stereo_capture_adapter_test',path)
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls.calibration_for=staticmethod(module.calibration_for)

    def setUp(self):
        p=[184.,0.,240.,0.,0.,184.,150.,0.,0.,0.,1.,0.]
        r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        left=N(width=480,height=300,header=N(frame_id='camera_left'),p=p[:],r=r[:])
        right=N(width=480,height=300,header=N(frame_id='camera_right'),p=p[:],r=r[:])
        right.p[3]=-22.08
        disparity=N(header=N(frame_id='camera_left'),image=N(width=480,height=300,
            header=N(frame_id='camera_left'),encoding='32FC1'),f=184.,t=.12,min_disparity=0.,
            max_disparity=63.,delta_d=.0625,valid_window=N(x_offset=70,y_offset=7,width=402,height=285))
        self.messages=dict(left_info=left,right_info=right,disparity=disparity,
                           left=N(width=480,height=300),right=N(width=480,height=300))

    def test_standard_calibration(self):
        value=self.calibration_for(self.messages)
        self.assertEqual(value.valid_window,(70,7,402,285))

    def test_actual_image_dimensions(self):
        for side in ['left','right']:
            messages=deepcopy(self.messages)
            messages[side].width=400
            with self.assertRaises(ValueError):self.calibration_for(messages)

    def test_rectified_intrinsic_mismatches(self):
        for index in [0,2,5,6]:
            messages=deepcopy(self.messages)
            messages['right_info'].p[index]+=1.
            with self.assertRaises(ValueError):self.calibration_for(messages)

    def test_projection_rows_and_vertical_offset(self):
        for side,index in [('right_info',7),('left_info',8),('right_info',10)]:
            messages=deepcopy(self.messages)
            messages[side].p[index]+=1.
            with self.assertRaises(ValueError):self.calibration_for(messages)


if __name__=='__main__':unittest.main()
