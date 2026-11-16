"""Gyro direction/time/gap and ROS pixel encoding controls, without dataset errors."""
import unittest
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from subt_gyro_consistency import integrate_gyro,summarize
from audit_subt_gyro_consistency import gyro_quaternion
from qualify_subt_raw_binding import pixels,nearest_index,numeric_imu

class GyroControls(unittest.TestCase):
    def test_target_from_source_is_inverse_body_integral(self):
        t=np.arange(0,1_000_000_001,10_000_000,dtype=np.int64);g=np.tile([0.,0.,.2],(len(t),1));r=integrate_gyro(t,g,0,500_000_000)
        self.assertTrue(r['supported']);np.testing.assert_allclose(r['target_from_source_imu'],Rotation.from_rotvec([0,0,-.1]).as_matrix(),atol=1e-13)
    def test_noncommuting_piecewise_motion_and_partial_boundaries(self):
        t=np.array([0,10,20,30])*1_000_000;g=np.array([[1.,0,0],[0,2.,0],[0,0,3.],[0,0,0]])
        r=integrate_gyro(t,g,5_000_000,25_000_000);expected=(Rotation.from_rotvec([.005,0,0])*Rotation.from_rotvec([0,.02,0])*Rotation.from_rotvec([0,0,.015])).inv().as_matrix()
        np.testing.assert_allclose(r['target_from_source_imu'],expected,atol=1e-13)
    def test_reject_gap_without_relaxing_support(self):
        t=np.array([0,10,40,50])*1_000_000;g=np.zeros((4,3));self.assertEqual(integrate_gyro(t,g,0,50_000_000)['reason'],'imu_gap')
    def test_reject_extrapolation_and_reversed_interval(self):
        t=np.array([10,20,30])*1_000_000;g=np.zeros((3,3));self.assertFalse(integrate_gyro(t,g,0,25_000_000)['supported']);self.assertFalse(integrate_gyro(t,g,10_000_000,31_000_000)['supported']);self.assertFalse(integrate_gyro(t,g,20_000_000,10_000_000)['supported'])
    def test_stationary_control_is_identity(self):
        r=integrate_gyro(np.array([0,10,20])*1_000_000,np.zeros((3,3)),0,20_000_000);np.testing.assert_array_equal(r['target_from_source_imu'],np.eye(3));self.assertEqual(r['rotation_deg'],0.)
    def test_rgb_encoding_and_padded_rows(self):
        m=SimpleNamespace(height=1,width=1,step=4,data=np.array([1,2,3,99],np.uint8),encoding='rgb8');np.testing.assert_array_equal(pixels(m,'rgb'),[[[3,2,1]]])
    def test_big_endian_thermal_preserves_values(self):
        m=SimpleNamespace(height=1,width=2,step=6,data=np.array([1,2,3,4,99,99],np.uint8),encoding='mono16',is_bigendian=True);np.testing.assert_array_equal(pixels(m,'thermal'),[[258,772]])
    def test_nearest_tie_uses_earlier_stamp(self):self.assertEqual(nearest_index(np.array([0,10]),5),0)
    def test_independent_quaternion_direction_and_gap(self):
        t=np.array([0,10,20,30])*1_000_000;g=np.array([[1.,0,0],[0,2.,0],[0,0,3.],[0,0,0]])
        q=gyro_quaternion(t,g,5_000_000,25_000_000);r=np.array(integrate_gyro(t,g,5_000_000,25_000_000)['target_from_source_imu']);np.testing.assert_allclose(Rotation.from_quat(q).inv().as_matrix(),r,atol=1e-13)
        self.assertGreater(np.linalg.norm(Rotation.from_quat(q).as_matrix()-r),.01)
        self.assertIsNone(gyro_quaternion(np.array([0,10,40])*1_000_000,np.zeros((3,3)),0,40_000_000))
    def test_moving_added_and_zero_denominators(self):
        labels=[dict(arm=a,index=i,rotation_deg=[0,3,8][i]) for a in ['rgb','thermal'] for i in range(3)]
        rows=[];decisions=[]
        for f in ['sift','learned']:
            for i in range(3):
                decisions.append(dict(frontend=f,index=i,policies={'rgb':'rgb' if i==0 else None,'thermal':'thermal','rgb_first':'rgb' if i==0 else 'thermal'}))
                rows.extend([dict(frontend=f,arm='rgb',index=i,disagreement_deg=0 if i==0 else None),dict(frontend=f,arm='thermal',index=i,disagreement_deg=[0,.5,1.][i])])
        s=summarize(rows,decisions,labels,{'common_gyro':[0,1,2],'paired_rotation_ge2deg':[1,2]},3)['frontends']['learned']['paired_rotation_ge2deg']
        self.assertEqual(s['added_thermal']['within1deg'],2);self.assertEqual(s['added_thermal']['zero_within1deg'],0);self.assertEqual(s['added_thermal']['zero_within5deg'],1);self.assertEqual(s['rgb']['missing_outputs_on_common'],2);self.assertIsNone(s['rgb_first']['pose_accuracy'])

if __name__=='__main__':unittest.main()
