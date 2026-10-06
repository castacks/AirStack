import copy
import importlib.util
from pathlib import Path
import unittest
spec=importlib.util.spec_from_file_location('pose_compare',Path(__file__).parents[1]/'scripts/airstack_pose_compare.py')
pc=importlib.util.module_from_spec(spec);spec.loader.exec_module(pc)

def fixture():
    truth=[{'schema':'airstack-physical-truth/v1','sequence':i,'capture_id':'a',
            'vehicle_path':'/World/drone','recorder_sha256':'a'*64,'sim_time_s':i*.1,
            'rigid_body_position_m':[i*.1,0.,1.], 'sensor_state_position_m':[i*.1,0.,1.]}
           for i in range(31)]
    events=[{'channel':'odom','source_stamp_ns':i*100_000_000+50_000_000,
             'message':{'header':{'frame_id':'map','stamp':{'sec':i//10,'nanosec':(i%10)*100_000_000+50_000_000}},'child_frame_id':'base_link',
                        'pose':{'pose':{'position':{'x':i*.1+.05-.02,'y':.03,'z':.9}}}}}
            for i in range(30)]
    return truth,events

class PoseCompareTests(unittest.TestCase):
    def test_known_translation_and_interpolation(self):
        t,e=fixture();r=pc.compare(t,e,0,1.1)
        self.assertEqual(r['matched_samples'],30)
        for got,expected in zip(r['baseline_raw']['median_xyz_m'],[.02,-.03,.1]):self.assertAlmostEqual(got,expected)
        self.assertLess(r['after_baseline_translation_removed']['rms_norm_m'],1e-14)
        self.assertEqual(r['body_sensor_max_distance_m'],0)
    def test_motion_error_is_not_fitted_away(self):
        t,e=fixture()
        for v in e[15:]:v['message']['pose']['pose']['position']['x']+=.2
        r=pc.compare(t,e,0,1.1)
        self.assertAlmostEqual(r['samples'][-1]['grounded_translation_removed_xyz_m'][0],-.2)
    def test_epoch_reset_and_mixed_identity_rejected(self):
        for field,value in [('sim_time_s',0),('capture_id','other'),('sequence',99)]:
            t,e=fixture();t[-1][field]=value
            with self.assertRaises(ValueError):pc.compare(t,e,0,1.1)
        t,e=fixture();e[-1]['source_stamp_ns']=0
        with self.assertRaises(ValueError):pc.compare(t,e,0,1.1)
    def test_strict_identity_sequence_and_header(self):
        for field,value in [('sequence',True),('vehicle_path',''),('recorder_sha256','abc')]:
            t,e=fixture();t[1][field]=value
            with self.assertRaises(ValueError):pc.compare(t,e,0,1.1)
        t,e=fixture();e[1]['source_stamp_ns']+=1
        with self.assertRaises(ValueError):pc.compare(t,e,0,1.1)
    def test_sparse_brackets_skip_without_extrapolation(self):
        t,e=fixture();t=t[:13]+t[20:]
        for i,v in enumerate(t):v['sequence']=i
        e.append(copy.deepcopy(e[-1]));e[-1]['source_stamp_ns']=4_000_000_000;e[-1]['message']['header']['stamp']={'sec':4,'nanosec':0}
        r=pc.compare(t,e,0,1.1)
        self.assertGreater(r['skipped']['wide_bracket'],0)
        self.assertEqual(r['skipped']['outside_overlap'],1)
    def test_invalid_baseline_frame_and_nonfinite_rejected(self):
        t,e=fixture()
        with self.assertRaises(ValueError):pc.compare(t,e,0,.1)
        e[0]['message']['header']['frame_id']='odom'
        with self.assertRaises(ValueError):pc.compare(t,e,0,1.1)
        t,e=fixture();t[0]['rigid_body_position_m'][0]=float('nan')
        with self.assertRaises(ValueError):pc.compare(t,e,0,1.1)
if __name__=='__main__':unittest.main()
