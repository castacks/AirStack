"""Translation/epipolar, E-branch, null and rehashed corruption controls."""
import contextlib,io,json,tempfile,shutil,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from motion_compression_location import pair,skew,plane,ROOT,sha
import audit_motion_compression_location as audit

def fixture():
    rng=np.random.default_rng(7);X=np.column_stack([rng.uniform(-2,2,30),rng.uniform(-1,1,30),rng.uniform(5,10,30)]);R=Rotation.from_euler('y',8,degrees=True).as_matrix();t=np.array([.2,.02,.01]);Y=X@R.T+t;k=np.array([[400.,0,320],[0,400,240],[0,0,1.]]);pixels=lambda A:(A[:,:2]/A[:,2,None])*400+np.array([320,240]);T=np.eye(4);T[:3,:3]=R;T[:3,3]=t;p={'frontend':'learned','arm':'rgb','index':0,'status':'ok','matches':len(X),'points0':pixels(X).tolist(),'points1':pixels(Y).tolist(),'E':(skew(t)@R).tolist(),'R':R.tolist(),'t':(t/np.linalg.norm(t)).tolist(),'essential_mask':[1]*len(X),'pose_mask':[1]*len(X),'cheirality_inliers':len(X)};return p,k,T
class Finite(unittest.TestCase):
    def test_translation_has_zero_epipolar_and_nonzero_rotation_warp(self):
        p,k,T=fixture();r=pair(p,k,T,True)['subsets']['accepted'];self.assertLess(r['reference_epipolar']['median'],1e-8);self.assertGreater(r['reference_warp']['median'],.1)
    def test_wrong_matches_break_reference_epipolar(self):
        p,k,T=fixture();p['points1']=p['points1'][1:]+p['points1'][:1];self.assertGreater(pair(p,k,T,True)['subsets']['accepted']['reference_epipolar']['median'],.1)
    def test_E_contains_reference_rotation(self):
        p,k,T=fixture();r=pair(p,k,T,True);self.assertLess(min(c['error_deg'] for c in r['candidates']),1e-6);self.assertGreater(max(c['error_deg'] for c in r['candidates']),170)
    def test_empty_and_degenerate_remain_null(self):
        p,k,T=fixture();p.update(status='failed',matches=0,points0=[],points1=[],essential_mask=[],pose_mask=[],cheirality_inliers=0);p.pop('E');r=pair(p,k,T,False);self.assertEqual(r['candidates'],[]);self.assertIsNone(r['subsets']['accepted']['ray_change']['median']);self.assertEqual(len(plane(np.array([[0.,0,1]]),np.array([[0.,0,1]]),np.zeros((3,3)))),0)
class Corruption(unittest.TestCase):
    def setUp(self):
        source=ROOT/'results/motion-compression-location/source/config.json'
        if not source.exists():self.skipTest('Real diagnostic not yet scored')
        cfg=json.loads(source.read_text());original=ROOT/cfg['output']
        if not (original/'completion.json').exists():self.skipTest('Real diagnostic not yet scored')
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'results',prefix='motion-location-tests-');self.out=Path(self.temp.name)/'out';shutil.copytree(original,self.out);cfg['output']=str(self.out.relative_to(ROOT));self.config=self.out.parent/'config.json';self.config.write_text(json.dumps(cfg));f=json.loads((self.out/'input-freeze.json').read_text());f['pins'][str(self.config.relative_to(ROOT))]=sha(self.config);(self.out/'input-freeze.json').write_text(json.dumps(f))
    def tearDown(self):
        if hasattr(self,'temp'):self.temp.cleanup()
    def test_valid(self):
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(audit.main(self.config)['status'],'pass')
    def corrupt(self,file,mutate):
        p=self.out/(file+'.json');d=json.loads(p.read_text());mutate(d);p.write_text(json.dumps(d));receipt=self.out/'completion.json';r=json.loads(receipt.read_text());r[file+'_sha256']=sha(p);receipt.write_text(json.dumps(r))
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):audit.main(self.config)
    def test_reject_residual_corruption(self):self.corrupt('rows',lambda d:d['FlashLight'][0]['subsets']['accepted']['reference_epipolar'].update(median=999))
    def test_reject_candidate_error_corruption(self):
        def mutate(d):
            r=next(r for r in d['FlashLight'] if r['candidates']);r['candidates'][0]['error_deg']+=10
        self.corrupt('rows',mutate)
    def test_reject_branch_count_corruption(self):self.corrupt('summary',lambda d:d['SmokeRoom']['learned']['rgb']['R5'].update(wrong_output_alternative_correct=999))
if __name__=='__main__':unittest.main()
