"""Mixture/null/projection controls and rehashed semantic corruptions."""
import contextlib,io,json,tempfile,shutil,unittest
from pathlib import Path
import numpy as np
from essential_inlier_mixture import feature,native,divide,ROOT,sha
import audit_essential_inlier_mixture as audit

def fixture():
    k=np.array([[400.,0,320],[0,400,240],[0,0,1.]]);cam=(k,np.zeros(4),0,k.copy());p=np.array([[100.,100],[200,100],[100,200],[200,200]]);q=p.copy();q[2:,0]+=20;r={'frontend':'learned','arm':'rgb','index':0,'status':'failed','matches':4,'points0':p.tolist(),'points1':q.tolist(),'essential_mask':[1,1,0,0],'pose_mask':[1,0,0,0],'cheirality_inliers':1};return r,cam
class Finite(unittest.TestCase):
    def test_stationary_retention_is_separate_from_moving(self):
        p,c=fixture();r=feature(p,c,480,640);self.assertEqual(r['retention'],{'accepted_low':2,'accepted_high':2,'essential_low':2,'essential_high':0,'low_retention':1,'high_retention':0});self.assertEqual(r['masks']['essential']['low_fraction'],1);self.assertEqual(r['masks']['nonessential']['low_fraction'],0)
    def test_empty_masks_and_denominator_null(self):
        p,c=fixture();p.update(essential_mask=[0]*4,pose_mask=[0]*4,cheirality_inliers=0);r=feature(p,c,480,640);self.assertIsNone(r['masks']['essential']['low_fraction']);self.assertIsNone(r['masks']['essential']['angle_quartiles']);self.assertIsNone(divide(0,0))
    def test_projection_uses_passed_camera(self):
        p,c=fixture();points=np.asarray(p['points0']);np.testing.assert_allclose(native(points,c),points,atol=1e-10);k=c[0].copy();k[0,0]*=2;other=(k,c[1],c[2],c[3]);self.assertFalse(np.allclose(native(points,c),native(points,other)))
    def test_pose_must_be_subset_of_essential(self):
        p,c=fixture();p['pose_mask']=[0,0,1,0]
        with self.assertRaises(AssertionError):feature(p,c,480,640)
class Corruption(unittest.TestCase):
    def setUp(self):
        source=ROOT/'results/essential-inlier-mixture/source/config.json'
        if not source.exists():self.skipTest('Real study not joined')
        cfg=json.loads(source.read_text());original=ROOT/cfg['output']
        if not (original/'completion.json').exists():self.skipTest('Real study not joined')
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'results',prefix='inlier-mixture-tests-');self.out=Path(self.temp.name)/'out';shutil.copytree(original,self.out);cfg['output']=str(self.out.relative_to(ROOT));self.config=self.out.parent/'config.json';self.config.write_text(json.dumps(cfg));f=json.loads((self.out/'input-freeze.json').read_text());f['pins'][str(self.config.relative_to(ROOT))]=sha(self.config);(self.out/'input-freeze.json').write_text(json.dumps(f))
    def tearDown(self):
        if hasattr(self,'temp'):self.temp.cleanup()
    def test_valid(self):
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(audit.main(self.config)['status'],'pass')
    def corrupt(self,file,mutation):
        p=self.out/(file+'.json');d=json.loads(p.read_text());mutation(d);p.write_text(json.dumps(d));receipt=self.out/('feature-lock.json' if file=='features' else 'completion.json');d=json.loads(receipt.read_text());d[file+'_sha256']=sha(p);receipt.write_text(json.dumps(d))
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):audit.main(self.config)
    def test_reject_low_motion_count(self):self.corrupt('features',lambda d:d['FlashLight'][0]['masks']['essential'].update(low_count=99999))
    def test_reject_spatial_grid(self):self.corrupt('features',lambda d:d['FlashLight'][0]['masks']['accepted']['source']['counts'].__setitem__(0,99999))
    def test_reject_aggregate(self):self.corrupt('summary',lambda d:d['SmokeRoom']['learned']['rgb'].update(anchors=99999))
if __name__=='__main__':unittest.main()
