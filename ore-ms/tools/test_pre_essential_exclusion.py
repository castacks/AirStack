"""Image-only union, endpoint/boundary, hash and rehashed audit controls."""
import contextlib,io,json,tempfile,shutil,unittest
from pathlib import Path
import numpy as np
from pre_essential_exclusion import image_masks,kept_indices,index_sha,exact_pose,ROOT,sha
import audit_pre_essential_exclusion as audit
class Finite(unittest.TestCase):
    def test_union_uses_all_images_and_fixed_bottom(self):
        a=np.zeros((6,4),np.uint8);b=a.copy();b[0,0]=1;zero,bottom=image_masks([a,b]);self.assertEqual(int(zero.sum()),1);self.assertTrue(zero[0,0]);self.assertTrue(bottom[:4].all());self.assertFalse(bottom[4:].any())
    def test_both_endpoints_must_survive(self):
        k=np.array([[400.,0,2],[0,400,2],[0,0,1]]);camera=(k,np.zeros(4),0,k.copy());row={'matches':3,'points0':[[0,0],[1,1],[2,2]],'points1':[[0,0],[3,3],[2,2]]};mask=np.ones((4,4),bool);mask[3,3]=False;np.testing.assert_array_equal(kept_indices(row,camera,mask),[0,2])
    def test_nearest_pixel_and_index_order(self):
        k=np.array([[400.,0,2],[0,400,2],[0,0,1]]);camera=(k,np.zeros(4),0,k.copy());row={'matches':2,'points0':[[1.49,1],[1.51,1]],'points1':[[1.49,1],[1.51,1]]};mask=np.ones((4,4),bool);mask[1,2]=False;np.testing.assert_array_equal(kept_indices(row,camera,mask),[0]);self.assertNotEqual(index_sha([0,2]),index_sha([2,0]))
    def test_exact_control_rejects_changed_pose_or_missing_field(self):
        self.assertEqual(exact_pose({'status':'input_unsupported','matches':0,'frontend':'learned'},{'status':'input_unsupported','matches':0}),2)
        with self.assertRaises(AssertionError):exact_pose({'status':'ok','matches':8,'R':[[1]]},{'status':'ok','matches':8,'R':[[0]]})
class Corruption(unittest.TestCase):
    def setUp(self):
        source=ROOT/'results/pre-essential-exclusion/source/config.json'
        if not source.exists():self.skipTest('Full experiment not scored yet')
        cfg=json.loads(source.read_text());original=ROOT/cfg['output']
        if not (original/'completion.json').exists():self.skipTest('Full experiment not scored yet')
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'results',prefix='pre-E-tests-');self.out=Path(self.temp.name)/'out';shutil.copytree(original,self.out);cfg['output']=str(self.out.relative_to(ROOT));self.config=self.out.parent/'config.json';self.config.write_text(json.dumps(cfg));f=json.loads((self.out/'input-freeze.json').read_text());f['pins'][str(self.config.relative_to(ROOT))]=sha(self.config);(self.out/'input-freeze.json').write_text(json.dumps(f))
    def tearDown(self):
        if hasattr(self,'temp'):self.temp.cleanup()
    def test_reject_rehashed_subset_fingerprint(self):
        p=self.out/'predictions.jsonl';rows=p.read_text().splitlines();r=json.loads(rows[0]);r['kept_sha256']='0'*64;rows[0]=json.dumps(r);p.write_text('\n'.join(rows)+'\n');lock=self.out/'prediction-lock.json';d=json.loads(lock.read_text());d['predictions_sha256']=sha(p);lock.write_text(json.dumps(d))
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):audit.main(self.config)
    def test_reject_rehashed_score(self):
        p=self.out/'scores.json';d=json.loads(p.read_text());d[0]['R_error']=999;p.write_text(json.dumps(d));lock=self.out/'completion.json';d=json.loads(lock.read_text());d['scores_sha256']=sha(p);lock.write_text(json.dumps(d))
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):audit.main(self.config)
    def test_reject_rehashed_cohort(self):
        p=self.out/'summary.json';d=json.loads(p.read_text());d['FlashLight']['original']['learned']['paired_rotation_ge2deg']['rgb']['outputs']+=1;p.write_text(json.dumps(d));lock=self.out/'completion.json';d=json.loads(lock.read_text());d['summary_sha256']=sha(p);lock.write_text(json.dumps(d))
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):audit.main(self.config)
if __name__=='__main__':unittest.main()
