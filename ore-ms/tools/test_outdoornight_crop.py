"""Boundary/count mapping controls and positive/rehashed full-result audits."""
import contextlib,io,json,tempfile,shutil,unittest
from pathlib import Path
import numpy as np
import outdoornight_crop as runner
import audit_outdoornight_crop as audit
from deployment_lighting import sha
ROOT=runner.ROOT
class Finite(unittest.TestCase):
    def setUp(self):self.model=runner.Comparison(ROOT/'results/outdoornight-matching/source/config.json')
    def test_native_roundtrip_and_independent_analytic_projection(self):
        for a in ['rgb','thermal']:
            c=self.model.cameras[a+'_camera'];h,w=c['image_height'],c['image_width'];points=np.array([[0.,0.],[w-1.,h-1.],[w/2,h/2],[29.25,82.75],[300.1,350.9]]);p=self.model.lift(points,a);np.testing.assert_allclose(audit.native_pixels(p,c),points,atol=1e-7);np.testing.assert_allclose(audit.native_pixels(p,c),runner.native(p,self.model.camera(a)),atol=1e-8)
    def test_both_endpoints_boundary_and_count(self):
        for a in ['rgb','thermal']:
            c=self.model.cameras[a+'_camera'];h,w=c['image_height'],c['image_width'];boundary=int(np.ceil(2*h/3));p0=np.array([[20.,boundary-1.],[20.,boundary],[30.,0.],[50.,boundary-1.]]);p1=p0.copy();p1[3,1]=boundary;row={'frontend':'learned','arm':a,'index':0,'matches':4,'points0':self.model.lift(p0,a).tolist(),'points1':self.model.lift(p1,a).tolist()};mask=runner.image_masks([np.zeros((h,w),np.uint8)])[1];keep=runner.kept_indices(row,self.model.camera(a),mask);np.testing.assert_array_equal(keep,[0,2]);np.testing.assert_array_equal(keep,audit.crop_indices(row,c));row['points0'][0]=[1e8,1e8]
            with self.assertRaises(AssertionError):audit.crop_indices(row,c)
    def test_single_count_rank_exact_and_empty_full_order(self):
        for n in [0,1,8,37,300]:
            for k in sorted(set([0,n,n//2])):
                ids=runner.count_indices(n,k,'OutdoorNight','learned','thermal',12);self.assertEqual(len(ids),k);self.assertEqual(ids.tolist(),sorted(set(ids.tolist())));np.testing.assert_array_equal(ids,audit.rank(n,k,'learned','thermal',12))
        self.assertFalse(np.array_equal(audit.rank(100,25,'learned','rgb',0),audit.rank(100,25,'learned','rgb',1)))
class Corruption(unittest.TestCase):
    def setUp(self):
        source=ROOT/'results/outdoornight-crop/source/config.json';cfg=json.loads(source.read_text());original=ROOT/cfg['output']
        if not (original/'completion.json').exists():self.skipTest('Full experiment not scored yet')
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'results',prefix='outdoor-crop-tests-');self.out=Path(self.temp.name)/'out';shutil.copytree(original,self.out);cfg['output']=str(self.out.relative_to(ROOT));self.config=self.out.parent/'config.json';self.config.write_text(json.dumps(cfg));f=json.loads((self.out/'input-freeze.json').read_text());f['pins'][str(self.config.relative_to(ROOT))]=sha(self.config);(self.out/'input-freeze.json').write_text(json.dumps(f))
    def tearDown(self):
        if hasattr(self,'temp'):self.temp.cleanup()
    def reject(self):
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):audit.main(self.config)
    def test_valid_full_audit(self):
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(audit.main(self.config)['status'],'pass')
    def test_rehashed_subset(self):
        p=self.out/'predictions.jsonl';rows=p.read_text().splitlines();j=next(j for j,l in enumerate(rows) if json.loads(l)['variant']=='bottom');r=json.loads(rows[j]);self.assertNotEqual(r['kept_sha256'],'0'*64);r['kept_sha256']='0'*64;rows[j]=json.dumps(r);p.write_text('\n'.join(rows)+'\n');lock=self.out/'prediction-lock.json';d=json.loads(lock.read_text());d['predictions_sha256']=sha(p);lock.write_text(json.dumps(d));self.reject()
    def test_rehashed_disagreement(self):
        p=self.out/'disagreements.json';d=json.loads(p.read_text());row=next(r for r in d if r['variant']=='bottom');self.assertNotEqual(row['disagreement_deg'],999);row['disagreement_deg']=999;p.write_text(json.dumps(d));lock=self.out/'completion.json';d=json.loads(lock.read_text());d['disagreements_sha256']=sha(p);lock.write_text(json.dumps(d));self.reject()
    def test_rehashed_transition(self):
        p=self.out/'transitions.json';d=json.loads(p.read_text());d['count_to_bottom']['learned']['paired_rotation_ge2deg']['rgb_first']['1']['gained']+=1;p.write_text(json.dumps(d));lock=self.out/'completion.json';d=json.loads(lock.read_text());d['transitions_sha256']=sha(p);lock.write_text(json.dumps(d));self.reject()
if __name__=='__main__':unittest.main()
