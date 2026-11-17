"""Finite count/subset controls and rehashed semantic audit corruptions."""
import contextlib,io,json,tempfile,shutil,unittest
from pathlib import Path
import numpy as np
import matched_count_control as runner
import audit_matched_count_control as audit
from deployment_lighting import sha
ROOT=runner.ROOT
class Finite(unittest.TestCase):
    def test_exact_sorted_unique_count_and_independent_ranking(self):
        for key in [('SmokeRoom','learned','rgb',7),('FlashLight','sift','thermal',0)]:
            for n in [0,1,7,8,31,250]:
                for k in sorted(set([0,n,n//2])):
                    a=runner.count_indices(n,k,*key);self.assertEqual(len(a),k);self.assertEqual(a.tolist(),sorted(set(a.tolist())));np.testing.assert_array_equal(a,audit.rank_indices(n,k,*key));np.testing.assert_array_equal(a,runner.count_indices(n,k,*key))
    def test_key_separation_and_invalid_count(self):
        a=runner.count_indices(100,25,'SmokeRoom','learned','rgb',7)
        for key in [('FlashLight','learned','rgb',7),('SmokeRoom','sift','rgb',7),('SmokeRoom','learned','thermal',7),('SmokeRoom','learned','rgb',8)]:self.assertFalse(np.array_equal(a,runner.count_indices(100,25,*key)))
        with self.assertRaises(AssertionError):runner.count_indices(3,4,'x','sift','rgb',0)
    def test_scoped_helper_restores_even_on_exception(self):
        before=runner.parent.VARIANTS
        with self.assertRaises(RuntimeError):
            with runner.variant_scope():
                self.assertEqual(runner.parent.VARIANTS,['original','bottom','count']);raise RuntimeError('control')
        self.assertIs(runner.parent.VARIANTS,before);self.assertEqual(before,['original','zero','bottom','combined'])
class Corruption(unittest.TestCase):
    def setUp(self):
        source=ROOT/'results/matched-count-control/source/config.json';cfg=json.loads(source.read_text());original=ROOT/cfg['output']
        if not (original/'completion.json').exists():self.skipTest('Full experiment not scored yet')
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'results',prefix='matched-count-tests-');self.out=Path(self.temp.name)/'out';shutil.copytree(original,self.out);cfg['output']=str(self.out.relative_to(ROOT));self.config=self.out.parent/'config.json';self.config.write_text(json.dumps(cfg));f=json.loads((self.out/'input-freeze.json').read_text());f['pins'][str(self.config.relative_to(ROOT))]=sha(self.config);(self.out/'input-freeze.json').write_text(json.dumps(f))
    def tearDown(self):
        if hasattr(self,'temp'):self.temp.cleanup()
    def reject(self):
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):audit.main(self.config)
    def test_valid_full_audit(self):
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(audit.main(self.config)['status'],'pass')
    def test_rehashed_count_fingerprint(self):
        p=self.out/'predictions.jsonl';rows=p.read_text().splitlines();j=next(j for j,l in enumerate(rows) if json.loads(l)['variant']=='count');r=json.loads(rows[j]);self.assertNotEqual(r['kept_sha256'],'0'*64);r['kept_sha256']='0'*64;rows[j]=json.dumps(r);p.write_text('\n'.join(rows)+'\n');lock=self.out/'prediction-lock.json';d=json.loads(lock.read_text());d['predictions_sha256']=sha(p);lock.write_text(json.dumps(d));self.reject()
    def test_rehashed_score(self):
        p=self.out/'scores.json';d=json.loads(p.read_text());row=next(r for r in d if r['variant']=='count');self.assertNotEqual(row['R_error'],999);row['R_error']=999;p.write_text(json.dumps(d));lock=self.out/'completion.json';d=json.loads(lock.read_text());d['scores_sha256']=sha(p);lock.write_text(json.dumps(d));self.reject()
    def test_rehashed_paired_contrast(self):
        p=self.out/'crop-count-contrast.json';d=json.loads(p.read_text());d['SmokeRoom']['learned']['paired_rotation_ge2deg']['rgb_first']['R5']['crop_correct']+=1;p.write_text(json.dumps(d));lock=self.out/'contrast-lock.json';d=json.loads(lock.read_text());d['crop_count_contrast_sha256']=sha(p);lock.write_text(json.dumps(d));self.reject()
if __name__=='__main__':unittest.main()
