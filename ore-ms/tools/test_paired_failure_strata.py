"""Descriptor boundary cases and rehashed stratum corruption controls."""
import contextlib,io,json,tempfile,shutil,unittest
from pathlib import Path
from paired_failure_strata import features,runs,describe,ROOT,sha
import audit_paired_failure_strata as audit

class FiniteCases(unittest.TestCase):
    def test_zero_match_fraction_stays_null_and_endpoint_minimum(self):
        raw=[{'frontend':'learned','arm':'rgb','index':0,'status':'failed','qualified':True,'matches':0}];inputs=[{'arm':'rgb','index':0,'anchor_ns':0,'source':'s','target':'t','times_ns':[0,1]}];images=[{'path':'s','metrics':{'std':10,'p05_p95_range':30}},{'path':'t','metrics':{'std':20,'p05_p95_range':15}}];r=features(raw,inputs,images)[0];self.assertIsNone(r['inlier_fraction']);self.assertEqual((r['gray_std_min'],r['gray_range_min']),(10,15));self.assertEqual(r['inliers'],0)
    def test_runs_do_not_bridge_a_missing_anchor(self):self.assertEqual(runs([1,2,4,5,8]),[[1,2],[4,5],[8]])
    def test_empty_stratum_and_quartiles(self):
        f={(a,i):{'matches':v,'inliers':v,'inlier_fraction':None,'gray_std_min':v,'gray_range_min':v} for a in ['rgb','thermal'] for i,v in [(0,2),(1,6)]};cases=[{'endpoint':'R1','cell':'both_correct','index':i,'both_supported':True} for i in [0,1]];r=describe(cases,f,4);self.assertEqual(r['R1']['both_correct']['features']['rgb']['matches']['quartiles'],[3,4,5]);self.assertEqual(r['R1']['both_correct']['features']['rgb']['inlier_fraction'],{'n':0,'missing':2,'quartiles':None});self.assertEqual(r['R5']['neither_correct']['count'],0)

class SemanticCorruption(unittest.TestCase):
    def setUp(self):
        src=ROOT/'results/paired-failure-strata/source/config.json'
        if not src.exists():self.skipTest('Real replay not yet joined')
        cfg=json.loads(src.read_text());original=ROOT/cfg['output']
        if not (original/'completion.json').exists():self.skipTest('Real replay not yet joined')
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'results',prefix='failure-strata-test-');self.out=Path(self.temp.name)/'out';shutil.copytree(original,self.out);cfg['output']=str(self.out.relative_to(ROOT));self.config=Path(self.temp.name)/'config.json';self.config.write_text(json.dumps(cfg));f=json.loads((self.out/'input-freeze.json').read_text());f['pins'][str(self.config.relative_to(ROOT))]=sha(self.config);(self.out/'input-freeze.json').write_text(json.dumps(f))
    def tearDown(self):
        if hasattr(self,'temp'):self.temp.cleanup()
    def test_valid_replay(self):
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(audit.main(self.config)['status'],'pass')
    def corrupt(self,file,mutate):
        p=self.out/(file+'.json');d=json.loads(p.read_text());mutate(d);p.write_text(json.dumps(d));receipt=self.out/('feature-lock.json' if file=='features' else 'sample-lock.json' if file=='samples' else 'completion.json');r=json.loads(receipt.read_text());r[file+'_sha256']=sha(p);receipt.write_text(json.dumps(r))
        if file=='samples':
            p=self.out/'completion.json';r=json.loads(p.read_text());r['samples_sha256']=sha(self.out/'samples.json');p.write_text(json.dumps(r))
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):audit.main(self.config)
    def test_reject_feature_corruption(self):self.corrupt('features',lambda d:d['FlashLight'][0].update(matches=d['FlashLight'][0]['matches']+1))
    def test_reject_case_corruption(self):
        def mutate(d):
            r=d['FlashLight'][0];r['cell']='both_correct' if r['cell']!='both_correct' else 'neither_correct'
        self.corrupt('cases',mutate)
    def test_reject_summary_corruption(self):self.corrupt('summary',lambda d:d['SmokeRoom']['R5']['both_correct'].update(count=99999))
    def test_reject_sample_corruption(self):self.corrupt('samples',lambda d:d['SmokeRoom'][0].update(index=d['SmokeRoom'][0]['index']+1))
if __name__=='__main__':unittest.main()
