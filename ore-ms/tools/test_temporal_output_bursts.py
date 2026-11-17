"""Known gap/censor/span controls and positive/rehashed full audits."""
import contextlib,io,json,tempfile,shutil,unittest
from pathlib import Path
import temporal_output_bursts as runner
import audit_temporal_output_bursts as audit
from deployment_lighting import sha
ROOT=runner.ROOT
class Finite(unittest.TestCase):
    def test_gap_and_cohort_boundaries_do_not_bridge(self):
        states=['missing','disagree','unknown','missing','good','outside','disagree'];times=[i*500000000 for i in range(len(states))];ep,s=runner.describe(states,times);self.assertEqual(ep,audit.stream(states,times));self.assertEqual([(e['first_index'],e['last_index']) for e in ep],[(0,1),(3,3),(6,6)]);self.assertEqual(s['bad'],4);self.assertEqual(s['unknown'],1);self.assertEqual(s['outside'],1);self.assertEqual(s['observed_returns'],1);self.assertIsNone(ep[0]['observed_return_lag_ns']);self.assertEqual(ep[1]['observed_return_lag_ns'],500000000);self.assertTrue(ep[-1]['right_censored'])
    def test_clean_singleton_and_terminal_censoring(self):
        ep,s=runner.describe(['good']*3,[0,500000000,1000000000]);self.assertEqual(ep,[]);self.assertEqual(s['length_histogram'],{});self.assertIsNone(s['longest_length']);self.assertIsNone(s['conditional_max_return_lag_ns']);ep,s=runner.describe(['good','missing'],[0,500000000]);self.assertFalse(ep[0]['left_censored']);self.assertTrue(ep[0]['right_censored']);self.assertEqual(ep[0]['sampled_span_ns'],0);self.assertIsNone(ep[0]['observed_return_lag_ns'])
    def test_mixed_failure_block_and_known_return(self):
        states=['good','missing','disagree','disagree','good'];times=[i*500000000 for i in range(5)];ep,s=runner.describe(states,times);self.assertEqual(ep,audit.stream(states,times));self.assertEqual(len(ep),1);self.assertEqual(ep[0]['length'],3);self.assertEqual(ep[0]['missing'],1);self.assertEqual(ep[0]['disagree'],2);self.assertEqual(ep[0]['sampled_span_ns'],1000000000);self.assertEqual(ep[0]['observed_return_lag_ns'],1500000000);self.assertEqual(s['length_histogram'],{'3':1});self.assertFalse(ep[0]['left_censored'] or ep[0]['right_censored'])
    def test_reference_priority_moving_and_exact_thresholds(self):
        e={'moving':False,'common':False,'sensor':None,'error_deg':None};self.assertEqual(runner.state(e,'full','R5'),'unknown');self.assertEqual(runner.state(e,'moving','R5'),'outside');self.assertEqual(runner.state(e,'full','missing'),'missing');e.update(moving=True,common=True,sensor='rgb',error_deg=1.);self.assertEqual(runner.state(e,'moving','R1'),'good');e['error_deg']=5.;self.assertEqual(runner.state(e,'full','R5'),'good');self.assertEqual(runner.state(e,'full','R1'),'disagree')
class Corruption(unittest.TestCase):
    def setUp(self):
        source=ROOT/'results/temporal-output-bursts/source/config.json';cfg=json.loads(source.read_text());original=ROOT/cfg['output']
        if not (original/'completion.json').exists():self.skipTest('Temporal results not complete yet')
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'results',prefix='temporal-burst-tests-');self.out=Path(self.temp.name)/'out';shutil.copytree(original,self.out);cfg['output']=str(self.out.relative_to(ROOT));self.config=self.out.parent/'config.json';self.config.write_text(json.dumps(cfg));f=json.loads((self.out/'input-freeze.json').read_text());f['pins'][str(self.config.relative_to(ROOT))]=sha(self.config);(self.out/'input-freeze.json').write_text(json.dumps(f))
    def tearDown(self):
        if hasattr(self,'temp'):self.temp.cleanup()
    def relock(self,name):
        lock=self.out/'completion.json';d=json.loads(lock.read_text());d[name+'_sha256']=sha(self.out/(name+'.json'));lock.write_text(json.dumps(d))
    def reject(self):
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):audit.main(self.config)
    def test_valid_full_audit(self):
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(audit.main(self.config)['status'],'pass')
    def test_rehashed_event(self):
        p=self.out/'events.json';d=json.loads(p.read_text());self.assertNotEqual(d[0]['error_deg'],999);d[0]['error_deg']=999;p.write_text(json.dumps(d));self.relock('events');self.reject()
    def test_rehashed_episode_censoring(self):
        p=self.out/'episodes.json';d=json.loads(p.read_text());d[0]['right_censored']=not d[0]['right_censored'];p.write_text(json.dumps(d));self.relock('episodes');self.reject()
    def test_rehashed_summary(self):
        p=self.out/'summary.json';d=json.loads(p.read_text());d[0]['bad']+=1;p.write_text(json.dumps(d));self.relock('summary');self.reject()
if __name__=='__main__':unittest.main()
