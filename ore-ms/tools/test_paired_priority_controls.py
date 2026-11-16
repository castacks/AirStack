"""Finite supplied-reference cases and rehashed semantic-corruption controls."""
import contextlib,io,json,tempfile,unittest,shutil
from pathlib import Path
from paired_priority_controls import evaluate,choices,ROOT,sha
import audit_paired_priority_controls as auditor

def fixture():
    raw=[];scores=[];meta=[]
    for a in ['rgb','thermal']:
        for i in range(4):
            qualified=not(a=='thermal' and i==2);ok=(a=='rgb' and i in [0,2]) or (a=='thermal' and i in [0,1]);status='ok' if ok else 'failed' if qualified else 'input_unsupported';meta.append({'arm':a,'index':i,'qualified':qualified,'source':a+str(i),'target':a+str(i+1),'times_ns':[i*10,i*10+1]})
            for f in ['sift','learned']:
                raw.append({'frontend':f,'arm':a,'index':i,'qualified':qualified,'status':status,'seconds':(2 if a=='rgb' else 3) if qualified else 0})
                err=(.5 if a=='rgb' else 6 if i==0 else 2) if ok and i!=2 else None;t=(30 if a=='rgb' else 10) if err is not None else None;scores.append({'frontend':f,'arm':a,'index':i,'status':status,'R_error':err,'t_error':t,'R1':err is not None and err<=1,'R5':err is not None and err<=5,'joint':err is not None and err<=5 and t<=15,'reference_rotation_deg':3 if i in [0,1] else .1 if i==3 else None})
    pop={'common_reference':[0,1,3],'paired_rotation_ge2deg':[0,1],'translation_ge1cm_both':[0,1]};return raw,meta,scores,pop

class FiniteCases(unittest.TestCase):
    def run_case(self):
        raw,meta,scores,pop=fixture();return evaluate(raw,choices(raw,meta,4),scores,pop,4)['frontends']['learned']
    def test_priority_can_keep_wrong_available_pose(self):
        s=self.run_case()['paired_rotation_ge2deg'];self.assertEqual((s['rgb_first']['R1'],s['thermal_first']['R1']),(1,0));self.assertEqual((s['rgb_first']['R5'],s['thermal_first']['R5']),(2,1));self.assertEqual(s['headroom']['R5']['oracle_either_correct'],2)
    def test_reference_missing_is_not_pose_failure(self):
        s=self.run_case()['all_declared']['rgb_first'];self.assertEqual((s['anchors'],s['outputs'],s['rotation_eligible'],s['rotation_compared_outputs']),(4,3,3,2));self.assertEqual(s['translation_eligible'],2)
    def test_failed_calls_cost_but_unsupported_calls_do_not(self):
        s=self.run_case()['all_declared'];self.assertEqual((s['rgb_first']['matcher_calls'],s['rgb_first']['projected_seconds']),(6,14));self.assertEqual((s['thermal_first']['matcher_calls'],s['thermal_first']['projected_seconds']),(5,13));self.assertEqual(s['rgb_first']['outputs'],s['thermal_first']['outputs'])
    def test_unsupported_input_check_time_is_not_a_matcher_call(self):
        raw,meta,scores,pop=fixture()
        for r in raw:
            if not r['qualified']:r['seconds']=.125
        s=evaluate(raw,choices(raw,meta,4),scores,pop,4)['frontends']['learned']['all_declared']['thermal_first'];self.assertEqual(s['matcher_calls'],5);self.assertEqual(s['unsupported_primary_seconds'],.125);self.assertEqual(s['projected_seconds'],13.125)
    def test_joint_uses_translation_and_is_not_rotation_only(self):
        s=self.run_case()['paired_rotation_ge2deg'];self.assertEqual(s['rgb_first']['R5'],2);self.assertEqual(s['rgb_first']['joint'],1);self.assertEqual(s['headroom']['joint']['oracle_either_correct'],1);self.assertFalse(s['headroom']['joint']['deployable'])
    def test_static_identity_control_and_conflict_attribution(self):
        s=self.run_case()['common_reference'];self.assertEqual(s['rgb_or_zero_rotation'],{'1':2,'5':3});h=s['headroom']['R1'];self.assertEqual(h['thermal_wrong_rgb_correct'],1);self.assertEqual(h['no_correct_available_proposal'],2)

class SemanticAudit(unittest.TestCase):
    def setUp(self):
        cfg=ROOT/'results/paired-priority-controls/source/flash-config.json';self.cfg_source=cfg
        if not cfg.exists() or not (ROOT/'results/paired-priority-controls/flash/completion.json').exists():self.skipTest('Real frozen replay not yet scored')
        self.temp=tempfile.TemporaryDirectory(prefix='paired-priority-test-',dir=ROOT/'results');self.base=Path(self.temp.name);self.out=self.base/'out';shutil.copytree(ROOT/'results/paired-priority-controls/flash',self.out);d=json.loads(cfg.read_text());d['output']=str(self.out.relative_to(ROOT));self.cfg=self.base/'config.json';self.cfg.write_text(json.dumps(d));freeze=json.loads((self.out/'input-freeze.json').read_text());freeze['pins'][str(self.cfg.relative_to(ROOT))]=sha(self.cfg);(self.out/'input-freeze.json').write_text(json.dumps(freeze))
    def tearDown(self):
        if hasattr(self,'temp'):self.temp.cleanup()
    def test_valid_complete_replay(self):
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(auditor.main(self.cfg)['status'],'pass')
    def corrupt(self,filename,mutate,receipt,key):
        path=self.out/filename;d=json.loads(path.read_text());mutate(d);path.write_text(json.dumps(d));r=self.out/receipt;v=json.loads(r.read_text());v[key]=sha(path);r.write_text(json.dumps(v))
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):auditor.main(self.cfg)
    def test_reject_rehashed_camera_choice(self):self.corrupt('choices.json',lambda d:d[0]['policies']['thermal_first'].update(sensor='rgb'),'choice-lock.json','choices_sha256')
    def test_reject_rehashed_cost(self):self.corrupt('summary.json',lambda d:d['frontends']['learned']['all_declared']['thermal_first'].update(matcher_calls=9999),'completion.json','summary_sha256')
    def test_reject_rehashed_oracle(self):self.corrupt('summary.json',lambda d:d['frontends']['learned']['paired_rotation_ge2deg']['headroom']['R1'].update(oracle_either_correct=9999),'completion.json','summary_sha256')
    def test_reject_rehashed_joint_eligibility(self):self.corrupt('summary.json',lambda d:d['frontends']['learned']['all_declared']['thermal_first'].update(translation_eligible=9999),'completion.json','summary_sha256')
    def test_reject_rehashed_target_relabelling(self):self.corrupt('summary.json',lambda d:d.update(independent_physical_truth=True),'completion.json','summary_sha256')

if __name__=='__main__':unittest.main()
