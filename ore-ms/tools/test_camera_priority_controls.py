"""Finite policy controls and exhaustive proof of tight summary-level bounds."""
import itertools,unittest,tempfile,json,contextlib,io
from collections import defaultdict
from pathlib import Path
from camera_priority_controls import priority,priority_bounds,learned_table,choices,evaluate,Replay,sha
import audit_camera_priority_controls as independent

class PriorityControls(unittest.TestCase):
    def test_swapped_priority_truth_table(self):
        states=[{'qualified':False,'status':'input_unsupported'},{'qualified':True,'status':'failed'},{'qualified':True,'status':'ok'}]
        for rgb,thermal,first in itertools.product(states,states,['rgb','thermal']):
            rows={'rgb':rgb,'thermal':thermal};selected,calls=priority(first,rows);order=[first,'thermal' if first=='rgb' else 'rgb'];expected=[];sensor=None
            for a in order:
                if rows[a]['qualified']:expected.append(a)
                if rows[a]['status']=='ok':sensor=a;break
            self.assertEqual((selected,calls),(sensor,expected))
    def test_tight_bounds_all_four_case_populations(self):
        states=[(r,t) for r in [0,1,2] for t in [0,1,2]];groups=defaultdict(list)
        # 0=missing, 1=available/disagrees, 2=available/agrees.
        for population in itertools.product(states,repeat=4):
            nr=sum(r>0 for r,t in population);nt=sum(t>0 for r,t in population);union=sum(r>0 or t>0 for r,t in population);cr=sum(r==2 for r,t in population);ct=sum(t==2 for r,t in population);actual=sum(t==2 or (t==0 and r==2) for r,t in population);groups[nr,nt,union,cr,ct].append(actual)
        for (nr,nt,union,cr,ct),values in groups.items():
            bound=priority_bounds(4,nr,nt,union,cr,ct);self.assertEqual((bound['lower'],bound['upper']),(min(values),max(values)))
        self.assertGreater(len(groups),100)
    def test_documentary_limits_are_not_replay_scores(self):
        root=Path(__file__).resolve().parents[1]
        flash=learned_table(root/'docs/research/flashlight-transfer-findings-2026-11-16.md');smoke=learned_table(root/'docs/research/smoke-rgb-first-findings-2026-11-16.md')
        self.assertEqual((flash['anchors'],smoke['anchors']),(192,236))
        b=priority_bounds(192,192,168,192,176,57);self.assertEqual((b['lower'],b['upper']),(65,81));self.assertLess(b['upper'],176)
        b=priority_bounds(236,221,231,236,151,116);self.assertEqual((b['lower'],b['upper']),(116,121));self.assertLess(b['upper'],151)
    def fixture(self,rgood=False,tgood=False):
        raw=[];meta=[];errors=[]
        for a,ok in [('rgb',rgood),('thermal',tgood)]:
            meta.append(dict(arm=a,index=0,source=a+'0',target=a+'1',times_ns=[0,1],qualified=True))
            for f in ['sift','learned']:
                raw.append(dict(frontend=f,arm=a,index=0,status='ok' if ok else 'failed',qualified=True,seconds=2 if a=='rgb' else 3))
                errors.append(dict(frontend=f,arm=a,index=0,disagreement_deg=(.5 if a=='rgb' else 6) if ok else None))
        return raw,meta,errors
    def test_failed_requests_cost_and_no_fabricated_accuracy(self):
        raw,meta,errors=self.fixture();selected=choices(raw,meta,1);s=evaluate(raw,selected,errors,{'common_gyro':[0],'paired_rotation_ge2deg':[0]},1)['frontends']['learned']['all_declared']
        self.assertEqual(s['thermal_first']['matcher_calls'],2);self.assertEqual(s['thermal_first']['projected_seconds'],5);self.assertEqual(s['thermal_first']['outputs'],0);self.assertIsNone(s['thermal_first']['pose_accuracy'])
    def test_oracle_is_a_ceiling_not_thermal_priority(self):
        raw,meta,errors=self.fixture(True,True);selected=choices(raw,meta,1);s=evaluate(raw,selected,errors,{'common_gyro':[0],'paired_rotation_ge2deg':[0]},1)['frontends']['learned']['paired_rotation_ge2deg']
        self.assertEqual(s['thermal_first']['within1deg'],0);self.assertEqual(s['rgb_first']['within1deg'],1);self.assertEqual(s['headroom']['1']['oracle_either_agrees'],1);self.assertFalse(s['headroom']['1']['deployable'])

class AuditCorruptions(unittest.TestCase):
    def setUp(self):
        root=Path(__file__).resolve().parents[1];self.temp=tempfile.TemporaryDirectory(prefix='priority-tests-',dir=root/'results');base=Path(self.temp.name);parent=base/'matching';gyro=base/'gyro';self.out=base/'out';parent.mkdir();gyro.mkdir();raw,meta,errors=PriorityControls().fixture(True,True)
        self.write=lambda p,v:p.write_text(json.dumps(v));rel=lambda p:str(p.relative_to(root))
        (parent/'predictions.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in raw));self.write(parent/'prediction-lock.json',{'predictions_sha256':sha(parent/'predictions.jsonl')});self.write(parent/'matcher-inputs.json',meta);self.write(parent/'input-freeze.json',{'anchors':1});self.write(parent/'audit.json',{'status':'pass'})
        self.write(parent/'decisions.json',[{'frontend':f,'index':0,'policies':{'rgb_first':'rgb'}} for f in ['sift','learned']]);self.write(parent/'decision-lock.json',{'decisions_sha256':sha(parent/'decisions.json')})
        self.write(gyro/'disagreements.json',errors);self.write(gyro/'population.json',{'common_gyro':[0],'paired_rotation_ge2deg':[0]});self.write(gyro/'target-lock.json',{});self.write(gyro/'completion.json',{'disagreements_sha256':sha(gyro/'disagreements.json')});self.write(gyro/'audit.json',{'status':'pass'})
        self.cfg=base/'config.json';self.write(self.cfg,{'matching':rel(parent),'gyro':rel(gyro),'output':rel(self.out),'protocol':'docs/research/camera-priority-protocol-2026-11-16.md','historical_findings':{'FlashLight':'docs/research/flashlight-transfer-findings-2026-11-16.md','SmokeRoom':'docs/research/smoke-rgb-first-findings-2026-11-16.md'}})
        with contextlib.redirect_stdout(io.StringIO()):r=Replay(self.cfg);r.freeze();r.select();r.score()
    def tearDown(self):self.temp.cleanup()
    def test_fixture_audits(self):
        with contextlib.redirect_stdout(io.StringIO()):independent.main(self.cfg)
    def corrupt(self,filename,mutation,lockfile,key):
        p=self.out/filename;d=json.loads(p.read_text());mutation(d);self.write(p,d);lock=self.out/lockfile;l=json.loads(lock.read_text());l[key]=sha(p);self.write(lock,l)
        with self.assertRaises(AssertionError):independent.main(self.cfg)
    def test_reject_cost_corruption(self):
        self.corrupt('summary.json',lambda d:d['frontends']['learned']['all_declared']['thermal_first'].update(matcher_calls=9),'completion.json','summary_sha256')
    def test_reject_oracle_corruption(self):
        self.corrupt('summary.json',lambda d:d['frontends']['learned']['all_declared']['headroom']['1'].update(oracle_either_agrees=2),'completion.json','summary_sha256')
    def test_reject_documentary_bound_corruption(self):
        self.corrupt('historical-bounds.json',lambda d:d['FlashLight']['thresholds']['1'].update(upper=999),'completion.json','bounds_sha256')
    def test_reject_label_based_choice_corruption(self):
        self.corrupt('choices.json',lambda d:d[0]['policies']['thermal_first'].update(sensor='rgb'),'choice-lock.json','choices_sha256')

if __name__=='__main__':unittest.main()
