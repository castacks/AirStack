"""GPU-free support/cost regression and semantic audit corruption controls."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from subt_export_comparison import Comparison, sha, support_summary
import audit_subt_export_comparison as auditor

ROOT = Path(__file__).resolve().parents[1]


class SupportComparison(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='subt-support-tests-', dir=ROOT / 'results')
        self.base = Path(self.temp.name); self.out = self.base / 'out'; self.parent = self.base / 'parent'
        self.out.mkdir(); self.parent.mkdir()
        def write(p, value): p.write_text(json.dumps(value))
        self.write = write
        rows = []
        for a in ['rgb', 'thermal']:
            for i in range(4):
                rows.append(dict(arm=a, index=i, anchor_ns=i*500_000_000, source=f'{a}/{i}.npy', target=f'{a}/{i+1}.npy', times_ns=[i*500_000_000, (i+1)*500_000_000], qualified=not(a=='thermal' and i==0), reference_supported=False))
        write(self.parent/'intervals.json', rows)
        write(self.parent/'population.json', dict(common_reference=[], paired_rotation_ge2deg=[], translation_ge1cm_both=[]))
        write(self.parent/'calibration.json', {a:dict(effective_camera_to_imu=np.eye(4).tolist()) for a in ['rgb', 'thermal']})
        inputs=[{k:r[k] for k in ['arm','index','anchor_ns','source','target','times_ns','qualified']} for r in rows]
        write(self.out/'matcher-inputs.json', inputs); write(self.out/'input-freeze.json',dict(anchors=4)); write(self.out/'execution-freeze.json',dict(pins={}))
        write(self.out/'qualification.json',dict(model_states_unchanged=True))
        raw=[]
        valid={('sift','rgb'):[0,3],('sift','thermal'):[1,3],('learned','rgb'):[1],('learned','thermal'):[2,3]}
        for f in ['sift','learned']:
            for r in rows:
                ok=r['index'] in valid[f,r['arm']];p={k:r[k] for k in ['arm','index','qualified']};p.update(frontend=f,seconds=1. if r['arm']=='rgb' else 2.,matches=8 if ok else 0,status='ok' if ok else 'insufficient_matches' if r['qualified'] else 'input_unsupported')
                if r['qualified']:p.update(points0=[[1.,2.]]*p['matches'],points1=[[2.,3.]]*p['matches'])
                if ok:p.update(R=np.eye(3).tolist(),t=[1.,0.,0.],cheirality_inliers=8,pose_mask=[1]*8)
                raw.append(p)
        (self.out/'predictions.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in raw))
        write(self.out/'prediction-lock.json',dict(predictions_sha256=sha(self.out/'predictions.jsonl'),model_states_unchanged=True,scoring_started=False))
        rel=lambda p:str(p.relative_to(ROOT))
        self.config=self.base/'config.json'
        write(self.config,dict(output=rel(self.out),qualification=rel(self.parent),intrinsics='results/outdoornight-qualification/source/intrinsics.yaml',reference_free=True))
        with contextlib.redirect_stdout(io.StringIO()):
            runner=Comparison(self.config);runner.select();runner.score()

    def tearDown(self): self.temp.cleanup()

    def test_complete_fixture_passes_independent_audit(self):
        with contextlib.redirect_stdout(io.StringIO()):auditor.main(self.config)
        self.assertEqual(json.loads((self.out/'audit.json').read_text())['status'],'pass')

    def test_reference_backed_scoring_branch_is_retained(self):
        cfg=json.loads(self.config.read_text());cfg['reference_free']=False;self.write(self.config,cfg)
        rows=json.loads((self.parent/'intervals.json').read_text());truth=np.eye(4);truth[0,3]=1.
        for r in rows:r.update(reference_supported=True,target_from_source=truth.tolist(),reference_rotation_deg=0.,translation_m=1.)
        self.write(self.parent/'intervals.json',rows)
        self.write(self.parent/'population.json',dict(common_reference=list(range(4)),paired_rotation_ge2deg=[],translation_ge1cm_both=list(range(4))))
        for name in ['summary.json','completion.json']:(self.out/name).unlink()
        with contextlib.redirect_stdout(io.StringIO()):
            Comparison(self.config).score();auditor.main(self.config)
        s=json.loads((self.out/'summary.json').read_text())
        self.assertEqual(s['sift']['all_declared']['rgb_first']['R1'],3)
        self.assertEqual(s['learned']['all_declared']['rgb_first']['joint'],3)

    def test_support_and_cost_include_failed_calls(self):
        s=json.loads((self.out/'summary.json').read_text())['frontends']
        self.assertEqual(s['sift']['all_declared']['rgb_first']['outputs'],3)
        self.assertEqual(s['sift']['all_declared']['rgb_first']['matcher_calls'],6)
        self.assertEqual(s['sift']['all_declared']['rgb_first']['projected_seconds'],8.)
        self.assertEqual(s['learned']['all_declared']['rgb_first']['outputs'],3)
        self.assertEqual(s['learned']['all_declared']['rgb']['longest_no_output_run_anchors'],2)
        self.assertEqual(s['learned']['all_declared']['added']['outputs'],2)
        self.assertIsNone(s['learned']['all_declared']['rgb_first']['pose_accuracy'])
        self.assertEqual(sum(s['learned']['third'+str(i)]['rgb']['anchors'] for i in [1,2,3]),4)

    def corrupt_summary(self, mutation):
        s=json.loads((self.out/'summary.json').read_text());mutation(s);self.write(self.out/'summary.json',s)
        done=json.loads((self.out/'completion.json').read_text());done['summary_sha256']=sha(self.out/'summary.json');self.write(self.out/'completion.json',done)
        with self.assertRaises(AssertionError):auditor.main(self.config)

    def test_reject_invented_accuracy(self):
        self.corrupt_summary(lambda s:s.update(pose_accuracy=1.))

    def test_reject_omitted_failed_call(self):
        self.corrupt_summary(lambda s:s['frontends']['sift']['all_declared']['rgb_first'].update(matcher_calls=5))

    def test_reject_wrong_missing_run(self):
        self.corrupt_summary(lambda s:s['frontends']['learned']['all_declared']['rgb'].update(longest_no_output_run_anchors=1))

    def test_reject_wrong_sensor_selection(self):
        d=json.loads((self.out/'decisions.json').read_text());d[0]['policies']['rgb_first']='thermal';self.write(self.out/'decisions.json',d)
        lock=json.loads((self.out/'decision-lock.json').read_text());lock['decisions_sha256']=sha(self.out/'decisions.json');self.write(self.out/'decision-lock.json',lock)
        with self.assertRaises(AssertionError):auditor.main(self.config)


if __name__=='__main__':unittest.main()
