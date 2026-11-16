"""Mocap scalar angular-motion diagnostic; no model or reference-frame fitting."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import deployment_lighting as previous
ROOT=previous.ROOT;SOURCE=ROOT/'results/deployment-characterization';OUT=ROOT/'results/reference-contract';read=previous.read;sha=previous.sha;write=previous.write

def degrees(r):return float(np.degrees(Rotation.from_matrix(r).magnitude()))
def qualify():
 rng=np.random.default_rng(20261116);errors=[]
 for _ in range(100):
  r0,r1,x,g=Rotation.random(4,random_state=rng).as_matrix();body=r1.T@r0;camera=(g@r1@x).T@(g@r0@x);errors.append(abs(degrees(body)-degrees(camera)))
 assert max(errors)<1e-9
 wrong_a=Rotation.from_rotvec([.5,0,0]).as_matrix();wrong_b=Rotation.from_rotvec([0,.5,0]).as_matrix();assert abs(degrees(wrong_a)-degrees(wrong_b))<1e-12 and degrees(wrong_a@wrong_b.T)>30
 q=Rotation.random(random_state=rng).as_quat();np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(),Rotation.from_quat(-q).as_matrix(),atol=1e-15)
 files=[Path(__file__).resolve(),ROOT/'tools/deployment_lighting.py',ROOT/'tools/smokeseer_sift.py',ROOT/'docs/research/reference-contract-protocol-2026-11-16.md',SOURCE/'references.json',SOURCE/'predictions.jsonl',SOURCE/'prediction-lock.json',SOURCE/'execution-freeze.json',SOURCE/'common-reference-indices.json',*[SOURCE/(c+'-mocap.json') for c in ['bright','switching','dim']]]
 assert sha(SOURCE/'predictions.jsonl')==read(SOURCE/'prediction-lock.json')['predictions_sha256']
 parent=read(SOURCE/'execution-freeze.json');pins={str(p.relative_to(ROOT)):sha(p) for p in files}
 for k,h in pins.items():
  if k in parent['pins']:assert h==parent['pins'][k]
 write(OUT/'angle-qualification.json',{'random_conjugacy_cases':100,'max_conjugacy_difference_deg':max(errors),'wrong_axis_same_angle_control':True,'quaternion_sign_control':True,'new_inference_calls':0,'scope':'Scalar angle only; not full orientation or translation'})
 pins[str((OUT/'angle-qualification.json').relative_to(ROOT))]=sha(OUT/'angle-qualification.json');write(OUT/'angle-freeze.json',{'pins':pins,'runtime':'Same pinned geometry image as parent; CPU scoring only'})
 print('Scalar-angle source and synthetic controls frozen',flush=True)

def run():
 frozen=read(OUT/'angle-freeze.json')
 for k,h in frozen['pins'].items():assert sha(ROOT/k)==h,k
 references={(r['condition'],r['arm'],r['index']):r for r in read(SOURCE/'references.json')};poses={c:read(SOURCE/(c+'-mocap.json')) for c in ['bright','switching','dim']};common=read(SOURCE/'common-reference-indices.json');scores=[]
 for line in (SOURCE/'predictions.jsonl').read_text().splitlines():
  p=json.loads(line);ref=references[(p['condition'],p['arm'],p['index'])];r={k:p[k] for k in ['condition','arm','frontend','index','status']};r.update(common_reference=p['index'] in common[p['condition']],reference_angle_deg=None,estimated_angle_deg=None,angle_error_deg=None,success_1deg=False,identity_success_1deg=False)
  if r['common_reference']:
   a,b=[previous.interpolate(poses[p['condition']],t) for t in ref['times_ns']];assert a is not None and b is not None;theta=degrees(b[:3,:3].T@a[:3,:3]);r['reference_angle_deg']=theta;r['identity_success_1deg']=theta<=1
   if p['status']=='ok':
    pred=degrees(np.array(p['R']));r['estimated_angle_deg']=pred;r['angle_error_deg']=abs(pred-theta);r['success_1deg']=r['angle_error_deg']<=1
  scores.append(r)
 assert len(scores)==1684
 def aggregate(rows):
  n=len(rows);err=[r['angle_error_deg'] for r in rows if r['angle_error_deg'] is not None];return {'intervals':n,'supported':len(err),'success_1deg':sum(r['success_1deg'] for r in rows),'success_rate':sum(r['success_1deg'] for r in rows)/n if n else None,'identity_success_1deg':sum(r['identity_success_1deg'] for r in rows),'conditional_median_error_deg':float(np.median(err)) if err else None,'conditional_p90_error_deg':float(np.percentile(err,90)) if err else None}
 summary={}
 for c in ['bright','switching','dim']:
  summary[c]={}
  for f in ['sift','learned']:
   summary[c][f]={}
   for arm in ['rgb','thermal']:
    rows=[r for r in scores if r['condition']==c and r['frontend']==f and r['arm']==arm and r['common_reference']];summary[c][f][arm]={s:aggregate([r for r in rows if s=='all' or (r['reference_angle_deg']>=2 if s=='rotation_ge2deg' else r['reference_angle_deg']<2)]) for s in ['all','rotation_ge2deg','rotation_lt2deg']}
 write(OUT/'angle-scores.json',scores);write(OUT/'angle-summary.json',summary);write(OUT/'angle-completion.json',{'rows':len(scores),'new_inference_calls':0,'scores_sha256':sha(OUT/'angle-scores.json'),'summary_sha256':sha(OUT/'angle-summary.json'),'freeze_sha256':sha(OUT/'angle-freeze.json')});print(json.dumps(summary,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('stage',choices=['qualify','run']);a=p.parse_args();{'qualify':qualify,'run':run}[a.stage]()
