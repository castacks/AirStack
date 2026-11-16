"""Nominal IMU-contract reference scoring of immutable SmokeRoom predictions."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation,Slerp
from deployment_lighting import sha,read,write
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/smoke-reference-contract';PARENT=ROOT/'results/deployment-smoke-frontend'

def calibration():
 source=ROOT/'results/deployment-data-search/smoke_extrinsics.yaml';v=yaml.safe_load(source.read_text());transforms={};report={}
 for arm in ['rgb','thermal']:
  original=np.array(v[arm+'_camera_to_imu']['data']).reshape(4,4);assert np.isfinite(original).all();u,s,vt=np.linalg.svd(original[:3,:3]);r=u@vt;assert np.linalg.det(r)>0 and np.linalg.det(original[:3,:3])>0;correction=float(np.linalg.norm(r-original[:3,:3]));assert correction<1e-5
  t=np.eye(4);t[:3,:3]=r;t[:3,3]=original[:3,3];transforms[arm]=t;report[arm]={'source_matrix':original.tolist(),'effective_camera_to_imu':t.tolist(),'rotation_roundoff_frobenius':correction,'source_last_row':original[3].tolist(),'normalization':'Supplied top3x4, proper homogeneous bottom row; closestSO3 for tiny export roundoff only. No fit.'}
 return transforms,report
class Reference:
 def __init__(self):
  d=np.load(OUT/'reference.npz');self.t=d['time_ns'];self.p=d['position'];self.q=d['quaternion_xyzw'];assert np.all(np.diff(self.t)>0) and np.isfinite(self.p).all() and np.isfinite(self.q).all();assert np.max(abs(np.linalg.norm(self.q,axis=1)-1))<1e-3;self.origin=int(self.t[0]);self.rot=Slerp((self.t-self.origin)*1e-9,Rotation.from_quat(self.q))
 def pose(self,t):
  j=int(np.searchsorted(self.t,t))
  if j==len(self.t) or t<int(self.t[0]):return None
  if self.t[j]==t:position=self.p[j];r=Rotation.from_quat(self.q[j]).as_matrix()
  else:
   if j==0 or self.t[j]-self.t[j-1]>50_000_000:return None
   a=(t-int(self.t[j-1]))/int(self.t[j]-self.t[j-1]);position=(1-a)*self.p[j-1]+a*self.p[j];r=self.rot([(t-self.origin)*1e-9]).as_matrix()[0]
  out=np.eye(4);out[:3,:3]=r;out[:3,3]=position;return out

def qualify():
 raw=read(OUT/'raw-binding-qualification.json');assert raw['images_bound']==22800 and raw['observed_axes_timing_consistency_pass'];assert raw['csv_imu_equal_on_all_raw_support'];assert len(raw['reference_times_without_raw_sample'])==2
 transforms,report=calibration();write(OUT/'calibration-normalization.json',report);reference=Reference();rows=read(PARENT/'intervals.json');binding={x['cache_path']:x for x in read(OUT/'image-bindings.json')};bound=[];rng=np.random.default_rng(20261116)
 for _ in range(100):
  a,b,x=Rotation.random(3,random_state=rng).as_matrix();t0=np.eye(4);t1=np.eye(4);sensor=np.eye(4);t0[:3,:3]=a;t1[:3,:3]=b;sensor[:3,:3]=x;t0[:3,3]=rng.normal(size=3);t1[:3,3]=rng.normal(size=3);sensor[:3,3]=rng.normal(size=3);r=np.linalg.solve(t1@sensor,t0@sensor);np.testing.assert_allclose((t1@sensor)@r,t0@sensor,atol=1e-12)
 for row in rows:
  for key,t in zip(['source','target'],row['times_ns']):assert binding[row[key]]['time_ns']==t and binding[row[key]]['arm']==row['arm']
  pose=[reference.pose(t) for t in row['times_ns']];r={k:row[k] for k in ['arm','index','times_ns','source','target']};r['reference_supported']=all(t is not None for t in pose)
  if r['reference_supported']:
   a,b=[p@transforms[row['arm']] for p in pose];relative=np.linalg.solve(b,a);r['target_from_source']=relative.tolist();r['translation_m']=float(np.linalg.norm(relative[:3,3]));r['reference_rotation_deg']=float(np.degrees(Rotation.from_matrix(relative[:3,:3]).magnitude()))
  bound.append(r)
 bykey={(x['arm'],x['index']):x for x in bound};common=[i for i in sorted({x['index'] for x in bound}) if all(bykey[(a,i)]['reference_supported'] for a in ['rgb','thermal'])];translation=[i for i in common if all(bykey[(a,i)]['translation_m']>=.01 for a in ['rgb','thermal'])]
 write(OUT/'camera-reference.json',bound);write(OUT/'pose-population.json',{'declared_sensor_intervals':len(bound),'common_reference_indices':common,'translation_ge1cm_both_indices':translation,'unsupported':[{k:r[k] for k in ['arm','index','times_ns']} for r in bound if not r['reference_supported']],'raw_imu_missing_samples_retained':raw['reference_times_without_raw_sample'],'scope':'Nominal declared IMU reference contract; no independent origin/exposure metrology'})
 parent=read(PARENT/'execution-freeze.json');assert sha(PARENT/'predictions.jsonl')==read(PARENT/'prediction-lock.json')['predictions_sha256'];files=[Path(__file__).resolve(),ROOT/'tools/deployment_lighting.py',ROOT/'tools/smokeseer_sift.py',ROOT/'docs/research/smoke-reference-pose-protocol-2026-11-16.md',ROOT/'results/deployment-data-search/smoke_extrinsics.yaml',ROOT/'results/deployment-data-search/subt-smoke-gt.zip',PARENT/'intervals.json',PARENT/'images.json',PARENT/'predictions.jsonl',PARENT/'prediction-lock.json',PARENT/'execution-freeze.json',OUT/'reference.npz',OUT/'raw-gyro.npz',OUT/'image-bindings.json',OUT/'raw-binding-qualification.json',OUT/'gyro-reference-comparison.json',OUT/'camera-reference.json',OUT/'pose-population.json',OUT/'calibration-normalization.json',OUT/'contract-gates.json',OUT/'contract-gates-v2.json',OUT/'fusion-track.html']
 pins={str(p.relative_to(ROOT)):sha(p) for p in files}
 for k,h in pins.items():
  if k in parent['pins']:assert h==parent['pins'][k]
 write(OUT/'pose-freeze.json',{'pins':pins,'new_inference_calls':0,'reference_scope':'Supplied fused IMU-frame convention plus observed gyro consistency; independent physical truth not certified'})
 print('Frozen full-span references:',len(common),'common anchors;',len(translation),'translation-qualified',flush=True)

def score():
 plan=read(OUT/'pose-freeze.json')
 for k,h in plan['pins'].items():assert sha(ROOT/k)==h,k
 ref={(x['arm'],x['index']):x for x in read(OUT/'camera-reference.json')};pop=read(OUT/'pose-population.json');common=set(pop['common_reference_indices']);trans=set(pop['translation_ge1cm_both_indices']);scores=[]
 for line in (PARENT/'predictions.jsonl').read_text().splitlines():
  p=json.loads(line);r=ref[(p['arm'],p['index'])];v={k:p[k] for k in ['arm','frontend','index','status']};v.update(common_reference=p['index'] in common,translation_qualified=p['index'] in trans,rotation_error_deg=None,translation_error_deg=None,rotation_success_1deg=False,rotation_success_5deg=False,joint_success_5deg_15deg=False,identity_rotation_success_1deg=False,reference_rotation_deg=r.get('reference_rotation_deg'))
  if v['common_reference']:
   truth=np.array(r['target_from_source']);v['identity_rotation_success_1deg']=r['reference_rotation_deg']<=1
   if p['status']=='ok':
    error=float(np.degrees(Rotation.from_matrix(np.array(p['R'])@truth[:3,:3].T).magnitude()));v.update(rotation_error_deg=error,rotation_success_1deg=error<=1,rotation_success_5deg=error<=5)
    if v['translation_qualified']:
     a=np.array(p['t']);b=truth[:3,3];error_t=float(np.degrees(np.arccos(np.clip(a@b/(np.linalg.norm(a)*np.linalg.norm(b)),-1,1))));v.update(translation_error_deg=error_t,joint_success_5deg_15deg=bool(error<=5 and error_t<=15))
  scores.append(v)
 assert len(scores)==3352
 def aggregate(rows):
  n=len(rows);rr=[r['rotation_error_deg'] for r in rows if r['rotation_error_deg'] is not None];tt=[r['translation_error_deg'] for r in rows if r['translation_error_deg'] is not None];return {'intervals':n,'supported':len(rr),'rotation_success_1deg':sum(r['rotation_success_1deg'] for r in rows),'rotation_success_5deg':sum(r['rotation_success_5deg'] for r in rows),'identity_rotation_success_1deg':sum(r['identity_rotation_success_1deg'] for r in rows),'translation_qualified':sum(r['translation_qualified'] for r in rows),'joint_success_5deg_15deg':sum(r['joint_success_5deg_15deg'] for r in rows),'conditional_rotation_median_deg':float(np.median(rr)) if rr else None,'conditional_rotation_p90_deg':float(np.percentile(rr,90)) if rr else None,'conditional_translation_median_deg':float(np.median(tt)) if tt else None,'conditional_translation_p90_deg':float(np.percentile(tt,90)) if tt else None}
 summary={}
 for f in ['sift','learned']:
  summary[f]={}
  for arm in ['rgb','thermal']:
   base=[r for r in scores if r['frontend']==f and r['arm']==arm and r['common_reference']];strata={'all':base,'reference_rotation_ge2deg':[r for r in base if r['reference_rotation_deg']>=2],'reference_rotation_lt2deg':[r for r in base if r['reference_rotation_deg']<2],'translation_ge1cm_both':[r for r in base if r['translation_qualified']]};strata.update({'third'+str(j+1):[r for r in base if r['index']*3//838==j] for j in range(3)});summary[f][arm]={k:aggregate(v) for k,v in strata.items()}
 write(OUT/'pose-scores.json',scores);write(OUT/'pose-summary.json',summary);write(OUT/'pose-completion.json',{'rows':len(scores),'new_inference_calls':0,'scores_sha256':sha(OUT/'pose-scores.json'),'summary_sha256':sha(OUT/'pose-summary.json'),'freeze_sha256':sha(OUT/'pose-freeze.json')});print(json.dumps(summary,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('stage',choices=['qualify','score']);a=p.parse_args();{'qualify':qualify,'score':score}[a.stage]()
