"""Independent native endpoint/support/call and scalar-score reconstruction."""
import json
from decimal import Decimal
import math
import hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/native-gap-replay/run-001'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pose(v):
 x,y,z,w=np.asarray(v[3:],float);norm=math.sqrt(x*x+y*y+z*z+w*w);x,y,z,w=[q/norm for q in [x,y,z,w]]
 m=np.eye(4);m[:3,3]=v[:3];m[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
 [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];return m
def inverse(m):
 out=np.eye(4);out[:3,:3]=m[:3,:3].T;out[:3,3]=-m[:3,:3].T@m[:3,3];return out

def reference_at(directory, times):
 directory=ROOT/directory/'super_odometry/odometry'
 clock=np.asarray([int(Decimal(x)*1_000_000_000) for x in (directory/'interp_timestamps.txt').read_text().splitlines()],dtype=np.int64)
 values=np.loadtxt(directory/'interp_data.txt');out=[]
 for t in times:
  assert clock[0]<=t<=clock[-1]
  i=min(max(int(np.searchsorted(clock,t,side='right'))-1,0),len(clock)-2)
  fraction=(int(t)-int(clock[i]))/(int(clock[i+1])-int(clock[i]))
  xyz=values[i,:3]+fraction*(values[i+1,:3]-values[i,:3])
  q0=values[i,3:7]/np.linalg.norm(values[i,3:7]);q1=values[i+1,3:7]/np.linalg.norm(values[i+1,3:7])
  dot=float(q0@q1)
  if dot<0:q1=-q1;dot=-dot
  angle=math.acos(min(1.,max(-1.,dot)))
  if angle<1e-8:q=q0*(1-fraction)+q1*fraction
  else:q=(math.sin((1-fraction)*angle)*q0+math.sin(fraction*angle)*q1)/math.sin(angle)
  out.append(pose(np.r_[xyz,q]))
 return out

def main():
 plan=read(OUT/'plan.json');scores=read(OUT/'scores.json');lock=read(OUT/'capture-lock.json');checks=0
 for n,h in plan['input_source_sha256'].items():assert sha(ROOT/n)==h,n;checks+=1
 assert scores['capture_lock_sha256']==sha(OUT/'capture-lock.json') and scores['scorer_sha256']==sha(ROOT/'tools/native_gap_evaluate.py')
 for n,h in lock['sha256'].items():assert sha(ROOT/n)==h,n;checks+=1
 cases={(c['window'],c['camera'],c['arm']):c for c in plan['cases']};data={}
 for key,c in cases.items():
  mapping=read(ROOT/c['mapping']);rows=read(ROOT/c['manifest'])['pairs'];base=ROOT/c['capture']/c['camera']
  original=read(ROOT/c['dataset']/'manifest.json')['pairs']
  assert rows==[original[i] for i in mapping['source_indices']];checks+=len(rows)
  with np.load(base/'trajectory.npz') as x:poses=np.asarray([pose(v) for v in x['poses_camera_ned']]);times=x['time_ns'].reshape(-1)
  assert times.tolist()==[r[c['camera']+'_left']['time_ns'] for r in rows];checks+=len(rows)
  events=[json.loads(l) for l in (base/'arbiter-events/events.jsonl').read_text().splitlines()]
  ends={(e['time0_ns'],e['time1_ns']):e for e in events if e['kind']=='pair_end'}
  first={}
  for e in events:
   if e['kind']=='motion_result_written' and e['first_write_for_submission']:first[e['time0_ns'],e['time1_ns']]=e
  with np.load(base/'tracking-flags.npz') as x:flags=x['before_refinement'].reshape(-1)
  support=[];stages=[]
  for i in range(len(rows)-1):
   ts=(int(times[i]),int(times[i+1]));h=ends.get(ts);w=first.get(ts)
   support.append(bool(h is not None and h['execution_error'] is None and not h['original_need_interp'] and not flags[i+1] and w is not None and not w['original_need_interp'] and w['derived_camera_delta_xyzw'] is not None))
   stages.append(w['stage'] if w else None);checks+=1
  cost=read(base/'native-cost.json');pair=[r for r in cost['frontend_calls'] if r['kind']=='frontend_pair'];depth=[r for r in cost['frontend_calls'] if r['kind']=='frontend_depth']
  assert len(pair)==len(rows)-1 and len(depth)==2;checks+=2
  assert all(r['duration_ns']>=0 and r['duration_ns']==r['end_ns']-r['start_ns'] for r in cost['native_phases']+cost['frontend_calls']);checks+=len(cost['native_phases'])+len(cost['frontend_calls'])
  data[key]=(poses,times,support,stages,mapping,cost)
 for r in scores['rows']:
  w,c=r['window'],r['camera'];k=r['native_pair'];d=data[w,c,'dense'];s=data[w,c,'sparse'];ids=s[4]['source_indices'];a,b=ids[k:k+2]
  assert (a,b)==(r['source_start'],r['source_end']) and r['skipped_source_frames']==b-a-1
  assert r['is_adjacent_source_pair']==(b==a+1) and r['sensor_interval_ns']==[int(s[1][k]),int(s[1][k+1])]
  assert r['motion_written']==s[2][k] and r['write_stage']==s[3][k]
  common=s[2][k] and all(d[2][a:b]);assert common==r['common_supported'];checks+=8
  if not common:assert r['losses'] is None;continue
  T=np.asarray(cases[w,c,'sparse']['nominal_body_from_camera'][c]);Ti=inverse(T)
  delta_d=T@inverse(d[0][a])@d[0][b]@Ti;delta_s=T@inverse(s[0][k])@s[0][k+1]@Ti
  assert np.allclose(delta_d,r['dense_body_delta'],atol=1e-10) and np.allclose(delta_s,r['sparse_body_delta'],atol=1e-10);checks+=2
  # Reference matrices are separately provenance-pinned; arithmetic uses independent pose and rotation formulas.
  target=np.asarray(r['reference_delta'])
  rr=reference_at(cases[w,c,'sparse']['reference_dataset'],r['sensor_interval_ns']);independent_target=inverse(rr[0])@rr[1]
  assert np.allclose(target,independent_target,atol=1e-8,rtol=1e-8);checks+=1
  for label,pred,ref in [('dense_proxy',delta_d,target),('sparse_proxy',delta_s,target),('sparse_dense_disagreement',delta_s,delta_d)]:
   t=math.sqrt(sum(float(v)**2 for v in pred[:3,3]-ref[:3,3]))
   rot=ref[:3,:3].T@pred[:3,:3];angle=math.degrees(math.atan2(np.linalg.norm([rot[2,1]-rot[1,2],rot[0,2]-rot[2,0],rot[1,0]-rot[0,1]])/2,(np.trace(rot)-1)/2))
   assert math.isclose(t,r['losses'][label]['translation_m'],rel_tol=1e-7,abs_tol=1e-9)
   assert math.isclose(angle,r['losses'][label]['rotation_deg'],rel_tol=1e-6,abs_tol=1e-7);checks+=2
 for key,g in scores['groups'].items():
  window,camera=key.split('/');selected=[r for r in scores['rows'] if r['window']==window and r['camera']==camera]
  for label,group in [('all',selected),('gap',[r for r in selected if not r['is_adjacent_source_pair']]),('adjacent',[r for r in selected if r['is_adjacent_source_pair']])]:
   summary=g['strata'][label];eligible=[r for r in group if r['common_supported']]
   assert summary['intervals']==len(group) and summary['common_supported']==len(eligible);checks+=2
   for method in ['dense_proxy','sparse_proxy','sparse_dense_disagreement']:
    for metric in ['translation_m','rotation_deg']:
     vals=[r['losses'][method][metric] for r in eligible];record=summary['metrics'][method][metric]
     assert record['count']==len(vals)
     if vals:
      expected=math.sqrt(math.fsum(v*v for v in vals)/len(vals));assert math.isclose(record['rmse'],expected,rel_tol=1e-12,abs_tol=1e-12)
     else:assert record['rmse'] is None
     checks+=2
 for n,h in plan['scorer_only_reference_sha256'].items():assert sha(ROOT/n)==h;checks+=1
 result={'status':'PASS_INDEPENDENT_NATIVE_ENDPOINT_CALL_SCORE_AUDIT','checks':checks,'scored_rows':len(scores['rows']),
         'scores_sha256':sha(OUT/'scores.json'),'auditor_sha256':sha(__file__),
         'reference_check':'independent integer-time interpolation, quaternion SLERP and endpoint composition; recorded-SLAM provenance remains qualified'}
 with (OUT/'audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
