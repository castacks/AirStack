"""Saved-case replay, scalar error and hash checks, independent from producer."""
from pathlib import Path
import json,hashlib,math
import numpy as np
import nominal_joint_motion as solver
R=Path(__file__).resolve().parents[1];O=R/'results/dynamic-thermal-evidence/run-001';REC=R/'results/thermal-incremental-comparison/recovery'
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):
 with np.load(p) as z:return {k:z[k] for k in z.files}
lock=read(O/'ablation-prediction-lock.json');scores=read(O/'ablation-scores.json');assert sha(O/'ablation-prediction-lock.json')==scores['prediction_lock_sha256'];proof=[]
for c in ['day','night','rain']:
 row=next(x for x in lock['rows'] if x['condition']==c and x['event_category']=='hurts' and x['cell']==0);i=row['pair'];p=load(REC/f'packets/{c}/{i:03}-rgb.npz');q=load(REC/f'packets/{c}/{i:03}-thermal.npz');uv=q['observations'];ids=np.array([j for j,x in enumerate(uv) if 0<=x[0]<160 and 0<=x[1]<64],dtype=int);assert ids.tolist()==row['removed_local_ids'];keep=[j for j in range(len(uv)) if j not in set(ids.tolist())];qq={k:v[keep].copy() for k,v in q.items()};qq['covariance']/=.1;fit=solver.solve([p,qq],'cov',solver.CONTRACT);actual=np.array(row['models']['cell']['transform']);err=float(np.max(np.abs(np.array(fit['transform'])-actual)));assert err<1e-10;proof.append({'condition':c,'pair':i,'cell':0,'matched_ids':len(ids),'replay_pose_maxerror':err})
count=0;noop=0
for row in scores['rows']:
 ref=np.array(read(REC/f'scores-{row["condition"]}.json')['rows'][row['pair']]['reference_delta']);original=next(x for x in lock['rows'] if x['condition']==row['condition'] and x['pair']==row['pair'] and x['cell']==row['cell'])
 for name,model in original['models'].items():
  if model['transform'] is None:continue
  T=np.array(model['transform']);error=math.sqrt(sum((float(T[j,3])-float(ref[j,3]))**2 for j in range(3)));assert abs(error-row['scores'][name]['translation_error_m'])<1e-12;count+=1
  if not row['removed_count']:assert abs(row['scores'][name]['increase_over_full_fixed_m'])<1e-10;noop+=1
# Feature construction source inputs remain exactly bound to original packets.
packets=0
for c in ['day','night','rain']:
 for row in read(REC/f'binding-{c}.json')['rows']:
  for cam,e in row['cameras'].items():
   if e['valid']:assert sha(REC/e['packet'])==e['sha256'];packets+=1
report={'independent_cell0_replays':proof,'scalar_pose_error_checks':count,'emptycell_noop_pose_checks':noop,'original_packet_hashes_verified':packets,'event_selection_uses_previously_observed_reference_errors':True,'events_and_largest_effect_regions_are_not_independent_validation':True};(O/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
