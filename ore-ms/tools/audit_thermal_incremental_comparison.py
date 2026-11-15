"""Standalone saved-output verification; no import of experiment implementation."""
import hashlib,json,math
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1]/'results/thermal-incremental-comparison';P=R/'run-001'
def load(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
lock=load(P/'prediction-lock.json');features=load(P/'features.json');scores=load(P/'scores.json');assert sha(P/'features.json')==lock['features_sha256'];assert sha(P/'prediction-lock.json')==scores['prediction_lock_sha256'];checks=0;oracles={}
for c in ['day','night','rain']:
 source=load(R/'recovery'/(c+'-source.json'));binding=load(R/'recovery'/('binding-'+c+'.json'))
 for row in source['records'][:100]:
  for cam in ['rgb','thr']:
   p=R/'recovery/images'/c/cam/(row['stem']+'.png');assert sha(p)==row['cameras'][cam]['left']['sha256'];checks+=1
 refs={r['pair']:r for r in load(R/'recovery'/('scores-'+c+'.json'))['rows']};by={r['pair']:r for r in features['rows'] if r['condition']==c};rr=[r for r in lock['rows'] if r['condition']==c];oracle=[];rgb=[];count=0
 for row in rr:
  f=by[row['pair']];a=np.array(f['rgb']);b=np.array(f['thermal']);ref=np.array(refs[row['pair']]['reference_delta']);assert row['rgb_rotation']==a[:3,:3].tolist();checks+=1
  d=b[:3,3]-a[:3,3];e=a[:3,3]-ref[:3,3];w=float(np.clip(-e@d/(d@d),0,1)) if d@d>0 else 0.;oracle.append(float(np.linalg.norm(e+w*d)));rgb.append(float(np.linalg.norm(e)));count+=w>0
  for mode,w in row['weights'].items():
   assert 0<=w<=1
   for j in range(3):assert abs(row['translations'][mode][j]-(a[j,3]+w*(b[j,3]-a[j,3])))<1e-12;checks+=1
 for mode,m in scores['summary'][c]['methods'].items():
  err=[math.sqrt(sum((r['translations'][mode][j]-refs[r['pair']]['reference_delta'][j][3])**2 for j in range(3))) for r in rr];assert math.isclose(sum(err)/len(err),m['mean_m'],abs_tol=1e-12);assert math.isclose(math.sqrt(sum(x*x for x in err)/len(err)),m['rmse_m'],abs_tol=1e-12);checks+=2
 oracles[c]={'paired':len(rr),'oracle_positive_weights':int(count),'oracle_mean_m':float(np.mean(oracle)),'rgb_mean_m':float(np.mean(rgb)),'oracle_rmse_m':float(np.sqrt(np.mean(np.square(oracle)))),'mean_headroom_percent':float(100*(1-np.mean(oracle)/np.mean(rgb)))}
report={'saved_output_checks':checks,'input_images_verified':600,'all_rgb_rotations_identical':True,'fit_reconstructions':lock['fit_checks'],'posthoc_oracle':oracles,'oracle_scope':'Post-scoring reference-informed convex-translation diagnostic only; not a deployed method or predeclared endpoint.'};(P/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
