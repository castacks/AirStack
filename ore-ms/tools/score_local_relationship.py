"""Post-lock metrics and independent saved-gate/NumPy pose checks."""
from pathlib import Path
import json,hashlib,math
import numpy as np
from scipy.spatial.transform import Rotation
import nominal_joint_motion as original
R=Path(__file__).resolve().parents[1];REC=R/'results/thermal-incremental-comparison/recovery';CACHE=R/'results/local-relationship-contract/cache-001';OUT=R/'results/local-relationship-learning/run-001';MODES=['constant','affine','local','context','all','rgb_only']
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def packet(p):
 with np.load(p) as z:return {k:z[k] for k in z.files}
def controls():
 for c in ['day','night','rain']:
  rows=[]
  for row in read(REC/f'binding-{c}.json')['rows']:
   ps=[]
   for cam in ['rgb','thermal']:
    e=row['cameras'][cam]
    if not e['valid']:continue
    p=packet(REC/e['packet'])
    if cam=='thermal':p['covariance']=p['covariance']/.1
    ps.append(p)
   result=original.solve(ps,'cov',original.CONTRACT);rows.append({'pair':row['pair'],'transform':result['transform']})
  write(OUT/f'fixed-point1-{c}.json',{'rows':rows,'reference_values_read':False})
 print('Fixed0.1 controls captured',flush=True)
def lock():
 files={}
 for c in ['day','night','rain']:
  for mode in MODES:
   for name in ['model.json','predictions.json']:
    p=OUT/c/mode/name;d=read(p)
    if name=='model.json':assert len(d['history'])==40 and d['held']==c and c not in d['training']
    else:assert len(d['rows'])==99 and all(np.isfinite(r['transform']).all() for r in d['rows'])
    files[str(p.relative_to(OUT))]=sha(p)
  p=OUT/f'fixed-point1-{c}.json';assert len(read(p)['rows'])==99;files[p.name]=sha(p)
 write(OUT/'global-prediction-lock.json',{'files':files,'fold_models':18,'learned_predictions':1782,'training_updates':720,'training_freeze_sha256':sha(OUT/'training-freeze.json'),'scoring_started':False});print('GLOBAL LOCK',len(files),flush=True)
def numpy_gate(F,m):
 x=(F-m['mean'])/m['scale'];s=m['state'];mode=m['mode']
 if mode=='constant':logit=np.full(len(F),s['bias'])
 elif mode=='affine':logit=(x[:,[4,6,7,2,3]]@np.array(s['linear.weight']).T+np.array(s['linear.bias']))[:,0]
 else:
  if mode=='local':x[:,16:]=0
  if mode=='context':x[:,:16]=0
  logit=(np.tanh(x@np.array(s['first.weight']).T+np.array(s['first.bias']))@np.array(s['last.weight']).T+np.array(s['last.bias']))[:,0]
 return 1/(1+np.exp(-logit))
def score():
 locked=read(OUT/'global-prediction-lock.json');assert all(sha(OUT/name)==h for name,h in locked['files'].items());summary={};audit=[];details=[]
 for c in ['day','night','rain']:
  refs={r['pair']:r for r in read(REC/f'scores-{c}.json')['rows']};binding=read(REC/f'binding-{c}.json')['rows'];baseline=read(REC/f'predictions-{c}.json')['rows'];methods={};models={}
  for mode in MODES:
   m=read(OUT/c/mode/'model.json');models[mode]=m;pr=read(OUT/c/mode/'predictions.json');methods[mode]=pr['rows'];camera='rgb' if mode=='rgb_only' else 'thermal';heldchecks=0
   for row in pr['rows']:
    i=row['pair'];e=binding[i]['cameras'][camera]
    if not e['valid']:assert row['fallback'];continue
    with np.load(CACHE/f'{c}-{i:03}-{camera}.npz') as z:F=z['features']
    w=numpy_gate(F,m);assert abs(float(w.mean())-row['gate_mean'])<1e-12 and abs(float(w.min())-row['gate_min'])<1e-12 and abs(float(w.max())-row['gate_max'])<1e-12;heldchecks+=1
    if i==0:
     p=packet(REC/binding[i]['cameras']['rgb']['packet']);ps=[p]
     if mode=='rgb_only':p['covariance']/=w[:,None,None]
     else:
      q=packet(REC/e['packet']);q['covariance']/=w[:,None,None];ps.append(q)
     T=np.array(original.solve(ps,'cov',original.CONTRACT)['transform']);err=float(np.max(np.abs(T-np.array(row['transform']))));assert err<1e-8;poseerr=err
   audit.append({'condition':c,'mode':mode,'numpy_gate_rows':heldchecks,'numpy_first_pose_max_error':poseerr})
  methods['fixed_point1']=read(OUT/f'fixed-point1-{c}.json')['rows']
  for key in ['rgb_cov','thermal_cov','joint_cov','native_rgb']:
   methods[key]=[{'pair':row['pair'],'transform':row['methods'][key].get('transform')} for row in baseline]
  result={}
  for name,rows in methods.items():
   ts=[];rs=[];missing=[]
   for row in rows:
    i=row['pair'];t=row['transform']
    if t is None:missing.append(i);continue
    T=np.array(t);ref=np.array(refs[i]['reference_delta']);D=np.linalg.inv(ref)@T;tv=float(np.linalg.norm(D[:3,3]));rv=float(np.rad2deg(Rotation.from_matrix(D[:3,:3]).magnitude()));assert abs(tv-sum((float(T[j,3])-float(ref[j,3]))**2 for j in range(3))**.5)<1e-10;ts.append(tv);rs.append(rv);details.append({'condition':c,'method':name,'pair':i,'translation_m':tv,'rotation_deg':rv})
   result[name]={'available':len(ts),'missing':missing,'translation_mean_m':float(np.mean(ts)),'translation_rmse_m':float(np.sqrt(np.mean(np.square(ts)))),'translation_p95_m':float(np.percentile(ts,95)),'rotation_mean_deg':float(np.mean(rs)),'rotation_rmse_deg':float(np.sqrt(np.mean(np.square(rs)))),'rotation_p95_deg':float(np.percentile(rs,95))}
  summary[c]=result
 write(OUT/'scores.json',{'global_lock_sha256':sha(OUT/'global-prediction-lock.json'),'summary':summary,'rows':details,'scope':'Previously exposed nominal same-ID development, not independent physicalGT/fullVO trajectory or author-system reproduction.'});write(OUT/'audit.json',{'independent_numpy_gates_and_first_pose':audit,'all_prediction_hashes_unchanged':True});print('SCORED',flush=True)
if __name__=='__main__':
 import sys
 {'controls':controls,'lock':lock,'score':score}[sys.argv[1]]()
