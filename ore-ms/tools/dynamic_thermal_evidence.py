"""Time-resolved association and fixed-cell thermal-factor intervention."""
from pathlib import Path
import json,hashlib,math
import numpy as np
from scipy.stats import spearmanr
from scipy.spatial.transform import Rotation
import nominal_joint_motion as geom
R=Path(__file__).resolve().parents[1];REC=R/'results/thermal-incremental-comparison/recovery';CACHE=R/'results/local-relationship-contract/cache-001';PREV=R/'results/local-relationship-learning/run-001';OUT=R/'results/dynamic-thermal-evidence/run-001'
GEOMETRY=['count_ratio','depth_ratio','depth_iqr_ratio','uv_variance_ratio','relative_disparity_variance_ratio','coverage_ratio','residual_energy_ratio'];IMAGE=['flow_ratio','patch_contrast_ratio','patch_gradient_ratio']
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def load(p):
 with np.load(p) as z:return {k:z[k] for k in z.files}
def stats(p,T,shape):
 uv,_=geom.project(p['points'],p['camera_R'],p['camera_t'],p['Kb'],T);r=np.linalg.solve(np.linalg.cholesky(p['covariance']),(uv-p['observations'])[...,None])[...,0];depth=(p['points']@p['camera_R'][0].T+p['camera_t'][0])[:,2];coords=p['observations'][:,:2];h,w=shape;cells=np.clip(np.floor(coords/[w,h]*4),0,3).astype(int);occupied=len(set(map(tuple,cells)))
 return np.array([len(depth),np.median(depth),np.subtract(*np.percentile(depth,[75,25])),np.median(p['covariance'][:,0,0]+p['covariance'][:,1,1]),np.median(p['covariance'][:,2,2]/p['observations'][:,2]**2),occupied/16,np.mean(np.sum(r*r,axis=1))])
def errors(T,ref):
 d=np.linalg.solve(ref,np.asarray(T));return float(np.linalg.norm(d[:3,3])),float(np.rad2deg(Rotation.from_matrix(d[:3,:3]).magnitude()))
def rho(x,y):
 if len(x)<3 or np.ptp(x)==0 or np.ptp(y)==0:return None
 return float(spearmanr(x,y).statistic)
def associations(rows,features):
 result={}
 for c in ['day','night','rain']:
  rr=[r for r in rows if r['condition']==c and r.get('features') is not None];d={}
  consecutive=[(a,b) for a,b in zip(rr,rr[1:]) if b['pair']==a['pair']+1]
  for name in features:
   x=np.array([r['features'][name] for r in rr]);y=np.array([r['translation_gain_m'] for r in rr]);dx=[b['features'][name]-a['features'][name] for a,b in consecutive];dy=[b['translation_gain_m']-a['translation_gain_m'] for a,b in consecutive]
   d[name]={'intervals':len(rr),'spearman_level':rho(x,y),'adjacent_change_pairs':len(consecutive),'spearman_adjacent_changes':rho(dx,dy)}
  result[c]=d
 return result

def primary():
 OUT.mkdir(parents=True,exist_ok=False);rows=[];events=[]
 parent=read(PREV/'global-prediction-lock.json');assert all(sha(PREV/k)==v for k,v in parent['files'].items());cache=read(CACHE/'lock.json');cachehash={(x['condition'],x['pair'],x['camera']):x['sha256'] for x in cache['rows']}
 for c in ['day','night','rain']:
  binding=read(REC/f'binding-{c}.json')['rows'];pred=read(REC/f'predictions-{c}.json')['rows'];ref={r['pair']:r['reference_delta'] for r in read(REC/f'scores-{c}.json')['rows']};fixed=read(PREV/f'fixed-point1-{c}.json')['rows'];source=read(REC/(c+'-source.json'))
  for i,b in enumerate(binding):
   T=np.asarray(pred[i]['methods']['rgb_cov']['source_to_target_body']);tr,rr=errors(pred[i]['methods']['rgb_cov']['transform'],ref[i]);tf,rf=errors(fixed[i]['transform'],ref[i]);gain=tr-tf;row={'condition':c,'sequence':source['sequence'],'pair':i,'time_ns':source['records'][i+1]['cameras']['rgb']['left']['recorded_time_ns'],'rgb_translation_error_m':tr,'fixed_translation_error_m':tf,'translation_gain_m':gain,'rotation_gain_deg':rr-rf,'category':'helps' if gain>=.001 else 'hurts' if gain<=-.001 else 'neutral','rgb_motion_m':float(np.linalg.norm(np.asarray(pred[i]['methods']['rgb_cov']['transform'])[:3,3])),'features':None}
   if not all(b['cameras'][cam]['valid'] for cam in ['rgb','thermal']):row['reason']='MISSING_NATIVE_CAMERA';rows.append(row);continue
   ps=[];imgs=[]
   for cam in ['rgb','thermal']:
    e=b['cameras'][cam];p=REC/e['packet'];assert sha(p)==e['sha256'];ps.append(load(p));f=CACHE/f'{c}-{i:03}-{cam}.npz';assert sha(f)==cachehash[c,i,cam]
    with np.load(f) as z:F=z['features'];imgs.append(np.array([np.median(np.linalg.norm(F[:,8:10],axis=1)),np.median((F[:,10]+F[:,11])/2),np.median((F[:,12]+F[:,13])/2)]))
   try:
    aa,bb=stats(ps[0],T,(384,1224)),stats(ps[1],T,(256,640));v=np.log((bb+1e-12)/(aa+1e-12));vi=np.log((imgs[1]+1e-12)/(imgs[0]+1e-12));row['features']=dict(zip(GEOMETRY+IMAGE,np.r_[v,vi].tolist()))
   except ValueError as e:row['reason']=str(e)
   rows.append(row)
  available=[r for r in rows if r['condition']==c and r['features'] is not None]
  for cat in ['helps','hurts','neutral']:
   group=[r for r in available if r['category']==cat]
   selected=min(group,key=lambda r:(-r['translation_gain_m'] if cat=='helps' else r['translation_gain_m'] if cat=='hurts' else abs(r['translation_gain_m']),r['pair'])) if group else None
   events.append({'condition':c,'category':cat,'pair':None if selected is None else selected['pair'],'reason':'NO_ELIGIBLE_CASE' if selected is None else 'Fixed extreme/nearest-neutral rule before spatial interventions'})
 write(OUT/'primary-evidence.json',{'rows':rows,'associations':associations(rows,GEOMETRY+IMAGE),'events':events,'threshold_m':.001,'scope':'Exploratory associations, no environmental cause or independent prediction claim.'});print('PRIMARY',len(rows),events,flush=True)
def subset(p,mask):return {k:v[mask] for k,v in p.items()}
def fit_removed(p,q,ids):
 keep=np.ones(len(q['points']),bool);keep[ids]=False;packets=[p]
 if keep.sum():
  remaining=subset(q,keep);remaining['covariance']=remaining['covariance']/.1;packets.append(remaining)
 try:return geom.solve(packets,'cov',geom.CONTRACT)
 except (ValueError,np.linalg.LinAlgError) as e:return {'status':'UNAVAILABLE_FIT','reason':str(e),'transform':None}
def ablate():
 evidence=read(OUT/'primary-evidence.json');rows=[]
 for event in evidence['events']:
  if event['pair'] is None:continue
  c,i=event['condition'],event['pair'];binding=read(REC/f'binding-{c}.json')['rows'][i];p=load(REC/binding['cameras']['rgb']['packet']);q=load(REC/binding['cameras']['thermal']['packet']);n=len(q['points']);cell=np.clip(np.floor(q['observations'][:,:2]/[640,256]*4),0,3).astype(int);cellid=cell[:,1]*4+cell[:,0]
  for label in range(16):
   ids=np.flatnonzero(cellid==label);models={};models['cell']=fit_removed(p,q,ids)
   # Remove the same number, independent fixed random controls.
   for seed in range(4):
    value=int.from_bytes(hashlib.sha256(f'{seed}:{c}:{i}:{label}'.encode()).digest()[:8],'little');random=np.random.default_rng(value).choice(n,len(ids),replace=False);models[f'random{seed}']=fit_removed(p,q,random)
   depth=(q['points'][ids]@q['camera_R'][0].T+q['camera_t'][0])[:,2] if len(ids) else np.array([]);rows.append({'condition':c,'event_category':event['category'],'pair':i,'cell':label,'removed_count':len(ids),'removed_local_ids':ids.tolist(),'removed_depth_median_m':float(np.median(depth)) if len(depth) else None,'models':models})
  print('ABLATION',c,event['category'],i,flush=True)
 write(OUT/'ablation-prediction-lock.json',{'rows':rows,'source_sha256':sha(__file__),'evidence_sha256':sha(OUT/'primary-evidence.json'),'reference_values_read_in_ablation_capture':False})
def score_ablation():
 lock=read(OUT/'ablation-prediction-lock.json');evidence=read(OUT/'primary-evidence.json');base={(r['condition'],r['pair']):r for r in evidence['rows']};summaries=[]
 for row in lock['rows']:
  c,i=row['condition'],row['pair'];ref=np.array(read(REC/f'scores-{c}.json')['rows'][i]['reference_delta']);b=base[c,i];scores={}
  for name,m in row['models'].items():
   if m['transform'] is None:scores[name]={'status':m['status']};continue
   t,r=errors(m['transform'],ref);scores[name]={'translation_error_m':t,'increase_over_full_fixed_m':t-b['fixed_translation_error_m'],'thermal_gain_vs_rgb_m':b['rgb_translation_error_m']-t,'rotation_error_deg':r}
  summaries.append({**{k:v for k,v in row.items() if k!='models'},'baseline':b,'scores':scores})
 write(OUT/'ablation-scores.json',{'prediction_lock_sha256':sha(OUT/'ablation-prediction-lock.json'),'rows':summaries});print('SCORED',len(summaries),'cells',flush=True)
if __name__=='__main__':
 import sys
 {'primary':primary,'ablate':ablate,'score':score_ablation}[sys.argv[1]]()
