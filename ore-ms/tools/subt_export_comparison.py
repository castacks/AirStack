"""Configurable export comparison composing source-pinned matcher primitives.

Adds calibration adapter, matcher-only inputs and separate prediction/selection/
score locks; existing frozen smoke and lighting runners are never modified.
"""
import argparse,json,hashlib,time,math
from pathlib import Path
import numpy as np,cv2,yaml
from scipy.spatial.transform import Rotation
import deployment_lighting as base
from smoke_rgb_first_fallback import choose
ROOT=Path(__file__).resolve().parents[1]
read,write,sha=base.read,base.write,base.sha

class Comparison:
 def __init__(self,config):
  self.config=Path(config).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output'];self.parent=ROOT/self.cfg['qualification'];self.cameras=yaml.safe_load((self.parent/'intrinsics.yaml').read_text())
 def camera(self,arm):
  c=self.cameras[arm+'_camera'];p=c['projection_parameters'];k=np.array([[p.get('fx',p.get('gamma1')),0,p.get('cx',p.get('u0'))],[0,p.get('fy',p.get('gamma2')),p.get('cy',p.get('v0'))],[0,0,1.]]);d=np.array([c['distortion_parameters'][x] for x in ['k1','k2','p1','p2']]);v=np.array([[400.,0,c['image_width']/2],[0,400.,c['image_height']/2],[0,0,1.]]);return k,d,c.get('mirror_parameters',{}).get('xi',0),v
 def lift(self,points,arm):
  if not len(points):return np.empty((0,2))
  k,d,xi,v=self.camera(arm);xy=cv2.undistortPoints(np.asarray(points,float)[:,None,:],k,d,criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,100,1e-12)).reshape(-1,2)
  if xi:
   r2=np.sum(xy*xy,axis=1);disc=1+(1-xi*xi)*r2;assert (disc>0).all();lam=(xi+np.sqrt(disc))/(1+r2);rays=np.column_stack([lam[:,None]*xy,lam-xi]);assert (rays[:,2]>0).all();xy=rays[:,:2]/rays[:,2:]
  return xy*400+v[:2,2]
 def verify(self,file='execution-freeze.json'):
  plan=read(self.out/file)
  for p,h in plan['pins'].items():assert sha(ROOT/p)==h,p
  return plan
 def weights(self,arm):return ROOT/('models/macslam/MACVO_FrontendCov.pth' if arm=='rgb' else 'models/macslam/MACT_Offroad.pth')
 def freeze(self):
  self.out.mkdir(exist_ok=True);assert read(self.parent/'audit.json')['status']=='pass';report=read(self.parent/'qualification.json');assert report['observed_axes_time_consistency_pass'] and report['export_imu_equal_all_shared'];pins={}
  for name in ['input-freeze.json','completion.json']:
   for p,h in read(self.parent/name)['pins'].items():assert sha(ROOT/p)==h,p;pins[p]=h
  frames=read(self.parent/'images.json')
  for f in frames:assert sha(ROOT/f['path'])==f['sha256'];pins[f['path']]=f['sha256']
  rows=read(self.parent/'intervals.json');inputs=[{k:r[k] for k in ['arm','index','anchor_ns','source','target','times_ns','qualified']} for r in rows];assert len(inputs)==2*report['anchors'] and len({(r['arm'],r['index']) for r in inputs})==len(inputs)
  write(self.out/'matcher-inputs.json',inputs)
  files=[self.config,Path(__file__),ROOT/self.cfg['protocol'],ROOT/'tools/deployment_lighting.py',ROOT/'tools/smokeseer_sift.py',ROOT/'tools/smokeseer_preflight.py',ROOT/'tools/qualify_native_frontend.py',ROOT/'tools/smoke_rgb_first_fallback.py',ROOT/'tools/deployment_smoke_frontend.py',ROOT/'MAC-VO-main/Scripts/Setup/compare_factor_proxy.py',self.parent/'images.json',self.parent/'intervals.json',self.parent/'population.json',self.parent/'calibration.json',self.parent/'audit.json',self.out/'matcher-inputs.json',self.weights('rgb'),self.weights('thermal')]
  for folder in ['FlowFormer','FlowFormerCov']:files.extend((ROOT/'MAC-VO-main/Module/Network'/folder).rglob('*.py'))
  pins.update({str(p.relative_to(ROOT)):sha(p) for p in files});write(self.out/'input-freeze.json',{'pins':pins,'anchors':report['anchors'],'expected_prediction_rows':2*len(inputs),'gray_images':len(frames),'runtime_image':'sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'});print('Full span frozen',len(inputs),'sensor intervals;',len(frames),'images',flush=True)
 def qualify(self):
  import torch
  from qualify_native_frontend import load_model,deterministic
  import deployment_smoke_frontend as old
  deterministic();torch.set_num_threads(1);cv2.setNumThreads(1);plan=self.verify('input-freeze.json');checks=0
  for rs in ['ok','bad','input_unsupported']:
   for supported in [False,True]:
    for ts in ['ok','bad','input_unsupported']:
     selected,request=choose(rs,supported,ts);assert selected==('rgb' if rs=='ok' else 'thermal' if supported and ts=='ok' else None) and request==(rs!='ok' and supported);checks+=1
  for arm in ['rgb','thermal']:
   k,d,xi,v=self.camera(arm);c=self.cameras[arm+'_camera'];points=base.grid(c['image_height'],c['image_width']);lifted=self.lift(points,arm);rays=np.column_stack([(lifted-v[:2,2])/400,np.ones(len(points))]);uv=rays[:,:2]/(rays[:,2]+xi*np.linalg.norm(rays,axis=1))[:,None];back=cv2.projectPoints(np.column_stack([uv,np.ones(len(uv))]),np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2);assert np.linalg.norm(back-points,axis=1).max()<.001;checks+=1
   # Reuse old calibration only for a synthetic equivalence test, never the transfer.
   current=self.cameras;self.cameras=yaml.safe_load((ROOT/'results/deployment-data-search/smoke_intrinsics.yaml').read_text());assert np.allclose(self.lift(points,arm),old.lift(points,arm),atol=1e-10);self.cameras=current;checks+=1
  rows=read(self.out/'matcher-inputs.json');reports=[]
  for arm in ['rgb','thermal']:
   model,loading=load_model(self.weights(arm));state={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};r=next(r for r in rows if r['arm']==arm and r['qualified']);a,b=[np.load(ROOT/r[k],allow_pickle=False) for k in ['source','target']];f0=base.infer(model,a,b);f1=base.infer(model,a,b);assert np.array_equal(f0,f1);assert state=={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};reports.append({'arm':arm,'shape':list(f0.shape),'bitwise_repeat':True,'loading':loading,'flow_sha256':hashlib.sha256(f0.tobytes()).hexdigest()});del model,f0,f1;torch.cuda.empty_cache()
  write(self.out/'qualification.json',{'mechanics_checks':checks,'reports':reports,'numpy':np.__version__,'opencv':cv2.__version__,'torch':torch.__version__,'gpu':torch.cuda.get_device_name(0),'model_states_unchanged':True,'full_run_predictions':0});pins=dict(plan['pins']);pins[str((self.out/'qualification.json').relative_to(ROOT))]=sha(self.out/'qualification.json');write(self.out/'execution-freeze.json',{'pins':pins,'runtime_image':plan['runtime_image']});print('Runtime qualified',checks,'checks',flush=True)
 def run(self):
  import torch
  from qualify_native_frontend import load_model,deterministic
  deterministic();torch.set_num_threads(1);cv2.setNumThreads(1);plan=self.verify();rows=read(self.out/'matcher-inputs.json');start=time.monotonic();n=0
  with (self.out/'predictions.jsonl').open('x') as output:
   for frontend in ['sift','learned']:
    for arm in ['rgb','thermal']:
     model=None;cache={}
     if frontend=='learned':model,_=load_model(self.weights(arm));state={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
     for row in [r for r in rows if r['arm']==arm]:
      if model is not None:torch.cuda.synchronize()
      t=time.perf_counter();r={'frontend':frontend,'arm':arm,'index':row['index'],'qualified':row['qualified']}
      if not row['qualified']:r.update(status='input_unsupported',matches=0)
      else:
       a,b=[np.load(ROOT/row[k],allow_pickle=False) for k in ['source','target']]
       if frontend=='learned':p,q,info=base.correspondences(base.infer(model,a,b));r.update(info)
       else:
        features=[]
        for key,image in zip(['source','target'],[a,b]):
         if row[key] not in cache:
          kp,desc=base.geometry.sift().detectAndCompute(image,None);cache[row[key]]=(np.array([k.pt for k in kp]).reshape(-1,2),desc)
         features.append(cache[row[key]])
        (p0,d0),(p1,d1)=features;matches=base.geometry.mutual_matches(d0,d1);p=np.array([p0[i] for i,j in matches]).reshape(-1,2);q=np.array([p1[j] for i,j in matches]).reshape(-1,2);r.update(source_keypoints=len(p0),target_keypoints=len(p1))
       p,q=self.lift(p,arm),self.lift(q,arm);r.update(base.geometry.pose(p,q,self.camera(arm)[3]));r.update(points0=p.tolist(),points1=q.tolist())
      if model is not None:torch.cuda.synchronize()
      r['seconds']=time.perf_counter()-t;output.write(json.dumps(r,allow_nan=False)+'\n');output.flush();n+=1
      if n%100==0:print('Predictions',n,'elapsed',round(time.monotonic()-start,1),flush=True)
     if model is not None:
      assert state=={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};del model;torch.cuda.empty_cache()
  self.verify();assert n==read(self.out/'input-freeze.json')['expected_prediction_rows'];write(self.out/'prediction-lock.json',{'rows':n,'seconds':time.monotonic()-start,'predictions_sha256':sha(self.out/'predictions.jsonl'),'scoring_started':False,'model_states_unchanged':True});print('Predictions locked',n,flush=True)
 def select(self):
  self.verify();assert sha(self.out/'predictions.jsonl')==read(self.out/'prediction-lock.json')['predictions_sha256'];raw={(r['frontend'],r['arm'],r['index']):r for r in (json.loads(s) for s in (self.out/'predictions.jsonl').read_text().splitlines())};inputs={(r['arm'],r['index']):r for r in read(self.out/'matcher-inputs.json')};cal=read(self.parent/'calibration.json');decisions=[];n=read(self.out/'input-freeze.json')['anchors']
  for frontend in ['sift','learned']:
   for i in range(n):
    rgb,thermal=[raw[frontend,a,i] for a in ['rgb','thermal']];selected,request=choose(rgb['status'],inputs['thermal',i]['qualified'],thermal['status']);r={'frontend':frontend,'index':i,'thermal_requested':request,'policies':{'rgb':'rgb' if rgb['status']=='ok' else None,'thermal':'thermal' if thermal['status']=='ok' else None,'rgb_first':selected},'selected_proposal':None}
    if selected:
     p=raw[frontend,selected,i];meta=inputs[selected,i];rot=np.array(cal[selected]['effective_camera_to_imu'])[:3,:3];r['selected_proposal']={'sensor':selected,'source':meta['source'],'target':meta['target'],'times_ns':meta['times_ns'],'R_imu':(rot@np.array(p['R'])@rot.T).tolist(),'translation_direction_camera_target':p['t'],'metric_body_translation':None}
    decisions.append(r)
  write(self.out/'decisions.json',decisions);write(self.out/'decision-lock.json',{'rows':len(decisions),'decisions_sha256':sha(self.out/'decisions.json'),'reference_errors_used_for_selection':False,'scoring_started':False});print('Choices locked',len(decisions),flush=True)
 def score(self):
  self.verify();assert sha(self.out/'decisions.json')==read(self.out/'decision-lock.json')['decisions_sha256'];raw=[json.loads(s) for s in (self.out/'predictions.jsonl').read_text().splitlines()];assert sha(self.out/'predictions.jsonl')==read(self.out/'prediction-lock.json')['predictions_sha256'];refs={(r['arm'],r['index']):r for r in read(self.parent/'intervals.json')};pop=read(self.parent/'population.json');common=set(pop['common_reference']);trans=set(pop['translation_ge1cm_both']);scores=[]
  for p in raw:
   i,a=p['index'],p['arm'];ref=refs[a,i];s={k:p[k] for k in ['frontend','arm','index','status']};s.update(R_error=None,t_error=None,R1=False,R5=False,joint=False,reference_rotation_deg=ref.get('reference_rotation_deg'))
   if i in common and p['status']=='ok':
    truth=np.array(ref['target_from_source']);e=float(np.degrees(Rotation.from_matrix(np.array(p['R'])@truth[:3,:3].T).magnitude()));s.update(R_error=e,R1=e<=1,R5=e<=5)
    if i in trans:
     a=np.array(p['t']);b=truth[:3,3];et=float(np.degrees(np.arccos(np.clip(a@b/(np.linalg.norm(a)*np.linalg.norm(b)),-1,1))));s.update(t_error=et,joint=bool(e<=5 and et<=15))
   scores.append(s)
  write(self.out/'scores.json',scores);smap={(s['frontend'],s['arm'],s['index']):s for s in scores};rmap={(r['frontend'],r['arm'],r['index']):r for r in raw};decisions=read(self.out/'decisions.json');n=read(self.out/'input-freeze.json')['anchors'];cohorts={'all_declared':set(range(n)),'common_reference':common,'paired_rotation_ge2deg':set(pop['paired_rotation_ge2deg']),'translation_ge1cm_both':trans,**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}};summary={}
  for f in ['sift','learned']:
   summary[f]={}
   for name,ids in cohorts.items():
    group=[d for d in decisions if d['frontend']==f and d['index'] in ids];result={}
    for policy in ['rgb','thermal','rgb_first']:
     selected=[smap[f,d['policies'][policy],d['index']] for d in group if d['policies'][policy]];calls=0;seconds=0
     for d in group:
      arms=['rgb']+(['thermal'] if d['thermal_requested'] else []) if policy=='rgb_first' else [policy]
      for a in arms:
       r=rmap[f,a,d['index']];calls+=int(r['qualified']);seconds+=r['seconds']
     rr=[s['R_error'] for s in selected if s['R_error'] is not None];tt=[s['t_error'] for s in selected if s['t_error'] is not None];result[policy]={'anchors':len(group),'outputs':len(selected),'R1':sum(s['R1'] for s in selected),'R5':sum(s['R5'] for s in selected),'joint':sum(s['joint'] for s in selected),'rotation_eligible':len(ids&common),'translation_eligible':len(ids&trans),'R_median':float(np.median(rr)) if rr else None,'t_median':float(np.median(tt)) if tt else None,'matcher_calls':calls,'projected_seconds':seconds}
    added=[d for d in group if d['policies']['rgb_first']=='thermal'];result['added']={'attempts':sum(d['thermal_requested'] for d in group),'outputs':len(added),'failed_attempts':sum(d['thermal_requested'] and not d['policies']['rgb_first'] for d in group)}
    for endpoint,eligible in [('R1',common),('R5',common),('joint',trans)]:
     valid=[d for d in added if d['index'] in eligible];good=sum(smap[f,'thermal',d['index']][endpoint] for d in valid);result['added'][endpoint]={'eligible':len(valid),'correct':good,'incorrect':len(valid)-good,'hindsight_union':sum(smap[f,'rgb',i][endpoint] or smap[f,'thermal',i][endpoint] for i in ids&eligible)}
    result['rgb_or_zero_rotation']={str(tol):sum(smap[f,'rgb',d['index']][field] if d['policies']['rgb'] else refs['rgb',d['index']]['reference_rotation_deg']<=tol for d in group if d['index'] in common) for tol,field in [(1,'R1'),(5,'R5')]};summary[f][name]=result
  write(self.out/'summary.json',summary);write(self.out/'completion.json',{'prediction_rows':len(scores),'decision_rows':len(decisions),'scores_sha256':sha(self.out/'scores.json'),'summary_sha256':sha(self.out/'summary.json'),'scope':'Supplied-reference pair proposals; projected costs only, no stateful VO or metric trajectory'});print('Scoring complete',len(scores),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','qualify','run','select','score']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Comparison(a.config),a.stage)()
