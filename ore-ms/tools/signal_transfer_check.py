"""EXP132 independent frozen-model, source geometry, reference and metric audit."""
import copy,json,math,hashlib,collections
import numpy as np,yaml
from scipy.linalg import expm,logm
from scipy.spatial.transform import Rotation,Slerp
from decimal import Decimal
from signal_transfer import ROOT,OUT,RECOVERY,PLAN,PRIOR,read,write,sha,verify,receipt

def main():
 p=verify();checks=0;mutations=0
 def check(v,label):
  nonlocal checks
  assert v,label
  checks+=1
 f=read(OUT/'features-and-motions.json');prediction=read(OUT/'predictions.json');pl=read(OUT/'prediction-lock.json');labels=read(OUT/'labels.json');ev=read(OUT/'evaluation.json');features={(r['case'],r['pair']):r for r in f['rows']};pred={(r['case'],r['pair']):r for r in prediction['rows']}
 check(len(features)==len(pred)==171 and set(features)==set(pred),'all joint IDs')
 check(pl['plan_sha256']==sha(PLAN) and pl['features_sha256']==sha(OUT/'features-and-motions.json') and pl['predictions_sha256']==sha(OUT/'predictions.json'),'prediction locks')
 check(labels['prediction_lock_sha256']==sha(OUT/'prediction-lock.json'),'labels after frozen predictions')
 check(prediction['refitted'] is False and prediction['model_source_sha256']==sha(PRIOR/'predictions.json'),'unchanged model provenance')
 expectedmodels={}
 for fold in read(PRIOR/'predictions.json')['folds'].values():
  for name,m in fold['models'].items():expectedmodels[name+'@'+fold['training_recordings'][0]]=m
 check(p['models']==expectedmodels,'frozen coefficient source equality')
 for n,h in p['expected_sources'].items():check(sha(ROOT/n)==h,'source camera/reference SHA')
 manifest=read(RECOVERY/'manifest.json');archive=RECOVERY/'geometry-support-transfer.tar.gz';check(sha(archive)==manifest['files'][archive.name]['sha256'],'recovery archive SHA')
 for phase in ['metadata','poses']:
  for n,h in read(OUT/('recovery-'+phase+'.json'))['members'].items():check(sha(ROOT/n)==h,'recovery member SHA')
 graph=collections.defaultdict(list)
 for edge in yaml.safe_load((ROOT/'data/calibration/yamaha.yaml').read_text())['transform_params']:
  E=np.eye(4);E[:3,:3]=Rotation.from_quat(edge['quaternion']).as_matrix();E[:3,3]=edge['translation'];a,b=edge['from_frame'],edge['to_frame'];graph[a].append((b,E));graph[b].append((a,np.linalg.inv(E)))
 logs={};references={};focals={};transforms={}
 for w in p['windows']:
  case=w['window'];logs[case]={'health':{},'motion':{}};timeline=[json.loads(l) for l in (ROOT/w['timeline']).read_text().splitlines()]
  for e in timeline:
   if e['kind']!='observer_event':continue
   v=e['event'];k=e['camera'],v.get('time1_ns')
   if v['kind']=='pair_end':logs[case]['health'][k]=e
   elif v['kind']=='motion_result_written' and v['first_write_for_submission']:logs[case]['motion'][k]=e
  focals[case]={};transforms[case]={}
  for c,name,goal in [('rgb','multisense_left_rect_color_camera_info','multisense/left_camera_optical_frame'),('thermal','thermal_left_processed_camera_info','thermal_left/optical_frame')]:
   info=yaml.safe_load((ROOT/w['metadata_dataset']/'sensors'/name/'data.yaml').read_text());P=np.asarray(info['p']).reshape(3,4);focals[case][c]=float((P[0,0]**2+P[1,1]**2)/2);R=np.array(info['r']);u,_,vt=np.linalg.svd(R);basis=np.eye(4);basis[:3,:3]=(u@vt).T@np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
   q=collections.deque([('vehicle',np.eye(4))]);seen=set();found=None
   while q:
    frame,T=q.popleft()
    if frame in seen:continue
    seen.add(frame)
    if frame==goal:found=T@basis;break
    for child,E in graph[frame]:q.append((child,T@E))
   check(found is not None and np.allclose(found,w['nominal_body_from_camera'][c],atol=1e-12,rtol=0),'independent nominal source graph');transforms[case][c]=found
  if w['recording_id'] not in references:
   d=ROOT/w['reference_source']/'super_odometry/odometry';info=yaml.safe_load((d/'info.yaml').read_text());check(info=={'frame_id':'sensor_init','child_frame_id':'vehicle'},'source frames');t=np.array([int((Decimal(s)*1000000000).to_integral_value()) for s in (d/'interp_timestamps.txt').read_text().splitlines()],dtype=np.int64);poses=np.loadtxt(d/'interp_data.txt')[:,:7];check(len(t)==len(poses) and np.all(np.diff(t)>0) and np.isfinite(poses).all() and np.allclose(np.linalg.norm(poses[:,3:],axis=1),1.,atol=1e-3),'reference validity');references[w['recording_id']]=(t,poses)
  # Rebuild each core eligibility directly, including zero-joint windows.
  decisions={e['pair_ordinal']:e for e in timeline if e['kind']=='decision'};joint=[]
  for i in range(2,w['frames']-3):
   good=[];deadline=decisions[i]['controller_monotonic_ns']
   for c in ['rgb','thermal']:
    interval=[w['rows'][j][c+'_left']['time_ns'] for j in [i,i+1]];h=logs[case]['health'][c,interval[1]];m=logs[case]['motion'].get((c,interval[1]));hv=h['event'];mv=m['event'] if m else None
    eligible=bool(hv['execution_error'] is None and hv['original_need_interp'] is False and m and h['controller_monotonic_ns']<=deadline and m['controller_monotonic_ns']<=deadline and [mv['time0_ns'],mv['time1_ns']]==interval and mv['stage']=='pair_processing' and mv['original_need_interp'] is False and mv['derived_camera_delta_xyzw'] is not None);good.append(eligible)
   if all(good):joint.append(i)
  check(joint==[r['pair'] for r in f['rows'] if r['case']==case],'independent joint population')
 for key,r in features.items():
  for c in ['rgb','thermal']:
   own=r['receipts'][c]['native_interval_ns'];h=logs[r['case']]['health'][c,own[1]];m=logs[r['case']]['motion'][c,own[1]];hv=h['event'];mv=m['event'];check([hv['time0_ns'],hv['time1_ns']]==own and h['controller_monotonic_ns']<=r['deadline_ns'] and m['controller_monotonic_ns']<=r['deadline_ns'],'exact causal receipts')
   check(r['raw'][c]=={k:hv[k] for k in r['raw'][c]} and r['focal_squared'][c]==focals[r['case']][c],'raw feature provenance')
   D=np.eye(4);v=mv['derived_camera_delta_xyzw'];D[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();D[:3,3]=v[:3];E=transforms[r['case']][c];body=E@D@np.linalg.inv(E);check(np.allclose(body,r['experts'][c]['body_delta'],atol=1e-12,rtol=0),'independent body delta');common=r['common_interval_ns'];model=expm(logm(body)*(common[1]-common[0])/(own[1]-own[0]));check(np.allclose(model,r['experts'][c]['delta_model'],atol=1e-9,rtol=0),'independent screw projection')
  if r['features'] is not None:
   a,b=r['raw']['rgb'],r['raw']['thermal'];u,v=r['focal_squared']['rgb'],r['focal_squared']['thermal'];x=[math.log(a['grid_trace_q10_q50_q90_px2'][i]/u)-math.log(b['grid_trace_q10_q50_q90_px2'][i]/v) for i in range(3)]+[a['retained_count']/a['inbound_count']-b['retained_count']/b['inbound_count'],(a['occupied_3x3_cells']-b['occupied_3x3_cells'])/9,math.log(a['inbound_count']/b['inbound_count'])];check(np.allclose(x,r['features'],atol=1e-12,rtol=0),'independent feature vector')
   expected={'fixed_rgb':'rgb','fixed_thermal':'thermal','training_best_fixed':'thermal','raw_uncertainty':'rgb' if a['grid_trace_q10_q50_q90_px2'][1]<b['grid_trace_q10_q50_q90_px2'][1] else 'thermal','angular_uncertainty':'rgb' if x[1]<0 else 'thermal','retention':'rgb' if x[3]>0 else 'thermal','occupancy':'rgb' if x[4]>0 else 'thermal'}
  else:expected={name:'thermal' for name in ['training_best_fixed','raw_uncertainty','angular_uncertainty','retention','occupancy']};expected.update(fixed_rgb='rgb',fixed_thermal='thermal')
  for name,m in expectedmodels.items():
   z=0. if r['features'] is None else sum((r['features'][i]-mean)/sd*beta for i,mean,sd,beta in zip(m['columns'],m['mean'],m['scale'],m['coefficients']))+m['intercept'];expected[name]='rgb' if z<0 else 'thermal'
  check(expected==pred[key]['choices'],'all unchanged choices')
 for r in labels['rows']:
  if r['exclusion'] is not None:continue
  t,poses=references[r['clip']];targets=[]
  for time in r['common_interval_ns']:
   j=int(np.searchsorted(t,time));check(0<j<len(t) and t[j]-t[j-1]<=150000000,'reference interpolation support');fraction=(time-int(t[j-1]))/int(t[j]-t[j-1]);P=np.eye(4);P[:3,3]=poses[j-1,:3]+fraction*(poses[j,:3]-poses[j-1,:3]);P[:3,:3]=Slerp([0.,1.],Rotation.from_quat(poses[j-1:j+1,3:]))([fraction]).as_matrix()[0];targets.append(P)
  target=np.linalg.inv(targets[0])@targets[1];check(np.allclose(target,r['reference_delta'],atol=1e-10,rtol=0),'independent relative reference')
  for c in ['rgb','thermal']:
   P=np.array(features[r['case'],r['pair']]['experts'][c]['delta_model']);tr=float(np.linalg.norm(P[:3,3]-target[:3,3]));angle=float(np.degrees(Rotation.from_matrix(target[:3,:3].T@P[:3,:3]).magnitude()));check(abs(tr-r['loss'][c]['translation_m'])<1e-10 and abs(angle-r['loss'][c]['rotation_deg'])<1e-10,'independent endpoint losses')
 valid={(r['case'],r['pair']):r['loss'] for r in labels['rows'] if r['exclusion'] is None}
 for name,g in [('aggregate',ev['aggregate'])]+list(ev['recordings'].items())+list(ev['windows'].items()):
  rr=[r for key,r in pred.items() if key in valid and (name=='aggregate' or r['case']==name or r['clip']==name)];check(len(rr)==g['rows'],'all summary denominators')
  for method,q in g['methods'].items():
   values=[valid[r['case'],r['pair']][r['choices'][method]] for r in rr];check(abs(sum(v['rotation_deg'] for v in values)/len(rr)-q['rotation_mean_deg'])<1e-12 and abs(sum(v['translation_m'] for v in values)/len(rr)-q['translation_mean_m'])<1e-12,'selected summary losses');check(q['camera_counts']=={c:sum(r['choices'][method]==c for r in rr) for c in ['rgb','thermal']},'choice counts')
 # Timing/interval refusal and model-integrity mutation tests.
 sample=next(iter(features.values()));own=sample['receipts']['rgb']['native_interval_ns'];original=logs[sample['case']]['health']['rgb',own[1]]
 for kind in ['late','interval']:
  altered=copy.deepcopy(original)
  if kind=='late':altered['controller_monotonic_ns']=sample['deadline_ns']+1
  else:altered['event']['time0_ns']-=1
  rejected=False
  try:receipt(altered,own,sample['deadline_ns'])
  except AssertionError:rejected=True
  check(rejected,'refuse '+kind);mutations+=1
 changed=copy.deepcopy(p['models']);first=next(iter(changed));changed[first]['intercept']+=1.;check(changed!=expectedmodels,'changed model detectable');mutations+=1
 # Reference labels are not arguments to the fixed decision calculation.
 before=copy.deepcopy(pred);poisoned=copy.deepcopy(labels)
 for r in poisoned['rows']:
  if r['exclusion'] is None:r['loss']['rgb']['rotation_deg']=1e6
 check(before==pred and sha(OUT/'predictions.json')==pl['predictions_sha256'],'label-independent frozen predictions');mutations+=1
 from Scripts.Setup.td3_reference import PoseReference
 rejected=False
 try:PoseReference([0,0],[[0,0,0,0,0,0,1],[0,0,0,0,0,0,1]])
 except ValueError:rejected=True
 check(rejected,'invalid reference rejection');mutations+=1
 write(OUT/'audit.json',{'pass':True,'checks':checks,'semantic_mutations':mutations,'joint_rows':171,'scored_rows':len(valid),'evaluation_sha256':sha(OUT/'evaluation.json'),'predictions_sha256':sha(OUT/'predictions.json'),'source_sha256':sha(__file__)})
 print('PASS',checks,'checks',mutations,'semantic mutations')
if __name__=='__main__':main()
