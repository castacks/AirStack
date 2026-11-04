"""EXP132 saved MSA-cohort sufficiency and unchanged completed-output signal transfer."""
import argparse,copy,json,math,subprocess,sys,tarfile
from pathlib import Path
from provided_quality_headroom import ROOT,read,write,sha
from rotation_signal_audit import receipt,feature_vector,fixed_choices,FEATURES
OUT=ROOT/'results/signal-transfer/run-001';RECOVERY=ROOT/'results/signal-transfer/recovery-001';PLAN=ROOT/'experiments/signal-transfer-2026-11-04.json'
OLD=ROOT/'results/exposed-data-adequacy/run-001/inventory.json';PRIOR=ROOT/'results/rotation-signal-audit/run-001'

def verify():
 p=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in p['sha256'].items():assert sha(ROOT/n)==h,n
 return p

def freeze():
 assert not OUT.exists() and not PLAN.exists()
 sources=read(ROOT/'experiments/guarded-probe-cache-2026-11-03-fixture-v2.json')['sources'];inventory=read(OLD);windows=[];pins={};expected={};recovered={}
 files=[Path(__file__),ROOT/'tools/signal_transfer_check.py',ROOT/'docs/research/signal-transfer-protocol-2026-11-04.md',ROOT/'tools/rotation_signal_audit.py',ROOT/'tools/provided_quality_headroom.py',ROOT/'data/calibration/yamaha.yaml',OLD,ROOT/'experiments/guarded-probe-cache-2026-11-03-fixture-v2.json',ROOT/'experiments/exposed-data-adequacy-2026-11-03.json',RECOVERY/'manifest.json',PRIOR/'predictions.json',PRIOR/'prediction-lock.json',PRIOR/'audit.json']
 adequate=read(ROOT/'experiments/exposed-data-adequacy-2026-11-03.json');lookup={w['window']:w for w in adequate['windows']};manifest=read(RECOVERY/'manifest.json')
 for s in sources:
  if s['tag'] not in ['exp112','exp113']:continue
  priorplan=ROOT/s['base']/s['plan'];files.append(priorplan);plan=read(priorplan)
  for case in plan['cases']:
   window=s['tag']+'/'+case['window'];w=copy.deepcopy(case);old=lookup[window];w.update(window=window,rows=old['rows'],timeline=old['timeline'],recipe='original RGB + EXP023 MSA thermal',tag=s['tag']);files.append(ROOT/w['timeline']);w['metadata_dataset']=w['dataset'];w['reference_source']=w['reference_dataset']
   if s['tag']=='exp113':
    w['metadata_dataset']=str((RECOVERY/'selected/ore-ms'/w['dataset']).relative_to(ROOT));w['reference_source']=str((RECOVERY/'selected/ore-ms'/w['reference_dataset']).relative_to(ROOT))
   for c,name in [('rgb','multisense_left_rect_color_camera_info'),('thermal','thermal_left_processed_camera_info')]:
    original=w['dataset']+'/sensors/'+name+'/data.yaml';digest=plan['input_source_sha256'][original];target=ROOT/w['metadata_dataset']/'sensors'/name/'data.yaml'
    if s['tag']=='exp113':
     member='ore-ms/'+original;assert manifest['member_sha256'][member]==digest;recovered[member]=digest
    else:assert sha(target)==digest;files.append(target)
    expected[str(target.relative_to(ROOT))]=digest
   for name in ['info.yaml','interp_timestamps.txt','interp_data.txt']:
    original=w['reference_dataset']+'/super_odometry/odometry/'+name;digest=plan['scorer_only_reference_sha256'][original];target=ROOT/w['reference_source']/'super_odometry/odometry'/name;expected[str(target.relative_to(ROOT))]=digest
    if s['tag']=='exp113':
     member='ore-ms/'+original;assert manifest['member_sha256'][member]==digest;recovered[member]=digest
    else:assert sha(target)==digest;files.append(target)
   windows.append(w)
 for f in files:pins[str(f.relative_to(ROOT))]=sha(f)
 for name in ['td3_reference.py','arbiter_pose_proxy.py','causal_interval_model.py','arbiter_motion_output.py','audit_td3_geometry.py']:
  f=ROOT/'MAC-VO-main/Scripts/Setup'/name;pins[str(f.relative_to(ROOT))]=sha(f)
 prior=read(PRIOR/'predictions.json');models={}
 for fold in prior['folds'].values():
  training=fold['training_recordings'][0]
  for name,model in fold['models'].items():models[name+'@'+training]=copy.deepcopy(model)
 methods=['fixed_rgb','fixed_thermal','training_best_fixed','raw_uncertainty','angular_uncertainty','retention','occupancy']+list(models)
 assert len(windows)==9 and len(models)==6 and sum(inventory['windows'][w['window']]['masks']['both'] for w in windows)==171
 p={'date':'2026-11-04','parent_commit':'8f6e76f3e','sha256':pins,'expected_sources':expected,'recovery_members':recovered,'windows':windows,'methods':methods,'models':models,'expected_joint_rows':171,'availability_rows':295,'reference_max_gap_ns':150000000,'runtime_image':'sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09','development_exposed':True,'trained':False,'cohort':'EXP023 MSA thermal; never pooled with provided exports'}
 write(PLAN,p);write(OUT/'plan.json',p);print('FROZEN',len(pins),'pins',len(recovered),'recovery members;171joint/295core')

def recover(stage):
 p=verify();manifest=read(RECOVERY/'manifest.json');archive=RECOVERY/'geometry-support-transfer.tar.gz';info=manifest['files'][archive.name]
 if stage=='metadata':
  assert not archive.exists(),'existing attempt retained'
  with archive.open('xb') as f:
   cmd=['rclone','--config','/root/.config/rclone/ore-ms-recovery.conf','--box-root-folder-id','428559630945','cat','ore_ms:backups/2026-11-02-geometry-support-transfer/'+archive.name,'--contimeout','15s','--timeout','90s','--retries','2'];subprocess.run(cmd,stdout=f,check=True)
 else:assert read(OUT/'prediction-lock.json')['predictions_sha256']==sha(OUT/'predictions.json')
 assert archive.stat().st_size==info['bytes'] and sha(archive)==info['sha256'],'recovery archive hash'
 wanted={n:h for n,h in p['recovery_members'].items() if ('/sensors/' in n)==(stage=='metadata')};restored={}
 with tarfile.open(archive) as t:
  allnames=t.getnames();assert len(allnames)==len(set(allnames))
  for name,digest in wanted.items():
   m=t.getmember(name);assert m.isfile() and '..' not in Path(name).parts and not Path(name).is_absolute();data=t.extractfile(m).read();assert __import__('hashlib').sha256(data).hexdigest()==digest
   target=RECOVERY/'selected'/name;target.parent.mkdir(parents=True,exist_ok=True)
   with target.open('xb') as f:f.write(data)
   restored[str(target.relative_to(ROOT))]=digest
 write(OUT/('recovery-'+stage+'.json'),{'archive_sha256':sha(archive),'members':restored});print('RECOVERED',stage,len(restored),'exact members')

def scientific():
 sys.path.insert(0,str(ROOT/'MAC-VO-main'))
 import numpy as np,yaml
 from Scripts.Setup.arbiter_pose_proxy import extrinsic,losses
 from Scripts.Setup.td3_reference import matrices,load_reference
 from Scripts.Setup.causal_interval_model import project
 return np,yaml,extrinsic,losses,matrices,load_reference,project

def sufficiency():
 np,yaml,extrinsic,_,matrices,_,project=scientific();p=verify();inventory=read(OLD);rows=[];groups={};checks=0
 def check(v,label):
  nonlocal checks
  assert v,label
  checks+=1
 for w in p['windows']:
  case=w['window'];timeline=[json.loads(l) for l in (ROOT/w['timeline']).read_text().splitlines()];D={e['pair_ordinal']:e for e in timeline if e['kind']=='decision'};H={};M={}
  for e in timeline:
   if e['kind']!='observer_event':continue
   v=e['event'];key=e['camera'],v.get('time1_ns')
   if v['kind']=='pair_end':check(key not in H,'unique health');H[key]=e
   elif v['kind']=='motion_result_written' and v['first_write_for_submission']:check(key not in M,'unique first motion');M[key]=e
  f2={};transforms={}
  for c,name in [('rgb','multisense_left_rect_color_camera_info'),('thermal','thermal_left_processed_camera_info')]:
   path=ROOT/w['metadata_dataset']/'sensors'/name/'data.yaml';check(sha(path)==p['expected_sources'][str(path.relative_to(ROOT))],'exact source camera info')
   P=np.asarray(yaml.safe_load(path.read_text())['p']).reshape(3,4);f2[c]=float((P[0,0]**2+P[1,1]**2)/2)
   E=extrinsic(ROOT/'data/calibration/yamaha.yaml',ROOT/w['metadata_dataset'],c);check(np.max(np.abs(E-np.array(w['nominal_body_from_camera'][c])))<=1e-12,'existing nominal transform');transforms[c]=E
  rr=[]
  for i in range(2,w['frames']-3):
   d=D[i];B=d['controller_monotonic_ns'];check(d['boundary']=={'kind':'group_complete','group':i+2},'same output deadline');completion=[e for e in timeline if e['kind']=='completed' and e['frame']==i+2];check(len(completion)==2 and max(e['controller_monotonic_ns'] for e in completion)<=B,'both complete')
   native=next(r for r in inventory['rows'] if r['window']==case and r['pair']==i);raw={};receipts={};motions={};eligible={}
   for c in ['rgb','thermal']:
    interval=[w['rows'][j][c+'_left']['time_ns'] for j in [i,i+1]];h=H[c,interval[1]];m=M.get((c,interval[1]));hv=h['event'];mv=m['event'] if m else None
    good=bool(hv['execution_error'] is None and hv['original_need_interp'] is False and m and h['controller_monotonic_ns']<=B and m['controller_monotonic_ns']<=B and [hv['time0_ns'],hv['time1_ns']]==interval and [mv['time0_ns'],mv['time1_ns']]==interval and mv['stage']=='pair_processing' and mv['original_need_interp'] is False and mv['derived_camera_delta_xyzw'] is not None)
    check(good==native['experts'][c]['eligible']==d['availability'][c],'independent native support');eligible[c]=good
    if good:
     receipt(h,interval,B);raw[c]={k:hv[k] for k in ['grid_trace_q10_q50_q90_px2','inbound_count','retained_count','occupied_3x3_cells']};receipts[c]={'controller_monotonic_ns':h['controller_monotonic_ns'],'native_interval_ns':interval,'sequence':hv['sequence']}
     common=[w['rows'][j]['rgb_left']['time_ns'] for j in [i,i+1]];E=transforms[c];body=E@matrices([mv['derived_camera_delta_xyzw']])[0]@np.linalg.inv(E);motions[c]=dict(project(body,interval,common),body_delta=body.tolist(),motion_sequence=mv['sequence'],motion_receipt_ns=m['controller_monotonic_ns'])
   check(all(eligible.values())==(native['mask']=='both'),'joint eligibility')
   rr.append({'window':case,'pair':i,'eligible':eligible})
   if not all(eligible.values()):continue
   x,reason=feature_vector(raw,f2);rows.append({'case':case,'clip':w['recording_id'],'pair':i,'deadline_ns':B,'common_interval_ns':common,'raw':raw,'receipts':receipts,'focal_squared':f2,'features':x,'invalid_reason':reason,'experts':motions})
  groups[case]={'core_rows':len(rr),'joint_rows':sum(all(r['eligible'].values()) for r in rr),'reference_source':w['reference_source'],'recipe':w['recipe'],'camera_metadata_qualified':True}
 check(len(rows)==171 and sum(g['core_rows'] for g in groups.values())==295,'all target population')
 write(OUT/'features-and-motions.json',{'rows':rows,'groups':groups,'reference_values_parsed':False,'checks':checks})
 write(OUT/'sufficiency.json',{'pass':True,'checks':checks,'groups':groups,'core_rows':295,'joint_rows':171,'invalid_features':sum(r['features'] is None for r in rows),'feature_sha256':sha(OUT/'features-and-motions.json')});print('PASS SUFFICIENCY',checks,'checks;171joint/295core')

def predict():
 import numpy as np
 p=verify();assert read(OUT/'sufficiency.json')['feature_sha256']==sha(OUT/'features-and-motions.json');rows=[]
 for r in read(OUT/'features-and-motions.json')['rows']:
  choices=dict(fixed_rgb='rgb',fixed_thermal='thermal',training_best_fixed='thermal',**fixed_choices(r))
  for name,m in p['models'].items():
   if r['features'] is None:choice='thermal'
   else:
    x=np.array([r['features'][i] for i in m['columns']]);score=float((x-np.array(m['mean']))/np.array(m['scale'])@np.array(m['coefficients'])+m['intercept']);choice='rgb' if score<0 else 'thermal'
   choices[name]=choice
  rows.append({'case':r['case'],'clip':r['clip'],'pair':r['pair'],'choices':choices,'signal_fallback':r['features'] is None})
 write(OUT/'predictions.json',{'rows':rows,'model_source_sha256':sha(PRIOR/'predictions.json'),'refitted':False})
 write(OUT/'prediction-lock.json',{'predictions_sha256':sha(OUT/'predictions.json'),'features_sha256':sha(OUT/'features-and-motions.json'),'plan_sha256':sha(PLAN),'models_sha256':sha(PRIOR/'predictions.json'),'reference_values_parsed':False});print('LOCKED',len(rows),'rows;',len(p['methods']),'methods; no refit')

def labels():
 np,_,_,losses,_,load_reference,_=scientific();p=verify();lock=read(OUT/'prediction-lock.json');assert lock['predictions_sha256']==sha(OUT/'predictions.json') and lock['features_sha256']==sha(OUT/'features-and-motions.json') and lock['plan_sha256']==sha(PLAN)
 refs={};refaudit={};windows={w['window']:w for w in p['windows']}
 for w in p['windows']:
  key=w['recording_id']
  if key in refaudit:continue
  for n in ['info.yaml','interp_timestamps.txt','interp_data.txt']:
   path=ROOT/w['reference_source']/'super_odometry/odometry'/n;assert sha(path)==p['expected_sources'][str(path.relative_to(ROOT))],path
  try:
   refs[key],info=load_reference(ROOT/w['reference_source']);refaudit[key]={'valid_source':True,'frames':info,'rows':len(refs[key].time_ns),'max_gap_ns':int(np.diff(refs[key].time_ns).max())}
  except ValueError as e:refaudit[key]={'valid_source':False,'reason':str(e)}
 rows=[]
 for source in read(OUT/'features-and-motions.json')['rows']:
  r={k:source[k] for k in ['case','clip','pair','common_interval_ns']};ref=refs.get(r['clip']);t=np.array(r['common_interval_ns'],dtype=np.int64);reason=None
  if not all(e['valid_model_projection'] for e in source['experts'].values()):reason='invalid_projection'
  elif ref is None:reason='invalid_reference_source'
  elif t[0]<ref.time_ns[0] or t[-1]>ref.time_ns[-1]:reason='reference_outside_support'
  else:
   for time in t:
    j=int(np.searchsorted(ref.time_ns,time))
    if j<len(ref.time_ns) and time==ref.time_ns[j]:continue
    if ref.time_ns[j]-ref.time_ns[j-1]>p['reference_max_gap_ns']:reason='reference_bracketing_gap'
  r['exclusion']=reason
  if reason is None:
   P=ref.at(t);target=np.linalg.solve(P[0],P[1]);r['reference_delta']=target.tolist();r['loss']={c:losses(e['delta_model'],target) for c,e in source['experts'].items()}
  rows.append(r)
 write(OUT/'labels.json',{'rows':rows,'reference_audit':refaudit,'prediction_lock_sha256':sha(OUT/'prediction-lock.json')});print('SCORED REFERENCES',len(rows),'excluded',sum(r['exclusion'] is not None for r in rows))

def evaluate():
 p=verify();prediction=read(OUT/'predictions.json')['rows'];label=read(OUT/'labels.json');valid={(r['case'],r['pair']):r['loss'] for r in label['rows'] if r['exclusion'] is None};rows=[r for r in prediction if (r['case'],r['pair']) in valid]
 def summary(rr):
  result={'rows':len(rr),'methods':{}}
  if not rr:return result
  fixed={c:sum(valid[r['case'],r['pair']][c]['rotation_deg'] for r in rr)/len(rr) for c in ['rgb','thermal']};oracle=sum(min(valid[r['case'],r['pair']][c]['rotation_deg'] for c in ['rgb','thermal']) for r in rr)/len(rr);best=min(fixed.values());toracle=sum(min(valid[r['case'],r['pair']][c]['translation_m'] for c in ['rgb','thermal']) for r in rr)/len(rr);tbest=min(sum(valid[r['case'],r['pair']][c]['translation_m'] for r in rr)/len(rr) for c in ['rgb','thermal'])
  for name in p['methods']:
   rm=sum(valid[r['case'],r['pair']][r['choices'][name]]['rotation_deg'] for r in rr)/len(rr);tm=sum(valid[r['case'],r['pair']][r['choices'][name]]['translation_m'] for r in rr)/len(rr);accuracy=sum(abs(valid[r['case'],r['pair']]['rgb']['rotation_deg']-valid[r['case'],r['pair']]['thermal']['rotation_deg'])<=1e-12 or valid[r['case'],r['pair']][r['choices'][name]]['rotation_deg']==min(valid[r['case'],r['pair']][c]['rotation_deg'] for c in ['rgb','thermal']) for r in rr)/len(rr)
   result['methods'][name]={'rotation_mean_deg':rm,'translation_mean_m':tm,'translation_oracle_mean_m':toracle,'translation_best_fixed_mean_m':tbest,'rotation_winner_accuracy':accuracy,'camera_counts':{c:sum(r['choices'][name]==c for r in rr) for c in ['rgb','thermal']},'oracle_mean_deg':oracle,'best_fixed_mean_deg':best,'headroom_recovered_fraction':(best-rm)/(best-oracle) if best>oracle else None}
  return result
 result={'cohort':p['cohort'],'aggregate':summary(rows),'recordings':{clip:summary([r for r in rows if r['clip']==clip]) for clip in sorted({w['recording_id'] for w in p['windows']})},'windows':{w['window']:summary([r for r in rows if r['case']==w['window']]) for w in p['windows']},'joint_population':171,'availability_population':295,'excluded':171-len(rows),'refitted':False,'development_exposed':True,'policy_adopted':False}
 write(OUT/'evaluation.json',result)
 for clip,g in [('aggregate',result['aggregate'])]+list(result['recordings'].items()):
  print(clip,g['rows'])
  for name,q in g['methods'].items():print(name,round(q['rotation_mean_deg'],6),round(q['translation_mean_m'],6),q['camera_counts'])
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','recover_metadata','sufficiency','predict','recover_poses','labels','evaluate','verify']);a=p.parse_args()
 if a.action.startswith('recover_'):recover(a.action.split('_')[1])
 else:globals()[a.action]()
