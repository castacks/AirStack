"""EXP131 fixed retrospective recording-disjoint completed-output signal audit."""
import argparse,json,math,sys
from pathlib import Path
from provided_quality_headroom import ROOT,read,write,sha,verify as parent_verify
OUT=ROOT/'results/rotation-signal-audit/run-001';PLAN=ROOT/'experiments/rotation-signal-audit-2026-11-04.json'
NATIVE=ROOT/'results/provided-export-survey/run-001';QUALITY=ROOT/'results/provided-quality-headroom/run-001'
FEATURES=['log_angular_q10_ratio','log_angular_q50_ratio','log_angular_q90_ratio','retention_fraction_difference','occupancy_fraction_difference','log_inbound_ratio']
RIDGE={'ridge_uncertainty':[0,1,2],'ridge_health':[3,4,5],'ridge_combined':[0,1,2,3,4,5]}
METHODS=['fixed_rgb','fixed_thermal','training_best_fixed','raw_uncertainty','angular_uncertainty','retention','occupancy']+list(RIDGE)
def verify():
 p=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in p['sha256'].items():assert sha(ROOT/n)==h,n
 parent_verify();return p

def freeze():
 assert not OUT.exists() and not PLAN.exists();parent_verify()
 windows=read(ROOT/'experiments/provided-export-survey-2026-11-04.json')['windows']
 files=[Path(__file__),ROOT/'tools/rotation_signal_audit_check.py',ROOT/'docs/research/rotation-signal-audit-protocol-2026-11-04.md',QUALITY/'scores.json',QUALITY/'audit.json',QUALITY/'prediction-lock.json',NATIVE/'native-survey.json',NATIVE/'native-audit.json']
 for w in windows:
  files.append(NATIVE/'captures'/w['case']/'timeline.jsonl')
  for c,name in [('rgb','multisense_left_rect_color_camera_info'),('thermal','thermal_left_processed_camera_info')]:files.append(ROOT/w['dataset']/'sensors'/name/'data.yaml')
 plan={'date':'2026-11-04','parent_commit':'a00a87918','windows':[{k:w[k] for k in ['case','clip','dataset']} for w in windows],'sha256':{str(f.relative_to(ROOT)):sha(f) for f in files},'features':FEATURES,'ridge_columns':RIDGE,'lambda':1.,'primary':'ridge_combined','methods':METHODS,'fold_unit':'complete_recording','development_exposed':True,'prior_reference_outcomes_inspected':True,'expected_rows':323,'availability_rows':380,'decision':'completed-output at existing group i+2 decision; both experts already executed','physical_ground_truth':False,'reserved_payload_access':False,'runtime_image':read(QUALITY/'plan.json')['runtime_image']}
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(plan['sha256']),'direct pins plus EXP130 ancestry')

def receipt(event,interval,deadline):
 assert event['controller_monotonic_ns']<=deadline,'signal after output deadline'
 assert [event['event']['time0_ns'],event['event']['time1_ns']]==interval,'signal interval mismatch'
 return event['event']

def feature_vector(raw,f2):
 try:
  for c in ['rgb','thermal']:
   r=raw[c];q=r['grid_trace_q10_q50_q90_px2'];assert len(q)==3 and all(math.isfinite(x) and x>0 for x in q)
   assert r['inbound_count']>0 and 0<=r['retained_count']<=r['inbound_count'] and 0<=r['occupied_3x3_cells']<=9
   assert math.isfinite(f2[c]) and f2[c]>0
  x=[math.log((raw['rgb']['grid_trace_q10_q50_q90_px2'][i]/f2['rgb'])/(raw['thermal']['grid_trace_q10_q50_q90_px2'][i]/f2['thermal'])) for i in range(3)]
  x += [raw['rgb']['retained_count']/raw['rgb']['inbound_count']-raw['thermal']['retained_count']/raw['thermal']['inbound_count'],(raw['rgb']['occupied_3x3_cells']-raw['thermal']['occupied_3x3_cells'])/9,math.log(raw['rgb']['inbound_count']/raw['thermal']['inbound_count'])]
  assert all(math.isfinite(v) for v in x);return x,None
 except (AssertionError,TypeError,KeyError,ZeroDivisionError,ValueError):return None,'invalid_signal_values'

def features():
 import numpy as np,yaml
 p=verify();native=read(NATIVE/'native-survey.json')['rows'];rows=[];windows={w['case']:w for w in p['windows']}
 for case,w in windows.items():
  events=[json.loads(l) for l in (NATIVE/'captures'/case/'timeline.jsonl').read_text().splitlines()];health={};f2={}
  for e in events:
   if e['kind']=='observer_event' and e['event']['kind']=='pair_end':
    key=e['camera'],e['event']['time1_ns'];assert key not in health;health[key]=e
  for c,name in [('rgb','multisense_left_rect_color_camera_info'),('thermal','thermal_left_processed_camera_info')]:
   P=np.asarray(yaml.safe_load((ROOT/w['dataset']/'sensors'/name/'data.yaml').read_text())['p']).reshape(3,4);f2[c]=float((P[0,0]**2+P[1,1]**2)/2)
  for r in native:
   if r['case']!=case or r['mask']!='both':continue
   raw={};receipts={}
   for c in ['rgb','thermal']:
    interval=r['experts'][c]['native_interval_ns'];event=health[c,interval[1]];h=receipt(event,interval,r['deadline_ns']);assert event['controller_monotonic_ns']==r['experts'][c]['health_receipt_ns']
    raw[c]={k:h[k] for k in ['grid_trace_q10_q50_q90_px2','inbound_count','retained_count','occupied_3x3_cells']};receipts[c]={'controller_monotonic_ns':event['controller_monotonic_ns'],'native_interval_ns':interval,'sequence':h['sequence']}
   x,reason=feature_vector(raw,f2)
   rows.append({'case':case,'clip':w['clip'],'pair':r['pair'],'deadline_ns':r['deadline_ns'],'receipts':receipts,'focal_squared':f2,'raw':raw,'features':x,'invalid_reason':reason})
 assert len(rows)==323;write(OUT/'features.json',{'columns':FEATURES,'rows':rows,'reference_used':False})
 write(OUT/'feature-lock.json',{'plan_sha256':sha(PLAN),'features_sha256':sha(OUT/'features.json'),'rows':len(rows)});print('LOCKED FEATURES',len(rows),'invalid',sum(r['features'] is None for r in rows))

def fit_predict(training,evaluation,labels,columns):
 import numpy as np
 valid=[r for r in training if r['features'] is not None]
 if not valid:return {'fallback':'no_valid_training_rows'},['thermal']*len(evaluation)
 X=np.asarray([[r['features'][i] for i in columns] for r in valid]);mu=X.mean(axis=0);sd=X.std(axis=0);sd=np.where(sd==0.,1.,sd);Z=(X-mu)/sd
 y=np.array([labels[r['case'],r['pair']]['rgb']['rotation_deg']-labels[r['case'],r['pair']]['thermal']['rotation_deg'] for r in valid]);intercept=float(y.mean());b=np.linalg.solve(Z.T@Z/len(Z)+np.eye(len(columns)),Z.T@(y-intercept)/len(Z))
 model={'columns':columns,'mean':mu.tolist(),'scale':sd.tolist(),'coefficients':b.tolist(),'intercept':intercept,'training_ids':[[r['case'],r['pair']] for r in valid],'training_rows':len(valid),'zero_variance_columns':[columns[i] for i in range(len(columns)) if X[:,i].std()==0.]}
 choices=['thermal' if r['features'] is None else 'rgb' if float(((np.array([r['features'][i] for i in columns])-mu)/sd)@b+intercept)<0 else 'thermal' for r in evaluation]
 return model,choices

def fixed_choices(r):
 if r['features'] is None:return {k:'thermal' for k in ['raw_uncertainty','angular_uncertainty','retention','occupancy']}
 x=r['features'];return {'raw_uncertainty':'rgb' if r['raw']['rgb']['grid_trace_q10_q50_q90_px2'][1]<r['raw']['thermal']['grid_trace_q10_q50_q90_px2'][1] else 'thermal','angular_uncertainty':'rgb' if x[1]<0 else 'thermal','retention':'rgb' if x[3]>0 else 'thermal','occupancy':'rgb' if x[4]>0 else 'thermal'}

def predict():
 p=verify();fl=read(OUT/'feature-lock.json');assert fl['features_sha256']==sha(OUT/'features.json') and fl['plan_sha256']==sha(PLAN)
 rows=read(OUT/'features.json')['rows'];labels={(r['case'],r['pair']):r['loss'] for r in read(QUALITY/'scores.json')['rows']};folds={};outputs=[]
 for clip in sorted({r['clip'] for r in rows}):
  training=[r for r in rows if r['clip']!=clip];evaluation=[r for r in rows if r['clip']==clip];assert len({r['clip'] for r in training})==1
  # Only training-recording labels enter fitting and the learned fixed-camera baseline.
  trainlabels={(r['case'],r['pair']):labels[r['case'],r['pair']] for r in training}
  trainmeans={c:sum(v[c]['rotation_deg'] for v in trainlabels.values())/len(training) for c in ['rgb','thermal']};best='rgb' if trainmeans['rgb']<trainmeans['thermal'] else 'thermal';models={};predictions={}
  for name,columns in RIDGE.items():models[name],predictions[name]=fit_predict(training,evaluation,trainlabels,columns)
  folds[clip]={'training_recordings':sorted({r['clip'] for r in training}),'evaluation_rows':len(evaluation),'training_rotation_means':trainmeans,'training_best_fixed':best,'models':models}
  for i,r in enumerate(evaluation):outputs.append({'case':r['case'],'clip':clip,'pair':r['pair'],'choices':dict(fixed_rgb='rgb',fixed_thermal='thermal',training_best_fixed=best,**fixed_choices(r),**{name:v[i] for name,v in predictions.items()}),'signal_fallback':r['features'] is None})
 write(OUT/'predictions.json',{'folds':folds,'rows':outputs,'test_labels_used_for_prediction':False,'development_exposed':True})
 write(OUT/'prediction-lock.json',{'feature_lock_sha256':sha(OUT/'feature-lock.json'),'predictions_sha256':sha(OUT/'predictions.json'),'source_sha256':sha(__file__)});print('LOCKED',len(outputs),'recording-disjoint choices;',len(METHODS),'methods')

def summary(rows,labels):
 import numpy as np
 result={'rows':len(rows),'methods':{}}
 for name in METHODS:
  chosen=[labels[r['case'],r['pair']][r['choices'][name]] for r in rows];rotation=np.array([v['rotation_deg'] for v in chosen]);translation=[v['translation_m'] for v in chosen]
  oracle=np.array([min(v['rgb']['rotation_deg'],v['thermal']['rotation_deg']) for r in rows for v in [labels[r['case'],r['pair']]]]);fixed={c:np.mean([labels[r['case'],r['pair']][c]['rotation_deg'] for r in rows]) for c in ['rgb','thermal']};best=min(fixed.values());headroom=best-float(oracle.mean());mean=float(rotation.mean())
  accuracy=sum(abs(labels[r['case'],r['pair']]['rgb']['rotation_deg']-labels[r['case'],r['pair']]['thermal']['rotation_deg'])<=1e-12 or labels[r['case'],r['pair']][r['choices'][name]]['rotation_deg']==min(labels[r['case'],r['pair']][c]['rotation_deg'] for c in ['rgb','thermal']) for r in rows)/len(rows)
  ordered=sorted(rows,key=lambda r:(r['case'],r['pair']));changes=sum(a['case']==b['case'] and b['pair']==a['pair']+1 and a['choices'][name]!=b['choices'][name] for a,b in zip(ordered,ordered[1:]))
  result['methods'][name]={'rotation_mean_deg':mean,'translation_mean_m':float(np.mean(translation)),'rotation_winner_accuracy':accuracy,'oracle_mean_deg':float(oracle.mean()),'best_fixed_mean_deg':float(best),'oracle_gap_deg':mean-float(oracle.mean()),'headroom_recovered_fraction':(best-mean)/headroom if headroom>0 else None,'camera_counts':{c:sum(r['choices'][name]==c for r in rows) for c in ['rgb','thermal']},'adjacent_choice_changes':changes}
 return result

def evaluate():
 p=verify();lock=read(OUT/'prediction-lock.json');assert lock['predictions_sha256']==sha(OUT/'predictions.json') and lock['feature_lock_sha256']==sha(OUT/'feature-lock.json')
 rows=read(OUT/'predictions.json')['rows'];labels={(r['case'],r['pair']):r['loss'] for r in read(QUALITY/'scores.json')['rows']}
 result={'aggregate':summary(rows,labels),'recordings':{c:summary([r for r in rows if r['clip']==c],labels) for c in sorted({r['clip'] for r in rows})},'windows':{c:summary([r for r in rows if r['case']==c],labels) for c in sorted({r['case'] for r in rows})},'fallback_rows':sum(r['signal_fallback'] for r in rows),'availability_population':380,'development_exposed':True,'physical_ground_truth':False,'policy_adopted':False}
 from scipy.stats import spearmanr
 fr=read(OUT/'features.json')['rows'];associations={}
 for clip in sorted({r['clip'] for r in fr}):
  rr=[r for r in fr if r['clip']==clip and r['features'] is not None];y=[labels[r['case'],r['pair']]['rgb']['rotation_deg']-labels[r['case'],r['pair']]['thermal']['rotation_deg'] for r in rr];associations[clip]={}
  for i,name in enumerate(FEATURES):
   x=[r['features'][i] for r in rr];rho=float(spearmanr(x,y).statistic) if len(set(x))>1 and len(set(y))>1 else None;associations[clip][name]={'min':min(x),'max':max(x),'spearman_loss_difference':rho}
 result['descriptive_signal_associations']=associations
 write(OUT/'evaluation.json',result)
 for clip,g in [('aggregate',result['aggregate'])]+list(result['recordings'].items()):
  print(clip)
  for name,q in g['methods'].items():print(name,round(q['rotation_mean_deg'],6),q['camera_counts'],round(q['headroom_recovered_fraction'],4))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','features','predict','evaluate','verify']);a=p.parse_args();globals()[a.action]()
