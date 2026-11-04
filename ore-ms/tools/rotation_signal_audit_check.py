"""Independent provenance, ridge and selector-loss checks plus causal mutations."""
import copy,json,math
import numpy as np,yaml
from rotation_signal_audit import ROOT,OUT,NATIVE,QUALITY,PLAN,FEATURES,RIDGE,METHODS,read,write,sha,verify,receipt,fit_predict,feature_vector,fixed_choices

def main():
 p=verify();checks=0
 def check(v,label):
  nonlocal checks
  assert v,label
  checks+=1
 fl=read(OUT/'feature-lock.json');pl=read(OUT/'prediction-lock.json');features=read(OUT/'features.json')['rows'];prediction=read(OUT/'predictions.json');evaluation=read(OUT/'evaluation.json');quality=read(QUALITY/'scores.json')['rows'];labels={(r['case'],r['pair']):r['loss'] for r in quality};source={(r['case'],r['pair']):r for r in read(NATIVE/'native-survey.json')['rows']}
 check(fl['plan_sha256']==sha(PLAN) and fl['features_sha256']==sha(OUT/'features.json'),'feature lock')
 check(pl['predictions_sha256']==sha(OUT/'predictions.json') and pl['feature_lock_sha256']==sha(OUT/'feature-lock.json'),'prediction lock')
 check(len(features)==323 and {(r['case'],r['pair']) for r in features}==set(labels),'population fixed')
 preds={(r['case'],r['pair']):r for r in prediction['rows']};check(set(preds)==set(labels),'all choices retained')
 windows={w['case']:w for w in p['windows']};events={};focals={}
 for case,w in windows.items():
  events[case]={}
  for line in (NATIVE/'captures'/case/'timeline.jsonl').read_text().splitlines():
   e=json.loads(line)
   if e['kind']=='observer_event' and e['event']['kind']=='pair_end':events[case][e['camera'],e['event']['time1_ns']]=e
  focals[case]={}
  for c,name in [('rgb','multisense_left_rect_color_camera_info'),('thermal','thermal_left_processed_camera_info')]:
   P=np.array(yaml.safe_load((ROOT/w['dataset']/'sensors'/name/'data.yaml').read_text())['p']).reshape(3,4);focals[case][c]=float((P[0,0]**2+P[1,1]**2)*.5)
 for r in features:
  src=source[r['case'],r['pair']];check(src['mask']=='both' and src['deadline_ns']==r['deadline_ns'],'availability and deadline')
  for c in ['rgb','thermal']:
   interval=src['experts'][c]['native_interval_ns'];e=events[r['case']][c,interval[1]];h=e['event']
   check(e['controller_monotonic_ns']<=r['deadline_ns'] and [h['time0_ns'],h['time1_ns']]==interval,'causal interval receipts')
   check(r['receipts'][c]['controller_monotonic_ns']==e['controller_monotonic_ns'] and r['receipts'][c]['sequence']==h['sequence'],'receipt provenance')
   check(r['raw'][c]=={k:h[k] for k in r['raw'][c]},'raw feature whitelist')
   check(r['focal_squared'][c]==focals[r['case']][c],'provided camera scale')
  if r['features'] is not None:
   a,b=r['raw']['rgb'],r['raw']['thermal'];scale=focals[r['case']]
   x=[math.log(a['grid_trace_q10_q50_q90_px2'][i])-math.log(b['grid_trace_q10_q50_q90_px2'][i])-math.log(scale['rgb'])+math.log(scale['thermal']) for i in range(3)]+[a['retained_count']/a['inbound_count']-b['retained_count']/b['inbound_count'],a['occupied_3x3_cells']/9-b['occupied_3x3_cells']/9,math.log(a['inbound_count'])-math.log(b['inbound_count'])]
   check(np.allclose(x,r['features'],atol=1e-12,rtol=0),'independent feature expressions')
  for name,c in fixed_choices(r).items():check(preds[r['case'],r['pair']]['choices'][name]==c,'fixed signal rule')
 mutations=0
 for clip,fold in prediction['folds'].items():
  train=[r for r in features if r['clip']!=clip];test=[r for r in features if r['clip']==clip];check(not {r['clip'] for r in train}&{r['clip'] for r in test},'recording disjointness')
  valid=[r for r in train if r['features'] is not None];trainlabels={(r['case'],r['pair']):labels[r['case'],r['pair']] for r in train};means={c:np.mean([v[c]['rotation_deg'] for v in trainlabels.values()]) for c in ['rgb','thermal']};best='rgb' if means['rgb']<means['thermal'] else 'thermal'
  check(fold['training_best_fixed']==best,'train-only best fixed')
  for name,columns in RIDGE.items():
   model=fold['models'][name];X=np.array([[r['features'][i] for i in columns] for r in valid]);mu=np.sum(X,axis=0)/len(X);sd=np.sqrt(np.sum((X-mu)**2,axis=0)/len(X));sd[sd==0]=1.;Z=(X-mu)/sd;y=np.array([trainlabels[r['case'],r['pair']]['rgb']['rotation_deg']-trainlabels[r['case'],r['pair']]['thermal']['rotation_deg'] for r in valid]);intercept=y.mean()
   # Independent augmented least-squares solution to mean-MSE + L2 objective.
   A=np.vstack([Z/np.sqrt(len(Z)),np.eye(len(columns))]);b=np.r_[(y-intercept)/np.sqrt(len(Z)),np.zeros(len(columns))];beta=np.linalg.lstsq(A,b,rcond=None)[0]
   check(np.allclose(mu,model['mean'],atol=1e-12) and np.allclose(sd,model['scale'],atol=1e-12),'training-only scaling')
   check(np.allclose(beta,model['coefficients'],atol=1e-12) and abs(intercept-model['intercept'])<1e-12,'independent ridge solution')
   check(model['training_ids']==[[r['case'],r['pair']] for r in valid],'training ID membership')
   for r in test:
    c='thermal' if r['features'] is None else 'rgb' if ((np.array([r['features'][i] for i in columns])-mu)/sd)@beta+intercept<0 else 'thermal'
    check(c==preds[r['case'],r['pair']]['choices'][name],'fold choice reproduction')
   # Arbitrary evaluation labels must never affect this fold's model or choices.
   poisoned=copy.deepcopy(labels)
   for r in test:
    poisoned[r['case'],r['pair']]['rgb']['rotation_deg']=10000.;poisoned[r['case'],r['pair']]['thermal']['rotation_deg']=0.
   m1,c1=fit_predict(train,test,labels,columns);m2,c2=fit_predict(train,test,poisoned,columns);check(m1==m2 and c1==c2,'evaluation label invariance');mutations+=1
 for name,group in [('aggregate',evaluation['aggregate'])]+list(evaluation['recordings'].items())+list(evaluation['windows'].items()):
  rows=prediction['rows'] if name=='aggregate' else [r for r in prediction['rows'] if r['clip']==name or r['case']==name];check(len(rows)==group['rows'],'matched summary denominator')
  for method,q in group['methods'].items():
   selected=[labels[r['case'],r['pair']][r['choices'][method]] for r in rows];rm=sum(v['rotation_deg'] for v in selected)/len(rows);tm=sum(v['translation_m'] for v in selected)/len(rows)
   check(abs(rm-q['rotation_mean_deg'])<1e-12 and abs(tm-q['translation_mean_m'])<1e-12,'independent selected losses')
   check(q['camera_counts']=={c:sum(r['choices'][method]==c for r in rows) for c in ['rgb','thermal']},'camera counts')
   check(q['oracle_gap_deg']>=-1e-12,'oracle bound')
 sample=features[0];src=source[sample['case'],sample['pair']];interval=src['experts']['rgb']['native_interval_ns'];event=events[sample['case']]['rgb',interval[1]]
 for kind in ['late','interval']:
  altered=copy.deepcopy(event)
  if kind=='late':altered['controller_monotonic_ns']=sample['deadline_ns']+1
  else:altered['event']['time0_ns']-=1
  rejected=False
  try:receipt(altered,interval,sample['deadline_ns'])
  except AssertionError:rejected=True
  check(rejected,'reject '+kind+' signal');mutations+=1
 raw=copy.deepcopy(sample['raw']);raw['rgb']['grid_trace_q10_q50_q90_px2']=None;check(feature_vector(raw,sample['focal_squared'])[0] is None,'invalid signal detected');mutations+=1
 bad=copy.deepcopy(sample);bad['features']=None;check(set(fixed_choices(bad).values())=={'thermal'},'invalid-feature fallback');mutations+=1
 write(OUT/'audit.json',{'pass':True,'checks':checks,'semantic_mutations':mutations,'rows':len(features),'source_sha256':sha(__file__),'plan_sha256':sha(PLAN),'evaluation_sha256':sha(OUT/'evaluation.json'),'prediction_sha256':sha(OUT/'predictions.json'),'independent_ridge':'augmented least squares'})
 print('PASS',checks,'checks;',mutations,'semantic mutations')
if __name__=='__main__':main()
