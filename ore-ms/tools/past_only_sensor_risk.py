"""Freeze, original-rig timing replication, past-only record-excluded prediction."""
from pathlib import Path
from decimal import Decimal
import argparse,collections,datetime,hashlib,json,math,sys,time
import numpy as np
from scipy.stats import spearmanr
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/past-only-sensor-risk/run-001';PLAN=ROOT/'experiments/past-only-sensor-risk-2026-11-03.json'
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.future_sensor_risk import current_history,fit_fold,predict,bind_target
SOURCES=[('exp112','results/sensor-admission-execution/run-002','plan.json','fresh-prefix-predictions.json','results/geometry-support-transfer/run-001/prior-context/locked-actions.json'),('exp113','results/geometry-support-transfer/run-001','plan.json','prefix-predictions.json','results/geometry-support-transfer/run-001/locked-actions.json'),('exp114','results/ms2-broader-range/run-001','execution-plan.json','prefix-predictions.json','results/ms2-broader-range/run-001/locked-actions.json')]
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,x):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(x,f,indent=2,allow_nan=False);f.write('\n')
def corr(x,y):
 x,y=np.asarray(x,float),np.asarray(y,float);valid=np.isfinite(x)&np.isfinite(y);x,y=x[valid],y[valid]
 return {'n':len(x),'rho':float(spearmanr(x,y).statistic) if len(x)>=10 and np.ptp(x)>0 and np.ptp(y)>0 else None}
def freeze():
 assert not PLAN.exists();sources=[];pins={};labels={}
 for tag,base,pname,prefix,controls in SOURCES:
  base=ROOT/base;plan=read(base/pname);cases=plan['cases'];sources.append({'tag':tag,'base':str(base.relative_to(ROOT)),'plan':pname,'prefix':prefix,'controls':controls,'cases':cases})
  for f in [base/pname,base/prefix,ROOT/controls]:pins[str(f.relative_to(ROOT))]=sha(f)
  f=base/'fresh-labels.json';labels[str(f.relative_to(ROOT))]=sha(f)
  for c in cases:
   capture=base/'captures'/c['case']/'on'
   for f in [capture/'timeline.jsonl',*capture.glob('experts/*/uncertainty/pair-*.npz'),*capture.glob('experts/*/report.json')]:pins[str(f.relative_to(ROOT))]=sha(f)
   if tag!='exp114':
    for camera in ['rgb','thermal']:
     for side in ['left','right']:
      p=ROOT/c['reference_dataset']/Path(c['rows'][0][camera+'_'+side]['path']).parent
      for f in [p/'timestamps.txt',p/'info.yaml']:
       if f.exists():pins[str(f.relative_to(ROOT))]=sha(f)
 for f in [Path(__file__),ROOT/'MAC-VO-main/Scripts/Setup/future_sensor_risk.py',ROOT/'docs/research/past-only-sensor-risk-protocol-2026-11-03.md',*sorted((ROOT/'MAC-VO-main/Scripts/UnitTest').glob('test_future_sensor_risk.py'))]:pins[str(f.relative_to(ROOT))]=sha(f)
 for f in (ROOT/'tools').glob('past_only_sensor_risk*.py'):pins[str(f.relative_to(ROOT))]=sha(f)
 for n in ['MACVO_FrontendCov.pth','MACT_Offroad.pth']:
  f=ROOT/'models/macslam'/n;pins[str(f.relative_to(ROOT))]=sha(f)
 plan={'parent_commit':'04f51f35d','client_date':'2026-11-03','created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'sources':sources,'input_source_sha256':pins,'separate_label_sha256':labels,'horizons_ns':[500_000_000,1_000_000_000],'primary_horizon_ns':500_000_000,'ridge':1.,'retrospective_exposed_development':True,'reserve_access':False,'scientific_runtime':'sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'}
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(pins),'source pins;',len(labels),'label files')
def verify():
 p=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in {**p['input_source_sha256'],**p['separate_label_sha256']}.items():assert sha(ROOT/n)==h,n
 return p

def prepare():
 plan=verify();forecasts=[];series=[];clock=[];reject=collections.Counter();latency=[]
 for source in plan['sources']:
  tag=source['tag'];base=ROOT/source['base'];controls={r['sample_id']:r for r in read(ROOT/source['controls'])['rows']};prefix={(r['window'],r['pair_ordinal']):r for r in read(base/source['prefix'])['rows']}
  for c in source['cases']:
   rows=c['rows'];n=len(rows)-1;timeline=[json.loads(s) for s in (base/'captures'/c['case']/'on/timeline.jsonl').read_text().splitlines()];decisions={e['pair_ordinal']:e for e in timeline if e['kind']=='decision'};events={cam:[e for e in timeline if e['kind']=='observer_event' and e['camera']==cam] for cam in ['rgb','thermal']};covs={};durations={};health={};domain='ms2' if tag=='exp114' else 'td3';lookup={r['pair_ordinal']:r for r in controls.values() if r['window']==c['window']}
   for cam in ['rgb','thermal']:
    stamps=[r[cam+'_left']['time_ns'] for r in rows];dt=np.diff(np.asarray(stamps,np.int64))/1e9;values=[];health[cam]={}
    for e in events[cam]:
     event=e['event']
     if event['kind']=='pair_end':health[cam][(event['time0_ns'],event['time1_ns'])]=e
    for i in range(n):
     p=base/'captures'/c['case']/'on/experts'/cam/'uncertainty'/f'pair-{i:03d}.npz'
     with np.load(p,allow_pickle=False) as a:
      assert int(a['time0_ns'])==stamps[i];K=a['K'].reshape(3,3);v=a['grid_raw_flow_cov'];v=v[:,0]/K[0,0]**2+v[:,1]/K[1,1]**2;v=v[np.isfinite(v)&(v>=0)];values.append(float(np.median(v)) if len(v) else None)
    covs[cam]=values;durations[cam]=dt.tolist();raw=np.asarray([x if x is not None else np.nan for x in values]);rate=raw/dt**2
    series.append({'cohort':tag,'domain':domain,'recording_id':c['recording_id'],'window':c['window'],'camera':cam,'covariance_trace':values,'native_dt_s':dt.tolist(),'dt_min_median_max':np.quantile(dt,[0,.5,1]).tolist(),'covariance_interval':corr(raw,dt),'lags':{str(k):{'raw':corr(raw[:-k],raw[k:]),'rate':corr(rate[:-k],rate[k:])} for k in [1,3,10]}})
    if domain=='td3':
     for side in ['left','right']:
      path=ROOT/c['reference_dataset']/Path(rows[0][cam+'_'+side]['path']).parent/'timestamps.txt';audit={'recording_id':c['recording_id'],'window':c['window'],'camera':cam,'side':side,'path':str(path.relative_to(ROOT)),'physical_exposure_verified':False,'source_available':path.exists()}
      if path.exists():
       vals=path.read_text().split();diff=[]
       for r in rows:
        sample=r[cam+'_'+side];expected=int((Decimal(vals[sample['index']])*Decimal(10**9)).to_integral_value());diff.append(sample['time_ns']-expected)
       assert max(map(abs,diff))<=1000;audit.update(samples=len(diff),max_absolute_serialization_ns=max(map(abs,diff)),source_sha256=sha(path))
      clock.append(audit)
    begin={}
    for entry in events[cam]:
     e=entry['event']
     if e['kind']=='pair_begin':begin[(e['interval']['time0_ns'],e['interval']['time1_ns'])]=e['observation_monotonic_ns']
     elif e['kind']=='pair_end':
      duration=(e['observation_monotonic_ns']-begin[(e['time0_ns'],e['time1_ns'])])/1e9;assert duration>=0;latency.append({'domain':domain,'recording_id':c['recording_id'],'window':c['window'],'camera':cam,'pair_duration_s':duration})
   for i in range(n):
    if i<2:reject['cold_start']+=len(plan['horizons_ns']);continue
    d=decisions[i];p=prefix[c['window'],i];assert p['controller_deadline_ns']==d['controller_monotonic_ns'];origin=lookup[i];receipt=[];valid=True
    for cam in ['rgb','thermal']:
     for k in range(3):
      e=health[cam].get((rows[i-k][cam+'_left']['time_ns'],rows[i-k+1][cam+'_left']['time_ns']))
      if e is None or e['controller_monotonic_ns']>p['controller_deadline_ns'] or covs[cam][i-k] is None:valid=False;continue
      receipt.append({'camera':cam,'pair':i-k,'event_sequence':e['event']['sequence'],'received_ns':e['controller_monotonic_ns']})
    if not valid:reject['missing_past_covariance_or_receipt']+=len(plan['horizons_ns']);continue
    state={cam:{'available':origin['native_available'][cam],'information':origin['normalized_information_min_eigenvalue'][cam],'retained':origin['support_features'][cam]['retained_count'] if origin['support_features'][cam] else 0,'cells':origin['support_features'][cam]['occupied_cells'] if origin['support_features'][cam] else 0,'fraction':origin['support_features'][cam]['retained_fraction'] if origin['support_features'][cam] else 0} for cam in ['rgb','thermal']}
    current,history=current_history(state,{cam:[covs[cam][i-k] for k in range(3)] for cam in state},{cam:[durations[cam][i-k] for k in range(3)] for cam in state})
    released=[e for e in timeline if e['kind']=='release' and e['controller_monotonic_ns']<=p['controller_deadline_ns']];latest=max(e['frame'] for e in released);release=min(e['controller_monotonic_ns'] for e in released if e['frame']==i+1);elapsed=(p['controller_deadline_ns']-release)/1e9
    for h in plan['horizons_ns']:
     j=bind_target(rows,i,h)
     if j is None:reject['no_time_match']+=1;continue
     if latest>=j:reject['target_already_released']+=1;continue
     target=lookup[j];sid=tag+'/'+origin['sample_id'];tid=tag+'/'+target['sample_id'];lead=(rows[j]['rgb_left']['time_ns']-rows[i+1]['rgb_left']['time_ns'])/1e9;observed_latest=max(rows[e['frame']][e['camera']+'_left']['time_ns'] for e in released)
     if observed_latest>=rows[j]['rgb_left']['time_ns']:reject['target_start_not_after_observed_sensor_time']+=1;continue
     forecasts.append({'forecast_id':sid+'@'+str(h),'origin_id':sid,'target_id':tid,'recording_id':c['recording_id'],'domain':domain,'window':tag+'/'+c['window'],'origin_pair':i,'target_pair':j,'target_start_ns':rows[j]['rgb_left']['time_ns'],'origin_end_ns':rows[i+1]['rgb_left']['time_ns'],'horizon_ns':h,'realized_start_lead_s':lead,'current':current,'history':history,'origin_information_action':origin['actions']['information_rank'],'receipts':receipt,'origin_controller_deadline_ns':p['controller_deadline_ns'],'latest_released_frame':latest,'observed_offline_prefix_delay_s':elapsed,'remaining_instrumented_budget_s':lead-elapsed})
 write(OUT/'source-features.json',{'rows':forecasts,'labels_read':False,'feature_dimensions':{'current':16,'history':20},'rejections':dict(reject)});write(OUT/'persistence-replication.json',{'series':series,'sensor_timestamp_audit':clock,'physical_exposure_clocks_verified':False});write(OUT/'native-offline-latencies.json',{'rows':latency,'live_deadline_verified':False});verify();print('PREPARED',len(forecasts),'forecasts;',len(series),'sensor series;',dict(reject))

def load_labels(plan):
 labels={}
 for s in plan['sources']:
  labels.update({s['tag']+'/'+k:v for k,v in read(ROOT/s['base']/'fresh-labels.json').items()})
 return labels

def train_predict():
 plan=verify();rows=read(OUT/'source-features.json')['rows'];labels=load_labels(plan);models=[];outputs=[]
 for h in plan['horizons_ns']:
  subset=[r for r in rows if r['horizon_ns']==h]
  for record in sorted({r['recording_id'] for r in subset}):
   test=[r for r in subset if r['recording_id']==record];trainlabels={r['target_id']:labels[r['target_id']] for r in subset if r['recording_id']!=record and r['target_id'] in labels};predictions={}
   for family in ['current','history']:
    model=fit_fold(subset,trainlabels,record,family);model.update(horizon_ns=h,model_id=str(h)+'/'+record+'/'+family);models.append(model)
    for r in test:
     start=time.perf_counter_ns();value=predict(model,r[family]);value['predict_cpu_seconds']=(time.perf_counter_ns()-start)/1e9;predictions.setdefault(r['forecast_id'],{})[family]=value
   for r in test:outputs.append({**{k:r[k] for k in ['forecast_id','target_id','recording_id','domain','window','origin_pair','target_pair','target_start_ns','horizon_ns','realized_start_lead_s','observed_offline_prefix_delay_s','remaining_instrumented_budget_s']},'predictions':predictions[r['forecast_id']],'remaining_instrumented_budget_including_prediction_s':r['remaining_instrumented_budget_s']-max(v['predict_cpu_seconds'] for v in predictions[r['forecast_id']].values()),'actions':{'current':predictions[r['forecast_id']]['current']['action'],'history':predictions[r['forecast_id']]['history']['action'],'rgb_only':'RGB','thermal_only':'THERMAL','origin_information':r['origin_information_action']}})
 write(OUT/'models.json',{'models':models,'heldout_recording_excluded_before_label_lookup':True});write(OUT/'locked-predictions.json',{'rows':outputs,'test_labels_used_for_prediction':False,'retrospective_exposed_data':True});write(OUT/'prediction-lock.json',{'predictions_sha256':sha(OUT/'locked-predictions.json'),'models_sha256':sha(OUT/'models.json'),'features_sha256':sha(OUT/'source-features.json'),'source_sha256':sha(__file__),'test_labels_used_for_prediction':False});verify();print('LOCKED',len(outputs),'forecasts;',len(models),'fixed fits')
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','verify','prepare','train_predict']);a=p.parse_args()
 if a.action=='verify':print('PASS',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()
