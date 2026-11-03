"""Fresh matched native preservation and receipt-stage timing; no pose-loss scoring."""
from collections import Counter
import argparse,json
import numpy as np
from early_health_execution import ROOT,OUT,read,write,sha,verify
EARLY={'early_covariance_available','early_tracking_available'}
def journal(p):return [json.loads(s) for s in p.read_text().splitlines()]
def quantiles(values):return {'n':len(values),'min':float(np.min(values)),'median':float(np.median(values)),'p95':float(np.quantile(values,.95)),'max':float(np.max(values))} if values else {'n':0}
def run():
 plan=verify();equal=0;drift=[];caseproof=[];timing=[];earlyevents=0
 for c in plan['fresh_cases']:
  base=OUT/'captures'/c['case'];arms={arm:base/arm for arm in ['off','on']};arrays=0
  a={str(p.relative_to(arms['off'])):p for p in arms['off'].rglob('*.npz')};b={str(p.relative_to(arms['on'])):p for p in arms['on'].rglob('*.npz')};assert a.keys()==b.keys()
  for rel,p in a.items():
   with np.load(p,allow_pickle=False) as x,np.load(b[rel],allow_pickle=False) as y:
    assert set(x.files)==set(y.files)
    for k in x.files:
     assert x[k].shape==y[k].shape and x[k].dtype==y[k].dtype
     if np.array_equal(x[k],y[k],equal_nan=True):arrays+=1;continue
     allowed=(rel.endswith('tensor-map.npz') and k=='match//obs2_covTc') or ('/uncertainty/pair-' in rel and k in ['endpoint1_depth_variance_before','endpoint1_depth_variance_after']);assert allowed,(c['case'],rel,k)
     u=x[k].astype(float);v=y[k].astype(float);assert np.array_equal(np.isfinite(u),np.isfinite(v));f=np.isfinite(u)&np.isfinite(v);d=abs(u-v);drift.append({'case':c['case'],'path':rel,'key':k,'max_absolute':float(np.max(d[f])),'max_relative':float(np.max(d[f]/np.maximum(np.maximum(abs(u[f]),abs(v[f])),1e-30)))})
  expert=[]
  for cam in ['rgb','thermal']:
   aa=arms['off']/'experts'/cam;bb=arms['on']/'experts'/cam
   assert read(aa/'effective-config.json')==read(bb/'effective-config.json');reports=[read(p/'report.json') for p in [aa,bb]];assert reports[0]['status']==reports[1]['status'] and reports[0]['tracking_diagnostics']==reports[1]['tracking_diagnostics'];assert reports[0]['completed_frames']==c['frames'];assert read(aa/'terminal-capture-contract.json')==read(bb/'terminal-capture-contract.json')
   ea=journal(aa/'arbiter-events/events.jsonl');eb=journal(bb/'arbiter-events/events.jsonl')
   def normalized(events):return [{k:v for k,v in e.items() if k not in ['sequence','observation_monotonic_ns']} for e in events if e['kind'] not in EARLY]
   assert normalized(ea)==normalized(eb)
   for p in [aa,bb]:
    summary=read(p/'arbiter-events/summary.json');assert summary['status']=='COMPLETE' and not summary['observer_errors']
   cov=[e for e in eb if e['kind']=='early_covariance_available'];track=[e for e in eb if e['kind']=='early_tracking_available'];ends=[e for e in eb if e['kind']=='pair_end'];assert len(cov)==len(track)==len(ends)==c['frames']-1
   for cv,tr,end in zip(cov,track,ends):
    assert [cv['time0_ns'],cv['time1_ns']]==[tr['time0_ns'],tr['time1_ns']]==[end['time0_ns'],end['time1_ns']];assert cv['sequence']<tr['sequence']<end['sequence'];assert tr['native_min_num_point']==10;assert tr['tracking_count_threshold_met']==(tr['retained_count']>=tr['native_min_num_point'])
    assert tr['tracking_count_threshold_met']==(not end['original_need_interp']) and end['execution_error'] is None
    for k in ['selected_count','grid_count','grid_positive_uu_vv_count','grid_trace_q10_q50_q90_px2']:assert cv[k]==end[k]
    for k in ['selected_count','inbound_count','retained_count','occupied_3x3_cells']:assert tr[k]==end[k]
   earlyevents+=len(cov)+len(track);expert.append({'camera':cam,'native_status':reports[0]['status'],'tracking':reports[0]['tracking_diagnostics'],'early_events':len(cov)+len(track)})
  logs={arm:journal(p/'timeline.jsonl') for arm,p in arms.items()}
  def signature(log):
   return [(e['pair_ordinal'],e['action'],e['availability'],e['boundary'],{cam:{k:v for k,v in info.items() if k not in ['health_event_sequence','motion_event_sequence']} for cam,info in e['evidence'].items()}) for e in log if e['kind']=='decision']
  assert signature(logs['off'])==signature(logs['on']) and len(signature(logs['on']))==c['frames']-1
  for arm,log in logs.items():
   decisions={e['pair_ordinal']:e for e in log if e['kind']=='decision'}
   for i in range(c['frames']-1):
    release=min(e['controller_monotonic_ns'] for e in log if e['kind']=='release' and e['frame']==i+1);parts={}
    for cam in ['rgb','thermal']:
     first,last=c['rows'][i][cam+'_left']['time_ns'],c['rows'][i+1][cam+'_left']['time_ns'];parts[cam]={}
     for kind,name in [('early_covariance_available','covariance'),('early_tracking_available','tracking'),('pair_end','pair_end'),('native_disparity_information','information')]:
      found=[e for e in log if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']==kind and [e['event']['time0_ns'],e['event']['time1_ns']]==[first,last]]
      assert len(found)<=1
      parts[cam][name]=(found[0]['controller_monotonic_ns']-release)/1e9 if found else None
    record={'case':c['case'],'domain':c['domain'],'arm':arm,'pair':i,'per_camera':parts,'old_decision_prefix_s':(decisions[i]['controller_monotonic_ns']-release)/1e9,'source_rgb_dt_s':(c['rows'][i+1]['rgb_left']['time_ns']-c['rows'][i]['rgb_left']['time_ns'])/1e9}
    for name in ['covariance','tracking','pair_end','information']:record['joint_'+name+'_s']=max(parts[cam][name] for cam in parts) if all(parts[cam][name] is not None for cam in parts) else None
    if arm=='on':assert record['joint_covariance_s']<=record['joint_tracking_s']<=record['joint_pair_end_s']<=record['old_decision_prefix_s']
    timing.append(record)
  caseproof.append({'case':c['case'],'arrays_bitwise_equal':arrays,'native_events_and_original_decisions_equal':True,'experts':expert});equal+=arrays
 gates={};summaries={}
 for domain in ['td3','ms2']:
  rows=[r for r in timing if r['domain']==domain and r['arm']=='on'];summaries[domain]={k:quantiles([r[k] for r in rows if r[k] is not None]) for k in ['joint_covariance_s','joint_tracking_s','joint_pair_end_s','joint_information_s','old_decision_prefix_s']};summaries[domain]['matched_case_details']={c['case']:{arm:{k:quantiles([r[k] for r in timing if r['case']==c['case'] and r['arm']==arm and r[k] is not None]) for k in ['joint_covariance_s','joint_tracking_s','joint_pair_end_s','old_decision_prefix_s']} for arm in ['off','on']} for c in plan['fresh_cases'] if c['domain']==domain};gates[domain]=summaries[domain]['joint_tracking_s']['p95']<.5
 availability=read(OUT/'availability-scores.json');passed=availability['primary_availability_noninferiority_passed'] and all(gates.values())
 verify();write(OUT/'preservation.json',{'status':'PASS_ACTIVE_NATIVE_PRESERVATION','arrays_bitwise_equal':equal,'strict_all_arrays_exact':not drift,'diagnostic_drift':drift,'cases':caseproof,'early_events':earlyevents});write(OUT/'fresh-stage-timings.json',{'rows':timing,'summaries':summaries,'fresh_instrumented_readiness_gate':gates,'pose_error_labels_read':False,'live_deadline_verified':False});write(OUT/'primary-verdict.json',{'availability_noninferiority_passed':availability['primary_availability_noninferiority_passed'],'fresh_instrumented_readiness_gate':gates,'primary_combined_passed':bool(passed),'policy_adopted':False,'sensor_skipping_tested':False});print('PRESERVED',equal,'arrays;',len(drift),'diagnostic drift;',earlyevents,'early events; gate',gates,'combined',bool(passed))
if __name__=='__main__':run()
