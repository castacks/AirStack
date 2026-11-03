"""Matched preservation, label-free fixed controls, separate fresh proxy scoring."""
from pathlib import Path
import argparse,json,sys
import numpy as np
from geometry_support_transfer import ROOT,OUT,PLAN,read,write,sha,verify
sys.path.insert(0,str(ROOT/'MAC-VO-main'))

def parity():
 plan=verify();reports=[];drift=[];equal=0
 for c in plan['cases']:
  base=OUT/'captures'/c['case'];a=base/'off';b=base/'on';arrays=0;geometries=0;native=[]
  first={str(p.relative_to(a)):p for p in a.rglob('*.npz')};second={str(p.relative_to(b)):p for p in b.rglob('*.npz')};assert first.keys()==second.keys()
  for rel,p in first.items():
   with np.load(p,allow_pickle=False) as x,np.load(second[rel],allow_pickle=False) as y:
    assert set(x.files)==set(y.files)
    for key in x.files:
     assert x[key].shape==y[key].shape and x[key].dtype==y[key].dtype
     if np.array_equal(x[key],y[key],equal_nan=True):arrays+=1;continue
     allowed=(rel.endswith('tensor-map.npz') and key=='match//obs2_covTc') or ('/uncertainty/pair-' in rel and key in ['endpoint1_depth_variance_before','endpoint1_depth_variance_after'])
     assert allowed,(c['case'],rel,key)
     u=x[key].astype(float);v=y[key].astype(float);assert np.array_equal(np.isfinite(u),np.isfinite(v));finite=np.isfinite(u)&np.isfinite(v);diff=np.abs(u-v)
     drift.append({'case':c['case'],'file':rel,'key':key,'max_absolute':float(np.max(diff[finite])),'max_relative':float(np.max(diff[finite]/np.maximum(np.maximum(abs(u[finite]),abs(v[finite])),1e-30)))})
  for camera in ['rgb','thermal']:
   aa=a/'experts'/camera;bb=b/'experts'/camera
   assert read(aa/'effective-config.json')==read(bb/'effective-config.json')
   reports0=[read(x/'report.json') for x in [aa,bb]];assert reports0[0]['status']==reports0[1]['status'];assert reports0[0]['tracking_diagnostics']==reports0[1]['tracking_diagnostics']
   assert read(aa/'terminal-capture-contract.json')==read(bb/'terminal-capture-contract.json')
   for x in [aa,bb]:assert read(x/'arbiter-events/summary.json')['status']=='COMPLETE'
   ea=[json.loads(s) for s in (aa/'arbiter-events/events.jsonl').read_text().splitlines()];eb=[json.loads(s) for s in (bb/'arbiter-events/events.jsonl').read_text().splitlines()]
   def strip(events):return [{k:v for k,v in e.items() if k not in ['sequence','observation_monotonic_ns']} for e in events if e['kind']!='native_disparity_information']
   assert strip(ea)==strip(eb)
   gi=[e for e in eb if e['kind']=='native_disparity_information'];firstwrites=[e for e in eb if e['kind']=='motion_result_written' and e['first_write_for_submission']];assert len(gi)==len(firstwrites)
   for g in gi:
    m,=[m for m in firstwrites if m['submission_id']==g['submission_id']];assert g['sequence']<m['sequence'] and [g['time0_ns'],g['time1_ns']]==[m['time0_ns'],m['time1_ns']]
   if reports0[0]['status']=='FAILED':
    assert reports0[0]['error']=='No visual matches were retained' and reports0[0]['completed_frames']==20 and not gi and not firstwrites
   native.append({'camera':camera,'native_status':reports0[0]['status'],'tracking':reports0[0]['tracking_diagnostics'],'geometry_events':len(gi)});geometries+=len(gi)
  timelines=[[json.loads(s) for s in (base/arm/'timeline.jsonl').read_text().splitlines()] for arm in ['off','on']]
  def decisions(entries):return [(e['pair_ordinal'],e['action'],e['availability'],e['boundary']) for e in entries if e['kind']=='decision']
  assert decisions(timelines[0])==decisions(timelines[1]) and len(decisions(timelines[0]))==19
  reports.append({'case':c['case'],'arrays_bitwise_equal':arrays,'geometry_events':geometries,'experts':native,'original_worker_events_and_raw_decisions_equal':True});equal+=arrays
 verify();write(OUT/'preservation.json',{'status':'PASS_ACTIVE_GRAPH_PRESERVATION','cases':reports,'arrays_bitwise_equal':equal,'strict_all_arrays_exact':not drift,'diagnostic_drift':drift,'native_failures_preserved':True});print('PRESERVED',equal,'arrays;',len(drift),'diagnostic differences')

def predict():
 from Scripts.Setup.prefix_interval_model import predict_prefix
 from Scripts.Setup.prepare_sensor_admission_contract import prepare
 from Scripts.Setup.sensor_admission_comparison import predict as predict_base
 from Scripts.Setup.geometry_support_controls import health_features,augment
 plan=verify();assert read(OUT/'preservation.json')['status']=='PASS_ACTIVE_GRAPH_PRESERVATION';allrows=[];allprefix=[];contracts=[];proofs=[]
 for c in plan['cases']:
  base=OUT/'captures'/c['case']/'on';timeline=[json.loads(s) for s in (base/'timeline.jsonl').read_text().splitlines()]
  prefixes={'rows':[dict(r,window=c['window']) for r in predict_prefix(c['rows'],timeline,c['nominal_body_from_camera'])]};assert len(prefixes['rows'])==19
  inputs,proof=prepare(prefixes,{c['window']:{'nominal_body_from_camera':c['nominal_body_from_camera']}},timeline,c['recording_id'],c['window'])
  base_rows=predict_base(inputs)['rows']
  for r,p in zip(base_rows,prefixes['rows']):
   assert r['pair_ordinal']==p['pair_ordinal'];allrows.append(augment(r,health_features(p,timeline)))
  allprefix+=prefixes['rows'];contracts+=inputs['rows'];proofs+=proof
 assert len(allrows)==57 and len({r['sample_id'] for r in allrows})==57
 write(OUT/'prefix-predictions.json',{'rows':allprefix});write(OUT/'admission-inputs.json',{'rows':contracts});write(OUT/'geometry-proof.json',{'bindings':proofs});write(OUT/'locked-actions.json',{'rows':allrows,'labels_read':False})
 verify();write(OUT/'prediction-lock.json',{'status':'PASS_LABEL_FREE_FIXED_CONTROLS','actions_sha256':sha(OUT/'locked-actions.json'),'prefix_sha256':sha(OUT/'prefix-predictions.json'),'inputs_sha256':sha(OUT/'admission-inputs.json'),'both_supported_ids':[r['sample_id'] for r in allrows if r['both_supported']],'labels_read':False})
 print('LOCKED',57,'decisions;',sum(r['both_supported'] for r in allrows),'both-supported')

def score():
 from Scripts.Setup.td3_reference import load_reference
 from Scripts.Setup.evaluate_interval_model import loss
 from Scripts.Setup.geometry_support_controls import evaluate
 plan=verify();lock=read(OUT/'prediction-lock.json');assert lock['actions_sha256']==sha(OUT/'locked-actions.json') and lock['prefix_sha256']==sha(OUT/'prefix-predictions.json')
 reference,_=load_reference(ROOT/plan['cases'][0]['reference_dataset']);prefix=read(OUT/'prefix-predictions.json')['rows'];rows=read(OUT/'locked-actions.json')['rows'];labels={};reference_rows=[]
 for r in prefix:
  a,b=reference.at(np.asarray(r['common_interval_ns'],np.int64));delta=np.linalg.solve(a,b);sid=f"{r['window']}/{r['pair_ordinal']}"
  labels[sid]={camera:loss(e['delta_model'],delta) if e['valid_model_projection'] else None for camera,e in r['experts'].items()};reference_rows.append({'sample_id':sid,'delta':delta.tolist()})
 write(OUT/'fresh-labels.json',labels);write(OUT/'references.json',{'rows':reference_rows})
 result=evaluate(rows,labels);result['windows']={c['case']:evaluate([r for r in rows if r['window']==c['window']],{r['sample_id']:labels[r['sample_id']] for r in rows if r['window']==c['window']}) for c in plan['cases']}
 verify();write(OUT/'scores.json',result);print(json.dumps({'primary_criterion_passed':result['primary_criterion_passed'],'all':result['groups']['all']['population'],'both_supported':result['groups']['both_supported']['population'],'methods':{m:{k:v for k,v in s.items() if k!='ids'} for m,s in result['groups']['both_supported']['methods'].items()}},indent=2))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['parity','predict','score']);a=p.parse_args();globals()[a.action]()
