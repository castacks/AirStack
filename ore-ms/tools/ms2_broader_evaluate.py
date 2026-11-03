"""Broader MS² preservation, label-free fixed actions, qualified reference scores."""
from pathlib import Path
import argparse,json,sys
from collections import Counter
import numpy as np
from ms2_broader_execution import ROOT,OUT,EXEC,read,write,sha,verify
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
    assert reports0[0]['error']=='No visual matches were retained' and reports0[0]['completed_frames']==c['frames'] and not gi and not firstwrites
   native.append({'camera':camera,'native_status':reports0[0]['status'],'tracking':reports0[0]['tracking_diagnostics'],'geometry_events':len(gi)});geometries+=len(gi)
  timelines=[[json.loads(s) for s in (base/arm/'timeline.jsonl').read_text().splitlines()] for arm in ['off','on']]
  def decisions(entries):return [(e['pair_ordinal'],e['action'],e['availability'],e['boundary']) for e in entries if e['kind']=='decision']
  assert decisions(timelines[0])==decisions(timelines[1]) and len(decisions(timelines[0]))==c['frames']-1
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
  prefixes={'rows':[dict(r,window=c['window']) for r in predict_prefix(c['rows'],timeline,c['nominal_body_from_camera'])]};assert len(prefixes['rows'])==c['frames']-1
  inputs,proof=prepare(prefixes,{c['window']:{'nominal_body_from_camera':c['nominal_body_from_camera']}},timeline,c['recording_id'],c['window'])
  base_rows=predict_base(inputs)['rows']
  for r,p in zip(base_rows,prefixes['rows']):
   assert r['pair_ordinal']==p['pair_ordinal'];allrows.append(augment(r,health_features(p,timeline)))
  allprefix+=prefixes['rows'];contracts+=inputs['rows'];proofs+=proof
 assert len(allrows)==plan['expected_decisions'] and len({r['sample_id'] for r in allrows})==plan['expected_decisions']
 write(OUT/'prefix-predictions.json',{'rows':allprefix});write(OUT/'admission-inputs.json',{'rows':contracts});write(OUT/'geometry-proof.json',{'bindings':proofs});write(OUT/'locked-actions.json',{'rows':allrows,'labels_read':False})
 verify();write(OUT/'prediction-lock.json',{'status':'PASS_LABEL_FREE_FIXED_CONTROLS','actions_sha256':sha(OUT/'locked-actions.json'),'prefix_sha256':sha(OUT/'prefix-predictions.json'),'inputs_sha256':sha(OUT/'admission-inputs.json'),'both_supported_ids':[r['sample_id'] for r in allrows if r['both_supported']],'labels_read':False})
 print('LOCKED',len(allrows),'decisions;',sum(r['both_supported'] for r in allrows),'both-supported')

def reference_pose(path):
 a=np.loadtxt(path).reshape(4,4)
 if not np.isfinite(a).all() or not np.array_equal(a[3],[0,0,0,1]):return None,{'valid':False,'reason':'nonfinite_or_homogeneous_row'}
 r=a[:3,:3];orth=float(np.linalg.norm(r.T@r-np.eye(3)));det=float(np.linalg.det(r))
 if orth>1e-5 or abs(det-1)>1e-5:return None,{'valid':False,'reason':'nonrigid','orthogonality':orth,'determinant':det}
 u,_,vt=np.linalg.svd(r);proper=u@np.diag([1,1,np.linalg.det(u@vt)])@vt;correction=float(np.max(abs(proper-r)));a[:3,:3]=proper
 return a,{'valid':True,'orthogonality':orth,'determinant':det,'rotation_max_correction':correction}

def score():
 from Scripts.Setup.evaluate_interval_model import loss
 from Scripts.Setup.geometry_support_controls import evaluate
 plan=verify();lock=read(OUT/'prediction-lock.json');assert lock['actions_sha256']==sha(OUT/'locked-actions.json') and lock['prefix_sha256']==sha(OUT/'prefix-predictions.json')
 prefix=read(OUT/'prefix-predictions.json')['rows'];rows=read(OUT/'locked-actions.json')['rows'];labels={};inverse_labels={};reference_rows=[];invalid=[];poses={}
 for c in plan['cases']:
  for stem in c['stems']:
   path=OUT/'reference'/c['sequence']/'rgb'/(stem+'.txt');pose,report=reference_pose(path);poses[(c['case'],stem)]=(pose,report)
 cases={c['case']:c for c in plan['cases']}
 for r in prefix:
  c=cases[r['window']];i=r['pair_ordinal'];a,ra=poses[(c['case'],c['stems'][i])];b,rb=poses[(c['case'],c['stems'][i+1])];sid=f"{r['window']}/{i}"
  assert r['common_interval_ns']==[c['rows'][j]['rgb_left']['time_ns'] for j in [i,i+1]]
  if a is None or b is None:invalid.append({'sample_id':sid,'endpoint_reports':[ra,rb]});continue
  delta=np.linalg.solve(a,b);inverse_delta=a@np.linalg.inv(b)
  labels[sid]={camera:loss(e['delta_model'],delta) if e['valid_model_projection'] else None for camera,e in r['experts'].items()}
  inverse_labels[sid]={camera:loss(e['delta_model'],inverse_delta) if e['valid_model_projection'] else None for camera,e in r['experts'].items()}
  reference_rows.append({'sample_id':sid,'delta':delta.tolist(),'inverse_convention_delta':inverse_delta.tolist(),'endpoint_reports':[ra,rb]})
 eligible=[r for r in rows if r['sample_id'] in labels];assert len(eligible)+len(invalid)==891
 def report(selected,labs):return evaluate(selected,{r['sample_id']:labs[r['sample_id']] for r in selected})
 result=report(eligible,labels);result['windows']={c['case']:report([r for r in eligible if r['window']==c['window']],labels) for c in plan['cases']}
 result['recordings']={rec:report([r for r in eligible if cases[r['window']]['recording']==rec],labels) for rec in ['day','night','rain']}
 result['native_support_by_window']={c['case']:dict(Counter('both' if r['both_supported'] else 'rgb' if r['native_available']['rgb'] else 'thermal' if r['native_available']['thermal'] else 'neither' for r in rows if r['window']==c['window'])) for c in plan['cases']}
 result['translation_winner_balance_by_window']={c['case']:dict(Counter('rgb' if labels[r['sample_id']]['rgb']['translation_m']<labels[r['sample_id']]['thermal']['translation_m'] else 'thermal' if labels[r['sample_id']]['rgb']['translation_m']>labels[r['sample_id']]['thermal']['translation_m'] else 'tie' for r in eligible if r['window']==c['window'] and r['both_supported'])) for c in plan['cases']}
 result['reference_coverage']={'native_decisions':len(rows),'scoreable_reference_decisions':len(eligible),'excluded_reference_decisions':len(invalid),'excluded':invalid}
 result['inverse_pose_convention_sensitivity']={'selected_for_primary':False,'result':report(eligible,inverse_labels)}
 write(OUT/'fresh-labels.json',labels);write(OUT/'inverse-convention-labels.json',inverse_labels);write(OUT/'references.json',{'rows':reference_rows,'poses':[{'window':w,'stem':s,**r} for (w,s),(_,r) in poses.items()]});write(OUT/'scores.json',result)
 verify();print(json.dumps({'primary_pass':result['primary_criterion_passed'],'reference_coverage':{k:v for k,v in result['reference_coverage'].items() if k!='excluded'},'native_support':result['native_support_by_window'],'translation_winner_balance':result['translation_winner_balance_by_window'],'recordings':{k:{'all_population':v['groups']['all']['population'],'both_population':v['groups']['both_supported']['population'],'methods':{m:{a:b for a,b in vv.items() if a!='ids'} for m,vv in v['groups']['both_supported']['methods'].items()}} for k,v in result['recordings'].items()}},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['parity','predict','score']);a=p.parse_args();globals()[a.action]()
