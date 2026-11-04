"""EXP125 label-free native transfer and conditional exact-cost camera readouts."""
import argparse
import collections
import numpy as np
from night_transfer_study import ROOT,OUT,PLAN,read,write,sha,verify
from buffered_handover_evaluate import replay,camera_output,latencies
from closed_loop_sparse_evaluate import costs


def byte_controls(current,old):
 count=0
 current_files={p.relative_to(current):p for p in current.rglob('*.npz')}
 old_files={p.relative_to(old):p for p in old.rglob('*.npz')}
 assert current_files.keys()==old_files.keys(),'array file set differs'
 for relative,p in current_files.items():
  with np.load(p) as a,np.load(old_files[relative]) as b:
   assert set(a.files)==set(b.files),(relative,'array keys')
   for key in a.files:
    x,y=a[key],b[key]
    assert x.dtype==y.dtype and x.shape==y.shape and x.tobytes()==y.tobytes(),(relative,key)
    count+=1
 return {'arrays':count,'all_logical_cell_bytes_exact':True}


def predict():
 plan=verify();w=plan['window'];lock=read(OUT/'capture-lock.json')
 assert lock['plan_sha256']==sha(PLAN)
 for mapping in [lock['sha256'],plan['prior_control_sha256']]:
  for path,digest in mapping.items():assert sha(ROOT/path)==digest,path
 arms={};rows=[];checks=0
 for arm in ['unperturbed']+plan['arms']:
  base=OUT/'captures'/arm;log,decisions,n=replay(w,base);checks+=n;stats=costs(base)
  first=next(e['controller_monotonic_ns'] for e in log if e['kind']=='release')
  last=max(e['controller_monotonic_ns'] for e in log if e['kind']=='group_complete')
  stats['controller_processing_seconds']=(last-first)/1e9
  meta=read(base/'summary.json');result={'cost':stats,'service':read(base/'lease-service.json'),
    'readiness_extra_used':meta['readiness_extra_used'],'readiness_sessions':meta['readiness_sessions'],
    'catchup_queries':meta['catchup_queries'],'retention':read(base/'retained-inputs.json')}
  if arm!='unperturbed':
   rr=[]
   for pair in range(2,w['frames']-3):
    d=decisions[pair];experts={c:camera_output(w,log,pair,c) for c in ['rgb','thermal']};selected=experts.get(d['action'].lower())
    row={'arm':arm,'pair':pair,'action':d['action'],'requests':d['requests'],'decision_ns':d['decision_ns'],
      'experts':experts,'selected_native_eligible':bool(selected and selected['native_eligible']),
      'opportunity':any(e['native_eligible'] for e in experts.values())}
    rows.append(row);rr.append(row)
   result.update(slots=len(rr),eligible=sum(r['selected_native_eligible'] for r in rr),
      unavailable_selected=sum(r['action']!='ABSTAIN' and not r['selected_native_eligible'] for r in rr),
      uncommitted_while_opportunity=sum(r['action']=='ABSTAIN' and r['opportunity'] for r in rr),
      uncommitted_none_available=sum(r['action']=='ABSTAIN' and not r['opportunity'] for r in rr),
      transition_latency=latencies(plan,arm,log,decisions,rr))
  arms[arm]=result
 controls={'unperturbed':byte_controls(OUT/'captures/unperturbed/experts',ROOT/plan['prior_unperturbed'])}
 write(OUT/'predictions.json',{'arms':arms,'rows':rows,'controls':controls,'prefix_checks':checks,
       'pose_references_read':False,'capture_lock_sha256':sha(OUT/'capture-lock.json')})
 write(OUT/'prediction-lock.json',{'predictions_sha256':sha(OUT/'predictions.json'),'source_sha256':sha(__file__),'pose_references_read':False})
 print('LOCKED',len(rows),'core rows;',checks,'prefix checks;',sum(x['arrays'] for x in controls.values()),'byte-exact arrays')


def score():
 import copy
 plan=verify();lock=read(OUT/'prediction-lock.json');assert lock['predictions_sha256']==sha(OUT/'predictions.json') and lock['source_sha256']==sha(__file__)
 pred=read(OUT/'predictions.json');groups=pred['arms'];candidate=groups['leased_probe'];base=groups['bounded_probe'];full=groups['full_read_cache']
 matched={};matched_rows=[]
 for camera in ['rgb','thermal']:
  arm='matched_'+camera;rr=[]
  for r in pred['rows']:
   if r['arm']!='leased_probe':continue
   row={'arm':arm,'pair':r['pair'],'action':camera.upper(),
        'selected_native_eligible':r['experts'][camera]['native_eligible'],
        'reason':r['experts'][camera]['reason'],'candidate_source_arm':'leased_probe'}
   rr.append(row);matched_rows.append(row)
  eligible=sum(r['selected_native_eligible'] for r in rr)
  matched[arm]={'slots':len(rr),'eligible':eligible,'unavailable_selected':len(rr)-eligible,
      'cost':copy.deepcopy(candidate['cost']),'conditioning':'candidate exact native input history/deadlines; fixed choice only; not standalone scheduler',
      'independent_native_run':False}
 execution={'strict_byte_control':all(x['all_logical_cell_bytes_exact'] for x in pred['controls'].values()),
  'resource_reports_bounded':all(g['retention']['peak_retained_bytes']<=g['service']['store_capacity_bytes'] and
      g['service']['arena']['peak_leased_bytes']<=plan['arena_bytes'] and g['service']['arena']['active'] is None and
      g['service']['arena']['leases']==g['service']['arena']['acks'] and
      all(p['max_sent_bytes']<=65536 and p['max_received_bytes']<=65536 and p['max_step_bytes']<=4096 for p in g['service']['control_channels'].values()) for g in groups.values()),
  'readiness_caps':all(g['readiness_extra_used']<=20 and all(s['attempts']<=4 for s in g['readiness_sessions']) for g in groups.values()),
  'query_caps':all(g['catchup_queries']<=2 for g in groups.values()),
  'exact_matched_cost':all(m['cost']==candidate['cost'] for m in matched.values())}
 research={'eligible_noninferior_full':candidate['eligible']>=full['eligible'],
  'eligible_noninferior_original':candidate['eligible']>=base['eligible'],
  'fewer_calls_than_full':candidate['cost']['total_pair_calls']<full['cost']['total_pair_calls'],
  'strictly_better_both_native_fixed':candidate['eligible']>max(groups[a]['eligible'] for a in ['fixed_rgb','fixed_thermal']),
  'strictly_better_both_matched_readouts':candidate['eligible']>max(m['eligible'] for m in matched.values())}
 write(OUT/'scores.json',{'primary_criteria':research,'primary_contract_passed':all(research.values()),
  'execution_criteria':execution,'execution_contract_passed':all(execution.values()),
  'groups':groups,'rows':pred['rows'],'matched_readouts':matched,'matched_rows':matched_rows,
  'controls':pred['controls'],'prefix_checks':pred['prefix_checks'],'prediction_lock_sha256':sha(OUT/'prediction-lock.json'),
  'pose_references_read':False,'policy_adopted':False,'scope':'previously inspected night recording; synthetic serial condition transfer; no pose labels'})
 print('EXECUTION',all(execution.values()),execution);print('RESEARCH',all(research.values()),research)
 for arm,g in groups.items():print(arm,g.get('eligible'),g['cost']['total_pair_calls'],g['cost']['total_run_seconds'])
 for arm,g in matched.items():print(arm,g['eligible'],g['cost']['total_pair_calls'],'conditional readout')

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['predict','score']);a=p.parse_args();globals()[a.action]()
