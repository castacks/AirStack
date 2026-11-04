"""Independent raw-journal availability sets, receipt clocks and semantic mutations."""
import copy
from decision_headroom_study import ROOT,OUT,PLAN,read,write,sha,verify,journal


def validate(data,plan):
 checks=0
 def check(ok,label):
  nonlocal checks
  if not ok:raise AssertionError(label)
  checks+=1
 check(data['pose_references_read'] is False and data['new_native_execution'] is False and
       data['new_policy'] is False and data['prior_research_result_changed'] is False and data['retrospective'] is True,'claims')
 check(len(data['rows'])==plan['expected_rows'],'all rows retained')
 check(set(data['groups'])=={x['condition']+'/'+x['arm'] for x in plan['cohorts']},'cohort set')
 for cohort in plan['cohorts']:
  key=cohort['condition']+'/'+cohort['arm'];base=ROOT/cohort['base'];log=journal(base/'timeline.jsonl')
  rows={r['pair']:r for r in data['rows'] if r['cohort']==key};check(set(rows)==set(range(2,97)),'source population')
  w=cohort['window']['rows'];pred={e['pair']:e for e in log if e['kind']=='preselection'}
  boundary={e['source']:e['controller_monotonic_ns'] for e in log if e['kind']=='group_complete'}
  health={};motion={};releases={}
  for e in log:
   if e['kind'] in ['release','skip_release','transport_drop','catchup_release']:
    releases.setdefault((e['camera'],e['source']),[]).append(e)
   if e['kind']!='observer_event':continue
   ev=e['event']
   if ev['kind']=='pair_end':
    ident=(e['camera'],ev['time1_ns']);check(ident not in health,'unique health');health[ident]=e
   elif ev['kind']=='motion_result_written' and ev['first_write_for_submission']:
    ident=(e['camera'],ev['time1_ns']);check(ident not in motion,'unique first motion');motion[ident]=e
  R=set();T=set();S=set();loss_abstain=set();loss_wrong=set();known={c:set() for c in ['rgb','thermal']}
  prior={r['pair']:r for r in read(ROOT/cohort['scores'])['rows'] if r['arm']==cohort['arm']}
  for pair,r in rows.items():
   check(r['condition']==cohort['condition'] and r['arm']==cohort['arm'] and r['action']==pred[pair]['action'],'cohort/action')
   check(r['prior_views']==pred[pair]['views'],'original prior evidence')
   D=pred[pair]['decision_ns'];B=boundary[pair+2];check(D<B,'clock order')
   available={}
   for c,aset in [('rgb',R),('thermal',T)]:
    interval=[w[i][c+'_left']['time_ns'] for i in [pair,pair+1]];ident=c,interval[1]
    h=health.get(ident);m=motion.get(ident)
    history=releases[c,pair+1];old=[e for e in history if e['kind']=='catchup_release'];rel=old[0] if old else history[0]
    structural=bool(h and m and rel['policy_requested'] and rel['delivered'])
    if structural:
     hv,mv=h['event'],m['event']
     structural=(hv['time0_ns']==mv['time0_ns']==interval[0] and hv['time1_ns']==mv['time1_ns']==interval[1]
       and hv['stage']==mv['stage']=='pair_processing' and hv['execution_error'] is None
       and hv['original_need_interp'] is False and mv['original_need_interp'] is False
       and mv['derived_camera_delta_xyzw'] is not None)
    ready=max(rel['controller_monotonic_ns'],h['controller_monotonic_ns'],m['controller_monotonic_ns']) if structural else None
    at_deadline=ready is not None and ready<=B;at_decision=ready is not None and ready<=D
    if at_deadline:aset.add(pair)
    if at_decision:known[c].add(pair)
    available[c]=at_deadline;expert=r['experts'][c]
    expected={'camera':c,'native_interval_ns':interval,'decision_ns':D,'deadline_ns':B,
       'release_kind':rel['kind'],'release_receipt_ns':rel['controller_monotonic_ns'],
       'health_receipt_ns':h['controller_monotonic_ns'] if h else None,'motion_receipt_ns':m['controller_monotonic_ns'] if m else None,
       'health_event_sequence':h['event']['sequence'] if h else None,'motion_event_sequence':m['event']['sequence'] if m else None,
       'health_known_at_decision':h is not None and h['controller_monotonic_ns']<=D,
       'motion_known_at_decision':m is not None and m['controller_monotonic_ns']<=D,
       'eligible_at_decision':at_decision,'eligible_at_deadline':at_deadline,
       'motion_lag_after_decision_ns':m['controller_monotonic_ns']-D if m else None,
       'motion_invocation_source':m['trigger_source'] if m else None,'motion_delivery_group':m['delivery_group'] if m else None}
    for field,value in expected.items():check(expert[field]==value,field)
    check(at_deadline==prior[pair]['experts'][c]['native_eligible'],'prior eligibility')
   action=r['action'];selected=action!='ABSTAIN' and available[action.lower()]
   if selected:S.add(pair)
   union=any(available.values())
   mask='both' if all(available.values()) else 'rgb_only' if available['rgb'] else 'thermal_only' if available['thermal'] else 'none'
   check(r['mask']==mask and r['selected_eligible']==selected,'mask/selection')
   if selected:category='delivered'
   elif not union:category='no_eligible_output'
   elif action=='ABSTAIN':category='abstain_with_opportunity';loss_abstain.add(pair)
   else:category='chosen_unavailable_with_alternate';loss_wrong.add(pair)
   check(r['loss_category']==category,'selection-loss accounting')
   check(selected==prior[pair]['selected_native_eligible'] and union==prior[pair]['opportunity'],'unchanged prior outcome')
  U=R|T;both=R&T;onlyR=R-T;onlyT=T-R;empty=set(rows)-U;g=data['groups'][key]
  expected={'slots':95,'mask_counts':{'none':len(empty),'rgb_only':len(onlyR),'thermal_only':len(onlyT),'both':len(both)},
    'selected_eligible':len(S),'rgb_constant_readout':len(R),'thermal_constant_readout':len(T),'conditional_union_ceiling':len(U),
    'ceiling_above_best_constant':len(U)-max(len(R),len(T)),'gate_minus_best_constant':len(S)-max(len(R),len(T)),
    'union_minus_gate':len(U-S),'abstain_with_opportunity':len(loss_abstain),'chosen_unavailable_with_alternate':len(loss_wrong),
    'selected_unavailable':sum(r['action']!='ABSTAIN' and not r['selected_eligible'] for r in rows.values()),
    'abstain_none':sum(r['action']=='ABSTAIN' and r['mask']=='none' for r in rows.values()),
    'current_target_eligible_at_decision':{c:len(v) for c,v in known.items()},'conditional_readouts_share_exact_cost':True}
  for field,value in expected.items():check(g[field]==value,field)
  check(len(U-S)==len(loss_abstain)+len(loss_wrong) and not(loss_abstain&loss_wrong),'disjoint lost-output partition')
  check(len(U)-max(len(R),len(T))==min(len(onlyR),len(onlyT)),'complementarity identity')
  check(95==len(S)+len(U-S)+len(empty),'complete population identity')
  calls=ns=0
  for c in ['rgb','thermal']:
   cost=read(base/'experts'/c/'native-cost.json');calls+=len([v for v in cost['frontend_calls'] if v['kind']=='frontend_pair'])
   ns+=sum(v['end_ns']-v['start_ns'] for v in cost['native_phases'] if v['kind']=='run')
   check(len([v for v in cost['frontend_calls'] if v['kind']=='frontend_pair'])==len(cost['processed_source_indices'])-1,'native source call count')
  start=next(v['controller_monotonic_ns'] for v in log if v['kind']=='release');end=max(boundary.values())
  check(g['cost']=={'pair_calls':calls,'native_run_ns':ns,'controller_processing_ns':end-start},'integer cost ledger')
 return checks


def audit():
 plan=verify();lock=read(OUT/'analysis-lock.json')
 assert lock['plan_sha256']==sha(PLAN) and lock['accounting_sha256']==sha(OUT/'accounting.json')
 assert lock['source_sha256']==sha(ROOT/'tools/decision_headroom_study.py')
 data=read(OUT/'accounting.json');checks=validate(data,plan)
 mutations={
  'future_output_marked_known':lambda d:d['rows'][0]['experts']['rgb'].__setitem__('eligible_at_decision',True),
  'fabricated_union':lambda d:d['groups']['day/leased_probe'].__setitem__('conditional_union_ceiling',96),
  'wrong_camera_mask':lambda d:d['rows'][0].__setitem__('mask','fabricated'),
  'lost_output_hidden':lambda d:d['rows'][0].__setitem__('loss_category','hidden'),
  'dropped_hard_slot':lambda d:d['rows'].pop(),
  'fabricated_cost':lambda d:d['groups']['night/leased_probe']['cost'].__setitem__('pair_calls',0),
  'pose_quality_claim':lambda d:d.__setitem__('pose_references_read',True),
  'new_policy_claim':lambda d:d.__setitem__('new_policy',True)}
 proof=[]
 for name,change in mutations.items():
  mutant=copy.deepcopy(data);change(mutant)
  try:validate(mutant,plan)
  except AssertionError as e:proof.append({'mutation':name,'rejected':True,'reason':str(e)})
  else:raise AssertionError('semantic mutation accepted: '+name)
 write(OUT/'audit.json',{'pass':True,'checks':checks,'rows':len(data['rows']),'cohorts':len(data['groups']),
   'direct_pins':len(plan['input_sha256']),'inherited_pins':len(plan['inherited_sha256']),
   'semantic_mutations':proof,'accounting_sha256':sha(OUT/'accounting.json'),'new_native_execution':False})
 print('PASS',checks,'checks;',len(proof),'semantic mutations rejected')

if __name__=='__main__':audit()
