"""EXP126 retrospective causal-timing and conditional availability accounting."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/decision-headroom/run-001'
PLAN=ROOT/'experiments/decision-headroom-2026-11-03.json'
COHORTS={'day':['full_read_cache','bounded_probe','leased_probe'],
         'night':['full_read_cache','bounded_probe','leased_probe','fixed_rgb','fixed_thermal']}
PRIOR={'day':'leased-handover','night':'night-transfer'}
CAMERAS=['rgb','thermal']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def journal(p):return [json.loads(l) for l in Path(p).read_text().splitlines()]


def freeze():
 assert not PLAN.exists() and not OUT.exists()
 pins={};cohorts=[];inherited={}
 for condition,arms in COHORTS.items():
  old=ROOT/'results'/PRIOR[condition]/'run-001';parent=read(old/'plan.json')
  for n,h in parent['input_source_sha256'].items():assert sha(ROOT/n)==h,n;inherited[n]=h
  for p in [old/'plan.json',old/'scores.json',old/'capture-lock.json',old/'prediction-lock.json',old/'audit.json']:
   pins[str(p.relative_to(ROOT))]=sha(p)
  if condition=='night':
   p=old/'audit-amendment.json';pins[str(p.relative_to(ROOT))]=sha(p);a=read(p)
   assert sha(ROOT/a['corrected_audit_source'])==a['corrected_sha256']
   pins[a['corrected_audit_source']]=a['corrected_sha256']
  assert read(old/'capture-lock.json')['plan_sha256']==sha(old/'plan.json')
  for path,digest in read(old/'capture-lock.json')['sha256'].items():assert sha(ROOT/path)==digest,path
  assert read(old/'prediction-lock.json')['predictions_sha256']==sha(old/'predictions.json')
  for arm in arms:
   base=old/'captures'/arm
   for p in [base/'timeline.jsonl',base/'summary.json',*(base/'experts').glob('*/native-cost.json')]:pins[str(p.relative_to(ROOT))]=sha(p)
   cohorts.append({'condition':condition,'arm':arm,'base':str(base.relative_to(ROOT)),
                   'scores':str((old/'scores.json').relative_to(ROOT)),'window':parent['window']})
 for n in ['tools/decision_headroom_study.py','tools/decision_headroom_audit.py',
           'MAC-VO-main/Scripts/UnitTest/test_decision_headroom.py',
           'docs/research/decision-headroom-protocol-2026-11-03.md']:
  pins[n]=sha(ROOT/n)
 plan={'date':'2026-11-03','parent_commit':'9d778e7d14e3d328df43ec3442d663bff7bce537',
       'cohorts':cohorts,'expected_rows':760,'input_sha256':pins,'inherited_sha256':inherited,
       'retrospective':True,'new_native_execution':False,'new_policy':False,
       'pose_references_read':False,'training':False,'reserve_access':False,'policy_adopted':False}
 write(PLAN,plan);write(OUT/'plan.json',plan)
 print('FROZEN',len(cohorts),'cohorts;',len(pins),'direct pins;',len(inherited),'inherited pins')


def verify():
 plan=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for mapping in [plan['input_sha256'],plan['inherited_sha256']]:
  for path,digest in mapping.items():assert sha(ROOT/path)==digest,path
 return plan


def observe(rows,log,pair,camera,decision,deadline):
 times=[rows[i][camera+'_left']['time_ns'] for i in [pair,pair+1]]
 health=[e for e in log if e['kind']=='observer_event' and e['camera']==camera and
         e['event']['kind']=='pair_end' and e['event']['time1_ns']==times[1]]
 motion=[e for e in log if e['kind']=='observer_event' and e['camera']==camera and
         e['event']['kind']=='motion_result_written' and e['event']['time1_ns']==times[1] and e['event']['first_write_for_submission']]
 assert len(health)<=1 and len(motion)<=1
 h=health[0] if health else None;m=motion[0] if motion else None
 ordinary=next(e for e in log if e['kind'] in ['release','skip_release','transport_drop'] and e['camera']==camera and e['source']==pair+1)
 historical=[e for e in log if e['kind']=='catchup_release' and e['camera']==camera and e['source']==pair+1]
 rel=historical[0] if historical else ordinary
 def eligible(clock):
  return bool(rel['policy_requested'] and rel['delivered'] and rel['controller_monotonic_ns']<=clock and h and m and
    h['event']['stage']=='pair_processing' and [h['event']['time0_ns'],h['event']['time1_ns']]==times and
    h['event']['execution_error'] is None and h['event']['original_need_interp'] is False and
    m['event']['stage']=='pair_processing' and [m['event']['time0_ns'],m['event']['time1_ns']]==times and
    m['event']['original_need_interp'] is False and m['event']['derived_camera_delta_xyzw'] is not None and
    h['controller_monotonic_ns']<=clock and m['controller_monotonic_ns']<=clock)
 return {'camera':camera,'native_interval_ns':times,'decision_ns':decision,'deadline_ns':deadline,
   'release_kind':rel['kind'],'release_receipt_ns':rel['controller_monotonic_ns'],
   'health_receipt_ns':h['controller_monotonic_ns'] if h else None,
   'motion_receipt_ns':m['controller_monotonic_ns'] if m else None,
   'health_event_sequence':h['event']['sequence'] if h else None,'motion_event_sequence':m['event']['sequence'] if m else None,
   'health_known_at_decision':bool(h and h['controller_monotonic_ns']<=decision),
   'motion_known_at_decision':bool(m and m['controller_monotonic_ns']<=decision),
   'eligible_at_decision':eligible(decision),'eligible_at_deadline':eligible(deadline),
   'motion_lag_after_decision_ns':m['controller_monotonic_ns']-decision if m else None,
   'motion_invocation_source':m['trigger_source'] if m else None,'motion_delivery_group':m['delivery_group'] if m else None}


def summarize(rows,cost):
 masks=collections.Counter(r['mask'] for r in rows);n=len(rows)
 R=masks['rgb_only']+masks['both'];T=masks['thermal_only']+masks['both'];U=n-masks['none']
 S=sum(r['selected_eligible'] for r in rows)
 abstain=sum(r['loss_category']=='abstain_with_opportunity' for r in rows)
 wrong=sum(r['loss_category']=='chosen_unavailable_with_alternate' for r in rows)
 assert U-S==abstain+wrong and U-max(R,T)==min(masks['rgb_only'],masks['thermal_only'])
 return {'slots':n,'mask_counts':{k:masks[k] for k in ['none','rgb_only','thermal_only','both']},
   'selected_eligible':S,'rgb_constant_readout':R,'thermal_constant_readout':T,'conditional_union_ceiling':U,
   'ceiling_above_best_constant':U-max(R,T),'gate_minus_best_constant':S-max(R,T),
   'union_minus_gate':U-S,'abstain_with_opportunity':abstain,'chosen_unavailable_with_alternate':wrong,
   'selected_unavailable':sum(r['action']!='ABSTAIN' and not r['selected_eligible'] for r in rows),
   'abstain_none':sum(r['action']=='ABSTAIN' and r['mask']=='none' for r in rows),
   'current_target_eligible_at_decision':{c:sum(r['experts'][c]['eligible_at_decision'] for r in rows) for c in CAMERAS},
   'cost':cost,'conditional_readouts_share_exact_cost':True}


def analyze():
 plan=verify();ledger=[];groups={}
 for cohort in plan['cohorts']:
  condition,arm=cohort['condition'],cohort['arm'];key=condition+'/'+arm;w=cohort['window']
  base=ROOT/cohort['base'];log=journal(base/'timeline.jsonl');prior=read(ROOT/cohort['scores'])
  priorrows={r['pair']:r for r in prior['rows'] if r['arm']==arm};rr=[]
  for pair in range(2,97):
   d=next(e for e in log if e['kind']=='preselection' and e['pair']==pair)
   deadline=next(e['controller_monotonic_ns'] for e in log if e['kind']=='group_complete' and e['source']==pair+2)
   experts={c:observe(w['rows'],log,pair,c,d['decision_ns'],deadline) for c in CAMERAS}
   available=[c for c in CAMERAS if experts[c]['eligible_at_deadline']]
   mask='both' if len(available)==2 else available[0]+'_only' if available else 'none'
   selected=d['action']!='ABSTAIN' and experts[d['action'].lower()]['eligible_at_deadline']
   category='delivered' if selected else 'no_eligible_output' if not available else 'abstain_with_opportunity' if d['action']=='ABSTAIN' else 'chosen_unavailable_with_alternate'
   for c in CAMERAS:assert experts[c]['eligible_at_deadline']==priorrows[pair]['experts'][c]['native_eligible'],(key,pair,c)
   assert selected==priorrows[pair]['selected_native_eligible'] and bool(available)==priorrows[pair]['opportunity']
   row={'cohort':key,'condition':condition,'arm':arm,'pair':pair,'action':d['action'],
      'decision_reason':d.get('reason',d.get('base_decision',{}).get('reason')),'prior_views':d['views'],
      'experts':experts,'mask':mask,'selected_eligible':selected,'loss_category':category}
   ledger.append(row);rr.append(row)
  calls=ns=0
  for c in CAMERAS:
   cost=read(base/'experts'/c/'native-cost.json');calls+=sum(e['kind']=='frontend_pair' for e in cost['frontend_calls'])
   ns+=sum(e['duration_ns'] for e in cost['native_phases'] if e['kind']=='run')
  first=next(e['controller_monotonic_ns'] for e in log if e['kind']=='release');last=max(e['controller_monotonic_ns'] for e in log if e['kind']=='group_complete')
  groups[key]=summarize(rr,{'pair_calls':calls,'native_run_ns':ns,'controller_processing_ns':last-first})
 assert len(ledger)==plan['expected_rows']
 write(OUT/'accounting.json',{'groups':groups,'rows':ledger,'retrospective':True,'pose_references_read':False,
   'new_native_execution':False,'new_policy':False,'prior_research_result_changed':False,
   'scope':'conditional availability ceiling on retained executed histories; no accuracy or different-scheduler bound'})
 write(OUT/'analysis-lock.json',{'accounting_sha256':sha(OUT/'accounting.json'),'plan_sha256':sha(PLAN),
                                'source_sha256':sha(__file__)})
 for key,g in groups.items():print(key,'selected',g['selected_eligible'],'union',g['conditional_union_ceiling'],
   'R/T',g['rgb_constant_readout'],g['thermal_constant_readout'],'headroom',g['ceiling_above_best_constant'],'lost',g['union_minus_gate'])

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','verify','analyze']);a=p.parse_args()
 if a.action=='verify':print('VERIFIED',len(verify()['input_sha256']),'direct pins')
 else:globals()[a.action]()
