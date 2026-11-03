"""Frozen guarded dense-trace replay with hidden unrequested observations."""
from pathlib import Path
import argparse,collections,datetime,hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/guarded-probe-cache/run-002';PLAN=ROOT/'experiments/guarded-probe-cache-2026-11-03-fixture-v2.json'
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.probe_state_cache import ProbeCache,MODES

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,x):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(x,f,indent=2,allow_nan=False);f.write('\n')
def freeze():
 assert not PLAN.exists();old=read(ROOT/'results/early-health-availability/run-001/plan.json');pins={};sources=old['sources']
 for s in sources:
  for n in [s['plan'],s['prefix']]:
   p=ROOT/s['base']/n;pins[str(p.relative_to(ROOT))]=sha(p)
  p=ROOT/s['controls'];pins[s['controls']]=sha(p)
  for c in s['cases']:
   p=ROOT/s['base']/'captures'/c['case']/'on/timeline.jsonl';pins[str(p.relative_to(ROOT))]=sha(p)
 for p in [ROOT/'MAC-VO-main/Scripts/Setup/probe_state_cache.py',ROOT/'MAC-VO-main/Scripts/UnitTest/test_probe_state_cache.py',ROOT/'docs/research/guarded-probe-cache-protocol-2026-11-03.md',*list((ROOT/'tools').glob('guarded_probe_cache*.py'))]:pins[str(p.relative_to(ROOT))]=sha(p)
 for p in [ROOT/'docs/research/guarded-probe-cache-fixture-amendment-2026-11-03.md',ROOT/'experiments/guarded-probe-cache-2026-11-03.json']:pins[str(p.relative_to(ROOT))]=sha(p)
 for n in ['MACVO_FrontendCov.pth','MACT_Offroad.pth']:
  p=ROOT/'models/macslam'/n;pins[str(p.relative_to(ROOT))]=sha(p)
 plan={'parent_commit':'e726eb57b','preexecution_fixture_amendment':True,'prior_lock_sha256':sha(ROOT/'experiments/guarded-probe-cache-2026-11-03.json'),'client_date':'2026-11-03','created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'sources':sources,'input_source_sha256':pins,'policies':list(MODES),'guard_frames':2,'bootstrap_pairs':2,'ttl_ns':1_000_000_000,'probe_period_ns':1_000_000_000,'probe_burst':2,'primary':'bounded_probe','native_sparse_counterfactual':False,'exposed_development':True,'reserve_access':False,'runtime':old['runtime_image']};write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(pins),'pins;18guardedwindows;6fixedpolicies')
def verify():
 p=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in p['input_source_sha256'].items():assert sha(ROOT/n)==h,n
 return p

def replay(c,timeline,mode,mutate_hidden=None):
 raw=c['rows'];endmap={cam:{row[cam+'_left']['time_ns']:j-1 for j,row in enumerate(raw) if j>0} for cam in ['rgb','thermal']};cache=ProbeCache(mode);decisions=[];ingest=[];decided=set();hidden=[];invocation={c:None for c in endmap}
 for entry in timeline:
  now=entry['controller_monotonic_ns']
  if entry['kind']=='release' and entry['frame']>0:
   pair=entry['frame']-1
   if pair not in decided:
    d=cache.plan(pair,now,force_both=pair<2 or pair>=len(raw)-3);d['score_core']=2<=pair<=len(raw)-4;d['guard']=pair>=len(raw)-3;d['source_release_sequence']=entry['sequence'];decisions.append(d);decided.add(pair)
  elif entry['kind']=='observer_event':
   event=entry['event'];cam=entry['camera'];kind=event['kind']
   if kind=='pair_begin':
    invocation[cam]=endmap[cam].get(event['interval']['time1_ns']);continue
   if kind not in ['pair_end','motion_result_written']:continue
   if kind=='motion_result_written' and not event['first_write_for_submission']:continue
   pair=endmap[cam].get(event['time1_ns'])
   if pair is None:continue
   output_requested=pair in cache.requested[cam];trigger_requested=invocation[cam] in cache.requested[cam]
   allowed=output_requested and (kind=='pair_end' or (event['stage']=='pair_processing' and trigger_requested))
   e=event
   if mutate_hidden is not None and (cam,pair) in mutate_hidden:
    e=dict(event)
    if kind=='pair_end':e['original_need_interp']=not event['original_need_interp'];e['execution_error']='hidden_mutation'
    else:e['derived_camera_delta_xyzw']=None;e['original_need_interp']=True
   if kind=='pair_end':
    value=e.get('execution_error') is None and e.get('original_need_interp') is False;accepted=cache.health_event(cam,pair,value,now) if allowed else False
   else:
    first=raw[pair][cam+'_left']['time_ns'];valid=e['time0_ns']==first and e['stage']=='pair_processing' and e.get('original_need_interp') is False and e.get('derived_camera_delta_xyzw') is not None;accepted=cache.motion_event(cam,pair,valid,now) if allowed else False
   assert accepted==allowed
   (ingest if accepted else hidden).append({'camera':cam,'pair':pair,'event_sequence':event['sequence'],'receipt_ns':now,'kind':kind,'trigger_pair':invocation[cam],'output_requested':output_requested,'trigger_requested':trigger_requested})
 return {'decisions':decisions,'accepted':ingest,'hidden':hidden}

def predict():
 plan=verify();outputs=[]
 for s in plan['sources']:
  for c in s['cases']:
   timeline=[json.loads(line) for line in (ROOT/s['base']/'captures'/c['case']/'on/timeline.jsonl').read_text().splitlines()]
   for mode in plan['policies']:
    result=replay(c,timeline,mode);assert len(result['decisions'])==len(c['rows'])-1;outputs.append({'window':s['tag']+'/'+c['window'],'recording_id':c['recording_id'],'domain':'ms2' if s['tag']=='exp114' else 'td3','mode':mode,**result})
 write(OUT/'locked-schedules.json',{'windows_policies':outputs,'truth_used':False,'native_inference_skipped':False});write(OUT/'prediction-lock.json',{'schedule_sha256':sha(OUT/'locked-schedules.json'),'producer_sha256':sha(__file__),'cache_sha256':sha(ROOT/'MAC-VO-main/Scripts/Setup/probe_state_cache.py'),'truth_used':False});verify();print('LOCKED',len(outputs),'window/policy schedules;',sum(d['score_core'] for o in outputs for d in o['decisions']),'core policy decisions')

def truth():
 plan=verify();targets={};proof=[]
 for s in plan['sources']:
  controls={r['sample_id']:r for r in read(ROOT/s['controls'])['rows']};prefix={(r['window'],r['pair_ordinal']):r for r in read(ROOT/s['base']/s['prefix'])['rows']}
  for c in s['cases']:
   log=[json.loads(line) for line in (ROOT/s['base']/'captures'/c['case']/'on/timeline.jsonl').read_text().splitlines()];dec={e['pair_ordinal']:e for e in log if e['kind']=='decision'};rows=c['rows'];cr={r['pair_ordinal']:r for r in controls.values() if r['window']==c['window']}
   for i in range(len(rows)-1):
    sid=s['tag']+'/'+c['window']+'/'+str(i);p=prefix[c['window'],i];tracking={}
    for cam in ['rgb','thermal']:
     a,b=rows[i][cam+'_left']['time_ns'],rows[i+1][cam+'_left']['time_ns'];health,=[e['event'] for e in log if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']=='pair_end' and [e['event']['time0_ns'],e['event']['time1_ns']]==[a,b]];tracking[cam]=health['original_need_interp'] is False and health.get('execution_error') is None
     if 2<=i<=len(rows)-4 and cr[i]['native_available'][cam]:
      writes=[e for e in log if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']=='motion_result_written' and e['event']['first_write_for_submission'] and [e['event']['time0_ns'],e['event']['time1_ns']]==[a,b] and e['controller_monotonic_ns']<=dec[i]['controller_monotonic_ns']];assert len(writes)==1 and writes[0]['event']['stage']=='pair_processing';proof.append({'sample_id':sid,'camera':cam,'write_sequence':writes[0]['event']['sequence'],'received_ns':writes[0]['controller_monotonic_ns'],'deadline_ns':dec[i]['controller_monotonic_ns']})
    if 2<=i<=len(rows)-4:assert dec[i]['boundary']['kind']=='group_complete'
    targets[sid]={'tracking':tracking,'camera_local':{cam:p['experts'][cam]['available_by_prefix'] for cam in ['rgb','thermal']},'projected':cr[i]['native_available'],'scored_core':2<=i<=len(rows)-4}
 return targets,proof

def stats(outputs,targets):
 counts=collections.Counter();ages=[];recover={};requests=0;non_guard_requests=0;possible=0
 for output in outputs:
  for d in output['decisions']:
   requests+=len(d['requests']);non_guard_requests+=0 if d['guard'] else len(d['requests'])
   if not d['score_core']:continue
   counts['core_pairs']+=1;counts['core_requests']+=len(d['requests']);t=targets[output['window']+'/'+str(d['pair'])]['projected'];a=d['action']
   if a=='ABSTAIN':counts['uncommitted_while_available' if any(t.values()) else 'uncommitted_none_available']+=1
   else:counts['selected_eligible' if t[a.lower()] else 'selected_unavailable']+=1
   counts['requested_some_eligible']+=int(any(t[c] for c in d['requests']));counts['refresh_decisions']+=int(d['reason']=='refresh_unknown_stale_or_failed')
   for c,v in d['views'].items():
    counts['view_'+v['status']]+=1
    if v['age_ns'] is not None:ages.append(v['age_ns']/1e9)
    if c in d['requests'] and a!='ABSTAIN' and c!=a.lower():counts['inactive_probe_requests']+=1
 return {**dict(counts),'all_requests_including_guards':requests,'bootstrap_and_core_requests':non_guard_requests,'guard_requests':requests-non_guard_requests,'resolved_age_median_s':float(__import__('numpy').median(ages)) if ages else None,'resolved_age_max_s':max(ages) if ages else None,'request_fraction_of_dense_core':counts['core_requests']/(2*counts['core_pairs']),'actual_inference_savings_measured':False}

def score():
 plan=verify();lock=read(OUT/'prediction-lock.json');assert lock['schedule_sha256']==sha(OUT/'locked-schedules.json');outputs=read(OUT/'locked-schedules.json')['windows_policies'];targets,proof=truth();write(OUT/'guarded-native-truth.json',targets);write(OUT/'guard-deadline-proof.json',{'bindings':proof,'ordinary_deadlines_only':True});results={}
 for domain in ['td3','ms2']:
  group=[o for o in outputs if o['domain']==domain];methods={m:stats([o for o in group if o['mode']==m],targets) for m in plan['policies']};a,b=methods['bounded_probe'],methods['full_read_cache'];passed=a.get('selected_eligible',0)>=b.get('selected_eligible',0) and a.get('selected_unavailable',0)<=b.get('selected_unavailable',0) and a.get('uncommitted_while_available',0)<=b.get('uncommitted_while_available',0) and a['bootstrap_and_core_requests']<b['bootstrap_and_core_requests'];results[domain]={'policies':methods,'development_contract_passed':passed}
 records={r:{m:stats([o for o in outputs if o['recording_id']==r and o['mode']==m],targets) for m in plan['policies']} for r in sorted({o['recording_id'] for o in outputs})};result={'domains':results,'records':records,'primary_contract_passed':all(x['development_contract_passed'] for x in results.values()),'masked_dense_replay':True,'native_sparse_validated':False,'policy_adopted':False,'pose_error_labels_read':False};write(OUT/'scores.json',result);verify();print(json.dumps({'primary_contract_passed':result['primary_contract_passed'],'domains':results},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','verify','predict','score']);a=p.parse_args()
 if a.action=='verify':print('PASS',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()
