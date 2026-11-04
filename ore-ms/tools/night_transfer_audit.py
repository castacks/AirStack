"""Independent night-transfer audit, native fixed cameras and exact-cost readouts."""
import collections
import json
from multiprocessing.reduction import ForkingPickler
import numpy as np
from night_transfer_study import ROOT,OUT,PLAN,read,write,sha,verify
CAMERAS=['rgb','thermal']

# EXP122 independent prefix model extended only for capacity-based lookup misses.
def prefix(plan,base,perturb,states):
    rows=plan['window']['rows'];N=len(rows);mode=read(base/'summary.json')['mode']
    log=[json.loads(l) for l in (base/'timeline.jsonl').read_text().splitlines()];active=None;checks=0;decision=None
    meta=read(base/'summary.json');ready=meta['readiness_enabled'];fixture=read(ROOT/plan['fixture'])['decisions'] if meta['frozen_request_fixture'] else None
    queries=[];live={c:None for c in CAMERAS};history=[];extra_used=0
    health={c:{} for c in CAMERAS};motion={c:{} for c in CAMERAS};resolved={c:{} for c in CAMERAS}
    requested={c:set() for c in CAMERAS};last={c:None for c in CAMERAS};burst={c:0 for c in CAMERAS};prior={c:0 for c in CAMERAS};bounds={c:{} for c in CAMERAS}
    endmaps={c:{r[c+'_left']['time_ns']:i for i,r in enumerate(rows)} for c in CAMERAS}
    for e in log:
        if e['kind']=='preselection':
            record=e;e=e['base_decision'] if ready else e
            i=e['pair'];now=e['decision_ns'];views={}
            for c in CAMERAS:
                if not resolved[c]:v={'status':'UNKNOWN','value':None,'last_value':None,'pair':None,'receipt_ns':None,'age_ns':None}
                else:
                    pair=max(resolved[c]);value,t=resolved[c][pair];age=now-t;assert age>=0;stale=age>=1_000_000_000
                    v={'status':'STALE' if stale else 'READY' if value else 'FAILED','value':None if stale else value,
                       'last_value':value,'pair':pair,'receipt_ns':t,'age_ns':age}
                views[c]=v
            assert views==e['views'];checks+=14
            if fixture is not None:
                declared=fixture[i]
                assert record['action']==declared['action'] and record['requests']==declared['requests'] and record['frozen_request_fixture']
                for c in declared['requests']:requested[c].add(i);last[c]=now
                active=declared['action'].lower() if declared['action']!='ABSTAIN' else None
                decision=record;checks+=3;continue
            good=[c for c in CAMERAS if views[c]['value'] is True]
            if mode.startswith('fixed_'):chosen=mode.removeprefix('fixed_');reason='fixed_camera'
            else:chosen=active if active in good else good[0] if good else None;reason='keep_fresh_active' if chosen==active and chosen is not None else 'switch_fresh_available' if chosen else 'no_fresh_available'
            active=chosen
            if i<2 or i>=N-3:req=set(CAMERAS);reason='common_bootstrap_or_guard'
            elif mode=='full_read_cache':req=set(CAMERAS);reason='full_read_control'
            elif chosen is None:req=set(CAMERAS);reason='refresh_unknown_stale_or_failed'
            else:
                req={chosen}
                if mode=='bounded_probe':
                    other=next(c for c in CAMERAS if c!=chosen);due=views[other]['status'] in ['UNKNOWN','STALE'] or last[other] is None or now-last[other]>=1_000_000_000
                    if due and burst[other]==0:burst[other]=2
                    if burst[other]>0:req.add(other)
            assert e['action']==(chosen.upper() if chosen else 'ABSTAIN') and e['reason']==reason and e['burst_before']==burst
            assert e['requests']==[c for c in CAMERAS if c in req];checks+=5
            if chosen and not mode.startswith('fixed_'):assert views[chosen]['value'] is True;checks+=1
            for c in req:requested[c].add(i);last[c]=now;burst[c]=max(0,burst[c]-1)
            assert e['burst_after']==burst;decision=record;checks+=1
            if ready:
                source=i+1;common=reason in ['common_bootstrap_or_guard','refresh_unknown_stale_or_failed']
                for c in CAMERAS:
                    session=live[c]
                    if session is None:continue
                    if session['status']=='pending':
                        status='became_active' if c==chosen else 'shared_refresh' if common else 'attempt_cap' if session['attempts']>=4 else 'extra_budget' if extra_used>=20 and c not in req else None
                        if status:session.update(status=status,closed_source=source)
                    if session['status']!='pending':live[c]=None
                for c in CAMERAS:
                    if not common and c!=chosen and c in req and e['burst_before'][c]==2 and live[c] is None:
                        session={'id':len(history),'camera':c,'start_source':source,'start_ns':now,'attempts':0,
                                 'requested_sources':[],'extra_sources':[],'status':'pending','evidence':None}
                        live[c]=session;history.append(session)
                extras=[]
                for c in CAMERAS:
                    session=live[c]
                    if session is not None and session['status']=='pending' and c not in req and session['attempts']<4 and extra_used<20:
                        extras.append(c);extra_used+=1;session['extra_sources'].append(source)
                        requested[c].add(i);last[c]=now
                actual=req|set(extras)
                for c in CAMERAS:
                    session=live[c]
                    if session is not None and session['status']=='pending' and c in actual:
                        session['attempts']+=1;session['requested_sources'].append(source);assert session['attempts']<=4
                assert record['requests']==[c for c in CAMERAS if c in actual]
                assert record['readiness_overlay']=={'extra_requests':extras,'extra_used':extra_used,'attempt_limit':4,'extra_limit':20,'sessions':live}
                checks+=8
        elif e['kind']=='buffer_query':
            c=e['camera'];group=e['delivery_group'];source=e['source'];queries.append(e)
            assert meta['catchup_enabled'] and len(queries)<=2 and sum(q['delivery_group']==group for q in queries)<=1
            assert source==group-1 and prior[c]==group-2 and decision['action']==c.upper()
            assert not (perturb and any(c in o['cameras'] and o['start_source']<=group<=o['end_source'] for o in plan['outages']))
            checks+=5
        elif e['kind']=='buffer_lookup':
            expected=(e['camera'],e['source']) in states[e['camera'],e['delivery_group']]
            assert e['retained']==expected;checks+=1
        elif e['kind']=='catchup_release':
            c=e['camera'];source=e['source'];pair=source-1
            assert source==prior[c]+1 and pair not in bounds[c] and pair not in health[c]
            requested[c].add(pair)
            bound={'pair':pair,'source_start':prior[c],'source_end':source,'sensor_interval_ns':[rows[j][c+'_left']['time_ns'] for j in [prior[c],source]]}
            assert e['binding']==bound;bounds[c][pair]=bound;prior[c]=source;checks+=4
        elif e['kind'] in ['release','skip_release','transport_drop']:
            c=e['camera'];source=e['source'];req=source==0 or source-1 in requested[c]
            unavailable=perturb and any(c in o['cameras'] and o['start_source']<=source<=o['end_source'] for o in plan['outages'])
            delivered=req and not unavailable
            assert e['policy_requested']==req and e['availability_observed']==req and e['delivered']==delivered
            assert e['kind']==('release' if delivered else 'transport_drop' if req else 'skip_release');checks+=4
            if source:
                assert decision['pair']==source-1 and decision['sequence']<e['sequence'];checks+=2
                if delivered:
                    bound={'pair':source-1,'source_start':prior[c],'source_end':source,'sensor_interval_ns':[rows[j][c+'_left']['time_ns'] for j in [prior[c],source]]}
                    assert bound==e['binding'];bounds[c][source-1]=bound;prior[c]=source;checks+=1
                else:assert e['binding'] is None;checks+=1
        elif e['kind']=='transport_unavailable':
            c=e['camera'];source=e['source'];pair=source-1
            assert pair in requested[c] and pair not in bounds[c] and pair not in health[c]
            health[c][pair]=(False,e['controller_monotonic_ns']);resolved[c][pair]=health[c][pair];checks+=3
            if ready and live[c] is not None and live[c]['status']=='pending':
                live[c].update(status='transport_failed',closed_source=source)
        elif e['kind']=='transport_ingress':
            assert prior[e['camera']]==e['native_previous_source'] and not e['native_health_fabricated'];checks+=2
        elif e['kind']=='observer_event':
            ev=e['event'];c=e['camera'];kind=ev['kind']
            if kind not in ['pair_end','motion_result_written'] or ev['stage']!='pair_processing':continue
            if kind=='motion_result_written' and not ev['first_write_for_submission']:continue
            pair=endmaps[c][ev['time1_ns']]-1;bound=bounds[c][pair]
            assert bound['sensor_interval_ns']==[ev['time0_ns'],ev['time1_ns']] and e['trigger_source']-1 in bounds[c];checks+=2
            if kind=='pair_end':assert pair not in health[c];health[c][pair]=(ev['execution_error'] is None and not ev['original_need_interp'],e['controller_monotonic_ns'])
            else:assert pair not in motion[c];motion[c][pair]=(not ev['original_need_interp'] and ev['derived_camera_delta_xyzw'] is not None,e['controller_monotonic_ns'])
            if pair in health[c]:
                value,t=health[c][pair]
                if not value:resolved[c][pair]=(False,t)
                elif pair in motion[c]:resolved[c][pair]=(motion[c][pair][0],max(t,motion[c][pair][1]))
            checks+=1
            if ready:
                session=live[c]
                valid=kind=='motion_result_written' and not ev['original_need_interp'] and ev['derived_camera_delta_xyzw'] is not None
                if session is not None and session['status']=='pending' and valid and pair in health[c] and health[c][pair][0] and bound['source_end']>=session['start_source'] and bound['source_start']+1==bound['source_end'] and e['trigger_source']==bound['source_end']+1 and e['delivery_group']<=bound['source_end']+1:
                    session['evidence']={'binding':bound,'trigger_source':e['trigger_source'],'receipt_ns':e['controller_monotonic_ns'],'event_sequence':ev['sequence']}
                    session.update(status='satisfied',closed_source=e['trigger_source'])
                    checks+=4
    meta=read(base/'summary.json')
    if ready:
        assert history==meta['readiness_sessions'] and extra_used==meta['readiness_extra_used']
        assert extra_used<=20 and all(session['attempts']<=4 for session in history);checks+=4
    assert len(queries)==meta['catchup_queries'];checks+=1
    return log,checks

def main():
 plan=verify();scores=read(OUT/'scores.json');lock=read(OUT/'capture-lock.json');checks=0
 def check(ok,label):
  nonlocal checks
  if not ok:raise AssertionError(label)
  checks+=1
 check(lock['plan_sha256']==sha(PLAN),'capture plan')
 for mapping in [plan['input_source_sha256'],plan['prior_control_sha256'],lock['sha256']]:
  for path,digest in mapping.items():check(sha(ROOT/path)==digest,path)
 predlock=read(OUT/'prediction-lock.json')
 check(predlock['predictions_sha256']==sha(OUT/'predictions.json') and predlock['source_sha256']==sha(ROOT/'tools/night_transfer_evaluate.py'),'prediction lock')
 logs={};w=plan['window'];array_count=0
 for arm in ['unperturbed']+plan['arms']:
  base=OUT/'captures'/arm;log=[json.loads(l) for l in (base/'timeline.jsonl').read_text().splitlines()];logs[arm]=log
  report=read(base/'retained-inputs.json');service=read(base/'lease-service.json')
  capacity=plan['diagnostic_store_bytes'] if arm=='diagnostic_probe' else plan['primary_store_bytes']
  check(service['store_capacity_bytes']==capacity and service['arena']['allocated_bytes']==plan['arena_bytes'],'capacities')
  store={};states={};image_meta={};ingress=peak=evicted=rejected=0
  order=[(e['camera'],e['source']) for e in log if e['kind'] in ['release','transport_drop','skip_release']]
  check([(r['camera'],r['source']) for r in report['arrivals']]==order,'arrival order')
  for r in report['arrivals']:
   c,i=r['camera'],r['source'];expired=sorted(s for cam,s in store if cam==c and s<i-1)
   check(expired==r['expired_sources'],'expiry')
   store={key:value for key,value in store.items() if not(key[0]==c and key[1]<i-1)}
   available=arm=='unperturbed' or not any(c in o['cameras'] and o['start_source']<=i<=o['end_source'] for o in plan['outages'])
   dropped=[];oversize=False;admit=False
   if available:
    images={}
    for side in ['left','right']:
     original=w['rows'][i][c+'_'+side];path=ROOT/w['dataset']/original['path']
     images[side]={'path':original['path'],'time_ns':original['time_ns'],'bytes':path.stat().st_size,'sha256':sha(path)}
    check(images==r['images'],'actual image metadata');image_meta[c,i]=images
    size=sum(x['bytes'] for x in images.values());ingress+=size;oversize=size>capacity
    if oversize:rejected+=1
    else:
     while sum(store.values())+size>capacity:
      key=min(store,key=lambda k:(k[1],k[0]));del store[key];dropped.append({'camera':key[0],'source':key[1]});evicted+=1
     store[c,i]=size;admit=True
   else:check('images' not in r,'unavailable has no bytes')
   current=sum(store.values());peak=max(peak,current);states[c,i]=dict(store)
   check(current<=capacity and current==r['resident_bytes'],'capacity state')
   check(r['stored']==admit and r['rejected_oversize']==oversize and r['evicted']==dropped,'admission')
   check(r['resident']=={cam:sorted(s for cc,s in store if cc==cam) for cam in CAMERAS},'full state')
  for field,value in [('ingress_bytes',ingress),('peak_retained_bytes',peak),('evicted_pairs',evicted),('rejected_pairs',rejected)]:check(report[field]==value,field)
  _,count=prefix(plan,base,arm!='unperturbed',states);checks+=count
  active=None;outstanding=None;descriptor=None;leases=acks=peaklease=retrieved=0;commands=0
  for e in log:
   kind=e['kind']
   if kind=='buffer_lookup':
    expected=(e['camera'],e['source']) in states[e['camera'],e['delivery_group']]
    check(e['retained']==expected,'lookup state')
    if expected:retrieved+=sum(x['bytes'] for x in image_meta[e['camera'],e['source']].values())
   elif kind=='lease_publish':
    d=e['descriptor'];check(active is None and outstanding is None,'publish ownership')
    leases+=1;identity={k:d[k] for k in ['token','camera','source','group']}
    check(identity=={'token':leases,'camera':e['camera'],'source':e['source'],'group':e['delivery_group']},'lease identity')
    offset=0
    for side in ['left','right']:
     expected=dict(image_meta[e['camera'],e['source']][side],offset=offset)
     check(d['images'][side]==expected,'lease descriptor original bytes');offset+=expected['bytes']
    check(offset==d['payload_bytes'] and offset<=plan['arena_bytes'],'arena ceiling')
    active=identity;descriptor=d;peaklease=max(peaklease,offset)
   elif kind=='lease_rejected':
    check(sum(x['bytes'] for x in image_meta[e['camera'],e['source']].values())>plan['arena_bytes'],'arena oversize')
   elif kind=='command_send':
    check(outstanding is None,'one global command credit');outstanding=(e['camera'],e['source']);commands+=1
    check(e['lease']==active,'command lease identity')
    release=next(x for x in log if x['kind'] in ['release','skip_release','transport_drop'] and x['camera']==e['camera'] and x['source']==e['source'])
    encoded=ForkingPickler.dumps(('step',e['source'],release['delivered'],descriptor))
    check(len(encoded)<=4096,'step bytes')
   elif kind=='completed':
    check(outstanding==(e['camera'],e['source']) and e['lease_ack']==active,'matching completion')
    check(e['buffered_count']==int(active is not None),'native buffered batch count')
   elif kind=='lease_acknowledged':
    check(active=={k:e[k] for k in ['token','camera','source','group']},'lease acknowledgment');active=None;descriptor=None;acks+=1
   elif kind=='command_acknowledged':
    check(outstanding==(e['camera'],e['source']) and active is None,'credit release');outstanding=None
   elif kind=='observer_event' and e['delivery_group'] is not None:
    check(outstanding==(e['camera'],e['delivery_group']),'native event in current command')
  check(commands==200 and active is None and outstanding is None,'completed service')
  check(report['retrieved_bytes']==retrieved,'retrieval counters')
  for field,value in [('leases',leases),('acks',acks),('active',None),('peak_leased_bytes',peaklease),('slots',1)]:check(service['arena'][field]==value,field)
  check(service['allocated_encoded_ceiling_bytes']==capacity+plan['arena_bytes'],'store plus allocated arena')
  for c in CAMERAS:
   path=base/'experts'/c;mapping=read(path/'source-mapping.json')
   releases=[e for e in log if e['kind'] in ['release','catchup_release'] and e['camera']==c];indices=[e['source'] for e in releases]
   check(indices==mapping['source_indices'] and all(a<b for a,b in zip(indices,indices[1:])),'strict native inputs')
   cost=read(path/'native-cost.json');runs=[x for x in cost['native_phases'] if x['kind']=='run']
   check([(x['source'],x['delivery_group']) for x in runs]==[(e['source'],e['delivery_group']) for e in releases],'native invocation versus delivery')
   check(sum(x['kind']=='frontend_pair' for x in cost['frontend_calls'])==len(indices)-1,'calls')
   for x in cost['native_phases']+cost['frontend_calls']:check(x['status']=='PASS' and x['duration_ns']==x['end_ns']-x['start_ns'],'native cost')
   with np.load(path/'trajectory.npz') as x:check(x['time_ns'].reshape(-1).tolist()==[w['rows'][i][c+'_left']['time_ns'] for i in indices],'native times')
   parent=service['control_channels'][c];worker=read(path/'control-channel.json')
   check(parent['sent']==worker['received']==101 and parent['received']==worker['sent'],'message conservation')
   check(parent['max_sent_bytes']==worker['max_received_bytes'] and parent['max_received_bytes']==worker['max_sent_bytes'],'message size agreement')
   for p in [parent,worker]:check(p['max_sent_bytes']<=65536 and p['max_received_bytes']<=65536 and p['max_step_bytes']<=4096,'message ceilings')
 for r in scores['rows']:
  log=logs[r['arm']];pair=r['pair'];deadline=next(e['controller_monotonic_ns'] for e in log if e['kind']=='group_complete' and e['source']==pair+2)
  d=next(e for e in log if e['kind']=='preselection' and e['pair']==pair)
  check(r['action']==d['action'] and r['decision_ns']==d['decision_ns'],'preselection')
  for c,expert in r['experts'].items():
   times=[w['rows'][j][c+'_left']['time_ns'] for j in [pair,pair+1]]
   hh=[e for e in log if e['kind']=='observer_event' and e['camera']==c and e['event']['kind']=='pair_end' and e['event']['time1_ns']==times[1]]
   mm=[e for e in log if e['kind']=='observer_event' and e['camera']==c and e['event']['kind']=='motion_result_written' and e['event']['time1_ns']==times[1] and e['event']['first_write_for_submission']]
   h=hh[0] if hh else None;m=mm[0] if mm else None
   delivered=any(e['kind'] in ['release','catchup_release'] and e['camera']==c and e['source']==pair+1 for e in log)
   ok=bool(delivered and h and m and [h['event']['time0_ns'],h['event']['time1_ns']]==times and
      h['event']['execution_error'] is None and not h['event']['original_need_interp'] and m['event']['stage']=='pair_processing' and
      h['controller_monotonic_ns']<=deadline and m['controller_monotonic_ns']<=deadline and
      [m['event']['time0_ns'],m['event']['time1_ns']]==times and not m['event']['original_need_interp'] and m['event']['derived_camera_delta_xyzw'] is not None)
   check(expert['native_eligible']==ok,'independent native eligibility')
   if ok:check(expert['motion_trigger_source']==m['delivery_group'] and expert['motion_invocation_source']==m['trigger_source'],'actual invocation delivery boundary')
  selected=r['experts'].get(r['action'].lower())
  check(r['selected_native_eligible']==bool(selected and selected['native_eligible']),'selected eligibility')
  check(r['opportunity']==any(e['native_eligible'] for e in r['experts'].values()),'opportunity')
 for arm,g in scores['groups'].items():
  base=OUT/'captures'/arm;calls=native_ns=0
  for c in CAMERAS:
   cost=read(base/'experts'/c/'native-cost.json');calls+=sum(x['kind']=='frontend_pair' for x in cost['frontend_calls']);native_ns+=sum(x['duration_ns'] for x in cost['native_phases'] if x['kind']=='run')
  check(g['cost']['total_pair_calls']==calls and g['cost']['total_run_seconds']==native_ns/1e9,'cost sum')
  log=logs[arm];first=next(e['controller_monotonic_ns'] for e in log if e['kind']=='release');last=max(e['controller_monotonic_ns'] for e in log if e['kind']=='group_complete')
  check(g['cost']['controller_processing_seconds']==(last-first)/1e9,'controller cost boundary')
  if arm=='unperturbed':continue
  rr=[r for r in scores['rows'] if r['arm']==arm]
  check(len(rr)==95 and g['eligible']==sum(r['selected_native_eligible'] for r in rr),'guarded count')
  check(g['unavailable_selected']==sum(r['action']!='ABSTAIN' and not r['selected_native_eligible'] for r in rr),'unavailable')
  check(g['uncommitted_while_opportunity']==sum(r['action']=='ABSTAIN' and r['opportunity'] for r in rr),'abstention opportunity')
  for episode in g['transition_latency']:
   c=episode['camera'];ret=episode['return_source'];horizon=episode['horizon_source']
   row=next((r for r in rr if ret<=r['pair']+1<=horizon and r['experts'][c]['native_eligible']),None)
   reported=episode['first_on_time_adjacent_return']
   check((row is None)==(reported is None),'return censoring')
   if row:check(reported['target_ending_source']==row['pair']+1 and reported['delivery_source']==row['experts'][c]['motion_trigger_source'],'return endpoint')
 for arm in ['unperturbed']:
  current=OUT/'captures'/arm/'experts';old=ROOT/plan['prior_unperturbed'];count=0
  current_paths={p.relative_to(current) for p in current.rglob('*.npz')};old_paths={p.relative_to(old) for p in old.rglob('*.npz')}
  check(current_paths==old_paths,'control file set')
  for rel in current_paths:
   with np.load(current/rel) as a,np.load(old/rel) as b:
    check(set(a.files)==set(b.files),'array key set')
    for key in a.files:
     x,y=a[key],b[key];check(x.dtype==y.dtype and x.shape==y.shape and x.tobytes()==y.tobytes(),'raw logical bytes');count+=1
  check(scores['controls'][arm]['arrays']==count,'control count');array_count+=count
 criteria=scores['primary_criteria'];g=scores['groups'];c=g['leased_probe'];b=g['bounded_probe']
 check(criteria['eligible_noninferior_full']==(c['eligible']>=g['full_read_cache']['eligible']),'criterion full')
 check(criteria['eligible_noninferior_original']==(c['eligible']>=b['eligible']),'criterion original')
 check(criteria['fewer_calls_than_full']==(c['cost']['total_pair_calls']<g['full_read_cache']['cost']['total_pair_calls']),'criterion calls')
 for camera in CAMERAS:
  arm='matched_'+camera;reported=scores['matched_readouts'][arm]
  source=[r for r in scores['rows'] if r['arm']=='leased_probe'];matched=[r for r in scores['matched_rows'] if r['arm']==arm]
  check(len(source)==len(matched)==95,'matched population')
  for r,m in zip(source,matched):
   check(m['pair']==r['pair'] and m['action']==camera.upper() and m['selected_native_eligible']==r['experts'][camera]['native_eligible'],'matched constant choice')
  check(reported['eligible']==sum(r['experts'][camera]['native_eligible'] for r in source),'matched eligibility')
  check(reported['unavailable_selected']==95-reported['eligible'],'matched unavailable')
  check(reported['cost']==c['cost'] and not reported['independent_native_run'],'exact realized native cost conditioning')
 check(criteria['strictly_better_both_native_fixed']==(c['eligible']>max(g[a]['eligible'] for a in ['fixed_rgb','fixed_thermal'])),'native fixed criterion')
 check(criteria['strictly_better_both_matched_readouts']==(c['eligible']>max(x['eligible'] for x in scores['matched_readouts'].values())),'matched readout criterion')
 check(scores['primary_contract_passed']==all(criteria.values()),'research conjunction')
 check(scores['execution_contract_passed']==all(scores['execution_criteria'].values()),'execution conjunction')
 write(OUT/'audit.json',{'pass':True,'checks':checks,'core_rows':len(scores['rows']),'byte_exact_arrays':array_count,
  'frozen_pins':len(plan['input_source_sha256']),'pose_references_read':False,'scores_sha256':sha(OUT/'scores.json')})
 print('PASS',checks,'checks;',array_count,'byte-exact arrays')

if __name__=='__main__':main()
