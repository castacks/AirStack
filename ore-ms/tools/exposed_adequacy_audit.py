"""Independent natural-availability sets, run boundaries and restored evidence audit."""
import copy
import itertools
import tarfile
from exposed_adequacy_study import ROOT,OUT,PLAN,EVIDENCE,TRUTH,read,write,sha,verify,journal


def validate(data,plan):
 checks=0
 def check(ok,label):
  nonlocal checks
  if not ok:raise AssertionError(label)
  checks+=1
 for field in ['new_native_execution','reserved_payload_access','new_policy','physical_accuracy_claim','messages_sent','policy_adopted']:
  check(data[field] is False,field)
 check(data['retrospective'] is True and len(data['rows'])==1150,'retrospective full population')
 truth=read(TRUTH);allsets={};health_results=[];identities=set()
 check(set(data['windows'])=={w['window'] for w in plan['windows']},'window set')
 for window in plan['windows']:
  raw=window['rows'];log=journal(ROOT/window['timeline']);dec={e['pair_ordinal']:e for e in log if e['kind']=='decision'}
  rows={r['pair']:r for r in data['rows'] if r['window']==window['window']}
  check(set(rows)==set(range(2,window['frames']-3)),'window full core')
  H={};M={}
  for e in log:
   if e['kind']!='observer_event':continue
   event=e['event'];key=e['camera'],event.get('time1_ns')
   if event['kind']=='pair_end':check(key not in H,'unique native health');H[key]=e
   elif event['kind']=='motion_result_written' and event['first_write_for_submission']:
    check(key not in M,'unique native first write');M[key]=e
  sets={c:set() for c in ['rgb','thermal']}
  for i,row in rows.items():
   boundary=dec[i];B=boundary['controller_monotonic_ns']
   check(boundary['boundary']=={'kind':'group_complete','group':i+2},'ordinary original deadline')
   done=[e for e in log if e['kind']=='completed' and e['frame']==i+2]
   check(len(done)==2 and all(e['controller_monotonic_ns']<=B for e in done),'both camera completion')
   check(not any(e['kind']=='release' and e['frame']>i+2 and e['controller_monotonic_ns']<=B for e in log),'no future releases')
   check(row['recording_id']==window['recording_id'] and row['domain']==window['domain'] and row['deadline_ns']==B,'identity')
   for c in ['rgb','thermal']:
    times=[raw[j][c+'_left']['time_ns'] for j in [i,i+1]];h=H[c,times[1]];m=M.get((c,times[1]));event=h['event']
    check([event['time0_ns'],event['time1_ns']]==times,'native health interval')
    supported=event['execution_error'] is None and event['original_need_interp'] is False
    good=supported
    if m is None:good=False
    else:
     motion=m['event'];good=good and h['controller_monotonic_ns']<=B and m['controller_monotonic_ns']<=B
     good=good and motion['time0_ns']==times[0] and motion['time1_ns']==times[1] and motion['stage']=='pair_processing'
     good=good and motion['original_need_interp'] is False and motion['derived_camera_delta_xyzw'] is not None
    if good:sets[c].add(i)
    expected={'supported':supported,'eligible':bool(good),'native_interval_ns':times,
      'health_event_sequence':event['sequence'],'motion_event_sequence':m['event']['sequence'] if m else None,
      'health_receipt_ns':h['controller_monotonic_ns'],'motion_receipt_ns':m['controller_monotonic_ns'] if m else None,
      'write_stage':m['event']['stage'] if m else None}
    for field,value in expected.items():check(row['experts'][c][field]==value,field)
    sample=window['window']+'/'+str(i)
    check(bool(good)==dec[i]['availability'][c]==truth[sample]['camera_local'][c],'old availability agreement')
   check(row['projected_eligibility']==truth[window['window']+'/'+str(i)]['projected'],'separate projected truth')
   mask='both' if i in sets['rgb'] and i in sets['thermal'] else 'rgb_only' if i in sets['rgb'] else 'thermal_only' if i in sets['thermal'] else 'none'
   check(row['mask']==mask,'availability mask')
   ident=(row['recording_id'],*(t for c in ['rgb','thermal'] for t in row['experts'][c]['native_interval_ns']))
   check(ident not in identities,'no repeated paired interval');identities.add(ident)
  allsets[window['window']]=sets
  for c in ['rgb','thermal']:
   values=[H[c,r[c+'_left']['time_ns']]['event']['execution_error'] is None and
      H[c,r[c+'_left']['time_ns']]['event']['original_need_interp'] is False for r in raw[1:]]
   rebuilt=[];start=0
   for value,group in itertools.groupby(values):
    length=len(list(group));rebuilt.append({'start_pair':start,'end_pair':start+length-1,'supported':value});start+=length
   returns=[{'failed_run':a,'returned_run':b} for a,b in zip(rebuilt,rebuilt[1:]) if not a['supported'] and b['supported']]
   cycles=[]
   for a,b,d in zip(rebuilt,rebuilt[1:],rebuilt[2:]):
    if a['supported'] and not b['supported'] and d['supported']:
     core=sorted(set(rows)&set(range(b['start_pair'],b['end_pair']+1)));other='thermal' if c=='rgb' else 'rgb'
     cycles.append({'healthy_run':a,'failed_run':b,'returned_run':d,
       'sustained':min(x['end_pair']-x['start_pair']+1 for x in [a,b,d])>=3,
       'failure_core_pairs':core,'alternate_eligible_all_failure_core':bool(core) and all(i in sets[other] for i in core),
       'failure_fully_core':len(core)==b['end_pair']-b['start_pair']+1})
   health_results.append({'window':window['window'],'recording_id':window['recording_id'],'camera':c,
      'runs':rebuilt,'false_to_true':returns,'healthy_failed_returned_cycles':cycles})
 check(data['health_windows']==health_results,'full camera runs and local recovery boundaries')
 def summary(rows):
  R={i for i,r in enumerate(rows) if r['experts']['rgb']['eligible']};T={i for i,r in enumerate(rows) if r['experts']['thermal']['eligible']}
  return {'slots':len(rows),'masks':{'none':len(rows)-len(R|T),'rgb_only':len(R-T),'thermal_only':len(T-R),'both':len(R&T)},
    'rgb_eligible':len(R),'thermal_eligible':len(T),'union_eligible':len(R|T),'headroom_above_best_constant':min(len(R-T),len(T-R))}
 for window in plan['windows']:
  rr=[r for r in data['rows'] if r['window']==window['window']]
  check(data['windows'][window['window']]==dict(summary(rr),recording_id=window['recording_id'],domain=window['domain']),'window counts')
 records={r['recording_id'] for r in data['rows']};check(len(records)==7 and set(data['recordings'])==records,'seven records')
 for rec in records:check(data['recordings'][rec]==summary([r for r in data['rows'] if r['recording_id']==rec]),'recording counts')
 ref=data['reference_inventory'];ledger=read(ROOT/'experiments/recording-exposure-ledger-2026-10-31.json')
 rig=read(ROOT/ref['td3_rig_evidence']);adapter=read(ROOT/'experiments/prospective-state-shadow-evaluation-2026-11-01-adapter-v2.json')
 check(ref['td3_rig_contract']=={k:rig[k] for k in ['gps_frame_in_supplied_graph','gps_equals_imu_assumed',
   'physical_body_transform_verified','independent_ground_truth','training_membership']},'rig source flags')
 check(ref['ms2_adapter_status']==adapter['status'],'retain native adapter progress')
 check(ref['training_membership']==ledger['checkpoint']['training_membership'],'training source')
 check(ref['physical_quality_target_established_by_inspected_evidence'] is False,'no physical truth promotion')
 check(ref['new_prospective_assignment']==adapter['partitions']['test'],'no new assignment')
 check(ref['unknown_training_alone_blocks_narrow_benchmark'] is False,'narrow benchmark distinction')
 check(ref['protected_records']==[r['id'] for r in ledger['records'] if r['exposure_status']=='reported_reserved'],'protected records retained')
 box=read(EVIDENCE/'box-root.json');names=sorted(e['Name'] for e in box if not e['IsDir'] and e['Name'].endswith('.zip'))
 check(names==[r['archive'] for r in ref['current_box_td3_archives']] and len(names)==9,'nine archive metadata')
 for e in ref['current_box_td3_archives']:
  old=next(r for r in ledger['records'] if e['archive'] in r['aliases'])
  check(e['recording_id']==old['id'] and e['exposure_status']==old['exposure_status']=='development_exposed','archive exposure evidence')
 return checks


def restoration():
 checks=0
 for item in read(EVIDENCE/'restoration.json'):
  archive=ROOT/item['archive'];manifest=read(ROOT/item['manifest']);assert sha(archive)==item['archive_sha256'];checks+=1
  filename=archive.name
  expected=manifest['archive_sha256'] if 'archive_sha256' in manifest else manifest['file_sha256'][filename] if 'file_sha256' in manifest else manifest['files'][filename]['sha256']
  assert sha(archive)==expected;checks+=1
  members=list(item['members'])
  if filename=='source-replies.tar.gz':members+=read(EVIDENCE/'supplemental-restoration.json')
  with tarfile.open(archive) as tar:
   for member in members:
    import hashlib
    payload=tar.extractfile(member['member']).read();digest=hashlib.sha256(payload).hexdigest()
    assert digest==member['sha256']==manifest['member_sha256'][member['member']]==sha(ROOT/member['local']);checks+=1
 return checks


def audit():
 plan=verify();lock=read(OUT/'analysis-lock.json')
 assert lock['inventory_sha256']==sha(OUT/'inventory.json') and lock['plan_sha256']==sha(PLAN)
 assert lock['source_sha256']==sha(ROOT/'tools/exposed_adequacy_study.py')
 data=read(OUT/'inventory.json');checks=validate(data,plan)+restoration()
 first=next(iter(data['windows']))
 mutations={
 'fabricated_headroom':lambda d:d['windows'][first].__setitem__('headroom_above_best_constant',99),
 'dropped_hard_slot':lambda d:d['rows'].pop(),
 'wrong_camera_mask':lambda d:d['rows'][0].__setitem__('mask','unknown'),
 'stitched_recovery':lambda d:d['health_windows'][0]['healthy_failed_returned_cycles'].append({'sustained':True}),
 'physical_truth_claim':lambda d:d['reference_inventory'].__setitem__('physical_quality_target_established_by_inspected_evidence',True),
 'reserve_erasure':lambda d:d['reference_inventory'].__setitem__('protected_records',[]),
 'adapter_progress_erased':lambda d:d['reference_inventory'].__setitem__('ms2_adapter_status','NOT_VERIFIED'),
 'new_policy_claim':lambda d:d.__setitem__('new_policy',True),
 'unknown_training_blanket_veto':lambda d:d['reference_inventory'].__setitem__('unknown_training_alone_blocks_narrow_benchmark',True)}
 proof=[]
 for name,change in mutations.items():
  mutant=copy.deepcopy(data);change(mutant)
  try:validate(mutant,plan)
  except AssertionError as e:proof.append({'mutation':name,'rejected':True,'reason':str(e)})
  else:raise AssertionError('accepted semantic mutation '+name)
 write(OUT/'audit.json',{'pass':True,'checks':checks,'core_rows':1150,'camera_windows':36,'recordings':7,
   'direct_pins':len(plan['input_sha256']),'inherited_pins':len(plan['inherited_sha256']),
   'semantic_mutations':proof,'inventory_sha256':sha(OUT/'inventory.json'),'new_native_execution':False})
 print('PASS',checks,'checks;',len(proof),'semantic mutations rejected')

if __name__=='__main__':audit()
