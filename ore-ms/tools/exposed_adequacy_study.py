"""EXP127 natural dense availability and source-backed target/evidence inventory."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/exposed-data-adequacy/run-001'
EVIDENCE=ROOT/'results/exposed-data-adequacy/evidence-001'
PLAN=ROOT/'experiments/exposed-data-adequacy-2026-11-03.json'
FIXTURE=ROOT/'experiments/guarded-probe-cache-2026-11-03-fixture-v2.json'
TRUTH=ROOT/'results/guarded-probe-cache/run-002/guarded-native-truth.json'
DOCS=['evaluation-readiness-2026-10-30.md','rig-reference-contract-2026-10-30.md','source-replies-2026-10-22.md',
      'recording-exposure-ledger-2026-10-31.md','ms2-temporal-adapter-findings-2026-11-01.md',
      'ms2-broader-range-findings-2026-11-02.md','geometry-support-transfer-findings-2026-11-02.md',
      'prospective-state-shadow-readiness-2026-11-01.md','cats-intake-2026-10-28.md']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def journal(p):return [json.loads(l) for l in Path(p).read_text().splitlines()]


def freeze():
 assert not PLAN.exists() and not OUT.exists()
 fixture=read(FIXTURE);pins={str(FIXTURE.relative_to(ROOT)):sha(FIXTURE),str(TRUTH.relative_to(ROOT)):sha(TRUTH)}
 for n,h in fixture['input_source_sha256'].items():assert sha(ROOT/n)==h,n
 windows=[]
 for s in fixture['sources']:
  for c in s['cases']:
   log=ROOT/s['base']/'captures'/c['case']/'on/timeline.jsonl';pins[str(log.relative_to(ROOT))]=sha(log)
   windows.append({'window':s['tag']+'/'+c['window'],'recording_id':c['recording_id'],
    'domain':'ms2' if c['recording_id'].startswith('ms2/') else 'td3',
    'timeline':str(log.relative_to(ROOT)),'frames':c['frames'],'rows':c['rows']})
 for name in DOCS:
  path='docs/research/'+name;pins[path]=sha(ROOT/path)
 for path in ['experiments/recording-exposure-ledger-2026-10-31.json',
   'experiments/prospective-state-shadow-evaluation-2026-11-01-adapter-v2.json',
   'tools/exposed_adequacy_study.py','tools/exposed_adequacy_audit.py',
   'MAC-VO-main/Scripts/UnitTest/test_exposed_adequacy.py','docs/research/exposed-data-adequacy-protocol-2026-11-03.md']:
  pins[path]=sha(ROOT/path)
 for p in EVIDENCE.rglob('*'):
  if p.is_file():pins[str(p.relative_to(ROOT))]=sha(p)
 assert len(windows)==18 and len({w['recording_id'] for w in windows})==7
 assert sum(w['frames']-5 for w in windows)==1150
 plan={'date':'2026-11-03','parent_commit':'8e7ac9588f709814ea43874a55219ed478654240',
  'windows':windows,'core_rows':1150,'camera_windows':36,'recordings':7,'minimum_health_run_pairs':3,
  'input_sha256':pins,'inherited_sha256':fixture['input_source_sha256'],
  'retrospective':True,'new_native_execution':False,'reserved_payload_access':False,'new_policy':False,
  'physical_accuracy_claim':False,'training':False,'messages_sent':False,'policy_adopted':False}
 write(PLAN,plan);write(OUT/'plan.json',plan)
 print('FROZEN',len(pins),'direct pins;',len(windows),'windows;1150core pairs;7records')


def verify():
 plan=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for mapping in [plan['input_sha256'],plan['inherited_sha256']]:
  for path,digest in mapping.items():assert sha(ROOT/path)==digest,path
 return plan


def runs(values):
 result=[]
 for i,value in enumerate(values):
  if result and result[-1]['supported']==value:result[-1]['end_pair']=i
  else:result.append({'start_pair':i,'end_pair':i,'supported':value})
 return result


def native_pair(rows,log,pair,camera,deadline):
 times=[rows[i][camera+'_left']['time_ns'] for i in [pair,pair+1]]
 hs=[e for e in log if e['kind']=='observer_event' and e['camera']==camera and e['event']['kind']=='pair_end' and
     [e['event']['time0_ns'],e['event']['time1_ns']]==times]
 ms=[e for e in log if e['kind']=='observer_event' and e['camera']==camera and e['event']['kind']=='motion_result_written' and
     e['event']['first_write_for_submission'] and e['event']['time1_ns']==times[1]]
 assert len(hs)==1 and len(ms)<=1
 h=hs[0];m=ms[0] if ms else None;hv=h['event'];supported=hv['original_need_interp'] is False and hv['execution_error'] is None
 eligible=bool(supported and m and h['controller_monotonic_ns']<=deadline and m['controller_monotonic_ns']<=deadline and
   m['event']['stage']=='pair_processing' and [m['event']['time0_ns'],m['event']['time1_ns']]==times and
   m['event']['original_need_interp'] is False and m['event']['derived_camera_delta_xyzw'] is not None)
 return {'supported':supported,'eligible':eligible,'native_interval_ns':times,'health_event_sequence':hv['sequence'],
   'motion_event_sequence':m['event']['sequence'] if m else None,'health_receipt_ns':h['controller_monotonic_ns'],
   'motion_receipt_ns':m['controller_monotonic_ns'] if m else None,'write_stage':m['event']['stage'] if m else None}


def count(rows):
 masks=collections.Counter(r['mask'] for r in rows);R=masks['rgb_only']+masks['both'];T=masks['thermal_only']+masks['both'];U=len(rows)-masks['none']
 return {'slots':len(rows),'masks':{k:masks[k] for k in ['none','rgb_only','thermal_only','both']},
   'rgb_eligible':R,'thermal_eligible':T,'union_eligible':U,'headroom_above_best_constant':U-max(R,T)}


def reference_inventory():
 rig_path=next(EVIDENCE.glob('**/rig-reference-contract/run-001/contract.json'));rig=read(rig_path)
 ledger=read(ROOT/'experiments/recording-exposure-ledger-2026-10-31.json')
 adapter=read(ROOT/'experiments/prospective-state-shadow-evaluation-2026-11-01-adapter-v2.json')
 box=read(EVIDENCE/'box-root.json');archives=sorted(x['Name'] for x in box if not x['IsDir'] and x['Name'].endswith('.zip'))
 mapped=[]
 for name in archives:
  records=[r for r in ledger['records'] if name in r['aliases']];assert len(records)==1 and records[0]['exposure_status']=='development_exposed'
  mapped.append({'archive':name,'recording_id':records[0]['id'],'exposure_status':records[0]['exposure_status']})
 assert len(mapped)==9
 return {'td3_rig_contract':{k:rig[k] for k in ['gps_frame_in_supplied_graph','gps_equals_imu_assumed',
    'physical_body_transform_verified','independent_ground_truth','training_membership']},
   'td3_rig_evidence':str(rig_path.relative_to(ROOT)),
   'ms2_adapter_status':adapter['status'],'training_membership':ledger['checkpoint']['training_membership'],
   'physical_quality_target_established_by_inspected_evidence':False,
   'retrospective_provided_reference_comparison':'reproducible under declared source/frame/time assumptions',
   'new_prospective_assignment':adapter['partitions']['test'],
   'unknown_training_alone_blocks_narrow_benchmark':False,
   'protected_records':[r['id'] for r in ledger['records'] if r['exposure_status']=='reported_reserved'],
   'current_box_td3_archives':mapped,'box_metadata_scope':'root and selected existing recovery metadata only; not an exhaustive evidence census',
   'cats_role':'static stereo evidence; no temporal motion validation',
   'reference_limits':['TD3 recorded SLAM is not independent physical truth',
      'GNSS signed axes/origin/timestamps and interval quality remain unresolved in inspected evidence',
      'MS2 provided references/calibration/timestamps support qualified comparisons, not unqualified physical truth',
      'MS2 gross invalid reference intervals were retained as exclusions in the prior study',
      'native temporal preservation verified; later reference/exposure/input qualification remains distinct']}


def analyze():
 plan=verify();truth=read(TRUTH);ledger=[];window_counts={};health_windows=[];identities=set()
 for window in plan['windows']:
  raw=window['rows'];log=journal(ROOT/window['timeline']);dec={e['pair_ordinal']:e for e in log if e['kind']=='decision'};rr=[]
  for i in range(2,len(raw)-3):
   d=dec[i];B=d['controller_monotonic_ns'];assert d['boundary']=={'kind':'group_complete','group':i+2}
   completions=[e for e in log if e['kind']=='completed' and e['frame']==i+2]
   assert len(completions)==2 and max(e['controller_monotonic_ns'] for e in completions)<=B
   assert not any(e['kind']=='release' and e['frame']>i+2 and e['controller_monotonic_ns']<=B for e in log)
   experts={c:native_pair(raw,log,i,c,B) for c in ['rgb','thermal']};mask=[c for c in experts if experts[c]['eligible']]
   sample=window['window']+'/'+str(i)
   for c in experts:assert experts[c]['eligible']==d['availability'][c]==truth[sample]['camera_local'][c]
   identity=(window['recording_id'],*(v for c in ['rgb','thermal'] for v in experts[c]['native_interval_ns']))
   assert identity not in identities;identities.add(identity)
   row={'window':window['window'],'recording_id':window['recording_id'],'domain':window['domain'],'pair':i,'deadline_ns':B,
    'experts':experts,'mask':'both' if len(mask)==2 else mask[0]+'_only' if mask else 'none',
    'projected_eligibility':truth[sample]['projected']}
   rr.append(row);ledger.append(row)
  window_counts[window['window']]=dict(count(rr),recording_id=window['recording_id'],domain=window['domain'])
  for c in ['rgb','thermal']:
   health={e['event']['time1_ns']:e['event'] for e in log if e['kind']=='observer_event' and e['camera']==c and e['event']['kind']=='pair_end'}
   values=[health[r[c+'_left']['time_ns']]['original_need_interp'] is False and health[r[c+'_left']['time_ns']]['execution_error'] is None for r in raw[1:]]
   seq=runs(values);cycles=[];returns=[]
   for a,b in zip(seq,seq[1:]):
    if not a['supported'] and b['supported']:returns.append({'failed_run':a,'returned_run':b})
   for a,b,d in zip(seq,seq[1:],seq[2:]):
    if a['supported'] and not b['supported'] and d['supported']:
     sustained=all(x['end_pair']-x['start_pair']+1>=3 for x in [a,b,d])
     core=[r for r in rr if b['start_pair']<=r['pair']<=b['end_pair']];other='thermal' if c=='rgb' else 'rgb'
     cycles.append({'healthy_run':a,'failed_run':b,'returned_run':d,'sustained':sustained,
      'failure_core_pairs':[r['pair'] for r in core],
      'alternate_eligible_all_failure_core':bool(core) and all(r['experts'][other]['eligible'] for r in core),
      'failure_fully_core':len(core)==b['end_pair']-b['start_pair']+1})
   health_windows.append({'window':window['window'],'recording_id':window['recording_id'],'camera':c,
      'runs':seq,'false_to_true':returns,'healthy_failed_returned_cycles':cycles})
 assert len(ledger)==1150 and len(health_windows)==36
 records={rec:count([r for r in ledger if r['recording_id']==rec]) for rec in sorted({r['recording_id'] for r in ledger})}
 write(OUT/'inventory.json',{'windows':window_counts,'recordings':records,'rows':ledger,'health_windows':health_windows,
   'reference_inventory':reference_inventory(),'retrospective':True,'new_native_execution':False,'reserved_payload_access':False,
   'new_policy':False,'physical_accuracy_claim':False,'messages_sent':False,'policy_adopted':False})
 write(OUT/'analysis-lock.json',{'inventory_sha256':sha(OUT/'inventory.json'),'plan_sha256':sha(PLAN),'source_sha256':sha(__file__)})
 for key,g in records.items():print(key,g)
 print('sustained cycles',sum(c['sustained'] for w in health_windows for c in w['healthy_failed_returned_cycles']))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','verify','analyze']);a=p.parse_args()
 if a.action=='verify':print('VERIFIED',len(verify()['input_sha256']),'direct pins')
 else:globals()[a.action]()
