"""Opt-in metadata replay; does not select experts or emit physical motion."""
from collections import Counter
from datetime import datetime,timezone
import hashlib,importlib.util,json
from pathlib import Path
from Scripts.Setup.predecessor_state import validate_predecessor

ROOT=Path('/root/AirStack/ore-ms');OUT=ROOT/'results/predecessor-state-validator/run-001'
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,v):
 with (OUT/n).open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def counts(rows):return dict(Counter(r['result']['reason'] for r in rows))


def main():
 inv=ROOT/'results/predecessor-evidence/run-001';spec=importlib.util.spec_from_file_location('frozen_inventory',inv/'inventory.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 cases,inputs=module.load_cases();inventory=read(inv/'inventory.json');proof=read(inv/'validation.json')
 assert proof['status']=='PASS_INDEPENDENT_PREDECESSOR_INVENTORY' and proof['inventory_sha256']==sha(inv/'inventory.json')
 for name in ('inventory.py','inventory.json','protocol.json','validation.json'):inputs[str((inv/name).relative_to(ROOT))]=sha(inv/name)
 indexed={(r['sample_id'],r['camera']):r for r in inventory['rows']}
 setup=Path(__file__).parent;protected={str((setup/n).relative_to(ROOT)):sha(setup/n) for n in ('arbiter_motion_output.py','arbiter_motion_chain.py','shared_arbiter_replay.py','arbiter_history.py','prefix_interval_model.py','temporal_quality_gate.py')}
 sources={str(p.relative_to(ROOT)):sha(p) for p in (Path(__file__),setup/'predecessor_state.py',setup.parent/'UnitTest/test_predecessor_state.py')}
 save('protocol.json',{'created_at':datetime.now(timezone.utc).isoformat(),'base_commit':'64a73b3dc32e69672426a84d6d071621ca546424',
  'image_id':'sha256:af124653ef9cd76660bb8a91f5ae609d6488d41fc78269fe03b3f484c5d35fd1','input_sha256':inputs,'source_sha256':sources,
  'protected_source_sha256':protected,'declaration_sha256':sha(OUT/'declaration.md'),'labels_read':False,'root_initialization_supplied':False,'production_defaults_changed':False})
 rows=[]
 for case in cases:
  journals={c:[] for c in ('rgb','thermal')};scope=case['prefix']+'/'+case['window']
  for item in case['timeline']:
   if item['kind']=='observer_event':
    c=item['camera'];journals[c].append({'scope_id':scope,'camera':c,'clock_id':'shared-controller','sequence':item['sequence'],'received_ns':item['controller_monotonic_ns'],'event':item['event']})
   elif item['kind']=='decision':
    pair=item['pair_ordinal'];sample=f'{scope}/{pair}'
    for c,journal in journals.items():
     interval=[case['rows'][i][c+'_left']['time_ns'] for i in (pair,pair+1)]
     candidates=[e for e in journal if e['event']['kind']=='motion_result_written' and e['event']['first_write_for_submission'] is True and e['event']['time1_ns']==interval[1]]
     child=candidates[0] if len(candidates)==1 and candidates[0]['event']['time0_ns']==interval[0] else None
     original=indexed[sample,c]
     assert (child is not None)==(original['candidate_status']=='EXACT_FIRST_WRITE')
     if child is None:result={'status':'NOT_CHECKED','reason':'no_unique_exact_child','physical_motion_admitted':False}
     else:
      assert child['sequence']==original['motion']['sequence']
      result=validate_predecessor(scope_id=scope,camera=c,clock_id='shared-controller',deadline_ns=item['controller_monotonic_ns'],child=child,observed_events=journal)
     rows.append({'sample_id':sample,'recording_id':case['recording_id'],'camera':c,'child_timeline_sequence':None if child is None else child['sequence'],
      'original_advice':item['action'],'original_available':original['current_available'],'result':result})
 assert len(rows)==548 and len({(r['sample_id'],r['camera']) for r in rows})==548
 assert all(r['result']['physical_motion_admitted'] is False for r in rows)
 for mapping in (inputs,sources,protected):assert all(sha(ROOT/n)==h for n,h in mapping.items())
 save('results.json',{'protocol_sha256':sha(OUT/'protocol.json'),'rows':rows,'counts':counts(rows),
  'recordings':{g:counts([r for r in rows if r['recording_id']==g]) for g in sorted({r['recording_id'] for r in rows})}})
 print(json.dumps({'counts':counts(rows),'slots':len(rows),'root_initialization_supplied':False},indent=2))

if __name__=='__main__':main()
