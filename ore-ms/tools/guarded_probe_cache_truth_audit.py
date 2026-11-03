"""Native guard/truth reconstruction and explicitly secondary recovery evidence."""
import json,collections
from guarded_probe_cache import ROOT,OUT,read,write,sha,verify

def main():
 plan=verify();targets=read(OUT/'guarded-native-truth.json');outputs=read(OUT/'locked-schedules.json')['windows_policies'];checks=0;core=0;proof=0;recovery=[]
 for s in plan['sources']:
  prefix={(r['window'],r['pair_ordinal']):r for r in read(ROOT/s['base']/s['prefix'])['rows']}
  for c in s['cases']:
   window=s['tag']+'/'+c['window'];raw=c['rows'];log=[json.loads(x) for x in (ROOT/s['base']/'captures'/c['case']/'on/timeline.jsonl').read_text().splitlines()];dec={e['pair_ordinal']:e for e in log if e['kind']=='decision'};native={(e['camera'],e['event']['sequence']):e for e in log if e['kind']=='observer_event'}
   for i in range(len(raw)-1):
    t=targets[window+'/'+str(i)];p=prefix[c['window'],i];scored=2<=i<=len(raw)-4;assert t['scored_core']==scored;core+=int(scored)
    if scored:assert dec[i]['boundary']=={'kind':'group_complete','group':i+2}
    for cam in ['rgb','thermal']:
     a,b=raw[i][cam+'_left']['time_ns'],raw[i+1][cam+'_left']['time_ns'];health,=[e['event'] for e in log if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']=='pair_end' and [e['event']['time0_ns'],e['event']['time1_ns']]==[a,b]];assert t['tracking'][cam]==(health['original_need_interp'] is False and health['execution_error'] is None);assert t['camera_local'][cam]==dec[i]['availability'][cam];assert t['projected'][cam]==bool(p['experts'][cam]['available_by_prefix'] and p['experts'][cam]['valid_model_projection']);checks+=4
     if scored and t['projected'][cam]:
      m,=[e for e in log if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']=='motion_result_written' and e['event']['first_write_for_submission'] and [e['event']['time0_ns'],e['event']['time1_ns']]==[a,b] and e['controller_monotonic_ns']<=dec[i]['controller_monotonic_ns']];assert m['event']['stage']=='pair_processing';proof+=1
   # Native-local False->True runs; fresh evidence inside a run is distinct from routing recovery.
   for cam in ['rgb','thermal']:
    starts=[i for i in range(3,len(raw)-3) if targets[window+'/'+str(i)]['camera_local'][cam] and not targets[window+'/'+str(i-1)]['camera_local'][cam]]
    for start in starts:
     end=start
     while end+1<=len(raw)-4 and targets[window+'/'+str(end+1)]['camera_local'][cam]:end+=1
     for o in [o for o in outputs if o['window']==window]:
      accepted=[e for e in o['accepted'] if e['camera']==cam];health={};motion={}
      for a in accepted:
       e=native[cam,a['event_sequence']]['event']
       if a['kind']=='pair_end':health[a['pair']]=(not e['original_need_interp'] and e['execution_error'] is None,a['receipt_ns'])
       else:motion[a['pair']]=(e['stage']=='pair_processing' and e['time0_ns']==raw[a['pair']][cam+'_left']['time_ns'] and e.get('original_need_interp') is False and e.get('derived_camera_delta_xyzw') is not None,a['receipt_ns'])
      captured=[i for i in range(start,end+1) if i in health and i in motion and health[i][0] and motion[i][0] and max(health[i][1],motion[i][1])<=dec[end]['controller_monotonic_ns']]
      preceding=start-1
      while preceding-1>=2 and not targets[window+'/'+str(preceding-1)]['camera_local'][cam]:preceding-=1
      known_failure=any(i in health and not health[i][0] for i in range(preceding,start)) or any(i in health and i in motion and health[i][0] and not motion[i][0] for i in range(preceding,start))
      recovery.append({'window':window,'domain':o['domain'],'mode':o['mode'],'camera':cam,'start_pair':start,'end_pair':end,'preceding_failure_evidence_captured':known_failure,'fresh_return_evidence_captured':bool(captured),'known_failure_and_return_captured':known_failure and bool(captured),'routing_recovery_validated':False})
 assert core==1150
 grouped={}
 for d in ['td3','ms2']:
  grouped[d]={}
  for mode in plan['policies']:
   r=[r for r in recovery if r['domain']==d and r['mode']==mode];grouped[d][mode]={'native_local_recovery_runs':len(r),'preceding_failure_evidence_captured':sum(x['preceding_failure_evidence_captured'] for x in r),'fresh_return_evidence_captured':sum(x['fresh_return_evidence_captured'] for x in r),'known_failure_and_return_captured':sum(x['known_failure_and_return_captured'] for x in r)}
 verify();write(OUT/'native-truth-audit.json',{'status':'PASS_INDEPENDENT_NATIVE_GUARD_TRUTH_AUDIT','checks':checks,'core_pairs':core,'eligible_processing_stage_write_bindings':proof,'source_sha256':sha(__file__),'truth_sha256':sha(OUT/'guarded-native-truth.json')});write(OUT/'recovery-evidence-diagnostic.json',{'scope':'secondary dense-native evidence coverage, not routing or native sparse recovery','summary':grouped,'runs':recovery,'native_sparse_recovery_validated':False});print('TRUTH',checks,'checks;',core,'corepairs;',proof,'processingwritebindings;recovery',grouped)
if __name__=='__main__':main()
