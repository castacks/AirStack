"""Independent fixed-control reconstruction and scoring on prior exposed context."""
from pathlib import Path
import json,hashlib,math
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/geometry-support-transfer/run-001/prior-context';OLD=ROOT/'results/sensor-admission-execution/run-002'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
 lock=read(OUT/'lock.json');assert lock['actions_sha256']==sha(OUT/'locked-actions.json')
 for n,h in lock['input_sha256'].items():assert sha(ROOT/n)==h
 old_completion=read(OLD/'completion.json');assert old_completion['artifact_sha256']['admission-inputs.json']==sha(OLD/'admission-inputs.json')
 rows=read(OUT/'locked-actions.json')['rows'];scores=read(OUT/'scores.json');labels=read(OLD/'fresh-labels.json');prefix={(r['window'],r['pair_ordinal']):r for r in read(OLD/'fresh-prefix-predictions.json')['rows']};old={r['sample_id']:r for r in read(OLD/'locked-actions.json')['rows']};inputs={r['sample_id']:r for r in read(OLD/'admission-inputs.json')['rows']};plan=read(ROOT/'experiments/sensor-admission-execution-2026-11-02-compiler.json');timelines={c['window']:[json.loads(s) for s in (OLD/'captures'/c['case']/'on/timeline.jsonl').read_text().splitlines()] for c in plan['cases']};checks=0;D=np.diag([.1]*3+[np.pi/180]*3)
 for r in rows:
  original=old[r['sample_id']];p=prefix[r['window'],r['pair_ordinal']];timeline=timelines[r['window']];available=original['native_available'];features={};minimum={}
  for camera in ['rgb','thermal']:
   if not available[camera]:assert r['support_features'][camera] is None;features[camera]=None;minimum[camera]=None;continue
   events={x['event']['sequence']:x['event'] for x in timeline if x['kind']=='observer_event' and x['camera']==camera and x['controller_monotonic_ns']<=p['controller_deadline_ns']};source=p['experts'][camera]['source'];h=events[source['health_event_sequence']];m=events[source['motion_event_sequence']];g,=[e for e in events.values() if e['kind']=='native_disparity_information' and e['submission_id']==m['submission_id']]
   expected={'retained_count':h['retained_count'],'occupied_cells':h['occupied_3x3_cells'],'retained_fraction':h['retained_count']/max(h['selected_count'],1),'native_factors':g['information']['factors'],'health_sequence':h['sequence'],'motion_sequence':m['sequence'],'geometry_sequence':g['sequence']};assert expected==r['support_features'][camera];features[camera]=expected;checks+=7
   H=np.asarray(inputs[r['sample_id']]['cameras'][camera]['information']['matrix']);minimum[camera]=float(np.linalg.eigvalsh(D@H@D)[0])
  assert r['both_supported']==all(available.values());assert r['actions']['geometry_admission']==original['actions']['geometry_current'];assert r['actions']['recovery']==original['actions']['candidate']
  for method in ['raw','history','rgb_only','thermal_only']:assert r['actions'][method]==original['actions'][method]
  for method in ['retained_count','occupied_cells','retained_fraction','information_rank','information_per_factor']:
   possible=[c for c in ['rgb','thermal'] if available[c]]
   def score(c):
    if method=='information_rank':return minimum[c]
    if method=='information_per_factor':return minimum[c]/features[c]['native_factors']
    return features[c][method]
   selected=max(possible,key=lambda c:(score(c),c=='rgb')) if possible else None;assert r['actions'][method]==(selected.upper() if selected else 'ABSTAIN');checks+=1
 for record,result in scores['records'].items():
  chosen=[r for r in rows if r['recording_id']==record]
  for name,g in result['groups'].items():
   population=chosen if name=='all' else [r for r in chosen if r['both_supported']];assert g['ids']==[r['sample_id'] for r in population]
   def stats(stat,selected,method):
    nonlocal checks
    valid=[r for r in selected if r['actions'][method]!='ABSTAIN'];assert stat['ids']==[r['sample_id'] for r in valid] and stat['scored']==len(valid)
    for metric in ['translation_m','rotation_deg']:
     value=math.sqrt(math.fsum(labels[r['sample_id']][r['actions'][method].lower()][metric]**2 for r in valid)/len(valid)) if valid else None
     if value is None:assert stat[metric] is None
     else:assert math.isclose(value,stat[metric],rel_tol=1e-12,abs_tol=1e-13)
     checks+=1
   for method,stat in g['methods'].items():stats(stat,population,method)
   for method,pair in g['matched'].items():
    common=[r for r in population if r['actions']['geometry_admission']!='ABSTAIN' and r['actions'][method]!='ABSTAIN'];assert pair['ids']==[r['sample_id'] for r in common];stats(pair['geometry_admission'],common,'geometry_admission');stats(pair[method],common,method)
 for n,h in lock['input_sha256'].items():assert sha(ROOT/n)==h
 out={'status':'PASS_INDEPENDENT_PRIOR_CONTEXT_AUDIT','checks':checks,'decisions':len(rows),'both_supported':sum(r['both_supported'] for r in rows),'scores_sha256':sha(OUT/'scores.json'),'auditor_sha256':sha(__file__),'retrospective_context_only':True,'replaces_primary':False}
 with (OUT/'audit.json').open('x') as f:json.dump(out,f,indent=2);f.write('\n')
 print(out)

if __name__=='__main__':main()
