"""Unchanged fixed diagnostics on all saved EXP112 prefixes; retrospective context."""
from pathlib import Path
import sys,json,hashlib
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/geometry-support-transfer/run-001/prior-context';OLD=ROOT/'results/sensor-admission-execution/run-002'
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.geometry_support_controls import health_features,augment,evaluate

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,x):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(x,f,indent=2,allow_nan=False);f.write('\n')

def predict():
 p=read(ROOT/'experiments/sensor-admission-execution-2026-11-02-compiler.json');actions=read(OLD/'locked-actions.json')['rows'];prefix=read(OLD/'fresh-prefix-predictions.json')['rows'];lookup={(r['window'],r['pair_ordinal']):r for r in prefix};out=[];pins={str((OLD/n).relative_to(ROOT)):sha(OLD/n) for n in ['locked-actions.json','fresh-prefix-predictions.json','scores.json','fresh-labels.json']}
 for c in p['cases']:
  path=OLD/'captures'/c['case']/'on/timeline.jsonl';pins[str(path.relative_to(ROOT))]=sha(path);timeline=[json.loads(x) for x in path.read_text().splitlines()]
  for r in [r for r in actions if r['window']==c['window']]:out.append(augment(r,health_features(lookup[r['window'],r['pair_ordinal']],timeline)))
 assert len(out)==274
 write(OUT/'locked-actions.json',{'rows':out,'labels_read':False,'retrospective_known_labels':True})
 write(OUT/'lock.json',{'input_sha256':pins,'actions_sha256':sha(OUT/'locked-actions.json'),'code_sha256':sha(__file__),'control_sha256':sha(ROOT/'MAC-VO-main/Scripts/Setup/geometry_support_controls.py'),'scope':'posthoc retrospective context; no replacement primary'})
 print('LOCKED',len(out),'prior-cohort diagnostic actions')

def score():
 lock=read(OUT/'lock.json');assert lock['actions_sha256']==sha(OUT/'locked-actions.json')
 for n,h in lock['input_sha256'].items():assert sha(ROOT/n)==h
 rows=read(OUT/'locked-actions.json')['rows'];labels=read(OLD/'fresh-labels.json');result={}
 for record in dict.fromkeys(r['recording_id'] for r in rows):
  selected=[r for r in rows if r['recording_id']==record];result[record]=evaluate(selected,{r['sample_id']:labels[r['sample_id']] for r in selected})
 write(OUT/'scores.json',{'records':result,'retrospective_context_only':True,'replaces_primary':False,'control_thresholds_changed':False})
 print(json.dumps({r:{'both':s['groups']['both_supported']['population'],'methods':{m:dict(n=v['scored'],T=v['translation_m'],R=v['rotation_deg']) for m,v in s['groups']['both_supported']['methods'].items()}} for r,s in result.items()},indent=2))

if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['predict','score']);a=p.parse_args();globals()[a.action]()
