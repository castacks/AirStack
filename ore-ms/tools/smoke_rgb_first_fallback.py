"""Fixed camera-tagged fallback replay; no inference and no ground-truth selection.

The older fallback study is a frozen state-feedback backend experiment. This
adapter instead consumes SmokeRoom temporal proposals, reuses pure preference,
and preserves each chosen sensor's frame and exposure endpoints.
"""
import argparse,hashlib,json,math,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];PARENT=ROOT/'results/deployment-smoke-frontend';REF=ROOT/'results/smoke-reference-contract';OUT=ROOT/'results/smoke-rgb-first-fallback'
sys.path.insert(0,str(ROOT/'MAC-VO-main/Scripts/Setup'))
from compare_factor_proxy import preference

def read(p):return json.loads(Path(p).read_text())
def write(p,v):
 with Path(p).open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def choose(rgb_status,thermal_input,thermal_status):
 request=rgb_status!='ok' and thermal_input
 selected,_=preference({'rgb':0.,'thermal':1.},{'rgb':rgb_status=='ok','thermal':request and thermal_status=='ok'})
 return (selected.lower() if selected!='ABSTAIN' else None),bool(request)
def verify():
 plan=read(OUT/'freeze.json')
 for k,h in plan['pins'].items():assert sha(ROOT/k)==h,k
 return plan

def qualify():
 OUT.mkdir(exist_ok=False);tests=0
 for rgb in ['ok','insufficient_cheirality','input_unsupported']:
  for ti in [False,True]:
   for thermal in ['ok','insufficient_cheirality','input_unsupported']:
    selected,request=choose(rgb,ti,thermal);expected='rgb' if rgb=='ok' else 'thermal' if ti and thermal=='ok' else None;assert selected==expected and request==(rgb!='ok' and ti);tests+=1
 for thermal in ['ok','invalid','input_unsupported']:
  assert choose('ok',True,thermal)==('rgb',False);tests+=1
 raw=[json.loads(line) for line in (PARENT/'predictions.jsonl').read_text().splitlines()];index={(r['frontend'],r['arm'],r['index']):r for r in raw};interval={(r['arm'],r['index']):r for r in read(PARENT/'intervals.json')};assert len(index)==3352 and len(interval)==1676
 for frontend in ['sift','learned']:
  for i in range(838):
   for arm in ['rgb','thermal']:
    r=index[(frontend,arm,i)];ref=interval[(arm,i)];assert math.isfinite(r['seconds']) and r['seconds']>=0
    assert (r['status']=='input_unsupported')==(not ref['qualified'])
    if r['status']=='ok':
     rotation=np.array(r['R']);t=np.array(r['t']);assert rotation.shape==(3,3) and t.shape==(3,) and np.isfinite(rotation).all() and np.isfinite(t).all();assert np.linalg.norm(rotation.T@rotation-np.eye(3))<1e-6 and abs(np.linalg.norm(t)-1)<1e-6
 parent=read(REF/'pose-freeze.json');comp=read(REF/'pose-completion.json');assert sha(REF/'pose-scores.json')==comp['scores_sha256'];assert sha(PARENT/'predictions.jsonl')==read(PARENT/'prediction-lock.json')['predictions_sha256']
 files=[Path(__file__).resolve(),ROOT/'MAC-VO-main/Scripts/Setup/compare_factor_proxy.py',ROOT/'docs/research/smoke-rgb-first-protocol-2026-11-16.md',PARENT/'predictions.jsonl',PARENT/'prediction-lock.json',PARENT/'intervals.json',REF/'pose-scores.json',REF/'pose-population.json',REF/'paired-moving-cohort.json',REF/'calibration-normalization.json',REF/'camera-reference.json',REF/'pose-freeze.json',REF/'pose-completion.json',REF/'pose-audit.json']
 pins={str(p.relative_to(ROOT)):sha(p) for p in files}
 for k,h in pins.items():
  if k in parent['pins']:assert h==parent['pins'][k]
 write(OUT/'qualification.json',{'synthetic_selector_checks':tests,'cached_sensor_rows':len(index),'anchors_per_frontend':838,'new_inference_calls':0,'selector_inputs':['rgb_status','thermal_input_supported','thermal_status_if_requested'],'reference_errors_are_selector_inputs':False})
 pins[str((OUT/'qualification.json').relative_to(ROOT))]=sha(OUT/'qualification.json');write(OUT/'freeze.json',{'pins':pins,'scope':'Exposed cached proposal baseline, selection before score join, projected cost only','new_inference_calls':0});print('Fallback qualified',tests,'selector checks',flush=True)

def select():
 verify();raw={(r['frontend'],r['arm'],r['index']):r for r in (json.loads(line) for line in (PARENT/'predictions.jsonl').read_text().splitlines())};interval={(r['arm'],r['index']):r for r in read(PARENT/'intervals.json')};cal=read(REF/'calibration-normalization.json');rows=[]
 # No score or reference-population file is read in this stage.
 for frontend in ['sift','learned']:
  for i in range(838):
   a,b=raw[(frontend,'rgb',i)],raw[(frontend,'thermal',i)];ref_t=interval[('thermal',i)];chosen,request=choose(a['status'],ref_t['qualified'],b['status']);policies={'fixed_rgb':'rgb' if a['status']=='ok' else None,'fixed_thermal':'thermal' if b['status']=='ok' else None,'rgb_first':chosen}
   record={'frontend':frontend,'index':i,'rgb_status':a['status'],'thermal_requested':request,'thermal_status_if_requested':b['status'] if request else None,'policies':policies,'rgb_calls':int(interval[('rgb',i)]['qualified']),'thermal_calls_fixed':int(ref_t['qualified']),'thermal_calls_fallback':int(request),'projected_seconds':{'fixed_rgb':a['seconds'],'fixed_thermal':b['seconds'],'rgb_first':a['seconds']+(b['seconds'] if request else 0)},'selected_proposal':None}
   if chosen is not None:
    p=raw[(frontend,chosen,i)];r=interval[(chosen,i)];rotation=np.array(cal[chosen]['effective_camera_to_imu'])[:3,:3];record['selected_proposal']={'sensor':chosen,'source':r['source'],'target':r['target'],'exposure_times_ns':r['times_ns'],'R_imu_target_from_source':(rotation@np.array(p['R'])@rotation.T).tolist(),'translation_direction_camera_target':p['t'],'translation_direction_sensor':chosen,'metric_body_translation':None}
   rows.append(record)
 write(OUT/'decisions.json',rows);write(OUT/'decision-lock.json',{'rows':len(rows),'decisions_sha256':sha(OUT/'decisions.json'),'freeze_sha256':sha(OUT/'freeze.json'),'score_join_started':False,'reference_errors_used_for_selection':False,'new_inference_calls':0});print('Decisions locked',len(rows),flush=True)

def score():
 verify();lock=read(OUT/'decision-lock.json');assert sha(OUT/'decisions.json')==lock['decisions_sha256'];decisions=read(OUT/'decisions.json');scores={(r['frontend'],r['arm'],r['index']):r for r in read(REF/'pose-scores.json')};pop=read(REF/'pose-population.json');common=set(pop['common_reference_indices']);translation=set(pop['translation_ge1cm_both_indices']);moving=set(read(REF/'paired-moving-cohort.json')['indices']);summary={}
 def aggregate(frontend,ids):
  d=[r for r in decisions if r['frontend']==frontend and r['index'] in ids];n=len(d);out={}
  for policy in ['fixed_rgb','fixed_thermal','rgb_first']:
   selected=[scores[(frontend,r['policies'][policy],r['index'])] if r['policies'][policy] else None for r in d];supported=[x for x in selected if x is not None];rr=[x['rotation_error_deg'] for x in supported if x['rotation_error_deg'] is not None];tt=[x['translation_error_deg'] for x in supported if x['translation_error_deg'] is not None];out[policy]={'anchors':n,'outputs':len(supported),'rotation_success_1deg':sum(x is not None and x['rotation_success_1deg'] for x in selected),'rotation_success_5deg':sum(x is not None and x['rotation_success_5deg'] for x in selected),'joint_success_5deg_15deg':sum(x is not None and x['joint_success_5deg_15deg'] for x in selected),'conditional_rotation_median_deg':float(np.median(rr)) if rr else None,'conditional_translation_median_deg':float(np.median(tt)) if tt else None,'projected_seconds':sum(r['projected_seconds'][policy] for r in d),'matcher_calls':sum(r['rgb_calls'] if policy=='fixed_rgb' else r['thermal_calls_fixed'] if policy=='fixed_thermal' else r['rgb_calls']+r['thermal_calls_fallback'] for r in d)}
  fallback=[r for r in d if r['policies']['rgb_first']=='thermal'];additions={'thermal_attempts':sum(r['thermal_requested'] for r in d),'thermal_outputs':len(fallback),'attempts_without_output':sum(r['thermal_requested'] and r['policies']['rgb_first'] is None for r in d),'no_output_either':sum(r['policies']['rgb_first'] is None for r in d)}
  for field in ['rotation_success_1deg','rotation_success_5deg','joint_success_5deg_15deg']:
   good=sum(scores[(frontend,'thermal',r['index'])][field] for r in fallback);additions[field]={'new_correct':good,'new_incorrect':len(fallback)-good,'hindsight_either_success':sum(scores[(frontend,'rgb',r['index'])][field] or scores[(frontend,'thermal',r['index'])][field] for r in d)};assert out['rgb_first'][field]==out['fixed_rgb'][field]+good
  out['added_to_rgb']=additions;return out
 cohorts={'all_declared':set(range(838)),'common_reference':common,'paired_rotation_ge2deg':moving,'translation_ge1cm_both':translation,**{'third'+str(j+1):{i for i in common if i*3//838==j} for j in range(3)}}
 for f in ['sift','learned']:summary[f]={name:aggregate(f,ids) for name,ids in cohorts.items()}
 write(OUT/'summary.json',summary);write(OUT/'completion.json',{'decision_rows':len(decisions),'new_inference_calls':0,'decision_lock_sha256':sha(OUT/'decision-lock.json'),'summary_sha256':sha(OUT/'summary.json'),'scope':'Sensor-tagged pair proposals, rotation in IMU axes; nominal selected-camera translation direction and projected timing only'});print(json.dumps(summary,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('stage',choices=['qualify','select','score']);a=p.parse_args();{'qualify':qualify,'select':select,'score':score}[a.stage]()
