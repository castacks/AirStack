"""Independent status-rule, frame, inherited-error, cost and cohort audit."""
import json,hashlib,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/smoke-rgb-first-fallback';PARENT=ROOT/'results/deployment-smoke-frontend';REF=ROOT/'results/smoke-reference-contract';checks=0

def read(p):return json.loads(Path(p).read_text())
def check(v,label):
 global checks
 assert v,label
 checks+=1

def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def angle(r):return math.degrees(math.acos(np.clip((np.trace(r)-1)/2,-1,1)))
def main():
 freeze=read(OUT/'freeze.json')
 for k,h in freeze['pins'].items():check(sha(ROOT/k)==h,'frozen '+k)
 decisions=read(OUT/'decisions.json');lock=read(OUT/'decision-lock.json');check(sha(OUT/'decisions.json')==lock['decisions_sha256'],'decision lock');check(not lock['reference_errors_used_for_selection'],'reference-blind selector declaration');completion=read(OUT/'completion.json');check(sha(OUT/'summary.json')==completion['summary_sha256'],'summary lock')
 raw={(r['frontend'],r['arm'],r['index']):r for r in (json.loads(x) for x in (PARENT/'predictions.jsonl').read_text().splitlines())};times={(r['arm'],r['index']):r for r in read(PARENT/'intervals.json')};scores={(r['frontend'],r['arm'],r['index']):r for r in read(REF/'pose-scores.json')};refs={(r['arm'],r['index']):r for r in read(REF/'camera-reference.json')};cal=read(REF/'calibration-normalization.json');seen=set()
 for d in decisions:
  f,i=d['frontend'],d['index'];check((f,i) not in seen,'unique decision');seen.add((f,i));rgb,thermal=raw[(f,'rgb',i)],raw[(f,'thermal',i)];attempt=rgb['status']!='ok' and times[('thermal',i)]['qualified'];chosen='rgb' if rgb['status']=='ok' else 'thermal' if attempt and thermal['status']=='ok' else None
  check(d['thermal_requested']==attempt and d['thermal_calls_fallback']==int(attempt),'lazy thermal call');check(d['thermal_status_if_requested']==(thermal['status'] if attempt else None),'unrequested thermal hidden from policy trace');check(d['policies']=={'fixed_rgb':'rgb' if rgb['status']=='ok' else None,'fixed_thermal':'thermal' if thermal['status']=='ok' else None,'rgb_first':chosen},'independent fixed policy')
  check(d['rgb_calls']==int(times[('rgb',i)]['qualified']) and d['thermal_calls_fixed']==int(times[('thermal',i)]['qualified']),'input-supported pair counts')
  for p,t in [('fixed_rgb',rgb['seconds']),('fixed_thermal',thermal['seconds']),('rgb_first',rgb['seconds']+(thermal['seconds'] if attempt else 0))]:check(abs(d['projected_seconds'][p]-t)<1e-12,'per-pair cost')
  proposal=d['selected_proposal']
  if chosen is None:check(proposal is None,'abstain has no fabricated proposal');continue
  selected=raw[(f,chosen,i)];meta=times[(chosen,i)];r=np.array(cal[chosen]['effective_camera_to_imu'])[:3,:3];body=np.array(proposal['R_imu_target_from_source']);expected=r@np.array(selected['R'])@r.T
  check(proposal['sensor']==chosen and proposal['source']==meta['source'] and proposal['target']==meta['target'] and proposal['exposure_times_ns']==meta['times_ns'],'chosen sensor/time/frame binding');check(np.allclose(body,expected,atol=1e-12),'camera to IMU rotation');check(proposal['translation_direction_sensor']==chosen and np.array_equal(proposal['translation_direction_camera_target'],selected['t']) and proposal['metric_body_translation'] is None,'no unknown-scale lever-arm transport')
  s=scores[(f,chosen,i)]
  if s['common_reference']:
   truth=np.array(refs[(chosen,i)]['target_from_source'])[:3,:3];body_truth=r@truth@r.T;check(abs(angle(body@body_truth.T)-s['rotation_error_deg'])<1e-5,'chosen camera/body error invariance')
 check(len(seen)==len(decisions)==1676,'all decisions');summary=read(OUT/'summary.json');population=read(REF/'pose-population.json');common=set(population['common_reference_indices']);trans=set(population['translation_ge1cm_both_indices']);moving=set(read(REF/'paired-moving-cohort.json')['indices']);cohorts={'all_declared':set(range(838)),'common_reference':common,'paired_rotation_ge2deg':moving,'translation_ge1cm_both':trans,**{'third'+str(j+1):{i for i in common if i*3//838==j} for j in range(3)}}
 fields=['rotation_success_1deg','rotation_success_5deg','joint_success_5deg_15deg']
 for f in ['sift','learned']:
  for name,ids in cohorts.items():
   group=[r for r in decisions if r['frontend']==f and r['index'] in ids];target=summary[f][name]
   for policy in ['fixed_rgb','fixed_thermal','rgb_first']:
    selected=[scores[(f,r['policies'][policy],r['index'])] if r['policies'][policy] else None for r in group];v=target[policy];check(v['anchors']==len(group) and v['outputs']==sum(x is not None for x in selected),'output denominator')
    for field in fields:check(v[field]==sum(x is not None and x[field] for x in selected),'success count '+field)
    for prefix,key in [('rotation','rotation_error_deg'),('translation','translation_error_deg')]:
     err=[x[key] for x in selected if x is not None and x[key] is not None];want=float(np.median(err)) if err else None;check(want==v['conditional_'+prefix+'_median_deg'],'conditional median')
    check(abs(v['projected_seconds']-math.fsum(r['projected_seconds'][policy] for r in group))<1e-8,'aggregate time projection');calls=sum(r['rgb_calls'] if policy=='fixed_rgb' else r['thermal_calls_fixed'] if policy=='fixed_thermal' else r['rgb_calls']+r['thermal_calls_fallback'] for r in group);check(calls==v['matcher_calls'],'exact pair call count')
   fallback=[r for r in group if r['policies']['rgb_first']=='thermal'];added=target['added_to_rgb'];check(added['thermal_attempts']==sum(r['thermal_requested'] for r in group) and added['thermal_outputs']==len(fallback),'attempt/output accounting');check(added['attempts_without_output']==sum(r['thermal_requested'] and r['policies']['rgb_first'] is None for r in group),'unsuccessful requests');check(added['no_output_either']==sum(r['policies']['rgb_first'] is None for r in group),'both absent')
   for field in fields:
    actual=sum(scores[(f,'thermal',r['index'])][field] for r in fallback);check(added[field]['new_correct']==actual and added[field]['new_incorrect']==len(fallback)-actual,'correct/incorrect additions');oracle=sum(scores[(f,'rgb',r['index'])][field] or scores[(f,'thermal',r['index'])][field] for r in group);check(added[field]['hindsight_either_success']==oracle,'oracle bound');check(target['rgb_first'][field]==target['fixed_rgb'][field]+actual,'RGB successes never replaced')
 result={'status':'pass','checks':checks,'decisions':len(decisions),'new_inference_calls':0,'source_sha256':sha(__file__),'scope':'Independent selection, frame/time binding, camera/body rotation-error invariance, costs/counts and all endpoints. Timing is projected; full stateful VO and metric translation not evaluated.'};(OUT/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
