"""Independent proposal/error/decision/cost auditor, no runner imports."""
from pathlib import Path
import json,hashlib,math
import numpy as np
ROOT=Path(__file__).resolve().parents[1];checks=0

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(v,label):
 global checks
 assert v,label
 checks+=1

def angle(r):
 v=np.array([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])/2
 return float(np.degrees(np.arctan2(np.linalg.norm(v),(np.trace(r)-1)/2)))
def main(config):
 cfg=read(config);out=ROOT/cfg['output'];parent=ROOT/cfg['qualification'];freeze=read(out/'execution-freeze.json');plock=read(out/'prediction-lock.json');dlock=read(out/'decision-lock.json');completion=read(out/'completion.json')
 for p,h in freeze['pins'].items():check(sha(ROOT/p)==h,'frozen '+p)
 for name,key,lock in [('predictions.jsonl','predictions_sha256',plock),('decisions.json','decisions_sha256',dlock),('scores.json','scores_sha256',completion),('summary.json','summary_sha256',completion)]:check(sha(out/name)==lock[key],'locked '+name)
 check(not dlock['reference_errors_used_for_selection'] and not plock['scoring_started'] and not dlock['scoring_started'],'no score-based routing');check(plock['model_states_unchanged'] and read(out/'qualification.json')['model_states_unchanged'],'models unchanged')
 raw=[json.loads(s) for s in (out/'predictions.jsonl').read_text().splitlines()];pred={(r['frontend'],r['arm'],r['index']):r for r in raw};scored=read(out/'scores.json');scores={(r['frontend'],r['arm'],r['index']):r for r in scored};intervals=read(parent/'intervals.json');refs={(r['arm'],r['index']):r for r in intervals};inputs=read(out/'matcher-inputs.json');cal=read(parent/'calibration.json');population=read(parent/'population.json');common=set(population['common_reference']);trans=set(population['translation_ge1cm_both']);n=read(out/'input-freeze.json')['anchors'];expected={(f,a,i) for f in ['sift','learned'] for a in ['rgb','thermal'] for i in range(n)}
 check(len(raw)==len(pred)==len(scored)==len(scores)==4*n and set(pred)==set(scores)==expected,'all prediction/score rows');check(len(inputs)==2*n and {(r['arm'],r['index']) for r in inputs}==set(refs),'all matcher inputs')
 for r in inputs:check(set(r)=={'arm','index','anchor_ns','source','target','times_ns','qualified'} and all(r[k]==refs[r['arm'],r['index']][k] for k in r),'reference-free exact matcher binding')
 recomputed={}
 for key,p in pred.items():
  f,a,i=key;ref=refs[a,i];s=scores[key];check(p['qualified']==ref['qualified'] and (p['status']=='input_unsupported')==(not ref['qualified']),'unsupported input preserved');check(math.isfinite(p['seconds']) and p['seconds']>=0,'finite timing');e=et=None;flags={'R1':False,'R5':False,'joint':False}
  if p['status']=='ok':
   r=np.array(p['R']);v=np.array(p['t']);check(r.shape==(3,3) and v.shape==(3,) and np.isfinite(r).all() and np.isfinite(v).all() and np.linalg.norm(r.T@r-np.eye(3))<1e-6 and abs(np.linalg.det(r)-1)<1e-6 and abs(np.linalg.norm(v)-1)<1e-6,'proper proposal');check(p['cheirality_inliers']>=8 and np.count_nonzero(p['pose_mask'])==p['cheirality_inliers'],'unchanged validity gate')
   if i in common:
    truth=np.array(ref['target_from_source']);e=angle(r@truth[:3,:3].T);flags.update(R1=e<=1,R5=e<=5)
    if i in trans:
     t=truth[:3,3];et=float(np.degrees(np.arctan2(np.linalg.norm(np.cross(v,t)),v@t)));flags['joint']=bool(e<=5 and et<=15)
  check(s['R_error'] is None if e is None else abs(s['R_error']-e)<1e-6,'independent rotation error');check(s['t_error'] is None if et is None else abs(s['t_error']-et)<1e-6,'independent directed translation error');check(all(s[k]==v for k,v in flags.items()),'all success flags');check(s['reference_rotation_deg']==ref.get('reference_rotation_deg'),'reference angle binding');recomputed[key]={'R_error':e,'t_error':et,**flags}
 decisions=read(out/'decisions.json');dmap={(d['frontend'],d['index']):d for d in decisions};check(len(decisions)==len(dmap)==2*n,'all decisions')
 for (f,i),d in dmap.items():
  rgb,thermal=[pred[f,a,i] for a in ['rgb','thermal']];request=rgb['status']!='ok' and refs['thermal',i]['qualified'];selected='rgb' if rgb['status']=='ok' else 'thermal' if request and thermal['status']=='ok' else None;check(d['thermal_requested']==request and d['policies']=={'rgb':'rgb' if rgb['status']=='ok' else None,'thermal':'thermal' if thermal['status']=='ok' else None,'rgb_first':selected},'independent status routing')
  if selected is None:check(d['selected_proposal'] is None,'abstention');continue
  proposal=d['selected_proposal'];meta=refs[selected,i];p=pred[f,selected,i];rc=np.array(cal[selected]['effective_camera_to_imu'])[:3,:3];r=np.array(proposal['R_imu']);check(proposal['sensor']==selected and proposal['source']==meta['source'] and proposal['target']==meta['target'] and proposal['times_ns']==meta['times_ns'],'selected sensor/time binding');check(np.allclose(r,rc@np.array(p['R'])@rc.T,atol=1e-12),'camera/body rotation binding');check(proposal['translation_direction_camera_target']==p['t'] and proposal['metric_body_translation'] is None,'no metric lever-arm transport')
  if i in common:
   truth=np.array(meta['target_from_source'])[:3,:3];check(abs(angle(r@(rc@truth@rc.T).T)-recomputed[f,selected,i]['R_error'])<1e-6,'rotation-error conjugation invariance')
 cohorts={'all_declared':set(range(n)),'common_reference':common,'paired_rotation_ge2deg':set(population['paired_rotation_ge2deg']),'translation_ge1cm_both':trans,**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}};summary=read(out/'summary.json')
 for f in ['sift','learned']:
  for name,ids in cohorts.items():
   group=[dmap[f,i] for i in sorted(ids)];result=summary[f][name]
   for policy in ['rgb','thermal','rgb_first']:
    selected=[recomputed[f,d['policies'][policy],d['index']] for d in group if d['policies'][policy]];r=result[policy];check(r['anchors']==len(ids) and r['outputs']==len(selected) and r['rotation_eligible']==len(ids&common) and r['translation_eligible']==len(ids&trans),'summary denominators')
    for endpoint in ['R1','R5','joint']:check(r[endpoint]==sum(s[endpoint] for s in selected),'independent endpoint counts')
    for field,key in [('R_error','R_median'),('t_error','t_median')]:
     errors=[s[field] for s in selected if s[field] is not None];median=float(np.median(errors)) if errors else None;check(r[key] is None if median is None else abs(r[key]-median)<1e-6,'conditional errors')
    calls=0;times=[]
    for d in group:
     arms=([policy] if policy!='rgb_first' else ['rgb','thermal'] if d['thermal_requested'] else ['rgb'])
     for a in arms:p=pred[f,a,d['index']];calls+=p['qualified'];times.append(p['seconds'])
    check(r['matcher_calls']==calls and abs(r['projected_seconds']-math.fsum(times))<1e-8,'exact calls and projected time')
   added=[d for d in group if d['policies']['rgb_first']=='thermal'];r=result['added'];check(r['outputs']==len(added) and r['attempts']==sum(d['thermal_requested'] for d in group) and r['failed_attempts']==r['attempts']-len(added),'added attempts including failures')
   for endpoint,eligible in [('R1',common),('R5',common),('joint',trans)]:
    valid=[d for d in added if d['index'] in eligible];good=sum(recomputed[f,'thermal',d['index']][endpoint] for d in valid);union=sum(recomputed[f,'rgb',i][endpoint] or recomputed[f,'thermal',i][endpoint] for i in ids&eligible);check(r[endpoint]=={'eligible':len(valid),'correct':good,'incorrect':len(valid)-good,'hindsight_union':union},'correct/incorrect eligible additions and oracle');check(result['rgb_first'][endpoint]==result['rgb'][endpoint]+good,'monotonic success construction')
   for tolerance,field in [(1,'R1'),(5,'R5')]:
    count=sum(recomputed[f,'rgb',i][field] if pred[f,'rgb',i]['status']=='ok' else refs['rgb',i]['reference_rotation_deg']<=tolerance for i in ids&common);check(result['rgb_or_zero_rotation'][str(tolerance)]==count,'rotation-only identity control')
 result={'status':'pass','checks':checks,'prediction_rows':len(raw),'decision_rows':len(decisions),'auditor_sha256':sha(__file__),'scope':'Independent frozen input/row integrity, direct rotation and translation errors, status-only routing, output frame/times, every cohort/endpoint/identity/count/cost; no model inference, physical ground-truth or stateful recovery certification'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
 import sys
 main(Path(sys.argv[1]))
