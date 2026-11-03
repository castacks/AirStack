"""Separate post-capture qualified proxy scoring, matched endpoints and call audit."""
import collections
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from native_gap_replay import ROOT,OUT,PLAN,read,write,sha,verify
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.td3_reference import load_reference,matrices


def loss(a,b):
 e=np.linalg.solve(b,a)
 return {'translation_m':float(np.linalg.norm(e[:3,3])),
         'rotation_deg':float(np.degrees(Rotation.from_matrix(e[:3,:3]).magnitude()))}

def stats(rows,key):
 values=np.asarray([r[key] for r in rows],dtype=float)
 return {'count':len(values),'rmse':float(np.sqrt(np.mean(values**2))) if len(values) else None,
         'mean':float(np.mean(values)) if len(values) else None,'max':float(np.max(values)) if len(values) else None}

def load_capture(c):
 base=ROOT/c['capture']/c['camera'];mapping=read(ROOT/c['mapping']);manifest=read(ROOT/c['manifest'])
 report=read(base/'report.json');assert report['status']=='PASS_EXECUTION_ONLY'
 assert report['input_rows']==manifest['pairs']
 with np.load(base/'trajectory.npz',allow_pickle=False) as x:
  poses=matrices(x['poses_camera_ned']);times=x['time_ns'].reshape(-1)
 with np.load(base/'tracking-flags.npz',allow_pickle=False) as x:flags=x['before_refinement'].reshape(-1).astype(bool)
 assert len(poses)==len(flags)==c['frames']==len(mapping['source_indices'])
 expected=[r[c['camera']+'_left']['time_ns'] for r in manifest['pairs']];assert times.tolist()==expected
 events=[json.loads(s) for s in (base/'arbiter-events/events.jsonl').read_text().splitlines()]
 health={(e['time0_ns'],e['time1_ns']):e for e in events if e['kind']=='pair_end'}
 writes={}
 for e in events:
  if e['kind']=='motion_result_written' and e['first_write_for_submission']:writes[e['time0_ns'],e['time1_ns']]=e
 native=[]
 for i in mapping['intervals']:
  interval=tuple(i['sensor_interval_ns']);h=health.get(interval);w=writes.get(interval)
  good=h is not None and h['execution_error'] is None and h['original_need_interp'] is False and not flags[i['native_pair']+1]
  motion=good and w is not None and w['original_need_interp'] is False and w['derived_camera_delta_xyzw'] is not None
  native.append(dict(i,health_supported=good,motion_written=motion,write_stage=w['stage'] if w else None,
                     first_write_event_sequence=w['sequence'] if w else None))
 cost=read(base/'native-cost.json');assert cost['status']=='PASS'
 runs=[x for x in cost['native_phases'] if x['kind']=='run'];assert len(runs)==c['frames']
 assert [r['sensor_time_ns'] for r in runs]==expected
 counts=collections.Counter(x['kind'] for x in cost['frontend_calls'])
 # MACVO performs one separate first-depth probe and one initialize depth call.
 assert counts['frontend_depth']==2 and counts['frontend_pair']==c['frames']-1
 assert all(x['status']=='PASS' and x['duration_ns']==x['end_ns']-x['start_ns'] and x['duration_ns']>=0 for x in cost['frontend_calls']+cost['native_phases'])
 return {'poses':poses,'times':times,'flags':flags,'intervals':native,'cost':{
   'run_seconds':sum(r['duration_ns'] for r in runs)/1e9,
   'frontend_depth_calls':counts['frontend_depth'],'frontend_pair_calls':counts['frontend_pair'],
   'host_seconds':cost['host_elapsed_ns']/1e9,
   'termination_seconds':sum(r['duration_ns'] for r in cost['native_phases'] if r['kind']=='terminate')/1e9},'mapping':mapping}

def repeat_comparison(a,b):
 aa={str(p.relative_to(a)):p for p in a.rglob('*.npz')};bb={str(p.relative_to(b)):p for p in b.rglob('*.npz')}
 assert aa.keys()==bb.keys();count=0;fail=[]
 for n in aa:
  with np.load(aa[n],allow_pickle=False) as x,np.load(bb[n],allow_pickle=False) as y:
   assert set(x.files)==set(y.files)
   for k in x.files:
    count+=1
    if x[k].dtype!=y[k].dtype or x[k].shape!=y[k].shape or not np.array_equal(x[k],y[k],equal_nan=True):fail.append({'file':n,'key':k})
 return {'compared_arrays':count,'all_arrays_exact':not fail,'mismatched_arrays':fail}

def main():
 plan=verify();lock=read(OUT/'capture-lock.json');assert lock['plan_sha256']==sha(PLAN)
 for n,h in lock['sha256'].items():assert sha(ROOT/n)==h,n
 # No model/policy decisions remain to make. Only now load the qualified references.
 for n,h in plan['scorer_only_reference_sha256'].items():assert sha(ROOT/n)==h,n
 captures={(c['window'],c['camera'],c['arm']):c for c in plan['cases']}
 groups={};rows=[];repeat={};refs={}
 for window,camera in sorted({(c['window'],c['camera']) for c in plan['cases']}):
  dense=load_capture(captures[window,camera,'dense']);sparse=load_capture(captures[window,camera,'sparse'])
  c=captures[window,camera,'sparse'];T=np.asarray(c['nominal_body_from_camera'][camera],float);Ti=np.linalg.inv(T)
  if c['reference_dataset'] not in refs:refs[c['reference_dataset']]=load_reference(ROOT/c['reference_dataset'])[0]
  reference=refs[c['reference_dataset']]
  group=[]
  for interval in sparse['intervals']:
   k=interval['native_pair'];a=interval['source_start'];b=interval['source_end'];dense_parts=dense['intervals'][a:b]
   sparse_ok=interval['motion_written'];dense_ok=all(x['motion_written'] for x in dense_parts)
   # Finalized endpoint estimates are descriptive; neither terminal nor later writes are online evidence.
   r=dict(window=window,camera=camera,**interval,dense_composite_supported=dense_ok,
          common_supported=bool(sparse_ok and dense_ok),scope='finalized native endpoint poses; qualified nominal-body recorded-SLAM proxy')
   r['dense_terminal_parts']=sum(x['write_stage']=='termination' for x in dense_parts)
   r['losses']=None
   if sparse_ok and dense_ok:
    assert dense['times'][a]==sparse['times'][k] and dense['times'][b]==sparse['times'][k+1]
    d=T@np.linalg.solve(dense['poses'][a],dense['poses'][b])@Ti
    s=T@np.linalg.solve(sparse['poses'][k],sparse['poses'][k+1])@Ti
    rr=reference.at(np.asarray(interval['sensor_interval_ns'],np.int64));target=np.linalg.solve(rr[0],rr[1])
    r['dense_body_delta']=d.tolist();r['sparse_body_delta']=s.tolist();r['reference_delta']=target.tolist()
    r['losses']={'dense_proxy':loss(d,target),'sparse_proxy':loss(s,target),'sparse_dense_disagreement':loss(s,d)}
   group.append(r);rows.append(r)
  summaries={}
  for label,selected in [('all',group),('gap',[r for r in group if not r['is_adjacent_source_pair']]),('adjacent',[r for r in group if r['is_adjacent_source_pair']])]:
   eligible=[r for r in selected if r['common_supported']]
   summaries[label]={'intervals':len(selected),'sparse_supported':sum(r['motion_written'] for r in selected),
     'dense_supported':sum(r['dense_composite_supported'] for r in selected),'common_supported':len(eligible),
     'sparse_first_write_stages':dict(collections.Counter(r['write_stage'] or 'unavailable' for r in selected)),
     'metrics':{method:{key:stats([r['losses'][method] for r in eligible],key) for key in ['translation_m','rotation_deg']}
                for method in ['dense_proxy','sparse_proxy','sparse_dense_disagreement']}}
  groups[window+'/'+camera]={'dense_frames':len(dense['poses']),'sparse_frames':len(sparse['poses']),
   'dense_cost':dense['cost'],'sparse_cost':sparse['cost'],'strata':summaries,
   'dense_health_supported':sum(x['health_supported'] for x in dense['intervals']),
   'sparse_health_supported':sum(x['health_supported'] for x in sparse['intervals'])}
  if sparse['mapping']['source_indices']==dense['mapping']['source_indices']:
   groups[window+'/'+camera]['all_read_parity']=repeat_comparison(ROOT/captures[window,camera,'dense']['capture']/camera,ROOT/c['capture']/camera)
  if window=='early':
   a=ROOT/captures[window,camera,'dense']['capture']/camera;b=ROOT/captures[window,camera,'dense-repeat']['capture']/camera
   repeat[camera]=repeat_comparison(a,b)
 verify()
 write(OUT/'scores.json',{'scope':'open-loop native gap engineering; not adaptive/online or physical accuracy validation',
  'capture_lock_sha256':sha(OUT/'capture-lock.json'),'scorer_sha256':sha(__file__),
  'groups':groups,'early_dense_repeats':repeat,'rows':rows,'policy_adopted':False})
 print(json.dumps({'groups':groups,'early_dense_repeats':repeat},indent=2))

if __name__=='__main__':main()
