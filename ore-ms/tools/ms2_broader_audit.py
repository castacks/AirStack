"""Independent native geometry, causal feature/control and fresh-score checks."""
from pathlib import Path
from decimal import Decimal
import json,hashlib,math
import numpy as np
from scipy.linalg import expm,logm
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/ms2-broader-range/run-001'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def skew(v):
 x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
def pose(v):
 v=np.asarray(v);t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();t[:3,3]=v[:3];return t

def main():
 plan=read(OUT/'execution-plan.json');actions=read(OUT/'locked-actions.json')['rows'];prefix=read(OUT/'prefix-predictions.json')['rows'];inputs={r['sample_id']:r for r in read(OUT/'admission-inputs.json')['rows']};labels=read(OUT/'fresh-labels.json');scores=read(OUT/'scores.json');lock=read(OUT/'prediction-lock.json');checks=0;geometry_count=0;transport_count=0
 assert lock['labels_read'] is False and lock['actions_sha256']==sha(OUT/'locked-actions.json') and lock['prefix_sha256']==sha(OUT/'prefix-predictions.json')
 assert len(actions)==len(prefix)==891 and len(labels)<=891
 byid={r['sample_id']:r for r in actions};D=np.diag([.1]*3+[np.pi/180]*3)
 def close(x,y,rtol=1e-8,atol=1e-9):
  nonlocal checks
  np.testing.assert_allclose(x,y,rtol=rtol,atol=atol);checks+=np.asarray(x).size
 for c in plan['cases']:
  base=OUT/'captures'/c['case']/'on';timeline=[json.loads(x) for x in (base/'timeline.jsonl').read_text().splitlines()];decisions={r['pair_ordinal']:r for r in timeline if r['kind']=='decision'}
  for camera in ['rgb','thermal']:
   report=read(base/'experts'/camera/'report.json');observer=read(base/'experts'/camera/'arbiter-events/summary.json');assert observer['status']=='COMPLETE';assert report['completed_frames']==c['frames']
   worker=[x['event'] for x in timeline if x['kind']=='observer_event' and x['camera']==camera];geometry=[g for g in worker if g['kind']=='native_disparity_information']
   if report['status']=='FAILED':
    assert report['error']=='No visual matches were retained' and report['tracking_diagnostics']['retained_observations']==0 and observer['optimization_submissions']==0 and not geometry
    assert not (base/'experts'/camera/'trajectory.npz').exists();continue
   with np.load(base/'experts'/camera/'tensor-map.npz') as saved:
    for g in geometry:
     m,=[m for m in worker if m['kind']=='motion_result_written' and m['first_write_for_submission'] and m['submission_id']==g['submission_id']]
     assert g['sequence']<m['sequence'];i=g['frame_idx'];mask=saved['edge/match2frame2/mapping']==i;ids=saved['edge/match2point/mapping'][mask];world=saved['points//pos_Tw'][ids].astype(float)
     t=pose(np.asarray(m['optimizer_pose_before_write_native_parameterization']).reshape(7));p=(world-t[:3,3])@t[:3,:3];K=saved['frames//K'][i].astype(float);bl=float(np.asarray(saved['frames//baseline'][i]).reshape(-1)[0]);n=len(p)
     uv=saved['match//pixel2_uv_cov'][mask];disp=saved['match//pixel2_disp_cov'][mask].reshape(-1);cov=np.zeros((n,3,3),np.float32);cov[:,0,0]=uv[:,0];cov[:,1,1]=uv[:,1];cov[:,0,1]=cov[:,1,0]=uv[:,2];cov[:,2,2]=disp
     jac=[]
     for x,y,z in p:
      q=np.array([[-K[0,0]*y/x**2,K[0,0]/x,0],[-K[1,1]*z/x**2,0,K[1,1]/x],[-K[0,0]*bl/x**2,0,0]])
      jac.append(q@np.concatenate([-np.eye(3),skew([x,y,z])],axis=1))
     H=np.einsum('nki,nkl,nlj->ij',jac,np.linalg.pinv(cov.astype(float),rcond=1e-15),jac);close((H+H.T)/2,g['information']['information_6x6'],rtol=1e-10,atol=1e-7);assert n==g['information']['factors'];geometry_count+=1
  # Independent provided-label reconstruction using polar decomposition.
  from scipy.linalg import polar
  refs={}
  for stem in c['stems']:
   a=np.asarray([float(v) for v in (OUT/'reference'/c['sequence']/'rgb'/(stem+'.txt')).read_text().split()]).reshape(4,4);r=a[:3,:3]
   valid=np.isfinite(a).all() and np.array_equal(a[3],[0,0,0,1]) and np.linalg.norm(r.T@r-np.eye(3))<=1e-5 and abs(np.linalg.det(r)-1)<=1e-5
   if valid:a[:3,:3]=polar(r)[0];refs[stem]=a
  for p in [p for p in prefix if p['window']==c['window']]:
   sid=f"{c['window']}/{p['pair_ordinal']}";row=byid[sid];contract=inputs[sid];deadline=decisions[p['pair_ordinal']]['controller_monotonic_ns'];available={};features={};mins={}
   for camera,e in p['experts'].items():
    available[camera]=bool(e['available_by_prefix'] and e['valid_model_projection']);assert available[camera]==row['native_available'][camera]
    if not available[camera]:features[camera]=None;mins[camera]=None;assert row['support_features'][camera] is None;continue
    observed=[x for x in timeline if x['kind']=='observer_event' and x['camera']==camera and x['controller_monotonic_ns']<=deadline];byseq={x['event']['sequence']:x for x in observed};source=e['source'];h=byseq[source['health_event_sequence']]['event'];m=byseq[source['motion_event_sequence']]['event'];g,=[x['event'] for x in observed if x['event']['kind']=='native_disparity_information' and x['event']['submission_id']==m['submission_id']]
    assert [h['time0_ns'],h['time1_ns']]==e['native_interval_ns']==[m['time0_ns'],m['time1_ns']]
    feature={'retained_count':h['retained_count'],'occupied_cells':h['occupied_3x3_cells'],'retained_fraction':h['retained_count']/max(h['selected_count'],1),'native_factors':g['information']['factors'],'health_sequence':h['sequence'],'motion_sequence':m['sequence'],'geometry_sequence':g['sequence']};assert feature==row['support_features'][camera];features[camera]=feature;checks+=7
    native=np.linalg.solve(pose(m['parent_pose_camera_map_xyzw']),pose(m['committed_pose_camera_map_xyzw']));close(native,pose(m['derived_camera_delta_xyzw']))
    t=np.asarray(c['nominal_body_from_camera'][camera]);body=t@native@np.linalg.inv(t);a,b=e['native_interval_ns'];s0,s1=p['common_interval_ns'];ratio=(s1-s0)/(b-a);model=np.real_if_close(expm(ratio*logm(body))).real;close(model,e['delta_model'])
    columns=[];eps=1e-6
    for k in range(6):
     tw=np.zeros((4,4));z=np.eye(6)[k]*eps;tw[:3,:3]=skew(z[3:]);tw[:3,3]=z[:3];values=[]
     for sign in [1,-1]:
      moved=t@native@expm(sign*tw)@np.linalg.inv(t);mapped=expm(ratio*logm(moved));difference=np.real_if_close(logm(np.linalg.solve(model,mapped))).real;values.append(np.r_[difference[:3,3],[difference[2,1],difference[0,2],difference[1,0]]])
     columns.append((values[0]-values[1])/(2*eps))
    A=np.column_stack(columns);inv=np.linalg.inv(A);H=inv.T@np.asarray(g['information']['information_6x6'])@inv;close(H,contract['cameras'][camera]['information']['matrix'],rtol=2e-5,atol=.05);transport_count+=1
    H=np.asarray(contract['cameras'][camera]['information']['matrix']);mins[camera]=float(np.linalg.eigvalsh(D@H@D)[0]);close(mins[camera],row['normalized_information_min_eigenvalue'][camera],rtol=1e-9,atol=1e-8)
   assert row['both_supported']==all(available.values());assert row['actions']['geometry_admission']==('RGB' if available['rgb'] and mins['rgb']>=1 else 'THERMAL' if available['thermal'] and mins['thermal']>=1 else 'ABSTAIN')
   for method in ['retained_count','occupied_cells','retained_fraction','information_rank','information_per_factor']:
    possible=[cam for cam in ['rgb','thermal'] if available[cam]]
    def score(cam):
     if method=='information_rank':return mins[cam]
     if method=='information_per_factor':return mins[cam]/features[cam]['native_factors']
     return features[cam][method]
    selected=max(possible,key=lambda cam:(score(cam),cam=='rgb')) if possible else None
    assert row['actions'][method]==(selected.upper() if selected else 'ABSTAIN');checks+=1
   i=p['pair_ordinal'];first,last=c['stems'][i:i+2]
   if first not in refs or last not in refs:assert sid not in labels;continue
   assert sid in labels;refdelta=np.linalg.solve(refs[first],refs[last])
   for camera,e in p['experts'].items():
    if not e['valid_model_projection']:assert labels[sid][camera] is None;continue
    error=np.linalg.solve(refdelta,np.asarray(e['delta_model']));R=error[:3,:3];sin=np.linalg.norm([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/2;cos=(np.trace(R)-1)/2;close(labels[sid][camera]['translation_m'],np.linalg.norm(error[:3,3]));close(labels[sid][camera]['rotation_deg'],np.degrees(np.arctan2(sin,cos)))
 # Independent exact-population and matched scoring for full and each window.
 def check_score(result,selected):
  for scope,g in result['groups'].items():
   subset=selected if scope=='all' else [r for r in selected if r['both_supported']];assert g['ids']==[r['sample_id'] for r in subset] and g['population']==len(subset)
   def stats(stat,chosen,method):
    valid=[r for r in chosen if r['actions'][method]!='ABSTAIN'];assert stat['ids']==[r['sample_id'] for r in valid] and stat['scored']==len(valid)
    for metric in ['translation_m','rotation_deg']:
     value=math.sqrt(math.fsum(labels[r['sample_id']][r['actions'][method].lower()][metric]**2 for r in valid)/len(valid)) if valid else None
     if value is None:assert stat[metric] is None
     else:close(stat[metric],value)
   for method,stat in g['methods'].items():stats(stat,subset,method)
   for method,pair in g['matched'].items():
    common=[r for r in subset if r['actions']['geometry_admission']!='ABSTAIN' and r['actions'][method]!='ABSTAIN'];assert pair['ids']==[r['sample_id'] for r in common];stats(pair['geometry_admission'],common,'geometry_admission');stats(pair[method],common,method)
  coverage=result['groups']['all']['methods'];pairs=result['groups']['both_supported']['matched'];required=['raw','history','rgb_only','thermal_only','retained_count','occupied_cells','retained_fraction'];passed=coverage['geometry_admission']['scored']>=coverage['raw']['scored'] and all(pairs[m]['ids'] and pairs[m]['geometry_admission']['translation_m']<pairs[m][m]['translation_m'] and pairs[m]['geometry_admission']['rotation_deg']<=pairs[m][m]['rotation_deg'] for m in required);assert bool(passed)==result['primary_criterion_passed']
 eligible=[r for r in actions if r['sample_id'] in labels];check_score(scores,eligible)
 for c in plan['cases']:check_score(scores['windows'][c['window']],[r for r in eligible if r['window']==c['window']])
 assert lock['both_supported_ids']==[r['sample_id'] for r in actions if r['both_supported']]
 for n,h in {**plan['input_source_sha256'],**plan['scorer_only_reference_sha256']}.items():assert sha(ROOT/n)==h
 result={'status':'PASS_INDEPENDENT_MS2_BROADER_AUDIT','numeric_checks':int(checks),'native_geometry_events':geometry_count,'eligible_transports':transport_count,'decisions':891,'both_supported':sum(r['both_supported'] for r in actions),'scores_sha256':sha(OUT/'scores.json'),'actions_sha256':sha(OUT/'locked-actions.json'),'auditor_sha256':sha(__file__),'physical_accuracy_verified':False}
 with (OUT/'audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(result)

if __name__=='__main__':main()
