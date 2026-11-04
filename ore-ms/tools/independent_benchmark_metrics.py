"""EXP133 exact-source-time trajectory metrics; no inference or scale fitting."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def pose(value,strict=False):
 if value is None:return None,'missing'
 try:
  a=np.asarray(value,dtype=float)
  if a.shape!=(4,4) or not np.isfinite(a).all() or not np.array_equal(a[3],[0,0,0,1]):raise ValueError('nonfinite_or_homogeneous')
  r=a[:3,:3]
  if np.linalg.norm(r.T@r-np.eye(3))>1e-5 or abs(np.linalg.det(r)-1)>1e-5:raise ValueError('nonrigid')
  u,_,vt=np.linalg.svd(r);out=a.copy();out[:3,:3]=u@np.diag([1.,1.,np.linalg.det(u@vt)])@vt
  return out,None
 except (ValueError,TypeError) as e:
  if strict:raise ValueError('invalid_estimated_pose: '+str(e))
  return None,str(e)

def alignment(estimated,reference):
 """Kabsch SE3 alignment, deliberately without scale parameter."""
 e=np.asarray(estimated);g=np.asarray(reference);em=e.mean(axis=0);gm=g.mean(axis=0)
 rank=int(np.linalg.matrix_rank(e-em,tol=1e-10))
 if rank<2 or np.linalg.matrix_rank(g-gm,tol=1e-10)<2:return None,rank
 u,_,vt=np.linalg.svd((e-em).T@(g-gm));R=vt.T@np.diag([1.,1.,np.linalg.det(vt.T@u.T)])@u.T
 T=np.eye(4);T[:3,:3]=R;T[:3,3]=gm-R@em;return T,rank

def score(reference,estimate):
 times=reference['time_ns'];assert len(times)>=2 and all(type(t) is int for t in times),'exact integer source timestamps required'
 assert all(a<b for a,b in zip(times,times[1:])),'source timestamp monotonicity'
 assert estimate['time_ns']==times,'estimated timestamps must exactly equal source timestamps'
 n=len(times);assert len(reference['poses'])==len(estimate['poses'])==len(estimate['segments'])==n,'parallel source arrays'
 G=[];E=[];invalid=[];segments={};closed=set();active=None;reference_correction=0.;estimate_correction=0.
 for i,(g,e,sid) in enumerate(zip(reference['poses'],estimate['poses'],estimate['segments'])):
  gp,reason=pose(g);ep,_=pose(e,strict=True);G.append(gp);E.append(ep)
  if gp is None:invalid.append({'index':i,'reason':reason})
  else:reference_correction=max(reference_correction,float(np.max(np.abs(gp[:3,:3]-np.asarray(g)[:3,:3]))))
  if ep is not None:estimate_correction=max(estimate_correction,float(np.max(np.abs(ep[:3,:3]-np.asarray(e)[:3,:3]))))
  if ep is None:
   assert sid is None,'missing pose requires null segment'
   if active is not None:closed.add(active);active=None
   continue
  assert type(sid) is int and sid>=0,'valid pose needs nonnegative segment'
  if sid!=active:
   if active is not None:closed.add(active)
   assert sid not in closed,'segment cannot resume after gap/reset';active=sid
  segments.setdefault(sid,[]).append(i)
 # Absolute errors use only valid-reference samples inside each estimator segment.
 absolute=[];origin_squares=[];fit_squares=[];any_degenerate=False
 for sid,indices in segments.items():
  idx=[i for i in indices if G[i] is not None]
  if not idx:continue
  ep=np.array([E[i][:3,3] for i in idx]);gp=np.array([G[i][:3,3] for i in idx]);T0=G[idx[0]]@np.linalg.inv(E[idx[0]]);origin=(T0[:3,:3]@ep.T).T+T0[:3,3];osq=np.sum((origin-gp)**2,axis=1);origin_squares.extend(osq.tolist());T,rank=alignment(ep,gp)
  fit=None
  if T is None:any_degenerate=True
  else:
   aligned=(T[:3,:3]@ep.T).T+T[:3,3];fsq=np.sum((aligned-gp)**2,axis=1);fit_squares.extend(fsq.tolist());fit=float(np.sqrt(fsq.mean()))
  absolute.append({'segment':sid,'source_frames':len(indices),'reference_matched_frames':len(idx),'position_rank':rank,'ate_origin_rmse_m':float(np.sqrt(osq.mean())),'ate_se3_rmse_m':fit})
 rpe=[];expected=[]
 for i,t in enumerate(times):
  j=int(np.searchsorted(times,t+1000000000))
  if j>=n or times[j]>t+1150000000:continue
  if any(G[k] is None for k in range(i,j+1)) or any(times[k+1]-times[k]>150000000 for k in range(i,j)):continue
  expected.append([i,j])
  if any(E[k] is None for k in range(i,j+1)) or any(estimate['segments'][k]!=estimate['segments'][i] for k in range(i,j+1)):continue
  target=np.linalg.solve(G[i],G[j]);pred=np.linalg.solve(E[i],E[j]);err=np.linalg.solve(target,pred)
  rpe.append({'source_indices':[i,j],'duration_ns':times[j]-times[i],'translation_m':float(np.linalg.norm(err[:3,3])),'rotation_deg':float(np.degrees(Rotation.from_matrix(err[:3,:3]).magnitude()))})
 count=sum(e is not None for e in E);longest=0;run=0;last_sid=None
 for i,e in enumerate(E):
  sid=estimate['segments'][i]
  if e is None:run=0
  elif i==0 or E[i-1] is None or sid!=last_sid or times[i]-times[i-1]>150000000:run=1
  else:run+=1
  longest=max(longest,run);last_sid=sid
 coverage=count/n;rc=len(rpe)/len(expected) if expected else 0.;resets=max(0,len(segments)-1)
 return {'source_frames':n,'published_pose_frames':count,'pose_coverage':coverage,'reference_valid_frames':n-len(invalid),'reference_exclusions':invalid,'segment_count':len(segments),'reset_count':resets,'longest_contiguous_frames':longest,'segment_ate':absolute,'ate_origin_rmse_m':float(np.sqrt(np.mean(origin_squares))) if origin_squares else None,'ate_se3_rmse_m':None if any_degenerate or not fit_squares else float(np.sqrt(np.mean(fit_squares))),'alignment':'SE3_only_no_scale','rpe_expected_pairs':len(expected),'rpe_scored_pairs':len(rpe),'rpe_coverage':rc,'rpe_translation_rmse_m':float(np.sqrt(np.mean([r['translation_m']**2 for r in rpe]))) if rpe else None,'rpe_rotation_rmse_deg':float(np.sqrt(np.mean([r['rotation_deg']**2 for r in rpe]))) if rpe else None,'rpe_rows':rpe,'terminal_success':estimate['terminal_success'] is True,'engineering_success':estimate['terminal_success'] is True and resets==0 and coverage>=.95 and rc>=.95,'scale_fitted':False,'reference_rotation_max_correction':reference_correction,'estimate_rotation_max_correction':estimate_correction}

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--reference',type=Path,required=True);p.add_argument('--estimate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=score(json.loads(a.reference.read_text()),json.loads(a.estimate.read_text()))
 a.output.parent.mkdir(parents=True,exist_ok=True)
 with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
