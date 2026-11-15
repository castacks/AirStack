"""Fixed-case correspondence/depth corroboration, not a fusion model."""
from pathlib import Path
import hashlib,json
import numpy as np,cv2
import nominal_joint_motion as geom
R=Path(__file__).resolve().parents[1];REC=R/'results/thermal-incremental-comparison/recovery';PRIOR=R/'results/dynamic-thermal-evidence/run-001';DEP=R/'results/fixed-group-diagnosis/recovery';OUT=R/'results/fixed-group-diagnosis/run-001'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def load(p):
 with np.load(p) as z:return {k:z[k] for k in z.files}
def ncc(a,b):
 a=a.astype(float).ravel();b=b.astype(float).ravel();a-=a.mean();b-=b.mean();d=np.linalg.norm(a)*np.linalg.norm(b);return float(a@b/d) if d>0 else None
def patch(a,u):
 x,y=np.floor(u+.5).astype(int);h,w=a.shape
 if x<5 or y<5 or x>=w-5 or y>=h-5:return None
 return a[y-5:y+6,x-5:x+6]
def gray(c,cam,i,source):
 p=REC/'images'/c/cam/f'{i:06}.png';assert sha(p)==source['cameras'][cam]['left']['sha256'];a=cv2.imread(str(p),cv2.IMREAD_UNCHANGED)
 return cv2.cvtColor(a,cv2.COLOR_BGR2GRAY) if a.ndim==3 else a

def tracker(a,b,uv0,uv1):
 def norm(x):
  lo,hi=np.percentile(x,[5,95]);return np.uint8(np.clip((x.astype(float)-lo)/max(hi-lo,1),0,1)*255)
 a8,b8=norm(a),norm(b);p=uv0.astype(np.float32)[:,None];crit=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01)
 q,s,_=cv2.calcOpticalFlowPyrLK(a8,b8,p,None,winSize=(21,21),maxLevel=3,criteria=crit);back,sb,_=cv2.calcOpticalFlowPyrLK(b8,a8,q,None,winSize=(21,21),maxLevel=3,criteria=crit);h,w=b.shape
 fb=np.linalg.norm(back[:,0]-p[:,0],axis=1);valid=(s[:,0]>0)&(sb[:,0]>0)&np.isfinite(q[:,0]).all(1)&np.isfinite(back[:,0]).all(1)&(q[:,0,0]>=0)&(q[:,0,0]<w)&(q[:,0,1]>=0)&(q[:,0,1]<h)&(fb<=1)
 return [{'lk_cycle_valid':bool(v),'lk_forward_backward_px':float(f) if np.isfinite(f) else None,'native_lk_endpoint_disagreement_px':float(np.linalg.norm(q[j,0]-uv1[j])) if v else None} for j,(v,f) in enumerate(zip(valid,fb))]
def sample(depth,uv):
 xy=np.floor(uv+.5).astype(int);h,w=depth.shape;inside=(xy[:,0]>=0)&(xy[:,0]<w)&(xy[:,1]>=0)&(xy[:,1]<h);z=np.zeros(len(uv));z[inside]=depth[xy[inside,1],xy[inside,0]].astype(float)/256;return z,inside&(z>0)
def summarize(rows,ids):
 rr=[rows[j] for j in ids];fields=['patch_ncc','native_lk_endpoint_disagreement_px','uv_residual_rgb_px','uv_residual_reference_px','source_depth_relative_abs_error','target_disparity_abs_error_px','uv_residual_reference_source_depth_substituted_px']
 out={'count':len(rr),'lk_cycle_supported':sum(r['lk_cycle_valid'] for r in rr)}
 for f in fields:
  vals=[x[f] for x in rr if x.get(f) is not None];out[f]={'supported':len(vals),'median':float(np.median(vals)) if vals else None,'mean':float(np.mean(vals)) if vals else None}
 return out

def main():
 OUT.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1);cv2.setRNGSeed(0);ab=read(PRIOR/'ablation-scores.json')['rows'];fixed=read(PRIOR/'illustrated-examples.json')['examples'];events=[]
 for c in ['day','night','rain']:
  for cat in ['helps','hurts','neutral']:
   if cat!='neutral':e=next(x for x in fixed if x['condition']==c and x['event_category']==cat)
   else:
    rr=[x for x in ab if x['condition']==c and x['event_category']==cat and x['removed_count'] and 'increase_over_full_fixed_m' in x['scores']['cell']];e=max(rr,key=lambda x:abs(x['scores']['cell']['increase_over_full_fixed_m']))
   events.append({k:e[k] for k in ['condition','event_category','pair','cell','removed_local_ids']})
 write(OUT/'fixed-events.json',{'events':events,'parent_artifact_sha256':sha(PRIOR/'ablation-scores.json'),'selection':'Existing6illustratedgroups plus3previousneutralcases, no neweventsearch.'});depthmeta={(x['condition'],int(x['stem'])):x for x in read(DEP/'inputs/metadata.json')['rows']};cases=[];checks=[]
 x=np.arange(121,dtype=float).reshape(11,11);assert abs(ncc(x,x)-1)<1e-12 and abs(ncc(x,4*x+12)-1)<1e-12 and abs(ncc(x,-x)+1)<1e-12 and ncc(np.ones((11,11)),x) is None
 for event in events:
  c,i=event['condition'],event['pair'];source=read(REC/f'{c}-source.json');binding=read(REC/f'binding-{c}.json')['rows'][i];pr=read(REC/f'predictions-{c}.json')['rows'][i];ref=np.array(read(REC/f'scores-{c}.json')['rows'][i]['reference_delta']);Tref=np.linalg.inv(ref);Trgb=np.array(pr['methods']['rgb_cov']['source_to_target_body']);record={'event':event,'cameras':{}}
  for cam,sensor in [('rgb','rgb'),('thermal','thr')]:
   e=binding['cameras'][cam];path=REC/e['packet'];assert sha(path)==e['sha256'];p=load(path);K=np.array(source['records'][i]['cameras'][sensor]['K']);pc=p['points']@p['camera_R'][0].T+p['camera_t'][0];uv0=(pc@K.T)[:,:2]/pc[:,2,None];uv1=p['observations'][:,:2];a,b=gray(c,sensor,i,source['records'][i]),gray(c,sensor,i+1,source['records'][i+1]);tracking=tracker(a,b,uv0,uv1)
   rgbuv,_=geom.project(p['points'],p['camera_R'],p['camera_t'],p['Kb'],Trgb);refuv,_=geom.project(p['points'],p['camera_R'],p['camera_t'],p['Kb'],Tref);r_rgb=np.linalg.norm(rgbuv[:,:2]-uv1,axis=1);r_ref=np.linalg.norm(refuv[:,:2]-uv1,axis=1)
   depths={}
   if cam=='thermal':
    for frame,coords in [(i,uv0),(i+1,uv1)]:
     m=depthmeta.get((c,frame))
     if m is None:continue
     depthpath=DEP/'inputs'/c/f'{frame:06}.png';assert sha(depthpath)==m['depth_sha256'] and m['sequence']==source['sequence'];assert np.allclose(K,np.array(m['K']),rtol=0,atol=1e-8);depth=cv2.imread(str(depthpath),cv2.IMREAD_UNCHANGED);assert depth.dtype==np.uint16 and depth.shape==(256,640);depths[frame]=sample(depth,coords)
   # Source-depth substitution only where exact own-camera supplied depth exists.
   replaced={}
   if i in depths:
    zz,valid=depths[i];ids=np.flatnonzero(valid)
    if len(ids):
     pp={k:v[ids].copy() for k,v in p.items()};newpc=pc[ids]*(zz[ids]/pc[ids,2])[:,None];pp['points']=(newpc-pp['camera_t'])@p['camera_R'][0];newuv,_=geom.project(pp['points'],pp['camera_R'],pp['camera_t'],pp['Kb'],Tref);replaced={int(j):float(v) for j,v in zip(ids,np.linalg.norm(newuv[:,:2]-uv1[ids],axis=1))}
   rows=[]
   for j in range(len(pc)):
    aa,bb=patch(a,uv0[j]),patch(b,uv1[j]);value=ncc(aa,bb) if aa is not None and bb is not None else None;row={'native_local_id':j,'source_uv':uv0[j].tolist(),'target_uv':uv1[j].tolist(),'source_inferred_depth_m':float(pc[j,2]),'patch_ncc':value,**tracking[j],'uv_residual_rgb_px':float(r_rgb[j]),'uv_residual_reference_px':float(r_ref[j]),'source_depth_relative_abs_error':None,'source_provided_depth_m':None,'target_disparity_abs_error_px':None,'target_provided_depth_m':None,'uv_residual_reference_source_depth_substituted_px':replaced.get(j)}
    if i in depths and depths[i][1][j]:z=float(depths[i][0][j]);row['source_provided_depth_m']=z;row['source_depth_relative_abs_error']=abs(pc[j,2]-z)/z;xy=np.floor(uv0[j]+.5).astype(int);assert z>0
    if i+1 in depths and depths[i+1][1][j]:z=float(depths[i+1][0][j]);row['target_provided_depth_m']=z;row['target_disparity_abs_error_px']=abs(float(p['observations'][j,2]-p['Kb'][j,0]*p['Kb'][j,4]/z))
    if j==0 and value is not None:
     av=aa.astype(float).ravel();bv=bb.astype(float).ravel();ma=sum(av)/len(av);mb=sum(bv)/len(bv);n=sum((v-ma)*(w-mb) for v,w in zip(av,bv));den=sum((v-ma)**2 for v in av)**.5*sum((w-mb)**2 for w in bv)**.5;assert abs(n/den-value)<1e-12
    rows.append(row)
   ids=event['removed_local_ids'] if cam=='thermal' else list(range(len(pc)));others=[j for j in range(len(pc)) if j not in set(ids)];record['cameras'][cam]={'packet_sha256':e['sha256'],'rows':rows,'selected':summarize(rows,ids),'remaining':summarize(rows,others),'source_map_available':i in depths,'target_map_available':i+1 in depths};checks.append({'condition':c,'pair':i,'camera':cam,'source_image_sha256':sha(REC/'images'/c/sensor/f'{i:06}.png'),'target_image_sha256':sha(REC/'images'/c/sensor/f'{i+1:06}.png')})
  cases.append(record);print('DIAGNOSED',c,event['event_category'],i,flush=True)
 write(OUT/'diagnosis.json',{'cases':cases,'source_sha256':sha(__file__),'depth_packet_scope':'Exactfirst20ownthermaldepthframes,no holesfilled. Projection/time/reference are nominal, not certifiedGT.','checks':checks,'no_new_pose_fits_or_learning':True});print('COMPLETE',len(cases),flush=True)
if __name__=='__main__':main()
