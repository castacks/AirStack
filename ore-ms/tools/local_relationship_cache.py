"""Reference-free local image + RGB-conditioned geometric evidence cache."""
import hashlib,json,time
from pathlib import Path
import numpy as np,cv2
import nominal_joint_motion as geom
R=Path(__file__).resolve().parents[1];REC=R/'results/thermal-incremental-comparison/recovery';OUT=R/'results/local-relationship-contract/cache-001'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())
def write(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def load(path):
 with np.load(path) as z:return {k:z[k] for k in z.files}
def image(c,cam,stem,expected):
 p=REC/'images'/c/cam/(stem+'.png');assert sha(p)==expected;a=cv2.imread(str(p),cv2.IMREAD_UNCHANGED)
 if a.ndim==3:a=cv2.cvtColor(a,cv2.COLOR_BGR2GRAY)
 lo,hi=np.percentile(a,[5,95]);return np.clip((a.astype(float)-lo)/max(float(hi-lo),1),0,1)
def patch(a,uv):
 h,w=a.shape;out=[]
 for u,v in uv:
  x,y=int(np.floor(u+.5)),int(np.floor(v+.5))
  if x<5 or y<5 or x>=w-5 or y>=h-5:out.append([0.,0.,0.]);continue
  z=a[y-5:y+6,x-5:x+6];gx=(z[1:-1,2:]-z[1:-1,:-2])/2;gy=(z[2:,1:-1]-z[:-2,1:-1])/2
  out.append([float(np.std(z)),float(np.sqrt(np.mean(gx*gx+gy*gy))),1.])
 return np.asarray(out)
def main():
 OUT.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1);rows=[];proof=[];started=time.perf_counter();missing=[]
 for c in ['day','night','rain']:
  src=read(REC/(c+'-source.json'));bind=read(REC/('binding-'+c+'.json'));poses=read(REC/('predictions-'+c+'.json'))['rows']
  for row,pr in zip(bind['rows'],poses):
   assert row['pair']==pr['pair'];i=row['pair'];rp=REC/row['cameras']['rgb'].get('packet','MISSING')
   if not row['cameras']['rgb']['valid'] or pr['methods']['rgb_cov'].get('transform') is None:
    missing.append({'condition':c,'pair':i,'reason':'NO_RGB_CONTEXT'});continue
   assert sha(rp)==row['cameras']['rgb']['sha256'];rgb=load(rp);length=float(np.median((rgb['points']@rgb['camera_R'][0].T+rgb['camera_t'][0])[:,2]));scale=np.diag([length]*3+[1]*3);T=np.asarray(pr['methods']['rgb_cov']['source_to_target_body']);uv,J=geom.project(rgb['points'],rgb['camera_R'],rgb['camera_t'],rgb['Kb'],T);L=np.linalg.cholesky(rgb['covariance']);A=np.linalg.solve(L,J@scale);H=np.einsum('nji,njk->ik',A,A);ev,V=np.linalg.eigh(H);assert np.all(ev>0)
   r=np.linalg.solve(L,(uv-rgb['observations'])[...,None])[...,0];rgb_energy=float(np.mean(np.sum(r*r,axis=1)));gap=float(np.min(np.diff(ev))/max(ev[-1],1e-30))
   for cam,sensor in [('rgb','rgb'),('thermal','thr')]:
    e=row['cameras'][cam]
    if not e['valid']:missing.append({'condition':c,'pair':i,'camera':cam,'reason':'UNAVAILABLE_NATIVE_PACKET'});continue
    path=REC/e['packet'];assert sha(path)==e['sha256'];p=load(path);n=len(p['points']);first,second=[src['records'][int(s)] for s in row['source_ids']];K=np.array(first['cameras'][sensor]['K']);pc=p['points']@p['camera_R'][0].T+p['camera_t'][0];assert np.all(pc[:,2]>0);sourceuv=(pc@K.T)[:,:2]/pc[:,2,None];targetuv=p['observations'][:,:2]
    a=image(c,sensor,row['source_ids'][0],first['cameras'][sensor]['left']['sha256']);b=image(c,sensor,row['source_ids'][1],second['cameras'][sensor]['left']['sha256']);aa,bb=patch(a,sourceuv),patch(b,targetuv);fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2];K1=np.array(second['cameras'][sensor]['K']);assert np.allclose(K,K1,rtol=0,atol=1e-8)
    n0=(sourceuv-[cx,cy])/[fx,fy];n1=(targetuv-[cx,cy])/[fx,fy];flow=n1-n0
    C=p['covariance'];assert np.all(p['observations'][:,2]>0);local=np.c_[n0,n1,np.log(pc[:,2]),np.log(p['observations'][:,2]),np.log(C[:,0,0]+C[:,1,1]),np.log(C[:,2,2]),flow,aa[:,0],bb[:,0],aa[:,1],bb[:,1],aa[:,2],bb[:,2]]
    pred,J=geom.project(p['points'],p['camera_R'],p['camera_t'],p['Kb'],T);L=np.linalg.cholesky(C);B=np.linalg.solve(L,J@scale);projected=(B@V)/np.sqrt(ev)[None,None,:];direction=np.sum(projected*projected,axis=1);res=np.linalg.solve(L,(pred-p['observations'])[...,None])[...,0];context=np.c_[np.log1p(direction),np.log1p(np.sum(res*res,axis=1)),np.full(n,np.log1p(rgb_energy))];F=np.c_[local,context];assert F.shape==(n,24) and np.isfinite(F).all();assert np.all(direction>=0)
    # Permutation equivariance and a second quadratic form calculation.
    permutation=np.arange(n)[::-1];other=np.einsum('nki,nkj->nij',B,B);diag=np.diagonal(V.T@other@V,axis1=1,axis2=2)/ev;assert np.allclose(direction,diag,rtol=1e-8,atol=1e-10);assert np.array_equal(F[permutation][permutation],F)
    dest=OUT/f'{c}-{i:03}-{cam}.npz';np.savez_compressed(dest,features=F,source_uv=sourceuv,target_uv=targetuv,native_factor_indices=e['native_factor_indices']);rows.append({'condition':c,'pair':i,'camera':cam,'factors':n,'feature_path':dest.name,'sha256':sha(dest),'packet_sha256':e['sha256'],'invalid_source_patches':int(np.sum(aa[:,2]==0)),'invalid_target_patches':int(np.sum(bb[:,2]==0)),'rgb_eigenvalue_relative_gap':gap})
   if i==0:proof.append({'condition':c,'rgb_factor_count':len(rgb['points']),'rgb_eigenvalues':ev.tolist()})
  print('CACHE',c,flush=True)
 write(OUT/'lock.json',{'rows':rows,'missing':missing,'source_sha256':sha(__file__),'geometry_sha256':sha(geom.__file__),'features_per_factor':24,'source_reference_values_read':False,'fold_normalization_fit':False,'seconds':time.perf_counter()-started,'checks':proof,'scope':'Model inputs only; no learned gate, new pose fits, training, performance or novelty claim.'});print('LOCK',len(rows),'packets',sum(x['factors'] for x in rows),'factors',len(missing),'missing',flush=True)
if __name__=='__main__':main()
