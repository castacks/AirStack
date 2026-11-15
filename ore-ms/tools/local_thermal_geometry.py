"""Standard information-selection intervention on saved RGB/thermal factors."""
import hashlib,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import nominal_joint_motion as solver
ROOT=Path(__file__).resolve().parents[1];REC=ROOT/'results/thermal-incremental-comparison/recovery';OUT=ROOT/'results/local-thermal-geometry/run-001'
ARMS=['rgb64','rgb32_thermal32_conditional','rgb32_thermal32_uniform','joint64','uniform32_each']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def packet(path):
 with np.load(path) as z:return {k:z[k] for k in z.files}
def blocks(p,length):
 _,J=solver.project(p['points'],p['camera_R'],p['camera_t'],p['Kb'],np.eye(4));J[:,:,:3]*=length;A=np.linalg.solve(np.linalg.cholesky(p['covariance']),J);return A.transpose(0,2,1)@A

def greedy(B,k,H):
 current=H.copy();available=np.ones(len(B),bool);chosen=[];gains=[]
 for _ in range(k):
  ids=np.flatnonzero(available);sign,ld=np.linalg.slogdet(current[None]+B[ids]);assert np.all(sign>0);idx=ids[int(np.argmax(ld))];gain=float(ld.max()-np.linalg.slogdet(current)[1]);chosen.append(int(idx));gains.append(gain);available[idx]=False;current+=B[idx]
 return chosen,gains,current

def subset(p,ids):return {k:v[ids] for k,v in p.items()}
def qualify():
 checks=[]
 for c in ['day','night','rain']:
  for cam in ['rgb','thermal']:
   p=packet(REC/f'packets/{c}/000-{cam}.npz');solver.validate(p);pred,J=solver.project(p['points'],p['camera_R'],p['camera_t'],p['Kb'],np.eye(4));maximum=0
   for j in range(6):
    d=np.zeros(6);d[j]=1e-6;Ts=[]
    for sign in [1,-1]:
     T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(sign*d[3:]).as_matrix();T[:3,3]=sign*d[:3];Ts.append(solver.project(p['points'],p['camera_R'],p['camera_t'],p['Kb'],T)[0])
    numerical=(Ts[0]-Ts[1])/2e-6;err=float(np.max(np.abs(numerical-J[:,:,j])/np.maximum(1,np.abs(J[:,:,j]))));assert err<1e-5;maximum=max(maximum,err)
   checks.append({'condition':c,'camera':cam,'maximum_scaled_jacobian_error':maximum})
 return checks

def capture():
 OUT.mkdir(parents=True,exist_ok=False);checks=qualify();write(OUT/'qualification.json',checks);rows=[];start=time.perf_counter()
 for c in ['day','night','rain']:
  binding=read(REC/f'binding-{c}.json')
  for row in binding['rows']:
   result={'condition':c,'pair':row['pair'],'source_ids':row['source_ids'],'methods':{}}
   try:
    ps=[]
    for cam in ['rgb','thermal']:
     e=row['cameras'][cam];assert e['valid'];path=REC/e['packet'];assert sha(path)==e['sha256'];ps.append(packet(path));solver.validate(ps[-1])
    p,q=ps;nr,nt=len(p['points']),len(q['points']);result['pool_counts']=[nr,nt]
    if nr<64 or nt<32:raise ValueError('INSUFFICIENT_FIXED_BUDGET_POOL')
    length=float(np.median(p['points'][:,2]));assert length>0;B,C=blocks(p,length),blocks(q,length);ridge=1e-6*np.trace(B.sum(0))/6;prior=np.eye(6)*ridge
    ri,_,_=greedy(B,64,prior);r32=ri[:32];H=prior+B[r32].sum(0);ti,gains,_=greedy(C,32,H);ji,_,_=greedy(np.r_[B,C],64,prior)
    seed=int.from_bytes(hashlib.sha256(f'0:{c}:{row["pair"]}'.encode()).digest()[:8],'little');rng=np.random.default_rng(seed);ru=rng.choice(nr,32,False).tolist();tu=rng.choice(nt,32,False).tolist()
    selections={'rgb64':[(p,ri)],'rgb32_thermal32_conditional':[(p,r32),(q,ti)],'rgb32_thermal32_uniform':[(p,r32),(q,tu)],'uniform32_each':[(p,ru),(q,tu)],'joint64':[(p,[i for i in ji if i<nr]),(q,[i-nr for i in ji if i>=nr])]}
    result['length_m']=length;result['ridge']=ridge;result['conditional_thermal_gains']=gains;result['thermal_selected_depths_m']=(q['points'][ti]@q['camera_R'][0].T+q['camera_t'][0])[:,2].tolist()
    # Independent determinant-lemma validation of each selected thermal gain.
    current=H.copy()
    for idx,gain in zip(ti,gains):
     L=np.linalg.cholesky(current);D=np.linalg.solve(L,C[idx]);D=np.linalg.solve(L,D.T).T;sign,ld=np.linalg.slogdet(np.eye(6)+D);assert sign>0 and abs(float(ld)-gain)<1e-7;current+=C[idx]
    for arm,parts in selections.items():
     chosen=[subset(z,ids) for z,ids in parts if len(ids)>0]
     ids=[list(map(int,x)) for _,x in parts];thermal_count=sum(len(v) for z,v in parts if z is q)
     try:fit=solver.solve(chosen,'cov',solver.CONTRACT)
     except (ValueError,np.linalg.LinAlgError) as e:fit={'status':'FIT_FAILED','reason':str(e)}
     result['methods'][arm]={**fit,'selected_ids':ids,'thermal_count':thermal_count}
   except (ValueError,AssertionError,np.linalg.LinAlgError) as e:
    result['unavailable']=str(e)
   rows.append(result)
  print('CAPTURE',c,flush=True)
 write(OUT/'prediction-lock.json',{'source_sha256':sha(__file__),'solver_sha256':sha(solver.__file__),'rows':rows,'seconds':time.perf_counter()-start,'references_read':False,'arms':ARMS})

def score():
 lock=read(OUT/'prediction-lock.json');summary={}
 for c in ['day','night','rain']:
  refs={r['pair']:r for r in read(REC/f'scores-{c}.json')['rows']};prior={r['pair']:r for r in read(REC/f'predictions-{c}.json')['rows']};rr=[r for r in lock['rows'] if r['condition']==c];common=[r for r in rr if all(r['methods'].get(a,{}).get('status')=='FIXED_BUDGET_COMPLETE' for a in ARMS)];methods={}
  for arm in ARMS+['rgb_all','joint_all','native_rgb']:
   ts=[];rs=[]
   for row in common:
    e=refs[row['pair']];ref=np.asarray(e['reference_delta']);m=row['methods'][arm] if arm in ARMS else prior[row['pair']]['methods'][{'rgb_all':'rgb_cov','joint_all':'joint_cov','native_rgb':'native_rgb'}[arm]]
    T=np.asarray(m['transform']);delta=np.linalg.inv(ref)@T;t=float(np.linalg.norm(delta[:3,3]));r=float(np.rad2deg(Rotation.from_matrix(delta[:3,:3]).magnitude()));scalar=sum((T[j,3]-ref[j,3])**2 for j in range(3))**.5;assert abs(t-scalar)<1e-10
    ts.append(t);rs.append(r)
   methods[arm]={'translation_mean_m':float(np.mean(ts)) if ts else None,'translation_rmse_m':float(np.sqrt(np.mean(np.square(ts)))) if ts else None,'rotation_mean_deg':float(np.mean(rs)) if rs else None,'rotation_rmse_deg':float(np.sqrt(np.mean(np.square(rs)))) if rs else None}
  summary[c]={'expected':99,'common':len(common),'unavailable_pairs':[{'pair':r['pair'],'reason':r.get('unavailable','FIT_FAILED')} for r in rr if r not in common],'own_available':{a:sum(r['methods'].get(a,{}).get('status')=='FIXED_BUDGET_COMPLETE' for r in rr) for a in ARMS},'methods':methods,'joint64_thermal_counts':[r['methods']['joint64']['thermal_count'] for r in common]}
 write(OUT/'scores.json',{'prediction_sha256':sha(OUT/'prediction-lock.json'),'summary':summary});print(json.dumps(summary,indent=2))
if __name__=='__main__':
 import sys
 {'capture':capture,'score':score}[sys.argv[1]]()
