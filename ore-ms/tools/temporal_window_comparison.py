"""Frozen variable-depth temporal weighting research comparison; no references."""
import argparse,hashlib,json,sys
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from scipy.linalg import block_diag,solve_triangular


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,v):
    with Path(p).open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')


def load_packet(root,c,start,mode):
    base=root/'results/windowed-geometry/run-001/packets';stem=f'{c}-{start:03}-{mode}'
    with np.load(base/(stem+'.npz'),allow_pickle=False) as z:p={k:z[k] for k in z.files}
    p.update(read(base/(stem+'-ids.json')))
    return p


def transforms(root,c,start,p,arm):
    groups=[];bylength={};base=root/'results/persistent-track-replay/run-001'/c/'tracks'
    cache={}
    for camera in set(x[0] for x in p['ids']):
        with np.load(base/camera/'windows'/f'{start:03}.npz') as z:a={k:z[k] for k in z.files}
        witnesses=[]
        for t in range(4):
            with np.load(base/camera/'witnesses'/f'{start+t:03}.npz') as z:witnesses.append({k:z[k] for k in ['active_track_ids','flow_spatial_jacobian','source_uv','flow_cov_corners']})
        cache[camera]=(a,witnesses)
    for j,(cam,identity) in enumerate(p['ids']):
        a,ws=cache[cam];track=int(np.flatnonzero(a['native_match_ids']==identity)[0]);n=int(a['valid'][track].sum());obs=np.flatnonzero(p['track']==j)
        assert len(obs)==n and np.array_equal(p['frame'][obs],np.arange(n))
        G=np.zeros((2*n,2*(n-1)));Qs=[]
        for t in range(1,n):
            w=ws[t-1];i=int(np.flatnonzero(w['active_track_ids']==track)[0]);u=w['source_uv'][i];dx,dy=u-np.floor(u)
            v=np.array([(1-dx)*(1-dy),dx*(1-dy),(1-dx)*dy,dx*dy])@w['flow_cov_corners'][i]
            Qs.append(np.array([[v[0],v[2]],[v[2],v[1]]]))
            G[2*t:2*t+2]=(np.eye(2)+w['flow_spatial_jacobian'][i])@G[2*t-2:2*t]
            G[2*t:2*t+2,2*(t-1):2*t]=np.eye(2)
        S=G@block_diag(*Qs)@G.T
        C=np.zeros((3*n,3*n))
        for t in range(n):
            C[3*t+2,3*t+2]=a['disparity_cov'][track,t]
            for s in range(n):
                if arm=='conditioned_joint' or t==s:C[3*t:3*t+2,3*s:3*s+2]=S[2*t:2*t+2,2*s:2*s+2]
        if arm=='original_adapter':C=block_diag(*p['covariance'][obs]);keep=np.arange(3*n)
        else:keep=np.arange(2,3*n)
        if arm=='conditioned_scale4':C*=4
        L=np.linalg.cholesky(C[np.ix_(keep,keep)])
        W=solve_triangular(L,np.eye(len(keep)),lower=True)@np.eye(3*n)[keep]
        bylength.setdefault((n,len(keep)),[]).append((obs,W,j))
    offset=0;rowids=[]
    for (n,dim),rows in bylength.items():
        groups.append((np.stack([x[0] for x in rows]),np.stack([x[1] for x in rows])))
        rowids.extend(np.repeat([x[2] for x in rows],dim).tolist())
    return groups,np.array(rowids)


def system(g,p,T,alpha,groups):
    r,A,d=g.system(p,T,alpha)
    # Undo original per-observation whitening, then apply one track transform.
    r=(p['L']@r[...,None]).squeeze(-1);A=p['L']@A;d=(p['L']@d[...,None]).squeeze(-1)
    rr=[];AA=[];dd=[]
    for ids,W in groups:
        rr.append((W@r[ids].reshape(len(ids),-1,1)).reshape(-1))
        AA.append((W@A[ids].reshape(len(ids),-1,24)).reshape(-1,24))
        dd.append((W@d[ids].reshape(len(ids),-1,1)).reshape(-1))
    return np.concatenate(rr),np.concatenate(AA),np.concatenate(dd)


def normal(r,A,d,track,n):
    H=A.T@A;g=A.T@r;X=np.zeros((n,24));D=np.zeros(n);gd=np.zeros(n)
    np.add.at(X,track,A*d[:,None]);np.add.at(D,track,d*d);np.add.at(gd,track,d*r)
    if np.any(D<=0):raise ValueError('unobservable_inverse_depth')
    return H,g,X,D,gd


def step(r,A,d,track,n):
    H,g,X,D,gd=normal(r,A,d,track,n);S=H-X.T@(X/D[:,None]);sv=np.linalg.svd(S,compute_uv=False)
    if sv[-1]/sv[0]<1e-10:raise ValueError('rank_deficient_pose_schur')
    Dd=D*(1+1e-6);S=H+1e-6*np.diag(np.diag(H))-X.T@(X/Dd[:,None])
    dp=np.linalg.solve(S,-g+X.T@(gd/Dd));da=(-gd-X@dp)/Dd
    grad=max(np.max(np.abs(g)/np.sqrt(np.maximum(np.diag(H),1e-300))),np.max(np.abs(gd)/np.sqrt(D)))
    return dp,da,float(g@dp+gd@da),float(grad)


def solve(g,p,groups,track,arm):
    T=np.broadcast_to(np.eye(4),(5,4,4)).copy();alpha=p['alpha0'].copy();history=[];status='FIXED_BUDGET_COMPLETE';first=None
    try:
        for it in range(20):
            r,A,d=system(g,p,T,alpha,groups);cost=float(.5*r@r);dp,da,deriv,grad=step(r,A,d,track,len(alpha))
            if first is None:first={'cost':cost,'dp':dp.tolist(),'da':da.tolist(),'directional':deriv}
            # Keep the declared stop invariant to a uniform covariance rescale.
            if arm=='conditioned_scale4':grad*=2
            if grad<=1e-6:status='SCALED_GRADIENT_STOP';break
            if not np.isfinite(deriv) or deriv>=0:raise ValueError('non_descent_step')
            trials=[];accepted=False
            for t in range(12):
                scale=2.**-t;U,b=g.update(T,alpha,dp,da,scale)
                try:
                    rr,_,_=system(g,p,U,b,groups);newcost=float(.5*rr@rr);valid=np.isfinite(newcost)
                except ValueError:newcost=None;valid=False
                trials.append({'scale':scale,'cost':newcost,'valid':bool(valid)})
                if valid and newcost<=cost+1e-4*scale*deriv:T,alpha=U,b;accepted=True;break
            history.append({'iteration':it,'cost_before':cost,'directional':deriv,'gradient':grad,'trials':trials,'accepted':accepted})
            if not accepted:raise ValueError('line_search_failed')
        r,_,_=system(g,p,T,alpha,groups);final=float(.5*r@r)
        if not np.isfinite(final) or final>first['cost']+1e-9*max(1,first['cost']):raise ValueError('objective_increase')
    except (ValueError,np.linalg.LinAlgError,FloatingPointError) as e:status='UNAVAILABLE';reason=str(e)
    result={'status':status,'root_to_body':T.tolist(),'log_inverse_depth':alpha.tolist(),'history':history,'first_step':first,'rows_after_whitening':len(track)}
    if status=='UNAVAILABLE':result['reason']=reason
    else:result.update(final_cost=final,relative_target_to_source=[(T[i]@np.linalg.inv(T[i+1])).tolist() for i in range(4)])
    return result


def qualify(root,g,out):
    results=[]
    for c in ['day','night','rain']:
      for mode in ['rgb','joint']:
        p=load_packet(root,c,0,mode);groups,track=transforms(root,c,0,p,'original_adapter')
        original=g.solve(p,True);adapter=solve(g,p,groups,track,'original_adapter')
        assert original['status']!='UNAVAILABLE' and adapter['status']!='UNAVAILABLE'
        diff=float(np.max(np.abs(np.array(original['root_to_body'])-adapter['root_to_body'])))
        assert diff<1e-8,('adapter_parity',c,mode,diff)
        groups,track=transforms(root,c,0,p,'conditioned_joint');T=np.broadcast_to(np.eye(4),(5,4,4)).copy();alpha=p['alpha0'].copy()
        r,A,d=system(g,p,T,alpha,groups);dp,da,deriv,grad=step(r,A,d,track,len(alpha));h=1e-5
        U,b=g.update(T,alpha,dp,da,h);rp=system(g,p,U,b,groups)[0]
        U,b=g.update(T,alpha,dp,da,-h);rm=system(g,p,U,b,groups)[0]
        finite=float((.5*rp@rp-.5*rm@rm)/(2*h));assert abs(finite-deriv)<=1e-6+1e-4*abs(deriv)
        H,gg,X,D,gd=normal(r,A,d,track,len(alpha));dense=np.block([[H+1e-6*np.diag(np.diag(H)),X.T],[X,np.diag(D*(1+1e-6))]])
        whole=np.linalg.solve(dense,-np.r_[gg,gd]);assert np.allclose(whole,np.r_[dp,da],atol=1e-7,rtol=1e-6)
        results.append({'condition':c,'mode':mode,'adapter_max_pose_difference':diff,'directional_error':finite-deriv,'dense_schur_max_difference':float(np.max(np.abs(whole-np.r_[dp,da])))})
    write(out/'qualification.json',{'status':'PASS','cases':results,'scope':'First window RGB/joint per record, transformed old weighting parity, joint-cost derivatives and dense/Schur solve.'})


def execute(root,g,plan,out):
    assert read(out/'qualification.json')['status']=='PASS'
    folder=out/'capture';folder.mkdir()
    for c in ['day','night','rain']:
      rows=[]
      for start in range(0,100,5):
        methods={}
        for mode in ['rgb','joint']:
          p=load_packet(root,c,start,mode)
          for arm in ['original','conditioned_marginal','conditioned_joint','conditioned_scale4']:
            if arm=='original':
              try:result=g.solve(p,True)
              except g.SolverFailure as e:result=e.receipt
            else:
              groups,track=transforms(root,c,start,p,arm);result=solve(g,p,groups,track,arm)
            methods[mode+'_'+arm]=result
        row={'start':start,'methods':methods};rows.append(row)
        with (folder/'journal.jsonl').open('a') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
        print(c,start,{k:v['status'] for k,v in methods.items()},flush=True)
      write(folder/f'predictions-{c}.json',{'rows':rows})
    files=list(folder.glob('*'));write(out/'prediction-lock.json',{'files':{str(p.relative_to(out)):sha(p) for p in files},'reference_values_parsed':False,'created':datetime.now(ZoneInfo('America/New_York')).isoformat()})


def main():
    ap=argparse.ArgumentParser();ap.add_argument('stage',choices=['qualify','execute']);ap.add_argument('--protocol',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();plan=read(a.protocol);root=Path(plan['recovery_root']);out=Path(a.output)
    for n,h in plan['source_sha256'].items():assert sha(n)==h,n
    for n,h in plan['input_sha256'].items():assert sha(root/n)==h,n
    sys.path.insert(0,str(root/'tools'));import windowed_stereo_geometry as g
    if a.stage=='qualify':out.mkdir(parents=True);qualify(root,g,out)
    else:execute(root,g,plan,out)
    for n,h in plan['input_sha256'].items():assert sha(root/n)==h,n

if __name__=='__main__':main()
