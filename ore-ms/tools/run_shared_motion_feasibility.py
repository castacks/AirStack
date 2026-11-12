"""Fixed synthetic qualification. Keeps every case, including failed ones."""
import argparse
import dataclasses
import hashlib
import json
from pathlib import Path
import subprocess
import traceback
from datetime import datetime, timezone
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import torch
from shared_motion_feasibility import Packet, project, solve, skew, pose_loss


def tensor(x): return torch.tensor(x,dtype=torch.float64)


def fixture(noisy_camera=None):
    rng = np.random.default_rng(17)
    points = rng.uniform([-2,-1,4],[2,1,12],(24,3))
    CR = np.repeat(np.eye(3)[None],24,axis=0)
    CR[12:] = Rotation.from_rotvec([.02,.08,-.03]).as_matrix()
    ct = np.zeros((24,3)); ct[12:] = [-.17,.04,.02]
    K = np.repeat(np.array([[600,610,320,240,.24]]),24,axis=0).astype(float)
    K[12:] = [420,410,160,128,.18]
    R = Rotation.from_rotvec([.012,-.018,.009]).as_matrix()
    t = np.array([.08,-.03,.12])
    c = np.einsum('nij,nj->ni',CR,points@R.T+t)+ct
    uv = np.stack((K[:,0]*c[:,0]/c[:,2]+K[:,2],
                   K[:,1]*c[:,1]/c[:,2]+K[:,3],K[:,0]*K[:,4]/c[:,2]),axis=1)
    noise = rng.normal(size=(12,3))*[.4,1.2,.2]
    if noisy_camera is not None: uv[noisy_camera*12:(noisy_camera+1)*12] += noise
    p = Packet(tensor(points),tensor(uv),tensor(CR),tensor(ct),tensor(K),
               torch.tensor([0]*12+[1]*12),tensor([[0,.1],[0,.1]]))
    return p,tensor(R),tensor(t)


def digest(p):
    return {f.name:hashlib.sha256(getattr(p,f.name).numpy().tobytes()).hexdigest()
            for f in dataclasses.fields(p)}


def replace_rows(p,index):
    return Packet(**{f.name:(getattr(p,f.name) if f.name=='intervals' else getattr(p,f.name)[index])
                     for f in dataclasses.fields(p)})


def independent_projection(p,R,t):
    pts=p.points.numpy(); C=p.camera_R.numpy(); ct=p.camera_t.numpy(); K=p.intrinsics.numpy()
    c=np.einsum('nij,nj->ni',C,pts@R.T+t)+ct
    return np.stack((K[:,0]*c[:,0]/c[:,2]+K[:,2],K[:,1]*c[:,1]/c[:,2]+K[:,3],K[:,0]*K[:,4]/c[:,2]),axis=1)


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--protocol',required=True); ap.add_argument('--output',required=True)
    args=ap.parse_args(); protocol=json.loads(Path(args.protocol).read_text())
    root=Path(__file__).resolve().parents[1]
    for rel,h in protocol['source_sha256'].items():
        assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==h,rel
    out=Path(args.output); out.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
    p,Rref,tref=fixture(0); lv=torch.zeros((24,3),dtype=torch.float64)
    original=digest(p)
    np.savez(out/'fixed-inputs.npz',**{f.name:getattr(p,f.name).numpy() for f in dataclasses.fields(p)},Rref=Rref.numpy(),tref=tref.numpy())
    cases=[]
    cfg=protocol['solver']; tol=protocol['tolerances']
    def fit(q=p,v=lv): return solve(q,v,**cfg)
    def check(name,fn):
        try:
            evidence=fn(); assert digest(p)==original,'fixed input mutated'
            cases.append({'name':name,'pass':True,'evidence':evidence})
        except Exception as e:
            cases.append({'name':name,'pass':False,'error':str(e),'traceback':traceback.format_exc()})
    def recovery():
        q,R,t=fixture(); r,u,h=fit(q); err=max(float((r-R).abs().max()),float((u-t).abs().max()))
        assert err<tol['recovery_abs']; return {'max_pose_error':err,'cost_history':h}
    check('noiseless_recovery',recovery)
    def noise(cam):
        q,R,t=fixture(cam); r,u,h=fit(q)
        assert np.isfinite(h).all() and h[-1]<=h[0]
        assert torch.isfinite(r).all() and torch.isfinite(u).all()
        return {'loss':float(pose_loss(r,u,R,t)),'cost_history':h}
    check('rgb_anisotropic_error',lambda:noise(0)); check('thermal_anisotropic_error',lambda:noise(1))
    def equivalent(kind):
        r,t,_=fit()
        if kind=='duplicate':
            q=replace_rows(p,torch.arange(24).repeat(2)); rr,tt,_=fit(q,lv.repeat(2,1)); er,et=r,t
        elif kind=='order':
            idx=torch.arange(23,-1,-1); rr,tt,_=fit(replace_rows(p,idx),lv[idx]); er,et=r,t
        else:
            Q=tensor(Rotation.from_rotvec([.2,-.1,.15]).as_matrix()); s=tensor([.4,-.2,.1])
            C=p.camera_R@Q.T
            q=dataclasses.replace(p,points=p.points@Q.T+s,camera_R=C,camera_t=p.camera_t-(C@s))
            rr,tt,_=fit(q); er=Q@r@Q.T; et=Q@t+s-er@s
        err=max(float((rr-er).abs().max()),float((tt-et).abs().max()))
        assert err<tol['equivariance_abs']; return {'max_pose_difference':err}
    check('duplicate_all_observations',lambda:equivalent('duplicate'))
    check('modality_order',lambda:equivalent('order'))
    check('body_coordinate_change',lambda:equivalent('body'))
    def jacobian():
        R=tensor(Rotation.from_rotvec([.01,.02,-.01]).as_matrix()); t=tensor([.02,.01,.03]); _,J=project(p,R,t)
        eps=tol['fd_step']; cols=[]
        for k in range(6):
            d=np.zeros(6); d[k]=eps
            def pert(d):
                dr=Rotation.from_rotvec(d[3:]).as_matrix()
                return independent_projection(p,dr@R.numpy(),dr@t.numpy()+d[:3])
            cols.append((pert(d)-pert(-d))/(2*eps))
        fd=np.stack(cols,axis=-1); err=float(np.max(np.abs(J.numpy()-fd)))
        assert np.allclose(J.numpy(),fd,rtol=tol['jacobian_rtol'],atol=tol['jacobian_atol'])
        return {'max_abs_error':err}
    check('measurement_jacobian',jacobian)
    def gradients(cam):
        q,R,t=fixture(cam); v=lv.clone().requires_grad_(); rr,tt,_=fit(q,v)
        loss=pose_loss(rr,tt,R,t); g=torch.autograd.grad(loss,v)[0]
        assert torch.isfinite(g).all()
        probes=[(cam*12+i,j) for i in [0,5] for j in range(3)]
        records=[]
        for i,j in probes:
            vp=lv.clone(); vm=lv.clone(); vp[i,j]+=tol['fd_step']; vm[i,j]-=tol['fd_step']
            rp,tp,_=fit(q,vp); rm,tm,_=fit(q,vm)
            fd=float((pose_loss(rp,tp,R,t)-pose_loss(rm,tm,R,t))/(2*tol['fd_step']))
            ad=float(g[i,j]); assert np.isclose(ad,fd,rtol=tol['gradient_rtol'],atol=tol['gradient_atol']),(i,j,ad,fd)
            records.append({'row':i,'channel':j,'autograd':ad,'finite_difference':fd})
        norms=[float(g[q.modality==m].norm()) for m in [0,1]]
        assert min(norms)>tol['nonzero_gradient']
        assert not any(getattr(q,f.name).requires_grad for f in dataclasses.fields(q))
        return {'loss':float(loss.detach()),'modality_gradient_norms':norms,'probes':records,'weight_gradient_sum':float(g.sum())}
    check('pose_gradient_rgb_noise',lambda:gradients(0)); check('pose_gradient_thermal_noise',lambda:gradients(1))
    def gauge():
        r,t,_=fit(); rr,tt,_=fit(p,lv+7.)
        err=max(float((r-rr).abs().max()),float((t-tt).abs().max()))
        assert err<tol['equivariance_abs']; return {'max_pose_difference':err}
    check('global_weight_gauge',gauge)
    def rejection(kind):
        q=dataclasses.replace(p); v=lv.clone()
        expected={'degenerate':'degenerate','observation':'nonfinite','variance':'nonfinite','time':'asynchronous','depth':'depth'}[kind]
        if kind=='degenerate': q.points=tensor(np.tile([0,0,6.],(24,1)))
        if kind=='observation': q.observations=p.observations.clone(); q.observations[0,0]=float('nan')
        if kind=='variance': v[0,0]=float('nan')
        if kind=='time': q.intervals=tensor([[0,.1],[.008,.108]])
        if kind=='depth': q.points=p.points.clone(); q.points[0,2]=-2.
        try: fit(q,v)
        except ValueError as e:
            assert expected in str(e); return {'rejected':str(e)}
        raise AssertionError('invalid case accepted')
    for name,kind in [('degenerate_geometry','degenerate'),('nonfinite_observation','observation'),('nonfinite_logvariance','variance'),('asynchronous_intervals','time'),('behind_camera','depth')]: check(name,lambda k=kind:rejection(k))
    def independent_fit():
        r,t,_=fit()
        def residual(x): return (independent_projection(p,Rotation.from_rotvec(x[3:]).as_matrix(),x[:3])-p.observations.numpy()).reshape(-1)
        result=least_squares(residual,np.zeros(6),method='lm',ftol=1e-12,xtol=1e-12,gtol=1e-12,max_nfev=200)
        assert result.success
        rr=Rotation.from_rotvec(result.x[3:]).as_matrix()
        err=max(float(np.max(np.abs(r.numpy()-rr))),float(np.max(np.abs(t.numpy()-result.x[:3]))))
        assert err<tol['independent_pose_abs']; return {'max_pose_difference':err,'scipy_status':int(result.status)}
    check('independent_scipy_pose',independent_fit)
    assert [c['name'] for c in cases]==protocol['cases']
    report={'timestamp_utc':datetime.now(timezone.utc).isoformat(),'commit':subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip(),'torch':torch.__version__,'protocol_sha256':hashlib.sha256(Path(args.protocol).read_bytes()).hexdigest(),'cases':cases,'pass':all(c['pass'] for c in cases),'fixed_inputs_unchanged':digest(p)==original,'scope':'synthetic conditional mechanics only; no training, real-data temporal qualification or VO gain'}
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'pass':report['pass'],'cases':len(cases),'failed':[c['name'] for c in cases if not c['pass']]}))
    return 0 if report['pass'] else 1

if __name__=='__main__': raise SystemExit(main())
