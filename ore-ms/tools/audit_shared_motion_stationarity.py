"""High-precision independent saved-packet audit; no producer/Torch imports.

S(x,v)=1/2 sum_j exp(-v_j+mean(v)) r_j(x)^2; g=S_x=0.
H includes residual curvature. dx/dv=-H^-1 g_v. For v=0,
g_vj= -r_j J_j + g/M, so dL/dvj = L_x H^-1 (r_j J_j-g/M).
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import numpy as np
import mpmath as mp
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def rotation(w):
    a,b,c=w; z=mp.mpf(0)
    K=mp.matrix([[z,-c,b],[c,z,-a],[-b,a,z]])
    angle=mp.sqrt(a*a+b*b+c*c)
    if angle==0: return mp.eye(3)
    return mp.eye(3)+mp.sin(angle)/angle*K+(1-mp.cos(angle))/angle**2*(K*K)


class IndependentProblem:
    def __init__(self,data):
        self.points=[mp.matrix(row.tolist()) for row in data['points']]
        self.C=[mp.matrix(row.tolist()) for row in data['camera_R']]
        self.ct=[mp.matrix(row.tolist()) for row in data['camera_t']]
        self.K=[[mp.mpf(v) for v in row] for row in data['intrinsics']]
        self.obs=[mp.mpf(v) for v in data['observations'].ravel()]
        self.Rref=mp.matrix(data['Rref'].tolist()); self.tref=mp.matrix(data['tref'].tolist())
    def residual(self,x):
        t=mp.matrix(x[:3]); R=rotation(x[3:]); result=[]
        for p,C,ct,K in zip(self.points,self.C,self.ct,self.K):
            q=C*(R*p+t)+ct; fx,fy,cx,cy,bl=K
            result.extend([fx*q[0]/q[2]+cx,fy*q[1]/q[2]+cy,fx*bl/q[2]])
        return mp.matrix([a-b for a,b in zip(result,self.obs)])
    def loss(self,*x):
        D=rotation(x[3:])-self.Rref; t=mp.matrix(x[:3])-self.tref
        return mp.fsum(v*v for v in D)/9+mp.fsum(v*v for v in t)
    def objective(self,weights):
        def fun(*x):
            r=self.residual(x)
            return mp.fsum(w*v*v for w,v in zip(weights,r))/2
        return fun


def gradient(fun,x):
    return mp.matrix([mp.diff(fun,tuple(x),tuple(int(j==i) for j in range(6))) for i in range(6)])


def hessian(fun,x):
    return mp.matrix([[mp.diff(fun,tuple(x),tuple(int(k==i)+int(k==j) for k in range(6))) for j in range(6)] for i in range(6)])


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--protocol',required=True); ap.add_argument('--output',required=True)
    args=ap.parse_args(); protocol_path=Path(args.protocol); pin=json.loads(protocol_path.read_text())
    root=Path(__file__).resolve().parents[1]
    for path,h in pin['input_sha256'].items(): assert hashlib.sha256((root/path).read_bytes()).hexdigest()==h,path
    for path,h in pin['source_sha256'].items(): assert hashlib.sha256((root/path).read_bytes()).hexdigest()==h,path
    out=Path(args.output); out.mkdir(parents=True,exist_ok=False)
    with np.load(root/pin['packet']) as z:data={k:z[k] for k in z.files}
    old=json.loads((root/pin['old_report']).read_text())
    probes=next(c['evidence']['probes'] for c in old['cases'] if c['name']=='pose_gradient_rgb_noise')
    mp.mp.dps=pin['decimal_precision']; problem=IndependentProblem(data)
    # Reproduce original audit exactly, including stopping criteria and method.
    def np_residual(x):
        b=data['points']@Rotation.from_rotvec(x[3:]).as_matrix().T+x[:3]
        c=np.einsum('nij,nj->ni',data['camera_R'],b)+data['camera_t']; fx,fy,cx,cy,bl=data['intrinsics'].T
        pred=np.column_stack((fx*c[:,0]/c[:,2]+cx,fy*c[:,1]/c[:,2]+cy,fx*bl/c[:,2]))
        return (pred-data['observations']).ravel()
    def old_fit(v):
        weight=np.exp(-.5*(v-v.mean()))
        fit=least_squares(lambda x:weight*np_residual(x),np.zeros(6),jac='3-point',method='trf',ftol=1e-14,xtol=1e-14,gtol=1e-14,max_nfev=200)
        assert fit.success
        R=Rotation.from_rotvec(fit.x[3:]).as_matrix()
        loss=float(np.mean((R-data['Rref'])**2)+np.sum((fit.x[:3]-data['tref'])**2))
        return fit,loss
    records=[]
    def refine(v,label):
        fit,old_loss=old_fit(np.array([float(x) for x in v])); weights=[mp.exp(-x+mp.fsum(v)/len(v)) for x in v]
        F=problem.objective(weights); start=tuple(mp.mpf(x) for x in fit.x)
        initial_gradient=gradient(F,start)
        print(json.dumps({'stage':label,'scipy_status':int(fit.status),'exact_initial_gradient_inf':float(mp.norm(initial_gradient,p=mp.inf))}),flush=True)
        def g(*x):return gradient(F,x)
        def H(*x):return hessian(F,x)
        solution=mp.findroot(g,start,J=H,solver='mdnewton',tol=mp.mpf(pin['root_tolerance']),maxsteps=pin['max_newton_steps'],verify=True)
        stationary=gradient(F,solution); fullH=hessian(F,solution)
        eigenvalues,_=mp.eigsy(fullH)
        checks={'stationarity':bool(mp.norm(stationary,p=mp.inf)<mp.mpf(pin['stationarity_limit'])),'positive_definite_hessian':bool(min(eigenvalues)>0)}
        record={'label':label,'scipy_status':int(fit.status),'scipy_message':fit.message,'scipy_optimality':float(fit.optimality),'exact_scipy_gradient_inf':str(mp.norm(initial_gradient,p=mp.inf)),'refined_gradient_inf':str(mp.norm(stationary,p=mp.inf)),'minimum_hessian_eigenvalue':str(min(eigenvalues)),'pose_correction_inf':str(mp.norm(solution-mp.matrix(start),p=mp.inf)),'old_float64_loss':old_loss,'high_precision_loss':str(problem.loss(*solution)),'checks':checks}
        records.append(record)
        # Partial evidence survives a later exception.
        (out/'stationarity-progress.json').write_text(json.dumps(records,indent=2)+'\n')
        return solution,stationary,fullH,old_loss
    zero=[mp.mpf(0)]*72
    x,g,H,old_base=refine(zero,'base')
    lx=gradient(problem.loss,x); adjoint=mp.lu_solve(H,lx); r=problem.residual(x)
    J=mp.matrix([[mp.diff(lambda *a:problem.residual(a)[k],tuple(x),tuple(int(j==i) for j in range(6))) for i in range(6)] for k in range(72)])
    implicit=[mp.fdot(adjoint,r[k]*J[k,:].T-g/72) for k in range(72)]
    comparisons=[]
    for p in probes:
        k=p['row']*3+p['channel']; value=float(implicit[k]); ad=p['autograd']
        comparisons.append({'row':p['row'],'channel':p['channel'],'autograd':ad,'implicit':value,'absolute_error':abs(ad-value),'pass':bool(np.isclose(ad,value,rtol=pin['gradient_rtol'],atol=pin['gradient_atol']))})
    eps=mp.mpf(pin['fd_step']); plus=zero.copy(); minus=zero.copy(); index=pin['failed_row']*3+pin['failed_channel']; plus[index]=eps; minus[index]=-eps
    xp,_,_,old_plus=refine(plus,'failed_probe_plus'); xm,_,_,old_minus=refine(minus,'failed_probe_minus')
    hp_plus=problem.loss(*xp); hp_minus=problem.loss(*xm)
    independent_fd=(hp_plus-hp_minus)/(2*eps); scipy_fd=(old_plus-old_minus)/(2*float(eps))
    discrepancy_from_fit_errors=((mp.mpf(old_plus)-hp_plus)-(mp.mpf(old_minus)-hp_minus))/(2*eps)
    reference=next(p['autograd'] for p in probes if p['row']==pin['failed_row'] and p['channel']==pin['failed_channel'])
    previous=json.loads((root/pin['old_audit']).read_text())
    previous_fd=next(c['independent'] for c in previous['checks'] if c['name']==f"gradient_{pin['failed_row']}_{pin['failed_channel']}")
    checks={'all_stationarity_and_curvature':all(all(z['checks'].values()) for z in records),'all_six_implicit_probes':all(p['pass'] for p in comparisons),'high_precision_central_difference':bool(np.isclose(float(independent_fd),reference,rtol=pin['gradient_rtol'],atol=pin['gradient_atol'])),'implicit_vs_high_precision_fd':bool(np.isclose(float(implicit[index]),float(independent_fd),rtol=pin['gradient_rtol'],atol=pin['gradient_atol'])),'reproduced_previous_scipy_fd':bool(abs(scipy_fd-previous_fd)<pin['replay_atol']),'error_accounting':bool(abs(mp.mpf(scipy_fd)-independent_fd-discrepancy_from_fit_errors)<mp.mpf(pin['accounting_atol']))}
    result={'timestamp_utc':datetime.now(timezone.utc).isoformat(),'method':'Independent 80-digit residual/objective, full-Hessian implicit differentiation and high-precision plus/minus stationarity solves. No producer imports.','protocol_sha256':hashlib.sha256(protocol_path.read_bytes()).hexdigest(),'stationarity':records,'probes':comparisons,'failed_probe':{'prototype_autograd':reference,'old_scipy_fd':scipy_fd,'high_precision_fd':str(independent_fd),'implicit':str(implicit[index]),'fd_error_explained_by_scipy_fit_losses':str(discrepancy_from_fit_errors),'plus_scipy_loss_error':str(mp.mpf(old_plus)-hp_plus),'minus_scipy_loss_error':str(mp.mpf(old_minus)-hp_minus)},'checks':checks,'pass':all(checks.values()),'scope':'Saved RGB-noise packet only. Does not independently qualify thermal fixture, real-data timing, trained covariance or VO benefit.'}
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps({'pass':result['pass'],'checks':checks,'failed_probe':result['failed_probe']}),flush=True)
    return 0 if result['pass'] else 1
if __name__=='__main__': raise SystemExit(main())
