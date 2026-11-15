"""Frozen synthetic linearized pose assay; known joint covariance is an oracle."""
import argparse
import hashlib
import json
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from scipy.linalg import block_diag, solve_triangular
from scipy.stats import chi2


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def geometry(rng):
    points=rng.uniform([-2,-1,4],[2,1,10],(24,3))
    H=np.zeros((24*4*2,24))
    for k,p in enumerate(points):
        for t in range(4):
            a=.045*t
            R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
            q=R@p+np.array([.12*t,.04*t,0])
            x,y,z=q
            J=np.array([[400/z,0,-400*x/z**2],[0,400/z,-400*y/z**2]])
            S=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            H[k*8+t*2:k*8+t*2+2,t*6:t*6+6]=J@np.column_stack([np.eye(3),-S])
    return H


def covariance(kind):
    blocks=[]
    for k in range(24):
        # Heterogeneous, deterministic per-step error orientations/scales.
        Q=[]
        for t in range(4):
            a=.37*k+.61*t
            R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
            Q.append(R@np.diag([.35+.03*(k%5),.8+.1*t]))
        L=block_diag(*Q)
        if kind=='independent':
            C=L@L.T
        else:
            rho=.6 if kind=='correlated_increments' else 0.
            K=rho**np.abs(np.arange(4)[:,None]-np.arange(4)[None,:])
            A=np.kron(np.tril(np.ones((4,4))),np.eye(2))
            C=A@L@np.kron(K,np.eye(2))@L.T@A.T
        blocks.append(C)
    return block_diag(*blocks)


def estimator(H,C):
    L=np.linalg.cholesky(C)
    W=solve_triangular(L,np.eye(len(C)),lower=True)
    B=W@H
    P=np.linalg.inv(B.T@B)
    return P@B.T@W,P


def run(protocol,output):
    p=json.loads(Path(protocol).read_text())
    assert sha(__file__)==p['source_sha256']
    assert not Path(output).exists(),'Never overwrite an attempt'
    rng=np.random.default_rng(p['seed']);H=geometry(rng)
    n=p['trials'];truth=rng.normal(size=(24,n))*np.tile([.01,.01,.01,.001,.001,.001],4)[:,None]
    scale=np.tile([.1,.1,.1,.01,.01,.01],4)
    report={'scope':p['scope'],'run_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'source_sha256':sha(__file__),'protocol_sha256':sha(protocol),'numpy':np.__version__,'cases':{},'mechanics':{},'thresholds':p['thresholds']}
    arrays={'H':H,'truth':truth}
    q95=float(chi2.ppf(.95,24));checks=[]
    for case in p['cases']:
        noise_case='independent_increments' if case=='bias_control' else case
        C=covariance(noise_case)
        D=block_diag(*[C[i:i+2,i:i+2] for i in range(0,len(C),2)])
        Ej,Pj=estimator(H,C);Ed,Pd=estimator(H,D);Ei,Pi=estimator(H,4*D)
        # Independent direct-precision construction, not the whitening path.
        precision=np.linalg.inv(C)
        E_direct=np.linalg.solve(H.T@precision@H,H.T@precision)
        mechanics={'cov_min_eigenvalue':float(np.linalg.eigvalsh(C)[0]),'state_rank':int(np.linalg.matrix_rank(H)),'noiseless_max_error':float(np.max(np.abs(Ej@H@truth-truth))),'direct_gls_max_difference':float(np.max(np.abs(E_direct-Ej))),'inflation_estimator_max_difference':float(np.max(np.abs(Ei-Ed)))}
        good=mechanics['cov_min_eigenvalue']>0 and mechanics['state_rank']==24 and mechanics['noiseless_max_error']<1e-9 and mechanics['direct_gls_max_difference']<1e-9 and mechanics['inflation_estimator_max_difference']<1e-10
        if case=='independent':
            mechanics['oracle_diagonal_max_difference']=float(np.max(np.abs(Ej-Ed)))
            good=good and mechanics['oracle_diagonal_max_difference']<1e-10
        checks.append(bool(good));report['mechanics'][case]=mechanics
        noise=np.linalg.cholesky(C)@rng.normal(size=(len(C),n))
        bias=np.zeros(24)
        if case=='bias_control':bias=np.tile([.015,-.01,.005,.002,-.001,.001],4)
        y=H@truth+noise+H@bias[:,None]
        rows={}
        for method,E,P in [('independent',Ed,Pd),('inflation4',Ei,Pi),('joint_oracle',Ej,Pj)]:
            error=E@y-truth
            actual=E@C@E.T
            mahal=np.sum(error*np.linalg.solve(P,error),axis=0)
            sign,ld=np.linalg.slogdet(P);assert sign==1
            theoretical=float(np.sum(np.diag(actual)/scale**2)+np.sum((bias/scale)**2))
            rows[method]={'translation_tangent_rmse_m':float(np.sqrt(np.mean(error.reshape(4,6,n)[:,:3]**2))),'rotation_tangent_rmse_rad':float(np.sqrt(np.mean(error.reshape(4,6,n)[:,3:]**2))),'normalized_risk':float(np.mean(np.sum((error/scale[:,None])**2,axis=0))),'analytic_normalized_risk':theoretical,'coverage95_joint24':float(np.mean(mahal<=q95)),'gaussian_nll_mean':float(np.mean(.5*(mahal+ld+24*np.log(2*np.pi)))),'expected_bias_max_error':float(np.max(np.abs(error.mean(1)-bias)))}
            arrays[f'{case}_{method}_errors']=error
            arrays[f'{case}_{method}_reported_cov']=P
        arrays[f'{case}_true_cov']=C
        report['cases'][case]=rows
    report['mechanics_pass']=all(checks)
    gates={}
    for case in p['cases']:
        r=report['cases'][case];j=r['joint_oracle'];d=r['independent'];f=r['inflation4']
        gates[case]={'inflation_does_not_change_point_risk':abs(d['normalized_risk']-f['normalized_risk'])<1e-10}
        if case!='bias_control':gates[case]['oracle_coverage_in_frozen_band']=p['thresholds']['coverage_low']<=j['coverage95_joint24']<=p['thresholds']['coverage_high']
        if case in ['independent_increments','correlated_increments']:
            gain=1-j['analytic_normalized_risk']/d['analytic_normalized_risk']
            gates[case]['analytic_risk_reduction_fraction']=gain
            gates[case]['minimum_analytic_gain']=gain>=p['thresholds']['minimum_analytic_gain']
        if case=='bias_control':
            gates[case]['joint_oracle_does_not_remove_bias']=j['expected_bias_max_error']<p['thresholds']['bias_mean_error_bound']
    report['gates']=gates
    passed=report['mechanics_pass'] and all(v for g in gates.values() for k,v in g.items() if isinstance(v,bool))
    report['decision']='PASS_KNOWN_COVARIANCE_LINEARIZED_MECHANICS_ONLY' if passed else 'STOP_FROZEN_SYNTHETIC_ASSAY'
    out=Path(output);out.mkdir(parents=True)
    np.savez_compressed(out/'arrays.npz',**arrays)
    report['arrays_sha256']=sha(out/'arrays.npz')
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'decision':report['decision'],'gates':gates},indent=2))


if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--protocol',required=True);a.add_argument('--output',required=True);args=a.parse_args();run(args.protocol,args.output)
