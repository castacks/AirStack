"""Qualify finite-difference task-weight gradients on fixed native-cache probes."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from paired_frontend_assessment import fit
from pose_covariance_replay import log_se3
from selection_policy_contract import bounded_pose_loss,residual_utility_weights,standardize
from qualify_native_frontend import now,read,sha


def hat(v):
    zero=v[0]*0
    return torch.stack([torch.stack([zero,-v[2],v[1]]),torch.stack([v[2],zero,-v[0]]),torch.stack([-v[1],v[0],zero])])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--protocol',required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
    plan,out=read(args.protocol),Path(args.output);out.mkdir(parents=True,exist_ok=False)
    for path,h in plan['pins'].items():assert sha(path)==h,path
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
    contract=read(plan['corpus_contract']);rows={r['id']:r for r in read(Path(plan['native_cache'])/'predictions.json')['rows']};renders={r['id']:r for r in read(plan['render_inventory'])['records']}
    checks=0;failures=[];receipts=[];gradients=[]
    def check(value,message):
        nonlocal checks
        if not value:failures.append(message)
        checks+=1
    for fold in contract['folds']:
        group=fold['excluded_group'];key=sorted(fold['training_scene_ids'])[0];row=rows[key]
        check(row['status']=='AVAILABLE',group+':fixed probe packet available')
        if row['status']!='AVAILABLE':continue
        statistics=read(Path(plan['derived_cache'])/(group+'-statistics.json'))
        with np.load(row['packet_path']) as z:
            all_packet={k:z[k].copy() for k in ['X0','X1','C0','C1','source_uv','length']};all_features=z['features'].copy()
        features=standardize(all_features,np.array(statistics['mean']),np.array(statistics['scale']))
        # Same deterministic first uniform-seed subset later used by the weight-only control.
        seed=int.from_bytes(hashlib.sha256(f"{contract['seed']}:{key}:uniform:0".encode()).digest()[:8],'little')
        ids=np.sort(np.random.default_rng(seed).choice(len(features),min(32,len(features)),replace=False))
        packet={k:(v[ids] if v.ndim>0 and len(v)==len(features) else v) for k,v in all_packet.items()};F=features[ids]
        with np.load(renders[key]['path']) as z:truth=z['target_to_source'].copy()
        for probe,theta in [('zero',np.zeros(8)),('fixed',np.linspace(-.1,.1,8))]:
            def solve(t,label):
                weight=residual_utility_weights(t,F)
                weighted={**packet,'C0':packet['C0']/weight[:,None,None],'C1':packet['C1']/weight[:,None,None]}
                try:result=fit(weighted,contract['solver'])
                except (ValueError,np.linalg.LinAlgError) as error:result={'successful':False,'status':'NUMERICAL_FAILURE','reason':str(error)}
                loss=bounded_pose_loss(log_se3(np.array(result['target_to_source'])@np.linalg.inv(truth))) if result['successful'] else 1.
                receipts.append({'fold':group,'scene_id':key,'probe':probe,'label':label,'theta':t.tolist(),'loss':loss,'fit':result})
                check(result['successful'],f'{group}:{probe}:{label} convergence')
                return result,loss
            result,loss=solve(theta,'base')
            finite={}
            for step in [1e-3,5e-4]:
                values=[]
                for j in range(8):
                    _,plus=solve(theta+np.eye(8)[j]*step,f'plus:{step}:{j}')
                    _,minus=solve(theta-np.eye(8)[j]*step,f'minus:{step}:{j}')
                    values.append((plus-minus)/(2*step))
                finite[step]=np.asarray(values)
            if not result['successful']:continue
            T=np.asarray(result['target_to_source']);relative=T@np.linalg.inv(truth)
            check(np.linalg.norm(relative-np.eye(4),ord=np.inf)<.5,group+':matrix log series domain')
            if np.linalg.norm(relative-np.eye(4),ord=np.inf)>=.5:continue
            X0,X1,C0,C1=[torch.tensor(packet[k],dtype=torch.float64) for k in ['X0','X1','C0','C1']]
            Ft=torch.tensor(F,dtype=torch.float64);Tt=torch.tensor(T,dtype=torch.float64);inverse=torch.tensor(np.linalg.inv(truth),dtype=torch.float64)
            zero=torch.zeros(6,dtype=torch.float64,requires_grad=True);th=torch.tensor(theta,dtype=torch.float64,requires_grad=True)
            def pose(delta):
                algebra=torch.cat([torch.cat([hat(delta[3:]),delta[:3,None]],1),torch.zeros((1,4),dtype=torch.float64)],0)
                return torch.matrix_exp(algebra)@Tt
            def score(delta,t):
                U=pose(delta);R=U[:3,:3];q=X1@R.T+U[:3,3];residual=q-X0
                covariance=C0+R@C1@R.T
                weight=torch.softmax(Ft@t,0)*len(F)
                weighted=torch.linalg.solve(covariance,residual[...,None])[...,0]*weight[:,None]
                J=torch.cat([torch.eye(3,dtype=torch.float64).expand(len(F),3,3),-torch.stack([hat(v) for v in q])],2)
                return torch.einsum('nji,nj->i',J,weighted)
            def error(delta):
                E=pose(delta)@inverse-torch.eye(4,dtype=torch.float64);power=E;matrix=E*0
                for order in range(1,41):
                    matrix=matrix+((-1.)**(order+1)/order)*power;power=power@E
                rotation=torch.stack([(matrix[2,1]-matrix[1,2])*.5,(matrix[0,2]-matrix[2,0])*.5,(matrix[1,0]-matrix[0,1])*.5])
                return torch.cat([matrix[:3,3],rotation])
            x=error(zero)
            check(np.allclose(x.detach().numpy(),log_se3(relative),rtol=1e-8,atol=1e-10),group+':matrix log vs analytic Log')
            risk=((x[:3]/.01)**2).sum()+((x[3:]/np.deg2rad(.1))**2).sum();objective=-torch.expm1(-risk)
            ell=torch.autograd.grad(objective,zero,retain_graph=True)[0]
            G=torch.autograd.functional.jacobian(lambda d:score(d,th),zero)
            B=torch.autograd.functional.jacobian(lambda t:score(zero,t),th)
            implicit=-(ell@torch.linalg.solve(G,B)).detach().numpy()
            check(np.isfinite(implicit).all(),group+':implicit gradient finite')
            for step,value in finite.items():check(np.allclose(value,implicit,rtol=.03,atol=1e-5),f'{group}:{probe}:implicit vs FD{step}')
            check(np.allclose(finite[1e-3],finite[5e-4],rtol=.03,atol=1e-5),group+':FD half step agreement')
            gradients.append({'fold':group,'scene_id':key,'probe':probe,'loss':loss,'implicit':implicit.tolist(),'score_jacobian':G.detach().numpy().tolist(),'parameter_score_jacobian':B.detach().numpy().tolist(),'loss_gradient':ell.detach().numpy().tolist(),'matrix_log_error':x.detach().numpy().tolist(),'finite_1e-3':finite[1e-3].tolist(),'finite_5e-4':finite[5e-4].tolist(),'maximum_abs_implicit_fd_difference':float(np.max(np.abs(implicit-finite[1e-3]))),'uniform_seed':seed,'selected_rows':ids.tolist()})
    (out/'fit-receipts.json').write_text(json.dumps(receipts,indent=2)+'\n')
    (out/'gradients.json').write_text(json.dumps(gradients,indent=2)+'\n')
    report={'created_utc':now(),'status':'PASS' if not failures else 'FAIL','checks':checks,'failures':failures,'probes':len(gradients),'fit_attempts':len(receipts),'converged_fits':sum(r['fit']['successful'] for r in receipts),'maximum_abs_implicit_fd_difference':max((g['maximum_abs_implicit_fd_difference'] for g in gradients),default=None),'auditor_sha256':sha(__file__),'scope':'Fixed predeclared native-cache weight probes; actual IRLS score Jacobian includes rotating covariance. No utility optimizer updates or policy performance evaluation.'}
    (out/'qualification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    for path,h in plan['pins'].items():assert sha(path)==h,path
    if failures:raise SystemExit(2)


if __name__=='__main__':main()
