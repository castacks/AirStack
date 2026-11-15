"""Independent raw-factor reconstruction, determinant-lemma selection and pose audit."""
import argparse,json,hashlib
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from scipy.linalg import solve_triangular


def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def crossmatrix(x):
    return np.array([[0,-x[2],x[1]],[x[2],0,-x[0]],[-x[1],x[0],0]],float)
def stats(values):
    if not values:return {'mean':None,'rmse':None}
    return {'mean':float(sum(values)/len(values)),'rmse':float(np.linalg.norm(values)/np.sqrt(len(values)))}

def audit(plan_path,out):
    plan=read(plan_path);lock=read(out/'prediction-lock.json');prediction=read(out/'predictions.json');scores=read(out/'scores.json');checks=0;maxdiff=0.;selection_checks=0;pose_checks=0
    def check(ok,msg):
        nonlocal checks
        assert ok,msg;checks+=1
    def close(a,b,rtol=2e-7,atol=1e-8):
        nonlocal checks,maxdiff
        np.testing.assert_allclose(a,b,rtol=rtol,atol=atol);checks+=1
        if np.asarray(a).size:maxdiff=max(maxdiff,float(np.max(np.abs(np.asarray(a)-np.asarray(b)))))
    for p,h in plan['pins'].items():check(sha(p)==h,p)
    for name,h in lock['files'].items():check(sha(out/name)==h,name)
    check(lock['camera_pairs']==114 and lock['attempts']==1254,'all attempts')
    check(lock['reference_values_parsed'] is prediction['reference_values_parsed'] is False,'global reference lock')
    check(scores['prediction_lock_sha256']==sha(out/'prediction-lock.json'),'scoring bound to lock')
    graphs={}
    for stream in plan['streams']:
        with np.load(stream['graph_path']) as z:
            keys=['edge/match2frame1/mapping','edge/match2frame2/mapping','frames//K','match//pixel1_uv','match//pixel2_uv','match//pixel1_d','match//pixel2_d','match//obs1_covTc','match//obs2_covTc']
            graphs[stream['condition'],stream['camera']]={k:z[k].copy() for k in keys}
    audited=[];outcomes={};capped=0;failed=0
    for row,scored in zip(prediction['rows'],scores['rows'],strict=True):
        c=row['condition'];camera=row['camera'];pair=row['pair'];g=graphs[c,camera]
        check([scored[k] for k in ['condition','camera','pair']]==[c,camera,pair],'row identity')
        # Every raw accepted factor, no per-arm eligibility changes.
        ids=np.flatnonzero((g['edge/match2frame1/mapping']==pair)&(g['edge/match2frame2/mapping']==pair+1));n=len(ids);k=min(32,n)
        X=[]
        for frame,prefix in [(pair,'pixel1'),(pair+1,'pixel2')]:
            uv=g['match//'+prefix+'_uv'][ids].astype(float);d=g['match//'+prefix+'_d'][ids].ravel().astype(float);K=g['frames//K'][frame].astype(float)
            close(K,[[K[0,0],0,K[0,2]],[0,K[1,1],K[1,2]],[0,0,1]],atol=1e-12)
            X.append(np.column_stack([(uv[:,0]-K[0,2])*d/K[0,0],(uv[:,1]-K[1,2])*d/K[1,1],d]))
        S=(g['match//obs1_covTc'][ids]+g['match//obs2_covTc'][ids]).astype(float)[:,[1,2,0]][:,:,[1,2,0]]
        length=float(np.median(g['match//pixel1_d'][ids].ravel().astype(float)));cholesky=np.linalg.cholesky(S)
        check(row['status']=='AVAILABLE_POOL' and row['pool_size']==n,'pool admission')
        check(sha(out/row['packet_path'])==row['packet_sha256'],'packet hash')
        with np.load(out/row['packet_path']) as packet:
            check(np.array_equal(packet['ids'],ids),'factor IDs');close(packet['X0'],X[0]);close(packet['X1'],X[1]);close(packet['S'],S);close(packet['length'],length)
        Bs=[]
        for x,L in zip(X[1],cholesky,strict=True):Bs.append(solve_triangular(L,np.concatenate([np.eye(3)*length,-crossmatrix(x)],axis=1),lower=True))
        B=np.array(Bs);blocks=np.array([b.T@b for b in B]);ridge=np.trace(np.sum(blocks,axis=0))*1e-6/6
        # Matrix determinant lemma: rank-three information increments, independent of primary slogdet(6x6).
        wanted=[];H=np.eye(6)*ridge;remaining=list(range(n))
        if k==n:wanted=list(range(n))
        else:
          for _ in range(k):
            inverse=np.linalg.inv(H);gains=[]
            for j in remaining:
                determinant=np.linalg.slogdet(np.eye(3)+B[j]@inverse@B[j].T);check(determinant[0]>0,'positive gain determinant');gains.append(determinant[1])
            best=remaining[int(np.argmax(gains))];wanted.append(best);remaining.remove(best);H+=blocks[best]
        expected_information=sorted(wanted)
        # Nominal absolute thermal camera poses give an independent reference construction.
        P=[np.loadtxt(plan['references'][c][f]).reshape(4,4) for f in [pair,pair+1]]
        E=np.eye(4) if camera=='rgb' else np.array(plan['thermal_from_rgb'][c])
        W0=P[0]@np.linalg.inv(E);W1=P[1]@np.linalg.inv(E);truth=np.linalg.solve(W0,W1)
        close(scored['truth_target_to_source'],truth)
        methods={}
        for name,fit in row['methods'].items():
            idx=np.array(fit['selected_local_ids'],int);check(len(idx)==(n if name=='all_pool' else k),'budget');check(np.array_equal(idx,np.unique(idx)) and np.all((idx>=0)&(idx<n)),'unique sorted subset')
            check(fit['selected_native_ids']==ids[idx].tolist(),'native factor identity')
            check(np.isfinite(fit['selection_seconds']) and fit['selection_seconds']>=0,'selection time')
            if name=='information':check(idx.tolist()==expected_information,'determinant-lemma greedy IDs');selection_checks+=1
            elif name=='reliability':
                expected=sorted(sorted(range(n),key=lambda j:(float(np.trace(S[j])),int(ids[j])))[:k]);check(idx.tolist()==expected,'reliability rank')
            elif name=='all_pool' or k==n:check(idx.tolist()==list(range(n)),'full-pool set')
            else:
                method,seed_text=name.split('_');seed=int(seed_text);seed=int.from_bytes(hashlib.sha256(f'{seed}:{c}:{camera}:{pair}'.encode()).digest()[:8],'little');check(fit['derived_seed']==seed,'derived seed');rng=np.random.default_rng(seed)
                if method=='uniform':expected=np.sort(rng.choice(n,k,replace=False)).tolist()
                else:
                    h,w=plan['image_shapes'][camera];uv=g['match//pixel1_uv'][ids].astype(float)
                    cells=np.minimum(np.maximum((uv/np.array([w,h])*4).astype(int),0),3);cellid=cells[:,1]*4+cells[:,0]
                    queues=[rng.permutation(np.flatnonzero(cellid==a)).tolist()[::-1] for a in range(16)];order=rng.permutation(16);chosen=[];turn=0
                    while len(chosen)<k:
                        cell=int(order[turn%16]);turn+=1
                        if queues[cell]:chosen.append(queues[cell].pop(0))
                    expected=sorted(chosen)
                check(idx.tolist()==expected,'stochastic subset reproduction')
            if k==n:
                anchor=row['methods']['all_pool'];check(all(fit[key]==anchor[key] for key in ['target_to_source','status','final_cost','gradient_inf','cost_history']),'exact capped-pool parity');capped+=1
            T=np.array(fit['target_to_source']);R=T[:3,:3];t=T[:3,3];close(R.T@R,np.eye(3),atol=1e-10);close(np.linalg.det(R),1,atol=1e-10)
            residual=(X[1][idx]@R.T+t-X[0][idx]);invS=np.linalg.inv(S[idx]);cost=float(np.einsum('ni,nij,nj->',residual,invS,residual));close(fit['final_cost'],cost)
            gradient=np.zeros(6);information=np.zeros((6,6))
            for point,r,Cinv in zip(X[1][idx]@R.T+t,residual,invS,strict=True):
                J=np.concatenate([np.eye(3)*length,-crossmatrix(point)],axis=1);gradient+=J.T@Cinv@r;information+=J.T@Cinv@J
            gradient_inf=float(np.max(np.abs(gradient)));close(fit['gradient_inf'],gradient_inf,rtol=2e-6,atol=1e-6)
            limit=plan['solver']['gradient_relative']*max(1.,np.sqrt(np.linalg.eigvalsh(information)[-1])*np.sqrt(cost));close(fit['gradient_limit'],limit,rtol=1e-6,atol=1e-10)
            check(np.all(np.diff(fit['cost_history'])<=1e-8),'nonincreasing costs');close(fit['cost_history'][-1],cost)
            check(fit['successful']==(fit['status']=='CONVERGED_GRADIENT'),'status eligibility')
            if fit['successful']:check(gradient_inf<=limit*1.01+1e-6,'independent stationarity')
            else:failed+=1
            eig=np.linalg.eigvalsh(np.sum(blocks[idx],axis=0));close(fit['information_eigenvalues'],eig,rtol=2e-7,atol=1e-7)
            error=None
            if fit['successful']:
                D=np.linalg.solve(truth[:3,:3],R);skewvec=np.array([D[2,1]-D[1,2],D[0,2]-D[2,0],D[1,0]-D[0,1]])
                error={'translation_m':float(np.linalg.norm(t-truth[:3,3])),'rotation_deg':float(np.rad2deg(np.arctan2(np.linalg.norm(skewvec)/2,(np.trace(D)-1)/2)))}
                for metric,value in error.items():close(scored['methods'][name]['errors'][metric],value)
            else:check(scored['methods'][name]['errors'] is None,'failed fit null error')
            check(scored['methods'][name]['successful']==fit['successful'] and scored['methods'][name]['status']==fit['status'],'score eligibility')
            methods[name]={'successful':fit['successful'],'errors':error};pose_checks+=1;outcomes[fit['status']]=outcomes.get(fit['status'],0)+1
        audited.append({'condition':c,'camera':camera,'pair':pair,'methods':methods})
    def summary(rows,name):
        good=[r['methods'][name]['errors'] for r in rows if r['methods'][name]['successful']]
        return {'attempted':len(rows),'successful':len(good),'translation_m':stats([r['translation_m'] for r in good]),'rotation_deg':stats([r['rotation_deg'] for r in good])}
    def compare(actual,expected):
        if isinstance(expected,dict):
            check(set(actual)==set(expected),'mapping keys')
            for k,v in expected.items():compare(actual[k],v)
        elif isinstance(expected,list):check(actual==expected,'list equality')
        elif expected is None or isinstance(expected,(bool,str,int)):check(actual==expected,'scalar identity')
        else:close(actual,expected,atol=1e-9)
    passes=[]
    for c in ['day','night','rain']:
      for camera in ['rgb','thermal']:
        rows=[r for r in audited if r['condition']==c and r['camera']==camera];sc=scores['streams'][c+'-'+camera];names=list(rows[0]['methods']);own={name:summary(rows,name) for name in names};compare(sc['own'],own);cgates=[]
        for control in ['uniform','spatial']:
            controls=[f'{control}_{seed}' for seed in plan['seeds']];common=[r for r in rows if all(r['methods'][name]['successful'] for name in ['information']+controls)]
            information=summary(common,'information');perseed={n:summary(common,n) for n in controls};averages={metric:{stat:float(np.mean([perseed[n][metric][stat] for n in controls])) if common else None for stat in ['mean','rmse']} for metric in ['translation_m','rotation_deg']}
            coverage=all(own['information']['successful']>=own[n]['successful'] for n in controls)
            gate=bool(common) and coverage and all(information['translation_m'][stat]<=.95*averages['translation_m'][stat] and information['rotation_deg'][stat]<=averages['rotation_deg'][stat] for stat in ['mean','rmse']);cgates.append(gate)
            expected={'matched_pair_count':len(common),'matched_pair_ids':[r['pair'] for r in common],'information':information,'control_per_seed':perseed,'control_seed_mean':averages,'coverage_not_lower':coverage,'pass':gate};compare(sc['comparisons'][control],expected)
        check(sc['primary_pass']==all(cgates),'stream gate');passes.append(all(cgates))
    expected='ADVANCE_TO_SEPARATE_INTEGRATION_REVIEW' if all(passes) else 'CLOSE_THIS_ACCEPTED_POOL_SELECTION_CONFIGURATION_NO_RETUNING';check(scores['decision']==expected,'decision')
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'status':'PASS','checks':checks,'greedy_information_sets_verified':selection_checks,'fits_verified':pose_checks,'capped_pool_fit_parity_checks':capped,'failed_fit_receipts_verified':failed,'solver_status_counts':outcomes,'maximum_absolute_numeric_difference':maxdiff,'scores_sha256':sha(out/'scores.json'),'auditor_sha256':sha(__file__),'scope':'Independent source reconstruction/selection/3D costs and gradients/reference errors/aggregation. No physical timing/generalization or novelty proof.'}
    with (out/'independent-audit.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
    print(json.dumps(report))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--protocol',required=True);p.add_argument('--output',required=True);a=p.parse_args();audit(a.protocol,Path(a.output))
