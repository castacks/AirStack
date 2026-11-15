"""Fixed five-parameter shared-mode covariance baseline; development transfer only."""
import argparse
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.stats import chi2
from qualify_native_frontend import read,write,sha,now
from cross_match_covariance import maps,path
ROOT=Path(__file__).resolve().parents[1]
PRIMARY=['native','scalar','block_modes','common_modes','pooled_shape']
def verify(p):
    for f,h in p['pins'].items():assert sha(f)==h,f

def mode_matrix(X0,X1,R,K,baseline):
    d0=K[0,0]*baseline/X0[:,2];d1=K[0,0]*baseline/X1[:,2]
    B=np.empty((len(X0),3,4));B[:,:,0]=X1[:,2,None]*R[:,0]/K[0,0];B[:,:,1]=X1[:,2,None]*R[:,1]/K[1,1];B[:,:,2]=X0/d0[:,None];B[:,:,3]=-(X1@R.T)/d1[:,None]
    return B

def make_features(packet,T,K,baseline):
    H,G=maps(*[packet[k] for k in ['X0','X1','C0','C1']],T);P=np.linalg.inv(H);L=np.linalg.cholesky(P);B=mode_matrix(packet['X0'],packet['X1'],T[:3,:3],K,baseline);D=np.einsum('nij,njk->nik',G,B);common=D.sum(0)
    common_basis=np.einsum('ik,jk->kij',common,common);block_basis=np.einsum('nik,njk->kij',D,D)
    invL=np.linalg.inv(L)
    whiten=lambda basis:np.stack([invL@M@invL.T for M in basis])
    return {'P':P,'L':L,'B':B,'G':G,'common_basis':np.concatenate([np.eye(6)[None],whiten(common_basis)]),'block_basis':np.concatenate([np.eye(6)[None],whiten(block_basis)])}

def objective(theta,bases,moments):
    coef=np.exp(theta);value=0.;grad=np.zeros(5)
    for basis,M in zip(bases,moments):
        S=np.einsum('k,kij->ij',coef,basis);A=np.linalg.inv(S);sign,ld=np.linalg.slogdet(S)
        if sign<=0:raise ValueError('Nonpositive model covariance')
        value+=.5*(ld+np.trace(A@M));E=.5*(A-A@M@A);grad+=coef*np.einsum('ij,kji->k',E,basis)
    return float(value/len(bases)),grad/len(bases)

def prepare(p,out):
    out.mkdir(parents=True,exist_ok=False);pred=read(p['predictions']);K=np.array(p['K']);index=[]
    for scene in p['scenes']:
        fit=next(x for x in pred['rows'] if x['condition']==scene and x['trial']==-2)['arms']['fixed_ids'];f=path(fit['packet_path']);assert sha(f)==fit['packet_sha256']
        with np.load(f) as z:packet={k:z[k] for k in ['source_ids','source_uv','target_uv','X0','X1','C0','C1']}
        T=np.array(fit['target_to_source']);features=make_features(packet,T,K,p['baseline_m']);file=out/(scene+'.npz');np.savez_compressed(file,**features,T0=T,**packet)
        index.append({'id':scene,'recording':scene.split('-')[0],'path':str(file),'sha256':sha(file),'labels':str(ROOT/'results/cross-match-covariance/run-001'/(scene+'.npz')),'points':len(packet['X0'])})
    write(out/'index.json',{'created_utc':now(),'records':index,'feature_inputs':'Unperturbed baseline packet/pose only; no trial residuals/empirical covariances are used by feature function.'})

def qualify(p,out):
    checks=0;maxfd=0.;gradient_error=0.
    def close(a,b,rtol=1e-6,atol=1e-8):
        nonlocal checks
        np.testing.assert_allclose(a,b,rtol=rtol,atol=atol);checks+=1
    K=np.array(p['K']);B=p['baseline_m'];features=read(p['index'])['records']
    for rec in features:
        with np.load(rec['path']) as z:
            X0,X1,T,analytic=[z[k] for k in ['X0','X1','T0','B']]
            for j in range(min(4,len(X0))):
                uv0=np.array([K[0,0]*X0[j,0]/X0[j,2]+K[0,2],K[1,1]*X0[j,1]/X0[j,2]+K[1,2]])
                uv1=np.array([K[0,0]*X1[j,0]/X1[j,2]+K[0,2],K[1,1]*X1[j,1]/X1[j,2]+K[1,2]])
                base=np.r_[uv1,K[0,0]*B/X0[j,2],K[0,0]*B/X1[j,2]]
                def residual(q):
                    z0,z1=K[0,0]*B/q[2],K[0,0]*B/q[3]
                    a=np.array([(uv0[0]-K[0,2])*z0/K[0,0],(uv0[1]-K[1,2])*z0/K[1,1],z0]);b=np.array([(q[0]-K[0,2])*z1/K[0,0],(q[1]-K[1,2])*z1/K[1,1],z1])
                    return T[:3,:3]@b+T[:3,3]-a
                for k in range(4):
                    delta=np.eye(4)[k]*1e-4;fd=(residual(base+delta)-residual(base-delta))/2e-4;maxfd=max(maxfd,float(np.abs(fd-analytic[j,:,k]).max()));close(fd,analytic[j,:,k],atol=1e-9)
            close(z['P'],z['L']@z['L'].T,atol=1e-12)
    rng=np.random.default_rng(20261115)
    for _ in range(6):
        v=rng.normal(size=(4,6,4));common=v.sum(0);C=np.stack([np.eye(6)]+[np.outer(common[:,j],common[:,j]) for j in range(4)]);M0=rng.normal(size=(6,6));M=M0@M0.T+np.eye(6);theta=rng.normal(size=5)*.3;value,g=objective(theta,[C],[M]);fd=[]
        for k in range(5):
            e=np.eye(5)[k]*1e-5;fd.append((objective(theta+e,[C],[M])[0]-objective(theta-e,[C],[M])[0])/2e-5)
        gradient_error=max(gradient_error,float(np.abs(g-fd).max()));close(g,fd,atol=1e-7)
        A=v.transpose(1,0,2).reshape(6,-1);Q=np.diag([.1,.2,.3,.4]);direct=A@np.kron(np.ones((4,4)),Q)@A.T;close(direct,common@Q@common.T)
    for record in ['day','night','rain']:
        train=[x for x in features if x['recording']!=record];test=[x for x in features if x['recording']==record];assert len(train)==4 and len(test)==2 and {x['recording'] for x in train}.isdisjoint({record});checks+=1
    write(out,{'created_utc':now(),'status':'PASS','checks':checks,'max_mode_derivative_fd_error':maxfd,'max_likelihood_gradient_fd_error':gradient_error,'scope':'Baseline-only mode derivatives, PSD/common-mode identity and likelihood gradients/splits; no real target fitting or evaluation scores.'});print('PASS mechanics',checks,'checks',flush=True)

def labels(rec):
    with np.load(rec['labels']) as z:y=z['actual_left_tangent'].copy()
    assert y.shape==(128,6);return y

def train(p,out):
    out.mkdir(parents=True,exist_ok=False);records=read(p['index'])['records'];predictions=[];models=[]
    for excluded in ['day','night','rain']:
        training=[x for x in records if x['recording']!=excluded];evaluation=[x for x in records if x['recording']==excluded];moments=[];all_features=[]
        for rec in training:
            with np.load(rec['path']) as z:features={k:z[k] for k in ['L','block_basis','common_basis']}
            z=np.linalg.solve(features['L'],labels(rec).T).T;moments.append(z.T@z/128);all_features.append(features)
        pooled=np.mean(moments,axis=0);scalar=float(np.trace(pooled)/6);np.linalg.cholesky(pooled)
        parameters={};info={}
        for method,key in [('block_modes','block_basis'),('common_modes','common_basis')]:
            initial=np.log([scalar]+[p['initial_mode_variance_px2']]*4);bases=[f[key] for f in all_features];calls=[]
            def fun(theta):
                value,grad=objective(theta,bases,moments);calls.append({'theta':theta.tolist(),'objective':value,'gradient':grad.tolist()});return value,grad
            result=minimize(fun,initial,jac=True,method='L-BFGS-B',bounds=[p['log_bounds']]*5,options=p['optimizer'])
            parameters[method]=result.x;info[method]={'success':bool(result.success),'status':int(result.status),'message':str(result.message),'iterations':int(result.nit),'objective':float(result.fun),'gradient':result.jac.tolist(),'log_parameters':result.x.tolist(),'coefficients':np.exp(result.x).tolist(),'bound_hit':bool(np.any(np.isclose(result.x,p['log_bounds'][0],atol=1e-5)) or np.any(np.isclose(result.x,p['log_bounds'][1],atol=1e-5))),'function_calls':calls}
            print('fit exclude',excluded,method,result.success,result.nit,result.fun,flush=True)
        models.append({'excluded_recording':excluded,'training_scenes':[x['id'] for x in training],'evaluation_scenes':[x['id'] for x in evaluation],'scalar':scalar,'pooled_shape':pooled.tolist(),'fits':info})
        for rec in evaluation:
            # Prediction consumes baseline geometry only. No labels() call here.
            with np.load(rec['path']) as z:feat={k:z[k] for k in ['L','P','block_basis','common_basis']}
            covariances={'native':np.eye(6),'scalar':scalar*np.eye(6),'pooled_shape':pooled}
            for method,key in [('block_modes','block_basis'),('common_modes','common_basis')]:covariances[method]=np.einsum('k,kij->ij',np.exp(parameters[method]),feat[key])
            for method in PRIMARY:
                S=covariances[method];np.linalg.cholesky(S);predictions.append({'scene':rec['id'],'excluded_recording':excluded,'method':method,'whitened_covariance':S.tolist(),'covariance':(feat['L']@S@feat['L'].T).tolist(),'uses_evaluation_errors':False})
    write(out/'models.json',{'created_utc':now(),'folds':models,'scope':'Three recording-excluded DEVELOPMENT folds; all inputs previously exposed. Fixed Gaussian zero-mean baseline-relative perturbation target.'})
    write(out/'predictions.json',{'created_utc':now(),'rows':predictions,'covariances':30,'protocol_sha256':sha(p['protocol_path']),'heldout_labels_used_for_prediction':False})
    write(out/'prediction-lock.json',{'created_utc':now(),'files':{f.name:sha(f) for f in out.iterdir() if f.is_file()}})

def score(p,out):
    root=Path(p['training_output'])
    for f,h in read(root/'prediction-lock.json')['files'].items():assert sha(root/f)==h
    pred=read(root/'predictions.json');records=read(p['index'])['records'];rows=[]
    for rec in records:
        with np.load(rec['path']) as f:L=f['L'];P=f['P']
        y=labels(rec);z=np.linalg.solve(L,y.T).T;M=z.T@z/128;oracle=float(np.trace(M)/6)
        for method in PRIMARY+['oracle_test_scalar']:
            S=oracle*np.eye(6) if method=='oracle_test_scalar' else np.array(next(x['whitened_covariance'] for x in pred['rows'] if x['scene']==rec['id'] and x['method']==method));A=np.linalg.inv(S);quad=np.einsum('ti,ij,tj->t',z,A,z);ld=np.linalg.slogdet(S)[1];cov=L@S@L.T
            rows.append({'scene':rec['id'],'recording':rec['recording'],'method':method,'nll':float(.5*(ld+quad.mean())),'mahalanobis_mean':float(quad.mean()),'baseline_relative_95_containment':float(np.mean(quad<=chi2.ppf(.95,6))),'predicted_normalized_trace':float(np.trace(S)),'observed_second_moment_trace':float(np.trace(M)),'predicted_translation_spread_rms_m':float(np.sqrt(np.trace(cov[:3,:3]))),'predicted_rotation_spread_rms_rad':float(np.sqrt(np.trace(cov[3:,3:]))),'observed_centered_translation_spread_rms_m':float(np.sqrt(np.trace(np.cov(y,rowvar=False)[:3,:3]))),'observed_centered_rotation_spread_rms_rad':float(np.sqrt(np.trace(np.cov(y,rowvar=False)[3:,3:]))),'observed_mean_normalized_squared_norm':float(np.linalg.norm(z.mean(0))**2),'samples':128,'oracle':method=='oracle_test_scalar'})
    folds=[]
    for condition in ['day','night','rain']:
        aggregate={method:float(np.mean([x['nll'] for x in rows if x['recording']==condition and x['method']==method])) for method in PRIMARY+['oracle_test_scalar']}
        folds.append({'excluded_recording':condition,'scene_macro_nll':aggregate,'common_minus_controls':{m:aggregate['common_modes']-aggregate[m] for m in ['scalar','block_modes','pooled_shape','oracle_test_scalar']}})
    out.mkdir(parents=True,exist_ok=False);write(out/'scores.json',{'created_utc':now(),'rows':rows,'folds':folds,'scene_slots':6,'samples':768,'primary_predictive_covariances':30,'prediction_lock_sha256':sha(root/'prediction-lock.json'),'scope':'Development recording-excluded baseline-relative perturbation uncertainty, not physical/true-pose coverage or pose accuracy improvement. Testscalar is oracle diagnostic only.'});print('Scored all30predictive covariances +6oracle diagnostics',flush=True)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['prepare','qualify','train','score']);ap.add_argument('--protocol',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();p=read(a.protocol);verify(p);{'prepare':prepare,'qualify':qualify,'train':train,'score':score}[a.mode](p,Path(a.output));verify(p)
