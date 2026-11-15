"""Oracle diagnostic separating Gaussian covariance scale and shape; no prediction."""
import argparse
from pathlib import Path
import numpy as np
from scipy.optimize import minimize_scalar
from qualify_native_frontend import read,write,sha,now
METHODS=['native','scalar','block_modes','common_modes','pooled_shape']

def quantities(S,M):
    q=float(np.trace(np.linalg.solve(S,M)));beta=q/6
    if beta<=0:raise ValueError('Positive second moment required')
    ld=float(np.linalg.slogdet(S)[1]);original=.5*(ld+q);profile=.5*(ld+6*np.log(beta)+6);penalty=3*(beta-1-np.log(beta))
    return {'original_nll':original,'oracle_profile_nll':float(profile),'oracle_scale_multiplier':beta,'scale_penalty_nats':float(penalty)}

def qualify(out):
    checks=0
    def close(a,b,atol=1e-8):
        nonlocal checks
        np.testing.assert_allclose(a,b,rtol=1e-7,atol=atol);checks+=1
    rng=np.random.default_rng(20261115)
    for _ in range(6):
        A=rng.normal(size=(6,6));S=A@A.T+np.eye(6);B=rng.normal(size=(6,6));M=B@B.T+np.eye(6);C=rng.normal(size=(6,6));C+=4*np.eye(6);v=quantities(S,M)
        risk=lambda t:.5*(np.linalg.slogdet(np.exp(t)*S)[1]+np.trace(np.linalg.solve(np.exp(t)*S,M)))
        optimum=minimize_scalar(risk,bounds=(-20,20),method='bounded',options={'xatol':1e-10})
        close(v['oracle_profile_nll'],optimum.fun);close(np.log(v['oracle_scale_multiplier']),optimum.x,atol=1e-6);close(v['original_nll']-v['oracle_profile_nll'],v['scale_penalty_nats'])
        for scale in [.01,100.]:close(quantities(scale*S,M)['oracle_profile_nll'],v['oracle_profile_nll'])
        native=quantities(np.eye(6),M);transformed=quantities(C@S@C.T,C@M@C.T);reference=quantities(C@C.T,C@M@C.T)
        close(v['oracle_profile_nll']-native['oracle_profile_nll'],transformed['oracle_profile_nll']-reference['oracle_profile_nll']);close(quantities(v['oracle_scale_multiplier']*S,M)['scale_penalty_nats'],0,atol=1e-12)
    write(out,{'created_utc':now(),'status':'PASS','checks':checks,'scope':'Oracle scalar profiling against numerical1Dminimization, pairwise/congruence/scale identities; no realdata outcomes.'});print('PASS',checks,'mechanics checks')

def assess(p,out):
    for f,h in p['pins'].items():assert sha(f)==h,f
    pred=read(p['predictions'])['rows'];records=read(p['index'])['records'];old=read(p['old_scores']);rows=[]
    for rec in records:
        with np.load(rec['path']) as f:L=f['L']
        with np.load(rec['labels']) as f:y=f['actual_left_tangent']
        z=np.linalg.solve(L,y.T).T;M=z.T@z/128
        for method in METHODS:
            prediction=next(x for x in pred if x['scene']==rec['id'] and x['method']==method);S=np.array(prediction['whitened_covariance']);v=quantities(S,M);oldrow=next(x for x in old['rows'] if x['scene']==rec['id'] and x['method']==method);np.testing.assert_allclose(v['original_nll'],oldrow['nll'],rtol=1e-10,atol=1e-10)
            rows.append({'scene':rec['id'],'recording':rec['recording'],'method':method,**v,'samples':128,'oracle_only':True,'test_errors_used':True})
    pairs=[]
    for rec in records:
        q={x['method']:x for x in rows if x['scene']==rec['id']};np.testing.assert_allclose(q['native']['oracle_profile_nll'],q['scalar']['oracle_profile_nll'],atol=1e-10)
        for control in ['native','scalar','block_modes','pooled_shape']:
            common=q['common_modes'];other=q[control]
            pairs.append({'scene':rec['id'],'recording':rec['recording'],'control':control,'original_common_minus_control':common['original_nll']-other['original_nll'],'shape_common_minus_control':common['oracle_profile_nll']-other['oracle_profile_nll'],'scale_penalty_common_minus_control':common['scale_penalty_nats']-other['scale_penalty_nats']})
    fold=[]
    for record in ['day','night','rain']:
        fold.append({'excluded_recording':record,'methods':{m:{key:float(np.mean([x[key] for x in rows if x['recording']==record and x['method']==m])) for key in ['original_nll','oracle_profile_nll','scale_penalty_nats']} for m in METHODS},'common_shape_minus_native':float(np.mean([x['shape_common_minus_control'] for x in pairs if x['recording']==record and x['control']=='native']))})
    out.mkdir(parents=True,exist_ok=False);write(out/'results.json',{'created_utc':now(),'rows':rows,'pairwise':pairs,'folds':fold,'scene_method_slots':30,'scenes':6,'samples':768,'protocol_sha256':sha(p['protocol_path']),'scope':'TEST-ERROR-INFORMED ORACLE DIAGNOSTIC ONLY. No retraining/new available predictor, correctedcoverage or pose changes.'})
    write(out/'artifact-lock.json',{'files':{f.name:sha(f) for f in out.iterdir() if f.is_file()}})
    for f,h in p['pins'].items():assert sha(f)==h,f
    for x in fold:print(x['excluded_recording'],'common_shape_minus_native',x['common_shape_minus_native'],'common_scale_penalty',x['methods']['common_modes']['scale_penalty_nats'],flush=True)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['qualify','assess']);ap.add_argument('--protocol');ap.add_argument('--output',required=True);a=ap.parse_args()
    if a.mode=='qualify':qualify(Path(a.output))
    else:assess(read(a.protocol),Path(a.output))
