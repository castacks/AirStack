"""Physical-coordinate likelihood reconstruction and independent scalar minimization."""
import argparse
from pathlib import Path
import numpy as np
from scipy.linalg import logm
from scipy.optimize import minimize_scalar
from qualify_native_frontend import read,write,sha,now

def audit(p,out):
    checks=0
    def ck(v,msg):
        nonlocal checks
        assert v,msg
        checks+=1
    def close(a,b,atol=2e-8):
        nonlocal checks
        np.testing.assert_allclose(a,b,rtol=1e-7,atol=atol);checks+=1
    for f,h in p['pins'].items():ck(sha(f)==h,'input/source pin')
    folder=Path(p['output']);d=read(folder/'results.json');pred=read(p['predictions'])['rows'];original=read(p['original_pose_receipts']);old=read(p['old_scores']);ck(d['protocol_sha256']==sha(p['protocol_path']),'diagnostic freeze')
    for f,h in read(folder/'artifact-lock.json')['files'].items():ck(sha(folder/f)==h,'outputlock')
    scenes=sorted({x['scene'] for x in pred});expected={}
    for scene in scenes:
        base=next(x for x in original['rows'] if x['condition']==scene and x['trial']==-2)['arms']['fixed_ids'];T0=np.array(base['target_to_source']);P=np.array(base['information_covariance']);ldP=np.linalg.slogdet(P)[1];y=[]
        records=sorted([x for x in original['rows'] if x['condition']==scene and x['trial']>=0],key=lambda x:x['trial']);ck([x['trial'] for x in records]==list(range(128)),'128slots')
        for x in records:
            Z=logm(np.array(x['arms']['fixed_ids']['target_to_source'])@np.linalg.inv(T0));ck(abs(np.imag(Z)).max()<1e-11,'real logarithm');Z=np.real(Z);y.append(np.r_[Z[:3,3],Z[2,1],Z[0,2],Z[1,0]])
        y=np.array(y)
        for model in [x for x in pred if x['scene']==scene]:
            method=model['method'];C=np.array(model['covariance']);A=np.linalg.inv(C);q=float(np.mean(np.sum((y@A)*y,axis=1)));ld=float(np.linalg.slogdet(C)[1]-ldP);risk=lambda t:.5*(ld+6*t+np.exp(-t)*q)
            opt=minimize_scalar(risk,bounds=(-30,30),method='bounded',options={'xatol':1e-10});ck(opt.success,'independent scalar minimizer');row=next(x for x in d['rows'] if x['scene']==scene and x['method']==method)
            close(row['original_nll'],risk(0));close(row['oracle_profile_nll'],opt.fun);close(np.log(row['oracle_scale_multiplier']),opt.x,atol=2e-6);close(row['scale_penalty_nats'],risk(0)-opt.fun);ck(row['scale_penalty_nats']>=-1e-10,'nonnegative excess');ck(row['oracle_only'] and row['test_errors_used'],'oracle labels mandatory')
            close(row['original_nll'],next(x['nll'] for x in old['rows'] if x['scene']==scene and x['method']==method));expected[(scene,method)]=row
        close(expected[(scene,'native')]['oracle_profile_nll'],expected[(scene,'scalar')]['oracle_profile_nll'])
    for row in d['pairwise']:
        a=expected[(row['scene'],'common_modes')];b=expected[(row['scene'],row['control'])];close(row['original_common_minus_control'],a['original_nll']-b['original_nll']);close(row['shape_common_minus_control'],a['oracle_profile_nll']-b['oracle_profile_nll']);close(row['scale_penalty_common_minus_control'],a['scale_penalty_nats']-b['scale_penalty_nats']);close(row['original_common_minus_control'],row['shape_common_minus_control']+row['scale_penalty_common_minus_control'])
    for fold in d['folds']:
        for method,values in fold['methods'].items():
            rows=[x for x in d['rows'] if x['recording']==fold['excluded_recording'] and x['method']==method]
            for metric,v in values.items():close(v,np.mean([x[metric] for x in rows]))
        close(fold['common_shape_minus_native'],np.mean([x['shape_common_minus_control'] for x in d['pairwise'] if x['recording']==fold['excluded_recording'] and x['control']=='native']))
    ck(len(d['rows'])==30 and len(d['pairwise'])==24 and len(d['folds'])==3 and d['samples']==768,'complete diagnostic denominator')
    write(out,{'created_utc':now(),'status':'PASS','checks':checks,'profiled_slots':30,'scenes':6,'saved_trials':768,'scope':'Independent original-receipt pose logs/physical covariance likelihood, numerical scalar minima and complete shape/scale identities. Oracle diagnostics only; no predictor fitted.'});print('PASS',checks,'independent checks')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--protocol',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();audit(read(a.protocol),Path(a.output))
