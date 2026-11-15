"""Independent baseline geometry, training objectives and excluded-record scores."""
import argparse
from pathlib import Path
import numpy as np
from scipy.linalg import block_diag,logm
from scipy.stats import chi2
from qualify_native_frontend import read,write,sha,now
ROOT=Path(__file__).resolve().parents[1]
def resolve(s):
    p=Path(s);return p if p.is_absolute() else ROOT.parent/p

def audit(p,out):
    checks=0
    def ck(v,msg):
        nonlocal checks
        assert v,msg
        checks+=1
    def close(a,b,rtol=3e-7,atol=1e-10):
        nonlocal checks
        np.testing.assert_allclose(a,b,rtol=rtol,atol=atol);checks+=1
    for f,h in p['pins'].items():ck(sha(f)==h,'source/input pins')
    root=Path(p['training_output']);pred=read(root/'predictions.json');models=read(root/'models.json');scores=read(Path(p['score_output'])/'scores.json');index=read(p['index'])['records'];original=read(p['predictions']);K=np.array(p['K']);features={};targets={};Pphysical={}
    for f,h in read(root/'prediction-lock.json')['files'].items():ck(sha(root/f)==h,'prediction lock')
    ck(pred['heldout_labels_used_for_prediction'] is False and pred['protocol_sha256']==sha(p['protocol_path']),'prediction contract');ck(scores['prediction_lock_sha256']==sha(root/'prediction-lock.json'),'score lock')
    for rec in index:
        ck(sha(rec['path'])==rec['sha256'],'feature lock');base=next(x for x in original['rows'] if x['condition']==rec['id'] and x['trial']==-2)['arms']['fixed_ids'];T=np.array(base['target_to_source']);R=T[:3,:3];t=T[:3,3]
        with np.load(resolve(base['packet_path'])) as z:X0,X1,C0,C1=[z[k] for k in ['X0','X1','C0','C1']]
        J=[];B=[]
        for x0,x1,q in zip(X0,X1,X1@R.T+t):
            qh=np.array([[0,-q[2],q[1]],[q[2],0,-q[0]],[-q[1],q[0],0]]);J.append(np.c_[np.eye(3),-qh]);d0=K[0,0]*p['baseline_m']/x0[2];d1=K[0,0]*p['baseline_m']/x1[2];B.append(np.c_[R[:,0]*x1[2]/K[0,0],R[:,1]*x1[2]/K[1,1],x0/d0,-R@x1/d1])
        J=np.concatenate(J);W=block_diag(*[np.linalg.inv(c) for c in C0+R@C1@R.T]);P=np.linalg.inv(J.T@W@J);G=-P@J.T@W;L=np.linalg.cholesky(P);invL=np.linalg.inv(L);D=np.stack([G[:,3*i:3*i+3]@b for i,b in enumerate(B)]);U=D.sum(0);common=[];block=[]
        for j in range(4):common.append(invL@np.outer(U[:,j],U[:,j])@invL.T);block.append(invL@sum(np.outer(x[:,j],x[:,j]) for x in D)@invL.T)
        bases={'common_modes':np.concatenate([np.eye(6)[None],common]),'block_modes':np.concatenate([np.eye(6)[None],block])};features[rec['id']]=bases;Pphysical[rec['id']]=(P,L)
        with np.load(rec['path']) as z:
            close(z['P'],P);close(z['L'],L);close(z['B'],B);close(z['common_basis'],bases['common_modes'],atol=1e-7);close(z['block_basis'],bases['block_modes'],atol=1e-7)
        y=[]
        for row in sorted([x for x in original['rows'] if x['condition']==rec['id'] and x['trial']>=0],key=lambda x:x['trial']):
            log=logm(np.array(row['arms']['fixed_ids']['target_to_source'])@np.linalg.inv(T));ck(abs(np.imag(log)).max()<1e-11,'matrix logarithm real');log=np.real(log);y.append(np.r_[log[:3,3],log[2,1],log[0,2],log[1,0]])
        y=np.array(y)
        with np.load(rec['labels']) as z:close(z['actual_left_tangent'],y,atol=2e-12)
        targets[rec['id']]=(np.linalg.solve(L,y.T).T,y)
    primary=['native','scalar','block_modes','common_modes','pooled_shape'];expected={}
    for fold in models['folds']:
        excluded=fold['excluded_recording'];train=[x for x in index if x['recording']!=excluded];evaluation=[x for x in index if x['recording']==excluded]
        ck(fold['training_scenes']==[x['id'] for x in train] and fold['evaluation_scenes']==[x['id'] for x in evaluation],'recording excluded');ck(len(train)==4 and len(evaluation)==2 and {x['recording'] for x in train}.isdisjoint({excluded}),'recording disjoint')
        moments={x['id']:targets[x['id']][0].T@targets[x['id']][0]/128 for x in train};pooled=sum(moments.values())/4;scalar=np.trace(pooled)/6;close(fold['scalar'],scalar);close(fold['pooled_shape'],pooled)
        for method in ['common_modes','block_modes']:
            result=fold['fits'][method];theta=np.array(result['log_parameters']);ck(np.all(theta>=p['log_bounds'][0]) and np.all(theta<=p['log_bounds'][1]),'fixed log bounds');close(result['coefficients'],np.exp(theta));ck(result['iterations']<=p['optimizer']['maxiter'],'finite optimizer budget')
            def obj(q):
                value=0.;g=np.zeros(5);coefs=np.exp(q)
                for rec in train:
                    basis=features[rec['id']][method];C=sum(a*M for a,M in zip(coefs,basis));inv=np.linalg.inv(C);M=moments[rec['id']];value+=.5*(np.linalg.slogdet(C)[1]+np.trace(inv@M));E=(inv-inv@M@inv)/2
                    for j in range(5):g[j]+=coefs[j]*np.trace(E@basis[j])
                return value/4,g/4
            value,gradient=obj(theta);close(result['objective'],value,atol=1e-7);close(result['gradient'],gradient,atol=1e-6)
            first=result['function_calls'][0];close(first['theta'],np.log([scalar]+[p['initial_mode_variance_px2']]*4));ck(len(result['function_calls'])>0,'retained optimization trace')
            for call in result['function_calls']:
                v,g=obj(np.array(call['theta']));close(call['objective'],v,atol=1e-7);close(call['gradient'],g,atol=2e-5)
            for rec in evaluation:expected[(rec['id'],method)]=sum(a*M for a,M in zip(np.exp(theta),features[rec['id']][method]))
        for rec in evaluation:
            expected[(rec['id'],'native')]=np.eye(6);expected[(rec['id'],'scalar')]=scalar*np.eye(6);expected[(rec['id'],'pooled_shape')]=pooled
    ck(len(pred['rows'])==len(expected)==30,'all30predicted covariances')
    for row in pred['rows']:
        key=(row['scene'],row['method']);S=expected[key];close(row['whitened_covariance'],S,atol=1e-7);L=Pphysical[row['scene']][1];close(row['covariance'],L@S@L.T,atol=1e-12);ck(np.linalg.eigvalsh(S).min()>0,'positive prediction');ck(row['uses_evaluation_errors'] is False,'prediction feature-only declaration')
    ck(len(scores['rows'])==36,'allprimary andoracle scores')
    for row in scores['rows']:
        scene,method=row['scene'],row['method'];z,y=targets[scene];oracle=method=='oracle_test_scalar';S=np.eye(6)*np.mean(np.sum(z*z,axis=1))/6 if oracle else expected[(scene,method)];q=np.sum((z@np.linalg.inv(S))*z,axis=1);nll=.5*(np.linalg.slogdet(S)[1]+q.mean());close(row['nll'],nll,atol=1e-7);close(row['mahalanobis_mean'],q.mean(),atol=1e-7);close(row['baseline_relative_95_containment'],np.mean(q<=chi2.ppf(.95,6)));close(row['predicted_normalized_trace'],np.trace(S));close(row['observed_second_moment_trace'],np.mean(np.sum(z*z,axis=1)))
        close(row['observed_mean_normalized_squared_norm'],np.sum(z.mean(0)**2),atol=1e-12);ck(row['oracle']==oracle and row['samples']==128,'oracle anddenominator')
        L=Pphysical[scene][1];C=L@S@L.T;V=np.cov(y,rowvar=False)
        for key,M,sl in [('predicted_translation_spread_rms_m',C,slice(0,3)),('predicted_rotation_spread_rms_rad',C,slice(3,6)),('observed_centered_translation_spread_rms_m',V,slice(0,3)),('observed_centered_rotation_spread_rms_rad',V,slice(3,6))]:close(row[key],np.sqrt(np.trace(M[sl,sl])),atol=1e-12)
    for fold in scores['folds']:
        for method in primary+['oracle_test_scalar']:close(fold['scene_macro_nll'][method],np.mean([x['nll'] for x in scores['rows'] if x['recording']==fold['excluded_recording'] and x['method']==method]))
        for method,diff in fold['common_minus_controls'].items():close(diff,fold['scene_macro_nll']['common_modes']-fold['scene_macro_nll'][method])
    write(out,{'created_utc':now(),'status':'PASS','checks':checks,'folds':3,'primary_predictions':30,'scored_scene_method_slots':36,'samples':768,'scope':'Independent baseline-mode geometry, original-receipt SE3log labels, recording split, full optimizer traces/objectives/gradients, covariances and excluded-record scores. No physical calibration or new pose fits.'});print('PASS',checks,'independent checks')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--protocol',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();audit(read(a.protocol),Path(a.output))
