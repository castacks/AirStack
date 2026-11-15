"""Independent dense residual-covariance propagation and matrix-log audit."""
import argparse
from pathlib import Path
import numpy as np
from scipy.linalg import block_diag,logm
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
    def close(a,b,rtol=2e-7,atol=1e-18):
        nonlocal checks
        np.testing.assert_allclose(a,b,rtol=rtol,atol=atol);checks+=1
    for f,h in p['pins'].items():ck(sha(f)==h,'immutable pin')
    folder=Path(p['output']);res=read(folder/'results.json');pred=read(p['predictions']);ck(res['protocol_sha256']==sha(p['protocol_path']),'analysis protocol')
    for f,h in read(folder/'artifact-lock.json')['files'].items():ck(sha(folder/f)==h,'result lock')
    for scene in p['scenes']:
        records=sorted([x for x in pred['rows'] if x['condition']==scene],key=lambda x:x['trial']);ck(len(records)==130 and [x['trial'] for x in records]==[-2,-1]+list(range(128)),'complete population');base=records[0]['arms']['fixed_ids'];T=np.array(base['target_to_source']);R=T[:3,:3];t=T[:3,3]
        with np.load(resolve(base['packet_path'])) as z:ids=z['source_ids'];X0,X1,C0,C1=[z[k] for k in ['X0','X1','C0','C1']]
        N=len(ids);W=block_diag(*[np.linalg.inv(c) for c in C0+R@C1@R.T]);J=[]
        for x in X1@R.T+t:
            xh=np.array([[0,-x[2],x[1]],[x[2],0,-x[0]],[-x[1],x[0],0]])
            J.append(np.concatenate([np.eye(3),-xh],axis=1))
        J=np.concatenate(J,axis=0);H=J.T@W@J;G=-np.linalg.inv(H)@J.T@W;close(np.linalg.inv(H),base['information_covariance'],rtol=1e-7,atol=1e-10)
        residual=[];actual=[]
        for x in records[2:]:
            f=x['arms']['fixed_ids'];ck(f['successful'],'existing converged fit')
            with np.load(resolve(f['packet_path'])) as z:
                ck(np.array_equal(ids,z['source_ids']),'fixed population');residual.append((z['X1']@R.T+t-z['X0']-(X1@R.T+t-X0)).ravel())
            delta=np.array(f['target_to_source'])@np.linalg.inv(T);log=logm(delta);ck(np.abs(np.imag(log)).max()<1e-11,'real SE3log');log=np.real(log);actual.append(np.r_[log[:3,3],log[2,1],log[0,2],log[1,0]])
        d=np.array(residual);d-=d.mean(0);Sigma=d.T@d/127;Sigma_block=block_diag(*[Sigma[i:i+3,i:i+3] for i in range(0,3*N,3)])
        V=G@Sigma@G.T;Vb=G@Sigma_block@G.T;actual=np.array(actual);actual-=actual.mean(0);Va=actual.T@actual/127;lin=d@G.T;rem=actual-lin
        row=next(x for x in res['rows'] if x['scene']==scene)
        with np.load(folder/(scene+'.npz')) as z:
            ck(np.array_equal(ids,z['source_ids']),'saved IDs');close(z['H'],H,atol=1e-7);close(z['G'].transpose(1,0,2).reshape(6,-1),G,atol=1e-12)
            for name,M in [('V_full',V),('V_block',Vb),('V_cross',V-Vb),('V_actual',Va)]:close(z[name],M)
            close(z['linear_tangent']-z['linear_tangent'].mean(0),lin,atol=2e-12);close(z['actual_left_tangent']-z['actual_left_tangent'].mean(0),actual,atol=2e-12)
        for name,M in [('full',V),('block',Vb),('cross',V-Vb),('actual',Va)]:
            close(row['normalized_trace'][name],np.trace(H@M),atol=1e-16);close(row['translation_trace_m2'][name],np.trace(M[:3,:3]));close(row['rotation_trace_rad2'][name],np.trace(M[3:,3:]))
        close(row['full_over_block_trace'],np.trace(H@V)/np.trace(H@Vb),atol=1e-10);close(row['linear_over_actual_trace'],np.trace(H@V)/np.trace(H@Va),atol=1e-10)
        numerator=np.mean(np.sum((rem@H)*rem,axis=1));denominator=np.mean(np.sum((actual@H)*actual,axis=1));close(row['relative_centered_linear_remainder_rms'],np.sqrt(numerator/denominator),atol=1e-9)
        ck(np.linalg.eigvalsh(V).min()>=-1e-16 and np.linalg.eigvalsh(Vb).min()>=-1e-16,'propagated covariance PSD');ck(row['trials']==128 and row['points']==N,'row denominators')
    ck(len(res['rows'])==res['scenes']==6 and res['trials']==768 and res['new_model_calls']==res['new_pose_fits']==0,'complete/no reruns')
    write(out,{'created_utc':now(),'status':'PASS','checks':checks,'scenes':6,'saved_trials':768,'scope':'Independent dense residual covariance/full-vs-block propagation, SE3logm, metrics/IDs/source-output binding; no physical calibration or new covariance estimator.'});print('PASS',checks,'independent checks')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--protocol',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();audit(read(a.protocol),Path(a.output))
