"""Post-result independent saved-array audit; imports no producer code."""
import json
import hashlib
from pathlib import Path
import numpy as np
from scipy.stats import chi2


def audit(folder):
    folder=Path(folder);report=json.loads((folder/'report.json').read_text())
    assert hashlib.sha256((folder/'arrays.npz').read_bytes()).hexdigest()==report['arrays_sha256']
    a=np.load(folder/'arrays.npz');H=a['H'];sc=np.tile([.1,.1,.1,.01,.01,.01],4)
    checks=1;details={}
    for case,rows in report['cases'].items():
        C=a[f'{case}_true_cov'];D=np.zeros_like(C)
        for i in range(0,192,2):D[i:i+2,i:i+2]=C[i:i+2,i:i+2]
        matrices={}
        for name,M in [('joint_oracle',C),('independent',D),('inflation4',4*D)]:
            values,vectors=np.linalg.eigh(M);assert values.min()>0
            precision=(vectors/values)@vectors.T
            P=np.linalg.solve(H.T@precision@H,np.eye(24));E=P@H.T@precision
            assert np.allclose(E@H,np.eye(24),atol=1e-9,rtol=0)
            assert np.allclose(P,a[f'{case}_{name}_reported_cov'],atol=1e-10,rtol=1e-7)
            errors=a[f'{case}_{name}_errors'];r=rows[name]
            risk=np.mean(np.sum((errors/sc[:,None])**2,axis=0))
            assert np.isclose(risk,r['normalized_risk'],rtol=1e-12)
            maha=np.einsum('in,ij,jn->n',errors,np.linalg.solve(P,np.eye(24)),errors)
            coverage=float(np.mean(maha<=chi2.ppf(.95,24)))
            assert coverage==r['coverage95_joint24']
            sign,logdet=np.linalg.slogdet(P);assert sign==1
            assert np.isclose(.5*(maha.mean()+logdet+24*np.log(2*np.pi)),r['gaussian_nll_mean'],rtol=1e-10,atol=1e-9)
            bias=np.tile([.015,-.01,.005,.002,-.001,.001],4) if case=='bias_control' else np.zeros(24)
            analytical=np.trace((E@C@E.T)/sc[:,None]/sc[None,:])+np.sum((bias/sc)**2)
            assert np.isclose(analytical,r['analytic_normalized_risk'],rtol=1e-8)
            mean_z=np.abs(errors.mean(1)-bias)/np.sqrt(np.diag(E@C@E.T)/errors.shape[1])
            assert mean_z.max()<5
            details[f'{case}/{name}']={'mean_error_max_standard_errors':float(mean_z.max())}
            matrices[name]=E;checks+=8
        assert np.allclose(matrices['inflation4'],matrices['independent'],atol=1e-10,rtol=0);checks+=1
        if case in ['independent_increments','bias_control']:
            # Difference accumulated positions back to increments; all off-time
            # blocks must vanish even though position off-time blocks are nonzero.
            delta=np.eye(4)-np.diag(np.ones(3),-1);T=np.kron(delta,np.eye(2))
            for k in range(24):
                block=C[k*8:k*8+8,k*8:k*8+8];inc=T@block@T.T
                diag=np.zeros_like(inc)
                for t in range(4):diag[t*2:t*2+2,t*2:t*2+2]=inc[t*2:t*2+2,t*2:t*2+2]
                assert np.max(np.abs(inc-diag))<1e-12
                assert np.linalg.norm(block[:2,2:4])>0
                checks+=2
    out={'status':'PASS','checks':checks,'scope':'Saved-array GLS, metrics, bias expectation and accumulated-position covariance; post-result, not blind; no real-data or nonlinear-solver validation','details':details}
    (folder.parent/'independent-audit.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({'status':out['status'],'checks':checks}))


if __name__=='__main__':
    import sys
    audit(sys.argv[1])
