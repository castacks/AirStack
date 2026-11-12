"""Independent NumPy/SciPy audit of saved inputs; imports no prototype code."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('run'); args=ap.parse_args()
    run=Path(args.run); report=json.loads((run/'report.json').read_text())
    with np.load(run/'fixed-inputs.npz') as z: data={k:z[k] for k in z.files}
    def residual(x):
        b=data['points']@Rotation.from_rotvec(x[3:]).as_matrix().T+x[:3]
        c=np.einsum('nij,nj->ni',data['camera_R'],b)+data['camera_t']
        fx,fy,cx,cy,baseline=data['intrinsics'].T
        pred=np.column_stack((fx*c[:,0]/c[:,2]+cx,fy*c[:,1]/c[:,2]+cy,fx*baseline/c[:,2]))
        return (pred-data['observations']).ravel()
    def loss(v):
        weight=np.exp(-.5*(v-v.mean())).ravel()
        fit=least_squares(lambda x:weight*residual(x),np.zeros(6),jac='3-point',method='trf',ftol=1e-14,xtol=1e-14,gtol=1e-14,max_nfev=200)
        assert fit.success,fit.message
        R=Rotation.from_rotvec(fit.x[3:]).as_matrix()
        return float(np.mean((R-data['Rref'])**2)+np.sum((fit.x[:3]-data['tref'])**2))
    cases={c['name']:c for c in report['cases']}
    expected=cases['rgb_anisotropic_error']['evidence']['loss']; actual=loss(np.zeros((24,3)))
    checks=[{'name':'independent_pose_loss','producer':expected,'independent':actual,'pass':bool(np.isclose(actual,expected,rtol=1e-5,atol=1e-10))}]
    for probe in cases['pose_gradient_rgb_noise']['evidence']['probes']:
        i,j=probe['row'],probe['channel']; plus=np.zeros((24,3)); minus=plus.copy()
        plus[i,j]=1e-4; minus[i,j]=-1e-4
        fd=(loss(plus)-loss(minus))/2e-4
        checks.append({'name':f'gradient_{i}_{j}','autograd':probe['autograd'],'independent':fd,'abs_error':abs(fd-probe['autograd']),'pass':bool(np.isclose(fd,probe['autograd'],rtol=.005,atol=2e-8))})
    result={'method':'Independent SciPy trust-region solve and central differences; no producer imports. RGB-noise packet only.','source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'checks':checks,'pass':all(c['pass'] for c in checks)}
    dest=run/'independent-audit.json'
    with dest.open('x') as f: json.dump(result,f,indent=2); f.write('\n')
    print(json.dumps(result)); return 0 if result['pass'] else 1
if __name__=='__main__': raise SystemExit(main())
