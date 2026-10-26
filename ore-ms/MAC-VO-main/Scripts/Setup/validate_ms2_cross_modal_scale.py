"""Check image-wide scale control directly against raw model variances/disparities."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import numpy as np


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def validate(root,base,output):
    if output.exists():raise FileExistsError(output)
    r=json.loads((base/'scale-control-run-001/report.json').read_text());cal={k:np.array(v['values'],float) for k,v in json.loads((root/'results/ms2-rain-replication/calibration.json').read_text()).items()};risk_checks=pair_predictions=0
    for row in r['rows']:
        stem=row['stem']
        with np.load(base/'run-001'/f'pairs-{stem}.npz') as f:p={k:f[k] for k in f.files}
        with np.load(base/'metric-run-001'/f'metric-{stem}.npz') as f:m={k:f[k] for k in f.files}
        with np.load(base/'scale-control-run-001'/f'scale-{stem}.npz') as f:saved=f['choose_rgb']
        scores={}
        for mod in ('rgb','thr'):
            k=cal[f'K_{mod}L'];fx=float(k[0,0]);fy=float(k[1,1]);b=-float(cal[f'T_{mod}R'][0,0])/1000
            with np.load(root/'results/ms2-rain-replication/frontend-run-001'/f'{mod}-stereo-{stem}-{stem}.npz') as f:flow=f['flow'][0];cov=f['covariance'][0]
            angular=[];geometry=[]
            for u,v in p[f'{mod}_xy'][m['both_valid_mask']]:
                d=-float(flow[0,v,u]);angular.append(float(cov[0,v,u])/fx**2)
                ray2=((int(u)-float(k[0,2]))/fx)**2+((int(v)-float(k[1,2]))/fy)**2+1
                geometry.append(ray2*(fx*b/d)**4/b**2);pair_predictions+=1
            median=statistics.median(angular);assert math.isclose(median,row['median_angular_variances'][mod],rel_tol=1e-10)
            scores[mod]=[median*g for g in geometry]
        choose=[1. if a<b else 0. if a>b else .5 for a,b in zip(scores['rgb'],scores['thr'])];assert np.array_equal(choose,saved)
        for anchor in ('midpoint','rgb','thr'):
            for loss,power in [('mse',2),('mean_l2',1)]:
                ar=[float(x)**power for x in m[f'{anchor}_rgb_error_m']];at=[float(x)**power for x in m[f'{anchor}_thr_error_m']]
                risk=math.fsum(s*a+(1-s)*b for s,a,b in zip(choose,ar,at))/len(ar)
                learned=math.fsum(float(s)*a+(1-float(s))*b for s,a,b in zip(m['choose_rgb_on_both_valid'],ar,at))/len(ar)
                assert math.isclose(risk,row['scores'][anchor][loss]['constant_scale']['uncertainty'],rel_tol=1e-10)
                assert math.isclose(learned/risk,row['scores'][anchor][loss]['learned_over_constant_scale'],rel_tol=1e-10);risk_checks+=1
    for p,h in r['frozen_input_sha256'].items():assert digest(p)==h
    result={'status':'PASS','frames':20,'raw_variance_geometry_pairs_checked':pair_predictions,'scale_control_risks_checked':risk_checks,'frozen_hash_checks':len(r['frozen_input_sha256']),'validator_sha256':digest(__file__)}
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ['root','base','output']:p.add_argument('--'+n,type=Path,required=True)
    a=p.parse_args();validate(a.root,a.base,a.output)
