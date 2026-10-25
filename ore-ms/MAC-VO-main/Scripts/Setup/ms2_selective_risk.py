"""Fixed-coverage descriptive reference risk; no model fitting or deployment policy."""
import argparse
import json
from pathlib import Path
import numpy as np
from Scripts.Setup.ms2_timing_metadata import sources, sha

COVERAGES=(1.,.75,.5,.25)


def risk(score, loss, coverage):
    """Expected risk under uniform fractional retention of a tied cutoff group."""
    order=np.argsort(score,kind='stable');s=score[order];v=loss[order]
    mass=len(s)*coverage;boundary=s[int(np.ceil(mass))-1]
    lo=int(np.searchsorted(s,boundary,side='left'));hi=int(np.searchsorted(s,boundary,side='right'))
    total=float(v[:lo].sum())+(mass-lo)*float(v[lo:hi].mean())
    return total/mass



def run(root,output):
    root=Path(root).resolve();output=Path(output);output.mkdir(parents=True,exist_ok=False)
    frozen={str(Path(__file__).resolve()):sha(__file__)};rows=[]
    for window,(_,p) in sources(root).items():
        frozen[str(p)]=sha(p);report=json.loads(p.read_text());assert report['status']=='PASS'
        for rec in report['records']:
            f=p.parent/rec['file'];assert sha(f)==rec['sha256'];frozen[str(f)]=sha(f)
            with np.load(f,allow_pickle=False) as a:
                cov=a['covariance_normalized'];res=a['residual_normalized']
                assert cov.shape==res.shape and cov.shape[1]==2
                uncertainty=cov.sum(axis=1);loss=np.square(res).sum(axis=1)
                assert np.isfinite(uncertainty).all() and np.isfinite(loss).all() and (cov>=0).all()
                for support in ('own_support','common_support'):
                    mask=np.ones(len(loss),dtype=bool) if support=='own_support' else a['common_support']
                    u,e=uncertainty[mask],loss[mask];assert len(e)==rec[support]['count'] and len(e)>0
                    baseline=float(e.mean());assert baseline>0
                    curve=[]
                    for c in COVERAGES:
                        low,oracle,high=(risk(s,e,c) for s in (u,e,-u))
                        curve.append({'coverage':c,'retained_mass':c*len(e),'low_uncertainty_mse':low,
                                      'random_expected_mse':baseline,'oracle_mse':oracle,'high_uncertainty_mse':high,
                                      'risk_ratio':low/baseline,'oracle_ratio':oracle/baseline,'high_ratio':high/baseline})
                    rows.append({'window':window,'modality':rec['modality'],'variant':rec['variant'],'stem':rec['stem'],
                                 'support':support,'count':len(e),'file':str(f),'curve':curve})
    aggregates=[]
    for w in sources(root):
        for m in ('rgb','thr'):
            for v in ('depth','depth_multi','depth_filtered'):
                for s in ('own_support','common_support'):
                    g=[r for r in rows if (r['window'],r['modality'],r['variant'],r['support'])==(w,m,v,s)];assert len(g)==20
                    for j,c in enumerate(COVERAGES):
                        ratios=[r['curve'][j]['risk_ratio'] for r in g]
                        aggregates.append({'window':w,'modality':m,'variant':v,'support':s,'coverage':c,'images':20,
                                           'ratio_min_median_max':np.quantile(ratios,[0,.5,1]).tolist(),
                                           'images_below_random':sum(x<1-1e-12 for x in ratios),
                                           'primary':v=='depth' and s=='own_support' and c==.5})
    assert len(rows)==960 and len(aggregates)==192
    assert all(sha(p)==h for p,h in frozen.items())
    (output/'frozen-inputs.json').write_text(json.dumps(frozen,indent=2)+'\n')
    (output/'report.json').write_text(json.dumps({'status':'PASS','scope':'provided-reference selective risk, exploratory',
        'rows':rows,'aggregate':aggregates},indent=2,allow_nan=False)+'\n')
    print('PASS:960 cases,3840 curve points')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.root,a.output)
