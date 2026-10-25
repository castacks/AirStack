"""Independent cutoff masks, support joins, aggregate and preservation checks."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from Scripts.Setup.ms2_timing_metadata import sources,sha


def cutoff_risk(score,loss,c):
    mass=c*len(loss);threshold=np.partition(score,int(math.ceil(mass))-1)[int(math.ceil(mass))-1]
    below=score<threshold;equal=score==threshold
    return float((loss[below].sum()+(mass-below.sum())*loss[equal].mean())/mass)


def validate(root,run):
    report=json.loads((run/'report.json').read_text());checks=0
    def check(x):
        nonlocal checks
        assert x;checks+=1
    def close(x,y):check(math.isclose(x,y,rel_tol=1e-10,abs_tol=1e-15))
    from Scripts.Setup.ms2_selective_risk import risk
    for scores in ([1.,1.,1.,1.],[3.,1.,1.,2.]):
        u=np.array(scores);e=np.array([1.,3.,8.,20.])
        for c in (1.,.75,.5,.25):
            expected=cutoff_risk(u,e,c);close(risk(u,e,c),expected)
            close(risk(u[::-1],e[::-1],c),expected)
    old={w:{(r['modality'],r['variant'],r['stem']):r for r in json.loads(p.read_text())['records']} for w,(_,p) in sources(root).items()}
    seen=set()
    for row in report['rows']:
        w,m,v,s,stem=[row[k] for k in ('window','modality','variant','support','stem')];key=w,m,v,s,stem
        check(key not in seen);seen.add(key);rec=old[w][m,v,stem]
        expected_path=sources(root)[w][1].parent/rec['file'];check(Path(row['file'])==expected_path);check(sha(expected_path)==rec['sha256'])
        with np.load(expected_path,allow_pickle=False) as a:
            mask=np.ones(len(a['x']),dtype=bool) if s=='own_support' else a['common_support']
            e=(a['residual_normalized'][mask,0]**2+a['residual_normalized'][mask,1]**2)
            u=a['covariance_normalized'][mask,0]+a['covariance_normalized'][mask,1]
            check(row['count']==len(e)==rec[s]['count']);base=float(e.mean())
            check([x['coverage'] for x in row['curve']]==[1.,.75,.5,.25])
            for p in row['curve']:
                c=p['coverage'];close(p['retained_mass'],len(e)*c);close(p['random_expected_mse'],base)
                for field,score in [('low_uncertainty_mse',u),('oracle_mse',e),('high_uncertainty_mse',-u)]:close(p[field],cutoff_risk(score,e,c))
                for field,num in [('risk_ratio','low_uncertainty_mse'),('oracle_ratio','oracle_mse'),('high_ratio','high_uncertainty_mse')]:close(p[field],p[num]/base)
                check(p['oracle_mse']<=p['low_uncertainty_mse']+1e-15)
                check(p['oracle_mse']<=base+1e-15)
                if c==1.:
                    for k in ('risk_ratio','oracle_ratio','high_ratio'):close(p[k],1.)
    check(len(seen)==960);aggregate_seen=set()
    for a in report['aggregate']:
        key=tuple(a[k] for k in ('window','modality','variant','support','coverage'));check(key not in aggregate_seen);aggregate_seen.add(key)
        g=[r for r in report['rows'] if tuple(r[k] for k in ('window','modality','variant','support'))==key[:4]];check(len(g)==20)
        vals=sorted(next(p['risk_ratio'] for p in r['curve'] if p['coverage']==key[4]) for r in g)
        for got,want in zip(a['ratio_min_median_max'],[vals[0],(vals[9]+vals[10])/2,vals[-1]]):close(got,want)
        check(a['images_below_random']==sum(x<1-1e-12 for x in vals));check(a['primary']==(key[2]=='depth' and key[3]=='own_support' and key[4]==.5))
    check(len(aggregate_seen)==192 and sum(a['primary'] for a in report['aggregate'])==8)
    frozen=json.loads((run/'frozen-inputs.json').read_text())
    for p,h in frozen.items():check(sha(p)==h)
    history=0
    for name in ('exp-043','exp-044-complete','exp-046','exp-047','exp-048','exp-049','exp-050'):
        manifest=json.loads((Path('/root/ore-ms-recovery')/name/'manifest.json').read_text())
        for p,h in manifest['member_sha256'].items():
            if p.startswith('ore-ms/results/'):check(sha(root.parent/p)==h);history+=1
    return {'status':'PASS','checks':checks,'cases':len(seen),'curve_points':len(seen)*4,'frozen_inputs':len(frozen),'historical_hash_checks':history}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    assert not a.output.exists();result=validate(a.root,a.run);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
