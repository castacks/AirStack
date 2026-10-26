"""Independent partition masks and bin joins for selective-risk controls."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from Scripts.Setup.ms2_timing_metadata import sources,sha
from Scripts.Setup.validate_ms2_selective_risk import cutoff_risk


def validate(root,run):
    report=json.loads((run/'report.json').read_text());checks=0
    def check(x):
        nonlocal checks
        assert x;checks+=1
    def close(a,b):
        if a is None or b is None:check(a is b)
        else:check(math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-15))
    check(report['status']=='PASS' and report['new_model_inference'] is False and report['sparse_bins_excluded'] is False)
    check(report['depth_edges_lower_inclusive']==[0.,5.,10.,20.,40.,80.] and report['grid_shape']==[3,3])
    from Scripts.Setup.ms2_selection_controls import grouped_risk
    for u,e,g in [(np.array([1.,1.,3.,2.]),np.array([1.,7.,3.,20.]),np.array([0,0,53,4])),
                  (np.array([1.,1.,1.,1.]),np.array([1.,7.,3.,20.]),np.array([0,0,0,0]))]:
        for c in (1.,.75,.5,.25):
            expected=sum(np.sum(g==k)*cutoff_risk(u[g==k],e[g==k],c) for k in set(g))/len(e)
            close(grouped_risk(u,e,g,c),expected);close(grouped_risk(u[::-1],e[::-1],g[::-1],c),expected)
    # Literal lower-inclusive edge expectations, independent of production searchsorted.
    fixture=np.array([.1,4.999,5.,10.,20.,40.,80.,100.]);bands=np.zeros(len(fixture),dtype=int)
    for edge in (5.,10.,20.,40.,80.):bands+=(fixture>=edge)
    check(bands.tolist()==[0,0,1,2,3,4,5,5])
    prior=json.loads((root/'results/ms2-selective-risk/run-001/report.json').read_text())
    prior={tuple(r[k] for k in ('window','modality','variant','support','stem')):r for r in prior['rows']}
    definitions={}
    for w,(mp,rp) in sources(root).items():
        metadata={r['stem']:r for r in json.loads(mp.read_text())['records']}
        original={(r['modality'],r['variant'],r['stem']):r for r in json.loads(rp.read_text())['records']}
        definitions[w]=(metadata,original,rp)
    baselines=('far_depth','near_depth','central','peripheral');seen=set()
    for row in report['rows']:
        key=tuple(row[k] for k in ('window','modality','variant','support','stem'));w,m,v,s,stem=key
        check(key not in seen);seen.add(key);metadata,original,rp=definitions[w];rec=original[m,v,stem]
        path=rp.parent/rec['file'];check(Path(row['file'])==path);check(sha(path)==rec['sha256'])
        h,width=metadata[stem]['cameras'][m]['left']['shape'][:2];check((row['height'],row['width'])==(h,width))
        with np.load(path,allow_pickle=False) as a:
            mask=np.ones(len(a['x']),dtype=bool) if s=='own_support' else a['common_support']
            x,y=a['x'][mask],a['y'][mask];z=a['depth_m'][mask]
            if s=='common_support':
                with np.load(rp.parent/original[m,'depth',stem]['file'],allow_pickle=False) as p:
                    pm=p['common_support'];check(np.array_equal(x,p['x'][pm]) and np.array_equal(y,p['y'][pm]));z=p['depth_m'][pm]
            check(row['group_depth_source']==('depth' if s=='common_support' else v))
            u=a['covariance_normalized'][mask,0]+a['covariance_normalized'][mask,1]
            e=a['residual_normalized'][mask,0]**2+a['residual_normalized'][mask,1]**2
            check(len(e)==row['count']==rec[s]['count']);check(np.isfinite(e).all() and (e>=0).all() and (z>0).all())
            bands=np.zeros(len(e),dtype=int)
            for edge in (5.,10.,20.,40.,80.):bands+=(z>=edge)
            cols=(x*3>=width).astype(int)+(x*3>=2*width).astype(int)
            lines=(y*3>=h).astype(int)+(y*3>=2*h).astype(int)
            groups=bands*9+lines*3+cols
            bins=[(u[groups==k],e[groups==k]) for k in range(54)];counts=[len(b[0]) for b in bins]
            check(counts==row['group_counts'] and sum(counts)==len(e))
            check(sum(n for n in counts if 0<n<30)==row['sparse_pixels_lt30'])
            radius=((2*x+1-width)/width)**2+((2*y+1-h)/h)**2
            scores={'uncertainty':u,'oracle':e,'far_depth':-z,'near_depth':z,'central':radius,'peripheral':-radius}
            check([p['coverage'] for p in row['curve']]==[1.,.75,.5,.25])
            for j,p in enumerate(row['curve']):
                c=p['coverage'];risks=p['risks'];check(set(risks)==set(scores)|{'random','stratified_uncertainty','stratified_oracle'})
                close(p['retained_mass'],len(e)*c);close(sum(n*c for n in counts),p['retained_mass']);close(risks['random'],float(e.mean()))
                for name,score in scores.items():close(risks[name],cutoff_risk(score,e,c))
                for name in ('stratified_uncertainty','stratified_oracle'):
                    expected=sum(len(b)*cutoff_risk(a if name=='stratified_uncertainty' else b,b,c) for a,b in bins if len(b))/len(e)
                    close(risks[name],expected)
                close(p['stratified_ratio'],risks['stratified_uncertainty']/risks['random'])
                for b in baselines:close(p['uncertainty_to_baseline'][b],risks['uncertainty']/risks[b] if risks[b]>0 else None)
                previous=prior[key]['curve'][j]
                close(risks['uncertainty'],previous['low_uncertainty_mse']);close(risks['oracle'],previous['oracle_mse']);close(risks['random'],previous['random_expected_mse'])
                for name,value in risks.items():
                    check(np.isfinite(value) and value>=0)
                    check(risks['oracle']<=value+1e-15)
                    if c==1.:close(value,risks['random'])
                check(risks['stratified_oracle']<=risks['stratified_uncertainty']+1e-15)
                check(risks['stratified_oracle']<=risks['random']+1e-15)
    check(len(seen)==960);aggregate_seen=set()
    for a in report['aggregate']:
        key=tuple(a[k] for k in ('window','modality','variant','support','coverage'));check(key not in aggregate_seen);aggregate_seen.add(key)
        rows=[r for r in report['rows'] if tuple(r[k] for k in ('window','modality','variant','support'))==key[:4]];check(len(rows)==20)
        points=[next(p for p in r['curve'] if p['coverage']==key[4]) for r in rows]
        summaries=[(a['stratified_ratio'],[p['stratified_ratio'] for p in points])]
        summaries += [(a['uncertainty_to_baseline'][b],[p['uncertainty_to_baseline'][b] for p in points]) for b in baselines]
        for summary,values in summaries:
            vals=sorted(x for x in values if x is not None);check(summary['available_images']==len(vals))
            check(summary['images_below_one']==sum(x<1-1e-12 for x in vals))
            if vals:
                n=len(vals);med=vals[n//2] if n%2 else (vals[n//2-1]+vals[n//2])/2
                for got,want in zip(summary['min_median_max'],[vals[0],med,vals[-1]]):close(got,want)
            else:check(summary['min_median_max'] is None)
        check(a['primary']==(key[2]=='depth' and key[3]=='own_support' and key[4]==.5))
    check(len(aggregate_seen)==192 and sum(a['primary'] for a in report['aggregate'])==8)
    frozen=json.loads((run/'frozen-inputs.json').read_text())
    for p,h in frozen.items():check(sha(p)==h)
    history=0
    for name in ('exp-043','exp-044-complete','exp-046','exp-047','exp-048','exp-049','exp-050','exp-051'):
        manifest=json.loads((Path('/root/ore-ms-recovery')/name/'manifest.json').read_text())
        for p,h in manifest['member_sha256'].items():
            if p.startswith('ore-ms/results/'):check(sha(root.parent/p)==h);history+=1
    return {'status':'PASS','checks':checks,'cases':len(seen),'curve_points':len(seen)*4,'frozen_inputs':len(frozen),
            'historical_hash_checks':history,'exp051_global_curve_points_reproduced':3840,'validator_sha256':sha(__file__)}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    assert not a.output.exists();result=validate(a.root,a.run);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
