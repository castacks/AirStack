"""Independent source, timing, eligibility and scipy-statistic reconstruction."""
from pathlib import Path
import copy
import hashlib
import json
import math
import numpy as np
from scipy.stats import spearmanr
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/covariance-lead-time/run-001'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def audit(d):
    p=json.loads((OUT/'protocol.json').read_text());assert d['protocol_sha256']==sha(OUT/'protocol.json')
    for n,h in p['sources'].items(): assert sha(ROOT/n)==h
    assert len(d['summaries'])==48 and len(d['bindings'])==3102
    original={}
    f=json.loads((ROOT/'results/temporal-gate-development/run-001/features.json').read_text())
    labels=json.loads((ROOT/'results/temporal-gate-development/run-001/labels.json').read_text())
    names=f['feature_names']
    for r in f['rows']:
        group=r['recording_id']+'::'+r['window'];v=r['vector'];gaps=[]
        if v is not None:
            for lag in range(3):gaps.append(v[names.index(f'rgb/lag{lag}/log_median_trace')]-v[names.index(f'thermal/lag{lag}/log_median_trace')])
        e={'id':r['sample_id'],'ordinal':r['pair_ordinal'],'start':r['common_interval_ns'][0],'end':r['common_interval_ns'][1],
            'eligible':r['eligible'],'both_supported':r['both_supported'],'current_gap':gaps[0] if gaps else None,'history_gap':sum(gaps)/3 if gaps else None}
        for key in ['translation_m','rotation_deg']:
            l=labels[r['sample_id']];e[key]=l['rgb'][key]**2-l['thermal'][key]**2 if l['rgb'] and l['thermal'] else None
        original.setdefault(('gate',group),[]).append(e)
    plan=json.loads((ROOT/'results/long-night-validation/angular-001/plan.json').read_text())
    data=json.loads((ROOT/'results/long-night-validation/angular-001/analysis.json').read_text())
    for c in plan['cases']:
        group=c['case'];mod=c['modality'];rows=[]
        for i in range(99):
            a=next(x for x in data['pairs'] if (x['case'],x['pair'],x['population'])==(group,i,'all'))
            b=next(x for x in data['pairs'] if (x['case'],x['pair'],x['population'])==(group,i,'lk_valid'))
            def med(r,k):return r[k]['p10_p50_p90'][1] if r[k]['finite_count'] else None
            def log(x):return math.log(x) if x is not None and x>0 and math.isfinite(x) else None
            rows.append({'id':f'{group}/{i}','ordinal':i,'start':c['rows'][i][mod+'_left']['time_ns'],'end':c['rows'][i+1][mod+'_left']['time_ns'],
                'log_variance':log(med(a,'variance_rad2')),'log_disagreement':log(med(b,'disagreement_rad')),
                'source_std':med(a,'source_std'),'support':b['count'],'grid':a['grid_count']})
        original['long_night',group]=rows
    checks=0
    def equal(x,y):
        nonlocal checks
        checks+=1
        if isinstance(x,dict):
            assert x.keys()==y.keys(),(x.keys(),y.keys())
            for k in x:equal(x[k],y[k])
        elif isinstance(x,(float,np.floating)):
            assert math.isclose(x,y,rel_tol=1e-12,abs_tol=1e-13),(x,y)
        else:assert x==y,(x,y)
    def desc(v):return {'min':min(v),'median':float(np.median(v)),'max':max(v)} if v else None
    for (source,group),rows in original.items():
        rows.sort(key=lambda x:x['ordinal'])
        for h in p['horizons_seconds']:
            expected=[]
            for i,a in enumerate(rows):
                opts=[]
                for b in rows[i+1:]:
                    if b['start']>=a['end'] and abs(b['start']-a['end']-round(h*1e9))<=50_000_000:opts.append(b)
                if not opts:continue
                b=sorted(opts,key=lambda b:(abs(b['start']-a['end']-round(h*1e9)),b['start']))[0]
                ok=True if source=='long_night' else a['eligible'] and b['both_supported'] and all(a[k] is not None and b[k] is not None for k in ['translation_m','rotation_deg'])
                expected.append({'source':source,'group':group,'horizon':h,'origin':a,'target':b,'eligible':bool(ok),
                    'start_lead_s':(b['start']-a['end'])/1e9,'end_lead_s':(b['end']-a['end'])/1e9})
            observed=[b for b in d['bindings'] if (b['source'],b['group'],b['horizon'])==(source,group,h)]
            equal(len(expected),len(observed))
            for a,b in zip(expected,observed):equal(a,b)
            s=next(x for x in d['summaries'] if (x['source'],x['group'],x['horizon'])==(source,group,h))
            selected=[b for b in expected if b['eligible']]
            equal(s['origins'],len(rows));equal(s['eligible'],len(selected));equal(s['time_matched'],len(expected))
            equal(s['realized_start_lead_s'],desc([r['start_lead_s'] for r in selected]));equal(s['realized_end_lead_s'],desc([r['end_lead_s'] for r in selected]))
            if source=='long_night':
                keys={'covariance_persistence':('log_variance','log_variance'),'covariance_future_disagreement':('log_variance','log_disagreement'),
                    'disagreement_persistence_oracle':('log_disagreement','log_disagreement'),'texture_future_disagreement':('source_std','log_disagreement')}
                changes=[b['target']['log_variance']-b['origin']['log_variance'] for b in selected]
                equal(s['log_covariance_change'],desc(changes));equal(s['carry_forward_abs_log_error'],desc(list(map(abs,changes))));equal(s['future_support'],desc([b['target']['support'] for b in selected]))
            else:keys={f'{x}_{y}':(x,y) for y in ['translation_m','rotation_deg'] for x in ['current_gap','history_gap',y]}
            equal(set(s['associations']),set(keys))
            for name,(x,y) in keys.items():
                values=[(b['origin'][x],b['target'][y]) for b in selected if b['origin'][x] is not None and b['target'][y] is not None and math.isfinite(b['origin'][x]) and math.isfinite(b['target'][y])]
                n=len(values);a=s['associations'][name];equal(a['n'],n)
                status='fewer_than_8' if n<8 else 'constant' if len(set(x for x,y in values))==1 or len(set(y for x,y in values))==1 else 'defined'
                equal(a['status'],status);equal(a['rho'],float(spearmanr(*zip(*values)).statistic) if status=='defined' else None)
    return checks

if __name__=='__main__':
    d=json.loads((OUT/'diagnostic.json').read_text());n=audit(d);mutations=[]
    for name in ['timestamp','window','eligibility','statistic']:
        x=copy.deepcopy(d)
        if name=='timestamp':x['bindings'][0]['target']['start']+=1
        if name=='window':x['bindings'][0]['group']='different-window'
        if name=='eligibility':x['bindings'][0]['eligible']=False
        if name=='statistic':x['summaries'][0]['associations']['covariance_persistence']['rho']+=.01
        try:audit(x)
        except AssertionError:mutations.append(name)
        else:raise AssertionError('mutation accepted: '+name)
    result={'status':'PASS_INDEPENDENT_LEAD_AUDIT','checks':n,'mutations_rejected':mutations,'diagnostic_sha256':sha(OUT/'diagnostic.json'),'auditor_sha256':sha(__file__)}
    (OUT/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
