"""Retrospective interval-aligned diagnostics. No fitting or neural execution."""
from pathlib import Path
import datetime
import hashlib
import json
import math
import numpy as np
from scipy.stats import rankdata

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = ROOT/'docs/research/covariance-lead-time-protocol-2026-11-02.json'
OUT = ROOT/'results/covariance-lead-time/run-001'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def positive_log(x):
    return math.log(x) if x is not None and math.isfinite(x) and x > 0 else None

def median(row, key):
    return row[key]['p10_p50_p90'][1] if row[key]['finite_count'] else None

def match(rows, horizon):
    """Return local index bindings, using integer nanoseconds throughout."""
    result = []
    for i, origin in enumerate(rows):
        candidates = [(abs(t['start']-origin['end']-round(horizon*1e9)), t['start'], j)
                      for j, t in enumerate(rows) if j > i and t['start'] >= origin['end']]
        if candidates:
            distance, _, j = min(candidates)
            if distance <= 50_000_000:
                result.append((i, j))
    return result

def association(x, y):
    valid = [(a,b) for a,b in zip(x,y) if a is not None and b is not None
             and math.isfinite(a) and math.isfinite(b)]
    if len(valid) < 8:
        return {'n':len(valid), 'rho':None, 'status':'fewer_than_8'}
    a,b = np.asarray(valid).T
    if np.ptp(a) == 0 or np.ptp(b) == 0:
        return {'n':len(valid), 'rho':None, 'status':'constant'}
    return {'n':len(valid), 'rho':float(np.corrcoef(rankdata(a),rankdata(b))[0,1]), 'status':'defined'}

def series():
    analysis = json.loads((ROOT/'results/long-night-validation/angular-001/analysis.json').read_text())
    plan = json.loads((ROOT/'results/long-night-validation/angular-001/plan.json').read_text())
    lookup = {(r['case'],r['pair'],r['population']):r for r in analysis['pairs']}
    long = {}
    for case in plan['cases']:
        rows=[]; modality=case['modality']
        for i in range(len(case['rows'])-1):
            a=lookup[case['case'],i,'all']; b=lookup[case['case'],i,'lk_valid']
            rows.append({'id':f"{case['case']}/{i}", 'ordinal':i,
                'start':case['rows'][i][modality+'_left']['time_ns'],
                'end':case['rows'][i+1][modality+'_left']['time_ns'],
                'log_variance':positive_log(median(a,'variance_rad2')),
                'log_disagreement':positive_log(median(b,'disagreement_rad')),
                'source_std':median(a,'source_std'), 'support':b['count'], 'grid':a['grid_count']})
        long[case['case']]=rows
    f=json.loads((ROOT/'results/temporal-gate-development/run-001/features.json').read_text())
    labels=json.loads((ROOT/'results/temporal-gate-development/run-001/labels.json').read_text())
    names=f['feature_names']; gate={}
    for r in f['rows']:
        v=r['vector']; gaps=[v[names.index(f'rgb/lag{k}/log_median_trace')]-v[names.index(f'thermal/lag{k}/log_median_trace')] for k in range(3)] if v is not None else None
        losses={}
        for target in ['translation_m','rotation_deg']:
            label=labels.get(r['sample_id'],{});rgb=label.get('rgb');thermal=label.get('thermal')
            losses[target]=rgb[target]**2-thermal[target]**2 if rgb and thermal else None
        key=r['recording_id']+'::'+r['window']
        gate.setdefault(key,[]).append({'id':r['sample_id'],'ordinal':r['pair_ordinal'],
            'start':r['common_interval_ns'][0],'end':r['common_interval_ns'][1],
            'eligible':r['eligible'],'both_supported':r['both_supported'],
            'current_gap':gaps[0] if gaps else None,'history_gap':sum(gaps)/3 if gaps else None,
            **losses})
    for rows in [*long.values(),*gate.values()]:
        rows.sort(key=lambda r:r['ordinal'])
        assert all(a['ordinal']<b['ordinal'] and a['end']<=b['start'] for a,b in zip(rows,rows[1:]))
    return long,gate

def describe(values):
    return {'min':min(values),'median':float(np.median(values)),'max':max(values)} if values else None

def compute(protocol):
    long,gate=series();bindings=[];summaries=[]
    for source, groups in [('long_night',long),('gate',gate)]:
        for group, rows in groups.items():
            for h in protocol['horizons_seconds']:
                pairs=match(rows,h);bound=[]
                for i,j in pairs:
                    a,b=rows[i],rows[j]
                    if source=='gate':
                        eligible=a['eligible'] and b['both_supported'] and all(a[t] is not None and b[t] is not None for t in ['translation_m','rotation_deg'])
                    else: eligible=True
                    bound.append({'source':source,'group':group,'horizon':h,'origin':a,'target':b,'eligible':bool(eligible),
                        'start_lead_s':(b['start']-a['end'])/1e9,'end_lead_s':(b['end']-a['end'])/1e9})
                bindings.extend(bound);chosen=[r for r in bound if r['eligible']]
                summary={'source':source,'group':group,'horizon':h,'origins':len(rows),
                    'time_matched':len(bound),'eligible':len(chosen),
                    'realized_start_lead_s':describe([r['start_lead_s'] for r in chosen]),
                    'realized_end_lead_s':describe([r['end_lead_s'] for r in chosen]),'associations':{}}
                if source=='long_night':
                    tests=[('covariance_persistence','log_variance','log_variance'),
                        ('covariance_future_disagreement','log_variance','log_disagreement'),
                        ('disagreement_persistence_oracle','log_disagreement','log_disagreement'),
                        ('texture_future_disagreement','source_std','log_disagreement')]
                    changes=[r['target']['log_variance']-r['origin']['log_variance'] for r in chosen]
                    summary['log_covariance_change']=describe(changes)
                    summary['carry_forward_abs_log_error']=describe(list(map(abs,changes)))
                    summary['future_support']=describe([r['target']['support'] for r in chosen])
                else:
                    tests=[(f'{pred}_{target}',pred,target) for target in ['translation_m','rotation_deg'] for pred in ['current_gap','history_gap',target]]
                for label,x,y in tests:
                    summary['associations'][label]=association([r['origin'][x] for r in chosen],[r['target'][y] for r in chosen])
                summaries.append(summary)
    return {'created_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'protocol_sha256':sha(PROTOCOL),'bindings':bindings,'summaries':summaries,
        'inference_calls':0,'optimizer_steps':0,'candidate_promoted':False}

def main():
    p=json.loads(PROTOCOL.read_text())
    for path,digest in p['sources'].items(): assert sha(ROOT/path)==digest,path
    OUT.mkdir(parents=True,exist_ok=False)
    (OUT/'protocol.json').write_bytes(PROTOCOL.read_bytes())
    result=compute(p)
    (OUT/'diagnostic.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for path,digest in p['sources'].items(): assert sha(ROOT/path)==digest,path
    print('Saved',len(result['bindings']),'bindings and',len(result['summaries']),'summaries')

if __name__=='__main__': main()
