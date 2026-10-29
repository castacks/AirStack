"""Independent raw covariance, prefix and diagnostic population reconstruction."""
import argparse
import hashlib
import json
from pathlib import Path
import math
import numpy as np
from scipy.stats import rankdata

DIRECTIONS={'flow_trace_px2':'lower','ray_trace_rad2':'lower','retained_fraction':'higher','occupied_cells':'higher','prior_supported':'higher'}


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def check_scalar(actual,expected,angular=False):
    if expected is None:assert actual is None
    else:np.testing.assert_allclose(actual,expected,rtol=1e-8 if angular else 1e-6,atol=1e-20 if angular else 1e-12)


def population(actual,values):
    assert actual['count']==len(values)
    if not values:assert actual['rmse'] is None and actual['mean'] is None
    else:
        check_scalar(actual['mean'],math.fsum(values)/len(values))
        check_scalar(actual['rmse'],math.sqrt(math.fsum(v*v for v in values)/len(values)))


def validate(root,run):
    protocol=json.loads((run/'protocol.json').read_text());features=json.loads((run/'features.json').read_text());analysis=json.loads((run/'analysis.json').read_text())
    assert protocol['features']==DIRECTIONS and not protocol['labels_loaded'] and not protocol['newer_intervals_used'] and not protocol['fitted']
    assert not features['labels_loaded'] and features['protocol_sha256']==sha(run/'protocol.json')
    assert analysis['features_sha256']==sha(run/'features.json')
    for rel,d in protocol['input_sha256'].items():assert sha(root/rel)==d
    assert all('pose-proxy' not in rel and 'proxy-001' not in rel for rel in protocol['input_sha256'])
    index=0;points=0;telemetry={'available_first_motions':0,'convergence_nonnull':0,'pose_covariance_nonnull':0}
    for case in protocol['cases']:
        source=root/case['run_relative'];groups=json.loads((source/'protocol.json').read_text())['input_rows'];prefix={'rgb':[],'thermal':[]}
        for line in (source/'timeline.jsonl').read_text().splitlines():
            d=json.loads(line)
            if d['kind']=='observer_event':prefix[d['camera']].append(d['event'])
            if d['kind']!='decision':continue
            row=features['rows'][index];i=d['pair_ordinal'];index+=1
            assert row['window']==case['window'] and row['pair_ordinal']==i and row['source_decision_sequence']==d['sequence']
            assert row['deadline_monotonic_ns']==d['controller_monotonic_ns'] and row['observed_event_counts']==d['observed_event_counts']
            assert row['availability']==d['availability']
            for c in prefix:
                record=row['experts'][c];times=[groups[j][c+'_left']['time_ns'] for j in (i,i+1)]
                hs=[e for e in prefix[c] if e['kind']=='pair_end' and [e['time0_ns'],e['time1_ns']]==times];assert len(hs)==1;h=hs[0]
                previous=[groups[j][c+'_left']['time_ns'] for j in (i-1,i)] if i else None
                ps=[e for e in prefix[c] if e['kind']=='pair_end' and [e['time0_ns'],e['time1_ns']]==previous] if previous else []
                assert len(ps)<=1;p=ps[0] if ps else None
                assert record['health_event_sequence']==h['sequence'] and record['prior_health_event_sequence']==(p['sequence'] if p else None)
                assert record['health_observation_monotonic_ns']==h['observation_monotonic_ns']<=d['controller_monotonic_ns']
                raw_path=source/'experts'/c/'uncertainty'/f'pair-{i:03d}.npz';assert root/record['raw_relative']==raw_path
                with np.load(raw_path,allow_pickle=False) as raw:
                    assert raw['time0_ns'].dtype.kind in 'iu' and int(raw['time0_ns'])==times[0]
                    cov=raw['grid_raw_flow_cov'].astype(float);uv=raw['grid_uv1'].astype(float);k=raw['K'].reshape(3,3).astype(float)
                    # Independent analytic metric A^T(I-nn^T)A/||r||², rather than producer J.
                    # Preserve the source K inversion dtype used by the historical helper.
                    inverse=np.linalg.inv(raw['K'].reshape(3,3)).astype(float);r=np.column_stack((uv,np.ones(len(uv))))@inverse.T
                    length2=np.sum(r*r,axis=1);n=r/np.sqrt(length2)[:,None];a=inverse[:,:2];na=n@a
                    g=(a.T@a)[None]-na[:,:,None]*na[:,None,:];g/=length2[:,None,None]
                    angular=g[:,0,0]*cov[:,0]+g[:,1,1]*cov[:,1]+2*g[:,0,1]*cov[:,2]
                    valid=np.isfinite(uv).all(1)&np.isfinite(cov).all(1)&(cov[:,0]>0)&(cov[:,1]>0)&(cov[:,2]**2<=cov[:,0]*cov[:,1])&np.isfinite(angular)&(angular>=0)
                    pop=record['covariance_population'];assert pop=={'grid_count':len(uv),'valid_covariance_grid_count':int(valid.sum()),'invalid_covariance_grid_count':int((~valid).sum())}
                    check_scalar(record['values']['flow_trace_px2'],float(np.median(raw['grid_raw_flow_cov'][valid,:2].sum(1))) if valid.any() else None)
                    check_scalar(record['values']['ray_trace_rad2'],float(np.median(angular[valid])) if valid.any() else None,angular=True)
                    selected=len(raw['selected_uv0']);inbound=int(raw['selected_inbound'].sum());retained=int(raw['outlier_keep_inbound'].sum())
                    assert (selected,inbound,retained,len(uv))==(h['selected_count'],h['inbound_count'],h['retained_count'],h['grid_count'])
                    check_scalar(record['values']['retained_fraction'],retained/selected if selected else None)
                    height,width=raw['height_width'];uv0=raw['selected_uv0'][raw['selected_inbound']][raw['outlier_keep_inbound']]
                    cells={(int(y*3//height),int(x*3//width)) for x,y in uv0}
                    assert record['values']['occupied_cells']==h['occupied_3x3_cells']==len(cells)
                    expected=None if p is None or type(p.get('original_need_interp')) is not bool else int(not p['original_need_interp'] and p.get('execution_error') is None)
                    assert record['values']['prior_supported']==expected
                    points+=len(uv)
                if row['availability'][c]:
                    m=prefix[c][d['evidence'][c]['motion_event_sequence']];telemetry['available_first_motions']+=1
                    telemetry['convergence_nonnull']+=m.get('optimizer_convergence') is not None;telemetry['pose_covariance_nonnull']+=m.get('pose_covariance') is not None
    assert index==len(features['rows'])==len(analysis['rows'])==217 and features['optimizer_telemetry']==telemetry
    labels={}
    for rel,digest in analysis['label_report_sha256'].items():
        source=root/rel;proof=json.loads((source/'validation.json').read_text());assert proof['status']=='PASS' and proof['report_sha256']==sha(source/'pose-proxy.json')==digest
        for r in json.loads((source/'pose-proxy.json').read_text())['rows']:labels[(r['window'],r['pair_ordinal'])]=r
    assert len(labels)==217
    for f,row in zip(features['rows'],analysis['rows']):
        assert (row['window'],row['pair_ordinal'])==(f['window'],f['pair_ordinal']);label=labels[(f['window'],f['pair_ordinal'])]
        assert label['source_decision_sequence']==f['source_decision_sequence']
        for c in ('rgb','thermal'):assert row['losses'][c]==(label['experts'][c]['common_loss'] if label['experts'][c] else None)
        assert row['both_eligible']==all(f['availability'].values())
        if not row['both_eligible']:assert not row['diagnostics'] and row['translation_proxy_winner'] is None;continue
        losses=[row['losses'][c]['translation_m'] for c in ('rgb','thermal')]
        winner='tie' if abs(losses[0]-losses[1])<=1e-12 else 'rgb' if losses[0]<losses[1] else 'thermal'
        assert row['translation_proxy_winner']==winner
        for name,direction in DIRECTIONS.items():
            a,b=[f['experts'][c]['values'][name] for c in ('rgb','thermal')]
            if a is None or b is None:choice,kind=None,'missing'
            elif a==b:choice,kind='rgb','tie_rgb_preference'
            else:choice,kind=('rgb' if (a<b if direction=='lower' else a>b) else 'thermal'),'decisive'
            assert row['diagnostics'][name]=={'choice':choice,'kind':kind,'translation_correct':None if choice is None or winner=='tie' else choice==winner}
    by_key={(r['window'],r['pair_ordinal']):r for r in features['rows']}
    for window in ('early','middle','day-middle'):
        all_rows=[r for r in analysis['rows'] if r['window']==window];common=[r for r in all_rows if r['both_eligible']];s=analysis['windows'][window]
        assert s['decisions']==len(all_rows) and s['both_eligible_pairs']==len(common)
        for name in DIRECTIONS:
            actual=s['diagnostics'][name];comp=[r for r in common if r['diagnostics'][name]['choice'] is not None];dec=[r for r in comp if r['diagnostics'][name]['kind']=='decisive']
            for k,v in {'comparable_pairs':len(comp),'missing_pairs':len(common)-len(comp),'feature_ties':len(comp)-len(dec),'decisive_pairs':len(dec),
                'reference_translation_ties':sum(r['translation_proxy_winner']=='tie' for r in comp),
                'decisive_correct':sum(r['diagnostics'][name]['translation_correct'] is True for r in dec),
                'correct_including_rgb_feature_ties':sum(r['diagnostics'][name]['translation_correct'] is True for r in comp)}.items():assert actual[k]==v
            for k in ('translation_m','rotation_deg'):
                population(actual['selected'][k],[r['losses'][r['diagnostics'][name]['choice']][k] for r in comp])
                for c in ('rgb','thermal'):population(actual['matched_baselines'][c][k],[r['losses'][c][k] for r in comp])
            population(actual['matched_translation_oracle'],[min(r['losses'][c]['translation_m'] for c in ('rgb','thermal')) for r in comp])
            for c in ('rgb','thermal'):
                valid=[r for r in common if by_key[(window,r['pair_ordinal'])]['experts'][c]['values'][name] is not None]
                x=[by_key[(window,r['pair_ordinal'])]['experts'][c]['values'][name] for r in valid]
                for k in ('translation_m','rotation_deg'):
                    result=actual['per_camera_spearman'][c][k];assert result['count']==len(valid)
                    if len(valid)<3:assert result['rho'] is None and result['reason']=='fewer_than_three'
                    else:
                        a=rankdata(x);b=rankdata([r['losses'][c][k] for r in valid]);a=a-a.mean();b=b-b.mean()
                        if not np.any(a) or not np.any(b):assert result['rho'] is None and result['reason']=='constant_ranks'
                        else:check_scalar(result['rho'],float(a@b/np.sqrt((a@a)*(b@b))));assert result['reason'] is None
    report={'status':'PASS','rows':217,'expert_slots':434,'raw_grid_points_checked':points,'both_eligible_pairs':sum(r['both_eligible'] for r in analysis['rows']),
        'features_sha256':sha(run/'features.json'),'analysis_sha256':sha(run/'analysis.json'),'validator_sha256':sha(Path(__file__)),
        'scope':'independent numeric/prefix/population audit, not physical calibration or generalization'}
    (run/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();validate(a.root,a.run)
