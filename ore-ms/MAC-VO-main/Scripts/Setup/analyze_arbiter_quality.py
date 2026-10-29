"""Descriptive fixed-direction diagnostics; labels join AFTER feature extraction."""
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import json
from pathlib import Path
import numpy as np
from scipy.stats import rankdata
from Scripts.Setup.arbiter_quality_features import FEATURES,sha


def stats(values):
    a=np.asarray(values,float)
    return {'count':len(a),'rmse':float(np.sqrt(np.mean(a*a))),'mean':float(np.mean(a))} if len(a) else {'count':0,'rmse':None,'mean':None}


def rank_association(x,y):
    if len(x)<3:return {'rho':None,'reason':'fewer_than_three','count':len(x)}
    a,b=rankdata(x),rankdata(y)
    if np.ptp(a)==0 or np.ptp(b)==0:return {'rho':None,'reason':'constant_ranks','count':len(x)}
    return {'rho':float(np.corrcoef(a,b)[0,1]),'reason':None,'count':len(x)}


def preference(a,b,direction):
    if a is None or b is None:return None,'missing'
    if a==b:return 'rgb','tie_rgb_preference'
    return ('rgb' if (a<b if direction=='lower' else a>b) else 'thermal'),'decisive'


def analyze(root,run):
    if (run/'analysis.json').exists():raise FileExistsError('analysis already exists')
    features=json.loads((run/'features.json').read_text());protocol=json.loads((run/'protocol.json').read_text())
    if features['protocol_sha256']!=sha(run/'protocol.json') or features['labels_loaded']:raise ValueError('feature stage changed')
    for rel,d in protocol['input_sha256'].items():
        if sha(root/rel)!=d:raise ValueError('frozen source changed')
    labels=[];proofs={}
    for rel in ('results/arbiter-pose-proxy/run-002','results/history-day-replication/proxy-001'):
        directory=root/rel;proof=json.loads((directory/'validation.json').read_text())
        if proof['status']!='PASS' or proof['report_sha256']!=sha(directory/'pose-proxy.json') or proof['protocol_sha256']!=sha(directory/'protocol.json'):
            raise ValueError('unverified proxy labels')
        labels += json.loads((directory/'pose-proxy.json').read_text())['rows'];proofs[rel]=sha(directory/'pose-proxy.json')
    lookup={(r['window'],r['pair_ordinal']):r for r in labels}
    if len(lookup)!=len(labels) or len(lookup)!=len(features['rows']):raise ValueError('duplicate or missing label population')
    rows=[]
    for f in features['rows']:
        label=lookup[(f['window'],f['pair_ordinal'])]
        if label['source_decision_sequence']!=f['source_decision_sequence']:raise ValueError('mismatched source decision')
        losses={c:label['experts'][c]['common_loss'] if label['experts'][c] else None for c in ('rgb','thermal')}
        if any((losses[c] is not None)!=f['availability'][c] for c in losses):raise ValueError('availability/label mismatch')
        both=all(f['availability'].values());diagnostics={}
        if both:
            a,b=[losses[c]['translation_m'] for c in ('rgb','thermal')]
            winner='tie' if abs(a-b)<=1e-12 else 'rgb' if a<b else 'thermal'
            for name,direction in FEATURES.items():
                choice,kind=preference(f['experts']['rgb']['values'][name],f['experts']['thermal']['values'][name],direction)
                diagnostics[name]={'choice':choice,'kind':kind,'translation_correct':None if choice is None or winner=='tie' else choice==winner}
        else:winner=None
        rows.append({'window':f['window'],'pair_ordinal':f['pair_ordinal'],'both_eligible':both,'translation_proxy_winner':winner,'losses':losses,'diagnostics':diagnostics})
    windows={}
    for window in sorted({r['window'] for r in rows}):
        selected=[r for r in rows if r['window']==window];common=[r for r in selected if r['both_eligible']];summary={}
        feature_by_pair={f['pair_ordinal']:f for f in features['rows'] if f['window']==window}
        for name in FEATURES:
            comparable=[r for r in common if r['diagnostics'][name]['choice'] is not None]
            decisive=[r for r in comparable if r['diagnostics'][name]['kind']=='decisive']
            chosen=[r['losses'][r['diagnostics'][name]['choice']] for r in comparable]
            correlations={}
            for c in ('rgb','thermal'):
                valid=[r for r in common if feature_by_pair[r['pair_ordinal']]['experts'][c]['values'][name] is not None]
                x=[feature_by_pair[r['pair_ordinal']]['experts'][c]['values'][name] for r in valid]
                correlations[c]={k:rank_association(x,[r['losses'][c][k] for r in valid]) for k in ('translation_m','rotation_deg')}
            summary[name]={'comparable_pairs':len(comparable),'missing_pairs':len(common)-len(comparable),
                'feature_ties':len(comparable)-len(decisive),'decisive_pairs':len(decisive),
                'reference_translation_ties':sum(r['translation_proxy_winner']=='tie' for r in comparable),
                'decisive_correct':sum(r['diagnostics'][name]['translation_correct'] is True for r in decisive),
                'correct_including_rgb_feature_ties':sum(r['diagnostics'][name]['translation_correct'] is True for r in comparable),
                'selected':{k:stats([v[k] for v in chosen]) for k in ('translation_m','rotation_deg')},
                'matched_baselines':{c:{k:stats([r['losses'][c][k] for r in comparable]) for k in ('translation_m','rotation_deg')} for c in ('rgb','thermal')},
                'matched_translation_oracle':stats([min(r['losses'][c]['translation_m'] for c in ('rgb','thermal')) for r in comparable]),
                'per_camera_spearman':correlations}
        windows[window]={'decisions':len(selected),'both_eligible_pairs':len(common),'diagnostics':summary}
    report={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'features_sha256':sha(run/'features.json'),
        'label_report_sha256':proofs,'analyzer_sha256':sha(Path(__file__)),'windows':windows,'rows':rows,
        'scope':'fixed-direction retrospective diagnostics, no chosen/fitted gate, significance or calibrated cross-camera probability claim'}
    (run/'analysis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({w:{'both_eligible':v['both_eligible_pairs'],'diagnostics':{n:{'comparable':d['comparable_pairs'],'decisive':d['decisive_pairs'],'correct':d['correct_including_rgb_feature_ties'],'ties':d['feature_ties'],'rmse_m':d['selected']['translation_m']['rmse']} for n,d in v['diagnostics'].items()}} for w,v in windows.items()},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();analyze(a.root,a.run)
