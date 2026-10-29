"""Offline label join for frozen history predictions; never a predictor input."""
import argparse
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
import math
from pathlib import Path

POLICIES=('rgb_only','thermal_only','fixed_rgb_preference','history')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    values=sorted(values);n=len(values)
    if not n:return {'count':0,'mean':None,'rmse':None,'median':None}
    median=values[n//2] if n%2 else (values[n//2-1]+values[n//2])/2
    return {'count':n,'mean':math.fsum(values)/n,'rmse':math.sqrt(math.fsum(v*v for v in values)/n),'median':median}


def population(losses,indices):
    return {k:stats([losses[i][k] for i in indices]) for k in ('translation_m','rotation_deg')}


def score(root,run,labels):
    if (run/'scores.json').exists():raise FileExistsError('scores already exist')
    prediction_path=run/'predictions.json';frozen=json.loads(prediction_path.read_text());protocol=json.loads((run/'protocol.json').read_text())
    if frozen['protocol_sha256']!=sha(run/'protocol.json'):raise ValueError('prediction protocol changed')
    for rel,digest in protocol['input_sha256'].items():
        if sha(root/rel)!=digest:raise ValueError('frozen input changed')
    proof=json.loads((labels/'validation.json').read_text());reference_protocol=json.loads((labels/'protocol.json').read_text())
    if proof['status']!='PASS' or proof['report_sha256']!=sha(labels/'pose-proxy.json') or proof['protocol_sha256']!=sha(labels/'protocol.json'):
        raise ValueError('unverified offline proxy labels')
    if reference_protocol['independent_ground_truth'] or reference_protocol['physical_body_transform_verified']:
        raise ValueError('expected conditional nominal proxy protocol')
    label_rows=json.loads((labels/'pose-proxy.json').read_text())['rows']
    lookup={(r['window'],r['pair_ordinal']):r for r in label_rows}
    if len(lookup)!=len(label_rows) or len(frozen['decisions'])!=len(lookup):raise ValueError('duplicate/missing label rows')
    if {(d['window'],d['pair_ordinal']) for d in frozen['decisions']}!=set(lookup):raise ValueError('prediction population differs from labels')
    rows=[]
    for d in frozen['decisions']:
        if d['action'] not in ('RGB','THERMAL','ABSTAIN') or d['reference_or_loss_used']:raise ValueError('invalid history prediction')
        key=(d['window'],d['pair_ordinal']);label=lookup[key]
        if label['source_decision_sequence']!=d['source_decision_sequence'] or label['action']!=d['baseline_action']:
            raise ValueError('prediction/label identity mismatch')
        losses={}
        for policy in POLICIES:
            camera=policy.removesuffix('_only') if policy.endswith('_only') else d['baseline_action'].lower() if policy=='fixed_rgb_preference' else d['action'].lower()
            expert=label['experts'].get(camera)
            if policy=='history' and camera in ('rgb','thermal') and not d['confirmed_availability'][camera]:raise ValueError('unconfirmed selected expert')
            losses[policy]=expert['common_loss'] if expert else None
        rows.append({'window':d['window'],'pair_ordinal':d['pair_ordinal'],'source_decision_sequence':d['source_decision_sequence'],
            'action':d['action'],'baseline_action':d['baseline_action'],'losses':losses})
    windows={}
    for window in sorted({r['window'] for r in rows}):
        selected=[r for r in rows if r['window']==window];full={};matched={}
        for policy in POLICIES:
            values={r['pair_ordinal']:r['losses'][policy] for r in selected if r['losses'][policy] is not None}
            ids=sorted(values)
            full[policy]={'available':len(ids),'abstain':len(selected)-len(ids),'pair_ordinals':ids,**population(values,ids)}
            common=[r['pair_ordinal'] for r in selected if r['losses']['history'] is not None and r['losses'][policy] is not None]
            history={r['pair_ordinal']:r['losses']['history'] for r in selected if r['losses']['history'] is not None}
            matched[policy]={'count':len(common),'pair_ordinals':common,'history':population(history,common),'baseline':population(values,common)}
        windows[window]={'decisions':len(selected),'actions':dict(Counter(r['action'] for r in selected)),
            'action_changes':dict(Counter(r['baseline_action']+'->'+r['action'] for r in selected if r['baseline_action']!=r['action'])),
            'full_available':full,'matched_with_history':matched}
    report={'scope':'retrospective history rule evaluated with conditional SLAM-proxy labels; not independent physical accuracy/generalization',
        'predictions_sha256':sha(prediction_path),'label_report_sha256':sha(labels/'pose-proxy.json'),
        'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'scorer_sha256':sha(Path(__file__)),
        'label_directory_relative':str(labels.relative_to(root)),'windows':windows,'rows':rows}
    (run/'scores.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({w:{'actions':r['actions'],'availability':{p:d['available'] for p,d in r['full_available'].items()},
        'matched_fixed_rgb_rmse_m':r['matched_with_history']['fixed_rgb_preference']['baseline']['translation_m']['rmse'],
        'matched_history_rmse_m':r['matched_with_history']['fixed_rgb_preference']['history']['translation_m']['rmse']} for w,r in windows.items()},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('root','run','labels'):p.add_argument('--'+n,type=Path,required=True)
    a=p.parse_args();score(a.root,a.run,a.labels)
