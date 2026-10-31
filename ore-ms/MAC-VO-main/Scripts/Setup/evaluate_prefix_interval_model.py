"""Freeze shared-prefix model predictions before separate nominal-proxy scoring."""
import argparse
from collections import Counter
from itertools import combinations
import hashlib
import json
from pathlib import Path
import numpy as np
from Scripts.Setup.evaluate_interval_model import load, read, write, sha, loss, summary
from Scripts.Setup.prefix_interval_model import predict_prefix

CASES=[('early','shared-arbiter-events',19),('middle','arbiter-support-collapse',99),('day','history-day-replication',99)]
POLICIES=('rgb_only','thermal_only','raw_advice','history')


def prepare(root,out):
    original,frozen,qualifications=load(root);contract={}
    declaration=read(out/'declaration.json')
    assert hashlib.sha256(declaration['declaration_text'].encode()).hexdigest()==declaration['declaration_sha256']
    for window,name,count in CASES:
        base=root/'results'/name/'run-001'
        for name in ('protocol.json','timeline.jsonl','summary.json','validation.json'):frozen[str((base/name).relative_to(root))]=sha(base/name)
        proof=read(base/'validation.json');report=read(base/'summary.json');protocol=read(base/'protocol.json')
        assert proof['status']=='PASS' and report['status']=='PASS_REPLAY_ONLY'
        assert proof['timeline_sha256']==report['timeline_sha256']==sha(base/'timeline.jsonl')
        assert protocol['mode']=='offline_serial_sensor_ordered' and protocol['policy_fitted'] is False and protocol['wall_clock_paced'] is False
        assert len(protocol['input_rows'])==count+1
        transforms={}
        for camera in ('rgb','thermal'):
            rows=[r for r in original if r['window']==window and r['camera']==camera]
            assert len(rows)==count
            transforms[camera]=rows[0]['nominal_body_from_camera']
            assert all(r['nominal_body_from_camera']==transforms[camera] for r in rows)
            for r in rows:
                i=r['pair_ordinal']
                assert r['sensor_interval_ns']==[protocol['input_rows'][j][camera+'_left']['time_ns'] for j in (i,i+1)]
        contract[window]={'run_relative':str(base.relative_to(root)),'input_rows':protocol['input_rows'],'nominal_body_from_camera':transforms}
    names=('prefix_interval_model.py','evaluate_prefix_interval_model.py','causal_interval_model.py','shared_arbiter_replay.py','pair_arbiter_events.py','arbiter_history.py','arbiter_motion_output.py','evaluate_interval_model.py')
    frozen.update({str(Path(__file__).with_name(n).relative_to(root)):sha(Path(__file__).with_name(n)) for n in names})
    for name in ('predictions.json','validation.json'):
        p=root/'results/causal-interval-model/run-001'/name;frozen[str(p.relative_to(root))]=sha(p)
    write(out/'predictor-contract.json',contract)
    write(out/'protocol.json',{'input_sha256':frozen,'contract_sha256':sha(out/'predictor-contract.json'),'declaration_sha256':sha(out/'declaration.json'),
        'gradient_audit_exceptions':qualifications,'policies':POLICIES,'model_changed':False,'physical_or_live_verification':False,
        'scope':'existing shared-prefix rules plus unchanged EXP088 projection; no new selector'})
    print(json.dumps({'windows':list(contract),'declared_decisions':217,'predictor_contract_sha256':sha(out/'predictor-contract.json')}))


def verify(root,out):
    p=read(out/'protocol.json')
    assert p['contract_sha256']==sha(out/'predictor-contract.json') and p['declaration_sha256']==sha(out/'declaration.json')
    assert all(sha(root/rel)==digest for rel,digest in p['input_sha256'].items())
    return p


def predict(root,out):
    verify(root,out);contract=read(out/'predictor-contract.json');predictions=[]
    for window,name,count in CASES:
        c=contract[window];path=root/c['run_relative']/'timeline.jsonl'
        # Only this whitelist enters prediction. No report/pose/loss file is read.
        with path.open() as stream:rows=list(predict_prefix(c['input_rows'],(json.loads(s) for s in stream),c['nominal_body_from_camera']))
        assert len(rows)==count
        predictions += [dict(r,window=window) for r in rows]
    verify(root,out)
    write(out/'predictions.json',{'protocol_sha256':sha(out/'protocol.json'),'rows':predictions})
    write(out/'prediction-proof.json',{'status':'PASS_PREFIX_PREDICTION','predictions_sha256':sha(out/'predictions.json'),
        'protocol_sha256':sha(out/'protocol.json'),'decisions':len(predictions),'camera_slots':len(predictions)*2,
        'model_available':sum(e['valid_model_projection'] for r in predictions for e in r['experts'].values()),
        'actions':{w:{p:dict(Counter(r['model_actions'][p] for r in predictions if r['window']==w)) for p in POLICIES} for w,_,_ in CASES}})
    print(json.dumps(read(out/'prediction-proof.json'),indent=2))


def score(root,out):
    verify(root,out);lock=read(out/'prediction-proof.json');assert lock['predictions_sha256']==sha(out/'predictions.json')
    source,_,_=load(root);lookup={(r['window'],r['camera'],r['pair_ordinal']):r for r in source}
    predictions=read(out/'predictions.json')['rows'];scored=[];native_parity=0;largest=0.
    old=read(root/'results/causal-interval-model/run-001/predictions.json')
    old_lookup={(r['window'],r['camera'],r['pair_ordinal']):r for r in old['rows']}
    old_proof=read(root/'results/causal-interval-model/run-001/validation.json')
    assert old_proof['status']=='PASS' and old_proof['predictions_sha256']==sha(root/'results/causal-interval-model/run-001/predictions.json')
    for r in predictions:
        camera_losses={}
        for camera,e in r['experts'].items():
            if not e['valid_model_projection']:camera_losses[camera]=None;continue
            original=lookup[r['window'],camera,r['pair_ordinal']]
            difference=float(np.max(np.abs(np.array(e['source']['native_body_delta'])-np.array(original['own_body_delta']))));largest=max(largest,difference)
            np.testing.assert_allclose(e['source']['native_body_delta'],original['own_body_delta'],atol=1e-9,rtol=1e-8)
            prior=old_lookup[r['window'],camera,r['pair_ordinal']];assert prior['valid_model_projection']
            np.testing.assert_allclose(e['delta_model'],prior['delta_model'],atol=1e-9,rtol=1e-8);native_parity+=1
            refs=original['reference_poses'];reference=np.linalg.solve(np.array(refs['common_start']),np.array(refs['common_end']))
            camera_losses[camera]=loss(e['delta_model'],reference)
        losses={p:None if action=='ABSTAIN' else camera_losses[action.lower()] for p,action in r['model_actions'].items()}
        scored.append({'window':r['window'],'pair_ordinal':r['pair_ordinal'],'actions':r['model_actions'],'losses':losses})
    groups={}
    for window,_,count in CASES:
        rows=[r for r in scored if r['window']==window];policies={};comparisons={}
        for p in POLICIES:
            valid=[r for r in rows if r['losses'][p] is not None]
            policies[p]={'available':len(valid),'abstain':count-len(valid),
                'conditional_losses':{metric:summary([r['losses'][p][metric] for r in valid]) for metric in ('translation_m','rotation_deg')}}
        for a,b in combinations(POLICIES,2):
            matched=[r for r in rows if r['losses'][a] is not None and r['losses'][b] is not None]
            comparisons[a+'__'+b]={'matched_pair_ordinals':[r['pair_ordinal'] for r in matched],
                'policies':{p:{metric:summary([r['losses'][p][metric] for r in matched]) for metric in ('translation_m','rotation_deg')} for p in (a,b)}}
        groups[window]={'decisions':count,'policies':policies,'exact_matched_comparisons':comparisons}
    verify(root,out)
    write(out/'scores.json',{'predictions_sha256':sha(out/'predictions.json'),'groups':groups,'rows':scored,
        'EXP088_comparison':{'paired_native_and_model_outputs':native_parity,'max_native_matrix_absolute_difference':largest,
            'prediction_sha256':sha(root/'results/causal-interval-model/run-001/predictions.json')},
        'scope':'nominal-frame recorded-SLAM proxy at observed offline shared prefixes; no learned/physical/live-accuracy claim'})
    print(json.dumps({'parity':native_parity,'max_native_difference':largest,'availability':{w:{p:v['available'] for p,v in g['policies'].items()} for w,g in groups.items()}},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('prepare','predict','score'));p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();{'prepare':prepare,'predict':predict,'score':score}[a.action](a.root,a.output)
