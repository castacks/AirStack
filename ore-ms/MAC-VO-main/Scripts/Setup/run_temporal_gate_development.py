"""EXP091 fixed three-recording development folds; separate fit/predict/score."""
import argparse
from collections import Counter
from itertools import combinations
import hashlib,json,sys
from pathlib import Path
import numpy as np
import scipy
from Scripts.Setup.temporal_quality_gate import prefix_features,FAMILIES,FEATURE_NAMES,OPTIONS,RIDGE,fit_fold,choose

BASELINES=('rgb_only','thermal_only','raw_advice','history')
METHODS=BASELINES+tuple(FAMILIES)


def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')


def extract(rows,timeline,model_rows,window,recording,prefix):
    predictions={r['pair_ordinal']:r for r in model_rows};events={c:[] for c in ('rgb','thermal')};receipts={c:{} for c in events};output=[];previous=-1
    for i,entry in enumerate(timeline):
        now=entry['controller_monotonic_ns']
        assert type(now) is int and now>=previous and entry['sequence']==i;previous=now
        if entry['kind']=='observer_event':
            c,e=entry['camera'],entry['event'];assert e['sequence']==len(events[c]);events[c].append(e);receipts[c][e['sequence']]=now
        elif entry['kind']=='decision':
            pair=entry['pair_ordinal'];assert pair==len(output);p=predictions[pair]
            assert p['source_decision_sequence']==i and p['controller_deadline_ns']==now
            assert p['observed_event_counts']=={c:len(v) for c,v in events.items()}
            vector,evidence=prefix_features(rows,pair,events)
            for c,sequences in evidence['health_sequences'].items():assert all(receipts[c][n]<=now for n in sequences)
            both=all(e['valid_model_projection'] for e in p['experts'].values())
            output.append({'sample_id':f'{prefix}/{window}/{pair}','recording_id':recording,'window':window,'pair_ordinal':pair,
                'common_interval_ns':p['common_interval_ns'],'vector':vector,'feature_evidence':evidence,'both_supported':both,
                'eligible':bool(both and vector is not None),'baseline_actions':p['model_actions'],'source_controller_sequence':i})
    assert len(output)==len(rows)-1
    return output


def prepare(root,out):
    inputs={}
    def load(p):inputs[str(p.relative_to(root))]=sha(p);return read(p)
    def timeline(p):inputs[str(p.relative_to(root))]=sha(p);return [json.loads(s) for s in p.read_text().splitlines()]
    old=root/'results/prefix-model-pipeline/run-001';proof=load(old/'validation.json');pred=load(old/'predictions.json');contract=load(old/'predictor-contract.json');protocol=load(old/'protocol.json')
    assert proof['status']=='PASS' and proof['predictions_sha256']==sha(old/'predictions.json')
    assert pred['protocol_sha256']==sha(old/'protocol.json') and protocol['contract_sha256']==sha(old/'predictor-contract.json')
    features=[]
    for window,c in contract.items():
        recording='td3/20251027/0_salon_so_day' if window=='day' else 'td3/20251027/night_mac_salon_warehousefig8'
        features+=extract(c['input_rows'],timeline(root/c['run_relative']/'timeline.jsonl'),[r for r in pred['rows'] if r['window']==window],window,recording,'EXP089')
    new=root/'results/third-recording-shared-capture/run-001';plan=load(new/'capture-plan.json');p_lock=load(new/'prediction-lock.json');p_proof=load(new/'annotation-validation.json')
    assert p_lock['capture_plan_sha256']==sha(new/'capture-plan.json') and p_proof['status']=='PASS_CONDITIONAL_ANNOTATION'
    for spec in plan['windows']:
        window=spec['case'];base=new/'windows'/window;predictions=load(base/'model-predictions.json');t=base/'shared/timeline.jsonl';proof_new=load(base/'shared/validation.json')
        assert proof_new['status']=='PASS' and proof_new['timeline_sha256']==predictions['timeline_sha256']==sha(t)
        assert p_lock['model_prediction_sha256'][str((base/'model-predictions.json').relative_to(root))]==sha(base/'model-predictions.json')
        features+=extract(spec['input_rows'],timeline(t),predictions['rows'],window,'td3/20251027/1_salon_mac_cov_day','EXP090')
    assert len(features)==274 and len({r['sample_id'] for r in features})==274
    assert sum(r['eligible'] for r in features)==161
    write(out/'features.json',{'feature_names':FEATURE_NAMES,'rows':features,'scope':'observed-prefix inputs only; metadata is not part of numerical X'})
    # Scoring files are parsed only after feature materialization.
    old_scores=load(old/'scores.json');assert load(old/'validation.json')['scores_sha256']==sha(old/'scores.json')
    annotations=load(new/'annotation.json');assert p_proof['annotation_sha256']==sha(new/'annotation.json')
    labels={}
    for r in old_scores['rows']:labels[f"EXP089/{r['window']}/{r['pair_ordinal']}"]={'rgb':r['losses']['rgb_only'],'thermal':r['losses']['thermal_only']}
    for r in annotations['rows']:labels[f"EXP090/{r['case']}/{r['pair_ordinal']}"]=r['camera_losses']
    assert set(labels)=={r['sample_id'] for r in features}
    index=load(new/'development-index.json');expected={(r['recording_id'],tuple(r['common_interval_ns'])):r for r in index['entries']}
    for r in features:
        if r['eligible']:
            entry=expected[r['recording_id'],tuple(r['common_interval_ns'])]
            assert labels[r['sample_id']]['rgb']['translation_m']==entry['rgb_translation_proxy_loss_m']
            assert labels[r['sample_id']]['thermal']['translation_m']==entry['thermal_translation_proxy_loss_m']
    assert len(expected)==161
    write(out/'labels.json',labels)
    sources=[Path(__file__),Path(__file__).with_name('temporal_quality_gate.py')]
    write(out/'protocol.json',{'scope':'retrospective recording-excluded development; no prospective test or reserve','feature_sha256':sha(out/'features.json'),
        'labels_sha256':sha(out/'labels.json'),'input_sha256':inputs,'source_sha256':{str(p.relative_to(root)):sha(p) for p in sources},
        'declaration_sha256':sha(out/'declaration.md'),'families':FAMILIES,'ridge':RIDGE,'solver_options':OPTIONS,
        'recordings':sorted({r['recording_id'] for r in features}),'rows':274,'eligible':161,
        'numpy':np.__version__,'scipy':scipy.__version__,'python':sys.version,'primary':'combined','no_hyperparameter_search':True})
    print(json.dumps({'features':274,'fit_eligible':161,'recordings':3,'features_sha256':sha(out/'features.json'),'labels_sha256':sha(out/'labels.json')}))


def verify_fit(root,out):
    p=read(out/'protocol.json');assert p['feature_sha256']==sha(out/'features.json') and p['labels_sha256']==sha(out/'labels.json')
    assert p['declaration_sha256']==sha(out/'declaration.md')
    assert all(sha(root/rel)==h for rel,h in p['input_sha256'].items())
    assert all(sha(root/rel)==h for rel,h in p['source_sha256'].items())
    return p


def train(root,out):
    protocol=verify_fit(root,out);rows=read(out/'features.json')['rows'];labels=read(out/'labels.json');models={};statuses={}
    for family in FAMILIES:
        for index,recording in enumerate([*protocol['recordings'],None]):
            name=f'fold-{index}' if recording is not None else 'all-development'
            model=fit_fold(rows,labels,family,recording);model['protocol_sha256']=sha(out/'protocol.json')
            path=out/'models'/family/(name+'.json');write(path,model);models[str(path.relative_to(out))]=sha(path);statuses[family+'/'+name]={'status':model['status'],'reason':model['reason'],'gradient_inf':model.get('gradient_inf')}
    verify_fit(root,out)
    write(out/'models-lock.json',{'protocol_sha256':sha(out/'protocol.json'),'models_sha256':models,'statuses':statuses})
    print(json.dumps(statuses,indent=2))


def make_predictions(rows,models,recordings):
    result=[]
    for r in rows:
        fold=recordings.index(r['recording_id']);decisions={}
        for family in FAMILIES:
            model=models[family,fold];assert model['excluded_recording']==r['recording_id'] and r['recording_id'] not in model['training_recordings']
            decisions[family]=choose(model,r['vector'],r['eligible'],r['baseline_actions']['raw_advice'],FAMILIES[family])
            assert (decisions[family]['action']=='ABSTAIN')==(r['baseline_actions']['raw_advice']=='ABSTAIN')
        result.append({'sample_id':r['sample_id'],'recording_id':r['recording_id'],'actions':r['baseline_actions']|{f:d['action'] for f,d in decisions.items()},'learned_decisions':decisions})
    return result


def predict(root,out):
    # Provenance check here reads no labels or source loss/report files.
    protocol=read(out/'protocol.json');lock=read(out/'models-lock.json')
    assert protocol['feature_sha256']==sha(out/'features.json') and lock['protocol_sha256']==sha(out/'protocol.json')
    assert all(sha(root/p)==h for p,h in protocol['source_sha256'].items())
    assert all(sha(out/p)==h for p,h in lock['models_sha256'].items())
    models={(family,i):read(out/'models'/family/(f'fold-{i}.json')) for family in FAMILIES for i in range(3)}
    rows=read(out/'features.json')['rows'];predictions=make_predictions(rows,models,protocol['recordings'])
    write(out/'predictions.json',{'models_lock_sha256':sha(out/'models-lock.json'),'features_sha256':sha(out/'features.json'),'rows':predictions})
    write(out/'prediction-proof.json',{'status':'PASS_FROZEN_PREDICTIONS','predictions_sha256':sha(out/'predictions.json'),'labels_read_at_prediction':False,
        'coverage':{method:sum(r['actions'][method]!='ABSTAIN' for r in predictions) for method in METHODS}})
    print(json.dumps(read(out/'prediction-proof.json'),indent=2))


def stats(values):
    a=np.array(values,dtype=float)
    return {'count':len(a),'mean':float(a.mean()),'median':float(np.median(a)),'rmse':float(np.sqrt(np.mean(a*a))),'max':float(a.max())} if len(a) else {'count':0,'mean':None,'median':None,'rmse':None,'max':None}


def score(root,out):
    protocol=verify_fit(root,out);proof=read(out/'prediction-proof.json');assert proof['predictions_sha256']==sha(out/'predictions.json')
    predictions=read(out/'predictions.json')['rows'];labels=read(out/'labels.json');scored=[]
    for r in predictions:
        losses={}
        for method,action in r['actions'].items():
            value=None if action=='ABSTAIN' else labels[r['sample_id']][action.lower()]
            assert (value is None)==(action=='ABSTAIN');losses[method]=value
        scored.append({'sample_id':r['sample_id'],'recording_id':r['recording_id'],'losses':losses})
    groups={}
    for record in protocol['recordings']:
        rows=[r for r in scored if r['recording_id']==record];policies={};comparisons={}
        for method in METHODS:
            valid=[r['losses'][method] for r in rows if r['losses'][method] is not None]
            policies[method]={'available':len(valid),'abstain':len(rows)-len(valid),'conditional':{metric:stats([v[metric] for v in valid]) for metric in ('translation_m','rotation_deg')}}
        for a,b in combinations(METHODS,2):
            matched=[r for r in rows if r['losses'][a] is not None and r['losses'][b] is not None]
            comparisons[a+'__'+b]={'matched_sample_ids':[r['sample_id'] for r in matched],
                'policies':{m:{metric:stats([r['losses'][m][metric] for r in matched]) for metric in ('translation_m','rotation_deg')} for m in (a,b)}}
        groups[record]={'population':len(rows),'policies':policies,'matched_comparisons':comparisons}
    checks={}
    for family in FAMILIES:
        per_record={}
        for record,g in groups.items():
            per_record[record]={}
            for baseline in ('rgb_only','thermal_only','raw_advice'):
                c=g['matched_comparisons'][baseline+'__'+family];n=len(c['matched_sample_ids'])
                per_record[record][baseline]={'matched':n,'strict_translation_improvement':bool(n and c['policies'][family]['translation_m']['rmse']<c['policies'][baseline]['translation_m']['rmse'])}
        checks[family]={'development_consistent_translation_diagnostic':all(v['strict_translation_improvement'] for c in per_record.values() for v in c.values()),'per_record':per_record}
    macro={method:{metric:float(np.mean([g['policies'][method]['conditional'][metric]['rmse'] for g in groups.values()])) for metric in ('translation_m','rotation_deg')} for method in METHODS}
    verify_fit(root,out)
    write(out/'scores.json',{'predictions_sha256':sha(out/'predictions.json'),'groups':groups,'rows':scored,'development_diagnostic':checks,
        'equal_recording_conditional_rmse_mean':macro,'scope':'retrospective development only; nominal-body recorded-SLAM proxy; no physical/prospective/population claim'})
    print(json.dumps({'primary_combined_diagnostic':checks['combined'],'conditional_rmse':{r:{m:g['policies'][m]['conditional']['translation_m']['rmse'] for m in METHODS} for r,g in groups.items()}},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('prepare','fit','predict','score'));p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();{'prepare':prepare,'fit':train,'predict':predict,'score':score}[a.action](a.root,a.output)
