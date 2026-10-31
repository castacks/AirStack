"""Frozen EXP088 prediction then separate qualified saved-report scoring."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from Scripts.Setup.causal_interval_model import project, se3

CASES = [('early','end-to-end-solver-control',19),('middle','solver-collapse-replication',99),('day','solver-day-replication',99)]


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path): return json.loads(Path(path).read_text())
def write(path, value):
    with Path(path).open('x') as f: json.dump(value,f,indent=2,allow_nan=False); f.write('\n')


def load(root):
    rows=[];frozen={};qualifications={}
    for window,name,count in CASES:
        base=root/'results'/name/'run-001'
        for n in ('evaluation.json','evaluation-protocol.json','protocol.json','validation.json'):
            frozen[str((base/n).relative_to(root))]=sha(base/n)
        report=read(base/'evaluation.json');proof=read(base/'validation.json');decl=read(base/'evaluation-protocol.json');protocol=read(base/'protocol.json')
        assert proof['status']=='PASS' and proof['evaluation_sha256']==sha(base/'evaluation.json')
        assert proof['evaluation_protocol_sha256']==report['evaluation_protocol_sha256']==sha(base/'evaluation-protocol.json')
        assert proof['execution_protocol_sha256']==sha(base/'protocol.json')
        assert decl['physical_body_transform_verified'] is False and report['independent_ground_truth'] is False
        original=[dict(r,window=window) for r in report['rows'] if r['arm']=='original']
        assert {(r['camera'],r['pair_ordinal']) for r in original}=={(c,i) for c in ('rgb','thermal') for i in range(count)}
        assert len(original)==count*2
        for r in original:
            i=r['pair_ordinal'];c=r['camera']
            assert r['sensor_interval_ns']==[protocol['input_rows'][j][c+'_left']['time_ns'] for j in (i,i+1)]
            assert r['common_interval_ns']==[protocol['input_rows'][j]['rgb_left']['time_ns'] for j in (i,i+1)]
        rows.extend(original);qualifications[window]=proof.get('gradient_component_exceptions',[])
    return rows,frozen,qualifications


def predict_rows(rows):
    # This whitelist is the entire model input. Reference poses/losses excluded.
    outputs=[]
    for row in rows:
        out={k:row[k] for k in ('window','camera','pair_ordinal','sensor_interval_ns','common_interval_ns','motion_stage')}
        if row['own_body_delta'] is None:
            out.update(valid_model_projection=False,delta_model=None,reason='source_unavailable')
        elif row['motion_stage']!='pair_processing':
            out.update(valid_model_projection=False,delta_model=None,reason='non_processing_stage')
        else:
            assert row['original_need_interp'] is False
            out.update(project(row['own_body_delta'],row['sensor_interval_ns'],row['common_interval_ns']))
        outputs.append(out)
    return outputs


def predict(root,output):
    rows,frozen,qualifications=load(root)
    files=[Path(__file__),Path(__file__).with_name('causal_interval_model.py'),Path(__file__).with_name('arbiter_motion_output.py')]
    frozen.update({str(p.relative_to(root)):sha(p) for p in files})
    declaration=read(output/'declaration.json')
    assert declaration['declaration_sha256']==hashlib.sha256(declaration['declaration_text'].encode()).hexdigest()
    write(output/'protocol.json',{'scope':'EXP088 frozen stationary body twist; saved original arms only',
        'input_sha256':frozen,'declaration_sha256':sha(output/'declaration.json'),'expected_rows':434,
        'gradient_audit_exceptions':qualifications,'max_extrapolation':'one native duration, no search',
        'scoring_inputs_not_predictor_features':True,'physical_frame_clock_deadline_verified':False})
    predictions=predict_rows(rows)
    # Remove labels/bridges entirely, and independently corrupt extra fields.
    allowed=('window','camera','pair_ordinal','sensor_interval_ns','common_interval_ns','motion_stage','own_body_delta','original_need_interp')
    stripped=[{k:r[k] for k in allowed} for r in rows]
    assert predict_rows(stripped)==predictions
    for r in stripped:r.update(reference_poses='forbidden',common_body_delta='forbidden',common_loss={'translation_m':1e100},reference_delta='forbidden')
    assert predict_rows(stripped)==predictions
    assert len(predictions)==434 and all(sha(root/p)==h for p,h in frozen.items())
    write(output/'predictions.json',{'protocol_sha256':sha(output/'protocol.json'),'rows':predictions})
    write(output/'prediction-proof.json',{'status':'PASS_LABEL_EXCLUSION','labels_removed_and_mutated_equal':True,
        'predictions_sha256':sha(output/'predictions.json'),'protocol_sha256':sha(output/'protocol.json'),
        'availability':dict(Counter(r['reason'] for r in predictions))})
    print(json.dumps(read(output/'prediction-proof.json'),indent=2))


def loss(predicted,reference):
    error=np.linalg.inv(se3(reference))@se3(predicted)
    return {'translation_m':float(np.linalg.norm(error[:3,3])),
            'rotation_deg':float(np.degrees(Rotation.from_matrix(error[:3,:3]).magnitude()))}


def summary(values):
    v=np.array(values,dtype=float)
    return {'count':len(v),'mean':float(v.mean()),'median':float(np.median(v)),
            'rmse':float(np.sqrt(np.mean(v*v))),'max':float(v.max())} if len(v) else {'count':0,'mean':None,'median':None,'rmse':None,'max':None}


def score(root,output):
    lock=read(output/'prediction-proof.json');protocol=read(output/'protocol.json')
    assert lock['predictions_sha256']==sha(output/'predictions.json') and lock['protocol_sha256']==sha(output/'protocol.json')
    assert sha(output/'declaration.json')==protocol['declaration_sha256']
    assert all(sha(root/p)==h for p,h in protocol['input_sha256'].items())
    source,_,_=load(root);predictions=read(output/'predictions.json')['rows']
    lookup={(r['window'],r['camera'],r['pair_ordinal']):r for r in source}
    scored=[]
    for p in predictions:
        item={k:p[k] for k in ('window','camera','pair_ordinal','reason','valid_model_projection')}
        item['losses']=None
        if p['valid_model_projection']:
            r=lookup[p['window'],p['camera'],p['pair_ordinal']];refs=r['reference_poses']
            reference=np.linalg.inv(se3(refs['common_start']))@se3(refs['common_end'])
            item['losses']={'model_proxy':loss(p['delta_model'],reference),
                'unchanged_native_proxy':loss(r['own_body_delta'],reference),
                'historic_bridged_proxy':loss(r['common_body_delta'],reference),
                'model_vs_historic_bridge':loss(p['delta_model'],r['common_body_delta'])}
            for key,value in r['common_loss'].items():
                assert np.isclose(item['losses']['historic_bridged_proxy'][key],value,rtol=1e-8,atol=1e-9)
        scored.append(item)
    groups={}
    for window,_,count in CASES:
        for camera in ('rgb','thermal'):
            subset=[r for r in scored if r['window']==window and r['camera']==camera]
            eligible=[r for r in subset if r['losses'] is not None]
            groups[window+'/'+camera]={'slots':count,'eligible':len(eligible),'unavailable_reasons':dict(Counter(r['reason'] for r in subset if not r['valid_model_projection'])),
                'metrics':{method:{metric:summary([r['losses'][method][metric] for r in eligible]) for metric in ('translation_m','rotation_deg')}
                    for method in ('model_proxy','unchanged_native_proxy','historic_bridged_proxy','model_vs_historic_bridge')}}
    assert all(sha(root/p)==h for p,h in protocol['input_sha256'].items())
    write(output/'scores.json',{'predictions_sha256':sha(output/'predictions.json'),'scope':'descriptive nominal-frame recorded-SLAM proxy; no independent physical accuracy or deployed causal deadline claim',
        'physical_accuracy_validated':False,'fit_performed':False,'groups':groups,'rows':scored})
    print(json.dumps({k:{'eligible':v['eligible'],'translation_disagreement_rmse':v['metrics']['model_vs_historic_bridge']['translation_m']['rmse'],
        'model_translation_proxy_rmse':v['metrics']['model_proxy']['translation_m']['rmse'],'bridge_translation_proxy_rmse':v['metrics']['historic_bridged_proxy']['translation_m']['rmse']} for k,v in groups.items()},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('predict','score'));p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();(predict if a.action=='predict' else score)(a.root,a.output)
