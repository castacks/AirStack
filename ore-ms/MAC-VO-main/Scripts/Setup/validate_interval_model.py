"""Independent matrix-log/exponential and arithmetic audit for EXP088."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.linalg import expm, logm


def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def independent_loss(predicted,reference):
    a,b=np.array(predicted),np.array(reference)
    rotation=b[:3,:3].T@a[:3,:3]
    sine=.5*np.linalg.norm([rotation[2,1]-rotation[1,2],rotation[0,2]-rotation[2,0],rotation[1,0]-rotation[0,1]])
    cosine=.5*(np.trace(rotation)-1)
    return {'translation_m':float(np.linalg.norm(a[:3,3]-b[:3,3])),
            'rotation_deg':float(np.degrees(np.arctan2(sine,cosine)))}


def audit_payload(source,predictions,scores):
    key=lambda r:(r['window'],r['camera'],r['pair_ordinal'])
    sources={key(r):r for r in source};pred={key(r):r for r in predictions};scored={key(r):r for r in scores['rows']}
    assert len(sources)==len(pred)==len(scored)==len(source)==len(predictions)==len(scores['rows'])==434
    assert set(sources)==set(pred)==set(scored)
    values={};projected=checks=0;largest=0.
    for identity,r in sources.items():
        p,s=pred[identity],scored[identity]
        assert p['sensor_interval_ns']==r['sensor_interval_ns'] and p['common_interval_ns']==r['common_interval_ns']
        expected='source_unavailable' if r['own_body_delta'] is None else 'non_processing_stage' if r['motion_stage']!='pair_processing' else 'model_projection_only'
        assert p['reason']==s['reason']==expected
        eligible=expected=='model_projection_only'
        assert p['valid_model_projection'] is eligible and s['valid_model_projection'] is eligible
        if not eligible:
            assert p['delta_model'] is None and s['losses'] is None
            continue
        assert p['admission_ready'] is False and p['reference_used_for_prediction'] is False
        a,b=r['sensor_interval_ns'];r0,r1=r['common_interval_ns'];ratio=(r1-r0)/(b-a)
        outside=max(a-r0,r0-b,a-r1,r1-b,0)
        assert outside<=b-a and p['horizon_ns']==b-a and p['max_outside_native_ns']==outside and p['duration_ratio']==ratio
        assert p['native_interval_ns']==[a,b] and p['target_interval_ns']==[r0,r1]
        matrix=expm(logm(np.array(r['own_body_delta']))*ratio)
        assert np.max(np.abs(matrix.imag))<1e-9
        difference=float(np.max(np.abs(matrix.real-np.array(p['delta_model']))));largest=max(largest,difference)
        np.testing.assert_allclose(matrix.real,p['delta_model'],rtol=1e-8,atol=1e-9)
        refs=r['reference_poses'];ref=np.linalg.solve(np.array(refs['common_start']),np.array(refs['common_end']))
        methods={'model_proxy':independent_loss(matrix.real,ref),
            'unchanged_native_proxy':independent_loss(r['own_body_delta'],ref),
            'historic_bridged_proxy':independent_loss(r['common_body_delta'],ref),
            'model_vs_historic_bridge':independent_loss(matrix.real,r['common_body_delta'])}
        for method,metrics in methods.items():
            for metric,value in metrics.items():
                np.testing.assert_allclose(value,s['losses'][method][metric],rtol=1e-7,atol=2e-9);checks+=1
        values.setdefault(identity[:2],[]).append(methods);projected+=1
    for group,summary in scores['groups'].items():
        window,camera=group.split('/');rows=[r for r in predictions if r['window']==window and r['camera']==camera]
        assert summary['slots']==len(rows) and summary['eligible']==len(values[window,camera])
        for method,metrics in summary['metrics'].items():
            for metric,statistics in metrics.items():
                x=np.array([r[method][metric] for r in values[window,camera]])
                expected={'count':len(x),'mean':x.mean(),'median':np.median(x),'rmse':np.linalg.norm(x)/np.sqrt(len(x)),'max':x.max()}
                for name,value in expected.items():np.testing.assert_allclose(value,statistics[name],rtol=1e-7,atol=2e-9);checks+=1
    assert projected==331
    return {'status':'PASS','rows':434,'independently_projected':projected,'scalar_checks':checks,
            'max_matrix_absolute_difference':largest,'method':'independent scipy.linalg logm/expm and direct translation/atan2 rotation arithmetic'}


def audit(root,run):
    protocol=read(run/'protocol.json');proof=read(run/'prediction-proof.json');pred=read(run/'predictions.json');scores=read(run/'scores.json')
    assert proof['predictions_sha256']==scores['predictions_sha256']==sha(run/'predictions.json')
    assert proof['protocol_sha256']==pred['protocol_sha256']==sha(run/'protocol.json')
    assert all(sha(root/p)==h for p,h in protocol['input_sha256'].items())
    source=[]
    for window,name in [('early','end-to-end-solver-control'),('middle','solver-collapse-replication'),('day','solver-day-replication')]:
        source += [dict(r,window=window) for r in read(root/'results'/name/'run-001/evaluation.json')['rows'] if r['arm']=='original']
    result=audit_payload(source,pred['rows'],scores)
    result.update(predictions_sha256=sha(run/'predictions.json'),scores_sha256=sha(run/'scores.json'),auditor_sha256=sha(__file__))
    with (run/'validation.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();audit(a.root,a.run)
