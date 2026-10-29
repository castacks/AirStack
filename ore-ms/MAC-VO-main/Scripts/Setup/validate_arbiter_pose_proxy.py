"""Independent source-journal reconstruction of conditional offline pose labels."""
import argparse
from collections import defaultdict
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from Scripts.Setup.td3_reference import load_reference, camera_reference


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix(v):
    result=np.eye(4);result[:3,:3]=Rotation.from_quat(v[3:7]).as_matrix();result[:3,3]=v[:3];return result


def metrics(estimate,truth):
    # Direct residual components instead of composing the producer's loss matrix.
    rt=truth[:3,:3].T
    return {'translation_m':float(np.linalg.norm(rt@(estimate[:3,3]-truth[:3,3]))),
            'rotation_deg':float(np.degrees(Rotation.from_matrix(rt@estimate[:3,:3]).magnitude()))}


def stats(v):
    v=np.asarray(v,float)
    if not len(v):return {'count':0,'mean':None,'median':None,'rmse':None}
    return dict(count=len(v),mean=float(np.sum(v)/len(v)),median=float(np.median(v)),rmse=float(np.linalg.norm(v)/np.sqrt(len(v))))


def validate(root,run):
    protocol=json.loads((run/'protocol.json').read_text());report=json.loads((run/'pose-proxy.json').read_text())
    assert not protocol['physical_body_transform_verified'] and not protocol['independent_ground_truth']
    assert not protocol['causal_feature_eligible'] and not protocol['gate_fitted']
    assert protocol['reference_used_for_offline_transport']
    for rel,digest in protocol['input_sha256'].items():assert sha(root/rel)==digest,rel
    source=root/'data/tartandrive3/20251027/night_mac_salon_warehousefig8'
    d=source/'super_odometry/odometry';times=np.array([int(Decimal(s)*1000000000) for s in (d/'interp_timestamps.txt').read_text().splitlines()],np.int64)
    poses=np.loadtxt(d/'interp_data.txt');seconds=(times-times[0])/1e9
    slerp=Slerp(seconds,Rotation.from_quat(poses[:,3:7]));oldref,_=load_reference(source)
    def reference(t):
        assert type(t) is int and times[0]<=t<=times[-1]
        q=(t-int(times[0]))/1e9
        result=np.eye(4);result[:3,:3]=slerp([q]).as_matrix()[0]
        result[:3,3]=[np.interp(q,seconds,poses[:,i]) for i in range(3)]
        return result
    expected_rows=[];checked=0
    for window,rel,datarel in (
        ('early','shared-arbiter-events','new-night-validation/prepared-001/datasets/early'),
        ('middle','arbiter-support-collapse','long-night-validation/prepared-001/datasets/middle')):
        path=root/'results'/rel/'run-001';dataset=root/'results'/datarel
        timeline=[json.loads(s) for s in (path/'timeline.jsonl').read_text().splitlines()]
        groups=json.loads((path/'protocol.json').read_text())['input_rows']
        events={c:[json.loads(s) for s in (path/'experts'/c/'arbiter-events/events.jsonl').read_text().splitlines()] for c in ('rgb','thermal')}
        for decision in (r for r in timeline if r['kind']=='decision'):
            i=decision['pair_ordinal'];row=report['rows'][len(expected_rows)]
            assert row['window']==window and row['pair_ordinal']==i and row['source_decision_sequence']==decision['sequence'] and row['action']==decision['action']
            common=[groups[j]['rgb_left']['time_ns'] for j in (i,i+1)];r0,r1=map(reference,common);truth=np.linalg.solve(r0,r1)
            assert row['common_interval_ns']==common
            np.testing.assert_allclose(row['reference_delta'],truth,atol=1e-11,rtol=1e-11)
            for c in events:
                actual=row['experts'][c]
                if not decision['availability'][c]:assert actual is None;continue
                index=decision['evidence'][c]['motion_event_sequence'];event=events[c][index]
                assert actual['motion_event_sequence']==index<decision['observed_event_counts'][c]
                own=[groups[j][c+'_left']['time_ns'] for j in (i,i+1)];assert actual['sensor_interval_ns']==own
                a,b=map(reference,own)
                # Recover nominal E with the independently existing camera-reference path.
                camera=camera_reference(oldref,np.array([own[0]],np.int64),dataset,root/'data/calibration/yamaha.yaml',c)[0]
                e=np.linalg.solve(a,camera)
                # Independently normalize the rounded composite rotation by polar SVD.
                u,_,vt=np.linalg.svd(e[:3,:3]);e[:3,:3]=u@vt
                dc=matrix(event['derived_camera_delta_xyzw'])
                er,et=e[:3,:3],e[:3,3]
                db=np.eye(4);db[:3,:3]=er@dc[:3,:3]@er.T
                db[:3,3]=er@dc[:3,3]+et-db[:3,:3]@et
                aligned=np.linalg.solve(r0,a)@db@np.linalg.solve(b,r1)
                for name,value in [('nominal_body_from_camera',e),('own_body_delta',db),('common_body_delta',aligned)]:
                    np.testing.assert_allclose(actual[name],value,atol=1e-10,rtol=1e-10);checked+=16
                for name,pred,ref in [('own_loss',db,np.linalg.solve(a,b)),('common_loss',aligned,truth)]:
                    expected=metrics(pred,ref)
                    assert set(actual[name])==set(expected)
                    for key,value in expected.items():np.testing.assert_allclose(actual[name][key],value,atol=1e-9,rtol=1e-9);checked+=1
            expected_rows.append(row)
    assert len(report['rows'])==len(expected_rows)==118
    for window in ('early','middle'):
        rows=[r for r in expected_rows if r['window']==window];summary=report['windows'][window]
        assert summary['decisions']==len(rows)
        common=[r for r in rows if all(r['experts'][c] is not None for c in ('rgb','thermal'))]
        assert summary['common_eligible_count']==len(common)
        wins=defaultdict(int)
        for row in common:
            a,b=[row['experts'][c]['common_loss']['translation_m'] for c in ('rgb','thermal')]
            wins['tie' if abs(a-b)<=1e-12 else 'rgb' if a<b else 'thermal']+=1
        assert summary['common_translation_wins']=={c:wins[c] for c in ('rgb','thermal','tie')}
        def compare(actual,expected):
            assert actual['count']==expected['count']
            for k in ('mean','median','rmse'):
                if expected[k] is None:assert actual[k] is None
                else:np.testing.assert_allclose(actual[k],expected[k],atol=1e-11,rtol=1e-11)
        for c in ('rgb','thermal'):
            for k in ('translation_m','rotation_deg'):compare(summary['common_eligible_losses'][c][k],stats([r['experts'][c]['common_loss'][k] for r in common]))
        compare(summary['common_translation_oracle'],stats([min(r['experts'][c]['common_loss']['translation_m'] for c in ('rgb','thermal')) for r in common]))
        for policy in ('rgb_only','thermal_only','advisory'):
            values=[]
            for row in rows:
                c=row['action'].lower() if policy=='advisory' else policy.split('_')[0]
                if c in row['experts'] and row['experts'][c] is not None:values.append(row['experts'][c]['common_loss'])
            p=summary['policies'][policy];assert p['available']==len(values) and p['abstain']==len(rows)-len(values)
            for k in ('translation_m','rotation_deg'):compare(p[k],stats([v[k] for v in values]))
    result={'status':'PASS','rows':len(expected_rows),'numeric_components_checked':checked,
        'report_sha256':sha(run/'pose-proxy.json'),'protocol_sha256':sha(run/'protocol.json'),'validator_sha256':sha(Path(__file__)),
        'scope':'independent numeric/population check of nominal SLAM-proxy labels; no physical validation'}
    (run/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();validate(a.root,a.run)
