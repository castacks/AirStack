"""Independent full-population score/coverage audit; does not call the estimator."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/smokeseer-sift'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def angle_vectors(a,b):
    a=np.asarray(a,dtype=float); b=np.asarray(b,dtype=float)
    a=a/np.linalg.norm(a); b=b/np.linalg.norm(b)
    return math.degrees(math.atan2(np.linalg.norm(np.cross(a,b)),np.dot(a,b)))


def angle_rotations(a,b):
    u,_,vt=np.linalg.svd(np.asarray(b,dtype=float))
    relative=np.asarray(a,dtype=float).T@(u@vt)
    sine=np.linalg.norm([relative[2,1]-relative[1,2],relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]])/2
    cosine=(np.trace(relative)-1)/2
    return math.degrees(math.atan2(sine,cosine))


def evaluate(pred,ref):
    supported=pred['status']=='ok'
    if not supported:
        return supported,False,False,None,None
    t=np.asarray(ref['source_to_next_optical'])
    rerr=angle_rotations(pred['R'],t[:3,:3])
    terr=angle_vectors(pred['t'],t[:3,3]) if ref['translation_direction_defined'] else None
    return True,rerr<=5,rerr<=5 and terr is not None and terr<=15,rerr,terr


def main():
    checks=0
    freeze=json.loads((OUT/'execution-freeze.json').read_text())
    for name,expected in freeze['pins'].items():
        assert digest(ROOT/name)==expected,name
        checks+=1
    lock=json.loads((OUT/'prediction-lock.json').read_text())
    assert digest(OUT/'execution-freeze.json')==lock['execution_freeze_sha256']
    assert digest(OUT/'predictions.jsonl')==lock['predictions_sha256']
    reference=json.loads((ROOT/'results/smokeseer-preflight/declared-pairs.json').read_text())
    predictions=[json.loads(line) for line in (OUT/'predictions.jsonl').read_text().splitlines()]
    scores=json.loads((OUT/'scores.json').read_text())
    summary=json.loads((OUT/'summary.json').read_text())
    assert len(reference)==len(predictions)==len(scores)==3465
    assert lock['rows']==3465 and lock['sift_image_calls']==3495
    keys=set(); independent=[]
    for ref,pred,score in zip(reference,predictions,scores):
        key=tuple(ref[x] for x in ['scene','arm','index'])
        assert key not in keys
        keys.add(key)
        for field in ['scene','arm','index','source','target']:
            assert pred[field]==ref[field]
            checks+=1
        assert len(pred['points0'])==len(pred['points1'])==pred['matches']
        assert pred['matches']<=min(pred['source_keypoints'],pred['target_keypoints'])
        if pred['status']=='ok':
            r=np.asarray(pred['R']); t=np.asarray(pred['t'])
            np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-8)
            assert abs(np.linalg.det(r)-1)<1e-8 and abs(np.linalg.norm(t)-1)<1e-8
            assert pred['matches']>=8 and pred['cheirality_inliers']>=8
            assert sum(bool(x) for x in pred['pose_mask'])==pred['cheirality_inliers']
            assert all(not bool(a) or bool(b) for a,b in zip(pred['pose_mask'],pred['essential_mask']))
            checks+=5
        supported,primary,joint,rerr,terr=evaluate(pred,ref)
        assert (score['supported'],score['primary_success'],score['joint_success'])==(supported,primary,joint)
        for computed,name in [(rerr,'rotation_error_deg'),(terr,'translation_error_deg')]:
            if computed is None:
                assert score[name] is None
            else:
                assert abs(computed-score[name])<1e-5,(key,name,computed,score[name])
            checks+=1
        identity=angle_rotations(np.eye(3),np.asarray(ref['source_to_next_optical'])[:3,:3])<=5
        assert identity==score['identity_success']
        independent.append(dict(scene=ref['scene'],arm=ref['arm'],status=pred['status'],supported=supported,
                        primary=primary,joint=joint,rerr=rerr,terr=terr,identity=identity))
        checks+=5
    for arm in ['rgb_smoke','rgb_clean','thermal']:
        for scene in [None]+sorted({x['scene'] for x in independent}):
            group=[x for x in independent if x['arm']==arm and (scene is None or x['scene']==scene)]
            expected=summary['pooled'][arm] if scene is None else summary['scenes'][arm][scene]
            supported=[x for x in group if x['supported']]
            values={'pairs':len(group),'supported':len(supported),'primary_success':sum(x['primary'] for x in group),
                    'joint_success':sum(x['joint'] for x in group),'identity_success':sum(x['identity'] for x in group)}
            for name,value in values.items():
                assert value==expected[name],(arm,scene,name)
                checks+=1
            assert dict(Counter(x['status'] for x in group))==expected['status_counts']
            for name,count in [('primary_rate','primary_success'),('joint_rate','joint_success'),('support_rate','supported')]:
                assert abs(values[count]/len(group)-expected[name])<1e-12
                checks+=1
            for name,field in [('conditional_rotation_median_deg','rerr'),('conditional_translation_median_deg','terr')]:
                value=statistics.median(x[field] for x in supported) if supported else None
                assert value is expected[name] if value is None else abs(value-expected[name])<1e-5
                checks+=1
        for field in ['primary_rate','support_rate','joint_rate']:
            value=statistics.mean(x[field] for x in summary['scenes'][arm].values())
            assert abs(value-summary['macro'][arm][field])<1e-12
            checks+=1
    # Direction-sign, failed-support and rotation-direction mutation guards.
    identity=np.eye(4); identity[0,3]=1
    ref={'source_to_next_optical':identity.tolist(),'translation_direction_defined':True}
    base={'status':'ok','R':np.eye(3).tolist(),'t':[1,0,0]}
    assert evaluate(base,ref)[2]
    assert not evaluate(dict(base,t=[-1,0,0]),ref)[2]
    assert not evaluate(dict(base,status='insufficient_cheirality'),ref)[1]
    a=.2; r=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
    assert abs(angle_rotations(r,r.T)-math.degrees(.4))<1e-10
    result={'status':'pass','checks':checks,'mutation_guards':3,'rows':len(independent),
            'scope':'All pins, unique pair order/coverage, recovered rotations/directions, masks, independent angular scores, scene/pooled denominators and macro rates',
            'prediction_sha256':digest(OUT/'predictions.jsonl')}
    with (OUT/'audit.json').open('x') as stream:
        json.dump(result,stream,indent=2)
        stream.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
