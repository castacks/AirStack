"""Frozen three-arm monocular diagnostic; no training or parameter search."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PREFLIGHT = ROOT/'results/smokeseer-preflight'
OUT = ROOT/'results/smokeseer-sift'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def sift():
    return cv2.SIFT_create(nfeatures=4096, nOctaveLayers=3, contrastThreshold=.04,
                          edgeThreshold=10, sigma=1.6, enable_precise_upscale=False)


def gray(array):
    v = array.astype(np.float64)/np.iinfo(array.dtype).max
    if array.ndim == 2:
        result = v
    else:
        result = .299*v[..., 2] + .587*v[..., 1] + .114*v[..., 0]
        if array.shape[2] == 4:
            result *= v[..., 3]
    return np.clip(np.rint(result*255), 0, 255).astype(np.uint8)


def mutual_matches(a, b):
    if a is None or b is None or len(a)<2 or len(b)<2:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    def accepted(x, y):
        return {m.queryIdx: m.trainIdx for pair in matcher.knnMatch(x, y, k=2)
                if len(pair)==2 for m,n in [pair] if m.distance < .75*n.distance}
    forward, reverse = accepted(a,b), accepted(b,a)
    return sorted((i,j) for i,j in forward.items() if reverse.get(j)==i)


def pose(points0, points1, k):
    cv2.setRNGSeed(0)
    result = {'status':'insufficient_matches', 'matches':len(points0)}
    if len(points0)<8:
        return result
    try:
        e, mask = cv2.findEssentialMat(points0, points1, cameraMatrix=k,
                    method=cv2.RANSAC, prob=.999, threshold=1., maxIters=1000)
        if e is None or mask is None:
            return dict(result, status='essential_unavailable')
        result['E'] = e.tolist()
        result['essential_mask'] = mask.reshape(-1).tolist()
        if e.shape != (3,3):
            return dict(result, status='essential_not_single_3x3')
        if not np.isfinite(e).all():
            return dict(result, status='essential_nonfinite')
        count, rotation, translation, selected = cv2.recoverPose(
            e, points0, points1, k, mask=mask.copy())
        result.update(cheirality_inliers=int(count), R=rotation.tolist(),
                      t=translation.reshape(-1).tolist(), pose_mask=selected.reshape(-1).tolist())
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            return dict(result, status='pose_nonfinite')
        return dict(result, status='ok' if count>=8 else 'insufficient_cheirality')
    except cv2.error as exc:
        return dict(result, status='opencv_error', error=str(exc))


def verify_input_freeze():
    freeze = json.loads((PREFLIGHT/'freeze.json').read_text())
    for name, digest in freeze['pins'].items():
        assert sha(ROOT/name)==digest, name
    assert cv2.__version__=='5.0.0' and np.__version__=='2.2.6'
    return freeze


def qualify():
    OUT.mkdir(parents=True, exist_ok=True)
    assert not (OUT/'qualification.json').exists()
    cv2.setNumThreads(1)
    freeze = verify_input_freeze()
    checks = 0
    # Explicit pixel-value controls include alpha and 16-bit data.
    for dtype in [np.uint8, np.uint16]:
        maximum = np.iinfo(dtype).max
        rgba = np.array([[[maximum,0,0,maximum],[0,0,maximum,0]]],dtype=dtype)
        np.testing.assert_array_equal(gray(rgba), [[29,0]])
        checks += 1
    rng = np.random.default_rng(21)
    image = rng.integers(0,256,(192,192),dtype=np.uint8)
    detector = sift()
    k0,d0 = detector.detectAndCompute(image,None)
    k1,d1 = detector.detectAndCompute(image,None)
    assert len(k0)>8
    np.testing.assert_array_equal(d0,d1)
    np.testing.assert_array_equal([x.pt for x in k0],[x.pt for x in k1])
    assert len(mutual_matches(d0,d1))>=8 and mutual_matches(None,d1)==[]
    assert pose(np.empty((0,2)),np.empty((0,2)),np.eye(3))['status']=='insufficient_matches'
    checks += 5
    # Nonplanar exact geometry: verifies API, coordinate direction and t sign.
    world = rng.uniform([-1,-1,4],[1,1,8],(150,3))
    rotation = cv2.Rodrigues(np.array([.02,.07,-.03]))[0]
    translation = np.array([.3,-.02,.1])
    k = np.array([[700.,0,400],[0,700,400],[0,0,1]])
    def project(x):
        uv = x@k.T
        return uv[:,:2]/uv[:,2:]
    a,b = project(world),project(world@rotation.T+translation)
    first,second = pose(a,b,k),pose(a,b,k)
    assert first==second and first['status']=='ok'
    # A thresholded five-point RANSAC estimate need not recover exact geometry
    # to machine precision. Qualify direction independently with known E.
    tx,ty,tz=translation
    cross=np.array([[0,-tz,ty],[tz,0,-tx],[-ty,tx,0]])
    exact_e=cross@rotation
    count,known_r,known_t,_=cv2.recoverPose(exact_e,a,b,k,mask=np.ones((len(a),1),np.uint8))
    assert count==len(a)
    np.testing.assert_allclose(known_r, rotation, atol=1e-6)
    np.testing.assert_allclose(known_t.reshape(-1), translation/np.linalg.norm(translation), atol=1e-6)
    # Confirm estimated E follows OpenCV's documented pixel-distance contract.
    h0=np.column_stack([a,np.ones(len(a))])@np.linalg.inv(k).T
    h1=np.column_stack([b,np.ones(len(b))])@np.linalg.inv(k).T
    estimated=np.asarray(first['E'])
    f=np.linalg.inv(k).T@estimated@np.linalg.inv(k)
    p0=np.column_stack([a,np.ones(len(a))]); p1=np.column_stack([b,np.ones(len(b))])
    lines0=p1@f; lines1=p0@f.T
    residual=np.abs(np.sum(p1*lines1,axis=1))
    sampson=residual/np.sqrt(np.sum(lines0[:,:2]**2,axis=1)+np.sum(lines1[:,:2]**2,axis=1))
    mask=np.asarray(first['essential_mask'],bool)
    assert mask.sum()>=8 and np.max(sampson[mask])<=1
    checks += 5
    # Whole population revalidation before the first real feature prediction.
    receipts = json.loads((PREFLIGHT/'image-receipts.json').read_text())
    cache = OUT/'gray'
    cache.mkdir(exist_ok=False)
    gray_pins = {}
    for i,row in enumerate(receipts):
        path = ROOT/'data/smokeseer-synthetic'/row['member']
        assert sha(path)==row['sha256'], row['member']
        array = cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
        assert array is not None and hashlib.sha256(array.tobytes()).hexdigest()==row['decoded_sha256']
        value = gray(array)
        assert hashlib.sha256(value.tobytes()).hexdigest()==row['gray_sha256']
        target = cache/f'{i:04}.npy'
        np.save(target,value,allow_pickle=False)
        gray_pins[row['member']]={'path':str(target.relative_to(ROOT)), 'sha256':sha(target)}
        checks += 3
        if (i+1)%500==0:
            print('Qualified images',i+1,flush=True)
    write(OUT/'gray-pins.json',gray_pins)
    write(OUT/'qualification.json',{'status':'pass','checks':checks,'synthetic_geometry_points':150,
          'images':len(receipts),'real_pair_predictions':0,'opencv':cv2.__version__,
          'numpy':np.__version__,'threads':cv2.getNumThreads(),'completed_utc':datetime.now(timezone.utc).isoformat()})
    pins = {str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__).resolve(),
            PREFLIGHT/'freeze.json',OUT/'gray-pins.json',OUT/'qualification.json']}
    pins.update(freeze['pins'])
    write(OUT/'execution-freeze.json',{'pins':pins,'runtime_image':freeze['runtime_image'],
          'created_utc':datetime.now(timezone.utc).isoformat(),
          'ground_truth_use':'Scoring only; estimator receives pixels and K',
          'geometry_scoring':'Closest SO3 projection of float-exported reference rotation; directed t angle',
          'recoverPose_distance_threshold':'OpenCV default, unchanged'})
    print('Qualification passed',checks,'checks; execution frozen',flush=True)


def verify_execution():
    freeze = json.loads((OUT/'execution-freeze.json').read_text())
    for name,digest in freeze['pins'].items():
        assert sha(ROOT/name)==digest,name
    verify_input_freeze()
    return json.loads((OUT/'gray-pins.json').read_text())


def run():
    cv2.setNumThreads(1)
    pins = verify_execution()
    # Verify every cache before first real SIFT call, not only when reached.
    for row in pins.values():
        assert sha(ROOT/row['path'])==row['sha256']
    rows=json.loads((PREFLIGHT/'declared-pairs.json').read_text())
    detector=sift()
    previous=None
    calls=0
    def features(name):
        nonlocal calls
        image=np.load(ROOT/pins[name]['path'],allow_pickle=False)
        cv2.setRNGSeed(0)
        keypoints,descriptors=detector.detectAndCompute(image,None)
        calls+=1
        return np.array([x.pt for x in keypoints],dtype=np.float64).reshape(-1,2),descriptors
    started=time.monotonic()
    with (OUT/'predictions.jsonl').open('x') as journal:
        for number,row in enumerate(rows):
            start=time.monotonic()
            if previous is not None and previous[0]==row['source']:
                points0,descriptors0=previous[1]
            else:
                points0,descriptors0=features(row['source'])
            points1,descriptors1=features(row['target'])
            previous=(row['target'],(points1,descriptors1))
            matched=mutual_matches(descriptors0,descriptors1)
            a=np.array([points0[i] for i,j in matched],dtype=np.float64).reshape(-1,2)
            b=np.array([points1[j] for i,j in matched],dtype=np.float64).reshape(-1,2)
            result=pose(a,b,np.asarray(row['K']))
            result.update(scene=row['scene'],arm=row['arm'],index=row['index'],
                          source=row['source'],target=row['target'],
                          source_keypoints=len(points0),target_keypoints=len(points1),
                          points0=a.tolist(),points1=b.tolist(),
                          elapsed_seconds=time.monotonic()-start)
            journal.write(json.dumps(result,allow_nan=False)+'\n')
            journal.flush()
            if (number+1)%100==0:
                print('Completed',number+1,'/',len(rows),'elapsed',round(time.monotonic()-started,1),flush=True)
    verify_execution()
    write(OUT/'prediction-lock.json',{'rows':len(rows),'sift_image_calls':calls,
          'elapsed_seconds':time.monotonic()-started,'predictions_sha256':sha(OUT/'predictions.jsonl'),
          'execution_freeze_sha256':sha(OUT/'execution-freeze.json'),
          'completed_utc':datetime.now(timezone.utc).isoformat(),'scoring_started':False})


def score():
    verify_execution()
    lock=json.loads((OUT/'prediction-lock.json').read_text())
    assert sha(OUT/'predictions.jsonl')==lock['predictions_sha256']
    predictions=[json.loads(s) for s in (OUT/'predictions.jsonl').read_text().splitlines()]
    references=json.loads((PREFLIGHT/'declared-pairs.json').read_text())
    assert len(predictions)==len(references)==3465
    scored=[]
    for row,ref in zip(predictions,references):
        assert all(row[k]==ref[k] for k in ['scene','arm','index','source','target'])
        record={k:row[k] for k in ['scene','arm','index','status']}
        record.update(supported=row['status']=='ok',rotation_error_deg=None,translation_error_deg=None,
                      primary_success=False,joint_success=False,identity_success=ref['reference_rotation_deg']<=5)
        if record['supported']:
            t=np.asarray(ref['source_to_next_optical'])
            u,_,vt=np.linalg.svd(t[:3,:3])
            rotation=u@vt
            delta=np.asarray(row['R'])@rotation.T
            record['rotation_error_deg']=float(np.degrees(np.arccos(np.clip((np.trace(delta)-1)/2,-1,1))))
            if ref['translation_direction_defined']:
                truth=t[:3,3]/np.linalg.norm(t[:3,3])
                estimate=np.asarray(row['t']); estimate/=np.linalg.norm(estimate)
                record['translation_error_deg']=float(np.degrees(np.arccos(np.clip(truth@estimate,-1,1))))
            record['primary_success']=record['rotation_error_deg']<=5
            record['joint_success']=bool(record['primary_success'] and record['translation_error_deg'] is not None and record['translation_error_deg']<=15)
        scored.append(record)
    def aggregate(group):
        n=len(group); supported=[r for r in group if r['supported']]
        return {'pairs':n,'supported':len(supported),'primary_success':sum(r['primary_success'] for r in group),
                'joint_success':sum(r['joint_success'] for r in group),'identity_success':sum(r['identity_success'] for r in group),
                'primary_rate':sum(r['primary_success'] for r in group)/n,
                'support_rate':len(supported)/n,'joint_rate':sum(r['joint_success'] for r in group)/n,
                'conditional_rotation_median_deg':float(np.median([r['rotation_error_deg'] for r in supported])) if supported else None,
                'conditional_translation_median_deg':float(np.median([r['translation_error_deg'] for r in supported])) if supported else None,
                'status_counts':dict(Counter(r['status'] for r in group))}
    summary={'pooled':{},'scenes':{},'macro':{}}
    for arm in ['rgb_smoke','rgb_clean','thermal']:
        summary['pooled'][arm]=aggregate([r for r in scored if r['arm']==arm])
        scene_results={scene:aggregate([r for r in scored if r['arm']==arm and r['scene']==scene]) for scene in sorted({r['scene'] for r in scored})}
        summary['scenes'][arm]=scene_results
        summary['macro'][arm]={key:float(np.mean([v[key] for v in scene_results.values()])) for key in ['primary_rate','support_rate','joint_rate']}
    write(OUT/'scores.json',scored)
    write(OUT/'summary.json',summary)
    print(json.dumps({'pooled':summary['pooled'],'macro':summary['macro']},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('stage',choices=['qualify','run','score'])
    stage=parser.parse_args().stage
    {'qualify':qualify,'run':run,'score':score}[stage]()
