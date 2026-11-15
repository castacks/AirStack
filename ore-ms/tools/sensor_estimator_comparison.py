"""Frozen native-frontend assessment of four qualified synthetic timing arms."""
import argparse
import hashlib
from pathlib import Path

import numpy as np
from scipy.ndimage import minimum_filter

from qualify_native_frontend import read, write, sha, now, deterministic, load_model
from paired_frontend_assessment import native_algorithms, pick, packet, fit
from continuity_sequence import error

ARMS = ['sharp', 'memory', 'rolling', 'combined']


def verify(plan):
    for path, digest in plan['pins'].items():
        assert sha(path) == digest, path


def safe_mask(mask):
    result = minimum_filter(mask.astype(np.uint8), size=31, mode='constant', cval=0).astype(bool)
    crop = np.zeros_like(result)
    crop[32:-32, 32:-32] = True
    return result&crop


def eligible(ids, flow, mask):
    ids = np.asarray(ids, dtype=np.int64).reshape(-1, 2)
    uv = ids.astype(np.float32)+flow[2, :, ids[:, 1], ids[:, 0]]
    inside = (uv[:, 0] > 32)&(uv[:, 0] < 608)&(uv[:, 1] > 32)&(uv[:, 1] < 224)
    xy = np.clip(uv, [0, 0], [639, 255]).astype(np.int64)
    good = inside&mask[ids[:, 1], ids[:, 0]]&mask[xy[:, 1], xy[:, 0]]
    return ids[good]


def build(flow, covariance, camera, ids, cov_model, mask, minimum):
    ids = eligible(ids, flow, mask)
    if len(ids) < minimum:
        raise ValueError('INSUFFICIENT_SHARED_SUPPORT')
    p = packet(flow, covariance, camera, ids, cov_model)
    if len(p['source_ids']) < minimum:
        raise ValueError('INSUFFICIENT_FINITE_PACKET_SUPPORT')
    return p


def joint_ids(dense, seed, camera, cov_model, mask, minimum):
    ids = np.asarray(seed, dtype=np.int64).reshape(-1, 2)
    for f, c in dense:
        try:
            ids = build(f, c, camera, ids, cov_model, mask, minimum)['source_ids']
        except (ValueError, np.linalg.LinAlgError):
            return np.empty((0, 2), np.int64)
    return ids


def qualify(plan, out):
    selector, cov_model = native_algorithms()
    camera = {'K': plan['K'], 'baseline_m': plan['baseline_m']}
    mask = np.ones((256, 640), bool)
    safe = safe_mask(mask)
    assert safe[100, 100] and not safe[10, 10]
    f = np.zeros((3, 2, 256, 640), np.float32)
    f[:2, 0] = -np.float32(plan['K'][0][0]*plan['baseline_m']/4)
    f[2, 0] = -np.float32(plan['K'][0][0]*.02/4)
    c = np.full_like(f, .25)
    ids = np.array([(u, v) for u in [80, 160, 240, 320, 400, 480] for v in [60, 100, 140, 180]])
    p = build(f, c, camera, ids, cov_model, safe, plan['minimum_points'])
    np.testing.assert_allclose(p['X0']-p['X1'], np.tile([.02, 0., 0.], (len(p['X0']), 1)), atol=1e-6)
    result = fit(p, plan['solver'])
    assert result['successful']
    np.testing.assert_allclose(np.array(result['target_to_source'])[:3, 3], [.02, 0., 0.], atol=1e-6)
    common = joint_ids([(f, c)]*4, ids, camera, cov_model, safe, plan['minimum_points'])
    assert np.array_equal(common, ids)
    changed = f.copy()
    changed[2, 0, ids[0, 1], ids[0, 0]] = -1000
    common = joint_ids([(f, c), (changed, c), (f, c), (f, c)], ids, camera, cov_model, safe, plan['minimum_points'])
    assert np.array_equal(common, ids[1:])
    assert len(eligible(np.empty((0, 2), int), f, safe)) == 0
    assert len(joint_ids([(f, c)]*4, ids[:3], camera, cov_model, safe, plan['minimum_points'])) == 0
    null = error(np.eye(4), np.eye(4))
    assert null == {'translation_m': 0., 'rotation_deg': 0.}
    write(out, {'created_utc': now(), 'status': 'PASS', 'checks': 10,
                'scope': 'Shared-mask boundaries, analytic packet/fit, joint IDs, empty/minimum support and identity score fixtures; no learned predictions.'})
    print('PASS10mechanics fixtures; analytic pose fit converges', flush=True)


def capture(plan, out):
    import torch
    out.mkdir(parents=True, exist_ok=False)
    deterministic()
    model, loading = load_model(plan['checkpoint'])
    assert loading['migration'] == 'none'
    state = lambda: {k: hashlib.sha256(v.cpu().numpy().tobytes()).hexdigest() for k, v in model.state_dict().items()}
    original = state()
    selector, cov_model = native_algorithms()
    camera = {'K': plan['K'], 'baseline_m': plan['baseline_m']}
    rows, dense_rows = [], []
    for case in read(plan['inventory'])['records']:
        assert sha(case['path']) == case['sha256']
        with np.load(case['path']) as z:
            images = z['images'].astype(np.float32)/255
            mask = safe_mask(z['common_pixel_support'])
        dense, primaries = [], []
        for index, arm in enumerate(ARMS):
            image = images[index]
            source = np.stack([image[0, 0], image[1, 0], image[0, 0]])
            target = np.stack([image[0, 1], image[1, 1], image[1, 0]])
            a = torch.from_numpy(np.repeat(source[:, None], 3, axis=1)).cuda()
            b = torch.from_numpy(np.repeat(target[:, None], 3, axis=1)).cuda()
            with torch.inference_mode():
                f, c = model.inference(a, b)
            assert torch.isfinite(f).all() and torch.isfinite(c).all() and (c > 0).all()
            f, c = f.cpu().numpy(), c.cpu().numpy()
            dense.append((f, c))
            folder = out/case['id']/arm
            folder.mkdir(parents=True)
            path = folder/'dense.npz'
            np.savez_compressed(path, flow=f, covariance=c)
            proposed = pick(f, c, camera, selector, plan['selection_seed'])
            meta = {'case': case['id'], 'arm': arm, 'dense_path': str(path), 'dense_sha256': sha(path),
                    'source_input_sha256': hashlib.sha256(source.tobytes()).hexdigest(), 'target_input_sha256': hashlib.sha256(target.tobytes()).hexdigest(),
                    'native_proposed_ids': proposed.tolist(), 'median_temporal_covariance_trace': float(np.median(c[2].sum(0)[mask]))}
            dense_rows.append(meta)
            primaries.append((proposed, folder))
        def solve(arm_index, mode, ids):
            f, c = dense[arm_index]
            folder = primaries[arm_index][1]
            try:
                p = build(f, c, camera, ids, cov_model, mask, plan['minimum_points'])
                path = folder/(mode+'.npz')
                np.savez_compressed(path, **p)
                result = fit(p, plan['solver'])
                result.update(packet_path=str(path), packet_sha256=sha(path), accepted_ids=p['source_ids'].tolist(), points=len(p['source_ids']))
            except (ValueError, np.linalg.LinAlgError) as exc:
                result = {'successful': False, 'status': 'INSUFFICIENT_OR_INVALID_PACKET', 'reason': str(exc), 'accepted_ids': []}
            row = {'case': case['id'], 'arm': ARMS[arm_index], 'mode': mode, 'requested_ids': np.asarray(ids).tolist(), 'fit': result}
            rows.append(row)
            write(folder/(mode+'-receipt.json'), row)
            print(case['id'], ARMS[arm_index], mode, result['status'], flush=True)
            return result
        primary = [solve(i, 'regenerated', ids) for i, (ids, _) in enumerate(primaries)]
        common = joint_ids(dense, primary[0]['accepted_ids'], camera, cov_model, mask, plan['minimum_points'])
        for i in range(4):
            solve(i, 'common_ids', common)
    assert original == state()
    write(out/'predictions.json', {'created_utc': now(), 'rows': rows, 'dense_rows': dense_rows,
                                  'loading': loading, 'model_state_unchanged': True, 'true_pose_values_read': False})
    write(out/'prediction-lock.json', {'created_utc': now(), 'files': {str(p.relative_to(out)): sha(p) for p in out.rglob('*') if p.is_file()}})


def score(plan, out):
    folder = Path(plan['capture_output'])
    for rel, digest in read(folder/'prediction-lock.json')['files'].items():
        assert sha(folder/rel) == digest
    pred = read(folder/'predictions.json')
    rows = []
    cases = read(plan['inventory'])['records']
    for entry in pred['rows']:
        case = next(c for c in cases if c['id'] == entry['case'])
        with np.load(case['path']) as z:
            truth = z['target_to_source']
        result = entry['fit']
        metrics = None
        if result['successful']:
            metrics = error(np.array(result['target_to_source']), truth)
            P = np.array(result['information_covariance'])
            metrics.update(translation_information_std_proxy_m=float(np.sqrt(np.trace(P[:3, :3]))),
                           rotation_information_std_proxy_deg=float(np.rad2deg(np.sqrt(np.trace(P[3:, 3:])))))
        dense = next(r for r in pred['dense_rows'] if r['case'] == entry['case'] and r['arm'] == entry['arm'])
        rows.append({'case': entry['case'], 'motion': case['motion'], 'condition': case['condition'], 'arm': entry['arm'], 'mode': entry['mode'],
                     'successful': result['successful'], 'status': result['status'], 'points': result.get('points', 0),
                     'median_temporal_covariance_trace': dense['median_temporal_covariance_trace'], 'metrics': metrics})
    out.mkdir(parents=True, exist_ok=False)
    write(out/'scores.json', {'created_utc': now(), 'rows': rows, 'planned_dense_batches': 36, 'planned_pose_attempts': 72,
                             'prediction_lock_sha256': sha(folder/'prediction-lock.json'), 'protocol_sha256': sha(plan['protocol_path']),
                             'scope': 'Nine exposed synthetic cases; common pose timestamps, regenerated and same-query controls. No physical covariance calibration, new method or full native VO reproduction.'})


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['qualify', 'capture', 'score'])
    ap.add_argument('--protocol', required=True)
    ap.add_argument('--output')
    args = ap.parse_args()
    plan = read(args.protocol)
    verify(plan)
    output = Path(args.output) if args.output else Path(plan[args.mode+'_output'])
    {'qualify': qualify, 'capture': capture, 'score': score}[args.mode](plan, output)
    verify(plan)
