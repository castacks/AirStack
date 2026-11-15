"""A finite matched-count path comparison, not an adaptive keyframe method."""
import argparse
import hashlib
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import continuity_sequence as sequence
from qualify_native_frontend import read, write, sha, now
from audit_paired_frontend import independent_selection, point_covariance, hat


def measured(plan):
    parent = read(Path(plan['parent_capture'])/'predictions.json')['rows']
    new = read(Path(plan['capture_output'])/'predictions.json')['rows']
    return parent+new


def compose_path(fits, route):
    T = np.eye(4)
    for a, b in zip(route[:-1], route[1:]):
        edge = fits[(a, b)]
        if not edge['successful']:
            return None
        T = T@np.array(edge['target_to_source'])
    return T


def score(plan):
    rows = measured(plan)
    reports = []
    for scene in read(plan['inventory'])['records']:
        with np.load(scene['path']) as z:
            truth = z['world_from_camera']
        fits = {(r['source'], r['target']): r['fit'] for r in rows if r['scene'] == scene['id']}
        arms = {}
        for name, intermediate in [('via1', 1), ('via2', 2)]:
            values = []
            for i in [6, 7, 8, 9]:
                T = compose_path(fits, [0, intermediate, 6]+list(range(7, i+1)))
                values.append({'frame': i, 'pose': None if T is None else T.tolist(), 'error': None if T is None else sequence.error(T, truth[i])})
            valid = [v['error'] for v in values if v['error'] is not None]
            arms[name] = {'estimated_edges_to_return': 2, 'planned_postgap': 4, 'available_postgap': len(valid), 'rows': values,
                          'translation_rmse_m': None if not valid else float(np.sqrt(np.mean([v['translation_m']**2 for v in valid]))),
                          'rotation_rmse_deg': None if not valid else float(np.sqrt(np.mean([v['rotation_deg']**2 for v in valid])))}
        reports.append({'scene': scene['id'], 'arms': arms})
    out = Path(plan['score_output'])
    out.mkdir(parents=True, exist_ok=False)
    write(out/'scores.json', {'created_utc': now(), 'records': reports, 'protocol_sha256': sha(plan['protocol_path']),
                             'primary': 'return_frame6_translation_and_rotation_errors', 'secondary': 'frames6to9_original_frame_rmse',
                             'scope': 'Two fixed paths, same endpoints and composition count; reference timing/geometry jointly differ. No adaptive policy, attribution of all causes or new-method promotion.'})


def audit(plan):
    checks = 0
    def check(x, text):
        nonlocal checks
        assert x, text
        checks += 1
    def close(x, y, rtol=1e-8, atol=1e-10):
        nonlocal checks
        np.testing.assert_allclose(x, y, rtol=rtol, atol=atol)
        checks += 1
    for folder in [plan['parent_capture'], plan['capture_output']]:
        for path, digest in read(Path(folder)/'prediction-lock.json')['files'].items():
            check(sha(Path(folder)/path) == digest, 'prediction lock')
    pred = read(Path(plan['capture_output'])/'predictions.json')
    check(len(pred['rows']) == 12 and pred['model_state_unchanged'] and not pred['true_pose_read'], 'new complete population/provenance')
    check(pred['loading']['migration'] == 'none', 'strict checkpoint')
    K = np.array(plan['K'])
    success = 0
    for row in pred['rows']:
        check((row['source'], row['target']) in [(0, 2), (1, 6)], 'fixed new edges')
        scene = next(s for s in read(plan['inventory'])['records'] if s['id'] == row['scene'])
        with np.load(scene['path']) as z:
            im = z['images'].astype(np.float32)/255
        a, b = row['source'], row['target']
        for images, key in [(np.stack([im[a, 0], im[b, 0], im[a, 0]]), 'source_input_sha256'), (np.stack([im[a, 1], im[b, 1], im[b, 0]]), 'target_input_sha256')]:
            check(hashlib.sha256(images.tobytes()).hexdigest() == row[key], 'input binding')
        check(sha(row['dense_path']) == row['dense_sha256'], 'dense binding')
        with np.load(row['dense_path']) as z:
            f, c = z['flow'], z['covariance']
        check(f.shape == c.shape == (3, 2, 256, 640) and np.isfinite(f).all() and np.isfinite(c).all() and (c > 0).all(), 'finite dense')
        ids = independent_selection(c, plan['selection_seed'])
        check(ids.tolist() == row['proposed_ids'], 'selection')
        result = row['fit']
        if result['status'] == 'INVALID_PACKET':
            check(not result['successful'], 'invalid retained')
            continue
        check(sha(result['packet_path']) == result['packet_sha256'], 'packet binding')
        target = ids.astype(np.float32)+f[2, :, ids[:, 1], ids[:, 0]]
        good = (target[:, 0] > 32)&(target[:, 0] < 608)&(target[:, 1] > 32)&(target[:, 1] < 224)
        u0, u1 = ids[good], target[good]
        d = [np.float32(K[0, 0]*plan['baseline_m'])/np.abs(f[j, 0]) for j in [0, 1]]
        x, y = u0[:, 0], u0[:, 1]
        C0 = point_covariance(d[0], u0, np.full(len(u0), .25, np.float32), np.full(len(u0), .25, np.float32), K)
        C1 = point_covariance(d[1], u1, np.maximum(c[2, 0, y, x], .0625), np.maximum(c[2, 1, y, x], .0625), K)
        mask = np.isfinite(C0).all((1, 2))&np.isfinite(C1).all((1, 2))
        with np.load(result['packet_path']) as z:
            check(np.array_equal(z['source_ids'], u0[mask]), 'packet admission')
            close(z['C0'], C0[mask], rtol=4e-4, atol=2e-7)
            close(z['C1'], C1[mask], rtol=4e-4, atol=2e-7)
            for j, uv in enumerate([u0, u1]):
                depth = d[j][uv[:, 1].astype(int), uv[:, 0].astype(int)].astype(float)
                expected = np.column_stack([(uv[:, 0].astype(float)-K[0, 2])*depth/K[0, 0], (uv[:, 1].astype(float)-K[1, 2])*depth/K[1, 1], depth])
                close(z['X'+str(j)], expected[mask], rtol=3e-5, atol=1e-8)
            X0, X1, S0, S1 = [z[k] for k in ['X0', 'X1', 'C0', 'C1']]
        T = np.array(result['target_to_source'])
        close(T[:3, :3].T@T[:3, :3], np.eye(3))
        if result['successful']:
            R = T[:3, :3]
            q = X1@R.T+T[:3, 3]
            H, g = np.zeros((6, 6)), np.zeros(6)
            for p, r, C in zip(q, q-X0, S0+R@S1@R.T):
                J = np.concatenate([np.eye(3), -hat(p)], axis=1)
                H += J.T@np.linalg.solve(C, J)
                g += J.T@np.linalg.solve(C, r)
            check(np.sqrt(max(g@np.linalg.solve(H, g), 0)) <= plan['solver']['score_tolerance']*1.01, 'stationarity')
            close(result['information_covariance'], np.linalg.inv(H), rtol=1e-7)
            success += 1
    all_rows = measured(plan)
    report = read(Path(plan['score_output'])/'scores.json')
    for scene in read(plan['inventory'])['records']:
        key = scene['id']
        check(len([r for r in pred['rows'] if r['scene'] == key]) == 2, 'two new fits per scene')
        with np.load(scene['path']) as z:
            truth = z['world_from_camera']
        data = next(r for r in report['records'] if r['scene'] == key)
        lookup = {(r['source'], r['target']): r['fit'] for r in all_rows if r['scene'] == key}
        for name, k in [('via1', 1), ('via2', 2)]:
            arm = data['arms'][name]
            check(arm['estimated_edges_to_return'] == 2 and arm['planned_postgap'] == 4, 'matched count/denominator')
            values = []
            for row in arm['rows']:
                route = [0, k, 6]+list(range(7, row['frame']+1))
                T = np.eye(4)
                for a, b in zip(route[:-1], route[1:]):
                    fit = lookup[(a, b)]
                    T = T@np.array(fit['target_to_source']) if T is not None and fit['successful'] else None
                check((row['pose'] is not None) == (T is not None), 'failure propagation')
                if T is None:
                    check(row['error'] is None, 'missing retained')
                    continue
                close(row['pose'], T)
                expected = truth[row['frame']]
                t = np.linalg.norm(T[:3, 3]-expected[:3, 3])
                r = np.rad2deg(Rotation.from_matrix(expected[:3, :3].T@T[:3, :3]).magnitude())
                close(row['error']['translation_m'], t)
                close(row['error']['rotation_deg'], r)
                values.append([t, r])
            check(arm['available_postgap'] == len(values), 'availability')
            if values:
                close(arm['translation_rmse_m'], np.sqrt(np.mean(np.array(values)[:, 0]**2)))
                close(arm['rotation_rmse_deg'], np.sqrt(np.mean(np.array(values)[:, 1]**2)))
    write(Path(plan['score_output'])/'audit.json', {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'successful_new_fits': success, 'planned_new_fits': 12})
    print('PASS', checks, 'checks;', success, '/12 new fits converge', flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['capture', 'score', 'audit'])
    ap.add_argument('--protocol', required=True)
    args = ap.parse_args()
    plan = read(args.protocol)
    sequence.verify(plan)
    if args.mode == 'capture':
        sequence.edges = lambda: [(0, 2), (1, 6)]
        sequence.capture(plan, Path(plan['capture_output']))
    elif args.mode == 'score':
        score(plan)
    else:
        audit(plan)
    sequence.verify(plan)
