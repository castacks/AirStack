"""Finite synthetic association-validity assay with established geometric controls."""
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import continuity_sequence as sequence
from qualify_native_frontend import read, write, sha, now


def guarded_packet(flow, cov, camera, ids, covariance):
    # Native packet math is unchanged; reject empty support before tensor kernels.
    from paired_frontend_assessment import packet
    if len(ids) < 3:
        raise ValueError('INSUFFICIENT_PROPOSED_SUPPORT')
    uv = ids.astype(np.float32)+flow[2, :, ids[:, 1], ids[:, 0]]
    inside = (uv[:, 0] > 32)&(uv[:, 0] < 608)&(uv[:, 1] > 32)&(uv[:, 1] < 224)
    if inside.sum() < 3:
        raise ValueError('INSUFFICIENT_INBOUND_SUPPORT')
    return packet(flow, cov, camera, ids, covariance)


def rigid(X0, X1, rank_ratio=1e-8):
    a, b = X1-X1.mean(0), X0-X0.mean(0)
    U, s, Vt = np.linalg.svd(a.T@b)
    if len(X0) < 3 or s[1] <= rank_ratio*s[0]:
        return None
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(Vt.T@U.T)
    R = Vt.T@correction@U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, X0.mean(0)-R@X1.mean(0)
    return T


def geometric(packet, settings):
    X0, X1 = packet['X0'], packet['X1']
    if len(X0) < 3:
        return {'inlier_fraction': 0., 'inliers': 0, 'pose': None, 'hypotheses': []}
    rng = np.random.default_rng(settings['seed'])
    best, best_key, best_index, samples = None, None, None, []
    for i in range(settings['hypotheses']):
        ids = rng.choice(len(X0), 3, replace=False)
        samples.append(ids.tolist())
        T = rigid(X0[ids], X1[ids])
        if T is None:
            continue
        squared = np.sum((X1@T[:3, :3].T+T[:3, 3]-X0)**2, axis=1)
        count = int(np.sum(squared <= settings['residual_m']**2))
        key = (count, -float(np.minimum(squared, settings['residual_m']**2).sum()))
        if best_key is None or key > best_key:
            best, best_key, best_index = T, key, i
    count = 0 if best is None else best_key[0]
    return {'inlier_fraction': count/len(X0), 'inliers': count,
            'pose': None if best is None else best.tolist(), 'best_index': best_index, 'hypotheses': samples}


def auc_lower(positive, negative):
    if not positive or not negative:
        return None
    a, b = np.array(positive)[:, None], np.array(negative)[None, :]
    return float(np.mean((a < b)+.5*(a == b)))


def prepare(plan):
    out = Path(plan['prepare_output'])
    out.mkdir(parents=True, exist_ok=False)
    parents = read(plan['parent_inventory'])['records']
    rows = []
    for source in parents:
        for target in parents:
            if source['scene']['id'] != target['scene']['id']:
                continue
            for appearance in ['clean', 'gamma']:
                with np.load(source['path']) as a, np.load(target['path']) as b:
                    images = np.stack([a['images'][0], b['images'][6]])
                if appearance == 'gamma':
                    images[1] = np.rint(255*(images[1].astype(float)/255)**plan['gamma']).astype(np.uint8)
                key = source['id']+'__'+target['id']+'__'+appearance
                path = out/(key+'.npz')
                np.savez_compressed(path, images=images)
                rows.append({'id': key, 'path': str(path), 'sha256': sha(path),
                             'source_world': source['id'], 'target_world': target['id'],
                             'source_parent': source['path'], 'target_parent': target['path'],
                             'appearance': appearance, 'same_world': source['id'] == target['id']})
    write(out/'inventory.json', {'created_utc': now(), 'records': rows,
                                 'scope': 'Disjoint artificial world identity is a constructed association label, not verified natural place identity.'})


def score(plan):
    folder = Path(plan['capture_output'])
    for path, digest in read(folder/'prediction-lock.json')['files'].items():
        assert sha(folder/path) == digest
    pred = read(folder/'predictions.json')['rows']
    rows = []
    for case in read(plan['inventory'])['records']:
        directions = []
        for a, b in [(0, 1), (1, 0)]:
            entry = next(r for r in pred if r['scene'] == case['id'] and r['source'] == a and r['target'] == b)
            with np.load(entry['dense_path']) as z:
                c = z['covariance']
                uncertainty = float(np.median(c[2, :, 32:-32, 32:-32].sum(0)))
            fit = entry['fit']
            native = fit['successful'] and fit['points'] >= plan['gates']['min_points']
            geom = {'inlier_fraction': 0., 'inliers': 0, 'pose': None, 'hypotheses': []}
            if 'packet_path' in fit:
                with np.load(fit['packet_path']) as p:
                    geom = geometric(p, plan['ransac'])
            directions.append({'source': a, 'target': b, 'native_pass': native, 'uncertainty': uncertainty, 'geometry': geom})
        native = all(r['native_pass'] for r in directions)
        geometry = native and all(r['geometry']['inlier_fraction'] >= plan['gates']['inlier_fraction'] and r['geometry']['inliers'] >= plan['gates']['min_points'] for r in directions)
        cycle = None
        if all(r['geometry']['pose'] is not None for r in directions):
            closure = np.array(directions[0]['geometry']['pose'])@np.array(directions[1]['geometry']['pose'])
            cycle = {'translation_m': float(np.linalg.norm(closure[:3, 3])),
                     'rotation_deg': float(np.rad2deg(Rotation.from_matrix(closure[:3, :3]).magnitude()))}
        cyc = geometry and cycle is not None and cycle['translation_m'] <= plan['gates']['cycle_translation_m'] and cycle['rotation_deg'] <= plan['gates']['cycle_rotation_deg']
        positive_error = None
        fwd = next(r for r in pred if r['scene'] == case['id'] and r['source'] == 0)
        if case['same_world'] and fwd['fit']['successful']:
            with np.load(case['source_parent']) as z:
                T = z['world_from_camera'][6]
            positive_error = sequence.error(np.array(fwd['fit']['target_to_source']), T)
        rows.append({'id': case['id'], 'same_world': case['same_world'], 'appearance': case['appearance'],
                     'directions': directions, 'native_pass': native, 'geometry_pass': geometry,
                     'geometry_cycle_pass': cyc, 'cycle': cycle, 'positive_pose_error': positive_error})
    groups = {}
    for appearance in ['all', 'clean', 'gamma']:
        selected = [r for r in rows if appearance == 'all' or r['appearance'] == appearance]
        gates = {}
        for gate in ['native_pass', 'geometry_pass', 'geometry_cycle_pass']:
            positive = [r for r in selected if r['same_world']]
            negative = [r for r in selected if not r['same_world']]
            passed = [r for r in selected if r[gate]]
            p = [r['directions'][0]['uncertainty'] for r in passed if r['same_world']]
            n = [r['directions'][0]['uncertainty'] for r in passed if not r['same_world']]
            gates[gate] = {'positive_pass': sum(r[gate] for r in positive), 'positive_total': len(positive),
                           'negative_pass': sum(r[gate] for r in negative), 'negative_total': len(negative),
                           'conditional_uncertainty_auc': auc_lower(p, n), 'conditional_positive_n': len(p), 'conditional_negative_n': len(n)}
        groups[appearance] = {'gates': gates, 'uncertainty_auc': auc_lower([r['directions'][0]['uncertainty'] for r in selected if r['same_world']], [r['directions'][0]['uncertainty'] for r in selected if not r['same_world']])}
    out = Path(plan['score_output'])
    out.mkdir(parents=True, exist_ok=False)
    write(out/'scores.json', {'created_utc': now(), 'records': rows, 'groups': groups,
                             'protocol_sha256': sha(plan['protocol_path']), 'prediction_lock_sha256': sha(folder/'prediction-lock.json'),
                             'scope': 'Constructed synthetic association labels; covariance is a ranking readout, not certified place probability or physical uncertainty calibration.'})


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('mode', choices=['prepare', 'capture', 'score'])
    p.add_argument('--protocol', required=True)
    args = p.parse_args()
    plan = read(args.protocol)
    sequence.verify(plan)
    if args.mode == 'prepare':
        prepare(plan)
    elif args.mode == 'capture':
        sequence.edges = lambda: [(0, 1), (1, 0)]
        sequence.packet = guarded_packet
        sequence.capture(plan, Path(plan['capture_output']))
    else:
        score(plan)
    sequence.verify(plan)
