"""Independent labels, native packets, geometric hypotheses, cycles and AUC."""
import argparse
import hashlib
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from qualify_native_frontend import read, write, sha, now
from audit_paired_frontend import independent_selection, point_covariance, hat


def audit(plan, phase, output):
    checks = 0
    def check(x, text):
        nonlocal checks
        assert x, text
        checks += 1
    def close(a, b, rtol=1e-8, atol=1e-10):
        nonlocal checks
        np.testing.assert_allclose(a, b, rtol=rtol, atol=atol)
        checks += 1
    for p, h in plan['pins'].items():
        check(sha(p) == h, p)
    records = read(plan['inventory'])['records']
    check(len(records) == 36 and len({r['id'] for r in records}) == 36, 'unique fixed population')
    lut = np.array([round(255*(i/255)**plan['gamma']) for i in range(256)], np.uint8)
    image_arrays = {}
    for row in records:
        check(sha(row['path']) == row['sha256'], 'input lock')
        check(row['same_world'] == (row['source_world'] == row['target_world']), 'world label')
        with np.load(row['source_parent']) as a, np.load(row['target_parent']) as b, np.load(row['path']) as z:
            images = z['images']
            check(images.shape == (2, 2, 256, 640) and images.dtype == np.uint8, 'input schema')
            check(np.array_equal(images[0], a['images'][0]), 'source identity')
            expected = b['images'][6] if row['appearance'] == 'clean' else lut[b['images'][6]]
            check(np.array_equal(images[1], expected), 'clean/gamma stereo target')
            if row['same_world']:
                T = np.linalg.solve(a['world_from_camera'][0], a['world_from_camera'][6])
                close(T@np.linalg.inv(T), np.eye(4))
        image_arrays[row['id']] = images.astype(np.float32)/255
    for appearance in ['clean', 'gamma']:
        group = [r for r in records if r['appearance'] == appearance]
        check(len(group) == 18 and sum(r['same_world'] for r in group) == 6, 'balanced appearance classes')
    # Meaningful baseline mechanics fixtures, before model execution.
    from anchor_validity import rigid, geometric, auc_lower, guarded_packet
    rng = np.random.default_rng(405)
    X1 = rng.normal(size=(30, 3))
    R = Rotation.from_rotvec([.2, -.1, .3]).as_matrix()
    t = np.array([.15, -.04, .2])
    X0 = X1@R.T+t
    T = rigid(X0, X1)
    close(T[:3, :3], R)
    close(T[:3, 3], t)
    check(rigid(np.zeros((3, 3)), np.zeros((3, 3))) is None, 'degenerate rejection')
    check(geometric({'X0': X0[:2], 'X1': X1[:2]}, plan['ransac'])['pose'] is None, 'insufficient geometry')
    perfect = geometric({'X0': X0, 'X1': X1}, plan['ransac'])
    check(perfect['inliers'] == 30, 'perfect geometry')
    corrupted = X0.copy()
    corrupted[-10:] += [2., -1., .7]
    robust = geometric({'X0': corrupted, 'X1': X1}, plan['ransac'])
    check(robust['inliers'] == 20, 'known gross outlier recovery')
    close(robust['pose'], T)
    # These same means/transforms could be a false association between disjoint
    # scene instances. Perfect geometry and a closed cycle cannot prove identity.
    close(T@np.linalg.inv(T), np.eye(4))
    check(auc_lower([1., 2.], [3., 4.]) == 1., 'AUC direction')
    check(auc_lower([1.], [1.]) == .5, 'AUC ties')
    check(auc_lower([], [1.]) is None, 'undefined conditional AUC')
    try:
        guarded_packet(np.zeros((3, 2, 256, 640), np.float32), None, None, np.zeros((0, 2), int), None)
    except ValueError:
        check(True, 'empty admission retained')
    else:
        check(False, 'empty admission must reject')
    if phase == 'prepare':
        write(output, {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'candidates': 36,
                       'positive': 12, 'negative': 24, 'scope': 'Constructed labels, images and baseline mechanics; no predictions.'})
        print('PASS', checks, 'pre-inference contract/mechanics checks', flush=True)
        return
    folder = Path(plan['capture_output'])
    for rel, h in read(folder/'prediction-lock.json')['files'].items():
        check(sha(folder/rel) == h, 'prediction lock')
    prediction = read(folder/'predictions.json')
    check(len(prediction['rows']) == 72 and prediction['model_state_unchanged'] and not prediction['true_pose_read'], 'population/provenance')
    check(prediction['loading']['migration'] == 'none', 'checkpoint')
    scores = read(Path(plan['score_output'])/'scores.json')
    check(scores['prediction_lock_sha256'] == sha(folder/'prediction-lock.json'), 'score binding')
    K = np.array(plan['K'])
    success = 0
    for case in records:
        rows = [r for r in prediction['rows'] if r['scene'] == case['id']]
        check(len(rows) == 2 and {(r['source'], r['target']) for r in rows} == {(0, 1), (1, 0)}, 'two directions')
        result = next(r for r in scores['records'] if r['id'] == case['id'])
        check(result['same_world'] == case['same_world'] and result['appearance'] == case['appearance'], 'score label')
        geometry_passes, native_passes, poses = [], [], []
        for row in sorted(rows, key=lambda r: r['source']):
            a, b = row['source'], row['target']
            images = image_arrays[case['id']]
            for array, key in [(np.stack([images[a, 0], images[b, 0], images[a, 0]]), 'source_input_sha256'), (np.stack([images[a, 1], images[b, 1], images[b, 0]]), 'target_input_sha256')]:
                check(hashlib.sha256(array.tobytes()).hexdigest() == row[key], 'batch identity')
            check(sha(row['dense_path']) == row['dense_sha256'], 'dense identity')
            with np.load(row['dense_path']) as z:
                f, c = z['flow'], z['covariance']
            check(f.shape == c.shape == (3, 2, 256, 640) and np.isfinite(f).all() and np.isfinite(c).all() and (c > 0).all(), 'dense schema')
            ids = independent_selection(c, plan['selection_seed'])
            check(ids.tolist() == row['proposed_ids'], 'native selection')
            entry = result['directions'][a]
            # Explicit order-statistic median independent of np.median.
            values = sorted((c[2, 0, 32:-32, 32:-32]+c[2, 1, 32:-32, 32:-32]).ravel().tolist())
            middle = len(values)//2
            # Native np.median averages in float32; mirror the declared dtype.
            expected_cov = float(np.float32(np.float32(values[middle-1])+np.float32(values[middle]))/np.float32(2))
            close(entry['uncertainty'], expected_cov)
            fit = row['fit']
            native = fit['successful'] and fit['points'] >= plan['gates']['min_points']
            check(entry['native_pass'] == native, 'native gate')
            native_passes.append(native)
            if fit['status'] == 'INVALID_PACKET':
                check(not fit['successful'], 'invalid retained')
                geometry_passes.append(False)
                poses.append(None)
                continue
            check(sha(fit['packet_path']) == fit['packet_sha256'], 'packet identity')
            uv1 = ids.astype(np.float32)+f[2, :, ids[:, 1], ids[:, 0]]
            inside = (uv1[:, 0] > 32)&(uv1[:, 0] < 608)&(uv1[:, 1] > 32)&(uv1[:, 1] < 224)
            u0, u1 = ids[inside], uv1[inside]
            depths = [np.float32(K[0, 0]*plan['baseline_m'])/np.abs(f[j, 0]) for j in [0, 1]]
            x, y = u0[:, 0], u0[:, 1]
            C0 = point_covariance(depths[0], u0, np.full(len(u0), .25, np.float32), np.full(len(u0), .25, np.float32), K)
            C1 = point_covariance(depths[1], u1, np.maximum(c[2, 0, y, x], .0625), np.maximum(c[2, 1, y, x], .0625), K)
            mask = np.isfinite(C0).all((1, 2))&np.isfinite(C1).all((1, 2))
            with np.load(fit['packet_path']) as p:
                check(np.array_equal(p['source_ids'], u0[mask]), 'admission')
                close(p['C0'], C0[mask], rtol=4e-4, atol=2e-7)
                close(p['C1'], C1[mask], rtol=4e-4, atol=2e-7)
                for j, uv in enumerate([u0, u1]):
                    d = depths[j][uv[:, 1].astype(int), uv[:, 0].astype(int)].astype(float)
                    back = np.column_stack([(uv[:, 0].astype(float)-K[0, 2])*d/K[0, 0], (uv[:, 1].astype(float)-K[1, 2])*d/K[1, 1], d])
                    close(p['X'+str(j)], back[mask], rtol=3e-5, atol=1e-8)
                X0, X1, S0, S1 = [p[k] for k in ['X0', 'X1', 'C0', 'C1']]
            Tfit = np.array(fit['target_to_source'])
            if fit['successful']:
                R = Tfit[:3, :3]
                q = X1@R.T+Tfit[:3, 3]
                H, g = np.zeros((6, 6)), np.zeros(6)
                for v, residual, cov in zip(q, q-X0, S0+R@S1@R.T):
                    J = np.concatenate([np.eye(3), -hat(v)], axis=1)
                    H += J.T@np.linalg.solve(cov, J)
                    g += J.T@np.linalg.solve(cov, residual)
                check(np.sqrt(max(g@np.linalg.solve(H, g), 0)) <= plan['solver']['score_tolerance']*1.01, 'stationarity')
                close(fit['information_covariance'], np.linalg.inv(H), rtol=1e-7)
                success += 1
            geom = entry['geometry']
            sample_rng = np.random.default_rng(plan['ransac']['seed'])
            samples = [sample_rng.choice(len(X0), 3, replace=False).tolist() for _ in range(plan['ransac']['hypotheses'])]
            check(samples == geom['hypotheses'], 'hypothesis seed/budget')
            # Independent scipy alignment, not the producer's SVD routine.
            best_key, best_pose, index = None, None, None
            for j, ids3 in enumerate(samples):
                a0, a1 = X0[ids3], X1[ids3]
                aa, bb = a0-a0.mean(0), a1-a1.mean(0)
                singular = np.linalg.svd(bb.T@aa, compute_uv=False)
                if singular[1] <= 1e-8*singular[0]:
                    continue
                R, _ = Rotation.align_vectors(aa, bb)
                R = R.as_matrix()
                t = a0.mean(0)-R@a1.mean(0)
                squared = np.sum((X1@R.T+t-X0)**2, 1)
                count = int(np.sum(squared <= plan['ransac']['residual_m']**2))
                cost = float(np.minimum(squared, plan['ransac']['residual_m']**2).sum())
                key = (count, -cost)
                if best_key is None or key > best_key:
                    best_key, index = key, j
                    best_pose = np.eye(4)
                    best_pose[:3, :3], best_pose[:3, 3] = R, t
            check(geom['inliers'] == (0 if best_key is None else best_key[0]), 'independent RANSAC inliers')
            close(geom['inlier_fraction'], 0. if best_key is None else best_key[0]/len(X0))
            if best_pose is not None:
                # Near-equal optimal hypotheses may differ at floating roundoff;
                # verify the bound chosen hypothesis independently as well.
                ids3 = samples[geom['best_index']]
                a0, a1 = X0[ids3], X1[ids3]
                Q, _ = Rotation.align_vectors(a0-a0.mean(0), a1-a1.mean(0))
                expected = np.eye(4)
                expected[:3, :3], expected[:3, 3] = Q.as_matrix(), a0.mean(0)-Q.apply(a1.mean(0))
                close(geom['pose'], expected, rtol=1e-7, atol=1e-8)
                squared = np.sum((X1@expected[:3, :3].T+expected[:3, 3]-X0)**2, axis=1)
                check(int(np.sum(squared <= plan['ransac']['residual_m']**2)) == geom['inliers'], 'chosen hypothesis consensus')
                close(float(np.minimum(squared, plan['ransac']['residual_m']**2).sum()), -best_key[1], atol=1e-10)
                poses.append(expected)
            else:
                check(geom['pose'] is None, 'no geometric pose')
                poses.append(None)
            geometry_passes.append(geom['inlier_fraction'] >= plan['gates']['inlier_fraction'] and geom['inliers'] >= plan['gates']['min_points'])
        native = all(native_passes)
        geometric_gate = native and all(geometry_passes)
        check(result['native_pass'] == native and result['geometry_pass'] == geometric_gate, 'combined gates')
        cyc = False
        if all(T is not None for T in poses):
            closure = poses[0]@poses[1]
            trans = np.linalg.norm(closure[:3, 3])
            angle = np.rad2deg(Rotation.from_matrix(closure[:3, :3]).magnitude())
            close(result['cycle']['translation_m'], trans, atol=1e-8)
            close(result['cycle']['rotation_deg'], angle, atol=1e-7)
            cyc = geometric_gate and trans <= plan['gates']['cycle_translation_m'] and angle <= plan['gates']['cycle_rotation_deg']
        check(result['geometry_cycle_pass'] == cyc, 'cycle gate')
        if case['same_world'] and rows[0]['fit']['successful']:
            fwd = next(r for r in rows if r['source'] == 0)
            with np.load(case['source_parent']) as z:
                truth = z['world_from_camera'][6]
            T = np.array(fwd['fit']['target_to_source'])
            close(result['positive_pose_error']['translation_m'], np.linalg.norm(T[:3, 3]-truth[:3, 3]))
            close(result['positive_pose_error']['rotation_deg'], np.rad2deg(Rotation.from_matrix(truth[:3, :3].T@T[:3, :3]).magnitude()))
        if not case['same_world']:
            check(result['positive_pose_error'] is None, 'no false-map pose truth')
    # Rebuild every numerator/denominator and pairwise ranking without producer helpers.
    def independent_auc(rows):
        p = [r['directions'][0]['uncertainty'] for r in rows if r['same_world']]
        n = [r['directions'][0]['uncertainty'] for r in rows if not r['same_world']]
        if not p or not n:
            return None
        return sum(1. if a < b else .5 if a == b else 0. for a in p for b in n)/(len(p)*len(n))
    for group_name, group in scores['groups'].items():
        rows = [r for r in scores['records'] if group_name == 'all' or r['appearance'] == group_name]
        close(group['uncertainty_auc'], independent_auc(rows))
        for name, counts in group['gates'].items():
            pos, neg = [r for r in rows if r['same_world']], [r for r in rows if not r['same_world']]
            check(counts['positive_total'] == len(pos) and counts['negative_total'] == len(neg), 'all attempted denominators')
            check(counts['positive_pass'] == sum(r[name] for r in pos) and counts['negative_pass'] == sum(r[name] for r in neg), 'gate numerator')
            accepted = [r for r in rows if r[name]]
            auc = independent_auc(accepted)
            check(counts['conditional_positive_n'] == sum(r['same_world'] for r in accepted) and counts['conditional_negative_n'] == sum(not r['same_world'] for r in accepted), 'conditional denominator')
            if auc is None:
                check(counts['conditional_uncertainty_auc'] is None, 'undefined AUC retained')
            else:
                close(counts['conditional_uncertainty_auc'], auc)
    write(output, {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'planned_directions': 72,
                   'successful_fits': success, 'candidates': 36, 'scope': 'Independent input, packet, fit, geometric-control and score audit.'})
    print('PASS', checks, 'checks;', success, '/72 fits converge', flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('phase', choices=['prepare', 'results'])
    p.add_argument('--protocol', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    audit(read(args.protocol), args.phase, Path(args.output))
