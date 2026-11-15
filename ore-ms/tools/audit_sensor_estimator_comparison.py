"""Independent shared-support, native-packet, fixed-query and pose-error audit."""
import argparse
import hashlib
from pathlib import Path

import cv2
import numpy as np

from qualify_native_frontend import read, write, sha, now
from audit_paired_frontend import independent_selection, point_covariance, hat


def audit(plan, out):
    checks = 0
    def check(x, text):
        nonlocal checks
        assert x, text
        checks += 1
    def close(a, b, rtol=1e-8, atol=1e-10):
        nonlocal checks
        np.testing.assert_allclose(a, b, rtol=rtol, atol=atol)
        checks += 1
    for path, digest in plan['pins'].items():
        check(sha(path) == digest, path)
    folder = Path(plan['capture_output'])
    for rel, digest in read(folder/'prediction-lock.json')['files'].items():
        check(sha(folder/rel) == digest, 'prediction lock')
    prediction = read(folder/'predictions.json')
    scores = read(Path(plan['score_output'])/'scores.json')
    check(prediction['model_state_unchanged'] and not prediction['true_pose_values_read'], 'provenance')
    check(prediction['loading']['migration'] == 'none', 'strict checkpoint')
    check(len(prediction['rows']) == 72 and len(prediction['dense_rows']) == 36, 'all planned attempts')
    check(len({(r['case'], r['arm'], r['mode']) for r in prediction['rows']}) == 72, 'unique attempts')
    check(scores['prediction_lock_sha256'] == sha(folder/'prediction-lock.json'), 'score binding')
    K = np.array(plan['K'])
    cases = read(plan['inventory'])['records']
    check(len(cases) == 9, 'nine cases')
    successes = 0
    for case in cases:
        check(sha(case['path']) == case['sha256'], 'image lock')
        with np.load(case['path']) as z:
            images = z['images'].astype(np.float32)/255
            raw_mask = z['common_pixel_support']
            poses = z['world_from_left']
            truth = np.linalg.solve(poses[0], poses[1])
            close(truth, z['target_to_source'])
        mask = cv2.erode(raw_mask.astype(np.uint8), np.ones((31, 31), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
        mask[:32] = False
        mask[-32:] = False
        mask[:, :32] = False
        mask[:, -32:] = False
        check(mask.any(), 'nonempty fixed readout/support crop')
        dense = {}
        medians = {}
        proposed = {}
        for a, arm in enumerate(['sharp', 'memory', 'rolling', 'combined']):
            meta = next(r for r in prediction['dense_rows'] if r['case'] == case['id'] and r['arm'] == arm)
            source = np.stack([images[a, 0, 0], images[a, 1, 0], images[a, 0, 0]])
            target = np.stack([images[a, 0, 1], images[a, 1, 1], images[a, 1, 0]])
            check(hashlib.sha256(source.tobytes()).hexdigest() == meta['source_input_sha256'], 'batch source')
            check(hashlib.sha256(target.tobytes()).hexdigest() == meta['target_input_sha256'], 'batch target')
            check(sha(meta['dense_path']) == meta['dense_sha256'], 'dense lock')
            with np.load(meta['dense_path']) as z:
                f, c = z['flow'], z['covariance']
            check(f.shape == c.shape == (3, 2, 256, 640) and np.isfinite(f).all() and np.isfinite(c).all() and (c > 0).all(), 'dense values')
            dense[arm] = (f, c)
            ids = independent_selection(c, plan['selection_seed'])
            check(ids.tolist() == meta['native_proposed_ids'], 'native proposals')
            proposed[arm] = ids
            values = np.sort((c[2, 0]+c[2, 1])[mask])
            middle = len(values)//2
            median = float(values[middle] if len(values)%2 else np.float32(values[middle-1]+values[middle])/np.float32(2))
            close(meta['median_temporal_covariance_trace'], median)
            medians[arm] = median
        def reconstructed(arm, ids):
            f, c = dense[arm]
            ids = np.asarray(ids, int).reshape(-1, 2)
            uv = ids.astype(np.float32)+f[2, :, ids[:, 1], ids[:, 0]]
            inside = (uv[:, 0] > 32)&(uv[:, 0] < 608)&(uv[:, 1] > 32)&(uv[:, 1] < 224)
            xy = np.clip(uv, [0, 0], [639, 255]).astype(int)
            inside &= mask[ids[:, 1], ids[:, 0]]&mask[xy[:, 1], xy[:, 0]]
            u0, u1 = ids[inside], uv[inside]
            if len(u0) < plan['minimum_points']:
                return None
            depths = [np.float32(K[0, 0]*plan['baseline_m'])/np.abs(f[j, 0]) for j in [0, 1]]
            x, y = u0[:, 0], u0[:, 1]
            C0 = point_covariance(depths[0], u0, np.full(len(u0), .25, np.float32), np.full(len(u0), .25, np.float32), K)
            C1 = point_covariance(depths[1], u1, np.maximum(c[2, 0, y, x], .0625), np.maximum(c[2, 1, y, x], .0625), K)
            sane = np.isfinite(C0).all((1, 2))&np.isfinite(C1).all((1, 2))
            if sane.sum() < plan['minimum_points']:
                return None
            fields = {'ids': u0[sane], 'C0': C0[sane], 'C1': C1[sane], 'target_uv': u1[sane]}
            for j, uvj in enumerate([u0, u1]):
                d = depths[j][uvj[:, 1].astype(int), uvj[:, 0].astype(int)].astype(float)
                fields['X'+str(j)] = np.column_stack([(uvj[:, 0].astype(float)-K[0, 2])*d/K[0, 0], (uvj[:, 1].astype(float)-K[1, 2])*d/K[1, 1], d])[sane]
            return fields
        seed_packet = reconstructed('sharp', proposed['sharp'])
        common = np.empty((0, 2), int) if seed_packet is None else seed_packet['ids']
        for arm in ['sharp', 'memory', 'rolling', 'combined']:
            p = reconstructed(arm, common)
            common = np.empty((0, 2), int) if p is None else p['ids']
        stationary_fits = []
        for row in [r for r in prediction['rows'] if r['case'] == case['id']]:
            arm, mode, result = row['arm'], row['mode'], row['fit']
            expected_ids = proposed[arm] if mode == 'regenerated' else common
            check(row['requested_ids'] == expected_ids.tolist(), 'primary/common query identities')
            p = reconstructed(arm, expected_ids)
            scored = next(r for r in scores['rows'] if r['case'] == row['case'] and r['arm'] == arm and r['mode'] == mode)
            check(scored['successful'] == result['successful'] and scored['status'] == result['status'], 'status denominator')
            close(scored['median_temporal_covariance_trace'], medians[arm])
            if p is None:
                check(not result['successful'] and scored['metrics'] is None, 'missing support retained')
                continue
            check(sha(result['packet_path']) == result['packet_sha256'], 'packet lock')
            with np.load(result['packet_path']) as z:
                check(np.array_equal(z['source_ids'], p['ids']), 'admission')
                check(result['accepted_ids'] == p['ids'].tolist() and scored['points'] == len(p['ids']), 'accepted count')
                for k in ['X0', 'X1', 'target_uv']:
                    close(z[k], p[k], rtol=3e-5, atol=1e-8)
                for k in ['C0', 'C1']:
                    close(z[k], p[k], rtol=4e-4, atol=2e-7)
                X0, X1, S0, S1 = [z[k] for k in ['X0', 'X1', 'C0', 'C1']]
            if not result['successful']:
                check(scored['metrics'] is None, 'solver failure retained')
                continue
            successes += 1
            T = np.array(result['target_to_source'])
            R = T[:3, :3]
            close(R.T@R, np.eye(3))
            q = X1@R.T+T[:3, 3]
            H, g = np.zeros((6, 6)), np.zeros(6)
            for v, residual, C in zip(q, q-X0, S0+R@S1@R.T):
                J = np.concatenate([np.eye(3), -hat(v)], axis=1)
                H += J.T@np.linalg.solve(C, J)
                g += J.T@np.linalg.solve(C, residual)
            check(np.sqrt(max(g@np.linalg.solve(H, g), 0)) <= plan['solver']['score_tolerance']*1.01, 'stationarity')
            P = np.linalg.inv(H)
            close(result['information_covariance'], P, rtol=1e-7)
            metrics = scored['metrics']
            close(metrics['translation_m'], np.linalg.norm(T[:3, 3]-truth[:3, 3]))
            delta = truth[:3, :3].T@R
            sine = .5*np.linalg.norm([delta[2, 1]-delta[1, 2], delta[0, 2]-delta[2, 0], delta[1, 0]-delta[0, 1]])
            angle = np.rad2deg(np.arctan2(sine, (np.trace(delta)-1)/2))
            close(metrics['rotation_deg'], angle)
            close(metrics['translation_information_std_proxy_m'], np.sqrt(np.trace(P[:3, :3])))
            close(metrics['rotation_information_std_proxy_deg'], np.rad2deg(np.sqrt(np.trace(P[3:, 3:]))))
            if case['motion'] == 'stationary':
                stationary_fits.append(result)
        if case['motion'] == 'stationary':
            baseline = dense['sharp']
            for arm in ['memory', 'rolling', 'combined']:
                check(all(np.array_equal(x, y) for x, y in zip(baseline, dense[arm])), 'stationary dense null')
            if stationary_fits:
                for r in stationary_fits[1:]:
                    check(r['target_to_source'] == stationary_fits[0]['target_to_source'] and r['information_covariance'] == stationary_fits[0]['information_covariance'], 'stationary estimator null')
    check(len(scores['rows']) == 72 and scores['planned_dense_batches'] == 36 and scores['planned_pose_attempts'] == 72, 'complete scores')
    write(out, {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'planned_dense_batches': 36,
                'planned_pose_attempts': 72, 'successful_pose_fits': successes,
                'scope': 'Independent support/selection/packet/stationarity/common-query/null and center-reference pose scoring; no physical probability calibration.'})
    print('PASS', checks, 'checks;', successes, '/72 planned fits converge', flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--protocol', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    audit(read(args.protocol), Path(args.output))
