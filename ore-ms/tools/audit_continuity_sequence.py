"""Independent camera-ray, packet, pose-chain and interruption-semantics checks."""
import argparse
import hashlib
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from qualify_native_frontend import read, write, sha, now
from audit_paired_frontend import independent_selection, point_covariance, hat


def audit(plan, phase, output):
    checks = 0
    def check(x, message):
        nonlocal checks
        assert x, message
        checks += 1
    def close(a, b, rtol=1e-9, atol=1e-10):
        nonlocal checks
        np.testing.assert_allclose(a, b, rtol=rtol, atol=atol)
        checks += 1
    for path, digest in plan['pins'].items():
        check(sha(path) == digest, path)
    inventory = read(plan['inventory'])
    check(len(inventory['records']) == 6, 'six sequences')
    K = np.array(plan['K'])
    grid = np.array([(u, v) for u in [48, 121, 243, 359, 477, 589] for v in [45, 91, 149, 207]])
    truths = {}
    image_inputs = {}
    for scene in inventory['records']:
        check(sha(scene['path']) == scene['sha256'], 'render hash')
        with np.load(scene['path']) as z:
            images, support, poses, texture = [z[k] for k in ['images', 'texture_valid', 'world_from_camera', 'texture']]
        truths[scene['id']] = poses
        image_inputs[scene['id']] = images.astype(np.float32)/255
        check(images.shape == (10, 2, 256, 640) and images.dtype == np.uint8, 'stereo sequence schema')
        check(support.shape == images.shape and support.dtype == bool, 'support schema')
        for i in range(10):
            Q, _ = cv2.Rodrigues(np.array([0., np.deg2rad(i*plan['yaw_per_frame_deg']), 0.]))
            center = i*np.array(plan['center_increment_m'])
            close(poses[i, :3, :3], Q)
            close(poses[i, :3, 3], center)
            close(poses[i, 3], [0, 0, 0, 1])
            for side in [0, 1]:
                origin = center+Q@np.array([side*plan['baseline_m'], 0., 0.])
                rays = np.column_stack([grid, np.ones(len(grid))])@np.linalg.inv(K).T@Q.T
                for (u, v), ray in zip(grid, rays):
                    hits = []
                    for n, depth in enumerate(scene['scene']['depths_m']):
                        distance = (depth-origin[2])/ray[2]
                        point = origin+distance*ray
                        if distance > 0 and (not scene['scene']['near_halfplane'] or n != 0 or point[0] <= 0):
                            hits.append((distance, point))
                    check(bool(hits), 'ray intersects a plane')
                    world = min(hits, key=lambda x: x[0])[1]
                    uv = K@world
                    uv = uv[:2]/uv[2]
                    valid = 0 <= uv[0] <= 639 and 0 <= uv[1] <= 255
                    check(bool(support[i, side, v, u]) == valid, 'independent atlas support')
                    if valid:
                        x, y = np.floor(uv).astype(int)
                        x1, y1 = min(x+1, 639), min(y+1, 255)
                        ax, ay = uv-[x, y]
                        expected = ((1-ay)*((1-ax)*float(texture[y, x])+ax*float(texture[y, x1]))+
                                    ay*((1-ax)*float(texture[y1, x])+ax*float(texture[y1, x1])))
                        check(abs(float(images[i, side, v, u])-expected) <= .50000001, 'independent bilinear quantization')
                    local = (world-origin)@Q
                    projected = K@local
                    close(projected[:2]/projected[2], [u, v])
        # Gauge ambiguity: a rigid transform on a disconnected post-gap map
        # changes its world placement but leaves every within-segment edge equal.
        G = np.eye(4)
        G[:3, :3] = Rotation.from_rotvec([.1, -.2, .05]).as_matrix()
        G[:3, 3] = [1., -2., .5]
        for i in [6, 7, 8]:
            close(np.linalg.inv(G@poses[i])@(G@poses[i+1]), np.linalg.inv(poses[i])@poses[i+1])
        check(not np.allclose(G@poses[6], poses[6]), 'different global placements')
        # Test the producer's chain mechanics using perfect measurement fixtures.
        from continuity_sequence import assemble
        exact = {(a, b): np.linalg.inv(poses[a])@poses[b] for a, b in [(j, j+1) for j in range(9)]+[(2, 6), (0, 6)]}
        arms = assemble(exact)
        for i in [6, 7, 8, 9]:
            for name in ['uninterrupted', 'retained_gap', 'visual_anchor']:
                close(arms[name][i], poses[i])
            close(arms['local_restart'][i], np.linalg.inv(poses[6])@poses[i])
        check(np.linalg.norm(arms['hold_pose_splice'][6][:3, 3]-poses[6, :3, 3]) > .1, 'zero-motion splice loses gap displacement')
        exact[(2, 6)] = None
        missing = assemble(exact)
        check(all(missing['retained_gap'][i] is None for i in [6, 7, 8, 9]), 'failed bridge remains unavailable')
        check(all(missing['visual_anchor'][i] is not None for i in [6, 7, 8, 9]), 'independent anchor recovery path')
    if phase == 'render':
        write(output, {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'sequences': 6, 'views': 120,
                       'scope': 'Independent ray/sample, perfect-edge chain and disconnected gauge checks; no learned predictions.'})
        print('PASS', checks, 'pre-inference checks', flush=True)
        return
    captured = Path(plan['capture_output'])
    for path, digest in read(captured/'prediction-lock.json')['files'].items():
        check(sha(captured/path) == digest, 'prediction lock')
    pred = read(captured/'predictions.json')
    check(pred['model_state_unchanged'] and not pred['true_pose_read'], 'prediction provenance')
    check(pred['loading']['migration'] == 'none', 'strict checkpoint')
    check(len(pred['rows']) == 72, '72 planned fits including6nulls')
    expected_edges = {(j, j+1) for j in range(9)}|{(2, 6), (0, 6), (0, 0)}
    results = read(Path(plan['score_output'])/'scores.json')
    successes = 0
    for scene in inventory['records']:
        key = scene['id']
        rows = [r for r in pred['rows'] if r['scene'] == key]
        check(len(rows) == 12 and {(r['source'], r['target']) for r in rows} == expected_edges, 'fixed edge population')
        transforms = {}
        for row in rows:
            a, b = row['source'], row['target']
            im = image_inputs[key]
            s = np.stack([im[a, 0], im[b, 0], im[a, 0]])
            t = np.stack([im[a, 1], im[b, 1], im[b, 0]])
            check(hashlib.sha256(s.tobytes()).hexdigest() == row['source_input_sha256'], 'batch source')
            check(hashlib.sha256(t.tobytes()).hexdigest() == row['target_input_sha256'], 'batch target')
            check(sha(row['dense_path']) == row['dense_sha256'], 'dense identity')
            with np.load(row['dense_path']) as z:
                f, c = z['flow'], z['covariance']
            check(f.shape == c.shape == (3, 2, 256, 640) and np.isfinite(f).all() and np.isfinite(c).all() and (c > 0).all(), 'dense values')
            ids = independent_selection(c, plan['selection_seed'])
            check(ids.tolist() == row['proposed_ids'], 'independent native selection')
            result = row['fit']
            transforms[(a, b)] = np.array(result['target_to_source']) if result['successful'] else None
            if result['status'] == 'INVALID_PACKET':
                check(not result['successful'], 'invalid retained')
                continue
            check(sha(result['packet_path']) == result['packet_sha256'], 'packet identity')
            target = ids.astype(np.float32)+f[2, :, ids[:, 1], ids[:, 0]]
            good = (target[:, 0] > 32)&(target[:, 0] < 608)&(target[:, 1] > 32)&(target[:, 1] < 224)
            u0, u1 = ids[good], target[good]
            depths = [np.float32(K[0, 0]*plan['baseline_m'])/np.abs(f[j, 0]) for j in [0, 1]]
            aa, bb = u0[:, 0], u0[:, 1]
            C0 = point_covariance(depths[0], u0, np.full(len(u0), .25, np.float32), np.full(len(u0), .25, np.float32), K)
            C1 = point_covariance(depths[1], u1, np.maximum(c[2, 0, bb, aa], .0625), np.maximum(c[2, 1, bb, aa], .0625), K)
            finite = np.isfinite(C0).all((1, 2))&np.isfinite(C1).all((1, 2))
            with np.load(result['packet_path']) as p:
                check(np.array_equal(p['source_ids'], u0[finite]), 'packet admission')
                close(p['C0'], C0[finite], rtol=4e-4, atol=2e-7)
                close(p['C1'], C1[finite], rtol=4e-4, atol=2e-7)
                for side, uv in enumerate([u0, u1]):
                    d = depths[side][uv[:, 1].astype(int), uv[:, 0].astype(int)].astype(float)
                    expected = np.column_stack([(uv[:, 0].astype(float)-K[0, 2])*d/K[0, 0], (uv[:, 1].astype(float)-K[1, 2])*d/K[1, 1], d])
                    close(p['X'+str(side)], expected[finite], rtol=3e-5, atol=1e-8)
                X0, X1, S0, S1 = [p[n] for n in ['X0', 'X1', 'C0', 'C1']]
            T = np.array(result['target_to_source'])
            close(T[:3, :3].T@T[:3, :3], np.eye(3))
            if result['successful']:
                R, t = T[:3, :3], T[:3, 3]
                q = X1@R.T+t
                H, g = np.zeros((6, 6)), np.zeros(6)
                for point, residual, cov in zip(q, q-X0, S0+R@S1@R.T):
                    J = np.concatenate([np.eye(3), -hat(point)], axis=1)
                    H += J.T@np.linalg.solve(cov, J)
                    g += J.T@np.linalg.solve(cov, residual)
                check(float(np.sqrt(max(g@np.linalg.solve(H, g), 0))) <= plan['solver']['score_tolerance']*1.01, 'independent stationarity')
                close(result['information_covariance'], np.linalg.inv(H), rtol=1e-7, atol=1e-10)
                successes += 1
        summary = next(r for r in results['records'] if r['scene'] == key)
        for name, arm in summary['arms'].items():
            if name == 'local_restart':
                check(arm['coordinate_frame'] == 'restart_at6_unconnected_to_world', 'restart declared disconnected')
            check(arm['planned_postgap'] == 4 and not arm['global_recovery_claim'], 'denominator/claim')
            vals = []
            for row in arm['rows']:
                i = row['frame']
                route = list(range(i+1)) if name == 'uninterrupted' else ([0, 1, 2, 6]+list(range(7, i+1)))
                if name in ['local_restart', 'hold_pose_splice']:
                    route = list(range(6, i+1))
                if name == 'visual_anchor':
                    route = [0, 6]+list(range(7, i+1))
                expected = np.eye(4)
                for a, b in zip(route[:-1], route[1:]):
                    T = transforms[(a, b)]
                    expected = None if expected is None or T is None else expected@T
                if name == 'hold_pose_splice':
                    for T in [transforms[(1, 2)], transforms[(0, 1)]]:
                        expected = None if expected is None or T is None else T@expected
                check(row['available'] == (expected is not None), 'availability and failed chain')
                if expected is None:
                    check(row['error'] is None and row['pose'] is None, 'missing not filled')
                    continue
                close(row['pose'], expected)
                truth = truths[key][i]
                if name == 'local_restart':
                    truth = np.linalg.solve(truths[key][6], truth)
                dt = expected[:3, 3]-truth[:3, 3]
                angle = np.rad2deg(np.linalg.norm(Rotation.from_matrix(truth[:3, :3].T@expected[:3, :3]).as_rotvec()))
                close(row['error']['translation_m'], np.linalg.norm(dt))
                close(row['error']['rotation_deg'], angle)
                vals.append([np.linalg.norm(dt), angle])
            check(arm['available_postgap'] == len(vals), 'available denominator')
            if vals:
                close(arm['translation_rmse_m'], np.sqrt(np.mean(np.array(vals)[:, 0]**2)))
                close(arm['rotation_rmse_deg'], np.sqrt(np.mean(np.array(vals)[:, 1]**2)))
    write(output, {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'fit_attempts': 72,
                   'successful_fits': successes, 'scope': 'Independent render/packets/stationarity/coordinate-aware chain and score audit.'})
    print('PASS', checks, 'checks;', successes, '/72 fits converge', flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('phase', choices=['render', 'results'])
    p.add_argument('--protocol', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    audit(read(args.protocol), args.phase, Path(args.output))
