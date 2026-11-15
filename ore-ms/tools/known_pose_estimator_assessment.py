"""Fixed versus regenerated queries on exact-pose thermal-texture scenes."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.stats import chi2

from paired_frontend_assessment import native_algorithms, pick, packet, evaluate_dense, transport_cov
from qualify_native_frontend import deterministic, load_model, now, read, sha, write


def verify(plan):
    for path, digest in plan['pins'].items():
        assert sha(path) == digest, path


def batch_views(images):
    return np.stack([images[0, 0], images[1, 0], images[0, 0]]), np.stack([images[0, 1], images[1, 1], images[1, 0]])


def qualify(renderer_plan, inventory, out):
    import cv2
    deterministic()
    selector, covariance = native_algorithms()
    camera = {'K': renderer_plan['K'], 'baseline_m': renderer_plan['baseline_m']}
    checks, records = 0, []
    for row in inventory['records']:
        assert sha(row['path']) == row['sha256']
        with np.load(row['path']) as scene:
            raw, world = scene['images'], scene['world']
            assert raw.shape == (4, 256, 640) and raw.dtype == np.uint8
            images = raw.reshape(2, 2, 256, 640).astype(np.float32)/255
            assert images.dtype == np.float32 and np.isfinite(images).all() and images.min() >= 0 and images.max() <= 1
            first, second = batch_views(images)
            assert all(np.array_equal(first[i], images[a, b]) for i, (a, b) in enumerate([(0, 0), (1, 0), (0, 0)]))
            assert all(np.array_equal(second[i], images[a, b]) for i, (a, b) in enumerate([(0, 1), (1, 1), (1, 0)]))
            preview = cv2.imread(str(Path(row['path']).with_suffix('.png')), cv2.IMREAD_GRAYSCALE)
            assert np.array_equal(preview, np.concatenate([np.concatenate(raw[:2], 1), np.concatenate(raw[2:], 1)], 0))
            K, T = scene['K'], scene['target_to_source']
            target = (world[0]-T[:3, 3])@T[:3, :3]
            np.testing.assert_allclose(target@T[:3, :3].T+T[:3, 3], world[0], rtol=1e-10, atol=1e-10)
            z0 = world[0, ..., 2]
            z1 = ((world[2]-scene['centers'][2])@scene['rotations'][2])[..., 2]
            f = np.zeros((3, 2, 256, 640), np.float32)
            f[0, 0] = -K[0, 0]*camera['baseline_m']/z0
            f[1, 0] = -K[0, 0]*camera['baseline_m']/z1
            uv = target@K.T
            uv = uv[..., :2]/uv[..., 2:]
            v, u = np.indices((256, 640))
            f[2] = (uv-np.stack([u, v], -1)).transpose(2, 0, 1)
            c = np.full_like(f, .25)
            ids = pick(f, c, camera, selector, 20261114)
            p = packet(f, c, camera, ids, covariance)
            ids2 = p['source_ids']
            np.testing.assert_allclose(p['X0'], world[0, ids2[:, 1], ids2[:, 0]], rtol=3e-5, atol=1e-8)
            assert len(ids2) >= 3
            np.savez_compressed(out/(row['id']+'-analytic-interface.npz'), **p)
            checks += 9
            records.append({'id': row['id'], 'source_sha256': row['sha256'], 'analytic_selected': len(ids), 'analytic_accepted': len(ids2)})
    write(out/'interface-qualification.json', {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'records': records, 'scope': 'Rendered grayscale/batch ordering/PNG and analytic-field packet/coordinate compatibility. No learned model predictions or estimator pose-accuracy scores.'})
    print('PASS', checks, 'interface checks', flush=True)


def capture(plan, out):
    import torch
    deterministic()
    model, loading = load_model(plan['checkpoint'])
    assert loading['migration'] == 'none'
    selector, covariance = native_algorithms()
    state = lambda: {k: hashlib.sha256(v.cpu().numpy().tobytes()).hexdigest() for k, v in model.state_dict().items()}
    initial = state()
    rows = []
    def inference(images):
        first, second = batch_views(images)
        a = torch.from_numpy(np.repeat(first[:, None], 3, axis=1)).cuda()
        b = torch.from_numpy(np.repeat(second[:, None], 3, axis=1)).cuda()
        with torch.inference_mode():
            flow, cov = model.inference(a, b)
        assert torch.isfinite(flow).all() and torch.isfinite(cov).all() and (cov > 0).all()
        return flow.cpu().numpy(), cov.cpu().numpy()
    for scene in plan['scenes']:
        key = scene['id']
        # Only rendered input images are loaded here. True pose is read at scoring.
        with np.load(scene['path']) as z:
            images = z['images'].reshape(2, 2, 256, 640).astype(np.float32)/255
        camera = {'K': plan['K'], 'baseline_m': plan['baseline_m']}
        baseline_flow, baseline_cov = inference(images)
        original_ids = pick(baseline_flow, baseline_cov, camera, selector, plan['selection_seed'])
        for trial in [-2, -1]+list(range(plan['trials'])):
            seed = None
            if trial == -2:
                flow, cov = baseline_flow, baseline_cov
                noise = np.zeros_like(images)
            elif trial == -1:
                flow, cov = inference(images)
                noise = np.zeros_like(images)
            else:
                seed = int.from_bytes(hashlib.sha256(f"{plan['noise_seed']}:{key}:{trial}".encode()).digest()[:8], 'little')
                noise = np.random.default_rng(seed).uniform(-plan['half_lsb'], plan['half_lsb'], images.shape).astype(np.float32)
                flow, cov = inference(np.clip(images+noise, 0, 1))
            dense = out/'dense'/f'{key}-{trial+2:03}.npz'
            np.savez_compressed(dense, flow=flow, covariance=cov)
            folder = out/'packets'/f'{key}-{trial+2:03}'
            folder.mkdir()
            arms = evaluate_dense(flow, cov, camera, original_ids, selector, covariance, plan['selection_seed'], plan['solver'], folder)
            entry = {'condition': key, 'trial': trial, 'noise_seed': seed, 'noise_sha256': hashlib.sha256(noise.tobytes()).hexdigest(), 'dense_path': str(dense), 'dense_sha256': sha(dense), 'arms': arms}
            rows.append(entry)
            with (out/'journal.jsonl').open('a') as stream:
                stream.write(json.dumps(entry, allow_nan=False)+'\n')
            if trial == -1:
                assert np.array_equal(flow, baseline_flow) and np.array_equal(cov, baseline_cov)
                original = rows[-2]['arms']
                for arm in arms:
                    for key_name in ['status', 'target_to_source', 'information_covariance']:
                        assert arms[arm].get(key_name) == original[arm].get(key_name), key_name
            if trial < 0 or trial % 32 == 31:
                print(key, trial, {k: v['status'] for k, v in arms.items()}, flush=True)
    assert initial == state()
    write(out/'predictions.json', {'created_utc': now(), 'rows': rows, 'loading': loading, 'model_state_unchanged': True, 'true_pose_read_for_scoring': False})
    write(out/'prediction-lock.json', {'created_utc': now(), 'rows': len(rows), 'fit_attempts': len(rows)*2, 'files': {str(p.relative_to(out)): sha(p) for p in out.rglob('*') if p.is_file()}})


def score(plan, out):
    for rel, digest in read(out/'prediction-lock.json')['files'].items():
        assert sha(out/rel) == digest, rel
    rows = read(out/'predictions.json')['rows']
    reports, arrays = {}, {}
    for scene in plan['scenes']:
        key = scene['id']
        with np.load(scene['path']) as z:
            truth = z['target_to_source'].copy()
        selected = [r for r in rows if r['condition'] == key]
        base = next(r for r in selected if r['trial'] == -2)
        trials = [r for r in selected if r['trial'] >= 0]
        paired = [r for r in trials if all(r['arms'][a]['successful'] for a in ['fixed_ids', 'regenerated'])]
        base_fit = base['arms']['fixed_ids']
        L = np.linalg.cholesky(base_fit['information_covariance']) if base_fit['successful'] else None
        def summarize(records, arm, label):
            if not records:
                return {'valid_trials': 0, 'metrics': None}
            values = [transport_cov(np.array(r['arms'][arm]['target_to_source']), truth, np.array(r['arms'][arm]['information_covariance'])) for r in records]
            X = np.array([x for x, p in values])
            P = np.array([p for x, p in values])
            mean = X.mean(0)
            C = np.cov(X, rowvar=False, ddof=1) if len(X) > 1 else None
            second = X.T@X/len(X)
            if C is not None:
                np.testing.assert_allclose(second, (len(X)-1)/len(X)*C+np.outer(mean, mean), rtol=1e-9, atol=1e-10)
            W = None if L is None else np.linalg.solve(L, X.T).T
            quadratic = np.array([x@np.linalg.solve(p, x) for x, p in zip(X, P, strict=True)])
            result = {'valid_trials': len(X), 'mean_error_vector': mean.tolist(), 'centered_covariance_ddof1': None if C is None else C.tolist(), 'error_second_moment': second.tolist(),
                'mean_translation_error_m': float(np.linalg.norm(X[:, :3], axis=1).mean()), 'translation_rmse_m': float(np.sqrt(np.mean(np.sum(X[:, :3]**2, axis=1)))),
                'mean_rotation_error_deg': float(np.rad2deg(np.linalg.norm(X[:, 3:], axis=1)).mean()), 'rotation_rmse_deg': float(np.rad2deg(np.sqrt(np.mean(np.sum(X[:, 3:]**2, axis=1))))),
                'translation_bias_norm_m': float(np.linalg.norm(mean[:3])), 'rotation_bias_norm_deg': float(np.rad2deg(np.linalg.norm(mean[3:]))),
                'true_target_95pct_containment': float(np.mean(quadratic <= chi2.ppf(.95, 6))),
                'baseline_information_whitened_variance_trace': None if W is None or len(X) < 2 else float(np.trace(np.cov(W, rowvar=False, ddof=1))),
                'baseline_information_whitened_error_second_moment_trace': None if W is None else float(np.mean(np.sum(W**2, axis=1))),
                'baseline_information_whitened_bias_squared': None if W is None else float(W.mean(0)@W.mean(0)),
                'mean_points': float(np.mean([r['arms'][arm]['points'] for r in records]))}
            arrays[key+'_'+arm+'_'+label+'_xi'] = X
            arrays[key+'_'+arm+'_'+label+'_readout'] = P
            arrays[key+'_'+arm+'_'+label+'_trial_ids'] = np.asarray([r['trial'] for r in records])
            return result
        arm_reports = {}
        for arm in ['fixed_ids', 'regenerated']:
            available = [r for r in trials if r['arms'][arm]['successful']]
            arm_reports[arm] = {'statuses': {status: sum(r['arms'][arm]['status'] == status for r in trials) for status in {r['arms'][arm]['status'] for r in trials}},
                'baseline': summarize([base], arm, 'baseline') if base['arms'][arm]['successful'] else {'valid_trials': 0, 'metrics': None},
                'own': summarize(available, arm, 'own'), 'matched': summarize(paired, arm, 'matched')}
        reports[key] = {'attempted_trials': len(trials), 'paired_successful_trials': len(paired), 'arms': arm_reports}
    np.savez_compressed(out/'score-arrays.npz', **arrays)
    write(out/'scores.json', {'created_utc': now(), 'scenes': reports, 'prediction_lock_sha256': sha(out/'prediction-lock.json'), 'arrays_sha256': sha(out/'score-arrays.npz'), 'scope': 'True-pose-relative synthetic error versus dispersion, conditional half-LSB ensemble, fixed versus regenerated queries. Six artificial scenes, not physical day/night/rain recordings, sensor calibration or full native VO/MAC-I² reproduction.'})
    for key, record in reports.items():
        print(key, record['paired_successful_trials'], {a: {k: v for k, v in r['matched'].items() if k in ['translation_rmse_m', 'rotation_rmse_deg', 'baseline_information_whitened_variance_trace', 'baseline_information_whitened_error_second_moment_trace']} for a, r in record['arms'].items()}, flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', choices=['qualify', 'capture', 'score'])
    parser.add_argument('--protocol', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    plan, out = read(args.protocol), Path(args.output)
    verify(plan)
    if args.stage == 'qualify':
        out.mkdir(parents=True, exist_ok=False)
        qualify(plan, read(plan['inventory']), out)
    elif args.stage == 'capture':
        out.mkdir(parents=True, exist_ok=False)
        (out/'dense').mkdir()
        (out/'packets').mkdir()
        capture(plan, out)
    else:
        score(plan, out)
    verify(plan)


if __name__ == '__main__':
    main()
