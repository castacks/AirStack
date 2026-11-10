"""Independent NumPy reconstruction of adapter training, rain stereo and native poses."""
import json
import numpy as np
import torch
from datetime import datetime
from scipy.linalg import expm, logm
from audit_stereo_mean_adaptation import ROOT, labels, pose, read, sha
from audit_mean_descent_contract import load, prediction, smooth

OUT = ROOT/'results/final-residual-fit/run-001'
PLAN = ROOT/'experiments/final-residual-fit-freeze-2026-11-10.json'


def audit():
    plan = read(PLAN)
    pins = dict(plan['training_pins']); pins.update(plan['evaluation_pins'])
    for rel, digest in pins.items():
        assert sha(ROOT/rel) == digest, rel
    lock = read(OUT/'checkpoint-lock.json')
    assert lock['plan_sha256'] == sha(PLAN)
    assert sha(OUT/'final-residual.pth') == lock['checkpoint_sha256']
    state = torch.load(OUT/'final-residual.pth', map_location='cpu', weights_only=True)
    weight, bias = state['weight'].numpy().astype(float), state['bias'].numpy().astype(float)
    assert weight.shape == (1, 256, 3, 3) and bias.shape == (1,)
    assert np.isfinite(weight).all() and np.isfinite(bias).all()
    assert state['base_sha256'] == sha(ROOT/'models/macslam/MACT_Offroad.pth')
    steps = [json.loads(s) for s in (OUT/'training.jsonl').read_text().splitlines()]
    assert len(steps) == state['updates'] == lock['updates']
    assert len(steps) == 40 or lock['stop_reason'] is not None
    for i, step in enumerate(steps):
        assert step['update'] == i+1 and step['after'] < step['before']-1e-6
        assert step['before'] == (steps[i-1]['after'] if i else lock['initial_loss'])
        assert step['trials'][-1]['accepted'] and step['after'] == step['trials'][-1]['loss']
        for j, trial in enumerate(step['trials']):
            alpha = .001*.5**j
            assert trial['step_size'] == alpha
            threshold = step['before']-.0001*alpha*step['gradient_norm']
            np.testing.assert_allclose(threshold, trial['armijo_threshold'], atol=1e-12)
            assert trial['accepted'] == (np.isfinite(trial['loss']) and trial['loss'] <= threshold and trial['loss'] < step['before']-1e-6)
    initial, final, maximum_error = 0., 0., 0.
    caches = read(ROOT/'results/mean-descent-contract/run-001/cache-lock.json')['rows']
    for row in caches:
        assert row['condition'] in ['day', 'night']
        assert sha(ROOT/row['file']) == row['sha256']
        cache = load(ROOT/row['file']); pred = prediction(cache, weight, bias)
        initial += smooth(cache['base'][0, 0], cache)/40
        final += smooth(pred, cache)/40
        with np.load(OUT/'training-predictions'/f'{row["condition"]}-{row["source_id"]}.npz', allow_pickle=False) as z:
            delta = np.abs(z['final_at_valid']-pred[cache['valid']])
            maximum_error = max(maximum_error, float(delta.max()))
            np.testing.assert_allclose(z['final_at_valid'], pred[cache['valid']], atol=2e-5, rtol=2e-6)
    np.testing.assert_allclose([lock['initial_loss'], lock['final_loss']], [initial, final], atol=1e-6, rtol=1e-6)
    assert final < initial
    locked_at = datetime.fromisoformat(lock['created_at'])
    starts = list((OUT/'evaluation-starts').glob('*.json'))
    assert len(starts) == 5
    for p in starts:
        event = read(p)
        assert datetime.fromisoformat(event['started_at']) > locked_at
        assert event['checkpoint_sha256'] == lock['checkpoint_sha256']
        assert event['lock_sha256'] == sha(OUT/'checkpoint-lock.json')
    data = read(ROOT/'results/stereo-mean-adaptation/run-001/dataset.json')
    records = {r['condition']: r for r in data['records']}
    stereo = read(OUT/'stereo-scores.json'); stereo_checks = 0
    for row, score in zip(records['rain']['rows'], stereo['rows'], strict=True):
        target, valid = labels(row)
        assert row['source_id'] == score['source_id'] and int(valid.sum()) == score['valid_pixels']
        prediction = OUT/'stereo-predictions'/f'{row["source_id"]}.npz'
        assert sha(prediction) == score['prediction_sha256']
        with np.load(prediction, allow_pickle=False) as z:
            fb = row['K'][0][0]*row['baseline_m']
            # FP32 target and arithmetic mirror declared evaluation precision,
            # using independent NumPy arrays and Pillow label decoding.
            truth = np.float32(fb)/(-target[valid])
            for method, key in [('base', 'base_flow'), ('adapted', 'adapted_flow')]:
                px = z[key][0, 0][valid]; depth = np.float32(fb)/np.abs(px); err = depth-truth
                expected = {'signed_flow_mean_abs_error_px': np.abs(px-target[valid]).mean(),
                    'depth_mean_abs_error_m': np.abs(err).mean(), 'depth_rmse_m': np.sqrt(np.mean(err**2)),
                    'depth_absrel': np.mean(np.abs(err)/truth), 'nonnegative_horizontal_fraction': np.mean(px >= 0)}
                for k, v in expected.items():
                    np.testing.assert_allclose(score['methods'][method][k], v, rtol=3e-6, atol=1e-6)
                    stereo_checks += 1
    for method, metrics in stereo['equal_frame_mean_metrics'].items():
        for key, value in metrics.items():
            np.testing.assert_allclose(value, np.mean([r['methods'][method][key] for r in stereo['rows']]), atol=1e-12)
    vo = read(OUT/'vo-scores.json')
    old = read(ROOT/'results/pose-covariance/accuracy-001/predictions-v2/rain.json')
    truth = {r['pair']: np.asarray(r['reference_delta']) for r in read(ROOT/'results/native-information-weights/run-001/scores-rain.json')['rows']}
    calibration = read(ROOT/'results/pose-covariance/accuracy-001/rain/calibration.json')
    extrinsics = {}
    for camera in ['rgb', 'thr']:
        t = np.eye(4); t[:3, :3] = calibration['R_nir2'+camera]['values']
        t[:3, 3] = np.asarray(calibration['T_nir2'+camera]['values']).reshape(3)/1000
        extrinsics[camera] = t
    q = np.eye(4); q[:3, :3] = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]
    frames = {'rgb': q, 'thermal': extrinsics['rgb']@np.linalg.inv(extrinsics['thr'])@q}
    motion_checks = 0
    for arm, rows in vo['rows_by_arm'].items():
        camera = 'rgb' if arm == 'rgb' else 'thermal'
        ev = [json.loads(s) for s in (OUT/'vo'/arm/camera/'arbiter-events/events.jsonl').read_text().splitlines()]
        writes = {(e['from_idx'], e['frame_idx']): e for e in ev if e['kind'] == 'motion_result_written' and e['first_write_for_submission']}
        assert len(rows) == 19
        for i, row in enumerate(rows):
            event = writes[i, i+1]; a, b = event['time0_ns'], event['time1_ns']; c, d = old['rows'][i]['common_interval_ns']
            outside = max(a-c, c-b, a-d, d-b, 0)
            if event['derived_camera_delta_xyzw'] is None or outside > b-a:
                assert row['transform'] is None and row['errors'] is None
                continue
            local = pose(event['derived_camera_delta_xyzw']); frame = frames[camera]
            mapped = expm(logm(frame@local@np.linalg.inv(frame))*((d-c)/(b-a)))
            assert np.max(np.abs(mapped.imag)) < 1e-8
            mapped = mapped.real
            np.testing.assert_allclose(row['transform'], mapped, rtol=1e-8, atol=1e-9)
            r = truth[i][:3, :3].T@mapped[:3, :3]
            skew = [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]]
            angle = np.rad2deg(np.arctan2(np.linalg.norm(skew)/2, (np.trace(r)-1)/2))
            distance = np.linalg.norm(mapped[:3, 3]-truth[i][:3, 3])
            np.testing.assert_allclose([row['errors']['rotation_deg'], row['errors']['translation_m']], [angle, distance], rtol=1e-8, atol=1e-9)
            motion_checks += 1
        assert vo['coverage'][arm] == sum(r['errors'] is not None for r in rows)
    for i in range(19):
        assert vo['rows_by_arm']['original'][i]['transform'] == vo['rows_by_arm']['passthrough'][i]['transform']
    common = [i for i in range(19) if all(vo['rows_by_arm'][m][i]['errors'] is not None for m in vo['rows_by_arm'])]
    assert common == vo['common_pair_ids'] and len(common) == vo['common_count']
    for arm, metrics in vo['common_metrics'].items():
        for key, stats in metrics.items():
            values = np.array([vo['rows_by_arm'][arm][i]['errors'][key] for i in common])
            assert stats['count'] == len(values)
            for k, v in {'mean': values.mean(), 'median': np.median(values), 'rmse': np.sqrt(np.mean(values**2)), 'p95': np.percentile(values, 95), 'maximum': values.max()}.items():
                np.testing.assert_allclose(stats[k], v, atol=1e-12)
    # Reconstruct the final-only residual from captured rain features independently.
    rain_max_error = 0.
    for row in records['rain']['rows']:
        with np.load(OUT/'stereo-predictions'/f'{row["source_id"]}.npz', allow_pickle=False) as z:
            mask = z['mask'].reshape(1, 9, 8, 8, 32, 80).astype(float)
            prob = np.exp(mask-mask.max(axis=1, keepdims=True)); prob /= prob.sum(axis=1, keepdims=True)
            cache = {'hidden': z['hidden'].astype(float), 'probability': prob,
                     'base': z['base_flow'][:, :1].astype(float)}
            expected = prediction(cache, weight, bias)
            err = float(np.max(np.abs(expected-z['adapted_flow'][0, 0])))
            rain_max_error = max(rain_max_error, err)
            np.testing.assert_allclose(expected, z['adapted_flow'][0, 0], rtol=2e-6, atol=2e-5)
            np.testing.assert_array_equal(z['base_flow'][:, 1:], z['adapted_flow'][:, 1:])
    for arm in ['adapted', 'passthrough']:
        receipt = read(OUT/'vo'/arm/'frontend-contract.json')
        assert receipt['checkpoint_sha256'] == lock['checkpoint_sha256']
        assert len(receipt['receipts']) == 20
        assert all(r['temporal_flow_exact'] and r['vertical_flow_exact'] for r in receipt['receipts'])
        if arm == 'passthrough':
            assert all(r['zero_copy_equal'] and r['stereo_mean_max_change_px'] == 0 for r in receipt['receipts'])
    # Native array parity includes unmapped intervals and all timestamps.
    for filename in ['trajectory.npz']:
        a = OUT/'vo/original/thermal'/filename; b = OUT/'vo/passthrough/thermal'/filename
        with np.load(a) as aa, np.load(b) as bb:
            assert aa.files == bb.files
            for key in aa.files:
                np.testing.assert_array_equal(aa[key], bb[key])
    previous_stereo = read(ROOT/'results/stereo-mean-adaptation/run-001/stereo-scores.json')
    for a, b in zip(stereo['rows'], previous_stereo['rows'], strict=True):
        for k in a['methods']['base']:
            np.testing.assert_allclose(a['methods']['base'][k], b['methods']['base'][k], atol=1e-6, rtol=1e-6)
    return {'status': 'PASS', 'producer_or_scorer_imported': False, 'frozen_hashes_verified': len(pins),
            'training_updates_verified': len(steps), 'training_initial_loss_fp64': initial,
            'training_final_loss_fp64': final, 'training_max_prediction_error_px': maximum_error,
            'rain_max_adapter_prediction_error_px': rain_max_error,
            'checkpoint_locked_before_all_evaluations': True, 'stereo_metric_checks': stereo_checks,
            'native_motions_reconstructed': motion_checks, 'common_metric_statistics_verified': True,
            'previous_baseline_stereo_reproduced': True}


if __name__ == '__main__':
    result = audit()
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)
