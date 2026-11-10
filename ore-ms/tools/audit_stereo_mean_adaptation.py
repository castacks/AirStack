"""Independent head-state, sparse stereo and native-motion metric reconstruction."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation
import torch

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/stereo-mean-adaptation/run-001'


def read(p):
    return json.loads(p.read_text())


def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def labels(row):
    with Image.open(ROOT/row['depth']['path']) as im:
        depth = np.asarray(im).astype(np.float64)/256
    disparity = np.zeros_like(depth)
    np.divide(row['K'][0][0]*row['baseline_m'], depth, out=disparity, where=depth > 0)
    valid = depth > 0
    for y in range(len(depth)):
        right = np.arange(depth.shape[1])-disparity[y]
        valid[y] &= (right >= 0)&(right < depth.shape[1]-1)
    return -disparity.astype(np.float32), valid


def pose(v):
    t = np.eye(4); v = np.asarray(v).reshape(7)
    t[:3, :3] = Rotation.from_quat(v[3:]).as_matrix(); t[:3, 3] = v[:3]
    return t


def audit():
    plan = read(ROOT/'experiments/stereo-mean-adaptation-freeze-2026-11-10.json')
    for rel, digest in plan['pins'].items():
        assert sha(ROOT/rel) == digest, rel
    checkpoint = OUT/'final-mean-head.pth'; lock = read(OUT/'checkpoint-lock.json')
    assert sha(checkpoint) == lock['checkpoint_sha256']
    assert lock['plan_sha256'] == sha(ROOT/'experiments/stereo-mean-adaptation-freeze-2026-11-10.json')
    state = torch.load(checkpoint, map_location='cpu', weights_only=True)
    original = torch.load(ROOT/'models/macslam/MACT_Offroad.pth', map_location='cpu', weights_only=True)
    names = ['memory_decoder.update_block.flow_head.conv2.'+x for x in ['weight', 'bias']]
    assert set(state['mean_head']) == set(names) and state['steps'] == 40
    deltas = {}
    for name in names:
        base = original['module.'+name]; final = state['mean_head'][name]
        assert torch.equal(base[1], final[1]) and not torch.equal(base[0], final[0])
        assert torch.isfinite(final).all()
        deltas[name] = float((base[0]-final[0]).norm())
    data = read(OUT/'dataset.json'); records = {r['condition']: r for r in data['records']}
    assert len({r['sequence'] for r in data['records']}) == 3
    assert state['training_recordings'] == [records[c]['sequence'] for c in ['day', 'night']]
    assert state['excluded_recording'] == records['rain']['sequence'] not in state['training_recordings']
    journal = [json.loads(s) for s in (OUT/'training.jsonl').read_text().splitlines()]
    assert len(journal) == 40
    for i, r in enumerate(journal):
        condition = ['day', 'night'][i%2]
        assert r['condition'] == condition and r['source_id'] == f'{i//2:06}' and r['step'] == i+1
        _, valid = labels(records[condition]['rows'][i//2])
        assert int(valid.sum()) == r['valid_pixels'] > 0
        assert math.isfinite(r['loss']) and math.isfinite(r['gradient_norm_before_clip']) and r['gradient_norm_before_clip'] > 0
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
    fit = read(OUT/'training-fit-check.json')
    for r in fit['rows']:
        row = records[r['condition']]['rows'][int(r['source_id'])]; target, valid = labels(row)
        with np.load(OUT/'training-fit'/f'{r["condition"]}-{r["source_id"]}.npz', allow_pickle=False) as z:
            np.testing.assert_array_equal(z['target_at_valid'], target[valid])
            for method in ['base', 'adapted']:
                e = np.abs(z[method+'_at_valid']-target[valid])
                np.testing.assert_allclose(r['smooth_l1_px'][method], np.where(e < 1., .5*e**2, e-.5).mean(), rtol=1e-6)
    return {'status': 'PASS', 'producer_or_scorer_imported': False, 'frozen_hashes_verified': len(plan['pins']),
            'horizontal_head_parameter_change_norms': deltas, 'vertical_head_rows_exact': True,
            'artifact_only_contains_two_mean_parameters': True, 'training_steps_verified': 40,
            'excluded_record_not_in_training_journal': True, 'stereo_metric_checks': stereo_checks,
            'native_motions_reconstructed': motion_checks, 'passthrough_slots_exact': 19,
            'common_metric_statistics_verified': True, 'postfit_training_rows_verified': len(fit['rows'])}


if __name__ == '__main__':
    result = audit()
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)
