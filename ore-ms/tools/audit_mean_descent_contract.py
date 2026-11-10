"""Independent NumPy adjoint, convolution, upsampling and descent reconstruction."""
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
import torch

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/mean-descent-contract/run-001'
OLD = ROOT/'results/stereo-mean-adaptation/run-001'


def read(p):
    return json.loads(p.read_text())


def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k].copy() if k == 'valid' else z[k].astype(np.float64) for k in z.files}


def windows(hidden):
    return np.lib.stride_tricks.sliding_window_view(np.pad(hidden[0], ((0, 0), (1, 1), (1, 1))), (3, 3), axis=(1, 2))


def prediction(cache, weight, bias):
    low = np.einsum('cyxij,cij->yx', windows(cache['hidden']), weight[0], optimize=True)+bias[0]
    h, w = low.shape; padded = np.pad(low, 1); tiles = np.zeros((8, 8, h, w))
    for dy in range(3):
        for dx in range(3):
            tiles += 8*cache['probability'][0, 3*dy+dx]*padded[dy:dy+h, dx:dx+w][None, None]
    return cache['base'][0, 0]+tiles.transpose(2, 0, 3, 1).reshape(8*h, 8*w)


def smooth(pred, cache):
    e = np.abs(pred[cache['valid']]-cache['target'][cache['valid']])
    return float(np.where(e < 1., .5*e**2, e-.5).mean())


def gradient_at_zero(cache, count):
    _, c, h, w = cache['hidden'].shape
    e = cache['base'][0, 0]-cache['target']
    gradient = np.zeros_like(e)
    gradient[cache['valid']] = np.clip(e[cache['valid']], -1, 1)/(int(cache['valid'].sum())*count)
    tiles = gradient.reshape(h, 8, w, 8).transpose(1, 3, 0, 2)
    low = np.zeros((h, w))
    for oy in range(-1, 2):
        for ox in range(-1, 2):
            contribution = 8*np.sum(cache['probability'][0, (oy+1)*3+ox+1]*tiles, axis=(0, 1))
            y0, y1, x0, x1 = max(0, -oy), min(h, h-oy), max(0, -ox), min(w, w-ox)
            low[y0+oy:y1+oy, x0+ox:x1+ox] += contribution[y0:y1, x0:x1]
    weight = np.einsum('cyxij,yx->cij', windows(cache['hidden']), low, optimize=True)[None]
    return weight, np.array([low.sum()])


def audit():
    plan = read(ROOT/'experiments/mean-descent-contract-freeze-2026-11-10.json')
    for rel, digest in plan['pins'].items():
        assert sha(ROOT/rel) == digest, rel
    manifest = read(OUT/'cache-lock.json'); dataset = read(OLD/'dataset.json')
    records = {r['condition']: r for r in dataset['records'] if r['role'] == 'train'}
    assert set(records) == {'day', 'night'} and len(manifest['rows']) == 40
    grad_w, grad_b, initial = np.zeros((1, 256, 3, 3)), np.zeros(1), 0.
    pixels = 0
    for index, row in enumerate(manifest['rows']):
        condition = ['day', 'night'][index%2]
        assert row['condition'] == condition and row['source_id'] == f'{index//2:06}'
        assert row['sequence'] == records[condition]['sequence']
        path = ROOT/row['file']; assert sha(path) == row['sha256']
        cache = load(path); source = records[condition]['rows'][index//2]
        with Image.open(ROOT/source['depth']['path']) as im:
            z = np.asarray(im).astype(float)/256
        disparity = np.zeros_like(z)
        np.divide(source['K'][0][0]*source['baseline_m'], z, out=disparity, where=z > 0)
        right = np.arange(z.shape[1])[None]-disparity
        valid = (z > 0)&(right >= 0)&(right < z.shape[1]-1)
        np.testing.assert_array_equal(cache['valid'], valid)
        np.testing.assert_array_equal(cache['target'], (-disparity).astype(np.float32))
        assert row['valid_pixels'] == int(valid.sum())
        assert np.isfinite(cache['hidden']).all() and np.isfinite(cache['probability']).all()
        assert np.min(cache['probability']) >= 0
        np.testing.assert_allclose(cache['probability'].sum(1), 1., atol=3e-7, rtol=3e-7)
        assert row['zero_residual_output_exact'] is True
        g, b = gradient_at_zero(cache, 40); grad_w += g; grad_b += b
        initial += smooth(cache['base'][0, 0], cache)/40; pixels += int(valid.sum())
    norm = np.sqrt(np.sum(grad_w**2)+np.sum(grad_b**2))
    direction_w, direction_b = -grad_w/norm, -grad_b/norm
    fd = read(OUT/'finite-difference.json'); result = read(OUT/'descent.json')
    artifact = OUT/'training-control-residual.pth'; assert sha(artifact) == result['artifact_sha256']
    state = torch.load(artifact, map_location='cpu', weights_only=True)
    assert state['training_control_only'] is True and state['updates'] == 3
    weight, bias = state['weight'].numpy().astype(float), state['bias'].numpy().astype(float)
    numerical_losses = {epsilon: {-1: 0., 1: 0.} for epsilon in [1e-4, 1e-5]}
    final = 0.; max_prediction_error = 0.
    for row in manifest['rows']:
        cache = load(ROOT/row['file'])
        response = prediction(cache, direction_w, direction_b)-cache['base'][0, 0]
        for epsilon in numerical_losses:
            for sign in [-1, 1]:
                numerical_losses[epsilon][sign] += smooth(cache['base'][0, 0]+sign*epsilon*response, cache)/40
        predicted = prediction(cache, weight, bias)
        final += smooth(predicted, cache)/40
        with np.load(OUT/'training-predictions'/f'{row["condition"]}-{row["source_id"]}.npz', allow_pickle=False) as z:
            np.testing.assert_array_equal(z['base_at_valid'], cache['base'][0, 0][cache['valid']])
            np.testing.assert_array_equal(z['target_at_valid'], cache['target'][cache['valid']])
            error = np.max(np.abs(z['final_at_valid']-predicted[cache['valid']]))
            max_prediction_error = max(max_prediction_error, float(error))
            np.testing.assert_allclose(z['final_at_valid'], predicted[cache['valid']], atol=2e-5, rtol=2e-6)
    np.testing.assert_allclose(fd['initial_loss'], initial, atol=1e-11)
    for probe in fd['probes']:
        values = numerical_losses[probe['epsilon']]
        central = (values[1]-values[-1])/(2*probe['epsilon'])
        np.testing.assert_allclose(probe['autograd_derivative'], -norm, rtol=1e-10, atol=1e-10)
        np.testing.assert_allclose(probe['central_derivative'], central, rtol=1e-7, atol=1e-7)
        assert abs(central+norm) <= 1e-6+1e-3*norm
    np.testing.assert_allclose(result['initial_loss'], initial, atol=1e-6, rtol=1e-6)
    np.testing.assert_allclose(result['final_loss'], final, atol=1e-6, rtol=1e-6)
    assert final < initial
    assert len(result['steps']) == 3
    for index, step in enumerate(result['steps']):
        if index:
            assert step['before'] == result['steps'][index-1]['after']
        assert step['after'] < step['before']-1e-6
        assert step['trials'][-1]['accepted'] and step['after'] == step['trials'][-1]['loss']
        for i, trial in enumerate(step['trials']):
            expected_step = 1e-3*(.5**i)
            assert trial['step_size'] == expected_step
            threshold = step['before']-1e-4*expected_step*step['gradient_norm']
            np.testing.assert_allclose(threshold, trial['armijo_threshold'], atol=1e-12)
            assert trial['accepted'] == (trial['loss'] <= threshold and trial['loss'] < step['before']-1e-6)
    assert sha(OLD/'final-mean-head.pth') == plan['previous_candidate_sha256'] == result['previous_checkpoint_sha256']
    return {'status': 'PASS', 'producer_or_scorer_imported': False, 'frozen_hashes_verified': len(plan['pins']),
            'training_caches_verified': 40, 'training_label_pixels': pixels, 'independent_analytic_gradient_norm': float(norm),
            'finite_difference_scales_verified': 2, 'accepted_step_conditions_verified': 3,
            'initial_loss_independent_fp64': initial, 'final_loss_independent_fp64': final,
            'maximum_final_prediction_discrepancy_px': max_prediction_error,
            'previous_checkpoint_unchanged': True, 'excluded_record_evaluated': False}


if __name__ == '__main__':
    result = audit()
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)
