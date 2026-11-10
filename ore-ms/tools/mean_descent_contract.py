"""Training-only gradient diagnosis and a frozen-decoder final residual control."""
import argparse
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from stereo_mean_adaptation import (ROOT, OUT as PREVIOUS, build, configure,
                                    deny_evaluation_data, load_pair, objective,
                                    read, setup, sha, write)

OUT = ROOT/'results/mean-descent-contract/run-001'
PLAN = ROOT/'experiments/mean-descent-contract-freeze-2026-11-10.json'


def strict_math():
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.set_float32_matmul_precision('highest')


def verify():
    plan = read(PLAN)
    for rel, digest in plan['pins'].items():
        assert sha(ROOT/rel) == digest, rel
    return plan


def state_digest(model):
    digest = hashlib.sha256()
    for name, value in model.state_dict().items():
        digest.update(name.encode()); digest.update(value.detach().cpu().contiguous().numpy().tobytes())
    return digest.hexdigest()


@contextmanager
def capture_final(model):
    """Observe native tensors only; original model computation is unchanged."""
    cache = {'head_calls': 0, 'upsample_calls': 0}
    def head_input(_, args):
        cache['head_calls'] += 1
        cache['hidden'] = args[0].detach().clone()
    decoder = model.memory_decoder
    original = decoder.upsample_flow
    def upsample(flow, mask):
        if cache['upsample_calls'] % 2 == 0:
            cache['mask'] = mask.detach().clone()
        cache['upsample_calls'] += 1
        return original(flow, mask)
    handle = decoder.update_block.flow_head.conv2.register_forward_pre_hook(head_input)
    decoder.upsample_flow = upsample
    try:
        yield cache
    finally:
        handle.remove(); decoder.upsample_flow = original


def convex_upsample(flow, probability):
    n, c, h, w = flow.shape
    if probability.shape != (n, 9, 8, 8, h, w):
        raise ValueError('native frozen convex mask dimensions required')
    patches = F.unfold(8*flow, (3, 3), padding=1).view(n, c, 9, h, w)
    weighted = (probability.unsqueeze(1)*patches.unsqueeze(-3).unsqueeze(-3)).sum(2)
    return weighted.permute(0, 1, 4, 2, 5, 3).reshape(n, c, 8*h, 8*w)


def residual_prediction(head, cache):
    return cache['base']+convex_upsample(F.conv2d(cache['hidden'], head[0], head[1], padding=1), cache['probability'])


def total_loss(head, caches, gradient=False):
    if gradient:
        for p in head:
            p.grad = None
    value = 0.
    for cache in caches:
        predicted = residual_prediction(head, cache)[0, 0]
        valid = cache['valid']
        loss = F.smooth_l1_loss(predicted[valid], cache['target'][valid], beta=1., reduction='mean')/len(caches)
        if gradient:
            loss.backward()
        value += float(loss.detach())
    return value


def zero_head(device, dtype):
    return [torch.nn.Parameter(torch.zeros((1, 256, 3, 3), device=device, dtype=dtype)),
            torch.nn.Parameter(torch.zeros(1, device=device, dtype=dtype))]


def cached_rows(device, dtype):
    manifest = read(OUT/'cache-lock.json'); rows = []
    for record in manifest['rows']:
        path = ROOT/record['file']; assert sha(path) == record['sha256']
        with np.load(path, allow_pickle=False) as z:
            row = {k: torch.from_numpy(z[k].copy()).to(device=device, dtype=torch.bool if k == 'valid' else dtype)
                   for k in ['hidden', 'probability', 'base', 'target', 'valid']}
        row['condition'], row['source_id'] = record['condition'], record['source_id']
        rows.append(row)
    return rows


def shared_probe():
    verify(); setup(); strict_math()
    dataset = read(PREVIOUS/'dataset.json'); model = build(); names = configure(model)
    parameters = [p for n, p in model.named_parameters() if n in names]
    results = []
    with deny_evaluation_data(dataset):
        for condition in ['day', 'night']:
            row = next(r for r in dataset['records'] if r['condition'] == condition)['rows'][0]
            a, b, target, valid = load_pair(row)
            model.zero_grad(set_to_none=True)
            outputs, _ = model(a, b); loss = objective(outputs[-1], target, valid); loss.backward()
            originals = [p.detach().clone() for p in parameters]
            gradients = [p.grad.detach().clone() for p in parameters]
            for g in gradients:
                g[1].zero_()
            norm = torch.sqrt(sum(g.square().sum() for g in gradients))
            directions = [-g/norm for g in gradients]
            probes = []
            with torch.no_grad():
                for epsilon in [1e-3, 1e-4]:
                    values = []
                    for sign in [-1., 1.]:
                        for p, original, direction in zip(parameters, originals, directions):
                            p.copy_(original+sign*epsilon*direction)
                        prediction, _ = model(a, b)
                        values.append(float(objective(prediction[-1], target, valid)))
                    probes.append({'epsilon': epsilon, 'loss_minus': values[0], 'loss_plus': values[1],
                                   'central_directional_derivative': (values[1]-values[0])/(2*epsilon)})
                for p, original in zip(parameters, originals):
                    p.copy_(original)
                prediction, _ = model(a, b)
                restored = float(objective(prediction[-1], target, valid))
            assert restored == float(loss.detach())
            results.append({'condition': condition, 'source_id': row['source_id'], 'loss': restored,
                            'autograd_directional_derivative': -float(norm), 'probes': probes,
                            'parameters_restored_exactly': all(torch.equal(p, original) for p, original in zip(parameters, originals))})
    write(OUT/'shared-probe.json', {'strict_fp32_tf32_disabled': True, 'rows': results,
          'optimizer_steps': 0, 'excluded_record_reads': 0, 'scope': 'diagnostic; native coordinate detach remains unchanged'})
    print(results)


def cache_training():
    verify(); setup()
    dataset = read(PREVIOUS/'dataset.json'); model = build()
    for p in model.parameters():
        p.requires_grad_(False)
    before = state_digest(model); rows = []
    zero = zero_head('cuda', torch.float32)
    records = {r['condition']: r for r in dataset['records']}
    with deny_evaluation_data(dataset), torch.inference_mode():
        for i in range(20):
            for condition in ['day', 'night']:
                row = records[condition]['rows'][i]
                a, b, target, valid = load_pair(row)
                with capture_final(model) as captured:
                    prediction, covariance = model.inference(a, b)
                assert captured['head_calls'] == 12 and captured['upsample_calls'] == 24
                hidden, mask = captured['hidden'], captured['mask']
                n, _, h, w = hidden.shape
                probability = mask.view(n, 9, 8, 8, h, w).softmax(1)
                cache = {'hidden': hidden, 'probability': probability, 'base': prediction[:, :1], 'target': target, 'valid': valid}
                actual = residual_prediction(zero, cache)
                assert torch.equal(actual, cache['base'])
                path = OUT/'cache'/f'{condition}-{i:06}.npz'; path.parent.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(path, **{k: v.cpu().numpy() for k, v in cache.items()})
                rows.append({'condition': condition, 'sequence': records[condition]['sequence'], 'source_id': row['source_id'],
                             'file': str(path.relative_to(ROOT)), 'sha256': sha(path), 'valid_pixels': int(valid.sum()),
                             'zero_residual_output_exact': True})
                print(condition, row['source_id'], int(valid.sum()), flush=True)
    assert before == state_digest(model)
    write(OUT/'cache-lock.json', {'rows': rows, 'model_state_unchanged': True, 'original_state_sha256': before,
          'rain_data_read': False, 'source_model_sha256': sha(ROOT/'models/macslam/MACT_Offroad.pth'),
          'precision': {'matmul_tf32': torch.backends.cuda.matmul.allow_tf32, 'cudnn_tf32': torch.backends.cudnn.allow_tf32},
          'model': 'frozen final horizontal output + frozen-mask upsampling of zero-initialized residual convolution on final hidden features'})


def finite_difference():
    verify(); setup()
    caches = cached_rows('cpu', torch.float64); head = zero_head('cpu', torch.float64)
    initial = total_loss(head, caches, True)
    norm = torch.sqrt(sum(p.grad.square().sum() for p in head))
    direction = [-p.grad.detach().clone()/norm for p in head]
    probes = []
    with torch.no_grad():
        for epsilon in [1e-4, 1e-5]:
            values = []
            for sign in [-1., 1.]:
                for p, d in zip(head, direction):
                    p.copy_(sign*epsilon*d)
                values.append(total_loss(head, caches))
            central = (values[1]-values[0])/(2*epsilon)
            expected = -float(norm)
            passed = abs(central-expected) <= 1e-6+1e-3*abs(expected)
            probes.append({'epsilon': epsilon, 'loss_minus': values[0], 'loss_plus': values[1],
                           'central_derivative': central, 'autograd_derivative': expected, 'pass': passed})
        for p in head:
            p.zero_()
        restored = total_loss(head, caches)
    assert restored == initial and all(p['pass'] for p in probes)
    write(OUT/'finite-difference.json', {'status': 'PASS', 'dtype': 'float64', 'training_frames': len(caches),
          'initial_loss': initial, 'restored_loss': restored, 'probes': probes, 'optimizer_steps': 0})
    print(probes)


def armijo_step(head, caches, initial_step=1e-3, max_trials=12):
    current = total_loss(head, caches, True)
    norm = torch.sqrt(sum(p.grad.square().sum() for p in head))
    if not torch.isfinite(norm) or norm <= 0:
        raise ValueError('nonzero finite gradient required')
    direction = [-p.grad.detach().clone()/norm for p in head]
    original = [p.detach().clone() for p in head]
    trials = []
    with torch.no_grad():
        for i in range(max_trials):
            step = initial_step*(.5**i)
            for p, old, d in zip(head, original, direction):
                p.copy_(old+step*d)
            candidate = total_loss(head, caches)
            threshold = current-1e-4*step*float(norm)
            accepted = np.isfinite(candidate) and candidate <= threshold and candidate < current-1e-6
            trials.append({'step_size': step, 'loss': candidate, 'armijo_threshold': threshold, 'accepted': bool(accepted)})
            if accepted:
                return {'before': current, 'after': candidate, 'gradient_norm': float(norm), 'trials': trials}
        for p, old in zip(head, original):
            p.copy_(old)
    raise ValueError('no accepted objective-decreasing training step; original parameters restored; '+json.dumps(trials))


def descent():
    verify(); setup(); strict_math()
    assert read(OUT/'finite-difference.json')['status'] == 'PASS'
    caches = cached_rows('cuda', torch.float32); head = zero_head('cuda', torch.float32)
    steps = []
    journal = OUT/'descent-steps.jsonl'
    with journal.open('x') as stream:
        for i in range(3):
            step = armijo_step(head, caches)
            steps.append(step); stream.write(json.dumps({'update': i+1, **step})+'\n'); stream.flush()
    final = total_loss(head, caches)
    assert final == steps[-1]['after'] and all(s['after'] < s['before'] for s in steps)
    with torch.no_grad():
        for row in caches:
            prediction = residual_prediction(head, row)[0, 0]
            valid = row['valid']; path = OUT/'training-predictions'/f'{row["condition"]}-{row["source_id"]}.npz'
            path.parent.mkdir(exist_ok=True)
            np.savez_compressed(path, base_at_valid=row['base'][0, 0][valid].cpu().numpy(),
                                final_at_valid=prediction[valid].cpu().numpy(), target_at_valid=row['target'][valid].cpu().numpy())
    artifact = OUT/'training-control-residual.pth'
    with artifact.open('xb') as f:
        torch.save({'weight': head[0].detach().cpu(), 'bias': head[1].detach().cpu(),
                    'model': 'final-only frozen-feature horizontal residual', 'training_control_only': True, 'updates': 3}, f)
    write(OUT/'descent.json', {'status': 'PASS_TRAINING_DESCENT_ONLY', 'steps': steps, 'initial_loss': steps[0]['before'],
          'final_loss': final, 'training_frames': len(caches), 'excluded_record_evaluation': False,
          'artifact_sha256': sha(artifact), 'previous_checkpoint_sha256': sha(PREVIOUS/'final-mean-head.pth'),
          'scope': 'three training-only contract updates; no accuracy or deployment claim'})
    print(read(OUT/'descent.json'))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['shared', 'cache', 'finite-difference', 'descent'])
    {'shared': shared_probe, 'cache': cache_training, 'finite-difference': finite_difference, 'descent': descent}[p.parse_args().action]()
