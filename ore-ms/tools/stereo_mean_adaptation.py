"""Opt-in horizontal thermal stereo mean adaptation; original models unchanged."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import io
import builtins
import json
from pathlib import Path
import random
import sys

import numpy as np
import torch
import torch.nn.functional as F

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'MAC-VO-main'))
OUT = ROOT/'results/stereo-mean-adaptation/run-001'
PLAN = ROOT/'experiments/stereo-mean-adaptation-freeze-2026-11-10.json'
PREFIX = 'memory_decoder.update_block.flow_head.conv2.'


def read(p):
    return json.loads(p.read_text())


def write(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)


def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def build():
    from Module.Network.FlowFormer.configs.submission import get_cfg
    from Module.Network.FlowFormerCov import build_flowformer
    cfg = get_cfg(); cfg.latentcostformer.pretrain = False
    cfg.latentcostformer.decoder_depth = 12
    model = build_flowformer(cfg, torch.float32, torch.float32)
    model.load_ddp_state_dict(torch.load(ROOT/'models/macslam/MACT_Offroad.pth', map_location='cpu', weights_only=True))
    return model.eval().cuda()


def configure(model):
    model.eval(); model.memory_decoder.training = True
    names = []
    for name, p in model.named_parameters():
        p.requires_grad_(name.startswith(PREFIX))
        if p.requires_grad:
            names.append(name)
    if names != [PREFIX+'weight', PREFIX+'bias']:
        raise ValueError('exact final mean convolution required')
    layer = model.memory_decoder.update_block.flow_head.conv2
    if layer.out_channels != 2 or layer.bias is None:
        raise ValueError('native two-output mean convolution required')
    return names


def sparse_target(depth, fx, baseline):
    """Original own-camera label mask, independent of the predicted flow."""
    if depth.ndim != 2 or depth.dtype != np.uint16 or fx <= 0 or baseline <= 0:
        raise ValueError('native uint16 depth and positive calibration required')
    z = depth.astype(np.float64)/256
    disparity = np.zeros_like(z)
    np.divide(fx*baseline, z, out=disparity, where=z > 0)
    u = np.arange(depth.shape[1])[None, :]
    valid = (z > 0)&(u-disparity >= 0)&(u-disparity < depth.shape[1]-1)
    return -disparity.astype(np.float32), valid


def load_pair(row):
    import cv2
    from DataLoader.Dataset.MS2 import thermal_producer_replay
    tensors = []
    for side in ['left', 'right']:
        path = ROOT/row[side]['path']; assert sha(path) == row[side]['sha256']
        raw = cv2.imread(str(path), -1)
        assert raw.dtype == np.uint16 and raw.shape == (256, 640)
        gray = thermal_producer_replay(raw)
        tensor = torch.from_numpy(np.repeat(gray[None, None], 3, axis=1).copy()).float()/255
        tensors.append(tensor.cuda())
    path = ROOT/row['depth']['path']; assert sha(path) == row['depth']['sha256']
    depth = cv2.imread(str(path), -1)
    target, valid = sparse_target(depth, row['K'][0][0], row['baseline_m'])
    if not valid.any():
        raise ValueError('empty sparse stereo label support')
    return (*tensors, torch.from_numpy(target).cuda(), torch.from_numpy(valid).cuda())


def objective(prediction, target, valid):
    if prediction.shape[0] != 1 or prediction.shape[1] != 2 or prediction.shape[2:] != target.shape or not valid.any():
        raise ValueError('final full-resolution stereo prediction and nonempty label mask required')
    return F.smooth_l1_loss(prediction[0, 0][valid], target[valid], beta=1., reduction='mean')


@contextmanager
def deny_evaluation_data(dataset):
    forbidden = {str((ROOT/r[k]['path']).resolve()) for record in dataset['records'] if record['condition'] == 'rain'
                 for r in record['rows'] for k in ['left', 'right', 'depth']}
    original = builtins.open, io.open
    def guard(fn):
        def opened(path, *args, **kwargs):
            if isinstance(path, (str, Path)):
                p = Path(path).resolve()
                if str(p) in forbidden or p.name.startswith('scores-'):
                    raise ValueError('excluded-record data or earlier score read during fit')
            return fn(path, *args, **kwargs)
        return opened
    builtins.open, io.open = guard(original[0]), guard(original[1])
    try:
        yield
    finally:
        builtins.open, io.open = original


def setup():
    import cv2
    torch.set_num_threads(1); cv2.setNumThreads(1)
    torch.manual_seed(0); np.random.seed(0); random.seed(0)


def freeze_check():
    plan = read(PLAN)
    for rel, digest in plan['pins'].items():
        if sha(ROOT/rel) != digest:
            raise ValueError('frozen source/input changed: '+rel)
    return plan


def contract():
    setup(); dataset = read(OUT/'dataset.json'); model = build(); names = configure(model)
    row = next(r for r in dataset['records'] if r['condition'] == 'day')['rows'][0]
    with deny_evaluation_data(dataset):
        a, b, target, valid = load_pair(row)
        predictions, _ = model(a, b); loss = objective(predictions[-1], target, valid)
        loss.backward()
    gradients = {}
    for name, parameter in model.named_parameters():
        if name not in names:
            assert parameter.grad is None
        else:
            assert parameter.grad is not None and torch.isfinite(parameter.grad).all()
            gradients[name] = {'horizontal_norm': float(parameter.grad[0].norm()), 'vertical_norm': float(parameter.grad[1].norm())}
    assert all(v['horizontal_norm'] > 0 for v in gradients.values())
    write(OUT/'contract.json', {'status': 'PASS', 'trainable_names': names, 'gradients': gradients,
          'valid_training_pixels': int(valid.sum()), 'recurrent_stages': len(predictions),
          'loss': float(loss.detach()), 'optimizer_steps': 0, 'excluded_record_reads': 0,
          'horizontal_trainable_scalars': sum(p[0].numel() for n, p in model.named_parameters() if n in names)})
    print(read(OUT/'contract.json'))


def train():
    setup(); plan = freeze_check(); dataset = read(OUT/'dataset.json')
    assert read(OUT/'contract.json')['status'] == 'PASS'
    model = build(); names = configure(model)
    initial = {n: t.detach().cpu().clone() for n, t in model.state_dict().items()}
    optimizer = torch.optim.Adam([p for p in model.parameters() if p.requires_grad], lr=1e-5, weight_decay=0)
    records = {r['condition']: r for r in dataset['records']}
    assert {r['sequence'] for r in dataset['records']}.__len__() == 3
    journal = OUT/'training.jsonl'; journal.parent.mkdir(parents=True, exist_ok=True)
    with deny_evaluation_data(dataset), journal.open('x') as stream:
        for step in range(40):
            condition = ['day', 'night'][step%2]; row = records[condition]['rows'][step//2]
            a, b, target, valid = load_pair(row)
            optimizer.zero_grad(set_to_none=True)
            predictions, _ = model(a, b)
            loss = objective(predictions[-1], target, valid)
            if not torch.isfinite(loss):
                raise ValueError('nonfinite training loss')
            loss.backward()
            for name, p in model.named_parameters():
                if name in names:
                    if p.grad is None or not torch.isfinite(p.grad).all():
                        raise ValueError('invalid mean-head gradient')
                    p.grad[1].zero_()
                elif p.grad is not None:
                    raise ValueError('frozen parameter received gradient')
            norm = torch.nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad], 1.)
            if not torch.isfinite(norm):
                raise ValueError('invalid gradient norm')
            optimizer.step()
            record = {'step': step+1, 'condition': condition, 'source_id': row['source_id'],
                      'loss': float(loss.detach()), 'valid_pixels': int(valid.sum()), 'gradient_norm_before_clip': float(norm)}
            stream.write(json.dumps(record)+'\n'); stream.flush(); print(record, flush=True)
    state = {n: t.detach().cpu().clone() for n, t in model.state_dict().items()}
    changed = []
    for name, value in state.items():
        if name in names:
            assert torch.equal(value[1], initial[name][1])
            if not torch.equal(value[0], initial[name][0]):
                changed.append(name)
        else:
            assert torch.equal(value, initial[name]), name
    assert changed == names
    checkpoint = OUT/'final-mean-head.pth'
    with checkpoint.open('xb') as f:
        torch.save({'mean_head': {n: state[n] for n in names}, 'steps': 40, 'training_recordings': [records[c]['sequence'] for c in ['day', 'night']],
                    'excluded_recording': records['rain']['sequence'], 'base_sha256': sha(ROOT/'models/macslam/MACT_Offroad.pth')}, f)
    write(OUT/'checkpoint-lock.json', {'created_at': datetime.now(timezone.utc).isoformat(), 'checkpoint_sha256': sha(checkpoint),
          'plan_sha256': sha(PLAN), 'changed_parameter_names': changed, 'all_other_state_bitwise_unchanged': True,
          'vertical_mean_head_rows_unchanged': True, 'optimizer_steps': 40, 'evaluation_performed': False,
          'training_recordings': [records[c]['sequence'] for c in ['day', 'night']], 'excluded_recording': records['rain']['sequence']})


def load_mean_head(model):
    lock = read(OUT/'checkpoint-lock.json'); checkpoint = OUT/'final-mean-head.pth'
    assert sha(checkpoint) == lock['checkpoint_sha256']
    payload = torch.load(checkpoint, map_location='cpu', weights_only=True)
    current = model.state_dict()
    assert set(payload['mean_head']) == {PREFIX+'weight', PREFIX+'bias'}
    for name, value in payload['mean_head'].items():
        assert torch.equal(current[name][1].cpu(), value[1])
        current[name].copy_(value.to(current[name].device))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['contract', 'train'])
    {'contract': contract, 'train': train}[parser.parse_args().action]()
