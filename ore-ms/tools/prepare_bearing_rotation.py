"""Fixed-grid covariance recovery for a subsequent bearing-rotation assay.

No pose references or pose optimizer are used. Real-motion scoring is separate.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/bearing-rotation-preflight/run-001'
OLD = ROOT/'results/thermal-matcher-pilot/run-001'
PLAN = ROOT/'experiments/bearing-rotation-acquisition-freeze-2026-11-12.json'
sys.path.insert(0, str(ROOT/'MAC-VO-main'))


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)


def grid():
    x, y = np.meshgrid(np.arange(16, 625, 16), np.arange(16, 241, 16))
    return np.column_stack([x.ravel(), y.ravel()])


def cached_pair(index):
    name = f'flow/{index:03}.npz'
    locked = json.loads((OLD/'prediction-lock.json').read_text())['pins'][name]
    if sha(OLD/name) != locked:
        raise ValueError('Changed cached prediction')
    with np.load(OLD/name, allow_pickle=False) as data:
        return data['original'].copy(), data['replacement'].copy()


def inventory():
    uv = grid(); counts = []
    source = json.loads((ROOT/'results/pose-covariance/accuracy-001/rain/source-manifest.json').read_text())
    for row in source['records'][:20]:
        for side in ['left', 'right']:
            item = row['cameras']['thr'][side]
            if sha(ROOT/item['path']) != item['sha256']:
                raise ValueError('Changed source image')
    for index in range(19):
        flows = cached_pair(index)
        valid = np.ones(len(uv), bool)
        for flow in flows:
            if flow.shape != (1, 2, 256, 640):
                raise ValueError('Unexpected flow shape')
            endpoint = uv+flow[0, :, uv[:, 1], uv[:, 0]]
            valid &= np.isfinite(endpoint).all(1)&(endpoint >= 0).all(1)&(endpoint < [640, 256]).all(1)
        counts.append(int(valid.sum()))
    write(OUT/'inventory.json', {'grid_points': len(uv), 'pair_count': 19,
        'common_finite_inbound_before_covariance': counts,
        'source_images_verified': 40, 'pose_references_read': False,
        'covariance_cached_in_old_packet': False,
        'note': 'Inbound is not visibility or correspondence truth; no feature/score-based selection.'})


def verify():
    plan = json.loads(PLAN.read_text())
    for name, digest in plan['pins'].items():
        if sha(ROOT/name) != digest:
            raise ValueError('Changed frozen acquisition input: '+name)


def acquire():
    verify()
    from thermal_matcher_pilot import verify as verify_old
    verify_old()
    import torch
    import cv2
    from Utility.Config import asNamespace
    from DataLoader.Dataset.MS2 import MS2ManifestSequence, THERMAL_RECIPE
    from Module.Frontend.Frontend import FlowFormerCovFrontend
    torch.manual_seed(0); np.random.seed(0); torch.set_num_threads(1); cv2.setNumThreads(1)
    config = json.loads((OLD/'vo/original/thermal/effective-config.json').read_text())['Odometry']['frontend']['args']
    model = FlowFormerCovFrontend(asNamespace(config))
    source = MS2ManifestSequence(ROOT, ROOT/'results/pose-covariance/accuracy-001/rain/source-manifest.json',
                                'thr', thermal_transform=THERMAL_RECIPE)
    out = OUT/'packets'; out.mkdir(exist_ok=False)
    uv = grid(); rows = []
    # Match the prior runner's explicit first-depth capture and native initialization.
    model.estimate_depth(source[0].stereo); model.estimate_depth(source[0].stereo)
    try:
        for i in range(19):
            first, second = source[i].stereo, source[i+1].stereo
            _, match = model.estimate_pair(first, second)
            original, replacement = cached_pair(i)
            actual = match.flow.detach().cpu().numpy()
            if not np.array_equal(actual, original):
                raise ValueError(f'Original flow parity fails at pair {i}; stop without tolerance adjustment')
            packed = match.cov.detach().cpu().numpy()[0, :, uv[:, 1], uv[:, 0]]
            covariance = np.stack([packed[:, 0], packed[:, 2], packed[:, 2], packed[:, 1]], -1).reshape(-1, 2, 2)
            endpoints = [uv+f[0, :, uv[:, 1], uv[:, 0]] for f in [original, replacement]]
            finite = np.isfinite(covariance).all((1, 2))
            spd = np.zeros(len(uv), bool)
            spd[finite] = np.linalg.eigvalsh(covariance[finite])[:, 0] > 0
            common = finite&spd
            for points in endpoints:
                common &= np.isfinite(points).all(1)&(points >= 0).all(1)&(points < [640, 256]).all(1)
            path = out/f'{i:03}.npz'
            with path.open('xb') as stream:
                np.savez_compressed(stream, source_uv=uv, original_uv=endpoints[0], replacement_uv=endpoints[1],
                    covariance=covariance, common=common, source_K=first.K.cpu().numpy(), target_K=second.K.cpu().numpy())
            rows.append({'pair': i, 'time0_ns': first.frame_ns, 'time1_ns': second.frame_ns,
                         'common_support': int(common.sum()), 'invalid_covariance': int((~spd).sum()),
                         'original_flow_bitwise_exact': True, 'packet_sha256': sha(path)})
            print(rows[-1], flush=True)
        verify(); verify_old()
        write(OUT/'acquisition.json', {'status': 'PASS_INPUT_RECOVERY_ONLY', 'rows': rows,
              'neural_pair_calls': 19, 'initial_depth_calls': 2, 'pose_solves': 0, 'reference_reads': 0,
              'created_at': datetime.now(timezone.utc).isoformat()})
    except BaseException as exc:
        write(OUT/'acquisition-failure.json', {'error': str(exc), 'completed_pairs': len(rows),
              'action': 'STOP_INPUT_RECOVERY_WITHOUT_RETRY_OR_RESCORING'})
        raise


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['inventory', 'acquire'])
    {'inventory': inventory, 'acquire': acquire}[p.parse_args().action]()
