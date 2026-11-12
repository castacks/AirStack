"""One fixed exposed-rain comparison; no training or matcher threshold search."""
import argparse
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/thermal-matcher-pilot/run-001'
PLAN = ROOT/'experiments/thermal-matcher-pilot-freeze-2026-11-12.json'
sys.path.insert(0, str(ROOT/'MAC-VO-main'))


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)


def verify():
    plan = json.loads(PLAN.read_text())
    for name, digest in plan['pins'].items():
        path = Path(name) if name.startswith('/') else ROOT/name
        if sha(path) != digest:
            raise ValueError('Changed frozen input: '+name)
    return plan


def load_matcher():
    from tartanmatch import TartanMatch
    return TartanMatch.from_pretrained('/matcher/tartanmatch_v1.safetensors', device='cuda')


@contextmanager
def frontend_override(arm):
    import numpy as np
    import torch
    from Module.Frontend.Frontend import FlowFormerCovFrontend as Frontend
    from Module.Frontend.Matching import IMatcher
    original = Frontend.estimate_pair
    model = load_matcher() if arm == 'adapted' else None
    receipts = []

    @torch.inference_mode()
    def pair(self, first, second):
        depth, native = original(self, first, second)
        flow = native.flow
        if model is not None:
            result = model.predict(first.imageL, 'thermal', second.imageL, 'thermal', mixed_precision=True)
            flow = result.flow.to(device=native.flow.device, dtype=native.flow.dtype)
            if flow.shape != native.flow.shape or not torch.isfinite(flow).all():
                raise ValueError('Invalid replacement flow; stop without retry')
            path = OUT/'flow'/f'{len(receipts):03}.npz'
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('xb') as f:
                np.savez_compressed(f, original= native.flow.cpu().numpy(),
                    replacement=flow.cpu().numpy(), covisibility=result.covisibility.cpu().numpy())
        receipts.append({'pair_call': len(receipts), 'source_time_ns': first.frame_ns,
            'target_time_ns': second.frame_ns, 'flow_shape': list(flow.shape),
            'max_flow_change_px': float((flow-native.flow).abs().max()),
            'covariance_control': 'unchanged original temporal covariance; not calibrated for replacement',
            'validity_mask_control': 'unchanged native mask; no covisibility threshold',
            'depth_output_unchanged': True})
        return depth, IMatcher.Output(flow=flow, cov=native.cov, mask=native.mask)

    Frontend.estimate_pair = pair
    try:
        yield receipts
    finally:
        Frontend.estimate_pair = original


def smoke():
    import torch
    from DataLoader.Dataset.MS2 import MS2ManifestSequence, THERMAL_RECIPE
    source = MS2ManifestSequence(ROOT, ROOT/'results/pose-covariance/accuracy-001/rain/source-manifest.json',
                                'thr', thermal_transform=THERMAL_RECIPE)
    model = load_matcher()
    a, b = source[0].stereo, source[1].stereo
    result = model.predict(a.imageL, 'thermal', b.imageL, 'thermal', mixed_precision=True)
    assert result.flow.shape == (1, 2, a.height, a.width)
    assert torch.isfinite(result.flow).all()
    assert result.covisibility.shape == (1, a.height, a.width)
    assert torch.isfinite(result.covisibility).all()
    write(OUT/'smoke.json', {'status': 'PASS_INTERFACE_ONLY', 'frames': [0, 1],
        'shape': list(result.flow.shape), 'checkpoint_sha256': sha('/matcher/tartanmatch_v1.safetensors'),
        'motion_or_reference_scored': False})


def native(arm):
    verify()
    from Scripts.Setup.ms2_execution_adapter import adapted_runner
    args = SimpleNamespace(dataset=ROOT,
        manifest=ROOT/'results/pose-covariance/accuracy-001/rain/paired-manifest.json',
        output=OUT/'vo'/arm, frames=20, start=0, acknowledge_provisional_geometry=True,
        record_uncertainty=False, record_arbiter_events=True, record_root_initialization=False)
    with adapted_runner() as runner:
        if arm in ('adapted', 'passthrough'):
            with frontend_override(arm) as receipts:
                runner.run(args, 'thermal')
            write(OUT/'vo'/arm/'frontend-contract.json', {'arm': arm, 'receipts': receipts})
        else:
            runner.run(args, 'rgb' if arm == 'rgb' else 'thermal')
    verify()


def score():
    verify()
    import evaluate_stereo_mean as scorer
    scorer.OUT, scorer.freeze_check = OUT, verify
    scorer.score_vo()
    scores = json.loads((OUT/'vo-scores.json').read_text())
    base, candidate = scores['common_metrics']['original'], scores['common_metrics']['adapted']
    coverage = scores['coverage']
    checks = {'common_support_nonempty': scores['common_count'] > 0,
        'coverage_not_worse': coverage['adapted'] >= coverage['original']}
    for key in ('mean', 'rmse'):
        checks['translation_'+key+'_improves'] = candidate['translation_m'][key] < base['translation_m'][key]
        checks['rotation_'+key+'_not_worse'] = candidate['rotation_deg'][key] <= base['rotation_deg'][key]
    write(OUT/'decision.json', {'checks': checks,
        'decision': 'ADVANCE_TO_SEPARATE_RECORDING_COMPARISON' if all(checks.values()) else 'STOP_THIS_CONFIGURATION',
        'scope': 'exposed development prefix; nominal supplied-reference motion, no direct flow truth or generalization'})


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['smoke', 'native', 'score'])
    p.add_argument('--arm', choices=['original', 'passthrough', 'adapted', 'rgb'])
    args = p.parse_args()
    if args.action == 'native':
        if args.arm is None:
            p.error('--arm required for native')
        native(args.arm)
    else:
        {'smoke': smoke, 'score': score}[args.action]()
