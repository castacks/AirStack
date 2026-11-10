"""Fixed-budget final-only stereo residual, locked before exposed-rain evaluation."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
from types import SimpleNamespace

import numpy as np
import torch
import torch.nn.functional as F

import mean_descent_contract as contract
from stereo_mean_adaptation import (ROOT, OUT as PREVIOUS, build, deny_evaluation_data,
                                    load_pair, read, setup, sha, write)

OUT = ROOT/'results/final-residual-fit/run-001'
PLAN = ROOT/'experiments/final-residual-fit-freeze-2026-11-10.json'


def verify(evaluation=False):
    plan = read(PLAN)
    pins = dict(plan['training_pins'])
    if evaluation:
        pins.update(plan['evaluation_pins'])
    for rel, digest in pins.items():
        assert sha(ROOT/rel) == digest, rel
    return plan


def fit():
    plan = verify(); setup(); contract.strict_math()
    assert read(contract.OUT/'audit.json')['status'] == 'PASS'
    dataset = read(PREVIOUS/'dataset.json')
    with deny_evaluation_data(dataset):
        caches = contract.cached_rows('cuda', torch.float32)
        head = contract.zero_head('cuda', torch.float32)
        initial = contract.total_loss(head, caches)
        steps = []; stop = None
        with (OUT/'training.jsonl').open('x') as stream:
            for i in range(plan['training']['maximum_updates']):
                try:
                    step = contract.armijo_step(head, caches)
                except ValueError as e:
                    stop = str(e)
                    break
                steps.append(step)
                stream.write(json.dumps({'update': i+1, **step})+'\n'); stream.flush()
                print(i+1, step['before'], step['after'], flush=True)
        final = contract.total_loss(head, caches)
        assert steps and final == steps[-1]['after']
        path = OUT/'final-residual.pth'
        with path.open('xb') as stream:
            torch.save({'weight': head[0].detach().cpu(), 'bias': head[1].detach().cpu(),
                        'updates': len(steps), 'model': 'final-only frozen-feature horizontal residual',
                        'base_sha256': sha(ROOT/'models/macslam/MACT_Offroad.pth')}, stream)
        # This immutable receipt is created before any evaluation process starts.
        write(OUT/'checkpoint-lock.json', {'created_at': datetime.now(timezone.utc).isoformat(),
              'checkpoint_sha256': sha(path), 'plan_sha256': sha(PLAN), 'updates': len(steps),
              'initial_loss': initial, 'final_loss': final, 'stop_reason': stop,
              'evaluation_performed': False, 'excluded_record_reads': 0,
              'zero_initialized': True, 'selected_checkpoint': 'last accepted update, no evaluation selection'})
        with torch.no_grad():
            for row in caches:
                prediction = contract.residual_prediction(head, row)[0, 0]
                valid = row['valid']; dest = OUT/'training-predictions'/f'{row["condition"]}-{row["source_id"]}.npz'
                dest.parent.mkdir(exist_ok=True)
                np.savez_compressed(dest, final_at_valid=prediction[valid].cpu().numpy())


def load_head(device):
    lock = read(OUT/'checkpoint-lock.json')
    assert lock['plan_sha256'] == sha(PLAN) and lock['evaluation_performed'] is False
    assert sha(OUT/'final-residual.pth') == lock['checkpoint_sha256']
    state = torch.load(OUT/'final-residual.pth', map_location=device, weights_only=True)
    return [state['weight'], state['bias']]


def apply_stereo(flow, captured, head):
    """Only the first (stereo) batch element and horizontal channel may change."""
    assert captured['head_calls'] == 12 and captured['upsample_calls'] == 24
    hidden = captured['hidden'][:1]; n, _, h, w = hidden.shape
    probability = captured['mask'][:1].view(n, 9, 8, 8, h, w).softmax(1)
    delta = contract.convex_upsample(F.conv2d(hidden, head[0], head[1], padding=1), probability)
    mean = flow.clone(); mean[:1, :1] += delta
    assert torch.equal(mean[:1, 1:], flow[:1, 1:])
    assert torch.equal(mean[1:], flow[1:])
    return mean


@contextmanager
def residual_frontend(arm):
    from Module.Frontend.Frontend import FlowFormerCovFrontend as Frontend
    old = Frontend.__init__, Frontend.estimate_depth, Frontend.estimate_pair
    receipts = []
    def initialize(self, cfg):
        old[0](self, cfg)
        self.final_residual = (load_head(cfg.device) if arm == 'adapted'
                               else contract.zero_head(cfg.device, torch.float32))
    @torch.inference_mode()
    def infer(self, a, b):
        with contract.capture_final(self.model) as captured:
            flow, variance = self.model.inference(a, b)
        mean = apply_stereo(flow, captured, self.final_residual)
        if arm == 'passthrough':
            assert torch.equal(mean, flow)
        receipts.append({'call': len(receipts), 'batch': len(a),
                         'stereo_mean_max_change_px': float((mean[:1, :1]-flow[:1, :1]).abs().max()),
                         'temporal_flow_exact': torch.equal(mean[1:], flow[1:]),
                         'vertical_flow_exact': torch.equal(mean[:, 1:], flow[:, 1:]),
                         'disparity_variance_source': 'original unmodified inference tensor',
                         'zero_copy_equal': torch.equal(mean, flow)})
        return flow.float(), mean.float(), variance.float()
    @torch.inference_mode()
    def depth(self, frame):
        _, mean, variance = infer(self, frame.imageL.to(self.config.device), frame.imageR.to(self.config.device))
        return self.inference_2_depth(mean, variance, frame, self.config.enforce_positive_disparity)
    @torch.inference_mode()
    def pair(self, first, second):
        a = torch.cat([second.imageL, first.imageL], 0).to(self.config.device)
        b = torch.cat([second.imageR, second.imageL], 0).to(self.config.device)
        original, mean, variance = infer(self, a, b)
        return (self.inference_2_depth(mean[:1], variance[:1], second, self.config.enforce_positive_disparity),
                self.inference_2_match(original[1:], variance[1:]))
    Frontend.__init__, Frontend.estimate_depth, Frontend.estimate_pair = initialize, depth, pair
    try:
        yield receipts
    finally:
        Frontend.__init__, Frontend.estimate_depth, Frontend.estimate_pair = old


def evaluation_start(action):
    verify(True); load_head('cpu')
    write(OUT/'evaluation-starts'/f'{action}.json', {'started_at': datetime.now(timezone.utc).isoformat(),
          'checkpoint_sha256': sha(OUT/'final-residual.pth'), 'lock_sha256': sha(OUT/'checkpoint-lock.json')})


def native(arm):
    evaluation_start('native-'+arm)
    from Scripts.Setup.ms2_execution_adapter import adapted_runner
    modality = 'rgb' if arm == 'rgb' else 'thermal'
    args = SimpleNamespace(dataset=ROOT,
        manifest=ROOT/'results/pose-covariance/accuracy-001/rain/paired-manifest.json',
        output=OUT/'vo'/arm, frames=20, start=0, acknowledge_provisional_geometry=True,
        record_uncertainty=False, record_arbiter_events=True, record_root_initialization=False)
    with adapted_runner() as runner:
        if arm in ['adapted', 'passthrough']:
            with residual_frontend(arm) as receipts:
                runner.run(args, modality)
            write(OUT/'vo'/arm/'frontend-contract.json', {'arm': arm, 'receipts': receipts,
                  'checkpoint_sha256': sha(OUT/'final-residual.pth')})
        else:
            runner.run(args, modality)


def stereo():
    evaluation_start('stereo'); setup()
    rain = next(r for r in read(PREVIOUS/'dataset.json')['records'] if r['condition'] == 'rain')
    model = build(); before = contract.state_digest(model); head = load_head('cuda'); results = []
    with torch.inference_mode():
        for row in rain['rows']:
            a, b, target, valid = load_pair(row)
            with contract.capture_final(model) as captured:
                flow, covariance = model.inference(a, b)
            mean = apply_stereo(flow, captured, head)
            assert torch.equal(apply_stereo(flow, captured, contract.zero_head('cuda', torch.float32)), flow)
            dest = OUT/'stereo-predictions'/f'{row["source_id"]}.npz'; dest.parent.mkdir(exist_ok=True)
            # Frozen features allow independent adapter reconstruction on evaluation frames.
            np.savez_compressed(dest, base_flow=flow.cpu().numpy(), adapted_flow=mean.cpu().numpy(),
                original_disparity_variance=covariance[:, :1].cpu().numpy(),
                hidden=captured['hidden'].cpu().numpy(),
                mask=captured['mask'].cpu().numpy())
            frame = {'source_id': row['source_id'], 'valid_pixels': int(valid.sum()),
                     'prediction_sha256': sha(dest), 'methods': {}}
            fb = row['K'][0][0]*row['baseline_m']; truth_z = fb/(-target[valid])
            for name, pred in [('base', flow), ('adapted', mean)]:
                px = pred[0, 0][valid]; pred_z = fb/px.abs()
                assert torch.isfinite(pred_z).all(), 'failed support; do not clip'
                e = pred_z-truth_z
                frame['methods'][name] = {'signed_flow_mean_abs_error_px': float((px-target[valid]).abs().mean()),
                    'depth_mean_abs_error_m': float(e.abs().mean()), 'depth_rmse_m': float(e.square().mean().sqrt()),
                    'depth_absrel': float((e.abs()/truth_z).mean()),
                    'nonnegative_horizontal_fraction': float((px >= 0).float().mean())}
            results.append(frame); print(frame, flush=True)
    assert before == contract.state_digest(model)
    aggregate = {m: {k: float(np.mean([r['methods'][m][k] for r in results])) for k in results[0]['methods']['base']}
                 for m in ['base', 'adapted']}
    write(OUT/'stereo-scores.json', {'recording': rain['sequence'], 'source_frames': 20, 'rows': results,
          'equal_frame_mean_metrics': aggregate, 'checkpoint_sha256': sha(OUT/'final-residual.pth'),
          'native_model_state_unchanged': True, 'zero_residual_exact_frames': 20})


def score_vo():
    # Deliberately reuse the already frozen common-frame scorer and its controls.
    import evaluate_stereo_mean as scorer
    scorer.OUT = OUT
    scorer.freeze_check = lambda: verify(True)
    scorer.score_vo()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['fit', 'stereo', 'native', 'score-vo'])
    p.add_argument('--arm', choices=['original', 'passthrough', 'adapted', 'rgb'])
    args = p.parse_args(); OUT.mkdir(parents=True, exist_ok=True)
    if args.action == 'native':
        native(args.arm)
    else:
        {'fit': fit, 'stereo': stereo, 'score-vo': score_vo}[args.action]()
