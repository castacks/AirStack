"""Fixed-checkpoint stereo and native VO evaluation; no fitting or selection."""
import argparse
from contextlib import contextmanager
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import torch

from stereo_mean_adaptation import (ROOT, OUT, build, freeze_check, load_mean_head,
                                    load_pair, read, setup, sha, write)


@contextmanager
def mean_frontend(arm):
    from Module.Frontend.Frontend import FlowFormerCovFrontend as Frontend
    old = Frontend.__init__, Frontend.estimate_depth, Frontend.estimate_pair
    receipts = []
    def initialize(self, cfg):
        old[0](self, cfg)
        self.stereo_mean_model = deepcopy(self.model)
        if arm == 'adapted':
            load_mean_head(self.stereo_mean_model)
        self.stereo_mean_model.eval()
    @torch.inference_mode()
    def infer(self, a, b, initial):
        flow, variance = self.model.inference(a, b)
        mean, _ = self.stereo_mean_model.inference(a, b)
        if arm == 'passthrough':
            assert torch.equal(flow, mean), 'zero-change copy does not preserve native output'
        receipts.append({'call': len(receipts), 'initial': initial,
                         'stereo_mean_max_change_px': float((mean[0:1]-flow[0:1]).abs().max()),
                         'temporal_flow_source': 'original_model', 'disparity_variance_source': 'original_model',
                         'zero_copy_equal': bool(torch.equal(flow, mean))})
        return flow.float(), mean.float(), variance.float()
    @torch.inference_mode()
    def depth(self, frame):
        a, b = frame.imageL.to(self.config.device), frame.imageR.to(self.config.device)
        _, mean, variance = infer(self, a, b, True)
        return self.inference_2_depth(mean, variance, frame, self.config.enforce_positive_disparity)
    @torch.inference_mode()
    def pair(self, first, second):
        a = torch.cat([second.imageL, first.imageL], 0).to(self.config.device)
        b = torch.cat([second.imageR, second.imageL], 0).to(self.config.device)
        original, mean, variance = infer(self, a, b, False)
        return (self.inference_2_depth(mean[0:1], variance[0:1], second, self.config.enforce_positive_disparity),
                self.inference_2_match(original[1:2], variance[1:2]))
    Frontend.__init__, Frontend.estimate_depth, Frontend.estimate_pair = initialize, depth, pair
    try:
        yield receipts
    finally:
        Frontend.__init__, Frontend.estimate_depth, Frontend.estimate_pair = old


def native(arm):
    freeze_check()
    if arm in ['adapted', 'passthrough']:
        assert read(OUT/'checkpoint-lock.json')['evaluation_performed'] is False
    from Scripts.Setup.ms2_execution_adapter import adapted_runner
    modality = 'rgb' if arm == 'rgb' else 'thermal'
    args = SimpleNamespace(dataset=ROOT,
        manifest=ROOT/'results/pose-covariance/accuracy-001/rain/paired-manifest.json',
        output=OUT/'vo'/arm, frames=20, start=0, acknowledge_provisional_geometry=True,
        record_uncertainty=False, record_arbiter_events=True, record_root_initialization=False)
    with adapted_runner() as runner:
        if arm in ['adapted', 'passthrough']:
            with mean_frontend(arm) as receipts:
                runner.run(args, modality)
            write(OUT/'vo'/arm/'frontend-contract.json', {'arm': arm, 'receipts': receipts,
                  'mean_checkpoint_sha256': read(OUT/'checkpoint-lock.json')['checkpoint_sha256']})
        else:
            runner.run(args, modality)


def stereo():
    freeze_check(); setup()
    dataset = read(OUT/'dataset.json'); rain = next(r for r in dataset['records'] if r['condition'] == 'rain')
    base = build(); adapted = deepcopy(base); load_mean_head(adapted)
    results = []
    with torch.inference_mode():
        for row in rain['rows']:
            a, b, target, valid = load_pair(row)
            flow, covariance = base.inference(a, b); mean, _ = adapted.inference(a, b)
            # Keep predictions for independent scoring on the same label mask.
            dest = OUT/'stereo-predictions'/f'{row["source_id"]}.npz'; dest.parent.mkdir(exist_ok=True)
            np.savez_compressed(dest, base_flow=flow.cpu().numpy(), adapted_flow=mean.cpu().numpy(),
                                original_disparity_variance=covariance[:, :1].cpu().numpy())
            frame = {'source_id': row['source_id'], 'valid_pixels': int(valid.sum()), 'prediction_sha256': sha(dest), 'methods': {}}
            fb = row['K'][0][0]*row['baseline_m']; truth_z = fb/(-target[valid])
            for name, pred in [('base', flow), ('adapted', mean)]:
                px = pred[0, 0][valid]; pred_z = fb/px.abs()
                if not torch.isfinite(pred_z).all():
                    raise ValueError('nonfinite evaluation depth; report failed support instead of clipping')
                e = pred_z-truth_z
                frame['methods'][name] = {'signed_flow_mean_abs_error_px': float((px-target[valid]).abs().mean()),
                    'depth_mean_abs_error_m': float(e.abs().mean()), 'depth_rmse_m': float(e.square().mean().sqrt()),
                    'depth_absrel': float((e.abs()/truth_z).mean()), 'nonnegative_horizontal_fraction': float((px >= 0).float().mean())}
            results.append(frame); print(frame, flush=True)
    keys = results[0]['methods']['base']
    aggregate = {m: {k: float(np.mean([r['methods'][m][k] for r in results])) for k in keys} for m in ['base', 'adapted']}
    write(OUT/'stereo-scores.json', {'recording': rain['sequence'], 'source_frames': 20, 'rows': results,
          'equal_frame_mean_metrics': aggregate, 'checkpoint_sha256': read(OUT/'checkpoint-lock.json')['checkpoint_sha256']})


def score_vo():
    from pose_covariance_accuracy import transforms, project
    from pose_covariance_replay import pose_xyzw
    from score_pose_covariance_accuracy import error, statistics
    import json
    freeze_check()
    old = read(ROOT/'results/pose-covariance/accuracy-001/predictions-v2/rain.json')
    references = {r['pair']: r['reference_delta'] for r in read(ROOT/'results/native-information-weights/run-001/scores-rain.json')['rows']}
    calibration = read(ROOT/'results/pose-covariance/accuracy-001/rain/calibration.json')
    loader = ROOT/'results/pose-covariance/cohort-preflight/snapshot/ore-ms/results/provided-reference-full-record/run-001/source-001/official-loader.py'
    frames = transforms(calibration, loader)
    arms = ['original', 'passthrough', 'adapted', 'rgb']
    methods = {}
    parity = {}
    for arm in arms:
        camera = 'rgb' if arm == 'rgb' else 'thermal'
        capture = OUT/'vo'/arm/camera
        assert read(capture/'report.json')['status'] == 'PASS_EXECUTION_ONLY'
        events = [json.loads(s) for s in (capture/'arbiter-events/events.jsonl').read_text().splitlines()]
        writes = {(e['from_idx'], e['frame_idx']): e for e in events if e['kind'] == 'motion_result_written' and e['first_write_for_submission']}
        result = []
        for i in range(19):
            e = writes.get((i, i+1)); transform = None
            if e is not None and e['derived_camera_delta_xyzw'] is not None:
                local = pose_xyzw(e['derived_camera_delta_xyzw'])
                converted = project(local, None, frames[camera], [e['time0_ns'], e['time1_ns']], old['rows'][i]['common_interval_ns'])
                if converted['valid']:
                    transform = converted['transform'].tolist()
            result.append({'pair': i, 'transform': transform, 'errors': error(transform, references[i]) if transform is not None else None})
        methods[arm] = result
    for i in range(19):
        a, b = methods['original'][i]['transform'], methods['passthrough'][i]['transform']
        np.testing.assert_array_equal(a, b)
    parity['zero_change_native_intervals_exact'] = 19
    common = [i for i in range(19) if all(methods[m][i]['errors'] is not None for m in arms)]
    metrics = {m: {k: statistics([methods[m][i]['errors'][k] for i in common]) for k in ['rotation_deg', 'translation_m']} for m in arms}
    write(OUT/'vo-scores.json', {'expected_intervals': 19, 'common_count': len(common), 'common_pair_ids': common,
          'coverage': {m: sum(r['errors'] is not None for r in rows) for m, rows in methods.items()},
          'common_metrics': metrics, 'parity': parity, 'rows_by_arm': methods,
          'reference_frame_and_time_scope': 'inherited nominal common RGB frame and constant-twist interval mapping; no reference fit',
          'runtime_metrics': False})
    print(metrics)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['stereo', 'native', 'score-vo'])
    parser.add_argument('--arm', choices=['original', 'passthrough', 'adapted', 'rgb'])
    args = parser.parse_args()
    if args.action == 'native':
        native(args.arm)
    elif args.action == 'stereo':
        stereo()
    else:
        score_vo()
