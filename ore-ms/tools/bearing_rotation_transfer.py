"""Unchanged rain assay applied once to fixed exposed day/night stereo pilots."""
import argparse
from contextlib import contextmanager
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import run_bearing_rotation_assay as core

ROOT = core.ROOT
BASE = ROOT/'results/bearing-rotation-transfer/run-001'
FREEZE = ROOT/'experiments/bearing-rotation-transfer-freeze-2026-11-12.json'
CASES = {'day': ('ms2-day-replication', '_2021-08-06-11-23-45'),
         'night': ('ms2-night-pilot', '_2021-08-13-21-18-04')}


def verify():
    plan = json.loads(FREEZE.read_text())
    for name, digest in plan['pins'].items():
        if core.sha(ROOT/name) != digest:
            raise ValueError('Changed frozen transfer input: '+name)
    return plan


def prepare():
    import cv2
    from Scripts.Setup.ms2_execution_adapter import prepare as prepare_paired
    for condition, (folder, sequence) in CASES.items():
        old = ROOT/'results'/folder
        calibration = json.loads((old/'calibration.json').read_text())
        raw = old/'pilot/sync_data'/sequence
        times = {(sensor, side): [int(v) for v in (raw/sensor/f'img_{side}_timestamp.txt').read_text().split()]
                 for sensor in ['rgb', 'thr'] for side in ['left', 'right']}
        records = []
        for i in range(20):
            row = {'stem': f'{i:06}', 'cameras': {}}
            for sensor in ['rgb', 'thr']:
                camera = {'K': calibration['K_'+sensor+'L']['values'],
                          'baseline_m': float(-np.asarray(calibration['T_'+sensor+'R']['values']).reshape(-1)[0]/1000)}
                for side in ['left', 'right']:
                    path = raw/sensor/('img_'+side)/(row['stem']+'.png')
                    image = cv2.imread(str(path), -1)
                    if image is None:
                        raise ValueError('Missing source image')
                    camera[side] = {'path': str(path.relative_to(ROOT)), 'sha256': core.sha(path),
                        'shape': list(image.shape), 'dtype': str(image.dtype), 'recorded_time_ns': times[sensor, side][i]}
                row['cameras'][sensor] = camera
            records.append(row)
        target = BASE/'inputs'/condition
        core.write(target/'source-manifest.json', {'sequence': sequence, 'records': records,
            'scope': 'Existing exposed first20 original groups; exact archived pixels/timestamps, no physical synchronization claim'})
        prepare_paired(ROOT, target/'source-manifest.json', target/'paired-manifest.json')
        refs = [str((old/'pilot/odom'/sequence/'rgb'/f'{i:06}.txt').relative_to(ROOT)) for i in range(20)]
        core.write(target/'metadata.json', {'condition': condition, 'sequence': sequence,
             'calibration': str((old/'calibration.json').relative_to(ROOT)), 'reference_rgb_poses': refs,
             'source_manifest': str((target/'source-manifest.json').relative_to(ROOT)),
             'reference_convention': 'Provided RGB camera-to-initial poses, source IDs and actual RGB left times'})


def capture(condition):
    verify()
    read_case(condition)
    import torch
    from Module.Frontend.Frontend import FlowFormerCovFrontend as Frontend
    from Scripts.Setup.ms2_execution_adapter import adapted_runner
    from thermal_matcher_pilot import load_matcher
    from prepare_bearing_rotation import grid
    model = load_matcher()
    uv = grid(); rows = []
    original = Frontend.estimate_pair
    target = BASE/'inputs'/condition
    packets = target/'packets'; packets.mkdir(exist_ok=False)
    @torch.inference_mode()
    def observe(self, first, second):
        depth, match = original(self, first, second)
        replacement = model.predict(first.imageL, 'thermal', second.imageL, 'thermal', mixed_precision=True)
        original_flow = match.flow.detach().cpu().numpy()
        new_flow = replacement.flow.detach().cpu().numpy()
        if original_flow.shape != new_flow.shape or not np.isfinite(new_flow).all():
            raise ValueError('Invalid replacement output; stop without retry')
        packed = match.cov.detach().cpu().numpy()[0, :, uv[:, 1], uv[:, 0]]
        covariance = np.stack([packed[:, 0], packed[:, 2], packed[:, 2], packed[:, 1]], -1).reshape(-1, 2, 2)
        endpoints = [uv+f[0, :, uv[:, 1], uv[:, 0]] for f in [original_flow, new_flow]]
        finite = np.isfinite(covariance).all((1, 2)); spd = np.zeros(len(uv), bool)
        spd[finite] = np.linalg.eigvalsh(covariance[finite])[:, 0] > 0
        common = spd.copy()
        for points in endpoints:
            common &= np.isfinite(points).all(1)&(points >= 0).all(1)&(points < [640, 256]).all(1)
        i = len(rows); path = packets/f'{i:03}.npz'
        with path.open('xb') as f:
            np.savez_compressed(f, source_uv=uv, original_uv=endpoints[0], replacement_uv=endpoints[1],
                covariance=covariance, common=common, source_K=first.K.cpu().numpy(), target_K=second.K.cpu().numpy())
        rows.append({'pair': i, 'time0_ns': first.frame_ns, 'time1_ns': second.frame_ns,
            'common_support': int(common.sum()), 'invalid_covariance': int((~spd).sum()), 'packet_sha256': core.sha(path),
            'native_depth_and_match_returned_unmodified': True})
        return depth, match
    args = SimpleNamespace(dataset=ROOT, manifest=target/'paired-manifest.json', output=BASE/'native'/condition,
        frames=20, start=0, acknowledge_provisional_geometry=True, record_uncertainty=False,
        record_arbiter_events=True, record_root_initialization=False)
    Frontend.estimate_pair = observe
    try:
        with adapted_runner() as runner:
            runner.run(args, 'thermal')
    finally:
        Frontend.estimate_pair = original
    if len(rows) != 19:
        raise ValueError('Unexpected pair-call count')
    core.write(target/'acquisition.json', {'condition': condition, 'rows': rows,
        'native_pair_calls': 19, 'replacement_matcher_calls': 19, 'reference_values_used': False})
    verify()
    pins = {str(p.relative_to(BASE)): core.sha(p) for folder in [packets, BASE/'native'/condition]
            for p in folder.rglob('*') if p.is_file()}
    pins[str((target/'acquisition.json').relative_to(BASE))] = core.sha(target/'acquisition.json')
    core.write(target/'input-lock.json', {'execution_freeze_sha256': core.sha(FREEZE), 'pins': pins})


def verify_condition(condition):
    verify()
    lock = json.loads((BASE/'inputs'/condition/'input-lock.json').read_text())
    if lock['execution_freeze_sha256'] != core.sha(FREEZE):
        raise ValueError('Changed execution freeze')
    for name, digest in lock['pins'].items():
        if core.sha(BASE/name) != digest:
            raise ValueError('Changed acquired packet/native control: '+name)


@contextmanager
def binding(condition):
    old = core.OUT, core.INPUT, core.verify, core.FREEZE
    core.OUT, core.INPUT, core.verify, core.FREEZE = BASE/'assay'/condition, BASE/'inputs'/condition, lambda: verify_condition(condition), FREEZE
    try:
        yield
    finally:
        core.OUT, core.INPUT, core.verify, core.FREEZE = old


def predict(condition):
    verify_condition(condition)
    with binding(condition):
        core.predict()


def read_case(condition):
    metadata = json.loads((BASE/'inputs'/condition/'metadata.json').read_text())
    source = json.loads((ROOT/metadata['source_manifest']).read_text())
    if (metadata['condition'] != condition or metadata['sequence'] != CASES[condition][1]
            or source['sequence'] != metadata['sequence']
            or [r['stem'] for r in source['records']] != [f'{i:06}' for i in range(20)]):
        raise ValueError('Recording identity or exact original prefix changed')
    return metadata, source


def reference_data(condition):
    from pose_covariance_accuracy import transforms
    from score_pose_covariance_accuracy import reference_pose
    metadata, source = read_case(condition)
    frame = transforms(json.loads((ROOT/metadata['calibration']).read_text()),
        ROOT/'results/pose-covariance/cohort-preflight/snapshot/ore-ms/results/provided-reference-full-record/run-001/source-001/official-loader.py')['thermal']
    qualified = [reference_pose(ROOT/p) for p in metadata['reference_rgb_poses']]
    refs = {i: None if qualified[i][0] is None or qualified[i+1][0] is None else
            np.linalg.solve(qualified[i][0], qualified[i+1][0])[:3, :3] for i in range(19)}
    clocks = [[source['records'][i]['cameras']['rgb']['left']['recorded_time_ns'],
               source['records'][i+1]['cameras']['rgb']['left']['recorded_time_ns']] for i in range(19)]
    return frame, refs, clocks, [v[1] for v in qualified]


def score(condition):
    verify_condition(condition)
    from pose_covariance_replay import pose_xyzw
    from pose_covariance_accuracy import project
    out = BASE/'assay'/condition
    global_lock = json.loads((BASE/'global-prediction-lock.json').read_text())
    for case in CASES:
        if global_lock[case] != core.sha(BASE/'assay'/case/'prediction-lock.json'):
            raise ValueError('Changed global prediction lock')
    lock = json.loads((out/'prediction-lock.json').read_text())
    if lock['execution_freeze_sha256'] != core.sha(FREEZE):
        raise ValueError('Changed execution freeze')
    for name, digest in lock['pins'].items():
        if core.sha(out/name) != digest:
            raise ValueError('Changed prediction')
    frame, refs, clocks, qualification = reference_data(condition)
    events = [json.loads(line) for line in (BASE/'native'/condition/'thermal/arbiter-events/events.jsonl').read_text().splitlines()]
    native = {(e['from_idx'], e['frame_idx']): e for e in events
              if e['kind'] == 'motion_result_written' and e['first_write_for_submission']}
    errors = {m+'/'+o: {} for m in core.MEANS for o in core.OBJECTIVES}; errors['native'] = {}
    scored = []
    for row in json.loads((out/'predictions.json').read_text())['rows']:
        i = row['pair']; entries = {}
        for mean in core.MEANS:
            for objective in core.OBJECTIVES:
                key = mean+'/'+objective; value = row['methods'][mean][objective]
                rotation = None; reason = value['status']
                if value['status'] == 'CONVERGED':
                    rotation = core.mapped_rotation(value['rotation'], frame, [row['time0_ns'], row['time1_ns']], clocks[i])
                    if rotation is None: reason = 'HORIZON_EXCEEDED'
                e = core.rotation_error(rotation, refs[i]) if rotation is not None and refs[i] is not None else None
                if e is not None: errors[key][i] = e
                entries[key] = {'reason': reason, 'mapped_rotation': core.safe(rotation), 'error_deg': e}
        event = native.get((i, i+1)); rotation = None
        if event is not None and event['derived_camera_delta_xyzw'] is not None:
            if [event['time0_ns'], event['time1_ns']] != [row['time0_ns'], row['time1_ns']]:
                raise ValueError('Native/source interval mismatch')
            value = project(pose_xyzw(event['derived_camera_delta_xyzw']), None, frame,
                [event['time0_ns'], event['time1_ns']], clocks[i])
            if value['valid']: rotation = value['transform'][:3, :3]
        e = core.rotation_error(rotation, refs[i]) if rotation is not None and refs[i] is not None else None
        if e is not None: errors['native'][i] = e
        entries['native'] = {'mapped_rotation': core.safe(rotation), 'error_deg': e}
        scored.append({'pair': i, 'reference_rotation': core.safe(refs[i]), 'entries': entries})
    result = {'condition': condition, 'rows': scored, 'primary': core.primary_decision(errors),
        'reference_qualification': qualification, 'coverage': {k: len(v) for k, v in errors.items()},
        'own_support_rotation_deg': {k: core.statistics(list(v.values())) for k, v in errors.items()},
        'scope': 'Existing exposed development pilot, nominal common-frame/time mapping, rotation only'}
    for mean in core.MEANS:
        keys = [mean+'/'+o for o in core.OBJECTIVES]+['native']
        common = sorted(set.intersection(*(set(errors[k]) for k in keys)))
        result[mean+'_all_method_common'] = {'pairs': common,
            'rotation_deg': {k: core.statistics([errors[k][i] for i in common]) for k in keys}}
    core.write(out/'scores.json', result)
    print(condition, result['primary'], flush=True)


def lock_predictions():
    for condition in CASES:
        verify_condition(condition)
    core.write(BASE/'global-prediction-lock.json', {condition: core.sha(BASE/'assay'/condition/'prediction-lock.json') for condition in CASES})


def transfer_decision(recordings):
    passed = {c: all(recordings[c]['primary']['checks'].values()) for c in CASES}
    return {'recording_gates': passed, 'decision': 'DAY_NIGHT_DEVELOPMENT_GATES_PASS' if all(passed.values())
            else 'STOP_TRANSFER_PROMOTION_NO_RETUNING', 'scope': 'Small exposed development prefixes; no full-VO or training-generalization claim'}


def summarize():
    verify()
    recordings = {c: json.loads((BASE/'assay'/c/'scores.json').read_text()) for c in CASES}
    result = transfer_decision(recordings)
    result['primary_by_recording'] = {c: r['primary'] for c, r in recordings.items()}
    core.write(BASE/'summary.json', result)
    print(result, flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['prepare', 'capture', 'predict', 'score', 'lock', 'summarize'])
    p.add_argument('--condition', choices=list(CASES))
    args = p.parse_args()
    if args.action == 'prepare': prepare()
    elif args.action == 'lock': lock_predictions()
    elif args.action == 'summarize': summarize()
    elif args.condition is None: p.error('--condition required')
    else: {'capture': capture, 'predict': predict, 'score': score}[args.action](args.condition)
