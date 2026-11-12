"""Fixed100-frame three-record test: thermal replacement flow plus unweighted NEC."""
import argparse
from contextlib import contextmanager
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import run_bearing_rotation_assay as core

ROOT = core.ROOT
BASE = ROOT/'results/unweighted-longer/run-001'
FREEZE = ROOT/'experiments/unweighted-longer-freeze-2026-11-12.json'
FRAMES = 100
CASES = {'day': ('ms2-day-replication', '_2021-08-06-11-23-45'),
         'night': ('ms2-night-pilot', '_2021-08-13-21-18-04'),
         'rain': ('pose-covariance/accuracy-001/rain', '_2021-08-06-16-19-00')}


def verify():
    plan = json.loads(FREEZE.read_text())
    for name, digest in plan['pins'].items():
        if core.sha(ROOT/name) != digest:
            raise ValueError('Changed frozen transfer input: '+name)
    return plan


def prepare():
    import cv2
    from copy import deepcopy
    from Scripts.Setup.ms2_execution_adapter import prepare as prepare_paired
    for condition, (folder, sequence) in CASES.items():
        old = ROOT/'results'/folder
        archived = json.loads((ROOT/'results/droid-longer-prefix/run-001'/f'{condition}-source-manifest.json').read_text())
        zero_path = (ROOT/'results/bearing-rotation-transfer/run-001/inputs'/condition/'source-manifest.json') if condition != 'rain' else old/'source-manifest.json'
        zero = json.loads(zero_path.read_text())['records'][0]
        if archived['sequence'] != sequence:
            raise ValueError('Wrong archived recording')
        records = deepcopy(archived['records'][:FRAMES])
        for i, row in enumerate(records):
            for sensor in ['rgb', 'thr']:
                for side in ['left', 'right']:
                    record = row['cameras'][sensor][side]
                    if i == 0:
                        record['path'] = zero['cameras'][sensor][side]['path']
                    path = ROOT/record['path']
                    if core.sha(path) != record['sha256']:
                        raise ValueError('Archived raw image mismatch')
                    image = cv2.imread(str(path), -1)
                    if image is None or list(image.shape) != record['shape'] or str(image.dtype) != record['dtype']:
                        raise ValueError('Source image schema mismatch')
        target = BASE/'inputs'/condition
        core.write(target/'source-manifest.json', {'sequence': sequence, 'records': records,
            'scope': 'Existing exposed first100 original groups; exact archived pixels/timestamps, no physical synchronization claim'})
        prepare_paired(ROOT, target/'source-manifest.json', target/'paired-manifest.json')
        refs = [f'results/provided-reference-full-record/run-001/reference/{sequence}/rgb/{i:06}.txt' for i in range(FRAMES)]
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
        frames=FRAMES, start=0, acknowledge_provisional_geometry=True, record_uncertainty=False,
        record_arbiter_events=True, record_root_initialization=False)
    Frontend.estimate_pair = observe
    try:
        with adapted_runner() as runner:
            runner.run(args, 'thermal')
    finally:
        Frontend.estimate_pair = original
    if len(rows) != FRAMES-1:
        raise ValueError('Unexpected pair-call count')
    core.write(target/'acquisition.json', {'condition': condition, 'rows': rows,
        'native_pair_calls': FRAMES-1, 'replacement_matcher_calls': FRAMES-1, 'reference_values_used': False})
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


def read_case(condition):
    metadata = json.loads((BASE/'inputs'/condition/'metadata.json').read_text())
    source = json.loads((ROOT/metadata['source_manifest']).read_text())
    if (metadata['condition'] != condition or metadata['sequence'] != CASES[condition][1]
            or source['sequence'] != metadata['sequence']
            or [r['stem'] for r in source['records']] != [f'{i:06}' for i in range(FRAMES)]):
        raise ValueError('Recording identity or exact original prefix changed')
    return metadata, source


def capture_rgb(condition):
    verify(); read_case(condition)
    from Scripts.Setup.ms2_execution_adapter import adapted_runner
    args = SimpleNamespace(dataset=ROOT, manifest=BASE/'inputs'/condition/'paired-manifest.json',
        output=BASE/'native'/condition, frames=FRAMES, start=0, acknowledge_provisional_geometry=True,
        record_uncertainty=False, record_arbiter_events=True, record_root_initialization=False)
    with adapted_runner() as runner: runner.run(args, 'rgb')
    verify()


def predict(condition):
    verify_condition(condition)
    core.cv2.setNumThreads(1)
    import bearing_rotation as br
    target = BASE/'inputs'/condition; out = BASE/'assay'/condition; out.mkdir(parents=True, exist_ok=False)
    rows = []; calls = 0
    with (out/'journal.jsonl').open('x') as log:
        for item in json.loads((target/'acquisition.json').read_text())['rows']:
            i = item['pair']
            with np.load(target/'packets'/f'{i:03}.npz', allow_pickle=False) as z: packet = {k: z[k] for k in z.files}
            selected, masks, status = core.filter_packet(packet)
            path = out/'support'/f'{i:03}.npz'; path.parent.mkdir(exist_ok=True)
            with path.open('xb') as f: np.savez_compressed(f, common=selected, original=masks['original'], replacement=masks['replacement'])
            if int(selected.sum()) < 8 or any(v != 'OK' for v in status.values()):
                result = {'status': 'UNAVAILABLE_COMMON_FILTER', 'rotation': None, 'translation_direction': None}
            else:
                try:
                    f0 = br.bearings(packet['source_uv'][selected], packet['source_K'].reshape(3, 3))[0]
                    f1 = br.bearings(packet['replacement_uv'][selected], packet['target_K'].reshape(3, 3))[0]
                    calls += 1; result = br.solve(f0, f1, None)
                except (ValueError, np.linalg.LinAlgError) as exc:
                    result = {'status': 'NUMERICAL_FAILURE', 'reason': str(exc), 'rotation': None, 'translation_direction': None}
            row = {'pair': i, 'time0_ns': item['time0_ns'], 'time1_ns': item['time1_ns'],
                'before_filter': int(packet['common'].sum()), 'after_filter': int(selected.sum()),
                'filter_status': status, 'support_sha256': core.sha(path), 'candidate': core.safe(result)}
            rows.append(row); log.write(json.dumps(row)+'\n'); log.flush()
            print(condition, i, result['status'], flush=True)
    verify_condition(condition)
    core.write(out/'predictions.json', {'rows': rows, 'solver_calls': calls, 'reference_values_used': False})
    pins = {str(p.relative_to(out)): core.sha(p) for p in out.rglob('*') if p.is_file()}
    core.write(out/'prediction-lock.json', {'execution_freeze_sha256': core.sha(FREEZE), 'pins': pins})


def lock_predictions():
    for condition in CASES: verify_condition(condition)
    core.write(BASE/'global-prediction-lock.json', {c: core.sha(BASE/'assay'/c/'prediction-lock.json') for c in CASES})


def gate(errors, pair_ids):
    populations = {k: set(v)&set(pair_ids) for k, v in errors.items()}
    common = sorted(set.intersection(*populations.values()))
    metrics = {k: core.statistics([v[i] for i in common]) for k, v in errors.items()}
    checks = {'nonempty_common': bool(common),
        'no_RGB_coverage_loss': populations['native_rgb'] <= populations['candidate'],
        'no_thermal_coverage_loss': populations['native_thermal'] <= populations['candidate']}
    for control in ['native_rgb', 'native_thermal']:
        for metric in ['mean', 'rmse']:
            checks['improves_'+control+'_'+metric] = bool(common) and metrics['candidate'][metric] < metrics[control][metric]
    return {'checks': checks, 'pass': all(checks.values()), 'matched_pair_ids': common,
            'rotation_deg': metrics, 'coverage': {k: len(v) for k, v in populations.items()}, 'expected_intervals': len(pair_ids)}


def score(condition):
    verify_condition(condition)
    from pose_covariance_accuracy import transforms, project
    from pose_covariance_replay import pose_xyzw
    from score_pose_covariance_accuracy import reference_pose
    metadata, source = read_case(condition); out = BASE/'assay'/condition
    global_lock = json.loads((BASE/'global-prediction-lock.json').read_text())
    for c in CASES:
        if global_lock[c] != core.sha(BASE/'assay'/c/'prediction-lock.json'): raise ValueError('Changed global prediction lock')
    for name, digest in json.loads((out/'prediction-lock.json').read_text())['pins'].items():
        if core.sha(out/name) != digest: raise ValueError('Changed prediction')
    frames = transforms(json.loads((ROOT/metadata['calibration']).read_text()),
        ROOT/'results/pose-covariance/cohort-preflight/snapshot/ore-ms/results/provided-reference-full-record/run-001/source-001/official-loader.py')
    poses = [reference_pose(ROOT/p) for p in metadata['reference_rgb_poses']]
    events = {}
    for mode in ['rgb', 'thermal']:
        values = [json.loads(s) for s in (BASE/'native'/condition/mode/'arbiter-events/events.jsonl').read_text().splitlines()]
        events[mode] = {(e['from_idx'], e['frame_idx']): e for e in values if e['kind'] == 'motion_result_written' and e['first_write_for_submission']}
    errors = {k: {} for k in ['candidate', 'native_rgb', 'native_thermal']}; rows = []
    for row in json.loads((out/'predictions.json').read_text())['rows']:
        i = row['pair']; common = [source['records'][j]['cameras']['rgb']['left']['recorded_time_ns'] for j in [i, i+1]]
        ref = None if poses[i][0] is None or poses[i+1][0] is None else np.linalg.solve(poses[i][0], poses[i+1][0])[:3, :3]
        predicted = row['candidate']; mapped = core.mapped_rotation(predicted['rotation'], frames['thermal'],
            [row['time0_ns'], row['time1_ns']], common) if predicted['status'] == 'CONVERGED' else None
        entries = {'candidate': {'mapped_rotation': core.safe(mapped), 'error_deg': core.rotation_error(mapped, ref) if mapped is not None and ref is not None else None}}
        for mode in ['rgb', 'thermal']:
            e = events[mode].get((i, i+1)); mapped = None; sensor = 'rgb' if mode == 'rgb' else 'thr'
            native_times = [source['records'][j]['cameras'][sensor]['left']['recorded_time_ns'] for j in [i, i+1]]
            if e is not None and e['derived_camera_delta_xyzw'] is not None:
                if [e['time0_ns'], e['time1_ns']] != native_times: raise ValueError('Native/source interval mismatch')
                value = project(pose_xyzw(e['derived_camera_delta_xyzw']), None, frames[mode], native_times, common)
                if value['valid']: mapped = value['transform'][:3, :3]
            entries['native_'+mode] = {'mapped_rotation': core.safe(mapped),
                'error_deg': core.rotation_error(mapped, ref) if mapped is not None and ref is not None else None}
        for key, entry in entries.items():
            if entry['error_deg'] is not None: errors[key][i] = entry['error_deg']
        rows.append({'pair': i, 'reference_rotation': core.safe(ref), 'entries': entries})
    result = {'condition': condition, 'rows': rows, 'primary_extension': gate(errors, list(range(19, FRAMES-1))),
        'full_prefix': gate(errors, list(range(FRAMES-1))), 'reference_qualification': [v[1] for v in poses],
        'scope': 'Rotation-only previously exposed100frame development prefix; primary added80intervals, nativeRGB/thermal controls'}
    core.write(out/'scores.json', result); print(condition, result['primary_extension'], flush=True)


def overall(recordings):
    passed = {c: recordings[c]['primary_extension']['pass'] for c in CASES}
    return {'recording_gates': passed, 'decision': 'ADVANCE_TO_SEPARATE_FULL_MOTION_DESIGN' if all(passed.values()) else 'CLOSE_UNWEIGHTED_ROUTE_NO_RETUNING'}


def summarize():
    verify()
    result = overall({c: json.loads((BASE/'assay'/c/'scores.json').read_text()) for c in CASES})
    core.write(BASE/'summary.json', result); print(result, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'capture-rgb', 'capture', 'predict', 'lock', 'score', 'summarize'])
    parser.add_argument('--condition', choices=list(CASES))
    args = parser.parse_args()
    if args.action in ['prepare', 'lock', 'summarize']:
        {'prepare': prepare, 'lock': lock_predictions, 'summarize': summarize}[args.action]()
    elif args.condition is None: parser.error('--condition required')
    else: {'capture-rgb': capture_rgb, 'capture': capture, 'predict': predict, 'score': score}[args.action](args.condition)


