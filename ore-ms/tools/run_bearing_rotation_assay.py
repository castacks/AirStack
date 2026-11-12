"""Single frozen rain rotation assay; no training or full-VO integration."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import bearing_rotation as br

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/bearing-rotation-assay/run-001'
INPUT = ROOT/'results/bearing-rotation-preflight/run-001'
OLD = ROOT/'results/thermal-matcher-pilot/run-001'
FREEZE = ROOT/'experiments/bearing-rotation-execution-freeze-2026-11-12.json'
MEANS = ('original', 'replacement')
OBJECTIVES = ('NEC', 'PNEC_unit_pixel_covariance', 'PNEC_original_pixel_covariance')
Q = np.array([[0., 1, 0], [0, 0, 1], [1, 0, 0]])


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(v) for v in value]
    return value


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as f:
        json.dump(safe(value), f, indent=2, allow_nan=False)


def verify():
    plan = json.loads(FREEZE.read_text())
    for name, digest in plan['pins'].items():
        if sha(ROOT/name) != digest:
            raise ValueError('Changed frozen input: '+name)
    return plan


def essential_mask(source, target, source_k, target_k):
    source, target = np.asarray(source, float), np.asarray(target, float)
    if len(source) < 8:
        return 'INSUFFICIENT_POINTS', np.zeros(len(source), bool)
    if not np.allclose(source_k, target_k, rtol=0, atol=1e-12):
        return 'UNSUPPORTED_INTRINSIC_CHANGE', np.zeros(len(source), bool)
    cv2.setRNGSeed(0)
    try:
        essential, mask = cv2.findEssentialMat(source, target, cameraMatrix=source_k,
            method=cv2.RANSAC, prob=.999, threshold=1., maxIters=5000)
    except cv2.error:
        return 'ESSENTIAL_FILTER_ERROR', np.zeros(len(source), bool)
    if essential is None or mask is None:
        return 'ESSENTIAL_UNAVAILABLE', np.zeros(len(source), bool)
    if essential.shape != (3, 3):
        return 'MULTIPLE_ESSENTIAL_SOLUTIONS', np.zeros(len(source), bool)
    if not np.isfinite(essential).all() or mask.size != len(source):
        return 'INVALID_ESSENTIAL_RESULT', np.zeros(len(source), bool)
    return 'OK', mask.reshape(-1).astype(bool)


def filter_packet(packet):
    base = packet['common'].astype(bool)
    k0, k1 = packet['source_K'].reshape(3, 3), packet['target_K'].reshape(3, 3)
    masks, status = {}, {}
    for mean in MEANS:
        code, selected = essential_mask(packet['source_uv'][base], packet[mean+'_uv'][base], k0, k1)
        full = np.zeros(len(base), bool); full[base] = selected
        masks[mean], status[mean] = full, code
    common = base&masks['original']&masks['replacement']
    return common, masks, status


def predict():
    verify(); OUT.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    acquisition = json.loads((INPUT/'acquisition.json').read_text())
    rows = []; calls = 0
    with (OUT/'journal.jsonl').open('x') as journal:
        for item in acquisition['rows']:
            i = item['pair']
            with np.load(INPUT/'packets'/f'{i:03}.npz', allow_pickle=False) as z:
                packet = {key: z[key] for key in z.files}
            selected, masks, filter_status = filter_packet(packet)
            path = OUT/'support'/f'{i:03}.npz'; path.parent.mkdir(exist_ok=True)
            with path.open('xb') as f:
                np.savez_compressed(f, common=selected, original=masks['original'], replacement=masks['replacement'])
            row = {'pair': i, 'time0_ns': item['time0_ns'], 'time1_ns': item['time1_ns'],
                   'before_filter': int(packet['common'].sum()), 'after_filter': int(selected.sum()),
                   'filter_status': filter_status, 'support_sha256': sha(path), 'methods': {}}
            for mean in MEANS:
                row['methods'][mean] = {}
                for objective in OBJECTIVES:
                    if int(selected.sum()) < 8 or any(v != 'OK' for v in filter_status.values()):
                        result = {'status': 'UNAVAILABLE_COMMON_FILTER', 'rotation': None, 'translation_direction': None}
                    else:
                        try:
                            f0 = br.bearings(packet['source_uv'][selected], packet['source_K'].reshape(3, 3))[0]
                            pixel_cov = packet['covariance'][selected]
                            if objective == 'PNEC_unit_pixel_covariance':
                                pixel_cov = np.broadcast_to(np.eye(2), pixel_cov.shape)
                            f1, _, bearing_cov = br.bearings(packet[mean+'_uv'][selected],
                                packet['target_K'].reshape(3, 3), pixel_cov)
                            calls += 1
                            result = br.solve(f0, f1, None if objective == 'NEC' else bearing_cov)
                        except (ValueError, np.linalg.LinAlgError) as exc:
                            result = {'status': 'NUMERICAL_FAILURE', 'reason': str(exc),
                                      'rotation': None, 'translation_direction': None}
                    row['methods'][mean][objective] = safe(result)
            rows.append(row); journal.write(json.dumps(row, allow_nan=False)+'\n'); journal.flush()
            print(i, row['after_filter'], {m: [r['status'] for r in d.values()] for m, d in row['methods'].items()}, flush=True)
    verify()
    write(OUT/'predictions.json', {'rows': rows, 'solver_calls': calls,
        'expected_slots': 114, 'pose_reference_values_used': False, 'runtime_metrics': False})
    pins = {'predictions.json': sha(OUT/'predictions.json'), 'journal.jsonl': sha(OUT/'journal.jsonl')}
    pins.update({str(p.relative_to(OUT)): sha(p) for p in (OUT/'support').glob('*.npz')})
    write(OUT/'prediction-lock.json', {'created_at': datetime.now(timezone.utc).isoformat(),
          'execution_freeze_sha256': sha(FREEZE), 'scoring_started': False, 'pins': pins})


def mapped_rotation(rotation, frame, native_interval, common_interval):
    a, b = native_interval; c, d = common_interval
    if a >= b or c >= d:
        raise ValueError('strict positive time intervals required')
    if max(a-c, c-b, a-d, d-b, 0) > b-a:
        return None
    # Optical target->source to native NED, then the inherited common frame.
    local = Q.T@np.asarray(rotation)@Q
    common = frame[:3, :3]@local@frame[:3, :3].T
    return Rotation.from_rotvec((d-c)/(b-a)*Rotation.from_matrix(common).as_rotvec()).as_matrix()


def rotation_error(prediction, reference):
    return float(np.degrees(Rotation.from_matrix(np.asarray(reference).T@prediction).magnitude()))


def statistics(values):
    a = np.asarray(values, float)
    if not len(a):
        return {'count': 0, 'mean': None, 'rmse': None, 'median': None, 'p95': None, 'maximum': None}
    return {'count': len(a), 'mean': float(a.mean()), 'rmse': float(np.sqrt(np.mean(a*a))),
            'median': float(np.median(a)), 'p95': float(np.percentile(a, 95)), 'maximum': float(a.max())}


def primary_decision(errors):
    n, c, native = (errors[k] for k in ['original/NEC', 'original/PNEC_original_pixel_covariance', 'native'])
    joint = sorted(set(n)&set(c)&set(native))
    metrics = {key: statistics([errors[key][i] for i in joint])
               for key in ['original/NEC', 'original/PNEC_original_pixel_covariance', 'native']}
    checks = {'nonempty_matched_support': bool(joint), 'no_NEC_coverage_loss': set(n) <= set(c)}
    for metric in ['mean', 'rmse']:
        checks['improves_NEC_'+metric] = bool(joint) and metrics['original/PNEC_original_pixel_covariance'][metric] < metrics['original/NEC'][metric]
        checks['not_worse_native_'+metric] = bool(joint) and metrics['original/PNEC_original_pixel_covariance'][metric] <= metrics['native'][metric]
    return {'decision': 'RAIN_DEVELOPMENT_GATE_PASS' if all(checks.values()) else 'STOP_THIS_PROTOTYPE_CONFIGURATION',
            'checks': checks, 'matched_pair_ids': joint, 'matched_rotation_deg': metrics}


def score():
    verify()
    lock = json.loads((OUT/'prediction-lock.json').read_text())
    if lock['execution_freeze_sha256'] != sha(FREEZE):
        raise ValueError('Changed execution freeze')
    for name, digest in lock['pins'].items():
        if sha(OUT/name) != digest:
            raise ValueError('Changed locked prediction: '+name)
    from pose_covariance_accuracy import transforms
    source = ROOT/'results/pose-covariance/accuracy-001'
    frame = transforms(json.loads((source/'rain/calibration.json').read_text()),
        ROOT/'results/pose-covariance/cohort-preflight/snapshot/ore-ms/results/provided-reference-full-record/run-001/source-001/official-loader.py')['thermal']
    clocks = json.loads((source/'predictions-v2/rain.json').read_text())['rows']
    refs = {r['pair']: np.asarray(r['reference_delta'])[:3, :3] for r in json.loads(
        (ROOT/'results/native-information-weights/run-001/scores-rain.json').read_text())['rows']}
    native_rows = json.loads((OLD/'vo-scores.json').read_text())['rows_by_arm']['original']
    errors = {m+'/'+o: {} for m in MEANS for o in OBJECTIVES}; errors['native'] = {}
    scored = []
    for row in json.loads((OUT/'predictions.json').read_text())['rows']:
        i = row['pair']; entries = {}
        for mean in MEANS:
            for objective in OBJECTIVES:
                key = mean+'/'+objective; value = row['methods'][mean][objective]
                rotation = None; reason = value['status']
                if value['status'] == 'CONVERGED':
                    rotation = mapped_rotation(value['rotation'], frame, [row['time0_ns'], row['time1_ns']], clocks[i]['common_interval_ns'])
                    if rotation is None:
                        reason = 'HORIZON_EXCEEDED'
                e = rotation_error(rotation, refs[i]) if rotation is not None else None
                if e is not None:
                    errors[key][i] = e
                entries[key] = {'reason': reason, 'mapped_rotation': safe(rotation), 'error_deg': e}
        native = native_rows[i]['transform']
        e = rotation_error(np.asarray(native)[:3, :3], refs[i]) if native is not None else None
        if e is not None:
            errors['native'][i] = e
        entries['native'] = {'error_deg': e, 'mapped_rotation': safe(np.asarray(native)[:3, :3]) if native is not None else None}
        scored.append({'pair': i, 'reference_rotation': safe(refs[i]), 'entries': entries})
    primary = primary_decision(errors)
    result = {'rows': scored, 'primary': primary, 'coverage': {k: len(v) for k, v in errors.items()},
              'own_support_rotation_deg': {k: statistics(list(v.values())) for k, v in errors.items()},
              'scope': 'Exposed rain development; inherited nominal frame/clock assumptions; rotation only, no metric translation'}
    for mean in MEANS:
        keys = [mean+'/'+o for o in OBJECTIVES]+['native']
        common = sorted(set.intersection(*(set(errors[k]) for k in keys)))
        result[mean+'_all_method_common'] = {'pairs': common,
            'rotation_deg': {k: statistics([errors[k][i] for i in common]) for k in keys}}
    write(OUT/'scores.json', result)
    print(primary, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['predict', 'score'])
    {'predict': predict, 'score': score}[parser.parse_args().action]()
