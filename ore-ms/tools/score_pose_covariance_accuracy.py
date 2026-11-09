"""Reference-only error scorer after a global immutable prediction lock."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from pose_covariance_accuracy import METHODS, read, sha, write
from pose_covariance_replay import validate_transform


def reference_pose(path):
    a = np.loadtxt(path).reshape(4, 4)
    if not np.isfinite(a).all() or not np.array_equal(a[3], [0, 0, 0, 1]):
        return None, 'nonfinite_or_homogeneous_row'
    r = a[:3, :3]
    if np.linalg.norm(r.T@r-np.eye(3)) > 1e-5 or abs(np.linalg.det(r)-1) > 1e-5:
        return None, 'nonrigid'
    u, _, vt = np.linalg.svd(r)
    a[:3, :3] = u@np.diag([1, 1, np.linalg.det(u@vt)])@vt
    return a, 'qualified_with_serialization_roundoff_projection'


def error(prediction, reference):
    prediction, reference = np.asarray(prediction), np.asarray(reference)
    validate_transform(prediction)
    validate_transform(reference)
    relative = np.linalg.solve(reference, prediction)
    return {'translation_m': float(np.linalg.norm(relative[:3, 3])),
            'rotation_deg': float(np.degrees(Rotation.from_matrix(relative[:3, :3]).magnitude()))}


def statistics(values):
    a = np.asarray(values, float)
    if len(a) == 0:
        return dict(count=0, mean=None, median=None, rmse=None, p95=None, maximum=None)
    return dict(count=len(a), mean=float(a.mean()), median=float(np.median(a)),
                rmse=float(np.sqrt(np.mean(a*a))), p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def summarize(rows, methods=METHODS):
    return {method: {metric: statistics([r['errors'][method][metric] for r in rows
                                       if r['errors'][method] is not None])
                     for metric in ['rotation_deg', 'translation_m']}
            for method in methods}


def score(prediction_path, reference_directory, lock_path):
    prediction_path, reference_directory, lock_path = map(Path, (prediction_path, reference_directory, lock_path))
    lock = read(lock_path)
    if lock['predictions'].get(str(prediction_path)) != sha(prediction_path):
        raise ValueError('prediction is absent from or changed after global lock')
    d = read(prediction_path)
    assert d['reference_used'] is False and d['runtime_metrics'] is False
    ref_lock = read(reference_directory/'reference-lock.json')
    if ref_lock['prediction_lock_sha256'] != sha(lock_path):
        raise ValueError('references not bound to the preceding prediction lock')
    poses, qualification = {}, {}
    for stem, digest in ref_lock['sha256'].items():
        if not stem.isdigit() or len(stem) != 6:
            raise ValueError('reference source ID schema')
        path = reference_directory/(stem+'.txt')
        if sha(path) != digest:
            raise ValueError('reference changed')
        poses[stem], qualification[stem] = reference_pose(path)
    rows, invalid = [], []
    for row in d['rows']:
        a, b = [poses[s] for s in row['source_ids']]
        if a is None or b is None:
            invalid.append({'pair': row['pair'], 'source_ids': row['source_ids'],
                            'reasons': [qualification[s] for s in row['source_ids']]})
            continue
        reference = np.linalg.solve(a, b)
        errors = {m: error(row['methods'][m]['transform'], reference)
                  if row['methods'][m]['transform'] is not None else None for m in METHODS}
        rows.append({'pair': row['pair'], 'source_ids': row['source_ids'],
                     'reference_delta': reference.tolist(), 'errors': errors})
    common = [r for r in rows if all(r['errors'][m] is not None for m in METHODS)]
    pairs = {}
    for baseline in METHODS:
        matched = [r for r in rows if r['errors']['replay_full'] is not None and r['errors'][baseline] is not None]
        pairs[baseline] = {'count': len(matched), 'metrics': summarize(matched, ['replay_full', baseline])}
    return {'recording': d['recording'], 'prediction_sha256': sha(prediction_path),
            'reference_lock_sha256': sha(reference_directory/'reference-lock.json'),
            'expected_intervals': len(d['rows']), 'reference_qualified_intervals': len(rows),
            'invalid_reference': invalid, 'prediction_coverage': d['coverage'],
            'full_strategies': d['full_strategies'], 'own_support': summarize(rows),
            'common_all_count': len(common), 'common_all_metrics': summarize(common),
            'full_vs_baseline_matched': pairs, 'rows': rows, 'runtime_metrics': False,
            'scope': 'development_provided_reference_proxy_rotation_translation_separate_no_trajectory_claim'}


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prediction', type=Path, required=True)
    ap.add_argument('--references', type=Path, required=True)
    ap.add_argument('--lock', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    result = score(a.prediction, a.references, a.lock)
    write(a.output, result)
    print(json.dumps({'expected': result['expected_intervals'], 'common': result['common_all_count'],
                      'metrics': result['common_all_metrics']}))
