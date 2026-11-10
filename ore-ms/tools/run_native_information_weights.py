"""Immutable raw-readout lock, isolated recording-excluded folds, then scoring."""
import argparse
import builtins
from contextlib import contextmanager
from datetime import datetime
import io
import json
import os
from pathlib import Path
from zoneinfo import ZoneInfo

from native_information_weights import apply_model, fit_scales, reference_rows, score_predictions
from pose_covariance_accuracy import read, sha, write

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/native-information-weights/run-001'
PLAN = ROOT/'experiments/native-information-weights-freeze-2026-11-09.json'


def verify_raw():
    lock = read(OUT/'raw-lock.json')
    if lock['plan_sha256'] != sha(PLAN):
        raise ValueError('plan changed')
    for rel, digest in {**lock['raw_sha256'], **lock['code_sha256'], **read(PLAN)['pins']}.items():
        if sha(ROOT/rel) != digest:
            raise ValueError('frozen source/input changed: '+rel)
    return lock


@contextmanager
def deny_held_labels(directory):
    """Reject target-record reference reads and all prior score-file reads."""
    directory = directory.resolve()
    original = builtins.open, io.open
    def guard(fn):
        def opened(path, *args, **kwargs):
            if isinstance(path, (str, bytes, os.PathLike)):
                p = Path(os.fsdecode(path)).resolve()
                if p.is_relative_to(directory) or p.name.startswith('scores-'):
                    raise ValueError('held-record reference or previous score read attempted')
            return fn(path, *args, **kwargs)
        return opened
    builtins.open, io.open = guard(original[0]), guard(original[1])
    try:
        yield
    finally:
        builtins.open, io.open = original


def lock_raw():
    plan = read(PLAN)
    for rel, digest in plan['pins'].items():
        if sha(ROOT/rel) != digest:
            raise ValueError('frozen source/input changed')
    raw = {str((OUT/'raw'/f'{c}.json').relative_to(ROOT)): sha(OUT/'raw'/f'{c}.json') for c in plan['cases']}
    for c in plan['cases']:
        d = read(OUT/'raw'/f'{c}.json')
        assert d['reference_used'] is False and not d['native_means_changed']
    code = {str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__), ROOT/'tools/native_information_weights.py']}
    write(OUT/'raw-lock.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
                              'plan_sha256': sha(PLAN), 'raw_sha256': raw, 'code_sha256': code,
                              'model_fit_executed': False})


def fold(condition):
    verify_raw()
    plan = read(PLAN)
    raw = {c: read(OUT/'raw'/f'{c}.json') for c in plan['cases']}
    assert len({d['recording'] for d in raw.values()}) == 3
    held = raw[condition]['recording']
    held_directory = ROOT/plan['cases'][condition]['references']
    training = {}
    reference_locks = {}
    with deny_held_labels(held_directory):
        for c in plan['cases']:
            if c == condition:
                continue
            ref = ROOT/plan['cases'][c]['references']
            qualified = reference_rows(raw[c], ref)
            joint = [r for r in qualified if all(e['valid'] and e['information_inverse'] is not None for e in r['experts'].values())]
            training[raw[c]['recording']] = joint
            reference_locks[str((ref/'reference-lock.json').relative_to(ROOT))] = sha(ref/'reference-lock.json')
        model = fit_scales(training, held)
        model['training_reference_locks'] = reference_locks
        assert model['held_out'] == raw[condition]['recording']
        prediction = apply_model(raw[condition], model)
        prediction['raw_information_reference_used'] = prediction.pop('reference_used')
        prediction['reference_used_for_fitting'] = True
        prediction['held_record_labels_used_for_fit_or_application'] = False
    # No target labels are available in the model application API.
    verify_raw()
    write(OUT/'predictions'/f'{condition}.json', prediction)
    print(json.dumps({'condition': condition, 'model': model, 'coverage': prediction['coverage']}))


def lock_predictions():
    verify_raw()
    files = {str((OUT/'predictions'/f'{c}.json').relative_to(ROOT)): sha(OUT/'predictions'/f'{c}.json')
             for c in read(PLAN)['cases']}
    write(OUT/'prediction-lock.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
          'raw_lock_sha256': sha(OUT/'raw-lock.json'), 'predictions': files,
          'training_labels_used_only_in_other_record_folds': True, 'evaluation_scores_computed': False})


def score(condition):
    verify_raw()
    lock = read(OUT/'prediction-lock.json')
    for rel, digest in lock['predictions'].items():
        assert sha(ROOT/rel) == digest
    plan = read(PLAN)
    prediction = read(OUT/'predictions'/f'{condition}.json')
    result = score_predictions(prediction, ROOT/plan['cases'][condition]['references'])
    write(OUT/f'scores-{condition}.json', result)
    print(json.dumps({'condition': condition, 'count': result['common_all_count'], 'metrics': result['common_all_metrics']}))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action', choices=['lock-raw', 'fold', 'lock-predictions', 'score'])
    ap.add_argument('--condition', choices=['day', 'night', 'rain'])
    a = ap.parse_args()
    if a.action == 'lock-raw': lock_raw()
    elif a.action == 'lock-predictions': lock_predictions()
    elif a.action == 'fold': fold(a.condition)
    else: score(a.condition)
