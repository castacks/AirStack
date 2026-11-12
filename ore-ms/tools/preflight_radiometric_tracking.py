"""First-pair, original-flow, raw-uint16 mechanics only; no pose/reference score."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import radiometric_tracking as tracker

ROOT = Path(__file__).resolve().parents[1]
PRIOR = ROOT/'results/unweighted-longer/run-001/inputs'
PLAN = ROOT/'experiments/radiometric-tracking-preflight-2026-11-12.json'
OUT = ROOT/'results/radiometric-tracking-preflight/run-001'
CONDITIONS = ('day', 'night', 'rain')


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def freeze():
    pins = {str(p.relative_to(ROOT)): sha(p) for p in [
        Path(__file__), ROOT/'tools/radiometric_tracking.py',
        ROOT/'tools/test_radiometric_tracking.py']}
    cases = {}
    for condition in CONDITIONS:
        base = PRIOR/condition
        manifest_path, acquisition_path = base/'source-manifest.json', base/'acquisition.json'
        manifest = json.loads(manifest_path.read_text())
        acquisition = json.loads(acquisition_path.read_text())
        packet = base/'packets/000.npz'
        assert acquisition['rows'][0]['pair'] == 0
        assert sha(packet) == acquisition['rows'][0]['packet_sha256']
        images = []
        for row in manifest['records'][:2]:
            item = row['cameras']['thr']['left']
            assert item['dtype'] == 'uint16' and item['shape'] == [256, 640]
            assert sha(ROOT/item['path']) == item['sha256']
            images.append(item['path'])
            pins[item['path']] = item['sha256']
        for p in (manifest_path, acquisition_path, packet):
            pins[str(p.relative_to(ROOT))] = sha(p)
        cases[condition] = {'sequence': manifest['sequence'], 'images': images,
                           'packet': str(packet.relative_to(ROOT)), 'pair': 0}
    write(PLAN, {'scope': 'mechanical feasibility only; no motion/flow accuracy claim',
        'initialization': 'original MAC flow; replacement flow never used',
        'all_grid_points': 585, 'case_selection': 'first adjacent pair of each existing development prefix',
        'radius': tracker.RADIUS, 'max_correction_px': tracker.MAX_CORRECTION_PX,
        'max_nfev': tracker.MAX_EVALUATIONS, 'minimum_raw_std': tracker.MIN_RAW_STD,
        'minimum_singular_ratio': tracker.MIN_SINGULAR_RATIO,
        'no_parameter_search': True, 'cases': cases, 'pins': pins,
        'decision_rule': 'Stop at mechanics; no accuracy advancement from fit residuals or support counts.'})


def run():
    plan = json.loads(PLAN.read_text())

    def verify():
        for name, expected in plan['pins'].items():
            if sha(ROOT/name) != expected:
                raise ValueError('Changed frozen input: '+name)

    verify()
    OUT.mkdir(parents=True, exist_ok=False)
    reports = {}
    for condition in CONDITIONS:
        case = plan['cases'][condition]
        a, b = [cv2.imread(str(ROOT/p), cv2.IMREAD_UNCHANGED) for p in case['images']]
        assert a is not None and b is not None and a.dtype == b.dtype == np.uint16
        with np.load(ROOT/case['packet'], allow_pickle=False) as data:
            source_uv, initial_uv = data['source_uv'].copy(), data['original_uv'].copy()
        assert source_uv.shape == initial_uv.shape == (plan['all_grid_points'], 2)
        rows = []
        for index, (point, initial) in enumerate(zip(source_uv, initial_uv)):
            result = tracker.refine(a, b, point, initial)
            # JSON has no NaN/Infinity: preserve bad coordinates as null values.
            rows.append({'index': index, 'source_uv': [float(x) if np.isfinite(x) else None for x in point],
                         'initial_uv': [float(x) if np.isfinite(x) else None for x in initial], **result})
        write(OUT/f'{condition}-attempts.json', rows)
        reports[condition] = {'attempts': len(rows), 'status_counts': dict(Counter(r['status'] for r in rows)),
                              'input_dtypes': [str(a.dtype), str(b.dtype)],
                              'input_shape': list(a.shape), 'pose_reference_values_read': False}
        print(condition, reports[condition]['status_counts'], flush=True)
    verify()
    write(OUT/'summary.json', {'freeze_sha256': sha(PLAN), 'cases': reports,
        'decision': 'MECHANICS_ONLY_NO_ACCURACY_ADVANCEMENT',
        'neural_calls': 0, 'pose_solves': 0, 'reference_values_read': False,
        'limitations': 'No visibility labels, flow truth, motion accuracy or calibrated covariance. No production hook.'})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['freeze', 'run'])
    {'freeze': freeze, 'run': run}[parser.parse_args().action]()
