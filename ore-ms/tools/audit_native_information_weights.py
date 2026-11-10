"""Independent native-mean, fold-scale, CI-mean and error reconstruction."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.linalg import expm

from audit_pose_covariance_accuracy import hat, matrix_log

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/native-information-weights/run-001'


def read(path):
    return json.loads(Path(path).read_text())


def truth_pose(path):
    a = np.loadtxt(path).reshape(4, 4)
    assert np.isfinite(a).all() and np.array_equal(a[3], [0, 0, 0, 1])
    assert np.linalg.norm(a[:3, :3].T@a[:3, :3]-np.eye(3)) <= 1e-5
    assert abs(np.linalg.det(a[:3, :3])-1) <= 1e-5
    u, _, v = np.linalg.svd(a[:3, :3])
    a[:3, :3] = u@np.diag([1, 1, np.linalg.det(u@v)])@v
    return a


def audit(condition):
    plan = read(ROOT/'experiments/native-information-weights-freeze-2026-11-09.json')
    all_raw = {c: read(OUT/'raw'/f'{c}.json') for c in plan['cases']}
    raw = all_raw[condition]
    pred = read(OUT/'predictions'/f'{condition}.json')
    score = read(OUT/f'scores-{condition}.json')
    old = read(ROOT/plan['cases'][condition]['baseline'])
    model = pred['model']
    assert model['held_out'] == pred['recording'] and model['held_out'] not in model['training_recordings']
    expected = {camera: {block: [] for block in ['translation', 'rotation']} for camera in ['rgb', 'thermal']}
    for c, data in all_raw.items():
        if c == condition:
            continue
        assert data['recording'] in model['training_recordings']
        directory = ROOT/plan['cases'][c]['references']
        poses = {f'{i:06}': truth_pose(directory/f'{i:06}.txt') for i in range(600)}
        values = {cam: {b: [] for b in ['translation', 'rotation']} for cam in ['rgb', 'thermal']}
        for row in data['rows']:
            if not all(x['valid'] and x['information_inverse'] is not None for x in row['experts'].values()):
                continue
            a, b = (poses[s] for s in row['source_ids'])
            truth = np.linalg.solve(a, b)
            for cam in ['rgb', 'thermal']:
                expert = row['experts'][cam]
                e = matrix_log(truth@np.linalg.inv(np.asarray(expert['transform'])))
                p = np.asarray(expert['information_inverse'])
                for block, sl in [('translation', slice(0, 3)), ('rotation', slice(3, 6))]:
                    values[cam][block].append(float(e[sl]@np.linalg.solve(p[sl, sl], e[sl]))/3)
        assert len(values['rgb']['rotation']) == model['training_counts'][data['recording']]
        for cam in values:
            for block in values[cam]:
                expected[cam][block].append(math.fsum(values[cam][block])/len(values[cam][block]))
    for cam in expected:
        for block in expected[cam]:
            np.testing.assert_allclose(sum(expected[cam][block])/2, model['scales'][cam][block], rtol=1e-7, atol=1e-8)
    combinations, components, preserved = 0, 0, 0
    scored = {row['pair']: row for row in score['rows']}
    for i, row in enumerate(pred['rows']):
        for cam in ['rgb', 'thermal']:
            a, b = row['experts'][cam]['transform'], old['rows'][i]['native_experts'][cam]['transform']
            if a is None:
                assert b is None
            else:
                np.testing.assert_array_equal(a, b)
            np.testing.assert_array_equal(row['methods']['native_'+cam]['transform'], old['rows'][i]['methods']['native_'+cam]['transform'])
            preserved += 1
        experts = row['experts']
        for prefix in ['raw', 'cal']:
            for mode in ['scalar', 'full']:
                method = row['methods'][prefix+'_'+mode]
                a, b = (np.asarray(experts[c]['transform']) if experts[c]['valid'] else None for c in ['rgb', 'thermal'])
                if a is None or b is None:
                    t = b if a is None else a
                    if t is None:
                        assert method['transform'] is None
                    else:
                        np.testing.assert_array_equal(method['transform'], t)
                    continue
                ca, cb = (experts[c]['information_inverse'] for c in ['rgb', 'thermal'])
                if ca is None or cb is None:
                    chosen = 'rgb' if ca is not None or cb is None else 'thermal'
                    np.testing.assert_array_equal(method['transform'], experts[chosen]['transform'])
                    continue
                covs = []
                for cam, t, c in [('rgb', a, ca), ('thermal', b, cb)]:
                    c = np.asarray(c)
                    if prefix == 'cal':
                        scales = model['scales'][cam]
                        d = np.diag([math.sqrt(scales['translation'])]*3+[math.sqrt(scales['rotation'])]*3)
                        c = d@c@d
                    if mode == 'scalar':
                        c = np.diag([np.trace(c[:3, :3])/3]*3+[np.trace(c[3:, 3:])/3]*3)
                    j = np.column_stack([(matrix_log(expm(hat(1e-6*e))@t@np.linalg.inv(a))
                                          - matrix_log(expm(hat(-1e-6*e))@t@np.linalg.inv(a)))/2e-6 for e in np.eye(6)])
                    covs.append(j@c@j.T)
                ia, ib = [np.linalg.inv(c) for c in covs]
                w = method['rgb_weight']
                assert min(abs(w-x) for x in np.linspace(0, 1, 21)) < 1e-12
                info = w*ia+(1-w)*ib
                mu = np.linalg.solve(info, (1-w)*ib@matrix_log(b@np.linalg.inv(a)))
                np.testing.assert_allclose(method['transform'], expm(hat(mu))@a, atol=2e-8, rtol=2e-8)
                assert -np.linalg.slogdet(info)[1] <= min(-np.linalg.slogdet(x*ia+(1-x)*ib)[1] for x in np.linspace(0, 1, 21))+1e-7
                combinations += 1
        if i not in scored:
            continue
        s = scored[i]
        truth = np.asarray(s['reference_delta'])
        for method, value in row['methods'].items():
            if value['transform'] is None:
                assert s['errors'][method] is None
                continue
            t = np.asarray(value['transform'])
            r = truth[:3, :3].T@t[:3, :3]
            sine = np.linalg.norm([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])/2
            angle = np.degrees(np.arctan2(sine, (np.trace(r)-1)/2))
            trans = np.linalg.norm(t[:3, 3]-truth[:3, 3])
            assert math.isclose(angle, s['errors'][method]['rotation_deg'], rel_tol=1e-9, abs_tol=1e-9)
            assert math.isclose(trans, s['errors'][method]['translation_m'], rel_tol=1e-9, abs_tol=1e-10)
            components += 2
    return {'status': 'PASS', 'condition': condition, 'native_mean_slots_preserved': preserved,
            'combination_checks': combinations, 'error_component_checks': components,
            'fold_scales_independently_verified': True, 'producer_or_scorer_imported': False}


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--condition', choices=['day', 'night', 'rain'], required=True)
    a = ap.parse_args()
    result = audit(a.condition)
    with (OUT/f'audit-{a.condition}.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result))
