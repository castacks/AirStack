"""Independent combination/metric reconstruction; imports no producer/scorer."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.linalg import expm, logm


def matrix_log(t):
    a = np.real_if_close(logm(t), tol=1000)
    if np.iscomplexobj(a):
        raise ValueError('complex rotation-log branch')
    return np.r_[a[:3, 3], a[2, 1], a[0, 2], a[1, 0]]


def hat(v):
    a = np.zeros((4, 4))
    x, y, z = v[3:]
    a[:3, :3] = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
    a[:3, 3] = v[:3]
    return a


def audit(prediction, score):
    p, s = (json.loads(Path(x).read_text()) for x in [prediction, score])
    scored = {r['pair']: r for r in s['rows']}
    metrics, combination_checks = 0, 0
    for row in p['rows']:
        pair = row['pair']
        for family in ['native', 'replay']:
            experts = row[family+'_experts']
            a, b = (np.asarray(experts[c]['transform']) if experts[c]['valid'] else None for c in ['rgb', 'thermal'])
            expected = (None if a is None and b is None else b if a is None else a if b is None
                        else expm(.5*hat(matrix_log(b@np.linalg.inv(a))))@a)
            actual = row['methods'][family+'_equal']['transform']
            if expected is None:
                assert actual is None
            else:
                np.testing.assert_allclose(actual, expected, atol=2e-9, rtol=1e-9)
            combination_checks += 1
        experts = row['replay_experts']
        for mode in ['scalar', 'full']:
            method = row['methods']['replay_'+mode]
            if method['strategy'] != mode+'_local_ci':
                equal = row['methods']['replay_equal']['transform']
                if equal is None:
                    assert method['transform'] is None
                else:
                    np.testing.assert_allclose(method['transform'], equal, atol=1e-12)
                continue
            a, b = (np.asarray(experts[c]['transform']) for c in ['rgb', 'thermal'])
            covariances = []
            for camera, t in [('rgb', a), ('thermal', b)]:
                c = np.asarray(experts[camera]['covariance'])
                if mode == 'scalar':
                    c = np.diag([np.trace(c[:3, :3])/3]*3+[np.trace(c[3:, 3:])/3]*3)
                j = np.column_stack([(matrix_log(expm(hat(1e-6*e))@t@np.linalg.inv(a))
                                      - matrix_log(expm(hat(-1e-6*e))@t@np.linalg.inv(a)))/2e-6
                                     for e in np.eye(6)])
                covariances.append(j@c@j.T)
            ia, ib = [np.linalg.inv(c) for c in covariances]
            w = method['rgb_weight']
            assert min(abs(w-x) for x in np.linspace(0, 1, 21)) < 1e-12
            info = w*ia+(1-w)*ib
            coordinate = np.linalg.solve(info, (1-w)*ib@matrix_log(b@np.linalg.inv(a)))
            expected = expm(hat(coordinate))@a
            np.testing.assert_allclose(method['transform'], expected, atol=2e-8, rtol=2e-8)
            chosen = -np.linalg.slogdet(info)[1]
            optimum = min(-np.linalg.slogdet(x*ia+(1-x)*ib)[1] for x in np.linspace(0, 1, 21))
            assert chosen <= optimum+1e-7
            combination_checks += 1
        if pair not in scored:
            continue
        r = scored[pair]
        reference = np.asarray(r['reference_delta'])
        for name, method in row['methods'].items():
            if method['transform'] is None:
                assert r['errors'][name] is None
                continue
            t = np.asarray(method['transform'])
            rotation = reference[:3, :3].T@t[:3, :3]
            sine = np.linalg.norm([rotation[2, 1]-rotation[1, 2], rotation[0, 2]-rotation[2, 0], rotation[1, 0]-rotation[0, 1]])/2
            angle = np.degrees(np.arctan2(sine, (np.trace(rotation)-1)/2))
            trans = np.linalg.norm(t[:3, 3]-reference[:3, 3])
            assert math.isclose(angle, r['errors'][name]['rotation_deg'], abs_tol=1e-9, rel_tol=1e-9)
            assert math.isclose(trans, r['errors'][name]['translation_m'], abs_tol=1e-10, rel_tol=1e-9)
            metrics += 2
    common = [r for r in s['rows'] if all(v is not None for v in r['errors'].values())]
    assert len(common) == s['common_all_count']
    for method in s['common_all_metrics']:
        for field in ['rotation_deg', 'translation_m']:
            values = [r['errors'][method][field] for r in common]
            summary = s['common_all_metrics'][method][field]
            assert len(values) == summary['count']
            if values:
                assert math.isclose(math.fsum(values)/len(values), summary['mean'], rel_tol=1e-12, abs_tol=1e-12)
                assert math.isclose(math.sqrt(math.fsum(x*x for x in values)/len(values)), summary['rmse'], rel_tol=1e-12, abs_tol=1e-12)
    return {'recording': p['recording'], 'status': 'PASS', 'combination_checks': combination_checks,
            'error_component_checks': metrics, 'common_rows': len(common), 'producer_or_scorer_imported': False}


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prediction', type=Path, required=True)
    ap.add_argument('--score', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    result = audit(a.prediction, a.score)
    with a.output.open('x') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result))
