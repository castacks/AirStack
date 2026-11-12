"""Independent NumPy interpolation/cost audit; no optimizer or pose labels."""
from collections import Counter
import json
import cv2
import numpy as np
from preflight_radiometric_tracking import ROOT, PLAN, OUT, sha, write


def patch(image, center, radius):
    # Deliberately independent of SciPy's interpolation implementation.
    yy, xx = np.mgrid[-radius:radius+1, -radius:radius+1]
    x, y = xx.ravel()+center[0], yy.ravel()+center[1]
    ix, iy = np.floor(x).astype(int), np.floor(y).astype(int)
    assert (ix >= 0).all() and (iy >= 0).all()
    assert (ix+1 < image.shape[1]).all() and (iy+1 < image.shape[0]).all()
    dx, dy = x-ix, y-iy
    z = image.astype(float)
    return (z[iy, ix]*(1-dx)*(1-dy)+z[iy, ix+1]*dx*(1-dy)
            +z[iy+1, ix]*(1-dx)*dy+z[iy+1, ix+1]*dx*dy)


def cost(a, b, uv, endpoint, radius):
    p, q = patch(a, uv, radius), patch(b, endpoint, radius)
    residual = (q/q.mean()-p/p.mean())*(p.mean()/p.std())
    return float(np.sum(residual**2))


def audit():
    plan = json.loads(PLAN.read_text())
    summary = json.loads((OUT/'summary.json').read_text())
    assert summary['freeze_sha256'] == sha(PLAN)
    for name, expected in plan['pins'].items():
        assert sha(ROOT/name) == expected, name
    checks, results = 0, {}
    for condition, case in plan['cases'].items():
        a, b = [cv2.imread(str(ROOT/p), cv2.IMREAD_UNCHANGED) for p in case['images']]
        rows = json.loads((OUT/f'{condition}-attempts.json').read_text())
        with np.load(ROOT/case['packet'], allow_pickle=False) as z:
            source, initial = z['source_uv'], z['original_uv']
        assert len(rows) == len(source) == plan['all_grid_points']
        for i, row in enumerate(rows):
            assert row['index'] == i
            assert np.array_equal(row['source_uv'], source[i])
            assert np.array_equal(row['initial_uv'], initial[i])
            checks += 3
            if row['initial_cost'] is not None:
                actual = cost(a, b, source[i], initial[i], plan['radius'])
                np.testing.assert_allclose(actual, row['initial_cost'], rtol=1e-9, atol=1e-10)
                checks += 1
            if row['status'] == 'OK':
                endpoint = np.asarray(row['endpoint_uv'])
                np.testing.assert_allclose(endpoint-initial[i], row['correction_px'], atol=1e-12, rtol=0)
                assert np.max(np.abs(endpoint-initial[i])) < plan['max_correction_px']
                actual = cost(a, b, source[i], endpoint, plan['radius'])
                np.testing.assert_allclose(actual, row['final_cost'], rtol=1e-9, atol=1e-10)
                assert row['final_cost'] <= row['initial_cost']+1e-10
                assert row['singular_ratio'] >= plan['minimum_singular_ratio']
                checks += 5
            else:
                assert row['endpoint_uv'] is None and row['correction_px'] is None
                checks += 1
        counts = dict(Counter(r['status'] for r in rows))
        assert counts == summary['cases'][condition]['status_counts']
        results[condition] = counts
    assert summary['reference_values_read'] is False and summary['pose_solves'] == 0
    assert summary['neural_calls'] == 0
    report = {'status': 'PASS_MECHANICS_ONLY', 'checks': checks, 'cases': results,
              'auditor_sha256': sha(__file__), 'pose_reference_values_read': False,
              'does_not_verify': 'physical correspondence, optimizer correctness, or motion improvement'}
    write(OUT/'audit.json', report)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    audit()
