"""Independent saved-output check: no replay producer import or pose labels."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.linalg import expm


def cross(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def objective(t, p, q, cs, ct):
    r = q@t[:3, :3].T+t[:3, 3]-p
    c = cs+t[:3, :3]@ct@t[:3, :3].T
    return .5*sum(x@np.linalg.solve(s, x)+np.linalg.slogdet(s)[1] for x, s in zip(r, c))


def audit(report_path):
    report_path = Path(report_path)
    d = json.loads(report_path.read_text())
    for rel, digest in d['input_sha256'].items():
        assert hashlib.sha256(Path(rel).read_bytes()).hexdigest() == digest
    tensor = next(Path(x) for x in d['input_sha256'] if x.endswith('tensor-map.npz'))
    worst = 0.
    statuses = {}
    stages = {}
    checks = 0
    with np.load(tensor, allow_pickle=False) as archive:
        names = ['frames//K', 'frames//time_ns', 'edge/match2frame1/mapping', 'edge/match2frame2/mapping']
        names += [f'match//{field}' for field in ('pixel1_uv', 'pixel2_uv', 'pixel1_d', 'pixel2_d',
                                                'obs1_covTc', 'obs2_covTc')]
        z = {k: archive[k] for k in names}
        assert len(d['rows']) == d.get('group_prefix', len(z['frames//time_ns']))-1
        assert [r['from_idx'] for r in d['rows']] == list(range(len(d['rows'])))
        for row in d['rows']:
            a, b = row['from_idx'], row['frame_idx']
            assert b == a+1
            assert row['source_time_ns'] == int(z['frames//time_ns'][a])
            assert row['target_time_ns'] == int(z['frames//time_ns'][b])
            mask = (z['edge/match2frame1/mapping'] == a) & (z['edge/match2frame2/mapping'] == b)
            assert int(mask.sum()) == row['points']
            statuses[row['status']] = statuses.get(row['status'], 0)+1
            stage = row.get('native_write_stage', 'none')
            stages[stage] = stages.get(stage, 0)+1
            if row['status'] != 'converged':
                assert row.get('covariance') is None
                continue
            points = []
            for side, idx in [(1, a), (2, b)]:
                uv = z[f'match//pixel{side}_uv'][mask].astype(float)
                depth = z[f'match//pixel{side}_d'][mask].reshape(-1).astype(float)
                k = z['frames//K'][idx].astype(float)
                # Backproject optical pixel rays, then reorder to NED.
                rays = np.linalg.solve(k, np.column_stack((uv, np.ones(len(uv)))).T).T
                points.append((rays*depth[:, None])[:, [2, 0, 1]])
            p, q = points
            cs, ct = z['match//obs1_covTc'][mask], z['match//obs2_covTc'][mask]
            t = np.asarray(row['transform'])
            covariance = np.asarray(row['covariance'])
            np.linalg.cholesky(covariance)
            h = np.zeros((6, 6))
            hm = np.zeros((6, 6))
            for point, c1, c2 in zip(q, cs, ct):
                pred = t[:3, :3]@point+t[:3, 3]
                rotated = t[:3, :3]@c2@t[:3, :3].T
                inv = np.linalg.inv(c1+rotated)
                j = np.column_stack((np.eye(3), -cross(pred)))
                info = j.T@inv@j
                hm += info
                h += info
                derivatives = [np.zeros((3, 3)) for _ in range(3)]
                derivatives += [cross(e)@rotated-rotated@cross(e) for e in np.eye(3)]
                for i in range(6):
                    for k in range(6):
                        h[i, k] += .5*np.trace(inv@derivatives[i]@inv@derivatives[k])
            np.testing.assert_allclose(covariance@h, np.eye(6), atol=1e-8)
            np.testing.assert_allclose(np.asarray(row['mean_only_covariance'])@hm, np.eye(6), atol=1e-8)
            direct = objective(t, p, q, cs, ct)
            np.testing.assert_allclose(direct, row['history'][-1]['nll'], rtol=1e-10, atol=1e-8)
            # Finite differences in covariance-scaled coordinates are better
            # conditioned than mixing raw metre and radian derivative steps.
            chol = np.linalg.cholesky(covariance)
            for direction in chol.T:
                delta = 1e-3*direction
                hat = np.zeros((4, 4))
                hat[:3, :3], hat[:3, 3] = cross(delta[3:]), delta[:3]
                derivative = (objective(expm(hat)@t, p, q, cs, ct)
                              - objective(expm(-hat)@t, p, q, cs, ct))/.002
                worst = max(worst, abs(derivative))
                assert abs(derivative) < 2e-5, (report_path.name, a, derivative)
                checks += 1
    return {'report': str(report_path), 'status_counts': statuses, 'native_write_stages': stages,
            'finite_difference_directions': checks, 'max_covariance_scaled_derivative': worst,
            'independent_information_and_objective_check': 'PASS', 'reference_labels_read': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reports', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {'producer_imported': False, 'checks': [audit(p) for p in args.reports]}
    with args.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2))
