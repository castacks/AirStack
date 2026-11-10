"""Independent scalar sums for the descriptive error decomposition."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/fusion-mechanism-review/run-001'


def read(p):
    return json.loads(p.read_text())


def dot(a, b):
    return math.fsum(x*y for x, y in zip(a, b))


def audit():
    report = read(OUT/'decomposition.json'); checks = 0
    for rel, digest in report['source_sha256'].items():
        assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() == digest
    def close(a, b):
        nonlocal checks
        assert math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-12), (a, b)
        checks += 1
    for condition, result in report['conditions'].items():
        raw = read(ROOT/f'results/native-information-weights/run-001/raw/{condition}.json')
        scores = read(ROOT/f'results/translation-fusion/run-001/scores-{condition}.json')
        truth = {r['pair']: [v[3] for v in r['reference_delta'][:3]] for r in scores['rows']}
        rows = [r for r in raw['rows'] if all(e['valid'] for e in r['experts'].values())]
        assert result['pair_ids'] == [r['pair'] for r in rows]
        errors = {c: [[r['experts'][c]['transform'][i][3]-truth[r['pair']][i] for i in range(3)] for r in rows] for c in ['rgb', 'thermal']}
        n = len(rows); means = {c: [math.fsum(e[i] for e in errors[c])/n for i in range(3)] for c in errors}
        for cam in errors:
            e, mean = errors[cam], means[cam]
            mse = math.fsum(dot(v, v) for v in e)/n
            centered = math.fsum(dot([v[i]-mean[i] for i in range(3)], [v[i]-mean[i] for i in range(3)]) for v in e)/n
            close(result['cameras'][cam]['mse_m2'], mse)
            close(result['cameras'][cam]['mean_error_squared_m2'], dot(mean, mean))
            close(result['cameras'][cam]['centered_mse_m2'], centered)
            close(result['cameras'][cam]['mean_norm_error_m'], math.fsum(math.sqrt(dot(v, v)) for v in e)/n)
            radial = []
            for row, v in zip(rows, e):
                t = truth[row['pair']]; length = math.sqrt(dot(t, t))
                if length > 0:
                    radial.append(dot(v, t)/length)
            close(result['cameras'][cam]['motion_axis_mse_m2'], math.fsum(x*x for x in radial)/len(radial))
            close(result['cameras'][cam]['motion_axis_mean_error_m'], math.fsum(radial)/len(radial))
        a, b = errors['rgb'], errors['thermal']
        d = [[y-x for x, y in zip(aa, bb)] for aa, bb in zip(a, b)]
        A = math.fsum(dot(v, v) for v in a)/n
        B = math.fsum(dot(v, delta) for v, delta in zip(a, d))/n
        D = math.fsum(dot(delta, delta) for delta in d)/n
        for key, value in [('A_m2', A), ('B_m2', B), ('D_m2', D)]:
            close(result['risk_polynomial'][key], value)
        alpha = max(0, min(1, -B/D)) if D else 0
        close(result['same_record_oracle_constant']['alpha'], alpha)
        close(result['same_record_oracle_constant']['mse_m2'], A+2*alpha*B+alpha*alpha*D)
        close(result['same_record_oracle_constant']['mean_translation_error_m'], math.fsum(math.sqrt(dot([v[i]+alpha*delta[i] for i in range(3)], [v[i]+alpha*delta[i] for i in range(3)])) for v, delta in zip(a, d))/n)
        close(result['cross_error_second_moment_m2'], math.fsum(dot(v, z) for v, z in zip(a, b))/n)
        close(result['mean_error_dot_product_m2'], dot(means['rgb'], means['thermal']))
    return {'status': 'PASS', 'scalar_checks': checks, 'records': 3, 'producer_imported': False, 'source_hashes_verified': True}


if __name__ == '__main__':
    result = audit()
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)
