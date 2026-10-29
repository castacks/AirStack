"""Independent NumPy simulation of bounded native analytic-LM probe traces."""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import numpy as np

NAMES = ('unweighted_matrix', 'explicit_weight_matrix', 'constructor_only_weight', 'explicit_overrides_constructor',
         'swapped_explicit_weight', 'huber_inliers', 'huber_outliers', 'correlated_factor_huber',
         'accept_raw_improves_mahalanobis_worsens', 'reject_raw_worsens_mahalanobis_improves',
         'native_pinv_huber_conflict', 'native_pinv_identity_weight_control', 'forced_rejection_cap', 'forced_solver_failure')


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())


def close(actual, expected):
    if isinstance(expected, dict):
        assert isinstance(actual, dict) and set(actual) == set(expected)
        for k in expected: close(actual[k], expected[k])
    elif expected is None or isinstance(expected, (str, bool, int)): assert actual == expected
    else: np.testing.assert_allclose(actual, expected, rtol=2e-10, atol=1e-12)


def kernel(s, delta):
    s = np.asarray(s)
    return s if delta is None else np.where(s < delta*delta, s, 2*delta*np.sqrt(s)-delta*delta)


def costs(theta, matrix, target, weight, delta):
    r = matrix @ theta-target; block_size = r.shape[1]
    q = [r[i] @ weight[i*block_size:(i+1)*block_size, i*block_size:(i+1)*block_size] @ r[i] for i in range(len(r))]
    s = np.sum(r*r, axis=-1)
    return {'raw_kernel_sum': float(sum(kernel(s, delta))), 'mahalanobis_kernel_sum': float(sum(kernel(q, delta))),
            'weighted_quadratic': float(sum(q)), 'raw_quadratic': float(sum(s))}


def strategy_update(pg, last, loss, j, d, r):
    jd = j @ d; denominator = float(-(jd.T @ (2*r+jd)).item())
    numerator = last-loss
    quality = numerator/denominator if denominator else float('nan') if numerator == 0 else math.copysign(float('inf'), numerator)
    state = pg.copy(); state['radius'] = 1/state['damping']
    if quality > state['high']: state['radius'] *= state['up']; state['down'] = .5
    elif quality > state['low']: state['down'] = .5
    else: state['radius'] *= state['down']; state['down'] *= state['factor']
    state['down'] = min(1e16, max(1e-6, state['down'])); state['radius'] = min(1e16, max(1e-6, state['radius'])); state['damping'] = 1/state['radius']
    return state, quality, denominator


def validate(output):
    protocol = read(output/'protocol.json'); report = read(output/'probes.json'); source = read(output/'native-source.json')
    assert report['protocol_sha256'] == sha(output/'protocol.json') and report['native_source_sha256'] == sha(output/'native-source.json')
    assert report['torch'] == '2.7.1+cpu' and report['pypose'] == '0.9.5' and not report['native_vo_solver_replayed'] and not report['estimator_defaults_changed']
    for p, digest in protocol['source_sha256'].items(): assert sha(p) == digest
    assert report['dependency_method_sha256'] == {k: hashlib.sha256(v.encode()).hexdigest() for k, v in source.items()}
    dependency_modules = {'RobustModel': 'optimizer', '_Optimizer': 'optimizer', 'FastTriggs': 'corrector',
                          'TrustRegion': 'strategy', 'Huber': 'kernel', 'StopOnPlateau': 'scheduler'}
    for name, snapshot in source.items():
        class_name, method = name.split('.')
        path = Path('/workspace/MAC-VO-main/Module/Optimization/PyposeOptimizers.py') if class_name == 'LM_analytic' else Path('/opt/solver-contract/pypose/optim')/(dependency_modules[class_name]+'.py')
        text = path.read_text(); tree = ast.parse(text)
        cls, = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name]
        node, = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == method]
        start = min([node.lineno]+[d.lineno for d in node.decorator_list])
        assert ''.join(text.splitlines(keepends=True)[start-1:node.end_lineno]).rstrip() == snapshot.rstrip()
    assert tuple(c['name'] for c in protocol['fixtures']) == NAMES and len(report['rows']) == len(NAMES)
    evidence = {}; total_solves = total_strategy = 0
    for declared, row in zip(protocol['fixtures'], report['rows']):
        assert row['case'] == declared
        c = declared; matrix, target, theta = [np.array(c[k], dtype=float) for k in ('matrix', 'target', 'initial')]
        weight = np.eye(target.size) if c['step_weight'] is None else np.array(c['step_weight'])
        before = costs(theta, matrix, target, weight, c['delta']); close(row['before_objectives'], before)
        raw = matrix @ theta-target; j = matrix.reshape(-1, len(theta)); s = np.sum(raw*raw, axis=-1)
        derivative = np.ones_like(s)
        if c['delta'] is not None:
            outliers = s >= c['delta']**2; derivative[outliers] = c['delta']/np.sqrt(s[outliers])
        rc = np.sqrt(derivative)[:, None]*raw; jc = np.repeat(np.sqrt(derivative), target.shape[1])[:, None]*j
        corrector, = row['corrector_calls']; close(corrector, {'raw_R': raw, 'raw_J': j, 'corrected_R': rc, 'corrected_J': jc})
        a = jc.T @ weight @ jc; a[np.diag_indices_from(a)] = np.clip(np.diag(a), 1e-6, 1e32)
        b = -jc.T @ weight @ rc.reshape(-1, 1)
        pg = {'radius': 1000., 'damping': .001, 'high': .5, 'low': .001, 'up': 2., 'down': .5, 'factor': .5} if c['native_strategy'] else {'damping': .001}
        loss = last = before['raw_kernel_sum']; rejects = 0; updates = 0; negative_predicted = 0; negative_actual_positive_quality = 0
        candidates = []
        for index, call in enumerate(row['solver_calls']):
            a[np.diag_indices_from(a)] *= 1+pg['damping']
            close(call['A'], a); close(call['b'], b)
            if c['fail']:
                assert index == 0 and call['injected_error'] == 'RuntimeError' and 'D' not in call
                assert 'deliberate contract probe solver failure' in row['stdout']; break
            if c['steps'] is not None: d = np.array(c['steps'][min(index, len(c['steps'])-1)]).reshape(b.shape)
            else: d = np.linalg.pinv(a) @ b
            close(call['D'], d); theta += d.ravel(); candidate = costs(theta, matrix, target, weight, c['delta']); loss = candidate['raw_kernel_sum']; candidates.append(candidate)
            strategy = row['strategy_calls'][updates]; updates += 1
            close(strategy['last'], last); close(strategy['loss'], loss); close(strategy['J'], jc); close(strategy['D'], d); close(strategy['R'], rc.reshape(-1, 1)); close(strategy['before'], pg)
            if c['native_strategy']:
                pg, quality, denominator = strategy_update(pg, last, loss, jc, d, rc.reshape(-1, 1))
                negative_predicted += int(denominator < 0)
                negative_actual_positive_quality += int(loss > last and quality > .5 and denominator < 0)
            close(strategy['after'], pg)
            if last < loss and rejects < c['reject']: theta -= d.ravel(); loss = last; rejects += 1; assert index+1 < len(row['solver_calls'])
            else: assert index+1 == len(row['solver_calls']); break
        assert updates == len(row['strategy_calls'])
        close(row['final_parameter'], theta); close(row['returned_loss'], loss); close(row['last'], last); assert row['reject_count'] == rejects
        after = costs(theta, matrix, target, weight, c['delta']); close(row['after_objectives'], after); close(loss, after['raw_kernel_sum'])
        patience = int(last-loss < 1e-5); assert row['scheduler_patience_count'] == patience and row['scheduler_continual_after_one_step'] == (rejects == 0)
        initial = np.array(c['initial'], dtype=float); raw_gradient = []; white_gradient = []; step = 1e-6
        for v in np.eye(len(initial)):
            plus, minus = costs(initial+v*step, matrix, target, weight, c['delta']), costs(initial-v*step, matrix, target, weight, c['delta'])
            raw_gradient.append((plus['raw_kernel_sum']-minus['raw_kernel_sum'])/(2*step)); white_gradient.append((plus['mahalanobis_kernel_sum']-minus['mahalanobis_kernel_sum'])/(2*step))
        linear_gradient = -2*b.ravel()
        evidence[c['name']] = {'solver_calls': len(row['solver_calls']), 'reject_count': rejects, 'native_step_gradient': linear_gradient.tolist(),
            'raw_acceptance_fd_gradient': raw_gradient, 'mahalanobis_huber_fd_gradient': white_gradient,
            'gradient_difference_from_acceptance_norm': float(np.linalg.norm(linear_gradient-raw_gradient)),
            'gradient_difference_from_mahalanobis_norm': float(np.linalg.norm(linear_gradient-white_gradient)),
            'negative_predicted_reductions': negative_predicted, 'raw_loss_increase_high_quality_steps': negative_actual_positive_quality,
            'candidate_objectives': candidates}
        total_solves += len(row['solver_calls']); total_strategy += updates
    r = {row['case']['name']: row for row in report['rows']}
    close(r['constructor_only_weight']['solver_calls'][0]['A'], r['unweighted_matrix']['solver_calls'][0]['A'])
    close(r['constructor_only_weight']['solver_calls'][0]['b'], [[0.]])
    close(r['explicit_weight_matrix']['solver_calls'][0]['b'], [[8.]])
    close(r['swapped_explicit_weight']['solver_calls'][0]['b'], [[-8.]])
    close(r['explicit_overrides_constructor']['solver_calls'][0]['b'], [[8.]])
    accepted = r['accept_raw_improves_mahalanobis_worsens']; assert accepted['after_objectives']['raw_kernel_sum'] < accepted['before_objectives']['raw_kernel_sum'] and accepted['after_objectives']['mahalanobis_kernel_sum'] > accepted['before_objectives']['mahalanobis_kernel_sum']
    rejected = r['reject_raw_worsens_mahalanobis_improves']; assert rejected['reject_count'] == 1 and evidence[rejected['case']['name']]['candidate_objectives'][0]['mahalanobis_kernel_sum'] < rejected['before_objectives']['mahalanobis_kernel_sum']
    native = r['native_pinv_huber_conflict']; assert native['reject_count'] == 16 and len(native['solver_calls']) == 17 and native['returned_loss'] > native['last']
    assert evidence['native_pinv_huber_conflict']['raw_loss_increase_high_quality_steps'] == 17
    assert r['forced_rejection_cap']['final_parameter'] == [1.] and r['forced_rejection_cap']['reject_count'] == 2
    assert r['forced_solver_failure']['scheduler_continual_after_one_step'] and r['forced_solver_failure']['final_parameter'] == [0.]
    assert evidence['native_pinv_identity_weight_control']['gradient_difference_from_acceptance_norm'] < 1e-8
    assert evidence['correlated_factor_huber']['gradient_difference_from_acceptance_norm'] > .01 and evidence['correlated_factor_huber']['gradient_difference_from_mahalanobis_norm'] > .01
    result = {'status': 'PASS', 'cases': len(report['rows']), 'solver_calls_checked': total_solves, 'strategy_updates_checked': total_strategy, 'installed_method_snapshots_checked': len(source),
        'probes_sha256': sha(output/'probes.json'), 'protocol_sha256': sha(output/'protocol.json'), 'native_source_sha256': sha(output/'native-source.json'),
        'validator_sha256': sha(Path(__file__)), 'evidence': evidence, 'scope': 'independent toy equation/state-machine/finite-difference audit, no recorded-VO causal or physical accuracy claim'}
    (output/'validation.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n'); print(json.dumps({k: result[k] for k in ('status', 'cases', 'solver_calls_checked', 'strategy_updates_checked')}))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--output', type=Path, required=True); validate(p.parse_args().output)
