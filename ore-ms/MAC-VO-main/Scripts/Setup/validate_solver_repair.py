"""Independent NumPy projection, SE3 retraction and best-state repair checks."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def read(p): return json.loads(Path(p).read_text())
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def close(a, b, rtol=2e-7, atol=1e-8): np.testing.assert_allclose(a, b, rtol=rtol, atol=atol)
def skew(x):
    a, b, c = x; return np.array([[0, -c, b], [c, 0, -a], [-b, a, 0.]])
def rotation(p):
    q = np.asarray(p)[3:]; v, w = q[:3], q[3]; s = skew(v); return np.eye(3)+2*w*s+2*s@s


def update(p, d, graph):
    d = np.asarray(d).ravel()
    if not graph: return p+d
    v, w = d[:3], d[3:6]; angle = np.linalg.norm(w); s = skew(w)
    a = .5-angle**2/24 if angle < 1e-4 else (1-np.cos(angle))/angle**2
    b = 1/6-angle**2/120 if angle < 1e-4 else (angle-np.sin(angle))/angle**3
    t = Rotation.from_rotvec(w).apply(p[:3])+(np.eye(3)+a*s+b*s@s)@v
    u, z = Rotation.from_rotvec(w).as_quat()[:3], Rotation.from_rotvec(w).as_quat()[3]
    old_v, old_w = p[3:6], p[6]
    q = np.r_[z*old_v+old_w*u+np.cross(u, old_v), z*old_w-np.dot(u, old_v)]
    return np.r_[t, q]


def residual_jac(p, ctx):
    if not ctx['graph']: return ctx['matrix']@p-ctx['target'], ctx['matrix'].reshape(-1, len(p))
    world, k, baseline = ctx['world'], ctx['K'], ctx['baseline']; rmat = rotation(p)
    camera = (world-p[:3])@rmat; x, y, z = camera.T; fx, fy = k[0, 0], k[1, 1]
    residual = np.column_stack((fx*y/x+k[0, 2]-ctx['uv'][:, 0], fy*z/x+k[1, 2]-ctx['uv'][:, 1], fx*baseline/x-ctx['disp']))
    n = len(world); projection = np.zeros((n, 3, 3)); projection[:, 0, 0] = -fx*y/x**2; projection[:, 0, 1] = fx/x
    projection[:, 1, 0] = -fy*z/x**2; projection[:, 1, 2] = fy/x; projection[:, 2, 0] = -fx*baseline/x**2
    motion = np.zeros((n, 3, 7)); motion[:, :, :3] = -rmat.T
    motion[:, :, 3:6] = rmat.T @ np.array([skew(point) for point in world])
    return residual, (projection@motion).reshape(-1, 7)


def values(p, ctx):
    r, j = residual_jac(p, ctx); wr = np.linalg.solve(ctx['L'], r[..., None])[..., 0]
    wj = np.linalg.solve(ctx['L'], j.reshape(len(r), r.shape[1], len(p)))
    s = (wr*wr).sum(1); rho = np.where(s < .01, s, .2*np.sqrt(s)-.01)
    derivative = np.ones_like(s); mask = s >= .01; derivative[mask] = .1/np.sqrt(s[mask])
    rc = (wr*np.sqrt(derivative)[:, None]).reshape(-1, 1); jc = (wj*np.sqrt(derivative)[:, None, None]).reshape(-1, len(p))
    raw_s = (r*r).sum(1); raw_loss = float(sum(np.where(raw_s < .01, raw_s, .2*np.sqrt(raw_s)-.01)))
    return float(sum(rho)), jc.T@jc, jc.T@rc, raw_loss


def saved_context(root, case):
    with np.load(root/case['expert_relative']/'tensor-map.npz', allow_pickle=False) as s:
        i = case['pair_ordinal']; take = (s['edge/match2frame1/mapping'] == i) & (s['edge/match2frame2/mapping'] == i+1)
        ids = s['edge/match2point/mapping'][take]; uv_cov = s['match//pixel2_uv_cov'][take].astype(float); c = np.zeros((len(ids), 3, 3))
        c[:, 0, 0] = uv_cov[:, 0]; c[:, 1, 1] = uv_cov[:, 1]; c[:, 0, 1] = c[:, 1, 0] = uv_cov[:, 2]; c[:, 2, 2] = s['match//pixel2_disp_cov'][take].ravel()
        return {'graph': True, 'world': s['points//pos_Tw'][ids].astype(float), 'K': s['frames//K'][i+1].astype(float), 'baseline': float(s['frames//baseline'][i+1]),
                'uv': s['match//pixel2_uv'][take].astype(float), 'disp': s['match//pixel2_disp'][take].ravel().astype(float), 'L': np.linalg.cholesky(c)}


def repair_check(result, initial, ctx, max_steps, scripted=None):
    best = np.asarray(initial).ravel(); best_loss, _, _, _ = values(best, ctx); close(result['initial_loss'], best_loss)
    damping = .001; accepted = 0; fd_error = 0.; checked_linearizations = set(); last_outer = -1; retry_expected = 0
    for index, a in enumerate(result['attempts']):
        if a['outer'] != last_outer:
            assert a['outer'] == last_outer+1; last_outer += 1; retry_expected = 0
        assert a['retry'] == retry_expected and a['retry'] < 17; retry_expected += 1
        close(a['parameters_before'][0], best.reshape(np.array(a['parameters_before'][0]).shape)); close(a['loss_before'], best_loss); close(a['damping'], damping)
        loss, h, g, _ = values(best, ctx); close(a['H'], h); close(a['g'], g)
        if a['outer'] not in checked_linearizations:
            # Saved precision-weighted factors can cross the Huber transition
            # under a 1e-6 pose perturbation. Use the same smaller step for all
            # saved states, after a documented multiscale numerical check.
            checked_linearizations.add(a['outer']); eps = 1e-8 if ctx['graph'] else 1e-6; fd = []
            for direction in np.eye(len(best)):
                plus = values(update(best, direction*eps, ctx['graph']), ctx)[0]; minus = values(update(best, -direction*eps, ctx['graph']), ctx)[0]
                fd.append((plus-minus)/(2*eps))
            error = np.linalg.norm(np.array(fd)-2*g.ravel())/(1+np.linalg.norm(2*g))
            assert error < 1e-3; fd_error = max(fd_error, float(error))
        assert a['accepted'] is False or a['reason'] == 'OBJECTIVE_DECREASE'
        if a['reason'] == 'SOLVER_ERROR':
            assert not a['accepted'] and result['termination'] == 'LINEAR_SOLVER_ERROR' and index == len(result['attempts'])-1; break
        d = np.array(a['D']); system = h.copy(); system[np.diag_indices_from(system)] = np.clip(np.diag(h), 1e-6, 1e32)*(1+damping)
        if scripted is None: close(d, np.linalg.pinv(system)@-g, rtol=3e-6, atol=1e-8)
        else: close(d.ravel(), scripted[min(index, len(scripted)-1)])
        if a['reason'] == 'SMALL_STEP':
            assert np.linalg.norm(d) <= 1e-10 and result['termination'] == 'NO_PROGRESS_SMALL_STEP' and not a['accepted']; break
        pred = float(-(2*g.T@d+d.T@h@d).item()); close(a['predicted_reduction'], pred)
        if pred <= 0:
            assert a['reason'] == 'NONPOSITIVE_MODEL_REDUCTION' and not a['accepted'] and a['candidate_parameters'] is None
            damping = min(1e12, damping*10); continue
        candidate = update(best, d, ctx['graph']); close(np.array(a['candidate_parameters'][0]).ravel(), candidate, rtol=1e-8, atol=1e-9)
        candidate_loss = values(candidate, ctx)[0]; close(a['candidate_loss'], candidate_loss)
        actual = best_loss-candidate_loss; quality = actual/pred; close(a['actual_reduction'], actual); close(a['quality'], quality, rtol=2e-4, atol=1e-6)
        should_accept = actual > 0 and quality >= 1e-4
        assert a['accepted'] == should_accept
        if should_accept:
            best = candidate; best_loss = candidate_loss; accepted += 1; retry_expected = 0
            damping = max(1e-12, min(1e12, damping/2 if quality > .75 else damping*10 if quality < .25 else damping))
        else: damping = min(1e12, damping*10)
    close(np.array(result['parameters'][0]).ravel(), best, rtol=1e-8, atol=1e-9)
    close(result['best_loss'], best_loss); close(result['returned_loss'], best_loss); assert result['accepted_steps'] == accepted and best_loss <= result['initial_loss']+1e-8
    if result['termination'] == 'GRADIENT_TOLERANCE': assert np.max(np.abs(values(best, ctx)[2])) <= 1e-9
    elif result['termination'] == 'MAX_ITERATIONS': assert last_outer == max_steps-1 and result['attempts'][-1]['accepted']
    elif result['termination'] == 'REJECTION_LIMIT': assert result['attempts'][-1]['retry'] == 16 and not result['attempts'][-1]['accepted']
    else: assert result['termination'] in ('NO_PROGRESS_SMALL_STEP', 'LINEAR_SOLVER_ERROR')
    return len(result['attempts']), accepted, fd_error


def validate(root, output):
    p = read(output/'protocol.json'); result = read(output/'comparison.json')
    assert result['protocol_sha256'] == sha(output/'protocol.json') and not result['labels_loaded'] and not result['production_defaults_changed']
    assert p['delta_whitened'] == .1 and p['max_steps'] == 10 and p['max_attempts'] == 17
    for path, digest in p['input_sha256'].items(): assert sha(root/path) == digest
    original_probes = read(root/'results/solver-contract/run-001/probes.json'); original_proof = read(root/'results/solver-contract/run-001/validation.json')
    assert original_proof['status'] == 'PASS' and original_proof['probes_sha256'] == sha(root/'results/solver-contract/run-001/probes.json')
    totals = []; fd_error = 0.
    assert len(result['toys']) == 14
    for c, row, legacy in zip(p['toy_fixtures'], result['toys'], original_probes['rows']):
        assert row['name'] == c['name'] and row['original'] == legacy
        weight = c['step_weight'] if c['step_weight'] is not None else c['constructor_weight']; n, d = np.array(c['target']).shape
        weight = np.eye(n*d) if weight is None else np.array(weight)
        covariance = np.stack([np.linalg.inv(weight[i*d:(i+1)*d, i*d:(i+1)*d]) for i in range(n)])
        ctx = {'graph': False, 'matrix': np.array(c['matrix']), 'target': np.array(c['target']), 'L': np.linalg.cholesky(covariance)}
        checked, accepted, error = repair_check(row['repair'], c['initial'], ctx, 1, c['steps']); totals.append((checked, accepted)); fd_error = max(fd_error, error)
    assert len(result['saved_cases']) == len(p['cases']) == 6
    parity = []; objective_improvements = 0
    for declared, row in zip(p['cases'], result['saved_cases']):
        assert row['case'] == declared; ctx = saved_context(root, declared); original = row['original']
        assert len(ctx['world']) == declared['factor_count']
        initial = np.array(declared['initial_pose_xyzw'], dtype=np.float32).astype(float)
        first = values(initial, ctx); close(original['initial_costs']['raw_huber'], first[3]); close(original['initial_costs']['whitened_huber'], first[0]); close(row['repair_initial_costs']['whitened_huber'], first[0])
        for state in original['history']:
            cost = values(np.array(state['pose_xyzw']), ctx); close(state['costs']['raw_huber'], cost[3]); close(state['costs']['whitened_huber'], cost[0]); close(state['returned_loss'], cost[3])
        last = values(np.array(original['pose_xyzw']), ctx); close(original['final_costs']['raw_huber'], last[3]); close(original['final_costs']['whitened_huber'], last[0])
        reference = np.array(declared['recorded_native_pose_xyzw']); current = np.array(original['pose_xyzw']); translation = float(np.linalg.norm(reference[:3]-current[:3])); rotation_error = float(np.degrees((Rotation.from_quat(reference[3:]).inv()*Rotation.from_quat(current[3:])).magnitude()))
        close(row['original_vs_recorded']['translation_m'], translation); close(row['original_vs_recorded']['rotation_deg'], rotation_error)
        passed = translation <= .001 and rotation_error <= .01; assert row['original_vs_recorded']['within_declared_tolerance'] == passed; parity.append(passed)
        checked, accepted, error = repair_check(row['repair'], initial, ctx, 10); totals.append((checked, accepted)); fd_error = max(fd_error, error)
        repaired = values(np.array(row['repair_pose_xyzw']), ctx); close(row['repair']['parameters'][0][0], row['repair_pose_xyzw']); close(row['repair_final_costs']['raw_huber'], repaired[3]); close(row['repair_final_costs']['whitened_huber'], repaired[0])
        objective_improvements += int(repaired[0] < last[0])
    assert result['all_original_pose_controls_pass'] == all(parity)
    verification = {'status': 'PASS', 'toy_controls': 14, 'saved_cases': 6, 'original_pose_controls_pass': sum(parity), 'repair_attempts_checked': sum(n for n, _ in totals),
        'accepted_steps_checked': sum(n for _, n in totals), 'saved_cases_lower_new_objective_than_original': objective_improvements,
        'max_relative_finite_difference_gradient_error': fd_error, 'protocol_sha256': sha(output/'protocol.json'), 'comparison_sha256': sha(output/'comparison.json'),
        'validator_sha256': sha(Path(__file__)), 'scope': 'fixed-factor objective/best-state and original-pose controls; no trajectory or physical accuracy evaluation'}
    (output/'validation.json').write_text(json.dumps(verification, indent=2)+'\n'); print(json.dumps(verification))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--root', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); validate(a.root, a.output)
