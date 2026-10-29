"""Opt-in research solver for a single explicit fixed-covariance objective.

No production optimizer imports or defaults are changed. A termination reason
describes a numerical event, not physical accuracy or calibrated convergence.
"""
import math
import torch
from pypose.optim.solver import PINV


def objective(residual, cholesky, delta=.1):
    wr = torch.linalg.solve_triangular(cholesky, residual.unsqueeze(-1), upper=False).squeeze(-1)
    s = wr.square().sum(-1)
    return torch.where(s < delta*delta, s, 2*delta*s.sqrt()-delta*delta).sum(), wr


def consistent_system(residual, jacobian, cholesky, delta=.1):
    loss, wr = objective(residual, cholesky, delta)
    n, d = residual.shape; p = jacobian.shape[-1]
    wj = torch.linalg.solve_triangular(cholesky, jacobian.reshape(n, d, p), upper=False)
    s = wr.square().sum(-1); derivative = torch.ones_like(s); outside = s >= delta*delta
    derivative[outside] = delta/s[outside].sqrt()
    scale = derivative.sqrt(); rc = (wr*scale[:, None]).reshape(-1, 1)
    jc = (wj*scale[:, None, None]).reshape(-1, p)
    return loss, jc.T @ jc, jc.T @ rc


@torch.no_grad()
def solve(model, covariance, *, delta=.1, max_steps=10, max_attempts=17, solver=None):
    params = [p for p in model.parameters() if p.requires_grad]
    if not params: raise ValueError('at least one trainable parameter required')
    if not math.isfinite(delta) or delta <= 0 or max_steps < 1 or max_attempts < 1: raise ValueError('positive objective and budget settings required')
    solver = PINV() if solver is None else solver
    def snapshot(): return [p.detach().clone() for p in params]
    def restore(state):
        for p, value in zip(params, state): p.copy_(value)
    def clean(value):
        if isinstance(value, list): return [clean(v) for v in value]
        return value if math.isfinite(value) else None
    def packed(): return [clean(p.detach().cpu().numpy().copy().tolist()) for p in params]
    initial_state = snapshot(); best_state = initial_state; initial_loss = best_loss = None; damping = .001
    attempts = []; accepted_steps = 0; termination = 'MAX_ITERATIONS'; message = None
    try:
        r = model()
        if not all(torch.isfinite(p).all() for p in params): raise ValueError('nonfinite initial parameters')
        c = torch.as_tensor(covariance, dtype=r.dtype, device=r.device)
        if r.ndim != 2 or not len(r) or c.shape != (len(r), r.shape[1], r.shape[1]) or not torch.isfinite(r).all() or not torch.isfinite(c).all() or not torch.allclose(c, c.mT, rtol=0, atol=1e-12):
            raise ValueError('finite residuals and aligned symmetric covariance blocks required')
        l = torch.linalg.cholesky(c)
        initial_loss = best_loss = float(objective(r, l, delta)[0])
        if not math.isfinite(best_loss): raise ValueError('nonfinite initial objective')
    except (RuntimeError, ValueError) as error:
        restore(initial_state)
        return {'termination': 'INVALID_INPUT', 'message': str(error), 'initial_loss': None, 'best_loss': None,
                'returned_loss': None, 'accepted_steps': 0, 'attempts': [], 'parameters': packed(), 'objective': 'sum Huber(||L^-1 residual||²)', 'delta_whitened': delta}
    for outer in range(max_steps):
        restore(best_state)
        try:
            r = model(); j = model.jacobian()
            if j.ndim != 2 or j.shape != (r.numel(), sum(p.numel() for p in params)) or not torch.isfinite(j).all(): raise ValueError('finite aligned analytic Jacobian required')
            loss, h, g = consistent_system(r, j, l, delta)
            if not torch.isfinite(h).all() or not torch.isfinite(g).all(): raise ValueError('nonfinite normal equations')
        except (RuntimeError, ValueError) as error:
            termination, message = 'LINEARIZATION_ERROR', str(error); break
        if float(g.abs().max()) <= 1e-10:
            termination = 'GRADIENT_TOLERANCE'; break
        accepted = False
        for retry in range(max_attempts):
            restore(best_state); record = {'outer': outer, 'retry': retry, 'damping': damping, 'loss_before': best_loss,
                'parameters_before': packed(), 'H': h.detach().cpu().numpy().tolist(), 'g': g.detach().cpu().numpy().tolist(),
                'candidate_loss': None, 'predicted_reduction': None, 'actual_reduction': None, 'quality': None,
                'accepted': False, 'reason': None, 'candidate_parameters': None}
            attempts.append(record)
            a = h.clone(); diag = torch.diag(h).clamp(1e-6, 1e32); a.diagonal().copy_(diag*(1+damping))
            try:
                d = solver(A=a, b=-g)
                if d.shape != g.shape or not torch.isfinite(d).all(): raise ValueError('nonfinite or malformed solver step')
            except (RuntimeError, ValueError) as error:
                record['reason'] = 'SOLVER_ERROR'; termination, message = 'LINEAR_SOLVER_ERROR', str(error); break
            record['D'] = d.detach().cpu().numpy().tolist()
            if float(torch.linalg.vector_norm(d)) <= 1e-10:
                record['reason'] = 'SMALL_STEP'; termination = 'NO_PROGRESS_SMALL_STEP'; break
            pred = float(-(2*g.mT @ d+d.mT @ h @ d).squeeze())
            record['predicted_reduction'] = pred if math.isfinite(pred) else None
            if not math.isfinite(pred) or pred <= 0:
                record['reason'] = 'NONPOSITIVE_MODEL_REDUCTION'
            else:
                cursor = 0
                try:
                    for p in params:
                        size = p.numel(); p.add_(d[cursor:cursor+size].reshape(p.shape)); cursor += size
                    record['candidate_parameters'] = packed()
                    candidate = float(objective(model(), l, delta)[0])
                    if not math.isfinite(candidate): raise ValueError('nonfinite candidate objective')
                    actual = best_loss-candidate; ratio = actual/pred
                    record.update(candidate_loss=candidate, actual_reduction=actual, quality=ratio)
                    if actual > 0 and ratio >= 1e-4:
                        record.update(accepted=True, reason='OBJECTIVE_DECREASE')
                        best_loss = candidate; best_state = snapshot(); accepted_steps += 1; accepted = True
                        damping = max(1e-12, min(1e12, damping/2 if ratio > .75 else damping*10 if ratio < .25 else damping))
                        break
                    record['reason'] = 'INSUFFICIENT_OBJECTIVE_DECREASE'
                except (RuntimeError, ValueError) as error:
                    record['reason'] = 'INVALID_CANDIDATE'; record['error'] = str(error)
            restore(best_state); damping = min(1e12, damping*10)
        if termination in ('LINEAR_SOLVER_ERROR', 'NO_PROGRESS_SMALL_STEP'): break
        if not accepted:
            termination = 'REJECTION_LIMIT'; break
    restore(best_state)
    returned = float(objective(model(), l, delta)[0])
    return {'termination': termination, 'message': message, 'initial_loss': initial_loss, 'best_loss': best_loss,
            'returned_loss': returned, 'accepted_steps': accepted_steps, 'attempts': attempts, 'parameters': packed(),
            'objective': 'sum Huber(||L^-1 residual||²)', 'delta_whitened': delta}
