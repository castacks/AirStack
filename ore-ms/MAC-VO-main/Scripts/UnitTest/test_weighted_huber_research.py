"""Consistent-objective and best-state controls for the isolated research arm."""
import numpy as np
import pytest
import torch
from pypose.optim.corrector import FastTriggs
from pypose.optim.kernel import Huber
from Scripts.Setup.solver_contract_probe import Affine, RecordSolver
from Scripts.Setup.weighted_huber_research import solve, objective, consistent_system


def model(): return Affine([[[1.]], [[1.]]], [[0.], [.2]], [.05])
def covariance(): return torch.tensor([[[1/9]], [[1.]]], dtype=torch.float64)


def test_correlated_finite_difference_gradient():
    matrix = [[[1., 0], [0, 1.], [1., 1.]], [[2., 0], [0, -1.], [1., -1.]]]
    m = Affine(matrix, [[1., .1, -.2], [-.03, .02, .04]], [0., 0.])
    c = torch.tensor([[[2., .3, 0], [.3, 4., 0], [0, 0, .5]], [[1., .1, 0], [.1, 3., 0], [0, 0, 2.]]], dtype=torch.float64)
    l = torch.linalg.cholesky(c); r = m(); _, _, g = consistent_system(r, m.jacobian(), l)
    fd = []
    with torch.no_grad():
        for i in range(2):
            m.theta.zero_(); m.theta[i] = 1e-6; a = float(objective(m(), l)[0])
            m.theta[i] = -1e-6; b = float(objective(m(), l)[0]); fd.append((a-b)/2e-6)
    np.testing.assert_allclose(2*g.detach().numpy().ravel(), fd, rtol=1e-8, atol=1e-10)


def test_identity_covariance_matches_raw_corrector():
    m = model(); r = m(); j = m.jacobian(); l = torch.ones(2, 1, 1, dtype=torch.float64)
    loss, h, g = consistent_system(r, j, l); rc, jc = FastTriggs(Huber(.1))(r, j)
    np.testing.assert_allclose(h.detach().numpy(), (jc.T@jc).detach().numpy())
    np.testing.assert_allclose(g.detach().numpy(), (jc.T@rc.reshape(-1, 1)).detach().numpy())
    assert float(loss) == float(Huber(.1)(r.square().sum(-1)).sum())


def test_consistent_control_is_monotone_and_returns_best_state():
    m = model(); result = solve(m, covariance()); accepted = [a for a in result['attempts'] if a['accepted']]
    previous = result['initial_loss']
    for a in accepted:
        assert a['predicted_reduction'] > 0 and a['actual_reduction'] > 0 and a['quality'] >= 1e-4
        assert a['candidate_loss'] < previous; previous = a['candidate_loss']
    assert accepted and result['returned_loss'] == result['best_loss'] <= result['initial_loss']
    assert float(objective(m(), torch.linalg.cholesky(covariance()))[0]) == result['returned_loss']


def test_rejection_limit_restores_initial_exactly():
    m = model(); initial = m.theta.detach().clone(); result = solve(m, covariance(), max_attempts=3, solver=RecordSolver([[1.]]))
    assert result['termination'] == 'REJECTION_LIMIT' and len(result['attempts']) == 3
    assert not any(a['accepted'] for a in result['attempts']) and torch.equal(initial, m.theta)
    assert result['returned_loss'] == result['initial_loss']
    assert all(a['reason'] == 'NONPOSITIVE_MODEL_REDUCTION' for a in result['attempts'])


def test_solver_exception_explicit_and_restored():
    m = model(); initial = m.theta.detach().clone(); result = solve(m, covariance(), solver=RecordSolver(None, fail=True))
    assert result['termination'] == 'LINEAR_SOLVER_ERROR' and 'deliberate contract probe' in result['message']
    assert torch.equal(initial, m.theta) and result['returned_loss'] == result['initial_loss']


def test_error_after_improvement_preserves_improved_best():
    from pypose.optim.solver import PINV
    class LaterFailure:
        def __init__(self): self.calls = 0
        def __call__(self, A, b):
            self.calls += 1
            if self.calls > 1: raise RuntimeError('later solve failure')
            return PINV()(A=A, b=b)
    m = model(); result = solve(m, covariance(), solver=LaterFailure())
    assert result['termination'] == 'LINEAR_SOLVER_ERROR' and result['accepted_steps'] == 1
    assert result['returned_loss'] == result['attempts'][0]['candidate_loss'] < result['initial_loss']
    np.testing.assert_array_equal(m.theta.detach().numpy(), np.array(result['attempts'][0]['candidate_parameters'][0]))


def test_small_step_is_not_declared_convergence():
    result = solve(model(), covariance(), solver=RecordSolver([[0.]]))
    assert result['termination'] == 'NO_PROGRESS_SMALL_STEP' and result['accepted_steps'] == 0


def test_stationary_gradient_needs_no_solve():
    m = Affine([[[1.]]], [[0.]], [0.]); solver = RecordSolver(None, fail=True)
    result = solve(m, torch.ones(1, 1, 1, dtype=torch.float64), solver=solver)
    assert result['termination'] == 'GRADIENT_TOLERANCE' and result['returned_loss'] == 0 and solver.calls == []


def test_nonfinite_candidate_restores_best():
    class Boundary(Affine):
        def forward(self):
            r = super().forward()
            return r*float('nan') if float(self.theta[0]) < .03 else r
    m = Boundary([[[1.]], [[1.]]], [[0.], [.2]], [.05]); original = m.theta.detach().clone()
    result = solve(m, covariance(), max_attempts=1)
    assert result['termination'] == 'REJECTION_LIMIT' and result['attempts'][0]['reason'] == 'INVALID_CANDIDATE'
    assert torch.equal(m.theta, original) and result['returned_loss'] == result['initial_loss']


def test_nonfinite_solver_step_is_failure():
    result = solve(model(), covariance(), solver=RecordSolver([[float('nan')]]))
    assert result['termination'] == 'LINEAR_SOLVER_ERROR' and result['returned_loss'] == result['initial_loss']


@pytest.mark.parametrize('value', [0, -1, float('nan')])
def test_invalid_covariance_explicit(value):
    c = covariance(); c[0, 0, 0] = value; m = model(); initial = m.theta.detach().clone(); result = solve(m, c)
    assert result['termination'] == 'INVALID_INPUT' and result['returned_loss'] is None and torch.equal(initial, m.theta)


def test_covariance_shape_rejected():
    assert solve(model(), torch.ones(1, 1, 1, dtype=torch.float64))['termination'] == 'INVALID_INPUT'


def test_asymmetric_covariance_rejected():
    m = Affine([[[1.], [2.]]], [[0., .2]], [.05]); c = torch.tensor([[[1., .1], [.2, 1.]]], dtype=torch.float64)
    assert solve(m, c)['termination'] == 'INVALID_INPUT'


def test_rejected_se3_candidate_restores_bitwise():
    import pypose as pp
    class NonlinearPose(torch.nn.Module):
        def __init__(self):
            super().__init__(); self.pose = pp.Parameter(pp.SE3(torch.tensor([[.1, 0, 0, 0, 0, 0, 1.]], dtype=torch.float64)))
        def forward(self): return (self.pose.tensor()[:, :1]**2-1)
        def jacobian(self):
            j = torch.zeros(1, 7, dtype=torch.float64); j[0, 0] = 2*self.pose.tensor()[0, 0]; return j
    m = NonlinearPose(); initial = m.pose.detach().clone(); result = solve(m, torch.ones(1, 1, 1, dtype=torch.float64), max_attempts=1)
    assert result['termination'] == 'REJECTION_LIMIT' and result['attempts'][0]['candidate_parameters'] is not None
    assert result['attempts'][0]['reason'] == 'INSUFFICIENT_OBJECTIVE_DECREASE'
    assert torch.equal(m.pose, initial) and result['returned_loss'] == result['initial_loss']


def test_bad_jacobian_keeps_initial():
    class Invalid(Affine):
        def build_jacobian(self): return torch.full((2, 1), float('nan'), dtype=torch.float64)
    m = Invalid([[[1.]], [[1.]]], [[0.], [.2]], [.05]); result = solve(m, covariance())
    assert result['termination'] == 'LINEARIZATION_ERROR' and result['returned_loss'] == result['initial_loss']


def test_one_iteration_budget_explicit():
    result = solve(model(), covariance(), max_steps=1)
    assert result['termination'] == 'MAX_ITERATIONS' and result['accepted_steps'] == 1
