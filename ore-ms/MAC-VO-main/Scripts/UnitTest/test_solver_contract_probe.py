"""Native optimizer contract fixtures, without modifying its implementation."""
import copy
import hashlib
import numpy as np
import pytest
from Scripts.Setup.solver_contract_probe import fixtures, run_case, CORE, Affine, core
from Scripts.Setup.validate_solver_contract import costs, strategy_update


def case(name): return copy.deepcopy(next(c for c in fixtures() if c['name'] == name))


@pytest.mark.parametrize('name,a,b', [('unweighted_matrix', 2.002, 0), ('explicit_weight_matrix', 10.01, 8),
    ('constructor_only_weight', 2.002, 0), ('explicit_overrides_constructor', 10.01, 8),
    ('swapped_explicit_weight', 10.01, -8), ('huber_inliers', 10.01, .25), ('huber_outliers', 1.001, .8)])
def test_native_equations(name, a, b):
    r = run_case(case(name))
    np.testing.assert_allclose(r['solver_calls'][0]['A'], [[a]], rtol=1e-12)
    np.testing.assert_allclose(r['solver_calls'][0]['b'], [[b]], atol=1e-12)


def test_corrector_uses_raw_norm_before_weights():
    a = case('huber_outliers'); b = copy.deepcopy(a); b['step_weight'] = [[9, 0], [0, 1]]
    x, y = run_case(a), run_case(b)
    assert x['corrector_calls'] == y['corrector_calls']
    np.testing.assert_allclose(x['corrector_calls'][0]['corrected_R'], [[np.sqrt(.1)], [-np.sqrt(.1)]])
    assert x['returned_loss'] == y['returned_loss'] == .38


def test_acceptance_ignores_mahalanobis_objective_increase():
    r = run_case(case('accept_raw_improves_mahalanobis_worsens'))
    np.testing.assert_allclose(r['final_parameter'], [.075])
    assert r['after_objectives']['mahalanobis_kernel_sum'] > r['before_objectives']['mahalanobis_kernel_sum']
    assert r['returned_loss'] < r['last'] and r['reject_count'] == 0


def test_rejects_mahalanobis_improving_direction():
    r = run_case(case('reject_raw_worsens_mahalanobis_improves'))
    assert r['reject_count'] == 1 and len(r['solver_calls']) == 2
    np.testing.assert_allclose(r['final_parameter'], [.05])


def test_native_pinv_cap_retains_worse_raw_loss():
    r = run_case(case('native_pinv_huber_conflict'))
    assert r['reject_count'] == 16 and len(r['solver_calls']) == 17
    assert r['returned_loss'] > r['last'] and not r['scheduler_continual_after_one_step']


def test_native_identity_control():
    r = run_case(case('native_pinv_identity_weight_control'))
    assert r['returned_loss'] < r['last'] and r['reject_count'] == 0 and len(r['solver_calls']) == 1


def test_scripted_cap_is_bounded():
    r = run_case(case('forced_rejection_cap'))
    assert len(r['solver_calls']) == 3 and r['final_parameter'] == [1.]
    assert r['returned_loss'] == 2. and r['last'] == 0.


def test_injected_failure_returns_finite_loss_with_printed_error():
    r = run_case(case('forced_solver_failure'))
    assert np.isfinite(r['returned_loss']) and r['final_parameter'] == [0.]
    assert 'deliberate contract probe solver failure' in r['stdout']
    assert r['scheduler_continual_after_one_step'] and not r['strategy_calls']


def test_original_core_unchanged():
    digest = hashlib.sha256(CORE.read_bytes()).hexdigest(); run_case(case('native_pinv_huber_conflict'))
    assert hashlib.sha256(CORE.read_bytes()).hexdigest() == digest


@pytest.mark.parametrize('name', ['native_pinv_huber_conflict', 'native_pinv_identity_weight_control'])
def test_recording_wrappers_preserve_native_toy_result(name):
    import torch
    from pypose.optim.solver import PINV
    from pypose.optim.strategy import TrustRegion
    from pypose.optim.kernel import Huber
    from pypose.optim.corrector import FastTriggs
    c = case(name); observed = run_case(c); m = Affine(c['matrix'], c['target'], c['initial'])
    optimizer = core.LM_analytic(m, solver=PINV(), strategy=TrustRegion(radius=1e3), kernel=Huber(.1), corrector=FastTriggs(Huber(.1)))
    loss = optimizer.step(input=(), weight=torch.tensor(c['step_weight'], dtype=torch.float64))
    assert np.array_equal(m.theta.detach().numpy(), observed['final_parameter'])
    assert float(loss) == observed['returned_loss'] and optimizer.reject_count == observed['reject_count']


def test_independent_objective_known_values():
    c = case('accept_raw_improves_mahalanobis_worsens')
    result = costs(np.array(c['initial']), np.array(c['matrix']), np.array(c['target']), np.array(c['step_weight']), .1)
    np.testing.assert_allclose(result['raw_kernel_sum'], .0225)
    np.testing.assert_allclose(result['mahalanobis_kernel_sum'], .04)


def test_trust_region_positive_ratio_for_two_increases():
    # Corrected residual/Jacobian at x=.05; weighted step points uphill in the raw model.
    r = np.array([[.05], [-.15*np.sqrt(2/3)]])
    j = np.array([[1.], [np.sqrt(2/3)]])
    pg = {'radius':1000.,'damping':.001,'high':.5,'low':.001,'up':2.,'down':.5,'factor':.5}
    state, quality, denominator = strategy_update(pg, .0225, .0274, j, np.array([[-.036]]), r)
    assert denominator < 0 and quality > .5 and state['damping'] < pg['damping']
