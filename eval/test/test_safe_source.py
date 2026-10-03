import numpy as np

from eval.agents.safe_source import SafeAggressive, SafeConservative
from integrations.aerial_nav.agents.baselines import Aggressive, Conservative


def test_safe_aggressive_delegates_to_source_policy():
    obs = {
        'agent': np.array([0., 0., 1.]),
        'target': np.array([5., 0., 1.]),
        'static_obstacles': np.empty((0, 3)),
        'static_obstacle_aabbs': np.empty((0, 4)),
        'env_bounds': np.array([-10., -10., 0., 10., 10., 5.]),
        'max_speed': 3.,
    }
    actual = SafeAggressive().act(obs)
    expected = Aggressive().act(obs) / obs['max_speed']
    np.testing.assert_allclose(actual, expected)


def test_safe_conservative_delegates_to_source_policy():
    obs = {
        'agent': np.array([0., 0., 1.]),
        'target': np.array([5., 0., 1.]),
        'static_obstacles': np.empty((0, 3)),
        'static_cylinders': np.empty((0, 3)),
        'dynamic_obstacles': np.empty((0, 6)),
        'static_obstacle_aabbs': np.empty((0, 4)),
        'env_bounds': np.array([-10., -10., 0., 10., 10., 5.]),
        'max_speed': 3.,
    }
    actual = SafeConservative().act(obs)
    reference = Conservative()
    reference.reset()
    expected = reference.act(obs) / obs['max_speed']
    np.testing.assert_allclose(actual, expected)
