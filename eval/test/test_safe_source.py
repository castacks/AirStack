import numpy as np

from eval.agents.safe_source import (
    SafeAggressive, SafeConservative, SafeAdaptiveApf, _swap_xy,
    _to_safe_benchmark_frame)
from integrations.aerial_nav.agents.baselines import Aggressive, Conservative, AdaptiveApf


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
    expected = _swap_xy(Aggressive().act(_to_safe_benchmark_frame(obs))) / obs['max_speed']
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
    expected = _swap_xy(reference.act(_to_safe_benchmark_frame(obs))) / obs['max_speed']
    np.testing.assert_allclose(actual, expected)


def test_frame_conversion_preserves_geometry():
    obs = {
        'agent': np.array([2., 5., 1.]),
        'target': np.array([7., 9., 1.]),
        'dynamic_obstacles': np.array([[3., 4., 1., 0.1, 0.2, 0.]]),
        'static_obstacle_aabbs': np.array([[1., 2., 4., 8.]]),
        'env_bounds': np.array([-10., -20., 0., 10., 20., 5.]),
    }
    safe = _to_safe_benchmark_frame(obs)
    np.testing.assert_array_equal(safe['agent'], [5., 2., 1.])
    np.testing.assert_array_equal(safe['dynamic_obstacles'][0],
                                  [4., 3., 1., 0.2, 0.1, 0.])
    np.testing.assert_array_equal(safe['static_obstacle_aabbs'][0],
                                  [2., 1., 8., 4.])
    np.testing.assert_array_equal(safe['env_bounds'],
                                  [-20., -10., 0., 20., 10., 5.])
    np.testing.assert_array_equal(obs['agent'], [2., 5., 1.])


def test_safe_adaptive_apf_delegates_to_source_policy():
    obs = {
        'agent': np.array([0., 0., 1.]),
        'target': np.array([5., 0., 1.]),
        'static_obstacles': np.empty((0, 3)),
        'static_cylinders': np.empty((0, 3)),
        'dynamic_obstacles': np.empty((0, 6)),
        'static_obstacle_aabbs': np.empty((0, 4)),
        'env_bounds': np.array([-10., -10., 0., 10., 10., 5.]),
        'max_speed': 3.,
        'r_unsafe': 0.91,
        't': 0.,
    }
    reference = AdaptiveApf()
    reference.reset()
    expected = _swap_xy(reference.act(_to_safe_benchmark_frame(obs))) / obs['max_speed']
    actual = SafeAdaptiveApf().act(obs)
    np.testing.assert_allclose(actual, expected)
