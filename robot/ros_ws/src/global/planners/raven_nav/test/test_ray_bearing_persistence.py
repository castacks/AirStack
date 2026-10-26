"""Exercise production ray selection without starting DDS or the simulator."""
from types import MethodType, SimpleNamespace as S

import numpy as np
import pytest

from test_sharing import _nav_method


def group(distance, direction=(1., 0., 0.), origin=(0., 0., 0.)):
    return S(avg_dist_to_robot=distance, avg_dir=np.array(direction),
             avg_origin=np.array(origin))


def node(enabled, held):
    n = S(_ray_bearing_persistence=enabled,
          _committed_target_last_origin=np.zeros(3),
          _committed_target_last_dir=np.array([1., 0., 0.]),
          _committed_bearing_lock_ts=0., _commit_swap_frac=.25,
          _commit_min_hold_s=4.)
    n._ray_pin_cost = MethodType(_nav_method('_ray_pin_cost'), n)
    n._select_ray_group = MethodType(_nav_method('_select_ray_group'), n)
    n._match_committed_group = lambda cands: held
    return n


@pytest.mark.parametrize('direction,origin', [
    ((-1., 0., 0.), (0., 0., 0.)),
    ((1., 0., 0.), (0., 5., 0.)),
])
def test_disabled_persistence_removes_both_score_penalties(direction, origin):
    held = group(10.)
    closer = group(5., direction, origin)
    control, ablated = node(True, held), node(False, held)
    assert control._select_ray_group([held, closer], 10.) is held
    assert ablated._ray_pin_cost(closer, np.zeros(3), np.array([1., 0., 0.])) == 5.
    assert ablated._select_ray_group([held, closer], 10.) is closer


@pytest.mark.parametrize('distance,now', [(9., 10.), (5., 1.)])
def test_disabled_persistence_bypasses_margin_and_minimum_time(distance, now):
    held, closer = group(10.), group(distance)
    assert node(True, held)._select_ray_group([held, closer], now) is held
    assert node(False, held)._select_ray_group([held, closer], now) is closer


def test_control_switches_after_both_conditions_are_met():
    held, closer = group(10.), group(5.)
    n = node(True, held)
    assert n._select_ray_group([held, closer], 5.) is closer
    assert n._committed_bearing_lock_ts == 5.


@pytest.mark.parametrize('enabled', [True, False])
def test_missing_incumbent_does_not_block_selection(enabled):
    closer, farther = group(5.), group(10.)
    assert node(enabled, None)._select_ray_group([farther, closer], 1.) is closer
