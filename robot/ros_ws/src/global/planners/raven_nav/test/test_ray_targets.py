"""Unit tests for raven_nav.ray_targets.

Run inside the robot-desktop container:
  bws --packages-select raven_nav
  pytest src/global/planners/raven_nav/test/test_ray_targets.py -v
"""
import numpy as np
import pytest

from raven_nav.ray_groups import RayGroup
from raven_nav.ray_targets import (
    ANGLE_MIN_DEG, EPS_MAX_M, NOMINAL_RANGE_M, T_MIN_M,
    build_targets, is_same_target, least_squares_intersect,
    ray_aabb_hits, triangulate,
)


def _make_group(label: str, origin, direction, score: float = 0.9,
                num_rays: int = 5) -> RayGroup:
    """Synthesize a RayGroup whose avg_origin/avg_dir hit the target values."""
    o = np.asarray(origin, dtype=float)
    d = np.asarray(direction, dtype=float)
    d = d / (np.linalg.norm(d) + 1e-9)
    origins = np.tile(o, (num_rays, 1)) + np.random.default_rng(0).normal(
        scale=0.05, size=(num_rays, 3))
    # Renormalize each per-ray dir near `d` so the group's avg_dir ~= d.
    dirs = np.tile(d, (num_rays, 1))
    scores = np.full(num_rays, score)
    g = RayGroup(label=label, ray_origins=origins, ray_dirs=dirs,
                 ray_scores=scores)
    # Force exact avg fields so the test doesn't depend on rng jitter.
    g._finalize(np.zeros(3))
    g.avg_origin = o
    g.avg_dir = d
    return g


# ---------- triangulate ----------

def test_triangulate_intersecting_rays_returns_intersection_point():
    # Two rays meeting at (10, 10, 5), coming from different directions.
    o_a = np.array([0.0, 10.0, 5.0])
    d_a = np.array([1.0, 0.0, 0.0])  # +x
    o_b = np.array([10.0, 0.0, 5.0])
    d_b = np.array([0.0, 1.0, 0.0])  # +y
    p, eps, t_a, t_b, angle_deg = triangulate(o_a, d_a, o_b, d_b)
    assert np.allclose(p, [10.0, 10.0, 5.0], atol=1e-6)
    assert eps < 1e-6
    assert abs(t_a - 10.0) < 1e-6
    assert abs(t_b - 10.0) < 1e-6
    assert abs(angle_deg - 90.0) < 1e-6


def test_triangulate_parallel_lines_returns_infinity():
    o_a = np.array([0.0, 0.0, 5.0])
    d_a = np.array([1.0, 0.0, 0.0])
    o_b = np.array([0.0, 5.0, 5.0])  # parallel offset by 5m on y
    d_b = np.array([1.0, 0.0, 0.0])
    p, eps, t_a, t_b, angle_deg = triangulate(o_a, d_a, o_b, d_b)
    assert eps == float('inf')
    assert angle_deg < 1e-6  # nearly 0deg
    assert t_a == 0.0 and t_b == 0.0


def test_triangulate_skew_lines_eps_nonzero():
    # Two skew lines 3m apart at closest approach.
    o_a = np.array([0.0, 0.0, 0.0])
    d_a = np.array([1.0, 0.0, 0.0])
    o_b = np.array([10.0, 0.0, 3.0])
    d_b = np.array([0.0, 1.0, 0.0])
    p, eps, t_a, t_b, angle_deg = triangulate(o_a, d_a, o_b, d_b)
    assert abs(eps - 3.0) < 1e-6
    # Midpoint sits halfway between the two feet of perpendicular.
    assert np.allclose(p, [10.0, 0.0, 1.5], atol=1e-6)


# ---------- is_same_target ----------

def test_is_same_target_two_drones_perpendicular_views():
    # Two drones look at a house at (20, 20, 2) from perpendicular angles.
    g_a = _make_group('house', origin=[0, 20, 5], direction=[1, 0, -0.15])
    g_b = _make_group('house', origin=[20, 0, 5], direction=[0, 1, -0.15])
    same, p, eps = is_same_target(g_a, g_b, polygon_xy=None)
    assert same is True
    assert p is not None
    assert np.allclose(p[:2], [20.0, 20.0], atol=1.0)
    assert eps < EPS_MAX_M


def test_is_same_target_label_mismatch_rejects():
    g_a = _make_group('house', origin=[0, 20, 5], direction=[1, 0, 0])
    g_b = _make_group('car', origin=[20, 0, 5], direction=[0, 1, 0])
    same, _, _ = is_same_target(g_a, g_b, polygon_xy=None)
    assert same is False


def test_is_same_target_parallel_rays_rejected():
    g_a = _make_group('house', origin=[0, 0, 5], direction=[1, 0, 0])
    g_b = _make_group('house', origin=[0, 3, 5], direction=[1, 0, 0])
    same, _, _ = is_same_target(g_a, g_b, polygon_xy=None)
    assert same is False  # angle < ANGLE_MIN_DEG


def test_is_same_target_eps_too_large_rejected():
    # Two perpendicular rays that miss each other by 25m vertically.
    # Both have t > T_MIN_M, so eps is the rejection reason (not the t gate).
    g_a = _make_group('house', origin=[-10, 0, 5], direction=[1, 0, 0])
    g_b = _make_group('house', origin=[0, -10, 30], direction=[0, 1, 0])
    same, _, eps = is_same_target(g_a, g_b, polygon_xy=None)
    assert eps > EPS_MAX_M
    assert same is False


def test_is_same_target_origin_above_target_rejected():
    # User's scenario: ray A starts on top of house 1 pointing at house 2.
    # House 1 is at (10, 10, 2); house 2 at (30, 10, 2).
    # Ray A originates at (10, 10, 15) pointing at house 2.
    # Ray B originates at (10, 30, 5) pointing at house 1.
    # Triangulation point lands ~near house 1, but t_a is ~0 (ray A's origin
    # is right above house 1), so t_min gate rejects.
    g_a = _make_group('house', origin=[10, 10, 15], direction=[1, 0, -0.5])
    g_b = _make_group('house', origin=[10, 30, 5], direction=[0, -1, -0.1])
    same, _, _ = is_same_target(g_a, g_b, polygon_xy=None)
    # Should NOT merge house 2's bearing with house 1's bearing.
    assert same is False


def test_is_same_target_outside_polygon_rejected():
    g_a = _make_group('house', origin=[0, 100, 5], direction=[1, 0, 0])
    g_b = _make_group('house', origin=[100, 0, 5], direction=[0, 1, 0])
    poly = np.array([[0, 0], [50, 0], [50, 50], [0, 50]], dtype=float)
    same, _, _ = is_same_target(g_a, g_b, polygon_xy=poly)
    assert same is False  # P at (100,100) is outside.


# ---------- ray_aabb_hits ----------

def test_ray_aabb_hits_straight_through():
    # AABB centered at (20, 0, 2), 4x4x4. Ray from origin pointing +x hits.
    bb = np.array([20.0, 0.0, 2.0, 4.0, 4.0, 4.0])
    hit, t_enter = ray_aabb_hits(
        origin=np.array([0.0, 0.0, 2.0]),
        direction=np.array([1.0, 0.0, 0.0]),
        bb=bb,
    )
    assert hit is True
    assert abs(t_enter - 18.0) < 1e-6  # 20 - 2 = 18


def test_ray_aabb_hits_misses_to_the_side():
    bb = np.array([20.0, 0.0, 2.0, 4.0, 4.0, 4.0])
    hit, _ = ray_aabb_hits(
        origin=np.array([0.0, 100.0, 2.0]),
        direction=np.array([1.0, 0.0, 0.0]),
        bb=bb,
    )
    assert hit is False


def test_ray_aabb_hits_origin_inside_rejected_via_t_min():
    # Origin INSIDE the box. t_enter clamps to 0 which is < T_MIN_M, reject.
    bb = np.array([20.0, 0.0, 2.0, 4.0, 4.0, 4.0])
    hit, _ = ray_aabb_hits(
        origin=np.array([20.0, 0.0, 2.0]),
        direction=np.array([1.0, 0.0, 0.0]),
        bb=bb,
    )
    assert hit is False


def test_ray_aabb_hits_box_behind_origin_rejected():
    # Ray points +x; box is at -20 (behind).
    bb = np.array([-20.0, 0.0, 2.0, 4.0, 4.0, 4.0])
    hit, _ = ray_aabb_hits(
        origin=np.array([0.0, 0.0, 2.0]),
        direction=np.array([1.0, 0.0, 0.0]),
        bb=bb,
    )
    assert hit is False


# ---------- least_squares_intersect ----------

def test_least_squares_three_rays_converge():
    # Three rays meeting at (10, 10, 5) from different bearings.
    origins = np.array([
        [0.0, 10.0, 5.0],
        [10.0, 0.0, 5.0],
        [0.0, 0.0, 5.0],
    ])
    # Direction from each origin toward (10,10,5).
    targets = np.array([10.0, 10.0, 5.0])
    dirs = targets - origins
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    p, cov = least_squares_intersect(origins, dirs)
    assert np.allclose(p, [10.0, 10.0, 5.0], atol=0.1)
    assert np.trace(cov) < 1.0


# ---------- build_targets ----------

def test_build_targets_single_bearing_yields_unconfirmed():
    own = [_make_group('house', origin=[0, 0, 5], direction=[1, 0, 0])]
    targets = build_targets(own, [], known_bbs=[], polygon_xy=None,
                            now_ts=0.0)
    assert len(targets) == 1
    assert targets[0].status == 'unconfirmed'
    assert targets[0].label == 'house'
    # Display estimate is origin + dir * NOMINAL_RANGE_M.
    assert abs(targets[0].position[0] - NOMINAL_RANGE_M) < 1e-3


def test_build_targets_two_converging_yields_one_confirmed():
    own = [_make_group('house', origin=[0, 20, 5], direction=[1, 0, -0.15])]
    peer = [_make_group('house', origin=[20, 0, 5], direction=[0, 1, -0.15])]
    targets = build_targets(own, peer, known_bbs=[], polygon_xy=None,
                            now_ts=0.0)
    assert len(targets) == 1
    assert targets[0].status == 'confirmed'
    assert len(targets[0].contributing_group_ids) == 2


def test_build_targets_two_houses_well_separated():
    # House 1 at (20,20), house 2 at (60,60). Two rays per house.
    own = [
        _make_group('house', origin=[0, 20, 5], direction=[1, 0, -0.15]),
        _make_group('house', origin=[0, 60, 5], direction=[1, 0, -0.15]),
    ]
    peer = [
        _make_group('house', origin=[20, 0, 5], direction=[0, 1, -0.15]),
        _make_group('house', origin=[60, 0, 5], direction=[0, 1, -0.15]),
    ]
    targets = build_targets(own, peer, known_bbs=[], polygon_xy=None,
                            now_ts=0.0)
    confirmed = [t for t in targets if t.status == 'confirmed']
    assert len(confirmed) == 2
    centers = sorted(float(t.position[0]) for t in confirmed)
    assert centers[0] < 30.0 and centers[1] > 50.0


def test_build_targets_bb_pre_filter_absorbs_matching_ray():
    # Known BB at (20, 20, 2) size 8x8x4 labeled 'house'.
    bb = np.array([20.0, 20.0, 2.0, 8.0, 8.0, 4.0])
    own = [_make_group('house', origin=[0, 20, 5], direction=[1, 0, -0.05])]
    targets = build_targets(
        own_groups=own, peer_groups=[],
        known_bbs=[(42, 'house', bb)],
        polygon_xy=None, now_ts=0.0,
    )
    assert len(targets) == 1
    assert targets[0].bb_id == 42
    assert targets[0].status == 'confirmed'
    assert np.allclose(targets[0].position, [20.0, 20.0, 2.0])


def test_build_targets_bb_label_mismatch_does_not_absorb():
    bb = np.array([20.0, 20.0, 2.0, 8.0, 8.0, 4.0])
    own = [_make_group('house', origin=[0, 20, 5], direction=[1, 0, -0.05])]
    targets = build_targets(
        own_groups=own, peer_groups=[],
        known_bbs=[(7, 'car', bb)],  # wrong label
        polygon_xy=None, now_ts=0.0,
    )
    # Falls through to triangulation as a singleton bearing.
    assert len(targets) == 1
    assert targets[0].bb_id is None
    assert targets[0].status == 'unconfirmed'


def test_build_targets_parallel_rays_stay_separate():
    # Two parallel rays toward the same general region must NOT be merged
    # (insufficient angular diversity to triangulate).
    own = [
        _make_group('house', origin=[0, 0, 5], direction=[1, 0, 0]),
        _make_group('house', origin=[0, 3, 5], direction=[1, 0, 0]),
    ]
    targets = build_targets(own, [], known_bbs=[], polygon_xy=None,
                            now_ts=0.0)
    assert len(targets) == 2
    for t in targets:
        assert t.status == 'unconfirmed'
