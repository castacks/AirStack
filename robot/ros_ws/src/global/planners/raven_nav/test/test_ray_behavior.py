"""RayBehavior — the OG ray-based search port."""
import numpy as np
import pytest

from helpers import ctx, rays, scores_for
from raven_nav.behaviors import ray_behavior as rb
from raven_nav.behaviors.ray_behavior import RayBehavior, angle_bin_groups


def test_og_constants_unchanged():
    assert rb.RAY_SCORE_THRESHOLD == 0.95
    assert rb.ANGLE_BIN_DEG == 45.0
    assert rb.MIN_RAYS_PER_GROUP == 1
    assert rb.DENSITY_WEIGHT == 5.0
    assert rb.MAGNITUDE_M == 6.0
    assert rb.UNLOCK_RADIUS_M == 4.0


# ── the 45-degree binning ───────────────────────────────────────────────────
def test_angle_binning_merges_within_45_degrees():
    d = np.array([[1.0, 0.0], [np.cos(np.deg2rad(30)), np.sin(np.deg2rad(30))]])
    assert len(angle_bin_groups(d)) == 1


def test_angle_binning_splits_beyond_45_degrees():
    d = np.array([[1.0, 0.0], [np.cos(np.deg2rad(80)), np.sin(np.deg2rad(80))]])
    assert len(angle_bin_groups(d)) == 2


def test_angle_binning_is_first_fit_and_order_sensitive():
    """OG:92-108 walks the rays in order and joins the FIRST group within
    tolerance, updating that group's running-mean centroid."""
    a = np.array([1.0, 0.0])
    b = np.array([np.cos(np.deg2rad(40)), np.sin(np.deg2rad(40))])
    c = np.array([np.cos(np.deg2rad(80)), np.sin(np.deg2rad(80))])
    # a, b, c: b joins a (40 deg), the centroid rotates to 20 deg, c is 60 deg
    # off that -> new group.
    assert [len(g['rays']) for g in angle_bin_groups(np.stack([a, b, c]))] == [2, 1]
    # b, a, c: a joins b, centroid at 20 deg, c 60 deg off -> new group again,
    # but the membership order differs, which is what "order sensitive" means.
    got = angle_bin_groups(np.stack([b, a, c]))
    assert [len(g['rays']) for g in got] == [2, 1]
    assert got[0]['indices'] == [0, 1]


def test_angle_binning_centroid_is_the_running_mean():
    d = np.array([[1.0, 0.0], [np.cos(np.deg2rad(40)), np.sin(np.deg2rad(40))]])
    g = angle_bin_groups(d)[0]
    assert np.rad2deg(np.arctan2(*g['centroid'][::-1])) == pytest.approx(20.0, abs=1e-6)


# ── condition ───────────────────────────────────────────────────────────────
def _one_ray_ctx(score=0.99, origin=(20.0, 0.0, 6.0), direction=(1.0, 0.0, 0.0),
                 **kw):
    kw.setdefault('cur_pose', np.array([0.0, 0.0, 6.0]))
    return ctx(**rays([origin], [direction], scores_for(1, 3, 0, score)), **kw)


def test_condition_false_below_threshold():
    b = RayBehavior()
    b.update(_one_ray_ctx(score=0.90))
    assert b.condition_check(_one_ray_ctx(score=0.90)) is False


def test_condition_true_above_threshold():
    c = _one_ray_ctx(score=0.99)
    b = RayBehavior()
    b.update(c)
    assert b.condition_check(c) is True


def test_condition_false_when_the_hot_column_is_not_a_target():
    c = ctx(**rays([(20.0, 0, 6)], [(1.0, 0, 0)], scores_for(1, 3, 2, 0.99)),
            cur_pose=np.array([0.0, 0.0, 6.0]))
    b = RayBehavior()
    b.update(c)
    assert b.condition_check(c) is False


def test_condition_false_with_no_targets():
    c = _one_ray_ctx()
    c.target_objects = []
    b = RayBehavior()
    b.update(c)
    assert b.condition_check(c) is False


def test_rays_pointing_back_at_the_drone_are_filtered_out():
    """OG:74-83 keeps a ray only when its target (origin + unit xy dir) lies
    further from the drone ALONG the ray. A ray 20 m ahead aimed back at the
    drone fails that test."""
    c = ctx(**rays([(20.0, 0.0, 6.0)], [(-1.0, 0.0, 0.0)],
                   scores_for(1, 3, 0, 0.99)),
            cur_pose=np.array([0.0, 0.0, 6.0]))
    b = RayBehavior()
    b.update(c)
    assert b.condition_check(c) is False


def test_a_ray_behind_the_drone_pointing_away_is_kept():
    """The same OG rule keeps this one — the filter is about the ray's own
    direction, not about which side of the drone the origin sits on."""
    c = ctx(**rays([(-40.0, 0.0, 6.0)], [(-1.0, 0.0, 0.0)],
                   scores_for(1, 3, 0, 0.99)),
            cur_pose=np.array([0.0, 0.0, 6.0]))
    b = RayBehavior()
    b.update(c)
    assert b.condition_check(c) is True


# ── execution ───────────────────────────────────────────────────────────────
def test_waypoints_are_six_and_twelve_metres_along_the_bearing():
    c = _one_ray_ctx()
    b = RayBehavior()
    b.update(c)
    out = b.execute(c)
    assert len(out.path) == 2
    wp1, wp2 = out.path
    assert wp1[0] == pytest.approx(26.0)
    assert wp2[0] == pytest.approx(32.0)


def test_densest_nearby_group_wins():
    """cost = dist(origin) - 5*density (OG:128-129): 5 rays 40 m out beat one
    ray 10 m out."""
    o = [(30.0, 0.0, 6.0)] + [(40.0, 0.0, 6.0)] * 5
    d = [(0.0, 1.0, 0.0)] + [(1.0, 0.0, 0.0)] * 5
    c = ctx(**rays(o, d, scores_for(6, 3, 0, 0.99)),
            cur_pose=np.array([0.0, 0.0, 6.0]))
    b = RayBehavior()
    b.update(c)
    out = b.execute(c)
    assert out.path[0][0] == pytest.approx(46.0)


def test_unlock_branch_fires_when_wp2_is_within_four_metres():
    """OG:187-188. Driven through a hand-built analysis because — see the next
    test — the OG forward filter makes this unreachable from real ray data."""
    b = RayBehavior()
    origin = np.array([0.0, 0.0, 6.0])
    direction = np.array([1.0, 0.0, 0.0])
    b.analysis = rb.RayAnalysis(
        orig=origin.reshape(1, 3), dirs=direction.reshape(1, 3),
        scores=np.array([0.99]), labels=['person'],
        groups=[{'centroid': direction[:2], 'rays': [direction[:2]],
                 'indices': [0]}],
        averages=[(origin, direction, 1)], order=[0])
    out = b.execute(ctx(cur_pose=np.array([12.0, 0.0, 6.0]),
                        waypoint_locked=True))
    assert out.path[1][0] == pytest.approx(12.0)     # wp2 = origin + 12*dir
    assert out.waypoint_locked is False


def test_forward_filter_makes_the_ray_unlock_unreachable_from_real_rays():
    """Interaction of two OG rules, pinned because it explains why ray mode in
    practice only unlocks on a mode switch: the forward filter requires
    dir . (origin + dir - pose) > 0, i.e. origin_x > pose_x - 1 along the
    bearing, while wp2 = origin + 12*dir, so |pose - wp2| > 11 m always."""
    b = RayBehavior()
    for offset in (-30.0, -5.0, -0.5, 0.0, 5.0, 30.0):
        c = ctx(**rays([(offset, 0.0, 6.0)], [(1.0, 0.0, 0.0)],
                       scores_for(1, 3, 0, 0.99)),
                cur_pose=np.array([0.0, 0.0, 6.0]), waypoint_locked=True)
        b.update(c)
        out = b.execute(c)
        if out.path:
            assert np.linalg.norm(c.cur_pose - out.path[1]) > 4.0
            assert out.waypoint_locked is True


def test_lock_is_passed_through_when_far_away():
    c = _one_ray_ctx(waypoint_locked=True)
    b = RayBehavior()
    b.update(c)
    assert b.execute(c).waypoint_locked is True


def test_current_target_is_the_dominant_label():
    c = ctx(**rays([(20.0, 0, 6)], [(1.0, 0, 0)], scores_for(1, 3, 0, 0.99)),
            cur_pose=np.array([0.0, 0.0, 6.0]))
    b = RayBehavior()
    b.update(c)
    b.execute(c)
    assert b.current_target == 'person'


def test_deviation_1_z_clamped():
    c = _one_ray_ctx(origin=(20.0, 0.0, 60.0), min_altitude=3.0,
                     max_altitude=15.0)
    b = RayBehavior()
    b.update(c)
    for wp in b.execute(c).path:
        assert 3.0 <= wp[2] <= 15.0


def test_deviation_2_group_outside_polygon_is_skipped_for_the_next_one():
    poly = np.array([[-50.0, -50.0], [50.0, -50.0], [50.0, 50.0], [-50.0, 50.0]])
    # nearest group leads out of the polygon; the second stays inside.
    o = [(45.0, 0.0, 6.0), (5.0, 0.0, 6.0)]
    d = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
    c = ctx(**rays(o, d, scores_for(2, 3, 0, 0.99)),
            cur_pose=np.array([0.0, 0.0, 6.0]), search_area_xy=poly)
    b = RayBehavior()
    b.update(c)
    out = b.execute(c)
    assert out.path, 'the in-polygon group should still be flown'
    assert abs(out.path[1][0]) < 50.0 and abs(out.path[1][1]) < 50.0


def test_no_groups_publishes_nothing():
    b = RayBehavior()
    c = _one_ray_ctx(score=0.1)
    b.update(c)
    out = b.execute(c)
    assert out.path == []
    assert out.note


# ── reporting ───────────────────────────────────────────────────────────────
def test_ray_groups_carry_label_and_geometry():
    o = [(20.0, 0.0, 6.0), (21.0, 1.0, 6.0)]
    c = ctx(**rays(o, [(1.0, 0, 0)] * 2, scores_for(2, 3, 0, 0.99)),
            cur_pose=np.array([0.0, 0.0, 6.0]))
    b = RayBehavior()
    b.update(c)
    groups = b.ray_groups()
    assert len(groups) == 1
    g = groups[0]
    assert g.label == 'person'
    assert g.num_rays == 2
    assert g.avg_origin[0] == pytest.approx(20.5)
    assert g.avg_dir[0] == pytest.approx(1.0)


def test_arrows_map_every_kept_ray_to_a_group():
    o = [(20.0, 0.0, 6.0), (20.0, 0.0, 6.0)]
    d = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
    c = ctx(**rays(o, d, scores_for(2, 3, 0, 0.99)),
            cur_pose=np.array([0.0, 0.0, 6.0]))
    b = RayBehavior()
    b.update(c)
    origins, dirs, gid = b.arrows()
    assert origins.shape[0] == dirs.shape[0] == gid.shape[0] == 2
    assert set(gid.tolist()) == {0, 1}


def test_group_table_lists_every_group():
    o = [(20.0, 0.0, 6.0), (20.0, 0.0, 6.0)]
    d = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
    c = ctx(**rays(o, d, scores_for(2, 3, 0, 0.99)),
            cur_pose=np.array([0.0, 0.0, 6.0]))
    b = RayBehavior()
    b.update(c)
    assert 'ray groups=2' in b.group_table()
