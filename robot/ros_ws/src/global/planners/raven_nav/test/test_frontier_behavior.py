"""FrontierBehavior — the OG frontier exploration port.

Run (host, no ROS):
  cd robot/ros_ws/src/global/planners/raven_nav && \
  uv run --with numpy --with scipy --with scikit-learn --with pytest \
         --with opencv-python-headless python -m pytest test -q
"""
import numpy as np
import pytest

from helpers import ctx, frontier_cloud_flu
from raven_nav.behaviors import frontier_behavior as fb
from raven_nav.behaviors.frontier_behavior import FrontierBehavior


def _cluster(center, n=8, spread=0.5, rng=None):
    rng = rng or np.random.default_rng(0)
    return np.asarray(center, dtype=float)[None, :] + rng.normal(
        scale=spread, size=(n, 3))


def test_og_constants_unchanged():
    assert fb.FRONTIER_MIN_Z_M == 1.5
    assert fb.DBSCAN_EPS_M == 2.7
    assert fb.DBSCAN_MIN_SAMPLES == 3
    assert fb.VIEWPOINT_MIN_Z_M == 2.0
    assert fb.MOMENTUM_WEIGHT == 5.0
    assert fb.TOP_N == 5
    assert fb.LEAD_DIST_M == 2.0
    assert fb.UNLOCK_RADIUS_M == 5.0


def test_condition_check_always_true():
    assert FrontierBehavior().condition_check(ctx()) is True


def test_viewpoints_are_cluster_centroids():
    pts = np.vstack([_cluster([20.0, 0.0, 6.0]), _cluster([-20.0, 5.0, 8.0])])
    b = FrontierBehavior(rng=np.random.default_rng(1))
    vps = b.compute_viewpoints(ctx(frontiers=frontier_cloud_flu(pts)))
    assert vps.shape[0] == 2
    got = sorted(tuple(np.round(v, 0)) for v in vps)
    assert got[0][0] == pytest.approx(-20.0, abs=1.0)
    assert got[1][0] == pytest.approx(20.0, abs=1.0)


def test_low_frontiers_are_dropped_by_min_altitude():
    low = _cluster([15.0, 0.0, 0.4])            # under the 1.5 m floor
    high = _cluster([-15.0, 0.0, 7.0])
    b = FrontierBehavior(rng=np.random.default_rng(1))
    vps = b.compute_viewpoints(
        ctx(frontiers=frontier_cloud_flu(np.vstack([low, high]))))
    assert vps.shape[0] == 1
    assert vps[0][0] < 0


def test_centroid_below_two_metres_is_not_a_viewpoint():
    # Above min_altitude (1.5) but the centroid lands under the OG 2.0 gate.
    pts = _cluster([12.0, 0.0, 1.7], spread=0.05)
    b = FrontierBehavior(rng=np.random.default_rng(1))
    assert b.compute_viewpoints(
        ctx(frontiers=frontier_cloud_flu(pts))).shape[0] == 0


def test_waypoint_pair_and_lock():
    pts = _cluster([30.0, 0.0, 6.0], spread=0.05)
    b = FrontierBehavior(rng=np.random.default_rng(1))
    out = b.execute(ctx(frontiers=frontier_cloud_flu(pts)))
    assert out.waypoint_locked is True
    assert len(out.path) == 2
    wp1, wp2 = out.path
    # wp2 sits LEAD_DIST_M further along the same bearing.
    step = wp2 - wp1
    assert np.linalg.norm(step) == pytest.approx(fb.LEAD_DIST_M, abs=1e-6)
    assert np.dot(step[:2] / np.linalg.norm(step[:2]),
                  wp1[:2] / np.linalg.norm(wp1[:2])) == pytest.approx(1.0, abs=1e-6)


def test_unlocks_within_five_metres():
    pts = _cluster([3.0, 0.0, 6.0], spread=0.05)
    b = FrontierBehavior(rng=np.random.default_rng(1))
    out = b.execute(ctx(frontiers=frontier_cloud_flu(pts),
                        cur_pose=np.array([0.0, 0.0, 5.0])))
    assert out.waypoint_locked is False        # OG:106-107


def test_locked_waypoint_is_not_recomputed():
    pts = np.vstack([_cluster([40.0, 0.0, 6.0], spread=0.05),
                     _cluster([-40.0, 0.0, 6.0], spread=0.05)])
    b = FrontierBehavior(rng=np.random.default_rng(1))
    held = np.array([40.0, 0.0, 6.0])
    out = b.execute(ctx(frontiers=frontier_cloud_flu(pts),
                        waypoint_locked=True, target_waypoint=held,
                        target_waypoint2=held + np.array([2.0, 0, 0])))
    assert np.allclose(out.target_waypoint, held)


def test_momentum_prefers_the_direction_of_travel():
    """With a current waypoint ahead in +x, a 5 m viewpoint behind should lose
    to one slightly further ahead (OG:57-65 momentum term)."""
    ahead = _cluster([26.0, 0.0, 6.0], spread=0.02)
    behind = _cluster([-20.0, 0.0, 6.0], spread=0.02)
    b = FrontierBehavior(rng=np.random.default_rng(1))
    c = ctx(frontiers=frontier_cloud_flu(np.vstack([ahead, behind])),
            target_waypoint=np.array([100.0, 0.0, 6.0]))
    b.execute(c)
    # scores are stored per viewpoint in compute order
    assert b.last_scores.shape[0] == 2
    ahead_i = int(np.argmax(b.viewpoints[:, 0]))
    behind_i = 1 - ahead_i
    assert b.last_scores[ahead_i] < b.last_scores[behind_i]


def test_no_momentum_term_without_a_current_waypoint():
    ahead = _cluster([26.0, 0.0, 6.0], spread=0.02)
    behind = _cluster([-20.0, 0.0, 6.0], spread=0.02)
    b = FrontierBehavior(rng=np.random.default_rng(1))
    b.execute(ctx(frontiers=frontier_cloud_flu(np.vstack([ahead, behind]))))
    d = np.linalg.norm(b.viewpoints, axis=1)
    assert np.allclose(np.sort(b.last_scores), np.sort(d))


def test_random_pick_is_over_the_top_five():
    pts = np.vstack([_cluster([10.0 * (i + 1), 0.0, 6.0], spread=0.05)
                     for i in range(8)])
    c = ctx(frontiers=frontier_cloud_flu(pts))
    chosen = set()
    for seed in range(40):
        b = FrontierBehavior(rng=np.random.default_rng(seed))
        b.execute(c)
        chosen.add(round(float(b.viewpoints[b.chosen_index][0]), 0))
    assert len(chosen) > 1                       # it really is random
    assert len(chosen) <= fb.TOP_N               # ...but only over the top 5


def test_deviation_1_z_is_clamped_into_the_band():
    pts = _cluster([30.0, 0.0, 40.0], spread=0.05)
    b = FrontierBehavior(rng=np.random.default_rng(1))
    out = b.execute(ctx(frontiers=frontier_cloud_flu(pts),
                        min_altitude=3.0, max_altitude=15.0))
    for wp in out.path:
        assert 3.0 <= wp[2] <= 15.0


def test_deviation_2_viewpoints_outside_the_polygon_are_dropped():
    inside = _cluster([10.0, 0.0, 6.0], spread=0.05)
    outside = _cluster([200.0, 0.0, 6.0], spread=0.05)
    poly = np.array([[-50.0, -50.0], [50.0, -50.0], [50.0, 50.0], [-50.0, 50.0]])
    b = FrontierBehavior(rng=np.random.default_rng(1))
    vps = b.compute_viewpoints(
        ctx(frontiers=frontier_cloud_flu(np.vstack([inside, outside])),
            search_area_xy=poly))
    assert vps.shape[0] == 1
    assert vps[0][0] == pytest.approx(10.0, abs=1.0)


def test_no_frontiers_publishes_nothing():
    b = FrontierBehavior(rng=np.random.default_rng(1))
    out = b.execute(ctx(frontiers=None))
    assert out.path == []
    assert 'no viewpoints' in out.note


def test_frontier_table_mentions_counts():
    pts = _cluster([30.0, 0.0, 6.0], spread=0.05)
    c = ctx(frontiers=frontier_cloud_flu(pts))
    b = FrontierBehavior(rng=np.random.default_rng(1))
    b.execute(c)
    table = b.frontier_table(c)
    assert 'viewpoints=1' in table
