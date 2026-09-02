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


def test_novelty_default_is_off_and_env_enables_it(monkeypatch):
    """User call 2026-09-02: frontier REVISITS are allowed — only TARGET
    revisits are excluded. So the coverage-novelty weight defaults to 0.0
    and RAVEN_FRONTIER_NOVELTY_W re-enables the pre-rewrite bias."""
    assert fb.NOVELTY_WEIGHT == 0.0
    assert fb.NOVELTY_NEIGHBORHOOD_CELLS == 5
    import importlib
    monkeypatch.setenv('RAVEN_FRONTIER_NOVELTY_W', '100.0')
    importlib.reload(fb)
    try:
        assert fb.NOVELTY_WEIGHT == 100.0
    finally:
        monkeypatch.delenv('RAVEN_FRONTIER_NOVELTY_W')
        importlib.reload(fb)
    assert fb.NOVELTY_WEIGHT == 0.0


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


def test_high_frontiers_are_dropped_by_max_altitude():
    """DEVIATION 3 — the band has a ceiling, not just a floor.

    The OG filtered on `z > 1.5` only, so a frontier far above the mission
    ceiling stayed a candidate and clamp_z() then squashed its waypoint back
    down to max_altitude, yielding a viewpoint whose z bore no relation to the
    cluster it came from. Live setting for the t250 run is 1.0 - 25.0.
    """
    inside = _cluster([-15.0, 0.0, 7.0])
    above = _cluster([15.0, 0.0, 40.0])        # over a 25 m ceiling
    b = FrontierBehavior(rng=np.random.default_rng(1))
    vps = b.compute_viewpoints(
        ctx(frontiers=frontier_cloud_flu(np.vstack([inside, above])),
            min_altitude=1.0, max_altitude=25.0))
    assert vps.shape[0] == 1
    assert vps[0][0] < 0                        # the one that survived is `inside`
    assert vps[0][2] == pytest.approx(7.0, abs=1.0)


def test_default_max_altitude_keeps_og_behaviour():
    """The ceiling must be inert unless a mission sets max_altitude_agl.

    helpers.ctx() defaults to max_altitude=100.0 (params.py's default), so a
    frontier at 40 m is still a viewpoint — same as the OG floor-only filter.
    """
    high = _cluster([15.0, 0.0, 40.0])
    b = FrontierBehavior(rng=np.random.default_rng(1))
    vps = b.compute_viewpoints(ctx(frontiers=frontier_cloud_flu(high)))
    assert vps.shape[0] == 1
    assert vps[0][2] == pytest.approx(40.0, abs=1.0)


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


# ── anti-revisit / novelty ──────────────────────────────────────────────────

def _observed_block(x0, x1, y0, y1, cell_size_m=0.5):
    """Every cell whose index falls in the [x0,x1] x [y0,y1] metre box."""
    ix = range(int(np.floor(x0 / cell_size_m)), int(np.floor(x1 / cell_size_m)) + 1)
    iy = range(int(np.floor(y0 / cell_size_m)), int(np.floor(y1 / cell_size_m)) + 1)
    return {(a, b) for a in ix for b in iy}


def test_neighborhood_fraction_against_a_hand_built_set():
    """(c) The (2k+1)^2 window math, cell by cell (OG2:126-146)."""
    f = fb.neighborhood_observed_fraction
    pt = np.array([[0.5, 0.5]])                 # cell (0, 0) at 1 m cells

    # k=1 -> a 3x3 window, 9 cells.
    assert f(pt, {(0, 0)}, 1.0, 1)[0] == pytest.approx(1.0 / 9.0)
    assert f(pt, {(0, 0), (1, 1), (-1, 0)}, 1.0, 1)[0] == pytest.approx(3.0 / 9.0)
    full3 = {(a, b) for a in (-1, 0, 1) for b in (-1, 0, 1)}
    assert f(pt, full3, 1.0, 1)[0] == pytest.approx(1.0)
    # A cell just outside the window contributes nothing.
    assert f(pt, full3 | {(2, 0)}, 1.0, 1)[0] == pytest.approx(1.0)
    assert f(pt, {(2, 0)}, 1.0, 1)[0] == 0.0

    # Empty / None coverage is identically zero, whatever k.
    assert f(pt, set(), 1.0, 5)[0] == 0.0
    assert f(pt, None, 1.0, 5)[0] == 0.0
    assert f(np.zeros((0, 2)), {(0, 0)}, 1.0, 5).shape == (0,)

    # The shipped k=5 window is 11x11 = 121 cells at the 0.5 m coverage grid.
    pt_half = np.array([[10.1, -3.4]])          # cell (20, -7)
    assert f(pt_half, {(20, -7)}, 0.5, 5)[0] == pytest.approx(1.0 / 121.0)
    assert f(pt_half, _observed_block(7.0, 13.0, -6.5, -0.5), 0.5, 5)[0] \
        == pytest.approx(1.0)

    # Floor semantics match CoverageTracker.to_cells for negative coordinates.
    assert f(np.array([[-0.1, -0.1]]), {(-1, -1)}, 1.0, 0)[0] == pytest.approx(1.0)


def test_fully_observed_viewpoint_is_penalised_by_the_full_weight(monkeypatch):
    monkeypatch.setattr(fb, "NOVELTY_WEIGHT", 100.0)
    """(a, part 1) Two viewpoints at the same distance; the one standing in
    fully-observed cells pays exactly +NOVELTY_WEIGHT and loses the ordering."""
    stale = _cluster([30.0, 0.0, 6.0], spread=0.05)
    fresh = _cluster([-30.0, 0.0, 6.0], spread=0.05)
    observed = _observed_block(26.0, 34.0, -4.0, 4.0)
    c = ctx(frontiers=frontier_cloud_flu(np.vstack([stale, fresh])),
            observed_cells=observed, coverage_cell_size_m=0.5)
    b = FrontierBehavior(rng=np.random.default_rng(1))
    b.execute(c)

    stale_i = int(np.argmax(b.viewpoints[:, 0]))
    fresh_i = 1 - stale_i
    assert b.last_novelty[stale_i] == pytest.approx(fb.NOVELTY_WEIGHT)
    assert b.last_novelty[fresh_i] == 0.0
    # Same distance from the origin pose, so the gap IS the novelty term.
    d = np.linalg.norm(b.viewpoints, axis=1)
    assert d[stale_i] == pytest.approx(d[fresh_i], abs=0.2)
    assert (b.last_scores[stale_i] - b.last_scores[fresh_i]
            == pytest.approx(fb.NOVELTY_WEIGHT + d[stale_i] - d[fresh_i]))
    assert int(np.argsort(b.last_scores, kind='stable')[0]) == fresh_i


def test_covered_ground_is_never_picked_across_fifty_seeds(monkeypatch):
    monkeypatch.setattr(fb, "NOVELTY_WEIGHT", 100.0)
    """(a, part 2) The live symptom: a NEAR frontier sitting in ground the
    drone already cleared kept winning on distance alone. With the novelty term
    it is pushed out of the top-5 and never picked.

    Note the pick is still the OG uniform draw over the top-5, so novelty only
    REORDERS: with <= 5 viewpoints a stale one stays in the draw. That is the
    OG behaviour and is left alone — six candidates here so the top-5 cut bites.
    """
    stale = _cluster([20.0, 0.0, 6.0], spread=0.05)      # nearest of all
    fresh = [_cluster([-40.0 - 5.0 * i, 12.0 * i, 6.0], spread=0.05)
             for i in range(5)]
    observed = _observed_block(15.0, 25.0, -5.0, 5.0)
    c = ctx(frontiers=frontier_cloud_flu(np.vstack([stale] + fresh)),
            observed_cells=observed, coverage_cell_size_m=0.5)

    picks = []
    for seed in range(50):
        b = FrontierBehavior(rng=np.random.default_rng(seed))
        b.execute(c)
        assert b.viewpoints.shape[0] == 6
        picks.append(float(b.viewpoints[b.chosen_index][0]))
    assert all(p < 0.0 for p in picks)          # never the covered one at +20
    assert len(set(round(p) for p in picks)) > 1  # the OG randomness survives

    # Without the coverage set the same geometry picks the near stale one,
    # which is exactly the re-exploration the term exists to stop.
    c_blind = ctx(frontiers=c.frontiers)
    seen_stale = False
    for seed in range(50):
        b = FrontierBehavior(rng=np.random.default_rng(seed))
        b.execute(c_blind)
        seen_stale |= float(b.viewpoints[b.chosen_index][0]) > 0.0
    assert seen_stale


def test_no_observed_cells_reproduces_the_previous_scores_exactly():
    """(b) `observed_cells=None` -> the novelty term is identically 0 and the
    score is bit-for-bit `distance + momentum`, the pre-change formula."""
    pts = np.vstack([_cluster([26.0, 0.0, 6.0], spread=0.02),
                     _cluster([-20.0, 0.0, 6.0], spread=0.02),
                     _cluster([0.0, 30.0, 6.0], spread=0.02)])
    for wp in (None, np.array([100.0, 0.0, 6.0])):
        c = ctx(frontiers=frontier_cloud_flu(pts), target_waypoint=wp,
                waypoint_locked=bool(wp is not None))
        assert c.observed_cells is None          # the default
        b = FrontierBehavior(rng=np.random.default_rng(1))
        b.execute(c)

        pose = np.zeros(3)
        expected = np.linalg.norm(b.viewpoints - pose, axis=1)
        if wp is not None:
            motion = wp - pose
            motion = motion / (np.linalg.norm(motion) + 1e-6)
            cand = b.viewpoints - pose
            cand = cand / (np.linalg.norm(cand, axis=1, keepdims=True) + 1e-6)
            expected = expected + fb.MOMENTUM_WEIGHT * (1.0 - cand @ motion)
        assert np.array_equal(b.last_scores, expected)
        assert np.array_equal(b.last_novelty, np.zeros(b.viewpoints.shape[0]))

    # An EMPTY set is the same thing: recovers OG behaviour at mission start.
    c_empty = ctx(frontiers=frontier_cloud_flu(pts), observed_cells=set())
    b2 = FrontierBehavior(rng=np.random.default_rng(1))
    b2.execute(c_empty)
    assert np.array_equal(b2.last_novelty, np.zeros(b2.viewpoints.shape[0]))


def test_partial_coverage_scales_the_penalty_between_zero_and_the_weight(monkeypatch):
    monkeypatch.setattr(fb, "NOVELTY_WEIGHT", 100.0)
    pts = _cluster([30.0, 0.0, 6.0], spread=0.02)
    # Only the +x half-plane of the window is observed. The DBSCAN centroid
    # lands within a few cm of (30, 0) but not exactly on a cell edge, so the
    # expected count is derived from the centroid's own cell rather than
    # assumed — the point is that the fraction is strictly between 0 and 1.
    half = {(ix, iy) for ix in range(60, 200) for iy in range(-50, 51)}
    c = ctx(frontiers=frontier_cloud_flu(pts), observed_cells=half,
            coverage_cell_size_m=0.5)
    b = FrontierBehavior(rng=np.random.default_rng(1))
    b.execute(c)

    k = fb.NOVELTY_NEIGHBORHOOD_CELLS
    cx = int(np.floor(b.viewpoints[0, 0] / 0.5))
    cy = int(np.floor(b.viewpoints[0, 1] / 0.5))
    hits = sum(1 for dx in range(-k, k + 1) for dy in range(-k, k + 1)
               if (cx + dx, cy + dy) in half)
    assert 0 < hits < (2 * k + 1) ** 2
    assert b.last_novelty[0] == pytest.approx(
        fb.NOVELTY_WEIGHT * hits / float((2 * k + 1) ** 2), abs=1e-6)
    assert 0.0 < b.last_novelty[0] < fb.NOVELTY_WEIGHT


def test_frontier_table_reports_the_novelty_term(monkeypatch):
    monkeypatch.setattr(fb, "NOVELTY_WEIGHT", 100.0)
    pts = np.vstack([_cluster([30.0, 0.0, 6.0], spread=0.05),
                     _cluster([-30.0, 0.0, 6.0], spread=0.05)])
    observed = _observed_block(26.0, 34.0, -4.0, 4.0)
    c = ctx(frontiers=frontier_cloud_flu(pts), observed_cells=observed,
            coverage_cell_size_m=0.5)
    b = FrontierBehavior(rng=np.random.default_rng(1))
    b.execute(c)
    table = b.frontier_table(c)
    assert 'novelty' in table
    assert f'observed_cells={len(observed)}' in table
    assert 'k=5' in table and 'weight=100' in table
    # The fully-observed viewpoint's row carries the full +100.
    assert '100.00' in table
    # ...and the table still works with no coverage at all.
    assert 'observed_cells=0' in b.frontier_table(ctx(frontiers=c.frontiers))
