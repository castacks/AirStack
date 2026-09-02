"""VoxelBehavior — the OG voxel go-to-object port."""
import numpy as np
import pytest

from helpers import ctx, scores_for, voxel_box
from raven_nav.behaviors import voxel_behavior as vb
from raven_nav.behaviors.voxel_behavior import (
    VoxelBehavior, connected_components, cuboid_distance,
)


def test_og_constants_unchanged():
    assert vb.VOXEL_SCORE_THRESHOLD == 0.98
    assert vb.VOX_SIZE_M == 0.5
    assert vb.MIN_CLUSTER_VOXELS == 30
    assert vb.VISITED_NEAR_M == 10.0
    assert vb.STANDOFF_M == 1.0
    assert vb.MID_ALPHA == 0.8
    assert vb.VISIT_REACH_M == 3.0


# ── connected components ────────────────────────────────────────────────────
def test_ccl_single_component():
    coords = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 1]])
    assert len(set(connected_components(coords).tolist())) == 1


def test_ccl_is_26_connected_not_6():
    """A pure diagonal neighbour is one component under the OG 3x3x3
    structuring element (voxel_behavior.py:69)."""
    coords = np.array([[0, 0, 0], [1, 1, 1]])
    assert len(set(connected_components(coords).tolist())) == 1


def test_ccl_splits_disconnected_blobs():
    coords = np.array([[0, 0, 0], [1, 0, 0], [10, 10, 10]])
    assert len(set(connected_components(coords).tolist())) == 2


def test_ccl_empty():
    assert connected_components(np.zeros((0, 3), dtype=int)).shape == (0,)


def test_cuboid_distance_zero_on_overlap():
    assert cuboid_distance([0, 0, 0], [2, 2, 2], [1, 0, 0], [2, 2, 2]) == 0.0


def test_cuboid_distance_is_surface_to_surface():
    d = cuboid_distance([0, 0, 0], [2, 2, 2], [10, 0, 0], [2, 2, 2])
    assert d == pytest.approx(8.0)


# ── detection ───────────────────────────────────────────────────────────────
def _one_box_ctx(center=(20.0, 0.0, 3.0), score=0.99, **kw):
    pts = voxel_box(center, half_extent=1.0)          # 5^3 = 125 voxels
    n = pts.shape[0]
    kw.setdefault('cur_pose', np.array([0.0, 0.0, 3.0]))
    return ctx(vox_xyz=pts, vox_scores=scores_for(n, 3, 0, score), **kw)


def test_detects_a_solid_block():
    b = VoxelBehavior()
    clusters = b.detect(_one_box_ctx())
    assert len(clusters) == 1
    c = clusters[0]
    assert c.label == 'person'
    assert c.center[0] == pytest.approx(20.0, abs=0.3)
    assert c.num_voxels == 125
    assert c.confidence == pytest.approx(0.99)


def test_below_threshold_detects_nothing():
    assert VoxelBehavior().detect(_one_box_ctx(score=0.97)) == []


def test_too_few_voxels_detects_nothing():
    pts = voxel_box((20.0, 0.0, 3.0), half_extent=0.5)   # 3^3 = 27 < 30
    c = ctx(vox_xyz=pts, vox_scores=scores_for(pts.shape[0], 3, 0, 0.99),
            cur_pose=np.array([0.0, 0.0, 3.0]))
    assert VoxelBehavior().detect(c) == []
    assert len(VoxelBehavior(min_cluster_size=20).detect(c)) == 1


def test_two_separated_blocks_are_two_clusters():
    pts = np.vstack([voxel_box((20.0, 0.0, 3.0)), voxel_box((-20.0, 0.0, 3.0))])
    c = ctx(vox_xyz=pts, vox_scores=scores_for(pts.shape[0], 3, 0, 0.99),
            cur_pose=np.array([0.0, 0.0, 3.0]))
    assert len(VoxelBehavior().detect(c)) == 2


def test_label_is_the_dominant_target_column():
    pts = voxel_box((20.0, 0.0, 3.0))
    n = pts.shape[0]
    s = np.full((n, 3), 0.001)
    s[:, 1] = 0.99                                     # 'car'
    c = ctx(vox_xyz=pts, vox_scores=s, cur_pose=np.array([0.0, 0.0, 3.0]),
            target_objects=['person', 'car'])
    clusters = VoxelBehavior().detect(c)
    assert clusters[0].label == 'car'


def test_non_target_columns_are_ignored():
    pts = voxel_box((20.0, 0.0, 3.0))
    n = pts.shape[0]
    s = np.full((n, 3), 0.001)
    s[:, 2] = 0.99                                     # 'road', not a target
    c = ctx(vox_xyz=pts, vox_scores=s, cur_pose=np.array([0.0, 0.0, 3.0]))
    assert VoxelBehavior().detect(c) == []


def test_box_extent_matches_the_block():
    b = VoxelBehavior()
    c = b.detect(_one_box_ctx())[0]
    # 5 voxels of 0.5 m across each axis = 2.5 m
    assert np.allclose(c.size, 2.5, atol=1e-6)


# ── condition + visiting ────────────────────────────────────────────────────
def test_condition_true_when_something_unvisited_exists():
    b = VoxelBehavior()
    c = _one_box_ctx()
    b.update(c)
    assert b.condition_check(c) is True


def test_condition_false_once_visited():
    b = VoxelBehavior()
    c = _one_box_ctx()
    b.update(c)
    b.mark_visited(b.clusters[0])
    b.update(c)
    assert b.condition_check(c) is False
    assert b.unvisited == []


def test_is_near_visited_uses_the_ten_metre_cuboid_gate():
    b = VoxelBehavior()
    b.visited_clusters.append(np.array([0, 0, 0, 2.0, 2.0, 2.0]))
    assert b.is_near_visited(np.array([5.0, 0, 0]), np.array([2.0, 2.0, 2.0]))
    assert not b.is_near_visited(np.array([50.0, 0, 0]), np.array([2.0, 2.0, 2.0]))


# ── execution ───────────────────────────────────────────────────────────────
def test_standoff_is_one_metre_short_of_the_surface():
    b = VoxelBehavior()
    c = _one_box_ctx(center=(20.0, 0.0, 3.0))
    b.update(c)
    out = b.execute(c)
    assert out.waypoint_locked is True
    wp1, wp2 = out.path
    # box spans x in [18.75, 21.25]; entry surface at 18.75, standoff 1 m short.
    assert wp2[0] == pytest.approx(17.75, abs=0.3)
    # wp1 is the 0.8 blend from the pose to wp2.
    assert wp1[0] == pytest.approx(0.8 * wp2[0], abs=1e-6)


def test_nearest_cluster_wins():
    pts = np.vstack([voxel_box((40.0, 0.0, 3.0)), voxel_box((10.0, 0.0, 3.0))])
    c = ctx(vox_xyz=pts, vox_scores=scores_for(pts.shape[0], 3, 0, 0.99),
            cur_pose=np.array([0.0, 0.0, 3.0]))
    b = VoxelBehavior()
    b.update(c)
    out = b.execute(c)
    assert out.path[1][0] < 20.0


def test_arriving_marks_visited_and_unlocks():
    b = VoxelBehavior()
    c = _one_box_ctx(center=(4.0, 0.0, 3.0))
    b.update(c)
    out = b.execute(c)
    # standoff sits ~1.75 m from the pose -> inside VISIT_REACH_M
    assert np.linalg.norm(c.cur_pose - out.target_waypoint2) < vb.VISIT_REACH_M
    assert out.waypoint_locked is False
    assert len(b.visited_clusters) == 1
    assert b.newly_visited and b.newly_visited[0].label == 'person'


def test_locked_waypoint_is_held_across_ticks():
    b = VoxelBehavior()
    c = _one_box_ctx()
    b.update(c)
    first = b.execute(c)
    held = first.target_waypoint2.copy()
    c2 = _one_box_ctx(cur_pose=np.array([5.0, 0.0, 3.0]),
                      waypoint_locked=True,
                      target_waypoint=first.target_waypoint,
                      target_waypoint2=held)
    b.update(c2)
    assert np.allclose(b.execute(c2).target_waypoint2, held)


def test_deviation_1_z_clamped():
    b = VoxelBehavior()
    c = _one_box_ctx(center=(20.0, 0.0, 0.5), min_altitude=3.0,
                     max_altitude=15.0, cur_pose=np.array([0.0, 0.0, 5.0]))
    b.update(c)
    for wp in b.execute(c).path:
        assert 3.0 <= wp[2] <= 15.0


def test_deviation_2_cluster_outside_the_polygon_is_skipped():
    poly = np.array([[-50.0, -50.0], [50.0, -50.0], [50.0, 50.0], [-50.0, 50.0]])
    b = VoxelBehavior()
    c = _one_box_ctx(center=(200.0, 0.0, 3.0), search_area_xy=poly)
    b.update(c)
    assert b.clusters, 'still DETECTED (it is reported)'
    assert b.unvisited == [], 'but never navigated to'
    assert b.condition_check(c) is False


def test_voxel_table_reports_state():
    b = VoxelBehavior()
    c = _one_box_ctx()
    b.update(c)
    t = b.voxel_table()
    assert 'voxel clusters=1' in t and 'unvisited' in t


# ── CCL implementations agree ───────────────────────────────────────────────
def test_the_two_ccl_paths_agree():
    """The vectorised key-packing path and the dict-probe fallback must label
    identically (the fallback only runs for absurd extents)."""
    from raven_nav.behaviors.voxel_behavior import _dict_probe_components
    rng = np.random.default_rng(4)
    coords = np.unique(rng.integers(0, 12, size=(400, 3)), axis=0)
    a = connected_components(coords)
    b = _dict_probe_components(coords)

    def partition(lbl):
        return sorted(sorted(np.nonzero(lbl == v)[0].tolist())
                      for v in set(lbl.tolist()))
    assert partition(a) == partition(b)


def test_ccl_labels_are_first_appearance_ordered():
    coords = np.array([[50, 50, 50], [0, 0, 0], [51, 50, 50]])
    assert connected_components(coords).tolist() == [0, 1, 0]


def test_ccl_scales_to_a_large_field():
    """A loose voxel_score_threshold leaves tens of thousands of voxels; the
    tick must not stall on them."""
    import time
    r = np.arange(0, 40)
    g = np.stack(np.meshgrid(r, r, r[:20], indexing='ij'), axis=-1).reshape(-1, 3)
    t0 = time.time()
    labels = connected_components(g)
    assert time.time() - t0 < 5.0
    assert len(set(labels.tolist())) == 1


# ── visited-BB suppression (detection memory -> voxel tier) ─────────────────
# Live 2026-09-02: "it's going to voxels inside existing BBs". The node's
# ctx.visited_bbs (confirmed detections with status 'visited') only fed the
# ray tier; the voxel tier's own visited_clusters never hears about a target
# visited through the detection pipeline, so the mapper's re-emitted clusters
# at a confirmed casualty re-fired forever.

def _bb(center, size):
    return np.concatenate([np.asarray(center, float), np.asarray(size, float)])


def test_visited_bb_pad_matches_ray_tier():
    from raven_nav.behaviors import ray_behavior as rb
    assert vb.VISITED_BB_PAD_M == rb.VISITED_RAY_PAD_M == 3.0


def test_cluster_inside_visited_bb_is_suppressed():
    b = VoxelBehavior()
    c = _one_box_ctx(visited_bbs=[_bb((20.0, 0.0, 3.0), (2.0, 2.0, 2.0))])
    b.update(c)
    assert b.clusters, 'detection itself must still see the cluster'
    assert b.unvisited == []
    assert b.condition_check(c) is False


def test_cluster_within_pad_of_visited_bb_is_suppressed():
    # Surface-to-surface gap ~2.75 m < VISITED_BB_PAD_M=3.0: the detected
    # cluster spans x 18.75..21.25 (125 voxels + half-voxel margins), the
    # visited bb face sits at x=24.0 (center 24.8, size 1.6).
    b = VoxelBehavior()
    c = _one_box_ctx(visited_bbs=[_bb((24.8, 0.0, 3.0), (1.6, 1.6, 1.6))])
    b.update(c)
    assert b.unvisited == []


def test_cluster_beyond_pad_of_visited_bb_survives():
    # Gap ~8.9 m > 3.0: visited bb center x=31, size 2 -> face at 30;
    # cluster face at ~21.1.
    b = VoxelBehavior()
    c = _one_box_ctx(visited_bbs=[_bb((31.0, 0.0, 3.0), (2.0, 2.0, 2.0))])
    b.update(c)
    assert len(b.unvisited) == 1


def test_no_visited_bbs_changes_nothing():
    b = VoxelBehavior()
    a = _one_box_ctx()                 # visited_bbs defaults to None
    b.update(a)
    assert len(b.unvisited) == 1


# ── multi-positive SUMMED mass (query "person, casualty") ────────────────────
# Near-synonym positives split the softmax at exactly the voxels both
# describe (live 2026-09-02: buried casualty solo 0.68 -> per-column max 0.43
# under person+casualty; sum 0.81). Detection thresholds the SUM of the
# target columns; a single target is unchanged (sum of one column).

def test_two_positives_summed_mass_crosses_the_gate():
    pts = voxel_box((20.0, 0.0, 3.0), half_extent=1.0)
    n = pts.shape[0]
    # 4 columns: person, casualty, road, tree — person/casualty split 0.4/0.35
    scores = np.zeros((n, 4))
    scores[:, 0] = 0.40
    scores[:, 1] = 0.35
    c = ctx(vox_xyz=pts, vox_scores=scores,
            query_labels=['person', 'casualty', 'road', 'tree'],
            target_objects=['person', 'casualty'],
            cur_pose=np.array([0.0, 0.0, 3.0]))
    b = VoxelBehavior(score_threshold=0.6)
    clusters = b.detect(c)
    assert len(clusters) == 1, 'sum 0.75 > 0.6 must fire though no column does'
    assert clusters[0].label == 'person'          # strongest single wording
    assert clusters[0].confidence == pytest.approx(0.75, abs=1e-6)


def test_two_positives_below_summed_gate_stay_silent():
    pts = voxel_box((20.0, 0.0, 3.0), half_extent=1.0)
    n = pts.shape[0]
    scores = np.zeros((n, 4))
    scores[:, 0] = 0.25
    scores[:, 1] = 0.20
    c = ctx(vox_xyz=pts, vox_scores=scores,
            query_labels=['person', 'casualty', 'road', 'tree'],
            target_objects=['person', 'casualty'],
            cur_pose=np.array([0.0, 0.0, 3.0]))
    assert VoxelBehavior(score_threshold=0.6).detect(c) == []
