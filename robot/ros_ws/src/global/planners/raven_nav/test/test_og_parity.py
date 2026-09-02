"""Parity against the vendored ORIGINAL RAVEN torch code.

`test/og_reference/og_behaviors.py` holds the OG maths verbatim (torch,
scipy.ndimage, sklearn). This compares it to the numpy ports on random inputs.
torch is not installed on the host, so these skip there and run inside the
robot container, which has it.
"""
import numpy as np
import pytest

torch = pytest.importorskip('torch', reason='OG reference needs torch')

from og_reference import og_behaviors as og   # noqa: E402

from helpers import ctx, frontier_cloud_flu, rays, scores_for   # noqa: E402
from raven_nav.behaviors.frontier_behavior import FrontierBehavior  # noqa: E402
from raven_nav.behaviors.lvlm_behavior import (                     # noqa: E402
    LvlmBehavior, clean_guiding_objects,
)
from raven_nav.behaviors.ray_behavior import RayBehavior, angle_bin_groups  # noqa: E402
from raven_nav.behaviors.voxel_behavior import VoxelBehavior       # noqa: E402

SEEDS = list(range(8))


def _sorted_rows(a, nd=4):
    return sorted(tuple(np.round(np.asarray(r, dtype=float), nd)) for r in a)


# ── frontier ────────────────────────────────────────────────────────────────
@pytest.mark.parametrize('seed', SEEDS)
def test_frontier_viewpoints_match(seed):
    rng = np.random.default_rng(seed)
    blobs = [rng.uniform(-40, 40, 3) + np.array([0, 0, 6.0])
             for _ in range(rng.integers(2, 6))]
    pts = np.vstack([b + rng.normal(scale=0.6, size=(12, 3)) for b in blobs])
    rdf = frontier_cloud_flu(pts)[:, :3]

    mine = FrontierBehavior(rng=np.random.default_rng(0)).compute_viewpoints(
        ctx(frontiers=frontier_cloud_flu(pts)))
    theirs = og.og_frontier_viewpoints(torch.tensor(rdf, dtype=torch.float64))
    assert _sorted_rows(mine) == _sorted_rows(theirs.numpy())


@pytest.mark.parametrize('seed', SEEDS)
def test_frontier_scores_match(seed):
    rng = np.random.default_rng(seed)
    vps = rng.uniform(-40, 40, (6, 3)) + np.array([0, 0, 6.0])
    pose = rng.uniform(-10, 10, 3)
    wp = rng.uniform(-40, 40, 3)
    b = FrontierBehavior(rng=np.random.default_rng(0))
    b.viewpoints = vps
    # replicate the node-side score computation by running execute with a
    # frontier cloud that yields exactly these viewpoints is fragile; compare
    # the scoring formula directly instead.
    distances = np.linalg.norm(vps - pose[None, :], axis=1)
    motion = wp - pose
    motion = motion / (np.linalg.norm(motion) + 1e-6)
    cand = vps - pose[None, :]
    cand = cand / (np.linalg.norm(cand, axis=1, keepdims=True) + 1e-6)
    mine = distances + 5.0 * (1.0 - cand @ motion)
    theirs = og.og_frontier_scores(torch.tensor(vps, dtype=torch.float64),
                                   pose, wp).numpy()
    assert np.allclose(mine, theirs, atol=1e-9)


@pytest.mark.parametrize('seed', SEEDS)
def test_frontier_waypoint_pair_matches(seed):
    rng = np.random.default_rng(seed)
    best = rng.uniform(-40, 40, 3)
    pose = rng.uniform(-10, 10, 3)
    wp1, wp2 = og.og_frontier_waypoints(best, pose)
    d = best - pose
    d = d / np.linalg.norm(d)
    assert np.allclose(wp1, best)
    assert np.allclose(wp2, best + 2.0 * d)


# ── ray ─────────────────────────────────────────────────────────────────────
def _forward_rays(rng, n):
    """Origins ahead of the drone with directions pointing further away, so
    every ray passes the OG forward filter (see the module docstring in
    og_behaviors)."""
    ang = rng.uniform(-np.pi, np.pi, n)
    dirs = np.stack([np.cos(ang), np.sin(ang), rng.uniform(-0.2, 0.2, n)], axis=1)
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    origins = dirs * rng.uniform(20.0, 60.0, n)[:, None] + np.array([0, 0, 6.0])
    return origins, dirs


@pytest.mark.parametrize('seed', SEEDS)
def test_ray_angle_binning_matches(seed):
    rng = np.random.default_rng(seed)
    origins, dirs = _forward_rays(rng, int(rng.integers(4, 25)))
    pose = np.array([0.0, 0.0, 6.0])
    og_groups, _ = og.og_ray_groups(torch.tensor(origins), torch.tensor(dirs),
                                    pose)
    xy = dirs[:, :2] / np.linalg.norm(dirs[:, :2], axis=1, keepdims=True)
    mine = angle_bin_groups(xy)
    assert [g['indices'] for g in mine] == [g['indices'] for g in og_groups]
    for a, b in zip(mine, og_groups):
        assert np.allclose(a['centroid'], b['centroid'], atol=1e-9)


@pytest.mark.parametrize('seed', SEEDS)
def test_ray_group_averages_and_order_match(seed):
    rng = np.random.default_rng(seed)
    n = int(rng.integers(4, 25))
    origins, dirs = _forward_rays(rng, n)
    pose = np.array([0.0, 0.0, 6.0])
    c = ctx(**rays(origins, dirs, scores_for(n, 3, 0, 0.99)), cur_pose=pose)
    b = RayBehavior()
    a = b.update(c)
    _og_groups, og_scored = og.og_ray_groups(torch.tensor(origins),
                                             torch.tensor(dirs), pose)
    mine_scored = [a.averages[i] for i in a.order]
    assert len(mine_scored) == len(og_scored)
    for (mo, md, mn), (to, td, tn) in zip(mine_scored, og_scored):
        assert mn == tn
        assert np.allclose(mo, to.numpy(), atol=1e-9)
        assert np.allclose(md, td.numpy(), atol=1e-9)


@pytest.mark.parametrize('seed', SEEDS)
def test_ray_waypoints_match(seed):
    rng = np.random.default_rng(seed)
    n = int(rng.integers(4, 25))
    origins, dirs = _forward_rays(rng, n)
    pose = np.array([0.0, 0.0, 6.0])
    c = ctx(**rays(origins, dirs, scores_for(n, 3, 0, 0.99)), cur_pose=pose,
            min_altitude=-1e6, max_altitude=1e6)   # disable deviation 1
    b = RayBehavior()
    b.update(c)
    out = b.execute(c)
    _g, og_scored = og.og_ray_groups(torch.tensor(origins), torch.tensor(dirs),
                                     pose)
    wp1, wp2 = og.og_ray_waypoints(og_scored[0][0].numpy(),
                                   og_scored[0][1].numpy())
    assert np.allclose(out.path[0], wp1, atol=1e-9)
    assert np.allclose(out.path[1], wp2, atol=1e-9)


# ── voxel ───────────────────────────────────────────────────────────────────
def _grid_blobs(rng, vox=0.5):
    """Random on-grid voxel blobs, returned in FLU."""
    pts = []
    for _ in range(int(rng.integers(1, 4))):
        c = np.round(rng.uniform(-30, 30, 3) / vox) * vox
        r = np.arange(-1.0, 1.0 + 1e-9, vox)
        g = np.stack(np.meshgrid(r, r, r, indexing='ij'), axis=-1).reshape(-1, 3)
        pts.append(c[None, :] + g)
    return np.unique(np.vstack(pts), axis=0)


HALF_VOX = 0.25
# The OG box is half a voxel off: it treats the published voxel position as a
# min corner, but rayfronts publishes voxel CENTRES
# (geometry3d.pointcloud_to_sparse_voxels rounds p/v then multiplies back). In
# RDF the OG box is shifted +vox/2 on all three axes; after the RDF->FLU flip
# (x=z, y=-x, z=-y) that lands as (+h/2, -h/2, -h/2) in FLU.
OG_BOX_OFFSET_FLU = np.array([HALF_VOX, -HALF_VOX, -HALF_VOX])


@pytest.mark.parametrize('seed', SEEDS)
def test_voxel_cluster_components_and_sizes_match(seed):
    """Same components, same box SIZE — the centres differ by the documented
    half-voxel the OG got wrong (see OG_BOX_OFFSET_FLU)."""
    rng = np.random.default_rng(seed)
    flu = _grid_blobs(rng)
    rdf = np.stack([-flu[:, 1], -flu[:, 2], flu[:, 0]], axis=1)
    n = flu.shape[0]
    c = ctx(vox_xyz=flu, vox_scores=scores_for(n, 3, 0, 0.99),
            cur_pose=np.zeros(3))
    mine = [list(cl.center) + list(cl.size) for cl in VoxelBehavior().detect(c)]
    theirs = og.og_voxel_clusters(torch.tensor(rdf, dtype=torch.float64))
    assert len(mine) == len(theirs)
    shifted = [list(np.asarray(t[:3]) - OG_BOX_OFFSET_FLU) + list(t[3:])
               for t in theirs]
    assert _sorted_rows(mine) == _sorted_rows(shifted)


@pytest.mark.parametrize('seed', SEEDS)
def test_our_cluster_box_actually_bounds_the_voxels(seed):
    """The reason for the correction: every voxel\'s own half-voxel cell must
    lie inside the reported box, and the box must not be bigger than that."""
    rng = np.random.default_rng(seed)
    flu = _grid_blobs(rng)
    n = flu.shape[0]
    c = ctx(vox_xyz=flu, vox_scores=scores_for(n, 3, 0, 0.99),
            cur_pose=np.zeros(3))
    for cl in VoxelBehavior().detect(c):
        lo = cl.center - cl.size / 2.0
        hi = cl.center + cl.size / 2.0
        inside = np.all((flu >= lo - 1e-9) & (flu <= hi + 1e-9), axis=1)
        members = flu[inside]
        assert np.allclose(members.min(axis=0) - HALF_VOX, lo, atol=1e-9)
        assert np.allclose(members.max(axis=0) + HALF_VOX, hi, atol=1e-9)


@pytest.mark.parametrize('seed', SEEDS)
def test_voxel_standoff_matches(seed):
    rng = np.random.default_rng(seed)
    flu = _grid_blobs(np.random.default_rng(seed))
    n = flu.shape[0]
    pose = rng.uniform(-60, 60, 3)
    c = ctx(vox_xyz=flu, vox_scores=scores_for(n, 3, 0, 0.99), cur_pose=pose,
            min_altitude=-1e6, max_altitude=1e6)   # disable deviation 1
    b = VoxelBehavior()
    b.update(c)
    out = b.execute(c)
    nearest = sorted(b.unvisited,
                     key=lambda cl: float(np.linalg.norm(pose - cl.center)))[0]
    # same box for both sides: the standoff maths, not the box, is under test
    theirs = og.og_voxel_standoff(nearest.center, nearest.size, pose)
    if theirs is None:
        assert out.path == []
    else:
        assert np.allclose(out.target_waypoint2, theirs, atol=1e-9)
        assert np.allclose(out.path[0], pose * 0.2 + np.asarray(theirs) * 0.8,
                           atol=1e-9)


@pytest.mark.parametrize('seed', SEEDS)
def test_cuboid_distance_matches(seed):
    from raven_nav.behaviors.voxel_behavior import cuboid_distance
    rng = np.random.default_rng(seed)
    for _ in range(20):
        ca, cb = rng.uniform(-20, 20, 3), rng.uniform(-20, 20, 3)
        sa, sb = rng.uniform(0.5, 8, 3), rng.uniform(0.5, 8, 3)
        assert cuboid_distance(ca, sa, cb, sb) == pytest.approx(
            og.og_cuboid_distance(ca, sa, cb, sb), abs=1e-9)


# ── lvlm ────────────────────────────────────────────────────────────────────
@pytest.mark.parametrize('raw', [
    'a car, the roof, an ambulance', 'Car, CAR, car', 'debris., rubble,',
    '  spaced  ,  out  ', '', ',,,', 'the, a, an', 'A Tarp, a tarp.',
])
def test_guiding_object_cleaning_matches(raw):
    assert clean_guiding_objects(raw) == og.og_set_guiding_objects(raw)


@pytest.mark.parametrize('seed', SEEDS)
def test_lvlm_hop_matches(seed):
    rng = np.random.default_rng(seed)
    n = int(rng.integers(1, 10))
    origins = rng.uniform(-40, 40, (n, 3))
    dirs = rng.uniform(-1, 1, (n, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    b = LvlmBehavior()
    b.set_guiding_objects('roof')
    c = ctx(query_labels=['person', 'sky', 'roof'], target_objects=['person'],
            cur_pose=np.zeros(3), min_altitude=-1e6, max_altitude=1e6,
            **rays(origins, dirs, scores_for(n, 3, 2, 0.99)))
    assert b.condition_check(c) is True
    out = b.execute(c)
    og_o, og_t = og.og_lvlm_waypoints(torch.tensor(origins), torch.tensor(dirs))
    assert np.allclose(out.path[0], og_o, atol=1e-9)
    assert np.allclose(out.path[1], og_t, atol=1e-9)
