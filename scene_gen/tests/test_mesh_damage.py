"""
Tests for the USD port of the mesh-damage operators.

These need only `pxr` — an in-memory stage with a synthetic box stands in for a
building — so they run on the host with the rest of the suite. No Isaac, no
Nucleus, no assets.

The properties worth guarding are the ones that were easy to get wrong in the
port and would be invisible in a screenshot:

* the world-space round trip through a non-identity transform;
* `lean` and `pancake` leaving the FOOTPRINT alone — a building racks and
  collapses onto its own slab, it does not slide down the street;
* scale-relativity — the same profile on a 15 m house and a 90 m tower should
  do proportionally the same thing, which is why every size in the operators is
  a fraction of the bounding radius rather than a distance in metres;
* determinism, and severity 0 being an exact no-op.
"""

import os
import sys

import numpy as np
import pytest
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from disaster import mesh_damage as M  # noqa: E402

# Stages are kept alive for the duration of a test: dropping the Python
# reference invalidates every prim handed out from it, which surfaces as
# "Accessed schema on invalid prim" a long way from the cause.
_STAGES = []


def box(scale=1.0, translate=(0.0, 0.0, 0.0), rot_z=0.0, n=4):
    """A subdivided unit box as a stand-in building. Returns its mesh prims."""
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    xf = UsdGeom.Xform.Define(st, "/B")
    api = UsdGeom.XformCommonAPI(xf)
    api.SetTranslate(Gf.Vec3d(*translate))
    api.SetRotate(Gf.Vec3f(0.0, 0.0, rot_z))
    api.SetScale(Gf.Vec3f(scale, scale, scale))
    mesh = UsdGeom.Mesh.Define(st, "/B/Mesh")
    mesh.GetPointsAttr().Set([
        Gf.Vec3f(ix / (n - 1) - 0.5, iy / (n - 1) - 0.5, iz / (n - 1))
        for iz in range(n) for ix in range(n) for iy in range(n)])
    mesh.GetFaceVertexCountsAttr().Set([4])
    mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
    return M.mesh_prims(xf.GetPrim())



def solid_box(scale=1.0, n=6):
    """A properly-faced box — six subdivided sides, real topology.

    `box()` above is a point lattice with a single placeholder face, which is
    all the deformation operators need (they only touch `points`). Fracture
    needs faces, so it needs this.
    """
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    xf = UsdGeom.Xform.Define(st, "/B")
    UsdGeom.XformCommonAPI(xf).SetScale(Gf.Vec3f(scale, scale, scale))
    mesh = UsdGeom.Mesh.Define(st, "/B/Mesh")

    verts, faces = [], []
    def grid(fn):
        base = len(verts)
        for i in range(n):
            for j in range(n):
                verts.append(fn(i / (n - 1), j / (n - 1)))
        for i in range(n - 1):
            for j in range(n - 1):
                a = base + i * n + j
                faces.append([a, a + 1, a + n + 1, a + n])
    grid(lambda u, v: Gf.Vec3f(u - 0.5, v - 0.5, 0.0))
    grid(lambda u, v: Gf.Vec3f(u - 0.5, v - 0.5, 1.0))
    grid(lambda u, v: Gf.Vec3f(u - 0.5, -0.5, v))
    grid(lambda u, v: Gf.Vec3f(u - 0.5, 0.5, v))
    grid(lambda u, v: Gf.Vec3f(-0.5, u - 0.5, v))
    grid(lambda u, v: Gf.Vec3f(0.5, u - 0.5, v))

    mesh.GetPointsAttr().Set(verts)
    mesh.GetFaceVertexCountsAttr().Set([4] * len(faces))
    mesh.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])
    return M.mesh_prims(xf.GetPrim())


def shaded_box(scale=1.0, n=6):
    """A box carrying the per-face bookkeeping that face deletion must fix up.

    Real assets index three things by face — faceVarying `st`, uniform
    primvars, and `GeomSubset` material bindings — and all three go silently
    wrong if deletion renumbers the faces without them. Each is authored here
    with values that say which face they came from, so a test can assert they
    still line up afterwards.
    """
    prims = solid_box(scale=scale, n=n)
    prim = prims[0].GetPrim()
    mesh = UsdGeom.Mesh(prim)
    counts = list(mesh.GetFaceVertexCountsAttr().Get())
    n_faces, n_fv = len(counts), sum(counts)
    api = UsdGeom.PrimvarsAPI(prim)

    st = api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                           UsdGeom.Tokens.faceVarying)
    st.Set([Gf.Vec2f(float(i), 0.0) for i in range(n_fv)])
    tag = api.CreatePrimvar("origface", Sdf.ValueTypeNames.IntArray,
                            UsdGeom.Tokens.uniform)
    tag.Set(list(range(n_faces)))

    sub = UsdGeom.Subset.Define(prim.GetStage(),
                                prim.GetPath().AppendChild("half"))
    sub.CreateElementTypeAttr(UsdGeom.Tokens.face)
    sub.CreateIndicesAttr(list(range(0, n_faces, 2)))
    return prims


def face_tags(prim):
    """The `origface` values still on *prim*, i.e. which faces survived."""
    pv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("origface")
    return list(pv.Get())


def test_world_round_trip_under_a_transform():
    """Read world points, write them back unchanged, get the same geometry.

    The adapter is the entire Blender dependency that had to be replaced, and
    a wrong matrix convention here would quietly distort every damaged
    building. USD is row-vector: `world = local @ M[:3,:3] + M[3,:3]`.
    """
    prims = box(scale=2.0, translate=(10.0, -5.0, 3.0), rot_z=37.0)
    before = M.get_points(prims[0]).copy()
    M.set_points(prims[0], before)
    assert np.abs(M.get_points(prims[0]) - before).max() < 1e-9


def test_bounds_reflect_the_world_transform():
    prims = box(scale=2.0, translate=(10.0, -5.0, 3.0))
    b = M.bounds_of(prims)
    assert b is not None
    assert b.height == pytest.approx(2.0, abs=1e-6)   # unit box x scale 2
    assert b.base_z == pytest.approx(3.0, abs=1e-6)   # translated to z=3


@pytest.mark.parametrize("op", ["lean", "pancake"])
def test_footprint_stays_put(op):
    """A building racks or collapses onto its own slab — the base does not move.

    This is the property that makes mesh damage safe to apply after the layout
    is fixed: the geometry fails, the footprint does not migrate.
    """
    prims = box(scale=2.0, translate=(10.0, -5.0, 3.0), rot_z=37.0)
    b = M.bounds_of(prims)
    before = M.get_points(prims[0]).copy()
    if op == "lean":
        M.lean(prims, b, angle_deg=14.0, direction_deg=25.0)
    else:
        M.pancake(prims, b, z_lo=0.1, z_hi=0.35, collapse=0.8, spread=0.0)
    after = M.get_points(prims[0])
    at_base = before[:, 2] < b.base_z + 1e-6
    assert at_base.any()
    assert np.linalg.norm(after[at_base] - before[at_base], axis=1).max() < 1e-6
    assert np.linalg.norm(after - before, axis=1).max() > 1e-3   # did something


def test_profiles_are_scale_relative():
    """The same profile does proportionally the same thing at any size.

    The library spans 15 m houses to 90 m towers, so every size in the
    operators is a fraction of the bounding radius. A metre-valued knob would
    make one preset look right on exactly one asset.
    """
    fracs = []
    for scale in (1.0, 20.0):
        prims = box(scale=scale)
        before = M.get_points(prims[0]).copy()
        M.apply_profile(prims, "earthquake", 0.6, seed=7)
        moved = np.linalg.norm(M.get_points(prims[0]) - before, axis=1).max()
        fracs.append(moved / scale)          # height == scale for a unit box
    assert fracs[0] == pytest.approx(fracs[1], rel=1e-6)


def test_profiles_are_deterministic():
    out = []
    for _ in range(2):
        prims = box()
        M.apply_profile(prims, "tornado", 0.7, seed=11)
        out.append(M.get_points(prims[0]))
    assert np.abs(out[0] - out[1]).max() == 0.0


@pytest.mark.parametrize("disaster,profile", [
    ("earthquake", "structural_collapse"),
    ("explosion", "blast"),
    ("tornado", "wind_shear"),
    ("hurricane", "wind_shear"),
    ("fire", "burnout"),
    ("flood", "inundation"),
])
def test_every_disaster_type_maps_to_a_profile(disaster, profile):
    """The taxonomy is fixed; the mapping is the only thing that changes."""
    prims = box()
    before = M.get_points(prims[0]).copy()
    assert M.apply_profile(prims, disaster, 0.8, seed=3) == profile
    assert np.linalg.norm(M.get_points(prims[0]) - before, axis=1).max() > 1e-4


@pytest.mark.parametrize("disaster", ["earthquake", "tornado", "flood", "none"])
def test_zero_severity_is_an_exact_noop(disaster):
    prims = box()
    before = M.get_points(prims[0]).copy()
    M.apply_profile(prims, disaster, 0.0, seed=3)
    assert np.abs(M.get_points(prims[0]) - before).max() == 0.0


def test_flood_barely_deforms_but_earthquake_does():
    """Profiles differ in kind, not just degree — flood is not a small quake.

    `GENERATION.md` says flood causes little structural loss: things float
    away, buildings stay up. That has to show in the geometry.
    """
    def moved(disaster):
        prims = box()
        before = M.get_points(prims[0]).copy()
        M.apply_profile(prims, disaster, 0.8, seed=5)
        return np.linalg.norm(M.get_points(prims[0]) - before, axis=1).max()

    assert moved("flood") < 0.25 * moved("earthquake")


def test_blast_does_not_bury_the_building():
    """A blast throws a building outward; it does not push it into the ground.

    `shockwave` displaces radially from the epicentre, and the blast profile
    seats that epicentre low on a facade — so without a floor clamp every
    vertex below it is driven straight down. Measured on a real brownstone
    before the clamp: the base sank **1.29 m**, which reads as the building
    being swallowed rather than blown apart.
    """
    prims = box(scale=10.0)
    b0 = M.bounds_of(prims)
    M.apply_profile(prims, "explosion", 0.9, seed=13)
    b1 = M.bounds_of(prims)
    sank = b0.base_z - b1.base_z
    assert sank < 0.02 * b0.height, (
        f"base sank {sank:.3f} on a {b0.height:.1f} building — "
        "shockwave needs floor_z=bounds.base_z")


def test_deinstance_opens_nested_instances():
    """Assets ship internally instanced; their meshes cannot take an opinion.

    The AEC brownstone is `/World/Brownstone02_Instanced/...` — 307 meshes
    behind one instance root — and `set_points` raises on an instance proxy.
    Synthetic test geometry never reproduced this, which is why it survived
    until the port met a real asset.
    """
    from pxr import Sdf

    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    src = UsdGeom.Xform.Define(st, "/Proto")
    m = UsdGeom.Mesh.Define(st, "/Proto/Mesh")
    m.GetPointsAttr().Set([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0),
                           Gf.Vec3f(1, 1, 0), Gf.Vec3f(0, 1, 1)])
    inst = UsdGeom.Xform.Define(st, "/Inst").GetPrim()
    inst.GetReferences().AddInternalReference(Sdf.Path("/Proto"))
    inst.SetInstanceable(True)
    assert inst.IsInstance()

    assert M.deinstance(inst) >= 1
    prims = M.mesh_prims(inst)
    assert prims
    pts = M.get_points(prims[0])
    M.set_points(prims[0], pts + 1.0)          # would raise on a proxy
    assert np.abs(M.get_points(prims[0]) - (pts + 1.0)).max() < 1e-5


def test_earthquake_uses_more_than_one_failure_mode():
    """A quake-hit street is mixed, not a row of identical casualties.

    `structural_collapse` used to be one recipe — rack, pancake one storey,
    spall lightly — so every building failed the same way. It now picks among
    six modes, and which one is mostly a function of intensity.
    """
    modes = {M._pick_quake_mode(0.6, np.random.default_rng(i))
             for i in range(200)}
    assert len(modes) >= 4, f"only saw {modes}"


def test_severity_shifts_the_failure_mix_toward_collapse():
    """Low severity leaves buildings standing; high severity brings them down."""
    def share(intensity, wanted):
        picks = [M._pick_quake_mode(intensity, np.random.default_rng(i))
                 for i in range(400)]
        return sum(1 for p in picks if p in wanted) / len(picks)

    standing = {"racking", "spall"}
    collapsed = {"soft_story", "mid_story", "total", "partial"}
    assert share(0.2, standing) > share(0.9, standing)
    assert share(0.9, collapsed) > share(0.2, collapsed)
    assert share(0.9, collapsed) > 0.7


@pytest.mark.parametrize("mode,collapses", [
    ("racking", False), ("spall", False),
    ("soft_story", True), ("mid_story", True),
    ("total", True), ("partial", True),
])
def test_failure_modes_differ_in_kind(mode, collapses):
    """The standing modes keep their height; the collapse modes lose it.

    Guards the distinction that makes the mix worth having — if every mode
    squashed the building, picking between them would be decoration.
    """
    prims = box(scale=10.0, n=6)
    b0 = M.bounds_of(prims)
    M.structural_collapse(prims, 0.8, seed=5, mode=mode)
    lost = b0.height - M.bounds_of(prims).height
    if collapses:
        assert lost > 0.05 * b0.height, f"{mode} kept its height"
    else:
        assert lost < 0.05 * b0.height, f"{mode} collapsed but should stand"


def test_fracture_divides_the_shell_not_the_air():
    """Seeds come off the surface, so no one fragment holds the building.

    A building is a thin shell inside a mostly-empty bounding box. Seeding the
    box uniformly — which is what the Blender prototype does — puts most seeds
    in interior air and leaves one enormous cell with the bulk of the geometry:
    measured on a real brownstone, 31k of 61k triangles in a single fragment,
    i.e. not shattered at all. Surface seeding took that to 7-9%.
    """
    prims = solid_box(scale=10.0)
    b = M.bounds_of(prims)
    frags = M.fracture(prims, b, n_cells=24, seed=3)
    assert len(frags) >= 10
    sizes = sorted(len(f) for f in frags)
    assert sizes[-1] < 0.35 * sum(sizes), "one fragment holds most of the mesh"


def test_fracture_covers_the_source():
    """Cells partition the geometry — nothing is lost and nothing invented."""
    prims = solid_box(scale=10.0)
    b = M.bounds_of(prims)
    soup, _ = M._mesh_soup(prims)
    frags = M.fracture(prims, b, n_cells=20, seed=3)
    allv = np.concatenate([f.verts for f in frags])
    assert allv[:, 2].min() == pytest.approx(soup.verts[:, 2].min(), abs=1e-3)
    assert allv[:, 2].max() == pytest.approx(soup.verts[:, 2].max(), abs=1e-3)
    # Clipping adds geometry at the cuts and capping fills them, so the
    # fragment set is legitimately larger than the source — what is pinned is
    # that it is bounded, not that it is unchanged. Measured: 3.3x uncapped,
    # 21.9x capped on this uniformly-tessellated box (a real building runs
    # ~1.4x, because its faces are not all subdivided grids).
    assert sum(len(f) for f in frags) < 40 * len(soup)
    # And with capping off the old, tighter bound still holds.
    bare = M.fracture(prims, b, n_cells=20, seed=3, cap=False)
    assert sum(len(f) for f in bare) < 4 * len(soup)


def test_clip_by_plane_keeps_the_negative_half():
    v = np.array([[0, 0, 0], [2, 0, 0], [0, 2, 0]], dtype=float)
    f = np.array([[0, 1, 2]])
    cv, cf, _, _ = M._clip_by_plane(v, f, normal=[1, 0, 0], origin=[1, 0, 0])
    assert len(cf) > 0
    assert cv[np.unique(cf)][:, 0].max() <= 1.0 + 1e-9


def test_clip_interpolates_uvs_at_the_cut():
    """A fragment's UVs must follow its geometry, or it renders untextured.

    The cut invents vertices; each one needs the UV its position interpolates
    to. Getting this wrong is invisible in a unit test of positions alone and
    very visible on a building.
    """
    v = np.array([[0, 0, 0], [2, 0, 0], [0, 2, 0]], dtype=float)
    uv = np.array([[0, 0], [1, 0], [0, 1]], dtype=float)
    f = np.array([[0, 1, 2]])
    cv, cf, cuv, _ = M._clip_by_plane(v, f, [1, 0, 0], [1, 0, 0], uv=uv)
    assert cuv is not None and len(cuv) == len(cv)
    # u is x/2 everywhere on this triangle; the clip must preserve that.
    assert np.abs(cuv[:, 0] - cv[:, 0] / 2.0).max() < 1e-9


def test_fragments_land_where_they_were_cut_under_a_transform():
    """A fragment is a CHILD of the building, so it inherits its transform.

    Everything upstream works in world space, and authoring world points under
    a prim that already carries the placement's translate/rotate/scale applies
    all three twice. On the centimetre-authored AEC packs (`scale: 0.01`) that
    puts the debris a hundred times too small and a long way off. Only a
    non-identity transform exposes it, which is why it survived.
    """
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    xf = UsdGeom.Xform.Define(st, "/B")
    UsdGeom.XformCommonAPI(xf).SetTranslate(Gf.Vec3d(40.0, -12.0, 0.0))
    UsdGeom.XformCommonAPI(xf).SetRotate(Gf.Vec3f(0.0, 0.0, 31.0))
    UsdGeom.XformCommonAPI(xf).SetScale(Gf.Vec3f(0.01, 0.01, 0.01))
    src = solid_box(scale=1000.0, n=8)[0]
    m = UsdGeom.Mesh.Define(st, "/B/Mesh")
    for attr in ("GetPointsAttr", "GetFaceVertexCountsAttr",
                 "GetFaceVertexIndicesAttr"):
        getattr(m, attr)().Set(getattr(UsdGeom.Mesh(src), attr)().Get())

    prims = M.mesh_prims(xf.GetPrim())
    b0 = M.bounds_of(prims)
    cut = M.fracture_to_stage(st, xf.GetPrim(), b0, n_cells=14, seed=3,
                              throw=0.0)
    assert cut["paths"]
    v = np.concatenate([M.get_points(st.GetPrimAtPath(p))
                        for p in cut["paths"]])
    assert v.min(axis=0) == pytest.approx(b0.lo, abs=0.05 * b0.height)
    assert v.max(axis=0) == pytest.approx(b0.hi, abs=0.05 * b0.height)


def test_fragments_are_authored_and_the_source_is_hidden():
    """Fragments become real prims; the mesh they came from stops rendering.

    Deactivated rather than deleted — the source lives in a referenced layer
    where RemovePrim does not compose, the trap `prune_prims` documents.
    """
    prims = solid_box(scale=10.0)
    stage = prims[0].GetStage()
    root = stage.GetPrimAtPath("/B")
    b = M.bounds_of(prims)
    paths = M.fracture_to_stage(stage, root, b, n_cells=16, seed=3)["paths"]
    assert len(paths) >= 6
    assert all(stage.GetPrimAtPath(p).IsValid() for p in paths)
    assert not any(p.IsActive() for p in prims), "source mesh still rendering"


# ---------------------------------------------------------------------------
# removing material — the half the port was missing
# ---------------------------------------------------------------------------


def test_delete_faces_keeps_per_face_data_aligned():
    """Deleting faces renumbers them, and everything indexed by face follows.

    A stale faceVarying `st` slides every UV onto the wrong corner and the
    texture crawls across the facade; a stale `GeomSubset` paints the windows
    with the roof material. Both are invisible until an asset with real
    shading meets the operator, which is exactly how this class of bug ships.
    """
    prims = shaded_box(scale=4.0)
    prim = prims[0].GetPrim()
    counts = list(UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get())
    n_faces = len(counts)
    before_sub = set(UsdGeom.Subset(
        prim.GetChild("half")).GetIndicesAttr().Get())

    keep = np.ones(n_faces, dtype=bool)
    keep[::3] = False                       # drop every third face
    assert M.delete_faces(prim, keep) == int((~keep).sum())

    mesh = UsdGeom.Mesh(prim)
    survivors = face_tags(prim)
    assert survivors == list(np.nonzero(keep)[0]), "uniform data misaligned"

    # faceVarying `st` was authored as its own face-vertex slot number, so the
    # surviving values must be exactly the slots of the surviving faces.
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    want = [s + k for f in survivors for s, k in
            [(starts[f], j) for j in range(counts[f])]]
    got = [int(v[0]) for v in
           UsdGeom.PrimvarsAPI(prim).GetPrimvar("st").Get()]
    assert got == want, "faceVarying st misaligned"

    # The subset must still name the same faces, in the new numbering.
    new_idx = UsdGeom.Subset(prim.GetChild("half")).GetIndicesAttr().Get()
    assert ([survivors[i] for i in new_idx]
            == sorted(before_sub & set(survivors))), "subset misaligned"
    assert len(mesh.GetFaceVertexCountsAttr().Get()) == len(survivors)


def test_punch_hole_actually_removes_faces():
    """The operator that makes a building broken rather than merely bent."""
    prims = solid_box(scale=10.0, n=12)
    b = M.bounds_of(prims)
    before = len(UsdGeom.Mesh(prims[0]).GetFaceVertexCountsAttr().Get())
    removed = M.punch_hole(prims, b, (1.0, 0.0, 0.5), radius=0.25, seed=3)
    after = len(UsdGeom.Mesh(prims[0]).GetFaceVertexCountsAttr().Get())
    assert removed > 0
    assert after == before - removed


def test_punch_hole_refuses_faces_far_larger_than_itself():
    """`max_face_span` is what replaces the prototype's `refine`.

    On a coarse asset a single face can be bigger than the whole hole. Testing
    its centroid would delete an entire wall for a 1 m opening — worse than
    doing nothing, which is what the guard makes it do instead.
    """
    prims = solid_box(scale=10.0, n=2)        # one enormous face per side
    b = M.bounds_of(prims)
    assert M.punch_hole(prims, b, (1.0, 0.0, 0.5), radius=0.02, seed=3) == 0


def test_surface_points_land_on_the_building():
    """Sampled points must be ON geometry — the interior is hollow.

    A point drawn from the bounding box lands in empty space most of the time
    and the hole punched there silently does nothing.
    """
    prims = solid_box(scale=10.0, n=8)
    b = M.bounds_of(prims)
    pts = M.surface_points(prims, 30, b, seed=5)
    assert len(pts) == 30
    for x, y, z in pts:
        # every face of a box is on a bounding plane
        assert (max(abs(x), abs(y)) > 0.95 or z < 0.02 or z > 0.98), (x, y, z)


def test_surface_points_respect_a_height_band():
    prims = solid_box(scale=10.0, n=8)
    b = M.bounds_of(prims)
    pts = M.surface_points(prims, 20, b, seed=5, z_range=(0.7, 1.0))
    assert pts and all(p[2] >= 0.7 - 1e-6 for p in pts)


@pytest.mark.parametrize("disaster", ["earthquake", "tornado", "explosion",
                                     "fire"])
def test_destructive_profiles_remove_material(disaster):
    """Deformation alone leaves every wall standing. These must not."""
    prims = solid_box(scale=10.0, n=12)
    before = len(UsdGeom.Mesh(prims[0]).GetFaceVertexCountsAttr().Get())
    M.apply_profile(prims, disaster, 0.85, seed=4)
    after = len(UsdGeom.Mesh(prims[0]).GetFaceVertexCountsAttr().Get())
    assert after < before, f"{disaster} left the envelope intact"


def test_wind_takes_the_top_not_the_base():
    """Hurricane and tornado tear the roof off; they do not level a building.

    The distinction is the whole point of the height band — without it the only
    fracture available is all-or-nothing, and a storm that shatters a tower to
    its foundations is the wrong picture.
    """
    prims = solid_box(scale=10.0, n=12)
    b = M.bounds_of(prims)
    M.wind_shear(prims, 0.9, seed=4, direction_deg=0.0)
    # A box's faces start evenly spread in z, so comparing what SURVIVES high
    # against what survives low measures where the damage went.
    cen, _ = M._face_geometry(prims[0])
    left_high = (cen[:, 2] > b.base_z + 0.75 * b.height).sum()
    left_low = (cen[:, 2] < b.base_z + 0.25 * b.height).sum()
    assert left_low > left_high, "wind damage is not concentrated up top"


@pytest.mark.parametrize("disaster,banded,travels", [
    ("earthquake", False, False), ("explosion", False, True),
    ("tornado", True, True), ("hurricane", True, True),
    # Fire takes the top like wind does, and that is where the resemblance
    # ends: wind CARRIES the band downwind, fire consumes it and drops it
    # straight in. Same silhouette, opposite debris — if `throw` ever became
    # non-zero here, a burnt-out terrace would grow a downwind debris trail.
    ("fire", True, False),
])
def test_fracture_plans_differ_in_kind(disaster, banded, travels):
    """Which band comes apart, and whether its pieces go anywhere.

    Every preset used to emit the same `fracture:` block, so every disaster
    shattered buildings identically and only the debris around them differed.
    """
    plan = M.fracture_plan(disaster, 0.8)
    assert plan is not None
    if banded:
        lo, hi = plan["z_range"]
        assert hi == 1.0 and lo > 0.3, plan["z_range"]
    else:
        assert plan["z_range"] is None
    assert (plan["throw"] > 0.1) == travels, plan["throw"]


def test_flood_does_not_shatter():
    assert M.fracture_plan("flood", 1.0) is None


def test_banded_fracture_leaves_the_building_standing():
    """Only the band becomes fragments; the rest stays as its own prims.

    Which is also what makes it affordable: a fifth of the building needs a
    fifth of the cells, and the four fifths below keep their own materials,
    UVs and subsets untouched rather than being re-authored as bare meshes.
    """
    prims = solid_box(scale=10.0, n=12)
    stage = prims[0].GetStage()
    b = M.bounds_of(prims)
    n_before = len(UsdGeom.Mesh(prims[0]).GetFaceVertexCountsAttr().Get())

    paths = M.fracture_to_stage(stage, stage.GetPrimAtPath("/B"), b,
                                n_cells=16, seed=3, z_range=(0.7, 1.0),
                                keep_base=0.0, throw=0.0)["paths"]
    assert paths, "nothing came off"
    assert prims[0].IsActive(), "the whole building was consumed"

    cen, _ = M._face_geometry(prims[0])
    assert cen is not None and len(cen) < n_before
    top = b.base_z + 0.7 * b.height
    assert (cen[:, 2] > top).sum() == 0, "band was not removed from the source"
    for p in paths:
        v = M.get_points(stage.GetPrimAtPath(p))
        assert v[:, 2].min() >= top - 1e-6, "a fragment came from below the band"


def textured_material(stage, path="/Mat", opacity_from_alpha=False):
    """A material shaped like the real assets: albedo driven by a texture.

    This is what every Objaverse→USD conversion writes and what the AEC packs
    use — `diffuseColor` *connected* to a `UsdUVTexture`, plus roughness and
    metallic packed into two channels of a second one.
    """
    mat = UsdShade.Material.Define(stage, path)
    surf = UsdShade.Shader.Define(stage, path + "/Surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    tex = UsdShade.Shader.Define(stage, path + "/Albedo")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("./albedo.png")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f) \
        .ConnectToSource(tex.ConnectableAPI(), "rgb")

    orm = UsdShade.Shader.Define(stage, path + "/ORM")
    orm.CreateIdAttr("UsdUVTexture")
    orm.CreateOutput("g", Sdf.ValueTypeNames.Float)
    orm.CreateOutput("b", Sdf.ValueTypeNames.Float)
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float) \
        .ConnectToSource(orm.ConnectableAPI(), "g")
    surf.CreateInput("metallic", Sdf.ValueTypeNames.Float) \
        .ConnectToSource(orm.ConnectableAPI(), "b")
    if opacity_from_alpha:
        tex.CreateOutput("a", Sdf.ValueTypeNames.Float)
        surf.CreateInput("opacity", Sdf.ValueTypeNames.Float) \
            .ConnectToSource(tex.ConnectableAPI(), "a")

    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    return mat, surf, tex, orm


def test_scorch_darkens_a_texture_driven_material():
    """Setting `diffuseColor` does nothing when a texture is connected to it.

    A connection beats a value in USD, so the old `inp.Set(darker)` was a
    silent no-op on essentially every real building — the exact wall the
    prototype hit in Blender. Soot has to go somewhere composition will
    actually consult: `UsdUVTexture.inputs:scale`, which multiplies whatever
    was sampled.
    """
    prims = solid_box(scale=10.0)
    stage = prims[0].GetStage()
    mat, surf, tex, _orm = textured_material(stage)
    UsdShade.MaterialBindingAPI.Apply(prims[0].GetPrim()).Bind(mat)

    assert not tex.GetInput("scale"), "nothing scaled yet"
    M.scorch(prims, strength=1.0)

    assert tex.GetInput("scale"), "soot never reached the texture"
    scale = tex.GetInput("scale").Get()
    assert scale[0] < 0.5, f"albedo not darkened: {scale}"
    assert scale[0] == scale[1] == scale[2]


def test_scorch_leaves_alpha_and_shared_channels_alone():
    """Two ways of darkening a texture that would break the asset instead.

    Scaling all four components fades a building OUT as it chars, whenever the
    albedo's `.a` drives opacity. And roughness here is one channel of a
    texture whose other channel is metallic, so scaling that texture to roughen
    the surface would silently rewrite metallic too — which is why roughness is
    only touched when it is NOT connected.
    """
    prims = solid_box(scale=10.0)
    stage = prims[0].GetStage()
    mat, surf, tex, orm = textured_material(stage, opacity_from_alpha=True)
    UsdShade.MaterialBindingAPI.Apply(prims[0].GetPrim()).Bind(mat)

    M.scorch(prims, strength=1.0)
    assert tex.GetInput("scale").Get()[3] == pytest.approx(1.0), \
        "alpha was scaled — the building will char to transparent"
    assert not orm.GetInput("scale"), \
        "the roughness/metallic texture was scaled; metallic is now wrong"


def test_scorch_is_local_when_given_an_epicentre():
    """A charge blackens the wall it went off against, not the far side."""
    def soot_at(epicenter):
        prims = solid_box(scale=10.0)
        stage = prims[0].GetStage()
        mat, _s, tex, _o = textured_material(stage)
        UsdShade.MaterialBindingAPI.Apply(prims[0].GetPrim()).Bind(mat)
        b = M.bounds_of(prims)
        M.scorch(prims, strength=1.0, bounds=b, epicenter=epicenter,
                 radius=0.25)
        inp = tex.GetInput("scale")
        return float(inp.Get()[0]) if inp else 1.0

    # The box spans the whole bounds, so a point on it is always in reach;
    # a point far outside must not soot it.
    assert soot_at((1.0, 0.0, 0.2)) < 0.9
    assert soot_at((14.0, 14.0, 6.0)) == pytest.approx(1.0)


def test_scorch_compounds_once_per_material():
    """Materials are shared between prims; visiting twice would double the soot."""
    prims = solid_box(scale=10.0)
    stage = prims[0].GetStage()
    mat, _s, tex, _o = textured_material(stage)
    for p in prims:
        UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(mat)

    M.scorch(prims * 4, strength=1.0)          # same material, four visits
    assert tex.GetInput("scale").Get()[0] == pytest.approx(0.2, abs=1e-5)


def test_fire_chars_the_whole_building_and_takes_the_roof():
    """The burnt-out silhouette: walls standing, roof gone, everything black."""
    prims = solid_box(scale=10.0, n=12)
    stage = prims[0].GetStage()
    mat, _s, tex, _o = textured_material(stage)
    UsdShade.MaterialBindingAPI.Apply(prims[0].GetPrim()).Bind(mat)
    b = M.bounds_of(prims)

    assert M.apply_profile(prims, "fire", 0.9, seed=5) == "burnout"
    assert tex.GetInput("scale").Get()[0] < 0.6, "not charred"

    cen, _ = M._face_geometry(prims[0])
    left_high = (cen[:, 2] > b.base_z + 0.75 * b.height).sum()
    left_low = (cen[:, 2] < b.base_z + 0.25 * b.height).sum()
    assert left_low > left_high, "fire did not take the roof"


def test_only_freed_fragments_are_loose():
    """Pieces still standing on their footings must not be handed to PhysX.

    This is the prototype's `partition`, and skipping it is the one mistake
    that makes intensity meaningless: settle every fragment and gravity levels
    the building whatever the severity was, so a 0.3 quake and a 0.9 one both
    end as the same flat pile. Caught by the damage gallery, where a row of
    houses simply vanished into their own footprints.
    """
    prims = solid_box(scale=10.0, n=10)
    stage = prims[0].GetStage()
    b = M.bounds_of(prims)
    cut = M.fracture_to_stage(stage, stage.GetPrimAtPath("/B"), b,
                              n_cells=24, seed=3, keep_base=0.4, throw=0.0)
    assert cut["paths"]
    assert set(cut["loose"]) <= set(cut["paths"])
    assert cut["loose"], "nothing came loose at all"
    assert len(cut["loose"]) < len(cut["paths"]), "everything came loose"

    z_keep = b.base_z + 0.4 * b.height
    for p in cut["paths"]:
        low = M.get_points(stage.GetPrimAtPath(p))[:, 2].min()
        assert (low > z_keep) == (p in cut["loose"])


def test_a_racked_building_sheds_less_than_a_collapsed_one():
    """The failure mode and the fracture have to agree about what survived.

    `structural_collapse` picks among six modes and two of them leave the
    building standing — so shattering it to its foundations anyway would make
    the choice invisible the moment the pieces hit the ground.
    """
    keep = {m: M.fracture_plan("earthquake", 0.8, mode=m)["keep_base"]
            for m in ("racking", "spall", "soft_story", "total")}
    assert keep["racking"] > keep["soft_story"] > keep["total"]
    assert keep["spall"] > keep["total"]


def test_fragments_carry_uvs_and_a_material():
    """Otherwise shattered geometry renders as untextured white confetti.

    `st` and the binding are what make a fragment look like part of the
    building it came from rather than a paper scrap next to it.
    """
    prims = shaded_box(scale=10.0, n=8)
    stage = prims[0].GetStage()
    mat = UsdShade.Material.Define(stage, "/Mat")
    UsdShade.MaterialBindingAPI.Apply(prims[0].GetPrim()).Bind(mat)

    b = M.bounds_of(prims)
    paths = M.fracture_to_stage(stage, stage.GetPrimAtPath("/B"), b,
                                n_cells=14, seed=3)["paths"]
    assert paths
    frag = stage.GetPrimAtPath(paths[0])
    st = UsdGeom.PrimvarsAPI(frag).GetPrimvar("st")
    assert st and len(st.Get()) == len(UsdGeom.Mesh(frag).GetPointsAttr().Get())
    bound = UsdShade.MaterialBindingAPI(frag).ComputeBoundMaterial()[0]
    assert bound and bound.GetPath() == mat.GetPath()


def test_fragments_keep_every_material_they_span():
    """Not just the majority one — a small building's cells span several.

    Binding the majority alone survives on a tower cut into 50 cells, where a
    cell really is one material. On a house cut into 40 it is wrong in nearly
    every cell, and the damage gallery showed the result: a street of buildings
    shattering into one flat off-white.
    """
    prims = shaded_box(scale=10.0, n=8)
    stage = prims[0].GetStage()
    prim = prims[0].GetPrim()
    n_faces = len(UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get())

    roof = UsdShade.Material.Define(stage, "/Roof")
    wall = UsdShade.Material.Define(stage, "/Wall")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(wall)
    # The existing "half" subset stands in for a second material on the mesh.
    UsdShade.MaterialBindingAPI.Apply(prim.GetChild("half")).Bind(roof)

    b = M.bounds_of(prims)
    paths = M.fracture_to_stage(stage, stage.GetPrimAtPath("/B"), b,
                                n_cells=12, seed=3)["paths"]
    assert paths
    spanning = 0
    for p in paths:
        f = stage.GetPrimAtPath(p)
        bound = {UsdShade.MaterialBindingAPI(f).ComputeBoundMaterial()[0]
                 .GetPath()}
        for c in f.GetChildren():
            if c.IsA(UsdGeom.Subset):
                bound.add(UsdShade.MaterialBindingAPI(c)
                          .ComputeBoundMaterial()[0].GetPath())
        if len(bound) > 1:
            spanning += 1
    assert spanning, (
        f"no fragment of a {n_faces}-face two-material box kept both")


def stage_with_buildings(n, scale=10.0, faces=10):
    """A stage holding *n* separate box "buildings", with their placements."""
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    UsdGeom.Xform.Define(st, "/World")
    placements = []
    for i in range(n):
        path = f"/World/b{i}"
        xf = UsdGeom.Xform.Define(st, path)
        UsdGeom.XformCommonAPI(xf).SetTranslate(Gf.Vec3d(i * 40.0, 0.0, 0.0))
        proto = solid_box(scale=scale, n=faces)[0]
        m = UsdGeom.Mesh.Define(st, path + "/Mesh")
        m.GetPointsAttr().Set(UsdGeom.Mesh(proto).GetPointsAttr().Get())
        m.GetFaceVertexCountsAttr().Set(
            UsdGeom.Mesh(proto).GetFaceVertexCountsAttr().Get())
        m.GetFaceVertexIndicesAttr().Set(
            UsdGeom.Mesh(proto).GetFaceVertexIndicesAttr().Get())
        placements.append({"prim_path": path, "category": "house"})
    return st, placements


def compiled_disaster_config(dtype, severity=0.8, seed=42):
    """The `disaster:` block a real compiled preset carries for *dtype*."""
    import compile_disaster as cd

    block = cd.DISASTERS[dtype](severity, {}, (400.0, 400.0))
    block["type"] = dtype
    return {"seed": seed, "disaster": block}


@pytest.mark.parametrize("dtype", ["earthquake", "tornado", "hurricane",
                                   "fire", "explosion", "flood"])
def test_apply_to_stage_runs_the_compiled_config(dtype):
    """The seam between the compiler and the operators, on real config.

    Every previous bug in this path was a name mismatch rather than bad maths —
    a knob the compiler stopped emitting, a plan key spelled differently — and
    none of it shows up in a unit test that hand-builds its own config dict.
    """
    def run(thickness):
        st, placements = stage_with_buildings(3)
        for p in placements:
            p["_mesh_damage"] = 0.9
        cfg = compiled_disaster_config(dtype)
        if thickness is not None:
            cfg["disaster"].setdefault("mesh_damage", {})["thickness"] = \
                thickness

        def faces():
            return [len(UsdGeom.Mesh(st.GetPrimAtPath(p["prim_path"] + "/Mesh"))
                        .GetFaceVertexCountsAttr().Get())
                    for p in placements]

        before = faces()
        out = M.apply_to_stage(st, cfg, placements)
        return st, out, before, faces()

    # Damage only ever REMOVES material — as long as nothing is adding any.
    # Every operator in a profile either moves vertices or deletes faces, so
    # with `solidify` switched off the face count can only fall, and that is
    # still the invariant worth pinning: it is what catches a profile that has
    # quietly stopped opening the building up.
    st, out, before, after = run({"enabled": False})
    assert out["tally"], f"{dtype} did nothing at all"
    assert "thickened" not in out["tally"]
    assert all(a <= b for a, b in zip(after, before))

    # With thickening on — the default — every damaged building gets wall
    # volume first. The face count is no longer monotonic in either direction:
    # `solidify` adds an inner shell and a rim, and then `fire` and `tornado`
    # delete a height band of 55-60% of the building to turn it into fragments.
    # So only the BOUND is asserted here; that solidify actually doubles the
    # geometry is pinned directly in `test_solidify_*` instead, where nothing
    # else is deleting faces at the same time.
    st, out, before, after = run(None)
    assert out["tally"].get("thickened") == len(before), \
        "every mesh-damaged building should have been given wall volume"
    assert all(a <= 4 * b for a, b in zip(after, before))

    if dtype == "flood":
        assert not out["fragments"], "water does not shatter masonry"
    else:
        assert out["fragments"], f"{dtype} shattered nothing"
        assert all(st.GetPrimAtPath(f).IsValid() for f in out["fragments"])


def test_the_storm_bearing_is_read_off_the_compiled_config():
    """Every tornado building must throw its debris the SAME way.

    `heading_deg` is authored inside the `field` block and never at the top
    level, so reading only the top level returned None for every scene ever
    generated and each building picked a random bearing — turning the track's
    signature into a bomb site. Invisible to the unit test that passes
    `direction_deg` in by hand.
    """
    assert M._heading_of(compiled_disaster_config("tornado")["disaster"]) == 35.0

    def blown(heading):
        """Where this building's debris ends up at a given storm bearing."""
        st, placements = stage_with_buildings(1)
        placements[0]["_mesh_damage"] = 0.9
        cfg = compiled_disaster_config("tornado")
        cfg["disaster"]["field"]["heading_deg"] = heading
        M.apply_to_stage(st, cfg, placements)
        frags = list(st.GetPrimAtPath("/World/b0/fragments").GetChildren())
        assert frags
        return np.array([M.get_points(f).mean(axis=0)
                         for f in frags]).mean(axis=0)

    # Same seed, so the fracture is identical and only the throw differs.
    east, north = blown(0.0), blown(90.0)
    assert east[0] > north[0], "debris did not travel east on a due-east storm"
    assert north[1] > east[1], "debris did not travel north on a due-north one"


def test_shatter_budget_goes_to_the_worst_hit_buildings():
    """Not to whichever buildings the placement list happened to reach first.

    Placement order is essentially packing order, so a first-come budget hands
    the shattering to one corner of the map and leaves the epicentre — the one
    place a viewer looks — deformed but whole.
    """
    st, placements = stage_with_buildings(4)
    for p, k in zip(placements, (0.15, 0.2, 0.95, 0.3)):
        p["_mesh_damage"] = k
    cfg = compiled_disaster_config("earthquake")
    cfg["disaster"]["mesh_damage"]["fracture"]["max_buildings"] = 1

    frags = M.apply_to_stage(st, cfg, placements)["fragments"]
    assert frags
    assert all(f.startswith(placements[2]["prim_path"]) for f in frags), frags


def test_config_still_overrides_the_plan():
    """A preset can override any plan key; it just no longer has to supply it."""
    st, placements = stage_with_buildings(1)
    placements[0]["_mesh_damage"] = 0.9
    cfg = compiled_disaster_config("earthquake")
    cfg["disaster"]["mesh_damage"]["fracture"]["z_range"] = [0.8, 1.0]

    M.apply_to_stage(st, cfg, placements)
    cen, _ = M._face_geometry(st.GetPrimAtPath("/World/b0/Mesh"))
    # Only the top fifth was consumed, so the source keeps everything below it.
    assert cen is not None and (cen[:, 2] < 0.5 * 10.0).any()


def test_throw_is_directional_and_spares_the_base():
    """A windstorm carries debris one way; a collapse drops it where it stood.

    Cross-wind displacement must stay ~0 — that directionality is the whole
    distinction from a blast, per the prototype's docstring.
    """
    def centroids(throw):
        prims = solid_box(scale=10.0)
        stage = prims[0].GetStage()
        b = M.bounds_of(prims)
        paths = M.fracture_to_stage(stage, stage.GetPrimAtPath("/B"), b,
                                    n_cells=16, seed=3, direction_deg=0.0,
                                    throw=throw, keep_base=0.0)["paths"]
        return np.array([M.get_points(stage.GetPrimAtPath(p)).mean(axis=0)
                         for p in paths]), b

    c0, b = centroids(0.0)
    c1, _ = centroids(0.4)
    assert (c1[:, 0] - c0[:, 0]).max() > 0.05 * b.height   # went downwind
    assert np.abs(c1[:, 1] - c0[:, 1]).max() < 1e-6        # not sideways


# ---------------------------------------------------------------------------
# solidify — giving the shell volume before it is broken
# ---------------------------------------------------------------------------


def _face_count(prim):
    return len(UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get())


def _point_count(prim):
    return len(UsdGeom.Mesh(prim).GetPointsAttr().Get())


def test_solidify_doubles_the_shell_and_rims_its_edges():
    """The whole mechanism, on geometry whose answer can be counted by hand.

    One open grid — a single wall panel, which is what an asset is made of —
    has `n*n` points and `(n-1)^2` quads, and every edge of its outer ring is a
    boundary. Solidified it must hold exactly twice the points, twice the
    faces plus one rim quad per boundary edge, and nothing else.
    """
    n = 6
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    mesh = UsdGeom.Mesh.Define(st, "/Wall")
    verts = [Gf.Vec3f(i / (n - 1) * 4.0, 0.0, j / (n - 1) * 3.0)
             for i in range(n) for j in range(n)]
    faces = []
    for i in range(n - 1):
        for j in range(n - 1):
            a = i * n + j
            faces.append([a, a + 1, a + n + 1, a + n])
    mesh.GetPointsAttr().Set(verts)
    mesh.GetFaceVertexCountsAttr().Set([4] * len(faces))
    mesh.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])

    added = M.solidify(mesh.GetPrim(), 0.25)
    assert added

    assert _point_count(mesh.GetPrim()) == 2 * n * n
    # A grid's boundary is its outer ring: 4*(n-1) edges.
    assert _face_count(mesh.GetPrim()) == 2 * (n - 1) ** 2 + 4 * (n - 1)

    # Every point has a twin exactly `thickness` away, and the panel is now a
    # slab: it gained depth on the one axis it had none.
    pts = np.array(mesh.GetPointsAttr().Get())
    outer, inner = pts[:n * n], pts[n * n:]
    assert np.allclose(np.linalg.norm(inner - outer, axis=1), 0.25)
    assert abs(pts[:, 1].max() - pts[:, 1].min() - 0.25) < 1e-5


def test_solidify_never_inflates_the_building():
    """Walls thicken INWARD.

    The layout stage packs buildings to setbacks, so a wall that grows outward
    walks into its neighbour. The direction is decided per point against the
    building's own box precisely so this holds on assets whose winding is
    inconsistent — which the library's are.
    """
    st, placements = stage_with_buildings(1, scale=10.0, faces=8)
    prims = M.mesh_prims(st.GetPrimAtPath(placements[0]["prim_path"]))
    before = M.bounds_of(prims)

    M.solidify_prims(prims, 0.25, bounds=before)

    after = M.bounds_of(prims)
    assert np.all(after.hi <= before.hi + 1e-6)
    assert np.all(after.lo >= before.lo - 1e-6)


def test_solidify_thickens_a_hollow_closed_box():
    """A closed surface is not the same thing as a solid, and this is the case
    that mattered.

    This test used to assert the OPPOSITE — that a closed box is skipped —
    because `solidify` decided solidity from the divergence-theorem volume,
    which for anything closed is the volume of the AIR inside it. Measured on
    the packs the generator actually uses:

        BG_Building_D   enclosed  50,617 m3 -> "solid"   real thickness 2.00 m
        BG_Building_A   enclosed 143,792 m3 -> "solid"   real thickness 3.11 m

    So every Nucleus building in a downtown scene was left as a paper balloon
    and fracturing one exposed zero-thickness walls at every cut. The guard is
    now `wall_thickness`, a ray probe for material actually behind the surface,
    and an 8 m hollow box is what it is: a shell.
    """
    prim = solid_box(scale=8.0, n=4)[0]
    mesh = UsdGeom.Mesh(prim.GetStage().GetPrimAtPath("/B/Mesh"))
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    centre = (pts.max(axis=0) + pts.min(axis=0)) * 0.5

    n0 = _face_count(mesh.GetPrim())
    assert M.solidify(mesh.GetPrim(), 0.25, ref_centre=centre) > 0, \
        "a hollow box has no material behind its walls and must be thickened"
    assert _face_count(mesh.GetPrim()) > n0


def test_solid_flag_in_the_asset_set_skips_thickening():
    """`solid: true` on an entry is the ONLY thing that stops thickening.

    The pipeline no longer classifies. Two attempts at that are in the history
    and both were wrong in practice — the enclosed volume (which for a closed
    building is the air in its rooms, so every Nucleus building was declared
    solid and left a paper balloon) and then a ray probe (right, but a guess
    about art remade every run). The author of the asset knows; the asset set
    records it.
    """
    def run(solid):
        st, placements = stage_with_buildings(2)
        for p in placements:
            p["_mesh_damage"] = 0.9
            p["usd"] = "airstack://packs/box.usd"
        cfg = compiled_disaster_config("earthquake")
        entry = {"usd": "airstack://packs/box.usd"}
        if solid is not None:
            entry["solid"] = solid
        cfg["usds"] = {"buildings": {"damaged": [entry]}}
        cfg["disaster"].setdefault("mesh_damage", {})["subdivide"] = \
            {"enabled": False}
        return M.apply_to_stage(st, cfg, placements)["tally"]

    # Unmarked and explicitly false both thicken: the default is "shell",
    # because that is what nearly all of the library is and because failing to
    # thicken one leaves zero-thickness walls at every cut.
    for flag in (None, False):
        t = run(flag)
        assert t.get("thickened") == 2, f"solid={flag} should have thickened"
        assert "already_solid" not in t

    t = run(True)
    assert "thickened" not in t, "solid: true must not be thickened"
    assert t.get("already_solid") == 2


def test_solidify_keeps_uvs_and_materials_attached():
    """Fragments cut from a slab must still look like the building.

    `st` is faceVarying, `origface` is uniform and a `GeomSubset` is a
    face-index set, so all three have to grow onto the inner shell and the rim
    or the extrusion renders untextured and bound to whatever material sat on
    the prim — the "white confetti" failure `_bind_materials` documents,
    arriving by a different route.

    `origface` is what makes this checkable rather than merely plausible: it
    labels each source face with its own index, so after the extrusion every
    new face states which one it grew from, and the subset can be verified
    against it instead of against an assumed layout.
    """
    prim = shaded_box(scale=6.0, n=5)[0].GetPrim()
    mesh = UsdGeom.Mesh(prim)
    n_faces = _face_count(prim)
    st_pv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
    sub = next(UsdGeom.Subset(c) for c in prim.GetChildren()
               if c.IsA(UsdGeom.Subset))
    bound = set(int(i) for i in sub.GetIndicesAttr().Get())

    assert M.solidify(prim, 0.2)

    new_faces = _face_count(prim)
    new_fv = int(sum(mesh.GetFaceVertexCountsAttr().Get()))
    # Exactly doubled, and no rim: this box's six grids meet at its edges, so
    # welding by position closes it and there is no open edge to cap. The rim
    # is counted where there IS one, in the open-panel test above.
    assert new_faces == 2 * n_faces
    assert len(st_pv.Get()) == new_fv, "st must follow the new face-vertices"

    # Each new face says which source face it came from...
    origin = list(UsdGeom.PrimvarsAPI(prim).GetPrimvar("origface").Get())
    assert len(origin) == new_faces
    assert origin[:n_faces] == list(range(n_faces))          # outer, untouched
    assert origin[n_faces:2 * n_faces] == list(range(n_faces))   # inner twins
    assert all(0 <= o < n_faces for o in origin[2 * n_faces:])   # rim quads

    # ...and the material follows that same mapping, for every face.
    now = set(int(i) for i in sub.GetIndicesAttr().Get())
    assert now == {i for i in range(new_faces) if origin[i] in bound}


def test_a_single_mesh_building_keeps_its_fragments():
    """Consuming the source must not retire the rubble along with it.

    Half the library is a single mesh referenced straight at the placement
    path, so the prim `fracture_to_stage` consumes is also the prim it just
    authored the fragments under. Deactivating that prim takes the whole
    subtree, and a whole-building fracture — earthquake and explosion, the two
    that pass `z_range=None` — left nothing standing AND nothing on the ground.

    The multi-mesh case is the control: there the consumed prim is a sibling of
    the fragments scope, and retiring it is correct.
    """
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    src = solid_box(scale=10.0, n=8)[0]

    # The building prim IS the mesh, as a single-mesh asset composes.
    mesh = UsdGeom.Mesh.Define(st, "/World/house_0")
    for attr in ("GetPointsAttr", "GetFaceVertexCountsAttr",
                 "GetFaceVertexIndicesAttr"):
        getattr(mesh, attr)().Set(getattr(UsdGeom.Mesh(src), attr)().Get())

    root = mesh.GetPrim()
    b = M.bounds_of([root])
    out = M.fracture_to_stage(st, root, b, n_cells=24, seed=5,
                              keep_base=0.0, z_range=None)

    assert out["paths"], "the whole building should have come apart"
    assert root.IsActive(), \
        "retiring the source took the fragments with it — they are its children"
    assert all(st.GetPrimAtPath(p).IsValid() and st.GetPrimAtPath(p).IsActive()
               for p in out["paths"])
    # The source itself is spent: consumed entirely, so it holds no faces.
    assert len(mesh.GetFaceVertexCountsAttr().Get()) == 0
