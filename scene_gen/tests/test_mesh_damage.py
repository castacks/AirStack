"""
Tests for the mesh-damage pipeline: thicken, fail, propagate, settle.

Needs only `pxr` — an in-memory stage with a synthetic box stands in for a
building — so it runs on the host with the rest of the suite.

What is worth guarding is not "does it look broken", which no assertion
reaches, but the properties that are load-bearing and invisible in a
screenshot: that the failure fields differ IN KIND (an earthquake fails a
building at the base and a windstorm at the top, or the disaster axis means
nothing); that severity is monotone; that fragments are closed; that failure
propagates, or a collapsed building floats; that the USD bookkeeping survives
face renumbering; determinism; and scale-relativity across a 15 m house and a
90 m tower.
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

DISASTERS = ["earthquake", "tornado", "hurricane", "fire", "flood"]


# ---------------------------------------------------------------------------
# stand-in geometry
# ---------------------------------------------------------------------------


def _box_arrays(scale, n, height=1.0):
    """Points and quads of a closed box, `n` x `n` per side."""
    verts, faces = [], []

    def grid(fn):
        base = len(verts)
        for i in range(n):
            for j in range(n):
                verts.append(Gf.Vec3f(*[float(c) * scale
                                        for c in fn(i / (n - 1), j / (n - 1))]))
        for i in range(n - 1):
            for j in range(n - 1):
                a = base + i * n + j
                faces.append([a, a + 1, a + n + 1, a + n])

    h = height
    grid(lambda u, v: (u - .5, v - .5, 0.0))
    grid(lambda u, v: (u - .5, v - .5, h))
    grid(lambda u, v: (u - .5, -.5, v * h))
    grid(lambda u, v: (u - .5, .5, v * h))
    grid(lambda u, v: (-.5, u - .5, v * h))
    grid(lambda u, v: (.5, u - .5, v * h))
    return verts, faces


def box(stage=None, path="/B", scale=10.0, n=6, height=1.0,
        translate=(0.0, 0.0, 0.0), rot_z=0.0, xform_scale=1.0):
    """A closed box as a stand-in building. Returns its root prim."""
    if stage is None:
        stage = Usd.Stage.CreateInMemory()
        _STAGES.append(stage)
    xf = UsdGeom.Xform.Define(stage, path)
    api = UsdGeom.XformCommonAPI(xf)
    api.SetTranslate(Gf.Vec3d(*translate))
    api.SetRotate(Gf.Vec3f(0.0, 0.0, rot_z))
    api.SetScale(Gf.Vec3f(xform_scale, xform_scale, xform_scale))
    mesh = UsdGeom.Mesh.Define(stage, path + "/Mesh")
    verts, faces = _box_arrays(scale, n, height)
    mesh.GetPointsAttr().Set(verts)
    mesh.GetFaceVertexCountsAttr().Set([4] * len(faces))
    mesh.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])
    return xf.GetPrim()


def shaded_box(scale=4.0, n=6):
    """A box carrying the per-face bookkeeping that face deletion must fix up.

    Real assets index three things by face — faceVarying `st`, uniform
    primvars, and `GeomSubset` material bindings — and all three go silently
    wrong if deletion renumbers the faces without them. Each is authored here
    with values that say which face they came from.
    """
    prim = box(scale=scale, n=n)
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    mesh = UsdGeom.Mesh(mesh_prim)
    counts = list(mesh.GetFaceVertexCountsAttr().Get())
    n_faces, n_fv = len(counts), sum(counts)
    api = UsdGeom.PrimvarsAPI(mesh_prim)

    st = api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                           UsdGeom.Tokens.faceVarying)
    st.Set([Gf.Vec2f(float(i), 0.0) for i in range(n_fv)])
    tag = api.CreatePrimvar("origface", Sdf.ValueTypeNames.IntArray,
                            UsdGeom.Tokens.uniform)
    tag.Set(list(range(n_faces)))

    sub = UsdGeom.Subset.Define(mesh_prim.GetStage(),
                                mesh_prim.GetPath().AppendChild("half"))
    sub.CreateElementTypeAttr(UsdGeom.Tokens.face)
    sub.CreateIndicesAttr(list(range(0, n_faces, 2)))
    return prim


def open_edge_count(pts, faces, tol=1e-3):
    """Edges used by exactly one triangle — i.e. how far from closed this is."""
    w = M._weld(np.asarray(pts, dtype=np.float64), tol)
    ea = w[np.asarray(faces)[:, [0, 1, 2]].reshape(-1)]
    eb = w[np.asarray(faces)[:, [1, 2, 0]].reshape(-1)]
    key = np.stack([np.minimum(ea, eb), np.maximum(ea, eb)], axis=1)
    _, cnt = np.unique(key, axis=0, return_counts=True)
    return int((cnt == 1).sum())


def compiled_disaster_config(dtype, severity=0.8, seed=42):
    """The `disaster:` block a real compiled preset carries for *dtype*."""
    import compile_disaster as cd

    block = cd.DISASTERS[dtype](severity, {}, (400.0, 400.0))
    block["type"] = dtype
    return {"seed": seed, "disaster": block}


def stage_with_buildings(n, scale=10.0, faces=6):
    """A stage holding *n* separate box "buildings", with their placements."""
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    UsdGeom.Xform.Define(st, "/World")
    placements = []
    for i in range(n):
        path = f"/World/b{i}"
        box(st, path, scale=scale, n=faces, translate=(i * 40.0, 0.0, 0.0))
        placements.append({"prim_path": path, "category": "house", "usd": ""})
    return st, placements


# ---------------------------------------------------------------------------
# the USD adapter
# ---------------------------------------------------------------------------


def test_world_round_trip_under_a_transform():
    """Read world points, write them back unchanged, get the same geometry."""
    prim = box(translate=(31.0, -7.0, 2.0), rot_z=27.0, xform_scale=0.01)
    mesh = M.mesh_prims(prim)[0]
    before = M.get_points(mesh)
    M.set_points(mesh, before)
    assert np.abs(M.get_points(mesh) - before).max() < 1e-4


def test_bounds_reflect_the_world_transform():
    """`Bounds` is world space, so a placed building measures where it stands."""
    b = M.bounds_of(M.mesh_prims(box(translate=(100.0, 0.0, 0.0))))
    assert b.center[0] == pytest.approx(100.0, abs=0.5)


def test_deinstance_opens_nested_instances():
    """Assets ship internally instanced; their meshes cannot take an opinion.

    The AEC brownstone is `/World/Brownstone02_Instanced/...` — 307 meshes
    behind one instance root — and `set_points` raises on an instance proxy.
    Synthetic test geometry never reproduced this, which is why it survived
    until the port met a real asset.
    """
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    UsdGeom.Xform.Define(st, "/Proto")
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


# ---------------------------------------------------------------------------
# stage two — the failure fields, which is where a disaster is a disaster
# ---------------------------------------------------------------------------


def _sample(field, bounds, u=0.0, v=0.0, t=0.5):
    return float(field.damage(bounds.to_world((u, v, t))[None, :])[0])


#: A 3x3 plan grid. Averaging over it is what makes "this field fails the
#: base" a statement about the building rather than about one vertical line
#: through it — the fields are anisotropic in plan (windward bias, an
#: epicentre, a surviving envelope) and a single column samples one side.
_PLAN = [(u, v) for u in (-0.8, 0.0, 0.8) for v in (-0.8, 0.0, 0.8)]


def _profile(field, bounds, n=25, plan=None):
    """Mean damage at each height, base first."""
    plan = _PLAN if plan is None else plan
    return np.array([np.mean([_sample(field, bounds, u, v, t)
                              for u, v in plan])
                     for t in np.linspace(0.02, 0.98, n)])


UNIT = M.Bounds([0.0, 0.0, 0.0], [12.0, 12.0, 30.0])


@pytest.mark.parametrize("dtype", DISASTERS)
def test_every_disaster_type_has_a_field(dtype):
    f = M.failure_field(dtype, UNIT, 0.8, 3)
    assert f is not None and f.name == dtype


def test_none_and_zero_intensity_have_no_field():
    """Severity 0 must be an exact no-op, not a very small disaster."""
    assert M.failure_field("none", UNIT, 0.9, 3) is None
    for dtype in DISASTERS:
        assert M.failure_field(dtype, UNIT, 0.0, 3) is None


@pytest.mark.parametrize("dtype", DISASTERS)
def test_damage_is_a_probability(dtype):
    """The field is a fraction of integrity lost; nothing else is meaningful."""
    rng = np.random.default_rng(0)
    p = rng.random((400, 3)) * np.array([20.0, 20.0, 40.0]) - 4.0
    d = M.failure_field(dtype, UNIT, 0.9, 3).damage(p)
    assert d.shape == (400,)
    assert d.min() >= 0.0 and d.max() <= 1.0


@pytest.mark.parametrize("dtype", DISASTERS)
def test_fields_are_deterministic(dtype):
    """Same seed, same field; different seed, different field."""
    rng = np.random.default_rng(1)
    p = rng.random((200, 3)) * np.array([12.0, 12.0, 30.0])
    a = M.failure_field(dtype, UNIT, 0.7, 5).damage(p)
    b = M.failure_field(dtype, UNIT, 0.7, 5).damage(p)
    c = M.failure_field(dtype, UNIT, 0.7, 6).damage(p)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


@pytest.mark.parametrize("dtype", DISASTERS)
def test_more_severity_is_more_damage(dtype):
    """Monotone in intensity — the severity sweep depends on nothing else."""
    rng = np.random.default_rng(2)
    p = rng.random((600, 3)) * np.array([12.0, 12.0, 30.0])
    means = [float(M.failure_field(dtype, UNIT, i, 5).damage(p).mean())
             for i in (0.2, 0.5, 0.9)]
    assert means[0] < means[1] < means[2], means


def test_earthquake_fails_the_bottom_of_the_building():
    """Base shear: demand is the inertia of everything above, so it peaks low.

    This is the property that distinguishes an earthquake from every other
    field here, and the reason a quake-hit building settles onto its own
    footprint instead of losing its top.
    """
    base = top = 0.0
    for seed in range(8):
        prof = _profile(M.failure_field("earthquake", UNIT, 0.8, seed), UNIT)
        base += prof[:8].mean()
        top += prof[-8:].mean()
    assert base > 2.0 * top, f"base {base / 8:.2f} vs top {top / 8:.2f}"


def test_earthquake_fails_a_storey_rather_than_a_smooth_gradient():
    """Real buildings fail at a level, and the noise grain says so.

    A purely smooth height profile would give the same damage to every point at
    a given height in every building. The storey-shaped grain is what produces
    a *weak level* — and with it the soft-storey and mid-storey collapses that
    an earlier version had to enumerate as named modes with a weighted draw.
    """
    banded = 0
    for seed in range(12):
        prof = _profile(M.failure_field("earthquake", UNIT, 0.7, seed), UNIT)
        # A local maximum above the base means the worst level is not simply
        # the ground: there is structure in z beyond the shear ramp.
        interior = prof[2:-2]
        if interior.max() > prof[:2].max() * 1.05:
            banded += 1
    assert banded >= 4, f"only {banded}/12 buildings had a weak level"


@pytest.mark.parametrize("dtype", ["tornado", "hurricane"])
def test_wind_fails_the_top_of_the_building(dtype):
    """Wind load grows with height and the roof goes first — the opposite end
    of the building from an earthquake, which is what makes the two legible
    apart from a drone."""
    f = M.failure_field(dtype, UNIT, 0.8, 11, heading_deg=0.0)
    prof = _profile(f, UNIT)
    assert prof[-6:].mean() > 5.0 * prof[:6].mean()


def test_a_tornado_peels_deeper_than_a_hurricane():
    """Same mechanism, deliberately different depth — otherwise the two types
    are one type with two names."""
    def depth(dtype):
        f = M.failure_field(dtype, UNIT, 0.8, 11, heading_deg=0.0)
        return float((_profile(f, UNIT, n=50) >= 0.5).mean())
    assert depth("tornado") > depth("hurricane")


def test_wind_damage_is_biased_to_the_windward_face():
    """The pressure is on the face the storm hit, and that asymmetry is what
    a whole street of buildings shares in a windstorm."""
    f = M.failure_field("tornado", UNIT, 0.8, 11, heading_deg=0.0)
    # The storm travels toward +x, so it strikes the -x face.
    windward = _sample(f, UNIT, u=-0.95, t=0.9)
    leeward = _sample(f, UNIT, u=+0.95, t=0.9)
    assert windward > leeward


def test_wind_throws_everything_the_same_way():
    """Shared bearing is the storm signature — the difference between a track
    and a row of unrelated bomb sites."""
    f = M.failure_field("tornado", UNIT, 0.9, 11, heading_deg=90.0)
    rng = np.random.default_rng(3)
    p = rng.random((200, 3)) * np.array([12.0, 12.0, 30.0])
    p[:, 2] = 28.0                                   # up where it fails
    e = f.ejecta(p)
    moved = np.linalg.norm(e, axis=1) > 1e-6
    assert moved.any()
    assert (e[moved, 1] > 0).all(), "debris did not travel due north"
    assert np.abs(e[moved, 0]).max() < 1e-6, "debris drifted cross-wind"
    assert (e[moved, 2] > 0).all(), "nothing was lofted"
def test_fire_consumes_the_inside_and_spares_the_envelope():
    """Timber spans burn; masonry in compression does not. The surviving
    envelope IS the silhouette that distinguishes fire from wind."""
    f = M.failure_field("fire", UNIT, 0.9, 11)
    core = _sample(f, UNIT, u=0.0, v=0.0, t=0.95)
    wall = _sample(f, UNIT, u=0.98, v=0.0, t=0.95)
    assert core > wall * 1.5


def test_fire_drops_what_it_consumes_where_it_stood():
    """The opposite of wind: same silhouette from above, opposite debris."""
    f = M.failure_field("fire", UNIT, 0.9, 11)
    rng = np.random.default_rng(4)
    p = rng.random((50, 3)) * np.array([12.0, 12.0, 30.0])
    assert np.abs(f.ejecta(p)).max() == 0.0


def test_flood_damages_the_waterline_and_nothing_above_it():
    f = M.failure_field("flood", UNIT, 0.9, 11)
    prof = _profile(f, UNIT)
    assert prof[0] > 0.1
    assert prof[len(prof) // 2:].max() == 0.0


@pytest.mark.parametrize("dtype", DISASTERS)
def test_fields_are_scale_relative(dtype):
    """The same event on a 15 m house and a 90 m tower must do proportionally
    the same thing, or a severity sweep compares building sizes instead of
    severities."""
    small = M.Bounds([0, 0, 0], [8.0, 8.0, 15.0])
    large = M.Bounds([0, 0, 0], [48.0, 48.0, 90.0])
    # Averaged over seeds, because the noise grain is a world length and so is
    # deliberately NOT scale-relative: a storey is a storey on both buildings,
    # which is the whole point of `_grain` taking metres. What has to match is
    # the profile the grain modulates.
    a = np.mean([_profile(M.failure_field(dtype, small, 0.8, s), small, n=9)
                 for s in range(6)], axis=0)
    b = np.mean([_profile(M.failure_field(dtype, large, 0.8, s), large, n=9)
                 for s in range(6)], axis=0)
    assert abs(a.mean() - b.mean()) < 0.20, f"{a.mean():.2f} vs {b.mean():.2f}"
    # Not 1.0, and the earthquake is the loosest of the six on purpose: its
    # grain is a STOREY, so a 15 m building gets seven weak-level candidates
    # and a 90 m one eighteen. The profiles agree on where the building fails;
    # they cannot agree on exactly which level.
    assert np.corrcoef(a, b)[0, 1] > 0.85, "the two disagree about where it fails"


# ---------------------------------------------------------------------------
# propagation — failure spreading through the structure
# ---------------------------------------------------------------------------


def test_a_lost_ground_floor_releases_the_storeys_above_it():
    """The propagation rule, on a field that fails only the bottom.

    Without it a collapsed building floats: the base fragments come free and
    the intact upper storeys hang in the air above the gap.
    """
    b = M.Bounds([0, 0, 0], [10.0, 10.0, 30.0])
    ground = M.Failure(
        "test", lambda p: np.where(p[:, 2] < 4.0, 1.0, 0.0))
    up_high = np.array([[5.0, 5.0, 25.0], [5.0, 5.0, 15.0]])
    assert (M.release_column(up_high, ground, b) >= 0.8).all()


def test_an_intact_ground_floor_supports_what_is_above_it():
    """The converse, which is what stops a wind peel taking the whole tower."""
    b = M.Bounds([0, 0, 0], [10.0, 10.0, 30.0])
    roof = M.Failure("test", lambda p: np.where(p[:, 2] > 26.0, 1.0, 0.0))
    mid = np.array([[5.0, 5.0, 15.0]])
    assert float(M.release_column(mid, roof, b)[0]) == 0.0


# ---------------------------------------------------------------------------
# stage one — thickness
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
    verts, faces = [], []
    for i in range(n):
        for j in range(n):
            verts.append(Gf.Vec3f(i * 2.0, 0.0, j * 2.0))
    for i in range(n - 1):
        for j in range(n - 1):
            a = i * n + j
            faces.append([a, a + 1, a + n + 1, a + n])
    mesh.GetPointsAttr().Set(verts)
    mesh.GetFaceVertexCountsAttr().Set([4] * len(faces))
    mesh.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])

    prim = mesh.GetPrim()
    added = M.solidify(prim, 0.5, ref_centre=(5.0, 5.0, 5.0))
    assert added
    assert _point_count(prim) == 2 * n * n
    assert _face_count(prim) == 2 * (n - 1) ** 2 + 4 * (n - 1)


def test_solidify_never_inflates_the_building():
    """A wall that thickens OUTWARD is the one artifact this must not make.

    The layout stage packs buildings to setbacks; one that grows half a metre
    on every face starts intersecting its neighbours.
    """
    prim = box(scale=10.0)
    prims = M.mesh_prims(prim)
    before = M.bounds_of(prims)
    M.solidify_prims(prims, 0.5, bounds=before)
    after = M.bounds_of(prims)
    assert (after.lo >= before.lo - 1e-3).all()
    assert (after.hi <= before.hi + 1e-3).all()


def test_solidify_gives_a_hollow_box_wall_volume():
    """A cut through a solidified shell must show material, not a knife edge."""
    prim = box(scale=10.0)
    prims = M.mesh_prims(prim)
    n0 = _point_count(prims[0])
    got = M.solidify_prims(prims, 0.5, bounds=M.bounds_of(prims))
    assert got["meshes"] == 1
    assert _point_count(prims[0]) == 2 * n0


def test_solidify_keeps_uvs_and_materials_attached():
    """Solidify grows every face array, and anything indexed by face has to
    grow with it or USD drops the primvar as inconsistent."""
    prim = shaded_box()
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    mesh = UsdGeom.Mesh(mesh_prim)
    n_faces0 = len(mesh.GetFaceVertexCountsAttr().Get())
    M.solidify(mesh_prim, 0.3, ref_centre=M.bounds_of([mesh_prim]).center)

    counts = mesh.GetFaceVertexCountsAttr().Get()
    st = UsdGeom.PrimvarsAPI(mesh_prim).GetPrimvar("st")
    tag = UsdGeom.PrimvarsAPI(mesh_prim).GetPrimvar("origface")
    assert len(st.Get()) == sum(counts), "faceVarying st no longer matches"
    assert len(tag.Get()) == len(counts), "uniform primvar no longer matches"
    sub = UsdGeom.Subset(mesh_prim.GetChild("half"))
    ind = np.asarray(sub.GetIndicesAttr().Get())
    assert ind.max() < len(counts), "subset indexes faces that do not exist"
    # The inner shell of face f is at f + n_faces0, so the subset must have
    # picked up its twins.
    assert len(ind) > n_faces0 // 2


# ---------------------------------------------------------------------------
# topology bookkeeping
# ---------------------------------------------------------------------------


def test_delete_faces_keeps_per_face_data_aligned():
    """Deleting faces renumbers them, and everything indexed by face follows.

    A stale faceVarying `st` slides every UV onto the wrong corner and the
    texture crawls across the facade; a stale `GeomSubset` paints the windows
    with the roof material. Both are invisible until an asset with real
    shading meets the operator, which is exactly how this class of bug ships.
    """
    prim = shaded_box()
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    mesh = UsdGeom.Mesh(mesh_prim)
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    keep = np.ones(len(counts), dtype=bool)
    keep[::3] = False

    removed = M.delete_faces(mesh_prim, keep)
    assert removed == int((~keep).sum())

    api = UsdGeom.PrimvarsAPI(mesh_prim)
    new_counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    assert len(new_counts) == int(keep.sum())
    # The uniform tag says which original face each survivor was.
    assert list(api.GetPrimvar("origface").Get()) == list(np.nonzero(keep)[0])
    assert len(api.GetPrimvar("st").Get()) == int(new_counts.sum())
    sub = UsdGeom.Subset(mesh_prim.GetChild("half"))
    ind = np.asarray(sub.GetIndicesAttr().Get())
    assert len(ind) == 0 or ind.max() < len(new_counts)


def test_delete_faces_retires_a_prim_it_empties():
    prim = box(scale=4.0)
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    counts = UsdGeom.Mesh(mesh_prim).GetFaceVertexCountsAttr().Get()
    M.delete_faces(mesh_prim, np.zeros(len(counts), dtype=bool))
    assert not mesh_prim.IsActive()


def test_subdivide_enforces_an_edge_length():
    """The resolution floor: a cell cut from a 22 m triangle is a 22 m shard."""
    prim = box(scale=40.0, n=3)
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    before = _face_count(mesh_prim)
    assert M.subdivide(mesh_prim, 4.0) > 0
    assert _face_count(mesh_prim) > before

    counts, idx = M._face_arrays(mesh_prim)
    pts = M.get_points(mesh_prim)
    tri = idx[M._triangulate(counts)[0]]
    edges = np.linalg.norm(pts[tri[:, 0]] - pts[tri[:, 1]], axis=1)
    assert edges.max() < 8.0, "edges are still far above the target"


# ---------------------------------------------------------------------------
# the clip and the cap — closed fragments
# ---------------------------------------------------------------------------


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


def test_a_capped_cut_through_a_closed_solid_stays_closed():
    """The single most important property in the module.

    Uncapped, a fragment is a shell and looking into the fracture surface shows
    the hollow between the two sheets solidify just built. This is that
    property at its simplest: one plane, one closed box.
    """
    prims = M.mesh_prims(box(scale=10.0))
    soup, _ = M._mesh_soup(prims)
    assert open_edge_count(soup.verts, soup.faces) == 0, "the box is not closed"

    for origin in ([1.7, 0, 0], [0, -2.3, 0], [0.4, 0.6, 3.1]):
        cv, cf, _, _ = M._clip_by_plane(soup.verts, soup.faces, [1, 0.3, 0.2],
                                        origin, soup.uv, soup.fmat, cap=True)
        assert len(cf)
        assert open_edge_count(cv, cf) == 0, f"open at {origin}"


def test_capping_is_what_closes_a_fracture():
    """Pins the mechanism rather than the tuning: with the cap off, fragments
    are open shells; with it on, most of them are solids.

    The rate is a function of how big the cells are relative to the geometry —
    a bigger cell has a more complicated cross-section and more chances to hit
    a case `_cap_fan` declines, so this is measured at a stated cell count
    rather than at whatever the default happens to be. Measured on this box:
    60% at 30 cells, 71% at 69, 79% at 130. What is being pinned is that
    capping is the difference between "no fragment is closed" and "most are",
    not any particular percentage.

    NOT ASSERTED AT THE COARSE END, deliberately. Thirty cells on this box is
    a thirty-fragment sample and the rate swings 0.41 to 0.60 on reseeding or
    on `CORNER_AVOID`, which is sampling noise rather than a property. It is
    also not the regime the pipeline runs in any more: `quake` sizes rubble in
    absolute metres, so a real building is hundreds to thousands of cells.
    Both counts asserted here are in that regime, where the rate is stable to
    within a few points across seeds and corner weights.
    """
    prims = M.mesh_prims(box(scale=10.0))
    b = M.bounds_of(prims)
    M.solidify_prims(prims, 0.5, bounds=b)
    b = M.bounds_of(prims)
    fail = M.failure_field("earthquake", b, 0.9, 42)

    def closed(cap, max_cells):
        frags = M.fracture(prims, b, fail, seed=42, cap=cap,
                           max_cells=max_cells)
        assert len(frags) >= 8
        n_closed = sum(1 for f in frags
                       if open_edge_count(f.verts, f.faces) == 0)
        return n_closed / len(frags)

    assert closed(False, 140) < 0.05
    assert closed(True, 140) > 0.70
    assert closed(True, 300) > 0.70


# ---------------------------------------------------------------------------
# stage three — fracture
# ---------------------------------------------------------------------------


def test_fracture_divides_the_material_not_the_air():
    """Seeds come off the surface, so no one fragment holds the building.

    A building is a thin shell inside a mostly-empty bounding box. Seeding the
    box uniformly puts most seeds in interior air and leaves one enormous cell
    with the bulk of the geometry: measured on a real brownstone, 31k of 61k
    triangles in a single fragment, i.e. not shattered at all.
    """
    prims = M.mesh_prims(box(scale=10.0))
    b = M.bounds_of(prims)
    fail = M.failure_field("earthquake", b, 0.9, 3)
    frags = M.fracture(prims, b, fail, seed=3)
    assert len(frags) >= 8
    sizes = sorted(len(f) for f in frags)
    assert sizes[-1] < 0.35 * sum(sizes), "one fragment holds most of the mesh"


def test_seeds_go_where_the_damage_is():
    """Cell density follows the field: fine where demand is highest, coarse or
    absent away from it. That gradient is what makes a soft-storey collapse
    read as pulverised at the base against the upper floors merely cracking."""
    prims = M.mesh_prims(box(scale=10.0))
    b = M.bounds_of(prims)
    fail = M.failure_field("earthquake", b, 0.9, 3)
    soup, _ = M._mesh_soup(prims, lambda c: fail.damage(c) >= 0.12)
    seeds = M.fracture_seeds(soup, fail, 1.2, 3, 48)
    assert len(seeds) >= 8
    # Every seed sits where the field actually damaged something.
    assert (fail.damage(seeds) > 0.0).all()
    # …and their centre of mass is pulled DOWN the shear profile, toward the
    # base, rather than sitting at the centroid of the geometry.
    assert seeds[:, 2].mean() < b.center[2]


def test_crack_spacing_is_respected():
    """Two seeds a few centimetres apart on opposite faces of a wall put their
    bisector ALONG the wall, splitting the slab back into two sheets — the
    exact artifact stage one exists to prevent.

    The floor is the radius at damage 1, where the material is most finely
    comminuted: `fragment_m * (1 - 0.6) * spacing_frac`.
    """
    prims = M.mesh_prims(box(scale=10.0))
    b = M.bounds_of(prims)
    fail = M.failure_field("earthquake", b, 0.9, 3)
    soup, _ = M._mesh_soup(prims, lambda c: fail.damage(c) >= 0.12)
    for fragment_m in (1.5, 3.0):
        seeds = M.fracture_seeds(soup, fail, fragment_m, 3, 200)
        assert len(seeds) >= 4
        d = np.linalg.norm(seeds[:, None, :] - seeds[None, :, :], axis=-1)
        np.fill_diagonal(d, np.inf)
        assert d.min() >= fragment_m * 0.40 * 0.7 - 1e-9


def test_fragment_size_is_a_world_length_not_a_count():
    """Rubble is a constant size in the world, so a tower comes apart into more
    pieces than a shed and a finer `fragment_m` into more than a coarse one —
    without anything being tuned per asset."""
    def seeds(scale, fragment_m):
        # Densely tessellated on purpose: the candidate pool is face centroids,
        # so a coarse mesh would cap the seed count for reasons that have
        # nothing to do with the length being tested.
        prims = M.mesh_prims(box(scale=scale, n=16))
        b = M.bounds_of(prims)
        fail = M.failure_field("earthquake", b, 0.9, 3)
        soup, _ = M._mesh_soup(prims, lambda c: fail.damage(c) >= 0.12)
        return len(M.fracture_seeds(soup, fail, fragment_m, 3, 100_000))

    assert seeds(30.0, 4.0) > 4 * seeds(8.0, 4.0)
    assert seeds(20.0, 2.0) > 2 * seeds(20.0, 5.0)


def test_the_cell_budget_coarsens_rather_than_truncating():
    """A budget that just stops adding seeds leaves most of the building
    unseeded — an earthquake saturates at the base, so every seed landed in one
    corner of the ground floor and the four fifths above it fell into whichever
    peripheral cell was nearest, at 13,000-32,000 triangles each. The spacing
    is scaled up instead, so the seeds still cover the damaged region."""
    prims = M.mesh_prims(box(scale=20.0, n=10))
    b = M.bounds_of(prims)
    fail = M.failure_field("earthquake", b, 0.95, 3)
    soup, _ = M._mesh_soup(prims, lambda c: fail.damage(c) >= 0.12)

    plenty = M.fracture_seeds(soup, fail, 1.0, 3, 100_000)
    budget = M.fracture_seeds(soup, fail, 1.0, 3, 16)
    assert len(plenty) > 16 >= len(budget) > 4

    def span(s):
        return float(np.linalg.norm(s.max(axis=0) - s.min(axis=0)))
    # The budgeted seeding still reaches as far across the damaged region.
    assert span(budget) > 0.75 * span(plenty)


def test_fragments_land_where_they_were_cut_under_a_transform():
    """A fragment is a CHILD of the building, so it inherits its transform.

    Everything upstream works in world space, and authoring world points under
    a prim that already carries the placement's translate/rotate/scale applies
    all three twice. On the centimetre-authored AEC packs (`scale: 0.01`) that
    puts the debris a hundred times too small and a long way off.
    """
    prim = box(scale=1000.0, n=8, translate=(40.0, -12.0, 0.0), rot_z=31.0,
               xform_scale=0.01)
    stage = prim.GetStage()
    prims = M.mesh_prims(prim)
    b0 = M.bounds_of(prims)
    # A field with no ejecta, so nothing is expected to have moved.
    still = M.Failure("test", lambda p: np.ones(len(p)))
    cut = M.fracture_to_stage(stage, prim, b0, still, seed=3, max_cells=14)
    assert cut["paths"]
    v = np.concatenate([M.get_points(stage.GetPrimAtPath(p))
                        for p in cut["paths"]])
    assert v.min(axis=0) == pytest.approx(b0.lo, abs=0.05 * b0.height)
    assert v.max(axis=0) == pytest.approx(b0.hi, abs=0.05 * b0.height)


def test_fragments_are_authored_and_the_source_is_retired():
    """Fragments become real prims; the mesh they came from stops rendering.

    Deactivated rather than deleted — the source lives in a referenced layer
    where RemovePrim does not compose, the trap `prune_prims` documents.
    """
    prim = box(scale=10.0)
    stage = prim.GetStage()
    prims = M.mesh_prims(prim)
    b = M.bounds_of(prims)
    everything = M.Failure("test", lambda p: np.ones(len(p)))
    paths = M.fracture_to_stage(stage, prim, b, everything, seed=3)["paths"]
    assert len(paths) >= 6
    assert all(stage.GetPrimAtPath(p).IsValid() for p in paths)
    assert not any(p.IsActive() for p in prims), "source mesh still rendering"


def test_a_single_mesh_building_keeps_its_fragments():
    """Half the library is ONE mesh referenced straight at the placement path,
    and there the consumed prim IS the fragments' parent.

    Deactivating a prim takes its whole subtree with it, so retiring that one
    deleted the rubble it had just produced: a whole-building fracture made the
    building vanish entirely — no ruin, no debris, nothing but the ground. It
    is emptied instead.
    """
    prim = box(scale=10.0)
    stage = prim.GetStage()
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    b = M.bounds_of([mesh_prim])
    everything = M.Failure("test", lambda p: np.ones(len(p)))
    # Fractured against the MESH, so the fragments are authored beneath it.
    paths = M.fracture_to_stage(stage, mesh_prim, b, everything,
                                seed=3)["paths"]
    assert paths and all(p.startswith(str(mesh_prim.GetPath())) for p in paths)
    assert mesh_prim.IsActive(), "the fragments' own parent was retired"
    assert all(stage.GetPrimAtPath(p).IsValid() for p in paths)
    assert len(UsdGeom.Mesh(mesh_prim).GetFaceVertexCountsAttr().Get()) == 0


def test_a_partial_failure_leaves_the_rest_of_the_building_standing():
    """Only the damaged region is consumed, so a hurricane that fails the top
    of a tower pays for the top and leaves the rest with its own materials,
    UVs and subsets untouched."""
    prim = box(scale=10.0, n=10)
    stage = prim.GetStage()
    prims = M.mesh_prims(prim)
    b = M.bounds_of(prims)
    top = M.Failure("test",
                    lambda p: np.where(p[:, 2] > b.base_z + 0.75 * b.height,
                                       1.0, 0.0))
    before = _face_count(prims[0])
    cut = M.fracture_to_stage(stage, prim, b, top, seed=3)
    assert cut["paths"]
    after = _face_count(prims[0])
    assert 0 < after < before, "the whole building was consumed"
    cen = M.face_centroids(prims[0])
    assert cen[:, 2].max() < b.base_z + 0.8 * b.height


def test_nothing_is_authored_when_nothing_comes_free():
    """An anchored fragment is geometrically identical to the source it would
    replace, so a building that cracks without losing anything is left alone
    rather than re-authored into pieces that look exactly like it did."""
    prim = box(scale=10.0)
    stage = prim.GetStage()
    prims = M.mesh_prims(prim)
    b = M.bounds_of(prims)
    # Damaged everywhere, but never past the release threshold.
    cracked = M.Failure("test", lambda p: np.full(len(p), 0.3))
    cut = M.fracture_to_stage(stage, prim, b, cracked, seed=3)
    assert cut["paths"] == [] and cut["loose"] == []
    assert _face_count(prims[0]) > 0, "the source was consumed anyway"


def test_only_released_fragments_are_loose():
    """Hand PhysX every fragment and gravity levels the building whatever the
    severity was — a 0.3 earthquake and a 0.9 one end as the same flat pile,
    and the severity sweep stops comparing anything."""
    prim = box(scale=10.0, n=10)
    stage = prim.GetStage()
    prims = M.mesh_prims(prim)
    b = M.bounds_of(prims)
    # A graded field, because a binary one makes the cut region and the
    # released region the same set by construction. This one cracks from
    # halfway up (past `support`) and only frees the top quarter (past
    # `release`), which is the case the two thresholds exist for.
    ramp = M.Failure("test", lambda p: np.clip(
        (p[:, 2] - b.base_z) / b.height * 2.0 - 1.0, 0.0, 1.0))
    cut = M.fracture_to_stage(stage, prim, b, ramp, seed=3)
    assert cut["loose"]
    assert len(cut["loose"]) < len(cut["paths"]), "everything came free"
    freed = set(cut["loose"])
    z_of = {p: float(M.get_points(stage.GetPrimAtPath(p))[:, 2].mean())
            for p in cut["paths"]}
    up = [z for p, z in z_of.items() if p in freed]
    held = [z for p, z in z_of.items() if p not in freed]
    assert held, "nothing stayed attached"
    # The boundary is fuzzy — a fragment is judged on its MEAN damage, so one
    # straddling the release height goes whichever way its bulk sits — but the
    # two populations must separate, with everything freed above the crack and
    # everything held below it.
    assert min(up) > b.base_z + 0.55 * b.height
    assert max(held) < b.base_z + 0.85 * b.height
    assert np.mean(up) > np.mean(held)


def test_fragments_carry_uvs_and_a_material():
    """Untextured fragments read as confetti, not as a building coming apart."""
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    prim = box(st, "/B", scale=10.0)
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    counts = UsdGeom.Mesh(mesh_prim).GetFaceVertexCountsAttr().Get()
    pv = UsdGeom.PrimvarsAPI(mesh_prim).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set([Gf.Vec2f(i * 0.01, 0.5) for i in range(sum(counts))])
    mat = UsdShade.Material.Define(st, "/Looks/Brick")
    UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(mat)

    b = M.bounds_of([mesh_prim])
    cut = M.fracture_to_stage(st, prim, b,
                              M.Failure("t", lambda p: np.ones(len(p))), seed=3)
    assert cut["paths"]
    for path in cut["paths"]:
        frag = st.GetPrimAtPath(path)
        got = UsdGeom.PrimvarsAPI(frag).GetPrimvar("st")
        assert got and len(got.Get()) == len(M.get_points(frag))
        bound = UsdShade.MaterialBindingAPI(frag).ComputeBoundMaterial()[0]
        assert bound and bound.GetPrim().GetPath() == mat.GetPrim().GetPath()


# ---------------------------------------------------------------------------
# the driver, on real compiled config
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("dtype", DISASTERS)
def test_apply_to_stage_runs_the_compiled_config(dtype):
    """The seam between the compiler and the pipeline, on real config.

    Every previous bug in this path was a name mismatch rather than bad maths —
    a knob the compiler stopped emitting, a plan key spelled differently — and
    none of it shows up in a unit test that hand-builds its own config dict.
    """
    st, placements = stage_with_buildings(2)
    for p in placements:
        p["_mesh_damage"] = 0.9
    out = M.apply_to_stage(st, compiled_disaster_config(dtype), placements)

    assert out["tally"].get(dtype) == 2, f"{dtype} did not run"
    assert all(st.GetPrimAtPath(f).IsValid() for f in out["fragments"])
    assert set(out["loose"]) <= set(out["fragments"])
    if dtype == "flood":
        # Water undermines a building; it does not shatter masonry. The field
        # never reaches the release threshold, so the building is dismissed
        # before it is thickened — see `field_flood`.
        assert out["fragments"] == []
        assert out["tally"].get("unbroken") == 2
    else:
        assert out["fragments"], f"{dtype} broke nothing"
        assert out["tally"].get("thickened") == 2


@pytest.mark.parametrize("dtype", ["earthquake", "tornado",
                                   "hurricane", "fire"])
def test_severity_frees_more_material(dtype):
    """The invariant the whole generator rests on, end to end."""
    def loose(sev):
        st, placements = stage_with_buildings(1, scale=20.0, faces=8)
        placements[0]["_mesh_damage"] = sev
        out = M.apply_to_stage(
            st, compiled_disaster_config(dtype, severity=sev), placements)
        return len(out["loose"])

    low, high = loose(0.3), loose(0.95)
    assert high > low, f"{dtype}: {low} loose at 0.3, {high} at 0.95"


def test_zero_severity_touches_nothing():
    st, placements = stage_with_buildings(2)
    before = [_face_count(M.mesh_prims(st.GetPrimAtPath(p["prim_path"]))[0])
              for p in placements]
    # No `_mesh_damage` marker at all: `disaster_stage` did not select these.
    out = M.apply_to_stage(st, compiled_disaster_config("earthquake"),
                           placements)
    assert out["tally"] == {} and out["fragments"] == []
    after = [_face_count(M.mesh_prims(st.GetPrimAtPath(p["prim_path"]))[0])
             for p in placements]
    assert before == after


def test_the_storm_bearing_is_read_off_the_compiled_config():
    """Every tornado building must throw its debris the SAME way.

    `heading_deg` is authored inside the `field` block and never at the top
    level, so reading only the top level returned None for every scene ever
    generated and each building picked a random bearing — turning the track's
    signature into a bomb site.
    """
    cfg = compiled_disaster_config("tornado")
    assert M._heading_of(cfg["disaster"]) == 35.0

    def blown(heading):
        """Where the loose debris ends up, relative to the building centre.

        Measured against the building rather than against another run: the
        bearing enters the FIELD as well as the throw — it is what makes the
        windward face fail — so two headings do not fracture the same building
        the same way and their fragments are not comparable one to one.
        """
        st, placements = stage_with_buildings(1, scale=20.0, faces=8)
        placements[0]["_mesh_damage"] = 0.95
        c = compiled_disaster_config("tornado", severity=0.95)
        c["disaster"]["field"]["heading_deg"] = heading
        # THE CENTRE IS READ BEFORE THE DAMAGE. Fragments are authored as
        # children of the building prim, so its bounds afterwards are the
        # bounds of the debris — and on the rung where the whole structure is
        # freed and thrown together, that reference travels with the thing it
        # is supposed to be a reference for and every bearing reads as zero.
        centre = M.bounds_of(M.mesh_prims(
            st.GetPrimAtPath(placements[0]["prim_path"]))).center
        out = M.apply_to_stage(st, c, placements)
        # NOT THE SLABS. `loose` also carries whole uncut pieces that lost
        # their load path (`fracture_to_stage`'s SUPPORT note) — the wind did
        # not throw those, gravity dropped them where they stood, and they are
        # the largest bodies in the pile. Averaging them in is averaging the
        # building back into the measurement.
        thrown = [p for p in out["loose"] if p not in set(out["slabs"])]
        assert thrown, "nothing was thrown"
        debris = np.array([M.get_points(st.GetPrimAtPath(p)).mean(axis=0)
                           for p in thrown]).mean(axis=0)
        return debris - centre

    # Opposed bearings, so the two are as far apart as the knob can put them.
    # The claim here is only that the configured bearing REACHES the pipeline —
    # that it points the debris the right way is pinned on the field itself, in
    # `test_wind_throws_everything_the_same_way`, where the fracture is not
    # also moving underneath the measurement.
    east, west = blown(0.0), blown(180.0)
    assert east[0] - west[0] > 0.5, f"bearing ignored: {east} vs {west}"


def test_the_budget_goes_to_the_worst_hit_buildings():
    """Not to whichever buildings the placement list happened to reach first.

    Placement order is essentially packing order, so a first-come budget hands
    the damage to one corner of the map and leaves the epicentre — the one
    place a viewer looks — untouched.
    """
    st, placements = stage_with_buildings(4)
    for p, k in zip(placements, (0.15, 0.2, 0.95, 0.3)):
        p["_mesh_damage"] = k
    cfg = compiled_disaster_config("earthquake")
    cfg["disaster"]["mesh_damage"]["fracture"]["max_buildings"] = 1

    out = M.apply_to_stage(st, cfg, placements)
    assert out["fragments"]
    assert all(f.startswith(placements[2]["prim_path"])
               for f in out["fragments"])


def test_the_environment_can_cap_the_budget(monkeypatch):
    """`SCENE_DAMAGE_BUDGET` bounds live damage without editing a preset.

    A preset's `max_buildings` is written for the path it expects: the ones
    tuned around a baked archetype library carry a budget of 60 because an
    archetype costs nothing to place. Turning the library off
    (`SCENE_ARCHETYPES=0`) hands every one of those buildings to live mesh
    damage, which is hours rather than minutes — so the cap has to be
    reachable from outside the config.
    """
    st, placements = stage_with_buildings(4)
    for p, k in zip(placements, (0.15, 0.2, 0.95, 0.3)):
        p["_mesh_damage"] = k
    cfg = compiled_disaster_config("earthquake")
    cfg["disaster"]["mesh_damage"]["fracture"]["max_buildings"] = 4

    monkeypatch.setenv("SCENE_DAMAGE_BUDGET", "1")
    out = M.apply_to_stage(st, cfg, placements)

    # One building damaged, and it is the WORST-HIT one, not the first placed.
    assert out["fragments"]
    assert all(f.startswith(placements[2]["prim_path"])
               for f in out["fragments"])


def test_a_junk_budget_in_the_environment_is_ignored(monkeypatch):
    """A typo must not silently disable the damage stage."""
    st, placements = stage_with_buildings(2)
    for p in placements:
        p["_mesh_damage"] = 0.9
    cfg = compiled_disaster_config("earthquake")

    monkeypatch.setenv("SCENE_DAMAGE_BUDGET", "lots")
    assert M.apply_to_stage(st, cfg, placements)["fragments"]


def test_the_config_still_overrides_the_defaults():
    """A preset can override any fracture knob; it just no longer has to
    supply any of them."""
    def cells(fragment_m):
        st, placements = stage_with_buildings(1, scale=20.0, faces=8)
        placements[0]["_mesh_damage"] = 0.9
        cfg = compiled_disaster_config("earthquake")
        cfg["disaster"]["mesh_damage"]["fracture"].update(
            {"fragment_m": fragment_m, "max_cells": 400})
        return len(M.apply_to_stage(st, cfg, placements)["fragments"])

    assert cells(1.5) > cells(6.0)


def test_disabling_the_pipeline_is_honoured():
    st, placements = stage_with_buildings(2)
    for p in placements:
        p["_mesh_damage"] = 0.9
    cfg = compiled_disaster_config("earthquake")
    cfg["disaster"]["mesh_damage"]["fracture"]["enabled"] = False
    out = M.apply_to_stage(st, cfg, placements)
    assert out["tally"] == {} and out["fragments"] == []


def test_a_solid_asset_is_not_thickened():
    """`solid: true` in the asset pack says the art already has wall volume;
    thickening it again costs points and gains nothing."""
    st, placements = stage_with_buildings(1)
    placements[0]["_mesh_damage"] = 0.9
    placements[0]["usd"] = "omniverse://x/SolidTower.usd"
    cfg = compiled_disaster_config("earthquake")
    cfg["asset_root"] = ""
    cfg["usds"] = {"buildings": {"damaged": [
        {"usd": "omniverse://x/SolidTower.usd", "solid": True}]}}

    out = M.apply_to_stage(st, cfg, placements)
    assert out["tally"].get("thickened") is None
    assert out["tally"].get("already_solid") == 1


def test_apply_to_stage_is_deterministic():
    def run():
        st, placements = stage_with_buildings(2)
        for p in placements:
            p["_mesh_damage"] = 0.8
        out = M.apply_to_stage(st, compiled_disaster_config("earthquake"),
                               placements)
        pts = [float(M.get_points(st.GetPrimAtPath(f)).sum())
               for f in out["fragments"]]
        return out["fragments"], out["loose"], pts

    a, b = run(), run()
    assert a[0] == b[0] and a[1] == b[1]
    assert np.allclose(a[2], b[2])


def test_damage_is_confined_to_the_building_it_hit():
    """Damage authors overrides on placed geometry; it must never reach into
    the neighbour, which shares the stage and often the referenced layer."""
    st, placements = stage_with_buildings(2)
    placements[0]["_mesh_damage"] = 0.95           # only the first
    victim = M.mesh_prims(st.GetPrimAtPath("/World/b0"))[0]
    bystander = M.mesh_prims(st.GetPrimAtPath("/World/b1"))[0]
    before = M.get_points(bystander).copy()
    n_before = _face_count(bystander)

    M.apply_to_stage(st, compiled_disaster_config("earthquake"), placements)
    assert not st.GetPrimAtPath("/World/b1/fragments").IsValid()
    assert _face_count(bystander) == n_before
    assert np.array_equal(M.get_points(bystander), before)
    assert st.GetPrimAtPath("/World/b0/fragments").IsValid()
    assert victim  # kept alive


# ---------------------------------------------------------------------------
# soot
# ---------------------------------------------------------------------------


def _textured_material(stage, path="/Looks/Brick"):
    mat = UsdShade.Material.Define(stage, path)
    surf = UsdShade.Shader.Define(stage, path + "/Surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    tex = UsdShade.Shader.Define(stage, path + "/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f) \
        .ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    return mat, tex


def test_scorch_darkens_a_texture_driven_material():
    """Assets drive albedo from an image, so a connection beats a value and
    setting `diffuseColor` does nothing at all. Soot goes on the texture's
    `inputs:scale`, which multiplies whatever it sampled."""
    prim = box(scale=10.0)
    st = prim.GetStage()
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    mat, tex = _textured_material(st)
    UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(mat)

    b = M.bounds_of([mesh_prim])
    assert M.scorch([mesh_prim], M.failure_field("fire", b, 0.9, 3)) == 1
    scale = tex.GetInput("scale").Get()
    assert scale is not None
    assert scale[0] < 0.6 and scale[1] < 0.6 and scale[2] < 0.6
    # Alpha is untouched: a texture whose `.a` drives opacity would otherwise
    # fade the building out as it charred.
    assert scale[3] == pytest.approx(1.0)


def test_scorch_is_proportional_to_the_field():
    """A blast blackens the wall it went off against, not the far side."""
    def tint(field, mesh_prim, tex):
        M.scorch([mesh_prim], field)
        return float(tex.GetInput("scale").Get()[0])

    prim = box(scale=10.0)
    st = prim.GetStage()
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    mat, tex = _textured_material(st)
    UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(mat)
    b = M.bounds_of([mesh_prim])

    dark = tint(M.failure_field("fire", b, 0.9, 3), mesh_prim, tex)

    prim2 = box(scale=10.0)
    st2 = prim2.GetStage()
    mp2 = M.mesh_prims(prim2)[0].GetPrim()
    mat2, tex2 = _textured_material(st2)
    UsdShade.MaterialBindingAPI.Apply(mp2).Bind(mat2)
    light = tint(M.failure_field("flood", b, 0.3, 3), mp2, tex2)
    assert dark < light


def test_scorch_does_not_compound_on_a_shared_material():
    """The material lives in the referenced layer and is shared between prims,
    so a second visit would double the tint."""
    prim = box(scale=10.0)
    st = prim.GetStage()
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    second = UsdGeom.Mesh.Define(st, "/B/Mesh2")
    src = UsdGeom.Mesh(mesh_prim)
    for attr in ("GetPointsAttr", "GetFaceVertexCountsAttr",
                 "GetFaceVertexIndicesAttr"):
        getattr(second, attr)().Set(getattr(src, attr)().Get())
    mat, tex = _textured_material(st)
    for p in (mesh_prim, second.GetPrim()):
        UsdShade.MaterialBindingAPI.Apply(p).Bind(mat)

    prims = M.mesh_prims(prim)
    b = M.bounds_of(prims)
    assert M.scorch(prims, M.failure_field("fire", b, 0.9, 3)) == 1
    once = float(tex.GetInput("scale").Get()[0])
    assert once > 0.0


# ---------------------------------------------------------------------------
# where the cracks land, and what is left holding the building up
# ---------------------------------------------------------------------------


def test_the_cracks_run_along_the_edges_not_around_them():
    """Corners and seams are WEAK POINTS, so the cut has to land on them.

    This had the sign the wrong way round and the property is what says so.
    A Voronoi seed is the CENTRE of a fragment and the cracks are the
    bisectors halfway between seeds, so biasing the seeds toward high
    `cornerness` — which is what `CORNER_GAIN` used to do — buries every corner
    in the middle of an intact cell and guarantees the one place the building
    should split is the one place it cannot.

    Measured as distance to the nearest crack: for a face, ``d2 - d1`` over its
    two nearest seeds is zero exactly on a bisector and grows into the middle
    of a cell. A corner should be closer to a crack than flat wall is.
    """
    prims = M.mesh_prims(box(scale=12.0, n=8, height=1.4))
    for mp in prims:
        M.subdivide(mp, 2.0)
    b = M.bounds_of(prims)
    M.solidify_prims(prims, 0.4, bounds=b)
    b = M.bounds_of(prims)
    fail = M.failure_field("earthquake", b, 0.95, 3)

    soup, _ = _mesh_soup_all(prims)
    frag_m = 1.5
    seeds = M.fracture_seeds(soup, fail, frag_m, 3, 400)
    assert len(seeds) > 40

    cen = soup.centroids()
    corner = M.cornerness(soup, frag_m)
    d = np.linalg.norm(cen[:, None, :] - seeds[None, :, :], axis=2)
    near = np.sort(d, axis=1)[:, :2]
    to_crack = (near[:, 1] - near[:, 0]) * 0.5

    hot = corner >= np.percentile(corner, 90)
    flat = corner <= np.percentile(corner, 40)
    assert hot.sum() > 20 and flat.sum() > 20
    assert to_crack[hot].mean() < to_crack[flat].mean(), (
        "corners are no nearer a crack than flat wall is — the seeds are not "
        "being pushed off the seams")


def _mesh_soup_all(prims):
    """The whole of *prims* as one soup, damage ignored."""
    return M._mesh_soup(prims)


def _band(bounds, lo, hi):
    """A field that fails a horizontal slice of the building and nothing else."""
    def damage(p):
        t = (p[:, 2] - bounds.base_z) / max(bounds.height, 1e-9)
        return np.where((t >= lo) & (t <= hi), 1.0, 0.0)
    return M.Failure("earthquake", damage)


def test_the_block_above_a_failed_storey_is_released_whole():
    """A soft storey means the mass above it COMES DOWN, not that it hovers.

    The bug this pins: `support` leaves undamaged material on the source prims
    uncut, and uncut used to mean unconsidered — the retained remainder was fed
    to the support graph as an unconditional root, i.e. as ground. So a
    building whose ground floor was pulverised kept every storey above it
    exactly where it was, and the ladder's heavy rungs rendered as a pile of
    rubble with a roof plate floating over it.

    Here the field fails a band at the base and nothing else, so everything
    above is retained. It has to come back as a `slab` — one released body, not
    a thousand fragments — and it has to be gone from the source prim.
    """
    prim = box(scale=14.0, n=8, height=3.0)
    prims = M.mesh_prims(prim)
    for mp in prims:
        M.subdivide(mp, 3.0)
    b = M.bounds_of(prims)
    M.solidify_prims(prims, 0.4, bounds=b)
    b = M.bounds_of(prims)

    before = sum(len(M.face_centroids(p)) for p in prims)
    rep = M.fracture_to_stage(prim.GetStage(), prim, b, _band(b, 0.0, 0.18),
                              seed=1, fragment_m=1.5, max_cells=300,
                              support=0.5, release=0.5, collapse=0.6)
    assert rep["cells"] > 20
    assert rep["slabs"], "the mass above the failed storey was left in the air"
    assert set(rep["slabs"]) <= set(rep["loose"])

    # `prims`, captured before the cut: `mesh_prims` walks the subtree and
    # would now count the fragments authored under it as well.
    after = sum(len(M.face_centroids(p) or ()) for p in prims
                if p.GetPrim().IsActive())
    assert after < before * 0.25, (
        "the released block is still on the source prim as well as authored "
        "as a body, so the building is there twice")


def test_only_what_is_above_a_failed_band_comes_free():
    """The other half of the same property: `support` is not a licence to drop
    everything. A band failed in the MIDDLE leaves the material below it on the
    source prim, standing on the ground, and takes only the block above.
    """
    prim = box(scale=14.0, n=8, height=3.0)
    prims = M.mesh_prims(prim)
    for mp in prims:
        M.subdivide(mp, 3.0)
    b = M.bounds_of(prims)
    M.solidify_prims(prims, 0.4, bounds=b)
    b = M.bounds_of(prims)
    cut = b.base_z + 0.62 * b.height

    rep = M.fracture_to_stage(prim.GetStage(), prim, b, _band(b, 0.55, 0.70),
                              seed=1, fragment_m=1.5, max_cells=300,
                              support=0.5, release=0.5, collapse=0.6)
    assert rep["cells"] > 20
    assert len(rep["slabs"]) == 1, "the top should come off as ONE block"

    # `prims`, captured before the cut: `mesh_prims` walks the subtree and
    # would now return the fragments and the slab as well.
    left = np.vstack([M.face_centroids(p) for p in prims
                      if p.GetPrim().IsActive()
                      and M.face_centroids(p) is not None])
    assert (left[:, 2] < cut).mean() > 0.9, (
        "material below the failed band was released; only the block above it "
        "had lost its load path")




def test_seeds_are_the_same_in_every_process():
    """`hash()` is salted per process, so anything seeded off it is not seeded.

    Pinned as literal values rather than as `f(x) == f(x)`, which is true of
    `hash()` too and catches nothing. These numbers are what a second process
    has to agree with, and the whole determinism claim — config plus seed
    reproduces the scene — rests on that being true across a re-bake.
    """
    assert M.stable_seed("bg_c", "crack") == 8785
    assert M.stable_seed("bg_f", "pancake") == 5210
    assert M.stable_seed("soft_storey", mod=100_000) == 87144
    assert 0 <= M.stable_seed(("a", 1), 2.5) < 9973


# ---------------------------------------------------------------------------
# the cut faces show the material's core, from ONE shared material
# ---------------------------------------------------------------------------

def test_cut_faces_get_one_shared_core_material():
    """A broken wall shows brick at the break, not a second copy of its siding
    — and every break in the scene binds the SAME core prim, because the
    renderer compiles each distinct MDL material at load."""
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    UsdGeom.Xform.Define(st, "/World")
    prim = box(st, "/World/B", scale=10.0)
    mesh_prim = M.mesh_prims(prim)[0].GetPrim()
    siding = UsdShade.Material.Define(st, "/World/Looks/Siding")
    UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(siding)

    core = M.core_material(st, "masonry")
    assert core == "/World/Looks/FractureCore_masonry"
    assert M.core_material(st, "masonry") == core        # defined once
    b = M.bounds_of([mesh_prim])
    cut = M.fracture_to_stage(st, prim, b,
                              M.Failure("t", lambda p: np.ones(len(p))),
                              seed=3, core=core)
    assert cut["paths"]

    with_core = 0
    for path in cut["paths"]:
        frag = st.GetPrimAtPath(path)
        bound = {str(UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
                     .GetPrim().GetPath())
                 for p in [frag] + [s.GetPrim() for s in
                                    UsdGeom.Subset.GetAllGeomSubsets(
                                        UsdGeom.Imageable(frag))]}
        # The exterior is still the exterior ...
        assert "/World/Looks/Siding" in bound
        with_core += core in bound
    # ... and the cut faces are the core, on a real share of the pieces.
    assert with_core >= len(cut["paths"]) // 2
    cores = [p for p in st.Traverse() if "FractureCore" in p.GetName()]
    assert len(cores) == 1


def test_a_scene_binds_one_core_per_material_kind():
    """`apply_to_stage` on two buildings makes ONE core material, not two."""
    st, placements = stage_with_buildings(2)
    for p in placements:
        p["_mesh_damage"] = 0.9
    out = M.apply_to_stage(st, compiled_disaster_config("earthquake"),
                           placements)
    assert out["fragments"]
    cores = [str(p.GetPath()) for p in st.Traverse()
             if p.GetName().startswith("FractureCore_")]
    assert len(cores) == 1, cores


def test_the_subdivide_override_reaches_stage_a_too():
    """`SCENE_SUBDIVIDE_M` is the documented lever for archetype file size, and
    for a long time it reached only the live cut — a bake with it set produced
    a byte-identical library. Both paths resolve it through one function now."""
    import os

    old = os.environ.get("SCENE_SUBDIVIDE_M")
    try:
        os.environ.pop("SCENE_SUBDIVIDE_M", None)
        assert M.subdivide_edge_m(4.0) == 4.0
        os.environ["SCENE_SUBDIVIDE_M"] = "8"
        assert M.subdivide_edge_m(4.0) == 8.0
        # COARSER ONLY: it is a floor, so it can never sharpen the cut below
        # what the config asked for.
        assert M.subdivide_edge_m(12.0) == 12.0
        # Subdivision switched off stays off.
        assert M.subdivide_edge_m(0.0) == 0.0
        os.environ["SCENE_SUBDIVIDE_M"] = "nonsense"
        assert M.subdivide_edge_m(4.0) == 4.0
    finally:
        os.environ.pop("SCENE_SUBDIVIDE_M", None)
        if old is not None:
            os.environ["SCENE_SUBDIVIDE_M"] = old


def test_a_constant_bool_primvar_does_not_kill_subdivision():
    """A scalar primvar must be skipped, not `len()`-ed.

    `primvars:doNotCastShadows` is a CONSTANT `bool` the Omniverse AEC packs
    author on every mesh. `len()` on a bool raises rather than returning 0, so
    the primvar walk in `subdivide` died on it — and because Stage A catches a
    bad cell and moves on, the whole thing surfaced only as
    `SKIP Reference_Brownstone02_cracked: TypeError: object of type 'bool' has
    no len()`, once per rung, for all eight brownstone types: 32 archetypes.
    """
    prim = box(scale=8.0, n=2, height=1.0)
    mesh = UsdGeom.Mesh(prim.GetChild("Mesh"))
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "doNotCastShadows", Sdf.ValueTypeNames.Bool,
        UsdGeom.Tokens.constant)
    pv.Set(False)

    added = M.subdivide(mesh.GetPrim(), 1.0)

    assert added > 0, "subdivision must still run with a scalar primvar present"
    # And the primvar is left exactly as authored — a constant says the same
    # thing about the mesh however finely it is cut.
    assert pv.Get() is False
    assert pv.GetInterpolation() == UsdGeom.Tokens.constant


def test_sized_accepts_arrays_and_rejects_scalars():
    """The guard itself: arrays through, scalars and empties out."""
    assert M._sized(None) is None
    assert M._sized(False) is None
    assert M._sized(3.5) is None
    assert M._sized("st") is None
    assert M._sized([]) is None
    assert M._sized([1, 2]) == [1, 2]


def _soup_and_seeds(n=10, cells=12):
    """A box's triangle soup plus a handful of Voronoi seeds inside it."""
    prim = box(scale=8.0, n=n, height=1.0)
    soup, _taken = M._mesh_soup(M.mesh_prims(prim))
    cen = soup.centroids()
    step = max(1, len(cen) // cells)
    return soup, np.ascontiguousarray(cen[::step][:cells])


def test_threaded_clip_loop_matches_the_serial_one_exactly():
    """Threading `_fracture_soup` is a scheduling change, not a maths one.

    The loop reads a shared immutable `Soup`, calls a pure clip, and touches no
    USD — so the fragments must come back byte-identical AND in the same order,
    since Stage A names prims from their index.
    """
    soup, seeds = _soup_and_seeds()
    old_threads, old_min = M.CLIP_THREADS, M.CLIP_THREAD_MIN_FACES
    try:
        M.CLIP_THREADS, M.CLIP_THREAD_MIN_FACES = 1, 10 ** 9
        serial = M._fracture_soup(soup, seeds)
        # Force the threaded path on a soup far below the production gate.
        M.CLIP_THREADS, M.CLIP_THREAD_MIN_FACES = 4, 0
        threaded = M._fracture_soup(soup, seeds)
    finally:
        M.CLIP_THREADS, M.CLIP_THREAD_MIN_FACES = old_threads, old_min

    assert len(serial) == len(threaded) > 1
    for a, b in zip(serial, threaded):
        assert np.array_equal(a.verts, b.verts)
        assert np.array_equal(a.faces, b.faces)
        assert np.array_equal(a.uv, b.uv)
        assert np.array_equal(a.fmat, b.fmat)


def test_small_soups_take_the_serial_path():
    """The gate is on the SOUP, not the seed count.

    Threading a tiny soup costs more than it saves — measured 0.27 s serial
    against 0.45 s on four threads for a `selected_citydemo` tower — so the
    production constant has to sit above anything that small.
    """
    assert M.CLIP_THREAD_MIN_FACES >= 1000
    soup, _seeds = _soup_and_seeds(n=2, cells=4)
    assert len(soup) < M.CLIP_THREAD_MIN_FACES


def test_fragments_keep_the_source_uv_set_name():
    """A fragment's UVs are useless under a name the material is not reading.

    Materials sample through a `UsdPrimvarReader_float2` that asks for ONE
    primvar by name. `_author_soup` used to write `st` unconditionally, so a
    fragment cut from an asset whose UV set is `uv0` carried its texture
    coordinates under a name nothing looked up: the reader returned nothing,
    the texture sampled nothing, and every piece rendered as the shader's flat
    fallback colour. Measured on `midrise_14_0204a` (a `uv0` asset), the whole
    wreck came out solid pink and off-white beside a fully textured pristine
    copy — while `old_brick_shop` and `house_04` looked right, which is what
    made it read as "some assets but not all".
    """
    prim = box(scale=6.0, n=3)
    mesh = UsdGeom.Mesh(prim.GetChild("Mesh"))
    n_pts = len(mesh.GetPointsAttr().Get())
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "uv0", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set([(0.25, 0.75)] * n_pts)

    soup, _taken = M._mesh_soup(M.mesh_prims(prim))
    assert "uv0" in soup.uv_names, "the soup must record what it read"

    out = M._author_soup(prim.GetStage(), Sdf.Path("/Frag"), soup,
                         soup.verts, np.eye(4))
    got = {p.GetBaseName()
           for p in UsdGeom.PrimvarsAPI(
               prim.GetStage().GetPrimAtPath(out)).GetPrimvars()}
    # BOTH: `st` for anything that assumes the convention, and the source's own
    # name for the reader that actually exists on this asset's material.
    assert "uv0" in got and "st" in got, got


def test_uv_names_survive_the_fracture():
    """The name has to reach the FRAGMENTS, not just the parent soup."""
    prim = box(scale=6.0, n=3)
    mesh = UsdGeom.Mesh(prim.GetChild("Mesh"))
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "uv0", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set([(0.5, 0.5)] * len(mesh.GetPointsAttr().Get()))

    soup, _t = M._mesh_soup(M.mesh_prims(prim))
    cen = soup.centroids()
    seeds = np.ascontiguousarray(cen[:: max(1, len(cen) // 6)][:6])
    frags = M._fracture_soup(soup, seeds)
    assert frags, "expected the box to break into something"
    assert all("uv0" in f.uv_names for f in frags)
