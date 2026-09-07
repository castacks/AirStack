"""Stage B, step 5: reference the pre-baked archetype instead of damaging live.

The library is gitignored and takes an Isaac Sim session to bake, so these
tests plant a SYNTHETIC manifest and check the swap logic against it. That is
the part worth regressing on anyway — whether a scene picks the right archetype
and stops asking for live damage — rather than whether PhysX settled correctly.
"""

import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import compile_disaster as cd                                    # noqa: E402
import generate_scene                                            # noqa: E402
import scene_generator as sg                                     # noqa: E402
from archetypes import library as lib                            # noqa: E402
from archetypes import plan as P                                 # noqa: E402
from disaster import disaster_stage as ds                        # noqa: E402
from disaster import levels as L                                 # noqa: E402

PRESET, SEED, SEVERITY = "urban_small", 42, 0.8
DTYPE = "earthquake"


def _config():
    cfg = cd.load_scene_config(PRESET)
    spec = {"locale": "urban", "disaster-type": DTYPE, "severity": SEVERITY,
            "asset-pack": cfg.get("asset_pack"), "seed": SEED,
            "region_m": list(cfg["layout"]["region_m"])}
    import yaml
    base = yaml.safe_load(open(os.path.join(
        _SCENE_GEN, "config", "low_level", "default.yaml")))
    out = cd.compile_spec(spec, base)
    return sg.resolve_asset_pack(out)


@pytest.fixture(scope="module")
def config():
    return _config()


@pytest.fixture
def full_library(config, monkeypatch):
    """A manifest covering every planned (type, level) for this config."""
    recs = []
    for it in P.build_plan(config, DTYPE):
        for _t, lv in it.combos:
            recs.append({"type": it.type, "level": lv, "kind": it.kind,
                         "source": str(it.source),
                         "usd": lib.archetype_name(it.type, lv) + ".usd"})
    library = lib.Library(
        path=os.path.join(_SCENE_GEN, "assets", "archetypes", DTYPE,
                          lib.MANIFEST_NAME),
        doc={"disaster": DTYPE, "archetypes": recs})
    monkeypatch.setattr(ds, "_ARCH_CACHE", {ds._arch_key(DTYPE): library})
    return library


@pytest.fixture
def no_library(monkeypatch):
    monkeypatch.setattr(ds, "_ARCH_CACHE", {ds._arch_key(DTYPE): None})


def _build(config):
    # COLD. `_make_resolver` defaults to the on-disk measurement cache, so a
    # test that takes the default asserts against whatever a container run
    # last measured — the layout changes under it and the failure looks like a
    # code regression. `snapshot.py` already builds cold for this reason.
    resolver = sg._make_resolver(config, cache=False)
    placements, layout, _ = generate_scene.build_scene(config, resolver)
    return placements


def _damaged(placements):
    return [p for p in placements
            if p.get("category") == "house" and p.get("_damage_level")
            and p["_damage_level"] != "pristine"]


# --------------------------------------------------------------------------

def test_without_a_library_nothing_breaks(config, no_library):
    """A fresh checkout has no library — it is gitignored and needs an Isaac
    session. The scene must still build, exactly as before Stage A existed."""
    placements = _build(config)
    assert placements
    hit = [p for p in placements if p.get("_archetype")]
    assert not hit, "referenced an archetype with no library loaded"


def test_with_a_library_buildings_reference_archetypes(config, full_library):
    placements = _build(config)
    hit = [p for p in placements if p.get("_archetype")]
    assert hit, "a full library was loaded and nothing referenced it"
    for p in hit:
        assert "assets/archetypes/" in p["usd"]
        assert p["usd"].startswith("airstack://scene_gen/")


def test_archetypes_replace_live_mesh_damage(config, full_library):
    """The payoff. A building with an archetype must NOT also be queued for
    live deformation — that is the per-scene physics Stage A abolishes."""
    placements = _build(config)
    for p in placements:
        if p.get("_archetype"):
            assert "_mesh_damage" not in p, (
                "building has both an archetype and a live-damage marker; "
                "it would be fractured at scene time for nothing")


def test_a_full_library_beats_the_max_buildings_budget(config, full_library):
    """Stage A's whole point: damage stops being rationed.

    Live mesh damage carries `fracture.max_buildings` (50-80) because each
    building costs 35-50 s. Referencing costs nothing, so every damaged
    building can have real geometry rather than the tilt-and-sink stand-in.
    """
    budget = ((config["disaster"].get("mesh_damage") or {})
              .get("fracture") or {}).get("max_buildings")
    assert budget, "this preset has no budget to beat"

    with_lib = [p for p in _build(config) if p.get("_archetype")]
    assert len(with_lib) > budget, (
        f"only {len(with_lib)} buildings got an archetype against a live "
        f"budget of {budget} — the library is not buying anything")


def test_archetype_geometry_is_taken_as_authored(config, full_library):
    """A baked archetype is re-centred, metres, Z-up, with its transform baked.

    Carrying the SOURCE asset's scale across the swap is how a 0.01-scaled
    Nucleus building comes back a hundred times too small.
    """
    for p in _build(config):
        if p.get("_archetype"):
            assert p["scale"] == 1.0
            assert p["axis_up"] == "Z"
            assert p["z_m"] == 0.0


def test_level_matches_the_field(config, full_library):
    """The referenced level must be the one the field and ladder agree on."""
    from disaster.field import make_damage_field
    dis = config["disaster"]
    region = tuple(float(v) for v in config["layout"]["region_m"])
    f = make_damage_field(dis.get("field"), (0.0, 0.0, region[0], region[1]))
    sev = float(dis["severity"])
    for p in _build(config):
        if not p.get("_archetype"):
            continue
        want = L.level_at(DTYPE, L.local_damage(f(p["x_m"], p["y_m"]), sev))
        assert p["_damage_level"] == want.name


def test_a_partial_library_falls_back_but_still_places(config, monkeypatch):
    """A bake that half-failed must still assemble, degrading down the ladder
    rather than leaving holes."""
    recs = []
    for it in P.build_plan(config, DTYPE):
        recs.append({"type": it.type, "level": "cracked", "kind": it.kind,
                     "source": str(it.source),
                     "usd": f"{it.type}_cracked.usd"})
    library = lib.Library(
        path=os.path.join(_SCENE_GEN, "assets", "archetypes", DTYPE,
                          lib.MANIFEST_NAME),
        doc={"disaster": DTYPE, "archetypes": recs})
    monkeypatch.setattr(ds, "_ARCH_CACHE", {ds._arch_key(DTYPE): library})
    placements = _build(config)
    hit = [p for p in placements if p.get("_archetype")]
    assert hit
    assert all("_cracked.usd" in p["usd"] for p in hit)
    assert library.misses, "fell back silently instead of recording it"


# --------------------------------------------------------------------------
# The swap has to COMPOSE, not just resolve. Everything above checks the
# placement list; none of it puts an archetype on a stage, which is how a
# library whose every root was a (non-Xformable) Scope shipped and every
# archetype-backed building came up as an empty lot with a debris ring.

def _box_source(x, y):
    """A 10 x 6 x 4 m box building at (x, y), the way a scene would hold it."""
    from pxr import Gf, Usd, UsdGeom
    src = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(src, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(src, 1.0)
    UsdGeom.Xform.Define(src, "/World")
    xf = UsdGeom.Xform.Define(src, "/World/Bldg")
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    m = UsdGeom.Mesh.Define(src, "/World/Bldg/box")
    pts = [(a, b, c) for c in (0.0, 4.0) for b in (0.0, 6.0) for a in (0.0, 10.0)]
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([4] * 6)
    m.CreateFaceVertexIndicesAttr([0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1,
                                   2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3])
    return src


def _world_min(stage, path):
    from pxr import Usd, UsdGeom
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_])
    return cache.ComputeWorldBound(stage.GetPrimAtPath(path)) \
        .ComputeAlignedRange().GetMin()


def _place(url):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    p = {"usd": url, "category": "house", "x_m": 20.0, "y_m": -30.0,
         "z_m": 0.0, "yaw_deg": 0.0, "scale": 1.0, "axis_up": "Z"}
    sg.apply_placements(stage, [p], parent_path="/World/gen", resolver=None)
    return stage, p["prim_path"]


def test_a_baked_archetype_composes_where_it_was_placed(tmp_path, monkeypatch):
    from pxr import Sdf, Usd, UsdGeom
    from disaster import bake as B

    monkeypatch.setitem(sg.LOCAL_ASSET_ROOTS, "airstack", str(tmp_path))
    out = tmp_path / "arch" / "T_pancaked.usd"
    out.parent.mkdir()
    assert B.export_object(_box_source(100.0, 50.0), None, ["/World/Bldg"],
                           str(out), recenter=(100.0, 50.0, 0.0))

    # The archetype says what it is: metres, Z-up, an Xformable root.
    st = Usd.Stage.Open(str(out))
    assert st.GetDefaultPrim().IsA(UsdGeom.Xformable)
    assert UsdGeom.GetStageUpAxis(st) == UsdGeom.Tokens.z
    assert UsdGeom.GetStageMetersPerUnit(st) == 1.0

    # It can be measured through the scheme the placement carries ...
    fp = sg._measure_footprint_raw("airstack://arch/T_pancaked.usd")
    assert fp is not None
    assert abs(fp["sx"] - 10.0) < 1e-3 and abs(fp["sy"] - 6.0) < 1e-3

    # ... and it lands on its lot, re-centred, rather than being skipped.
    stage, path = _place("airstack://arch/T_pancaked.usd")
    assert stage.GetPrimAtPath(path).IsA(UsdGeom.Xformable)
    lo = _world_min(stage, path)
    assert abs(lo[0] - 20.0) < 1e-3 and abs(lo[1] + 30.0) < 1e-3 \
        and abs(lo[2]) < 1e-3

    # A library baked before the root became an Xform is promoted in place:
    # Scope -> Xform loses nothing, and re-baking hours of archetypes to fix
    # a prim type is not a reasonable ask.
    legacy = tmp_path / "arch" / "T_legacy.usd"
    layer = Sdf.Layer.FindOrOpen(str(out))
    layer.GetPrimAtPath("/Baked").typeName = "Scope"
    layer.Export(str(legacy))
    stage, path = _place("airstack://arch/T_legacy.usd")
    assert stage.GetPrimAtPath(path).IsA(UsdGeom.Xformable)
    lo = _world_min(stage, path)
    assert abs(lo[0] - 20.0) < 1e-3 and abs(lo[1] + 30.0) < 1e-3


def test_export_keeps_relative_textures_resolvable(tmp_path):
    """The first library was black: every texture was authored relative to
    the SOURCE layer (`Textures/x.png` beside the building) and re-anchored
    verbatim into the archetype directory, where no `Textures/` exists."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
    from disaster import bake as B

    src_dir = tmp_path / "src"
    (src_dir / "Textures").mkdir(parents=True)
    tex = src_dir / "Textures" / "wall.png"
    tex.write_bytes(b"\x89PNG\r\n\x1a\n")
    src_path = str(src_dir / "building.usda")
    st = _box_source(0.0, 0.0)
    mat = UsdShade.Material.Define(st, "/World/Looks/Wall")
    sh = UsdShade.Shader.Define(st, "/World/Looks/Wall/tex")
    sh.CreateIdAttr("UsdUVTexture")
    sh.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath("Textures/wall.png"))
    UsdShade.MaterialBindingAPI.Apply(
        st.GetPrimAtPath("/World/Bldg/box")).Bind(mat)
    st.GetRootLayer().Export(src_path)
    st = Usd.Stage.Open(src_path)

    out = tmp_path / "arch" / "T.usd"
    out.parent.mkdir()
    assert B.export_object(st, None, ["/World/Bldg"], str(out))
    assert B.unresolved_textures(str(out)) == []
    got = Usd.Stage.Open(str(out))
    shader = next(p for p in got.Traverse() if p.GetName() == "tex")
    ap = UsdShade.Shader(shader).GetInput("file").Get()
    assert ap.path == str(tex) or ap.resolvedPath == str(tex), ap
    # and the exported file binds through an APPLIED schema — pxr warns on
    # every prim otherwise, and stricter builds ignore the binding.
    mesh = next(p for p in got.Traverse() if p.IsA(UsdGeom.Mesh))
    assert mesh.HasAPI(UsdShade.MaterialBindingAPI)


def test_export_merges_settled_fragments_per_material(tmp_path):
    """A settled wreck is static at load, so its ~60 fragments export as one
    mesh per material with a static collider — the renderer and PhysX pay
    per mesh, on every launch, for geometry that is fixed at bake time."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    from disaster import bake as B

    st = _box_source(0.0, 0.0)
    brick = UsdShade.Material.Define(st, "/World/Looks/Brick")
    core = UsdShade.Material.Define(st, "/World/Looks/Core")
    UsdShade.MaterialBindingAPI.Apply(
        st.GetPrimAtPath("/World/Bldg/box")).Bind(brick)
    UsdGeom.Scope.Define(st, "/World/Bldg/fragments")
    n_tri = 0
    for i in range(3):
        m = UsdGeom.Mesh.Define(st, f"/World/Bldg/fragments/frag_{i:03d}")
        m.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0),
                            Gf.Vec3f(0, 1, 0), Gf.Vec3f(1, 1, 1)])
        m.CreateFaceVertexCountsAttr([3, 3])
        m.CreateFaceVertexIndicesAttr([0, 1, 2, 1, 3, 2])
        UsdGeom.PrimvarsAPI(m).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.vertex).Set([Gf.Vec2f(0, 0)] * 4)
        UsdGeom.Xformable(m).AddTranslateOp().Set(Gf.Vec3d(5.0 * i, 0, 0))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(brick)
        sub = UsdGeom.Subset.CreateGeomSubset(
            m, "mat_core", UsdGeom.Tokens.face, [1], "materialBind")
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(core)
        n_tri += 2
    out = tmp_path / "T.usd"
    assert B.export_object(st, None, ["/World/Bldg"], str(out))
    got = Usd.Stage.Open(str(out))
    meshes = {p.GetName(): p for p in got.Traverse() if p.IsA(UsdGeom.Mesh)}
    assert set(meshes) == {"box", "rubble_Brick", "rubble_Core"}, meshes
    tri = {n: len(UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get())
           for n, p in meshes.items()}
    assert tri["rubble_Brick"] + tri["rubble_Core"] == n_tri
    assert tri["rubble_Core"] == 3                     # one face per fragment
    rb = meshes["rubble_Brick"]
    assert rb.HasAPI(UsdPhysics.CollisionAPI)
    assert str(UsdShade.MaterialBindingAPI(rb).ComputeBoundMaterial()[0]
               .GetPrim().GetName()) == "Brick"
    pts = UsdGeom.Mesh(rb).GetPointsAttr().Get()
    assert len(UsdGeom.PrimvarsAPI(rb).GetPrimvar("st").Get()) == len(pts)
    assert max(p[0] for p in pts) > 9.0                # world space kept


# --------------------------------------------------------------------------
# the debris a baked archetype carries inside it
# --------------------------------------------------------------------------

@pytest.fixture
def library_with_debris(config, monkeypatch):
    """The same manifest, but every record says how far its rubble reaches."""
    recs = []
    for it in P.build_plan(config, DTYPE):
        for _t, lv in it.combos:
            recs.append({"type": it.type, "level": lv, "kind": it.kind,
                         "source": str(it.source),
                         "usd": lib.archetype_name(it.type, lv) + ".usd",
                         "debris_r_m": 0.0 if lv == "pristine" else 11.5})
    library = lib.Library(
        path=os.path.join(_SCENE_GEN, "assets", "archetypes", DTYPE,
                          lib.MANIFEST_NAME),
        doc={"disaster": DTYPE, "archetypes": recs})
    monkeypatch.setattr(ds, "_ARCH_CACHE", {ds._arch_key(DTYPE): library})
    return library


def test_an_archetype_carries_its_own_debris_and_scatters_none(config,
                                                               full_library):
    """The debris is baked INSIDE the archetype USD, so a building that got one
    must not also have rubble scattered around it — that was the whole cost of
    doing it at placement time."""
    placements = _build(config)
    arch = [p for p in placements if p.get("_archetype")]
    assert arch
    loose = [p for p in placements
             if p.get("category") in ("debris", "debris_pile")]
    for p in loose:
        near = min((abs(p["x_m"] - q["x_m"]) + abs(p["y_m"] - q["y_m"]))
                   for q in arch)
        assert near > 1.0, "scattered rubble on top of an archetype's own"


def test_the_baked_debris_reach_reaches_the_survey(config,
                                                   library_with_debris):
    """Once the rubble is inside the USD it is not a placement, and `targets`
    samples casualties against the placement list — which is how the first
    debris-aware run put a victim inside a pile. The building carries the
    reach instead."""
    import targets as T

    placements = _build(config)
    arch = [p for p in placements if p.get("_archetype")]
    assert arch and all(p.get("_debris_r_m") == 11.5 for p in arch)

    survey = T.survey_from_placements(placements, disaster_type=DTYPE)
    discs = {(round(d["x"], 3), round(d["y"], 3)) for d in survey["debris"]}
    for p in arch:
        assert (round(p["x_m"], 3), round(p["y_m"], 3)) in discs


def test_archetypes_are_instanced_and_nothing_else_is(config, full_library):
    """SPEC's "repeated archetypes are instanced so identical references share
    geometry". The flag has to be on the archetypes and ONLY on them: an
    instanceable prim has no traversable children, so instancing a building the
    cutter is about to edit, or one `add_colliders` still has to reach into,
    breaks the sim silently."""
    placements = _build(config)
    arch = [p for p in placements if p.get("_archetype")]
    assert arch, "a full library was loaded and nothing referenced it"
    assert all(p.get("_instanceable") for p in arch)

    marked = [p for p in placements if p.get("_instanceable")]
    assert len(marked) == len(arch), "something that is not an archetype is instanced"
    # The cutter edits geometry inside its buildings — those must stay unique.
    assert not any(p.get("_mesh_damage") for p in marked)


def test_instancing_shares_one_prototype_per_archetype(tmp_path):
    """The point of the flag, measured on a stage: N placements of the same
    asset must compose to ONE prototype, not N copies.

    Self-contained rather than built on `full_library`, which plants a manifest
    and no USD files — the references cannot compose, so nothing becomes an
    instance and the test would pass or fail for the wrong reason."""
    from pxr import Usd, UsdGeom

    import scene_generator as sg

    assets = []
    for i in range(2):
        f = str(tmp_path / f"a{i}.usda")
        a = Usd.Stage.CreateNew(f)
        UsdGeom.SetStageMetersPerUnit(a, 1.0)
        c = UsdGeom.Cube.Define(a, "/root")
        c.CreateSizeAttr(2.0 + i)
        a.SetDefaultPrim(c.GetPrim())
        a.GetRootLayer().Save()
        assets.append(f)

    pls = [{"usd": assets[i % 2], "category": "house", "x_m": 10.0 * i,
            "y_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0, "roll_deg": 0.0,
            "pitch_deg": 0.0, "scale": 1.0, "axis_up": "Z",
            "_instanceable": True} for i in range(6)]
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    sg.apply_placements(st, pls, "/World/gen", 1.0)

    instanced = [p for p in st.Traverse() if p.IsInstance()]
    assert len(instanced) == 6
    assert len({p.GetPrototype() for p in instanced}) == 2, "nothing was shared"

    # And WITHOUT the flag the same six are unique prims, which is the default
    # every other category still relies on.
    plain = [dict(q, _instanceable=False) for q in pls]
    st2 = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st2, 1.0)
    sg.apply_placements(st2, plain, "/World/gen", 1.0)
    assert not [p for p in st2.Traverse() if p.IsInstance()]


# --------------------------------------------------------------------------
# which library a run reads — SCENE_ARCHETYPE_ROOT
# --------------------------------------------------------------------------
#
# An iteration bake lives BESIDE the live library rather than replacing it
# (`assets/archetypes_urban_v2/` next to `assets/archetypes/`), so a scene has
# to be able to say which one it means. Getting this wrong is silent: the
# wrong root simply has fewer types in it, Stage B falls back down the ladder,
# and the scene comes out gently damaged instead of wrecked.

def _plant(root: str, types=("house_a",), levels=("pristine", "pancaked")):
    """Write a real manifest under *root* and return its path."""
    recs = [{"type": t, "level": lv, "kind": L.STRUCTURE, "source": t,
             "usd": lib.archetype_name(t, lv) + ".usd"}
            for t in types for lv in levels]
    path = os.path.join(lib.disaster_dir("", DTYPE, root), lib.MANIFEST_NAME)
    return lib.write_manifest(path, recs)


def test_archetype_root_env_selects_the_library(tmp_path, monkeypatch):
    monkeypatch.setattr(ds, "_ARCH_CACHE", {})
    root = str(tmp_path / "archetypes_elsewhere")
    _plant(root, types=("only_here",))

    monkeypatch.setenv("SCENE_ARCHETYPE_ROOT", root)
    got = ds._archetypes(DTYPE)
    assert got is not None
    assert sorted(got.types()) == ["only_here"]
    # and it really came from there, not from the repo's own library
    assert str(got.path).startswith(root)


def test_archetype_root_unset_keeps_the_default(tmp_path, monkeypatch):
    """The default must not move — every existing preset depends on it."""
    monkeypatch.setattr(ds, "_ARCH_CACHE", {})
    monkeypatch.delenv("SCENE_ARCHETYPE_ROOT", raising=False)
    # `_archetypes` returns None when nothing is baked, which is the normal
    # state of a fresh checkout; either way the PATH it consulted is the point.
    expected = os.path.join(_SCENE_GEN, lib.DEFAULT_ROOT, DTYPE)
    assert ds._arch_key(DTYPE) == (DTYPE, "")
    assert lib.disaster_dir(_SCENE_GEN, DTYPE, "") == expected


def test_two_roots_do_not_share_a_cache_entry(tmp_path, monkeypatch):
    """The bug this keys the cache on the root to prevent: a sweep that
    switches libraries kept being handed whichever one it loaded first."""
    monkeypatch.setattr(ds, "_ARCH_CACHE", {})
    a, b = str(tmp_path / "a"), str(tmp_path / "b")
    _plant(a, types=("from_a",))
    _plant(b, types=("from_b",))

    monkeypatch.setenv("SCENE_ARCHETYPE_ROOT", a)
    assert sorted(ds._archetypes(DTYPE).types()) == ["from_a"]
    monkeypatch.setenv("SCENE_ARCHETYPE_ROOT", b)
    assert sorted(ds._archetypes(DTYPE).types()) == ["from_b"]
