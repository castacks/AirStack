#!/usr/bin/env python3
"""test_tornado_urban_usd.py — does `disaster/tornado_urban_usd.py` apply a
§2.8-shaped tornado plan to a sliced building correctly?

    python3 scene_gen/tests/test_tornado_urban_usd.py
    pytest -q scene_gen/tests/test_tornado_urban_usd.py

WHAT THIS FILE COVERS AND WHAT IT CANNOT SEE
----------------------------------------------
`tornado_urban_usd.apply_plan` is the USD half of the urban-tornado ladder
(`scene_gen/_plans/urban_tornado_plan.md` §2.9): it takes a PLAN — a plain
JSON dict a pure planner (`disaster/tornado_urban.py`, stream L, written in
parallel and not necessarily present yet) produced — and does to a live
stage what the plan says: deactivate removed pieces, rebind broken-glass
subsets to a void material, rigid-displace hanging panels and macroblocks,
sweep roof furniture, and author the removed pieces' street debris as one
merged mesh per (kind, material) class.

Since stream L's planner may not exist yet, this file builds its OWN plan —
`tests/fixtures/tornado_urban_plan_fixture.json`, a HAND-WRITTEN example in
the §2.8 schema — against a synthetic ~30-piece "sliced building" this file
authors itself with `quake_flow._box` (the same box-authoring convention the
rest of this codebase's damage ladders use). If
`tests/fixtures/tornado_urban_plan_example.json` (stream L's own real
example) exists by the time this runs, a second test applies that one too
and checks its debris count matches `len(plan["debris"])` — nothing more,
since a real plan's prim paths belong to a real sliced building this file
does not have.

RUNS WITHOUT ISAAC. `pxr` (usd-core) is on the host; VTK is not, and nothing
here needs it — no fracture, no slicing, just plan application to an
in-memory stage.

WHAT THIS FILE CANNOT SEE: whether a rebuilt void pane actually reads as a
dark hole, whether the debris field looks like a wrecked building's
material rather than a shape, whether the wind-side rule (§2.4/§2.5, stream
L's job) actually put more removed pieces on the windward side. That needs
the planner (for the wind-side/toothing/cap checks, which are ITS tests,
not this file's) and a render.
"""

import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import numpy as np                                   # noqa: E402
import pytest                                         # noqa: E402
from pxr import Sdf, Usd, UsdGeom, UsdShade, Vt          # noqa: E402

from disaster import damage                             # noqa: E402
from disaster import quake_flow as qf                  # noqa: E402
from disaster import quake_sliced as qs                # noqa: E402
from disaster import tornado_urban_usd as tu            # noqa: E402

FIXTURE_PATH = os.path.join(_HERE, "fixtures", "tornado_urban_plan_fixture.json")
EXAMPLE_PATH = os.path.join(_HERE, "fixtures", "tornado_urban_plan_example.json")

CELL = "/World/cell"
PIECES = CELL + "/pieces"

# The six glass-bearing corner pieces at storeys 3-5 (see the fixture) —
# these get a GeomSubset bound to a material whose NAME matches
# `detail.gac_slice.is_glazing` (the "win"/"glass"/"glazing" token list),
# the same way a real GAC piece's glazing subset is named (the lead's own
# note: "e.g. M_Building_02_Glass_Inst").
GLASS_STOREY_BAYS = ((3, 0), (3, 4), (4, 0), (4, 4), (5, 0), (5, 4))


def _load_plan(path=FIXTURE_PATH):
    with open(path) as f:
        return json.load(f)


def _build_stage():
    """A synthetic sliced building: 6 storeys x 5 bays of wall pieces under
    `/World/cell/pieces/wall_S_<storey>_<bay>`, plus one extra identity-
    transform piece (`disp_panel_0`) for the displaced-matrix exactness
    check. `/World`, `/World/cell` and `/World/cell/pieces` are all bare
    `Xform.Define`s with NO ops — every piece's parent chain is the
    identity, which is what `quake_flow._transform_prims`'s own docstring
    requires ("Only valid for prims whose parent is at the identity") and
    what lets the displaced-piece test compare a WORLD matrix directly
    against `rigid_matrix(spec)` with no piece-parent transform folded in.
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))
    UsdGeom.Xform.Define(stage, Sdf.Path(PIECES))

    glass_mat = UsdShade.Material.Define(stage, Sdf.Path(CELL + "/Looks/M_Glass_Inst"))

    for storey in range(6):
        for bay in range(5):
            path = "{0}/wall_S_{1}_{2}".format(PIECES, storey, bay)
            cx = bay * 4.0 - 8.0
            cz = storey * 4.0 + 2.0
            qf._box(stage, path, cx, 0.0, cz, 4.0, 0.3, 4.0, yaw_deg=0.0)
            if (storey, bay) in GLASS_STOREY_BAYS:
                mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
                sub = UsdGeom.Subset.CreateGeomSubset(
                    mesh, "pane", UsdGeom.Tokens.face, Vt.IntArray([0, 1]))
                UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(glass_mat)

    # The one piece the displaced-matrix test needs at a pure identity
    # transform (no translate/rotateZ ops the way `_box`-authored pieces
    # carry) so its post-`apply_plan` WORLD matrix equals `rigid_matrix`
    # with nothing else folded in.
    UsdGeom.Xform.Define(stage, Sdf.Path(PIECES + "/disp_panel_0"))

    return stage


def _fresh_ctx(stage):
    return {"stage": stage, "parent": CELL, "tag": "tp", "mats": {},
            "static_extra": [], "loose": [], "authored": [], "info": {},
            "notes": []}


def _all_frag_lowest_z(stage, mesh_path):
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(mesh_path))
    pts = mesh.GetPointsAttr().Get()
    return min(float(p[2]) for p in pts)


def _bind_gac_glazing(stage, mesh_path, indices, mat_path,
                      tex_name="M_Building_02_Glass_BaseColor.png"):
    """Author the REAL GAC glazing shape on `indices` faces of the mesh at
    `mesh_path`: a `materialBind`-family `GeomSubset` bound to a material
    prim named `UnrealMaterial` whose `UsdPreviewSurface.diffuseColor` is
    CONNECTED to a `UsdUVTexture` (`file = tex_name`) rather than holding a
    value directly — the shape the lead's container probe measured on
    SM_Building_02 and the reason `_glass_tex_and_name`/`_void_glass`/
    `annotate_glazing` all exist instead of a plain `.Get()` + name check.
    Returns the `UsdGeom.Subset`.
    """
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(mesh_path))
    surf = UsdShade.Shader.Define(stage, Sdf.Path(mat_path).AppendChild("PBRShader"))
    surf.CreateIdAttr("UsdPreviewSurface")
    tex = UsdShade.Shader.Define(stage, Sdf.Path(mat_path).AppendChild("tex"))
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex_name))
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    mat = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")

    sub = UsdGeom.Subset.CreateGeomSubset(
        mesh, "gac_glazing", UsdGeom.Tokens.face, Vt.IntArray(list(indices)),
        UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
    UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)
    return sub


# ---------------------------------------------------------------------------

def test_removed_pieces_go_inactive_and_missing_are_counted_not_raised():
    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)

    out = tu.apply_plan(stage, ctx, plan, verbose=False)

    assert out["n_removed"] == 5, out
    assert out["n_missing"] == 2, out
    for path in plan["removed"]:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            assert not prim.IsActive(), path
    # the two missing paths never existed and nothing raised getting here
    assert stage.GetPrimAtPath("/World/cell/pieces/ghost_piece_zz").IsValid() is False


def test_glass_subsets_rebind_to_void_material():
    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)

    out = tu.apply_plan(stage, ctx, plan, verbose=False)
    assert out["n_glass"] == len(plan["glass"]), out  # 1 subset each
    void = ctx["mats"]["void"]
    assert void is not None

    for path in plan["glass"]:
        mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh))
        assert subs, path
        for s in subs:
            bound = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
            assert bound.GetPrim().GetPath() == void.GetPrim().GetPath(), (path, bound)


def test_void_glass_follows_unrealmaterial_texture_connection_qs_finds_nothing():
    """Pins the exact defect the lead's END-TO-END CONTAINER PROBE found on
    a REAL slice (SM_Building_02, level T3): the planner listed 13 glass
    pieces and `quake_sliced._void_glass` voided 0. Reproduces the two
    reasons at once — a material prim named `UnrealMaterial` (so the old
    NAME check never fires) whose `UsdPreviewSurface.diffuseColor` is
    CONNECTED to a `UsdUVTexture` rather than holding a value (so the old
    TEXTURE check, `quake_sliced._tex_of`'s plain `.Get()`, never fires
    either) — and asserts `quake_sliced._void_glass` finds nothing on it
    while this module's OWN `_void_glass` (`_glass_tex_and_name`, which
    follows the connection the way `gac_slice.window_centres._tex` does)
    rebinds it correctly.
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))
    UsdGeom.Xform.Define(stage, Sdf.Path(PIECES))

    piece_path = PIECES + "/wall_S_real_glazing"
    qf._box(stage, piece_path, 0.0, 0.0, 6.0, 4.0, 0.3, 4.0, yaw_deg=0.0)

    # THE REAL GAC SHAPE (lead's own note): the piece prim itself is bound
    # to a ShellFallbackLooks material elsewhere in the real pipeline; its
    # SUBSET binds the source asset's own `Section*/UnrealMaterial` — this
    # test only needs the subset side, which is what `_void_glass` walks.
    mat_path = piece_path + "/src/asset/LOD0/Section6/UnrealMaterial"
    sub = _bind_gac_glazing(stage, piece_path, [2, 3], mat_path)
    mat = UsdShade.Material.Get(stage, mat_path)

    void_mat = damage._pbr(stage, CELL + "/TestVoidLook", (0.03, 0.028, 0.03), 0.15)

    # THE EARTHQUAKE STREAM'S FUNCTION: must find nothing on this asset —
    # this is the container-probe measurement, pinned.
    n_old = qs._void_glass(stage, [piece_path], void_mat)
    assert n_old == 0, n_old
    still_bound = UsdShade.MaterialBindingAPI(sub.GetPrim()).ComputeBoundMaterial()[0]
    assert still_bound.GetPrim().GetPath() == mat.GetPrim().GetPath(), \
        "quake_sliced._void_glass must not have touched the subset"

    # THIS MODULE'S OWN FUNCTION: must rebind it.
    n_new = tu._void_glass(stage, [piece_path], void_mat)
    assert n_new == 1, n_new
    bound = UsdShade.MaterialBindingAPI(sub.GetPrim()).ComputeBoundMaterial()[0]
    assert bound.GetPrim().GetPath() == void_mat.GetPrim().GetPath()


def test_annotate_glazing_stamps_faces_and_frac_per_piece():
    """`annotate_glazing` measures glazing per PIECE, not by role — the
    fix for the live-probe finding that a role-based "wall = pane" guess
    can never find SM_Building_02's glass (it lives on `pier`/`core`
    pieces, never `wall`, per the slicer's BAY_SPLITS phase). Three
    placements, one carrying the real connected-texture glazing shape
    (`_bind_gac_glazing`), two with no glazing at all: stamp must read
    1/0/0 on `_glass_faces`, and the glazed piece's `_glass_frac` must be
    in `(0, 1]`.
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))
    UsdGeom.Xform.Define(stage, Sdf.Path(PIECES))

    glazed_path = PIECES + "/pier_S_glazed"
    plain_a_path = PIECES + "/wall_S_plain_a"
    plain_b_path = PIECES + "/wall_S_plain_b"
    for path in (glazed_path, plain_a_path, plain_b_path):
        qf._box(stage, path, 0.0, 0.0, 6.0, 4.0, 0.3, 4.0, yaw_deg=0.0)

    # Glazing on TWO of the glazed piece's six faces — same connected-
    # texture shape `_void_glass` needs, on a `pier`-named piece (the
    # role the real asset actually carries its glass on).
    _bind_gac_glazing(stage, glazed_path, [0, 1],
                      glazed_path + "/src/asset/LOD0/Section7/UnrealMaterial")
    # `plain_a` carries a bound but non-glazing material, `plain_b` carries
    # no subset (and no binding) at all — both must stamp zero.
    wallback = UsdShade.Material.Define(
        stage, Sdf.Path(plain_a_path + "/src/asset/LOD0/Section3/UnrealMaterial"))
    UsdShade.MaterialBindingAPI.Apply(
        stage.GetPrimAtPath(plain_a_path)).Bind(wallback)

    placements = [{"prim_path": glazed_path}, {"prim_path": plain_a_path},
                 {"prim_path": plain_b_path}]

    n_hit = tu.annotate_glazing(stage, placements)
    assert n_hit == 1, n_hit

    faces = [p["_glass_faces"] for p in placements]
    fracs = [p["_glass_frac"] for p in placements]
    assert faces[0] > 0 and faces[1] == 0 and faces[2] == 0, faces
    assert 0.0 < fracs[0] <= 1.0, fracs[0]
    assert fracs[1] == 0.0 and fracs[2] == 0.0, fracs
    assert isinstance(faces[0], int) and isinstance(fracs[0], float)


def test_displaced_prim_carries_the_exact_rigid_matrix():
    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)

    out = tu.apply_plan(stage, ctx, plan, verbose=False)
    assert out["n_displaced"] == 1, out

    path, spec = next(iter(plan["displaced"].items()))
    want = qs.rigid_matrix(spec)
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.XformCache()
    got = np.array(xf.GetLocalToWorldTransform(prim)).reshape(4, 4)

    assert np.allclose(got, want, atol=1e-6), (got, want)
    assert path in ctx["static_extra"]


def test_roof_props_sweep_deactivates_ctx_roof_plant_and_roof_fixed():
    stage = _build_stage()
    ac_path = CELL + "/roof_ac_0"
    tank_path = CELL + "/roof_tank_0"
    qf._box(stage, ac_path, 0.0, 0.0, 25.0, 1.0, 0.6, 1.0, yaw_deg=0.0)
    qf._box(stage, tank_path, 2.0, 0.0, 25.0, 1.4, 1.4, 2.4, yaw_deg=0.0)

    plan = _load_plan()
    ctx = _fresh_ctx(stage)
    ctx["roof_plant"] = [ac_path]
    ctx["roof_fixed"] = [tank_path]

    out = tu.apply_plan(stage, ctx, plan, verbose=False)
    assert out["n_roof_props"] == 2, out
    assert not stage.GetPrimAtPath(ac_path).IsActive()
    assert not stage.GetPrimAtPath(tank_path).IsActive()


def test_roof_props_keep_leaves_everything_active():
    stage = _build_stage()
    ac_path = CELL + "/roof_ac_0"
    qf._box(stage, ac_path, 0.0, 0.0, 25.0, 1.0, 0.6, 1.0, yaw_deg=0.0)

    plan = dict(_load_plan())
    plan["roof_props"] = "keep"
    ctx = _fresh_ctx(stage)
    ctx["roof_plant"] = [ac_path]

    out = tu.apply_plan(stage, ctx, plan, verbose=False)
    assert out["n_roof_props"] == 0, out
    assert stage.GetPrimAtPath(ac_path).IsActive()


def test_debris_one_mesh_per_kind_material_class():
    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)

    out = tu.apply_plan(stage, ctx, plan, verbose=False)
    classes = sorted({(f["kind"], f["material"]) for f in plan["debris"]})
    assert out["n_debris_meshes"] == len(classes) == 6, (out, classes)
    assert out["n_fragments"] == len(plan["debris"]) == 18, out

    for kind, material in classes:
        path = "{0}/tornado_debris/{1}_{2}".format(CELL, kind, material)
        prim = stage.GetPrimAtPath(path)
        assert prim.IsValid(), path
        mesh = UsdGeom.Mesh(prim)
        n_frags = sum(1 for f in plan["debris"]
                     if f["kind"] == kind and f["material"] == material)
        normals = mesh.GetNormalsAttr().Get()
        assert mesh.GetNormalsInterpolation() == UsdGeom.Tokens.faceVarying, path
        assert len(normals) == 24 * n_frags, (path, len(normals), n_frags)
        pts = mesh.GetPointsAttr().Get()
        assert len(pts) == 8 * n_frags, (path, len(pts))


def test_debris_meshes_are_bound_to_a_material_and_glass_is_the_void_tone():
    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)
    tu.apply_plan(stage, ctx, plan, verbose=False)

    void = ctx["mats"]["void"]
    glass_mesh_path = CELL + "/tornado_debris/glass_glass"
    prim = stage.GetPrimAtPath(glass_mesh_path)
    assert prim.IsValid()
    bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    assert bound.GetPrim().GetPath() == void.GetPrim().GetPath(), \
        "glass shard debris must wear the SAME void material as a rebound pane"

    for kind, material in {(f["kind"], f["material"]) for f in plan["debris"]}:
        path = "{0}/tornado_debris/{1}_{2}".format(CELL, kind, material)
        b = UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(path)).ComputeBoundMaterial()[0]
        assert b is not None and b.GetPrim().IsValid(), path


def test_no_debris_material_in_urban_path_binds_wood_texture_or_wood_material():
    """R5 (round 2): `deck` fragments never take `planks.wood_material`
    again. `_classify`'s bucket vocabulary is now CLOSED to `glass` /
    `brick` / `concrete` / `metal` / `membrane` plus the flat "unrecognised"
    fallback (see that function's and `debris_material`'s own R5
    docstrings) -- there is no longer a `kind == "deck"` escape hatch that
    routes to `planks.wood_material`. Checked two ways over both the
    fixture plan's own classes AND every (kind, material) pair
    `tornado_urban`'s planner vocabulary can itself produce (past what this
    hand-written fixture happens to contain): no `.../TornadoDebrisLooks/
    deck_wood` prim (the retired look's own path) exists on the stage at
    all, and no bound material's shader carries any of `planks.WOOD_BASE` /
    `WOOD_NORMAL` / `WOOD_ORM` -- compared by the `Ash_Planks` pack folder
    name that uniquely identifies them, since `debris_material`'s own
    textures are resolved to an absolute/Nucleus path at authoring time
    (`_resolve_texture`) while `planks`'s constants stay `airstack://`.
    """
    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)
    tu.apply_plan(stage, ctx, plan, verbose=False)

    # Exercise the vocabulary directly too, past whatever the fixture
    # happens to contain -- every (kind, material) pair `tornado_urban`'s
    # OWN `_kind_of`/`_material_hint` can produce post-R5.
    for kind, material in (("panel", "concrete_panel"), ("block", "brick"),
                           ("coping", "coping"), ("glass", "glass"),
                           ("membrane", "membrane"), ("metal", "metal")):
        tu.debris_material(stage, ctx, kind, material)

    assert not stage.GetPrimAtPath(
        CELL + "/TornadoDebrisLooks/deck_wood").IsValid()

    checked = 0
    for mat in ctx["mats"].values():
        sh = UsdShade.Shader.Get(stage, str(mat.GetPrim().GetPath()) + "/Shader")
        assert sh is not None, mat.GetPrim().GetPath()
        for tex_key in ("diffuse_texture", "normalmap_texture", "ORM_texture"):
            inp = sh.GetInput(tex_key)
            v = inp.Get() if inp is not None else None
            if v is None:
                continue
            resolved = str(v.path)
            assert "Ash_Planks" not in resolved, (mat.GetPrim().GetPath(), resolved)
            checked += 1
    assert checked > 0


def test_textured_debris_classes_resolve_through_the_airstack_scheme(monkeypatch):
    """PORTABILITY (lead review fix 1): `brick`/`concrete` are the two
    textured classes, and their texture must be resolved through
    `scene_generator._join_asset_root` AT AUTHORING TIME, not baked into a
    module-level filesystem-path constant at import time — the exact
    `.agents/skills/freeze-portable-scenes/SKILL.md` "build-machine asset
    paths" trap.

    Proven by monkeypatching `scene_generator.LOCAL_ASSET_ROOTS["airstack"]`
    to a fake Nucleus root AFTER both modules are already imported (as they
    are for the whole test session): `_expand_scheme` reads that dict fresh
    on every call, so if `tornado_urban_usd` had instead computed and cached
    an absolute path once at import time (its first draft), this monkeypatch
    would change nothing and the assertions below would fail against the
    OLD (real, `/home/...`) path instead.
    """
    import scene_generator as sg

    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)

    fake_root = "omniverse://test-nucleus:3009/Projects/SEI-COA"
    monkeypatch.setitem(sg.LOCAL_ASSET_ROOTS, "airstack", fake_root)
    tu.apply_plan(stage, ctx, plan, verbose=False)

    for suffix, ext in (("brick", ".jpg"), ("concrete", ".png")):
        mat_path = "{0}/TornadoDebrisLooks/{1}".format(CELL, suffix)
        sh = UsdShade.Shader.Get(stage, mat_path + "/Shader")
        assert sh and sh.GetPrim().IsValid(), mat_path
        asset = sh.GetInput("diffuse_texture").Get()
        resolved = str(asset.path)
        assert resolved.startswith(fake_root), resolved
        assert not resolved.startswith("/home"), resolved
        assert not resolved.startswith(_HERE), resolved  # no test-tree path either
        assert resolved.endswith(ext), resolved
        assert "scene_gen/assets/materials/megascans" in resolved, resolved


def test_every_fragment_lowest_vertex_within_bedding_tolerance_of_ground():
    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)
    tu.apply_plan(stage, ctx, plan, verbose=False)

    classes = sorted({(f["kind"], f["material"]) for f in plan["debris"]})
    lo, hi = -0.03, 0.01
    for kind, material in classes:
        path = "{0}/tornado_debris/{1}_{2}".format(CELL, kind, material)
        z = _all_frag_lowest_z(stage, path)
        assert lo <= z <= hi, (path, z)


def test_z_lift_fragment_rests_on_the_heap_height():
    """ROUND 3 (stream DB's berm stacking + the lead's builder-side fix):
    a fragment carrying `z_lift` — its authored height within a wall-base
    rubble berm (`tornado_urban._deposit_berm`'s profile) — must come out
    LIFTED by exactly that much on top of the ordinary face-seated z,
    while a liftless fragment in the same class stays in the grade bedding
    band. Before the fix `build_debris` seated every fragment with
    `_seat_z` alone, flattening the berm to a mat — DB measured and
    documented the gap rather than faking stacking, and the builder now
    honours the field."""
    from pxr import UsdGeom as _UG

    stage = _build_stage()
    ctx = _fresh_ctx(stage)
    lift = 0.6
    frag = {"kind": "block", "size": [0.4, 0.3, 0.2], "y": 1.0, "z": 0.0,
            "yaw_deg": 10.0, "tilt_deg": 0.0, "material": "brick",
            "from": "berm_test"}
    plan = {"schema": "tornado_urban_plan.v1", "level": "T4",
            "debris": [dict(frag, x=3.0, z_lift=lift, stacked=True),
                       dict(frag, x=-3.0)]}
    tu.apply_plan(stage, ctx, plan, verbose=False)
    mesh = _UG.Mesh(stage.GetPrimAtPath(
        "{0}/tornado_debris/block_brick".format(CELL)))
    pts = mesh.GetPointsAttr().Get()
    assert pts and len(pts) == 16, pts
    lifted_lo = min(p[2] for p in pts if p[0] > 0.0)
    flat_lo = min(p[2] for p in pts if p[0] < 0.0)
    assert -0.03 <= flat_lo <= 0.01, flat_lo
    assert (lift - 0.03) <= lifted_lo <= (lift + 0.01), lifted_lo


def test_tilted_fragment_top_is_above_the_flat_ones_own_size():
    """The fixture's first two `glass` fragments share a size and differ
    ONLY in `tilt_deg` (0 vs 25) — the tilted one's box top must sit higher
    than the flat one's, the direct check `_seat_z`'s docstring promises."""
    plan = _load_plan()
    glass_frags = [f for f in plan["debris"] if f["kind"] == "glass"]
    flat = glass_frags[0]
    tilted = glass_frags[1]
    assert flat["tilt_deg"] == 0.0 and tilted["tilt_deg"] > 0.0
    assert flat["size"] == tilted["size"]

    l, w, t = flat["size"]
    z_flat = tu._seat_z(t, w, flat["tilt_deg"], ground_z=0.0)
    z_tilt = tu._seat_z(t, w, tilted["tilt_deg"], ground_z=0.0)
    half_flat = 0.5 * t
    tilt_rad = math.radians(tilted["tilt_deg"])
    half_tilt = 0.5 * (t * math.cos(tilt_rad) + w * math.sin(tilt_rad))

    top_flat = z_flat + half_flat
    top_tilt = z_tilt + half_tilt
    assert top_tilt > top_flat, (top_tilt, top_flat)


def test_materials_cache_holds_one_entry_per_class_after_two_applies():
    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)

    tu.apply_plan(stage, ctx, plan, verbose=False)
    n_after_first = len([k for k in ctx["mats"] if k.startswith("tornado_debris:")])

    # Re-apply the SAME plan on the SAME ctx/stage (removed pieces already
    # inactive, so counts drop, but the material cache must not grow).
    tu.apply_plan(stage, ctx, plan, verbose=False)
    n_after_second = len([k for k in ctx["mats"] if k.startswith("tornado_debris:")])

    assert n_after_first == n_after_second
    classes = {(f["kind"], f["material"]) for f in plan["debris"]}
    # every class resolves to a bucket key; unique buckets <= unique classes
    assert 0 < n_after_first <= len(classes)


def test_apply_plan_is_deterministic_across_fresh_stages():
    plan = _load_plan()

    stage_a = _build_stage()
    out_a = tu.apply_plan(stage_a, _fresh_ctx(stage_a), plan, verbose=False)

    stage_b = _build_stage()
    out_b = tu.apply_plan(stage_b, _fresh_ctx(stage_b), plan, verbose=False)

    keys = ("n_removed", "n_missing", "n_glass", "n_displaced",
            "n_debris_meshes", "n_fragments", "n_roof_props")
    for k in keys:
        assert out_a[k] == out_b[k], (k, out_a, out_b)


def test_extent_is_authored_and_encloses_all_points():
    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)
    tu.apply_plan(stage, ctx, plan, verbose=False)

    classes = sorted({(f["kind"], f["material"]) for f in plan["debris"]})
    for kind, material in classes:
        path = "{0}/tornado_debris/{1}_{2}".format(CELL, kind, material)
        mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
        extent = mesh.GetExtentAttr().Get()
        assert extent is not None and len(extent) == 2
        lo, hi = extent
        pts = mesh.GetPointsAttr().Get()
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
        assert lo[0] <= min(xs) + 1e-6 and hi[0] >= max(xs) - 1e-6
        assert lo[1] <= min(ys) + 1e-6 and hi[1] >= max(ys) - 1e-6
        assert lo[2] <= min(zs) + 1e-6 and hi[2] >= max(zs) - 1e-6


def test_debris_authored_into_static_extra_not_loose():
    stage = _build_stage()
    plan = _load_plan()
    ctx = _fresh_ctx(stage)
    tu.apply_plan(stage, ctx, plan, verbose=False)

    classes = sorted({(f["kind"], f["material"]) for f in plan["debris"]})
    for kind, material in classes:
        path = "{0}/tornado_debris/{1}_{2}".format(CELL, kind, material)
        assert path in ctx["static_extra"], path
    assert ctx["loose"] == [], "debris is authored geometry, never simulated this round"


# ---------------------------------------------------------------------------
# ROUND 3b (§8e F3, stream FX2) — `annotate_surface` / textured
# `debris_material` / `build_debris` grouping
# ---------------------------------------------------------------------------
def _make_textured_material(stage, mat_path, tex_name):
    """A `UsdPreviewSurface` whose `diffuseColor` is CONNECTED to a
    `UsdUVTexture` (`file = tex_name`) — the same connection-following
    shape `_bind_gac_glazing` authors for a glazing subset (see that
    helper's own docstring), factored out here so a test can bind it
    either to a subset OR directly to a mesh prim (a kit module carries no
    subset at all — `annotate_surface`'s own "subset-less -> mesh's own
    binding" branch)."""
    surf = UsdShade.Shader.Define(stage, Sdf.Path(mat_path).AppendChild("PBRShader"))
    surf.CreateIdAttr("UsdPreviewSurface")
    tex = UsdShade.Shader.Define(stage, Sdf.Path(mat_path).AppendChild("tex"))
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex_name))
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    mat = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    return mat


def test_annotate_surface_dominant_subset_by_face_count_wins_and_skips_glazing():
    """A piece with THREE subsets: a small non-glazing cladding subset (2
    faces), a LARGER non-glazing cladding subset (3 faces) and a glazing
    subset (1 face). `annotate_surface` must stamp the LARGER non-glazing
    subset's texture — "subsets first (largest face count wins)" — and
    never the glazing one, even though nothing here is the biggest subset
    on the piece overall (glazing was excluded from the contest entirely,
    not merely outranked)."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))
    UsdGeom.Xform.Define(stage, Sdf.Path(PIECES))

    path = PIECES + "/wall_S_dominant"
    qf._box(stage, path, 0.0, 0.0, 6.0, 4.0, 0.3, 4.0, yaw_deg=0.0)
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))

    mat_small = _make_textured_material(
        stage, path + "/src/asset/LOD0/Section1/UnrealMaterial",
        "T_Brick_Small_1K_B.jpg")
    sub_small = UsdGeom.Subset.CreateGeomSubset(
        mesh, "small", UsdGeom.Tokens.face, Vt.IntArray([0, 1]),
        UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
    UsdShade.MaterialBindingAPI.Apply(sub_small.GetPrim()).Bind(mat_small)

    mat_big = _make_textured_material(
        stage, path + "/src/asset/LOD0/Section2/UnrealMaterial",
        "T_Brick_Big_1K_B.jpg")
    sub_big = UsdGeom.Subset.CreateGeomSubset(
        mesh, "big", UsdGeom.Tokens.face, Vt.IntArray([2, 3, 4]),
        UsdShade.Tokens.materialBind, UsdGeom.Tokens.partition)
    UsdShade.MaterialBindingAPI.Apply(sub_big.GetPrim()).Bind(mat_big)

    _bind_gac_glazing(stage, path, [5], path + "/src/asset/LOD0/Section3/UnrealMaterial",
                      tex_name="M_Building_02_Glass_BaseColor.png")

    placements = [{"prim_path": path}]
    n_hit = tu.annotate_surface(stage, placements)
    assert n_hit == 1, n_hit
    assert placements[0]["_tex_name"] == "T_Brick_Big_1K_B.jpg", placements[0]
    assert placements[0]["_tex_url"].endswith("T_Brick_Big_1K_B.jpg"), placements[0]


def test_annotate_surface_subsetless_kit_mesh_uses_direct_binding():
    """A KIT module mesh (round-2 vocabulary) carries NO `GeomSubset`s at
    all — one material bound directly to the mesh prim. `annotate_surface`
    must fall back to that direct binding, the same shape
    `annotate_glazing`'s own subset-less branch already handles."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))
    UsdGeom.Xform.Define(stage, Sdf.Path(PIECES))

    path = PIECES + "/bld_dw_terrace_wall0_2_10"
    qf._box(stage, path, 0.0, 0.0, 6.0, 4.0, 0.3, 4.0, yaw_deg=0.0)
    mat = _make_textured_material(stage, CELL + "/Looks/Stucco_Cream",
                                  "T_Stucco_Cream_1K_B.jpg")
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(mat)

    placements = [{"prim_path": path}]
    n_hit = tu.annotate_surface(stage, placements)
    assert n_hit == 1, n_hit
    assert placements[0]["_tex_name"] == "T_Stucco_Cream_1K_B.jpg"
    assert placements[0]["_tex_url"]


def test_annotate_surface_glazing_only_piece_stamps_empty_and_missing_prim_too():
    """A piece whose ONLY binding is glazing (subset or subset-less) must
    stamp `""`/`""`, never fall through to some other guess — and a
    placement whose `prim_path` does not resolve on the stage at all must
    stamp `""`/`""` too rather than raising, the same "counted, never
    raised" discipline `apply_plan`'s missing-path handling uses."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))
    UsdGeom.Xform.Define(stage, Sdf.Path(PIECES))

    glazed_subset_path = PIECES + "/pier_S_all_glass"
    glazed_direct_path = PIECES + "/bld_kit_window0_3_04"
    for path in (glazed_subset_path, glazed_direct_path):
        qf._box(stage, path, 0.0, 0.0, 6.0, 4.0, 0.3, 4.0, yaw_deg=0.0)
    _bind_gac_glazing(stage, glazed_subset_path, [0, 1, 2, 3, 4, 5],
                      glazed_subset_path + "/src/asset/LOD0/Section7/UnrealMaterial")
    window_mat = _make_textured_material(
        stage, CELL + "/Looks/M_Window", "T_Window_1K_B.jpg")
    UsdShade.MaterialBindingAPI.Apply(
        stage.GetPrimAtPath(glazed_direct_path)).Bind(window_mat)

    placements = [{"prim_path": glazed_subset_path},
                 {"prim_path": glazed_direct_path},
                 {"prim_path": PIECES + "/ghost_piece_never_authored"}]
    n_hit = tu.annotate_surface(stage, placements)
    assert n_hit == 0, n_hit
    for p in placements:
        assert p["_tex_url"] == "" and p["_tex_name"] == "", p


def test_debris_material_one_material_per_distinct_source_texture_not_per_fragment():
    """A plan whose fragments span TWO distinct source textures must create
    exactly TWO `tornado_debris:src:` materials — not one per fragment, not
    one shared flat bucket — while a THIRD call naming a texture already
    seen reuses the cached material object exactly. The suburb-lesson
    sanity check the round's own brief asks for: "a bench building carries
    1-3 distinct cladding textures, never one per fragment"."""
    stage = _build_stage()
    ctx = _fresh_ctx(stage)

    mat_a1 = tu.debris_material(stage, ctx, "panel", "concrete_panel",
                                tex_url="airstack://x/T_stone_a.jpg",
                                tex_name="T_stone_a.jpg")
    mat_a2 = tu.debris_material(stage, ctx, "block", "brick",
                                tex_url="airstack://x/T_stone_a.jpg",
                                tex_name="T_stone_a.jpg")
    mat_b = tu.debris_material(stage, ctx, "panel", "concrete_panel",
                               tex_url="airstack://y/T_stone_b.jpg",
                               tex_name="T_stone_b.jpg")

    src_keys = sorted(k for k in ctx["mats"] if k.startswith("tornado_debris:src:"))
    assert src_keys == ["tornado_debris:src:T_stone_a_jpg",
                        "tornado_debris:src:T_stone_b_jpg"], src_keys
    assert mat_a1.GetPrim().GetPath() == mat_a2.GetPrim().GetPath(), \
        "same texture name (even off a different kind/material) must share ONE material"
    assert mat_a1.GetPrim().GetPath() != mat_b.GetPrim().GetPath()

    # texture-less calls are UNCHANGED — still the flat per-bucket path,
    # never touching the new `:src:` keys.
    tu.debris_material(stage, ctx, "coping", "coping")
    src_keys_after = sorted(k for k in ctx["mats"] if k.startswith("tornado_debris:src:"))
    assert src_keys_after == src_keys


def _shader_inputs(stage, mat):
    sh = UsdShade.Shader.Get(stage, str(mat.GetPrim().GetPath()) + "/Shader")
    assert sh and sh.GetPrim().IsValid(), mat.GetPrim().GetPath()
    out = {}
    for key in ("diffuse_texture", "diffuse_color_constant", "diffuse_tint"):
        inp = sh.GetInput(key)
        out[key] = inp.Get() if inp is not None else None
    return out


def test_textured_debris_wears_the_map_not_a_dark_class_tint():
    """ROUND 4 (D3) — THE BLACK-BERM REGRESSION GUARD.

    `OmniPBR_ClearCoat.mdl` (652-654, read in the container, not assumed):

        diffuse        = tex::texture_isvalid(diffuse_texture)
                         ? desaturated_base : diffuse_color_constant;
        tinted_diffuse = multiply_colors(diffuse, diffuse_tint, 1.0).tint;

    So on a TEXTURED material `diffuse_tint` is the entire multiply and
    `diffuse_color_constant` is only the map-failed fallback. Round 3 put
    the ~0.30 class rgb in BOTH slots, which knocked a ~0.4-mean cladding
    map down to ~0.12 — every berm on the round-3 bench rendered near-black
    whatever building stood over it, and the inherited texture was
    invisible. Two invariants, for the source-textured path AND for the two
    class-textured buckets:

      * the tint is NEAR-NEUTRAL GRIME — bright enough to read (>= 0.70)
        and desaturated (channel spread <= 0.10), so the SOURCE map carries
        the colour;
      * the fallback constant is a plausible class albedo, never white
        (white is the "field of paper litter" failure mode when a map does
        not resolve) and never near-black.
    """
    stage = _build_stage()
    ctx = _fresh_ctx(stage)

    mats = {"src": tu.debris_material(stage, ctx, "panel", "brick",
                                      tex_url="airstack://x/T_facade.jpg",
                                      tex_name="T_facade.jpg"),
            "brick": tu.debris_material(stage, ctx, "block", "brick"),
            "concrete": tu.debris_material(stage, ctx, "panel",
                                           "concrete_panel")}
    for name, mat in sorted(mats.items()):
        v = _shader_inputs(stage, mat)
        assert v["diffuse_texture"] is not None, name
        tint = tuple(float(c) for c in v["diffuse_tint"])
        const = tuple(float(c) for c in v["diffuse_color_constant"])
        assert min(tint) >= 0.70, (name, tint)
        assert max(tint) <= 1.0, (name, tint)
        assert max(tint) - min(tint) <= 0.10, (name, "tint is a colour, "
                                               "not grime", tint)
        assert min(const) < 0.99, (name, "white fallback constant", const)
        assert min(const) >= 0.20, (name, "near-black fallback", const)


def test_untextured_debris_classes_are_not_near_black():
    """The flat (map-less) buckets — `metal`, `membrane`, the unrecognised
    fallback — apply their rgb exactly once (no texture, so
    `diffuse_color_constant` IS the albedo and `_fix_diffuse_tint` is not
    called). They still have to be TELLABLE from the ~0.18 asphalt they lie
    on: round 3's membrane was 0.22 linear, a value step of 0.04."""
    stage = _build_stage()
    ctx = _fresh_ctx(stage)
    for kind, material in (("metal", "metal"), ("membrane", "membrane"),
                           ("panel", "something_unrecognised")):
        mat = tu.debris_material(stage, ctx, kind, material)
        v = _shader_inputs(stage, mat)
        assert v["diffuse_texture"] is None, (kind, material)
        const = tuple(float(c) for c in v["diffuse_color_constant"])
        assert min(const) >= 0.24, (kind, material, const)
        # no second multiply on an untextured material
        assert v["diffuse_tint"] is None or \
            min(float(c) for c in v["diffuse_tint"]) >= 0.99, (kind, material)


def test_debris_material_never_textures_the_glass_bucket():
    """`kind`/`material` classifying to the `glass` bucket must keep the
    unconditional void look even if (mistakenly, or defensively tested
    here) a caller hands it a `tex_url` — glass is never façade cladding."""
    stage = _build_stage()
    ctx = _fresh_ctx(stage)
    mat = tu.debris_material(stage, ctx, "glass", "glass",
                             tex_url="airstack://x/T_should_be_ignored.jpg",
                             tex_name="T_should_be_ignored.jpg")
    assert mat.GetPrim().GetPath() == ctx["mats"]["void"].GetPrim().GetPath()
    assert not any(k.startswith("tornado_debris:src:") for k in ctx["mats"])


def test_build_debris_groups_by_kind_and_source_tex_name_not_material():
    """Two `panel`/`concrete_panel` fragments with DIFFERENT
    `source_tex_name`s must author TWO meshes (two different buildings'
    cladding), while two fragments sharing the SAME `source_tex_name` merge
    into ONE mesh even off different raw `material` hint strings — the
    texture, not the class label, decides the group once one exists."""
    stage = _build_stage()
    ctx = _fresh_ctx(stage)
    base = {"size": [0.4, 0.3, 0.15], "y": 1.0, "z": 0.0, "yaw_deg": 0.0,
            "tilt_deg": 0.0, "from": "src_group_test"}
    plan = {"schema": "tornado_urban_plan.v1", "level": "T4", "debris": [
        dict(base, x=1.0, kind="panel", material="concrete_panel",
             source_tex="airstack://x/T_stone_a.jpg", source_tex_name="T_stone_a.jpg"),
        dict(base, x=2.0, kind="panel", material="concrete_panel",
             source_tex="airstack://x/T_stone_a.jpg", source_tex_name="T_stone_a.jpg"),
        dict(base, x=3.0, kind="panel", material="brick",
             source_tex="airstack://y/T_stone_b.jpg", source_tex_name="T_stone_b.jpg"),
    ]}
    out = tu.apply_plan(stage, ctx, plan, verbose=False)
    assert out["n_debris_meshes"] == 2, out
    made_a = CELL + "/tornado_debris/panel_T_stone_a_jpg"
    made_b = CELL + "/tornado_debris/panel_T_stone_b_jpg"
    assert stage.GetPrimAtPath(made_a).IsValid()
    assert stage.GetPrimAtPath(made_b).IsValid()
    pts_a = UsdGeom.Mesh(stage.GetPrimAtPath(made_a)).GetPointsAttr().Get()
    assert len(pts_a) == 16  # 2 fragments x 8 points


def test_berm_and_ballistic_fragments_sharing_a_source_tex_bind_the_same_material():
    """END TO END: a berm fragment (`stacked=True`, `z_lift>0`) and a
    ballistic fragment (`z_lift=0`) off the SAME stamped piece — one
    building's own cladding texture — must land in the SAME debris mesh,
    bound to the SAME material, and that material's shader must actually
    carry the source texture on its `diffuse_texture` input (not merely
    the flat class-bucket look a texture-less plan would get)."""
    stage = _build_stage()
    ctx = _fresh_ctx(stage)
    tex_url = "airstack://scene_gen/assets/materials/megascans/Grey_Stone/T_greystone_1K_B.jpg"
    tex_name = "T_greystone_1K_B.jpg"
    base = {"kind": "block", "size": [0.4, 0.3, 0.2], "y": 1.0, "z": 0.0,
            "yaw_deg": 5.0, "tilt_deg": 0.0, "material": "brick",
            "from": "berm_src_test", "source_tex": tex_url,
            "source_tex_name": tex_name}
    plan = {"schema": "tornado_urban_plan.v1", "level": "T4", "debris": [
        dict(base, x=3.0, z_lift=0.6, stacked=True),   # berm
        dict(base, x=-3.0, z_lift=0.0, stacked=False)]}  # ballistic
    out = tu.apply_plan(stage, ctx, plan, verbose=False)
    assert out["n_debris_meshes"] == 1, out

    mesh_path = CELL + "/tornado_debris/block_T_greystone_1K_B_jpg"
    prim = stage.GetPrimAtPath(mesh_path)
    assert prim.IsValid()
    bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    assert bound.GetPrim().IsValid()
    sh = UsdShade.Shader.Get(stage, str(bound.GetPrim().GetPath()) + "/Shader")
    tex_input = sh.GetInput("diffuse_texture").Get()
    assert tex_input is not None and str(tex_input.path) == tex_url, tex_input

    src_keys = [k for k in ctx["mats"] if k.startswith("tornado_debris:src:")]
    assert len(src_keys) == 1, src_keys

    pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
    assert len(pts) == 16  # both fragments merged into one mesh


@pytest.mark.skipif(not os.path.exists(EXAMPLE_PATH),
                    reason="stream L has not landed tests/fixtures/"
                          "tornado_urban_plan_example.json yet")
def test_real_planner_example_applies_without_error():
    """L's own real example, once it exists. This can only check what is
    true of ANY plan (it applies, the debris count matches) — the prim
    paths in a real plan belong to a real sliced building this file does
    not have, so removed/glass/displaced counts are not asserted here."""
    plan = _load_plan(EXAMPLE_PATH)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))
    ctx = _fresh_ctx(stage)

    out = tu.apply_plan(stage, ctx, plan, verbose=False)
    assert out["n_fragments"] == len(plan.get("debris") or [])


# ---------------------------------------------------------------------------
# ROUND 3b (§8e) — stream FX1's `apply_plan`/tears/fit-interior region.
#
# `_plan_with_one_recipe` runs exactly ONE `tornado_urban` recipe at level
# "TEST" — `test_tornado_urban.py`'s own `_run_one_recipe` isolation,
# reproduced here (not imported: it is a five-line helper, and importing a
# sibling test module for one function is worse than the duplication) so a
# test can clear `_wants_fit_interior`'s threshold via a NAMED recipe
# (`chunk`, one of `tornado_urban_usd._FIT_INTERIOR_RECIPES`) WITHOUT going
# through a T3/T4 `plan_damage` call — level "TEST" never satisfies F2a's
# `plan.get("level") in ("T3", "T4")` gate, so `plan["tears"]` stays empty
# and this file's own "RUNS WITHOUT ISAAC... VTK is not [needed]" contract
# holds: `_author_tears`'s fracture path (`fire_collapse._tear_perimeter` ->
# `fracture.prim_to_mesh`, which needs `trimesh`) is never reached by
# anything in this file, F2b's fit-interior/backing tests included.
# ---------------------------------------------------------------------------
def _plan_with_one_recipe(info, elements, btype, recipe, kw, seed=5,
                          wind=None, height_class=None, intensity=0.85):
    import random as _random

    from disaster import tornado_urban as tup

    wind = wind if wind is not None else {
        "bearing_deg": 0.0, "speed_frac": 0.85, "cross_frac": -0.4,
        "over": False}
    if height_class is None:
        height_class = tup.height_class_for(info["H"])
    rng = _random.Random(seed)
    plan = {
        "schema": "tornado_urban_plan.v1", "level": "TEST", "btype": btype,
        "style": info.get("style"), "H": float(info.get("H") or 0.0),
        "height_class": height_class, "wind": dict(wind),
        "recipes": [[recipe, dict(kw)]] if recipe else [],
        "removed": [], "displaced": {}, "glass": [], "glass_bands": [],
        "macroblocks": [], "regions": [], "roof_props": "keep", "debris": [],
        "notes": [], "stats": {}, "panels": [], "roof_shed": False,
        "tears": [], "tear_scope": {}, "_removed_set": set(),
    }
    weights = tup.side_weights(info, wind, rng)
    plan["side_weights"] = {k: float(v) for k, v in weights.items()}
    pctx = tup._pctx(info, elements, btype, rng, plan, wind, weights,
                     height_class, intensity)
    if recipe:
        tup.RECIPES_T[recipe](pctx, **kw)
    return tup._finalise(pctx, plan, height_class, wind, intensity)


def _describe_and_author(seed=7, W=30.0, D=24.0, H=40.0, storeys=10,
                         btype="urm"):
    """A REAL `quake_flow.describe`-shaped `info` (from `test_quake_sliced.
    fake_sliced_building`, the same fixture `test_tornado_urban.py`'s own
    `_fixture` wraps) plus a matching stage: one authored box per
    placement's own `prim_path`, `quake_flow._box`'s own convention (the
    same pattern `test_tornado_kit.py`'s `test_apply_plan_on_stub_stage_
    removes_and_authors_debris` already uses for a KIT adapter's own
    prim_paths)."""
    from test_quake_sliced import fake_sliced_building

    pls, style, _grid = fake_sliced_building(seed=seed, W=W, D=D, H=H,
                                             storeys=storeys)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = btype

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    cell = "/World/cell"
    UsdGeom.Xform.Define(stage, Sdf.Path(cell))
    for e in info["elements"]:
        p = e["p"]
        path = p.get("prim_path")
        sx, sy, sz = p.get("_size") or (1.0, 1.0, 3.0)
        cz = float(p.get("z_m", e.get("z", 0.0))) + max(sz, 0.05) / 2.0
        qf._box(stage, path, float(p.get("x_m", e.get("x", 0.0))),
                float(p.get("y_m", e.get("y", 0.0))), cz,
                max(sx, 0.05), max(sy, 0.05), max(sz, 0.05), yaw_deg=0.0)
    return info, stage, cell


def test_fit_interior_authors_for_a_chunk_plan_and_not_for_an_untouched_one():
    """§8e F2b, the mission's own ask: "fit-out prims exist for a T4 plan
    and not for T1". Checked at recipe granularity (see this section's own
    header for why): `chunk` is a NAMED `_FIT_INTERIOR_RECIPES` member and
    removes enough of a corner to clear the threshold on its own; an
    UNTOUCHED plan (no recipe at all) removes nothing and must stay a
    no-op."""
    info, stage, cell = _describe_and_author(seed=7)
    plan_chunk = _plan_with_one_recipe(info, info["elements"], "urm",
                                       "chunk", {}, seed=11,
                                       intensity=0.85)
    assert plan_chunk["removed"], "expected chunk to remove something"
    ctx = {"stage": stage, "parent": cell, "tag": "fit_chunk", "mats": {},
          "static_extra": [], "loose": [], "authored": [], "info": info,
          "notes": []}
    out = tu.apply_plan(stage, ctx, plan_chunk, verbose=False)
    assert out["n_fit"] > 0, "expected fit-out prims for a chunk plan"
    assert out["n_backing"] > 0, "expected backing quads for a chunk plan"
    assert stage.GetPrimAtPath(cell + "/tornado_interior_backing").IsValid()

    info2, stage2, cell2 = _describe_and_author(seed=7)
    plan_none = _plan_with_one_recipe(info2, info2["elements"], "urm",
                                      None, {}, seed=11, intensity=0.85)
    assert not plan_none["removed"]
    ctx2 = {"stage": stage2, "parent": cell2, "tag": "fit_none", "mats": {},
           "static_extra": [], "loose": [], "authored": [], "info": info2,
           "notes": []}
    out2 = tu.apply_plan(stage2, ctx2, plan_none, verbose=False)
    assert out2["n_fit"] == 0 and out2["n_backing"] == 0, \
        "an untouched plan must never clear the fit-interior threshold"
    assert not stage2.GetPrimAtPath(cell2 + "/tornado_interior_backing").IsValid()


def test_backing_quads_sit_inside_the_wall_line():
    """Every authored backing box's world position is CLOSER to the mass
    centre than the true wall line it backs — "a backing quad set just
    inside each opened side's wall line" (plan brief, §8e F2b), not
    coplanar with (or outside) the missing cladding."""
    info, stage, cell = _describe_and_author(seed=7)
    plan = _plan_with_one_recipe(info, info["elements"], "urm", "chunk", {},
                                 seed=11, intensity=0.85)
    ctx = {"stage": stage, "parent": cell, "tag": "fit_geom", "mats": {},
          "static_extra": [], "loose": [], "authored": [], "info": info,
          "notes": []}
    out = tu.apply_plan(stage, ctx, plan, verbose=False)
    assert out["n_backing"] > 0

    m = info["masses"]["main"]
    W, D = m["W"], m["D"]
    root = stage.GetPrimAtPath(cell + "/tornado_interior_backing")
    assert root.IsValid()
    n_checked = 0
    for child in root.GetChildren():
        # `backing_<side>_<storey>` -- the SIDE decides which axis "inside
        # the wall line" is measured on: a rectangular footprint has no
        # single inside-radius, so a S/N quad's local |y| must be < D/2
        # and an E/W quad's local |x| must be < W/2 (the true wall line on
        # that axis), never a Euclidean distance from the mass centre
        # (which a quad placed near a corner can legitimately exceed on
        # the OTHER axis while still sitting inside its own wall line).
        side = child.GetName().split("_")[1]
        xf = UsdGeom.Xformable(child)
        mat = xf.ComputeLocalToWorldTransform(0)
        wx, wy = float(mat[3][0]), float(mat[3][1])
        lx, ly = qf._to_local(m, wx, wy)
        if side in ("S", "N"):
            assert abs(ly) < D / 2.0, (child.GetPath(), ly, D)
        else:
            assert abs(lx) < W / 2.0, (child.GetPath(), lx, W)
        n_checked += 1
    assert n_checked == out["n_backing"]


def test_wants_fit_interior_pure_logic():
    """`_wants_fit_interior` needs no stage at all — the AREA threshold and
    the two named recipes, checked directly."""
    assert not tu._wants_fit_interior({"stats": {}, "regions": []})
    assert not tu._wants_fit_interior(
        {"stats": {"removed_frac": tu._FIT_INTERIOR_FRAC - 0.01},
         "regions": []})
    assert tu._wants_fit_interior(
        {"stats": {"removed_frac": tu._FIT_INTERIOR_FRAC}, "regions": []})
    assert tu._wants_fit_interior(
        {"stats": {}, "regions": [{"recipe": "chunk"}]})
    assert tu._wants_fit_interior(
        {"stats": {}, "regions": [{"recipe": "facade_collapse"}]})
    assert not tu._wants_fit_interior(
        {"stats": {}, "regions": [{"recipe": "cladding_band"}]})


# ---------------------------------------------------------------------------
# ROUND 4 (D2) -- TEAR FACES WEAR THE BUILDING'S OWN MATERIAL. Before this,
# the real SM_Building_02 T4 probe bound 854 of 1210 fragments' CUT faces to
# one generic megascans brick (`QuakeLooks/c_brick`), 83 whole fragments to
# the same, and 310 more to a `clad_*` triplanar of the blind `WallBack` map
# or a poster atlas -- the user's "some other building's material on it".
# ---------------------------------------------------------------------------
def _tear_ctx(stage, parent="/W/cell"):
    UsdGeom.Xform.Define(stage, Sdf.Path(parent))
    return {"stage": stage, "parent": parent, "tag": "t", "mats": {}}


def test_tear_material_uses_the_pieces_own_texture_and_darkens_only_the_cut():
    stage = Usd.Stage.CreateInMemory()
    ctx = _tear_ctx(stage)
    url = "omniverse://host/Textures/M_Building_01_Concrete_02_BaseColor.png"
    face = tu._tear_material(stage, ctx, url, "conc.png", False)
    cut = tu._tear_material(stage, ctx, url, "conc.png", True)
    assert face.GetPrim().GetPath() != cut.GetPrim().GetPath()
    for mat, tint in ((face, tu._TEAR_FACE_TINT), (cut, tu._TEAR_CUT_TINT)):
        sh = UsdShade.Shader.Get(stage, str(mat.GetPrim().GetPath()) + "/Shader")
        assert sh, mat.GetPrim().GetPath()
        tex = sh.GetInput("diffuse_texture").Get()
        assert str(tex.path) == url, tex          # THE PIECE'S OWN MAP
        got = sh.GetInput("diffuse_tint").Get()
        assert [round(float(c), 4) for c in got] == [round(c, 4) for c in tint]
    # the cut face is darker than the wall face, ONCE
    assert tu._TEAR_CUT_TINT[0] < tu._TEAR_FACE_TINT[0]
    # cached: same texture + same face kind -> the same material prim
    assert tu._tear_material(stage, ctx, url, "conc.png", True) == cut


def test_tear_material_falls_back_to_neutral_plaster_never_generic_brick():
    stage = Usd.Stage.CreateInMemory()
    ctx = _tear_ctx(stage)
    for cut in (False, True):
        mat = tu._tear_material(stage, ctx, "", "", cut)
        sh = UsdShade.Shader.Get(stage, str(mat.GetPrim().GetPath()) + "/Shader")
        assert sh.GetInput("diffuse_texture") is None or \
            sh.GetInput("diffuse_texture").Get() is None
        # NO second multiply on an untextured branch (the round-3 near-black
        # berm bug), and a neutral -- never brick-coloured -- albedo
        assert sh.GetInput("diffuse_tint") is None or \
            sh.GetInput("diffuse_tint").Get() is None
        rgb = [float(c) for c in sh.GetInput("diffuse_color_constant").Get()]
        assert max(rgb) - min(rgb) < 0.06, rgb        # grey, not brick
        assert 0.25 < sum(rgb) / 3.0 < 0.6, rgb


def test_reface_rebinds_generic_looks_and_keeps_a_real_facade_skin():
    """The whole point of the split: a fragment `fire_collapse.
    skin_fragment` already gave the parent's own material and UVs is LEFT
    ALONE; only `quake_flow`'s generic looks (and an unbound fragment) are
    replaced."""
    stage = Usd.Stage.CreateInMemory()
    ctx = _tear_ctx(stage)
    parent = ctx["parent"]
    url = "omniverse://host/Textures/own_wall_BaseColor.png"
    # a generic `QuakeLooks` look, and a "real" source-asset material
    generic = damage._pbr(stage, parent + "/QuakeLooks/c_brick",
                          (0.4, 0.3, 0.3), 0.9)
    skinned = damage._pbr(stage, parent + "/src/asset/UnrealMaterial",
                          (0.5, 0.5, 0.5), 0.9)
    paths = []
    for name, mat in (("brk_t_wall_S_1_00_0001", generic),
                      ("brk_t_wall_S_2_00_0002", skinned),
                      ("brk_t_wall_S_3_00_0003", None)):
        grp = parent + "/" + name
        UsdGeom.Xform.Define(stage, Sdf.Path(grp))
        frag = grp + "/frag_000"
        UsdGeom.Mesh.Define(stage, Sdf.Path(frag))
        UsdGeom.Subset.Define(stage, Sdf.Path(frag + "/core"))
        if mat is not None:
            UsdShade.MaterialBindingAPI(
                stage.GetPrimAtPath(frag)).Bind(mat)
        paths.append(frag)
    by_frag = {"brk_t_wall_S_1_00_0001": (url, "own.png"),
               "brk_t_wall_S_2_00_0002": (url, "own.png"),
               "brk_t_wall_S_3_00_0003": ("", "")}
    stats = {"cut": 0, "face": 0, "kept_skin": 0, "no_core": 0,
             "no_tex": 0, "missing": 0}
    tu._reface_tear_fragments(stage, ctx, by_frag, paths, stats)

    assert stats == {"cut": 3, "face": 2, "kept_skin": 1, "no_core": 0,
                     "no_tex": 1, "missing": 0}, stats

    def bound(path):
        m = UsdShade.MaterialBindingAPI(
            stage.GetPrimAtPath(path)).ComputeBoundMaterial()[0]
        return str(m.GetPrim().GetPath()) if m and m.GetPrim().IsValid() else ""

    # every cut face is now under TornadoTearLooks -- no generic brick left
    for frag in paths:
        assert "/TornadoTearLooks/cut_" in bound(frag + "/core"), frag
        assert "QuakeLooks" not in bound(frag + "/core")
    # the generic-bound and the unbound fragment were refaced...
    assert "/TornadoTearLooks/face_" in bound(paths[0])
    assert "/TornadoTearLooks/face_" in bound(paths[2])
    # ...and the skinned one kept fire_collapse's own facade skin
    assert bound(paths[1]) == parent + "/src/asset/UnrealMaterial"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
