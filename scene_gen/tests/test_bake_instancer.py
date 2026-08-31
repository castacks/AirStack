#!/usr/bin/env python3
"""test_bake_instancer.py — `bake.export_object` carries PointInstancers.

Round-4 rubble (`_plans/earthquake_round4_plan.md`) authors rubble debris as
`UsdGeom.PointInstancer`s — a few prototypes referenced from Nucleus, hundreds
of instances — instead of one authored prim per piece. Before this change
`export_object` only ever looked at `UsdGeom.Mesh` prims, so an instancer
under an exported object silently vanished from the baked archetype (its
prototypes are typically invisible geometry, so a plate built from the bake
would render with the mound/large-element layers but none of the scattered
debris that makes a heap of rubble read as one).

WHAT THIS PINS DOWN, per prim requirement:

  1. An instancer's OWN schema attributes (protoIndices, positions,
     orientations, scales, ids, invisibleIds, extent) survive by value,
     unmodified by the merge's dead-attribute stripping.
  2. Its `prototypes` relationship is rewritten to the COPIED prototypes'
     OUTPUT paths — a referenced prototype (from "Nucleus") keeps its
     reference arc (never flattened by value) and an inline prototype (no
     reference) is copied by value like everything else in this file.
  3. The instancer's OWN transform is baked to a world matrix exactly like a
     Mesh's, and `positions`/`orientations`/`scales` are left UNCHANGED —
     they are in the instancer's own local frame both before and after
     export; only the wrapping xform moves.
  4. `BAKE_MERGE=on` never gathers an instancer or its prototypes into a
     merge bucket — they are authored directly, `merge="off"` produces the
     identical instancer result.
  5. `validate()` counts and prints instancers/instances without changing
     its `(meshes, ok, miss)` return shape (its one caller,
     `tools/_o_remerge.py`, unpacks it positionally).
  6. An object with NO instancer bakes to the exact same set of prim paths
     as the pre-change code (loaded from the reviewer's snapshot under a
     different module name, so both versions run side by side).

RUNS ONLY UNDER A REAL `pxr` — unlike most of this file, which is written so
it can be imported with no USD installed:

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        python tests/test_bake_instancer.py
"""

import importlib.util
import io
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

from disaster import bake                                       # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt              # noqa: E402

SNAPSHOT_PATH = ("/tmp/claude-1000/-home-krrishjain-SEI-COA-disaster-dataset/"
                 "c4902492-72e9-4d0b-aec1-362a8a34d2ff/scratchpad/snap/bake.py")
# The last commit BEFORE round 4 touched bake.py ("fixed baselines not getting
# env on osmo", 2026-08-30). The scratchpad copy above is a session-local
# convenience that a reboot wipes; the commit is the durable baseline.
BASELINE_COMMIT = "b9ac378f"
_REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "..", ".."))


def _baseline_source():
    if os.path.exists(SNAPSHOT_PATH):
        with open(SNAPSHOT_PATH) as fh:
            return fh.read()
    import subprocess
    return subprocess.check_output(
        ["git", "-C", _REPO, "show",
         BASELINE_COMMIT + ":scene_gen/disaster/bake.py"], text=True)


def _load_snapshot():
    """The pre-change `bake.py`, imported under its own module name so it
    runs side by side with the working copy (`disaster.bake`) with neither
    clobbering the other in `sys.modules`. Read from the scratchpad snapshot
    when it exists, else from `BASELINE_COMMIT` via `git show`."""
    import types
    src = _baseline_source()
    mod = types.ModuleType("bake_snapshot")
    mod.__file__ = SNAPSHOT_PATH
    exec(compile(src, SNAPSHOT_PATH, "exec"), mod.__dict__)
    return mod


def _cube_points():
    return Vt.Vec3fArray([
        Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, -0.5, -0.5),
        Gf.Vec3f(0.5, 0.5, -0.5), Gf.Vec3f(-0.5, 0.5, -0.5),
        Gf.Vec3f(-0.5, -0.5, 0.5), Gf.Vec3f(0.5, -0.5, 0.5),
        Gf.Vec3f(0.5, 0.5, 0.5), Gf.Vec3f(-0.5, 0.5, 0.5)])


_CUBE_FACE_COUNTS = [4, 4, 4, 4, 4, 4]
_CUBE_FACE_IDX = [0, 1, 2, 3,  4, 5, 6, 7,  0, 1, 5, 4,
                  1, 2, 6, 5,  2, 3, 7, 6,  3, 0, 4, 7]


def _author_cube_mesh(stage, path):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(_cube_points())
    m.CreateFaceVertexCountsAttr(Vt.IntArray(_CUBE_FACE_COUNTS))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(_CUBE_FACE_IDX))
    m.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    return m


def _make_proto_file(path, name):
    """One standalone kit-like asset file: `/root` (untyped) -> `/root/geo`
    (Mesh, 8-point cube), default prim `/root` — so referencing the file with
    NO explicit prim path pulls in `geo` as a child of the referencing prim,
    which is the shape a real Nucleus-referenced prop takes."""
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    _author_cube_mesh(st, "/root/geo")
    st.SetDefaultPrim(st.GetPrimAtPath("/root"))
    st.GetRootLayer().Save()
    return path


def _omni_pbr_like(stage, looks_path, name, color):
    """A minimal OmniPBR-shaped material: a Material prim with one Shader
    child carrying an `info:mdl:sourceAsset` and a diffuse-color input —
    enough to exercise `_rebuild_material` / `_mat_fingerprint` without
    needing the real OmniPBR.mdl on disk (`attr.Get()` never resolves it)."""
    mat = UsdShade.Material.Define(stage, looks_path + "/" + name)
    shader = UsdShade.Shader.Define(stage, looks_path + "/" + name + "/Shader")
    shader.CreateIdAttr("OmniPBR")
    shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f
                       ).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(
        shader.ConnectableAPI(), "out")
    return mat


def _build_source(src_path, proto_a_path, proto_b_path):
    st = Usd.Stage.CreateNew(src_path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Scope.Define(st, "/World/Looks")
    UsdGeom.Xform.Define(st, "/World")
    bld = UsdGeom.Xform.Define(st, "/World/bld")
    bld.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(40.0, 10.0, 0.0))

    # a plain mesh + OmniPBR-like material, so the merge path is exercised
    # alongside the instancers.
    box = _author_cube_mesh(st, "/World/bld/box")
    mat = _omni_pbr_like(st, "/World/Looks", "M_Box", (0.8, 0.2, 0.2))
    UsdShade.MaterialBindingAPI.Apply(box.GetPrim())
    UsdShade.MaterialBindingAPI(box.GetPrim()).Bind(mat)

    # --- instancer 1: two REFERENCED prototypes, 7 instances -------------
    inst = UsdGeom.PointInstancer.Define(st, "/World/bld/rubble_chunk")
    inst.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(5.0, 2.0, 0.0))         # non-zero local offset from `bld`
    UsdGeom.Scope.Define(st, "/World/bld/rubble_chunk/Prototypes")
    proto_a = st.DefinePrim("/World/bld/rubble_chunk/Prototypes/a", "Xform")
    proto_a.GetReferences().AddReference(proto_a_path)
    proto_b = st.DefinePrim("/World/bld/rubble_chunk/Prototypes/b", "Xform")
    proto_b.GetReferences().AddReference(proto_b_path)
    inst.CreatePrototypesRel().SetTargets([
        Sdf.Path("/World/bld/rubble_chunk/Prototypes/a"),
        Sdf.Path("/World/bld/rubble_chunk/Prototypes/b")])

    n = 7
    positions = Vt.Vec3fArray([Gf.Vec3f(float(i), float(i) * 0.5, 0.1 * i)
                               for i in range(n)])
    orientations = Vt.QuathArray([
        Gf.Quath(Gf.Rotation(Gf.Vec3d(0, 0, 1), 10.0 * i).GetQuat())
        for i in range(n)])
    scales = Vt.Vec3fArray([Gf.Vec3f(1.0 + 0.05 * i, 1.0, 1.0 - 0.02 * i)
                            for i in range(n)])
    proto_indices = Vt.IntArray([i % 2 for i in range(n)])
    ids = Vt.Int64Array([i for i in range(n)])
    velocities = Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, -0.1 * i) for i in range(n)])
    inst.CreatePositionsAttr(positions)
    inst.CreateOrientationsAttr(orientations)
    inst.CreateScalesAttr(scales)
    inst.CreateProtoIndicesAttr(proto_indices)
    inst.CreateIdsAttr(ids)
    inst.CreateVelocitiesAttr(velocities)
    inst.CreateInvisibleIdsAttr(Vt.Int64Array([2]))
    inst.CreateExtentAttr([Gf.Vec3f(0, 0, -1), Gf.Vec3f(6, 3, 1)])

    # --- instancer 2: ONE INLINE prototype (no reference), 3 instances ---
    inst2 = UsdGeom.PointInstancer.Define(st, "/World/bld/rubble_scatter")
    UsdGeom.Scope.Define(st, "/World/bld/rubble_scatter/Prototypes")
    _author_cube_mesh(st, "/World/bld/rubble_scatter/Prototypes/cube")
    inst2.CreatePrototypesRel().SetTargets(
        [Sdf.Path("/World/bld/rubble_scatter/Prototypes/cube")])
    n2 = 3
    inst2.CreatePositionsAttr(
        Vt.Vec3fArray([Gf.Vec3f(i, 0.0, 0.0) for i in range(n2)]))
    inst2.CreateOrientationsAttr(
        Vt.QuathArray([Gf.Quath(1, 0, 0, 0) for _ in range(n2)]))
    inst2.CreateScalesAttr(
        Vt.Vec3fArray([Gf.Vec3f(1, 1, 1) for _ in range(n2)]))
    inst2.CreateProtoIndicesAttr(Vt.IntArray([0, 0, 0]))

    st.SetDefaultPrim(st.GetPrimAtPath("/World"))
    st.GetRootLayer().Save()
    return src_path, dict(
        positions=positions, orientations=orientations, scales=scales,
        proto_indices=proto_indices, ids=ids, velocities=velocities,
        n=n, n2=n2)


def _find_mesh_with_points(prim, want_points):
    """Depth-first search under `prim` for a Mesh with exactly `want_points`
    authored points; returns the prim or None."""
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):
            pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
            if pts is not None and len(pts) == want_points:
                return p
    return None


def _build_no_instancer_source(src_path):
    """Same shape as `_build_source`'s plain-mesh half, with NO instancer at
    all — the regression fixture for requirement 6."""
    st = Usd.Stage.CreateNew(src_path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Scope.Define(st, "/World/Looks")
    UsdGeom.Xform.Define(st, "/World")
    bld = UsdGeom.Xform.Define(st, "/World/bld")
    bld.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(40.0, 10.0, 0.0))
    box = _author_cube_mesh(st, "/World/bld/box")
    box2 = _author_cube_mesh(st, "/World/bld/box2")
    mat = _omni_pbr_like(st, "/World/Looks", "M_Box", (0.8, 0.2, 0.2))
    UsdShade.MaterialBindingAPI.Apply(box.GetPrim())
    UsdShade.MaterialBindingAPI(box.GetPrim()).Bind(mat)
    UsdShade.MaterialBindingAPI.Apply(box2.GetPrim())
    UsdShade.MaterialBindingAPI(box2.GetPrim()).Bind(mat)
    st.SetDefaultPrim(st.GetPrimAtPath("/World"))
    st.GetRootLayer().Save()
    return src_path


# ---------------------------------------------------------------------------
# prototype MATERIAL BINDING (the round-4-follow-on bug: `_copy_prototype_tree`
# copied a prototype's ATTRIBUTES by value but silently dropped its
# `material:binding` RELATIONSHIP, so a PointInstancer prototype rendered
# with its raw referenced-asset material instead of the rubble emitter's
# per-look override).
# ---------------------------------------------------------------------------
_OVERRIDE_TEX = "textures/rubble_x.jpg"
_OVERRIDE_TINT = (0.42, 0.37, 0.33)


def _make_rubble_look_material(stage, mat_path, tex_path, tint):
    """The real shape `quake_rubble_usd`'s textured looks produce
    (`damage._pbr` + `_apply_diffuse_tint` + `_add_preview_fallback`): an
    OmniPBR-like MDL `Shader` carrying `diffuse_texture` (asset) and
    `diffuse_tint` (Color3f) inputs, PLUS a `UsdPreviewSurface` fallback
    network — a `DiffuseTex` (`UsdUVTexture`) shader fed by an `StReader`
    (`UsdPrimvarReader_float2`) — bound to the universal render context so
    the material is never invisible to a universal-context consumer."""
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset
                  ).Set(Sdf.AssetPath(tex_path))
    sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f
                  ).Set(Gf.Vec3f(*tint))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")

    reader = UsdShade.Shader.Define(stage, mat_path + "/StReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    tex = UsdShade.Shader.Define(stage, mat_path + "/DiffuseTex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex_path))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    prev = UsdShade.Shader.Define(stage, mat_path + "/PreviewSurface")
    prev.CreateIdAttr("UsdPreviewSurface")
    prev.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f
                     ).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(prev.ConnectableAPI(), "surface")
    return mat


def _build_prototype_material_source(src_path, proto_a_path, proto_b_path):
    """A `/World/bld` with THREE PointInstancers, one per material-binding
    requirement:

      `rubble_over`    — two REFERENCED prototypes (wrapper Xforms, exactly
                         `quake_rubble_usd._author_instancer`'s shape), BOTH
                         bound `strongerThanDescendants` to the SAME shared
                         override material — requirement (1): binding +
                         strength carried, tint/texture survive, ONE
                         exported material for both.
      `rubble_inline`  — one INLINE prototype (a wrapper Xform with no
                         reference of its own) whose CHILD Mesh carries its
                         OWN direct binding (default strength) to a second,
                         distinct material, and the wrapper itself carries
                         NONE — requirement (2).
      `rubble_plain`   — one INLINE prototype with NO binding anywhere —
                         requirement (3): must export with no binding.
    """
    st = Usd.Stage.CreateNew(src_path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/World")
    UsdGeom.Scope.Define(st, "/World/QuakeLooks")
    override_mat = _make_rubble_look_material(
        st, "/World/QuakeLooks/rubble_tex_x", _OVERRIDE_TEX, _OVERRIDE_TINT)
    child_mat = _omni_pbr_like(st, "/World/QuakeLooks", "M_Child",
                               (0.1, 0.6, 0.2))

    UsdGeom.Xform.Define(st, "/World/bld")

    # --- rubble_over: two referenced prototypes, ONE shared override -----
    inst = UsdGeom.PointInstancer.Define(st, "/World/bld/rubble_over")
    UsdGeom.Scope.Define(st, "/World/bld/rubble_over/Prototypes")
    p0 = st.DefinePrim("/World/bld/rubble_over/Prototypes/p0", "Xform")
    p0.GetReferences().AddReference(proto_a_path)
    UsdShade.MaterialBindingAPI.Apply(p0).Bind(
        override_mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    p1 = st.DefinePrim("/World/bld/rubble_over/Prototypes/p1", "Xform")
    p1.GetReferences().AddReference(proto_b_path)
    UsdShade.MaterialBindingAPI.Apply(p1).Bind(
        override_mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    inst.CreatePrototypesRel().SetTargets([
        Sdf.Path("/World/bld/rubble_over/Prototypes/p0"),
        Sdf.Path("/World/bld/rubble_over/Prototypes/p1")])
    n = 4
    inst.CreatePositionsAttr(
        Vt.Vec3fArray([Gf.Vec3f(float(i), 0.0, 0.0) for i in range(n)]))
    inst.CreateOrientationsAttr(
        Vt.QuathArray([Gf.Quath(1, 0, 0, 0) for _ in range(n)]))
    inst.CreateScalesAttr(Vt.Vec3fArray([Gf.Vec3f(1, 1, 1) for _ in range(n)]))
    inst.CreateProtoIndicesAttr(Vt.IntArray([0, 1, 0, 1]))

    # --- rubble_inline: inline wrapper, binding on the CHILD mesh --------
    inst2 = UsdGeom.PointInstancer.Define(st, "/World/bld/rubble_inline")
    UsdGeom.Scope.Define(st, "/World/bld/rubble_inline/Prototypes")
    st.DefinePrim("/World/bld/rubble_inline/Prototypes/w", "Xform")
    child_mesh = _author_cube_mesh(
        st, "/World/bld/rubble_inline/Prototypes/w/geo")
    UsdShade.MaterialBindingAPI.Apply(child_mesh.GetPrim()).Bind(child_mat)
    inst2.CreatePrototypesRel().SetTargets(
        [Sdf.Path("/World/bld/rubble_inline/Prototypes/w")])
    inst2.CreatePositionsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 5.0, 0.0)]))
    inst2.CreateOrientationsAttr(Vt.QuathArray([Gf.Quath(1, 0, 0, 0)]))
    inst2.CreateScalesAttr(Vt.Vec3fArray([Gf.Vec3f(1, 1, 1)]))
    inst2.CreateProtoIndicesAttr(Vt.IntArray([0]))

    # --- rubble_plain: inline prototype, NO binding at all ----------------
    inst3 = UsdGeom.PointInstancer.Define(st, "/World/bld/rubble_plain")
    UsdGeom.Scope.Define(st, "/World/bld/rubble_plain/Prototypes")
    _author_cube_mesh(st, "/World/bld/rubble_plain/Prototypes/cube")
    inst3.CreatePrototypesRel().SetTargets(
        [Sdf.Path("/World/bld/rubble_plain/Prototypes/cube")])
    inst3.CreatePositionsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 10.0, 0.0)]))
    inst3.CreateOrientationsAttr(Vt.QuathArray([Gf.Quath(1, 0, 0, 0)]))
    inst3.CreateScalesAttr(Vt.Vec3fArray([Gf.Vec3f(1, 1, 1)]))
    inst3.CreateProtoIndicesAttr(Vt.IntArray([0]))

    st.SetDefaultPrim(st.GetPrimAtPath("/World"))
    st.GetRootLayer().Save()
    return src_path


def _no_binding(prim):
    """True when `prim` carries no material binding of its own (direct)."""
    db = UsdShade.MaterialBindingAPI(prim).GetDirectBinding()
    mat = db.GetMaterial()
    return not (mat and mat.GetPrim().IsValid())


class TestBakeInstancerPrototypeMaterial(unittest.TestCase):
    """Prototype `material:binding` survives the export — see the module
    docstring's section above. A relationship is not an attribute, so
    `_copy_attrs_by_value` never touches it; without `_carry_direct_binding`
    (called from `_copy_prototype_tree`) every one of these silently
    exported with no binding at all, i.e. the raw referenced-asset look."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = self.tmp.name
        self.proto_a = _make_proto_file(
            os.path.join(self.dir, "proto_a.usda"), "a")
        self.proto_b = _make_proto_file(
            os.path.join(self.dir, "proto_b.usda"), "b")
        self.src_path = _build_prototype_material_source(
            os.path.join(self.dir, "src_mat.usda"), self.proto_a, self.proto_b)

    def tearDown(self):
        self.tmp.cleanup()

    def _export(self):
        src_stage = Usd.Stage.Open(self.src_path)
        out_path = os.path.join(self.dir, "out_mat.usd")
        ok = bake.export_object(src_stage, None, ["/World/bld"], out_path,
                                merge="off")
        self.assertTrue(ok)
        return Usd.Stage.Open(out_path)

    def test_referenced_prototype_override_carried_and_deduped(self):
        out = self._export()
        pi = UsdGeom.PointInstancer(out.GetPrimAtPath("/Baked/rubble_over"))
        targets = pi.GetPrototypesRel().GetTargets()
        self.assertEqual(len(targets), 2)

        mat_paths = []
        for t in targets:
            tp = out.GetPrimAtPath(t)
            self.assertTrue(tp.IsValid())
            api = UsdShade.MaterialBindingAPI(tp)
            db = api.GetDirectBinding()
            mat = db.GetMaterial()
            self.assertTrue(mat and mat.GetPrim().IsValid(),
                            "prototype {0} lost its material binding".format(t))
            mp = mat.GetPrim()
            # bound INSIDE the exported root, never back at the source path
            self.assertTrue(str(mp.GetPath()).startswith("/Baked/"))
            strength = UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(
                db.GetBindingRel())
            self.assertEqual(strength, UsdShade.Tokens.strongerThanDescendants)
            mat_paths.append(mp.GetPath())

        # --- ONE exported material for both prototypes (dedup) -----------
        self.assertEqual(mat_paths[0], mat_paths[1])

        # --- shader inputs survived: tint, texture path ------------------
        mat_prim = out.GetPrimAtPath(mat_paths[0])
        shader = UsdShade.Shader(mat_prim.GetChild("Shader"))
        self.assertTrue(shader)
        tint_val = shader.GetInput("diffuse_tint").Get()
        for got, want in zip(tint_val, _OVERRIDE_TINT):
            self.assertAlmostEqual(got, want, places=5)
        tex_val = shader.GetInput("diffuse_texture").Get()
        self.assertTrue(str(tex_val.path).endswith("rubble_x.jpg"),
                        "texture path lost: {0}".format(tex_val))

        # --- the UsdPreviewSurface fallback network came along too -------
        self.assertTrue(mat_prim.GetChild("PreviewSurface").IsValid())
        self.assertTrue(mat_prim.GetChild("DiffuseTex").IsValid())
        self.assertTrue(mat_prim.GetChild("StReader").IsValid())

    def test_inline_prototype_child_binding_kept(self):
        out = self._export()
        pi = UsdGeom.PointInstancer(out.GetPrimAtPath("/Baked/rubble_inline"))
        targets = pi.GetPrototypesRel().GetTargets()
        self.assertEqual(len(targets), 1)
        wrapper = out.GetPrimAtPath(targets[0])
        self.assertTrue(wrapper.IsValid())
        self.assertTrue(_no_binding(wrapper), "wrapper should carry no "
                        "binding of its own — only its child does")

        child = wrapper.GetChild("geo")
        self.assertTrue(child.IsValid())
        db = UsdShade.MaterialBindingAPI(child).GetDirectBinding()
        mat = db.GetMaterial()
        self.assertTrue(mat and mat.GetPrim().IsValid(),
                        "inline prototype child lost its own direct binding")
        self.assertTrue(str(mat.GetPrim().GetPath()).startswith("/Baked/"))
        strength = UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(
            db.GetBindingRel())
        self.assertEqual(strength, UsdShade.Tokens.weakerThanDescendants)

    def test_unbound_prototype_stays_unbound(self):
        out = self._export()
        pi = UsdGeom.PointInstancer(out.GetPrimAtPath("/Baked/rubble_plain"))
        targets = pi.GetPrototypesRel().GetTargets()
        self.assertEqual(len(targets), 1)
        tp = out.GetPrimAtPath(targets[0])
        self.assertTrue(tp.IsValid() and tp.IsA(UsdGeom.Mesh))
        self.assertTrue(_no_binding(tp))


class TestBakeInstancer(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = self.tmp.name
        self.proto_a = _make_proto_file(
            os.path.join(self.dir, "proto_a.usda"), "a")
        self.proto_b = _make_proto_file(
            os.path.join(self.dir, "proto_b.usda"), "b")
        self.src_path, self.known = _build_source(
            os.path.join(self.dir, "src.usda"), self.proto_a, self.proto_b)

    def tearDown(self):
        self.tmp.cleanup()

    def _export(self, merge, out_name):
        src_stage = Usd.Stage.Open(self.src_path)
        out_path = os.path.join(self.dir, out_name)
        stats = {}
        ok = bake.export_object(src_stage, None, ["/World/bld"], out_path,
                                recenter=(40.0, 10.0, 0.0), merge=merge,
                                stats_out=stats)
        self.assertTrue(ok)
        return out_path, stats

    def test_instancers_present_and_recentred(self):
        out_path, stats = self._export("on", "out_on.usd")
        out = Usd.Stage.Open(out_path)
        instancers = [p for p in Usd.PrimRange(out.GetPrimAtPath("/Baked"))
                     if p.IsA(UsdGeom.PointInstancer)]
        self.assertEqual(len(instancers), 2)
        self.assertEqual(stats["instancers"], 2)
        self.assertEqual(stats["instances"],
                         self.known["n"] + self.known["n2"])

        chunk = out.GetPrimAtPath("/Baked/rubble_chunk")
        self.assertTrue(chunk.IsValid())
        self.assertTrue(chunk.IsA(UsdGeom.PointInstancer))

        # --- world position after recentring: source minus (40, 10, 0) ---
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        world_t = xc.GetLocalToWorldTransform(chunk).ExtractTranslation()
        # source world position of the instancer = bld(40,10,0) + its own
        # local (5,2,0) offset = (45,12,0); minus recenter (40,10,0) = (5,2,0)
        self.assertAlmostEqual(world_t[0], 5.0, places=5)
        self.assertAlmostEqual(world_t[1], 2.0, places=5)
        self.assertAlmostEqual(world_t[2], 0.0, places=5)

        # --- instance arrays copied unchanged, in the LOCAL frame ---------
        pi = UsdGeom.PointInstancer(chunk)
        self.assertEqual(list(pi.GetProtoIndicesAttr().Get()),
                         list(self.known["proto_indices"]))
        self.assertEqual(list(pi.GetPositionsAttr().Get()),
                         list(self.known["positions"]))
        self.assertEqual(list(pi.GetOrientationsAttr().Get()),
                         list(self.known["orientations"]))
        self.assertEqual(list(pi.GetScalesAttr().Get()),
                         list(self.known["scales"]))
        self.assertEqual(list(pi.GetIdsAttr().Get()), list(self.known["ids"]))
        self.assertEqual(list(pi.GetVelocitiesAttr().Get()),
                         list(self.known["velocities"]))
        self.assertEqual(list(pi.GetInvisibleIdsAttr().Get()), [2])

        # --- prototypes rel -> copied prims, children of the instancer ---
        targets = pi.GetPrototypesRel().GetTargets()
        self.assertEqual(len(targets), 2)
        for t in targets:
            self.assertTrue(str(t).startswith(str(chunk.GetPath()) + "/"))
            tp = out.GetPrimAtPath(t)
            self.assertTrue(tp.IsValid())
            mesh = _find_mesh_with_points(tp, 8)
            self.assertIsNotNone(
                mesh, "referenced prototype {0} has no 8-point mesh".format(t))

        # --- second instancer: inline prototype, no reference ------------
        scatter = out.GetPrimAtPath("/Baked/rubble_scatter")
        self.assertTrue(scatter.IsValid() and scatter.IsA(UsdGeom.PointInstancer))
        pi2 = UsdGeom.PointInstancer(scatter)
        targets2 = pi2.GetPrototypesRel().GetTargets()
        self.assertEqual(len(targets2), 1)
        tp2 = out.GetPrimAtPath(targets2[0])
        self.assertTrue(tp2.IsValid())
        self.assertTrue(tp2.IsA(UsdGeom.Mesh))       # inline copy IS the mesh
        self.assertEqual(len(UsdGeom.Mesh(tp2).GetPointsAttr().Get()), 8)

        # --- the merged box mesh still exists -----------------------------
        merged = [p for p in Usd.PrimRange(out.GetPrimAtPath("/Baked"))
                 if p.IsA(UsdGeom.Mesh) and p.GetName().startswith("merged_")]
        self.assertTrue(merged, "no merged mesh found")

    def test_validate_reports_instancers(self):
        out_path, _ = self._export("on", "out_validate.usd")
        buf = io.StringIO()
        with redirect_stdout(buf):
            meshes, ok, miss = bake.validate(out_path)
        self.assertIsInstance(meshes, int)
        self.assertIsInstance(ok, int)
        self.assertIsInstance(miss, int)
        printed = buf.getvalue()
        self.assertIn("2", printed)
        self.assertIn(str(self.known["n"] + self.known["n2"]), printed)

    def test_merge_off_gives_the_same_instancer_result(self):
        out_on, _ = self._export("on", "out_on2.usd")
        out_off, _ = self._export("off", "out_off2.usd")
        st_on = Usd.Stage.Open(out_on)
        st_off = Usd.Stage.Open(out_off)
        for name in ("rubble_chunk", "rubble_scatter"):
            pon = UsdGeom.PointInstancer(
                st_on.GetPrimAtPath("/Baked/" + name))
            poff = UsdGeom.PointInstancer(
                st_off.GetPrimAtPath("/Baked/" + name))
            self.assertEqual(list(pon.GetProtoIndicesAttr().Get()),
                             list(poff.GetProtoIndicesAttr().Get()))
            self.assertEqual(list(pon.GetPositionsAttr().Get()),
                             list(poff.GetPositionsAttr().Get()))
            self.assertEqual(len(pon.GetPrototypesRel().GetTargets()),
                             len(poff.GetPrototypesRel().GetTargets()))
            for t in pon.GetPrototypesRel().GetTargets():
                self.assertTrue(st_on.GetPrimAtPath(t).IsValid())
            for t in poff.GetPrototypesRel().GetTargets():
                self.assertTrue(st_off.GetPrimAtPath(t).IsValid())

    def test_no_instancer_object_matches_snapshot_prim_set(self):
        no_inst_path = os.path.join(self.dir, "src_no_inst.usda")
        _build_no_instancer_source(no_inst_path)

        snap = _load_snapshot()

        out_new = os.path.join(self.dir, "out_new.usd")
        st1 = Usd.Stage.Open(no_inst_path)
        ok1 = bake.export_object(st1, None, ["/World/bld"], out_new,
                                 recenter=(40.0, 10.0, 0.0), merge="on")
        self.assertTrue(ok1)

        out_old = os.path.join(self.dir, "out_old.usd")
        st2 = Usd.Stage.Open(no_inst_path)
        ok2 = snap.export_object(st2, None, ["/World/bld"], out_old,
                                 recenter=(40.0, 10.0, 0.0), merge="on")
        self.assertTrue(ok2)

        st_new = Usd.Stage.Open(out_new)
        st_old = Usd.Stage.Open(out_old)
        paths_new = {str(p.GetPath())
                    for p in Usd.PrimRange(st_new.GetPseudoRoot())}
        paths_old = {str(p.GetPath())
                    for p in Usd.PrimRange(st_old.GetPseudoRoot())}
        self.assertEqual(paths_new, paths_old)


if __name__ == "__main__":
    unittest.main(verbosity=2)
