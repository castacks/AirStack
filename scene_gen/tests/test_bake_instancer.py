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


def _load_snapshot():
    """The pre-change `bake.py`, imported under its own module name so it
    runs side by side with the working copy (`disaster.bake`) with neither
    clobbering the other in `sys.modules`."""
    spec = importlib.util.spec_from_file_location("bake_snapshot",
                                                   SNAPSHOT_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
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
