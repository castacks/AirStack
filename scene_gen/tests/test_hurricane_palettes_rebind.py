#!/usr/bin/env python3
"""test_hurricane_palettes_rebind.py — on a REFERENCED archetype, does two
different palettes actually bind two different materials?

    python3 scene_gen/tests/test_hurricane_palettes_rebind.py
    pytest -q scene_gen/tests/test_hurricane_palettes_rebind.py

WHY THIS EXISTS
---------------
`suburb_hurricane_launch_script.py` references one BAKED archetype USD many
times across the plate (`_ref(..., instance=True)`) — the whole reason a
1 km neighbourhood can be assembled at all rather than fractured live per
house. An instanced reference SHARES its prototype: rebinding a material
inside one instance rebinds it inside every instance of that prototype, and
USD refuses to author a new bind on a descendant of an instanceable prim in
the first place. So the only way for two houses referencing the SAME
archetype to wear two different palettes is to de-instance the ones that need
it and rebind materials on that one copy alone — the mechanism
`suburb_tornado_launch_script.py` already uses for row homes
(`instance=not _recolour`, then `mh.apply_palette` on the referenced prim)
and this hurricane launcher now uses for any detached house whose own draw
differs from the archetype's baked-in default (see the house loop's
`_recolour` computation).

This file does not build a real archetype (that needs the modular kit's
Nucleus assets, which this offline check does not have) — it builds the
SMALLEST possible stand-in with the two things `modular_house.apply_palette`
actually reads: a Mesh with wall/roof GeomSubsets bound to materials whose
shader carries a base-colour-looking texture path, which is how `apply_
palette.current()` identifies "this subset is currently wearing the kit's
default wall/roof" without knowing anything about the mesh. Saved to a real
file so `AddReference` has something resolvable with no Nucleus connection.

WHAT IT PROVES
--------------
  1. Two Xform prims referencing the SAME synthetic archetype, each
     `SetInstanceable(False)` and each given a DIFFERENT `palette` via
     `mh.apply_palette`, end up with their wall subset bound to two
     DIFFERENT material prims (and the roof subset likewise, for a palette
     pair whose roofs differ too) — the launcher's claim, measured rather
     than trusted.
  2. The SAME two references, left `SetInstanceable(True)`, do NOT diverge:
     `apply_palette`'s `Usd.PrimRange` does not descend into an instance's
     children at all, so it rebinds nothing on either one — which is
     *why* `_recolour` has to flip `instance` off, not a decorative safety
     check.

WHAT IT CANNOT SEE: the real archetype library's actual subset names on the
real kit meshes (`_WALL_SURFACES`/`_GABLE_SURFACES`/`_ROOF_PITCHED` are
exercised here with hand-built stand-ins, not the shipped `Outer_Wall_*`/
`Roof_*` USDs), and nothing about how this reads in a render. It is a check
on the REBIND MECHANISM only.
"""

import os
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SCENE_GEN)

from pxr import Sdf, Usd, UsdGeom, UsdShade                    # noqa: E402

from detail import modular_house as mh                         # noqa: E402


def _make_fake_archetype(path, wall_tex, roof_tex):
    """A one-mesh stand-in with a wall and a roof GeomSubset, each bound to a
    material whose shader carries an asset-valued "base colour" texture —
    the ONLY thing `apply_palette.current()` reads to identify a surface.
    Saved as a real `.usda` file so a later `AddReference` resolves offline.
    """
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Root")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/Root/Shell")
    # A trivial single quad — geometry is not what this test is about, but a
    # GeomSubset needs SOME face count to index into.
    mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])

    def _bind_subset(name, indices, tex_name):
        subset = UsdGeom.Subset.Define(stage, "/Root/Shell/" + name)
        subset.CreateElementTypeAttr("face")
        subset.CreateIndicesAttr(indices)
        mat = UsdShade.Material.Define(stage, "/Root/Looks/" + name + "Mat")
        shader = UsdShade.Shader.Define(stage, "/Root/Looks/" + name + "Mat/Tex")
        shader.CreateIdAttr("UsdPreviewSurface")
        # The base-colour hint lives in the FILENAME
        # (`_BASECOLOR_HINTS`/`current()`'s literal "basecolor" check), not in
        # the input's name, and the file need not exist on disk — `current()`
        # only ever inspects the `Sdf.AssetPath` string.
        shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(tex_name))
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(mat)
        return subset

    _bind_subset("wall", [0], wall_tex)
    _bind_subset("roof", [0], roof_tex)
    stage.GetRootLayer().Save()


def _bound_material_path(stage, prim_path):
    """The RAW `material:binding` relationship target, not `ComputeBound
    Material()`. The rebound materials point at real kit assets on Nucleus
    (`palette_material` -> `_usd(name)`), which this offline test cannot
    resolve — the referenced prim comes back with no composed `typeName` at
    all, and `ComputeBoundMaterial` requires the target to `IsA<Material>`,
    so it reports "no bound material" even though the relationship itself
    was authored correctly. The raw target path is exactly what the
    binding mechanism actually wrote and is unaffected by whether Nucleus
    is reachable, which is the property this offline test needs."""
    prim = stage.GetPrimAtPath(prim_path)
    assert prim.IsValid(), prim_path
    targets = UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel().GetTargets()
    return str(targets[0]) if targets else None


def _ref(stage, dst, usd, instance):
    prim = stage.DefinePrim(dst, "Xform")
    assert prim.GetReferences().AddReference(usd)
    if instance:
        prim.SetInstanceable(True)
    return prim


def test_two_recoloured_instances_bind_different_wall_and_roof_materials():
    with tempfile.TemporaryDirectory() as td:
        arch = os.path.join(td, "house_fake_pristine.usda")
        # The kit's OWN raw defaults, exactly as `apply_palette`'s
        # `_WALL_SURFACES`/`_ROOF_PITCHED` name them — this is what an
        # un-palette-stamped bake (the bug this whole stream fixes) leaves
        # every archetype wearing.
        _make_fake_archetype(arch, "Cladding_01_BaseColor.png",
                             "Roof_Tiles_01_Inst_BaseColor.png")

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        _ref(stage, "/World/h1", arch, instance=False)
        _ref(stage, "/World/h2", arch, instance=False)

        # BEFORE the rebind: both wear the archetype's own raw default, and
        # they wear the SAME one — the exact symptom this stream measured on
        # the real render ("uniform white/cream walls... repeated mossy
        # roofs" on every style, because no palette was ever stamped on the
        # placements the bake handed to `apply_palette`). Compared by LEAF
        # NAME, not full path: each reference composes the source layer's
        # material into its OWN namespace (`/World/h1/Shell/Looks/wallMat`
        # vs `/World/h2/Shell/Looks/wallMat`), which is ordinary reference
        # composition and not itself evidence of two different materials.
        w1_before = _bound_material_path(stage, "/World/h1/Shell/wall")
        w2_before = _bound_material_path(stage, "/World/h2/Shell/wall")
        assert w1_before is not None and w2_before is not None
        assert w1_before.rsplit("/", 1)[-1] == w2_before.rsplit("/", 1)[-1]

        n = mh.apply_palette(
            stage,
            [{"prim_path": "/World/h1", "palette": "wood_dark",
              "category": "house"},
             {"prim_path": "/World/h2", "palette": "stucco",
              "category": "house"}],
            parent_path="/World")
        # 2 houses x (wall + roof) = 4 subsets rebound.
        assert n == 4, n

        w1 = _bound_material_path(stage, "/World/h1/Shell/wall")
        w2 = _bound_material_path(stage, "/World/h2/Shell/wall")
        r1 = _bound_material_path(stage, "/World/h1/Shell/roof")
        r2 = _bound_material_path(stage, "/World/h2/Shell/roof")

        assert w1 is not None and w2 is not None
        assert w1 != w2, "both houses bound the same wall material: " + w1
        assert r1 is not None and r2 is not None
        assert r1 != r2, "both houses bound the same roof material: " + r1

        # AND THEY ARE THE RIGHT MATERIALS, not merely "different from each
        # other" — `PALETTES["wood_dark"]["wall"] == "Wood_01"`,
        # `PALETTES["stucco"]["wall"] == "Stucco_01_Inst"`, both looked up
        # under the `parent_path` this call was given.
        assert w1.endswith("/Looks/Wood_01")
        assert w2.endswith("/Looks/Stucco_01_Inst")
        # `wood_dark`'s pitched roof is `shingle_brown`; `stucco`'s PALETTES
        # entry sets `"roof": ROOF_DECK` ("Concrete_02") explicitly, so the
        # two also diverge on the roof despite both being looked up through
        # the pitched-roof branch (this synthetic mesh's roof subset is
        # bound to `Roof_Tiles_01_Inst`, which is in `_ROOF_PITCHED`).
        assert r1.endswith("/Looks/shingle_brown")
        assert r2.endswith("/Looks/Concrete_02")


def test_instanced_references_are_not_rebound_at_all():
    """The reason `_recolour` has to flip `instance` off. Left instanceable,
    `Usd.PrimRange` never descends past the instance root, so `apply_palette`
    finds no Mesh to rebind on either copy and both keep the archetype's raw
    default — proving the mechanism, not merely asserting it."""
    with tempfile.TemporaryDirectory() as td:
        arch = os.path.join(td, "house_fake_pristine.usda")
        _make_fake_archetype(arch, "Cladding_01_BaseColor.png",
                             "Roof_Tiles_01_Inst_BaseColor.png")

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        _ref(stage, "/World/h1", arch, instance=True)
        _ref(stage, "/World/h2", arch, instance=True)

        n = mh.apply_palette(
            stage,
            [{"prim_path": "/World/h1", "palette": "wood_dark",
              "category": "house"},
             {"prim_path": "/World/h2", "palette": "stucco",
              "category": "house"}],
            parent_path="/World")
        assert n == 0, (
            "expected an instanced reference to rebind nothing (proving why "
            "the launcher must de-instance a recoloured house), got {0}"
            .format(n))


# REMOVED 2026-08-31: this test pinned the per-house-palette launcher behaviour
# that the user's tornado-parity directive reverted (houses now recolour row-only,
# exactly like suburb_tornado_launch_script). The apply_palette mechanism tests
# below still stand — row homes still use it.
