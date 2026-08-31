#!/usr/bin/env python3
"""fire_city_dressing_probe — the two 2026-08-31 city-assembly passes
(SCORCHED VEGETATION, FIRE-SIDE DEBRIS APRON) exercised on a FAKE 26-building
city, offline.

    docker exec isaac-sim bash -c \\
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
       /isaac-sim/AirStack/scene_gen/tools/fire_city_dressing_probe.py"

BARE USD. No `SimulationApp`, no Kit, no GPU, no real bake/manifest — this
builds its OWN small in-memory stage (26 fake damaged buildings on a grid,
each with a fake sidecar `doc["fire"]` block, plus a scatter of fake street
trees around them) and runs `fire_assembly_lib.scorch_vegetation_pass` /
`fire_apron_pass` on it exactly the way `urban_fire_city_launch_script.py`'s
`scorch_vegetation`/`fire_debris_apron` methods do on a real one. Safe beside
a running sim (memory: "Nucleus USD without Kit").

WHY A FAKE CITY, NOT A REAL BAKE: `test_fire_city_dressing.py` already pins
down the math and the binding behaviour on minimal synthetic fixtures; what
that file CANNOT show is the PRIM-COUNT SHAPE at the scale the brief actually
asks about — "this is a 26-building city, keep the prim count bounded...
report the count and estimated VRAM". 26 buildings with a realistic level
mix (5xF1, 5xF2, 4xF3, 4xF4, 3xF5, 2xF5c, 2xF6, 1xF0 — matching
`urban_fire.LEVELS`) and ~60 street trees scattered around them is the
smallest fixture that answers that question honestly.

WHAT THIS DOES NOT MEASURE: real VRAM (there is no renderer here — no Kit,
no RTX). The "estimated VRAM" figure is a documented back-of-envelope byte
count for the authored Mesh/Material data only (points + face topology +
one small shader graph per material), which is what a bare-USD probe CAN
honestly report; the true number also depends on Hydra's own per-prim
overhead and is what `fire_assembly_lib.vram_mb`'s `nvidia-smi` reading (the
launcher's own real measurement, gated by `FC_SCORCH_VEG`/`FC_FIRE_APRON` in
`urban_fire_city_launch_script.py`) is for.

Exit 0 clean, 1 on any failed assertion.
"""

import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import fire_assembly_lib as fal               # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade              # noqa: E402

# 26 buildings, the level mix `build-urban-fire-scenes` / `urban_fire.LEVELS`
# would plausibly produce over a manifest this size.
BUILDING_LEVELS = (
    ["F1"] * 5 + ["F2"] * 5 + ["F3"] * 4 + ["F4"] * 4 + ["F5"] * 3
    + ["F5c"] * 2 + ["F6"] * 2 + ["F0"] * 1
)
assert len(BUILDING_LEVELS) == 26

SIDES_BY_LEVEL = {
    "F0": (),                    # untouched — no fire block at all
    "F1": ("S",), "F2": ("S",), "F3": ("S", "E"), "F4": ("S", "E"),
    "F5": ("S", "E", "N"), "F5c": ("S",), "F6": ("S", "E"),
}
GRID_COLS = 6
GRID_SPACING_M = 45.0
BUILDING_W, BUILDING_D, BUILDING_H = 18.0, 12.0, 15.0
N_TREES = 60
TREE_SPACING_M = 15.0

_FAILS = []


def check(name, cond, detail=""):
    if cond:
        print("PASS  " + name)
    else:
        print("FAIL  " + name + (": " + detail if detail else ""))
        _FAILS.append(name)


def build_city(stage):
    """`(placed_rows, placements)` — a fake but structurally-real fixture:
    `placed_rows` is `FireCityApp.placed`'s own shape (`i`/`stem`/`x`/`y`/
    `yaw`/`bbox`/`doc`/`masses`); `placements` is `FireCityApp.placements`'s
    own shape (only the fields `fire.select_fuels` reads)."""
    placed_rows = []
    for i, level in enumerate(BUILDING_LEVELS):
        col, row = i % GRID_COLS, i // GRID_COLS
        x, y = col * GRID_SPACING_M, row * GRID_SPACING_M
        yaw = 90.0 if (i % 3 == 0) else 0.0     # a third of the grid rotated
        doc = ({"fire": {"level": level, "sides": list(SIDES_BY_LEVEL[level]),
                         "mass": "main"}} if level != "F0" else {})
        placed_rows.append({
            "i": i, "stem": "bldg{0:02d}_{1}".format(i, level),
            "x": x, "y": y, "yaw": yaw,
            "bbox": [x - BUILDING_W / 2.0, y - BUILDING_D / 2.0, 0.0,
                    x + BUILDING_W / 2.0, y + BUILDING_D / 2.0, BUILDING_H],
            "doc": doc,
            "masses": {"main": {"cx": 0.0, "cy": 0.0, "yaw": 0.0,
                                "W": BUILDING_W, "D": BUILDING_D, "z0": 0.0,
                                "top": BUILDING_H, "levels": [0.0],
                                "spec": {"bands": []}}},
        })

    # A shared green material every tree starts on, plus one bark material —
    # both classify via `vegetation._kind`'s MDL-name substring rule.
    leaf_mat = UsdShade.Material.Define(stage, "/World/Looks/leafGreen")
    sh = UsdShade.Shader.Define(stage, "/World/Looks/leafGreen/Shader")
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("Oak_Leaves.mdl"), "mdl")
    leaf_mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    bark_mat = UsdShade.Material.Define(stage, "/World/Looks/barkBrown")
    sh2 = UsdShade.Shader.Define(stage, "/World/Looks/barkBrown/Shader")
    sh2.CreateIdAttr("OmniPBR")
    sh2.SetSourceAsset(Sdf.AssetPath("Oak_Bark.mdl"), "mdl")
    bark_mat.CreateSurfaceOutput("mdl").ConnectToSource(sh2.ConnectableAPI(), "out")

    def make_quad(path, mat):
        me = UsdGeom.Mesh.Define(stage, path)
        me.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0),
                            Gf.Vec3f(1, 1, 0), Gf.Vec3f(0, 1, 0)])
        me.CreateFaceVertexCountsAttr([4])
        me.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        UsdShade.MaterialBindingAPI(me.GetPrim()).Bind(mat)
        return me

    # A street-tree grid straddling the buildings, at the SAME spacing a
    # real `city_detail.street_trees` pass would plausibly produce
    # (`config/asset_sets/urban.yaml`'s own `spacing_m: 22.0`, halved here
    # so a fixture this small still puts several trees within reach).
    n_cols_trees = int(GRID_COLS * GRID_SPACING_M / TREE_SPACING_M) + 2
    n_rows_trees = int((len(BUILDING_LEVELS) // GRID_COLS + 1)
                       * GRID_SPACING_M / TREE_SPACING_M) + 2
    placements = []
    rng = random.Random(5)
    made = 0
    for r in range(n_rows_trees):
        for c in range(n_cols_trees):
            if made >= N_TREES:
                break
            tx = c * TREE_SPACING_M - GRID_SPACING_M
            ty = r * TREE_SPACING_M - GRID_SPACING_M
            path = "/World/stage/generated/street_tree_0_{0}".format(made)
            root = UsdGeom.Xform.Define(stage, path)
            root.AddTranslateOp().Set(Gf.Vec3d(tx, ty, 0.0))
            make_quad(path + "/leafMesh", leaf_mat)
            make_quad(path + "/trunkMesh", bark_mat)
            placements.append({"category": "street_tree", "x_m": tx,
                               "y_m": ty, "prim_path": path})
            made += 1
        if made >= N_TREES:
            break
    return placed_rows, placements


def approx_mesh_bytes(n_points, n_faces):
    """A documented, ROUGH byte estimate for one authored Mesh's own
    points + face topology — 12 bytes/point (float3), 4 bytes per face
    vertex count, 4 bytes per face-vertex index (quads throughout here).
    Not a Hydra/GPU number (there is no renderer in this probe) — see the
    module docstring."""
    return 12 * n_points + 4 * n_faces + 4 * n_faces * 4


def main():
    stage = Usd.Stage.CreateInMemory()
    placed_rows, placements = build_city(stage)

    n_prims_before = sum(1 for _ in stage.Traverse())

    veg_stats = fal.scorch_vegetation_pass(stage, placements, placed_rows,
                                           "/World/fire")
    n_prims_after_veg = sum(1 for _ in stage.Traverse())

    apron_rows = fal.fire_apron_pass(stage, "/World/fire", placed_rows,
                                     seed=7)
    n_prims_after_apron = sum(1 for _ in stage.Traverse())

    print()
    print("--- SCORCHED VEGETATION -------------------------------------")
    print("  {0} street tree(s) placed, {1} fuel(s) matched by "
          "fire.select_fuels, {2} scorched (within reach of a damaged "
          "building), {3} untouched (stay green)".format(
              len(placements), veg_stats["fuels_total"], veg_stats["scorched"],
              veg_stats["fuels_total"] - veg_stats["scorched"]))
    print("  {0} tree(s) touched: {1} leaf bind(s), {2} trunk bind(s)".format(
          veg_stats["trees"], veg_stats["leaf_binds"],
          veg_stats["trunk_binds"]))
    veg_new_prims = n_prims_after_veg - n_prims_before
    print("  new/changed prims for this pass: {0} (2 shared VegLooks "
          "materials + shaders; no NEW geometry, only rebinds)".format(
              veg_new_prims))
    check("veg: scorched count is within [0, fuels_total]",
          0 <= veg_stats["scorched"] <= veg_stats["fuels_total"])
    check("veg: at least one tree scorched and at least one left green "
          "(the fixture is not degenerate)",
          0 < veg_stats["scorched"] < veg_stats["fuels_total"],
          str(veg_stats["scorched"]))
    # every scorched tree's building must be F2 or worse
    bad_level = [t["level"] for t in veg_stats["targets"].values()
                if t["level"] in ("F0", "F1")]
    check("veg: nothing scorched by an F0/F1 building", not bad_level,
          str(bad_level))

    print()
    print("--- FIRE-SIDE DEBRIS APRON -----------------------------------")
    n_meshes = sum(1 for r in apron_rows if r.get("prim"))
    n_lumps = sum(r.get("n", 0) for r in apron_rows)
    n_gated_out = len(apron_rows) - n_meshes
    apron_new_prims = n_prims_after_apron - n_prims_after_veg
    print("  {0}/{1} building(s) got an apron ({2} gated out: F0/F5c/F6 or "
          "no venting sides), {3} lump(s) total".format(
              n_meshes, len(apron_rows), n_gated_out, n_lumps))
    print("  new prims for this pass: {0} (1 Mesh + <=2 materialBind "
          "GeomSubsets per building, plus 2 shared DebrisLooks materials)"
          .format(apron_new_prims))
    expect_meshes = sum(1 for lv in BUILDING_LEVELS if lv in fal.APRON_LEVELS)
    check("apron: exactly the F1-F5 buildings got a mesh (F0/F5c/F6 did not)",
          n_meshes == expect_meshes,
          "{0} vs expected {1}".format(n_meshes, expect_meshes))
    # Per building: 1 Mesh + up to 2 `materialBind` GeomSubsets = 3. Plus a
    # FIXED, one-time overhead of 6 — the `<root>/apron` and
    # `<root>/DebrisLooks` scope prims (auto-created ancestors) and the two
    # shared debris materials (1 Material + 1 Shader prim each) — paid ONCE
    # for the whole city, not per building.
    apron_bound = 3 * n_meshes + 6
    check("apron: prim count is BOUNDED — linear in buildings, not in lumps",
          apron_new_prims <= apron_bound,
          "{0} new prim(s) for {1} lump(s) across {2} building(s) (bound "
          "{3})".format(apron_new_prims, n_lumps, n_meshes, apron_bound))

    total_bytes = 0
    for r in apron_rows:
        if not r.get("prim"):
            continue
        prim = stage.GetPrimAtPath(r["prim"])
        me = UsdGeom.Mesh(prim)
        n_pts = len(me.GetPointsAttr().Get() or [])
        n_faces = len(me.GetFaceVertexCountsAttr().Get() or [])
        total_bytes += approx_mesh_bytes(n_pts, n_faces)

    print()
    print("--- ESTIMATE (documented approximation, no renderer here) ----")
    print("  apron geometry: {0} lump(s) across {1} mesh(es), ~{2:.0f} KiB "
          "of raw point+topology data (12 B/point + 8 B/face-vertex)"
          .format(n_lumps, n_meshes, total_bytes / 1024.0))
    print("  prim total for BOTH passes on this 26-building city: "
          "{0} (veg) + {1} (apron) = {2}, against {3} prims already on "
          "the fixture's fake city before either pass ran".format(
              veg_new_prims, apron_new_prims,
              veg_new_prims + apron_new_prims, n_prims_before))

    print()
    if _FAILS:
        print("FIRE CITY DRESSING PROBE *** PROBLEM ***")
        for f in _FAILS:
            print("  - " + f)
        return 1
    print("FIRE CITY DRESSING PROBE OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
