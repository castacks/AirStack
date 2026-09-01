#!/usr/bin/env python
"""raft_material_bench.py — isolates `washaway.build_rafts`'s FIXED raft
material, away from the full hurricane suburb, so a render can confirm the
flat-pastel defect (Job 2, DEBRIS density/material review, 2026-08-31) is
actually gone before spending a render window on the full 500 m plate.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/raft_material_bench \
    ISAAC_SIM_SCRIPT_NAME=raft_material_bench.py \
    airstack up isaac-sim

    (or, once the container is already up:
     docker exec isaac-sim /isaac-sim/python.sh \
         scene_gen/tools/raft_material_bench.py)

`SNAP_DIR=` MUST sit under the mounted log directory (the same rule
`suburb_hurricane_launch_script.py`'s own snapshot block documents) or the
PNGs land somewhere the host cannot read them.

WHY THIS EXISTS. `test_washaway_debris.py`'s
`test_g_raft_material_fallback_is_not_neutral` pins the FIX (`diffuse_
color_constant` no longer left at the neutral `(1, 1, 1)` `wood_material`
was called with before this fix) at the USD-attribute level, offline, with
no Kit at all. It cannot say whether the fix actually reads as textured wood
grain at RENDER time — only Kit/RTX can answer that, and this exact material
graph was found "correct" at the attribute level once already (the tint-
per-channel-math fix) while the shipped render still looked flat, pastel,
untextured. This bench is the fast, cheap way to close that loop: ~30 boxes,
no city, no fracture, no water sim, no suburb — the render this needs takes
well under a minute, not the 15+ minutes a full suburb build costs.

WHAT IT BUILDS. A 20x20 m ground plane, a flat "water" quad a few
centimetres above it (a plain, textureless OmniPBR stand-in — this bench is
about the RAFT material, not about `surge.py`'s own translucent water look,
which this file never touches or imports), and 30 rafts — 3 of EVERY kind in
`washaway._RAFT_DIMS` — laid out on a simple grid, built through the SAME
`washaway.build_rafts` the real suburb scene calls. Not a reimplementation:
a fix that lands in `washaway.py` is the fix this bench renders, and the fix
this bench renders is the fix the suburb renders.

NOT RUN BY THIS AGENT — needs a live Isaac Sim / RTX renderer, which the
offline verification (`test_g_raft_material_fallback_is_not_neutral`) does
not have. For the next render window.
"""

import os
import random
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

import omni.kit.app                                             # noqa: E402
import omni.usd                                                 # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade              # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..",
    "simulation", "isaac-sim"))
_SCENE_GEN_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_SCENE_GEN_DIR)   # tools/ -> scene_gen/
sys.path.insert(0, _SCENE_GEN_DIR)

from disaster import washaway as wash                            # noqa: E402

PARENT = "/World/bench"
SNAP_DIR = os.path.expanduser(os.environ.get(
    "SNAP_DIR", "~/hurricane_previews/offline/raft_material_bench"))

GROUND_HALF_M = 10.0     # a 20 x 20 m ground/water pad
# A "just wet" sheet -- inside `washaway.DEFAULTS["raft_min_depth_m"]`'s own
# band, matching what a real raft actually floats in (not a dry bench).
WATER_LEVEL_M = 0.05
N_PER_KIND = 3           # 3 x 10 kinds = 30 pieces total
GRID_COLS = 6
GRID_SPACING_M = 2.6     # clears the largest kind (a 4.5 m vegetation mat's
                          # own half-length) against its 3-per-kind neighbours


def _flat_material(stage, path, tint, roughness=0.7):
    """A bare, textureless OmniPBR — the ground/water STAND-INS only.
    Deliberately NOT `washaway.build_rafts`'s material: that one is the
    thing under test, and must not be shadowed by a second copy of this
    function's own logic."""
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_color_constant",
                  Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    sh.CreateInput("reflection_roughness_constant",
                  Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def _quad(stage, path, x0, y0, x1, y1, z, material):
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr([Gf.Vec3f(x0, y0, z), Gf.Vec3f(x1, y0, z),
                       Gf.Vec3f(x1, y1, z), Gf.Vec3f(x0, y1, z)])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    m.CreateExtentAttr([Gf.Vec3f(x0, y0, z), Gf.Vec3f(x1, y1, z)])
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material)
    return m


def build(stage):
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))
    ground_mat = _flat_material(stage, PARENT + "/GroundMat",
                                (0.30, 0.28, 0.22))
    water_mat = _flat_material(stage, PARENT + "/WaterMat",
                               (0.18, 0.27, 0.31), roughness=0.15)
    g = GROUND_HALF_M
    _quad(stage, PARENT + "/ground", -g, -g, g, g, 0.0, ground_mat)
    _quad(stage, PARENT + "/water", -g, -g, g, g, WATER_LEVEL_M, water_mat)

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(1000.0)
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.82, 0.88))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(0.8)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 0.0, 30.0))

    # 3 of every kind, on a plain grid -- `_one_raft` for the real
    # draft/attitude/tint arithmetic, `build_rafts` for the SHADER under
    # test (the exact function/call path the suburb scene uses).
    kn = wash.resolve_cfg({"water_level_m": WATER_LEVEL_M})
    kinds = sorted(wash._RAFT_DIMS)
    n_total = len(kinds) * N_PER_KIND
    n_rows = -(-n_total // GRID_COLS)   # ceil
    x0 = -(GRID_COLS - 1) * GRID_SPACING_M / 2.0
    y0 = -(n_rows - 1) * GRID_SPACING_M / 2.0
    rng = random.Random(7)
    specs = []
    i = 0
    for kind in kinds:
        for _ in range(N_PER_KIND):
            cx = x0 + (i % GRID_COLS) * GRID_SPACING_M
            cy = y0 + (i // GRID_COLS) * GRID_SPACING_M
            specs.append(wash._one_raft(cx, cy, rng, kn, weights={kind: 1.0}))
            i += 1
    wash.build_rafts(stage, PARENT + "/rafts", specs, ssf=1.0)
    print("[raft_material_bench] built {0} rafts across {1} kind(s)"
         .format(len(specs), len(kinds)))
    return len(specs), n_rows


def _load_snapshots_rp():
    """The PROVEN pattern `suburb_hurricane_launch_script.py`'s own snapshot
    block uses (dynamic file load, not a plain `import`) — `snapshots_rp.py`
    lives outside any package `sys.path` puts on the normal import graph."""
    import importlib.util as ilu
    path = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots_rp.py")
    spec = ilu.spec_from_file_location("snapshots_rp", path)
    mod = ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())

    n, n_rows = build(stage)
    app = omni.kit.app.get_app()
    for _ in range(10):
        app.update()

    os.makedirs(SNAP_DIR, exist_ok=True)
    srp = _load_snapshots_rp()
    # ONE oblique shot, framed to hold the whole grid -- the raft field's
    # own footprint is roughly `GRID_COLS x n_rows` cells of `GRID_SPACING_M`
    # each, so the fit scales with however many kinds `_RAFT_DIMS` carries
    # rather than a number hand-picked for today's 10.
    span_x = GRID_COLS * GRID_SPACING_M
    span_y = n_rows * GRID_SPACING_M
    fit = max(span_x, span_y) * 1.15
    srp.place_camera(stage, (-fit * 0.55, -fit * 0.85, fit * 0.55),
                     (0.0, 0.0, WATER_LEVEL_M), focal_mm=28.0)
    ok = srp.snapshot(stage, os.path.join(SNAP_DIR, "raft_bench_oblique.png"))
    print("[raft_material_bench] {0} rafts, snapshot {1} -> {2}".format(
        n, "OK" if ok else "FAILED", SNAP_DIR))


if __name__ == "__main__":
    main()
    simulation_app.close()
