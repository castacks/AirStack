#!/usr/bin/env python
"""aec_material_probe_launch_script.py — a THREE-WAY diagnostic for the AEC
brownstone material/soot/slice question, with no bake in the loop.

    ISAAC_SIM_HEADLESS=false KEEP_OPEN=1 \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/aec_probe \
    /isaac-sim/python.sh .../aec_material_probe_launch_script.py \
        --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts

WHY THIS EXISTS (user, 2026-09-02): *"you don't need to slice to verify
materials and overlays. You could have just spawned it and edited the mdl."*
Correct — four-minute bake cycles were being spent to answer a question about
a material. This script spawns, side by side:

  A  RAW  (x = 0)      the shipped asset, referenced directly, no damage at
                       all. This is the control: if THIS renders white, the
                       problem is MDL resolution in our Kit setup and has
                       nothing to do with the fire pipeline.
  B  OVERLAY (x = +60) a baked build that still carries the `mdlsootovl_*`
                       soot decals, so the decal's contribution is visible
                       against A.
  C  EXPLODED (y = -70) the CURRENT sliced bake with every piece pushed
                       radially out from the building centre by
                       `AEC_EXPLODE` (default 1.45), so the slice itself is
                       legible — which cuts landed where, what each piece
                       kept, and whether a piece is mis-materialled.

Env: AEC_RAW / AEC_OVERLAY / AEC_SLICED override the three inputs;
AEC_EXPLODE the separation factor; SNAP_DIR to capture.
"""
import os
import sys

from isaacsim import SimulationApp  # noqa: E402

HEADLESS = os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower() in (
    "1", "true", "yes")
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]
simulation_app = SimulationApp(launch_config={"headless": HEADLESS,
                                              "extra_args": KIT_ARGS})

import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "utils")))
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

RAW = os.environ.get(
    "AEC_RAW",
    "/isaac-sim/AirStack/scene_gen/assets/aec/brownstone/Assets/"
    "Create_Brownstone02/Reference_Brownstone5Row.usd")
OVERLAY = os.environ.get(
    "AEC_OVERLAY",
    "/isaac-sim/.cache/fire_bakes/aec_stale3/"
    "aec_Reference_Brownstone5Row_F3_s7.usd")
SLICED = os.environ.get(
    "AEC_SLICED",
    "/isaac-sim/.cache/fire_bakes/aec_Reference_Brownstone5Row_F3_s7.usd")
EXPLODE = float(os.environ.get("AEC_EXPLODE", "1.45"))
SNAP_DIR = os.environ.get("SNAP_DIR", "")
KEEP_OPEN = os.environ.get("KEEP_OPEN", "0").strip() == "1"

# AEC assets are authored in CENTIMETRES; the probe stage is metres.
RAW_SCALE = float(os.environ.get("AEC_RAW_SCALE", "0.01"))


def _auto_scale(usd):
    """`metersPerUnit(asset) / metersPerUnit(stage=1.0)`.

    The AEC assets are CENTIMETRE files; the baked ones are metres. Scaling
    only some columns is how this probe once put a 21 m building next to a
    2.1 KM one and reported the small one as "really small" — auto-detect
    removes the whole class of error.
    """
    try:
        s = Usd.Stage.Open(usd)
        if s:
            mpu = float(UsdGeom.GetStageMetersPerUnit(s))
            if 1e-6 < mpu <= 1e3:
                return mpu
    except Exception:
        pass
    return 1.0


def _ref(stage, path, usd, x, y, scale=1.0):
    prim = stage.DefinePrim(Sdf.Path(path), "Xform")
    if not prim.GetReferences().AddReference(usd):
        print("[aec] *** FAILED to reference {0}".format(usd))
        return None
    prim.Load()
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), 0.0))
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    return prim


def _explode(stage, root_path, factor):
    """Push every sliced piece radially out from the building centre."""
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return 0
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    whole = bc.ComputeWorldBound(root).ComputeAlignedRange()
    if whole.IsEmpty():
        return 0
    c = whole.GetMidpoint()
    moved = 0
    for prim in Usd.PrimRange(root):
        nm = prim.GetName()
        if not (nm.startswith("wall_") or nm.startswith("corner_")
                or nm.startswith("pier_") or nm.startswith("core_")
                or nm.startswith("parapet_") or nm.startswith("roof")):
            continue
        if not prim.IsA(UsdGeom.Xformable):
            continue
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        p = r.GetMidpoint()
        d = Gf.Vec3d(p[0] - c[0], p[1] - c[1], p[2] - c[2]) * (factor - 1.0)
        xf = UsdGeom.Xformable(prim)
        try:
            xf.AddTranslateOp(opSuffix="explode").Set(d)
            moved += 1
        except Exception:
            pass
    return moved



def _part_census(stage, root_path):
    """What is addressable on this asset, per ROW-HOUSE UNIT.

    `Geometry/<Category>/<Type>/<mesh>` under each
    `Reference_Brownstone02_0N` — the damage vocabulary, already authored:
    Windows and Doors to blow out, Lighting/EmissivePlane to kill, Roofs to
    hole, Structural/Walls to soot, Floors/Ceilings/Stairs already present so
    nothing needs a fabricated interior.
    """
    import collections
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return
    per_unit = collections.defaultdict(collections.Counter)
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        parts = str(prim.GetPath()).split("/")
        unit = next((q for q in parts if q.startswith("Reference_Brownstone02_")),
                    "?")
        cat = "?"
        if "Geometry" in parts:
            i = parts.index("Geometry")
            if i + 1 < len(parts):
                cat = parts[i + 1]
        per_unit[unit][cat] += 1
    print("[aec] ---- addressable parts, per row-house unit ----")
    for unit in sorted(per_unit):
        top = ", ".join("{0}={1}".format(k, v)
                        for k, v in per_unit[unit].most_common(7))
        print("[aec]   {0}: {1}".format(unit, top))


def main():
    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(1200.0)
    dome.CreateColorAttr(Gf.Vec3f(0.55, 0.72, 0.93))
    sun = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    sun.CreateIntensityAttr(3500.0)
    sun.CreateAngleAttr(0.8)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(-55.0, 0.0, 135.0))
    ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    s = 400.0
    ground.CreatePointsAttr([(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateDisplayColorAttr([(0.32, 0.32, 0.33)])

    print("\n[aec] A SOOTED   {0}".format(RAW))
    a = _ref(stage, "/World/A_sooted", RAW, 0.0, 0.0, _auto_scale(RAW))

    # SOOT, LIVE, IN THE MATERIAL — no export, no decal, no slice.
    # An earlier attempt wrote a converted copy with
    # `GetRootLayer().Export()`; the geometry of this asset lives in
    # REFERENCED layers, so that file carried the rebuilt materials and ZERO
    # meshes and rendered blank (measured 2026-09-02: 1535 -> 0 meshes).
    # Editing the composed stage in place avoids the whole question.
    if a is not None and float(os.environ.get("AEC_SOOT", "0")) > 0.0:
        sys.path.insert(0, "/isaac-sim/AirStack/scene_gen/disaster")
        sys.path.insert(0, "/isaac-sim/AirStack/scene_gen/tools")
        import aec_soot
        aec_soot.soot_building(
            stage, "/World/A_sooted",
            origin_frac=float(os.environ.get("AEC_ORIGIN_FRAC", "0.25")),
            strength=float(os.environ.get("AEC_SOOT", "0.85")))
        for _ in range(20):
            omni.kit.app.get_app().update()

    print("[aec] B CLEAN    {0}".format(OVERLAY))
    _ref(stage, "/World/B_clean", OVERLAY, 60.0, 0.0,
         _auto_scale(OVERLAY))
    # C (the sliced bake) and the mdlsootovl decal column are RETIRED:
    # slicing rebuilt geometry and lost the materials, and the decals read as
    # rectangles stuck on the wall. The asset is part-addressable, so damage
    # is done by addressing its OWN prims — nothing is cut and nothing floats.

    for _ in range(60):
        omni.kit.app.get_app().update()
    _part_census(stage, "/World/A_sooted")

    # what actually composed, per column
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    for nm in ("A_sooted", "B_clean"):
        p = stage.GetPrimAtPath("/World/{0}".format(nm))
        r = bc.ComputeWorldBound(p).ComputeAlignedRange() if p else None
        if r is None or r.IsEmpty():
            print("[aec] {0}: EMPTY (did not compose)".format(nm))
        else:
            mn, mx = r.GetMin(), r.GetMax()
            print("[aec] {0}: {1:.1f} x {2:.1f} x {3:.1f} m".format(
                nm, mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]))

    if SNAP_DIR:
        try:
            import importlib.util as ilu
            sp = os.path.join("/isaac-sim/AirStack/simulation/isaac-sim",
                              "utils", "snapshots.py")
            spec = ilu.spec_from_file_location("snapshots", sp)
            snaps = ilu.module_from_spec(spec)
            spec.loader.exec_module(snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            snaps.place_camera(stage, (30.0, -150.0, 55.0), (25.0, -20.0, 8.0))
            snaps.snapshot(os.path.join(SNAP_DIR, "aec_three_way.png"))
            snaps.views_around(stage, {"A_sooted": (0.0, 0.0)}, SNAP_DIR, 1.0,
                               top_h=60.0, obl_dist=45.0, obl_h=18.0,
                               azimuth_deg=225.0, aim_h=7.0)
            snaps.views_around(stage, {"B_clean": (60.0, 0.0)}, SNAP_DIR,
                               1.0, top_h=60.0, obl_dist=45.0, obl_h=18.0,
                               azimuth_deg=225.0, aim_h=7.0)
            print("[aec] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            print("[aec] snapshots FAILED: {0}".format(exc))

    print("AEC MATERIAL PROBE DONE")
    if KEEP_OPEN:
        while simulation_app.is_running():
            simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
