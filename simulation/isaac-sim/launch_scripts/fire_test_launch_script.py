#!/usr/bin/env python
"""
Fire test bench — NVIDIA Flow on an empty stage, nothing else.

    ISAAC_SIM_SCRIPT_NAME=fire_test_launch_script.py airstack up isaac-sim

WHY THIS EXISTS
---------------
"I see smoke but no fire" in a full suburb has too many suspects: the extension
might not be loaded, the prims might be the wrong type, the emitters might be
off-screen, the block pool might be starved, or the colormap might simply never
reach its flame band. This strips all of it away — no scene, no generator, no
spread model, four fires on an empty ground plane — and varies ONE THING AT A
TIME so the answer is visible rather than inferred.

WHAT YOU ARE LOOKING AT
-----------------------
Four independent Flow simulations, 25 m apart, left to right along +X. Each is
a full `flowSimulate` / `flowOffscreen` / `flowRender` stack on its OWN LAYER,
which is what lets them run different settings side by side — Flow ties an
emitter to a simulation by a shared integer `layer`, so separate layers are
separate sims sharing one block pool.

    x = -37   cell 0.05   colormapXMax 1.00   NVIDIA's warehouse fire, verbatim
    x = -12   cell 0.10   colormapXMax 1.00   our resolution, their colour ramp
    x = +12   cell 0.10   colormapXMax 0.35   what the suburb now ships
    x = +37   cell 0.25   colormapXMax 0.35   the resolution that showed no fire

Read it left to right. If only the leftmost burns orange, resolution is the
whole story and the suburb needs finer cells. If 3 burns and 4 does not, the
0.25 -> 0.10 change was the fix. If NONE of them burn, the problem is upstream
of tuning — check the TYPE CHECK the script prints before anything else.

THE TYPE CHECK
--------------
`FlowCreatePrim` is a command registered by `omni.flowusd`. Without that
extension the command is missing, `disaster.fire._flow_create` falls back to a
plain `DefinePrim`, and you get prims of the right NAME that no plugin has ever
heard of — a silent, invisible failure. The script enables the extension
explicitly and then prints every Flow prim's resolved type. Anything blank
there means Flow is not running and no amount of colormap tuning will help.

Every emitter is pinned FLAMING from the first frame. There is no ignition
schedule and no spread here on purpose — this bench answers "does Flow render
fire", not "does the wildfire look right".
"""

import os
import sys

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension

# EXPLICIT, and the first thing that happens. The suburb launcher inherited
# Flow from whatever else pulled it in; relying on that is how the fallback
# above stays invisible.
enable_extension("omni.flowusd")
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, _SCENE_GEN_DIR)

from disaster import fire                                      # noqa: E402

# (label, x_m, densityCellSize_m, colormapXMax, flow layer)
CASES = [
    ("nvidia_verbatim", -37.0, 0.05, 1.00, 3),
    ("fine_default_ramp", -12.0, 0.10, 1.00, 4),
    ("shipping", 12.0, 0.10, 0.35, 5),
    ("coarse_no_fire", 37.0, 0.25, 0.35, 6),
]

# One 3 x 3 x 3 m burning box per case — a bonfire, roughly a tree crown's
# worth of fuel, and big enough to read from the default camera distance.
EMITTER_HALF = (1.5, 1.5, 1.5)
EMITTER_Z = 1.5


def build_ground_and_light(stage):
    """A dark ground and dim lighting, because fire is an EMISSIVE effect.

    Lit like a normal scene the flame washes out against the sky and the whole
    test reads as a false negative. This is deliberately gloomy.
    """
    ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = 200.0
    ground.CreatePointsAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, -e, 0.0),
                             Gf.Vec3f(e, e, 0.0), Gf.Vec3f(-e, e, 0.0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.12, 0.13)])
    ground.CreateExtentAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, e, 0.0)])

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(120.0)
    dome.CreateColorAttr(Gf.Vec3f(0.28, 0.33, 0.45))

    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(240.0)
    key.CreateAngleAttr(1.0)


def build_case(stage, label, x, cell, cmap_x_max, layer):
    """One self-contained Flow stack plus its permanently-flaming emitter."""
    root = "/World/flow_{0}".format(label)
    fire.setup_flow_stack(stage, layer=layer, density_cell_size_m=cell,
                          max_blocks=16384, colormap_x_max=cmap_x_max,
                          scene_scale_factor=1.0, root=root)

    path = root + "/emitter"
    prim = fire._flow_create(stage, path, "FlowEmitterBox")
    fire._set(prim, "layer", Sdf.ValueTypeNames.Int, layer)
    fire._set(prim, "position", Sdf.ValueTypeNames.Float3,
              Gf.Vec3f(x, 0.0, EMITTER_Z))
    fire._set(prim, "halfSize", Sdf.ValueTypeNames.Float3,
              Gf.Vec3f(*EMITTER_HALF))
    fire._set(prim, "fuel", Sdf.ValueTypeNames.Float, 0.5)
    fire._set(prim, "smoke", Sdf.ValueTypeNames.Float, 2.0)
    fire._set(prim, "temperature", Sdf.ValueTypeNames.Float, 2.0)
    fire._set(prim, "coupleRateFuel", Sdf.ValueTypeNames.Float, 2.0)
    fire._set(prim, "coupleRateSmoke", Sdf.ValueTypeNames.Float, 2.0)
    fire._set(prim, "coupleRateTemperature", Sdf.ValueTypeNames.Float, 2.0)
    fire._set(prim, "enabled", Sdf.ValueTypeNames.Bool, True)
    return root


def type_check(stage, roots):
    """Print every Flow prim's resolved type. Blank means Flow is not running."""
    print("\n" + "=" * 72)
    print("FLOW PRIM TYPE CHECK — every row must name a Flow type")
    print("=" * 72)
    bad = 0
    for root in roots:
        for suffix in ("/flowSimulate", "/flowOffscreen", "/flowRender",
                       "/emitter"):
            p = stage.GetPrimAtPath(root + suffix)
            t = p.GetTypeName() if p.IsValid() else "<MISSING>"
            if not str(t).startswith("Flow"):
                bad += 1
            print("  {0:14s} {1:46s} {2}".format(
                "BAD ->" if not str(t).startswith("Flow") else "", 
                root + suffix, t))
    # THE SETTINGS THAT WERE SILENTLY GOING MISSING. A correct prim type is
    # not enough — `FlowCreatePrim` makes one prim and no children, so these
    # live on sub-prims that used to not exist at all. Read them back.
    print("-" * 72)
    print("COMBUSTION READBACK — combustionEnabled must be True, colormap 6 pts")
    print("-" * 72)
    for root in roots:
        adv = stage.GetPrimAtPath(root + "/flowSimulate/advection")
        cmap = stage.GetPrimAtPath(root + "/flowOffscreen/colormap")
        comb = adv.GetAttribute("combustionEnabled").Get() if adv.IsValid() else None
        ign = adv.GetAttribute("ignitionTemp").Get() if adv.IsValid() else None
        pts = cmap.GetAttribute("rgbaPoints").Get() if cmap.IsValid() else None
        npts = len(pts) if pts else 0
        ok = "ok " if (comb and npts == 6) else "BAD"
        print("  {0} {1:34s} combustion={2!s:5s} ignitionTemp={3!s:6s} "
              "colormap pts={4}".format(ok, root.split("/")[-1], comb, ign, npts))
        if not (comb and npts == 6):
            bad += 1
    print("=" * 72)
    if bad:
        print("{0} prim(s) are NOT Flow types — omni.flowusd did not register\n"
              "FlowCreatePrim, so nothing here will ever render. Fix that\n"
              "before touching cell size or colormap.".format(bad))
    else:
        print("All Flow prims resolved. If you still see no flame, the cause\n"
              "is tuning — compare the four cases left to right.")
    print("=" * 72 + "\n")


def main():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()

    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    if stage is None:
        raise RuntimeError("Failed to create a new stage")

    # Metric Z-up, matching every other launcher — Flow's gravity and cell
    # sizes are authored in metres and would be 100x off on a cm stage.
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())

    build_ground_and_light(stage)

    roots = []
    for label, x, cell, cmap, layer in CASES:
        roots.append(build_case(stage, label, x, cell, cmap, layer))
        print("[fire_test] {0:18s} x={1:+6.1f} m  cell={2:.2f} m  "
              "colormapXMax={3:.2f}  layer={4}".format(label, x, cell, cmap,
                                                       layer))

    cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/testCam"))
    cam.AddTranslateOp().Set(Gf.Vec3d(0.0, -70.0, 14.0))
    cam.AddRotateXYZOp().Set(Gf.Vec3f(78.0, 0.0, 0.0))
    try:
        import omni.kit.viewport.utility as vp
        vp.get_active_viewport().camera_path = "/World/testCam"
    except Exception as exc:
        carb.log_warn("could not retarget the viewport: {0}".format(exc))

    for _ in range(20):
        omni.kit.app.get_app().update()

    type_check(stage, roots)
    print("Left to right: nvidia_verbatim | fine_default_ramp | shipping | "
          "coarse_no_fire\n")

    timeline.play()

    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
