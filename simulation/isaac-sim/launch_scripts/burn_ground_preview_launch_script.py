#!/usr/bin/env python
"""
Ground only — the burn scar on grass, with nothing else in the scene.

    SCENE_CONFIG=suburb_mini_wildfire \\
    ISAAC_SIM_SCRIPT_NAME=burn_ground_preview_launch_script.py \\
    airstack up isaac-sim

WHY A BENCH FOR THE FLOOR
-------------------------
The ground has been the hardest thing in this scene to get right and every
attempt has been judged inside a 250 x 250 m block that takes twenty minutes
to build and fills the frame with burnt houses, black trees and three thousand
pieces of debris. That is a terrible way to look at a surface: the loop is
slow, and half of what you are seeing is not the thing under test.

So this is the floor and only the floor. Same grass material the block uses,
same fire model, same overlay code — `suburb_mini_wildfire`'s own config, read
through `compile_disaster`, so the ellipse here is the ellipse there.

WHAT TO LOOK AT
---------------
Three separate questions, and they fail differently:

  DOES THE OPACITY SURVIVE AT ALL?  ANSWERED — it does, but ONLY with
  `/rtx/raytracing/fractionalCutoutOpacity` on, which this script now sets.
  OmniPBR turns `opacity_constant` into a FRACTIONAL cutout opacity
  (`OmniPBRBase.mdl`: `cutout_opacity = enable_opacity ? opacity_value : 1`)
  and RTX Real-Time discards fractional cutout unless that setting is on —
  so 0.48-0.87 bands drew NOTHING, with no error. `UsdPreviewSurface` fails
  differently: its `opacity` is a diffuse/transmission blend gated by a hidden
  `enable_specular_transmission=false`, so it comes out OPAQUE — that was the
  masked-overlay attempt. The six-square row stays as the regression check:
  with the setting on, every square draws.

  DOES IT REPEAT?  The burnt floor tiled at ~8 m read as a grid of small
  squares from altitude. The default is now ONE tile across the whole plate
  (`GROUND_TILE_M=0`): a 2K map over 250 m is 12 cm/px, soft up close and
  unrepeated from the air. `GROUND_TILE_M=60` is the compromise to try.

  DOES IT READ AS ONE SCAR?  Banding you can count is the failure mode to
  watch for — more bands (`GROUND_BANDS`, now 12) or smaller cells fix it.

  IS THE EDGE NATURAL?  The first coverage field STEPPED from 0 to 0.45 at
  the arrival line, so the scar ended on the fire model's own ellipse with a
  hard cut against green grass. Three things soften it, all in `coverage_at`:
    feather   coverage ramps over `GROUND_EDGE_M` metres BEHIND the front
              (measured along the ray from the origin — arrival is linear
              along a ray, so this is exact), wide at the fast head and
              narrow on the slow flanks, which is what a real edge does;
    fingers   the front line is wobbled by +-`GROUND_FINGER_M` of seamless
              band-limited noise (25-80 m wavelengths — house-lot scale, and
              nothing large enough to read as one shape), so the outline
              fingers instead of tracing a conic;
    islands   `GROUND_ISLANDS` of the burnt area is left green in compact
              8-25 m patches the fire skipped. Islands only REMOVE coverage.

  IS IT THE RIGHT SHAPE?  It follows `coverage_at`, the same field that sets
  every building's damage level, so the scar is the ellipse the front actually
  swept from the epicentre. A grid of yellow posts marks the epicentre and the
  heading so the shape can be checked against what the fire model claims.

GROUND_ELAPSED overrides how far through the burn the scene is. The default
used to run the front just past the far corner — "the widest the scar ever
gets" — which burns the ENTIRE plate and leaves no edge on screen to judge;
now it stops when GROUND_BURNT_FRAC (0.55) of the plate is burnt, so head,
flanks and back are all visible with grass around them. GROUND_TILE_M sets the overlay tile (0 = the plate), GROUND_OPACITY_MIN /
GROUND_OPACITY_MAX the opacity of the lightest and heaviest band. GROUND_EDGE_M,
GROUND_FINGER_M and GROUND_ISLANDS shape the edge (see above).
"""

import math
import os
import sys

import carb
import numpy as np
from isaacsim import SimulationApp

# FRACTIONAL CUTOUT OPACITY, OR THE OVERLAY DOES NOT DRAW. See the docstring:
# OmniPBR's `opacity_constant` is a fractional cutout, and RTX Real-Time
# treats any fractional cutout as fully transparent until this is on.
#
# IT HAS TO BE ON THE COMMAND LINE. `carb.settings.set_bool(...)` after the
# app is up does nothing: ~12 s into startup `omni.usd-abi` MAPS the carb
# setting onto a USD render-settings property (`omni:rtx:rt:fractionalOpacity`)
# and the renderer reads the property from then on, so a value set after the
# constructor returns is never copied across. The overlay vanished again the
# run that tried it. `extra_args` is exactly the `--/...` flag path.
simulation_app = SimulationApp(launch_config={
    "headless": False,
    # == disaster.ground.KIT_ARGS (not importable yet: sys.path is set below)
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                   "--/rtx/pathtracing/fractionalCutoutOpacity=true"],
})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from compile_disaster import load_scene_config                 # noqa: E402
from disaster import damage, fire, ground                      # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_mini_wildfire")
ELAPSED = float(os.environ.get("GROUND_ELAPSED", "0")) or None
# Cell, bands, tile, opacity and edge knobs are `ground.knobs_from_env` —
# the SAME variables with the same defaults as the scene, so what is tuned
# here carries over unchanged.
BURNT_FRAC = float(os.environ.get("GROUND_BURNT_FRAC", "0.55"))
# The overlay runs this far past the plate, over the bench's grass margin.
# The ignition point sits ON the plate corner, so an overlay clipped to the
# plate cut the scar square there — a hard edge the fire never made. Letting
# the field run out over the margin shows the ellipse's real back and flanks,
# feathered like everywhere else.
MARGIN_M = 60.0

GRASS_MAT = "airstack://scene_gen/assets/materials/Grass_Countryside.usda"
BURNT_PNG = ground.BURNT_TEXTURE
# Just off the grass. Nothing else is in this scene to z-fight with.
BURN_Z = 0.010


# Opacity configurations, all rendered side by side. WHY A ROW RATHER THAN A
# GUESS: the scar came back invisible, and "invisible" has several possible
# causes that a single rebuild cannot separate — OmniPBR multiplying by an
# unset `opacity_texture`, `enable_opacity` inverting, the mesh not drawing at
# all. Each of these is one variable different from its neighbour, so whichever
# ones appear identify the cause by elimination in ONE run.
#
#   plain      no opacity inputs at all. If THIS is invisible the fault is the
#              geometry, not the material, and nothing else here matters.
#   off        enable_opacity False, constant set. Should be opaque.
#   on_050     enable_opacity True, constant 0.5 — what the bands use.
#   on_100     enable_opacity True, constant 1.0. Separates "opacity is being
#              read and is zero" from "opacity is being read correctly".
#   on_mode1   as on_050 with opacity_mode 1, in case the mode selects a
#              channel of a texture that does not exist.
#   preview    UsdPreviewSurface at opacity 0.5, for the record — this is the
#              one already known to lose its opacity in translation.
OPACITY_TESTS = [
    ("plain",    dict(enable=None, const=None, mode=None)),
    ("off",      dict(enable=False, const=0.5, mode=0)),
    ("on_050",   dict(enable=True, const=0.5, mode=0)),
    ("on_100",   dict(enable=True, const=1.0, mode=0)),
    ("on_mode1", dict(enable=True, const=0.5, mode=1)),
    ("preview",  dict(preview=True, const=0.5)),
]


def _test_mat(stage, path, texture, spec):
    if spec.get("preview"):
        mat = UsdShade.Material.Define(stage, Sdf.Path(path))
        sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor",
                       Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.2, 0.17, 0.15))
        sh.CreateInput("opacity",
                       Sdf.ValueTypeNames.Float).Set(float(spec["const"]))
        sh.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.94)
        mat.CreateSurfaceOutput().ConnectToSource(
            sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
        return mat

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture",
                   Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture))
    sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("texture_scale",
                   Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(0.12, 0.12))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(0.94)
    if spec["enable"] is not None:
        sh.CreateInput("enable_opacity",
                       Sdf.ValueTypeNames.Bool).Set(bool(spec["enable"]))
    if spec["const"] is not None:
        sh.CreateInput("opacity_constant",
                       Sdf.ValueTypeNames.Float).Set(float(spec["const"]))
        sh.CreateInput("opacity_threshold",
                       Sdf.ValueTypeNames.Float).Set(0.0)
    if spec["mode"] is not None:
        sh.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(int(spec["mode"]))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def add_opacity_row(stage, ssf, y_m, size_m=26.0, gap_m=6.0):
    """One square per opacity configuration, in a row on the grass."""
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/opacityTests"))
    n = len(OPACITY_TESTS)
    x0 = -0.5 * (n * size_m + (n - 1) * gap_m)
    print("[ground] opacity test row at y={0:+.0f}, left to right:".format(y_m))
    for i, (name, spec) in enumerate(OPACITY_TESTS):
        x = x0 + i * (size_m + gap_m)
        mat = _test_mat(stage, "{0}/OpacityTests/{1}".format(PARENT, name),
                        sg._join_asset_root(BURNT_PNG, ""), spec)
        path = "/World/opacityTests/{0}".format(name)
        sg._make_plane_mesh(stage, path, x, y_m, x + size_m, y_m + size_m,
                            BURN_Z, 4.0, ssf, display_color=(0.2, 0.18, 0.15))
        UsdShade.MaterialBindingAPI(
            stage.GetPrimAtPath(path)).Bind(mat)
        print("[ground]   {0:>2d}. {1:<9s} x={2:+7.1f}  {3}".format(
            i + 1, name, x, spec))


def add_marker(stage, path, x, y, z, h, rgb):
    c = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    c.CreateSizeAttr(1.0)
    c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    xf = UsdGeom.Xformable(c)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z + h * 0.5))
    xf.AddScaleOp().Set(Gf.Vec3f(0.6, 0.6, h))


def main():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))
    _, ssf = get_stage_meters_per_unit(stage)

    config = load_scene_config(SCENE_CONFIG)
    region = config.get("layout", {}).get("region_m") or [250, 250]
    rw, rh = float(region[0]) * 0.5, float(region[1]) * 0.5

    # GRASS, from the block's own material entry, so this is the surface the
    # scar will actually be laid over rather than a stand-in for it.
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground"))
    gm = stage.DefinePrim(Sdf.Path("/World/ground/grassMat"))
    gm.GetReferences().AddReference(sg._join_asset_root(GRASS_MAT, ""))
    gm.Load()
    sg._make_plane_mesh(stage, "/World/ground/grass",
                        -rw - MARGIN_M, -rh - MARGIN_M, rw + MARGIN_M,
                        rh + MARGIN_M,
                        0.0, 3.0, ssf, display_color=(0.24, 0.36, 0.17),
                        mat_prim_path="/World/ground/grassMat")

    # THE SAME FIRE, read from the same config the block uses.
    fcfg = dict(fire.DEFAULTS)
    fcfg.update((config.get("disaster") or {}).get("fire") or {})
    ox, oy = fcfg["origin_m"]
    th = math.radians(float(fcfg["heading_deg"]))
    cos_t, sin_t = math.cos(th), math.sin(th)
    head, flank, back = (float(fcfg["head_mps"]), float(fcfg["flank_mps"]),
                         float(fcfg["back_mps"]))

    def arrival(x, y):
        dx, dy = x - ox, y - oy
        u = dx * cos_t + dy * sin_t
        v = -dx * sin_t + dy * cos_t
        return fire._ignition_time(u, v, head, flank, back)

    # STOP WHEN PART OF THE PLATE IS BURNT, NOT WHEN ALL OF IT IS. The first
    # rule ran the front past the far corner so the scar was "as wide as it
    # gets" — and that burns every cell of the plate, leaving no edge on
    # screen at all; what was being judged as "the ellipse" was the opacity
    # bands. The arrival-time quantile puts the whole outline on the plate.
    plate = (-rw, -rh, rw, rh)
    elapsed = ELAPSED or ground.elapsed_for_fraction(arrival, plate, BURNT_FRAC)

    knobs = ground.knobs_from_env(max(float(region[0]), float(region[1])))
    overlay = (-rw - MARGIN_M, -rh - MARGIN_M, rw + MARGIN_M, rh + MARGIN_M)
    print("[ground] edge: feather {0:.0f} m, fingers +-{1:.0f} m, islands "
          "{2:.0%} of the burnt area".format(knobs["edge_m"],
                                             knobs["finger_m"],
                                             knobs["islands"]))
    coverage_at = ground.feathered_coverage(
        arrival, elapsed, (ox, oy), overlay,
        np.random.default_rng(int(fcfg.get("seed", 0))),
        edge_m=knobs["edge_m"], finger_m=knobs["finger_m"],
        islands=knobs["islands"])
    made = ground.build_overlay(
        stage, coverage_at, overlay, ssf, BURN_Z, material_parent=PARENT,
        cell_m=knobs["cell_m"], bands=knobs["bands"], tile_m=knobs["tile_m"],
        op_range=knobs["op_range"])

    # The epicentre, and the heading, so the shape can be checked against what
    # the fire model claims rather than against an impression of it.
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/markers"))
    add_marker(stage, "/World/markers/epicentre", ox, oy, 0.0, 8.0,
               (0.95, 0.85, 0.15))
    for k in range(1, 6):
        d = k * max(rw, rh) * 0.32
        add_marker(stage, "/World/markers/heading_{0}".format(k),
                   ox + cos_t * d, oy + sin_t * d, 0.0, 3.0, (0.95, 0.5, 0.1))

    # THE DIAGNOSTIC ROW, on clean grass north of the scar so nothing overlaps
    # it. If every square is invisible the fault is not opacity at all.
    add_opacity_row(stage, ssf, rh + 26.0)

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(950.0)
    dome.CreateColorAttr(Gf.Vec3f(0.74, 0.78, 0.86))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(2400.0)
    key.CreateAngleAttr(0.9)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-46.0, 0.0, 32.0))

    cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/groundCam"))
    cam.CreateFocalLengthAttr(20.0)
    cam.AddTranslateOp().Set(Gf.Vec3d(0.0, -rh * 1.35, max(rw, rh) * 1.9))
    cam.AddRotateXYZOp().Set(Gf.Vec3f(52.0, 0.0, 0.0))
    try:
        import omni.kit.viewport.utility as vp
        vp.get_active_viewport().camera_path = "/World/groundCam"
    except Exception as exc:
        carb.log_warn("could not retarget the viewport: {0}".format(exc))

    for _ in range(30):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 72)
    print("BURN GROUND PREVIEW — floor only")
    print("  region      {0:.0f} x {1:.0f} m".format(region[0], region[1]))
    print("  epicentre   ({0:.0f}, {1:.0f})  heading {2:.0f} deg  "
          "(yellow post, orange trail)".format(ox, oy, fcfg["heading_deg"]))
    print("  elapsed     {0:.0f} s ({1})".format(
        elapsed, "GROUND_ELAPSED" if ELAPSED else
        "{0:.0%} of the plate burnt".format(BURNT_FRAC)))
    print("  overlay     {0} band(s) at {1:.1f} m cells, tile {2}, "
          "opacity {3:.2f}-{4:.2f}, {5:.0f} m past the plate".format(
              len(made), knobs["cell_m"],
              ("{0:.0f} m".format(knobs["tile_m"]) if knobs["tile_m"]
               else "one per overlay"),
              knobs["op_range"][0], knobs["op_range"][1], MARGIN_M))
    print("  OPACITY TEST ROW is north of the scar — six squares, one per")
    print("  material configuration. Which of them you can SEE identifies")
    print("  why the scar did not draw; the log above names them in order.")
    print("  edge        feather {0:.0f} m, fingers +-{1:.0f} m, islands "
          "{2:.0%}".format(knobs["edge_m"], knobs["finger_m"],
                           knobs["islands"]))
    print("  GROUND_BANDS / GROUND_CELL / GROUND_ELAPSED / GROUND_TILE_M /")
    print("  GROUND_OPACITY_MIN / GROUND_OPACITY_MAX / GROUND_EDGE_M /")
    print("  GROUND_FINGER_M / GROUND_ISLANDS / GROUND_BURNT_FRAC to re-run.")
    print("=" * 72 + "\n")

    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
