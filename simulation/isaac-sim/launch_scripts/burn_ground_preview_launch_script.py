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

  DOES THE OPACITY SURVIVE AT ALL?  If the scar renders as flat opaque ash,
  OmniPBR's `opacity_constant` is being dropped the way `UsdPreviewSurface`'s
  opacity texture was, and this whole approach is dead in this renderer. That
  is the first thing to check and it needs no judgement: either grass shows
  through the outer bands or it does not.

  DOES IT READ AS ONE SCAR?  The overlay is eight uniform bands. Banding that
  you can count is the failure mode to watch for — the fix is more bands, or
  jittering each band's cells between two neighbouring opacities.

  IS IT THE RIGHT SHAPE?  It follows `coverage_at`, the same field that sets
  every building's damage level, so the scar is the ellipse the front actually
  swept from the epicentre. A grid of yellow posts marks the epicentre and the
  heading so the shape can be checked against what the fire model claims.

GROUND_ELAPSED overrides how far through the burn the scene is; the default
runs the front just past the far corner, which is the widest the scar ever
gets.
"""

import math
import os
import sys

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

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
from disaster import damage, fire                              # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_mini_wildfire")
ELAPSED = float(os.environ.get("GROUND_ELAPSED", "0")) or None
CELL_M = float(os.environ.get("GROUND_CELL", "4.0"))
BANDS = int(os.environ.get("GROUND_BANDS", "8"))

GRASS_MAT = "airstack://scene_gen/assets/materials/Grass_Countryside.usda"
BURNT_PNG = ("airstack://scene_gen/assets/materials/megascans/"
             "Burnt_Forest_Floor/T_uhwpehcdy_2K_B.png")
# Just off the grass. Nothing else is in this scene to z-fight with.
BURN_Z = 0.010


def burn_overlay_mat(stage, path, texture, opacity, scale_uv=(0.12, 0.12)):
    """A semi-transparent burnt-ground OmniPBR. Opacity is a CONSTANT.

    WHY A CONSTANT AND NOT A MASK. Putting the falloff in an opacity TEXTURE
    forces `UsdPreviewSurface`, because OmniPBR carries ONE `texture_scale`
    for every texture it samples and so cannot tile a diffuse while stretching
    a mask once across the plate. And a UsdPreviewSurface loses its opacity in
    this renderer's USD-to-MDL translation — it draws fully opaque, which is
    what turned an earlier version of this into a flat ash floor.

    A per-band constant needs no mask, so the material can be an OmniPBR,
    which is MDL natively and has nothing to lose in translation. The gradient
    comes from the geometry being split into bands instead.
    """
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture",
                   Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture))
    sh.CreateInput("diffuse_color_constant",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
    sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("texture_scale",
                   Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*scale_uv))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(0.94)
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("opacity_constant",
                   Sdf.ValueTypeNames.Float).Set(float(opacity))
    # 0 blends; anything above turns a soft edge into a stippled cutout.
    sh.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def build_burn_ground(stage, coverage_at, region, ssf, cell_m=4.0, bands=8):
    """The scar, as bands of a translucent overlay over the grass.

    ONE MESH PER BAND, not one prim per cell: the region is diced into
    `cell_m` quads, each bucketed by its coverage, and every quad in a bucket
    becomes another face of that bucket's single mesh. A 250 m block costs
    `bands` prims and `bands` materials rather than thousands of each.
    """
    x0, y0, x1, y1 = region
    nx = max(1, int(round((x1 - x0) / float(cell_m))))
    ny = max(1, int(round((y1 - y0) / float(cell_m))))
    dx, dy = (x1 - x0) / nx, (y1 - y0) / ny

    buckets = {}
    for iy in range(ny):
        for ix in range(nx):
            ax, ay = x0 + ix * dx, y0 + iy * dy
            cov = float(coverage_at(ax + dx * 0.5, ay + dy * 0.5))
            if cov <= 0.06:
                continue
            buckets.setdefault(
                min(int(bands) - 1, int(cov * int(bands))), []).append((ax, ay))
    if not buckets:
        print("[ground] coverage is zero everywhere — nothing to draw")
        return []

    UsdGeom.Scope.Define(stage, Sdf.Path("/World/burnGround"))
    made = []
    for b, cells in sorted(buckets.items()):
        op = 0.14 + 0.78 * (b + 0.5) / float(bands)
        mat = burn_overlay_mat(
            stage, "{0}/BurnLooks/band_{1}".format(PARENT, b),
            sg._join_asset_root(BURNT_PNG, ""), op)
        pts, counts, idx = [], [], []
        for (ax, ay) in cells:
            k = len(pts)
            e = 0.02          # hairline overlap, so cells in a band show no seam
            for (px, py) in ((ax - e, ay - e), (ax + dx + e, ay - e),
                             (ax + dx + e, ay + dy + e), (ax - e, ay + dy + e)):
                pts.append(Gf.Vec3f(px * ssf, py * ssf, BURN_Z * ssf))
            counts.append(4)
            idx += [k, k + 1, k + 2, k + 3]
        path = "/World/burnGround/band_{0}".format(b)
        m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        m.CreatePointsAttr(pts)
        m.CreateFaceVertexCountsAttr(counts)
        m.CreateFaceVertexIndicesAttr(idx)
        m.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * len(pts))
        m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        m.CreateDisplayColorAttr([Gf.Vec3f(0.16, 0.15, 0.13)])
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), BURN_Z * ssf),
                            Gf.Vec3f(max(xs), max(ys), BURN_Z * ssf)])
        UsdShade.MaterialBindingAPI(m.GetPrim()).Bind(mat)
        made.append(path)
        print("[ground] band {0}: {1:5d} cells  opacity {2:.2f}".format(
            b, len(cells), op))
    return made


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
                        -rw - 60.0, -rh - 60.0, rw + 60.0, rh + 60.0,
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

    # Run the front until it has just cleared the far corner, which is the
    # widest the scar ever gets and so the most it can be judged on.
    corners = [(-rw, -rh), (rw, -rh), (rw, rh), (-rw, rh)]
    finite = [t for t in (arrival(x, y) for x, y in corners)
              if math.isfinite(t)]
    elapsed = ELAPSED or (max(finite) * 1.05 if finite else 200.0)

    def coverage_at(x, y):
        t = arrival(x, y)
        if not math.isfinite(t):
            return 0.0
        d = elapsed - t
        if d < 0.0:
            return 0.0
        return min(1.0, 0.45 + 0.55 * min(1.0, d / max(1e-6, elapsed)))

    made = build_burn_ground(stage, coverage_at, (-rw, -rh, rw, rh), ssf,
                             cell_m=CELL_M, bands=BANDS)

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
    print("  elapsed     {0:.0f} s".format(elapsed))
    print("  overlay     {0} band(s) at {1:.1f} m cells".format(
        len(made), CELL_M))
    print("  OPACITY TEST ROW is north of the scar — six squares, one per")
    print("  material configuration. Which of them you can SEE identifies")
    print("  why the scar did not draw; the log above names them in order.")
    print("  GROUND_BANDS / GROUND_CELL / GROUND_ELAPSED to re-run.")
    print("=" * 72 + "\n")

    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
