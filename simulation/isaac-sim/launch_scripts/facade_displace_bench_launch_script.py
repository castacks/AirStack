#!/usr/bin/env python
"""
Façade displacement bench — does putting the DEPTH in the material actually
work in this renderer?

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/facade \
    ISAAC_SIM_SCRIPT_NAME=facade_displace_bench_launch_script.py airstack up isaac-sim

THE QUESTION
------------
The 112 standalone building assets are flat boxes with a photograph on them.
The proposed fix (Workflow 1) is to stop adding polygons and put the window
reveals, panel seams and sills in a HEIGHT MAP instead, displaced at render
time — no extra geometry, no physics cost, no USD bloat. If it works it is
much better than any remeshing, because remeshing adds vertices without
adding DETAIL: subdividing a flat wall gives you a finely tessellated flat
wall.

Two things had to be checked before believing the workflow as written:

  * **OmniPBR HAS NO DISPLACEMENT INPUT IN THIS BUILD.** `grep -c displacement
    /isaac-sim/kit/mdl/core/Base/OmniPBR.mdl` returns 0, so the step "create
    an OmniPBR material, scroll to the Displacement sub-menu, enable it" has
    no such sub-menu here. `OmniSurface.mdl` does: `geometry_displacement`,
    `geometry_displacement_image`, `geometry_displacement_scale`.
  * Whether the RTX renderer TESSELLATES for it. No `/rtx/**displacement**`
    or tessellation setting appears in the app config, and this renderer has
    form for silently dropping material features it does not implement — a
    `UsdPreviewSurface` opacity texture is dropped in the UsdToMdl
    translation, and fractional cutout opacity needed a command-line flag AND
    a re-assert after the stage loaded (both in the wildfire skill).

So the bench is four panels of the same wall, and the answer is whichever
pair looks alike:

    flat        albedo only                    — what a monolith is today
    normal      albedo + normal map            — shading only, no silhouette
    displaced   OmniSurface + height map       — the workflow under test
    geometry    real extruded reveals          — the ground truth

If `displaced` matches `geometry`, the workflow is the answer for all 112
assets. If it matches `normal`, the renderer is only shading it and the
depth is a lie that will break at every grazing angle and every silhouette —
which is exactly where a drone camera sees a façade.

Env:
    FD_KIND      office | residential | brutalist  (default office)
    FD_SCALE     displacement scale, metres (default 0.35)
    FD_PANEL     panel size, metres (default 12)
    SNAP_DIR     captures, under /isaac-sim/.nvidia-omniverse/logs/
"""

import math
import os
import sys

from isaacsim import SimulationApp


def _env(n, d=""):
    v = os.environ.get(n)
    return d if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
simulation_app = SimulationApp(launch_config={"headless": _HEADLESS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")

import omni.kit.app                                            # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, Vt    # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

KIND = _env("FD_KIND", "office")
SCALE = float(_env("FD_SCALE", "0.35"))
PANEL = float(_env("FD_PANEL", "12.0"))
SNAP_DIR = _env("SNAP_DIR", "")
TEX = os.path.join(_SG, "assets", "materials", "facade")


def _tex(name):
    return os.path.join(TEX, "{0}_{1}.png".format(KIND, name))


def omnipbr(stage, path, albedo, normal=None, rough=None):
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path + "/Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(albedo))
    sh.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(
        Gf.Vec2f(1.0, 1.0))
    if normal:
        sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(normal))
        sh.CreateInput("bump_factor", Sdf.ValueTypeNames.Float).Set(1.0)
    if rough:
        sh.CreateInput("reflectionroughness_texture",
                       Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(rough))
        sh.CreateInput("reflection_roughness_texture_influence",
                       Sdf.ValueTypeNames.Float).Set(1.0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    return mat


def omnisurface_displaced(stage, path, albedo, height, normal, scale):
    """OmniSurface with `geometry_displacement_image` wired up."""
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path + "/Shader"))
    sh.CreateIdAttr("OmniSurface")
    sh.SetSourceAsset(Sdf.AssetPath("OmniSurface.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniSurface", "mdl")
    sh.CreateInput("diffuse_reflection_color_image",
                   Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(albedo))
    sh.CreateInput("geometry_normal_image", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(normal))
    sh.CreateInput("geometry_displacement_image",
                   Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(height))
    sh.CreateInput("geometry_displacement_scale",
                   Sdf.ValueTypeNames.Float).Set(float(scale))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    return mat


def quad(stage, path, cx, w, h, mat, subdiv=0):
    """A wall panel facing -Y, optionally pre-tessellated.

    DISPLACEMENT NEEDS SOMETHING TO DISPLACE. Every implementation of it
    moves VERTICES, so a two-triangle quad has four of them and can only be
    tilted, never embossed — a detail that the "no extra polygons" pitch
    glosses over. `subdiv` builds an n x n grid so the panel has vertices at
    roughly the texel scale, which is the honest version of the comparison:
    the geometry cost does not vanish, it moves from modelling to a uniform
    grid that costs nothing to author.
    """
    n = max(1, int(subdiv))
    pts, uvs, idx = [], [], []
    for j in range(n + 1):
        for i in range(n + 1):
            u, v = i / float(n), j / float(n)
            pts.append(Gf.Vec3f(cx - w / 2 + u * w, 0.0, v * h))
            uvs.append(Gf.Vec2f(u, v))
    for j in range(n):
        for i in range(n):
            a = j * (n + 1) + i
            idx += [a, a + 1, a + n + 2, a + n + 1]
    me = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    me.CreatePointsAttr(Vt.Vec3fArray(pts))
    me.CreateFaceVertexCountsAttr(Vt.IntArray([4] * (n * n)))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    me.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    me.CreateExtentAttr([Gf.Vec3f(cx - w / 2, -0.1, 0.0),
                         Gf.Vec3f(cx + w / 2, 0.1, h)])
    pv = UsdGeom.PrimvarsAPI(me.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set(Vt.Vec2fArray(uvs))
    UsdShade.MaterialBindingAPI(me.GetPrim()).Bind(mat)
    return me


def real_reveals(stage, root, cx, w, h, bays, storeys, depth, mat):
    """The ground truth: a wall with the window reveals actually modelled."""
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    k = 0
    bw, bh = w / bays, h / storeys
    for j in range(storeys):
        for i in range(bays):
            x0 = cx - w / 2 + i * bw
            z0 = j * bh
            # spandrel under, mullion left, head over: four solid strips per
            # bay, with the window plane set back by `depth`
            for (ox, oz, sx, sz) in ((0.0, 0.0, bw, bh * 0.18),
                                     (0.0, bh * 0.80, bw, bh * 0.20),
                                     (0.0, bh * 0.18, bw * 0.12, bh * 0.62),
                                     (bw * 0.88, bh * 0.18, bw * 0.12,
                                      bh * 0.62)):
                p = "{0}/s{1}".format(root, k); k += 1
                me = UsdGeom.Cube.Define(stage, Sdf.Path(p))
                me.CreateSizeAttr(1.0)
                xf = UsdGeom.Xformable(me)
                xf.ClearXformOpOrder()
                xf.AddTranslateOp().Set(Gf.Vec3d(
                    x0 + ox + sx / 2, -depth / 2, z0 + oz + sz / 2))
                xf.AddScaleOp().Set(Gf.Vec3f(float(sx), float(depth),
                                             float(sz)))
                UsdShade.MaterialBindingAPI(me.GetPrim()).Bind(mat)
            # the glass plane, set back
            p = "{0}/g{1}".format(root, k); k += 1
            me = UsdGeom.Cube.Define(stage, Sdf.Path(p))
            me.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(me)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(x0 + bw / 2, depth * 0.6,
                                             z0 + bh * 0.49))
            xf.AddScaleOp().Set(Gf.Vec3f(float(bw * 0.76), 0.06,
                                         float(bh * 0.62)))
    return k


def main():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    w = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(w.GetPrim())
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(700.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2600.0)
    key.CreateAngleAttr(0.7)
    # RAKING, ~22 deg. Displacement and normal maps look identical head-on
    # and diverge completely under a low sun, which is the condition this
    # bench exists to create.
    key.AddRotateXYZOp().Set(Gf.Vec3f(-22.0, 0.0, 28.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = 300.0
    g.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                        Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.33, 0.33, 0.32)])
    g.CreateExtentAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, e, 0)])

    for f in ("albedo", "height", "normal", "rough"):
        if not os.path.exists(_tex(f)):
            raise SystemExit("missing map: {0}\nrun: python3 "
                             "scene_gen/tools/facade_maps.py {1} --kind {2}"
                             .format(_tex(f), TEX, KIND))

    pitch = PANEL * 1.35
    cols = []
    m_flat = omnipbr(stage, "/World/Looks/flat", _tex("albedo"))
    cols.append(("flat", quad(stage, "/World/p_flat", -1.5 * pitch, PANEL,
                              PANEL, m_flat, subdiv=1)))
    m_nrm = omnipbr(stage, "/World/Looks/nrm", _tex("albedo"),
                    normal=_tex("normal"), rough=_tex("rough"))
    cols.append(("normal", quad(stage, "/World/p_nrm", -0.5 * pitch, PANEL,
                                PANEL, m_nrm, subdiv=1)))
    m_disp = omnisurface_displaced(stage, "/World/Looks/disp", _tex("albedo"),
                                   _tex("height"), _tex("normal"), SCALE)
    # a 200 x 200 grid: ~6 cm vertices on a 12 m panel, so the displacement
    # has something to move
    cols.append(("displaced", quad(stage, "/World/p_disp", 0.5 * pitch, PANEL,
                                   PANEL, m_disp, subdiv=200)))
    n = real_reveals(stage, "/World/p_geom", 1.5 * pitch, PANEL, PANEL, 6, 6,
                     0.45, m_flat)
    cols.append(("geometry", None))

    for _ in range(20):
        omni.kit.app.get_app().update()
    print("\n" + "=" * 72)
    print("FACADE DISPLACEMENT BENCH   kind={0}  disp scale={1} m".format(
        KIND, SCALE))
    print("  flat / normal / displaced / geometry   ({0} solids in the "
          "modelled column)".format(n))
    print("  displaced panel: 200x200 grid = 40,401 verts")
    print("=" * 72 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            span = pitch * 4
            sn.place_camera(stage, (0.0, -span * 0.62, PANEL * 0.55),
                            (0.0, 0.0, PANEL * 0.45))
            sn.snapshot(os.path.join(SNAP_DIR, "row_front.png"))
            # THE GRAZING VIEW IS THE TEST. Head-on, a normal map and a real
            # reveal are indistinguishable; from 20 degrees off the wall,
            # parallax and self-shadowing separate them instantly.
            sn.place_camera(stage, (-span * 0.52, -span * 0.20, PANEL * 0.42),
                            (span * 0.15, 0.0, PANEL * 0.40))
            sn.snapshot(os.path.join(SNAP_DIR, "row_grazing.png"))
            for i, (nm, _q) in enumerate(cols):
                cx = (-1.5 + i) * pitch
                sn.place_camera(stage, (cx - 3.0, -9.0, PANEL * 0.55),
                                (cx, 0.0, PANEL * 0.5))
                sn.snapshot(os.path.join(SNAP_DIR, "{0}.png".format(nm)))
                sn.place_camera(stage, (cx - 11.0, -3.2, PANEL * 0.5),
                                (cx + 2.0, 0.0, PANEL * 0.45))
                sn.snapshot(os.path.join(SNAP_DIR,
                                         "{0}_graze.png".format(nm)))
            print("[facade] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[facade] snapshots FAILED: {0}".format(exc))

    print("FACADE BENCH DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
