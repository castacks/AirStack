#!/usr/bin/env python
"""
Monolith depth bench — the same building three ways: flat photograph, the
photograph with a NORMAL MAP DERIVED FROM ITSELF, and modelled greebles.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/mono_depth \
    ISAAC_SIM_SCRIPT_NAME=mono_depth_bench_launch_script.py airstack up isaac-sim

THE QUESTION THIS ANSWERS
--------------------------
"Is there a tool that can give the texture a depth map?" Yes — and the useful
finding is WHICH tool, because the obvious one is the wrong one.

`scene_gen/tools/texture_depth.py` runs two estimators over the same façade
photograph:

  * **Depth Anything V2 (small)**, the monocular depth model, on the GPU:
    8.5 s, and it returns a smooth vertical GRADIENT with the window
    structure gone. That is not a bug in the model, it is the model doing its
    job on the wrong input — it is trained to answer "which parts of this
    SCENE are further away", and a head-on crop of a wall has no scene in it,
    so it infers a plausible recession and throws away exactly the per-window
    relief that was wanted.
  * **Luminance-as-depth with a bilateral filter**: 0.02 s, and it keeps every
    mullion and storey band. A façade photographed square-on is a lit wall
    with dark recesses in it, so brightness IS depth to within a constant.
    Bilateral rather than Gaussian because the window EDGE is the whole
    signal and a Gaussian blurs it.

400x faster and better on this content. The general tool loses to the
specific one because the input is not what the general tool was built for.

WHAT THE DEPTH CAN AND CANNOT DRIVE
------------------------------------
It cannot displace anything: the four-panel test
(`facade_displace_bench_launch_script.py`) established that OmniPBR has no
displacement input in this build and that RTX Real-Time does not tessellate
for OmniSurface's. So the depth field is used for the two things that DO
render:

  column 2  a NORMAL MAP, connected to the asset's own `UsdPreviewSurface`
            through its own UVs — so the relief lands exactly on the painted
            windows, which is the alignment the greebles never had. Shading
            only: it will not change a silhouette.
  column 3  GEOMETRY (`detail/greeble.py`) — real pockets, real silhouette,
            and the one that survives a grazing view.

Env:
    MD_ASSET   default tower_03_0015
    SNAP_DIR   captures, under /isaac-sim/.nvidia-omniverse/logs/
"""

import math
import os
import sys
import time

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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade        # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

from detail import greeble                                     # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

ASSET = _env("MD_ASSET", "tower_03_0015")
SNAP_DIR = _env("SNAP_DIR", "")
NUC = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "selected_citydemo/tower/{0}.usd".format(ASSET))
DEPTH_DIR = os.path.join(_SG, "assets", "mono_upscale", "depth")


def attach_normal_maps(stage, root, strength=1.0):
    """Wire each material's own derived normal map into its shader.

    THROUGH THE ASSET'S OWN UVs, which is the entire point. The greebles are
    laid on an assumed grid and land wherever they land; this lands ON the
    painted windows by construction, because the map was computed FROM the
    texture those windows are painted in and is addressed by the same
    `primvars:uv0`. It is the alignment the geometry route does not have —
    bought at the price of having no silhouette, which is the alignment the
    geometry route does have. That trade is what the bench is showing.
    """
    n_ok = n_miss = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        sh = UsdShade.Shader(prim)
        if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
            continue
        d = sh.GetInput("diffuseColor")
        if d is None or not d.HasConnectedSource():
            continue
        tex_sh = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
        f = tex_sh.GetInput("file")
        v = f.Get() if f else None
        if not isinstance(v, Sdf.AssetPath) or not v.path:
            continue
        stem = os.path.splitext(v.path.rsplit("/", 1)[-1])[0]
        nmap = os.path.join(DEPTH_DIR, stem + "_classical_normal.png")
        if not os.path.exists(nmap):
            n_miss += 1
            continue
        base = str(tex_sh.GetPath().GetParentPath())
        ns = UsdShade.Shader.Define(stage, Sdf.Path(base + "/derivedNormal"))
        ns.CreateIdAttr("UsdUVTexture")
        ns.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(nmap))
        # A NORMAL MAP IS DATA, NOT COLOUR. Decoded as sRGB the vectors come
        # back wrong and the speculars blow out — the wildfire skill's
        # `sourceColorSpace` finding, and it applies to every non-colour map.
        ns.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
        ns.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(
            Gf.Vec4f(strength, strength, 1.0, 1.0))
        ns.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(
            Gf.Vec4f(-0.5 * strength, -0.5 * strength, 0.0, 0.0))
        ns.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        # reuse the diffuse texture's own st reader, so the normal is sampled
        # in exactly the UV space the photograph is
        st_in = tex_sh.GetInput("st")
        if st_in is not None and st_in.HasConnectedSource():
            src = st_in.GetConnectedSource()
            ns.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                UsdShade.Shader(src[0].GetPrim()).ConnectableAPI(), src[1])
        sh.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(
            ns.ConnectableAPI(), "rgb")
        n_ok += 1
    return n_ok, n_miss


def _ref(stage, path, url, x, y):
    p = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    child = stage.DefinePrim(Sdf.Path(path + "/asset"))
    child.GetReferences().AddReference(url)
    stage.Load(Sdf.Path(path))
    xf = UsdGeom.Xformable(p)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), 0.0))
    return p


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
    dome.CreateColorAttr(Gf.Vec3f(0.74, 0.78, 0.86))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(3200.0)
    key.CreateAngleAttr(0.6)
    # RAKING. A normal map and a flat wall are identical head-on and separate
    # completely under a low sun; that is the comparison.
    key.AddRotateXYZOp().Set(Gf.Vec3f(-18.0, 0.0, 26.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = 400.0
    g.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                        Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.30, 0.29)])
    g.CreateExtentAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, e, 0)])

    t0 = time.time()
    pitch = 95.0
    cols = [("flat", -pitch), ("normal_from_depth", 0.0), ("greebled", pitch)]
    for name, x in cols:
        _ref(stage, "/World/" + name, NUC, x, 0.0)
    for _ in range(6):
        omni.kit.app.get_app().update()

    n_ok, n_miss = attach_normal_maps(stage, "/World/normal_from_depth")
    print("[depth] normal maps wired: {0} material(s), {1} without a map"
          .format(n_ok, n_miss))

    parts = uf.mono_parts(stage, "/World/greebled")
    gm = greeble.materials(stage, "/World")
    made = greeble.greeble_parts(stage, "/World/greeble", parts, gm)
    print("[depth] greebles: {0} part(s) -> {1} prim(s)".format(
        len(parts), len(made)))

    for _ in range(12):
        omni.kit.app.get_app().update()
    H = max((p["z1"] for p in parts), default=83.0)
    print("\n" + "=" * 72)
    print("MONOLITH DEPTH BENCH   {0}   H={1:.0f} m".format(ASSET, H))
    print("  flat | normal-from-its-own-photo | greebled geometry")
    print("  depth: classical 0.02 s (keeps the mullions) vs "
          "Depth Anything V2 8.5 s (loses them)")
    print("  built in {0:.0f} s".format(time.time() - t0))
    print("=" * 72 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            sn.place_camera(stage, (0.0, -290.0, H * 0.55), (0.0, 0.0, H * 0.45))
            sn.snapshot(os.path.join(SNAP_DIR, "row.png"))
            for name, x in cols:
                sn.place_camera(stage, (x - 55.0, -80.0, H * 0.52),
                                (x, 0.0, H * 0.42))
                sn.snapshot(os.path.join(SNAP_DIR, name + ".png"))
                # the close raking view: where a normal map gives itself away
                sn.place_camera(stage, (x - 34.0, -40.0, 16.0),
                                (x + 6.0, 0.0, 26.0))
                sn.snapshot(os.path.join(SNAP_DIR, name + "_close.png"))
            print("[depth] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[depth] snapshots FAILED: {0}".format(exc))

    print("DEPTH BENCH DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
