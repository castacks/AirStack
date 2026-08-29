#!/usr/bin/env python
"""
Monolith greeble bench — a REAL standalone building asset, untouched next to
the same asset with modelled window reveals wrapped onto its elevations.

    MG_ASSET=tower_03_0015 SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/greeble \
    ISAAC_SIM_SCRIPT_NAME=mono_greeble_bench_launch_script.py airstack up isaac-sim

The four-panel displacement test settled that depth on these assets has to be
GEOMETRY: OmniPBR has no displacement input in this build, RTX Real-Time does
not tessellate for OmniSurface's, and a normal map does nothing at a
silhouette. This puts the geometry on a real building instead of a test wall,
and counts what it costs.

`detail/greeble.py` carries the construction and the reason for it — in
particular why the windows are a POCKET (continuous wall, recessed pane)
rather than a frame with a hole in it, which is what made the first version's
windows "seem floating instead of connected to the outside wall part".

Env:
    MG_ASSET     stem in scene_gen/assets/mono_upscale (default tower_03_0015)
    MG_BAY       bay pitch, m (default 3.6)
    MG_STOREY    floor-to-floor, m (default 3.6)
    MG_REVEAL    how far the pane sits back, m (default 0.42)
    MG_WALL_T    band / mullion depth, m (default 0.55)
    SNAP_DIR     captures, under /isaac-sim/.nvidia-omniverse/logs/
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

ASSET = _env("MG_ASSET", "tower_03_0015")
BAY = float(_env("MG_BAY", "3.6"))
STOREY = float(_env("MG_STOREY", "3.6"))
REVEAL = float(_env("MG_REVEAL", "0.42"))
WALL_T = float(_env("MG_WALL_T", "0.55"))
SNAP_DIR = _env("SNAP_DIR", "")
SRC = os.path.join(_SG, "assets", "mono_upscale", ASSET + ".usd")
# REFERENCE NUCLEUS, NOT THE LOCAL COPY. The downloaded asset keeps its
# texture paths as `../textures/x.jpg`, which resolved against Nucleus and
# does not resolve against `scene_gen/assets/mono_upscale/` — so the local
# file renders as a BLACK building. The Nucleus URL is the same asset with
# working textures; `MG_LOCAL=1` forces the local copy.
NUCLEUS = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
           "selected_citydemo/tower/{0}.usd".format(ASSET))
if _env("MG_LOCAL", "") != "1":
    SRC = NUCLEUS


def main():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    w = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(w.GetPrim())
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(750.0)
    dome.CreateColorAttr(Gf.Vec3f(0.74, 0.78, 0.86))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2800.0)
    key.CreateAngleAttr(0.8)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-24.0, 0.0, 32.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = 500.0
    g.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                        Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.33, 0.33, 0.32)])
    g.CreateExtentAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, e, 0)])

    if not SRC.startswith("omniverse://") and not os.path.exists(SRC):
        raise SystemExit("missing {0}".format(SRC))

    t0 = time.time()
    pitch = 90.0
    cols = []
    for i, label in enumerate(("original", "greebled")):
        holder = "/World/b{0}".format(i)
        hp = UsdGeom.Xform.Define(stage, Sdf.Path(holder))
        hp.GetPrim().GetReferences().AddReference(SRC)
        xf = UsdGeom.Xformable(hp)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(-pitch / 2.0 + i * pitch, 0.0, 0.0))
        for _ in range(4):
            omni.kit.app.get_app().update()
        cols.append({"i": i, "label": label, "holder": holder,
                     "x": -pitch / 2.0 + i * pitch})

    # measure the parts of the greebled copy
    parts = uf.mono_parts(stage, cols[1]["holder"])
    print("[greeble] {0}: {1} part(s)".format(ASSET, len(parts)))
    for p in parts:
        print("[greeble]   {0:<6} z {1:6.2f}..{2:<6.2f} {3:5.1f} x {4:<5.1f} "
              "{5}".format(p["kind"], p["z0"], p["z1"],
                           p["x1"] - p["x0"], p["y1"] - p["y0"],
                           p["texture"][:34]))
    mats = greeble.materials(stage, "/World")
    tb = time.time()
    made = greeble.greeble_parts(
        stage, "/World/greeble", parts, mats,
        bay_m=BAY, storey_m=STOREY, reveal_m=REVEAL, wall_t=WALL_T)
    dt = time.time() - tb

    H = max((p["z1"] for p in parts), default=40.0)
    W = max((p["x1"] - p["x0"] for p in parts), default=30.0)
    D = max((p["y1"] - p["y0"] for p in parts), default=30.0)
    for _ in range(10):
        omni.kit.app.get_app().update()
    n_tri = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath("/World/greeble")):
        if p.IsA(UsdGeom.Mesh):
            c = UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()
            n_tri += sum(max(0, q - 2) for q in (c or []))

    print("\n" + "=" * 74)
    print("MONOLITH GREEBLE BENCH   {0}".format(ASSET))
    print("  building {0:.0f} x {1:.0f} x {2:.0f} m, {3} wall part(s)".format(
        W, D, H, sum(1 for p in parts if p["kind"] != "roof")))
    print("  bay {0:.1f} m | storey {1:.1f} m | reveal {2:.2f} m | "
          "wall {3:.2f} m".format(BAY, STOREY, REVEAL, WALL_T))
    print("  {0} greeble prim(s), {1} triangles, built in {2:.2f} s".format(
        len(made), n_tri, dt))
    print("=" * 74 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            span = pitch * 1.9
            sn.place_camera(stage, (0.0, -span * 0.85, H * 0.62),
                            (0.0, 0.0, H * 0.45))
            sn.snapshot(os.path.join(SNAP_DIR, "pair.png"))
            for c in cols:
                fit = max(max(W, D) / 1.164, H / 0.655) * 1.12
                sn.place_camera(stage, (c["x"] - fit * 0.42, -fit * 0.80,
                                        H * 0.60), (c["x"], 0.0, H * 0.45))
                sn.snapshot(os.path.join(SNAP_DIR,
                                         "{0}.png".format(c["label"])))
                # THE CLOSE, RAKING VIEW IS THE TEST — a reveal only proves
                # itself when you can see along it and it self-shadows.
                sn.place_camera(stage, (c["x"] - W * 0.85, -D * 1.15, 12.0),
                                (c["x"] + W * 0.1, 0.0, 18.0))
                sn.snapshot(os.path.join(SNAP_DIR,
                                         "{0}_close.png".format(c["label"])))
            print("[greeble] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[greeble] snapshots FAILED: {0}".format(exc))

    print("GREEBLE BENCH DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
