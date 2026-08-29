#!/usr/bin/env python
"""
ONE GAC building, sliced into clean per-storey pieces. No damage.

    GS_NAME=SM_Building_01 SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/gac_storey \
    ISAAC_SIM_SCRIPT_NAME=gac_storey_launch_script.py airstack up isaac-sim

`detail/gac_storey_slice.py` clips the merged mesh with `vtkClipPolyData` on
its own measured floor lines, carrying the UVs as POINT data (VTK interpolates
them at the cut) and the material index as CELL data (VTK copies it to the
output triangles). So the material is sliced along with the geometry rather
than re-derived after it.

VERIFIED before this scene existed (`tools/storey_slice_verify.py`, on
SM_Building_01): worst overshoot past a cut plane 0.000000 m, all 14 source
materials present in the bands, UVs interpolated rather than snapped. The
earlier centroid-binned slicer overshot by up to a triangle, which is what
made its edges a sawtooth.

Env:
    GS_NAME     the asset (default SM_Building_01)
    GS_GAP      vertical gap between bands, m (default 1.2). 0 reassembles it,
                which should be indistinguishable from the original.
    GS_KEEP_SRC 1 leaves the merged original standing beside it for an A/B
    SNAP_DIR / KEEP_OPEN
"""
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
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux                  # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import gac_slice as gsl                            # noqa: E402
from detail import gac_storey_slice as gss                     # noqa: E402
from disaster import fracture                                  # noqa: E402

GAC = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
NAME = _env("GS_NAME", "SM_Building_01")
GAP = float(_env("GS_GAP", "1.2"))
KEEP_SRC = _env("GS_KEEP_SRC", "0") in ("1", "true", "yes")
SNAP_DIR = _env("SNAP_DIR")


def main():
    tl = omni.timeline.get_timeline_interface()
    tl.stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    w = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(w.GetPrim())
    _, ssf = get_stage_meters_per_unit(stage)
    t0 = time.time()
    fracture.ensure_vtk(verbose=True)

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(1200.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2600.0)
    key.CreateAngleAttr(0.9)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))
    gp = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = 160.0
    gp.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                         Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    gp.CreateFaceVertexCountsAttr([4])
    gp.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    gp.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    gp.CreateDisplayColorAttr([Gf.Vec3f(0.32, 0.32, 0.31)])

    # ---- the source, centred with its base on the ground -------------------
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/src"))
    kid = stage.DefinePrim(Sdf.Path("/World/src/asset"))
    kid.GetReferences().AddReference(GAC + NAME + ".usd")
    stage.Load(Sdf.Path("/World/src"))
    xf = UsdGeom.Xformable(kid)
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp()
    xf.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = bc.ComputeWorldBound(stage.GetPrimAtPath("/World/src")).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    tr.Set(Gf.Vec3d(-0.5 * (mn[0] + mx[0]), -0.5 * (mn[1] + mx[1]), -mn[2]))

    wins, bbox = gsl.window_centres(stage, "/World/src")
    g = gsl.measure_grid(wins, bbox, name=NAME)
    m = gss.read_mesh(stage, "/World/src")
    # CUT BETWEEN THE WINDOW ROWS, not through them: the measured lattice is
    # phase-locked to the window centres, so cutting on it slices 63% of the
    # windows in half. Shifted half a storey it cuts 1.8%. See `cut_lines`.
    lines = gss.cut_lines(g, float(_env("GS_CUT_OFFSET", "0.5")))
    worst, hit = gss.window_clearance(lines, wins)
    print("[gac_storey] cut clearance {0:.3f} m; {1} window face(s) crossed"
          .format(worst or 0.0, hit))
    bands = gss.storeys(m, lines)

    # ---- THE ROUND TRIP -------------------------------------------------
    # Three groups, one slice. If the REASSEMBLED column is not
    # indistinguishable from the ORIGINAL, the cut lost or moved something —
    # which is the only test that actually proves a slicer, and one a single
    # exploded view cannot give you.
    #   x = -SEP   ORIGINAL      the merged asset, untouched
    #   x =    0   REASSEMBLED   every band written back at its own height
    #   x = +SEP   EXPLODED      the same bands, pushed apart by GS_GAP
    SEP = max(g["W"], g["D"]) * 2.0 + 18.0
    UsdGeom.Xformable(stage.GetPrimAtPath("/World/src")).AddTranslateOp().Set(
        Gf.Vec3d(-SEP, 0.0, 0.0))
    # storey bands -> ring cells (corner / wall run / core)
    leg = max(1.5, 0.6 * ((g["bays"].get("E") or {}).get("pitch") or 4.0))
    bay = (g["bays"].get("E") or {}).get("pitch") or 4.0
    cells = []
    for i, (lo, hi, band) in enumerate(bands):
        bb = ((bbox[0][0], bbox[0][1], lo), (bbox[1][0], bbox[1][1], hi))
        for role, side, k, piece in gss.ring(band, bb, leg, bay):
            cells.append((i, role, side, k, piece))
    roles = {}
    for _i, role, _s, _k, _p in cells:
        roles[role] = roles.get(role, 0) + 1
    print("[gac_storey] {0} cell(s): {1}".format(
        len(cells), "  ".join("{0}={1}".format(a, b)
                              for a, b in sorted(roles.items()))))

    # exploded: each piece moves out along its OWN elevation and up by storey
    OUT = {"S": (0.0, -1.0), "N": (0.0, 1.0), "E": (1.0, 0.0),
           "W": (-1.0, 0.0), "SW": (-0.7, -0.7), "SE": (0.7, -0.7),
           "NW": (-0.7, 0.7), "NE": (0.7, 0.7), "-": (0.0, 0.0)}
    n = 0
    for grp, dx, gap in (("assembled", 0.0, 0.0), ("exploded", SEP, GAP)):
        scope = "/World/{0}".format(grp)
        UsdGeom.Scope.Define(stage, Sdf.Path(scope))
        for j, (i, role, side, k, piece) in enumerate(cells):
            path = "{0}/{1}_{2}_{3}_{4:02d}_{5}".format(
                scope, role, side.replace("-", "x"), k, i, j)
            if not gss.write_piece(stage, path, piece, m["mats"]):
                continue
            ox, oy = OUT.get(side, (0.0, 0.0))
            UsdGeom.Xformable(stage.GetPrimAtPath(path)).AddTranslateOp().Set(
                Gf.Vec3d(dx + ox * gap * 2.0, oy * gap * 2.0, gap * i))
            n += 1
    H = float(m["P"][:, 2].max()) + GAP * len(bands)
    print("\n" + "=" * 70)
    print("GAC STOREY SLICE   {0}".format(NAME))
    print("  {0:.1f} x {1:.1f} x {2:.1f} m, storey {3:.2f} m, {4} floor line(s)"
          .format(g["W"], g["D"], g["H"], g["storey_h"], len(g["storeys"])))
    print("  {0} band(s) -> {1} kit piece(s) x 2 group(s), {2} material(s)"
          .format(len(bands), len(cells), len(m["mats"])))
    print("  layout: ORIGINAL at x=-{0:.0f} | REASSEMBLED at x=0 | "
          "EXPLODED at x=+{0:.0f} (gap {1:.1f} m)".format(SEP, GAP))
    print("  {0:.0f} s".format(time.time() - t0))
    print("=" * 70 + "\n")

    app = omni.kit.app.get_app()
    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            for _ in range(90):
                app.update()
            # BOTH BUILDINGS FROM THE SAME SIDE, or the A/B is worthless.
            # SM_Building_01 is `front:E, blank:N,W,S` — it carries detail on
            # ONE elevation. A camera placed between the two columns shows the
            # original's E face against the reassembled's W face, i.e. the
            # detailed front against the blank back, which reads as a total
            # loss of texture and is purely a framing error. The SE oblique
            # below puts the E and S faces of BOTH in the same frame.
            span = SEP * 2.0 + max(g["W"], g["D"]) * 1.4
            for nm, eye, tgt in (
                    ("se_oblique", (span * 0.70, -span * 0.70, H * 0.75),
                     (0.0, 0.0, H * 0.35)),
                    ("east_on", (span * 0.95, 0.0, H * 0.42),
                     (0.0, 0.0, H * 0.42)),
                    ("orig_E", (SEP * -1.0 + g["W"] * 2.4, 0.0, H * 0.30),
                     (-SEP, 0.0, H * 0.30)),
                    ("rebuilt_E", (g["W"] * 2.4, 0.0, H * 0.30),
                     (0.0, 0.0, H * 0.30))):
                sn.place_camera(stage, tuple(q * ssf for q in eye),
                                tuple(q * ssf for q in tgt))
                sn.snapshot(os.path.join(SNAP_DIR, "{0}_{1}.png".format(NAME, nm)))
            print("[gac_storey] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[gac_storey] snapshots FAILED: {0}".format(exc))

    if _env("KEEP_OPEN") == "1" or not _HEADLESS:
        while simulation_app.is_running():
            app.update()
    tl.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
