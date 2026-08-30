#!/usr/bin/env python
"""gac_deck_probe -- measure the real roof-deck height (`deck_z`) of a GAC
asset against the bbox top (`top`, the parapet coping) and report the gap
(the parapet height) `gac_fire.mass_from_grid` now measures off the merged
mesh's upward-facing faces.

WHY. Row-2 review, 2026-08-30: "Lots of floating roof props and also
floating debris in B7, B6, B5, B4" (SM_Building_09 F6, SM_Building_05 F5,
SM_Building_23 F4, SM_Building_12 F3). The bake logs traced it to
`dress_roof_urban`/`_deck_slab`/`_rafter_teeth`/`r_roof_scorch` all seating
their geometry at `m["top"]` -- the highest point of the WHOLE bbox, which
is the parapet coping, not the roof surface itself. This probe is the
measurement that motivated `mass_from_grid`'s new `deck_z` field: it prints
`deck_z`, `top` and the parapet height (`top - deck_z`) for the buildings
named in that review plus every other asset passed on the command line, so
the fix can be checked against real numbers instead of a screenshot.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/gac_deck_probe.py"

    # or a specific set:
    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/gac_deck_probe.py SM_Building_09 SM_Building_13"

Default set: the five buildings actually named in the row-2 bake logs
(`gac_SM_Building_0{3,5,9}_F*_s*.log`, `gac_SM_Building_12_F3_s69.log`,
`gac_SM_Building_23_F4_s100.log`) plus `SM_Building_13`, named separately in
the task -- no "row-2 manifest" of a sixth building turned up in the repo,
so this is the full set of names the report actually gives evidence for.
"""
import sys
import time

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom                                   # noqa: E402
from detail import gac_slice as gsl, gac_storey_slice as gss   # noqa: E402
from disaster import gac_fire as gf                             # noqa: E402

DEFAULT_NAMES = ("SM_Building_03", "SM_Building_05", "SM_Building_09",
                 "SM_Building_12", "SM_Building_23", "SM_Building_13")


def measure(stage, name, ix):
    cell = "/W/b{0}".format(ix)
    UsdGeom.Xform.Define(stage, cell)
    src = gf.place_source(stage, cell, gf.GAC_DIR + name + ".usd", gf.GAC_SCALE)
    if not src:
        print("{0:<18} PLACE FAILED (nothing composed)".format(name))
        return None
    wins, bbox = gsl.window_centres(stage, src)
    if bbox is None:
        print("{0:<18} NO GEOMETRY".format(name))
        return None
    g, measured = gss.grid_for(stage, src, bbox, wins, name=name, verbose=False)
    mesh = gss.read_mesh(stage, src, verbose=False)
    m = gf.mass_from_grid(g, bbox, mesh=mesh)
    par = m["top"] - m["deck_z"]
    print("{0:<18} top {1:7.2f} m  deck_z {2:7.2f} m  parapet {3:5.2f} m  "
          "({4})".format(name, m["top"], m["deck_z"], par, m["deck_note"]))
    return m


if __name__ == "__main__":
    names = sys.argv[1:] or DEFAULT_NAMES
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    t0 = time.time()
    print("{0:<18} {1:>10}  {2:>13}  {3:>12}".format(
        "asset", "top (m)", "deck_z (m)", "parapet (m)"))
    for ix, name in enumerate(names):
        try:
            measure(st, name, ix)
        except Exception as exc:
            print("{0:<18} FAILED: {1}".format(name, exc))
    print("[gac_deck_probe] {0:.1f}s total".format(time.time() - t0))
