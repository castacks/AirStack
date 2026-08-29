#!/usr/bin/env python
"""Measure the GreatAmericanCity props into `_plans/gac_props.json`.

    bash scene_gen/tools/usd_python.sh scene_gen/tools/gac_props_measure.py

Roof plant, wall-mounted runs and parapet furniture, with the numbers a placer
needs: size in METRES (through each asset's own `metersPerUnit`, which is 0.01
across this pack), the bbox centre so a prop can be seated by its footprint
rather than by whatever pivot the exporter left, and `z0` so it sits ON the
roof instead of sunk into it.

`kind` is assigned here rather than at placement time because it is a property
of the ASSET: a water tank goes on a roof, a stair goes on a wall, and no
amount of layout context changes that.
"""
import json, os, time
from pxr import Usd, UsdGeom

ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
        "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "_plans", "gac_props.json")

# kind -> the assets that belong to it. Curated, not pattern-matched: the pack
# has no asset with "escape" in its name, so the fire-escape role is filled by
# the two stair meshes and judged by their measured shape below.
KINDS = {
    "roof_plant": ["SM_Air_Machine", "SM_Air_Tubes_Machine",
                   "SM_Air_Tubes_Machine_Part_02", "SM_Air_Tubes_Machine_Part_03",
                   "SM_Generator_Eletric", "SM_Building_Air"],
    "roof_tank": ["SM_Water_Tank", "SM_Water_Tank_02"],
    "roof_mast": ["SM_Communication_Tower", "SM_Tower"],
    "roof_house": ["SM_Superior_Construction_01", "SM_Superior_Construction_02",
                   "SM_Superior_Construction_03", "SM_Superior_Construction_04",
                   "SM_Glass_Roof"],
    "roof_pipe": ["SM_Steel_Pipe", "SM_Steel_Pipe_Plastic",
                  "SM_Construct_Tubes", "SM_Construct_Tubes_Curve"],
    # the fire-escape / service-face family: only ever on a BLANK elevation
    "wall_stair": ["SM_Building_Stair", "SM_Stair"],
    "wall_run": ["SM_Protect_Tube", "SM_Tube", "SM_Tube_Curve",
                 "SM_Tube_Curve_02", "SM_Cable", "SM_Protective_Grid"],
    "wall_door": ["SM_Exit_Door_02"],
}


def main():
    out, t0 = [], time.time()
    for kind, names in KINDS.items():
        for nm in names:
            try:
                st = Usd.Stage.Open(ROOT + nm + ".usd")
                st.Load()
            except Exception as exc:
                print("%-32s OPEN FAIL %s" % (nm, exc)); continue
            S = UsdGeom.GetStageMetersPerUnit(st)
            bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_])
            r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
            if r.IsEmpty():
                print("%-32s EMPTY" % nm); continue
            a, b = r.GetMin(), r.GetMax()
            npts = sum(len(UsdGeom.Mesh(p).GetPointsAttr().Get() or [])
                       for p in st.Traverse() if p.IsA(UsdGeom.Mesh))
            rec = {"name": nm, "kind": kind, "mpu": round(S, 6),
                   "usd": ("GreatAmericanCity/Assets/Game/GreatAmericanCity/"
                           "Meshes/" + nm + ".usd"),
                   "W": round((b[0]-a[0])*S, 2), "D": round((b[1]-a[1])*S, 2),
                   "H": round((b[2]-a[2])*S, 2),
                   "cx": round(0.5*(a[0]+b[0])*S, 3),
                   "cy": round(0.5*(a[1]+b[1])*S, 3),
                   "z0": round(a[2]*S, 3), "points": npts}
            out.append(rec)
            print("%-14s %-30s %6.2f x %6.2f x %6.2f m  base z %6.2f  %6dk pts"
                  % (kind, nm, rec["W"], rec["D"], rec["H"], rec["z0"],
                     npts // 1000), flush=True)
    json.dump(out, open(os.path.normpath(OUT), "w"), indent=1)
    print("\n%d props in %.0f s -> %s" % (len(out), time.time()-t0,
                                          os.path.normpath(OUT)))


if __name__ == "__main__":
    main()
