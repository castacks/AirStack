#!/usr/bin/env python
"""Measure every CitySample façade module into `_plans/citysample_kit.json`.

WHY A CATALOGUE AND NOT A RUNTIME MEASUREMENT. The modules are Nanite source
meshes — a single CHC ground-floor entrance is 731k points — so opening all
597 of them to find out how wide a bay is would dominate a scene build. They
are measured once here and the launcher reads boxes out of the JSON.

WHY THE MODULES HAVE TO BE STACKED BY US AT ALL: see the module docstring of
`detail/citysample_building.py`. In short, `All_Buildings_Lineup_Hero.usd`
holds the buildings' modules correctly GROUPED but not PLACED — every
`InstancedStaticMesh` prim is exactly one copy of its source module (measured
ratio 1.00) sharing one component transform, because Unreal's per-instance
transform arrays did not survive the USD export.

    bash scene_gen/tools/usd_python.sh scene_gen/tools/citysample_catalogue.py
"""

import json
import os
import re
import time

import omni.client
from pxr import Usd, UsdGeom

NUC = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
ROOT = NUC + "CitySample/Assets/Game/CitySampleBuildings/Building/"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "_plans", "citysample_kit.json")
KIND = re.compile(r"^SM_BLDG_[A-Za-z0-9]+_L\d+_[A-Za-z0-9]+_"
                  r"(Wall|Column|CornerExL|CornerExR|CornerEx|CornerIn|"
                  r"Entrance|Rotunda)_(\d+)_N\d+\.usd$", re.I)


def _dirs(url):
    res, ents = omni.client.list(url)
    if res != omni.client.Result.OK:
        return []
    return sorted(e.relative_path for e in ents
                  if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN)


def main():
    t0 = time.time()
    out = {}
    n_mod = 0
    for city in _dirs(ROOT):
        if city == "Library":
            continue
        for fam in _dirs(ROOT + city + "/"):
            for kit in _dirs(ROOT + city + "/" + fam + "/"):
                m = re.search(r"_L(\d+)_([A-Za-z0-9]+)$", kit)
                if not m:
                    continue
                lvl, var = int(m.group(1)), m.group(2).upper()
                base = ROOT + city + "/" + fam + "/" + kit + "/Mesh/"
                res, ents = omni.client.list(base)
                if res != omni.client.Result.OK:
                    continue
                mods = []
                for e in sorted(ents, key=lambda q: q.relative_path):
                    n = e.relative_path
                    if not n.lower().endswith(".usd"):
                        continue
                    km = KIND.match(n)
                    kind = km.group(1) if km else "Other"
                    try:
                        s = Usd.Stage.Open(base + n)
                        s.Load()
                        S = UsdGeom.GetStageMetersPerUnit(s)
                        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                               [UsdGeom.Tokens.default_])
                        r = bc.ComputeWorldBound(
                            s.GetPseudoRoot()).ComputeAlignedRange()
                        if r.IsEmpty():
                            continue
                        a, b = r.GetMin(), r.GetMax()
                        npt = sum(len(UsdGeom.Mesh(q).GetPointsAttr().Get()
                                      or [])
                                  for q in s.Traverse() if q.IsA(UsdGeom.Mesh))
                    except Exception:
                        continue
                    mods.append({
                        "usd": (base + n)[len(NUC):], "kind": kind,
                        "x0": round(a[0] * S, 3), "y0": round(a[1] * S, 3),
                        "z0": round(a[2] * S, 3), "x1": round(b[0] * S, 3),
                        "y1": round(b[1] * S, 3), "z1": round(b[2] * S, 3),
                        # THE MODULE'S OWN metersPerUnit, carried so the
                        # placement can undo it. The boxes above are already
                        # in metres; the GEOMETRY is not, and referencing a
                        # cm asset into a metres stage converts nothing -- a
                        # 3.25 m bay arrives 325 m wide.
                        "mpu": round(S, 6), "points": npt})
                    n_mod += 1
                if mods:
                    out.setdefault(city + fam, {}).setdefault(
                        var, {})[str(lvl)] = mods
                    print("%-6s %-3s L%-4s %2d modules  %8d pts  (%.0fs)"
                          % (city + fam, var, lvl, len(mods),
                             sum(q["points"] for q in mods),
                             time.time() - t0), flush=True)
    json.dump(out, open(os.path.normpath(OUT), "w"), indent=1)
    fams = sum(len(v) for v in out.values())
    print("\n%d families, %d family-variants, %d modules in %.0f s -> %s"
          % (len(out), fams, n_mod, time.time() - t0, os.path.normpath(OUT)))


if __name__ == "__main__":
    main()
