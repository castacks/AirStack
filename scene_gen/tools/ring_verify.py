#!/usr/bin/env python
"""Is the storey+ring partition EXACT? No dropped geometry, no duplicates.

The 3x3 plan partition is only trustworthy if the pieces add back up to the
band. Two failure modes, both invisible in a render of the assembled result:
  * a GAP drops geometry silently — the reassembly still looks fine from
    outside because the missing bit was interior;
  * an OVERLAP duplicates it into two pieces, which doubles the triangle count
    and puts coincident surfaces in the scene (z-fighting once damaged).
Area is the honest measure: the sum of the pieces' triangle area must equal
the band's, because clipping conserves area exactly.
"""
import os, sys
import numpy as np
from pxr import Sdf, Usd, UsdGeom
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from detail import gac_storey_slice as gss
from detail import gac_slice as gsl

SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
NAME = os.environ.get("GV_NAME", "SM_Building_01")


def area(m):
    if m is None or not len(m["tris"]):
        return 0.0
    V = m["P"][m["tris"]]
    return float(np.linalg.norm(np.cross(V[:, 1] - V[:, 0],
                                         V[:, 2] - V[:, 0]), axis=1).sum() * 0.5)


st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Xform.Define(st, "/W/src")
kid = st.DefinePrim("/W/src/asset")
kid.GetReferences().AddReference(SEI + NAME + ".usd")
st.Load(Sdf.Path("/W/src"))
UsdGeom.Xformable(kid).AddScaleOp().Set((0.01, 0.01, 0.01))

m = gss.read_mesh(st, "/W/src")
wins, bbox = gsl.window_centres(st, "/W/src")
g = gsl.measure_grid(wins, bbox, name=NAME)
bands = gss.storeys(m, g["storeys"], verbose=False)
src_area = area(m)
band_area = sum(area(b) for _lo, _hi, b in bands)
print("source area %.1f m2 ; %d band(s) total %.1f m2 ; delta %.4f%%"
      % (src_area, len(bands), band_area,
         100.0 * abs(band_area - src_area) / max(1e-9, src_area)))

leg = max(1.5, 0.6 * (g["bays"].get("E") or {}).get("pitch", 4.0))
bay = (g["bays"].get("E") or {}).get("pitch", 4.0)
print("\nring: leg %.2f m, bay %.2f m" % (leg, bay))
tot_pieces = 0
worst = 0.0
roles = {}
for i, (lo, hi, band) in enumerate(bands):
    cells = gss.ring(band, ((bbox[0][0], bbox[0][1], lo),
                            (bbox[1][0], bbox[1][1], hi)), leg, bay)
    a = sum(area(p) for _r, _s, _b, p in cells)
    d = 100.0 * abs(a - area(band)) / max(1e-9, area(band))
    worst = max(worst, d)
    tot_pieces += len(cells)
    for r, _s, _b, _p in cells:
        roles[r] = roles.get(r, 0) + 1
    if i < 3:
        print("  band %2d: %3d cell(s), area %.1f vs %.1f m2 (%.4f%%)"
              % (i, len(cells), a, area(band), d))
print("\n%d piece(s) total: %s" % (tot_pieces,
      "  ".join("%s=%d" % (k, v) for k, v in sorted(roles.items()))))
print("WORST band area delta: %.4f%%  -> %s" % (
    worst, "EXACT partition" if worst < 0.05 else "LEAKY (gap or overlap)"))
