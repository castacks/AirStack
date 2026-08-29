#!/usr/bin/env python
"""Do the storey cuts miss the windows, and is the ring still an exact partition?"""
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

m = gss.read_mesh(st, "/W/src", verbose=False)
wins, bbox = gsl.window_centres(st, "/W/src")
g = gsl.measure_grid(wins, bbox, verbose=False, name=NAME)
nwin = sum(len(v) for v in wins.values())
print("%s: storey %.2f m, %d window face(s)" % (NAME, g["storey_h"], nwin))

for label, lines in (("ON the window lattice (old)", g["storeys"]),
                     ("SHIFTED half a storey (new)",
                      gss.cut_lines(g, 0.5, verbose=False))):
    worst, hit = gss.window_clearance(lines, wins)
    print("  %-28s %2d line(s)  worst clearance %.3f m  windows cut: %d (%.1f%%)"
          % (label, len(lines), worst, hit, 100.0 * hit / max(1, nwin)))

lines = gss.cut_lines(g, 0.5, verbose=False)
bands = gss.storeys(m, lines, verbose=False)
leg = max(1.5, 0.6 * ((g["bays"].get("E") or {}).get("pitch") or 4.0))
bay = (g["bays"].get("E") or {}).get("pitch") or 4.0
tot, worst_d, roles = 0, 0.0, {}
n_bands = len(bands)
roofed = False
for i, (lo, hi, band) in enumerate(bands):
    bb = ((bbox[0][0], bbox[0][1], lo), (bbox[1][0], bbox[1][1], hi))
    # Same substitution as `ring_verify.py`: the pipeline (`slice_to_kit`)
    # runs the topmost band through `roof_and_parapet`, not a bare `ring()`.
    if i == n_bands - 1:
        cells, roofed = gss.roof_and_parapet(band, bb, leg, bay,
                                             g["storey_h"])
    else:
        cells = gss.ring(band, bb, leg, bay)
    a = sum(area(p) for _r, _s, _b, p in cells)
    worst_d = max(worst_d, 100.0 * abs(a - area(band)) / max(1e-9, area(band)))
    tot += len(cells)
    for r, _s, _b, _p in cells:
        roles[r] = roles.get(r, 0) + 1
ba = sum(area(b) for _l, _h, b in bands)
print("\n%d band(s), %d piece(s): %s" % (
    len(bands), tot, "  ".join("%s=%d" % (k, v) for k, v in sorted(roles.items()))))
print("top band roof/parapet split: %s" % ("OK" if roofed else
      "FELL BACK to an unlabelled ring (rare clip failure)"))
print("band area delta %.4f%% ; worst ring delta %.4f%% -> %s"
      % (100.0 * abs(ba - area(m)) / area(m), worst_d,
         "EXACT" if worst_d < 0.05 else "LEAKY"))
