#!/usr/bin/env python
"""What IS a given catalogue piece? Resolve /World/cat/aNN/assembled/pNNNN.

The catalogue names pieces by their index into the cell list, so `p0130` says
nothing about what it is. Reproduce the same slice offline and report the
cell's role/side/storey/bay and its measurements, so a blacklist can name the
THING rather than an ordinal that moves the next time the grid changes.
"""
import os, sys
import numpy as np
from pxr import Sdf, Usd, UsdGeom
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from detail import gac_storey_slice as gss
from detail import gac_slice as gsl

AEC = ("/isaac-sim/AirStack/scene_gen/assets/aec/brownstone/Assets/"
       "Create_Brownstone02/")
NAME = os.environ.get("IP_NAME", "Reference_Brownstone02")
USD = os.environ.get("IP_USD", AEC + NAME + ".usd")
WANT = int(os.environ.get("IP_INDEX", "130"))
FALLBACK = float(os.environ.get("IP_FALLBACK_H", "3.95"))

st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Xform.Define(st, "/W/src")
kid = st.DefinePrim("/W/src/asset")
kid.GetReferences().AddReference(USD)
st.Load(Sdf.Path("/W/src"))
UsdGeom.Xformable(kid).AddScaleOp().Set((0.01, 0.01, 0.01))

wins, bbox = gsl.window_centres(st, "/W/src")
g, measured = gss.grid_for(st, "/W/src", bbox, wins, name=NAME,
                           target=FALLBACK, verbose=False)
m = gss.read_mesh(st, "/W/src", verbose=False)
lines = gss.cut_lines(g, 0.5, verbose=False)
bands = gss.storeys(m, lines, verbose=False)
leg = max(1.2, 0.6 * ((g["bays"].get("E") or {}).get("pitch") or 3.5))
bay = (g["bays"].get("E") or {}).get("pitch") or 3.5
cells = []
for i, (lo, hi, band) in enumerate(bands):
    bb = ((bbox[0][0], bbox[0][1], lo), (bbox[1][0], bbox[1][1], hi))
    for role, side, k, piece in gss.ring(band, bb, leg, bay):
        cells.append((i, role, side, k, piece))

print("%s: %d cell(s), grid %s storey %.2f m, leg %.2f, bay %.2f"
      % (NAME, len(cells), "measured" if measured else "regular",
         g["storey_h"], leg, bay))
if WANT >= len(cells):
    print("index %d is out of range" % WANT)
    raise SystemExit(0)
i, role, side, k, piece = cells[WANT]
P = piece["P"]
sz = P.max(axis=0) - P.min(axis=0)
V = P[piece["tris"]]
area = float(np.linalg.norm(np.cross(V[:, 1] - V[:, 0],
                                     V[:, 2] - V[:, 0]), axis=1).sum() * 0.5)
print("\np%04d  ->  role=%s  side=%s  storey=%d  bay=%d" % (WANT, role, side, i, k))
print("   size  %.2f x %.2f x %.2f m   z %.2f..%.2f" % (
    sz[0], sz[1], sz[2], P.min(axis=0)[2], P.max(axis=0)[2]))
print("   tris  %d   area %.2f m2   materials %s" % (
    len(piece["tris"]), area, sorted(set(int(q) for q in piece["MID"]))[:8]))
# context: how does it compare to its siblings?
areas = []
for j, (_i, r, _s, _k, p) in enumerate(cells):
    Vv = p["P"][p["tris"]]
    areas.append(float(np.linalg.norm(np.cross(Vv[:, 1] - Vv[:, 0],
                                               Vv[:, 2] - Vv[:, 0]), axis=1).sum() * 0.5))
areas = np.array(areas)
print("   rank by area: %d of %d (median %.2f m2, max %.2f m2)"
      % (int((areas > area).sum()) + 1, len(areas), np.median(areas), areas.max()))
by_role = {}
for (_i, r, _s, _k, _p), a in zip(cells, areas):
    by_role.setdefault(r, []).append(a)
print("\n   cells by role: %s" % "  ".join(
    "%s=%d" % (r, len(v)) for r, v in sorted(by_role.items())))
