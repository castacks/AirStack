#!/usr/bin/env python
"""Is a sliced piece wearing the RIGHT material and the RIGHT UVs?

    ISAAC=/isaac-sim; $ISAAC/AirStack/scene_gen/tools/usd_python.sh \
        $ISAAC/AirStack/scene_gen/tools/piece_mat_probe.py \
        --asset SM_Building_02 --piece wall_S_19_04_0263

WHY THIS EXISTS. "That wall looks like it has graffiti — is that accurate?"
is not answerable by looking at it, because there are two completely
different explanations and they render identically from a distance:

  1. the GAC source art really does have graffiti painted into its texture
     at that spot, in which case the slice is FAITHFUL and there is nothing
     to fix; or
  2. the piece is sampling the wrong region of the façade atlas — a UV that
     was not carried through the clip, or a `MID` (material id) that came out
     of `vtkClipPolyData`'s cell data misaligned with the triangles it is
     supposed to describe — in which case the piece is wearing some other
     part of the building's texture and the slice is BROKEN.

THE TEST THAT SEPARATES THEM is not "does the piece have a material" — it
will have one either way. It is whether each of the piece's triangles carries
the SAME material id and the SAME UVs as the source triangle it was cut from.
So for every triangle in the piece we find the nearest source triangle by
centroid and compare. A clip only ever splits a triangle inside its own
plane, so a piece triangle's centroid is always within its parent triangle's
footprint; the nearest source triangle IS the parent, up to the tolerance
reported here.

Reports, per piece: world extent, material bindings with their texture files,
and the two agreement rates. Anything below 100% on the material rate is a
real defect. The UV rate is reported with a tolerance because the clip
legitimately INTERPOLATES uv at a new cut vertex — a cut triangle's centroid
uv is a barycentric blend of the parent's, not one of its corners.
"""
import argparse
import os
import sys

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdShade

_SG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, _SG)

from detail import gac_slice as gsl                       # noqa: E402
from detail import gac_storey_slice as gss                # noqa: E402

NUC = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")

ap = argparse.ArgumentParser()
ap.add_argument("--asset", default="SM_Building_02")
ap.add_argument("--piece", default="wall_S_19_04_0263")
ap.add_argument("--scale", type=float, default=0.01)
ap.add_argument("--uv-tol", type=float, default=0.02)
a = ap.parse_args()

st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Xform.Define(st, "/W/src")
kid = st.DefinePrim("/W/src/asset")
kid.GetReferences().AddReference(NUC + a.asset + ".usd")
st.Load(Sdf.Path("/W/src"))
UsdGeom.Xformable(kid).AddScaleOp().Set((a.scale, a.scale, a.scale))

wins, bbox = gsl.window_centres(st, "/W/src")
g, measured = gss.grid_for(st, "/W/src", bbox, wins, name=a.asset, verbose=True)
src = gss.read_mesh(st, "/W/src", verbose=True)
print("[probe] grid: %s, %d storey line(s)" % (
    "MEASURED" if measured else "regular", len(g["storeys"])))

# reproduce slice_to_kit's cell enumeration exactly
lines = gss.cut_lines(g, 0.5, verbose=False)
bands = gss.storeys(src, lines, verbose=False)
leg = max(1.2, 0.6 * ((g["bays"].get("E") or {}).get("pitch") or 3.5))
bay = (g["bays"].get("E") or {}).get("pitch") or 3.5
cells = []
for i, (lo, hi, band) in enumerate(bands):
    bb = ((bbox[0][0], bbox[0][1], lo), (bbox[1][0], bbox[1][1], hi))
    for role, side, k, piece in gss.ring(band, bb, leg, bay):
        cells.append((i, role, side, k, piece))

want = None
for j, (storey, role, side, k, piece) in enumerate(cells):
    nm = "{0}_{1}_{2}_{3:02d}_{4:04d}".format(
        role, side.replace("-", "x"), k, storey, j)
    if nm == a.piece:
        want = (j, storey, role, side, k, piece)
        break
if want is None:
    print("[probe] NO SUCH PIECE %r. %d cell(s) produced." % (a.piece, len(cells)))
    print("[probe] first 12: %s" % ", ".join(
        "{0}_{1}_{2}_{3:02d}_{4:04d}".format(r, s.replace("-", "x"), k, st_, j)
        for j, (st_, r, s, k, _p) in enumerate(cells[:12])))
    raise SystemExit(2)

j, storey, role, side, k, piece = want
P, UV, MID = piece["P"], piece["UV"], piece["MID"]
mn, mx = P.min(axis=0), P.max(axis=0)
print("\n" + "=" * 72)
print("PIECE %s   (index %d)" % (a.piece, j))
print("  role=%s side=%s bay=%s storey=%d" % (role, side, k, storey))
print("  %d triangle(s)" % len(piece["tris"]))
print("  local extent  x %.2f..%.2f   y %.2f..%.2f   z %.2f..%.2f m"
      % (mn[0], mx[0], mn[1], mx[1], mn[2], mx[2]))
print("  height above the building's own base: %.2f .. %.2f m"
      % (mn[2] - bbox[0][2], mx[2] - bbox[0][2]))
print("  uv range      u %.3f..%.3f   v %.3f..%.3f"
      % (UV[:, 0].min(), UV[:, 0].max(), UV[:, 1].min(), UV[:, 1].max()))


def tex_of(mat):
    if mat is None or not mat.GetPrim().IsValid():
        return "(none)"
    outs = []
    for c in Usd.PrimRange(mat.GetPrim()):
        sh = UsdShade.Shader(c)
        if not sh:
            continue
        for nm_ in ("diffuse_texture", "file", "inputs:diffuse_texture"):
            i = sh.GetInput(nm_.split(":")[-1])
            v = i.Get() if i is not None else None
            if isinstance(v, Sdf.AssetPath) and v.path:
                outs.append(v.path.rsplit("/", 1)[-1])
    return ", ".join(sorted(set(outs))) or "(no texture input found)"


print("\n  MATERIALS ON THIS PIECE")
for mi in sorted(set(int(q) for q in MID)):
    n = int((MID == mi).sum())
    m = src["mats"][mi] if mi < len(src["mats"]) else None
    nm_ = str(m.GetPrim().GetPath()).rsplit("/", 1)[-1] if m and m.GetPrim().IsValid() else "(none)"
    print("    id %-3d %5d tri (%5.1f%%)  %-34s %s"
          % (mi, n, 100.0 * n / len(MID), nm_, tex_of(m)))

# ---- the actual tests -----------------------------------------------------
# NEAREST-CENTROID MATCHING DOES NOT WORK HERE and the first version of this
# probe was wrong because of it. GAC walls are modelled as a handful of very
# large triangles, so a clipped fragment's centroid sits up to 1.3 m from its
# own parent's centroid and a DIFFERENT triangle is frequently nearer. That
# reported 78% material agreement on a piece that is in fact faithful. Both
# tests below are independent of triangle size.
sP, sUV, sMID = src["P"], src["UV"], src["MID"]


def tri_area(Q):
    return 0.5 * np.linalg.norm(np.cross(Q[:, 1] - Q[:, 0], Q[:, 2] - Q[:, 0]),
                                axis=1)


# TEST 1 - MATERIAL MIX BY AREA, piece vs the source inside the same box.
# A faithful slice reproduces the proportions of the region it was cut from.
lo, hi = mn - 0.01, mx + 0.01
sT = sP.reshape(-1, 3, 3)
sC = sT.mean(axis=1)
inside = np.all((sC >= lo) & (sC <= hi), axis=1)
sA, pA = tri_area(sT), tri_area(P[piece["tris"]])
print("\n  TEST 1  material mix by AREA, piece vs source in the same box")
print("    %d source triangle(s) have their centroid in this box" % int(inside.sum()))
ids = sorted(set(int(q) for q in MID) | set(int(q) for q in sMID[inside]))
print("    %-5s %>18s %>18s" .replace(">", "") % ("id", "piece area %", "source area %"))
worst = 0.0
for mi in ids:
    pa = float(pA[MID == mi].sum()) / max(1e-9, pA.sum()) * 100.0
    sa = float(sA[inside][sMID[inside] == mi].sum()) / max(1e-9, sA[inside].sum()) * 100.0
    worst = max(worst, abs(pa - sa))
    m = src["mats"][mi] if mi < len(src["mats"]) else None
    print("    %-5d %17.1f%% %17.1f%%   %s" % (mi, pa, sa, tex_of(m).split(",")[0]))
print("    worst share difference: %.1f percentage point(s)" % worst)

# TEST 2 - UV AT COINCIDENT VERTICES. A vertex the clip did not move must
# carry EXACTLY the uv it had in the source. A vertex the clip created lies on
# a source edge and its uv is a legitimate interpolation, so only coincident
# vertices are a valid test - and they are the decisive one, because a uv that
# was never carried would fail here by a wide margin.
uP = np.unique(np.round(P, 5), axis=0)
kept = 0
bad_uv = []
for q in uP:
    d = np.linalg.norm(sP - q, axis=1)
    i = int(d.argmin())
    if d[i] > 1e-4:
        continue
    kept += 1
    j2 = int(np.linalg.norm(P - q, axis=1).argmin())
    e = float(np.linalg.norm(UV[j2] - sUV[i]))
    if e > 1e-4:
        bad_uv.append((q, e, UV[j2], sUV[i]))
print("\n  TEST 2  uv at vertices the clip did NOT move")
print("    %d of %d unique piece vertices coincide with a source vertex"
      % (kept, len(uP)))
if kept:
    print("    uv identical at %d of them; %d differ"
          % (kept - len(bad_uv), len(bad_uv)))
    for q, e, a_, b_ in bad_uv[:5]:
        print("      at (%.2f %.2f %.2f)  piece uv (%.4f %.4f) vs source (%.4f %.4f)  d=%.4f"
              % (q[0], q[1], q[2], a_[0], a_[1], b_[0], b_[1], e))
else:
    print("    none - every vertex of this piece was created by a cut")
print("=" * 72)

# TEST 3 - UV RECONSTRUCTED FROM THE PARENT TRIANGLE'S OWN AFFINE MAP.
# Every vertex of a small piece can be cut-created, which makes TEST 2 mute
# (it was, on wall_S_19_04_0263: 0 of 34 coincident). This is the general
# test. A source triangle carries an affine map from its plane to uv;
# `vtkClipPolyData` interpolates uv barycentrically at each new vertex, so a
# correctly carried uv is EXACTLY what that map predicts.
#
# THE PARENT MUST BE DISAMBIGUATED BY MATERIAL. GAC models its posters and
# signs (`M_Images`, which carries an OpacityMask and an emissive) as decal
# planes lying ON the wall plane, so a point on the wall is inside BOTH the
# wall triangle and the decal triangle to within any sane tolerance. A first
# draft of this test took whichever candidate came first and reported a
# spurious 0.058 uv error on a piece that is exact. Restricting candidates to
# the triangle's own material removes the ambiguity.
print("\n  TEST 3  uv vs the parent triangle's own affine position->uv map")
sMin, sMax = sT.min(axis=1), sT.max(axis=1)
cand = np.where(np.all((sMax >= lo) & (sMin <= hi), axis=1))[0]
print("    %d source triangle(s) overlap this piece's box" % len(cand))


def bary(A, B, C, q):
    n = np.cross(B - A, C - A)
    ln = np.linalg.norm(n)
    if ln < 1e-12:
        return None
    if abs(float(np.dot(q - A, n / ln))) > 1e-3:
        return None
    v0, v1, v2 = B - A, C - A, q - A
    d00, d01, d11 = np.dot(v0, v0), np.dot(v0, v1), np.dot(v1, v1)
    den = d00 * d11 - d01 * d01
    if abs(den) < 1e-12:
        return None
    d20, d21 = np.dot(v2, v0), np.dot(v2, v1)
    b1 = (d11 * d20 - d01 * d21) / den
    b2 = (d00 * d21 - d01 * d20) / den
    b0 = 1.0 - b1 - b2
    return None if min(b0, b1, b2) < -1e-3 else (b0, b1, b2)


# ALL THREE CORNERS MUST COME FROM ONE PARENT. A clip never splits a
# triangle across two source triangles, so the parent is the one that
# contains the whole piece triangle. Requiring that removes the remaining
# ambiguity where two triangles of the SAME material are coplanar and
# overlapping - which GAC does, because a wall carries several separate
# poster quads on one plane, each with its own uv island.
errs, matched, unlocated, ambig = [], 0, 0, 0
by_mat = {}
for ti_p, tri in enumerate(piece["tris"]):
    mine = int(MID[ti_p])
    pool = []
    for t in cand:
        if int(sMID[t]) != mine:
            continue
        bs = [bary(sT[t][0], sT[t][1], sT[t][2], P[vi]) for vi in tri]
        if all(b is not None for b in bs):
            pool.append((t, bs))
    if not pool:
        unlocated += 3
        continue
    if len(pool) > 1:
        ambig += 1
    t, bs = pool[0]
    for vi, (b0, b1, b2) in zip(tri, bs):
        pred = b0 * sUV[3 * t] + b1 * sUV[3 * t + 1] + b2 * sUV[3 * t + 2]
        errs.append(float(np.linalg.norm(pred - UV[vi])))
        by_mat.setdefault(mine, []).append(errs[-1])
        matched += 1
if matched:
    e = np.asarray(errs)
    print("    %d face-corner(s) located on their parent triangle, %d not "
          "located; %d piece triangle(s) had >1 candidate parent"
          % (matched, unlocated, ambig))
    print("    uv error vs the map: median %.6f, max %.6f  (uv is 0..1)"
          % (float(np.median(e)), float(e.max())))
    print("    %d of %d corner(s) are wrong by more than 1e-3"
          % (int((e > 1e-3).sum()), len(e)))
    for mi in sorted(by_mat):
        v = np.asarray(by_mat[mi])
        m = src["mats"][mi] if mi < len(src["mats"]) else None
        print("      id %-3d %4d corner(s), %3d wrong, max %.6f   %s"
              % (mi, len(v), int((v > 1e-3).sum()), v.max(),
                 tex_of(m).split(",")[0]))
    # POINT MERGING IS THE SUSPECT. `vtkClipPolyData` inserts output points
    # through a `vtkMergePoints` locator, so two corners at the SAME position
    # collapse into one point and keep whichever uv was inserted first. The
    # input to `_to_vtk` is de-indexed (3 points per triangle, nothing
    # shared), so any shortfall here is the clip merging them.
    print("    point merging: %d point(s) for %d face corner(s) (%.2fx)"
          % (len(P), 3 * len(piece["tris"]),
             3.0 * len(piece["tris"]) / max(1, len(P))))
    print("    %s" % ("UV CARRIED CORRECTLY" if e.max() < 1e-3
                      else "UV DAMAGED - see the merge ratio above"))
else:
    print("    could not locate any face corner on a source triangle")
print("=" * 72)
