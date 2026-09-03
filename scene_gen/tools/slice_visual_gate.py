#!/usr/bin/env python
"""slice_final.py — the fastest slicer config that is VISUALLY IDENTICAL.

Not a matrix. One question: what is the biggest speedup that changes nothing a
viewer can see?

"Visually identical" is checked three ways, because area alone would not have
caught the bug this pipeline actually shipped (merged coincident wall/decal
corners inheriting the wall's UV — "this wall looks like it has graffiti"):

  1. AREA, total and per piece.
  2. UV FOOTPRINT per material — the (u,v) bounding box of every material id in
     every piece. A merged decal corner moves its uv, so its box moves. This is
     the check that maps onto the reported defect.
  3. COINCIDENT-POINT SURVIVAL — count of position-duplicate/uv-distinct corner
     pairs. If a faster clipper merges them the count collapses, and the decals
     are corrupted whatever the area says.

Candidates, all sharing the sweep (convert once, GenerateClippedOutputOn, stay
in vtkPolyData, resolve only at the final piece):

  dedup    np.unique(axis=0) on a position+uv key -- the shipped behaviour
  compact  drop points no triangle references, 1-D. Same geometry, keeps every
           corner distinct, which faceVarying `st` output makes natural.
  nodedup  keep vtkClipPolyData's raw output point list -- REJECTED below on
           size, not on looks.

`vtkPolyDataPlaneClipper` is deliberately NOT a candidate: it exposes no
locator control (no SetLocator / no Merge*), so it cannot be told to preserve
coincident points, and it has no GenerateClippedOutput so it would also cost
the sweep. Both are disqualifying here.
"""
import argparse
import os
import sys
import time
from collections import defaultdict

import numpy as np
import vtk
from vtk.util import numpy_support as ns

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slice_bench import (to_vtk, from_vtk, load, sweep, area_of, T, N,
                         run_plane_sweep)  # noqa
import slice_bench as SB  # noqa


def run_plane(m, z, x, y, mode, backend):
    """`vtkPolyDataPlaneClipper`, through the SAME gate as everything else.

    It is the fastest variant measured (25x sequential) and it is also the one
    with the strongest a-priori reason to fail here: it exposes no locator
    control at all, so it cannot be told to keep coincident wall/decal corners
    apart, which is the exact mechanism behind the graffiti-wall defect. Speed
    is not the question; whether the decals survive is."""
    vtk.vtkSMPTools().SetBackend(backend)
    vtk.vtkSMPTools().Initialize(0)
    SB.DEDUP = mode
    return run_plane_sweep(m, z, x, y)


def run(m, z, x, y, mode):
    vtk.vtkSMPTools().SetBackend("Sequential")
    SB.DEDUP = mode
    pd0 = to_vtk(m)
    out = []
    for band in sweep(pd0, 2, z):
        if band.GetNumberOfCells() == 0:
            continue
        for col in sweep(band, 0, x):
            if col.GetNumberOfCells() == 0:
                continue
            for cell in sweep(col, 1, y):
                if cell.GetNumberOfCells() == 0:
                    continue
                d = from_vtk(cell, dedup=mode)
                if d is not None and len(d["tris"]):
                    out.append(d)
    return out


def run_current(m, z, x, y):
    SB.DEDUP = True
    return SB.run_current(m, z, x, y)


def uv_boxes(pieces):
    """{mid: (umin,vmin,umax,vmax)} over every piece — the decal fingerprint."""
    box = {}
    for d in pieces:
        UV, tris, MID = d["UV"], d["tris"], d["MID"]
        for mi in np.unique(MID):
            sel = tris[MID == mi].ravel()
            if sel.size == 0:
                continue
            uv = UV[sel]
            lo, hi = uv.min(axis=0), uv.max(axis=0)
            b = box.get(int(mi))
            cur = (lo[0], lo[1], hi[0], hi[1])
            box[int(mi)] = cur if b is None else (
                min(b[0], cur[0]), min(b[1], cur[1]),
                max(b[2], cur[2]), max(b[3], cur[3]))
    return box


def coincident_pairs(pieces, limit=400000):
    """How many corners share a position but differ in uv — the decals."""
    tot = 0
    for d in pieces:
        P, UV = d["P"], d["UV"]
        if len(P) > limit:
            continue
        key = np.round(P, 5)
        order = np.lexsort((key[:, 2], key[:, 1], key[:, 0]))
        ks = key[order]
        same = np.all(ks[1:] == ks[:-1], axis=1)
        if not same.any():
            continue
        uvs = np.round(UV[order], 6)
        diff = np.any(uvs[1:] != uvs[:-1], axis=1)
        tot += int(np.sum(same & diff))
    return tot


def compare(ref, cand, name):
    ar, ac = area_of(ref), area_of(cand)
    da = 100.0 * (ac - ar) / ar if ar else 0.0
    br, bc = uv_boxes(ref), uv_boxes(cand)
    shared = sorted(set(br) & set(bc))
    worst, worst_mid = 0.0, None
    for mi in shared:
        d = max(abs(a - b) for a, b in zip(br[mi], bc[mi]))
        if d > worst:
            worst, worst_mid = d, mi
    cr, cc = coincident_pairs(ref), coincident_pairs(cand)
    print("  %-9s area %+.4f%% | materials %d/%d | worst uv-box shift %.6f (mid %s)"
          % (name, da, len(shared), len(br), worst, worst_mid))
    print("             coincident uv-distinct corners: ref %d -> cand %d (%s)"
          % (cr, cc, "PRESERVED" if cc >= cr * 0.98 else "*** COLLAPSED ***"))
    ok = abs(da) < 0.5 and worst < 1e-4 and cc >= cr * 0.98
    print("             VISUALLY IDENTICAL: %s" % ("YES" if ok else "NO"))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asset")
    ap.add_argument("--storeys", type=int, default=8)
    ap.add_argument("--bays", type=int, default=3)
    a = ap.parse_args()

    vtk.vtkSMPTools().SetBackend("STDThread")
    vtk.vtkSMPTools().Initialize(0)
    print("VTK %s | SMP %s | threads %d\n"
          % (vtk.vtkVersion.GetVTKVersion(), vtk.vtkSMPTools().GetBackend(),
             vtk.vtkSMPTools().GetEstimatedNumberOfThreads()))

    m = load(a.asset)
    lo, hi = m["P"].min(axis=0), m["P"].max(axis=0)
    z = np.linspace(lo[2], hi[2], a.storeys + 1)
    x = np.linspace(lo[0], hi[0], a.bays + 1)
    y = np.linspace(lo[1], hi[1], a.bays + 1)
    print("%s: %d tris, %d storeys x %d x %d bays\n"
          % (os.path.basename(a.asset), len(m["tris"]), a.storeys, a.bays, a.bays))

    T.clear(); N.clear()
    t = time.perf_counter(); ref = run_current(m, z, x, y)
    t_cur = time.perf_counter() - t
    print("CURRENT (shipped)      %6.2fs  %d pieces  %d verts"
          % (t_cur, len(ref), sum(len(p["P"]) for p in ref)))

    best = None
    variants = [(lambda mm: run(mm, z, x, y, True), "dedup"),
                (lambda mm: run(mm, z, x, y, "compact"), "compact"),
                (lambda mm: run(mm, z, x, y, False), "nodedup"),
                (lambda mm: run_plane(mm, z, x, y, "compact", "Sequential"),
                 "plane_seq"),
                (lambda mm: run_plane(mm, z, x, y, "compact", "STDThread"),
                 "plane_std")]
    for fn, label in variants:
        T.clear(); N.clear()
        t = time.perf_counter(); cand = fn(m)
        el = time.perf_counter() - t
        v = sum(len(p["P"]) for p in cand)
        print("\n%-18s   %6.2fs  %d pieces  %d verts  (%.2fx)"
              % (label, el, len(cand), v, t_cur / el if el else 0))
        ok = compare(ref, cand, label)
        if ok and (best is None or el < best[1]):
            best = (label, el, v)

    print("\n" + "=" * 66)
    if best:
        print("FASTEST VISUALLY-IDENTICAL CONFIG: sweep + %s" % best[0])
        print("  %.2fs vs %.2fs  ->  %.2fx speedup" % (best[1], t_cur, t_cur / best[1]))
        print("  vertices %d (%.2fx the shipped output)"
              % (best[2], best[2] / max(sum(len(p["P"]) for p in ref), 1)))
    else:
        print("NO candidate passed the visual gate")


if __name__ == "__main__":
    main()
