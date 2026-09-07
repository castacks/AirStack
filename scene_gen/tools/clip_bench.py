#!/usr/bin/env python3
"""clip_bench — is VTK's clipper faster than ours for Voronoi fracture?

    AirStack/.venv/bin/python scene_gen/tools/clip_bench.py

WHAT IS ACTUALLY BEING TIMED
----------------------------
`mesh_damage._voronoi_cells` builds each fragment by clipping the WHOLE soup
against up to `neighbors` bisector planes — so the cost is roughly
`cells x faces`, not `faces`. Both backends here do exactly that, on the same
mesh, with the same seeds and the same plane sequence, so the comparison is of
the CLIPPER and nothing else.

    ours   `mesh_damage._clip_by_plane` — Sutherland-Hodgman per triangle,
           vectorised in numpy, compacting after every cut.
    vtk    `vtkClipPolyData` with a `vtkPlane`, kept in VTK form across the
           whole plane sequence so the conversion is paid once per cell rather
           than once per clip.

WHAT THIS DOES NOT MEASURE, AND WHY IT MATTERS
----------------------------------------------
Our clipper carries UVs and per-face material ids through every cut and
interpolates them at each crossing; the archetypes depend on that, since a
fragment keeps the facade texture it was cut from. `vtkClipPolyData` can carry
point/cell data too, but this benchmark runs it positions-only — so any VTK
win here is an UPPER BOUND on the real one, and the honest reading is "is it
worth the port", not "we get this speedup".
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def box_shell(nx: int, ny: int, nz: int, size=(21.0, 13.0, 15.0)):
    """A subdivided closed box — the shape the cutter actually gets fed.

    Brownstone-sized by default, because that is the asset the bake is slow
    on. Face count is what drives the clip, so the tessellation is the knob.
    """
    sx, sy, sz = size
    verts, faces = [], []

    def grid(u, v, fn):
        base = len(verts)
        for i in range(u + 1):
            for j in range(v + 1):
                verts.append(fn(i / u, j / v))
        for i in range(u):
            for j in range(v):
                a = base + i * (v + 1) + j
                b = a + 1
                c = a + (v + 1)
                d = c + 1
                faces.append((a, c, b))
                faces.append((b, c, d))

    grid(nx, ny, lambda s, t: (s * sx, t * sy, 0.0))
    grid(nx, ny, lambda s, t: (s * sx, t * sy, sz))
    grid(nx, nz, lambda s, t: (s * sx, 0.0, t * sz))
    grid(nx, nz, lambda s, t: (s * sx, sy, t * sz))
    grid(ny, nz, lambda s, t: (0.0, s * sy, t * sz))
    grid(ny, nz, lambda s, t: (sx, s * sy, t * sz))
    return (np.asarray(verts, dtype=np.float64),
            np.asarray(faces, dtype=np.int64))


def seeds_for(verts, n, rng):
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    return lo + rng.random((n, 3)) * (hi - lo)


def plane_order(seeds, neighbors):
    d2 = np.sum((seeds[:, None, :] - seeds[None, :, :]) ** 2, axis=-1)
    np.fill_diagonal(d2, np.inf)
    return np.argsort(d2, axis=1)[:, :neighbors]


def run_ours(verts, faces, seeds, order):
    from disaster.mesh_damage import _clip_by_plane
    kept = 0
    for i in range(len(seeds)):
        p = seeds[i]
        cv, cf = verts, faces
        for j in order[i]:
            if len(cf) == 0:
                break
            q = seeds[j]
            gap = float(np.linalg.norm(q - p))
            if gap < 1e-9:
                continue
            if gap > 2.0 * float(np.linalg.norm(cv - p, axis=1).max()):
                break
            cv, cf, _u, _m = _clip_by_plane(cv, cf, q - p, (p + q) * 0.5,
                                            None, None, cap=True)
        kept += len(cf)
    return kept


def _to_vtk(verts, faces):
    import vtk
    from vtk.util import numpy_support as ns
    pts = vtk.vtkPoints()
    pts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(verts), deep=1))
    cells = vtk.vtkCellArray()
    quad = np.empty((len(faces), 4), dtype=np.int64)
    quad[:, 0] = 3
    quad[:, 1:] = faces
    cells.SetCells(len(faces),
                   ns.numpy_to_vtkIdTypeArray(quad.ravel(), deep=1))
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    pd.SetPolys(cells)
    return pd


def vtk_prepare(pd):
    """Weld, orient, triangulate — the input contract `vtkClipClosedSurface` has
    and our clipper does not.

    Ours works on an unwelded triangle SOUP and uses signed distance, so
    duplicated seam vertices and inside-out faces pass through unharmed — which
    is deliberate, because the packs are full of both. VTK's capping needs a
    genuinely closed, consistently-wound manifold: without this the caps never
    close, and the result LOOKS like a 9x speedup because it is skipping the
    capping work entirely.

    Measured on a 10 m box clipped in half (want volume 500, 0 boundary edges):

        raw                          333.3   86 boundary edges
        welded                       274.2   13
        welded + consistent normals  500.0    0

    Paid ONCE per mesh, not per cell, so it amortises over every fragment.
    """
    import vtk
    c = vtk.vtkCleanPolyData()
    c.SetInputData(pd); c.SetTolerance(1e-8); c.PointMergingOn(); c.Update()
    n = vtk.vtkPolyDataNormals()
    n.SetInputData(c.GetOutput())
    n.ConsistencyOn(); n.AutoOrientNormalsOn(); n.SplittingOff(); n.Update()
    t = vtk.vtkTriangleFilter()
    t.SetInputData(n.GetOutput()); t.Update()
    return t.GetOutput()


def run_vtk(verts, faces, seeds, order):
    """`vtkClipClosedSurface` with the cell's planes as ONE collection.

    NOT `vtkClipPolyData`. That was the first attempt and it flattered VTK by
    a factor of ten: it leaves the cut OPEN, so it was producing shells while
    ours produced closed solids (`cap=True`). A fragment has to be closed —
    PhysX needs a solid for its convex decomposition and an open cell reads as
    a hole in the wreck — so the capping filter is the only honest comparison.

    Passing the whole plane collection at once is also how you would really
    write it: one filter call per CELL instead of one per plane, which is
    VTK's structural advantage over our per-plane Python loop.
    """
    import vtk
    src = vtk_prepare(_to_vtk(verts, faces))

    kept = 0
    for i in range(len(seeds)):
        p = seeds[i]
        planes = vtk.vtkPlaneCollection()
        for j in order[i]:
            q = seeds[j]
            nrm = q - p
            gap = float(np.linalg.norm(nrm))
            if gap < 1e-9:
                continue
            pl = vtk.vtkPlane()
            pl.SetOrigin(*((p + q) * 0.5))
            # ClipClosedSurface KEEPS the positive side, ours keeps the
            # negative — so the normal is flipped to cut the same half.
            pl.SetNormal(*(-nrm / gap))
            planes.AddItem(pl)
        clip = vtk.vtkClipClosedSurface()
        clip.SetInputData(src)
        clip.SetClippingPlanes(planes)
        clip.GenerateFacesOn()
        clip.Update()
        kept += clip.GetOutput().GetNumberOfPolys()
    return kept


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cells", type=int, default=60)
    ap.add_argument("--neighbors", type=int, default=24)
    ap.add_argument("--tess", default="24,40,60",
                    help="comma-separated box tessellation levels")
    args = ap.parse_args()

    try:
        import vtk                                              # noqa: F401
        have_vtk = True
    except ImportError:
        have_vtk = False
        print("vtk not importable — timing ours only\n")

    rng = np.random.default_rng(7)
    print(f"{'faces':>9}{'cells':>7}{'ours (s)':>11}{'vtk (s)':>10}"
          f"{'speedup':>9}{'ours frags':>12}{'vtk frags':>11}")
    for t in [int(x) for x in args.tess.split(",")]:
        verts, faces = box_shell(t, t, t)
        seeds = seeds_for(verts, args.cells, rng)
        order = plane_order(seeds, args.neighbors)

        t0 = time.time()
        k_ours = run_ours(verts, faces, seeds, order)
        dt_ours = time.time() - t0

        dt_vtk, k_vtk = float("nan"), 0
        if have_vtk:
            t0 = time.time()
            k_vtk = run_vtk(verts, faces, seeds, order)
            dt_vtk = time.time() - t0

        sp = (dt_ours / dt_vtk) if have_vtk and dt_vtk > 0 else float("nan")
        print(f"{len(faces):>9,}{args.cells:>7}{dt_ours:>11.2f}{dt_vtk:>10.2f}"
              f"{sp:>8.2f}x{k_ours:>12,}{k_vtk:>11,}")


if __name__ == "__main__":
    main()
