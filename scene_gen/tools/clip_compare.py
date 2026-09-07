#!/usr/bin/env python3
"""clip_compare — our Voronoi clipper vs VTK's, on the same cut, side by side.

    AirStack/.venv/bin/python scene_gen/tools/clip_compare.py --out /tmp/cmp.usda

Writes ONE USD holding both results, ours on the left and VTK's on the right,
each fragment given its own `displayColor` so the cell structure is visible.
Render it with `tools/repreview.py` or open it in `tools/view_usd.py`.

SAME SEEDS, SAME PLANES, SAME MESH — only the clipper differs. The seeds come
from `mesh_damage.fracture_seeds` against a real `Failure`, so the cell count
and sizing are the pipeline's, not invented here.

WHY THIS EXISTS
---------------
`tools/clip_bench.py` measured VTK at 6.7-10x faster with capping on, and then
the per-cell volumes disagreed by up to 440% with two cells coming back EMPTY.
Aggregate volume matched to 6%, which is exactly how that kind of error hides.
Numbers could not say which backend was wrong, so this draws them.

WHAT TO LOOK FOR
----------------
* Do both sides fill the original volume, or does one leave gaps?
* Are VTK's cells convex polyhedra where ours are ragged (or the reverse)?
* Are the two EMPTY cells missing on the VTK side, or merged into a neighbour?

`_cap_fan` closes our cuts INCREMENTALLY — it needs the cross-section to be a
closed loop at every intermediate plane, so the closure of a final cell depends
on the closure of all the ones before it (see `_fracture_soup`'s docstring).
`vtkClipClosedSurface` computes the cap from the final cross-section instead.
That difference is the most likely explanation for both the speed gap and the
disagreement, and it is what the picture should settle.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def _parse(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", default="_bakelab/clip_compare.usda")
    p.add_argument("--tess", type=int, default=20,
                   help="box tessellation; higher = more faces to cut")
    p.add_argument("--fragment-m", type=float, default=2.5)
    p.add_argument("--max-cells", type=int, default=60)
    p.add_argument("--seed", type=int, default=7)
    return p.parse_args(argv)


def _colour(i, n):
    import colorsys
    r, g, b = colorsys.hsv_to_rgb((i * 0.61803) % 1.0, 0.55, 0.95)
    return (float(r), float(g), float(b))


def _author(stage, path, verts, faces, colour):
    from pxr import Gf, Sdf, UsdGeom, Vt
    if len(faces) == 0:
        return False
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, v))
                                      for v in verts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    m.CreateFaceVertexIndicesAttr(
        Vt.IntArray([int(i) for f in faces for i in f]))
    m.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*colour)]))
    m.CreateDoubleSidedAttr(True)
    return True


def main(argv=None) -> int:
    args = _parse(argv)
    from pxr import Sdf, Usd, UsdGeom
    from tools.clip_bench import box_shell, _to_vtk, vtk_prepare
    from disaster import mesh_damage as md
    from disaster import quake as Q

    verts, faces = box_shell(args.tess, args.tess, args.tess)
    soup = md.Soup(verts, None, faces, None, [], [])

    # THE PANCAKE MECHANISM'S OWN NUMBERS, not invented ones.
    mech = Q.MECHANISMS["pancake"]
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    bounds = md.Bounds(lo, hi)
    # A uniform failure at the mechanism's intensity: `pancake` releases
    # everything (`intensity` 1.0), which is what makes it the rung that cuts
    # the whole building and so the fairest stress test of a clipper.
    failure = md.Failure(
        "pancake",
        lambda pts: np.full(len(np.atleast_2d(pts)), float(mech.intensity)))
    seeds = md.fracture_seeds(soup, failure, float(args.fragment_m),
                              int(args.seed), int(args.max_cells))
    print(f"[compare] mesh {len(faces):,} faces · pancake fragment_m="
          f"{mech.fragment_m} · {len(seeds)} seeds")

    import time
    t0 = time.time()
    ours = md._fracture_soup(soup, seeds, cap=True)
    t_ours = time.time() - t0
    print(f"[compare] ours : {len(ours):>4} fragments in {t_ours:6.2f}s")

    # --- VTK, same seeds, same bisectors ---------------------------------
    import vtk
    from vtk.util import numpy_support as ns
    src = vtk_prepare(_to_vtk(verts, faces))
    d2 = np.sum((seeds[:, None, :] - seeds[None, :, :]) ** 2, axis=-1)
    np.fill_diagonal(d2, np.inf)
    order = np.argsort(d2, axis=1)[:, :24]

    t0 = time.time()
    vtk_cells = []
    for i in range(len(seeds)):
        p = seeds[i]
        planes = vtk.vtkPlaneCollection()
        for j in order[i]:
            q = seeds[j]
            n = q - p
            g = float(np.linalg.norm(n))
            if g < 1e-9:
                continue
            pl = vtk.vtkPlane()
            pl.SetOrigin(*((p + q) * 0.5))
            pl.SetNormal(*(-n / g))
            planes.AddItem(pl)
        cl = vtk.vtkClipClosedSurface()
        cl.SetInputData(src)
        cl.SetClippingPlanes(planes)
        cl.GenerateFacesOn()
        cl.Update()
        t2 = vtk.vtkTriangleFilter(); t2.SetInputData(cl.GetOutput()); t2.Update()
        out = t2.GetOutput()
        if out.GetNumberOfPolys() == 0:
            vtk_cells.append(None)
            continue
        v = ns.vtk_to_numpy(out.GetPoints().GetData()).astype(np.float64)
        f = ns.vtk_to_numpy(out.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
        vtk_cells.append((v, f))
    t_vtk = time.time() - t0
    n_vtk = sum(1 for c in vtk_cells if c is not None)
    print(f"[compare] vtk  : {n_vtk:>4} fragments in {t_vtk:6.2f}s "
          f"({t_ours / max(t_vtk, 1e-9):.1f}x faster)")
    print(f"[compare] vtk EMPTY cells: {sum(1 for c in vtk_cells if c is None)}"
          f" of {len(seeds)}")

    # --- one stage, side by side -----------------------------------------
    span = float(hi[0] - lo[0]) * 1.6
    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/ours"))
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/vtk"))

    n_o = 0
    for i, frag in enumerate(ours):
        if _author(stage, f"/World/ours/frag_{i:03d}", frag.verts, frag.faces,
                   _colour(i, len(ours))):
            n_o += 1
    n_v = 0
    for i, c in enumerate(vtk_cells):
        if c is None:
            continue
        v, f = c
        v = v.copy()
        v[:, 0] += span
        if _author(stage, f"/World/vtk/frag_{i:03d}", v, f, _colour(i, len(seeds))):
            n_v += 1
    stage.GetRootLayer().Save()
    print(f"\n[compare] authored {n_o} (ours, left) + {n_v} (vtk, right) "
          f"-> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
