#!/usr/bin/env python3
"""rung_compare — numpy vs VTK fracture, every earthquake rung, side by side.

    AirStack/.venv/bin/python scene_gen/tools/rung_compare.py

Writes one USD laid out as a grid: a ROW per backend (numpy in front, VTK
behind) and a COLUMN per rung, so the two cuts of the same rung sit next to
each other. Every fragment gets its own `displayColor` so the cell structure
reads. Render with `tools/view_usd.py --shot`.

Each rung is cut with ITS OWN mechanism out of `quake.MECHANISMS` — the real
`fragment_m`, `intensity` and `support`, not one setting reused four times —
because the whole question is whether the backends agree across the ladder,
and the rungs differ mostly in how finely they cut.

The mesh is a plain box on purpose. A simple closed shape makes a disagreement
between the two obvious; a real asset's own defects would muddy it.
"""

from __future__ import annotations

import argparse
import importlib
import os
import sys
import time

import numpy as np

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

RUNGS = ("cracked", "soft_storey", "partial_collapse", "pancaked")

#: Which mechanism defines each rung — the FIRST entry of its `RUNG_PLAN`,
#: which is the one the rung is named after.
MECH_FOR = {"cracked": "crack", "soft_storey": "soft_storey",
            "partial_collapse": "shear_off", "pancaked": "pancake"}


def _colour(i):
    import colorsys
    r, g, b = colorsys.hsv_to_rgb((i * 0.61803) % 1.0, 0.55, 0.95)
    return (float(r), float(g), float(b))


def _author(stage, path, verts, faces, colour, offset):
    from pxr import Gf, Sdf, UsdGeom, Vt
    if len(faces) == 0:
        return False
    v = np.asarray(verts, dtype=np.float64) + np.asarray(offset)
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in v]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    m.CreateFaceVertexIndicesAttr(
        Vt.IntArray([int(i) for f in faces for i in f]))
    m.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*colour)]))
    m.CreateDoubleSidedAttr(True)
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default="_bakelab/rung_compare.usda")
    ap.add_argument("--tess", type=int, default=22)
    ap.add_argument("--max-cells", type=int, default=120)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    from pxr import Sdf, Usd, UsdGeom
    from tools.clip_bench import box_shell
    from disaster import quake as Q

    verts, faces = box_shell(args.tess, args.tess, args.tess)
    span_x = float(verts[:, 0].max() - verts[:, 0].min()) * 1.7
    span_y = float(verts[:, 1].max() - verts[:, 1].min()) * 2.2

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    if os.path.exists(out_path):
        os.remove(out_path)
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())

    print(f"{'rung':<20}{'backend':>8}{'frags':>7}{'secs':>8}"
          f"{'points':>10}{'faces':>10}")
    results = {}
    for bi, backend in enumerate(("numpy", "vtk")):
        os.environ["SCENE_FRACTURE_BACKEND"] = backend
        import disaster.mesh_damage as md
        importlib.reload(md)
        if md.active_backend() != backend:
            print(f"  ({backend} unavailable — skipped)")
            continue
        for ri, rung in enumerate(RUNGS):
            mech = Q.MECHANISMS[MECH_FOR[rung]]
            soup = md.Soup(verts, None, faces, None, [], [])
            fail = md.Failure(
                rung,
                lambda p, m=mech: np.full(len(np.atleast_2d(p)),
                                          float(m.intensity)))
            seeds = md.fracture_seeds(soup, fail, float(mech.fragment_m),
                                      args.seed, args.max_cells)
            t0 = time.time()
            frags = md._fracture_soup(soup, seeds, cap=True)
            dt = time.time() - t0
            pts = sum(len(f.verts) for f in frags)
            fc = sum(len(f.faces) for f in frags)
            results[(backend, rung)] = (len(frags), dt, pts, fc)
            print(f"{rung:<20}{backend:>8}{len(frags):>7}{dt:>8.2f}"
                  f"{pts:>10,}{fc:>10,}")
            off = (ri * span_x, bi * span_y, 0.0)
            grp = f"/World/{backend}_{rung}"
            UsdGeom.Xform.Define(stage, Sdf.Path(grp))
            for k, fr in enumerate(frags):
                _author(stage, f"{grp}/frag_{k:03d}", fr.verts, fr.faces,
                        _colour(k), off)

    stage.GetRootLayer().Save()
    print(f"\n{'rung':<20}{'numpy s':>9}{'vtk s':>8}{'speedup':>9}"
          f"{'numpy pts':>11}{'vtk pts':>10}")
    for rung in RUNGS:
        a = results.get(("numpy", rung))
        b = results.get(("vtk", rung))
        if not a or not b:
            continue
        print(f"{rung:<20}{a[1]:>9.2f}{b[1]:>8.2f}{a[1]/max(b[1],1e-9):>8.1f}x"
              f"{a[2]:>11,}{b[2]:>10,}")
    print(f"\n-> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
