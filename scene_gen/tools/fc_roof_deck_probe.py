#!/usr/bin/env python
"""fc_roof_deck_probe — DOES THE ROOF ACTUALLY REACH UP THERE, under the
rectangle a `gac_props` roof prop was seated on?

    bash scene_gen/tools/usd_python.sh scene_gen/tools/fc_roof_deck_probe.py \
        --usd omniverse://.../Muyang/DownTown/Assets/BG_Building_C.usd \
        --scale 0.01 --rect -6.6 -17.6 22.8 16.9

THE FAILURE THIS MEASURES (user, 2026-08-31, on the live 500 m fire city):
"/World/stage/generated/roof_house_94_1354/LOD0 this roof house is floating
with no building near it".

`gac_props.roof_props` seats every roof prop at ONE height:
``roof_z = bld["z_m"] + H + dims["z0"]`` — the asset's own global max Z. That
is right for a flat massing lid and wrong for a building with a SETBACK or a
crown, where only part of the plan reaches the top and the rest of the roof
is a deck several storeys lower. `downtown_fire_500.yaml` already records
exactly this failure for two GAC towers and fixes it by hand:

    no_roof_props: ["SM_Building_31", "SM_Building_16"]
    # "the tops are odd shaped" ... "they carry a crown/setback, so the
    # parapet-inset maths seats a tank on a roof height that only part of
    # the footprint actually reaches."
    # `roof_flatness.py` does not catch this: it measures the share of
    # upward-facing area within 1.5 m of the asset's own max Z, and a big
    # flat crown ON TOP of a setback still scores well.

`roof_flatness.py` answers "how much of this roof is at the top" globally.
THIS answers the question that actually decides whether a prop floats: "at
the (u, v) rectangle this prop was put on, how high is the geometry?" — the
per-rectangle form the flatness metric cannot express.

METHOD. Same triangle binning as `roof_flatness.py`/`gac_faces.py`: bin every
triangle of the asset by its area-weighted normal, keep the UPWARD-facing set
(``n_z > 0.72``), and report the max Z of that set inside the requested
rectangle against the asset's own global max Z. The gap between them is how
far a prop seated at the global max floats above the deck it is standing on.

FRAME. ``--rect`` is in the BUILDING PLACEMENT's own local frame, centred on
the placement's (x_m, y_m) and un-rotated by its yaw — exactly the frame
`roof_props` computes its local `(lx, ly)` offsets in — and in METRES after
``--scale``. `tools/fc_prop_orphan_probe.py` prints the rectangle for a given
prop index.

NO `SimulationApp`. Runs under `tools/usd_python.sh` (bare USD + the
Omniverse resolver), so it is safe to run beside a live Isaac session.
"""
import argparse
import sys

import numpy as np
from pxr import Usd, UsdGeom, Gf

UP_THRESHOLD = 0.72


def tri_soup(stage, scale):
    """(centroids, normals_z, areas, max_z) over every Mesh, world-space,
    scaled to metres."""
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    C, NZ, A = [], [], []
    zmax = -1e30
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue
        M = cache.GetLocalToWorldTransform(prim)
        P = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        m = np.array(M, dtype=np.float64)          # row-vector convention
        P = (np.hstack([P, np.ones((len(P), 1))]) @ m)[:, :3] * float(scale)
        zmax = max(zmax, float(P[:, 2].max()))
        counts = np.asarray(counts)
        idx = np.asarray(idx)
        off = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for n, o in zip(counts, off):
            if n < 3:
                continue
            fan = idx[o:o + n]
            a = P[fan[0]]
            for k in range(1, n - 1):
                b, c = P[fan[k]], P[fan[k + 1]]
                nrm = np.cross(b - a, c - a)
                ar = 0.5 * float(np.linalg.norm(nrm))
                if ar <= 0.0:
                    continue
                C.append((a + b + c) / 3.0)
                NZ.append(float(nrm[2]) / (2.0 * ar))
                A.append(ar)
    if not C:
        return np.zeros((0, 3)), np.zeros(0), np.zeros(0), 0.0
    return np.array(C), np.array(NZ), np.array(A), zmax


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--rect", type=float, nargs=4, action="append",
                    metavar=("U0", "V0", "U1", "V1"),
                    help="local-frame rectangle in metres; repeatable")
    ap.add_argument("--bands", type=int, default=8,
                    help="also print a U-band profile of the upward deck")
    a = ap.parse_args(argv)

    stage = Usd.Stage.Open(a.usd)
    if stage is None:
        print("could not open " + a.usd)
        return 2
    C, NZ, A, zmax = tri_soup(stage, a.scale)
    up = NZ > UP_THRESHOLD
    print("\n[deck] {0}".format(a.usd))
    print("[deck] {0} triangle(s), {1} upward-facing; asset max Z = {2:.2f} m"
          .format(len(C), int(up.sum()), zmax))
    if not up.any():
        return 1
    Cu, Au = C[up], A[up]
    # the asset's own local origin is the placement frame's origin, and
    # `roof_props` works in the placement's CENTRED frame, so recentre on the
    # upward set's own XY midpoint the way the resolver's footprint does.
    x0 = 0.5 * (C[:, 0].min() + C[:, 0].max())
    y0 = 0.5 * (C[:, 1].min() + C[:, 1].max())
    print("[deck] bbox {0:.1f} x {1:.1f} m, centred on local ({2:.2f}, {3:.2f})"
          .format(C[:, 0].max() - C[:, 0].min(),
                  C[:, 1].max() - C[:, 1].min(), x0, y0))
    U, V = Cu[:, 0] - x0, Cu[:, 1] - y0
    tol = 1.5
    top_share = float(Au[Cu[:, 2] > zmax - tol].sum() / Au.sum())
    print("[deck] roof_flatness-style share of upward area within {0:.1f} m "
          "of the top: {1:.2f}".format(tol, top_share))

    if a.bands:
        print("[deck] U-band profile of the upward deck (max Z per band):")
        edges = np.linspace(U.min(), U.max(), a.bands + 1)
        for k in range(a.bands):
            sel = (U >= edges[k]) & (U <= edges[k + 1])
            if not sel.any():
                print("    u [{0:+7.1f},{1:+7.1f}]  (empty)".format(
                    edges[k], edges[k + 1]))
                continue
            print("    u [{0:+7.1f},{1:+7.1f}]  max z {2:7.2f} m  "
                  "area {3:8.1f} m2".format(edges[k], edges[k + 1],
                                            float(Cu[sel][:, 2].max()),
                                            float(Au[sel].sum())))

    for rect in (a.rect or []):
        u0, v0, u1, v1 = sorted(rect[:2] + rect[2:])[0], min(rect[1], rect[3]), \
            max(rect[0], rect[2]), max(rect[1], rect[3])
        u0, u1 = min(rect[0], rect[2]), max(rect[0], rect[2])
        v0, v1 = min(rect[1], rect[3]), max(rect[1], rect[3])
        sel = (U >= u0) & (U <= u1) & (V >= v0) & (V <= v1)
        print("\n[deck] rect u[{0:+.1f},{1:+.1f}] v[{2:+.1f},{3:+.1f}]"
              .format(u0, u1, v0, v1))
        if not sel.any():
            print("       NO UPWARD GEOMETRY AT ALL under this rectangle — a "
                  "prop seated at the asset max Z here has nothing under it")
            continue
        zt = float(Cu[sel][:, 2].max())
        print("       upward area {0:.1f} m2, deck max Z {1:.2f} m — a prop "
              "seated at the asset max Z ({2:.2f} m) floats {3:+.2f} m above "
              "it".format(float(Au[sel].sum()), zt, zmax, zmax - zt))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
