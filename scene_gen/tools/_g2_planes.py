#!/usr/bin/env python
"""_g2_planes.py -- recon: what PLANES does a kit wall module have?

Round 3, agent G2. Agent G measured the family 04/05 glazing because it is
separate glass GEOMETRY with a glass material. Families 01/02/03 paint their
windows -- but the REVEALS are real geometry, so the opening rectangle can be
recovered from the hole in the outer wall plane.

This probe just dumps the evidence: every face of a module, bucketed by its
dominant normal axis and its offset along that axis, in `_piece_frame`
coordinates (u along the piece, v up, out = outward from the face plane).

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_g2_planes.py G_NAMES=SM_MBuilding02_FirstFloor_C
"""
import os
import sys

REPO = "/isaac-sim/AirStack"
sys.path.insert(0, os.path.join(REPO, "scene_gen"))

NAMES = [n for n in os.environ.get("G_NAMES", "").split(",") if n]


def faces_of(stage, scale):
    """[(normal_axis, offset, [(x,y,z)...])] for every face, in metres,
    in the module's own authoring frame."""
    from pxr import Gf, Usd, UsdGeom
    out = []
    for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get()
        counts = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        wp = [xf.Transform(Gf.Vec3d(q[0], q[1], q[2])) for q in pts]
        wp = [(q[0] * scale, q[1] * scale, q[2] * scale) for q in wp]
        k = 0
        for c in counts:
            c = int(c)
            poly = [wp[idx[k + j]] for j in range(c)]
            k += c
            # newell normal
            nx = ny = nz = 0.0
            for j in range(c):
                a, b = poly[j], poly[(j + 1) % c]
                nx += (a[1] - b[1]) * (a[2] + b[2])
                ny += (a[2] - b[2]) * (a[0] + b[0])
                nz += (a[0] - b[0]) * (a[1] + b[1])
            out.append(((nx, ny, nz), poly))
    return out


def main():
    from pxr import Usd
    from detail import urban_building as ub
    for nm in NAMES:
        meas = ub.PIECES.get(nm)
        url = ub._usd(nm)
        scale = ub._kit(nm)[2]
        frame = ub._kit(nm)[1]
        st = Usd.Stage.Open(url)
        if not st:
            print("# CANNOT OPEN", nm)
            continue
        fs = faces_of(st, scale)
        print("=== {0}  meas={1}  frame={2}  faces={3}".format(nm, meas, frame, len(fs)))
        buckets = {}
        for n, poly in fs:
            a = max(range(3), key=lambda i: abs(n[i]))
            if abs(n[a]) < 1e-9:
                continue
            sgn = 1 if n[a] > 0 else -1
            off = round(sum(q[a] for q in poly) / len(poly), 2)
            key = (a, sgn, off)
            b = buckets.setdefault(key, [0, [1e9] * 3, [-1e9] * 3, 0.0])
            b[0] += 1
            for q in poly:
                for i in range(3):
                    b[1][i] = min(b[1][i], q[i])
                    b[2][i] = max(b[2][i], q[i])
            b[3] += abs(n[a]) / 2.0
        ax = "xyz"
        for key in sorted(buckets, key=lambda k: (k[0], k[2], k[1])):
            a, sgn, off = key
            n_, lo, hi, area = buckets[key]
            print("  n{0}{1} @ {2:7.2f}  faces {3:4d}  area {4:8.2f}  "
                  "x {5:6.2f}..{6:6.2f}  y {7:6.2f}..{8:6.2f}  z {9:6.2f}..{10:6.2f}"
                  .format("+-"[sgn < 0], ax[a], off, n_, area,
                          lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
