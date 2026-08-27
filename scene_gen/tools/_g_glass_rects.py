#!/usr/bin/env python
"""_g_glass_rects.py — WHERE the glass is in every kit module, measured.

Round 3, agent G. The kit paints its mullions into a texture, so blanking a
pane means authoring geometry over the module face; to do that at the right
place I need the glazed rectangle of every module in the same (u along,
v up, out) frame `quake_flow._piece_frame` / `_b_face_pt` use.

A subset counts as GLAZING if its bound material name says glass, or if it
carries no BaseColor texture at all (the kit's glass materials are bare
UsdPreviewSurfaces with opacity and no map — SM_MBuilding05_FirstFloor_A
Section2 is the type case).

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_g_glass_rects.py

Prints a Python dict literal for pasting into quake_flow._G_GLAZING.
"""
import math
import os
import sys

REPO = "/isaac-sim/AirStack"
sys.path.insert(0, os.path.join(REPO, "scene_gen"))

NAMES = os.environ.get("G_NAMES", "").split(",") if os.environ.get("G_NAMES") else None


def collect_names():
    import random
    from detail import urban_building as ub
    out = {}
    for st in ub.STYLES:
        try:
            pls = ub.build_building(st, 0, 0, 0, rng=random.Random(4))
        except Exception:
            continue
        for p in pls:
            n = str(p.get("usd", "")).rsplit("/", 1)[-1].rsplit(".", 1)[0]
            out.setdefault(n, set()).add(st)
    return out


def face_bbox(pts, counts, indices, idxs):
    lo = [1e9] * 3
    hi = [-1e9] * 3
    starts, k = [], 0
    for c in counts:
        starts.append(k)
        k += int(c)
    for fi in idxs:
        c = int(counts[fi]); s = starts[fi]
        for j in range(c):
            p = pts[indices[s + j]]
            for a in range(3):
                lo[a] = min(lo[a], p[a]); hi[a] = max(hi[a], p[a])
    return lo, hi


def main():
    from pxr import Usd, UsdGeom, UsdShade
    from detail import urban_building as ub

    names = {n: sorted(s) for n, s in collect_names().items()} if not NAMES \
        else {n: [] for n in NAMES}
    rows = {}
    for nm in sorted(names):
        meas = ub.PIECES.get(nm)
        if not meas:
            continue
        sx, sy, sz, xmin, ymin, zmin = meas
        frame = ub._kit(nm)[1]
        scale = ub._kit(nm)[2]
        st = Usd.Stage.Open(ub._usd(nm))
        if not st:
            print("# CANNOT OPEN", nm, file=sys.stderr)
            continue
        found = []
        for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            if not p.IsA(UsdGeom.Mesh):
                continue
            m = UsdGeom.Mesh(p)
            pts = m.GetPointsAttr().Get()
            counts = m.GetFaceVertexCountsAttr().Get()
            indices = m.GetFaceVertexIndicesAttr().Get()
            if pts is None:
                continue
            subs = list(UsdGeom.Subset.GetAllGeomSubsets(m))
            groups = []
            if subs:
                for s in subs:
                    groups.append((s.GetPrim(), list(s.GetIndicesAttr().Get() or [])))
            else:
                groups.append((p, list(range(len(counts or [])))))
            for prim, idxs in groups:
                if not idxs:
                    continue
                try:
                    bm = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
                except Exception:
                    bm = None
                mp = bm.GetPrim() if bm else None
                mname = ""
                base = None
                if mp and mp.IsValid():
                    # the real material name lives on the referenced asset path
                    mname = mp.GetName()
                    for q in Usd.PrimRange(mp):
                        sh = UsdShade.Shader(q)
                        if sh and sh.GetIdAttr().Get() == "UsdUVTexture":
                            f = sh.GetInput("file")
                            v = f.Get() if f else None
                            if v is not None and "BaseColor" in (v.path or ""):
                                base = str(v.path).rsplit("/", 1)[-1]
                    for st_ in (mp.GetPrimStack() or []):
                        lp = getattr(st_.layer, "identifier", "")
                        if "Materials/" in lp:
                            mname = lp.rsplit("/", 1)[-1].rsplit(".", 1)[0]
                            break
                is_glass = ("glass" in mname.lower()
                            or (base is None and mname)
                            or (base is not None and "window" in base.lower()))
                if not is_glass:
                    continue
                lo, hi = face_bbox(pts, counts, indices, idxs)
                found.append((mname, base, [c * scale for c in lo],
                              [c * scale for c in hi], len(idxs)))
        if not found:
            continue
        rows[nm] = []
        for mname, base, lo, hi, nf in found:
            if frame == "dw":
                u0, u1 = lo[1] - ymin, hi[1] - ymin
                out0 = -abs(xmin) - 0.02
                # outward is +x for dw; _b_face_pt's `out` grows outward
                o = 0.5 * (lo[0] + hi[0])
                out = -(abs(xmin) + 0.02) - o
                out = -(abs(xmin) + 0.02 + o) if False else (o - (abs(xmin)))
            else:
                u0, u1 = lo[0], hi[0]
                o = 0.5 * (lo[1] + hi[1])
                out = ymin - 0.02 - o
            rows[nm].append((round(u0, 3), round(u1, 3), round(lo[2], 3),
                             round(hi[2], 3), round(out, 3), nf, mname, base))
        print("# {0:<42s} {1}".format(nm, names.get(nm)))
        for r in rows[nm]:
            print("#    u {0:6.2f}..{1:6.2f}  v {2:6.2f}..{3:6.2f}  out {4:7.2f}"
                  "  faces {5:4d}  mat {6}  base {7}".format(*r))
    print()
    print("_G_GLAZING = {")
    for nm in sorted(rows):
        cells = ["({0}, {1}, {2}, {3}, {4})".format(*r[:5]) for r in rows[nm]]
        print('    "{0}": [{1}],'.format(nm, ", ".join(cells)))
    print("}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
