#!/usr/bin/env python
"""soot_uv_probe.py — does a kit module's UV layout let a per-prim crop of the
soot skin land where the soot is?

    docker exec isaac-sim bash -c \\
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
       /isaac-sim/AirStack/scene_gen/tools/soot_uv_probe.py [NAME ...]"

Bare USD in the container's python (no SimulationApp — safe next to a running
sim). For each module USD it prints, per mesh: the bound material, its base
colour texture, the UV bounding box, and how `v` runs against the mesh's own
height (does v=1 sit at the TOP of the module, and does 0..1 span the whole
module?). `soot_plume.piece_crop` / `merge_piece` assume a module's base map is
addressed corner to corner with image row 0 at the top — this is the test of
that assumption, per module family.
"""
import os
import sys

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

from pxr import Usd, UsdGeom, UsdShade, Sdf                 # noqa: E402
from detail import urban_building as ub                     # noqa: E402

DEFAULT = ["SM_MBuilding01_Facade_A", "SM_MBuilding01_Facade_B",
           "SM_MBuilding01_FirstFloor_A", "SM_MBuilding01_TopFloor_A",
           "SM_MBuilding04_Facade_A", "SM_MBuilding04_Facade_B",
           "SM_MBuilding04_FirstFloor_A", "SM_MBuilding04_Facade_Corner",
           "SM_MBuilding02_Facade_A", "SM_MBuilding05_SkyscraperFacade_A"]


def _tex_of(mat_prim):
    for p in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(p)
        if not sh:
            continue
        for inp in sh.GetInputs():
            try:
                v = inp.Get()
            except Exception:
                continue
            if isinstance(v, Sdf.AssetPath):
                f = v.resolvedPath or v.path or ""
                low = f.lower()
                if any(k in low for k in ("basecolor", "albedo", "diffuse", "_bc", "_col", "_d.")):
                    return inp.GetBaseName(), f
    return None, None


def probe(name):
    try:
        url = ub._usd(name)
    except Exception as exc:
        print("== {0}: no kit entry ({1})".format(name, exc))
        return
    st = Usd.Stage.Open(url)
    if not st:
        print("== {0}: cannot open {1}".format(name, url))
        return
    print("== {0}  {1}".format(name, url))
    meas = ub.PIECES.get(name)
    print("   PIECES:", meas)
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        cnt = m.GetFaceVertexCountsAttr().Get()
        pv = None
        for q in UsdGeom.PrimvarsAPI(p).GetPrimvars():
            if q.GetTypeName().role == "TextureCoordinate" or q.GetBaseName() in ("st", "uv", "UVMap"):
                pv = q
                break
        subsets = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(p)))
        targets = [(s.GetPrim(), set(int(i) for i in (s.GetIndicesAttr().Get() or []))) for s in subsets] or [(p, None)]
        print("   mesh {0}: {1} pts, {2} faces, {3} subset(s), uv primvar {4} ({5})".format(
            p.GetPath(), len(pts), len(cnt), len(subsets),
            pv.GetName() if pv else None, pv.GetInterpolation() if pv else "-"))
        if not pv:
            continue
        vals = pv.Get()
        ind = pv.GetIndices() if pv.IsIndexed() else None
        interp = pv.GetInterpolation()
        # per face: (uv list, point list)
        face_uv, face_pt = [], []
        k = 0
        for fi, c in enumerate(cnt):
            c = int(c)
            uvs, ps = [], []
            for j in range(c):
                vi = int(idx[k + j])
                if interp == "faceVarying":
                    t = int(ind[k + j]) if ind is not None else (k + j)
                else:
                    t = int(ind[vi]) if ind is not None else vi
                uvs.append(vals[t])
                ps.append(pts[vi])
            face_uv.append(uvs)
            face_pt.append(ps)
            k += c
        for tprim, fset in targets:
            bound = UsdShade.MaterialBindingAPI(tprim).ComputeBoundMaterial()[0]
            mp = bound.GetPrim() if bound else None
            inp, tex = _tex_of(mp) if mp is not None and mp.IsValid() else (None, None)
            faces = [i for i in range(len(cnt)) if fset is None or i in fset]
            us = [uv[0] for i in faces for uv in face_uv[i]]
            vs = [uv[1] for i in faces for uv in face_uv[i]]
            zs = [pt[2] for i in faces for pt in face_pt[i]]
            ys = [pt[1] for i in faces for pt in face_pt[i]]
            xs = [pt[0] for i in faces for pt in face_pt[i]]
            if not us:
                continue
            # correlation of v with height: v at the lowest and highest vertices
            pairs = [(pt[2], uv[1], uv[0], pt[0], pt[1]) for i in faces for pt, uv in zip(face_pt[i], face_uv[i])]
            pairs.sort()
            lo = pairs[:max(1, len(pairs) // 10)]
            hi = pairs[-max(1, len(pairs) // 10):]
            v_lo = sum(q[1] for q in lo) / len(lo)
            v_hi = sum(q[1] for q in hi) / len(hi)
            print("     target {0}".format(tprim.GetPath().name if fset is not None else "<mesh>"))
            print("        material {0}".format(mp.GetPath() if mp is not None else None))
            print("        basecolor {0} = {1}".format(inp, tex))
            print("        {0} face(s); x {1:.2f}..{2:.2f}  y {3:.2f}..{4:.2f}  z {5:.2f}..{6:.2f}".format(
                len(faces), min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))
            print("        u {0:.3f}..{1:.3f}   v {2:.3f}..{3:.3f}   v at bottom {4:.3f} / top {5:.3f}  -> {6}".format(
                min(us), max(us), min(vs), max(vs), v_lo, v_hi,
                "v RISES with height (v=1 top)" if v_hi > v_lo else "v FALLS with height (v=0 top)"))


if __name__ == "__main__":
    names = sys.argv[1:] or DEFAULT
    for n in names:
        probe(n)
