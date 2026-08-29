#!/usr/bin/env python
"""Does `urban_fire.mono_parts`' roof/glass classification work on the
GreatAmericanCity stock?

`mono_parts` classifies a monolith's parts by the BASE-COLOUR TEXTURE FILENAME
(`_MONO_ROOF_TEX` / `_MONO_GLASS_TEX`), a rule measured over the
selected_citydemo + DownTown packs. GAC is a different pack and nothing has
checked that the rule transfers. If it does not, `mono_roof_plane` falls back
to "the tallest wall part", which on a stepped block is the stair bulkhead —
the exact bug that put roof debris in the air.

Bare pxr, no Kit, no SimulationApp: safe beside a running sim.

    docker exec isaac-sim bash /isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
        /isaac-sim/AirStack/scene_gen/tools/mono_parts_probe.py
"""
import json
import os
import sys

import numpy as np
from pxr import Usd, UsdGeom, UsdShade, Sdf

NUC = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
ROOF_TEX = ("roof", "flatroof")
GLASS_TEX = ("glass", "window", "curtain", "glazing")

NAMES = (os.environ.get("PROBE_NAMES") or
         "SM_Building_01,SM_Building_04,SM_Building_10,SM_Building_16,"
         "SM_Building_20,SM_Building_31").split(",")


def bound_texture(prim):
    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if not mat or not mat.GetPrim().IsValid():
        return "", ""
    tex = ""
    for c in Usd.PrimRange(mat.GetPrim()):
        sh = UsdShade.Shader(c)
        if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
            continue
        d = sh.GetInput("diffuseColor")
        if d is not None and d.HasConnectedSource():
            ts = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
            f = ts.GetInput("file")
            v = f.Get() if f else None
            if isinstance(v, Sdf.AssetPath) and v.path:
                tex = v.path.rsplit("/", 1)[-1]
        break
    return str(mat.GetPrim().GetPath()), tex


def parts_of(stage, scale):
    out = []
    xc = UsdGeom.XformCache()
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        M = np.asarray(xc.GetLocalToWorldTransform(prim), dtype=float)
        P = np.asarray(pts, dtype=float)
        P = (np.c_[P, np.ones(len(P))] @ M)[:, :3] * scale
        subs = [s for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
                if (s.GetElementTypeAttr().Get() or "face") == "face"]
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        usable = bool(subs) and len(counts) and len(fvi) == int(counts.sum())
        targets = ([(s.GetPrim(), s) for s in subs] if usable else [(prim, None)])
        for tgt, sub in targets:
            if sub is None:
                V = P
            else:
                fi = np.asarray(sub.GetIndicesAttr().Get() or [], dtype=np.int64)
                fi = fi[(fi >= 0) & (fi < len(counts))]
                if not len(fi):
                    continue
                mask = np.zeros(len(counts), dtype=bool)
                mask[fi] = True
                V = P[np.unique(fvi[np.repeat(mask, counts)])]
            if not len(V):
                continue
            mat, tex = bound_texture(tgt)
            low = tex.lower()
            kind = ("roof" if any(q in low for q in ROOF_TEX) else
                    "glass" if any(q in low for q in GLASS_TEX) else "wall")
            mn, mx = V.min(axis=0), V.max(axis=0)
            out.append({"name": tgt.GetName(), "kind": kind, "tex": tex,
                        "mat": mat.rsplit("/", 1)[-1],
                        "x0": mn[0], "y0": mn[1], "z0": mn[2],
                        "x1": mx[0], "y1": mx[1], "z1": mx[2]})
    out.sort(key=lambda q: (q["z0"], -(q["z1"] - q["z0"])))
    return out


def main():
    report = {}
    for nm in NAMES:
        nm = nm.strip()
        if not nm:
            continue
        url = NUC + nm + ".usd"
        st = Usd.Stage.Open(url)
        if st is None:
            print("OPEN FAILED " + url)
            continue
        mpu = UsdGeom.GetStageMetersPerUnit(st)
        parts = parts_of(st, 1.0)          # asset units; mpu printed below
        zs = [p["z1"] for p in parts]
        top = max(zs) if zs else 0.0
        roofs = [p for p in parts if p["kind"] == "roof"]
        walls = [p for p in parts if p["kind"] in ("wall", "glass")]
        print("\n=== {0}  mpu={1}  parts={2} ===".format(nm, mpu, len(parts)))
        print("  top {0:.2f} (asset units)  roof-classified {1}  glass {2}"
              .format(top, len(roofs),
                      sum(1 for p in parts if p["kind"] == "glass")))
        # what mono_roof_plane would pick, and how far below the true top
        if roofs:
            best = max(roofs, key=lambda q: (q["x1"] - q["x0"]) * (q["y1"] - q["y0"]))
            src = "roof-textured"
        elif walls:
            best = max(walls, key=lambda q: q["z1"])
            src = "FALLBACK tallest wall"
        else:
            best = None
            src = "none"
        if best is not None:
            area = (best["x1"] - best["x0"]) * (best["y1"] - best["y0"])
            full = ((max(p["x1"] for p in parts) - min(p["x0"] for p in parts)) *
                    (max(p["y1"] for p in parts) - min(p["y0"] for p in parts)))
            print("  mono_roof_plane -> {0}: '{1}' z={2:.2f} ({3:+.2f} vs top), "
                  "area {4:.1f} = {5:.0%} of footprint"
                  .format(src, best["name"], best["z1"], best["z1"] - top,
                          area, area / full if full else 0.0))
        for p in parts[:40]:
            print("    {0:<28} {1:<5} z {2:8.2f}..{3:8.2f}  {4:.1f}x{5:.1f}  "
                  "tex={6}".format(p["name"][:28], p["kind"], p["z0"], p["z1"],
                                   p["x1"] - p["x0"], p["y1"] - p["y0"],
                                   p["tex"] or "(none)"))
        report[nm] = {"mpu": mpu, "n_parts": len(parts), "n_roof": len(roofs),
                      "top": top,
                      "kinds": sorted({p["kind"] for p in parts}),
                      "textures": sorted({p["tex"] for p in parts if p["tex"]})}
    out = os.environ.get("PROBE_OUT", "")
    if out:
        with open(out, "w") as fh:
            json.dump(report, fh, indent=1)
        print("\n-> " + out)


if __name__ == "__main__":
    sys.exit(main())
