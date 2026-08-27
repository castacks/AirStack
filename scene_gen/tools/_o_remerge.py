#!/usr/bin/env python
"""_o_remerge.py — re-export an ALREADY BAKED archetype both ways.

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_o_remerge.py \
        USD=<abs path>[,<abs path>...] OUT=<dir> [MERGE_ONLY=1]

A baked archetype is exactly the input `export_object` produces, so opening
one and running it back through gives an EXACT A/B on real geometry — same
fragments, same materials, same settle — without waiting for a 30-minute
bake. Bare pxr, no Kit, no GPU: safe beside a running sim.

Writes `<OUT>/raw/<name>` and `<OUT>/merged/<name>` and prints the table.
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..")))

from pxr import Usd, UsdGeom, UsdShade                       # noqa: E402

from disaster import bake                                    # noqa: E402


def census(path):
    st = Usd.Stage.Open(path)
    n = {"prims": 0, "meshes": 0, "materials": 0, "shaders": 0, "subsets": 0,
         "points": 0, "faces": 0}
    t0 = time.time()
    for p in st.Traverse():
        n["prims"] += 1
        tn = p.GetTypeName()
        if tn == "Material":
            n["materials"] += 1
        elif tn == "Shader":
            n["shaders"] += 1
        elif tn == "GeomSubset":
            n["subsets"] += 1
        if p.IsA(UsdGeom.Mesh):
            n["meshes"] += 1
            m = UsdGeom.Mesh(p)
            n["points"] += len(m.GetPointsAttr().Get() or [])
            n["faces"] += len(m.GetFaceVertexCountsAttr().Get() or [])
    n["traverse_s"] = round(time.time() - t0, 3)
    n["mb"] = round(os.path.getsize(path) / 1e6, 2)
    # unbound meshes are a look bug, so count them here rather than trusting
    meshes, ok, miss = bake.validate(path, root="/Baked")
    n["bound_missing"] = miss
    return n


def main():
    usds = [q.strip() for q in os.environ.get("USD", "").split(",") if q.strip()]
    out = os.environ.get("OUT") or "/tmp/o_remerge"
    only = os.environ.get("MERGE_ONLY", "").strip() in ("1", "true", "yes")
    os.makedirs(os.path.join(out, "raw"), exist_ok=True)
    os.makedirs(os.path.join(out, "merged"), exist_ok=True)
    rows = []
    for src_path in usds:
        name = os.path.basename(src_path)
        src = Usd.Stage.Open(src_path)
        if src is None:
            print("cannot open " + src_path)
            continue
        row = {"name": name, "src": census(src_path)}
        for mode, sub in (("off", "raw"), ("on", "merged")):
            if only and mode == "off":
                continue
            dst = os.path.join(out, sub, name)
            if os.path.exists(dst):
                os.remove(dst)
            st = {}
            t0 = time.time()
            bake.export_object(src, None, ["/Baked"], dst, merge=mode,
                               stats_out=st)
            st["export_s"] = round(time.time() - t0, 2)
            st.update(census(dst))
            row[sub] = st
        rows.append(row)
        print(json.dumps(row, indent=1))

    print("\n" + "=" * 118)
    hdr = ("{0:<30} {1:>8} {2:>8} {3:>8} {4:>8} {5:>8} {6:>8} {7:>8} {8:>9} "
           "{9:>8}")
    print(hdr.format("file", "variant", "MB", "prims", "meshes", "mats",
                     "shaders", "subsets", "points", "trav s"))
    for r in rows:
        for k in ("src", "raw", "merged"):
            c = r.get(k)
            if not c:
                continue
            print(hdr.format(r["name"] if k == "src" else "", k, c["mb"],
                             c["prims"], c["meshes"], c["materials"],
                             c["shaders"], c["subsets"], c["points"],
                             c["traverse_s"]))
    print("=" * 118)
    jp = os.path.join(out, "remerge.json")
    with open(jp, "w") as fh:
        json.dump(rows, fh, indent=1)
    print("wrote " + jp)


main()
