#!/usr/bin/env python
"""_t_shell_probe.py — are the kit modules open shells, and how open?

Round 3, agent T. The claim the whole "solidify" work rests on is that a kit
wall is a single surface with no thickness, so every Voronoi cell cut out of it
is a plate. This measures it instead of assuming it, and DUMPS a handful of
representative meshes to OBJ so `solidify` can be developed on the host with
plain trimesh (no Kit, no GPU, no Nucleus).

Run through the same bare-pxr wrapper `measure_fences.sh` uses:

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_t_shell_probe.py

Env:
    T_USD     archetype/stage to open  (default assets/archetypes_quake/bld_commercial_DG0.usd)
    T_OUT     json report path
    T_DUMP    directory for the OBJ dumps ("" = don't dump)
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_GEN = os.path.dirname(HERE)
REPO = os.path.dirname(SCENE_GEN)

USD = os.environ.get("T_USD", os.path.join(
    SCENE_GEN, "assets", "archetypes_quake", "bld_commercial_DG0.usd"))
OUT = os.environ.get("T_OUT", "/tmp/_t_shell_probe.json")
DUMP = os.environ.get("T_DUMP", "/tmp/_t_dump")


def tri(counts, indices):
    faces = []
    k = 0
    for c in counts:
        c = int(c)
        if c >= 3:
            for i in range(1, c - 1):
                faces.append((indices[k], indices[k + i], indices[k + i + 1]))
        k += c
    return np.asarray(faces, dtype=np.int64)


def boundary_edges(faces):
    """Count edges used by exactly one face (an open shell has many)."""
    import collections
    cnt = collections.Counter()
    for a, b, c in faces:
        for e in ((a, b), (b, c), (c, a)):
            cnt[(min(e), max(e))] += 1
    open_e = sum(1 for v in cnt.values() if v == 1)
    return open_e, len(cnt)


def main():
    from pxr import Usd, UsdGeom, UsdShade

    stage = Usd.Stage.Open(USD)
    if not stage:
        print("cannot open", USD)
        return 1
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    print("stage", USD, "metersPerUnit", mpu)

    xf = UsdGeom.XformCache()
    rows = []
    dumped = 0
    if DUMP:
        try:
            os.makedirs(DUMP)
        except OSError:
            pass
    for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get()
        cnt = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not cnt or not idx:
            continue
        f = tri(cnt, idx)
        if not len(f):
            continue
        mat = np.array(xf.GetLocalToWorldTransform(p), dtype=float)
        v = np.asarray([[q[0], q[1], q[2]] for q in pts], dtype=float)
        v = v @ mat[:3, :3] + mat[3, :3]
        ext = v.max(0) - v.min(0)
        oe, te = boundary_edges(f)
        try:
            bm = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
            mname = bm.GetPath().name if bm else ""
        except Exception:
            mname = ""
        row = dict(path=str(p.GetPath()), faces=int(len(f)), verts=int(len(v)),
                   open_edges=int(oe), edges=int(te),
                   extents=[round(float(x), 4) for x in ext],
                   thin=round(float(np.min(ext)), 4),
                   mat=mname)
        rows.append(row)
        # dump the first few of each "shape class" for offline work
        if DUMP and dumped < 40:
            name = "{0:03d}_{1}".format(dumped, str(p.GetPath()).strip("/").replace("/", "_"))[:120]
            with open(os.path.join(DUMP, name + ".obj"), "w") as fh:
                for q in v:
                    fh.write("v {0} {1} {2}\n".format(*q))
                for a, b, c in f:
                    fh.write("f {0} {1} {2}\n".format(a + 1, b + 1, c + 1))
            row["obj"] = name + ".obj"
            dumped += 1

    rows.sort(key=lambda r: -r["faces"])
    n_open = sum(1 for r in rows if r["open_edges"] > 0)
    print("meshes {0}, open shells {1} ({2:.0f}%)".format(
        len(rows), n_open, 100.0 * n_open / max(1, len(rows))))
    for r in rows[:45]:
        print("  {0:>6} f  open {1:>5}/{2:<6}  thin {3:>7.4f}  ext {4}  {5}  {6}".format(
            r["faces"], r["open_edges"], r["edges"], r["thin"], r["extents"],
            r["mat"][:22], r["path"][-58:]))
    with open(OUT, "w") as fh:
        json.dump(rows, fh, indent=1)
    print("-> ", OUT, " objs ->", DUMP, dumped)
    return 0


if __name__ == "__main__":
    sys.exit(main())
