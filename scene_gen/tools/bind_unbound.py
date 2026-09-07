#!/usr/bin/env python3
"""bind_unbound — give white, materialless meshes the asset's own material.

    AirStack/.venv/bin/python scene_gen/tools/bind_unbound.py <library dir> \
        [--apply] [--only TYPE[,TYPE...]]

`bake._merge_fragments` buckets fragment FACES by bound material, and any face
whose material did not resolve lands in a bucket of its own named
`rubble_unbound`. That bucket was never handed to the fallback binder that the
copy path uses, so it reached disk bound to nothing and rendered WHITE --
measured 2026-08-30 at 50.8% of `SM_Building_21_partial_collapse`.

The bake is fixed, but a fixed baker does nothing for the hundreds of
archetypes already on disk, and re-cutting them is hours. This binds those
meshes to the DOMINANT material of the same file -- the one already covering
the most geometry, which on these assets is the facade -- so the wreck matches
the building it came from. Geometry is untouched; only a binding is added.

Dry by default: pass --apply to write.
"""

from __future__ import annotations

import argparse
import collections
import os
import shutil
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def _survey(stage, retarget=""):
    """(mesh prims needing a binding, dominant material path) for one stage."""
    from pxr import UsdGeom, UsdShade
    weight = collections.Counter()
    loose = []
    for p in stage.Traverse():
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
        n = len(pts) if pts else 0
        subsets = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(p))
        m, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
        if retarget and p.GetName() == retarget:
            loose.append((p, n))
            continue
        if m and m.GetPrim().IsValid():
            weight[str(m.GetPrim().GetPath())] += n
            continue
        if subsets:
            # Bound PER FACE through its subsets, not white. Counting these as
            # unbound is what made `LOD0` look broken on first inspection.
            for sub in subsets:
                sm, _ = UsdShade.MaterialBindingAPI(
                    sub.GetPrim()).ComputeBoundMaterial()
                if sm and sm.GetPrim().IsValid():
                    weight[str(sm.GetPrim().GetPath())] += n // max(len(subsets), 1)
            continue
        if n:
            loose.append((p, n))
    # PREFER THE ASSET'S OWN OUTER MATERIAL. On a heavily damaged rung most
    # faces are CUT faces bound to `FractureCore_*`, so a plain vote by area
    # elects the core -- which is exactly the look this is meant to avoid: the
    # rubble should carry the building's brick, not the inside of the break.
    # Fall back to the core only if the file has nothing else.
    def _skip(path):
        leaf = path.rsplit("/", 1)[-1]
        return leaf.startswith("FractureCore_") or leaf.startswith(
            "InteriorStructure_")

    outer = collections.Counter({k: v for k, v in weight.items()
                                 if not _skip(k)})
    pick = outer or weight
    dominant = pick.most_common(1)[0][0] if pick else ""
    return loose, dominant


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("library")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--only", default="")
    ap.add_argument("--retarget", default="",
                    help="re-bind meshes with this prim name even if they are "
                         "already bound; how a wrong earlier choice is undone")
    a = ap.parse_args(argv)

    from pxr import Usd, UsdShade
    only = {x.strip() for x in a.only.split(",") if x.strip()}
    files = sorted(f for f in os.listdir(a.library)
                   if f.endswith(".usd") and ".orig." not in f)
    hits = fixed = 0
    for fn in files:
        stem = os.path.splitext(fn)[0]
        if only and not any(stem.startswith(t) for t in only):
            continue
        path = os.path.join(a.library, fn)
        stage = Usd.Stage.Open(path)
        loose, dominant = _survey(stage, a.retarget)
        if not loose:
            continue
        hits += 1
        tot = sum(n for _, n in loose)
        names = ", ".join(sorted({p.GetName() for p, _ in loose})[:3])
        print(f"  {stem:<46} {tot:>9,} verts unbound  [{names}]")
        if not a.apply:
            continue
        if not dominant:
            print(f"      SKIP: no bound material anywhere in this file")
            continue
        mat = UsdShade.Material.Get(stage, dominant)
        if not mat or not mat.GetPrim().IsValid():
            print(f"      SKIP: {dominant} is not a Material")
            continue
        for prim, _ in loose:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
        # The baker writes as root inside the container, so the file may not be
        # ours to open for writing; the directory is, so replace it.
        tmp = path + ".bind.tmp.usd"
        if not stage.GetRootLayer().Export(tmp):
            print("      SKIP: export failed")
            continue
        os.unlink(path)
        shutil.move(tmp, path)
        fixed += 1
        print(f"      bound -> {dominant.rsplit('/', 1)[-1]}")
    print(f"\n{hits} archetype(s) with unbound geometry"
          + (f"; {fixed} repaired" if a.apply else "; dry run, pass --apply"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
