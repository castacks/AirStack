"""aec_overshoot_detail_probe — which specific fit_interior-authored prims
(name PREFIX match: slab_/col_/part_/prop_, not path substring) sit outside
the shell's own measured bbox, on an already-baked AEC fire bake. Prints
each offending prim's path, kind and how far outside it sits per axis.

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \
      ./scene_gen/tools/usd_python.sh scene_gen/tools/aec_overshoot_detail_probe.py \
      /isaac-sim/.cache/fire_bakes/aec_Reference_Brownstone5Row_F3_s38.usd"
"""
import sys

import numpy as np


def main(path, tol=0.15):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(path)
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    root = stage.GetPseudoRoot()

    # shell bbox: every Mesh prim whose NAME starts with a shell-role prefix
    shell_prefixes = ("wall_", "corner_", "pier_", "roof_", "parapet_")
    fit_prefixes = ("slab_", "col_", "part_", "prop_")

    shell_lo = np.full(3, np.inf)
    shell_hi = np.full(3, -np.inf)
    fit_items = []
    for prim in Usd.PrimRange(root):
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Xformable)):
            continue
        name = prim.GetName()
        is_shell = any(name.startswith(p) for p in shell_prefixes)
        is_fit = any(name.startswith(p) for p in fit_prefixes)
        if not (is_shell or is_fit):
            continue
        r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        mn, mx = r.GetMin(), r.GetMax()
        lo = np.array([mn[0], mn[1], mn[2]])
        hi = np.array([mx[0], mx[1], mx[2]])
        if is_shell:
            # only leaf mesh prims count toward the shell envelope -- an
            # Xform ancestor would double count the same geometry
            if prim.IsA(UsdGeom.Mesh):
                shell_lo = np.minimum(shell_lo, lo)
                shell_hi = np.maximum(shell_hi, hi)
        if is_fit and prim.IsA(UsdGeom.Mesh):
            fit_items.append((str(prim.GetPath()), name, lo, hi))

    print("=== FILE:", path)
    print("shell bbox: lo={0}  hi={1}".format(shell_lo, shell_hi))
    print("{0} fit-out mesh prim(s) checked".format(len(fit_items)))

    over = []
    for p, name, lo, hi in fit_items:
        d_lo = shell_lo - lo    # positive = pokes out below/left of shell lo
        d_hi = hi - shell_hi    # positive = pokes out above/right of shell hi
        worst = max(float(d_lo.max()), float(d_hi.max()))
        if worst > tol:
            over.append((worst, p, name, d_lo, d_hi))
    over.sort(reverse=True)
    print("{0} fit-out prim(s) exceed the shell bbox by > {1} m on some axis:"
          .format(len(over), tol))
    for worst, p, name, d_lo, d_hi in over[:40]:
        print("  {0:7.3f} m  {1}  (name={2})  d_lo={3}  d_hi={4}".format(
            worst, p, name, d_lo, d_hi))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)
