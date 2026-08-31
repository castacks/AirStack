#!/usr/bin/env python
"""cascade_check_probe -- does an INACTIVE prim (something the SAME
`deactivate_airborne` batch already turned off) sit right behind one of the
still-active floating stamps, close enough to have been its `backing_path`
before it was turned off?

`Usd.PrimRange`'s default predicate skips inactive prims, so a normal
`_judge_candidates` re-probe never sees them -- but `SetActive(False)` does
not remove the prim, so its geometry is still sitting on the stage at
whatever position it was baked to, and `stage.GetPrimAtPath` + a Mesh schema
adapter can read it directly regardless of active state. This script walks
EVERY mesh under `fb.BAKE_ROOT` including inactive ones, finds every
INACTIVE candidate (by name, `fb._match_prefix`), and reports the closest
one (world-space centroid distance) to each of the named still-floating
prims -- if any inactive candidate sits within a few tens of cm of one,
that is exactly the kind of thing a live one-shot judge could have used as
"backing" for the floater, one batch before removing it.

    docker exec isaac-sim bash -c \
        "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
         /isaac-sim/AirStack/scene_gen/tools/cascade_check_probe.py \
         /isaac-sim/.cache/fire_bakes_dtc/gac_SM_Building_26_F4_s162.usd"

Read-only. Never calls SetActive or Save.
"""
import sys

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

from disaster import fire_bake as fb                            # noqa: E402


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else (
        "/isaac-sim/.cache/fire_bakes_dtc/gac_SM_Building_26_F4_s162.usd")
    names = sys.argv[2:] or [
        "spall_g5_115", "spallhalo_g5_114", "spall_g5_97", "spall_g5_70",
        "spallhalo_g5_69", "spallhalo_g5_29", "spall_g5_30",
        "spallhalo_g5_47", "spall_g5_24", "spallhalo_g5_23", "spall_g5_40",
        "spallhalo_g5_39", "spallhalo_g5_16", "spall_g5_17",
    ]

    import numpy as np
    from pxr import Sdf, Usd, UsdGeom

    stage = Usd.Stage.Open(path)
    xf = UsdGeom.XformCache()

    root = stage.GetPrimAtPath(Sdf.Path(fb.BAKE_ROOT))
    active_c, inactive_c = 0, 0
    inactive_candidates = []   # (path, prefix, centroid)
    targets = {}               # leaf name -> centroid

    for p in Usd.PrimRange(root, Usd.PrimAllPrimsPredicate):
        if not p.IsA(UsdGeom.Mesh):
            continue
        active = p.IsActive()
        active_c += 1 if active else 0
        inactive_c += 1 if not active else 0
        pfx = fb._match_prefix(p.GetPath().name)
        leaf = p.GetPath().name
        want = leaf in names
        if not (want or (pfx is not None and not active)):
            continue
        mesh = UsdGeom.Mesh(p)
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        M = np.array(xf.GetLocalToWorldTransform(p), dtype=float)
        W = np.asarray(pts, dtype=float) @ M[:3, :3] + M[3, :3]
        c = W.mean(axis=0)
        if want:
            targets[leaf] = c
        if pfx is not None and not active:
            inactive_candidates.append((str(p.GetPath()), pfx, c))

    print("[cascade] {0}: {1} active mesh(es), {2} INACTIVE mesh(es) under "
          "{3}".format(path.rsplit("/", 1)[-1], active_c, inactive_c,
                       fb.BAKE_ROOT))
    print("[cascade] {0} inactive CANDIDATE mesh(es) found (deactivated by "
          "the same airborne pass, still resident on the stage)"
          .format(len(inactive_candidates)))
    print("[cascade] {0}/{1} named floater(s) found on this stage\n"
          .format(len(targets), len(names)))

    for leaf, c in targets.items():
        best = None
        for ip, ipfx, ic in inactive_candidates:
            d = float(np.linalg.norm(ic - c))
            if best is None or d < best[0]:
                best = (d, ip, ipfx)
        if best is None:
            print("  {0:<24} no inactive candidate on the stage at all"
                  .format(leaf))
            continue
        d, ip, ipfx = best
        flag = "  <-- WITHIN fb._BACKING_MAX_M ({0} m)".format(
            fb._BACKING_MAX_M) if d <= fb._BACKING_MAX_M else ""
        print("  {0:<24} nearest INACTIVE candidate: {1:<45} ({2})  "
              "dist={3:.3f} m{4}".format(leaf, ip, ipfx, d, flag))


if __name__ == "__main__":
    sys.exit(main())
