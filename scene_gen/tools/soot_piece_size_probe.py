#!/usr/bin/env python
"""soot_piece_size_probe -- measurement only, touches nothing else.

How much would a SIZE-GATED `SOOT_BAKE_PX_SLICE` (small pieces baked at 128
px instead of 256) actually save on the real GAC bake corpus? Every sliced
GAC piece is exactly one Mesh (see `build-urban-fire-scenes` SKILL.md, "every
sliced piece is exactly one Mesh"), so this walks every Mesh in each bake,
asks whether any of its bound materials carries a `sootbake_*`/`gacsoot_*`
texture (`soot_plume.find_basecolor`, the same lookup `_bind_soot` itself
uses), and buckets the sooted ones by their own world bbox diagonal against
`SOOT_PIECE_SMALL_M` (urban_fire.py, default 6.0 m).

    docker exec isaac-sim bash -c \
        "cd /isaac-sim/AirStack && ./scene_gen/tools/usd_python.sh \
         scene_gen/tools/soot_piece_size_probe.py \
         '/isaac-sim/.cache/fire_bakes/city_138/gac_*.usd'"
"""
import glob
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from pxr import Usd, UsdGeom, UsdShade  # noqa: E402

from disaster import soot_plume as spl  # noqa: E402

SMALL_M = 6.0
PX_FULL = 256
PX_SMALL = 128


def _is_soot_tex(path):
    b = (path or "").rsplit("/", 1)[-1].lower()
    return b.startswith("sootbake_") or b.startswith("gacsoot_")


def _mesh_is_sooted(prim):
    subsets = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
    targets = [s.GetPrim() for s in subsets] or [prim]
    for gp in targets:
        mat = UsdShade.MaterialBindingAPI(gp).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            continue
        _sp, _inp, tex = spl.find_basecolor(mat.GetPrim())
        if _is_soot_tex(tex):
            return True
    return False


def main(pattern):
    paths = sorted(glob.glob(pattern))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=True)
    n_small = n_large = 0
    cur_full_mb = 0.0
    proj_mb = 0.0
    per_bake = []
    for p in paths:
        stage = Usd.Stage.Open(p)
        if stage is None:
            continue
        bs, bl = 0, 0
        for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            if not prim.IsActive() or not prim.IsA(UsdGeom.Mesh):
                continue
            if not _mesh_is_sooted(prim):
                continue
            rng = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            sz = rng.GetSize()
            diag = math.hypot(float(sz[0]), float(sz[1]))
            small = diag < SMALL_M
            if small:
                n_small += 1
                bs += 1
            else:
                n_large += 1
                bl += 1
            cur_mb = (PX_FULL * PX_FULL * 4 * 1.333) / (1024.0 * 1024.0)
            cur_full_mb += cur_mb
            proj_mb += cur_mb * ((PX_SMALL / float(PX_FULL)) ** 2 if small else 1.0)
        per_bake.append((os.path.basename(p), bs, bl))
        print("%-45s small=%-4d large=%-4d" % (os.path.basename(p), bs, bl))
    n = n_small + n_large
    print("\n==== TOTAL ====")
    print("sooted pieces: %d  (small <%.0fm: %d = %.0f%%,  large: %d)" %
          (n, SMALL_M, n_small, 100.0 * n_small / max(1, n), n_large))
    print("current VRAM at fixed %dpx: %.1f MB" % (PX_FULL, cur_full_mb))
    print("projected VRAM with small pieces at %dpx: %.1f MB  (-%.1f MB, -%.0f%%)" %
          (PX_SMALL, proj_mb, cur_full_mb - proj_mb,
           100.0 * (cur_full_mb - proj_mb) / max(1e-9, cur_full_mb)))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1] if len(sys.argv) > 1
                          else "/isaac-sim/.cache/fire_bakes/city_138/gac_*.usd"))
