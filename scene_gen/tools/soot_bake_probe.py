#!/usr/bin/env python
"""soot_bake_probe.py — bake a SYNTHETIC soot skin through a real kit module's
UVs and write the result, so the atlas mapping can be judged by eye before an
Isaac launch.

    docker exec isaac-sim bash -c \\
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
       /isaac-sim/AirStack/scene_gen/tools/soot_bake_probe.py [NAME ...]"

Bare USD, no SimulationApp. The skin is a test pattern, not a fire: alpha 1
wherever the wall is higher than SOOT_ABOVE_M, plus a vertical stripe at
STRIPE_U (metres along the S side). So in every baked map the soot must sit on
exactly the texels of faces above that height (the top part of the module,
wherever the atlas put it) and the stripe must appear as a vertical band —
and NOWHERE else. The unbaked base map is written beside it for comparison.

Outputs land in /isaac-sim/.nvidia-omniverse/logs/soot_bake_probe/ (host:
~/docker/isaac-sim/logs/soot_bake_probe/).
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

from pxr import Usd, UsdGeom, UsdShade                       # noqa: E402
from detail import urban_building as ub                     # noqa: E402
from disaster import soot_bake as sb                        # noqa: E402
from disaster import soot_plume as spl                      # noqa: E402
from disaster import urban_fire as uf                       # noqa: E402

OUT = "/isaac-sim/.nvidia-omniverse/logs/soot_bake_probe"
SOOT_ABOVE_M = 2.0
STRIPE_U = (1.0, 1.6)
PX = 512
DEFAULT = ["SM_MBuilding01_Facade_A", "SM_MBuilding04_Facade_A",
           "SM_MBuilding04_Facade_B", "SM_MBuilding01_FirstFloor_A"]


def synthetic_skin():
    ppm = 40.0
    per, H = 100.0, 30.0
    h, w = int(H * ppm), int(per * ppm)
    rgba = np.zeros((h, w, 4), dtype=np.float32)
    rgba[..., :3] = np.asarray(spl.SOOT_DARK, dtype=np.float32)
    rows_above = int((H - SOOT_ABOVE_M) * ppm)
    rgba[:rows_above, :, 3] = 1.0
    c0, c1 = int(STRIPE_U[0] * ppm), int(STRIPE_U[1] * ppm)
    rgba[:, c0:c1, 3] = 1.0
    return {"rgba": rgba, "ppm": ppm, "per": per, "H": H, "z0": 0.0,
            "offsets": {"S": 0.0, "E": 30.0, "N": 50.0, "W": 80.0}}


def main():
    from PIL import Image

    os.makedirs(OUT, exist_ok=True)
    sk = synthetic_skin()
    m = {"W": 30.0, "D": 20.0, "cx": 15.0, "cy": 10.0, "yaw": 0.0}
    names = sys.argv[1:] or DEFAULT
    for name in names:
        st = Usd.Stage.Open(ub._usd(name))
        if not st:
            print("== {0}: cannot open".format(name))
            continue
        xfc = UsdGeom.XformCache()
        for prim in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            arrays = uf._mesh_arrays(prim)
            if arrays is None:
                print("== {0} {1}: no UVs".format(name, prim.GetPath()))
                continue
            Mg = xfc.GetLocalToWorldTransform(prim)
            M = np.array([[float(Mg[r][c]) for c in range(4)] for r in range(4)])
            subsets = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
            targets = [(s.GetPrim(), s) for s in subsets] or [(prim, None)]
            for t, sub in targets:
                bound = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
                bprim = bound.GetPrim() if bound else None
                sh_path, inp, tex = spl.find_basecolor(bprim)
                face_ids = None
                if sub is not None:
                    face_ids = [int(k) for k in (sub.GetIndicesAttr().Get() or [])]
                t0 = time.time()
                pos, mask = sb.uv_position_map(
                    arrays["points"], arrays["counts"], arrays["indices"],
                    arrays["uv"], arrays["interp"], arrays["uv_indices"],
                    face_ids=face_ids, px=PX)
                t1 = time.time()
                base = spl._read_rgb(tex) if tex else None
                if base is None:
                    base = np.full((64, 64, 3), 0.6, dtype=np.float32)
                out = sb.bake_module(sk, "S", m, M, pos, mask, base, px=PX)
                t2 = time.time()
                sub_name = sub.GetPrim().GetName() if sub is not None else "mesh"
                tag = "{0}_{1}".format(name, sub_name)
                Image.fromarray((np.clip(out, 0, 1) * 255 + 0.5).astype(np.uint8)).save(
                    os.path.join(OUT, tag + "_baked.png"))
                Image.fromarray((mask * 255).astype(np.uint8)).save(
                    os.path.join(OUT, tag + "_mask.png"))
                bh, bw = base.shape[0], base.shape[1]
                Image.fromarray((np.clip(base, 0, 1) * 255 + 0.5).astype(np.uint8)).save(
                    os.path.join(OUT, tag + "_base.png"))
                world = pos[mask] @ M[:3, :3] + M[3, :3]
                print("== {0}: {1} faces covered {2:.1%} of texels; base {3}x{4} ({5}); "
                      "z of covered texels {6:.2f}..{7:.2f}; posmap {8:.2f}s bake {9:.2f}s"
                      .format(tag, len(face_ids) if face_ids is not None else len(arrays["counts"]),
                              float(mask.mean()), bw, bh,
                              (tex or "no texture").rsplit("/", 1)[-1],
                              float(world[:, 2].min()), float(world[:, 2].max()),
                              t1 - t0, t2 - t1))
    print("[soot_bake_probe] ->", OUT)


if __name__ == "__main__":
    main()
