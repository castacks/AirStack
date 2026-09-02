#!/usr/bin/env python
"""aec_burn_probe.py -- run `disaster.aec_burn.burn_row` on a brownstone row
with bare USD (no Kit, no GPU, safe beside a running sim) and write the 2D
oracle: the soot canvas with the window islands and the facade wall's own
triangles drawn on it.

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \
      ./scene_gen/tools/usd_python.sh scene_gen/tools/aec_burn_probe.py \
      Reference_Brownstone5Row F3 2,3 7"

Args: asset name, level, units (comma list, 1-based; `-` = default), seed.
Env: AEC_BURN_PREVIEW_DIR (default /isaac-sim/.nvidia-omniverse/logs/aec_burn,
which is ~/docker/isaac-sim/logs/aec_burn on the host), AEC_BURN_OUT (texture
cache), AEC_BURN_EXPORT=1 to also export the composed root layer next to the
previews for inspection.
"""
import os
import sys
import time

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Gf, Sdf, Usd, UsdGeom  # noqa: E402
from disaster import aec_burn, gac_fire as gf  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "Reference_Brownstone5Row"
level = sys.argv[2] if len(sys.argv) > 2 else "F3"
units = None
if len(sys.argv) > 3 and sys.argv[3] not in ("-", ""):
    units = tuple(int(q) for q in sys.argv[3].split(","))
seed = int(sys.argv[4]) if len(sys.argv) > 4 else 7
PREV = os.environ.get("AEC_BURN_PREVIEW_DIR") or "/isaac-sim/.nvidia-omniverse/logs/aec_burn"
os.makedirs(PREV, exist_ok=True)

url = gf.asset_url(name, "aec")
print("[probe] asset", url)
t0 = time.time()
stage = Usd.Stage.CreateInMemory()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = stage.DefinePrim("/World", "Xform")
stage.SetDefaultPrim(world)
root = stage.DefinePrim("/World/b0", "Xform")
root.GetReferences().AddReference(url)
src = Usd.Stage.Open(url)
mpu_asset = float(UsdGeom.GetStageMetersPerUnit(src)) or 1.0
xf = UsdGeom.Xformable(root)
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
xf.AddScaleOp().Set(Gf.Vec3f(mpu_asset, mpu_asset, mpu_asset))
print("[probe] composed in {0:.1f} s (asset mpu {1})".format(time.time() - t0, mpu_asset))

# winding sanity on the facade wall: near the front plane, faces with a
# strong across-row normal should point OUT (away from the unit centre)
t0 = time.time()
meas = aec_burn.measure_row(stage, "/World/b0")
print("[probe] measured in {0:.1f} s".format(time.time() - t0))
xfc = UsdGeom.XformCache(Usd.TimeCode.Default())
import numpy as np  # noqa: E402
perp = meas["perp"]
for unit in meas["units"][:2]:
    for mrec in unit["meshes"]:
        if mrec["cat"] != "Walls_Exterior":
            continue
        V, N = aec_burn._mesh_tris_world(mrec["prim"], meas["mpu"], xfc)
        C = V.mean(axis=1)
        strong = np.abs(N[:, perp]) > 0.9
        print("[probe] {0} wall: planes lo {1:.2f} hi {2:.2f}; deck {3:.2f}; "
              "strong-normal face positions p5/p50/p95 lo-half {4} hi-half {5}".format(
                  unit["name"], unit["plane_lo"], unit["plane_hi"], unit["deck_z"],
                  [round(float(v), 2) for v in np.percentile(
                      C[strong & (C[:, perp] < unit["cx" if perp == 0 else "cy"]), perp], [5, 50, 95])],
                  [round(float(v), 2) for v in np.percentile(
                      C[strong & (C[:, perp] >= unit["cx" if perp == 0 else "cy"]), perp], [5, 50, 95])]))
        # WINDING: within the wall's thickness the normal must point AWAY
        # from the wall's mid-plane (outer face outward, inner face inward).
        # ~100 % = consistent, ~50 % = mixed, ~0 % = inverted.
        for lbl, plane in (("lo", unit["plane_lo"]), ("hi", unit["plane_hi"])):
            near = strong & (np.abs(C[:, perp] - plane) < 0.6)
            if near.any():
                agree = float((np.sign(N[near, perp]) == np.sign(C[near, perp] - plane)).mean())
                print("[probe]   {0} faces within 0.6 m of the {1} plane: normal agrees "
                      "with side-of-plane {2:.0%}".format(int(near.sum()), lbl, agree))
    cats = {}
    for mrec in unit["meshes"]:
        k = (mrec["cat"], "int" if aec_burn._is_interior(mrec, unit, perp, unit["deck_z"]) else "ext")
        cats[k] = cats.get(k, 0) + 1
    ext = sorted((k[0], v) for k, v in cats.items() if k[1] == "ext")
    inter = sorted((k[0], v) for k, v in cats.items() if k[1] == "int")
    print("[probe] {0} exterior: {1}".format(unit["name"], ext))
    print("[probe] {0} interior: {1}".format(unit["name"], inter))

t0 = time.time()
plan = aec_burn.plan_row(meas, level=level, units=units, seed=seed)
print("[probe] planned in {0:.1f} s".format(time.time() - t0))
t0 = time.time()
dstats = aec_burn.damage_row(stage, meas, plan)
print("[probe] damaged in {0:.1f} s: {1}".format(time.time() - t0, dstats))
t0 = time.time()
stats = aec_burn.author_row(stage, meas, plan,
                            out_dir=os.environ.get("AEC_BURN_OUT") or os.path.join(PREV, "tex"))
print("[probe] authored in {0:.1f} s: {1}".format(time.time() - t0, stats))
# what is left standing / inactive, per category, on the burning units
import collections  # noqa: E402
dead = collections.Counter()
for unit in plan["burning"]:
    for mrec in unit["meshes"]:
        if mrec.get("dead"):
            dead[mrec["cat"]] += 1
print("[probe] deactivated per category:", dict(dead))
png = os.path.join(PREV, "{0}_{1}_u{2}_s{3}_canvas.png".format(
    name, level, "-".join(str(u) for u in plan["units"]), seed))
aec_burn.preview_png(meas, plan, png)
print("[probe] canvas ->", png)

# a census of what got bound to what
from pxr import UsdShade  # noqa: E402
n_char = n_soot_tri = 0
for prim in Usd.PrimRange(root):
    if prim.IsA(UsdGeom.Mesh):
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if mat and mat.GetPath().name == "char":
            n_char += 1
        if prim.GetPath().name in ("soot", "char_walls"):
            fvc = UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get()
            print("[probe] layer {0}: {1} face(s), bound {2}".format(
                prim.GetName(), len(fvc), mat.GetPath() if mat else None))
print("[probe] meshes bound to char:", n_char)
if os.environ.get("AEC_BURN_EXPORT", "0") == "1":
    out = os.path.join(PREV, "{0}_{1}.usda".format(name, level))
    stage.GetRootLayer().Export(out)
    print("[probe] exported", out)
print("AEC BURN PROBE DONE")
