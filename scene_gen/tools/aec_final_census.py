"""aec_final_census -- one combined offline (bare USD, no Kit) proof for the
AEC MDL soot-overlay fix: runs `gac_fire.burn_gac` end to end on the CURRENT
code and reports, on the live in-memory stage:

  1. material-class census (mdl vs preview) and a ShellFallbackLooks count
     across every sliced piece/subset, plus the same broken out for '_x'
     (region-cut merged) pieces specifically.
  2. fit-out (slab_/col_/part_) bbox vs the shell (pieces/*) bbox --
     authoring-time only, no physics settle (see fire_bake.py's own
     `_CANDIDATE_PREFIXES` note on `part_` for why a post-settle check needs
     Kit and cannot run here).
  3. the soot-routing note line itself (flat-material-fallback count, which
     must be 0 now that MDL materials are pre-baked into the overlay set).

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \
      ./scene_gen/tools/usd_python.sh scene_gen/tools/aec_final_census.py \
      Reference_Brownstone5Row F3 1 14"
"""
import random
import sys

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                        # noqa: E402
from disaster import fracture, gac_fire as gf, urban_fire as uf  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "Reference_Brownstone5Row"
level = sys.argv[2] if len(sys.argv) > 2 else "F3"
origin = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] else None
seed = int(sys.argv[4]) if len(sys.argv) > 4 else 7

fracture.ensure_vtk(verbose=False)
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Scope.Define(st, "/W/bench")
cell = "/W/bench/g0"
UsdGeom.Xform.Define(st, cell)
mats = uf.materials(st, "/W/bench")
rng = random.Random(seed)
nrng = np.random.default_rng(seed)

ctx = gf.burn_gac(st, cell, "aec:" + name, level, rng, nrng, mats, "g0",
                  flow_root=None, mat_cache={}, ssf=1.0, origin=origin,
                  use_baked_kit=False, verbose=False)
print("=== {0} {1} (seed={2} origin={3}) ===".format(name, level, seed, origin))
for n in ctx["notes"]:
    if any(k in n for k in ("smoke:", "gac:")):
        print("  note:", n)

# ---------------------------------------------------------------------------
# 1. material class + ShellFallback census
# ---------------------------------------------------------------------------
def classify(mat_prim):
    has_mdl = has_prev = False
    for p in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(p)
        if not sh:
            continue
        if sh.GetIdAttr().Get() == "UsdPreviewSurface":
            has_prev = True
        try:
            if sh.GetSourceAsset("mdl"):
                has_mdl = True
        except Exception:
            pass
    if has_mdl and has_prev:
        return "both"
    return "mdl" if has_mdl else ("preview" if has_prev else "neither")


mesh_hist, total_mesh, shellfb, x_total, x_fb = {}, 0, 0, 0, 0
for prim in Usd.PrimRange(st.GetPrimAtPath(cell + "/pieces")):
    if not prim.IsA(UsdGeom.Mesh):
        continue
    total_mesh += 1
    is_x = prim.GetName().startswith("wall_x") or "_x_" in prim.GetName()
    if is_x:
        x_total += 1
    subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
    targets = ([s.GetPrim() for s in subs] or [prim])
    piece_fb = False
    for t in targets:
        mb = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
        if not mb or not mb.GetPrim().IsValid():
            mesh_hist["unbound"] = mesh_hist.get("unbound", 0) + 1
            continue
        mpath = str(mb.GetPrim().GetPath())
        if "ShellFallbackLooks" in mpath:
            shellfb += 1
            piece_fb = True
            mesh_hist["shellfallback"] = mesh_hist.get("shellfallback", 0) + 1
        else:
            c = classify(mb.GetPrim())
            mesh_hist[c] = mesh_hist.get(c, 0) + 1
    if is_x and piece_fb:
        x_fb += 1

print("--- piece/subset material census ({0} mesh prim(s)) ---".format(total_mesh))
for k, v in sorted(mesh_hist.items(), key=lambda kv: -kv[1]):
    print("  {0:14s} {1}".format(k, v))
print("--- '_x' (region-cut merged) pieces: {0} total, {1} with >=1 "
      "ShellFallback subset ---".format(x_total, x_fb))

# ---------------------------------------------------------------------------
# 2. fit-out vs shell bbox (authoring-time, no physics)
# ---------------------------------------------------------------------------
bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
shell_lo = np.full(3, np.inf)
shell_hi = np.full(3, -np.inf)
for p in Usd.PrimRange(st.GetPrimAtPath(cell + "/pieces")):
    if not p.IsA(UsdGeom.Mesh):
        continue
    r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
    if r.IsEmpty():
        continue
    mn, mx = r.GetMin(), r.GetMax()
    shell_lo = np.minimum(shell_lo, [mn[0], mn[1], mn[2]])
    shell_hi = np.maximum(shell_hi, [mx[0], mx[1], mx[2]])

fit = ctx.get("fit") or {}
worst = 0.0
n_checked = 0
n_over = 0
for group_name, items in (("slabs", (fit.get("slabs") or {}).values()),
                          ("columns", [c for cs in (fit.get("columns") or {}).values() for c in cs]),
                          ("partitions", fit.get("partitions") or [])):
    for p in items:
        if not p:
            continue
        prim = st.GetPrimAtPath(p)
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        mn, mx = r.GetMin(), r.GetMax()
        n_checked += 1
        d_lo = float((shell_lo - np.array([mn[0], mn[1], mn[2]])).max())
        d_hi = float((np.array([mx[0], mx[1], mx[2]]) - shell_hi).max())
        w = max(d_lo, d_hi)
        if w > 0.15:
            n_over += 1
            print("  OVERSHOOT {0}: {1}  d_lo={2:.3f} d_hi={3:.3f}".format(
                group_name, p, d_lo, d_hi))
        worst = max(worst, w)

print("--- fit-out ({0} prim(s) checked, authoring-time / no physics) vs "
      "shell bbox ---".format(n_checked))
print("  shell bbox: lo={0} hi={1}".format(shell_lo, shell_hi))
print("  worst fit-out excursion past shell bbox: {0:.4f} m ({1} prim(s) "
      "> 0.15 m over)".format(worst, n_over))
