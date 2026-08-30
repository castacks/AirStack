#!/usr/bin/env python
"""_gac_region_probe -- THROWAWAY verification for CHANGE 2 (region-only
slicing) and its `region["top"]` upper bound, both in
`detail/gac_storey_slice.py`. For each of SM_Building_02/_01/_24/_06_Small:
slice FULL (no region), then region origin=3 sides=("E",), then region
origin=3 sides=("E","W"), then the SAME origin=3 sides=("E","W") with
top=7 added -- each into its own style name so `ub.STYLES`/`ub.PIECES`
entries from one variant never clobber another's.

Reports, per variant: piece count, per-role breakdown, total shell area (a
`ring_verify.py`-style conservation check against the FULL slice of the same
building -- clipping conserves area, so the two must match to a tiny
tolerance), and the mass-box identity (`ub.footprint` W/D, `_mass_specs`'s
top and `len(levels)`) with vs without the region.

KNOWN FALSE POSITIVE: `SM_Building_06_Small`'s "EWtop7" area check reports
"LEAKY" at ~1.95%, against 0.0000% everywhere else. This is `full` (and the
no-`top` region cut, which still runs `roof_and_parapet` the same way) being
too BIG, not "EWtop7" being too small -- `_ensure_roof`'s synthetic-slab
fallback fires on this building's real top band at every height it is cut
(no natural `ring()` core survives, whole-band or split), and the slab it
adds is genuine EXTRA area with no counterpart in the source mesh, because
the too-thin-for-a-middle guard in `ring()` already returned the WHOLE band,
100% of its real area, as one relabelled piece before the synthesis ever
runs. `region={"top": 7}` never reaches `roof_and_parapet` for this building
at all, so its area (24927.218 m2) matches the RAW, un-sliced source mesh's
own triangle area to six significant figures (measured directly via
`gac_storey_slice.read_mesh` on the bare asset) -- see the SKILL.md "Region-
only slicing" section for the full measurement. Fixing `_ensure_roof` is
outside the scope of the `region["top"]` change; this note exists so the
next reader does not chase the wrong number.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
        /isaac-sim/AirStack/scene_gen/tools/_gac_region_probe.py"
"""
import gc
import sys
import time

import numpy as np
from pxr import Sdf, Usd, UsdGeom

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from detail import gac_storey_slice as gss              # noqa: E402
from detail import urban_building as ub                 # noqa: E402
from disaster import quake_flow as qf                    # noqa: E402

SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")

NAMES = ["SM_Building_02", "SM_Building_01", "SM_Building_24",
         "SM_Building_06_Small"]
REGIONS = [("full", None),
           ("E", {"origin": 3, "sides": ("E",)}),
           ("EW", {"origin": 3, "sides": ("E", "W")}),
           ("EWtop7", {"origin": 3, "top": 7, "sides": ("E", "W")})]


def area(P, tris):
    if tris is None or not len(tris):
        return 0.0
    V = P[tris]
    return float(np.linalg.norm(
        np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0]), axis=1).sum() * 0.5)


def slice_one(name, tag, region):
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Xform.Define(st, "/W/src")
    kid = st.DefinePrim("/W/src/asset")
    kid.GetReferences().AddReference(SEI + name + ".usd")
    st.Load(Sdf.Path("/W/src"))
    UsdGeom.Xformable(kid).AddScaleOp().Set((0.01, 0.01, 0.01))

    style = "{0}_{1}".format(name, tag)
    t0 = time.time()
    pls, g, measured = gss.slice_to_kit(st, "/W/src", "/W/kit", style,
                                        verbose=False, region=region)
    dt = time.time() - t0

    tot_area = 0.0
    n_tri = 0
    for p in pls:
        prim = st.GetPrimAtPath(p["prim_path"])
        if not prim or not prim.IsValid():
            continue
        me = UsdGeom.Mesh(prim)
        pts_attr = me.GetPointsAttr().Get()
        fvi_attr = me.GetFaceVertexIndicesAttr().Get()
        if not pts_attr or not fvi_attr:
            continue
        pts = np.asarray(pts_attr, dtype=float)
        fvi = np.asarray(fvi_attr, dtype=np.int64)
        tris = fvi.reshape(-1, 3)          # write_piece always emits triangles
        tot_area += area(pts, tris)
        n_tri += len(tris)

    by_role = {}
    for p in pls:
        by_role[p["_role"]] = by_role.get(p["_role"], 0) + 1

    spec = ub.STYLES[style]
    W, D = ub.footprint(spec)
    specs = qf._mass_specs(style, 0.0, 0.0, 0.0)
    top = specs[0]["top"]
    n_levels = len(specs[0]["levels"])

    result = {"name": name, "tag": tag, "total": len(pls), "by_role": by_role,
              "area": tot_area, "n_tri": n_tri, "dt": dt, "W": W, "D": D,
              "top": top, "n_levels": n_levels, "measured": measured}
    del st, pls, g
    gc.collect()
    return result


results = {}
for name in NAMES:
    print("=" * 78)
    print("ASSET:", name)
    for tag, region in REGIONS:
        try:
            r = slice_one(name, tag, region)
        except Exception as e:
            import traceback
            print("  [{0}] FAILED: {1}".format(tag, e))
            traceback.print_exc()
            continue
        results[(name, tag)] = r
        print("  [{0:4s}] {1:4d} piece(s)  area={2:10.3f} m2  tri={3:6d}  "
              "W={4:.3f} D={5:.3f} top={6:.3f} levels={7}  {8:.1f}s  "
              "measured={9}  roles: {10}".format(
                  tag, r["total"], r["area"], r["n_tri"], r["W"], r["D"],
                  r["top"], r["n_levels"], r["dt"], r["measured"],
                  "  ".join("{0}={1}".format(k, v)
                           for k, v in sorted(r["by_role"].items()))))
    full = results.get((name, "full"))
    if full is None:
        continue
    for tag in ("E", "EW", "EWtop7"):
        r = results.get((name, tag))
        if r is None:
            continue
        d = 100.0 * abs(r["area"] - full["area"]) / max(1e-9, full["area"])
        mass_ok = (abs(r["W"] - full["W"]) < 1e-6
                  and abs(r["D"] - full["D"]) < 1e-6
                  and abs(r["top"] - full["top"]) < 1e-6
                  and r["n_levels"] == full["n_levels"])
        print("  region_check[{0}]: pieces {1} (region) vs {2} (full)  "
              "area delta {3:.6f}%  ({4})  mass box identical: {5}"
              .format(tag, r["total"], full["total"], d,
                      "OK <=0.0001%" if d <= 0.0001 else "LEAKY", mass_ok))

print("\n" + "=" * 78)
print("SUMMARY")
for name in NAMES:
    for tag, _region in REGIONS:
        r = results.get((name, tag))
        if r is None:
            continue
        print("{0:22s} {1:4s} {2:4d} piece(s)".format(name, tag, r["total"]))
