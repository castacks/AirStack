#!/usr/bin/env python
"""roof_flatness.py — is a non-GAC building's roof actually FLAT?

    bash scene_gen/tools/usd_python.sh scene_gen/tools/roof_flatness.py [--config downtown_gac]

`gac_props.dress()` may only put roof plant/tanks/masts on a building whose
basename is named explicitly in `building_props.flat_roof` — nothing about a
bounding box says whether the roof under it is a flat massing lid or a
pitched/stepped/domed one, and a plant box on a ridge line reads worse than a
bare roof. This measures it instead of guessing.

METHOD. Reuses `gac_faces.py`'s own technique — bin every triangle by its
area-weighted normal — but keeps the set THAT script discards:
`n_z > UP_THRESHOLD` (0.72, the same split), the upward-facing "roof or
floor" triangles it excludes because they aren't an elevation. Of that
upward area, this reports what share sits within `ROOF_TOL_M` of the asset's
own max Z. A flat-roofed massing shell has essentially all its upward area
right at the top — near 1.0. A pitched, stepped, or gabled roof spreads that
area down the slope, well below 1.0, in proportion to how much of the roof
actually slopes.

RESOLUTION. Every basename's (usd, scale) is resolved the way the generator
itself would — `scene_generator._normalize_usd_list` against the SAME
`usds.buildings.*` pools a real compiled config carries — rather than a
guessed path or a guessed scale. That is also what makes this immune to the
building pool changing size: it measures whatever is actually IN the pool
today, GAC included (kept, unscored, as an anchor — every GAC roof is known
flat by construction, so its numbers show what "near 1.0" looks like on this
same metric before trusting it on an unfamiliar asset).
"""
import argparse
import os
import sys

import numpy as np
from pxr import Usd, UsdGeom

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import scene_generator as sg                                     # noqa: E402
from compile_disaster import resolve_config_path, compile_spec, DEFAULT_BASE  # noqa: E402
import yaml                                                       # noqa: E402

UP_THRESHOLD = 0.72   # same roof/floor split `gac_faces.py` uses, mirrored
ROOF_TOL_M = 1.5      # "near the top" — same order as gac_props.PARAPET_M


def _basename(path):
    nm = str(path).rsplit("/", 1)[-1]
    for ext in (".usdc", ".usda", ".usd"):
        if nm.endswith(ext):
            return nm[: -len(ext)]
    return nm


def resolved_pool(cfg):
    """basename -> (usd_path, scale) for every entry in every
    `usds.buildings.*` pool of a compiled config. First occurrence wins if a
    basename appears in more than one pool at the same scale (GAC's own
    entries are absent here — this asset set restricts `buildings.*` to
    non-GAC stock, and GAC is handled by `gac_props.load()`'s own measured
    JSON, not this resolution path)."""
    asset_scale = float(cfg.get("asset_scale", 1.0))
    asset_root = str(cfg.get("asset_root", "") or "")
    out = {}
    for pool_name, lst in (cfg.get("usds", {}).get("buildings", {}) or {}).items():
        paths, sc_ov, au_ov, yo_ov, tag_ov = sg._normalize_usd_list(
            lst, asset_scale, asset_root)
        for p in paths:
            nm = _basename(p)
            out.setdefault(nm, (p, sc_ov.get(p, asset_scale), pool_name))
    return out


def measure(path, scale):
    """(max_z_m, upward_area_m2, near_top_area_m2, tri_count) or None."""
    try:
        st = Usd.Stage.Open(path)
    except Exception as exc:
        print(f"  OPEN FAIL: {exc}")
        return None
    if st is None:
        print("  OPEN FAIL: Stage.Open returned None")
        return None
    st.Load()

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                              useExtentsHint=True)
    rng = cache.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    if rng.IsEmpty():
        print("  EMPTY bbox")
        return None
    max_z = rng.GetMax()[2] * scale

    up_area, near_area, tri_n = 0.0, 0.0, 0
    for prim in st.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        counts = me.GetFaceVertexCountsAttr().Get()
        idxs = me.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        # WORLD space within this stage — a kit-bashed building is many mesh
        # prims each with their own local transform (a roof piece offset from
        # a wall piece), so this has to be per-prim, not a single stage-wide
        # points array the way a one-mesh GAC building lets `gac_faces.py`
        # get away with reading just the first Mesh it finds.
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        V = np.array([xf.Transform(p) for p in pts], dtype=np.float64) * scale
        counts = np.asarray(counts, dtype=np.int64)
        idx = np.asarray(idxs, dtype=np.int64)
        start = np.zeros(len(counts) + 1, dtype=np.int64)
        np.cumsum(counts, out=start[1:])
        for f in range(len(counts)):
            b, c = start[f], counts[f]
            if c < 3:
                continue
            for j in range(1, c - 1):
                a3, p2, p3 = V[idx[b]], V[idx[b + j]], V[idx[b + j + 1]]
                n = np.cross(p2 - a3, p3 - a3)
                ar = 0.5 * float(np.linalg.norm(n))
                if ar <= 1e-9:
                    continue
                nz = n[2] / (2.0 * ar)
                if nz <= UP_THRESHOLD:
                    continue           # not upward-facing: wall, floor, or steep slope
                up_area += ar
                tri_n += 1
                cen_z = (a3[2] + p2[2] + p3[2]) / 3.0
                if max_z - cen_z <= ROOF_TOL_M:
                    near_area += ar
    return max_z, up_area, near_area, tri_n


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="downtown_gac")
    ap.add_argument("names", nargs="*",
                    help="basenames to measure; default is every non-GAC "
                         "name in the config's usds.buildings.* pools")
    a = ap.parse_args()

    path = resolve_config_path(a.config)
    cfg = compile_spec(yaml.safe_load(open(path)),
                       yaml.safe_load(open(DEFAULT_BASE)))
    cfg = sg.resolve_asset_set(cfg, path)

    pool = resolved_pool(cfg)
    from detail import gac_props
    _, _, gac_dims = gac_props.load()

    if a.names:
        names = a.names
    else:
        names = sorted(nm for nm in pool if nm not in gac_dims)

    print(f"[roof_flatness] config={a.config}  candidates={len(names)}\n")
    rows = []
    for nm in names:
        if nm not in pool:
            print(f"{nm:32s}  NOT IN POOL (asset set changed?)")
            continue
        path_, scale, pool_name = pool[nm]
        r = measure(path_, scale)
        if r is None:
            print(f"{nm:32s}  ({pool_name:8s})  MEASURE FAILED")
            continue
        max_z, up_area, near_area, tri_n = r
        frac = (near_area / up_area) if up_area > 1e-6 else None
        rows.append((nm, pool_name, max_z, up_area, near_area, frac, tri_n))
        frac_s = f"{frac:5.3f}" if frac is not None else "  n/a"
        print(f"{nm:32s}  ({pool_name:8s})  max_z={max_z:7.1f}m  "
              f"up_area={up_area:9.1f}m2  near_top_frac={frac_s}  "
              f"({tri_n} up-facing tris)")

    print("\n[roof_flatness] sorted by near_top_frac (flattest first):")
    for nm, pool_name, max_z, up_area, near_area, frac, tri_n in sorted(
            rows, key=lambda r: (r[5] is None, -(r[5] or 0.0))):
        frac_s = f"{frac:5.3f}" if frac is not None else "  n/a"
        print(f"  {frac_s}  {nm:32s} ({pool_name})")


if __name__ == "__main__":
    main()
