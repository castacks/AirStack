#!/usr/bin/env python
"""piece_uv_probe — how many TEXTURE TILES does a sliced piece's subset span,
and how much of the 0-1 tile does its soot bake actually cover?

    usd_python.sh piece_uv_probe.py <bake.usd> <regex> [<regex>...]
    usd_python.sh piece_uv_probe.py <bake.usd> --census

Uses `urban_fire._mesh_arrays` + `soot_bake.triangles`/`_corner_uv` — the
exact arrays and indexing `_bind_soot` hands to `soot_bake.uv_position_map` —
so the `st` span printed here is the one the bake rasterises into.

A subset whose faces span much more than one tile is TILED: `uv_position_map`
wraps each triangle into 0-1 and clamps its raster to a single period, so the
bake lands in a fraction of the tile and the renderer repeats the CLEAN
remainder over the whole face. That is why a correctly-bound `sootbake_*.png`
can still read as an unburnt repeating rectangle (user review of the live
row-5 bench, 2026-08-30). `--census` tallies the span over every piece so a
threshold can be picked off the distribution rather than guessed.
"""
import re
import sys

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                            # noqa: E402
from disaster import soot_bake as sb, urban_fire as uf            # noqa: E402

args = sys.argv[2:]
census = "--census" in args
rx = [re.compile(p) for p in args if not p.startswith("--")] or [re.compile(".")]
st = Usd.Stage.Open(sys.argv[1])


def spans(arrays, face_ids):
    """(u_tiles, v_tiles) the subset's corner UVs span, as authored."""
    tri, _f, slot = sb.triangles(arrays["counts"], arrays["indices"], face_ids)
    if tri.shape[0] == 0:
        return None
    uvs = sb._corner_uv(tri, slot, arrays["uv"], arrays["interp"],
                        arrays["uv_indices"]).reshape(-1, 2)
    return (float(uvs[:, 0].max() - uvs[:, 0].min()),
            float(uvs[:, 1].max() - uvs[:, 1].min()))


def texname(prim):
    m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if not m:
        return "(none)", "(none)"
    for c in Usd.PrimRange(m.GetPrim()):
        sh = UsdShade.Shader(c)
        if sh and sh.GetIdAttr().Get() == "UsdPreviewSurface":
            i = sh.GetInput("diffuseColor")
            if i and i.HasConnectedSource():
                ts = UsdShade.Shader(i.GetConnectedSource()[0].GetPrim())
                f = ts.GetInput("file")
                return m.GetPrim().GetName(), str(f.Get() if f else "?"
                                                  ).rsplit("/", 1)[-1]
    return m.GetPrim().GetName(), "(constant)"


hist = {}
for pr in st.Traverse():
    p = pr.GetPath().pathString
    if "/pieces/" not in p or not pr.IsA(UsdGeom.Mesh):
        continue
    hit = any(r.search(p) for r in rx)
    if not (hit or census):
        continue
    arrays = uf._mesh_arrays(pr)
    if arrays is None:
        continue
    subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(pr)))
    if hit and not census:
        print(p)
    for sub in subs:
        ids = [int(k) for k in (sub.GetIndicesAttr().Get() or [])]
        if not ids:
            continue
        s = spans(arrays, ids)
        if s is None:
            continue
        mat, tex = texname(sub.GetPrim())
        sooted = tex.startswith(("sootbake_", "gacsoot_"))
        if hit and not census:
            print("   sub %-16s st span %6.2f x %6.2f tile(s)  %-10s %s"
                  % (sub.GetPrim().GetName(), s[0], s[1],
                     "SOOTED" if sooted else "raw", tex[:52]))
        if census:
            k = "sooted" if sooted else "raw"
            hist.setdefault(k, []).append(max(s))

if census:
    for k in sorted(hist):
        v = np.array(hist[k])
        qs = np.percentile(v, [10, 25, 50, 75, 90, 99])
        print("%-8s %5d subset(s)  max(st span) tiles: p10 %.2f p25 %.2f "
              "p50 %.2f p75 %.2f p90 %.2f p99 %.2f  max %.2f"
              % (k, len(v), qs[0], qs[1], qs[2], qs[3], qs[4], qs[5], v.max()))
        for thr in (1.2, 1.5, 2.0, 3.0):
            print("            > %.1f tiles: %4d (%.0f%%)"
                  % (thr, int((v > thr).sum()), 100.0 * (v > thr).mean()))
