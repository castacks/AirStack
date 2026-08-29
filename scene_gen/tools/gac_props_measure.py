#!/usr/bin/env python
"""Measure the GreatAmericanCity props into `_plans/gac_props.json`.

    bash scene_gen/tools/usd_python.sh scene_gen/tools/gac_props_measure.py

Roof plant, wall-mounted runs and parapet furniture, with the numbers a placer
needs: size in METRES (through each asset's own `metersPerUnit`, which is 0.01
across this pack), the bbox centre so a prop can be seated by its footprint
rather than by whatever pivot the exporter left, and `z0` so it sits ON the
roof instead of sunk into it.

`kind` is assigned here rather than at placement time because it is a property
of the ASSET: a water tank goes on a roof, a stair goes on a wall, and no
amount of layout context changes that.

MEASURED ON POINTS, NOT ON `UsdGeom.BBoxCache` — see
`.agents/skills/fix-floating-debris/SKILL.md`. `BBoxCache.ComputeWorldBound`
returns the AABB of an AABB: it takes a mesh's LOCAL extent box, transforms
its eight corners, and re-axis-aligns, which over-reports the extent of any
mesh whose geometry sits diagonally in its own local box — the skill's own
number, `tree_Black_Oak_snag/log_017`, read a 0.986 m bbox span against a
0.144 m true points span and reported a piece hanging 42 cm in the air as
`z0 = 0.000`. `gac_props.py`'s every seating decision, and this tool's own
prior version, were built on exactly that call. The fix is the same one the
skill documents: `disaster.bake.world_point_bounds(prim, xcache)`, the tight
world AABB from a mesh's TRANSFORMED POINTS — "the only number that is true"
— aggregated over every mesh in the stage (unlike the skill's single-mesh
debris pieces, several of these props are kit-bashed from more than one
mesh, so no single call to it is the whole story; see `_points_bounds`).
`gac_props_check.py` re-derives this SAME points measurement independently at
placement time rather than trusting this file's output, which is the other
half of the fix: a checker built on the same blind spot as the measurement
cannot see the defect either.

`roof_house` RECORDS ALSO CARRY A `material` — brick/concrete/glass/metal,
read off each bound material's own name (`_classify_material`) — because a
roof house is a STAIR/LIFT BULKHEAD, not a scatter prop, and the wrong one
reads as a mistake, not variety: "don't do roof houses that don't match with
the base building (brick doesn't go with concrete)" (user, 2026-08-29,
pointed at two placements built from `SM_Superior_Construction_04` and
`SM_Superior_Construction_01`). Measured directly: `_01`/`_04` are 35-36%
`M_Bricks_Superior_Construction_Inst` by triangle area (brick); `_02`/`_03`
are the SAME bulkhead mesh with the SAME secondary materials (trim, slab,
metal vents, window glass, all within a percentage point of each other
across all four) but 35-38% `M_Wall_Superior_Construction_Inst` instead — the
one thing that differs between the two pairs is brick vs this pack's plain
"wall" (non-brick precast/stucco) skin; `SM_Glass_Roof` is 100% named glass
materials, no brick/concrete/wall material bound anywhere on it. `gac_props.
roof_props` reads this field and refuses to put a `roof_house` of the wrong
material on a building — see that module's own docstring for the fuller
account and `_classify_material` below for how "material" is actually read.
"""
import json
import os
import sys
import time

import numpy as np
from pxr import Usd, UsdGeom, UsdShade

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import bake  # noqa: E402  — world_point_bounds, see module docstring

ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
        "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "_plans", "gac_props.json")

# kind -> the assets that belong to it. Curated, not pattern-matched: the pack
# has no asset with "escape" in its name, so `SM_Building_Stair` is measured
# here as `wall_stair` purely as a RECORD of what the asset is — `gac_props.py`
# no longer places it. A fire escape built from stacking it was tried,
# reviewed against the built scene, and rejected outright ("I see some fire
# escapes but they don't look right so just remove them" — user, 2026-08-29):
# not gated behind a knob, removed from placement entirely. Keeping the
# measurement costs nothing and stops anyone re-deriving the same "stack
# SM_Building_Stair" idea from scratch and re-discovering the same rejection.
KINDS = {
    "roof_plant": ["SM_Air_Machine", "SM_Air_Tubes_Machine",
                   "SM_Air_Tubes_Machine_Part_02", "SM_Air_Tubes_Machine_Part_03",
                   "SM_Generator_Eletric", "SM_Building_Air"],
    "roof_tank": ["SM_Water_Tank", "SM_Water_Tank_02"],
    "roof_mast": ["SM_Communication_Tower", "SM_Tower"],
    "roof_house": ["SM_Superior_Construction_01", "SM_Superior_Construction_02",
                   "SM_Superior_Construction_03", "SM_Superior_Construction_04",
                   "SM_Glass_Roof"],
    # SM_Steel_Pipe_Plastic is OUT (user, 2026-08-29 scene review, pointed at
    # `/World/stage/generated/roof_prop_61_3059`): its pivot sits 6.16 m below
    # its own geometry (`z0 = -6.16` under the old bbox measurement) — the
    # single largest bbox-vs-points gap of anything in this pack (see the
    # per-prop table this tool prints) and the asset most likely to still be
    # wrong even after the points fix below, since a 6 m offset is not
    # something `BBoxCache` merely over-reports, it is a genuinely displaced
    # pivot. Do not add it back without re-measuring it first.
    "roof_pipe": ["SM_Steel_Pipe", "SM_Construct_Tubes", "SM_Construct_Tubes_Curve"],
    "wall_stair": ["SM_Building_Stair", "SM_Stair"],
    "wall_run": ["SM_Protect_Tube", "SM_Tube", "SM_Tube_Curve",
                 "SM_Tube_Curve_02", "SM_Cable", "SM_Protective_Grid"],
    "wall_door": ["SM_Exit_Door_02"],
}

# (substring, family) for `_classify_material` below — checked in this
# order, first match wins. "wall" is last and deliberately excludes
# "wallback"/"wall_back" (this pack's blank REAR-panel material elsewhere,
# per `gac_faces.py`'s own `BLANK_TOKENS`; it has not been seen on this
# kit's single-mass bulkheads, but the substring is excluded anyway so a
# future asset with one does not silently classify as concrete). "wall" ->
# concrete is MEASURED, not guessed: see the module docstring's account of
# `SM_Superior_Construction_01`/`_04` (brick-dominant) against `_02`/`_03`
# (the identical bulkhead, the identical secondary materials, "wall"-
# dominant instead of brick) — the same mapping `gac_props.py`'s
# `_BUILDING_MATERIAL_FAMILY` independently derives for a BUILDING's own
# cladding, from a completely different measurement (image texture names on
# `_plans/gac_faces.json`, not bound-material names) agreeing with this one.
_MATERIAL_FAMILY = (("brick", "brick"), ("concrete", "concrete"),
                    ("glass", "glass"), ("metal", "metal"), ("wall", "concrete"))


def _mat_source_name(mat_prim):
    """The bound material's own asset basename, read off `info:unreal:
    sourceAsset` wherever it is set inside the material's prim subtree —
    this pack's Unreal-sourced materials carry it directly there (e.g.
    `/Game/.../M_Bricks_Superior_Construction_Inst.M_Bricks_Superior_
    Construction_Inst`; take the asset-path component before the final `.`,
    then its basename). NOT the diffuseColor -> texture-file chain
    `gac_faces.py` uses for a building: that chain returned nothing for
    every subset tried here (probed directly, on this exact pack, before
    writing this — the connection exists in the shading graph but did not
    resolve through `UsdShade.Shader(...).GetInput("file").Get()` the way it
    does for a building's own materials), while `info:unreal:sourceAsset` is
    unambiguous, always present, and IS the artist's own name for the
    material — better evidence than an image basename would be anyway. A
    roof house is also one boxy mass with no front/back elevations to
    reason about the way a building has, so there is no per-side logic to
    port from `gac_faces.py` in the first place — only "which material
    covers the most of it"."""
    for c in Usd.PrimRange(mat_prim):
        attr = c.GetAttribute("info:unreal:sourceAsset")
        if attr and attr.IsValid():
            v = attr.Get()
            if v:
                return str(v).split(".")[0].rsplit("/", 1)[-1]
    return None


def _classify_material(stage, S):
    """(material, evidence_m2) for a `roof_house` asset — the dominant bound
    material FAMILY by triangle area (`brick`/`concrete`/`glass`/`metal`),
    or `(None, {})` if nothing on it matches any family. `evidence_m2` is
    {material_asset_name: area_m2} for every DISTINCT bound material found
    (not yet collapsed into a family), largest first, in real square metres
    (points scaled by *S* — this pack's own `metersPerUnit` — before the
    cross product, same as `gac_faces.py`'s own per-triangle loop) — kept so
    a reader can see exactly what areas produced the verdict without
    re-running this. Which material has the plurality does not actually
    depend on the scale (a uniform per-asset factor scales every triangle
    the same way), but the evidence is reported in real units anyway since
    it is meant to be read, not just compared.
    """
    by_name = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if not pts:
            continue
        V = np.asarray(pts, dtype=np.float64) * S
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [],
                            dtype=np.int64)
        if len(counts) == 0:
            continue
        idx = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [],
                         dtype=np.int64)
        start = np.zeros(len(counts) + 1, dtype=np.int64)
        np.cumsum(counts, out=start[1:])
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
        sub_of = np.full(len(counts), -1, dtype=np.int64)
        names = []
        if subs:
            for si, s in enumerate(subs):
                fi = np.asarray(s.GetIndicesAttr().Get() or [], dtype=np.int64)
                fi = fi[(fi >= 0) & (fi < len(counts))]
                sub_of[fi] = si
                mat, _ = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()
                names.append(_mat_source_name(mat.GetPrim()) if mat else None)
        else:
            mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            names = [_mat_source_name(mat.GetPrim()) if mat else None]
        for f in range(len(counts)):
            b, c = start[f], counts[f]
            if c < 3:
                continue
            # fan triangulation, same as `gac_faces.py`'s own per-face loop
            for j in range(1, c - 1):
                a0, a1, a2 = V[idx[b]], V[idx[b + j]], V[idx[b + j + 1]]
                ar = 0.5 * float(np.linalg.norm(np.cross(a1 - a0, a2 - a0)))
                si = sub_of[f] if subs else 0
                nm = names[si] if 0 <= si < len(names) else None
                by_name[nm] = by_name.get(nm, 0.0) + ar

    fam = {"brick": 0.0, "concrete": 0.0, "glass": 0.0, "metal": 0.0}
    for nm, area in by_name.items():
        n = (nm or "").lower()
        for token, family in _MATERIAL_FAMILY:
            if token == "wall" and ("wallback" in n or "wall_back" in n):
                continue
            if token in n:
                fam[family] += area
                break
    material = max(fam, key=fam.get) if any(fam.values()) else None
    evidence = {nm: round(ar, 1) for nm, ar in
               sorted(by_name.items(), key=lambda kv: -kv[1]) if nm}
    return material, evidence


def _bbox_bounds(stage):
    """(lo, hi) in RAW stage units from `UsdGeom.BBoxCache` — kept only to
    report the delta against the points measurement below; no longer the
    source of truth for anything `gac_props.py` places on."""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    return list(r.GetMin()), list(r.GetMax())


def _points_bounds(stage):
    """(lo, hi) in RAW stage units, aggregated over every Mesh in the stage,
    each measured on its own transformed POINTS via `bake.world_point_bounds`
    — the fix `disaster/bake.py` already carries for exactly this failure,
    reused rather than re-derived (see module docstring). A kit-bashed prop
    (several of these are more than one mesh — a base and a lid, a tank and
    its legs) needs every mesh combined, which is the one thing this tool
    adds on top of `world_point_bounds` itself."""
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    lo = [1e30, 1e30, 1e30]
    hi = [-1e30, -1e30, -1e30]
    found = False
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        b = bake.world_point_bounds(prim, xcache)
        if b is None:
            continue
        found = True
        plo, phi = b
        for k in range(3):
            lo[k] = min(lo[k], plo[k])
            hi[k] = max(hi[k], phi[k])
    return (lo, hi) if found else None


def _dims(lo, hi, S):
    """W/D/H/cx/cy/z0, metric, from RAW-unit (lo, hi) and the stage's own
    `metersPerUnit` — same formulas the old bbox path used, just fed a
    different (lo, hi)."""
    return {
        "W": round((hi[0] - lo[0]) * S, 2), "D": round((hi[1] - lo[1]) * S, 2),
        "H": round((hi[2] - lo[2]) * S, 2),
        "cx": round(0.5 * (lo[0] + hi[0]) * S, 3),
        "cy": round(0.5 * (lo[1] + hi[1]) * S, 3),
        "z0": round(lo[2] * S, 3),
    }


def main():
    out, deltas, t0 = [], [], time.time()
    for kind, names in KINDS.items():
        for nm in names:
            try:
                st = Usd.Stage.Open(ROOT + nm + ".usd")
                st.Load()
            except Exception as exc:
                print("%-32s OPEN FAIL %s" % (nm, exc)); continue
            S = UsdGeom.GetStageMetersPerUnit(st)

            bbox_bounds = _bbox_bounds(st)
            pts_bounds = _points_bounds(st)
            if pts_bounds is None:
                print("%-32s EMPTY (no mesh points found)" % nm); continue

            pts_d = _dims(*pts_bounds, S)
            bbox_d = _dims(*bbox_bounds, S) if bbox_bounds else None
            dz = (pts_d["z0"] - bbox_d["z0"]) if bbox_d else float("nan")
            deltas.append((nm, kind, bbox_d["z0"] if bbox_d else None,
                          pts_d["z0"], dz))

            npts = sum(len(UsdGeom.Mesh(p).GetPointsAttr().Get() or [])
                       for p in st.Traverse() if p.IsA(UsdGeom.Mesh))
            rec = {"name": nm, "kind": kind, "mpu": round(S, 6),
                   # FULL absolute URL, not a path relative to `asset_root` —
                   # `shared.yaml` sets `asset_root` to
                   # `.../Library/Stages/`, but GreatAmericanCity lives under
                   # `.../Projects/SEI-COA/GreatAmericanCity/` (see `ROOT`
                   # above). `dress()` writes this straight into a placement
                   # without ever routing it through `_join_asset_root`, so a
                   # relative fragment here resolved to
                   # `Library/Stages/GreatAmericanCity/...` — a path that does
                   # not exist — and every prop failed to load. `urban_gac.yaml`
                   # carries full `omniverse://` URLs for exactly this reason.
                   "usd": ROOT + nm + ".usd",
                   "points": npts, **pts_d}
            mat_flag = ""
            if kind == "roof_house":
                # a property of the ASSET, same as `kind` itself (see module
                # docstring) — measured once here, not re-derived at
                # placement time the way a building's cladding is.
                material, evidence = _classify_material(st, S)
                rec["material"] = material
                rec["material_evidence_m2"] = evidence
                mat_flag = "  material=%s" % (material or "?")
            out.append(rec)
            flag = "  <-- %+.3f vs bbox" % dz if bbox_d and abs(dz) > 0.02 else ""
            print("%-14s %-30s %6.2f x %6.2f x %6.2f m  base z %7.3f  %6dk pts%s%s"
                  % (kind, nm, rec["W"], rec["D"], rec["H"], rec["z0"],
                     npts // 1000, flag, mat_flag), flush=True)
    json.dump(out, open(os.path.normpath(OUT), "w"), indent=1)
    print("\n%d props in %.0f s -> %s" % (len(out), time.time()-t0,
                                          os.path.normpath(OUT)))

    print("\n%-30s %-12s %10s %10s %10s" % ("name", "kind", "z0(bbox)",
                                            "z0(pts)", "delta"))
    for nm, kind, zb, zp, dz in sorted(deltas, key=lambda r: -abs(r[4]) if r[4] == r[4] else 0):
        zb_s = "%10.3f" % zb if zb is not None else "      n/a"
        dz_s = "%+10.3f" % dz if dz == dz else "       n/a"
        flag = "  ***" if dz == dz and abs(dz) > 0.5 else ("  *" if dz == dz and abs(dz) > 0.02 else "")
        print("%-30s %-12s %s %10.3f %s%s" % (nm, kind, zb_s, zp, dz_s, flag))


if __name__ == "__main__":
    main()
