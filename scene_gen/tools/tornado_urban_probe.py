#!/usr/bin/env python
"""tornado_urban_probe — the urban-tornado ladder end to end on a REAL merged
building, on a bare USD stage (no Kit, no Flow, no physics): place, slice,
describe, plan, apply, export. The `gac_burn_probe.py` of this pipeline
(`_plans/urban_tornado_plan.md` §3).

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/tornado_urban_probe.py SM_Building_02 T3 [SEED] [BEARING_DEG]"

    # a downtowncity block, same call with the pack prefix
    ... tornado_urban_probe.py dtc:Amar_Tower T2

Runs in the container ONLY (VTK + the Nucleus resolver), never inside a
`SimulationApp`. It takes `gac_storey_slice.slice_lock` while it slices, so
do not run it beside a live Kit slice.

WHAT IT PRINTS, in order: the slice (pieces by role / storeys / `n_sub`), the
element table's field coverage (every `_side`/`_role`/`_storey`/`_bay`/
`_size` the planner's `_Grid` relies on — so a real slice that does not
carry a field the synthetic fixture does is caught HERE, before a bake),
the wind the ladder was given, the plan's `stats` and `notes`, the debris
bbox and per-class counts, and the export path.

DEGRADES GRACEFULLY while the pipeline is being built: with
`disaster.tornado_urban` absent it stops after the element-table census;
with `disaster.tornado_urban_usd` absent it stops after the plan. Each stop
is announced on its own line.

The export keeps `<cell>/src` (the merged source) in the file — this is a
probe for GEOMETRY COUNTS, not a bake; a cold open of the .usd renders the
sliced pieces white unless `fire_bake.rehome_for_export` has run, which the
bake launcher (next round) does and this deliberately does not.
"""
import json
import os
import os as _os
import random
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Sdf, Usd, UsdGeom                              # noqa: E402
from detail import gac_storey_slice as gss                     # noqa: E402
from disaster import fracture                                   # noqa: E402
from disaster import gac_fire as gcf                            # noqa: E402
from disaster import quake_flow as qf                           # noqa: E402
from disaster import quake_sliced as qs                         # noqa: E402
from disaster import tornado as tn                              # noqa: E402

NAME = sys.argv[1] if len(sys.argv) > 1 else "SM_Building_02"
LEVEL = sys.argv[2] if len(sys.argv) > 2 else "T3"
SEED = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] else 7
BEARING = float(sys.argv[4]) if len(sys.argv) > 4 and sys.argv[4] else 35.0
OUT_DIR = os.environ.get("TP_OUT") or "/isaac-sim/.cache/tornado_probe"
FAMILY = {"urm": "01", "rc": "02", "rc_glass": "05"}


def _windward_side(bearing_deg):
    """The cell-frame side whose outward normal opposes the wind. The cell is
    unrotated, so S faces -y, N +y, E +x, W -x (quake_sliced._SIDE_NORMAL)."""
    import math as _m
    dx, dy = _m.cos(_m.radians(bearing_deg)), _m.sin(_m.radians(bearing_deg))
    normals = {"S": (0.0, -1.0), "N": (0.0, 1.0), "E": (1.0, 0.0), "W": (-1.0, 0.0)}
    return min(normals, key=lambda s: normals[s][0] * dx + normals[s][1] * dy)


def _bbox(stage, path):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    lo, hi = r.GetMin(), r.GetMax()
    return [round(float(v), 2) for v in (lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])]


def census(pls):
    """Field coverage of the sliced placements — what `quake_sliced._Grid`
    (and therefore the tornado planner) keys on."""
    from collections import Counter
    roles = Counter((p.get("_role") for p in pls))
    sides = Counter((p.get("_side") for p in pls))
    storeys = sorted(set(int(p.get("_storey", -1)) for p in pls))
    missing = {k: sum(1 for p in pls if k not in p)
               for k in ("_role", "_side", "_storey", "_bay", "_size",
                         "prim_path", "x_m", "y_m", "z_m")}
    return {"n": len(pls), "roles": dict(roles), "sides": dict(sides),
            "storeys": (storeys[0], storeys[-1], len(storeys)) if storeys else None,
            "missing_fields": {k: v for k, v in missing.items() if v}}


def main():
    t0 = time.time()
    fracture.ensure_vtk(verbose=False)
    kind, asset = gcf.split_kind(NAME)
    pack = gcf.PACKS[kind]
    url = gcf.asset_url(asset, kind)
    scale = gcf.asset_scale(url, pack["scale"], verbose=False)

    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(st, "/W/bench")
    cell = "/W/bench/g0"
    UsdGeom.Xform.Define(st, cell)

    src = gcf.place_source(st, cell, url, scale)
    if not src:
        raise RuntimeError("{0}: nothing composed".format(NAME))
    btype = qs.construction_type(asset)
    style = "tp_{0}".format(asset.lower())
    t1 = time.time()
    # TP_REGION=1: ring EVERY storey but keep bays only on the WINDWARD side
    # and the two sides sharing a corner with it (`_region_hot_sides`); the
    # lee collapses to one piece per run. `origin 0` + `top n-1` is the
    # region form that still rings the whole height (the fire's band cut
    # would merge storeys below its origin, which a tornado never wants).
    region = None
    if _os.environ.get("TP_REGION", "0") not in ("0", ""):
        # the wind the PLANNER will see (wind_at's bearing carries the inflow
        # tilt, not the raw heading) decides which side is hot
        _brg = BEARING
        if hasattr(tn, "wind_at"):
            _cfg = dict(tn.DEFAULTS)
            _cfg.update({"origin_m": [0.0, 60.0], "heading_deg": BEARING,
                         "width_m": 300.0, "wobble_m": 0.0, "edge_noise_m": 0.0,
                         "along_min": 1.0, "width_min": 1.0})
            _brg = tn.wind_at(_cfg, 0.0, 0.0)["bearing_deg"]
        region = {"origin": 0, "top": 10 ** 6, "sides": (_windward_side(_brg),)}
    pls, grid, measured = gss.slice_to_kit(
        st, src, cell, style, region=region, family=FAMILY.get(btype, "01"),
        verbose=False, force_regular=asset in (pack.get("force_regular_grid") or ()))
    if region is not None:
        print("[tp] region slice: hot side {0} (bearing {1:.0f})".format(
            region["sides"], BEARING))
    t_slice = time.time() - t1
    # MEASURED GLAZING PER PIECE, stamped onto the placement dicts before
    # `describe` so the planner picks panes by evidence, not by role: on
    # SM_Building_02 the windows live in PIER pieces, not `wall` ones (the
    # slicer's bay-split phase), and a role-based pick voided 0 panes.
    try:
        from disaster import tornado_urban_usd as _tuu
        if hasattr(_tuu, "annotate_glazing"):
            n_gl = _tuu.annotate_glazing(st, pls)
            print("[tp] glazing annotated: {0} of {1} pieces carry glass".format(
                n_gl, len(pls)))
    except Exception as exc:                                    # noqa: BLE001
        print("[tp] annotate_glazing unavailable ({0})".format(exc))
    # FX2 (§8e F3) — the debris-cladding-inheritance stamp, the same
    # measure-don't-guess discipline as annotate_glazing just above: what
    # is this piece's own DOMINANT non-glazing cladding texture, so a
    # removed piece's fragments can wear it instead of a flat class colour
    # (`tornado_urban._ledger_removed`, `tornado_urban_usd.debris_material`).
    try:
        if hasattr(_tuu, "annotate_surface"):
            n_srf = _tuu.annotate_surface(st, pls)
            print("[tp] surface annotated: {0} of {1} pieces carry a "
                  "resolved cladding texture".format(n_srf, len(pls)))
    except Exception as exc:                                    # noqa: BLE001
        print("[tp] annotate_surface unavailable ({0})".format(exc))
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    H = float(info["H"])
    print("[tp] {0} ({1}) btype {2}  H {3:.1f} m  sliced in {4:.0f} s  grid measured={5}"
          .format(asset, kind, btype, H, t_slice, bool(measured)))
    print("[tp] census {0}".format(json.dumps(census(pls))))
    els = info["elements"]
    from collections import Counter
    print("[tp] describe: {0} elements; side {1}; storey range {2}-{3}; roles {4}"
          .format(len(els), dict(Counter(e.get("side") for e in els)),
                  min(e.get("storey", 0) for e in els),
                  max(e.get("storey", 0) for e in els),
                  dict(Counter((e.get("p") or {}).get("_role") for e in els))))
    m = info["masses"]["main"]
    print("[tp] mass main: W {0:.1f} D {1:.1f} top {2:.1f} levels {3} yaw {4}"
          .format(m["W"], m["D"], m["top"], len(m["levels"]), m["yaw"]))

    # -- wind ------------------------------------------------------------
    wind = None
    if hasattr(tn, "wind_at"):
        # a synthetic track whose RIGHT flank the building sits on, so the
        # bearing is the forward one and the face hit is the back one
        cfg = dict(tn.DEFAULTS)
        cfg.update({"origin_m": [0.0, 60.0], "heading_deg": BEARING,
                    "width_m": 300.0, "wobble_m": 0.0, "edge_noise_m": 0.0,
                    "along_min": 1.0, "width_min": 1.0})
        wind = tn.wind_at(cfg, 0.0, 0.0)
        print("[tp] wind_at -> {0}".format(json.dumps(wind)))
    else:
        wind = {"bearing_deg": BEARING, "speed_frac": 0.8, "cross_frac": -0.4,
                "over": False}
        print("[tp] tornado.wind_at missing — synthetic wind {0}".format(wind))

    # -- plan ------------------------------------------------------------
    try:
        from disaster import tornado_urban as tu
    except Exception as exc:                                    # noqa: BLE001
        print("[tp] STOP: disaster.tornado_urban not importable ({0})".format(exc))
        return
    hc = tu.height_class_for(H) if hasattr(tu, "height_class_for") else None
    rng = random.Random(SEED)
    t2 = time.time()
    plan = tu.plan_damage(info, els, LEVEL, btype, rng, wind, height_class=hc,
                          intensity=0.8)
    print("[tp] plan {0} {1} hc={2} in {3:.1f} s".format(LEVEL, btype, hc,
                                                          time.time() - t2))
    for n in plan.get("notes") or []:
        print("[tp]   note: {0}".format(n))
    print("[tp] stats {0}".format(json.dumps(plan.get("stats"), sort_keys=True)))
    print("[tp] side_weights {0}".format(json.dumps(plan.get("side_weights"))))
    deb = plan.get("debris") or []
    if deb:
        kinds = Counter(d.get("kind") for d in deb)
        xs = [d["x"] for d in deb]
        ys = [d["y"] for d in deb]
        print("[tp] debris {0} fragments {1}; x [{2:.1f}, {3:.1f}] y [{4:.1f}, {5:.1f}]"
              .format(len(deb), dict(kinds), min(xs), max(xs), min(ys), max(ys)))
    json.dumps(plan)   # the schema contract: JSON-serialisable throughout

    # -- apply -----------------------------------------------------------
    try:
        from disaster import tornado_urban_usd as tuu
    except Exception as exc:                                    # noqa: BLE001
        print("[tp] STOP: disaster.tornado_urban_usd not importable ({0})".format(exc))
        return
    ctx = {"stage": st, "parent": cell, "tag": "tp", "mats": {},
           "static_extra": [], "loose": [], "authored": [], "info": info}
    t3 = time.time()
    counts = tuu.apply_plan(st, ctx, plan)
    print("[tp] apply {0} in {1:.1f} s".format(json.dumps(counts, sort_keys=True),
                                             time.time() - t3))
    deb_root = cell + "/tornado_debris"
    print("[tp] debris bbox {0}".format(_bbox(st, deb_root)))
    print("[tp] cell bbox   {0}".format(_bbox(st, cell)))

    # FX2 (§8e F3) — which debris mesh bound the SOURCE TEXTURE vs the flat
    # class bucket, and how many berm/ballistic fragments each carries: the
    # per-texture table the round's own review vehicle wants.
    n_src_frags = sum(1 for f in deb if f.get("source_tex"))
    print("[tp] source-textured fragments: {0} of {1} ({2} distinct "
          "source_tex_name)".format(
              n_src_frags, len(deb),
              len({f.get("source_tex_name") for f in deb if f.get("source_tex")})))
    src_keys = sorted(k for k in ctx["mats"] if k.startswith("tornado_debris:src:"))
    print("[tp] source-texture material cache ({0} entries): {1}".format(
          len(src_keys), src_keys))
    by_kind_mat = {}
    for f in deb:
        label = f.get("source_tex_name") or f.get("material")
        key = (f.get("kind"), label)
        e = by_kind_mat.setdefault(key, {"n": 0, "n_berm": 0, "textured": bool(f.get("source_tex"))})
        e["n"] += 1
        if f.get("stacked"):
            e["n_berm"] += 1
    for (kind, label), e in sorted(by_kind_mat.items()):
        print("[tp]   {0:<10s} {1:<40s} n={2:<4d} berm={3:<4d} textured={4}"
              .format(str(kind), str(label), e["n"], e["n_berm"], e["textured"]))

    # -- export ----------------------------------------------------------
    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "{0}_{1}_s{2}.usd".format(asset, LEVEL, SEED))
    st.GetRootLayer().Export(out)
    with open(out[:-4] + ".plan.json", "w") as fh:
        json.dump(plan, fh, indent=1, sort_keys=True)
    print("[tp] exported {0} ({1:.1f} MB) total {2:.0f} s".format(
        out, os.path.getsize(out) / 1e6, time.time() - t0))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
