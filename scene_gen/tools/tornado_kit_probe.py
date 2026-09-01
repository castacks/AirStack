#!/usr/bin/env python
"""tornado_kit_probe — `disaster/tornado_kit.py`'s adapter end to end on a
REAL kit-style building, on a bare USD stage (no Kit GUI, no Flow, no
physics): build the kit, describe, adapt, plan, apply, export. The
`tornado_urban_probe.py` of the KIT-STYLE half of the pipeline
(`_plans/urban_tornado_plan.md` §7's rule R3).

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \\
        bash scene_gen/tools/usd_python.sh \\
        scene_gen/tools/tornado_kit_probe.py brownstone_row T4 7 35"

    # walkup at T3, seed 7, bearing 35 (the defaults if omitted)
    ... tornado_kit_probe.py walkup T3

Runs in the container ONLY — `kit_substitute.build_kit` references REAL
`omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/...` kit
assets (`ModernCityEnvironment01`, `Downtown_West`, `CivilianArea`), which
need the Nucleus resolver. Never inside a `SimulationApp` — bare python via
`usd_python.sh` (pxr + Nucleus, no Kit) — and never touches the live Kit
GUI's tmux pane.

WHAT IT PRINTS, in order: the kit build (piece count, family), the census of
`adapt`'s own field coverage (every `_side`/`_role`/`_storey`/`_bay`/`_size`
the planner's `_Grid` relies on, plus `prim_path` — so a real authored kit
building that does not carry a field the pure host tests assumed is caught
HERE), the wind the ladder was given (`tornado.wind_at`, the same call
`tornado_urban_probe.py` makes), the glazing annotation count (`_glass_faces`
measured against the REAL authored prims — only possible now that the kit
is actually on the stage), the plan's `stats`/`notes`, the debris bbox and
per-class counts, and the export path.

The export keeps the kit's own referenced geometry (this is a probe for
GEOMETRY COUNTS and CENSUS, not a bake) — a cold open renders exactly what a
live Kit session would show for this building, no `fire_bake.
rehome_for_export`-style flattening.
"""
import json
import os
import random
import sys
import time
import traceback
from collections import Counter

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
import numpy as np                                              # noqa: E402
from pxr import Sdf, Usd, UsdGeom                                # noqa: E402

from disaster import kit_substitute as ksub                     # noqa: E402
from disaster import quake_flow as qf                            # noqa: E402
from disaster import quake_sliced as qs                          # noqa: E402
from disaster import tornado as tn                                # noqa: E402
from disaster import tornado_kit as tk                            # noqa: E402

STYLE = sys.argv[1] if len(sys.argv) > 1 else "brownstone_row"
LEVEL = sys.argv[2] if len(sys.argv) > 2 else "T3"
SEED = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] else 7
BEARING = float(sys.argv[4]) if len(sys.argv) > 4 and sys.argv[4] else 35.0
OUT_DIR = os.environ.get("TP_OUT") or "/isaac-sim/.cache/tornado_probe"


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
    """Field coverage of the ADAPTED placements — what
    `quake_sliced._Grid` (and therefore the tornado planner) keys on, the
    same shape `tornado_urban_probe.py`'s own `census` prints for a sliced
    building."""
    roles = Counter(p.get("_role") for p in pls)
    sides = Counter(p.get("_side") for p in pls)
    storeys = sorted(set(int(p.get("_storey", -1)) for p in pls))
    missing = {k: sum(1 for p in pls if k not in p)
              for k in ("_role", "_side", "_storey", "_bay", "_size",
                        "prim_path", "x_m", "y_m", "z_m")}
    return {"n": len(pls), "roles": dict(roles), "sides": dict(sides),
            "storeys": (storeys[0], storeys[-1], len(storeys)) if storeys else None,
            "missing_fields": {k: v for k, v in missing.items() if v}}


def main():
    t0 = time.time()
    tk._refuse_if_unsupported(STYLE)   # fail fast, before touching a stage

    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(st, "/W/bench")
    cell = "/W/bench/g0"
    UsdGeom.Xform.Define(st, cell)

    btype = ksub.styles()[STYLE]["type"]
    H_expect = ksub.styles()[STYLE]["H"]
    print("[tkp] {0} btype {1} H(expect) {2:.1f} m".format(STYLE, btype, H_expect))

    # -- wind, same call tornado_urban_probe.py makes ---------------------
    cfg = dict(tn.DEFAULTS)
    cfg.update({"origin_m": [0.0, 60.0], "heading_deg": BEARING,
               "width_m": 300.0, "wobble_m": 0.0, "edge_noise_m": 0.0,
               "along_min": 1.0, "width_min": 1.0})
    wind = tn.wind_at(cfg, 0.0, 0.0)
    print("[tkp] wind_at -> {0}".format(json.dumps(wind)))

    intensity = tk.LEVEL_INTENSITY.get(LEVEL, 0.7)
    rng = random.Random(SEED)
    nrng = np.random.default_rng(SEED & 0xFFFFFFFF)

    t1 = time.time()
    ctx = tk.wreck_kit(st, cell, STYLE, LEVEL, rng, nrng, {}, "tkp", wind,
                       seed=SEED, btype=btype, intensity=intensity,
                       verbose=True)
    t_wreck = time.time() - t1

    plan = ctx.get("plan")
    if plan is None:
        print("[tkp] STOP: wreck_kit produced no plan")
        return

    # -- census, off the REAL authored placements (build_kit's own return,
    #    re-adapted -- ctx["info"]["elements"]' own "p" dicts) ------------
    placements = [e["p"] for e in ctx["info"]["elements"]]
    print("[tkp] built + adapted {0} piece(s) in {1:.1f} s".format(
        len(placements), t_wreck))
    print("[tkp] census {0}".format(json.dumps(census(placements))))

    m = ctx["info"]["masses"]["main"]
    print("[tkp] mass main: W {0:.1f} D {1:.1f} top {2:.1f} levels {3}"
          .format(m["W"], m["D"], m["top"], len(m["levels"])))

    for n in plan.get("notes") or []:
        print("[tkp]   note: {0}".format(n))
    print("[tkp] stats {0}".format(json.dumps(plan.get("stats"), sort_keys=True)))
    print("[tkp] side_weights {0}".format(json.dumps(plan.get("side_weights"))))

    deb = plan.get("debris") or []
    if deb:
        kinds = Counter(d.get("kind") for d in deb)
        xs = [d["x"] for d in deb]
        ys = [d["y"] for d in deb]
        print("[tkp] debris {0} fragments {1}; x [{2:.1f}, {3:.1f}] "
              "y [{4:.1f}, {5:.1f}]".format(len(deb), dict(kinds),
                                            min(xs), max(xs), min(ys), max(ys)))
    json.dumps(plan)   # the schema contract: JSON-serialisable throughout

    print("[tkp] apply counts {0}".format(json.dumps(ctx["counts"], sort_keys=True)))
    deb_root = cell + "/tornado_debris"
    print("[tkp] debris bbox {0}".format(_bbox(st, deb_root)))
    print("[tkp] cell bbox   {0}".format(_bbox(st, cell)))

    # FX2 (§8e F3) — which debris mesh bound the SOURCE TEXTURE (this kit
    # style's own cladding) vs the flat class bucket, and how many berm/
    # ballistic fragments each carries.
    n_src_frags = sum(1 for f in deb if f.get("source_tex"))
    print("[tkp] source-textured fragments: {0} of {1} ({2} distinct "
          "source_tex_name)".format(
              n_src_frags, len(deb),
              len({f.get("source_tex_name") for f in deb if f.get("source_tex")})))
    src_keys = sorted(k for k in ctx["mats"] if k.startswith("tornado_debris:src:"))
    print("[tkp] source-texture material cache ({0} entries): {1}".format(
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
        print("[tkp]   {0:<10s} {1:<40s} n={2:<4d} berm={3:<4d} textured={4}"
              .format(str(kind), str(label), e["n"], e["n_berm"], e["textured"]))

    # -- export ------------------------------------------------------------
    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "kit_{0}_{1}_s{2}.usd".format(STYLE, LEVEL, SEED))
    st.GetRootLayer().Export(out)
    with open(out[:-4] + ".plan.json", "w") as fh:
        json.dump(plan, fh, indent=1, sort_keys=True)
    print("[tkp] exported {0} ({1:.1f} MB) total {2:.0f} s".format(
        out, os.path.getsize(out) / 1e6, time.time() - t0))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
