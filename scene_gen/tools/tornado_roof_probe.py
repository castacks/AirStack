#!/usr/bin/env python
"""tornado_roof_probe — `disaster/tornado_roof.py`'s roof-damage pass end to
end on TWO real buildings, on a bare USD stage (no Kit GUI, no Flow, no
physics): the `tornado_kit_probe.py` / `tornado_urban_probe.py` pattern, run
through `wreck_kit`/`wreck_urban` so the R7 HOOK (the two-line addition in
each of those functions) actually fires, exactly the way a real bake
launcher would call them. `_plans/urban_tornado_plan.md` §8's own probe ask
for stream RF.

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \\
        bash scene_gen/tools/usd_python.sh \\
        scene_gen/tools/tornado_roof_probe.py [SEED] [BEARING_DEG]"

    # SEED/BEARING default to 7 / 35, matching tornado_kit_probe.py /
    # tornado_urban_probe.py's own defaults so results are cross-comparable.

Runs in the container ONLY (VTK + the Nucleus resolver for the sliced half;
Nucleus alone for the kit half), never inside a `SimulationApp`.

WHAT IT DOES, per building (`kit walkup T3` then `sliced SM_Building_02
T3`): build/slice, `wreck_kit`/`wreck_urban` (which run BOTH the façade
ladder and — via the R7 hook — this module's own roof pass), then prints
the roof PLAN (windward side, side weights, patch list with area/corner/
substrate, scour, coping, props, stats), the `apply_roof` COUNTS the hook
itself already produced (`ctx["roof_counts"]`), and the roof-region bbox
(`<cell>/tornado_roof`, world-aligned) alongside the whole-cell bbox for
scale. Exports the root layer (geometry-counts probe, not a bake — same
caveat `tornado_kit_probe.py`/`tornado_urban_probe.py` carry: a cold open
renders the kit's own referenced geometry, no `fire_bake.rehome_for_export`
flattening).
"""
import json
import os
import random
import sys
import time
import traceback

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
import numpy as np                                              # noqa: E402
from pxr import Sdf, Usd, UsdGeom                                # noqa: E402

from detail import gac_storey_slice as gss                      # noqa: E402
from disaster import fracture                                    # noqa: E402
from disaster import gac_fire as gcf                              # noqa: E402
from disaster import kit_substitute as ksub                        # noqa: E402
from disaster import quake_sliced as qs                              # noqa: E402
from disaster import tornado as tn                                    # noqa: E402
from disaster import tornado_kit as tk                                 # noqa: E402
from disaster import tornado_urban_usd as tuu                           # noqa: E402

SEED = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1] else 7
BEARING = float(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] else 35.0
LEVEL = "T3"
OUT_DIR = os.environ.get("TP_OUT") or "/isaac-sim/.cache/tornado_probe"
FAMILY = {"urm": "01", "rc": "02", "rc_glass": "05"}


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


def _wind(bearing_deg):
    cfg = dict(tn.DEFAULTS)
    cfg.update({"origin_m": [0.0, 60.0], "heading_deg": bearing_deg,
               "width_m": 300.0, "wobble_m": 0.0, "edge_noise_m": 0.0,
               "along_min": 1.0, "width_min": 1.0})
    return tn.wind_at(cfg, 0.0, 0.0)


def _print_roof_plan(tag, ctx):
    rp = ctx.get("roof_plan")
    if rp is None:
        print("[trp:{0}] STOP: no roof_plan on ctx -- R7 hook did not run "
              "(is disaster/tornado_roof.py importable?)".format(tag))
        return
    print("[trp:{0}] roof: level {1} btype {2} height_class {3} skipped={4}"
          .format(tag, rp["level"], rp["btype"], rp["height_class"],
                  rp["skipped"]))
    if rp["skipped"]:
        print("[trp:{0}]   skip_reason: {1}".format(tag, rp["skip_reason"]))
        return
    print("[trp:{0}] windward_side {1}  side_weights {2}".format(
        tag, rp["windward_side"], json.dumps(rp["side_weights"])))
    print("[trp:{0}] roof plane: W {1:.1f} D {2:.1f} top {3:.1f} inset {4} "
          "rect_local {5}".format(tag, rp["roof"]["W"], rp["roof"]["D"],
                                  rp["roof"]["top"], rp["roof"]["inset"],
                                  [round(q, 2) for q in rp["roof"]["rect_local"]]))
    for i, p in enumerate(rp["patches"]):
        print("[trp:{0}]   patch {1}: side {2} corner {3} area {4:.1f} m2 "
              "z {5:.3f} substrate {6} lip_h {7:.2f} n_sheets {8}".format(
                  tag, i, p["side"], p["corner"], p["area_m2"], p["z"],
                  p["substrate"]["material"], p["lip"]["height_m"],
                  len(p["sheets"])))
    sc = rp["scour"]
    print("[trp:{0}] scour: side {1} frac {2:.2f}".format(
        tag, sc["side"], sc["frac"]))
    cp = rp["coping"]
    print("[trp:{0}] coping: target {1:.2f} already_removed {2:.2f} "
          "piece_removed {3} boxes {4}".format(
              tag, cp["target_frac"], cp["already_removed_frac"],
              len(cp["piece_removed"]), len(cp["boxes"])))
    pr = rp["props"]
    print("[trp:{0}] props: action {1} n_units {2} topple_frac {3:.2f} "
          "sweep_frac {4:.2f} n_topple {5} n_thrown {6}".format(
              tag, pr["action"], pr["n_units"], pr["topple_frac"],
              pr["sweep_frac"], pr["n_topple"], pr["n_thrown"]))
    print("[trp:{0}] stats {1}".format(tag, json.dumps(rp["stats"], sort_keys=True)))
    json.dumps(rp)   # the schema contract: JSON-serialisable throughout


def _run_kit(seed, bearing):
    tag = "kit:walkup"
    print("\n=== {0} T3 seed {1} bearing {2} ===".format(tag, seed, bearing))
    tk._refuse_if_unsupported("walkup")

    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(st, "/W/bench")
    cell = "/W/bench/g0"
    UsdGeom.Xform.Define(st, cell)

    btype = ksub.styles()["walkup"]["type"]
    wind = _wind(bearing)
    print("[trp:{0}] btype {1}  wind_at -> {2}".format(tag, btype, json.dumps(wind)))

    rng = random.Random(seed)
    nrng = np.random.default_rng(seed & 0xFFFFFFFF)
    t0 = time.time()
    ctx = tk.wreck_kit(st, cell, "walkup", LEVEL, rng, nrng, {}, "trp_kit",
                       wind, seed=seed, btype=btype, intensity=0.6,
                       verbose=True)
    print("[trp:{0}] wreck_kit in {1:.1f} s".format(tag, time.time() - t0))

    _print_roof_plan(tag, ctx)
    print("[trp:{0}] roof apply counts {1}".format(
        tag, json.dumps(ctx.get("roof_counts"), sort_keys=True)))
    roof_root = cell + "/tornado_roof"
    print("[trp:{0}] roof-region bbox {1}".format(tag, _bbox(st, roof_root)))
    print("[trp:{0}] cell bbox        {1}".format(tag, _bbox(st, cell)))

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "roof_kit_walkup_{0}_s{1}.usd".format(LEVEL, seed))
    st.GetRootLayer().Export(out)
    if ctx.get("roof_plan") is not None:
        with open(out[:-4] + ".roof_plan.json", "w") as fh:
            json.dump(ctx["roof_plan"], fh, indent=1, sort_keys=True)
    print("[trp:{0}] exported {1} ({2:.1f} MB)".format(
        tag, out, os.path.getsize(out) / 1e6))
    return ctx


def _run_sliced(seed, bearing):
    tag = "sliced:SM_Building_02"
    print("\n=== {0} T3 seed {1} bearing {2} ===".format(tag, seed, bearing))
    fracture.ensure_vtk(verbose=False)
    kind, asset = gcf.split_kind("SM_Building_02")
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
        raise RuntimeError("SM_Building_02: nothing composed")
    btype = qs.construction_type(asset)
    style = "trp_{0}".format(asset.lower())
    t0 = time.time()
    pls, grid, measured = gss.slice_to_kit(
        st, src, cell, style, region=None, family=FAMILY.get(btype, "01"),
        verbose=False, force_regular=asset in (pack.get("force_regular_grid") or ()))
    print("[trp:{0}] btype {1}  sliced {2} piece(s) in {3:.1f} s  "
          "grid measured={4}".format(tag, btype, len(pls), time.time() - t0,
                                     bool(measured)))

    wind = _wind(bearing)
    print("[trp:{0}] wind_at -> {1}".format(tag, json.dumps(wind)))

    rng = random.Random(seed)
    nrng = np.random.default_rng(seed & 0xFFFFFFFF)
    t1 = time.time()
    ctx = tuu.wreck_urban(st, cell, pls, style, LEVEL, rng, nrng, {},
                          "trp_sliced", wind, btype=btype, intensity=0.6,
                          usd=asset, verbose=True)
    print("[trp:{0}] wreck_urban in {1:.1f} s".format(tag, time.time() - t1))

    _print_roof_plan(tag, ctx)
    print("[trp:{0}] roof apply counts {1}".format(
        tag, json.dumps(ctx.get("roof_counts"), sort_keys=True)))
    roof_root = cell + "/tornado_roof"
    print("[trp:{0}] roof-region bbox {1}".format(tag, _bbox(st, roof_root)))
    print("[trp:{0}] cell bbox        {1}".format(tag, _bbox(st, cell)))

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "roof_{0}_{1}_s{2}.usd".format(asset, LEVEL, seed))
    st.GetRootLayer().Export(out)
    if ctx.get("roof_plan") is not None:
        with open(out[:-4] + ".roof_plan.json", "w") as fh:
            json.dump(ctx["roof_plan"], fh, indent=1, sort_keys=True)
    print("[trp:{0}] exported {1} ({2:.1f} MB)".format(
        tag, out, os.path.getsize(out) / 1e6))
    return ctx


def main():
    t0 = time.time()
    _run_kit(SEED, BEARING)
    _run_sliced(SEED, BEARING)
    print("\n[trp] total {0:.0f} s".format(time.time() - t0))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
