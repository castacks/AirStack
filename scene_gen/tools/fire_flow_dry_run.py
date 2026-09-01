#!/usr/bin/env python3
"""fire_flow_dry_run.py — host-side, no Kit, no GPU: loads the REAL sidecars
+ manifest + placements dump for `city_138` (the scene that is LIVE in the
GUI right now) and predicts exactly what `urban_fire_city_launch_script.
put_the_fire_back` would author under the OLD (already-live) knobs vs the
NEW ones this change adds — before paying for a relaunch to find out.

    python3 scene_gen/tools/fire_flow_dry_run.py
    python3 scene_gen/tools/fire_flow_dry_run.py --out /path/to/table.txt
    python3 scene_gen/tools/fire_flow_dry_run.py \\
        --manifest scene_gen/_plans/fire_city_500m_39.json \\
        --bakes ~/docker/isaac-sim/cache/main/fire_bakes/city_138 \\
        --dump scene_gen/_plans/fc_dump_500.json

WHY IT DOES NOT IMPORT THE LAUNCHER. `urban_fire_city_launch_script.py`
builds a `SimulationApp` at module scope — importing it starts Kit, and a
second Kit app in one process is a segfault (documented in that launcher's
own header). `_load_allocator` below extracts `_live`/`fire_state`/
`emitter_estimate`/`allocate_emitters`/`STATE_RANK` straight out of its
SOURCE with `ast` and execs them in a bare namespace — the exact trick
`scene_gen/tests/test_urban_fire_city_launch.py`'s own `_load_allocator`
uses, so the accounting this tool prints is never a re-implementation that
could drift from what a real run actually does; it is the real run's own
budgeting code, read off disk, so it automatically picks up every branch
`emitter_estimate` grew across all three review rounds (side smoke, roof
caps, cluster diversity, size scaling, residual flame pockets).

`disaster.fire_assembly_lib` (`fal`) IS safely importable here — it only
needs `pxr`, which this host has — so `fal.choose_street_side`/
`load_dump_positions`/`street_side_ranks`/`roof_has_collapsed`/
`flame_window_target`/`smoke_window_target`/`_flame_selection_order` are the
REAL functions, not mirrors.

WHAT IT PRINTS. Two full passes over the manifest's bakes — OLD (the knobs
the FIRST live relaunch used: `FA_EMITTERS=8 FA_EMITTER_BUDGET=400`, no
side-smoke/roof/cluster/size/residual knobs, no street bias) and NEW (this
change's defaults, all three review rounds) — a per-building table (state,
alloc, flame openings, how many CLUSTERS they span, how many landed on the
CHOSEN street-facing side, side-smoke count, roof-smoke count, roof-
collapsed?, predicted total), the boosted-side HISTOGRAM across the whole
manifest (the item-1 acceptance: spread over all four bearings, not
clustered), explicit PASS/FAIL rows for "every building with events shows
window smoke" and "SM_Building_23 shows both flame and smoke", grand
totals with the delta, and a VRAM/block projection calibrated against BOTH
live measurements on record (see `LIVE_POINTS`). Saves the same table to
`--out` if given.
"""
import argparse
import ast
import glob
import json
import os
import random
import sys
from collections import Counter

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
_REPO = os.path.normpath(os.path.join(_SCENE_GEN, ".."))
_LAUNCH = os.path.join(_REPO, "simulation", "isaac-sim", "launch_scripts")
_TOOLS = os.path.join(_SCENE_GEN, "tools")
for _p in (_SCENE_GEN, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from disaster import fire_assembly_lib as fal              # noqa: E402
from disaster import soot_plume as spl                     # noqa: E402
from disaster import urban_fire as uf                       # noqa: E402
import fire_city_manifest as fcm                            # noqa: E402

CITY_LAUNCHER = os.path.join(_LAUNCH, "urban_fire_city_launch_script.py")

DEFAULT_MANIFEST = os.path.join(_SCENE_GEN, "_plans", "fire_city_500m_39.json")
DEFAULT_BAKES = os.path.expanduser(
    "~/docker/isaac-sim/cache/main/fire_bakes/city_138")
DEFAULT_DUMP = os.path.join(_SCENE_GEN, "_plans", "fc_dump_500.json")
#: the exact acceptance building the third review names by stem-substring.
HEADLINE_BUILDING = "SM_Building_23"

# TWO LIVE MEASUREMENTS ON RECORD, both used to calibrate the projection
# below (see `vram_projection`):
#   round 1 (fire_city_v3_run.log, 2026-08-31 13:13Z): "VRAM Flow up (395
#   emitter(s)): 13737 / 16303 MiB", "... Flow 1038 MiB", "8192 block pool",
#   "Flow OOM check CLEAN".
#   round 2 (coordinator report, same day, later relaunch with this change's
#   round-1 defaults live): 560 emitters, 0 OOM, ~1472 MiB Flow, ~1.8 GB
#   headroom, "min block pool scaled ~11.6k at 560" (independently confirms
#   the SAME blocks-per-emitter ratio round 1 measured: 8192/395 = 20.74/
#   emitter vs 11600/560 = 20.71/emitter).
LIVE_TOTAL_MIB = 16303.0
LIVE_POINTS = ((395, 1038.0, 13737.0), (560, 1472.0, 14503.0))
# per-emitter Flow cost and per-emitter block cost, averaged across both
# points (they agree to within 0.2%, so this is not sensitive to which).
_FLOW_MIB_PER_EMITTER = sum(f / n for n, f, _u in LIVE_POINTS) / len(LIVE_POINTS)
#: 8192/395 (round 1) and 11600/560 (round 2's own "~11.6k" estimate) agree
#: to within 0.2%, so either anchor gives the same ratio.
_BLOCKS_PER_EMITTER = 8192.0 / 395.0
# non-Flow baseline used MiB, back-solved from round 2's own measured used
# total (14503) minus its own Flow estimate (1472) — this is a few hundred
# MiB higher than round 1's own back-solved baseline (12699), which is
# expected: round 2 already carries the fire apron / oblique-camera-
# occlusion passes round 1 did not.
_NONFLOW_BASELINE_MIB = LIVE_POINTS[-1][2] - LIVE_POINTS[-1][1]


def _load_allocator():
    """Extract `_live`/`fire_state`/`emitter_estimate`/`allocate_emitters`/
    `STATE_RANK` from the REAL launcher source — see this module's own
    docstring for why it cannot be imported."""
    with open(CITY_LAUNCHER) as fh:
        src = fh.read()
    tree = ast.parse(src, filename=CITY_LAUNCHER)
    fns = {n.name: n for n in ast.walk(tree)
          if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
    state_rank_assign = [
        n for n in tree.body if isinstance(n, ast.Assign)
        and any(getattr(t, "id", None) == "STATE_RANK" for t in n.targets)]
    if not state_rank_assign:
        raise RuntimeError("STATE_RANK not found in " + CITY_LAUNCHER)
    ns = {"uf": uf, "spl": spl, "fal": fal,
         "STATE_RANK": ast.literal_eval(
             ast.get_source_segment(src, state_rank_assign[0].value))}
    for name in ("_live", "fire_state", "emitter_estimate",
                 "allocate_emitters"):
        if name not in fns:
            raise RuntimeError("{0} not found in {1}".format(
                name, CITY_LAUNCHER))
        exec(compile(ast.get_source_segment(src, fns[name]), CITY_LAUNCHER,
                     "exec"), ns)
    return ns


def load_rows(manifest_path, bakes_dir):
    """One row per manifest record with a bake on disk — mirrors the real
    launcher's `bake_paths` (same `fcm.build_entry_and_stem` stem match),
    minus everything Kit-side (no cell resolution, no placement)."""
    top_seed, records = fcm.load_manifest(manifest_path)
    seed = fcm.resolve_city_seed(manifest_path, top_seed)
    by_stem = {}
    for jf in sorted(glob.glob(os.path.join(bakes_dir, "*.json"))):
        by_stem[os.path.splitext(os.path.basename(jf))[0]] = jf

    rows, missing = [], []
    for i, rec in enumerate(records):
        try:
            _entry, stem = fcm.build_entry_and_stem(rec, i)
        except Exception as exc:
            missing.append((i, "not a valid bake entry: {0}".format(exc)))
            continue
        jf = by_stem.get(stem)
        if not jf:
            missing.append((i, "no bake on disk for stem {0!r}".format(stem)))
            continue
        with open(jf) as fh:
            doc = json.load(fh)
        rows.append({"i": rec.get("i"), "stem": stem, "rec": rec, "doc": doc,
                    "events": doc.get("events") or [],
                    "n_st": int((doc.get("fire") or {}).get("n_storeys") or 0)})
    return seed, rows, missing


def _live_ev(ev):
    return all(not (o.get("e") or {}).get("dead")
              for o in (ev.get("ops") or []))


def flame_breakdown(events, doc, alloc, street_bias_side, bias_weight,
                    flame_min_clusters=None, flame_extra_max=3,
                    flame_size_scaling=False):
    """`(n_flame_openings, n_on_boosted_side, n_clusters, sides_hit)` for
    the PRIMARY flame loop + the cluster-diversity top-up (the two branches
    that select individual OPENINGS by side/storey group) — exactly `place_
    fire`'s own formulas, so this is what the real assembly would actually
    light for a "flame"-state building, not an approximation of it. Does
    NOT cover the F4 top-up or the residual-flame pockets (those are
    counted, correctly, inside `est["flame"]` already — this function
    exists only to report the CLUSTER SPREAD and STREET-SIDE SHARE, which
    `emitter_estimate` itself does not track)."""
    f = doc.get("fire") or {}
    n_st = int(f.get("n_storeys") or 0)
    if flame_size_scaling:
        max_open = min(alloc, fal.flame_window_target(n_st))
    else:
        max_open = (max(alloc, min(16, n_st // 2)) if n_st >= 12 else alloc)

    evs = [ev for ev in (events or []) if _live_ev(ev) and ev.get("ops")]
    flame_evs = [e for e in evs if e["state"] == "flame"]
    pairs = fal._flame_selection_order(flame_evs, street_bias_side=street_bias_side,
                                       bias_weight=bias_weight)
    chosen = pairs[:max_open]
    lit_groups = {(ev.get("side"), ev.get("storey")) for ev, _op in chosen}
    n_open = len(chosen)
    if flame_min_clusters is not None:
        extra_budget = min(max(0, max_open - n_open), max(0, int(flame_extra_max)))
        if len(lit_groups) < int(flame_min_clusters) and extra_budget > 0:
            extra_pool = [e for e in evs if e["state"] == "out"]
            extra_pairs = fal._flame_selection_order(
                extra_pool, street_bias_side=street_bias_side,
                bias_weight=bias_weight)[:extra_budget]
            chosen = chosen + extra_pairs
            lit_groups |= {(ev.get("side"), ev.get("storey"))
                          for ev, _op in extra_pairs}
    sides_hit = [ev["side"] for ev, _op in chosen]
    n_boosted = (sum(1 for s in sides_hit if s == street_bias_side)
                if street_bias_side else 0)
    return len(chosen), n_boosted, len(lit_groups), sides_hit


def run_pass(label, rows, ns, *, emitters, budget, side_smoke_flame_max=None,
            side_smoke_nonflame_max=None, roof_cap_intact=None,
            roof_cap_collapsed=None, street_positions=None,
            street_bias_weight=2, flame_min_clusters=None, flame_extra_max=3,
            flame_size_scaling=False, smoke_size_scaling=False,
            smoke_window_jets=False, residual_flame_frac=0.0, seed=7):
    """One full allocator + breakdown pass over `rows`. Returns
    `(per_building_rows, totals)`; does not mutate the caller's `rows`."""
    live_rows = []
    for r in rows:
        live_rows.append({"i": r["i"], "stem": r["stem"], "doc": r["doc"],
                          "events": r["events"]})
    spent = ns["allocate_emitters"](
        live_rows, budget, emitters, True, side_smoke_flame_max,
        side_smoke_nonflame_max, roof_cap_intact, roof_cap_collapsed,
        flame_min_clusters, flame_extra_max, flame_size_scaling,
        smoke_size_scaling, smoke_window_jets, residual_flame_frac)

    out = []
    boosted_hist = Counter()
    for r, orig in zip(live_rows, rows):
        doc = r["doc"]
        f = doc.get("fire") or {}
        level = f.get("level")
        alloc = r.get("alloc")
        rec_i = orig["rec"].get("i")
        sides = tuple(f.get("sides") or ())
        street_rank = {}
        street_bias_side = None
        if street_positions and rec_i is not None:
            street_rank = fal.street_side_ranks(street_positions, rec_i, sides)
            street_bias_side = fal.choose_street_side(
                street_rank, "{0}-{1}-street".format(seed, r["stem"]))
        if street_bias_side:
            boosted_hist[street_bias_side] += 1
        n_live_events = sum(1 for ev in (r["events"] or [])
                            if _live_ev(ev) and ev.get("ops"))
        if alloc is None:
            out.append({"i": rec_i, "stem": r["stem"], "level": level,
                       "state": r.get("state", "none"), "alloc": None,
                       "flame_open": 0, "flame_street": 0, "clusters": 0,
                       "side_smoke": 0, "roof": 0, "interior": 0, "total": 0,
                       "flame_total": 0,
                       "roof_collapsed": fal.roof_has_collapsed(doc),
                       "street_rank": street_rank,
                       "boosted_side": street_bias_side,
                       "starved": n_live_events == 0,
                       "note": r.get("drop_reason", "dropped")})
            continue
        state = r.get("state", "none")
        is_flame_state = state == "flame"
        n_flame_open = n_boosted = n_clusters = 0
        if is_flame_state:
            n_flame_open, n_boosted, n_clusters, _sides = flame_breakdown(
                r["events"], doc, alloc, street_bias_side, street_bias_weight,
                flame_min_clusters, flame_extra_max, flame_size_scaling)
        est = r["est"]
        out.append({"i": rec_i, "stem": r["stem"], "level": level,
                   "state": state, "alloc": alloc,
                   "flame_open": n_flame_open, "flame_street": n_boosted,
                   "clusters": n_clusters,
                   "flame_total": est.get("flame", 0),
                   "side_smoke": est.get("smoke", 0),
                   "roof": est.get("roof", 0), "interior": est.get("interior", 0),
                   "total": est.get("total", 0),
                   "roof_collapsed": fal.roof_has_collapsed(doc),
                   "street_rank": street_rank, "boosted_side": street_bias_side,
                   # STARVED = this bake's own `soot_plume.plan_events` left
                   # zero usable events at all — no amount of assembly-time
                   # placement can invent a window that was never planned;
                   # this is a bake-time gap ("starved-events trap" in the
                   # skill), out of scope for a Flow-placement-only change.
                   "starved": n_live_events == 0, "note": ""})
    n_side_only_gt_roof = sum(
        1 for o in out if not o["roof_collapsed"] and o["roof"] > 0
        and o["side_smoke"] > o["roof"])
    side_plus_int_fails = [
        o["stem"] for o in out
        if not o["roof_collapsed"] and o["roof"] > 0
        and (o["side_smoke"] + o["interior"]) <= o["roof"]]
    # "no burning building with events may have window-smoke = 0" — the
    # coordinator's explicit follow-up acceptance check ("even in residual
    # state i need smoke from windows if not fires"). Excludes only the
    # truly STARVED (zero baked events at all) and DROPPED (no budget)
    # buildings, which no Flow-placement change can give smoke to.
    zero_window_smoke = [o["stem"] for o in out
                         if o["alloc"] is not None and not o["starved"]
                         and o["side_smoke"] == 0]
    headline = [o for o in out if HEADLINE_BUILDING in o["stem"]]
    totals = {
        "predicted": spent,
        "buildings": len(out),
        "dropped": sum(1 for o in out if o["alloc"] is None),
        "starved": sum(1 for o in out if o["starved"]),
        "flame_open": sum(o["flame_open"] for o in out),
        "flame_street": sum(o["flame_street"] for o in out),
        "flame_total": sum(o["flame_total"] for o in out),
        "side_smoke": sum(o["side_smoke"] for o in out),
        "roof": sum(o["roof"] for o in out),
        "interior": sum(o["interior"] for o in out),
        "total": sum(o["total"] for o in out),
        "roof_collapsed_buildings": sum(1 for o in out if o["roof_collapsed"]),
        "side_gt_roof_strict": n_side_only_gt_roof,
        "side_plus_interior_fails": side_plus_int_fails,
        "zero_smoke": [o["stem"] for o in out
                      if o["alloc"] is not None
                      and (o["side_smoke"] + o["roof"] + o["interior"]) == 0],
        "zero_window_smoke": zero_window_smoke,
        "boosted_hist": boosted_hist,
        "headline_rows": headline,
    }
    return out, totals


def _fmt_row(o):
    street = ("{0}/{1}".format(o["flame_street"], o["flame_open"])
             if o["flame_open"] else "-")
    return ("  {0:<3} {1:<32} {2:<5} {3:<9} {4:>5} {5:>9} {6:>3} {7:>9} "
           "{8:>4} {9:>4} {10:>4} {11:>6}  {12}".format(
               o["i"], o["stem"][:32], o["level"] or "?", o["state"],
               "-" if o["alloc"] is None else o["alloc"], street,
               o["clusters"], o["side_smoke"], o["roof"], o["interior"],
               "Y" if o["roof_collapsed"] else "n", o["total"], o["note"]))


def render_table(label, rows, totals):
    lines = []
    lines.append("=== {0} ===".format(label))
    lines.append("  {0:<3} {1:<32} {2:<5} {3:<9} {4:>5} {5:>9} {6:>3} {7:>9} "
                 "{8:>4} {9:>4} {10:>4} {11:>6}  {12}".format(
                     "#", "stem", "level", "state", "alloc",
                     "boostF/N", "cls", "sideSmk", "roof", "int", "collp",
                     "total", "note"))
    for o in sorted(rows, key=lambda o: (o["i"] is None, o["i"])):
        lines.append(_fmt_row(o))
    lines.append("")
    lines.append("  buildings={buildings} dropped={dropped} starved={starved} "
                "predicted_total={predicted}".format(**totals))
    lines.append("  flame openings (primary+cluster only)={0} (of which on "
                "the chosen street side={1}, {2:.0%})".format(
                    totals["flame_open"], totals["flame_street"],
                    (totals["flame_street"] / totals["flame_open"])
                    if totals["flame_open"] else 0.0))
    lines.append("  flame TOTAL (primary+topup+cluster+residual)={0}".format(
        totals["flame_total"]))
    lines.append("  side smoke={0} roof smoke={1} interior smoke={2}"
                .format(totals["side_smoke"], totals["roof"],
                        totals["interior"]))
    lines.append("  roof-collapsed buildings={0}/{1}".format(
        totals["roof_collapsed_buildings"], totals["buildings"]))
    lines.append("  side (window/opening only) > roof on every intact-roof "
                "building: {0}/{1} pass".format(
                    totals["side_gt_roof_strict"],
                    sum(1 for o in rows if not o["roof_collapsed"]
                       and o["roof"] > 0)))
    lines.append("  side+interior > roof on every intact-roof building: "
                "{0} (failures: {1}{2})".format(
                    "YES" if not totals["side_plus_interior_fails"] else "NO",
                    len(totals["side_plus_interior_fails"]),
                    ("" if not totals["side_plus_interior_fails"] else
                     " -- " + ", ".join(totals["side_plus_interior_fails"]))))
    lines.append("  every building shows SOME smoke: {0}".format(
        "YES" if not totals["zero_smoke"] else
        "NO -- " + ", ".join(totals["zero_smoke"])))
    lines.append("  every building (with baked events) shows WINDOW smoke: "
                "{0}".format(
                    "YES" if not totals["zero_window_smoke"] else
                    "NO -- " + ", ".join(totals["zero_window_smoke"])))
    if totals["headline_rows"]:
        h = totals["headline_rows"][0]
        lines.append("  HEADLINE {0!r}: flame={1} side_smoke={2} state={3} "
                     "-> {4}".format(
                         h["stem"], h["flame_total"], h["side_smoke"],
                         h["state"],
                         "PASS" if h["flame_total"] > 0 and h["side_smoke"] > 0
                         else "FAIL"))
    else:
        lines.append("  HEADLINE {0!r}: NOT FOUND IN THIS MANIFEST"
                     .format(HEADLINE_BUILDING))
    if totals["boosted_hist"]:
        hist = totals["boosted_hist"]
        lines.append("  boosted-side histogram: " + ", ".join(
            "{0}={1}".format(s, n) for s, n in sorted(hist.items())))
        lines.append("  (item-1 acceptance: spread over all four bearings, "
                     "not clustered on one)")
    if totals["starved"]:
        lines.append("  {0} building(s) have ZERO baked events at all "
                     "(soot_plume.plan_events planned nothing — a bake-time "
                     "gap, not something this Flow-placement change can "
                     "fix); they still show interior-seat smoke as a last "
                     "resort so nothing is silent.".format(totals["starved"]))
    lines.append("  GRAND TOTAL Flow emitters = {0}".format(totals["total"]))
    return "\n".join(lines)


def vram_projection(new_total):
    flow_mib = _FLOW_MIB_PER_EMITTER * new_total
    blocks_needed = _BLOCKS_PER_EMITTER * new_total
    projected_used = _NONFLOW_BASELINE_MIB + flow_mib
    return {
        "flow_mib": flow_mib, "blocks_needed_min": blocks_needed,
        "projected_used_mib": projected_used,
        "projected_free_mib": LIVE_TOTAL_MIB - projected_used,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifest", default=DEFAULT_MANIFEST)
    ap.add_argument("--bakes", default=DEFAULT_BAKES)
    ap.add_argument("--dump", default=DEFAULT_DUMP)
    ap.add_argument("--out", default="")
    ap.add_argument("--old-emitters", type=int, default=8,
                    help="FA_EMITTERS the FIRST live run actually used")
    ap.add_argument("--old-budget", type=int, default=400,
                    help="FA_EMITTER_BUDGET the FIRST live run actually used")
    ap.add_argument("--new-emitters", type=int, default=30)
    ap.add_argument("--new-budget", type=int, default=900)
    ap.add_argument("--new-side-smoke-flame", type=int, default=5)
    ap.add_argument("--new-side-smoke-nonflame", type=int, default=6)
    ap.add_argument("--new-roof-intact", type=int, default=1)
    ap.add_argument("--new-roof-collapsed", type=int, default=2)
    ap.add_argument("--new-street-bias", type=int, default=2)
    ap.add_argument("--new-flame-min-clusters", type=int, default=3)
    ap.add_argument("--new-flame-extra-max", type=int, default=3)
    ap.add_argument("--new-flame-size-scaling", type=int, default=1)
    ap.add_argument("--new-smoke-size-scaling", type=int, default=1)
    ap.add_argument("--new-smoke-window-jets", type=int, default=1)
    ap.add_argument("--new-residual-flame-frac", type=float, default=0.4)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    if not os.path.isdir(args.bakes):
        print("*** bakes dir not found: {0}".format(args.bakes))
        return 1
    if not os.path.exists(args.manifest):
        print("*** manifest not found: {0}".format(args.manifest))
        return 1

    ns = _load_allocator()
    seed, rows, missing = load_rows(args.manifest, args.bakes)
    print("[dry-run] manifest {0} (seed {1}) -> {2} bake(s) matched, {3} "
         "missing/unmatched".format(args.manifest, seed, len(rows),
                                    len(missing)))
    for i, why in missing:
        print("  record {0}: {1}".format(i, why))

    street_positions = {}
    if os.path.exists(args.dump):
        street_positions = fal.load_dump_positions(args.dump)
        print("[dry-run] street positions: {0} building(s) loaded from {1}"
             .format(len(street_positions), args.dump))
    else:
        print("[dry-run] *** dump not found ({0}) — street bias will be "
             "OFF for the NEW pass".format(args.dump))

    old_rows, old_totals = run_pass(
        "OLD (first live run: FA_EMITTERS={0} FA_EMITTER_BUDGET={1}, no "
        "side/roof/cluster/size/residual knobs, no street bias)".format(
            args.old_emitters, args.old_budget),
        rows, ns, emitters=args.old_emitters, budget=args.old_budget,
        seed=args.seed)

    new_rows, new_totals = run_pass(
        "NEW (this change, all three review rounds)",
        rows, ns, emitters=args.new_emitters, budget=args.new_budget,
        side_smoke_flame_max=args.new_side_smoke_flame,
        side_smoke_nonflame_max=args.new_side_smoke_nonflame,
        roof_cap_intact=args.new_roof_intact,
        roof_cap_collapsed=args.new_roof_collapsed,
        street_positions=street_positions,
        street_bias_weight=args.new_street_bias,
        flame_min_clusters=(args.new_flame_min_clusters
                            if args.new_flame_min_clusters > 0 else None),
        flame_extra_max=args.new_flame_extra_max,
        flame_size_scaling=bool(args.new_flame_size_scaling),
        smoke_size_scaling=bool(args.new_smoke_size_scaling),
        smoke_window_jets=bool(args.new_smoke_window_jets),
        residual_flame_frac=args.new_residual_flame_frac, seed=args.seed)

    out_lines = []
    out_lines.append(render_table("OLD (matches the FIRST live 500 m city "
                                  "run)", old_rows, old_totals))
    out_lines.append("")
    out_lines.append(render_table(
        "NEW: FA_EMITTERS={0} FA_EMITTER_BUDGET={1} FA_SIDE_SMOKE_FLAME={2} "
        "FA_SIDE_SMOKE_MAX={3} FA_ROOF_INTACT_MAX={4} "
        "FA_ROOF_COLLAPSED_MAX={5} FA_STREET_BIAS={6} "
        "FA_FLAME_MIN_CLUSTERS={7} FA_FLAME_EXTRA_MAX={8} "
        "FA_FLAME_SIZE_SCALE={9} FA_SMOKE_SIZE_SCALE={10} "
        "FA_SMOKE_WINDOW_JETS={11} FA_RESIDUAL_FLAME_FRAC={12}".format(
            args.new_emitters, args.new_budget, args.new_side_smoke_flame,
            args.new_side_smoke_nonflame, args.new_roof_intact,
            args.new_roof_collapsed, args.new_street_bias,
            args.new_flame_min_clusters, args.new_flame_extra_max,
            int(args.new_flame_size_scaling), int(args.new_smoke_size_scaling),
            int(args.new_smoke_window_jets), args.new_residual_flame_frac),
        new_rows, new_totals))
    out_lines.append("")
    delta = new_totals["total"] - old_totals["total"]
    out_lines.append("=== DELTA ===")
    out_lines.append("  old total = {0}".format(old_totals["total"]))
    out_lines.append("  new total = {0}  (delta {1:+d}, {2:+.0%})".format(
        new_totals["total"], delta,
        delta / float(old_totals["total"]) if old_totals["total"] else 0.0))
    out_lines.append("  flame TOTAL: {0} -> {1}".format(
        old_totals["flame_total"], new_totals["flame_total"]))
    out_lines.append("  side smoke: {0} -> {1}   roof smoke: {2} -> {3}"
                     .format(old_totals["side_smoke"], new_totals["side_smoke"],
                             old_totals["roof"], new_totals["roof"]))
    proj_old = vram_projection(old_totals["total"])
    proj_new = vram_projection(new_totals["total"])
    out_lines.append("")
    out_lines.append("=== VRAM / BLOCK PROJECTION (two-point linear model, "
                     "calibrated against BOTH live measurements on record — "
                     "see LIVE_POINTS in this module; NOT a substitute for "
                     "the Kit-log OOM grep after a real relaunch) ===")
    for n, f, u in LIVE_POINTS:
        out_lines.append("  measured: {0} emitters -> {1:.0f} MiB Flow, "
                         "{2:.0f} MiB used, {3:.0f} MiB free of {4:.0f} MiB"
                         .format(n, f, u, LIVE_TOTAL_MIB - u, LIVE_TOTAL_MIB))
    out_lines.append("  projected OLD pass ({0} emitters): ~{1:.0f} MiB "
                     "Flow, ~{2:.0f} MiB used, ~{3:.0f} MiB free"
                     .format(old_totals["total"], proj_old["flow_mib"],
                             proj_old["projected_used_mib"],
                             proj_old["projected_free_mib"]))
    out_lines.append("  projected NEW pass ({0} emitters): ~{1:.0f} MiB "
                     "Flow, ~{2:.0f} MiB used, ~{3:.0f} MiB free of {4:.0f}"
                     .format(new_totals["total"], proj_new["flow_mib"],
                             proj_new["projected_used_mib"],
                             proj_new["projected_free_mib"], LIVE_TOTAL_MIB))
    cap_mib = 15.3 * 1024.0
    out_lines.append("  15.3 GB cap = {0:.0f} MiB -> {1} ({2:+.0f} MiB "
                     "margin)".format(
                         cap_mib,
                         "UNDER CAP" if proj_new["projected_used_mib"] <= cap_mib
                         else "*** OVER CAP ***",
                         cap_mib - proj_new["projected_used_mib"]))
    out_lines.append("  minimum block pool at this scale: ~{0:.0f} (round "
                     "UP, never down, and re-check the Kit log)".format(
                         proj_new["blocks_needed_min"]))
    out_lines.append("  minimum block pool if FA_CELL_M coarsens 0.45->0.55 "
                     "(voxel cost ~ 1/cell^3, so (0.45/0.55)^3={0:.3f}x): "
                     "~{1:.0f}".format(
                         (0.45 / 0.55) ** 3,
                         proj_new["blocks_needed_min"] * (0.45 / 0.55) ** 3))

    text = "\n".join(out_lines)
    print()
    print(text)
    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as fh:
            fh.write(text + "\n")
        print("\n[dry-run] wrote {0}".format(args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
