#!/usr/bin/env python3
"""fire_emitter_distribution_probe.py — offline before/after check for the
2026-08-31 `place_fire` fix (user review of the live 500 m fire city: "Live
fire seems to mostly only stay on 1 side of the building... also more
positions on each side. Also increase amount of smoke.").

RUNS ON THE HOST — real `pxr`, no Kit, no SimulationApp. Reads real bake
sidecars (read-only) from a `FA_BAKES`-style directory (default
`/home/krrishjain/docker/isaac-sim/cache/main/fire_bakes/city_4`) via
`fire_bake.load_for_assembly`, builds an in-memory `Usd.Stage`, and calls the
REAL `disaster.fire_assembly_lib.place_fire` — so this exercises the exact
code path both launchers use, not a re-implementation of it.

WHAT IT REPORTS

  1. Per-side / per-storey position spread for three sample buildings (a
     2-side F4, a 2-side F5, an F5c — see NOTE below on the "1-side F4"), new
     selection vs. the OLD nested-loop selection (kept verbatim in
     `_old_flame_selection` for the comparison only — it is not imported from
     anywhere, it is the pre-fix code as it read before this change).
  2. Smoke-related emitter counts and the actual authored Flow attribute
     values (radius, emission `smoke` field) at `FA_SMOKE_SCALE` 1.0 vs 2.0.
  3. Estimator parity: a local mirror of
     `urban_fire_city_launch_script.emitter_estimate`'s counting formula
     (copied verbatim — this script does not import that launcher, which
     builds a `SimulationApp` at module scope) checked against
     `place_fire`'s real returned counts, across every bake in the directory
     and several `max_emitters` values, with `smoke` True and False.

NOTE ON "A 1-SIDE F4": no bake in `city_4` happens to have its EVENTS
concentrated on only one side of an F4 building (every F4 bake there has
`sides` len 2, and in most of them both sides have live events) — the
directory just doesn't contain that combination today. So the "1-side F4"
sample is SYNTHESISED from a real 2-side F4 bake by dropping every event not
on its first side, which is an honest way to exercise the single-side case
end to end (real geometry references, real openings, real severities — only
the side filter is synthetic) without inventing fixture data from scratch.

    python3 scene_gen/tools/fire_emitter_distribution_probe.py [BAKE_DIR]
"""
import glob
import json
import os
import random
import sys
from collections import Counter

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SCENE_GEN)

from pxr import Usd, Sdf, UsdGeom  # noqa: E402

from disaster import fire_bake as fb              # noqa: E402
from disaster import fire_assembly_lib as fal     # noqa: E402
from disaster import urban_fire as uf             # noqa: E402
from disaster import soot_plume as spl            # noqa: E402

DEFAULT_DIR = "/home/krrishjain/docker/isaac-sim/cache/main/fire_bakes/city_4"


# ---------------------------------------------------------------------------
# 0) loading
# ---------------------------------------------------------------------------
def _live(ev):
    return all(not (o.get("e") or {}).get("dead") for o in (ev.get("ops") or []))


def load_all(bake_dir):
    out = []
    for j in sorted(glob.glob(os.path.join(bake_dir, "*.json"))):
        doc, masses, events = fb.load_for_assembly(j)
        out.append({"path": j, "stem": os.path.splitext(os.path.basename(j))[0],
                    "doc": doc, "masses": masses, "events": events})
    return out


def fire_state(doc, events):
    """Exactly `place_fire`'s own `(state, wisp_only)` decision, standalone
    (so this probe can pick sample buildings without authoring Flow)."""
    state = ((doc or {}).get("fire") or {}).get("state")
    if state:
        return state, False
    if any(ev.get("state") == "smoulder" and ev.get("ops")
           for ev in (events or [])):
        return "smoulder", True
    return None, False


# ---------------------------------------------------------------------------
# 1) the OLD selection, kept verbatim for the before/after comparison only
# ---------------------------------------------------------------------------
def _old_flame_pairs(evs_of_state, max_open):
    """The nested `for ev: for op: if n_open >= cap: break` selection as it
    read before this fix — a `break` only escapes the inner loop, so once
    the cap is hit every remaining event is still visited (and immediately
    re-broken on its own first opening). Returns the same `(ev, op)` pairs
    `_flame_selection_order` returns, in the OLD order, so the two can be
    compared side by side."""
    out = []
    n_open = 0
    for ev in evs_of_state:
        for op in ev["ops"]:
            if n_open >= max_open:
                break
            out.append((ev, op))
            n_open += 1
    return out


def _old_events_slice(events_list, budget):
    """The OLD smoke/wisp selection: a plain `list[:budget]` in whatever
    order the caller already sorted (or didn't)."""
    return list(events_list[:budget])


# ---------------------------------------------------------------------------
# 2) reporting helpers
# ---------------------------------------------------------------------------
def _side_storey_tally(pairs_or_events, is_pairs):
    c = Counter()
    for item in pairs_or_events:
        ev = item[0] if is_pairs else item
        c[(ev["side"], ev["storey"])] += 1
    return c


def report_selection(label, evs_of_state, max_open):
    old = _old_flame_pairs(evs_of_state, max_open)
    # `_flame_selection_order` itself does not cap — place_fire applies the
    # cap while consuming it. Reproduce that consumption here so the two are
    # compared under the identical stopping rule.
    new = []
    n_open = 0
    for ev, op in fal._flame_selection_order(evs_of_state):
        if n_open >= max_open:
            break
        new.append((ev, op))
        n_open += 1
    print("  [{0}] max_open={1}  total available ops={2}".format(
        label, max_open, sum(len(e["ops"]) for e in evs_of_state)))
    print("      OLD sides/storeys hit: {0}  (n={1})".format(
        dict(_side_storey_tally(old, True)), len(old)))
    print("      NEW sides/storeys hit: {0}  (n={1})".format(
        dict(_side_storey_tally(new, True)), len(new)))
    assert len(old) == len(new), "total selected must be IDENTICAL (order-only fix)"
    return old, new


# ---------------------------------------------------------------------------
# 3) the estimator, mirrored verbatim from
#    simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py
#    (not imported: that file builds a SimulationApp at module scope)
# ---------------------------------------------------------------------------
def emitter_estimate(doc, events, max_emitters, smoke):
    f = (doc or {}).get("fire") or {}
    state, wisp = fire_state(doc, events)
    zero = {"flame": 0, "smoke": 0, "interior": 0, "roof": 0, "openings": 0,
            "total": 0}
    if not state:
        return dict(zero)
    evs = [ev for ev in (events or []) if _live(ev) and ev.get("ops")]
    is_flame = state == "flame"
    n_st = int(f.get("n_storeys") or 0)
    max_open = (max(max_emitters, min(16, n_st // 2)) if n_st >= 12
                else max_emitters)
    n_flame = n_open = 0
    for ev in [e for e in evs if e["state"] == "flame"]:
        for _op in ev["ops"]:
            if n_open >= max_open:
                break
            n_flame += uf.FLAME_PER_OPENING
            n_open += 1
    if state == "smoulder" and not wisp:
        for ev in [e for e in evs if e["state"] == "smoulder"]:
            for _op in ev["ops"]:
                if n_open >= max(2, max_open // 2):
                    break
                n_flame += max(1, uf.FLAME_PER_OPENING - 1)
                n_open += 1
    if not smoke:
        return {"flame": n_flame, "smoke": 0, "interior": 0, "roof": 0,
                "openings": n_open, "total": n_flame}
    if wisp:
        n = len([e for e in evs if e["state"] == "smoulder"][:2])
        return {"flame": 0, "smoke": n, "interior": 0, "roof": 0,
                "openings": 0, "total": n}
    if is_flame:
        n_smoke = len([e for e in evs if e["state"] == "out"][:uf.SMOKE_EXTRA_MAX])
    else:
        n_smoke = len([e for e in evs
                       if e["state"] == "smoulder"][:spl.SMOULDER_EVENTS_MAX])
    seats = (doc or {}).get("seats") or {}
    n_int = 0 if is_flame else len(seats.get("interior") or [])
    n_roof = len(seats.get("roof") or []) if f.get("roof") else 0
    return {"flame": n_flame, "smoke": n_smoke, "interior": n_int,
            "roof": n_roof, "openings": n_open,
            "total": n_flame + n_smoke + n_int + n_roof}


# ---------------------------------------------------------------------------
# 4) running the real place_fire on an in-memory stage
# ---------------------------------------------------------------------------
def run_place_fire(row, max_emitters=8, smoke=True, smoke_scale=1.0, seed=7):
    stage = Usd.Stage.CreateInMemory()
    root = "/World/bldg"
    UsdGeom.Xform.Define(stage, Sdf.Path(root))
    rng = random.Random(seed)
    res = fal.place_fire(stage, root, row["doc"], row["masses"], row["events"],
                         "p", rng, None, 0.0, 0.0, scale=1.0,
                         max_emitters=max_emitters, smoke=smoke,
                         smoke_scale=smoke_scale)
    return stage, res


def flow_prim_values(stage, root="/World/bldg"):
    """`[(path, radius_or_None, smoke_field_or_None), ...]` for every
    FlowEmitter* prim authored — proves `smoke_scale` actually reached the
    Flow attributes, not just the returned counts."""
    out = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(Sdf.Path(root))):
        tn = prim.GetTypeName()
        if not tn.startswith("Flow"):
            continue
        r = prim.GetAttribute("radius")
        radius = r.Get() if r and r.IsValid() else None
        s = prim.GetAttribute("smoke")
        smoke_v = s.Get() if s and s.IsValid() else None
        out.append((str(prim.GetPath()), radius, smoke_v))
    return out


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    bake_dir = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DIR
    rows = load_all(bake_dir)
    print("[probe] loaded {0} sidecar(s) from {1}".format(len(rows), bake_dir))

    by_stem = {r["stem"]: r for r in rows}

    # ---- sample buildings -------------------------------------------------
    F4_2SIDE = "kit_tower_F4_o5_EN_s66"
    F5_2SIDE = "gac_SM_Building_30_F5_o13_EN_s159"
    F5C = "kit_commercial_mid_F5c_o1_SE_s159"
    for need in (F4_2SIDE, F5_2SIDE, F5C):
        if need not in by_stem:
            print("[probe] *** missing expected sample bake:", need)

    print("\n" + "=" * 78)
    print("1) POSITIONS PER SIDE / STOREY")
    print("=" * 78)

    # -- a) 2-side F4 --------------------------------------------------------
    r = by_stem[F4_2SIDE]
    doc, events = r["doc"], r["events"]
    f = doc["fire"]
    n_st = int(f.get("n_storeys") or 0)
    max_open = max(8, min(16, n_st // 2)) if n_st >= 12 else 8
    evs = [ev for ev in events if _live(ev) and ev.get("ops")]
    print("\n-- {0} (F4, sides={1}, n_storeys={2}, state={3}) --"
          .format(F4_2SIDE, f.get("sides"), n_st, f.get("state")))
    flame_evs = [e for e in evs if e["state"] == "flame"]
    if flame_evs:
        report_selection("flame events", flame_evs, max_open)
    smoulder_evs = [e for e in evs if e["state"] == "smoulder"]
    if smoulder_evs and f.get("state") == "smoulder":
        cap = max(2, max_open // 2)
        report_selection("F4 smoulder top-up", smoulder_evs, cap)

    # -- b) SYNTHETIC 1-side F4: same bake, events filtered to its first side
    first_side = (f.get("sides") or ["S"])[0]
    evs_one_side = [ev for ev in evs if ev["side"] == first_side]
    print("\n-- {0} FILTERED TO SIDE {1!r} ONLY (synthetic 1-side F4) --"
          .format(F4_2SIDE, first_side))
    smoulder_one = [e for e in evs_one_side if e["state"] == "smoulder"]
    if smoulder_one:
        cap = max(2, max_open // 2)
        old, new = report_selection("F4 smoulder top-up, 1 side", smoulder_one, cap)
        sides_new = {ev["side"] for ev, _ in new}
        assert sides_new == {first_side}, "single-side input must stay single-side"
        print("      (single side by construction — round-robin now spreads "
              "STOREYS/openings within it: {0})"
              .format(sorted({ev["storey"] for ev, _ in new})))

    # -- c) 2-side F5 ---------------------------------------------------------
    r = by_stem[F5_2SIDE]
    doc, events = r["doc"], r["events"]
    f = doc["fire"]
    evs = [ev for ev in events if _live(ev) and ev.get("ops")]
    print("\n-- {0} (F5, sides={1}, state={2}) --"
          .format(F5_2SIDE, f.get("sides"), f.get("state")))
    out_evs = [e for e in evs if e["state"] == "out"]
    out_ranked = sorted(out_evs, key=lambda e: (-e["storey"], e["id"]))
    old_out = _old_events_slice(out_ranked, uf.SMOKE_EXTRA_MAX)
    new_out = fal._select_events_by_side(out_ranked, uf.SMOKE_EXTRA_MAX)
    print("  [out->smoke, active-fire branch] budget={0}".format(uf.SMOKE_EXTRA_MAX))
    print("      OLD sides hit: {0}".format(dict(_side_storey_tally(old_out, False))))
    print("      NEW sides hit: {0}".format(dict(_side_storey_tally(new_out, False))))
    sm_evs = [e for e in evs if e["state"] == "smoulder"]
    old_sm = _old_events_slice(sm_evs, spl.SMOULDER_EVENTS_MAX)
    new_sm = fal._select_events_by_side(sm_evs, spl.SMOULDER_EVENTS_MAX)
    print("  [smoulder->smoke branch] budget={0}, candidates on sides {1}"
          .format(spl.SMOULDER_EVENTS_MAX, sorted({e["side"] for e in sm_evs})))
    print("      OLD sides hit: {0}".format(dict(_side_storey_tally(old_sm, False))))
    print("      NEW sides hit: {0}".format(dict(_side_storey_tally(new_sm, False))))
    if len({e["side"] for e in sm_evs}) < 2:
        print("      (only one side has 'smoulder'-state events in this real "
              "bake — the fix cannot spread across sides that have no "
              "candidates; this is a genuine event-state limitation, not a "
              "selection-order bug)")

    # -- d) F5c ---------------------------------------------------------------
    r = by_stem[F5C]
    doc, events = r["doc"], r["events"]
    f = doc["fire"]
    evs = [ev for ev in events if _live(ev) and ev.get("ops")]
    print("\n-- {0} (F5c, sides={1}, state={2}) --"
          .format(F5C, f.get("sides"), f.get("state")))
    sm_evs = [e for e in evs if e["state"] == "smoulder"]
    print("  live events by (state, side): {0}"
          .format(dict(Counter((e["state"], e["side"]) for e in evs))))
    if sm_evs:
        old_sm = _old_events_slice(sm_evs, spl.SMOULDER_EVENTS_MAX)
        new_sm = fal._select_events_by_side(sm_evs, spl.SMOULDER_EVENTS_MAX)
        print("      OLD sides hit: {0}".format(dict(_side_storey_tally(old_sm, False))))
        print("      NEW sides hit: {0}".format(dict(_side_storey_tally(new_sm, False))))
    else:
        print("      no 'smoulder'-state events in this bake at all (all "
              "{0} live events are 'out') — its event-based smoke is 0 "
              "either way; interior/roof SEAT plumes (scaled by "
              "smoke_scale below) are what actually shows for it"
              .format(len(evs)))

    # ------------------------------------------------------------------
    print("\n" + "=" * 78)
    print("2) SMOKE AMOUNT — FA_SMOKE_SCALE 1.0 vs 2.0")
    print("=" * 78)
    for stem in (F4_2SIDE, F5_2SIDE, F5C):
        r = by_stem[stem]
        stage1, res1 = run_place_fire(r, max_emitters=8, smoke=True, smoke_scale=1.0)
        stage2, res2 = run_place_fire(r, max_emitters=8, smoke=True, smoke_scale=2.0)
        print("\n-- {0} --".format(stem))
        print("  counts @1.0x: smoke={0} interior={1} roof={2}".format(
            res1["smoke"], res1["interior"], res1["roof"]))
        print("  counts @2.0x: smoke={0} interior={1} roof={2}".format(
            res2["smoke"], res2["interior"], res2["roof"]))
        assert (res1["smoke"], res1["interior"], res1["roof"]) == \
               (res2["smoke"], res2["interior"], res2["roof"]), \
               "smoke_scale must not change emitter COUNTS (budget parity)"
        v1 = flow_prim_values(stage1)
        v2 = flow_prim_values(stage2)
        r1 = [r for _, r, _ in v1 if r is not None]
        r2 = [r for _, r, _ in v2 if r is not None]
        s1 = [s for _, _, s in v1 if s is not None]
        s2 = [s for _, _, s in v2 if s is not None]
        if r1 and r2:
            print("  sphere radius (interior/roof), mean: {0:.3f} -> {1:.3f}"
                  .format(sum(r1) / len(r1), sum(r2) / len(r2)))
        if s1 and s2:
            print("  Flow 'smoke' field, mean: {0:.3f} -> {1:.3f}  (ratio {2:.2f}x)"
                  .format(sum(s1) / len(s1), sum(s2) / len(s2),
                          (sum(s2) / len(s2)) / max(1e-9, sum(s1) / len(s1))))

    # ------------------------------------------------------------------
    print("\n" + "=" * 78)
    print("3) ESTIMATOR PARITY")
    print("=" * 78)
    fails = 0
    checks = 0
    for r in rows:
        for max_emitters in (1, 4, 6, 8, 16):
            for smoke in (True, False):
                est = emitter_estimate(r["doc"], r["events"], max_emitters, smoke)
                _, res = run_place_fire(r, max_emitters=max_emitters, smoke=smoke)
                checks += 1
                got = {"flame": res.get("flame", 0), "smoke": res.get("smoke", 0),
                       "interior": res.get("interior", 0), "roof": res.get("roof", 0),
                       "openings": res.get("openings", 0)}
                # `emitter_estimate`'s own docstring: an UPPER BOUND (a Flow
                # prim can fail to create) — but on a bare in-memory stage
                # nothing fails, so this must be an EXACT match, not just <=.
                exp = {k: est[k] for k in got}
                if got != exp:
                    fails += 1
                    print("  MISMATCH {0} max_emitters={1} smoke={2}: "
                          "estimate={3} actual={4}".format(
                              r["stem"], max_emitters, smoke, exp, got))
    print("[probe] estimator parity: {0}/{1} checks matched".format(
        checks - fails, checks))
    if fails:
        print("[probe] *** {0} PARITY MISMATCH(ES) ***".format(fails))
        sys.exit(1)
    print("\n[probe] ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
