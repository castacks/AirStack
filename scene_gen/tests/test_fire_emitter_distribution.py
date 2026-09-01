#!/usr/bin/env python3
"""test_fire_emitter_distribution.py — the 2026-08-31 `place_fire` fix.

User review of the live 500 m fire city: "Live fire seems to mostly only
stay on 1 side of the building... also more positions on each side. Also
increase amount of smoke."

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        python tests/test_fire_emitter_distribution.py
    docker exec isaac-sim bash -c \\
        "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
         /isaac-sim/AirStack/scene_gen/tests/test_fire_emitter_distribution.py"

    # the CONTACT-SNAP section (2026-08-31, "fires floating outside the
    # building") also needs `vtk` — those tests SKIP (print "skip", no
    # failure) rather than error when it is not importable, same as
    # `fire_bake._judge_candidates`'s own graceful-degrade:
    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        --with vtk python tests/test_fire_emitter_distribution.py

RUNS ONLY UNDER A REAL `pxr` — same reason as `test_fire_city_dressing.py`:
`fire_assembly_lib` imports `pxr` at module level. Every fixture here is
FABRICATED (a minimal but structurally real mass/opening/event, built the
same shape `fire_bake.op_from_json`/`events_from_json` produce) rather than
read from a bake on disk, so this file is self-contained and portable —
`scene_gen/tools/fire_emitter_distribution_probe.py` is the counterpart that
drives the same code against real sidecars from a live bake cache.

WHAT THIS PINS DOWN

  1. `_spread_order` is a permutation of `range(n)` whose prefixes actually
     spread across the range (not left-to-right).
  2. `_round_robin` interleaves without dropping or reordering within a
     group.
  3. `_flame_selection_order` / `_select_events_by_side` reproduce the OLD
     nested-loop / plain-slice TOTAL exactly (order-only fix — the emitter
     budget accounting must not move) while actually spreading picks across
     sides once more than one side has candidates.
  4. THE REPORTED BUG, REPRODUCED AND FIXED: a side whose one event has far
     more openings than the whole budget used to eat the entire budget under
     the old nested loop (`break` only escapes the inner `for`); the new
     selection gives every side with candidates at least one pick.
  5. `place_fire` end to end: opening/flame/smoke/interior/roof counts are
     unaffected by `smoke_scale` (budget parity with the global emitter
     allocator), while the actual authored Flow `radius`/`smoke` attribute
     values scale with it; the default (no `smoke_scale` passed) matches
     `smoke_scale=1.0` exactly, which is what makes this change safe for the
     row launcher's existing call sites.
  6. A local mirror of `urban_fire_city_launch_script.emitter_estimate`
     (copied verbatim — that launcher builds a `SimulationApp` at import and
     cannot be imported here) matches `place_fire`'s real counts across
     flame, F4-smoulder, F1-wisp and F5/residual fixtures and several
     `max_emitters` values.

2026-08-31, ROUND TWO — the same review's other two asks ("especially on
street facing sides", "more smoke coming from sides than top... unless the
roof is collapsed"):

  7. `_round_robin(groups, weights=...)` — weight 1 for every group
     reproduces the ORIGINAL unweighted output exactly; a higher weight
     front-loads that one group without starving the others to zero.
  8. `_side_weights` — applies whichever ONE side `choose_street_side`
     already picked; `_group_by_side` itself carries no rank/side argument
     at all (2026-08-31, THIRD review: "street facing fires all seem to be
     in the top left. I want it randomized" — the deterministic-argmax
     version of this moved into a per-building RANDOM, weighted draw; see
     item 12 below and `choose_street_side`'s own docstring).
  9. `load_dump_positions`/`street_side_ranks` — round-trip a real
     `fire_city_placements_dump.v1` file and score a building's own sides
     against `disaster.urban_fire_spread.street_side_score`; a close
     neighbour on one side scores lower than open air on another.
 10. `place_fire`'s new `side_smoke_flame_max`/`side_smoke_nonflame_max`/
     `roof_cap_intact`/`roof_cap_collapsed` — `None` (unchanged call sites)
     reproduces the ORIGINAL fixed budgets and roof count exactly; the
     non-flame "out"-event fill-in only engages once
     `side_smoke_nonflame_max` is actually given; `roof_has_collapsed`
     picks the right cap from the level and from the `top_z`/`deck_z` gap.
 11. The `emitter_estimate` mirror, extended with the same four new
     arguments, stays in parity with `place_fire` across the new knobs too.
"""
import json
import math
import os
import random
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import fire_assembly_lib as fal              # noqa: E402
from disaster import gac_fire as gf                         # noqa: E402
from disaster import soot_plume as spl                     # noqa: E402
from disaster import urban_fire as uf                       # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom                        # noqa: E402

try:
    import vtk as _vtk_mod                                  # noqa: F401
    HAVE_VTK = True
except Exception:
    HAVE_VTK = False

_FAILS = []


def check(name, cond, detail=""):
    if cond:
        print("ok  " + name)
    else:
        print("FAIL " + name + (": " + detail if detail else ""))
        _FAILS.append(name)


# ---------------------------------------------------------------------------
# fixtures — fabricated but structurally real (same shape as
# fire_bake.mass_from_json / op_from_json / events_from_json)
# ---------------------------------------------------------------------------
def _fake_mass(tag="main", W=20.0, D=14.0, cx=0.0, cy=0.0, yaw=0.0, top=18.0,
              levels=None):
    return {"tag": tag, "cx": cx, "cy": cy, "yaw": yaw, "W": W, "D": D,
            "z0": 0.0, "top": top,
            "levels": levels or [0.0, 3.0, 6.0, 9.0, 12.0, 15.0],
            "module": 4.0, "spec": {"bands": []}}


def _fake_op(side, storey, m, mass_tag="main"):
    fr = (m["cx"], m["cy"], math.radians(m["yaw"]), 4.0, 3.0, -0.3, False)
    e = {"dead": False, "mass": mass_tag, "name": "wall", "role": "wall",
         "side": side, "storey": storey, "x": 0.0, "y": 0.0, "z": 0.0}
    return {"fr": fr, "side": side, "storey": storey, "mass": mass_tag,
            "span": (0.0, 4.0, 3.0, 6.0), "e": e, "m": m,
            "ua": 0.0, "ub": 4.0, "va": 3.0, "vb": 6.0, "out": -0.3}


def _fake_event(eid, side, storey, state, n_ops, m, mass_tag="main"):
    return {"id": eid, "mass": mass_tag, "side": side, "storey": storey,
            "state": state,
            "ops": [_fake_op(side, storey, m, mass_tag) for _ in range(n_ops)]}


def _fake_doc(level, state, sides, origin=4, top=6, n_storeys=8, roof=True,
             mass_tag="main", seats=None):
    return {"fire": {"level": level, "state": state, "sides": list(sides),
                    "origin": origin, "storeys": list(range(origin, top + 1)),
                    "top": top, "n_storeys": n_storeys, "mass": mass_tag,
                    "roof": roof},
           "seats": seats if seats is not None else
           {"interior": [{"x": 0.0, "y": 0.0, "z": 5.0, "radius": 1.2,
                          "scale": 1.0}],
            "roof": [{"x": 0.0, "y": 0.0, "z": 18.0, "radius": 1.4,
                      "scale": 1.0}]}}


# the exact shape from the review: one side's event has far more openings
# than any reasonable per-building budget, two other sides have a couple
# each — under the OLD code the big side alone would spend the whole budget.
def _bug_shape_events(m):
    return [
        _fake_event(0, "S", 5, "flame", 10, m),
        _fake_event(1, "N", 5, "flame", 2, m),
        _fake_event(2, "E", 5, "flame", 2, m),
    ]


def _new_stage(root="/World/b"):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, Sdf.Path(root))
    return stage, root


def _flow_values(stage, root):
    out = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(Sdf.Path(root))):
        if not prim.GetTypeName().startswith("Flow"):
            continue
        r = prim.GetAttribute("radius")
        s = prim.GetAttribute("smoke")
        out.append((str(prim.GetPath()),
                    r.Get() if r and r.IsValid() else None,
                    s.Get() if s and s.IsValid() else None))
    return out


# ---------------------------------------------------------------------------
# 1) _spread_order
# ---------------------------------------------------------------------------
def test_spread_order_is_a_permutation():
    for n in (0, 1, 2, 3, 6, 7, 13):
        order = fal._spread_order(n)
        check("test_spread_order_is_a_permutation[n={0}]".format(n),
              sorted(order) == list(range(n)), str(order))


def test_spread_order_prefix_spans_the_range():
    # first two picks of an 8-wide run must not be adjacent left-end indices
    order = fal._spread_order(8)
    first_two = sorted(order[:2])
    check("test_spread_order_prefix_spans_the_range",
          first_two != [0, 1] and (first_two[1] - first_two[0]) >= 2,
          str(order))


# ---------------------------------------------------------------------------
# 2) _round_robin
# ---------------------------------------------------------------------------
def test_round_robin_preserves_membership_and_group_order():
    groups = [["a1", "a2", "a3"], ["b1"], ["c1", "c2"]]
    flat = fal._round_robin(groups)
    check("test_round_robin_preserves_membership",
          sorted(flat) == sorted(x for g in groups for x in g), str(flat))
    check("test_round_robin_interleaves_first_round",
          flat[:3] == ["a1", "b1", "c1"], str(flat))
    # each group's own relative order survives
    check("test_round_robin_preserves_within_group_order",
          [x for x in flat if x.startswith("a")] == groups[0], str(flat))


def test_round_robin_empty_groups_are_skipped_not_padded():
    flat = fal._round_robin([[], ["x", "y"], []])
    check("test_round_robin_empty_groups_are_skipped_not_padded",
          flat == ["x", "y"], str(flat))


# ---------------------------------------------------------------------------
# 3) THE REPORTED BUG — reproduced against the OLD selection, fixed by the NEW
# ---------------------------------------------------------------------------
def _old_flame_pairs(evs_of_state, max_open):
    """The pre-fix nested loop, verbatim: a `break` only escapes the INNER
    `for`, so the outer loop still visits every remaining event once the cap
    is hit (and immediately re-breaks) — kept here ONLY as the baseline this
    test compares the fix against."""
    out, n_open = [], 0
    for ev in evs_of_state:
        for op in ev["ops"]:
            if n_open >= max_open:
                break
            out.append((ev, op))
            n_open += 1
    return out


def test_flame_selection_order_fixes_the_one_sided_bug():
    m = _fake_mass()
    events = _bug_shape_events(m)
    max_open = 6
    old = _old_flame_pairs(events, max_open)
    new = []
    for ev, op in fal._flame_selection_order(events):
        if len(new) >= max_open:
            break
        new.append((ev, op))
    old_sides = {ev["side"] for ev, _ in old}
    new_sides = {ev["side"] for ev, _ in new}
    check("test_flame_selection_order_reproduces_the_old_bug",
          old_sides == {"S"}, str(old_sides))
    check("test_flame_selection_order_fixes_the_one_sided_bug",
          new_sides == {"S", "N", "E"}, str(new_sides))
    check("test_flame_selection_order_total_selected_is_unchanged",
          len(old) == len(new) == max_open,
          "{0} vs {1}".format(len(old), len(new)))


def test_flame_selection_order_preserves_total_across_caps():
    m = _fake_mass()
    events = _bug_shape_events(m)
    for cap in (0, 1, 2, 3, 5, 6, 9, 14, 20):
        old = _old_flame_pairs(events, cap)
        new_all = fal._flame_selection_order(events)
        new = new_all[:min(cap, len(new_all))]
        check("test_flame_selection_order_preserves_total[cap={0}]".format(cap),
              len(old) == len(new), "{0} vs {1}".format(len(old), len(new)))


def test_flame_selection_order_is_deterministic():
    m = _fake_mass()
    events = _bug_shape_events(m)
    a = [(ev["id"], id(op)) for ev, op in fal._flame_selection_order(events)]
    b = [(ev["id"], id(op)) for ev, op in fal._flame_selection_order(events)]
    check("test_flame_selection_order_is_deterministic", a == b)


def test_flame_selection_order_spreads_openings_within_one_side():
    # a single side, one event, six openings, budget 2 — the two picks
    # should not both be the leftmost pair of the run.
    m = _fake_mass()
    ev = _fake_event(0, "S", 5, "flame", 6, m)
    picked = fal._flame_selection_order([ev])[:2]
    idx = sorted(ev["ops"].index(op) for _, op in picked)
    check("test_flame_selection_order_spreads_openings_within_one_side",
          idx != [0, 1], str(idx))


# ---------------------------------------------------------------------------
# 4) _select_events_by_side (the smoke/wisp event-level selector)
# ---------------------------------------------------------------------------
def test_select_events_by_side_round_robins():
    m = _fake_mass()
    events = [_fake_event(0, "S", 5, "out", 1, m),
             _fake_event(1, "S", 6, "out", 1, m),
             _fake_event(2, "N", 5, "out", 1, m)]
    picked = fal._select_events_by_side(events, 2)
    sides = [ev["side"] for ev in picked]
    check("test_select_events_by_side_round_robins",
          sides == ["S", "N"], str(sides))


def test_select_events_by_side_preserves_total_and_budget_cap():
    m = _fake_mass()
    events = [_fake_event(i, "SNEW"[i % 4], 5, "out", 1, m) for i in range(9)]
    for budget in (0, 1, 3, 5, 9, 20):
        picked = fal._select_events_by_side(events, budget)
        check("test_select_events_by_side_budget[budget={0}]".format(budget),
              len(picked) == min(budget, len(events)),
              "{0} != min({1},{2})".format(len(picked), budget, len(events)))


# ---------------------------------------------------------------------------
# 5) place_fire end to end
# ---------------------------------------------------------------------------
def test_place_fire_flame_openings_spread_across_sides():
    m = _fake_mass()
    masses = {"main": m}
    events = _bug_shape_events(m)
    doc = _fake_doc("F3", "flame", ["S", "N", "E"])
    stage, root = _new_stage()
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(1), None, 0.0, 0.0, max_emitters=6,
                         smoke=True)
    paths = [p for p, _, _ in _flow_values(stage, root)]
    tags_hit = {p.split("/")[-1].split("_")[1] for p in paths
               if "_e" in p}
    check("test_place_fire_flame_openings_spread_across_sides",
          res["openings"] == 6 and tags_hit == {"e0", "e1", "e2"},
          "openings={0} tags={1}".format(res["openings"], tags_hit))


def test_place_fire_smoke_scale_doubles_density_not_count():
    # "residual" (F5), not "smoulder" (F4): F4's own flame-widening loop
    # relights a few of these same "smoulder"-state events as `state="flame"`
    # sources, which `smoke_scale` deliberately does NOT touch (the ask was
    # more SMOKE, not a brighter fire) — using F5/residual keeps every
    # emitter in this fixture genuinely smoke-only, so "does smoke_scale
    # reach every smoke attribute" is not muddied by flame attributes that
    # are supposed to stay put.
    m = _fake_mass()
    masses = {"main": m}
    events = [_fake_event(0, "S", 4, "smoulder", 2, m),
             _fake_event(1, "N", 4, "smoulder", 2, m),
             _fake_event(2, "S", 3, "out", 1, m)]
    doc = _fake_doc("F5", "residual", ["S", "N"])
    s1, r1 = _new_stage("/World/b1")
    res1 = fal.place_fire(s1, r1, doc, masses, events, "t", random.Random(2),
                          None, 0.0, 0.0, max_emitters=6, smoke=True,
                          smoke_scale=1.0)
    s2, r2 = _new_stage("/World/b2")
    res2 = fal.place_fire(s2, r2, doc, masses, events, "t", random.Random(2),
                          None, 0.0, 0.0, max_emitters=6, smoke=True,
                          smoke_scale=2.0)
    same_counts = all(res1[k] == res2[k]
                      for k in ("flame", "smoke", "interior", "roof", "openings"))
    check("test_place_fire_smoke_scale_does_not_change_counts", same_counts,
          "{0} vs {1}".format(res1, res2))
    v1 = _flow_values(s1, r1)
    v2 = _flow_values(s2, r2)
    r1_vals = [r for _, r, _ in v1 if r is not None]
    r2_vals = [r for _, r, _ in v2 if r is not None]
    s1_vals = [s for _, _, s in v1 if s is not None]
    s2_vals = [s for _, _, s in v2 if s is not None]
    check("test_place_fire_smoke_scale_doubles_seat_radius",
          bool(r1_vals) and bool(r2_vals) and
          all(abs(b - 2.0 * a) < 1e-6 for a, b in zip(sorted(r1_vals), sorted(r2_vals))),
          "{0} vs {1}".format(r1_vals, r2_vals))
    check("test_place_fire_smoke_scale_doubles_smoke_emission",
          bool(s1_vals) and bool(s2_vals) and
          all(abs(b - 2.0 * a) < 1e-6 for a, b in zip(sorted(s1_vals), sorted(s2_vals))),
          "{0} vs {1}".format(s1_vals, s2_vals))


def test_place_fire_default_smoke_scale_matches_explicit_one():
    m = _fake_mass()
    masses = {"main": m}
    events = [_fake_event(0, "S", 4, "smoulder", 2, m),
             _fake_event(1, "N", 4, "smoulder", 2, m)]
    doc = _fake_doc("F4", "smoulder", ["S", "N"])
    s1, r1 = _new_stage("/World/d1")
    # exactly the row/city launchers' own positional call shape — no
    # smoke_scale at all.
    res1 = fal.place_fire(s1, r1, doc, masses, events, "t", random.Random(3),
                          None, 0.0, 0.0, 1.0, 6, True)
    s2, r2 = _new_stage("/World/d2")
    res2 = fal.place_fire(s2, r2, doc, masses, events, "t", random.Random(3),
                          None, 0.0, 0.0, scale=1.0, max_emitters=6,
                          smoke=True, smoke_scale=1.0)
    check("test_place_fire_default_smoke_scale_matches_explicit_one",
          res1 == res2, "{0} vs {1}".format(res1, res2))
    v1 = sorted(v for v in _flow_values(s1, r1))
    v2 = sorted(v for v in _flow_values(s2, r2))
    check("test_place_fire_default_smoke_scale_matches_explicit_one_attrs",
          [(r, s) for _, r, s in v1] == [(r, s) for _, r, s in v2],
          "{0} vs {1}".format(v1, v2))


def test_place_fire_smoke_scale_is_keyword_only():
    m = _fake_mass()
    masses = {"main": m}
    events = [_fake_event(0, "S", 4, "smoulder", 2, m)]
    doc = _fake_doc("F4", "smoulder", ["S"])
    stage, root = _new_stage("/World/kwonly")
    try:
        fal.place_fire(stage, root, doc, masses, events, "t",
                       random.Random(4), None, 0.0, 0.0, 1.0, 6, True, 2.0)
        raised = False
    except TypeError:
        raised = True
    check("test_place_fire_smoke_scale_is_keyword_only", raised,
          "positional 14th arg must be rejected (frozen positional "
          "signature — smoke_scale is additive-only)")


# ---------------------------------------------------------------------------
# 6) estimator parity — verbatim mirror of
#    urban_fire_city_launch_script.emitter_estimate (not importable: that
#    launcher builds a SimulationApp at module scope)
# ---------------------------------------------------------------------------
def _live(ev):
    return all(not (o.get("e") or {}).get("dead") for o in (ev.get("ops") or []))


def _fire_state(doc, events):
    state = ((doc or {}).get("fire") or {}).get("state")
    if state:
        return state, False
    if any(ev.get("state") == "smoulder" and ev.get("ops")
           for ev in (events or [])):
        return "smoulder", True
    return None, False


def _emitter_estimate(doc, events, max_emitters, smoke,
                      side_smoke_flame_max=None, side_smoke_nonflame_max=None,
                      roof_cap_intact=None, roof_cap_collapsed=None,
                      flame_min_clusters=None, flame_extra_max=None,
                      flame_size_scaling=False, smoke_size_scaling=False,
                      smoke_window_jets=False, residual_flame_frac=0.0):
    """Verbatim mirror of `urban_fire_city_launch_script.emitter_estimate`,
    kept in sync by hand (that launcher cannot be imported — SimulationApp)
    — see this module's own docstring, item 6/11."""
    f = (doc or {}).get("fire") or {}
    state, wisp = _fire_state(doc, events)
    if not state:
        return {"flame": 0, "smoke": 0, "interior": 0, "roof": 0,
                "openings": 0, "total": 0}
    evs = [ev for ev in (events or []) if _live(ev) and ev.get("ops")]
    is_flame = state == "flame"
    n_st = int(f.get("n_storeys") or 0)
    if flame_size_scaling:
        max_open = min(max_emitters, fal.flame_window_target(n_st))
    else:
        max_open = (max(max_emitters, min(16, n_st // 2)) if n_st >= 12
                    else max_emitters)
    n_flame = n_open = 0
    lit_groups = set()
    for ev in [e for e in evs if e["state"] == "flame"]:
        for _op in ev["ops"]:
            if n_open >= max_open:
                break
            n_flame += uf.FLAME_PER_OPENING
            lit_groups.add((ev.get("side"), ev.get("storey")))
            n_open += 1
    residual_on = bool(residual_flame_frac) and float(residual_flame_frac) > 0.0
    if state == "smoulder" and not wisp and not residual_on:
        for ev in [e for e in evs if e["state"] == "smoulder"]:
            for _op in ev["ops"]:
                if n_open >= max(2, max_open // 2):
                    break
                n_flame += max(1, uf.FLAME_PER_OPENING - 1)
                n_open += 1
    if is_flame and flame_min_clusters is not None:
        extra_max_v = fal.FLAME_EXTRA_MAX if flame_extra_max is None \
            else int(flame_extra_max)
        extra_budget = min(max(0, max_open - n_open), max(0, extra_max_v))
        if len(lit_groups) < int(flame_min_clusters) and extra_budget > 0:
            n_extra = 0
            for ev in [e for e in evs if e["state"] == "out"]:
                for _op in ev["ops"]:
                    if n_extra >= extra_budget:
                        break
                    n_flame += max(1, uf.FLAME_PER_OPENING - 1)
                    n_open += 1
                    n_extra += 1
                if n_extra >= extra_budget:
                    break
    if not is_flame and not wisp and residual_on:
        residual_target = max(
            fal.RESIDUAL_FLAME_MIN,
            int(round(float(residual_flame_frac) * fal.flame_window_target(n_st))))
        n_sm_pockets = len([e for e in evs if e["state"] == "smoulder"])
        n_residual_sm = min(n_sm_pockets, residual_target)
        n_residual_out = 0
        if n_residual_sm < residual_target:
            n_out_pockets = len([e for e in evs if e["state"] == "out"])
            n_residual_out = min(n_out_pockets, residual_target - n_residual_sm)
        n_residual = n_residual_sm + n_residual_out
        n_flame += n_residual
        n_open += n_residual
    if not smoke:
        return {"flame": n_flame, "smoke": 0, "interior": 0, "roof": 0,
                "openings": n_open, "total": n_flame}
    if wisp:
        n = len([e for e in evs if e["state"] == "smoulder"][:2])
        return {"flame": 0, "smoke": n, "interior": 0, "roof": 0,
                "openings": 0, "total": n}
    side_flame_cap = (fal.smoke_window_target(n_st) if smoke_size_scaling
                      else (uf.SMOKE_EXTRA_MAX if side_smoke_flame_max is None
                            else max(0, int(side_smoke_flame_max))))
    side_nonflame_cap = (fal.smoke_window_target(n_st) if smoke_size_scaling
                         else (spl.SMOULDER_EVENTS_MAX
                               if side_smoke_nonflame_max is None
                               else max(0, int(side_smoke_nonflame_max))))
    out_fill_enabled = side_smoke_nonflame_max is not None or smoke_size_scaling
    if is_flame:
        if smoke_window_jets:
            n_smoke = 0
            for ev in [e for e in evs if e["state"] == "out"]:
                for _op in ev["ops"]:
                    if n_smoke >= side_flame_cap:
                        break
                    n_smoke += 1
                if n_smoke >= side_flame_cap:
                    break
        else:
            n_smoke = len([e for e in evs
                          if e["state"] == "out"][:side_flame_cap])
    else:
        if smoke_window_jets:
            n_smoke = 0
            for ev in [e for e in evs if e["state"] == "smoulder"]:
                for _op in ev["ops"]:
                    if n_smoke >= side_nonflame_cap:
                        break
                    n_smoke += 1
                if n_smoke >= side_nonflame_cap:
                    break
            if out_fill_enabled and n_smoke < side_nonflame_cap:
                for ev in [e for e in evs if e["state"] == "out"]:
                    for _op in ev["ops"]:
                        if n_smoke >= side_nonflame_cap:
                            break
                        n_smoke += 1
                    if n_smoke >= side_nonflame_cap:
                        break
        else:
            n_sm = min(len([e for e in evs if e["state"] == "smoulder"]),
                      side_nonflame_cap)
            n_out = 0
            if out_fill_enabled:
                n_out = min(len([e for e in evs if e["state"] == "out"]),
                           max(0, side_nonflame_cap - n_sm))
            n_smoke = n_sm + n_out
    seats = (doc or {}).get("seats") or {}
    n_int = (len(seats.get("interior") or [])
            if (not is_flame or (n_flame == 0 and n_smoke == 0)) else 0)
    n_roof = 0
    if f.get("roof"):
        roof_cap = 2
        if roof_cap_intact is not None or roof_cap_collapsed is not None:
            collapsed = fal.roof_has_collapsed(doc or {})
            roof_cap = (roof_cap_collapsed if collapsed else roof_cap_intact)
            roof_cap = 2 if roof_cap is None else max(0, int(roof_cap))
        n_roof = min(len(seats.get("roof") or []), roof_cap)
    return {"flame": n_flame, "smoke": n_smoke, "interior": n_int,
            "roof": n_roof, "openings": n_open,
            "total": n_flame + n_smoke + n_int + n_roof}


def _assert_parity(label, doc, masses, events, **caps):
    for max_emitters in (0, 1, 2, 4, 6, 9, 16):
        for smoke in (True, False):
            est = _emitter_estimate(doc, events, max_emitters, smoke, **caps)
            stage, root = _new_stage("/World/parity")
            res = fal.place_fire(stage, root, doc, masses, events, "t",
                                 random.Random(5), None, 0.0, 0.0,
                                 max_emitters=max_emitters, smoke=smoke,
                                 **caps)
            got = {k: res.get(k, 0)
                  for k in ("flame", "smoke", "interior", "roof", "openings")}
            exp = {k: est[k] for k in got}
            check("test_estimator_parity[{0},max_emitters={1},smoke={2},"
                  "caps={3}]".format(label, max_emitters, smoke, caps),
                  got == exp, "{0} vs {1}".format(exp, got))


def test_estimator_parity_flame_building():
    m = _fake_mass()
    events = _bug_shape_events(m) + [_fake_event(3, "N", 4, "out", 1, m)]
    doc = _fake_doc("F3", "flame", ["S", "N", "E"])
    _assert_parity("F3-flame", doc, {"main": m}, events)


def test_estimator_parity_f4_smoulder_building():
    m = _fake_mass()
    events = [_fake_event(0, "S", 4, "smoulder", 3, m),
             _fake_event(1, "N", 4, "smoulder", 2, m),
             _fake_event(2, "S", 3, "out", 4, m)]
    doc = _fake_doc("F4", "smoulder", ["S", "N"], n_storeys=14)  # tall too
    _assert_parity("F4-smoulder-tall", doc, {"main": m}, events)


def test_estimator_parity_f5_residual_building():
    m = _fake_mass()
    events = [_fake_event(0, "S", 4, "smoulder", 1, m),
             _fake_event(1, "N", 3, "smoulder", 1, m),
             _fake_event(2, "S", 2, "out", 5, m)]
    doc = _fake_doc("F5", "residual", ["S", "N"])
    _assert_parity("F5-residual", doc, {"main": m}, events)


def test_estimator_parity_f1_wisp_building():
    m = _fake_mass()
    events = [_fake_event(0, "S", 2, "smoulder", 1, m),
             _fake_event(1, "N", 2, "smoulder", 1, m)]
    doc = _fake_doc("F1", None, ["S", "N"], roof=False)
    _assert_parity("F1-wisp", doc, {"main": m}, events)


# a burnt-out (F4/F5) shape with abundant "out" events and few "smoulder"
# ones -- the SAME imbalance measured on the real `city_138` sidecars
# (28-114 "out" vs 1-3 "smoulder") that motivates the fill-in.
def _burnt_out_shape(m):
    return ([_fake_event(0, "S", 4, "smoulder", 1, m)]
           + [_fake_event(k, "SNEW"[k % 4], 3, "out", 1, m)
              for k in range(1, 15)])


_ROOF2_SEATS = {"interior": [{"x": 0.0, "y": 0.0, "z": 5.0, "radius": 1.2,
                              "scale": 1.0}],
               "roof": [{"x": -1.0, "y": 0.0, "z": 18.0, "radius": 1.8,
                         "scale": 1.25},
                        {"x": 1.0, "y": 0.0, "z": 18.0, "radius": 1.8,
                         "scale": 1.25}]}


def test_estimator_parity_side_smoke_and_roof_caps():
    m = _fake_mass()
    events = _burnt_out_shape(m)
    doc = _fake_doc("F5", "residual", ["S", "N"], seats=_ROOF2_SEATS)
    for caps in (
            {},
            {"side_smoke_flame_max": 5},
            {"side_smoke_nonflame_max": 6},
            {"roof_cap_intact": 1, "roof_cap_collapsed": 2},
            {"side_smoke_flame_max": 5, "side_smoke_nonflame_max": 6,
             "roof_cap_intact": 1, "roof_cap_collapsed": 2}):
        _assert_parity("F5-caps", doc, {"main": m}, events, **caps)


def test_estimator_parity_side_smoke_and_roof_caps_collapsed():
    m = _fake_mass()
    events = _burnt_out_shape(m)
    doc = _fake_doc("F5c", "residual", ["S", "N"], seats=_ROOF2_SEATS)
    _assert_parity("F5c-caps", doc, {"main": m}, events,
                   side_smoke_nonflame_max=6, roof_cap_intact=1,
                   roof_cap_collapsed=2)


# ---------------------------------------------------------------------------
# 7) _round_robin weighting (2026-08-31, street-facing bias)
# ---------------------------------------------------------------------------
def test_round_robin_weight_of_one_matches_unweighted():
    groups = [["a1", "a2", "a3"], ["b1", "b2"], ["c1", "c2", "c3", "c4"]]
    check("test_round_robin_weight_of_one_matches_unweighted",
          fal._round_robin(groups, weights=[1, 1, 1]) == fal._round_robin(groups))


def test_round_robin_weighted_front_loads_the_heavier_group():
    groups = [["a1", "a2", "a3"], ["b1", "b2"], ["c1", "c2", "c3", "c4"]]
    flat = fal._round_robin(groups, weights=[3, 1, 1])
    check("test_round_robin_weighted_preserves_membership",
          sorted(flat) == sorted(x for g in groups for x in g), str(flat))
    check("test_round_robin_weighted_front_loads_the_heavier_group",
          flat[:5] == ["a1", "a2", "a3", "b1", "c1"], str(flat))
    check("test_round_robin_weighted_preserves_within_group_order",
          [x for x in flat if x.startswith("c")] == groups[2], str(flat))


def test_round_robin_weight_floors_at_one():
    groups = [["a1", "a2"], ["b1", "b2"]]
    flat = fal._round_robin(groups, weights=[0, -5])
    check("test_round_robin_weight_floors_at_one",
          flat == ["a1", "b1", "a2", "b2"], str(flat))


# ---------------------------------------------------------------------------
# 8) _group_by_side / _side_weights / choose_street_side (2026-08-31, SECOND
#    review: "street facing fires all seem to be in the top left. I want it
#    randomized." `_group_by_side` reverted to first-seen order ONLY —
#    street bias now lives entirely in `choose_street_side` (a RANDOM,
#    per-building draw) + `_side_weights` (applies whichever ONE side was
#    already chosen); `_group_by_side` no longer takes a rank argument at
#    all.
# ---------------------------------------------------------------------------
def test_group_by_side_is_first_seen_order_only():
    m = _fake_mass()
    events = [_fake_event(0, "S", 5, "flame", 1, m),
             _fake_event(1, "E", 5, "flame", 1, m),
             _fake_event(2, "N", 5, "flame", 1, m)]
    order, _ = fal._group_by_side(events)
    check("test_group_by_side_is_first_seen_order_only",
          order == ["S", "E", "N"], str(order))


def test_side_weights_boosts_only_the_chosen_side():
    order_sides = ["S", "E", "N"]
    check("test_side_weights_boosts_only_the_chosen_side",
          fal._side_weights(order_sides, "E", 3) == [1, 3, 1])
    check("test_side_weights_none_when_side_falsy",
          fal._side_weights(order_sides, None, 3) is None and
          fal._side_weights(order_sides, "", 3) is None)
    check("test_side_weights_none_when_side_not_present",
          fal._side_weights(order_sides, "W", 3) is None)
    check("test_side_weights_floors_weight_at_one",
          fal._side_weights(order_sides, "S", 0) == [1, 1, 1])


def test_choose_street_side_favours_higher_score_but_can_pick_either():
    # a heavily lopsided score pattern should draw the high side FAR more
    # often across many independent seeds, but never be literally 100% —
    # this is a random draw, not an argmax.
    from collections import Counter
    hist = Counter()
    for seed in range(300):
        side = fal.choose_street_side({"N": 1.0, "E": 90.0}, "seed{0}".format(seed))
        hist[side] += 1
    check("test_choose_street_side_favours_the_higher_score",
          hist["E"] > hist["N"], str(hist))
    check("test_choose_street_side_still_sometimes_picks_the_lower_one",
          hist["N"] > 0, str(hist))


def test_choose_street_side_clamps_the_200m_sentinel():
    # two sides BOTH capped at the raw 200.0 "nothing nearby" sentinel must
    # draw close to 50/50 -- if the ceiling clamp were missing this would
    # still be 50/50 (both equal), so this mainly guards the ceiling exists
    # and does not crash / bias toward whichever sorts first.
    from collections import Counter
    hist = Counter()
    for seed in range(400):
        side = fal.choose_street_side({"N": 200.0, "S": 200.0},
                                      "seed{0}".format(seed))
        hist[side] += 1
    ratio = hist["N"] / float(hist["N"] + hist["S"])
    check("test_choose_street_side_clamps_the_200m_sentinel",
          0.35 < ratio < 0.65, str(hist))


def test_choose_street_side_spreads_across_a_district_of_identical_scores():
    # THE ACTUAL REPORTED BUG: many buildings sharing the SAME score
    # pattern (the 200 m sentinel on N/E/W, a real close neighbour on S)
    # must NOT all boost the same side once seeded per-building.
    from collections import Counter
    rank = {"N": 200.0, "E": 200.0, "S": 5.0, "W": 200.0}
    hist = Counter()
    for i in range(200, 260):
        side = fal.choose_street_side(
            rank, "7-stem{0}-street".format(i))
        hist[side] += 1
    check("test_choose_street_side_spreads_across_a_district_n",
          hist["N"] > 5, str(hist))
    check("test_choose_street_side_spreads_across_a_district_e",
          hist["E"] > 5, str(hist))
    check("test_choose_street_side_spreads_across_a_district_w",
          hist["W"] > 5, str(hist))
    check("test_choose_street_side_spreads_across_a_district_not_one_side",
          max(hist.values()) < 55, str(hist))


def test_choose_street_side_is_deterministic_per_seed():
    a = fal.choose_street_side({"N": 10.0, "E": 20.0}, "same-seed")
    b = fal.choose_street_side({"N": 10.0, "E": 20.0}, "same-seed")
    check("test_choose_street_side_is_deterministic_per_seed", a == b)


def test_choose_street_side_none_cases():
    check("test_choose_street_side_none_when_rank_falsy",
          fal.choose_street_side({}, "s") is None and
          fal.choose_street_side(None, "s") is None)
    check("test_choose_street_side_none_when_nothing_positive",
          fal.choose_street_side({"N": -1.0, "E": 0.0}, "s") is None)


def test_flame_selection_order_biases_toward_the_chosen_side_without_starving_others():
    m = _fake_mass()
    events = [_fake_event(0, "S", 5, "flame", 4, m),
             _fake_event(1, "E", 5, "flame", 4, m),
             _fake_event(2, "N", 5, "flame", 4, m)]
    unweighted = fal._flame_selection_order(events)[:6]
    weighted = fal._flame_selection_order(
        events, street_bias_side="E", bias_weight=2)[:6]
    u_sides = [ev["side"] for ev, _ in unweighted]
    w_sides = [ev["side"] for ev, _ in weighted]
    check("test_flame_selection_order_biases_toward_the_chosen_side",
          w_sides.count("E") > u_sides.count("E"),
          "{0} vs {1}".format(w_sides, u_sides))
    check("test_flame_selection_order_bias_never_starves_other_sides",
          set(w_sides) == {"S", "E", "N"}, str(w_sides))
    total_unweighted = fal._flame_selection_order(events)
    total_weighted = fal._flame_selection_order(
        events, street_bias_side="E", bias_weight=2)
    check("test_flame_selection_order_bias_does_not_change_the_total",
          len(total_unweighted) == len(total_weighted) == 12,
          "{0} vs {1}".format(len(total_unweighted), len(total_weighted)))


# ---------------------------------------------------------------------------
# 9) load_dump_positions / street_side_ranks (the city-dump-derived bias)
# ---------------------------------------------------------------------------
def _write_dump(placements):
    doc = {"schema": "fire_city_placements_dump.v1", "preset": "t", "seed": 1,
          "region_m": [500.0, 500.0], "n_placements_total": len(placements),
          "placements": placements, "typology": {"blocks": []}}
    fh = tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False)
    json.dump(doc, fh)
    fh.close()
    return fh.name


def test_load_dump_positions_round_trips_a_real_schema():
    path = _write_dump([
        {"i": 5, "x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0, "W": 20.0, "D": 10.0},
        {"i": 9, "x_m": 30.0, "y_m": 0.0, "yaw_deg": 90.0, "W": 8.0, "D": 8.0}])
    try:
        pos = fal.load_dump_positions(path)
    finally:
        os.unlink(path)
    # "H" (2026-08-31, added alongside the review-camera occlusion work):
    # `load_dump_positions` now also carries building height, defaulting to
    # 0.0 when the dump record has none.
    check("test_load_dump_positions_round_trips_a_real_schema",
          pos == {5: {"x": 0.0, "y": 0.0, "W": 20.0, "D": 10.0, "H": 0.0,
                     "yaw": 0.0},
                 9: {"x": 30.0, "y": 0.0, "W": 8.0, "D": 8.0, "H": 0.0,
                    "yaw": 90.0}},
          str(pos))


def test_load_dump_positions_missing_file_is_empty_not_a_crash():
    check("test_load_dump_positions_missing_file_is_empty_not_a_crash",
          fal.load_dump_positions("/no/such/file.json") == {})


def test_load_dump_positions_bad_json_is_empty_not_a_crash():
    fh = tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False)
    fh.write("{not valid json")
    fh.close()
    try:
        check("test_load_dump_positions_bad_json_is_empty_not_a_crash",
              fal.load_dump_positions(fh.name) == {})
    finally:
        os.unlink(fh.name)


def test_street_side_ranks_favours_the_open_side_over_the_boxed_in_one():
    # building 0 at the origin, 20x10; a close neighbour 5 m off its EAST
    # wall, nothing anywhere near its N/S/W.
    path = _write_dump([
        {"i": 0, "x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0, "W": 20.0, "D": 10.0},
        {"i": 1, "x_m": 15.0, "y_m": 0.0, "yaw_deg": 0.0, "W": 20.0, "D": 10.0}])
    try:
        pos = fal.load_dump_positions(path)
    finally:
        os.unlink(path)
    ranks = fal.street_side_ranks(pos, 0, ["N", "E", "S", "W"])
    check("test_street_side_ranks_favours_the_open_side_over_the_boxed_in_one",
          ranks["E"] < ranks["N"] and ranks["E"] < ranks["W"]
          and ranks["E"] < ranks["S"], str(ranks))
    check("test_street_side_ranks_unknown_building_is_empty",
          fal.street_side_ranks(pos, 999, ["N"]) == {})
    check("test_street_side_ranks_no_sides_is_empty",
          fal.street_side_ranks(pos, 0, []) == {})
    check("test_street_side_ranks_no_positions_is_empty",
          fal.street_side_ranks({}, 0, ["N"]) == {})


# ---------------------------------------------------------------------------
# 10) roof_has_collapsed
# ---------------------------------------------------------------------------
def test_roof_has_collapsed_level_check():
    check("test_roof_has_collapsed_f5c",
          fal.roof_has_collapsed({"fire": {"level": "F5c"}}) is True)
    check("test_roof_has_collapsed_f6",
          fal.roof_has_collapsed({"fire": {"level": "F6"}}) is True)
    check("test_roof_has_collapsed_f5_not_by_level_alone",
          fal.roof_has_collapsed({"fire": {"level": "F5"}}) is False)
    check("test_roof_has_collapsed_f4_not_by_level_alone",
          fal.roof_has_collapsed({"fire": {"level": "F4"}}) is False)


def test_roof_has_collapsed_deck_gap_from_fire_dict():
    intact = {"fire": {"level": "F5", "deck_z": 40.0}, "top_z": 50.0}
    collapsed = {"fire": {"level": "F5", "deck_z": 40.0}, "top_z": 40.5}
    check("test_roof_has_collapsed_deck_gap_intact",
          fal.roof_has_collapsed(intact) is False)
    check("test_roof_has_collapsed_deck_gap_collapsed",
          fal.roof_has_collapsed(collapsed) is True)


def test_roof_has_collapsed_deck_gap_from_masses_fallback():
    # a GAC bake's `deck_z` on `fire` is what `place_fire` always has (moved
    # there by `fire_bake.place`), but `roof_has_collapsed` also has to work
    # straight off a raw sidecar doc, where it only lives under `masses`.
    doc = {"fire": {"level": "F5", "mass": "main"},
          "masses": {"main": {"deck_z": 40.0}}, "top_z": 40.9}
    check("test_roof_has_collapsed_deck_gap_from_masses_fallback",
          fal.roof_has_collapsed(doc) is True)


def test_roof_has_collapsed_no_deck_data_is_false_not_a_crash():
    # every kit mass — `fire_bake.mass_to_json`'s own "None for a kit mass"
    check("test_roof_has_collapsed_no_deck_data_is_false_not_a_crash",
          fal.roof_has_collapsed({"fire": {"level": "F5"}}) is False)


# ---------------------------------------------------------------------------
# 11) place_fire's new caps, end to end
# ---------------------------------------------------------------------------
def test_place_fire_side_smoke_unchanged_when_not_customised():
    """The row/bench launcher never passes `side_smoke_nonflame_max` — it
    must see the exact OLD smoulder-only budget, never the "out"-event
    fill-in, even though there are 14 "out" events sitting right there."""
    m = _fake_mass()
    masses = {"main": m}
    events = _burnt_out_shape(m)
    doc = _fake_doc("F5", "residual", ["S", "N"], seats=_ROOF2_SEATS)
    stage, root = _new_stage("/World/nocust")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(6), None, 0.0, 0.0, max_emitters=6,
                         smoke=True)
    check("test_place_fire_side_smoke_unchanged_when_not_customised",
          res["smoke"] == 1, "expected exactly the 1 live smoulder event, "
          "got {0}".format(res["smoke"]))
    check("test_place_fire_roof_unchanged_when_not_customised",
          res["roof"] == 2, "expected the old fixed roof count of 2, got "
          "{0}".format(res["roof"]))


def test_place_fire_side_smoke_fills_from_out_events_when_customised():
    m = _fake_mass()
    masses = {"main": m}
    events = _burnt_out_shape(m)
    doc = _fake_doc("F5", "residual", ["S", "N"], seats=_ROOF2_SEATS)
    stage, root = _new_stage("/World/cust")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(6), None, 0.0, 0.0, max_emitters=6,
                         smoke=True, side_smoke_nonflame_max=6)
    check("test_place_fire_side_smoke_fills_from_out_events_when_customised",
          res["smoke"] == 6, "expected the 1 smoulder + 5 out-event fill, "
          "got {0}".format(res["smoke"]))


def test_place_fire_side_smoke_never_double_counts_an_event():
    """The 1 smoulder event and the 14 out events all have DISTINCT ids
    (`_burnt_out_shape`'s own construction) — nothing in the fill-in may
    ever author the same event twice."""
    m = _fake_mass()
    masses = {"main": m}
    events = _burnt_out_shape(m)
    doc = _fake_doc("F5", "residual", ["S", "N"], seats=_ROOF2_SEATS)
    stage, root = _new_stage("/World/dup")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(6), None, 0.0, 0.0, max_emitters=6,
                         smoke=True, side_smoke_nonflame_max=20)
    # only 15 events exist in total (1 smoulder + 14 out); the fill-in must
    # not manufacture more sources than there are events to source them from
    check("test_place_fire_side_smoke_never_double_counts_an_event",
          res["smoke"] == 15, "got {0}, expected exactly 15 (1+14, capped "
          "by how many events actually exist)".format(res["smoke"]))


def test_place_fire_roof_cap_trades_roof_for_side_on_an_intact_roof():
    m = _fake_mass()
    masses = {"main": m}
    events = _burnt_out_shape(m)
    doc = _fake_doc("F5", "residual", ["S", "N"], seats=_ROOF2_SEATS)
    stage, root = _new_stage("/World/roofintact")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(6), None, 0.0, 0.0, max_emitters=6,
                         smoke=True, side_smoke_nonflame_max=6,
                         roof_cap_intact=1, roof_cap_collapsed=2)
    check("test_place_fire_roof_cap_trades_roof_for_side_on_an_intact_roof",
          res["roof"] == 1, "expected roof capped to 1, got {0}"
          .format(res["roof"]))
    check("test_place_fire_side_exceeds_roof_on_an_intact_roof",
          res["smoke"] > res["roof"],
          "smoke={0} roof={1}".format(res["smoke"], res["roof"]))
    check("test_place_fire_roof_collapsed_flag_is_false",
          res["roof_collapsed"] is False)


def test_place_fire_roof_cap_stays_full_once_the_roof_has_collapsed():
    m = _fake_mass()
    masses = {"main": m}
    events = _burnt_out_shape(m)
    doc = _fake_doc("F5c", "residual", ["S", "N"], seats=_ROOF2_SEATS)
    stage, root = _new_stage("/World/roofcollapsed")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(6), None, 0.0, 0.0, max_emitters=6,
                         smoke=True, side_smoke_nonflame_max=6,
                         roof_cap_intact=1, roof_cap_collapsed=2)
    check("test_place_fire_roof_cap_stays_full_once_the_roof_has_collapsed",
          res["roof"] == 2, "expected the uncapped roof count of 2, got "
          "{0}".format(res["roof"]))
    check("test_place_fire_roof_collapsed_flag_is_true",
          res["roof_collapsed"] is True)


# ---------------------------------------------------------------------------
# 12) SIZE-SCALED ALLOCATION (2026-08-31, third review, item 4)
# ---------------------------------------------------------------------------
def test_flame_window_target_and_smoke_window_target_formulas():
    check("test_flame_window_target_short_building",
          fal.flame_window_target(3) == 6, fal.flame_window_target(3))
    check("test_flame_window_target_min_floor",
          fal.flame_window_target(0) == fal.FLAME_WINDOWS_MIN)
    check("test_flame_window_target_max_ceiling",
          fal.flame_window_target(100) == fal.FLAME_WINDOWS_MAX)
    check("test_flame_window_target_monotonic",
          fal.flame_window_target(4) <= fal.flame_window_target(10)
          <= fal.flame_window_target(25))
    check("test_smoke_window_target_min_floor",
          fal.smoke_window_target(0) == fal.SMOKE_WINDOWS_MIN)
    check("test_smoke_window_target_max_ceiling",
          fal.smoke_window_target(100) == fal.SMOKE_WINDOWS_MAX)


def _many_flame_events(m, side, n_events, ops_each=1):
    return [_fake_event(k, side, 5 + k, "flame", ops_each, m)
           for k in range(n_events)]


def test_place_fire_flame_size_scaling_off_by_default():
    m = _fake_mass()
    masses = {"main": m}
    events = _many_flame_events(m, "S", 20)
    doc = _fake_doc("F3", "flame", ["S"], n_storeys=3)
    stage, root = _new_stage("/World/sizeoff")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(7), None, 0.0, 0.0, max_emitters=10,
                         smoke=False)
    # OLD formula: n_st=3 < 12, so max_open == max_emitters == 10, NOT the
    # size-scaled target (6) -- proves the row/bench launcher (which never
    # passes flame_size_scaling) is unaffected.
    check("test_place_fire_flame_size_scaling_off_by_default",
          res["openings"] == 10, str(res))


def test_place_fire_flame_size_scaling_taller_building_gets_more_flame():
    m = _fake_mass()
    masses = {"main": m}
    short_events = _many_flame_events(m, "S", 20)
    tall_events = _many_flame_events(m, "S", 30)
    short_doc = _fake_doc("F3", "flame", ["S"], n_storeys=3)
    tall_doc = _fake_doc("F3", "flame", ["S"], n_storeys=25)
    s1, r1 = _new_stage("/World/short")
    short = fal.place_fire(s1, r1, short_doc, masses, short_events, "t",
                           random.Random(7), None, 0.0, 0.0, max_emitters=30,
                           smoke=False, flame_size_scaling=True)
    s2, r2 = _new_stage("/World/tall")
    tall = fal.place_fire(s2, r2, tall_doc, masses, tall_events, "t",
                          random.Random(7), None, 0.0, 0.0, max_emitters=30,
                          smoke=False, flame_size_scaling=True)
    check("test_place_fire_flame_size_scaling_short_matches_formula",
          short["openings"] == fal.flame_window_target(3), str(short))
    check("test_place_fire_flame_size_scaling_tall_matches_formula",
          tall["openings"] == fal.flame_window_target(25), str(tall))
    check("test_place_fire_flame_size_scaling_taller_building_gets_more_flame",
          tall["flame"] > short["flame"],
          "{0} vs {1}".format(tall["flame"], short["flame"]))


def test_place_fire_flame_size_scaling_plateaus_at_the_target_not_max_emitters():
    # a caller granting a HUGE `max_emitters` step must not blow past a
    # short building's own small target -- `min`, not `max`.
    m = _fake_mass()
    masses = {"main": m}
    events = _many_flame_events(m, "S", 30)
    doc = _fake_doc("F3", "flame", ["S"], n_storeys=3)
    stage, root = _new_stage("/World/plateau")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(7), None, 0.0, 0.0, max_emitters=30,
                         smoke=False, flame_size_scaling=True)
    check("test_place_fire_flame_size_scaling_plateaus_at_the_target",
          res["openings"] == fal.flame_window_target(3), str(res))


# ---------------------------------------------------------------------------
# 13) MULTIPLE FLAME CLUSTERS (2026-08-31, second review, item 2)
# ---------------------------------------------------------------------------
def test_scattered_selection_order_never_repeats_an_event():
    m = _fake_mass()
    events = ([_fake_event(k, "S", k, "out", 5, m) for k in range(6)]
             + [_fake_event(100 + k, "E", k, "out", 3, m) for k in range(4)])
    picked = fal._scattered_selection_order(events)
    ids = [ev["id"] for ev, _op in picked]
    check("test_scattered_selection_order_never_repeats_an_event",
          len(ids) == len(set(ids)) == len(events),
          "{0} picks for {1} events".format(len(ids), len(events)))
    sides = {ev["side"] for ev, _op in picked[:2]}
    check("test_scattered_selection_order_spreads_across_sides_early",
          len(sides) >= 1)


def test_place_fire_cluster_diversity_extends_beyond_one_band():
    # ONLY one (side, storey) group is ever "flame" -- a climbing fire's
    # real shape -- but plenty of "out" events exist elsewhere on the
    # building for the top-up to draw from.
    # the primary flame event has FEWER openings (2) than `max_open` (6, the
    # old >=12-storey height floor at max_emitters=3) so there is genuine
    # budget HEADROOM left for the extra-cluster top-up to spend — a primary
    # event big enough to fill the whole budget on its own leaves nothing
    # for a second cluster no matter how the selection is ordered, which
    # would not be testing this mechanism at all.
    m = _fake_mass()
    masses = {"main": m}
    flame_events = [_fake_event(0, "S", 11, "flame", 2, m)]
    out_events = ([_fake_event(1 + k, "S", k, "out", 2, m) for k in range(9)]
                 + [_fake_event(50 + k, "N", k, "out", 2, m) for k in range(9)])
    events = flame_events + out_events
    doc = _fake_doc("F3", "flame", ["S", "N"], n_storeys=12)

    s1, r1 = _new_stage("/World/noclusters")
    off = fal.place_fire(s1, r1, doc, masses, events, "t", random.Random(8),
                         None, 0.0, 0.0, max_emitters=3, smoke=False)
    s2, r2 = _new_stage("/World/clusters")
    on = fal.place_fire(s2, r2, doc, masses, events, "t", random.Random(8),
                        None, 0.0, 0.0, max_emitters=3, smoke=False,
                        flame_min_clusters=3, flame_extra_max=3)
    check("test_place_fire_cluster_diversity_off_stays_one_sided",
          off["flame"] > 0, str(off))
    check("test_place_fire_cluster_diversity_extends_beyond_one_band",
          on["flame"] > off["flame"],
          "{0} vs {1}".format(on["flame"], off["flame"]))


# ---------------------------------------------------------------------------
# 14) RESIDUAL FLAME POCKETS (2026-08-31, third review, item 5 — the
#     headline case) + "even in residual state i need smoke from windows"
# ---------------------------------------------------------------------------
def _residual_tower_shape(m, n_out=72, n_sm=3):
    events = []
    eid = 0
    sides = ("S", "W")
    storey = 0
    while len(events) < n_out:
        side = sides[len(events) % 2]
        events.append(_fake_event(eid, side, storey, "out", 1, m))
        eid += 1
        storey += 1
    for k in range(n_sm):
        events.append(_fake_event(eid, sides[k % 2], 26 + k, "smoulder", 1, m))
        eid += 1
    return events


def test_place_fire_residual_flame_frac_zero_matches_old_behavior():
    """`FA_RESIDUAL_FLAME_FRAC=0` ("0 restores today's behavior") — an F5
    "residual" building must get exactly ZERO flame, and an F4 "smoulder"
    one must use the ORIGINAL top-up, not the new scattered mechanism."""
    m = _fake_mass()
    masses = {"main": m}
    events = _residual_tower_shape(m)
    doc = _fake_doc("F5", "residual", ["S", "W"], n_storeys=28)
    stage, root = _new_stage("/World/frac0")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(9), None, 0.0, 0.0, max_emitters=10,
                         smoke=False, residual_flame_frac=0.0)
    check("test_place_fire_residual_flame_frac_zero_gives_no_flame",
          res["flame"] == 0, str(res))


def test_place_fire_residual_flame_pockets_appear_and_are_scattered():
    m = _fake_mass()
    masses = {"main": m}
    events = _residual_tower_shape(m)
    doc = _fake_doc("F5", "residual", ["S", "W"], n_storeys=28)
    stage, root = _new_stage("/World/residualtower")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(9), None, 0.0, 0.0, max_emitters=10,
                         smoke=False, flame_size_scaling=True,
                         residual_flame_frac=0.4)
    # ACCEPTANCE: SM_Building_23-shaped tower (28 storeys, 72 out + 3
    # smoulder) must land "roughly 10-20 flame windows" per the coordinator.
    check("test_place_fire_residual_flame_pockets_appear_in_acceptance_range",
          10 <= res["flame"] <= 20, str(res))
    check("test_place_fire_residual_flame_pockets_openings_match_flame_count",
          res["openings"] == res["flame"], str(res))


def test_place_fire_residual_flame_prefers_smoulder_then_out():
    m = _fake_mass()
    masses = {"main": m}
    events = _residual_tower_shape(m, n_out=72, n_sm=3)
    doc = _fake_doc("F5", "residual", ["S", "W"], n_storeys=28)
    stage, root = _new_stage("/World/prefer")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(9), None, 0.0, 0.0, max_emitters=10,
                         smoke=False, flame_size_scaling=True,
                         residual_flame_frac=0.4)
    target = max(fal.RESIDUAL_FLAME_MIN,
                 int(round(0.4 * fal.flame_window_target(28))))
    check("test_place_fire_residual_flame_matches_target_formula",
          res["flame"] == min(target, 72 + 3), str(res))


def test_place_fire_residual_flame_scales_with_building_size():
    m = _fake_mass()
    masses = {"main": m}
    short_events = _residual_tower_shape(m, n_out=40, n_sm=2)
    tall_events = _residual_tower_shape(m, n_out=72, n_sm=3)
    short_doc = _fake_doc("F5", "residual", ["S", "W"], n_storeys=4)
    tall_doc = _fake_doc("F5", "residual", ["S", "W"], n_storeys=28)
    s1, r1 = _new_stage("/World/shortresidual")
    short = fal.place_fire(s1, r1, short_doc, masses, short_events, "t",
                           random.Random(9), None, 0.0, 0.0, max_emitters=10,
                           smoke=False, flame_size_scaling=True,
                           residual_flame_frac=0.4)
    s2, r2 = _new_stage("/World/tallresidual")
    tall = fal.place_fire(s2, r2, tall_doc, masses, tall_events, "t",
                          random.Random(9), None, 0.0, 0.0, max_emitters=10,
                          smoke=False, flame_size_scaling=True,
                          residual_flame_frac=0.4)
    check("test_place_fire_residual_flame_scales_with_building_size",
          tall["flame"] > short["flame"],
          "{0} vs {1}".format(tall["flame"], short["flame"]))


def test_place_fire_f4_smoulder_gets_residual_pockets_when_frac_on():
    m = _fake_mass()
    masses = {"main": m}
    events = _residual_tower_shape(m, n_out=30, n_sm=2)
    doc = _fake_doc("F4", "smoulder", ["S", "W"], n_storeys=10)
    stage, root = _new_stage("/World/f4residual")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(9), None, 0.0, 0.0, max_emitters=10,
                         smoke=False, flame_size_scaling=True,
                         residual_flame_frac=0.4)
    check("test_place_fire_f4_smoulder_gets_residual_pockets_when_frac_on",
          res["flame"] > 0, str(res))


# ---------------------------------------------------------------------------
# 15) WINDOW-JET SMOKE GEOMETRY (2026-08-31, second review, item 3) — and
#     "even in residual state i need smoke from windows if not fires"
#     (third review, follow-up)
# ---------------------------------------------------------------------------
def test_place_fire_smoke_window_jets_takes_several_openings_from_one_event():
    m = _fake_mass()
    masses = {"main": m}
    big_event = _fake_event(0, "S", 4, "out", 6, m)  # one event, 6 openings
    doc = _fake_doc("F3", "flame", ["S"], n_storeys=3)
    s1, r1 = _new_stage("/World/eventlevel")
    old = fal.place_fire(s1, r1, doc, masses, [big_event], "t",
                         random.Random(10), None, 0.0, 0.0, max_emitters=1,
                         smoke=True, side_smoke_flame_max=5)
    s2, r2 = _new_stage("/World/openinglevel")
    new = fal.place_fire(s2, r2, doc, masses, [big_event], "t",
                         random.Random(10), None, 0.0, 0.0, max_emitters=1,
                         smoke=True, side_smoke_flame_max=5,
                         smoke_window_jets=True)
    check("test_place_fire_smoke_window_jets_event_level_gives_exactly_one",
          old["smoke"] == 1, str(old))
    check("test_place_fire_smoke_window_jets_opening_level_gives_more",
          new["smoke"] > old["smoke"],
          "{0} vs {1}".format(new["smoke"], old["smoke"]))
    check("test_place_fire_smoke_window_jets_capped_at_the_budget",
          new["smoke"] == min(5, 6), str(new))


def test_place_fire_residual_tower_shows_both_flame_and_window_smoke():
    """THE FULL-STACK ACCEPTANCE CHECK: SM_Building_23-shaped (28-storey
    residual, 72 out + 3 smoulder events) must show BOTH flame pockets
    (item 5) AND window-jet side smoke (item 3 / "even in residual state i
    need smoke from windows if not fires") with every knob the city
    launcher actually turns on."""
    m = _fake_mass()
    masses = {"main": m}
    events = _residual_tower_shape(m, n_out=72, n_sm=3)
    doc = _fake_doc("F5", "residual", ["S", "W"], n_storeys=28,
                    seats=_ROOF2_SEATS)
    stage, root = _new_stage("/World/fullstack")
    res = fal.place_fire(stage, root, doc, masses, events, "t",
                         random.Random(9), None, 0.0, 0.0, max_emitters=10,
                         smoke=True, smoke_scale=1.8,
                         side_smoke_flame_max=5, side_smoke_nonflame_max=6,
                         roof_cap_intact=1, roof_cap_collapsed=2,
                         flame_min_clusters=3, flame_extra_max=3,
                         flame_size_scaling=True, smoke_size_scaling=True,
                         smoke_window_jets=True, residual_flame_frac=0.4)
    check("test_place_fire_residual_tower_has_flame",
          res["flame"] > 0, str(res))
    check("test_place_fire_residual_tower_has_window_smoke",
          res["smoke"] > 0, str(res))
    check("test_place_fire_residual_tower_flame_in_acceptance_range",
          10 <= res["flame"] <= 20, str(res))
    check("test_place_fire_residual_tower_smoke_matches_size_target",
          res["smoke"] == fal.smoke_window_target(28), str(res))


def test_estimator_parity_size_scaling_and_residual_flame():
    m = _fake_mass()
    events = _residual_tower_shape(m, n_out=72, n_sm=3)
    doc = _fake_doc("F5", "residual", ["S", "W"], n_storeys=28,
                    seats=_ROOF2_SEATS)
    for caps in (
            {"flame_size_scaling": True},
            {"smoke_size_scaling": True},
            {"smoke_window_jets": True},
            {"residual_flame_frac": 0.4},
            {"flame_size_scaling": True, "smoke_size_scaling": True,
             "smoke_window_jets": True, "residual_flame_frac": 0.4,
             "side_smoke_flame_max": 5, "side_smoke_nonflame_max": 6,
             "roof_cap_intact": 1, "roof_cap_collapsed": 2}):
        _assert_parity("F5-tower-fullstack", doc, {"main": m}, events, **caps)


def test_estimator_parity_cluster_diversity_and_flame_state():
    m = _fake_mass()
    flame_events = [_fake_event(0, "S", 11, "flame", 6, m)]
    out_events = ([_fake_event(1 + k, "S", k, "out", 2, m) for k in range(9)]
                 + [_fake_event(50 + k, "N", k, "out", 2, m) for k in range(9)])
    events = flame_events + out_events
    doc = _fake_doc("F3", "flame", ["S", "N"], n_storeys=12)
    for caps in ({"flame_min_clusters": 3, "flame_extra_max": 3},
                {"flame_min_clusters": 3, "flame_extra_max": 3,
                 "flame_size_scaling": True}):
        _assert_parity("F3-cluster-diversity", doc, {"main": m}, events, **caps)


# ---------------------------------------------------------------------------
# 13) CONTACT SNAP — "some of the fires seem to be floating outside the
#     building... make it go inside the building by some amount" (user,
#     `gac_SM_Building_11_F4_o18_SNW_s374`). Needs `vtk` — SKIPS (not a
#     failure) when it is not importable, same graceful-degrade
#     `fire_bake._judge_candidates` uses.
# ---------------------------------------------------------------------------
def _tall_mass():
    """A 16-storey mass, `max_synthetic_storey(16) == 13` — the same shape
    the top-storey-exclusion tests below need."""
    levels = [i * 4.0 for i in range(16)]
    return {"tag": "main", "cx": 0.0, "cy": 0.0, "yaw": 0.0, "W": 20.0,
            "D": 14.0, "z0": 0.0, "top": 64.0, "levels": levels,
            "module": 4.0, "spec": {"bands": []}}


def _quad_mesh(stage, path, y, x_half=10.0, z0=-5.0, z1=20.0):
    """A flat quad in the world XZ plane at world `y` — a "wall" a contact
    ray travelling along +/-Y can cross."""
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr([Gf.Vec3f(-x_half, y, z0), Gf.Vec3f(x_half, y, z0),
                          Gf.Vec3f(x_half, y, z1), Gf.Vec3f(-x_half, y, z1)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    return mesh


def test_contact_offset_positive_when_real_wall_is_further_out():
    if not HAVE_VTK:
        print("skip  test_contact_offset_positive_when_real_wall_is_further_out"
              " (vtk not importable)")
        return
    m = _fake_mass()
    op = _fake_op("S", 1, m)
    (px, py, pz), (ox, oy, oz) = fal._opening_test_ray(op)
    stage, root = _new_stage("/World/cs_pos")
    _quad_mesh(stage, root + "/wall", py + oy * 1.0)
    loc = fal.build_contact_locator(stage, root)
    t = fal.contact_offset(loc, op)
    check("test_contact_offset_positive_when_real_wall_is_further_out",
          t is not None and abs(t - 1.0) < 1e-3, str(t))


def test_contact_offset_negative_when_real_wall_is_further_in():
    if not HAVE_VTK:
        print("skip  test_contact_offset_negative_when_real_wall_is_further_in"
              " (vtk not importable)")
        return
    m = _fake_mass()
    op = _fake_op("S", 1, m)
    (px, py, pz), (ox, oy, oz) = fal._opening_test_ray(op)
    stage, root = _new_stage("/World/cs_neg")
    _quad_mesh(stage, root + "/wall", py - oy * 1.5)
    loc = fal.build_contact_locator(stage, root)
    t = fal.contact_offset(loc, op)
    check("test_contact_offset_negative_when_real_wall_is_further_in",
          t is not None and abs(t - (-1.5)) < 1e-3, str(t))


def test_contact_offset_none_when_nothing_within_reach():
    if not HAVE_VTK:
        print("skip  test_contact_offset_none_when_nothing_within_reach"
              " (vtk not importable)")
        return
    m = _fake_mass()
    op = _fake_op("S", 1, m)
    (px, py, pz), (ox, oy, oz) = fal._opening_test_ray(op)
    stage, root = _new_stage("/World/cs_none")
    # a "wall" 50 m away -- well past the default search reach
    _quad_mesh(stage, root + "/wall", py + oy * 50.0)
    loc = fal.build_contact_locator(stage, root)
    t = fal.contact_offset(loc, op)
    check("test_contact_offset_none_when_nothing_within_reach", t is None, str(t))


def test_build_contact_locator_none_without_geometry():
    if not HAVE_VTK:
        print("skip  test_build_contact_locator_none_without_geometry"
              " (vtk not importable)")
        return
    stage, root = _new_stage("/World/cs_empty")
    loc = fal.build_contact_locator(stage, root)
    check("test_build_contact_locator_none_without_geometry", loc is None)
    loc2 = fal.build_contact_locator(stage, root + "/does_not_exist")
    check("test_build_contact_locator_none_without_geometry_bad_path",
          loc2 is None)


def test_snap_events_to_geometry_leaves_touching_opening_unchanged():
    if not HAVE_VTK:
        print("skip  test_snap_events_to_geometry_leaves_touching_opening"
              "_unchanged (vtk not importable)")
        return
    m = _fake_mass()
    op = _fake_op("S", 1, m)
    (px, py, pz), (ox, oy, oz) = fal._opening_test_ray(op)
    ev = {"id": 0, "side": "S", "storey": 1, "state": "flame", "ops": [op]}
    stage, root = _new_stage("/World/cs_touch")
    _quad_mesh(stage, root + "/wall", py + oy * 0.02)   # 2 cm -- within tol
    out, stats = fal.snap_events_to_geometry(stage, root, [ev])
    check("test_snap_events_to_geometry_leaves_touching_opening_unchanged",
          stats["ok"] == 1 and stats["snapped"] == 0 and stats["dropped"] == 0
          and out[0]["ops"][0]["out"] == op["out"], str(stats))


def test_snap_events_to_geometry_snaps_offset_opening():
    if not HAVE_VTK:
        print("skip  test_snap_events_to_geometry_snaps_offset_opening"
              " (vtk not importable)")
        return
    m = _fake_mass()
    op = _fake_op("S", 1, m)
    (px, py, pz), (ox, oy, oz) = fal._opening_test_ray(op)
    ev = {"id": 0, "side": "S", "storey": 1, "state": "flame", "ops": [op]}
    stage, root = _new_stage("/World/cs_snap")
    _quad_mesh(stage, root + "/wall", py + oy * 2.0)    # 2 m -- past tol
    out, stats = fal.snap_events_to_geometry(
        stage, root, [ev], tol_m=0.3, inset_m=0.2)
    got_out = out[0]["ops"][0]["out"]
    check("test_snap_events_to_geometry_snaps_offset_opening",
          stats["snapped"] == 1 and stats["ok"] == 0
          and abs(got_out - (2.0 - 0.2)) < 1e-3,
          "stats={0} out={1}".format(stats, got_out))
    # non-mutating: the ORIGINAL op dict (and event) must be untouched
    check("test_snap_events_to_geometry_snaps_offset_opening_non_mutating",
          op["out"] == -0.3 and out[0] is not ev, str(op))


def test_snap_events_to_geometry_drops_phantom_opening():
    if not HAVE_VTK:
        print("skip  test_snap_events_to_geometry_drops_phantom_opening"
              " (vtk not importable)")
        return
    m = _fake_mass()
    op = _fake_op("S", 1, m)
    (px, py, pz), (ox, oy, oz) = fal._opening_test_ray(op)
    ev = {"id": 0, "side": "S", "storey": 1, "state": "flame", "ops": [op]}
    stage, root = _new_stage("/World/cs_phantom")
    _quad_mesh(stage, root + "/wall", py + oy * 50.0)
    out, stats = fal.snap_events_to_geometry(stage, root, [ev])
    check("test_snap_events_to_geometry_drops_phantom_opening",
          stats["dropped"] == 1 and len(out[0]["ops"]) == 0, str(stats))


def test_snap_events_to_geometry_noop_without_geom_root():
    events = [_fake_event(0, "S", 1, "flame", 2, _fake_mass())]
    out, stats = fal.snap_events_to_geometry(None, "", events)
    check("test_snap_events_to_geometry_noop_without_geom_root",
          out is events and stats["locator"] is False and stats["tested"] == 0)


def test_bake_geometry_root_with_holder():
    doc = {"root": "/World/bake", "default_prim": "/World"}
    check("test_bake_geometry_root_with_holder",
          fal.bake_geometry_root("/World/fire/stem", doc)
          == "/World/fire/stem/bake/bake")


def test_bake_geometry_root_without_holder():
    doc = {"root": "/World/bake", "default_prim": "/World"}
    check("test_bake_geometry_root_without_holder",
          fal.bake_geometry_root("", doc) == "/World/bake")


def test_place_fire_geom_root_none_reports_zeroed_snap_stats():
    m = _fake_mass()
    events = [_fake_event(0, "S", 1, "flame", 2, m)]
    doc = _fake_doc("F2", "flame", ["S"], origin=1, top=1, n_storeys=6)
    stage, root = _new_stage("/World/cs_offdefault")
    res = fal.place_fire(stage, root, doc, {"main": m}, events, "t",
                         random.Random(1), None, 0.0, 0.0, max_emitters=9)
    check("test_place_fire_geom_root_none_reports_zeroed_snap_stats",
          res["snap"] == {"tested": 0, "ok": 0, "snapped": 0, "dropped": 0,
                          "worst_offset_m": 0.0, "locator": False},
          str(res["snap"]))


def test_place_fire_geom_root_moves_the_emitter_toward_real_wall():
    if not HAVE_VTK:
        print("skip  test_place_fire_geom_root_moves_the_emitter_toward_real"
              "_wall (vtk not importable)")
        return
    m = _fake_mass()
    ev = _fake_event(0, "S", 1, "flame", 1, m)
    op = ev["ops"][0]
    doc = _fake_doc("F2", "flame", ["S"], origin=1, top=1, n_storeys=6)

    def _box_pos(stage, root):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(Sdf.Path(root))):
            if prim.GetTypeName() == "FlowEmitterBox":
                a = prim.GetAttribute("position")
                return a.Get() if a and a.IsValid() else None
        return None

    s1, r1 = _new_stage("/World/cs_before")
    res1 = fal.place_fire(s1, r1, doc, {"main": m}, [ev], "t",
                          random.Random(1), None, 0.0, 0.0, max_emitters=9)
    p1 = _box_pos(s1, r1)

    s2, r2 = _new_stage("/World/cs_after")
    geom_root = r2 + "/geom"
    UsdGeom.Xform.Define(s2, Sdf.Path(geom_root))
    (px, py, pz), (ox, oy, oz) = fal._opening_test_ray(op)
    _quad_mesh(s2, geom_root + "/wall", py + oy * 1.0)
    res2 = fal.place_fire(s2, r2, doc, {"main": m}, [ev], "t",
                          random.Random(1), None, 0.0, 0.0, max_emitters=9,
                          geom_root=geom_root, snap_tol_m=0.05,
                          snap_inset_m=0.2)
    p2 = _box_pos(s2, r2)
    moved = None if (p1 is None or p2 is None) else abs(p2[1] - p1[1])
    check("test_place_fire_geom_root_moves_the_emitter_toward_real_wall",
          res2["snap"]["snapped"] == 1 and moved is not None and moved > 0.9,
          "res1={0} res2={1} moved={2}".format(res1, res2, moved))


# ---------------------------------------------------------------------------
# 14) NO FIRE AT THE EXTREME TOP UNLESS THE WINDOWS ARE REAL — "avoid fires
#     at the extreme top of buildings... unless we're 100% sure about
#     windows on the top floor" (user).
# ---------------------------------------------------------------------------
def test_is_synthetic_op_explicit_field_wins():
    op_true = {"e": {"name": "gac_window", "synthetic": True}}
    op_false = {"e": {"name": "gac_window_synth", "synthetic": False}}
    check("test_is_synthetic_op_explicit_field_wins",
          fal.is_synthetic_op(op_true) is True
          and fal.is_synthetic_op(op_false) is False)


def test_is_synthetic_op_name_suffix_fallback():
    real = {"e": {"name": "gac_window"}}
    synth = {"e": {"name": "gac_window_synth"}}
    kit = {"e": {"name": "window"}}
    check("test_is_synthetic_op_name_suffix_fallback",
          fal.is_synthetic_op(real) is False
          and fal.is_synthetic_op(synth) is True
          and fal.is_synthetic_op(kit) is False)


def test_max_synthetic_storey_floors_at_zero():
    check("test_max_synthetic_storey_floors_at_zero",
          gf.max_synthetic_storey(16) == 13
          and gf.max_synthetic_storey(2) == 0
          and gf.max_synthetic_storey(1) == 0)


def test_synthetic_side_rects_excludes_top_two_storeys():
    m = _tall_mass()
    out, note = gf._synthetic_side_rects(m, "S", [10, 14, 15])
    check("test_synthetic_side_rects_excludes_top_two_storeys",
          sorted(out.keys()) == [10] and note is None, str((out.keys(), note)))


def test_synthetic_side_rects_falls_back_when_band_is_all_excluded():
    m = _tall_mass()
    out, note = gf._synthetic_side_rects(m, "S", [14, 15])
    check("test_synthetic_side_rects_falls_back_when_band_is_all_excluded",
          list(out.keys()) == [13] and note is not None, str((out.keys(), note)))


def test_drop_top_storey_synthetic_drops_only_synthetic_above_cap():
    m = _tall_mass()
    kept_lo = _fake_event(0, "S", 13, "flame", 1, m)
    kept_lo["ops"][0]["e"] = dict(kept_lo["ops"][0]["e"], synthetic=True)
    dropped_hi = _fake_event(1, "S", 15, "flame", 1, m)
    dropped_hi["ops"][0]["e"] = dict(dropped_hi["ops"][0]["e"], synthetic=True)
    real_hi = _fake_event(2, "S", 15, "flame", 1, m)   # no "synthetic" key
    out, stats = fal.drop_top_storey_synthetic(
        [kept_lo, dropped_hi, real_hi], 16)
    check("test_drop_top_storey_synthetic_drops_only_synthetic_above_cap",
          stats == {"tested": 2, "dropped": 1, "max_allowed": 13}
          and len(out[0]["ops"]) == 1 and len(out[1]["ops"]) == 0
          and len(out[2]["ops"]) == 1, str(stats))


def test_drop_top_storey_synthetic_noop_without_n_storeys():
    events = [_fake_event(0, "S", 15, "flame", 1, _tall_mass())]
    out, stats = fal.drop_top_storey_synthetic(events, 0)
    check("test_drop_top_storey_synthetic_noop_without_n_storeys",
          out is events and stats["max_allowed"] is None)


def test_place_fire_excludes_synthetic_top_storey_flame():
    m = _tall_mass()
    ev = _fake_event(0, "S", 15, "flame", 3, m)
    for op in ev["ops"]:
        op["e"] = dict(op["e"], synthetic=True)
    doc = _fake_doc("F3", "flame", ["S"], origin=15, top=15, n_storeys=16)
    stage, root = _new_stage("/World/topex")
    res = fal.place_fire(stage, root, doc, {"main": m}, [ev], "t",
                         random.Random(1), None, 0.0, 0.0, max_emitters=9)
    check("test_place_fire_excludes_synthetic_top_storey_flame",
          res["flame"] == 0 and res["synthetic_top"] ==
          {"tested": 3, "dropped": 3, "max_allowed": 13}, str(res))


def test_place_fire_keeps_real_top_storey_flame():
    m = _tall_mass()
    ev = _fake_event(0, "S", 15, "flame", 3, m)   # real (`_fake_op`'s default)
    doc = _fake_doc("F3", "flame", ["S"], origin=15, top=15, n_storeys=16)
    stage, root = _new_stage("/World/topreal")
    res = fal.place_fire(stage, root, doc, {"main": m}, [ev], "t",
                         random.Random(1), None, 0.0, 0.0, max_emitters=9)
    check("test_place_fire_keeps_real_top_storey_flame",
          res["flame"] > 0 and res["synthetic_top"]["dropped"] == 0, str(res))


# ---------------------------------------------------------------------------
# 15) SMOKE COMPLEMENTS THE FLAME VERTICALLY — "have more smoke on lower
#     floors so it looks like those have been burnt out if you're not
#     putting fire there" (user).
# ---------------------------------------------------------------------------
def _smoke_test_event(eid, side, storey, n_ops=1):
    return {"id": eid, "side": side, "storey": storey,
            "ops": [{"e": {"storey": storey, "side": side, "name": "x"}}
                    for _ in range(n_ops)]}


def test_pick_smoke_openings_prefers_not_lit_and_lower_storey():
    events = [_smoke_test_event(0, "S", 2), _smoke_test_event(1, "S", 5),
              _smoke_test_event(2, "S", 8), _smoke_test_event(3, "E", 3)]
    lit = {("S", 8)}
    picks = fal._pick_smoke_openings(events, lit, cap=2)
    ids = [p[0]["id"] for p in picks]
    check("test_pick_smoke_openings_prefers_not_lit_and_lower_storey",
          ids == [0, 2], str(ids))


def test_pick_smoke_openings_guarantee_noop_when_nothing_lit():
    events = [_smoke_test_event(0, "S", 2), _smoke_test_event(1, "S", 5)]
    picks = fal._pick_smoke_openings(events, set(), cap=1)
    ids = [p[0]["id"] for p in picks]
    check("test_pick_smoke_openings_guarantee_noop_when_nothing_lit",
          ids == [0], str(ids))


def test_pick_smoke_openings_full_budget_includes_lit_naturally():
    events = [_smoke_test_event(0, "S", 2), _smoke_test_event(1, "E", 3),
              _smoke_test_event(2, "S", 5), _smoke_test_event(3, "S", 8)]
    lit = {("S", 8)}
    picks = fal._pick_smoke_openings(events, lit, cap=4)
    ids = sorted(p[0]["id"] for p in picks)
    check("test_pick_smoke_openings_full_budget_includes_lit_naturally",
          ids == [0, 1, 2, 3], str(ids))


def test_pick_smoke_openings_zero_cap_returns_nothing():
    events = [_smoke_test_event(0, "S", 2)]
    picks = fal._pick_smoke_openings(events, {("S", 2)}, cap=0)
    check("test_pick_smoke_openings_zero_cap_returns_nothing", picks == [])


def test_pick_smoke_events_mirrors_opening_level_ranking():
    events = [_smoke_test_event(0, "S", 2), _smoke_test_event(1, "S", 5),
              _smoke_test_event(2, "S", 8), _smoke_test_event(3, "E", 3)]
    lit = {("S", 8)}
    picks = fal._pick_smoke_events(events, lit, budget=2)
    ids = [e["id"] for e in picks]
    check("test_pick_smoke_events_mirrors_opening_level_ranking",
          ids == [0, 2], str(ids))


def test_place_fire_smoke_vertical_bias_default_matches_explicit_false():
    m = _fake_mass()
    events = [_fake_event(0, "S", 2, "flame", 2, m),
              _fake_event(1, "S", 3, "out", 2, m),
              _fake_event(2, "N", 1, "out", 2, m)]
    doc = _fake_doc("F3", "flame", ["S", "N"], origin=1, top=3, n_storeys=6)
    s1, r1 = _new_stage("/World/vb_default")
    s2, r2 = _new_stage("/World/vb_explicit_false")
    res1 = fal.place_fire(s1, r1, doc, {"main": m}, events, "t",
                          random.Random(4), None, 0.0, 0.0, max_emitters=9)
    res2 = fal.place_fire(s2, r2, doc, {"main": m}, events, "t",
                          random.Random(4), None, 0.0, 0.0, max_emitters=9,
                          smoke_vertical_bias=False)
    check("test_place_fire_smoke_vertical_bias_default_matches_explicit_false",
          res1["smoke"] == res2["smoke"] and res1["flame"] == res2["flame"],
          str((res1, res2)))


def test_place_fire_smoke_vertical_bias_preserves_total_count():
    m = _tall_mass()
    flame_ev = _fake_event(0, "S", 13, "flame", 3, m)
    out_events = [_fake_event(10 + k, "S", k, "out", 1, m)
                 for k in (2, 5, 8, 13)]
    events = [flame_ev] + out_events
    doc = _fake_doc("F3", "flame", ["S"], origin=2, top=13, n_storeys=16)
    s1, r1 = _new_stage("/World/vb_off")
    s2, r2 = _new_stage("/World/vb_on")
    res_off = fal.place_fire(s1, r1, doc, {"main": m}, events, "t",
                             random.Random(5), None, 0.0, 0.0, max_emitters=3,
                             side_smoke_flame_max=2, smoke_vertical_bias=False)
    res_on = fal.place_fire(s2, r2, doc, {"main": m}, events, "t",
                            random.Random(5), None, 0.0, 0.0, max_emitters=3,
                            side_smoke_flame_max=2, smoke_vertical_bias=True)
    check("test_place_fire_smoke_vertical_bias_preserves_total_count",
          res_off["smoke"] == res_on["smoke"] == 2,
          str((res_off, res_on)))


ALL_TESTS = [v for k, v in sorted(globals().items())
            if k.startswith("test_") and callable(v)]


def main():
    for t in ALL_TESTS:
        t()
    print()
    if _FAILS:
        print("{0} FAILURE(S): {1}".format(len(_FAILS), ", ".join(_FAILS)))
        return 1
    print("ALL {0} TESTS OK".format(len(ALL_TESTS)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
