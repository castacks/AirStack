#!/usr/bin/env python3
"""fire_city_union.py — UNION several ignition-point solves of the SAME
placements dump into ONE fire-city manifest (`scene_gen/_plans/
fire_city_500m.json`'s own schema, plus `"origins"`/`"seeds"`/`"note"`).

WHY A UNION AT ALL
------------------
`urban_fire_spread.solve` is a single Dijkstra tree from ONE origin. On the
500 m downtown_fire_500 dump a single origin reaches a genuine, tightly
adjacent district (see `--census`'s own per-block tally: 73 of 87 house
placements are burnable, spread across ten typology blocks) but not the
whole city — some origins reach 30+ buildings, some reach 1. Hitting the
user's own density target ("more fires overall") on top of a SINGLE origin
means either accepting whatever one origin happens to reach, or combining
several origins' own solves. This tool does the latter: run `fire_city_dry_
run.run_dry_from_dump` once per seed against the SAME dump, then union the
resulting record sets, dedup'd by placement index `i` (first-listed seed
wins any collision — see `union_records`).

TWO THINGS A NAIVE UNION GETS WRONG, AND WHY THIS TOOL EXISTS RATHER THAN
JUST CONCATENATING THREE MANIFESTS BY HAND
---------------------------------------------------------------------------
1. THE ROOF-OUTCOME SHARE BUDGET IS PER-MANIFEST, NOT PER-SEED. `urban_
   fire_city.damaged_manifest` caps `ROOF_LEVELS` (F5c/F6) outcomes to
   `roof_collapse_max` (default 2) EACH TIME IT RUNS — once per seed here,
   since each seed is solved as its own standalone manifest. Three seeds
   each independently allowed up to 2 roof outcomes can total 6 in a naive
   concatenation, blowing straight through the "roof-affecting outcomes
   stay RARE" policy the budget exists to enforce citywide. `union_records`
   re-applies the SAME budget mechanics (`urban_fire_spread.ROOF_LEVELS`,
   preferring origins, demoting the rest to F5) to the FINAL unioned record
   list — see `_apply_roof_budget`.
2. A GENUINE PLACEMENT-GEOMETRY DEFECT IN THE DUMP FAILS `check_footprint`
   REGARDLESS OF HOW MANY SEEDS ARE UNIONED. `fc_dump_500.json` (2026-08-31,
   pending a layout-packer fix) places several building pairs closer than
   `check_footprint`'s own `OVERLAP_TOL_M` tolerance -- not touching,
   genuinely interpenetrating (`tools/fire_city_dry_run.check_footprint`'s
   own `_obb_overlap`). This is NOT introduced by unioning; a SINGLE seed
   run at a high enough `--n` already trips it. The honest fix (rather than
   silently dropping a record post-hoc, which can orphan a `via` chain and
   break `check_contiguity`) is to treat the higher-indexed member of each
   overlapping pair as an additional FIREBREAK -- never in the spread
   graph at all, the same "not on the graph" discipline `urban_fire_city.
   burnable`'s own pack-blacklist/height-cap gates already use -- via
   `fire_city_dry_run.run_dry_from_dump`'s `extra_blocked_global` (added by
   this same work item). `_clean_solve` does this as a fixed-point loop:
   solve, read `check_footprint`'s own violation list, block the loser of
   each pair, re-solve, repeat until clean or `max_iters` is exhausted.
   Expect this whole mechanism to matter far less once the layout-packer
   fix mentioned above lands -- it is kept because "a caller can request an
   exclusion" costs nothing when unused and is the honest fix when it is.

CONCENTRATION, NOT JUST COUNT -- THE USER'S OWN DIRECTIVE
-----------------------------------------------------------
"the fire seems to skip a lot of buildings, you need to have it more
concentrated" -- a union of three far-apart 7-building fires satisfies a
raw count target while reading as three unrelated house fires. This tool
reports (and `--auto` SELECTS for) two concentration numbers, both computed
directly off the final record geometry (`concentration_metrics`):
    adjacency_share   fraction of unioned buildings with >=1 OTHER unioned
                      building within `--adjacency-m` (default 25 m) --
                      NOT the fire-mechanism reach (radiation 13 m / spot
                      55 m); a tighter, purely visual "does this read as a
                      clump" radius.
    n_components      connected components of that same <=25 m adjacency
                      graph. 1-2 large components reads as "one coherent
                      burning quarter"; many small ones reads as scattered
                      singles regardless of the total count.
A THIRD metric, `street_facing_share` (`street_facing_metrics`), reports how
much of the CHOSEN venting geometry actually honours the "prefer a
street-facing facade" policy already wired into `urban_fire_spread.
street_side_score` / `fire_city_dry_run.build_manifest` -- computed
post-hoc, against the FULL burnable candidate list (every real neighbour,
not just the ones that ended up on fire), so it is meaningful even for a
manifest this tool did not itself build the sides for.

USAGE
-----
Explicit, reproducible (the common case once good seeds are known):

    python3 tools/fire_city_union.py _plans/fc_dump_500.json \\
        --seeds 43,35,0 --n 40 --collapse 1 --roof-collapse-max 2 \\
        --out _plans/fire_city_500m.json --md _plans/fire_city_500m_report.md

One-command re-solve against a REPLACEMENT dump (searches for its own
seeds -- see `auto_select_seeds`):

    python3 tools/fire_city_union.py _plans/fc_dump_NEW.json --auto \\
        --sweep-max 500 --target-min 20 --target-pref 26 \\
        --out _plans/fire_city_500m.json --md _plans/fire_city_500m_report.md

Both modes run the SAME union/clean/budget/metrics pipeline and end by
re-running `fire_city_dry_run.run_all_checks` (the same six checks the
tests hold every "real manifest" to) plus `check_determinism` on the final
result -- a union this tool writes is never handed back unchecked.
"""
import argparse
import json
import os
import random
import sys
from collections import Counter, defaultdict

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TOOLS_DIR)
if _SCENE_GEN_DIR not in sys.path:
    sys.path.insert(0, _SCENE_GEN_DIR)
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

import fire_city_dry_run as fdr  # noqa: E402


# ---------------------------------------------------------------------------
# Stage 0: the per-block census -- "burnable/refused with reasons, per block"
# ---------------------------------------------------------------------------
def census(dump_path):
    """`(overall, per_block)` for every house placement in `dump_path`.

    `overall` is `{"n_house", "n_burnable", "n_refused", "refusal_reason_
    tally"}`. `per_block` is a list of `{"rect", "typology", "n_house",
    "n_burnable", "n_refused", "reasons"}` dicts, one per typology block the
    dump's own `typology.blocks` names (`fire_city_dry_run.load_placements_
    dump`), sorted by typology name then rect -- this is `urban_fire_city.
    burnable`'s SIX gates applied to every placement, tallied by which
    block it physically stands in (`fire_city_dry_run._block_rect_at`), not
    just the citywide total `run_dry_from_dump`'s own `report_extras`
    already gives you.
    """
    config, layout, placements, _seed, _preset, _sha = \
        fdr.load_placements_dump(dump_path)
    burnable_list, refused_gate, building_typology = fdr.gather_burnable(
        config, layout, placements, None)
    burn_by_i = {i for i, _rec in burnable_list}
    refused_by_i = {r["i"]: r for r in refused_gate}

    per_block = defaultdict(lambda: {"typ": None, "n_house": 0,
                                     "n_burnable": 0, "n_refused": 0,
                                     "reasons": Counter()})
    overall_reasons = Counter()
    n_house = n_burnable = n_refused = 0
    for i, p in enumerate(placements):
        if p.get("category") != "house":
            continue
        n_house += 1
        rect = fdr._block_rect_at(layout, float(p["x_m"]), float(p["y_m"]))
        blk = per_block[rect]
        blk["typ"] = building_typology.get(i)
        blk["n_house"] += 1
        if i in burn_by_i:
            blk["n_burnable"] += 1
            n_burnable += 1
        else:
            blk["n_refused"] += 1
            n_refused += 1
            reason = refused_by_i[i]["reason"]
            short = (reason.split(" -- ")[0].split(":")[0][:60]
                    if isinstance(reason, str) else str(reason)[:60])
            blk["reasons"][short] += 1
            overall_reasons[short] += 1

    per_block_out = []
    for rect, d in per_block.items():
        per_block_out.append({
            "rect": list(rect) if rect else None, "typology": d["typ"],
            "n_house": d["n_house"], "n_burnable": d["n_burnable"],
            "n_refused": d["n_refused"], "reasons": dict(d["reasons"]),
        })
    per_block_out.sort(key=lambda d: (str(d["typology"]), d["rect"] or []))

    overall = {"n_house": n_house, "n_burnable": n_burnable,
              "n_refused": n_refused,
              "refusal_reason_tally": dict(overall_reasons)}
    return overall, per_block_out


# ---------------------------------------------------------------------------
# Stage 1: one seed, footprint-clean (the fixed-point block-and-retry loop)
# ---------------------------------------------------------------------------
def _clean_solve(dump_path, seed, n, collapse, roof_collapse_max,
                 extra_blocked=frozenset(), max_iters=6):
    """`(manifest, checks, extras, blocked)` -- `run_dry_from_dump`, re-run
    with a growing `extra_blocked_global` set until `check_footprint` is
    clean or `max_iters` is exhausted (whichever first; the last attempt is
    always returned, clean or not -- a caller checks `checks["footprint"]`
    itself rather than trusting this loop always wins). `extra_blocked` is
    the STARTING block set (e.g. one already discovered against a different
    seed sharing the same dump -- see `union_records`, which threads a
    single growing set through every seed so two DIFFERENT seeds' own
    records can never overlap each other either)."""
    blocked = frozenset(extra_blocked or ())
    manifest = checks = extras = None
    for _ in range(max(1, int(max_iters))):
        manifest, checks, extras = fdr.run_dry_from_dump(
            dump_path, seed=seed, n=n, collapse=collapse,
            roof_collapse_max=roof_collapse_max, extra_blocked_global=blocked)
        ok, detail = checks["footprint"]
        if ok:
            return manifest, checks, extras, blocked
        grown = set(blocked)
        for a, b in detail["overlaps"]:
            grown.add(max(a, b))
        if grown == blocked:
            break   # no progress possible -- e.g. the origin itself overlaps
        blocked = frozenset(grown)
    return manifest, checks, extras, blocked


# ---------------------------------------------------------------------------
# Stage 2: the union itself
# ---------------------------------------------------------------------------
def union_records(dump_path, seeds, n=40, collapse=1, roof_collapse_max=2,
                  max_iters=8):
    """`(records, refused, origins, blocked, per_seed)` -- the core union.

    Runs `_clean_solve` per seed, ONE SHARED `blocked` set threaded through
    all of them (grown by whichever seed's own footprint check trips first,
    then every seed downstream is re-solved against the grown set too) --
    this is what keeps two DIFFERENT seeds from placing overlapping
    buildings on the union even though neither one alone would ever trip
    `check_footprint` by itself. Dedup by placement index `i`: the
    FIRST-LISTED seed to reach a given building keeps it (documented
    precedence, matching the original two-seed union script this tool
    replaces) -- in practice every accepted seed combination in this dump's
    own solve space is fully disjoint (`check_contiguity`'s forest
    validation would refuse a genuine collision's `via` chain anyway if it
    were not), so this precedence rule is a tie-break that is rarely if
    ever exercised, not a silent-data-loss risk.

    `per_seed` is `{seed: {"n_achieved", "n_new", "origin", "footprint_ok"}}`
    for the report's own per-seed contribution table.
    """
    blocked = frozenset()
    manifests = {}
    # Outer fixed point: run every seed, collect any NEW cross-seed overlap
    # (one seed's kept records vs. another's), grow `blocked`, and redo the
    # whole batch -- `_clean_solve` alone only ever cleans ONE seed against
    # ITSELF.
    for _outer in range(max(1, int(max_iters))):
        manifests = {}
        for s in seeds:
            m, checks, extras, blk = _clean_solve(
                dump_path, s, n, collapse, roof_collapse_max,
                extra_blocked=blocked, max_iters=max_iters)
            manifests[s] = (m, checks, extras)
            blocked = blocked | blk

        by_i = {}
        for s in seeds:
            m, _checks, _extras = manifests[s]
            for r in m["records"]:
                by_i.setdefault(r["i"], r)
        cross_bl = [dict(x=r["x"], y=r["y"], W=r["W"], D=r["D"],
                        yaw=r["yaw_deg"], _i=r["i"])
                   for r in by_i.values()]
        from disaster import urban_fire_spread as ufs
        cross_overlaps = []
        cross_by_i = {r["i"]: r for r in by_i.values()}
        keys = list(by_i.keys())
        for ai in range(len(keys)):
            ra = cross_by_i[keys[ai]]
            for bi in range(ai + 1, len(keys)):
                rb = cross_by_i[keys[bi]]
                ca = fdr._obb_corners(ra["x"], ra["y"], ra["W"], ra["D"], ra["yaw_deg"])
                cb = fdr._obb_corners(rb["x"], rb["y"], rb["W"], rb["D"], rb["yaw_deg"])
                if fdr._obb_overlap(ca, cb):
                    cross_overlaps.append((keys[ai], keys[bi]))
        if not cross_overlaps:
            break
        grown = set(blocked)
        for a, b in cross_overlaps:
            grown.add(max(a, b))
        if grown == blocked:
            break
        blocked = frozenset(grown)

    seen = set()
    records = []
    per_seed = {}
    origins = []
    refused_by_i = {}
    for s in seeds:
        m, checks, _extras = manifests[s]
        origins.append(m["origin"])
        n_new = 0
        for r in m["records"]:
            if r["i"] not in seen:
                seen.add(r["i"])
                records.append(r)
                n_new += 1
        for r in m["refused"]:
            refused_by_i.setdefault(r["i"], r)
        per_seed[s] = {"n_achieved": m["n_achieved"], "n_new": n_new,
                       "origin": m["origin"],
                       "footprint_ok": checks["footprint"][0]}
    records.sort(key=lambda r: r["i"])
    refused = sorted(refused_by_i.values(),
                     key=lambda r: (r["i"] is None, r["i"]))
    return records, refused, origins, blocked, per_seed


def _apply_roof_budget(records, seed_base, roof_collapse_max):
    """Mutates `records` IN PLACE: re-applies `urban_fire_city.
    damaged_manifest`'s roof-outcome SHARE budget across the WHOLE unioned
    set (see module docstring point 1) -- each per-seed manifest already
    applied it against its OWN records only, which is not the same
    guarantee once several seeds are combined.

    Precedence for which survivors keep a `ROOF_LEVELS` (F5c/F6) outcome
    when there are more eligible records than the budget allows, in order:
      1. an ORIGIN record (`via is None`) for ANY of the union's seeds --
         matching `urban_fire_city._apply_height_class_policy`'s own
         "prefer the origin first" convention, extended to every origin in
         a multi-seed union rather than just one.
      2. `low` height-class records -- these are the ONLY class `enforce_
         roof_eligibility` ever lets carry the outcome in the first place,
         so preferring them here is redundant in theory but kept explicit:
         a non-`low` `ROOF_LEVELS` entry should never reach this function at
         all (every per-seed manifest already ran roof eligibility), and if
         one somehow did, this ordering still keeps a `low` survivor over
         it -- see the user's own directive that the F5c must land "on a
         low-rise/brownstone".
      3. a seeded shuffle of the rest, for a reproducible tie-break.
    `roof_collapse_max=None` means "no budget at all" (unusual; a caller
    that wants the `urban_fire_city.ROOF_COLLAPSE_MAX_DEFAULT` should pass
    that explicitly, mirroring `damaged_manifest`'s own convention)."""
    if roof_collapse_max is None:
        return
    from disaster import urban_fire_spread as ufs

    eligible = [r for r in records if r.get("level") in ufs.ROOF_LEVELS]
    if len(eligible) <= roof_collapse_max:
        return
    origin_first = [r for r in eligible if r.get("via") is None]
    low_next = [r for r in eligible
               if r.get("via") is not None
               and r.get("height_class") == ufs.HEIGHT_CLASS_LOW]
    rest = [r for r in eligible
           if r.get("via") is not None
           and r.get("height_class") != ufs.HEIGHT_CLASS_LOW]
    rng = random.Random(int(seed_base))
    rng.shuffle(low_next)
    rng.shuffle(rest)
    ordered = origin_first + low_next + rest
    keep_ids = {id(r) for r in ordered[:roof_collapse_max]}
    for r in eligible:
        if id(r) not in keep_ids:
            r["level"] = "F5"


# ---------------------------------------------------------------------------
# Stage 2b: the SEVERITY-SHARE budget -- "too many roof collapses" fix
# ---------------------------------------------------------------------------
#: `rebalance_severity`'s own precedence for WHICH level supplies a promotion
#: to F5c/F6 -- F5's own `urban_fire.LADDER` recipe is already a collapse
#: (`fire_collapse`), so handing an F5 record a *specific* collapse mode
#: costs nothing extra in realism and touches no record that was not already
#: going to change; F4 is the fallback once the F5 pool is exhausted.
_PROMOTE_SOURCES = ("F5", "F4")


def rebalance_severity(records, dump_path, target, min_side_score=0.0):
    """Mutates `records` IN PLACE toward `target`, a `{level: desired_count}`
    dict for a subset of `urban_fire.LEVELS` (any level omitted is left to
    absorb whatever the others don't claim, always as `"F3"` -- see below).
    Returns a diff list of `(i, old_level, new_level)` for every record that
    actually changed. Never touches anything but `entry["level"]` -- x/y/W/D/
    H/kind/asset/style/origin/sides/seed/age_s/t_ignite_s are exactly what
    they were, so a record `rebalance_severity` leaves alone is BYTE-
    IDENTICAL to its pre-rebalance self and a record it does touch costs
    exactly one re-bake, never a renamed or re-geometried building.

    THE COMPLAINT THIS FIXES (user, 2026-08-31): "too many roof collapses"
    for the F4/F5-heavy union `urban_fire_city.damaged_manifest`'s own
    `epoch_s` snapshot produces (most of a long-burning union's buildings
    have already passed `urban_fire_spread.T_OUT`), AND (a second, sharper
    complaint on the same axis) "almost no partial collapses, sides of
    buildings breaking off" -- `ROOF_LEVELS`' own share budget
    (`_apply_roof_budget`) caps F5c/F6 at a small count but ALSO has no floor,
    so a union can legally end up with exactly one F5c in 66 records. This
    function is a TWO-SIDED budget: a CEILING on F4/F5 (too much burnt-out/
    collapsed stock) and a FLOOR on F5c (too little of the specific "a wall
    came down" look), applied post-hoc to an already-solved manifest instead
    of by re-running the Dijkstra spread -- re-solving renames which
    buildings burn (`fire_city_union`'s own module docstring, point 1), which
    invalidates every existing bake; this is the "prefer demoting levels of
    existing records" alternative.

    Three passes, in order, each fully deterministic (age-sorted, never an
    unseeded `random()` call, so the same `records`+`target` always produces
    the same diff):

      1. F6 THEN F5c GAIN (only if `target` asks for more than are already
         present). F6 goes first so it gets first pick of the oldest
         candidates -- "gutted, near gone" is the more severe ask. Sourced
         from `_PROMOTE_SOURCES` in order (F5 pool first, F4 fallback),
         ranked by `age_s` DESCENDING within each source (the longest-
         burning building is the most physically "deserving" of the worst
         outcome). Eligible only: `urban_fire_spread.height_class(...) ==
         ROOF_ELIGIBLE_CLASS` ("low" -- rowhouse/lowrise, the joist-and-truss
         roof stock a real structural roof loss needs, `urban_fire_spread`'s
         own "ROOF ELIGIBILITY" doctrine) AND `btype in ("urm", "rc")`
         (never `rc_glass` -- a curtain-wall tower's F5c/F6 IS its F5, by
         `urban_fire.LADDER`'s own design) AND at least one of the record's
         OWN `sides` scores `> min_side_score` on `urban_fire_spread.
         street_side_score` against `dump_path`'s full burnable-candidate
         list -- the coordinator's "pick F5c records whose lost elevation
         faces a STREET so the collapse photographs from the drone."
         A record already promoted to F6 in this same call is never also
         eligible for the F5c pass (`level not in ("F5c", "F6")`).
      2. F6 THEN F5c CEILING (2026-09-01 addition -- see below): any excess
         over `target["F6"]` / `target["F5c"]` LEFT OVER after pass 1's own
         gain gets a chance to consume it (gain only ever ADDS when `want`
         exceeds what is already present; it never trims the other
         direction, so a target BELOW the union's own natural count -- e.g.
         "no F6 at this level, ever" -- would otherwise silently do
         nothing) demotes straight to `"F3"`, same rule and same reasoning
         as pass 3 below, oldest-kept / youngest-demoted. F6 first, then
         F5c, so a caller that wants "F6 reserved for the next level up,
         F5c capped at a small absolute count" (baseline Level 2's own
         profile) gets both halves of that enforced, not just whichever the
         gain pass happened to touch.
      3. F5 CEILING: any excess over `target["F5"]` (after passes 1-2's own
         outflow) demotes to `"F3"`, oldest-kept / youngest-demoted (`age_s`
         ASCENDING within the excess pool) -- the youngest F5 in the excess
         is the one closest to the F3/F4 boundary already, so relabelling it
         F3 is the smallest physical stretch.
      4. F4 CEILING: same rule, on whatever is still `"F4"` after passes 1-2
         may have pulled some of it into F5c/F6.

    THE 2026-09-01 F6/F5c CEILING (pass 2) exists because a target BELOW the
    union's natural count is not a hypothetical: baseline Level 2's own
    profile (`PROFILES["baseline_l2"]`, `tools/fire_city_union.py`) asks for
    `F6: 0` on a union whose solve-time `roof_collapse_max` budget (shared
    across F5c+F6 together) had already let one F6 through naturally --
    without this pass the F6 count silently stayed at 1, `--severity-target`
    reporting a target of 0 that plainly was not met. Caught by `apply_
    profile`'s own baseline_l2 sweep against `fc_dump_500.json`, not by a
    unit test written first -- see `test_fire_city_union.py`'s own
    regression test for the fixed shape.

    A demotion ALWAYS lands on `"F3"`, never `"F2"` -- `fire_city_dry_run.
    check_entry_points` requires F3+ to vent 2-3 sides and F1/F2 to vent
    exactly 1; every F4/F5 record already vents 2-3 (it was F3+ to begin
    with), so relabelling it F3 keeps that check passing for free, while F2
    would require also trimming `sides`, a geometry edit this function
    deliberately never makes.

    `target["F5c"]`/`target["F6"]` are TOTAL desired counts across the WHOLE
    `records` list passed in -- when this is called on a SUBSET of a bigger
    union (`fire_city_union.py`'s own two-pool `--severity-target`/
    `--severity-target-subset` split, so that a byte-identical subset stays
    byte-identical -- see that flag's own help text), pass the subset's own
    residual target (how many MORE this pool should contribute), not the
    union-wide total.
    """
    from disaster import urban_fire_spread as ufs

    layout, by_gi = _burnable_geometry(dump_path)
    all_buildings = list(by_gi.values())

    def street_facing(r):
        b = by_gi.get(r["i"])
        if b is None:
            return False
        rect = fdr._block_rect_at(layout, r["x"], r["y"])
        return any(ufs.street_side_score(b, s, all_buildings, block_rect=rect)
                  > min_side_score
                  for s in (r.get("sides") or ()))

    def roof_eligible(r):
        cls = ufs.height_class(typology=r.get("typology"),
                               n_storeys=r.get("n_storeys"))
        return (cls == ufs.ROOF_ELIGIBLE_CLASS
               and r.get("btype") in ("urm", "rc")
               and r["level"] not in ("F5c", "F6")
               and street_facing(r))

    diff = []

    def move(r, new_level):
        old = r["level"]
        if old == new_level:
            return
        r["level"] = new_level
        diff.append((r["i"], old, new_level))

    # --- pass 1: F6 then F5c gain -----------------------------------------
    for goal in ("F6", "F5c"):
        want = target.get(goal)
        if want is None:
            continue
        need = want - sum(1 for r in records if r["level"] == goal)
        for source in _PROMOTE_SOURCES:
            if need <= 0:
                break
            pool = sorted((r for r in records
                          if r["level"] == source and roof_eligible(r)),
                         key=lambda r: -(r.get("age_s") or 0.0))
            for r in pool:
                if need <= 0:
                    break
                move(r, goal)
                need -= 1

    # --- pass 2: F6 then F5c ceiling, excess -> F3 (2026-09-01) ------------
    # what pass 1's gain never covers: a target BELOW what is already
    # present. Same rule as pass 3/4 below (oldest-kept/youngest-demoted,
    # straight to F3) -- see the docstring's own "2026-09-01" note.
    for level in ("F6", "F5c"):
        want = target.get(level)
        if want is None:
            continue
        have = [r for r in records if r["level"] == level]
        excess = len(have) - want
        if excess <= 0:
            continue
        pool = sorted(have, key=lambda r: (r.get("age_s") or 0.0))
        for r in pool[:excess]:
            move(r, "F3")

    # --- pass 3/4: F5 then F4 ceiling, excess -> F3 -------------------------
    for level in ("F5", "F4"):
        want = target.get(level)
        if want is None:
            continue
        have = [r for r in records if r["level"] == level]
        excess = len(have) - want
        if excess <= 0:
            continue
        pool = sorted(have, key=lambda r: (r.get("age_s") or 0.0))
        for r in pool[:excess]:
            move(r, "F3")

    return diff


# ---------------------------------------------------------------------------
# Stage 3: concentration + street-facing metrics, and plain histograms
# ---------------------------------------------------------------------------
def concentration_metrics(records, adjacency_m=25.0):
    """`{"adjacency_share", "n_components", "components"}` -- see module
    docstring's "CONCENTRATION, NOT JUST COUNT" section. `components` is a
    list of index lists (each a connected component's own record `i`s,
    biggest first) for the report's own "which buildings clump together"
    table."""
    from disaster import urban_fire_spread as ufs

    n = len(records)
    if n == 0:
        return {"adjacency_share": 0.0, "n_components": 0, "components": []}
    bl = [dict(x=r["x"], y=r["y"], W=r["W"], D=r["D"], yaw=r["yaw_deg"])
         for r in records]
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    has_neighbor = [False] * n
    for i in range(n):
        for j in range(i + 1, n):
            if ufs.gap_m(bl[i], bl[j]) <= adjacency_m:
                has_neighbor[i] = has_neighbor[j] = True
                union(i, j)

    groups = defaultdict(list)
    for k in range(n):
        groups[find(k)].append(records[k]["i"])
    components = sorted(groups.values(), key=len, reverse=True)
    return {
        "adjacency_share": sum(has_neighbor) / n,
        "n_components": len(components),
        "components": components,
    }


def _burnable_geometry(dump_path):
    """`(layout, by_gi)` -- `by_gi` is `{global_i: {"x","y","W","D","yaw"}}`,
    ONE dict object per burnable candidate, shared by every caller in this
    module that needs a neighbour list for `urban_fire_spread.
    street_side_score`. THE SHARING IS LOAD-BEARING, not a style choice:
    `street_side_score`'s own neighbour loop skips a building's own entry
    with `if c is b: continue` -- an IDENTITY comparison. A caller that
    builds a FRESH `dict(x=r["x"], ...)` for `b` and a SEPARATE fresh list
    for the neighbours is never `is`-equal to anything in that list, so the
    self-skip silently fails and the building's own coordinates count as a
    phantom ZERO-GAP neighbour on whatever compass side its own zero
    displacement vector resolves to (`side_facing`'s `nx=ny=0` case always
    picks `"W"`) -- tanking that one side's score for no physical reason.
    This function exists so every caller passes the SAME object for `b`
    (looked up by the record's own global index) and for the neighbour
    list, exactly mirroring `fire_city_dry_run.build_manifest`'s own
    `b = buildings[p["i"]]` / `street_score(..., buildings, ...)` pairing --
    the first version of `street_facing_metrics` built fresh dicts for `b`
    and undercounted the true share for exactly this reason (caught by
    `street_facing_audit`'s own mismatch check reading 2/2 origins wrong
    before this fix, 0/2 after)."""
    config, layout, placements, _seed, _preset, _sha = \
        fdr.load_placements_dump(dump_path)
    burnable_list, _refused, _typ = fdr.gather_burnable(
        config, layout, placements, None)
    by_gi = {i: dict(x=rec["x"], y=rec["y"], W=rec["W"], D=rec["D"],
                     yaw=rec["yaw_deg"]) for i, rec in burnable_list}
    return layout, by_gi


def street_facing_metrics(dump_path, records):
    """`{"share", "n_sides_checked", "n_positive"}` -- see module docstring's
    third metric. Recomputes `urban_fire_spread.street_side_score` for
    every `(record, side)` pair actually chosen in the manifest, against
    the dump's FULL burnable-candidate list (every real neighbour, not just
    the ones on fire) and the block rect each record's own building stands
    in -- the same two inputs `fire_city_dry_run.build_manifest` used when
    it originally picked these sides, so this is a faithful post-hoc
    re-score, not an approximation. See `_burnable_geometry` for why `b`
    and the neighbour list must share object identity."""
    from disaster import urban_fire_spread as ufs

    layout, by_gi = _burnable_geometry(dump_path)
    all_buildings = list(by_gi.values())

    n_checked = n_pos = 0
    for r in records:
        b = by_gi.get(r["i"])
        if b is None:
            continue   # record not in the burnable set any more (stale dump)
        rect = fdr._block_rect_at(layout, r["x"], r["y"])
        for side in (r.get("sides") or ()):
            score = ufs.street_side_score(b, side, all_buildings,
                                          block_rect=rect)
            n_checked += 1
            if score > 0:
                n_pos += 1
    share = (n_pos / n_checked) if n_checked else 0.0
    return {"share": share, "n_sides_checked": n_checked, "n_positive": n_pos}


def street_facing_audit(dump_path, records):
    """Per-record explanation of WHY each chosen side is or is not
    street-facing -- the coordinator's own request: "for every record whose
    sides include a non-street side while a street-facing side was
    available and not chosen, explain why". Also directly answers "if the
    sampler draws sides before scoring, fix the order": it doesn't -- see
    below.

    THE THREE ROLES A SIDE IN `sides` CAN HAVE (`urban_fire_spread.
    entry_for_plan_fire`'s own docstring):
      origin        the fire's own free choice of venting side (`via is
                    None`) -- `entry_for_plan_fire` already picks
                    `max(_SIDE_RING, key=street_score)`, i.e. it SCORES ALL
                    FOUR COMPASS SIDES BEFORE CHOOSING, never draws blind.
      forced_entry  a non-origin record's entry side, forced to face
                    whichever neighbour actually lit it (contagion realism)
                    -- `entry_for_plan_fire` NEVER re-ranks this by score,
                    by design (module docstring: "the entry side itself is
                    never re-ranked by street_score").
      extra_free    F3+'s single additional corner elevation, when the
                    `rng.randint(1, 2)` "how many extra" draw comes up 1 --
                    `entry_for_plan_fire` picks `max(nb, key=street_score)`
                    over the two corner candidates, again scoring BEFORE
                    choosing.
      extra_spread  F3+'s additional corner elevation(s) when that same
                    draw comes up 2 -- BOTH remaining corners are taken,
                    because there is no choice left to make (the 2026-08-31
                    "needs to look like more than one side" policy), not
                    because scoring was skipped.

    For `origin` and `extra_free` (the only two roles where a real choice
    exists) this function VERIFIES the chosen side is the true best-scoring
    option among its candidates (all 4 compass sides for `origin`, the 2
    corners for `extra_free`) and counts any mismatch -- `n_mismatches`
    should be exactly 0; a caller seeing anything else has found a genuine
    bug, not a policy trade-off, and `entry_for_plan_fire` needs a real
    fix (mindful that `tests/test_urban_fire_spread.py::test_entry_for_
    plan_fire_f3_plus_tiebreak_follows_street_score` pins the EXACT rng-seed
    -> side-count mapping of the current `rng.randint` call, so a fix must
    not change which count a given seed draws).

    Returns `{"rows": [...], "n_mismatches": int, "free_choice_share",
    "forced_or_spread_share"}` -- the two shares split exactly where the
    algorithm does and does not have a free choice, so a caller can see
    that a low OVERALL `street_facing_metrics` share is a consequence of
    how much of `sides` is `forced_entry`/`extra_spread` (architecturally
    required), not of the free-choice slots picking badly.
    """
    from disaster import urban_fire_spread as ufs

    layout, by_gi = _burnable_geometry(dump_path)
    all_buildings = list(by_gi.values())

    rows = []
    n_mismatches = 0
    free_checked = free_pos = 0
    other_checked = other_pos = 0
    for r in records:
        b = by_gi.get(r["i"])
        if b is None:
            continue
        rect = fdr._block_rect_at(layout, r["x"], r["y"])
        scores = {s: ufs.street_side_score(b, s, all_buildings, block_rect=rect)
                  for s in ("S", "E", "N", "W")}
        sides = list(r.get("sides") or [])
        if not sides:
            continue
        first = sides[0]
        via = r.get("via")
        if via is None:
            role = "origin"
            best = max(scores, key=scores.get)
            match = (first == best)
            if not match:
                n_mismatches += 1
            free_checked += 1
            free_pos += scores[first] > 0
            reason = ("free choice: picked the best-scoring of all 4 sides"
                      if match else
                      "MISMATCH -- entry_for_plan_fire did not pick the "
                      "best-scoring side; investigate as a real bug")
        else:
            role = "forced_entry"
            other_checked += 1
            other_pos += scores[first] > 0
            reason = ("forced: faces the neighbour that actually lit this "
                     "building (contagion realism) -- never re-ranked by "
                     "street score, by design")
        rows.append({"i": r["i"], "role": role, "side": first,
                    "score": scores[first], "best_available": max(scores, key=scores.get),
                    "best_score": scores[max(scores, key=scores.get)],
                    "reason": reason})

        if len(sides) >= 2:
            nb = ufs.side_neighbors(first)
            extra = sides[1:]
            if len(extra) == 1:
                best_nb = max(nb, key=scores.get)
                match = (extra[0] == best_nb)
                if not match:
                    n_mismatches += 1
                free_checked += 1
                free_pos += scores[extra[0]] > 0
                reason = ("free choice: picked the best-scoring of the 2 "
                         "corner candidates (the rng.randint(1,2) 'how many "
                         "extra' draw came up 1)" if match else
                         "MISMATCH -- did not pick the best-scoring corner; "
                         "investigate as a real bug")
                rows.append({"i": r["i"], "role": "extra_free", "side": extra[0],
                            "score": scores[extra[0]], "best_available": best_nb,
                            "best_score": scores[best_nb], "reason": reason})
            elif len(extra) == 2:
                for e in extra:
                    other_checked += 1
                    other_pos += scores[e] > 0
                rows.append({"i": r["i"], "role": "extra_spread", "side": list(extra),
                            "score": [scores[e] for e in extra],
                            "best_available": None, "best_score": None,
                            "reason": ("both remaining corners taken -- the "
                                      "rng.randint(1,2) draw came up 2, so "
                                      "there was no side left to choose "
                                      "between (2026-08-31 'needs to look "
                                      "like more than one side' policy)")})

    free_share = (free_pos / free_checked) if free_checked else 0.0
    other_share = (other_pos / other_checked) if other_checked else 0.0
    return {"rows": rows, "n_mismatches": n_mismatches,
           "free_choice_checked": free_checked, "free_choice_positive": free_pos,
           "free_choice_share": free_share,
           "forced_or_spread_checked": other_checked,
           "forced_or_spread_positive": other_pos,
           "forced_or_spread_share": other_share}


# ---------------------------------------------------------------------------
# Stage 3b: VRAM-budget SUBSET -- trim a full union down to at most
# `max_records` for a GPU that cannot hold every bake at once (2026-08-31
# coordinator request: ~250 MB/bake x 45 bakes + a ~5.5 GB intact city +
# Flow fires does not fit a 16.3 GB card, but does fit a 32/48 GB target).
# ---------------------------------------------------------------------------
#: metres of "distance-from-centroid" a street-facing chosen side is worth
#: when ranking drop candidates -- see `build_subset`'s docstring for why
#: this is a flat bonus rather than a proportional one (the underlying
#: quantities, metres of centroid distance vs. a street_side_score value,
#: are not commensurable; a flat bonus is an honest, auditable knob rather
#: than a false precision).
STREET_BONUS_M_DEFAULT = 30.0
#: metres of "distance-from-centroid" credit per metre of a tower/highrise
#: record's own measured height -- "the user wants the tall ones seen
#: burning" (coordinator, 2026-08-31): a 140 m tower earns a 42 m discount
#: at the default coefficient, comfortably outweighing most intra-cluster
#: centroid distances in this city (clusters span up to ~190 m).
TOWER_BONUS_H_COEF_DEFAULT = 0.3


def _components_with_records(records, adjacency_m=25.0):
    """Like `concentration_metrics`'s own `components` list, but returns
    the actual RECORD dicts per component (not just their `i`s) -- what
    `build_subset` needs for centroids without a second `by_i` lookup."""
    conc = concentration_metrics(records, adjacency_m=adjacency_m)
    by_i = {r["i"]: r for r in records}
    return [[by_i[i] for i in comp] for comp in conc["components"]]


def build_subset(dump_path, full_union, max_records, adjacency_m=25.0,
                 street_bonus_m=STREET_BONUS_M_DEFAULT,
                 tower_bonus_coef=TOWER_BONUS_H_COEF_DEFAULT):
    """`(subset_union, dropped)` -- `full_union` trimmed to at most
    `max_records` records for a VRAM-constrained bake run. `dropped` is a
    list of `{"i", "reason"}` for every record actually removed, in the
    order it was removed (farthest/least-protected first).

    EVERY KEPT RECORD IS THE EXACT SAME DICT (a shallow copy of it) AS IN
    `full_union["records"]` -- no field is recomputed, re-derived, or
    reordered. This is the whole point: `fire_city_manifest.entry_string`/
    `build_entry_and_stem` are pure functions of `kind`/`asset-or-style`/
    `level`/`origin`/`sides`/`seed`, all copied verbatim, so a bake already
    on disk for a record that survives the subset is found by `fire_city_
    manifest.classify` as `"HAVE"` under EITHER manifest -- the two files
    share the same `city_<seed>/` bake cache. `verify_stem_equality` proves
    this rather than assuming it.

    FOUR PROTECTIONS, CHECKED IN THIS ORDER, before ANY record is eligible
    to be dropped for centroid distance alone (the coordinator's own
    "never dropping an F5c" plus two structural requirements the request's
    "keeping every cluster a single component" implies but does not spell
    out -- a manifest that fails `check_contiguity` or `check_level_
    distribution` is not a usable subset no matter how good its VRAM
    number is):

      1. F5c/F6 (`urban_fire_spread.ROOF_LEVELS`) -- "never dropping an
         F5c", taken literally; F6 is the same structural family and gets
         the same protection though this dump has none.
      2. a DECLARED ORIGIN (`full_union["origins"]`) -- dropping one would
         desynchronise `check_contiguity`'s "found roots == declared
         roots" invariant, and for `full_union["origin"]` specifically
         (the FIRST-listed origin) would also break `check_level_
         distribution`'s `origin_rec = records[i == manifest["origin"]]`
         lookup outright.
      3. a VIA-PARENT of any record still in the kept set at the moment it
         is considered -- dropping it would leave that child's `via`
         pointing at a building no longer in the manifest, which `check_
         contiguity` reads as "via references a building not in the
         manifest". Checked DYNAMICALLY (against the current kept set, not
         just the original one) because dropping proceeds one record at a
         time and an early drop can retroactively make a later candidate a
         leaf.
      4. would-fragment-its-cluster -- dropping a record whose OWN 25 m-
         adjacency cluster (computed ONCE, on the full set, so the target
         does not drift as trimming proceeds) would split into more than
         one component among its surviving members. A record that was its
         cluster's LAST surviving member has nothing left to fragment and
         is not protected by this rule alone (only by 1-3 above).

    Among the remaining (unprotected) candidates, drop priority is
    `distance_from_own_cluster_centroid MINUS bonuses` for (a) having a
    genuinely street-facing chosen side (`street_facing_audit`'s own
    per-record positive-score check, `street_bonus_m`) and (b) being a
    tower/highrise record, scaled by its own measured height
    (`tower_bonus_coef * H`) -- "preferring to keep records with street-
    facing sides and the taller tower records". Highest-ranked (farthest,
    least protected) is tried first; a candidate that would violate
    protection 3 or 4 at the moment it is tried is skipped (left in) and
    the next-ranked candidate is tried instead, so `len(subset["records"])`
    can come out ABOVE `max_records` if too few candidates are ever
    droppable without breaking a cluster or an in-progress via chain --
    reported honestly via `n_achieved`, never silently forced.
    """
    from disaster import urban_fire_spread as ufs

    records = list(full_union.get("records") or [])
    if len(records) <= max_records:
        subset = dict(full_union)
        subset["records"] = list(records)
        return subset, []

    by_i = {r["i"]: dict(r) for r in records}   # shallow per-record copies
    components = _components_with_records(list(by_i.values()), adjacency_m=adjacency_m)
    cluster_of = {}
    centroids = {}
    for cid, comp in enumerate(components):
        for r in comp:
            cluster_of[r["i"]] = cid
        xs = [r["x"] for r in comp]
        ys = [r["y"] for r in comp]
        centroids[cid] = (sum(xs) / len(xs), sum(ys) / len(ys))

    audit = street_facing_audit(dump_path, list(by_i.values()))
    has_street_side = {}
    for row in audit["rows"]:
        score = row["score"]
        pos = (score > 0) if not isinstance(score, list) else any(s > 0 for s in score)
        has_street_side[row["i"]] = has_street_side.get(row["i"], False) or pos

    protected = set()
    for r in records:
        if r.get("level") in ufs.ROOF_LEVELS:
            protected.add(r["i"])
    for o in (full_union.get("origins") or []):
        if o is not None:
            protected.add(o)

    def _dist(r):
        cx, cy = centroids[cluster_of[r["i"]]]
        return ((r["x"] - cx) ** 2 + (r["y"] - cy) ** 2) ** 0.5

    def _drop_rank(r):
        rank = _dist(r)
        if has_street_side.get(r["i"]):
            rank -= street_bonus_m
        if r.get("typology") in ("tower", "highrise"):
            rank -= tower_bonus_coef * float(r.get("H") or 0.0)
        return rank

    candidates = sorted((r for r in records if r["i"] not in protected),
                        key=_drop_rank, reverse=True)

    kept = dict(by_i)   # i -> record, mutated as we drop
    dropped = []
    n_to_drop = len(records) - max_records
    for r in candidates:
        if len(dropped) >= n_to_drop:
            break
        i = r["i"]
        # protection 3: a via-parent of a currently-kept record
        if any(k.get("via") == i for k in kept.values() if k["i"] != i):
            continue
        # protection 4: would fragment its own cluster
        cid = cluster_of[i]
        cluster_survivors = [k for k in kept.values()
                             if cluster_of.get(k["i"]) == cid and k["i"] != i]
        if cluster_survivors:
            sub_conc = concentration_metrics(cluster_survivors, adjacency_m=adjacency_m)
            if sub_conc["n_components"] > 1:
                continue
        del kept[i]
        dropped.append({"i": i, "reason": (
            "farthest-from-cluster-centroid trim for VRAM budget "
            "(dist={0:.1f} m, street_side={1}, typology={2})".format(
                _dist(r), has_street_side.get(i, False), r.get("typology")))})

    subset_records = sorted(kept.values(), key=lambda r: r["i"])
    subset = dict(full_union)
    subset["records"] = subset_records
    subset["n_achieved"] = len(subset_records)
    subset["roof_outcome_count"] = sum(1 for r in subset_records
                                       if r.get("level") in ufs.ROOF_LEVELS)
    subset["vram_subset_of"] = {
        "full_n_achieved": len(records), "max_records_requested": int(max_records),
        "dropped_i": [d["i"] for d in dropped],
    }
    subset["note"] = (
        "VRAM-budget SUBSET of the {0}-record union above (seeds {1}) -- "
        "{2} record(s) trimmed to {3} for a card that cannot hold every "
        "bake at once (~250 MB/bake measured, 16.3 GB local vs. 32/48 GB "
        "target). Every kept record is a byte-for-byte copy of its full-"
        "manifest entry (same kind/asset-or-style/level/origin/sides/seed) "
        "-- see verify_stem_equality -- so bakes are shared between the "
        "two manifests via the same city_{4}/ cache. Dropped: farthest "
        "from its own 25 m-adjacency cluster's centroid first, protecting "
        "(in order) every F5c/F6, every declared ignition origin, any "
        "record still acting as another kept record's via-parent, and any "
        "record whose removal would split its cluster into more than one "
        "component; among the rest, a street-facing chosen side and "
        "tower/highrise height both buy a centroid-distance discount so "
        "the tall towers are the last things cut."
    ).format(len(records), full_union.get("seeds"), len(dropped),
            len(subset_records), full_union.get("seed"))
    return subset, dropped


def verify_stem_equality(full_union, subset_union):
    """`(ok, detail)` -- proves every record kept in `subset_union` computes
    the EXACT SAME `fire_city_manifest.out_stem` as its counterpart in
    `full_union`, i.e. the two manifests genuinely share one bake cache
    rather than merely looking similar. This is the check the coordinator
    asked to see "verify stem equality in a test" -- run automatically by
    the `--max-records` CLI path (see `main`) rather than left to a
    separate, easy-to-forget invocation.

    Imports `tools.fire_city_manifest` directly (its own docstring: "PURE
    PYTHON, NO pxr AT IMPORT TIME") so this uses the REAL stem function a
    bake run will actually call, not a re-derivation that could quietly
    drift from it.
    """
    _ensure_tools_on_path()
    import fire_city_manifest as fcm

    full_by_i = {r["i"]: r for r in (full_union.get("records") or [])}
    mismatches = []
    checked = 0
    for r in (subset_union.get("records") or []):
        i = r["i"]
        full_r = full_by_i.get(i)
        if full_r is None:
            mismatches.append({"i": i, "reason": "not present in the full "
                               "manifest at all -- subset invented a record"})
            continue
        try:
            _entry_a, stem_a = fcm.build_entry_and_stem(full_r, 0)
            _entry_b, stem_b = fcm.build_entry_and_stem(r, 0)
        except Exception as exc:
            mismatches.append({"i": i, "reason": "stem computation raised: "
                               "{0}".format(exc)})
            continue
        checked += 1
        if stem_a != stem_b:
            mismatches.append({"i": i, "full_stem": stem_a, "subset_stem": stem_b})
    ok = not mismatches
    return ok, {"n_checked": checked, "n_mismatches": len(mismatches),
               "mismatches": mismatches}


def _ensure_tools_on_path():
    if _TOOLS_DIR not in sys.path:
        sys.path.insert(0, _TOOLS_DIR)


def histograms(records):
    """Plain tallies for the report: level, sides-count, kind, height
    class, and the single number the user's own directive (d) is a
    guardrail against (max burning height)."""
    levels = Counter(r.get("level") for r in records)
    sides_count = Counter(len(r.get("sides") or ()) for r in records)
    kinds = Counter(r.get("kind") for r in records)
    height_classes = Counter(r.get("height_class") for r in records)
    typologies = Counter(r.get("typology") for r in records)
    max_h = max((r.get("H") or 0.0) for r in records) if records else 0.0
    return {
        "levels": dict(sorted(levels.items())),
        "sides_count": dict(sorted(sides_count.items())),
        "kinds": dict(sorted(kinds.items())),
        "height_classes": dict(sorted(height_classes.items())),
        "typologies": dict(sorted(typologies.items())),
        "max_H": max_h,
        "n_records": len(records),
    }


# ---------------------------------------------------------------------------
# Stage 4: --auto seed selection
# ---------------------------------------------------------------------------
def _component_census(dump_path, sweep_max, n=40, collapse=1,
                      roof_collapse_max=2, seed_start=0):
    """`[(seed, iset, origin), ...]`, one entry per seed in
    `[seed_start, seed_start + sweep_max)` that reaches at least one
    burnable building, RAW (no footprint cleaning -- that happens once, on
    whatever seed COMBINATION `auto_select_seeds` finally picks, not on
    every candidate swept here -- cleaning is the expensive, iterative
    part). `iset` is the frozenset of placement indices that seed's own
    solve reaches at `n`."""
    out = []
    for seed in range(seed_start, seed_start + sweep_max):
        try:
            m, _checks, _extras = fdr.run_dry_from_dump(
                dump_path, seed=seed, n=n, collapse=collapse,
                roof_collapse_max=roof_collapse_max)
        except Exception:
            continue
        iset = frozenset(r["i"] for r in m["records"])
        if iset:
            out.append((seed, iset, m["origin"]))
    return out


def auto_select_seeds(dump_path, sweep_max=500, n=40, collapse=1,
                      roof_collapse_max=2, target_min=20, target_pref=26,
                      adjacency_m=25.0, seed_start=0, max_candidates=40):
    """Greedy seed selection for `--auto` -- see module docstring's usage
    section. Returns `(seeds, notes)`.

    1. Sweep `[seed_start, seed_start+sweep_max)`, dedup to DISTINCT
       reachable sets (many seeds land the same component; keep the
       smallest seed per distinct `iset`), keep the `max_candidates`
       largest.
    2. Greedily grow a union: take the single largest component first: it
       is, by construction, one origin's own connected reach and therefore
       already internally coherent. Repeatedly add whichever remaining
       candidate (a) contributes at least one NEW record and (b) has the
       SMALLEST min building-to-building gap to the union built so far
       (favouring an ADJACENT/nearby addition over a distant one, per the
       user's own "one or two large connected districts, not three
       far-apart clusters" directive) -- until the union reaches
       `target_pref` records or no candidate is left. Falls back to
       accepting whatever is reached once every candidate is exhausted, as
       long as it clears `target_min`; raises if even `target_min` is not
       reachable from this sweep range (the caller should widen
       `sweep_max`).

       THE SIZE CHECKED AGAINST `target_min`/`target_pref` IS THE TRUE,
       POST-FOOTPRINT-CLEAN union size (`union_records`, re-run after every
       addition), NOT the raw pre-clean reachable-set size the ranking
       above sorts by. A single seed can look huge before cleaning and lose
       most of that to the footprint-overlap firebreak (see module
       docstring point 2) -- checking the raw size here would let this
       function declare victory on a component that shrinks well below
       `target_min` the moment it is actually solved clean, exactly the
       failure mode a first version of this function had.
    """
    from disaster import urban_fire_spread as ufs

    raw = _component_census(dump_path, sweep_max, n=n, collapse=collapse,
                            roof_collapse_max=roof_collapse_max,
                            seed_start=seed_start)
    best_for_iset = {}
    for seed, iset, origin in raw:
        cur = best_for_iset.get(iset)
        if cur is None or seed < cur[0]:
            best_for_iset[iset] = (seed, origin)
    candidates = sorted(best_for_iset.items(), key=lambda kv: -len(kv[0]))
    candidates = candidates[:max_candidates]
    if not candidates:
        raise ValueError("auto_select_seeds: no seed in the swept range "
                         "reached any burnable building")

    def _bl_of(iset, m_cache):
        recs = m_cache[iset]
        return [dict(x=r["x"], y=r["y"], W=r["W"], D=r["D"], yaw=r["yaw_deg"])
               for r in recs]

    # need each candidate's own record geometry for the min-gap ranking --
    # cheap re-solve (no cleaning) at the same (seed, n) already swept.
    geo_cache = {}
    for iset, (seed, _origin) in candidates:
        m, _checks, _extras = fdr.run_dry_from_dump(
            dump_path, seed=seed, n=n, collapse=collapse,
            roof_collapse_max=roof_collapse_max)
        geo_cache[iset] = m["records"]

    def _clean_size(seed_list):
        recs, _refused, _origins, _blocked, _per_seed = union_records(
            dump_path, seed_list, n=n, collapse=collapse,
            roof_collapse_max=roof_collapse_max)
        return len(recs)

    chosen = [candidates[0]]
    chosen_i = set(candidates[0][0])
    chosen_bl = _bl_of(candidates[0][0], geo_cache)
    seeds = [candidates[0][1][0]]
    clean_n = _clean_size(seeds)
    notes = ["seed {0} (largest single component, {1} raw / {2} clean "
            "records) started the union".format(seeds[0],
                                                 len(candidates[0][0]), clean_n)]
    remaining = candidates[1:]
    while clean_n < target_pref and remaining:
        best_idx, best_gap, best_new = None, None, 0
        for idx, (iset, (seed, _origin)) in enumerate(remaining):
            new_i = iset - chosen_i
            if not new_i:
                continue
            bl = _bl_of(iset, geo_cache)
            gap = min(ufs.gap_m(a, b) for a in bl for b in chosen_bl)
            if best_gap is None or gap < best_gap:
                best_idx, best_gap, best_new = idx, gap, len(new_i)
        if best_idx is None:
            break
        iset, (seed, _origin) = remaining.pop(best_idx)
        chosen.append((iset, (seed, _origin)))
        chosen_i |= iset
        chosen_bl = chosen_bl + _bl_of(iset, geo_cache)
        seeds.append(seed)
        clean_n = _clean_size(seeds)
        notes.append("seed {0} added ({1} new raw records, {2:.1f} m from "
                     "the union so far, {3} clean records total)".format(
                         seed, best_new, best_gap, clean_n))
        if clean_n >= target_pref:
            break

    if clean_n < target_min:
        raise ValueError(
            "auto_select_seeds: best achievable CLEAN union from this "
            "sweep is only {0} records (< target_min={1}) -- widen "
            "--sweep-max".format(clean_n, target_min))

    notes.append("final union: {0} clean records from {1} seed(s) "
                "(target_min={2}, target_pref={3})".format(
                    clean_n, len(seeds), target_min, target_pref))
    return seeds, notes


# ---------------------------------------------------------------------------
# Stage 5: assembling + checking the final manifest dict
# ---------------------------------------------------------------------------
def build_union(dump_path, seeds, n=40, collapse=1, roof_collapse_max=2,
                adjacency_m=25.0, max_iters=8, note_extra=""):
    """`(union, metrics, per_seed, checks, det_ok, det_detail)` -- the whole
    pipeline: solve every seed footprint-clean and cross-seed-clean
    (`union_records`), re-apply the roof-outcome share budget across the
    WHOLE union (`_apply_roof_budget`), assemble the manifest dict, then
    verify it: `fire_city_dry_run.run_all_checks` (the same six checks
    every other manifest in this repo is held to) plus a determinism
    check (this function re-run on the same seeds must reproduce the same
    manifest byte-for-byte)."""
    records, refused, origins, blocked, per_seed = union_records(
        dump_path, seeds, n=n, collapse=collapse,
        roof_collapse_max=roof_collapse_max, max_iters=max_iters)

    seed_base = int(seeds[0])
    _apply_roof_budget(records, seed_base, roof_collapse_max)

    from disaster import urban_fire_spread as ufs
    roof_outcome_count = sum(1 for r in records if r["level"] in ufs.ROOF_LEVELS)

    # provenance: every seed solved against the identical dump, so every
    # manifest carries the same "placements_dump" -- read it off the first.
    first_manifest, _checks0, _extras0 = fdr.run_dry_from_dump(
        dump_path, seed=seeds[0], n=n, collapse=collapse,
        roof_collapse_max=roof_collapse_max, extra_blocked_global=blocked)
    epoch_s = first_manifest["epoch_s"]
    preset = first_manifest["preset"]
    dump_provenance = first_manifest.get("placements_dump")

    conc = concentration_metrics(records, adjacency_m=adjacency_m)
    street = street_facing_metrics(dump_path, records)
    street_audit = street_facing_audit(dump_path, records)
    hist = histograms(records)

    note = (
        "union of seed(s) {0} ({1} ignition point(s)) against {2}, built by "
        "tools/fire_city_union.py. {3} records, {4} connected component(s) "
        "at a {5:.0f} m visual-adjacency radius, adjacency_share={6:.2f}, "
        "street_facing_share={7:.2f} ({8} sides checked), roof_outcome_count="
        "{9} (budget {10}), max burning height {11:.1f} m. Firebreak set "
        "(footprint-overlap defect in this dump, see module docstring) auto-"
        "discovered and applied to every seed: {12}.{13}"
    ).format(seeds, len(seeds), os.path.abspath(dump_path), len(records),
            conc["n_components"], adjacency_m, conc["adjacency_share"],
            street["share"], street["n_sides_checked"], roof_outcome_count,
            roof_collapse_max, hist["max_H"], sorted(blocked),
            (" " + note_extra) if note_extra else "")

    union = {
        "seed": seed_base, "preset": preset, "n": int(n),
        "n_achieved": len(records), "origin": origins[0],
        "epoch_s": epoch_s, "records": records, "refused": refused,
        "roof_outcome_count": roof_outcome_count,
        "placements_dump": dump_provenance,
        "origins": origins, "seeds": [int(s) for s in seeds],
        "firebreak_indices": sorted(blocked),
        "note": note,
    }

    checks = fdr.run_all_checks(union)

    def _run_fn(_s):
        recs2, ref2, _org2, _blk2, _ps2 = union_records(
            dump_path, seeds, n=n, collapse=collapse,
            roof_collapse_max=roof_collapse_max, max_iters=max_iters)
        _apply_roof_budget(recs2, seed_base, roof_collapse_max)
        recs2.sort(key=lambda r: r["i"])
        return {"records": recs2, "refused": ref2}

    det_ok, det_detail = fdr.check_determinism(seed_base, _run_fn)

    metrics = {"concentration": conc, "street_facing": street,
              "street_facing_audit": street_audit, "histograms": hist}
    return union, metrics, per_seed, checks, det_ok, det_detail


# ---------------------------------------------------------------------------
# Stage 6: BASELINE LEVEL PROFILES -- `--profile baseline_l1|l2|l3`
#
# `scene_gen/_plans/baseline_fire_ladder.md` is the design doc this table
# implements; read it first for the one-line justification behind every
# number here. Every profile is DUMP-PARAMETERIZED: `burn_frac` is a SHARE of
# THIS dump's own `census()` burnable-candidate count (not a fixed absolute
# `--n`), and `severity_frac` is a SHARE of the union's own TRUE `n_achieved`
# AFTER solving (not the requested `--n`, which a footprint-overlap firebreak
# or an under-reaching seed combination can leave short of) -- so the same
# profile name produces a correctly-scaled manifest whether it runs against
# the 500 m stand-in (`fc_dump_500.json`, ~79 burnable candidates today) or a
# real 1 km dump (~4x the building stock, per the freeze-disaster-dataset
# skill's own scaling note). See `resolve_severity_target` / `apply_profile`.
# ---------------------------------------------------------------------------
PROFILES = {
    "baseline_l1": {
        "label": "Level 1 -- early / contained",
        "burn_frac": 0.12,
        "collapse": 0,               # per-seed F5c enforcement target fed to
                                     # `fire_city_dry_run._enforce_target_f5c`
        "roof_collapse_max": 0,      # NO F5c/F6 anywhere in the union
        "severity_frac": {
            "F4": 0.15,               # a small burnt-out tail only
            "F5": "origin_only",      # exactly 1 -- `check_level_
                                      # distribution` REQUIRES the origin at
                                      # F5+ for every level, "contained" or
                                      # not; this is that one mandatory record
            "F5c": 0.0, "F6": 0.0,    # no roof loss at all
        },
    },
    "baseline_l2": {
        "label": "Level 2 -- established multi-block fire (today's mix)",
        "burn_frac": 0.26,
        "collapse": 1,
        "roof_collapse_max": 2,       # today's ROOF_COLLAPSE_MAX_DEFAULT
        # "2-3 clusters" (module docstring's own "CONCENTRATION, NOT JUST
        # COUNT" metric) needs the PER-SEED reach capped well below
        # n_target, or the first (largest) auto candidate alone already
        # satisfies target_pref and auto_select_seeds never looks for a
        # second seed at all -- measured on fc_dump_500.json: at
        # per_seed_n == n_target, seed 0 alone reached all 21 in ONE
        # component; at 0.6x, no single seed's own solve can reach
        # target_pref, so >=2 seeds -- genuinely separate ignition points --
        # are required to hit it. See `apply_profile`'s own `per_seed_n`.
        "per_seed_n_frac": 0.6,
        "severity_frac": {
            "F4": 0.28,                # matches the measured 39-record mix
            "collapse_visible": 0.18,  # F5+F5c+F6 combined share
            "F5c": 2, "F6": 0,         # "1-2 partial collapses" is a narrow,
                                       # SCALE-INDEPENDENT ask -- an absolute
                                       # count, not a fraction of n_achieved;
                                       # F5 absorbs the rest of the 18%
        },
    },
    "baseline_l3": {
        "label": "Level 3 -- severe conflagration",
        "burn_frac": 0.38,
        "collapse": 2,
        "roof_collapse_max": None,     # computed -- see profile_roof_collapse_max
        "severity_frac": {
            "F4": 0.35,                 # a bigger burnt-out core than L2
            "collapse_visible": 0.28,   # still a MINORITY of the union
            "F5c": 0.14, "F6": 0.05,    # both proper shares now, not a
                                        # fixed absolute count -- F5 takes
                                        # the remaining 0.09 of the 0.28
        },
    },
}


def resolve_severity_target(spec, n_achieved):
    """One `PROFILES[...]["severity_frac"]` dict -> the absolute `{level:
    count}` dict `rebalance_severity`'s own `target` parameter wants,
    resolved against `n_achieved` (the union's TRUE achieved record count).
    Every value in `spec` is one of:

      `"origin_only"`   -> 1 -- the one record `check_level_distribution`
                           always requires at F5+ (see `PROFILES["baseline_
                           l1"]`'s own comment).
      an `int`          -> an ABSOLUTE count, independent of `n_achieved` --
                           the "1-2 partial collapses" ask is narrow and
                           scale-independent, not a fraction that would grow
                           with plate size.
      a `float`         -> `round(v * n_achieved)`, a SHARE of the union.

    `spec["collapse_visible"]`, if present, is the F5+F5c+F6 TOTAL share;
    F5's own count is back-derived as `collapse_visible - F5c - F6`, floored
    at 1 so the mandatory origin record always survives even when F5c+F6
    alone would already meet or exceed the total (a small, deliberately
    generous edge case -- `rebalance_severity`'s own F5-ceiling pass, given a
    target below what is actually present, only ever moves the YOUNGEST
    excess to F3, and the origin is always the union's OLDEST record by
    construction, so it is never at risk from that pass either way)."""
    def _one(v, default=0):
        if v is None:
            v = default
        if v == "origin_only":
            return 1
        if isinstance(v, bool):
            raise TypeError(f"severity_frac value {v!r} is a bool, not an "
                            f"int or float")
        if isinstance(v, int):
            return v
        return int(round(float(v) * n_achieved))

    f4 = _one(spec.get("F4", 0.0))
    f5c = _one(spec.get("F5c", 0))
    f6 = _one(spec.get("F6", 0))
    if "collapse_visible" in spec:
        cv = _one(spec["collapse_visible"])
        f5 = max(1, cv - f5c - f6)
    else:
        f5 = _one(spec.get("F5", 1))
    return {"F4": f4, "F5": f5, "F5c": f5c, "F6": f6}


def profile_roof_collapse_max(profile, n_target):
    """The `roof_collapse_max` handed to `build_union`'s OWN solve-time
    share budget for a `--profile` run. This only needs to land in the
    right BALLPARK, never the exact final count: `rebalance_severity`, run
    AFTER the union (see `apply_profile`), is what actually lands the
    precise F5c/F6 count `severity_frac` asks for, and it has no F5c/F6
    CEILING pass (only a GAIN one -- see its own docstring's three-pass
    list) -- so this cap's real job is to stop the union stage ITSELF from
    overshooting the target before rebalancing ever runs, not to compute
    the target exactly. Resolved against `n_target` (the requested `--n`,
    not yet the true `n_achieved` -- this function runs BEFORE the solve)."""
    explicit = profile.get("roof_collapse_max")
    if explicit is not None:
        return int(explicit)
    target = resolve_severity_target(profile["severity_frac"], n_target)
    return target["F5c"] + target["F6"]


def apply_profile(dump_path, profile_name, seeds=None, sweep_max=500,
                  seed_start=0, adjacency_m=25.0, max_iters=8,
                  min_side_score=0.0, note_extra=""):
    """The whole `--profile` pipeline for one baseline level: dump-
    parameterize `PROFILES[profile_name]` against THIS dump's own
    `census()`, solve (an explicit `seeds` list, or an `--auto`-style search
    via `auto_select_seeds` when `seeds` is `None`), then reshape the result
    toward the profile's own `severity_frac` table with `rebalance_severity`.
    See `scene_gen/_plans/baseline_fire_ladder.md` for the design this
    implements.

    Returns `(union, metrics, per_seed, checks, det_ok, det_detail, profile,
    severity_target, diff)` -- the same 6-tuple `build_union` returns (all
    six recomputed AFTER the rebalance, so a caller never sees the pre-
    rebalance numbers), plus the resolved profile dict, the resolved
    absolute severity target, and `rebalance_severity`'s own diff list."""
    profile = PROFILES[profile_name]
    overall_census, _per_block = census(dump_path)
    n_burnable = overall_census["n_burnable"]
    n_target = max(3, int(round(profile["burn_frac"] * n_burnable)))
    per_seed_n = max(3, int(round(n_target * profile.get("per_seed_n_frac", 1.0))))
    roof_max = profile_roof_collapse_max(profile, n_target)
    collapse = int(profile.get("collapse", 1))

    auto_notes = []
    if seeds is None:
        target_min = max(3, int(round(n_target * 0.8)))
        target_pref = n_target
        seeds, auto_notes = auto_select_seeds(
            dump_path, sweep_max=sweep_max, n=per_seed_n, collapse=collapse,
            roof_collapse_max=roof_max, target_min=target_min,
            target_pref=target_pref, adjacency_m=adjacency_m,
            seed_start=seed_start)
        for note in auto_notes:
            print(f"[fire_city_union] profile {profile_name} auto: {note}")

    profile_note = (
        f"profile={profile_name} ({profile['label']}) burn_frac="
        f"{profile['burn_frac']:.2f} of {n_burnable} burnable candidates "
        f"-> n_target={n_target}, per_seed_n={per_seed_n}, solve-time "
        f"roof_collapse_max={roof_max}, "
        f"collapse={collapse}.{(' ' + note_extra) if note_extra else ''}")

    union, _metrics0, per_seed, _checks0, _det_ok0, _det_detail0 = build_union(
        dump_path, seeds, n=per_seed_n, collapse=collapse,
        roof_collapse_max=roof_max, adjacency_m=adjacency_m,
        max_iters=max_iters, note_extra=profile_note)

    n_achieved = union["n_achieved"]
    severity_target = resolve_severity_target(profile["severity_frac"], n_achieved)
    diff = rebalance_severity(union["records"], dump_path, severity_target,
                              min_side_score=min_side_score)
    union["records"].sort(key=lambda r: r["i"])

    from disaster import urban_fire_spread as ufs
    union["roof_outcome_count"] = sum(1 for r in union["records"]
                                      if r["level"] in ufs.ROOF_LEVELS)
    union["profile"] = profile_name
    union["severity_target"] = severity_target
    union["note"] = (union["note"] + " rebalance_severity target={0}, "
                     "{1} record(s) relevelled.".format(severity_target, len(diff)))

    checks = fdr.run_all_checks(union)
    metrics = {
        "concentration": concentration_metrics(union["records"], adjacency_m=adjacency_m),
        "street_facing": street_facing_metrics(dump_path, union["records"]),
        "street_facing_audit": street_facing_audit(dump_path, union["records"]),
        "histograms": histograms(union["records"]),
    }

    def _run_fn(_s):
        u2, _m2, _ps2, _c2, _d2, _dd2 = build_union(
            dump_path, seeds, n=per_seed_n, collapse=collapse,
            roof_collapse_max=roof_max, adjacency_m=adjacency_m,
            max_iters=max_iters, note_extra=profile_note)
        rebalance_severity(u2["records"], dump_path, severity_target,
                           min_side_score=min_side_score)
        u2["records"].sort(key=lambda r: r["i"])
        return {"records": u2["records"], "refused": u2["refused"]}

    det_ok, det_detail = fdr.check_determinism(int(seeds[0]), _run_fn)

    return (union, metrics, per_seed, checks, det_ok, det_detail, profile,
           severity_target, diff)


def _parse_severity_spec(s):
    """`"F4:11,F5c:4,F6:1,collapse_visible:0.18"` -> a `severity_frac`-shaped
    dict `resolve_severity_target` accepts -- the CLI form of one
    `PROFILES[...]["severity_frac"]` entry, for `--severity-target` used
    standalone (no `--profile`) or as an explicit override on top of one.
    A value that parses as `int` is kept absolute; otherwise it is parsed as
    `float` (a fraction of `n_achieved`); the literal string `origin_only`
    is kept verbatim (only meaningful for the `F5` key)."""
    spec = {}
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" not in part:
            raise ValueError(f"--severity-target entry {part!r} is not "
                             f"'key:value'")
        k, v = part.split(":", 1)
        k, v = k.strip(), v.strip()
        if v == "origin_only":
            spec[k] = v
            continue
        try:
            spec[k] = int(v)
        except ValueError:
            spec[k] = float(v)
    return spec


# ---------------------------------------------------------------------------
# Markdown report
# ---------------------------------------------------------------------------
def _format_markdown(dump_path, union, metrics, per_seed, checks, det_ok,
                     det_detail, overall_census, per_block_census,
                     before=None):
    lines = [f"# Fire city union — `{os.path.basename(dump_path)}` "
            f"seeds {union['seeds']}\n"]
    lines.append(f"{union['n_achieved']} records, origins {union['origins']}, "
                f"epoch_s {union['epoch_s']:.1f} "
                f"({union['epoch_s']/60.0:.1f} min).\n")
    lines.append(union["note"] + "\n")

    lines.append("## Census (this dump)\n")
    lines.append(f"* {overall_census['n_house']} house placements\n"
                f"* {overall_census['n_burnable']} burnable candidates\n"
                f"* {overall_census['n_refused']} refused at the gate\n")
    lines.append("### Refusal reasons\n")
    lines.append("| reason (prefix) | count |")
    lines.append("|---|---|")
    for k, v in sorted(overall_census["refusal_reason_tally"].items(),
                       key=lambda kv: -kv[1]):
        lines.append(f"| {k} | {v} |")
    lines.append("")
    lines.append("### Per block\n")
    lines.append("| typology | rect | house | burnable | refused | reasons |")
    lines.append("|---|---|---|---|---|---|")
    for b in per_block_census:
        rect = ("(%.0f,%.0f)-(%.0f,%.0f)" % tuple(b["rect"])) if b["rect"] else "-"
        lines.append(f"| {b['typology']} | {rect} | {b['n_house']} | "
                    f"{b['n_burnable']} | {b['n_refused']} | {b['reasons']} |")
    lines.append("")

    lines.append("## Per-seed contribution\n")
    lines.append("| seed | origin | n_achieved (solo) | n_new (into union) | "
                "footprint_ok (solo) |")
    lines.append("|---|---|---|---|---|")
    for s, d in per_seed.items():
        lines.append(f"| {s} | {d['origin']} | {d['n_achieved']} | "
                    f"{d['n_new']} | {d['footprint_ok']} |")
    lines.append("")

    hist = metrics["histograms"]
    conc = metrics["concentration"]
    street = metrics["street_facing"]

    def _hist_table(title, d, before_d=None):
        out = [f"### {title}\n", "| key | count" +
              (" | before |" if before_d is not None else " |"),
              "|---|---" + ("|---|" if before_d is not None else "|")]
        keys = sorted(set(d) | set(before_d or {}), key=lambda k: str(k))
        for k in keys:
            row = f"| {k} | {d.get(k, 0)}"
            if before_d is not None:
                row += f" | {before_d.get(k, 0)}"
            out.append(row + " |")
        out.append("")
        return out

    def _fmt(v, spec=".2f"):
        if v is None or v == "n/a":
            return "n/a"
        try:
            return format(float(v), spec)
        except (TypeError, ValueError):
            return str(v)

    lines.append("## Histograms\n")
    b = before or {}
    bh = b.get("histograms", {})
    lines.extend(_hist_table("Level", hist["levels"], bh.get("levels")))
    lines.extend(_hist_table("Sides count", hist["sides_count"], bh.get("sides_count")))
    lines.extend(_hist_table("Kind", hist["kinds"], bh.get("kinds")))
    lines.extend(_hist_table("Height class", hist["height_classes"], bh.get("height_classes")))
    lines.extend(_hist_table("Typology", hist["typologies"], bh.get("typologies")))
    lines.append(f"Max burning height: **{hist['max_H']:.1f} m** "
                f"(before: {_fmt(bh.get('max_H'), '.1f')} m)\n")
    lines.append(f"n_records: **{hist['n_records']}** "
                f"(before: {bh.get('n_records', 'n/a')})\n")

    lines.append("## Concentration\n")
    lines.append(f"* adjacency_share (<= {25.0:.0f} m): **{conc['adjacency_share']:.2f}**"
                + (f" (before: {_fmt(b.get('concentration',{}).get('adjacency_share'))})"
                   if before else "") + "\n")
    lines.append(f"* n_components: **{conc['n_components']}**"
                + (f" (before: {b.get('concentration',{}).get('n_components','n/a')})"
                   if before else "") + "\n")
    lines.append(f"* street_facing_share: **{street['share']:.2f}** "
                f"({street['n_positive']}/{street['n_sides_checked']} sides)"
                + (f" (before: {_fmt(b.get('street_facing',{}).get('share'))})"
                   if before else "") + "\n")

    audit = metrics.get("street_facing_audit")
    if audit:
        lines.append("### Street-facing side audit\n")
        lines.append(
            "Every side any record shows falls into exactly one role: "
            "`origin` (the fire's own free choice of venting side) and "
            "`extra_free` (F3+'s single additional corner, when the "
            "`rng.randint(1,2)` draw comes up 1) are the only two roles "
            "where `entry_for_plan_fire` has an actual choice to make -- "
            "and it ALREADY scores all candidates before choosing in both "
            "(`urban_fire_spread.entry_for_plan_fire`: `max(_SIDE_RING, "
            "key=street_score)` for `origin`, `max(nb, key=street_score)` "
            "for `extra_free`). `forced_entry` (a non-origin record's own "
            "entry side, which must face whichever neighbour lit it -- "
            "contagion realism, never re-ranked by design) and "
            "`extra_spread` (F3+'s second corner, taken automatically when "
            "the same draw comes up 2, per the 2026-08-31 \"needs to look "
            "like more than one side\" policy) have NO choice to make at "
            "all, by design -- a low score there is not a scoring failure, "
            "it is the required trade-off.\n")
        lines.append(f"**n_mismatches: {audit['n_mismatches']}** (0 expected -- "
                    "a real bug in `entry_for_plan_fire` if this is ever "
                    "nonzero).\n")
        lines.append(f"* free-choice share (`origin` + `extra_free` roles, "
                    f"where scoring genuinely decides the outcome): "
                    f"**{audit['free_choice_share']:.2f}** "
                    f"({audit['free_choice_positive']}/{audit['free_choice_checked']})\n")
        lines.append(f"* forced/spread share (`forced_entry` + `extra_spread` "
                    f"roles, where the side is fixed regardless of score): "
                    f"**{audit['forced_or_spread_share']:.2f}** "
                    f"({audit['forced_or_spread_positive']}/{audit['forced_or_spread_checked']})\n")
        lines.append("\n<details><summary>Per-record side audit "
                    "({0} rows)</summary>\n".format(len(audit["rows"])))
        lines.append("\n| i | role | side | score | best available | reason |")
        lines.append("|---|---|---|---|---|---|")
        for row in audit["rows"]:
            side_s = row["side"] if not isinstance(row["side"], list) else ",".join(row["side"])
            score_s = (f"{row['score']:.1f}" if not isinstance(row["score"], list)
                      else ",".join(f"{s:.1f}" for s in row["score"]))
            best_s = (row["best_available"] if row["best_available"] is not None
                     else "-")
            lines.append(f"| {row['i']} | {row['role']} | {side_s} | {score_s} "
                        f"| {best_s} | {row['reason']} |")
        lines.append("\n</details>\n")

    lines.append("## Checks\n")
    lines.append("| check | result | detail |")
    lines.append("|---|---|---|")
    for name, (ok, detail) in checks.items():
        lines.append(f"| {name} | {'PASS' if ok else 'FAIL'} | "
                    f"`{json.dumps(detail)}` |")
    lines.append(f"| determinism | {'PASS' if det_ok else 'FAIL'} | "
                f"`{json.dumps(det_detail)}` |")
    lines.append("")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dump", help="placements dump JSON (FC_DUMP output)")
    ap.add_argument("--seeds", default=None,
                    help="comma-separated ignition seeds, e.g. 43,35,0 -- "
                         "required unless --auto")
    ap.add_argument("--auto", action="store_true",
                    help="search for a good seed combination instead of "
                         "taking --seeds explicitly (see auto_select_seeds)")
    ap.add_argument("--sweep-max", type=int, default=500,
                    help="--auto only: seeds [0, sweep-max) to sweep")
    ap.add_argument("--seed-start", type=int, default=0,
                    help="--auto only: first seed in the sweep range")
    ap.add_argument("--target-min", type=int, default=20,
                    help="--auto only: minimum acceptable union size")
    ap.add_argument("--target-pref", type=int, default=26,
                    help="--auto only: preferred union size to stop growing at")
    ap.add_argument("--n", type=int, default=40,
                    help="per-seed max_burnt cap (fire_city_dry_run's --n)")
    ap.add_argument("--collapse", type=int, default=1,
                    help="per-seed target F5c before the union-wide roof budget")
    ap.add_argument("--roof-collapse-max", type=int, default=2,
                    help="FINAL union-wide cap on F5c/F6 records (re-applied "
                         "across the whole union, not just per seed)")
    ap.add_argument("--adjacency-m", type=float, default=25.0,
                    help="visual-clumping radius for the concentration metric")
    ap.add_argument("--max-iters", type=int, default=8,
                    help="footprint-clean / cross-seed-clean fixed-point cap")
    ap.add_argument("--out", default=None)
    ap.add_argument("--md", default=None)
    ap.add_argument("--note-extra", default="",
                    help="free-text appended to the manifest's own \"note\" "
                         "field -- e.g. why a particular seed was chosen")
    ap.add_argument("--before", default=None,
                    help="path to a PRIOR union JSON to diff histograms/"
                         "concentration against in the report (e.g. the "
                         "file --out is about to overwrite)")
    ap.add_argument("--max-records", "--subset", dest="max_records", type=int,
                    default=None,
                    help="ALSO write a VRAM-budget subset manifest (see "
                         "build_subset) capped to this many records, next "
                         "to --out -- e.g. --max-records 32 on --out .../"
                         "fire_city_500m.json writes .../fire_city_500m_32."
                         "json alongside it")
    ap.add_argument("--subset-out", default=None,
                    help="override the subset JSON path (default: <out "
                         "stem>_<max-records>.json)")
    ap.add_argument("--subset-md", default=None,
                    help="override the subset markdown report path")
    ap.add_argument("--street-bonus-m", type=float,
                    default=STREET_BONUS_M_DEFAULT,
                    help="build_subset: centroid-distance discount (metres) "
                         "for a record with a street-facing chosen side")
    ap.add_argument("--tower-bonus-coef", type=float,
                    default=TOWER_BONUS_H_COEF_DEFAULT,
                    help="build_subset: centroid-distance discount (metres) "
                         "per metre of height for a tower/highrise record")
    ap.add_argument("--profile", choices=sorted(PROFILES.keys()), default=None,
                    help="baseline intensity-ladder profile (see "
                         "scene_gen/_plans/baseline_fire_ladder.md and this "
                         "module's own PROFILES table) -- computes --n / "
                         "solve-time --roof-collapse-max / --collapse and a "
                         "rebalance_severity target from THIS dump's own "
                         "census(), dump-parameterized so the same profile "
                         "name scales correctly from the 500 m stand-in to a "
                         "real 1 km dump. --seeds still selects an explicit "
                         "seed combination if given; otherwise seeds are "
                         "found the same way --auto does (--sweep-max/"
                         "--seed-start still apply). --n/--collapse/--roof-"
                         "collapse-max are IGNORED when --profile is given "
                         "(the profile computes its own) -- use --severity-"
                         "target on top for a one-off shaping override.")
    ap.add_argument("--severity-target", default=None,
                    help="rebalance_severity target as 'KEY:VALUE,...', e.g. "
                         "'F4:11,F5c:4,F6:1,collapse_visible:0.18' -- an int "
                         "value is an ABSOLUTE count, a float is a SHARE of "
                         "the union's n_achieved, and 'origin_only' (F5 "
                         "only) means exactly 1. Applied AFTER the union/"
                         "budget steps (and after --profile's own shaping, "
                         "if given, as an additional pass on top) -- see "
                         "resolve_severity_target / _parse_severity_spec.")
    args = ap.parse_args()

    overall_census, per_block_census = census(args.dump)
    print(f"[fire_city_union] census: {overall_census['n_house']} house, "
         f"{overall_census['n_burnable']} burnable, "
         f"{overall_census['n_refused']} refused")

    before = None
    if args.before and os.path.isfile(args.before):
        with open(args.before) as fh:
            before_union = json.load(fh)
        before = {
            "histograms": histograms(before_union.get("records") or []),
            "concentration": concentration_metrics(
                before_union.get("records") or [], adjacency_m=args.adjacency_m),
            "street_facing": street_facing_metrics(
                args.dump, before_union.get("records") or []),
        }
    elif args.out and os.path.isfile(args.out):
        with open(args.out) as fh:
            before_union = json.load(fh)
        before = {
            "histograms": histograms(before_union.get("records") or []),
            "concentration": concentration_metrics(
                before_union.get("records") or [], adjacency_m=args.adjacency_m),
            "street_facing": street_facing_metrics(
                args.dump, before_union.get("records") or []),
        }

    if args.profile:
        seeds_explicit = None
        if args.seeds:
            seeds_explicit = [int(s) for s in args.seeds.split(",")
                              if s.strip() != ""]
        (union, metrics, per_seed, checks, det_ok, det_detail, _profile,
         severity_target, sev_diff) = apply_profile(
            args.dump, args.profile, seeds=seeds_explicit,
            sweep_max=args.sweep_max, seed_start=args.seed_start,
            adjacency_m=args.adjacency_m, max_iters=args.max_iters,
            note_extra=args.note_extra)
        seeds = union["seeds"]
        print(f"[fire_city_union] profile {args.profile}: severity_target="
             f"{severity_target}, {len(sev_diff)} record(s) relevelled by "
             f"rebalance_severity")
    else:
        if args.auto:
            seeds, notes = auto_select_seeds(
                args.dump, sweep_max=args.sweep_max, n=args.n,
                collapse=args.collapse, roof_collapse_max=args.roof_collapse_max,
                target_min=args.target_min, target_pref=args.target_pref,
                adjacency_m=args.adjacency_m, seed_start=args.seed_start)
            for note in notes:
                print(f"[fire_city_union] auto: {note}")
        else:
            if not args.seeds:
                ap.error("--seeds is required unless --auto or --profile is given")
            seeds = [int(s) for s in args.seeds.split(",") if s.strip() != ""]

        union, metrics, per_seed, checks, det_ok, det_detail = build_union(
            args.dump, seeds, n=args.n, collapse=args.collapse,
            roof_collapse_max=args.roof_collapse_max,
            adjacency_m=args.adjacency_m, max_iters=args.max_iters,
            note_extra=args.note_extra)

    if args.severity_target:
        # standalone (no --profile), or an additional shaping pass ON TOP OF
        # the profile's own target -- see the flag's own help text. Both
        # `rebalance_severity` passes are individually deterministic (no
        # unseeded random() call -- its own docstring), so the composition
        # is deterministic by construction; re-verified below cheaply, on a
        # SNAPSHOT of the records from just before this override (never by
        # re-running the whole solve again) -- the solve side already got
        # its own full `check_determinism` pass, inside `build_union`/
        # `apply_profile` above, before this override was ever applied.
        # Redoing THAT pass here too (an early version of this code did,
        # via a closure that called `apply_profile` again) multiplies an
        # already-expensive `--auto` search by another 3x for no extra
        # coverage -- `check_determinism` itself already calls `run_fn`
        # three times, so a `run_fn` that redoes the full pipeline burns a
        # ~90 s `--profile` search NINE times for one `--severity-target`
        # invocation.
        import copy as _copy
        pre_override_records = _copy.deepcopy(union["records"])
        pre_override_refused = union["refused"]

        spec = _parse_severity_spec(args.severity_target)
        n_achieved = union["n_achieved"]
        extra_target = resolve_severity_target(spec, n_achieved)
        extra_diff = rebalance_severity(union["records"], args.dump,
                                        extra_target, min_side_score=0.0)
        union["records"].sort(key=lambda r: r["i"])
        from disaster import urban_fire_spread as ufs
        union["roof_outcome_count"] = sum(
            1 for r in union["records"] if r["level"] in ufs.ROOF_LEVELS)
        union["note"] = (union["note"] + " --severity-target override "
                         "{0}, {1} record(s) relevelled.".format(
                             extra_target, len(extra_diff)))
        print(f"[fire_city_union] --severity-target override: "
             f"{extra_target}, {len(extra_diff)} record(s) relevelled")

        checks = fdr.run_all_checks(union)
        metrics = {
            "concentration": concentration_metrics(union["records"], adjacency_m=args.adjacency_m),
            "street_facing": street_facing_metrics(args.dump, union["records"]),
            "street_facing_audit": street_facing_audit(args.dump, union["records"]),
            "histograms": histograms(union["records"]),
        }

        def _run_fn_sev(_s):
            recs2 = _copy.deepcopy(pre_override_records)
            rebalance_severity(recs2, args.dump, extra_target, min_side_score=0.0)
            recs2.sort(key=lambda r: r["i"])
            return {"records": recs2, "refused": pre_override_refused}

        det_ok, det_detail = fdr.check_determinism(int(seeds[0]), _run_fn_sev)

    out_json = args.out or os.path.join(
        _SCENE_GEN_DIR, "_plans",
        "fire_city_union_{0}.json".format("_".join(str(s) for s in seeds)))
    out_md = args.md or (os.path.splitext(out_json)[0] + "_report.md")
    os.makedirs(os.path.dirname(out_json), exist_ok=True)
    with open(out_json, "w") as fh:
        json.dump(union, fh, indent=1)
    print(f"[fire_city_union] wrote {out_json}")
    with open(out_md, "w") as fh:
        fh.write(_format_markdown(args.dump, union, metrics, per_seed, checks,
                                  det_ok, det_detail, overall_census,
                                  per_block_census, before=before))
    print(f"[fire_city_union] wrote {out_md}")

    print(f"\n[fire_city_union] seeds={union['seeds']} "
         f"n_achieved={union['n_achieved']} origins={union['origins']} "
         f"roof_outcome_count={union['roof_outcome_count']}")
    for name, (ok, detail) in checks.items():
        print(f"  {name:<20} {'PASS' if ok else 'FAIL'}  {detail}")
    print(f"  {'determinism':<20} {'PASS' if det_ok else 'FAIL'}  {det_detail}")
    print(f"  concentration: adjacency_share={metrics['concentration']['adjacency_share']:.2f} "
         f"n_components={metrics['concentration']['n_components']} "
         f"street_facing_share={metrics['street_facing']['share']:.2f}")

    all_ok = det_ok and all(ok for ok, _ in checks.values())

    if args.max_records is not None:
        subset, dropped = build_subset(
            args.dump, union, args.max_records, adjacency_m=args.adjacency_m,
            street_bonus_m=args.street_bonus_m,
            tower_bonus_coef=args.tower_bonus_coef)
        stem_ok, stem_detail = verify_stem_equality(union, subset)
        print(f"[fire_city_union] subset stem verification: "
             f"{'PASS' if stem_ok else 'FAIL'} {stem_detail}")

        subset_metrics = {
            "concentration": concentration_metrics(subset["records"], adjacency_m=args.adjacency_m),
            "street_facing": street_facing_metrics(args.dump, subset["records"]),
            "street_facing_audit": street_facing_audit(args.dump, subset["records"]),
            "histograms": histograms(subset["records"]),
        }
        subset_checks = fdr.run_all_checks(subset)

        def _subset_run_fn(_s):
            s2, _d2 = build_subset(
                args.dump, union, args.max_records, adjacency_m=args.adjacency_m,
                street_bonus_m=args.street_bonus_m,
                tower_bonus_coef=args.tower_bonus_coef)
            return {"records": s2["records"], "refused": s2["refused"]}
        subset_det_ok, subset_det_detail = fdr.check_determinism(
            int(subset["seed"]), _subset_run_fn)

        subset_before = {"histograms": metrics["histograms"],
                         "concentration": metrics["concentration"],
                         "street_facing": metrics["street_facing"]}

        subset_out = args.subset_out or (
            os.path.splitext(out_json)[0] + "_{0}.json".format(args.max_records))
        subset_md = args.subset_md or (os.path.splitext(subset_out)[0] + "_report.md")
        with open(subset_out, "w") as fh:
            json.dump(subset, fh, indent=1)
        print(f"[fire_city_union] wrote {subset_out}")
        with open(subset_md, "w") as fh:
            fh.write(_format_markdown(args.dump, subset, subset_metrics, per_seed,
                                      subset_checks, subset_det_ok, subset_det_detail,
                                      overall_census, per_block_census,
                                      before=subset_before))
        print(f"[fire_city_union] wrote {subset_md}")

        print(f"\n[fire_city_union subset] requested_max_records={args.max_records} "
             f"n_achieved={subset['n_achieved']} n_dropped={len(dropped)} "
             f"stem_verification={'PASS' if stem_ok else 'FAIL'}")
        for name, (ok, detail) in subset_checks.items():
            print(f"  {name:<20} {'PASS' if ok else 'FAIL'}  {detail}")
        print(f"  {'determinism':<20} {'PASS' if subset_det_ok else 'FAIL'}  {subset_det_detail}")
        print(f"  concentration: adjacency_share="
             f"{subset_metrics['concentration']['adjacency_share']:.2f} "
             f"n_components={subset_metrics['concentration']['n_components']} "
             f"street_facing_share={subset_metrics['street_facing']['share']:.2f}")

        subset_all_ok = (stem_ok and subset_det_ok
                        and all(ok for ok, _ in subset_checks.values()))
        all_ok = all_ok and subset_all_ok

    if not all_ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
