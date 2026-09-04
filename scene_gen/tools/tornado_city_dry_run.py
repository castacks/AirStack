#!/usr/bin/env python3
"""
tornado_city_dry_run.py — HOST-SIDE (no Isaac Sim, no docker, no Nucleus) dry
run of a CITY-SCALE urban tornado: the REAL `downtown_fire_1500`-family city
DUMP (typologies, blocks, real building footprints, the same one the fire
pipeline already built and gated) + `disaster.tornado`'s track field +
`disaster.tornado_city`'s damageable-set/level/height-class helpers,
producing `scene_gen/_plans/tornado_city_<seed>.json` and a markdown report —
the offline proof called for by `scene_gen/_plans/urban_tornado_plan.md` §4
(stream C's own file).

Modelled directly on `tools/fire_city_dry_run.py` — same shape (compile the
preset, load a REAL Kit-built placements dump, gate every house placement,
build a manifest, run numbered checks, write JSON + markdown + a plan PNG),
same warning repeated here because it matters just as much: A HOST-SIDE
LAYOUT BUILD IS A SIZE SOURCE, NEVER THE LAYOUT ITSELF. This tool has no
"rebuild the layout host-side" mode at all (unlike `fire_city_dry_run.py`'s
`run_dry` vs `run_dry_from_dump`) — the urban tornado dry run ALWAYS reads a
`fire_city_placements_dump.v1` dump (`--dump`, default resolved from
`--preset` via `PRESET_DUMP` below) via `fire_city_dry_run.
load_placements_dump` (imported, not reimplemented — see that module's own
docstring for the exact schema and the padded-non-house-placeholder trick a
reimplementation would have to get right a second time).

WHY THE SAME DUMP THE FIRE PIPELINE BUILT, NOT A NEW ONE — the dataset's
urban cells share LAYOUTS across disasters (plan §4/§5): `downtown_tornado_
1500{,_lvl2,_lvl3}.yaml` are thin copies of the matching `downtown_fire_1500*
.yaml` preset with the SAME `seed`/`region_m`, so `_plans/city_placements_
downtown_fire_1500{,_lvl2,_lvl3}.json` (already on disk, already Kit-built)
is the correct, matching dump for each tornado level too — see those
presets' own header comments.

THE MANIFEST IS BUILT OVER THE WHOLE 1500 m PLATE, NOT THE 1 KM CROP WINDOW
-- exactly like the fire dry run's own manifest (`downtown_fire_1500.yaml`'s
`epicenter: [0.0, 0.0]` is the plate centre, and the crop only happens LATER,
at assembly time, via `FC_CROP_WINDOW`). The 1 km window
(`LEVEL_WINDOW_CENTRE` below, mirroring `tools/baseline_layouts.LEVELS` —
copied rather than imported, since that module is not this stream's file and
other sessions may be editing this repo concurrently) is used ONLY for
report check 1 (corridor coverage) and the plan PNG's outline: the manifest
itself, and checks 2-7, run over every house placement in the dump.

GATES RUN FOR EVERY HOUSE PLACEMENT, REGARDLESS OF INTENSITY — the same
"a gate failure is a firebreak, evaluated whether or not the disaster ever
reaches that building" discipline `fire_city_dry_run.gather_burnable` already
follows for `urban_fire_city.burnable()`. `disaster.tornado_city.damageable`
does not know about intensity at all (see its own docstring); this tool
samples `disaster.tornado.intensity_field` separately and combines the two
per building. A building that clears all four gates but draws `"T0"` is
simply not recorded anywhere (not a manifest record, not `refused` — the
same "outside the corridor is T0 and is not a record" rule plan §2.3 gate 5
states) — accounted for instead via the manifest's own `n_t0_gate_passed`
counter, which check 7 (refused-list completeness) reads rather than
re-deriving it from a placement list `--check-only` mode does not have.

WIND: `disaster.tornado.wind_at` LANDED (stream P) while this tool was being
written and is used directly — `hasattr(tn, "wind_at")` still gates the call
against a FALLBACK (`{"bearing_deg": heading_deg, "speed_frac": i,
"cross_frac": None, "over": False}`, identical for every building), kept as
a guard for a stale/half-landed sibling module rather than deleted, but it
has not fired against the real `disaster.tornado` in any run this tool has
made — every manifest this tool wrote carries `"wind_at_available": true`
and check 5 (wind sides) is measured against the REAL model. See that
check's own docstring for what it would report if the fallback ever did
fire (a 0 deg left/right difference, an honest FAIL, not a faked pass).

Run:

    cd scene_gen && python3 tools/tornado_city_dry_run.py \\
        --preset downtown_tornado_1500

    cd scene_gen && python3 tools/tornado_city_dry_run.py --all-levels
"""
import argparse
import hashlib
import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_HERE)
for _p in (_HERE, _SCENE_GEN_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                             # noqa: E402

import compile_disaster                                        # noqa: E402
import fire_city_dry_run as fcdr                                # noqa: E402
from disaster import tornado as tn                              # noqa: E402
from disaster import tornado_city as tc                         # noqa: E402
from disaster import tornado_collapse as tcol                   # noqa: E402
from disaster import urban_fire_city as ufc                     # noqa: E402

#: `tornado_urban.t_facade_collapse`'s own `i >= 0.82` eligibility floor,
#: mirrored here (never imported as a hard dependency — this tool does not
#: otherwise touch `tornado_urban` at all) so `check_r11_real_collapse_
#: classes` below can report ELIGIBILITY for that recipe without this dry
#: run needing a piece grid, which it does not have (see that check's own
#: docstring for the "eligible, not verified-fired" honesty limit). Read
#: LIVE off `tornado_urban._FACADE_COLLAPSE_MIN_I` when that module is
#: importable, so a future threshold change there is picked up with no
#: edit here; falls back to the documented value otherwise.
try:
    from disaster import tornado_urban as _tu
    FACADE_COLLAPSE_MIN_I = float(getattr(_tu, "_FACADE_COLLAPSE_MIN_I", 0.82))
except ImportError:
    FACADE_COLLAPSE_MIN_I = 0.82

# ---------------------------------------------------------------------------
# level <-> preset <-> dump <-> crop-window table.
#
# `LEVEL_WINDOW_CENTRE`/`WINDOW_SIZE_M` are COPIED from `tools/
# baseline_layouts.LEVELS`/`WINDOW_SIZE_M` (the fire pipeline's own committed
# (seed, window) table), not imported: that module is not one of this
# stream's files (`_plans/urban_tornado_plan.md` §5's ownership table) and
# other sessions edit this repo concurrently — a copy pinned to the exact
# values that table already committed is safer than a live import of a file
# this stream does not own. If `baseline_layouts.LEVELS` ever changes, this
# table must be updated to match by hand.
# ---------------------------------------------------------------------------
WINDOW_SIZE_M = 1000.0
LEVEL_WINDOW_CENTRE = {1: (-180.0, 180.0), 2: (100.0, 150.0), 3: (20.0, 230.0)}

PRESET_LEVEL = {
    "downtown_tornado_1500": 1,
    "downtown_tornado_1500_lvl2": 2,
    "downtown_tornado_1500_lvl3": 3,
}
#: default `--dump` per preset — the matching FIRE level's own real Kit dump
#: (see the module docstring's "why the same dump" section).
PRESET_DUMP = {
    "downtown_tornado_1500": "city_placements_downtown_fire_1500.json",
    "downtown_tornado_1500_lvl2": "city_placements_downtown_fire_1500_lvl2.json",
    "downtown_tornado_1500_lvl3": "city_placements_downtown_fire_1500_lvl3.json",
}
ALL_PRESETS = ("downtown_tornado_1500", "downtown_tornado_1500_lvl2",
              "downtown_tornado_1500_lvl3")

#: ROUND 2 (`_plans/urban_tornado_plan.md` §7) — presets with NO matching
#: real Kit dump (the fire pipeline never built a `downtown_fire_bench_500`
#: dump; this preset is new to round 2, not a thin copy of a 1500 m level).
#: `solve()` below builds their layout HOST-SIDE via `fire_city_dry_run.
#: build_layout` instead of reading a dump file — see that function's own
#: docstring for the "size source, never the layout itself" caveat, which
#: applies here exactly as it does to every other host-side rebuild in this
#: codebase. Not auto-detected from "no PRESET_DUMP entry" so that a preset
#: name typo fails loudly (`run_one`'s `--dump` requirement) rather than
#: silently host-building something nobody asked to trust that way.
BENCH_PRESETS = ("downtown_tornado_bench_500",)


def window_for_preset(preset, region=None):
    """`(level, (wx0, wy0, wx1, wy1))`.

    Two cases:

      1. `preset` is one of the three 1500 m levels (`PRESET_LEVEL`) — the
         level's own 1 km CROP WINDOW, unchanged from round 1.
      2. otherwise (round 2's bench preset, or any future one-off preset
         this table does not name) — `region` (the compiled preset's own
         `region_m`, `(x0, y0, x1, y1)`) IS the window: a preset with no
         separate crop concept reads its own whole plate as "the corridor's
         window" for check 1's purposes, which is the correct generalisation
         (`fire_city_dry_run.load_placements_dump`'s "the manifest is built
         over the whole plate, the window only narrows the CHECK" pattern,
         applied to a plate that has no narrower window at all). Returns
         `(None, None)` only when `region` is not given either (a caller
         asking about window geometry with no way to answer it).
    """
    level = PRESET_LEVEL.get(preset)
    if level is not None:
        cx, cy = LEVEL_WINDOW_CENTRE[level]
        half = WINDOW_SIZE_M / 2.0
        return level, (cx - half, cy - half, cx + half, cy + half)
    if region is not None:
        return None, tuple(float(v) for v in region)
    return None, None


def default_dump_path(preset):
    name = PRESET_DUMP.get(preset)
    if name is None:
        return None
    return os.path.join(_SCENE_GEN_DIR, "_plans", name)


# ---------------------------------------------------------------------------
# Stage 1: load the dump (fire_city_dry_run.load_placements_dump, reused
# verbatim -- see the module docstring's "modelled directly on" note).
# ---------------------------------------------------------------------------
def load_dump(dump_path):
    """`(layout, placements, dump_seed, dump_preset, sha256)` — the `config`
    `fire_city_dry_run.load_placements_dump` also returns is dropped here: it
    exists only for a `same_art` placement's `prefer_type` scoring, which
    this tool does not wire up (`disaster.tornado_city.damageable` calls
    `kit_substitute.route` with `btype=tornado_city.btype_for(usd, H)`
    instead — a real, if slightly different, preference signal, computed
    from measured construction type rather than the district's own material
    key)."""
    _config, layout, placements, dump_seed, dump_preset, sha256 = \
        fcdr.load_placements_dump(dump_path)
    return layout, placements, dump_seed, dump_preset, sha256


# ---------------------------------------------------------------------------
# Stage 2: the centreline, sampled and clipped to the plate — used by check 1
# (corridor coverage) and the plan PNG.
# ---------------------------------------------------------------------------
def centreline_points(tcfg, plate, n=4000):
    """`[(x, y), ...]`, `n` points at a uniform step in the track's own
    `along` coordinate, covering the full extent of `plate = (x0, y0, x1,
    y1)` and a margin either side (so a track that enters/exits the plate
    inside the sampled range is not clipped at the sample boundary itself).

    THE SAME "invert the meander via `to_track`" TRICK `tools/tornado_png.py`
    draws its own centreline with (see that file for the derivation) — kept
    local rather than imported because `tornado_png.py` is a `__main__`-style
    plotting tool with a hard `matplotlib` dependency this module does not
    otherwise need until its own plotting section.

    A UNIFORM STEP IN `along` IS AN APPROXIMATION OF ARC LENGTH, not an exact
    one — the wobble is a shear in `along` (see `tornado.from_track`'s own
    docstring), so true arc length differs from a uniform `along` step by the
    wobble's own (small) derivative. Good enough for a coverage FRACTION
    (this tool's only use for it): the error is the same order on both the
    in-window and in-plate counts, so it drops out of the ratio to first
    order.
    """
    x0, y0, x1, y1 = plate
    to_track, (ux, uy), (vx, vy) = tn.frame(tcfg)
    ox, oy = (float(tcfg["origin_m"][0]), float(tcfg["origin_m"][1]))
    reach = math.hypot(x1 - x0, y1 - y0)
    pts = []
    for k in range(n):
        a = -reach + 2.0 * reach * k / float(n - 1)
        probe = (ox + ux * a, oy + uy * a)
        _a2, c2 = to_track(*probe)
        off = -c2
        pts.append((probe[0] + vx * off, probe[1] + vy * off))
    return pts


def track_window_coverage(tcfg, plate, window, n=4000):
    """`(frac_in_window_of_in_plate, n_in_plate, n_in_window)` — of the
    centreline's own length INSIDE THE PLATE (never the unbounded infinite
    line), what fraction is also inside `window`. This is check 1's "the
    track crosses the window (>= 60% of its length inside)" number."""
    x0, y0, x1, y1 = plate
    wx0, wy0, wx1, wy1 = window
    pts = centreline_points(tcfg, plate, n=n)
    in_plate = [(x, y) for (x, y) in pts if x0 <= x <= x1 and y0 <= y <= y1]
    if not in_plate:
        return 0.0, 0, 0
    in_window = [(x, y) for (x, y) in in_plate
                if wx0 <= x <= wx1 and wy0 <= y <= wy1]
    frac = len(in_window) / float(len(in_plate))
    return frac, len(in_plate), len(in_window)


# ---------------------------------------------------------------------------
# Stage 3: gate + intensity + level, per house placement.
# ---------------------------------------------------------------------------
def solve(preset, dump_path=None, seed=None, max_h=None, verbose=True):
    """Runs the whole pipeline once. Returns `(manifest, report_extras)`.

    `seed` overrides the SPREAD... no spread here, but the same convention
    (`fire_city_dry_run.run_dry_from_dump`'s own `seed` argument): `None`
    means "use the dump's own seed" (`dump_seed`), matching `--seed`'s
    documented default. The LAYOUT is always exactly what the dump recorded,
    regardless of `seed` — only the intensity-field noise draw and the
    per-building level jitter use it.

    `dump_path=None` (round 2, plan §7): for a preset in `BENCH_PRESETS`
    (`downtown_tornado_bench_500` — no matching real Kit dump exists, unlike
    the three 1500 m levels), the layout is built HOST-SIDE via
    `fire_city_dry_run.build_layout` instead of read from a file. THE SAME
    CAVEAT AS EVERY OTHER HOST-SIDE REBUILD IN THIS CODEBASE APPLIES: a
    host-side layout is a SIZE source, never the layout itself — the offline
    gate this round produces is legitimate BECAUSE it is offline (plan §0
    item 3, "nothing launches Isaac Sim this round"), but the Kit build
    remains the final truth once this scene is actually built, exactly as
    the `build-urban-tornado-scenes` skill's module docstring already states
    for the 1500 m dumps' own host-side SIZE lookups. For any OTHER preset,
    `dump_path=None` raises rather than silently guessing.
    """
    cfg = compile_disaster.load_scene_config(preset)
    tcfg = tn.resolve_cfg(cfg)
    region = tuple(float(v) for v in cfg["layout"]["region_m"])
    plate = (-region[0] / 2.0, -region[1] / 2.0, region[0] / 2.0, region[1] / 2.0)

    resolver = None
    wdh_cache = None
    if dump_path is not None:
        layout, placements, dump_seed, dump_preset, sha256 = load_dump(dump_path)
    elif preset in BENCH_PRESETS:
        if verbose:
            print(f"[dry_run] {preset}: no real Kit dump on this stream — "
                 f"building the layout HOST-SIDE (fire_city_dry_run."
                 f"build_layout). A HOST-SIDE LAYOUT IS A SIZE SOURCE, NEVER "
                 f"THE LAYOUT ITSELF — the Kit build remains the final truth.")
        host_cfg, layout, placements, resolver = fcdr.build_layout(preset, seed=seed)
        dump_seed = int(host_cfg.get("seed", 0))
        dump_preset = preset
        sha256 = None
        # A host-built placement (unlike a REAL Kit dump — see `gather_
        # burnable`'s own docstring) carries no `W`/`D`/`H` at all, only
        # `usd`/`scale`/`axis_up` — the layout generator never measures a
        # footprint onto the placement dict, it hands the resolver's own
        # bbox straight to `apply_placements`. `fire_city_dry_run._measure_
        # wdh`/`_gac_dtc_cache` are the SAME lookup `gather_burnable` uses
        # for its own host-build path (`resolver is not None` branch) —
        # reused verbatim, not re-derived, per this stream's own "do not
        # rebuild a size lookup a sibling tool already solved" discipline.
        wdh_cache = fcdr._gac_dtc_cache()
        from disaster import gac_fire as _gf
    else:
        _gf = None
        raise SystemExit(
            f"solve(): no dump given and {preset!r} is not in BENCH_PRESETS "
            f"(no host-build fallback) — pass --dump, or add this preset to "
            f"PRESET_DUMP if a real Kit dump exists for it")
    resolved_seed = int(seed) if seed is not None else int(dump_seed)

    level_num, window = window_for_preset(preset, region=plate)

    inten = tn.intensity_field(tcfg, plate, np.random.default_rng(resolved_seed + 23))

    has_wind_at = hasattr(tn, "wind_at")
    if not has_wind_at and verbose:
        print("[dry_run] wind_at missing -- heading used")

    max_h_eff = float(max_h) if max_h is not None else tc.TORNADO_MAX_H_M

    records, refused, t0_footprints = [], [], []
    n_house = 0
    n_t0_gate_passed = 0
    #: ROUND 2 (plan §7 R2's composition target) — the district TYPOLOGY of
    #: EVERY house placement, regardless of gate/level outcome. Deliberately
    #: separate from `records`/`refused` (which only ever carry T1+/gate-
    #: failed buildings): "the districts are rowhouse/lowrise/midrise
    #: dominant" is a statement about the WHOLE LAYOUT, not about which
    #: buildings the track happens to reach — a track placed differently
    #: next seed must not change what the composition check reports.
    typology_counts = {}
    n_protected_sky = 0
    for i, p in enumerate(placements):
        if p.get("category") != "house":
            continue
        n_house += 1
        usd = p.get("usd")
        x = float(p.get("x_m", 0.0))
        y = float(p.get("y_m", 0.0))
        yaw = float(p.get("yaw_deg", 0.0))
        if resolver is not None:
            W, D, H = fcdr._measure_wdh(usd, p, resolver, wdh_cache, _gf)
        else:
            W, D, H = float(p["W"]), float(p["D"]), float(p["H"])
        cell = p.get("cell") or p.get("prim_path")

        typology_all = ufc.typology_at(layout, x, y)
        # `None` (street/park/off-block) becomes the string key `"_none"` --
        # `json.dumps(..., sort_keys=True)` (check 6's determinism re-solve)
        # cannot sort a dict with a mixed `None`/`str` key set (`TypeError:
        # '<' not supported between instances of 'str' and 'NoneType'`), and
        # `typology_counts` is otherwise plain JSON.
        typology_key = typology_all if typology_all is not None else "_none"
        typology_counts[typology_key] = typology_counts.get(typology_key, 0) + 1

        i_val = float(inten(x, y))
        if has_wind_at:
            wind = tn.wind_at(tcfg, x, y)
        else:
            wind = {"bearing_deg": float(tcfg["heading_deg"]),
                   "speed_frac": i_val, "cross_frac": None, "over": False}

        ok, reason, route, kind, name, bakeable = tc.damageable(
            p, {usd: (W, D, H)}, max_h=max_h_eff)

        # Computed unconditionally (independent of `ok`/level) so refused AND
        # T0 buildings can carry it too -- the plan §7 R6 figure hatches
        # EVERY highrise/tower footprint, refused/T0 included (a height-
        # capped 312 m tower is still a skyscraper on the map, just a
        # pristine one), and `height_class` is what `--tune-track`'s own
        # hard constraint reads for a REFUSED building too (a refused tower
        # never becomes a record, so it costs nothing to omit here -- kept
        # anyway, for the figure and for a human auditing `refused` by eye).
        height_class = tc.height_class_for(H, typology_all)
        # ROUND 2, lead review (plan §7 R6 item 3): the R1-PROTECTED count
        # for the composition line/check 9 is `tc.is_protected_skyscraper`
        # (class `tower` OR H >= 75 m), NOT the typology-based `tower`/
        # `highrise` bucket `typology_counts` above already carries --
        # those answer a DIFFERENT question ("what did the district
        # generator zone this ground as") and must stay as they are for
        # the low/mid >= 70% target, but a `brick_midrise`-typology
        # building measuring 47-72 m is NOT a "skyscraper" by any EF-scale
        # reading (`_plans/urban_tornado_research.md` §1) even though its
        # H-band-fallback `height_class` used to read `highrise`. Tallied
        # over EVERY house placement (same population as `typology_counts`,
        # same `_none` exclusion at read time), independent of gate/level
        # outcome -- a pristine, height-capped supertall standing in the
        # core is still a "skyscraper in the plate" for this count.
        if tc.is_protected_skyscraper(height_class, H):
            n_protected_sky += 1

        level_rng = random.Random(resolved_seed * 1000003 + i)
        # R11 (§8c): an `industrial` record draws its OWN grade vocabulary
        # (`tornado_collapse.grade_for_intensity`: "partial"/"total"/`None`)
        # instead of the T0-T4 ladder every other kind uses — `None` is
        # that vocabulary's own "not a record" sentinel, folded into the
        # SAME `level in (None, "T0")` check below the T0-T4 ladder already
        # has, rather than overloading the string `"T0"` for a class that
        # is not on that ladder at all.
        if kind == "industrial":
            level = tcol.grade_for_intensity(i_val)
        else:
            level = tc.level_for_intensity(i_val, level_rng)

        if not ok:
            # `name`/`H` carried even on a refusal (`damageable()` resolves
            # `kind`/`name` BEFORE the height-cap gate runs — see that
            # function's own docstring) so a height-cap refusal can be
            # reported by name, not just by index -- see `cap_refused_in_
            # corridor` / the report's "Cap-refused buildings inside the
            # corridor" section (2026-09-01 lead review). `W`/`D`/`yaw`/
            # `height_class` are NEW this round (plan §7 R6) -- the figure
            # draws a refused building as a real footprint, not just an "x".
            refused.append({"i": i, "cell": cell, "usd": usd, "name": name,
                            "H": H, "W": W, "D": D, "yaw": yaw, "x": x, "y": y,
                            "height_class": height_class,
                            "intensity": round(i_val, 5), "level": level,
                            "reason": reason})
            continue

        if level in (None, "T0"):
            # `None` is `tornado_collapse.grade_for_intensity`'s own "not a
            # record" sentinel (i < 0.5, an industrial shed's own floor) --
            # folded into the SAME bucket "T0" already uses for every other
            # kind, per this branch's own comment below.
            n_t0_gate_passed += 1
            # ROUND 2 (plan §7 R6) -- the figure draws a T0 (pale grey)
            # footprint for every gate-passed-but-untouched building, which
            # `records` deliberately never carries (§2.6's "not a record"
            # rule, unchanged). A SEPARATE list, purely for rendering --
            # nothing downstream of the manifest reads `t0_footprints` as a
            # damage plan input.
            # `H` and `name` -- ADDED (lead review 2026-09-01): check 8's
            # fix needs `H` on every candidate item to apply `tc.
            # is_protected_skyscraper` (a T0 building can still be a
            # protected skyscraper standing pristine in the core -- the
            # exact defect this fix exists to catch), and `name` so a
            # violation is reportable by name, not just by coordinate.
            t0_footprints.append({"x": x, "y": y, "W": W, "D": D, "yaw": yaw,
                                  "H": H, "name": name,
                                  "height_class": height_class})
            continue

        # R11 (§8c): an industrial record's `btype` is its OWN kind, never
        # `tornado_city.btype_for`'s urm/rc/rc_glass guess -- that table
        # means nothing for a single merged-mesh shed with no sliced
        # element table to construction-type at all.
        btype = "industrial" if kind == "industrial" else tc.btype_for(usd, H)

        rec = tc.record(i, cell, usd, kind, name, x, y, yaw, W, D, H, btype,
                        height_class, i_val, level, wind, route, bakeable,
                        resolved_seed)
        rec["typology"] = typology_all
        if kind == "industrial" and level in tcol.GRADES:
            # A REAL `tornado_collapse.plan_industrial` run -- unlike the
            # sliced ladder (`tornado_urban.t_facade_collapse`), this class
            # needs only W/D/H/yaw/x/y, all of which the dry run already
            # has, so "did the recipe do something" is not just eligibility
            # here, it is a measured plan. Seeded off `i`, offset from
            # `level_rng`'s own seed so the two draws never correlate.
            ind_rng = random.Random(resolved_seed * 1000003 + i + 777)
            ind_plan = tcol.plan_industrial(W, D, H, yaw, x, y, level, wind,
                                            ind_rng)
            rec["industrial_stats"] = ind_plan["stats"]
        elif btype == "urm" and height_class == "lowrise" and level == "T4":
            # `tornado_urban.t_facade_collapse` ELIGIBILITY (not "fired" --
            # this dry run has no piece grid, so it cannot run the sliced
            # ladder at all; see `check_r11_real_collapse_classes`'s own
            # docstring for the honesty limit this implies). The three
            # gates knowable from a manifest record: urm, lowrise,
            # `intensity >= FACADE_COLLAPSE_MIN_I` -- the fourth gate
            # (<= 4 storeys) needs a real slice's own storey count, which
            # this dry run never measures.
            rec["facade_collapse_eligible"] = bool(i_val >= FACADE_COLLAPSE_MIN_I)
        records.append(rec)

    track_frac, n_in_plate, n_in_window = track_window_coverage(
        tcfg, plate, window, n=4000) if window else (None, 0, 0)

    n_house_in_window = 0
    n_t0_in_window = 0
    if window:
        wx0, wy0, wx1, wy1 = window
        for p in placements:
            if p.get("category") != "house":
                continue
            x, y = float(p.get("x_m", 0.0)), float(p.get("y_m", 0.0))
            if not (wx0 <= x <= wx1 and wy0 <= y <= wy1):
                continue
            n_house_in_window += 1
        in_window_recorded = sum(
            1 for r in records
            if wx0 <= r["x"] <= wx1 and wy0 <= r["y"] <= wy1)
        in_window_refused_t1 = sum(
            1 for r in refused
            if r.get("level") not in (None, "T0")
            and wx0 <= r["x"] <= wx1 and wy0 <= r["y"] <= wy1)
        n_t0_in_window = n_house_in_window - in_window_recorded - in_window_refused_t1

    manifest = {
        "schema": "tornado_city_manifest.v1",
        "preset": preset, "level": level_num, "seed": resolved_seed,
        "region_m": list(region), "window": list(window) if window else None,
        "tornado_cfg": dict(tcfg),
        "dump": {"path": (os.path.abspath(dump_path) if dump_path
                          else "<host-build:fire_city_dry_run.build_layout>"),
                "sha256": sha256, "preset": dump_preset, "seed": dump_seed},
        "n_house_placements": n_house,
        "n_t0_gate_passed": n_t0_gate_passed,
        "n_house_in_window": n_house_in_window,
        "n_t0_gate_passed_in_window": max(0, n_t0_in_window),
        "track_frac_in_window": track_frac,
        "n_track_samples_in_plate": n_in_plate,
        "n_track_samples_in_window": n_in_window,
        "wind_at_available": has_wind_at,
        "typology_counts": typology_counts,
        "n_protected_sky": n_protected_sky,
        "records": records,
        "refused": refused,
        "t0_footprints": t0_footprints,
    }

    extras = {"cfg": cfg, "tcfg": tcfg, "plate": plate, "window": window,
             "level": level_num, "layout": layout}
    return manifest, extras


# ---------------------------------------------------------------------------
# The seven §4 checks -- pure functions over a manifest dict (+ tcfg for
# check 5, recomputable from manifest["preset"] alone for --check-only).
# ---------------------------------------------------------------------------
def check_1_corridor_coverage(manifest):
    """8-30% of the buildings INSIDE the crop window are T1+; the track
    crosses the window (>= 60% of its length inside).

    ROUND 2: the 8-30% BAND IS NOT APPLIED when `window` is the preset's OWN
    WHOLE PLATE rather than a separate 1 km crop carved out of a larger one
    (`manifest.get("level") is None` — `window_for_preset`'s own signal,
    true for `BENCH_PRESETS` and any other one-off preset `PRESET_LEVEL`
    does not name). That band was calibrated for a small window inside a
    1500 m plate (`_plans/urban_tornado_C_notes.md`); on the bench preset
    the "window" IS the whole scene, so a much larger T1+ share is both
    expected and correct — R2's own bench wants a corridor that reads across
    most of the plate, not an 8-30% sliver of it. The coverage NUMBER is
    still reported (useful for the figure's title block either way); only
    the pass/fail band is skipped. The track-crosses-window requirement
    (>= 60% of centreline length inside) still applies unconditionally — a
    track that barely touches even its OWN plate is a broken track."""
    window = manifest.get("window")
    if window is None:
        return False, {"reason": "no crop window known for this preset"}
    whole_plate_window = manifest.get("level") is None
    wx0, wy0, wx1, wy1 = window
    n_total = manifest.get("n_house_in_window", 0)
    n_t1plus = sum(1 for r in manifest["records"]
                  if wx0 <= r["x"] <= wx1 and wy0 <= r["y"] <= wy1)
    n_t1plus += sum(1 for r in manifest["refused"]
                    if r.get("level") not in (None, "T0")
                    and wx0 <= r["x"] <= wx1 and wy0 <= r["y"] <= wy1)
    frac = (n_t1plus / float(n_total)) if n_total else 0.0
    track_frac = manifest.get("track_frac_in_window")
    coverage_ok = True if whole_plate_window else (0.08 <= frac <= 0.30)
    track_ok = track_frac is not None and track_frac >= 0.60
    ok = coverage_ok and track_ok
    return ok, {"n_buildings_in_window": n_total,
               "n_t1plus_in_window": n_t1plus,
               "frac_t1plus_in_window": round(frac, 4),
               "whole_plate_window": whole_plate_window,
               "coverage_in_band_8_30pct": coverage_ok,
               "track_frac_in_window": (round(track_frac, 4)
                                        if track_frac is not None else None),
               "track_frac_ge_60pct": track_ok}


def check_2_gradient(manifest):
    """No single level holds > 55% of the T1+ records; T3+T4 >= 15%."""
    recs = manifest["records"]
    n = len(recs)
    if n == 0:
        return False, {"reason": "no T1+ records at all"}
    from collections import Counter
    hist = Counter(r["level"] for r in recs)
    max_share = max(hist.values()) / float(n)
    t34_share = (hist.get("T3", 0) + hist.get("T4", 0)) / float(n)
    ok = max_share <= 0.55 and t34_share >= 0.15
    return ok, {"n_records": n, "histogram": dict(sorted(hist.items())),
               "max_level_share": round(max_share, 4),
               "t3_t4_share": round(t34_share, 4)}


def check_r1_skyscraper_exposure(manifest, tcfg=None):
    """Plan §7 R1's HARD check: every PROTECTED building's (`tc.
    is_protected_skyscraper` — class `tower` OR `H >= tc.
    SKYSCRAPER_PROTECTED_MIN_H_M`) RAW (un-jittered) intensity, sampled at
    its OWN FOOTPRINT'S CORNER nearest the track centreline, is `<= tc.
    SKYSCRAPER_MAX_I` (0.55 — at most T2 envelope damage). Uses `tc.
    skyscraper_exposure` directly (the SAME function `--tune-track`'s
    candidate scorer calls, so a candidate that passes the search cannot
    fail this check for a different reason).

    TWO LEAD-REVIEW FIXES from the first version of this check
    (2026-09-01), both load-bearing:

    1. **Scans `records` + `refused` + `t0_footprints` — EVERY skyscraper-
       class placement, not just T1+ records.** The first version only
       ever saw `manifest["records"]`, so a `TORNADO_MAX_H_M`-refused
       supertall (pristine BY CONSTRUCTION — the gate refuses it before any
       damage plan exists) could still stand dead in the corridor's core
       and this check would report a clean 0.32 off its unrelated T1+
       skyscrapers while missing it entirely. The user's rule ("skyscrapers
       can't be in the middle of the tornado path") is about WHERE these
       buildings physically stand relative to the corridor, independent of
       whether the ladder can currently touch them — a pristine 312 m
       tower standing in the eye of the storm reads as a bug regardless of
       whether it is refused, T0, or a T1+ record.
    2. **Samples the RAW field at each building's own OWN footprint's
       corner nearest the centreline, not its centre** — see `tc.
       skyscraper_exposure`'s own section docstring for the geometry (a
       large footprint's corner can sit well inside the corridor while its
       centre reads clear; this is what let `SM_Building_16`, 84.5 x
       56.9 m, through the first version's gate at i=0.32 while its real
       corner-nearest-centreline reading is 0.787).

    Re-derives `tcfg` from `manifest["preset"]` when not given (works
    under `--check-only`, the same convention `check_5_wind_sides` already
    uses) and rebuilds the intensity field at `manifest["seed"] + 23`
    (`solve()`'s own noise-draw seed) over the plate implied by `manifest[
    "region_m"]` — this needs the field itself, not just each record's own
    stored `intensity`, because a corner point is generally NOT any
    building's own `(x, y)` and so was never sampled at `solve()` time.
    """
    if tcfg is None:
        cfg = compile_disaster.load_scene_config(manifest["preset"])
        tcfg = tn.resolve_cfg(cfg)
    to_track, _u, _v = tn.frame(tcfg)

    region = manifest.get("region_m")
    w, h = float(region[0]), float(region[1])
    plate = (-w / 2.0, -h / 2.0, w / 2.0, h / 2.0)
    inten = tn.intensity_field(tcfg, plate,
                               np.random.default_rng(int(manifest["seed"]) + 23))

    items = (list(manifest["records"]) + list(manifest.get("refused") or [])
            + list(manifest.get("t0_footprints") or []))
    exposure = tc.skyscraper_exposure(items, inten, to_track=to_track)
    over = [e for e in exposure if e["i_raw"] > tc.SKYSCRAPER_MAX_I]
    ok = not over
    return ok, {"n_protected": len(exposure),
               "n_from_records": sum(1 for r in manifest["records"]
                                     if tc.is_protected_skyscraper(
                                         r.get("height_class"), r.get("H"))),
               "n_from_refused": sum(1 for r in (manifest.get("refused") or [])
                                     if tc.is_protected_skyscraper(
                                         r.get("height_class"), r.get("H"))),
               "n_from_t0": sum(1 for r in (manifest.get("t0_footprints") or [])
                                if tc.is_protected_skyscraper(
                                    r.get("height_class"), r.get("H"))),
               "n_over_cap": len(over),
               "max_i_raw": round(max((e["i_raw"] for e in exposure), default=0.0), 4),
               "cap": tc.SKYSCRAPER_MAX_I,
               "over": [{"name": e["name"], "height_class": e["height_class"],
                        "H": e["H"], "i_raw": round(e["i_raw"], 4)}
                       for e in over]}


#: plan §7 R2's own target, verbatim: ">= 70% of buildings in {rowhouse,
#: lowrise, midrise/brick_midrise} classes, tower+highrise <= 15%".
COMPOSITION_LOW_MID = frozenset(("rowhouse", "lowrise", "midrise", "brick_midrise"))
COMPOSITION_LOW_MID_MIN_FRAC = 0.70
COMPOSITION_SKY_MAX_FRAC = 0.15


def check_composition(manifest):
    """Plan §7 R2's composition target, measured over EVERY house placement
    in the layout.

    `frac_low_mid` is `manifest["typology_counts"]`'s own district
    TYPOLOGY sum (`urban_fire_city.typology_at`, the six-way scheme
    `districts.typologies` names: `rowhouse`/`lowrise`/`midrise`/
    `brick_midrise`/`tower`/`highrise`) — plan §7's own wording keys the
    LOW/MID side of the target to district typology ("Districts: rowhouse/
    lowrise/midrise DOMINANT"), the axis the preset's
    `overrides.districts.rings` actually controls.

    `frac_sky` is `manifest["n_protected_sky"]` — the R1-PROTECTED count
    (`tc.is_protected_skyscraper`: class `tower` OR measured `H >= tc.
    SKYSCRAPER_PROTECTED_MIN_H_M`), NOT the typology `tower`+`highrise`
    bucket the first version of this check used. LEAD REVIEW, 2026-09-01:
    those disagreed — a `brick_midrise`-TYPOLOGY building can measure
    47-72 m (the GAC pool's own range) and read `height_class ==
    "highrise"` purely through `height_class_for`'s coarse H-band fallback
    (its `highrise` floor is 45 m, nothing to do with the EF scale's own
    high-rise boundary — `_plans/urban_tornado_research.md` §1's DI 18/19
    tables put it at 20 storeys, ~75 m), so the OLD typology-based `n_sky`
    counted dozens of ordinary mid-rise buildings as "skyscrapers" while
    the figure's hatching (now fixed the same way, see `draw_png`) and
    check 8 both used a DIFFERENT, disagreeing definition. `n_protected_sky`
    is tallied once, in `solve()`'s own per-building loop, by the SAME
    predicate `skyscraper_exposure`/check 8 use, so this check, check 8 and
    the figure can never disagree with each other again.

    Both fractions share the SAME denominator, `n_zoned` (`typology_counts`
    minus `"_none"` — streets/parks/off-block): composition is a statement
    about ZONED building stock.
    """
    counts = manifest.get("typology_counts") or {}
    n_zoned = sum(v for k, v in counts.items() if k != "_none")
    n_low_mid = sum(v for k, v in counts.items() if k in COMPOSITION_LOW_MID)
    n_sky = int(manifest.get("n_protected_sky") or 0)
    frac_low_mid = (n_low_mid / float(n_zoned)) if n_zoned else 0.0
    frac_sky = (n_sky / float(n_zoned)) if n_zoned else 0.0
    ok = (n_zoned > 0 and frac_low_mid >= COMPOSITION_LOW_MID_MIN_FRAC
         and frac_sky <= COMPOSITION_SKY_MAX_FRAC)
    return ok, {"typology_counts": dict(sorted(counts.items())),
               "n_zoned": n_zoned,
               "n_low_mid": n_low_mid, "frac_low_mid": round(frac_low_mid, 4),
               "frac_low_mid_min": COMPOSITION_LOW_MID_MIN_FRAC,
               "n_sky_protected": n_sky, "frac_sky": round(frac_sky, 4),
               "frac_sky_max": COMPOSITION_SKY_MAX_FRAC}


def check_damage_capable_coverage(manifest):
    """>= 80% of T1+ records are damage-capable (`record["bakeable"]` —
    `kind in ("gac", "dtc")` unconditionally, `kind == "kit"` iff `disaster.
    tornado_kit` was importable when the manifest was solved, `kind ==
    "slice"` — AEC/standalone — never). Round-1 root cause #3: "13 of 20
    corridor records had NO damage path... so the corridor read intact"; the
    coverage number this check reads is exactly what would have caught it
    before the render, not after the user's review."""
    recs = manifest["records"]
    n = len(recs)
    if n == 0:
        return False, {"reason": "no T1+ records at all"}
    n_capable = sum(1 for r in recs if r.get("bakeable"))
    frac = n_capable / float(n)
    ok = frac >= 0.80
    by_kind = {}
    for r in recs:
        k = r.get("kind")
        by_kind.setdefault(k, [0, 0])
        by_kind[k][0] += 1
        if r.get("bakeable"):
            by_kind[k][1] += 1
    return ok, {"n_records": n, "n_capable": n_capable,
               "frac_capable": round(frac, 4), "frac_capable_min": 0.80,
               "by_kind": {k: {"n": v[0], "n_capable": v[1]}
                          for k, v in sorted(by_kind.items(),
                                             key=lambda kv: str(kv[0]))}}


#: plan §7's own words: "at peak >= 0.9: >= 3 T4 records on lowrise/midrise
#: urm — the 'I don't see partial collapses' fix, via exposure not force".
PARTIAL_COLLAPSE_MIN_PEAK = 0.90
PARTIAL_COLLAPSE_MIN_N = 3


def check_partial_collapse_presence(manifest):
    """REPLACES round 1's `check_3_core_reaches_fabric` for this round —
    that check's EF3/EF4+ bands REQUIRED >= 1 / >= 3 T4 records on a
    `highrise`/`tower` HEIGHT CLASS, which plan §7 R1 now forbids outright
    (a highrise/tower can never sample `i >= 0.74`, T4's own lower cut,
    while also honouring `i <= 0.55` — the two requirements are mutually
    exclusive by construction). The user's actual complaint ("I don't see
    any building damages, partial collapses, none of that") is answered by
    a DIFFERENT population instead: `urm` `lowrise`/`midrise` buildings,
    which the T4 ladder (`_plans/urban_tornado_plan.md` §2.6) gives a real
    top-storey-loss/out-of-plane mechanism on. "Via exposure not force":
    this is a pure MEASUREMENT of what the track's placement produced, not
    a target the search is allowed to hit by any means other than routing
    the corridor's core through low/mid fabric at real intensity (`--tune-
    track`'s own tiebreak: "more T3/T4 on lowrise/midrise").

    Only enforced at `peak >= 0.90` (`PARTIAL_COLLAPSE_MIN_PEAK`) — an EF2/
    EF3-class corridor credibly never reaches T4 on ANY building, the same
    "T4 allowed, not required" reasoning round 1's check_3 used for its own
    lowest band; below the threshold this reports the count for visibility
    but does not fail on it (`"applicable": false`).
    """
    peak = float(((manifest.get("tornado_cfg") or {}).get("peak")) or 0.0)
    recs = manifest["records"]
    t4_low_mid_urm = [r for r in recs if r["level"] == "T4"
                      and r.get("btype") == "urm"
                      and r.get("height_class") in ("lowrise", "midrise")]
    n = len(t4_low_mid_urm)
    if peak < PARTIAL_COLLAPSE_MIN_PEAK:
        return True, {"applicable": False, "peak": round(peak, 4),
                      "peak_threshold": PARTIAL_COLLAPSE_MIN_PEAK,
                      "n_t4_urm_lowmid": n}
    ok = n >= PARTIAL_COLLAPSE_MIN_N
    return ok, {"applicable": True, "peak": round(peak, 4),
               "peak_threshold": PARTIAL_COLLAPSE_MIN_PEAK,
               "n_t4_urm_lowmid": n, "n_required": PARTIAL_COLLAPSE_MIN_N,
               "examples": [r["name"] for r in t4_low_mid_urm[:5]]}


def check_3_core_reaches_fabric(manifest):
    """ROUND 1 ONLY — kept for the 1500 m level presets and their own
    report history, no longer in the default `CHECKS` tuple for round 2 (see
    `check_partial_collapse_presence`, which replaces it: its EF3/EF4+ bands
    demand T4 records on `highrise`/`tower`, which plan §7 R1's `i <= 0.55`
    cap now forbids by construction). LEVEL-AWARE, keyed off the compiled
    `peak` (`manifest["tornado_cfg"]
    ["peak"]` — a direct proxy for severity/EF class, always present in a
    manifest this tool wrote). A flat ">= 6 T4" bar is wrong for an EF2-class
    track (2026-09-01 lead review): it forced an earlier draft of the
    level-1 preset to route the corridor's core lengthwise along a tower/
    highrise column just to clear it, instead of a natural diagonal crossing
    of the crop window (see `_plans/urban_tornado_C_notes.md`).

        peak < 0.80           n_T3 >= 8,  n_T4 >= 0  (T4 allowed, not
                               required — an EF2 track credibly never
                               breaches a structural chunk)
        0.80 <= peak < 0.95    n_T4 >= 6,  n_T4 highrise/tower >= 1
        peak >= 0.95           n_T4 >= 15, n_T4 highrise/tower >= 3

    `band`/`n_*_required` in the detail dict name which row applied and what
    it demanded, so a report reader does not have to re-derive the cut from
    `peak` alone."""
    peak = float(((manifest.get("tornado_cfg") or {}).get("peak")) or 0.0)
    recs = manifest["records"]
    t3 = [r for r in recs if r["level"] == "T3"]
    t4 = [r for r in recs if r["level"] == "T4"]
    n_t3, n_t4 = len(t3), len(t4)
    n_t4_tall = sum(1 for r in t4 if r.get("height_class") in ("highrise", "tower"))

    if peak < 0.80:
        band, req_t3, req_t4, req_t4_tall = "EF2 (peak < 0.80)", 8, 0, 0
        ok = n_t3 >= req_t3
    elif peak < 0.95:
        band, req_t3, req_t4, req_t4_tall = "EF3 (0.80 <= peak < 0.95)", 0, 6, 1
        ok = n_t4 >= req_t4 and n_t4_tall >= req_t4_tall
    else:
        band, req_t3, req_t4, req_t4_tall = "EF4+ (peak >= 0.95)", 0, 15, 3
        ok = n_t4 >= req_t4 and n_t4_tall >= req_t4_tall

    return ok, {"peak": round(peak, 4), "band": band,
               "n_t3": n_t3, "n_t3_required": req_t3,
               "n_t4": n_t4, "n_t4_required": req_t4,
               "n_t4_highrise_or_tower": n_t4_tall,
               "n_t4_highrise_or_tower_required": req_t4_tall,
               "t4_height_classes": sorted({r.get("height_class") for r in t4})}


def check_4_no_blacklisted_or_over_cap(manifest):
    """No blacklisted or over-cap building in the manifest — 0 by
    construction (the gates already refuse these); asserted anyway, using the
    SAME live functions the gates use, re-run against the persisted manifest
    (never trusted) — the same "re-check rather than trust" discipline
    `urban_fire_city.damaged_manifest` documents for its own re-run of
    `burnable()`."""
    max_h = tc.TORNADO_MAX_H_M
    bad = []
    for r in manifest["records"]:
        reason = ufc._pack_blacklist_reason(
            r.get("kind"), r.get("name") if r.get("kind") in ("gac", "dtc") else None)
        if reason is not None:
            bad.append({"i": r["i"], "reason": reason})
            continue
        h = r.get("H")
        if h is not None and float(h) > max_h + 1e-6:
            bad.append({"i": r["i"], "reason": "{0:.1f} m > TORNADO_MAX_H_M "
                       "{1:.1f} m".format(float(h), max_h)})
    return (not bad), {"violations": bad, "n_checked": len(manifest["records"])}


# ---------------------------------------------------------------------------
# NOT one of the seven checks -- a REPORT section (2026-09-01 lead review):
# "make the height-cap consequence visible". `TORNADO_MAX_H_M` refuses a
# building outright (§2.3 gate 4/5) -- it never becomes a T3/T4 record no
# matter how hard the corridor's core hits it, so a genuine monster standing
# exactly on the centreline reads, in the manifest alone, as silence: it is
# simply absent from `records`, indistinguishable from a building the track
# never came near. This pulls every such building back out of `refused` so a
# human can see it and decide, per building, whether "the tower stays
# pristine on the centreline" is the right call for this dataset or whether
# `TORNADO_MAX_H_M` should be lifted for a future run.
# ---------------------------------------------------------------------------
def cap_refused_in_corridor(manifest, i_min=0.10):
    """Every `refused` entry that was refused SPECIFICALLY by the height cap
    (`disaster.tornado_city._height_cap_reason`'s own wording, matched by
    substring rather than a reason-code so this keeps working if the gate
    order around it ever changes) AND whose own intensity is inside the
    corridor at all (`i >= i_min`, §2.3 gate 5's own T1 lower cut, 0.10 by
    default — a building the track never reached is not a "consequence" of
    the cap, it would have been pristine either way). Sorted tallest first —
    the tallest refusal is the one most worth a human's attention."""
    out = [r for r in manifest.get("refused", [])
          if "taller than the tornado-height cap" in str(r.get("reason", ""))
          and r.get("intensity") is not None and float(r["intensity"]) >= i_min]
    out.sort(key=lambda r: -(r.get("H") or 0.0))
    return out


def check_5_wind_sides(manifest, tcfg=None):
    """The distribution of `bearing_deg` differs between the left and right
    flanks by ~180 deg — proof the §2.4 model actually fired, not just that
    every building got SOME wind value. `tcfg` is recomputed from
    `manifest["preset"]` when not given (the `--check-only` path — cheap,
    needs no dump).

    UNDER THE `wind_at`-MISSING FALLBACK (`manifest["wind_at_available"] is
    False`) THIS LEGITIMATELY FAILS: every record's `bearing_deg` is the
    same constant `heading_deg`, so the left/right difference is 0 deg, not
    ~180 — reported honestly (`"expected_to_fail_without_wind_at": true` in
    the detail dict) rather than skipped or faked, per plan §5's "report
    numbers from runs, not claims"."""
    if tcfg is None:
        cfg = compile_disaster.load_scene_config(manifest["preset"])
        tcfg = tn.resolve_cfg(cfg)
    to_track, _u, _v = tn.frame(tcfg)

    left, right = [], []
    for r in manifest["records"]:
        wind = r.get("wind") or {}
        bearing = wind.get("bearing_deg")
        if bearing is None:
            continue
        _a, c = to_track(r["x"], r["y"])
        (left if c > 0 else right).append(math.radians(float(bearing)))

    def _circ_mean_deg(angles):
        if not angles:
            return None
        s = sum(math.sin(a) for a in angles)
        c = sum(math.cos(a) for a in angles)
        return math.degrees(math.atan2(s, c)) % 360.0

    lm, rm = _circ_mean_deg(left), _circ_mean_deg(right)
    diff = None
    if lm is not None and rm is not None:
        d = abs(lm - rm) % 360.0
        diff = min(d, 360.0 - d)
    ok = diff is not None and 135.0 <= diff <= 225.0
    return ok, {"n_left": len(left), "n_right": len(right),
               "left_mean_bearing_deg": (round(lm, 1) if lm is not None else None),
               "right_mean_bearing_deg": (round(rm, 1) if rm is not None else None),
               "flank_bearing_diff_deg": (round(diff, 1) if diff is not None else None),
               "expected_to_fail_without_wind_at": not manifest.get("wind_at_available", False)}


def check_7_refused_complete(manifest):
    """The refused list is complete: every house placement is accounted for
    exactly once across `records` (T1+, gate-passed), `refused` (any level,
    gate-failed) and `n_t0_gate_passed` (T0, gate-passed, silently not a
    record — plan §2.6's own rule) — no fourth, silent bucket."""
    n_house = manifest.get("n_house_placements", 0)
    n_accounted = (len(manifest["records"]) + len(manifest["refused"])
                  + manifest.get("n_t0_gate_passed", 0))
    ok = n_accounted == n_house
    return ok, {"n_house_placements": n_house, "n_records": len(manifest["records"]),
               "n_refused": len(manifest["refused"]),
               "n_t0_gate_passed": manifest.get("n_t0_gate_passed", 0),
               "n_accounted": n_accounted}


def check_r11_real_collapse_classes(manifest):
    """R11 / §8c (user: "we have industrial buildings and brownstones.
    Those could collapse right ... So show those") — the record's only two
    REAL urban collapse classes must both be present at bench severity.

    BENCH-PRESET ONLY: the industrial shed pocket
    (`INDUSTRIAL_SHED_SUFFIXES`, `config/presets/downtown_tornado_bench_
    500.yaml`) is a bench-preset addition — the 1500 m level presets carry
    no industrial pool at all, so this check is `"applicable": false` for
    them, the same conditional-applicability precedent `check_partial_
    collapse_presence`'s own peak-gated band already sets.

    TWO COUNTS, both sourced from `solve()`'s own per-record tags:

      * `n_industrial_total` — `kind == "industrial"` records at `"total"`
        grade. This is a REAL measured count, not just eligibility: an
        industrial record's own `rec["industrial_stats"]` is a genuine
        `tornado_collapse.plan_industrial` run (panels/roof/joists/rubble
        actually authored), because that class needs only W/D/H/x/y/yaw —
        no piece grid — so the dry run CAN run it directly.
      * `n_facade_collapse_eligible` — `rec["facade_collapse_eligible"]`
        (urm, lowrise, T4, `intensity >= FACADE_COLLAPSE_MIN_I`). This is
        ELIGIBILITY, not a verified firing: `tornado_urban.t_facade_
        collapse` runs on a SLICED building's own element table, which this
        dry run never builds (it has no piece grid at all — the same
        honest limit `check_partial_collapse_presence`'s own T4/urm/
        lowrise-midrise proxy count already carries, for the identical
        reason). The recipe's own remaining gate (<= 4 storeys) needs a
        real slice to know at all.
    """
    if manifest.get("preset") not in BENCH_PRESETS:
        return True, {"applicable": False,
                      "reason": "the industrial shed pocket is a bench-"
                                "preset-only addition (Sec8c) -- this "
                                "preset carries no industrial pool"}
    recs = manifest["records"]
    industrial_total = [r for r in recs if r.get("kind") == "industrial"
                        and r.get("level") == "total"]
    industrial_partial = [r for r in recs if r.get("kind") == "industrial"
                          and r.get("level") == "partial"]
    fc_eligible = [r for r in recs if r.get("facade_collapse_eligible")]
    ok = bool(industrial_total) and bool(fc_eligible)
    return ok, {
        "applicable": True,
        "n_industrial_total": len(industrial_total),
        "n_industrial_partial": len(industrial_partial),
        "n_facade_collapse_eligible": len(fc_eligible),
        "industrial_total_examples": [r["name"] for r in industrial_total[:5]],
        "facade_collapse_examples": [r["name"] for r in fc_eligible[:5]],
    }


#: ROUND 2 (plan §7): "the round-1 checks that still apply (gradient,
#: determinism, refused-complete)" plus 4/5 (unaffected by R2 — the
#: blacklist/cap gate and the wind-side model did not change), MINUS
#: `3_core_reaches_fabric` (replaced — see `check_partial_collapse_presence`'s
#: own docstring for why the old check is now self-contradictory under R1),
#: PLUS the four new ones plan §7 item 5 calls for: the R1 hard check, the
#: composition check, damage-capable coverage, and partial-collapse
#: presence (`3_partial_collapse_presence` keeps the numeral 3 — same slot
#: in the report, different question). ROUND 3 (§8c, R11) adds an eleventh:
#: both real urban collapse classes must be present at bench severity.
CHECKS = (
    ("1_corridor_coverage", check_1_corridor_coverage),
    ("2_gradient", check_2_gradient),
    ("3_partial_collapse_presence", check_partial_collapse_presence),
    ("4_no_blacklisted_or_over_cap", check_4_no_blacklisted_or_over_cap),
    ("5_wind_sides", check_5_wind_sides),
    ("7_refused_complete", check_7_refused_complete),
    ("8_r1_skyscraper_exposure", check_r1_skyscraper_exposure),
    ("9_r2_composition", check_composition),
    ("10_damage_capable_coverage", check_damage_capable_coverage),
    ("11_r11_real_collapse_classes", check_r11_real_collapse_classes),
)


def run_all_checks(manifest, tcfg=None):
    """`{name: (ok, detail)}` for the ten manifest-only checks. Check 6
    (determinism) is run separately — see `check_6_determinism` — because it
    needs to RE-SOLVE, not just read one manifest."""
    out = {}
    for name, fn in CHECKS:
        if name in ("5_wind_sides", "8_r1_skyscraper_exposure"):
            out[name] = fn(manifest, tcfg=tcfg)
        else:
            out[name] = fn(manifest)
    return out


def check_6_determinism(preset, dump_path, seed):
    """`disaster.tornado_city`'s "planner determinism" requirement, at city
    scale: re-solve THREE times (twice back to back, once after 50 unrelated
    draws on the GLOBAL `random` module, matching `fire_city_dry_run.
    check_determinism`'s own third call) and compare the JSON-serialisable
    manifest byte-for-byte."""
    def _once():
        m, _e = solve(preset, dump_path, seed=seed, verbose=False)
        return m

    a = _once()
    b = _once()
    ja, jb = json.dumps(a, sort_keys=True), json.dumps(b, sort_keys=True)
    repeat_identical = ja == jb
    for _ in range(50):
        random.random()
    c = _once()
    stable_after_noise = json.dumps(c, sort_keys=True) == ja
    ok = repeat_identical and stable_after_noise
    return ok, {"repeat_identical": repeat_identical,
               "stable_after_unrelated_draws": stable_after_noise}


# ---------------------------------------------------------------------------
# THE TRACK SEARCH -- plan §7 R1's "the track is PLACED to satisfy [the
# skyscraper exposure cap]... an automatic search in the dry run".
#
# THE EXPENSIVE PART (compiling the preset, building/loading the layout) RUNS
# ONCE; only the cheap per-building intensity/level draw re-runs per
# candidate — `_precompute_buildings` below does everything `solve()` does
# EXCEPT sample the track (no `inten`, no `wind_at`, no level draw), so a
# search over ~3000 (epicenter, heading) candidates costs one layout build
# plus ~3000 cheap python passes over the placement list, not ~3000 layout
# builds. `solve()` itself is left untouched by all of this — a caller who
# wants the FULL manifest for a winning candidate still calls `solve()`
# (which re-derives everything from the compiled preset the ordinary way,
# the "never trust a shortcut's own bookkeeping over a real re-solve"
# discipline `check_6_determinism` already applies elsewhere in this file).
# ---------------------------------------------------------------------------
def _precompute_buildings(preset, seed=None, dump_path=None, max_h=None):
    """`(plate, base_tcfg, buildings, resolved_seed)` — everything about the
    layout and every building's TRACK-INDEPENDENT facts (`x`, `y`, `W`, `D`,
    `H`, `btype`, `typology`, `height_class`, `route`, `kind`, `name`,
    `bakeable`, `ok`, `reason`), computed exactly the way `solve()`'s own
    loop computes them (same `damageable()` call, same `btype_for`/
    `height_class_for`), but with NO intensity field sampled and no level
    drawn — those are the only two things that change between one track
    candidate and the next.

    `base_tcfg` is `tn.resolve_cfg(cfg)` off the COMPILED preset — its
    `width_m`/`peak`/`core_frac`/wobble/curvature all come from the
    preset's own `severity`/`region_m` and do not depend on `epicenter`/
    `heading_deg` at all (`compile_disaster.compile_tornado`'s own
    formulas), so a candidate only ever needs to override `origin_m`/
    `heading_deg` on a COPY of this dict — see `_candidate_tcfg`.
    """
    cfg = compile_disaster.load_scene_config(preset)
    base_tcfg = tn.resolve_cfg(cfg)
    region = tuple(float(v) for v in cfg["layout"]["region_m"])
    plate = (-region[0] / 2.0, -region[1] / 2.0, region[0] / 2.0, region[1] / 2.0)
    max_h_eff = float(max_h) if max_h is not None else tc.TORNADO_MAX_H_M

    resolver = None
    wdh_cache = None
    if dump_path is not None:
        layout, placements, dump_seed, _dp, _sha = load_dump(dump_path)
    elif preset in BENCH_PRESETS:
        host_cfg, layout, placements, resolver = fcdr.build_layout(preset, seed=seed)
        dump_seed = int(host_cfg.get("seed", 0))
        wdh_cache = fcdr._gac_dtc_cache()
        from disaster import gac_fire as _gf
    else:
        raise SystemExit(
            f"_precompute_buildings(): no dump given and {preset!r} is not "
            f"in BENCH_PRESETS — pass --dump")
    resolved_seed = int(seed) if seed is not None else int(dump_seed)

    buildings = []
    for i, p in enumerate(placements):
        if p.get("category") != "house":
            continue
        usd = p.get("usd")
        x = float(p.get("x_m", 0.0))
        y = float(p.get("y_m", 0.0))
        yaw = float(p.get("yaw_deg", 0.0))
        if resolver is not None:
            W, D, H = fcdr._measure_wdh(usd, p, resolver, wdh_cache, _gf)
        else:
            W, D, H = float(p["W"]), float(p["D"]), float(p["H"])
        ok, reason, route, kind, name, bakeable = tc.damageable(
            p, {usd: (W, D, H)}, max_h=max_h_eff)
        typology = ufc.typology_at(layout, x, y)
        btype = tc.btype_for(usd, H) if ok else None
        height_class = tc.height_class_for(H, typology)
        buildings.append({
            "i": i, "usd": usd, "x": x, "y": y, "yaw": yaw, "W": W, "D": D,
            "H": H, "btype": btype, "typology": typology,
            "height_class": height_class, "ok": ok, "reason": reason,
            "route": route, "kind": kind, "name": name, "bakeable": bakeable,
        })
    return plate, base_tcfg, buildings, resolved_seed


def _candidate_tcfg(base_tcfg, epicenter, heading_deg):
    """A COPY of `base_tcfg` with only `origin_m`/`heading_deg` overridden —
    every other track parameter (width, peak, wobble, curvature) stays the
    preset's own."""
    tcfg = dict(base_tcfg)
    tcfg["origin_m"] = [float(epicenter[0]), float(epicenter[1])]
    tcfg["heading_deg"] = float(heading_deg)
    return tcfg


def corridor_full_width_in_plate_frac(tcfg, plate, n=600):
    """Fraction of the (plate-clipped) centreline's length for which BOTH
    corridor EDGES — centreline +- half `width_m`, in the local perpendicular
    direction, so the meander/curvature bend the edges with it — also lie
    inside `plate`. Plan §7 R1's own wording: "the corridor's full width
    stays inside the plate for >= 60% of its length" — a stricter version of
    `track_window_coverage`'s centreline-only measure, needed here because a
    centreline that stays inside the plate can still carry an edge that
    walks off it on a narrow plate. Same "invert the meander via `to_track`"
    construction `centreline_points`/`draw_png` already use.
    """
    x0, y0, x1, y1 = plate
    to_track, (ux, uy), (vx, vy) = tn.frame(tcfg)
    ox, oy = float(tcfg["origin_m"][0]), float(tcfg["origin_m"][1])
    half = 0.5 * float(tcfg["width_m"])
    reach = math.hypot(x1 - x0, y1 - y0)
    n_in_plate = 0
    n_full_width = 0
    for k in range(n):
        a = -reach + 2.0 * reach * k / float(n - 1)
        probe = (ox + ux * a, oy + uy * a)
        _a2, c2 = to_track(*probe)
        off = -c2
        cx, cy = probe[0] + vx * off, probe[1] + vy * off
        if not (x0 <= cx <= x1 and y0 <= cy <= y1):
            continue
        n_in_plate += 1
        lx, ly = cx + vx * half, cy + vy * half
        rx, ry = cx - vx * half, cy - vy * half
        if (x0 <= lx <= x1 and y0 <= ly <= y1
                and x0 <= rx <= x1 and y0 <= ry <= y1):
            n_full_width += 1
    return (n_full_width / float(n_in_plate)) if n_in_plate else 0.0


def _evaluate_track(buildings, tcfg, plate, seed, i_min_intact=0.05,
                    intact_frac_min=0.15):
    """One candidate's full evaluation. Returns `None` if a HARD constraint
    fails, else a dict with the SCORE fields and every metric the report
    prints — `None` short-circuits before the (cheap but not free) per-
    building loop needs to run to completion in the common case a candidate
    is thrown out early by geometry alone (checked first, needs no
    per-building loop at all).

    HARD constraints (plan §7 R1, all three, in order):
      1. corridor full width inside the plate for >= 60% of its length.
      2. EVERY protected item (`tc.is_protected_skyscraper` — class `tower`
         OR `H >= tc.SKYSCRAPER_PROTECTED_MIN_H_M`) samples raw `i <= tc.
         SKYSCRAPER_MAX_I` at its footprint's CORNER nearest the centreline
         — via `tc.skyscraper_exposure(buildings, inten, to_track=to_track)`,
         the SAME function `check_r1_skyscraper_exposure` calls at report
         time, over the FULL `buildings` list regardless of `b["ok"]`.
         LEAD REVIEW, 2026-09-01, two fixes from the first version: (a) it
         used to gate on `b["ok"]`, so a candidate could route the corridor
         straight over a height-capped (permanently refused, permanently
         pristine) supertall and this constraint would never see it —
         wrong, because the user's rule is about where a skyscraper stands
         relative to the corridor, not about whether the ladder can
         currently damage it; (b) it used to sample each building's own
         centre `(x, y)`, missing a wide footprint whose CORNER is in the
         corridor while its centre reads clear (`SM_Building_16`, 84.5 x
         56.9 m, is the real example that broke the centre-only version).
      3. >= `intact_frac_min` (15%) of buildings on EACH side of the
         centreline sample raw `i < i_min_intact` (0.05) — both flanks keep
         visibly intact fabric, not just the far one.

    SCORE (soft, ranks candidates that pass all three hard constraints):
      `n_t1plus_capable` — T1+ records among damage-capable buildings only
      (an AEC/standalone record the ladder can never apply a plan to is not
      "coverage" this search should chase). Tiebreak `n_t3_t4_lowmid` — more
      T3/T4 damage on `lowrise`/`midrise` height-class buildings specifically
      (plan §7 item 2's own tiebreak wording), the population `check_
      partial_collapse_presence` reads. Both are still centre-sampled
      (`level_for_intensity`'s own draw is unchanged by this fix — only the
      HARD skyscraper-exposure gate moved to corner sampling; the score is
      an ordinary damage-coverage count, not a safety gate).
    """
    full_width_frac = corridor_full_width_in_plate_frac(tcfg, plate)
    if full_width_frac < 0.60:
        return None

    to_track, _u, _v = tn.frame(tcfg)
    inten = tn.intensity_field(tcfg, plate, np.random.default_rng(seed + 23))

    sky_exposure = tc.skyscraper_exposure(buildings, inten, to_track=to_track)
    over_sky = [e for e in sky_exposure if e["i_raw"] > tc.SKYSCRAPER_MAX_I]
    if over_sky:
        return None
    max_sky_i = max((e["i_raw"] for e in sky_exposure), default=0.0)

    n_left = n_right = 0
    n_left_intact = n_right_intact = 0
    n_t1plus_capable = 0
    n_t3_t4_lowmid = 0

    for b in buildings:
        x, y = b["x"], b["y"]
        i_val = float(inten(x, y))

        _a, c = to_track(x, y)
        if c > 0:
            n_left += 1
            if i_val < i_min_intact:
                n_left_intact += 1
        else:
            n_right += 1
            if i_val < i_min_intact:
                n_right_intact += 1

        if not b["ok"]:
            continue
        if b.get("kind") == "industrial":
            # R11 (§8c): its OWN grade vocabulary, never the T0-T4 ladder's
            # `level_for_intensity` (an industrial shed's `height_class` is
            # `lowrise` like most of the URM sliced stock, so counting it
            # through the T3/T4-lowmid tiebreak below with the WRONG
            # ladder's cuts would silently inflate that tiebreak).
            if tcol.grade_for_intensity(i_val) is None:
                continue
            if b["bakeable"]:
                n_t1plus_capable += 1
            continue
        level = tc.level_for_intensity(
            i_val, random.Random(seed * 1000003 + b["i"]))
        if level == "T0":
            continue
        if b["bakeable"]:
            n_t1plus_capable += 1
        if level in ("T3", "T4") and b["height_class"] in ("lowrise", "midrise"):
            n_t3_t4_lowmid += 1

    frac_left_intact = (n_left_intact / float(n_left)) if n_left else 0.0
    frac_right_intact = (n_right_intact / float(n_right)) if n_right else 0.0
    if frac_left_intact < intact_frac_min or frac_right_intact < intact_frac_min:
        return None

    return {
        "score": (n_t1plus_capable, n_t3_t4_lowmid),
        "n_t1plus_capable": n_t1plus_capable,
        "n_t3_t4_lowmid": n_t3_t4_lowmid,
        "full_width_in_plate_frac": round(full_width_frac, 4),
        "max_sky_i_raw": round(max_sky_i, 4),
        "n_protected_sky": len(sky_exposure),
        "n_left": n_left, "n_right": n_right,
        "frac_left_intact": round(frac_left_intact, 4),
        "frac_right_intact": round(frac_right_intact, 4),
    }


def tune_track(preset, seed=None, dump_path=None,
               epicenter_range=(-150.0, 150.0, 25.0),
               heading_range=(0.0, 170.0, 10.0), top_n=5, verbose=True):
    """The grid search itself. `epicenter_range`/`heading_range` are
    `(lo, hi, step)`, both ends INCLUSIVE (plan §7's own wording: "-150..150
    step 25" and "0..170 step 10" — 13 x 13 epicenter positions x 18
    headings = 3042 candidates at the defaults).

    Returns `(results, buildings, plate, base_tcfg, resolved_seed)` where
    `results` is every candidate that cleared all three HARD constraints,
    sorted by `score` descending (best first) — `results[0]` is the winner.
    An EMPTY `results` means no candidate in the grid cleared every hard
    constraint; the caller (CLI) reports that honestly rather than
    silently falling back to something that does not satisfy plan §7 R1.
    """
    plate, base_tcfg, buildings, resolved_seed = _precompute_buildings(
        preset, seed=seed, dump_path=dump_path)

    def _frange(lo, hi, step):
        vals = []
        v = lo
        while v <= hi + 1e-9:
            vals.append(round(v, 6))
            v += step
        return vals

    ex_lo, ex_hi, ex_step = epicenter_range
    hd_lo, hd_hi, hd_step = heading_range
    ex_vals = _frange(ex_lo, ex_hi, ex_step)
    ey_vals = _frange(ex_lo, ex_hi, ex_step)
    hd_vals = _frange(hd_lo, hd_hi, hd_step)

    if verbose:
        print(f"[tune_track] {preset}: {len(ex_vals)}x{len(ey_vals)} "
             f"epicenter x {len(hd_vals)} heading = "
             f"{len(ex_vals) * len(ey_vals) * len(hd_vals)} candidates, "
             f"{len(buildings)} buildings ({sum(1 for b in buildings if b['ok'])} "
             f"gate-passed)")

    results = []
    n_evaluated = 0
    for ex in ex_vals:
        for ey in ey_vals:
            for hd in hd_vals:
                n_evaluated += 1
                tcfg = _candidate_tcfg(base_tcfg, (ex, ey), hd)
                metrics = _evaluate_track(buildings, tcfg, plate, resolved_seed)
                if metrics is None:
                    continue
                results.append((ex, ey, hd, metrics))

    results.sort(key=lambda r: r[3]["score"], reverse=True)
    if verbose:
        print(f"[tune_track] evaluated {n_evaluated}, "
             f"{len(results)} cleared all three hard constraints")
        for rank, (ex, ey, hd, m) in enumerate(results[:top_n], 1):
            print(f"  #{rank}  epicenter=[{ex:.1f}, {ey:.1f}]  heading_deg={hd:.1f}  "
                 f"score={m['score']}  max_sky_i={m['max_sky_i_raw']}  "
                 f"full_width_frac={m['full_width_in_plate_frac']}  "
                 f"flank_intact=({m['frac_left_intact']}, {m['frac_right_intact']})")
        if results:
            ex, ey, hd, m = results[0]
            print(f"\n[tune_track] WINNER — paste into the preset:")
            print(f"epicenter: [{ex:.1f}, {ey:.1f}]")
            print(f"heading_deg: {hd:.1f}")
        else:
            print("\n[tune_track] NO candidate cleared all three hard "
                 "constraints over this grid — widen the range, or the "
                 "layout's tower/highrise cluster may be too large/central "
                 "for ANY track on this plate to route around (see the "
                 "districts.rings comment on the isotropic-ring limit).")
    return results, buildings, plate, base_tcfg, resolved_seed


# ---------------------------------------------------------------------------
# markdown report
# ---------------------------------------------------------------------------
def format_markdown(manifest, checks, det_ok, det_detail):
    seed = manifest["seed"]
    preset = manifest["preset"]
    lines = [f"# Urban tornado city dry run — `{preset}` seed {seed} "
            f"(level {manifest.get('level')})\n"]
    tcfg = manifest["tornado_cfg"]
    lines.append(f"origin_m {tcfg['origin_m']}, heading {tcfg['heading_deg']} deg, "
                f"width_m {tcfg['width_m']}, peak {tcfg['peak']:.3f}, "
                f"window {manifest.get('window')}\n")
    lines.append(f"{manifest['n_house_placements']} house placements, "
                f"{len(manifest['records'])} T1+ records, "
                f"{len(manifest['refused'])} refused at a gate, "
                f"{manifest['n_t0_gate_passed']} gate-passed T0 (not a record).\n")
    if not manifest.get("wind_at_available", False):
        lines.append("**`tornado.wind_at` is not implemented yet — every "
                    "record's wind is the fallback "
                    "`{bearing_deg: heading_deg, speed_frac: i, "
                    "cross_frac: None, over: False}`. Check 5 is expected "
                    "to fail under this fallback.**\n")

    lines.append("## Level histogram (T1+ records, whole plate)\n")
    from collections import Counter
    hist = Counter(r["level"] for r in manifest["records"])
    lines.append("| level | count |")
    lines.append("|---|---|")
    for lv in tc.LEVELS:
        if hist.get(lv):
            lines.append(f"| {lv} | {hist[lv]} |")
    lines.append("")

    # R11 (§8c): the industrial class's own grade vocabulary is NOT in
    # `tc.LEVELS` at all, so it never appears in the table above -- a
    # SEPARATE small table rather than silently folding it into the T0-T4
    # histogram it is not part of.
    ind_hist = Counter(r["level"] for r in manifest["records"]
                       if r.get("kind") == "industrial")
    if ind_hist:
        lines.append("## Industrial collapse grade histogram (R11, §8c)\n")
        lines.append("| grade | count |")
        lines.append("|---|---|")
        for grade in ("partial", "total"):
            if ind_hist.get(grade):
                lines.append(f"| {grade} | {ind_hist[grade]} |")
        lines.append("")

    lines.append("## Height class x level\n")
    hc = Counter((r.get("height_class"), r["level"]) for r in manifest["records"])
    lines.append("| height class | level | count |")
    lines.append("|---|---|---|")
    for (cls, lv), cnt in sorted(hc.items()):
        lines.append(f"| {cls} | {lv} | {cnt} |")
    lines.append("")

    lines.append("## Refusal reasons\n")
    tally = Counter()
    for r in manifest["refused"]:
        key = str(r.get("reason", ""))[:70]
        tally[key] += 1
    lines.append("| reason (prefix) | count |")
    lines.append("|---|---|")
    for k, v in sorted(tally.items(), key=lambda kv: -kv[1]):
        lines.append(f"| {k} | {v} |")
    lines.append("")

    lines.append("## Cap-refused buildings inside the corridor\n")
    cap_refused = cap_refused_in_corridor(manifest)
    lines.append(f"{len(cap_refused)} building(s) refused ONLY by "
                f"`TORNADO_MAX_H_M` ({tc.TORNADO_MAX_H_M:.1f} m) whose own "
                f"intensity is >= 0.10 (inside the corridor at all) — every "
                f"one of these stands PRISTINE in the scene no matter how "
                f"hard the track's core hits it, because the gate refuses it "
                f"before a level is ever assigned to a record. Set "
                f"`TORNADO_MAX_H_M=1e9` (env var) to lift the cap entirely "
                f"and let these into the ladder instead.\n")
    if cap_refused:
        lines.append("| i | name | H (m) | intensity | would-be level |")
        lines.append("|---|---|---|---|---|")
        for r in cap_refused:
            lines.append(f"| {r['i']} | {r.get('name')} | "
                        f"{r.get('H'):.1f} | {r['intensity']:.3f} | "
                        f"{r.get('level')} |")
        lines.append("")

    lines.append("## The checks (round 2: plan §7)\n")
    lines.append("| # | check | result | detail |")
    lines.append("|---|---|---|---|")
    names = {"1_corridor_coverage": "corridor coverage",
            "2_gradient": "gradient",
            "3_partial_collapse_presence": "partial-collapse presence (R2)",
            "4_no_blacklisted_or_over_cap": "no blacklisted/over-cap",
            "5_wind_sides": "wind sides",
            "7_refused_complete": "refused complete",
            "8_r1_skyscraper_exposure": "skyscraper exposure <= 0.55 (R1, HARD)",
            "9_r2_composition": "composition: low/mid >= 70%, sky <= 15% (R2)",
            "10_damage_capable_coverage": "damage-capable coverage >= 80%",
            "11_r11_real_collapse_classes": "both real collapse classes "
                                            "present (R11, industrial total "
                                            "+ facade_collapse eligible)"}
    for key, (ok, detail) in checks.items():
        n = key.split("_", 1)[0]
        lines.append(f"| {n} | {names.get(key, key)} | "
                    f"{'PASS' if ok else 'FAIL'} | `{json.dumps(detail)}` |")
    lines.append(f"| 6 | determinism | {'PASS' if det_ok else 'FAIL'} | "
                f"`{json.dumps(det_detail)}` |")
    lines.append("")

    lines.append("## Entry strings\n")
    lines.append("```")
    for r in sorted(manifest["records"], key=lambda r: r["i"]):
        lines.append(tc.entry_string(r))
    lines.append("```\n")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# the plan PNG -- yawed footprint rectangles by level, refused hatched,
# corridor edges + centreline, crop window outlined.
# ---------------------------------------------------------------------------
_LEVEL_COLOUR = {"T0": "#d8d8d0", "T1": "#e8d97a", "T2": "#e08a2e",
                 "T3": "#c23b1f", "T4": "#5c0f0a"}
#: R11 (§8c) — the industrial collapse class's OWN colours (never
#: `_LEVEL_COLOUR`'s T0-T4 ramp, which this class is not on) — an amber/
#: rust pair distinct from the URM ladder's red family, plus a distinct
#: MARKER (`_INDUSTRIAL_MARKER`) drawn on top of the footprint polygon so
#: an industrial building reads at a glance even next to a same-toned T3/T4
#: record.
_INDUSTRIAL_COLOUR = {"partial": "#c98a1f", "total": "#7a4a00"}
_INDUSTRIAL_MARKER = "P"          # a filled plus — matplotlib's own name


def _footprint_corners(x, y, W, D, yaw_deg):
    a = math.radians(yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    hw, hd = W / 2.0, D / 2.0
    return [(x + ca * dx - sa * dy, y + sa * dx + ca * dy)
           for dx, dy in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))]


#: district TYPOLOGY colours (round 2, plan §7 R6 item a) — a warm ramp,
#: light/low-rise to dark/tall, distinct in both hue and value from
#: `_LEVEL_COLOUR`'s cool-to-hot damage ramp so a reader's eye does not
#: conflate "what this ground is zoned as" with "how badly it is damaged".
_TYPOLOGY_COLOUR = {
    "rowhouse": "#f6e4bf", "lowrise": "#f0c98a", "midrise": "#e2a05a",
    "brick_midrise": "#b96f3d", "tower": "#7a8caa", "highrise": "#4d5f80",
}
#: The red dashed line is the OUTER edge of light damage.  Severe damage is
#: intentionally concentrated at the black centreline by presets which use a
#: zero-width core; calling this a "core band" previously suggested the exact
#: opposite severity relationship.
_MINOR_DAMAGE_I = 0.36
#: §2.3 gate 5's own T1 lower cut — the ground-evidence wash (item d);
#: matches `tc._URBAN_CUTS`' T0/T1 boundary exactly (imported at call time
#: via `tc.LEVELS`' own cut table would be circular here, so this is the
#: SAME 0.10 written as a literal in three other places in this file —
#: `check_1_corridor_coverage`'s docstring, `cap_refused_in_corridor`'s
#: `i_min` default — all three MUST move together if this ever changes).
_GROUND_EVIDENCE_I = 0.10


def _skyscraper_hatch(height_class, H):
    """`"////"` for a PROTECTED item (`tc.is_protected_skyscraper` — class
    `tower` OR `H >= tc.SKYSCRAPER_PROTECTED_MIN_H_M`), else `None` — plan
    §7 R6 item b: "skyscrapers ... HATCHED so they are identifiable at a
    glance", applied uniformly to every footprint this figure draws (T0
    pale grey, refused, and every T1-T4 level alike) so a skyscraper reads
    as a skyscraper whatever its damage state.

    LEAD REVIEW, 2026-09-01: this used to hatch `height_class in
    ("highrise", "tower")`, which disagreed with the composition line's own
    typology-based sky count and with check 8/9 — see `tornado_city.
    is_protected_skyscraper`'s own docstring for why (a `brick_midrise`
    building at 47-72 m used to read `height_class == "highrise"` purely
    via a coarse H-band fallback that has nothing to do with the EF scale's
    real high-rise boundary). Now the SAME predicate hatching, check 8, check
    9 and the composition line all share."""
    return "////" if tc.is_protected_skyscraper(height_class, H) else None


def _intensity_grid(tcfg, plate, seed, n=140):
    """`(xs, ys, Z)` — `Z[iy, ix] = intensity_field(tcfg)(xs[ix], ys[iy])`
    over `plate`, sampled at the SAME seed `solve()` used
    (`np.random.default_rng(seed + 23)`, `tn.intensity_field`'s own `rng`
    argument) so the figure's ground-evidence wash and core-band contour
    show EXACTLY the field the manifest's own records were drawn from, not
    a fresh (differently-seeded, if `edge_noise_m` is ever turned on)
    approximation of it."""
    x0, y0, x1, y1 = plate
    inten = tn.intensity_field(tcfg, plate, np.random.default_rng(seed + 23))
    xs = np.linspace(x0, x1, n)
    ys = np.linspace(y0, y1, n)
    Z = np.empty((n, n))
    for iy, yy in enumerate(ys):
        for ix, xx in enumerate(xs):
            Z[iy, ix] = inten(float(xx), float(yy))
    return xs, ys, Z


def draw_png(manifest, extras, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch, Polygon, Rectangle

    tcfg = extras["tcfg"]
    plate = extras["plate"]
    window = extras["window"]
    layout = extras.get("layout")
    x0, y0, x1, y1 = plate
    fig, ax = plt.subplots(figsize=(12, 12))

    # ---- (a) BLOCKS filled by district TYPOLOGY, bottom layer -------------
    # `layout["_typology_of"]` (`urban_fire_city.typology_at`'s own source
    # dict, `districts.rezone_blocks`) is a SMALL number of large contiguous
    # zoning-region rects — literally "each grid's type" plan §7 asked for,
    # not one rect per building block.
    typ_of = (layout or {}).get("_typology_of") or {}
    for (bx0, by0, bx1, by1), typ in typ_of.items():
        ax.add_patch(Rectangle((bx0, by0), bx1 - bx0, by1 - by0,
                               facecolor=_TYPOLOGY_COLOUR.get(typ, "#e0e0e0"),
                               edgecolor="none", alpha=0.60, zorder=0))

    # ---- (d) ground-evidence wash, intensity >= 0.10 -----------------------
    # the region stream G's own corridor debris/scour pass will fill; a
    # light shaded footprint here is the offline STAND-IN so the figure
    # already shows where that pass has ground to cover.
    xs = ys = Z = None
    try:
        xs, ys, Z = _intensity_grid(tcfg, plate, manifest["seed"])
        ax.contourf(xs, ys, Z, levels=[_GROUND_EVIDENCE_I, 1.01],
                   colors=["#8a7a5c"], alpha=0.28, zorder=1)
    except Exception as exc:                                    # pragma: no cover
        print(f"[dry_run] draw_png: ground-evidence wash skipped ({exc})")

    # ---- (b) buildings, by damage level, skyscrapers hatched ---------------
    def _draw(b, facecolor, alpha=1.0):
        corners = _footprint_corners(b["x"], b["y"], b["W"], b["D"], b["yaw"])
        ax.add_patch(Polygon(corners, closed=True, facecolor=facecolor,
                             edgecolor="k", linewidth=0.25,
                             hatch=_skyscraper_hatch(b.get("height_class"),
                                                     b.get("H")),
                             alpha=alpha, zorder=3))

    for b in manifest.get("t0_footprints", []):
        _draw(b, _LEVEL_COLOUR["T0"])
    for r in manifest["refused"]:
        if r.get("W") is not None:
            _draw(r, "#8f8f8f", alpha=0.85)
        else:
            # older/foreign manifests without W/D/yaw (pre-round-2, or a
            # `--check-only` re-run of one) fall back to the round-1 marker
            # rather than crashing on a missing key.
            ax.scatter([r["x"]], [r["y"]], marker="x", s=18, c="#444444",
                      linewidths=0.8, zorder=4)
    for r in manifest["records"]:
        if r.get("kind") == "industrial":
            # R11 (§8c): its own colour pair + a filled-plus marker on the
            # footprint centre, never the T0-T4 ramp (this class is not on
            # that ladder at all — `_LEVEL_COLOUR.get(..., "#999999")`
            # would otherwise silently grey it out, the SAME "unrecognised
            # -> flat neutral, printed so it is visible" discipline
            # `tornado_urban_usd._classify`'s own fallback uses, but this
            # class deserves its OWN look, not a fallback).
            _draw(r, _INDUSTRIAL_COLOUR.get(r["level"], "#7a4a00"))
            ax.scatter([r["x"]], [r["y"]], marker=_INDUSTRIAL_MARKER, s=70,
                      c="#ffcf40", edgecolors="black", linewidths=0.7, zorder=7)
        else:
            _draw(r, _LEVEL_COLOUR.get(r["level"], "#999999"))

    # ---- (c) corridor edges, centreline, minor-damage boundary --------------
    to_track, (ux, uy), (vx, vy) = tn.frame(tcfg)
    ox, oy = tcfg["origin_m"]
    half = 0.5 * float(tcfg["width_m"])
    reach = math.hypot(x1 - x0, y1 - y0)
    cl, ed_l, ed_r = [], [], []
    for k in range(-140, 141):
        a = reach * k / 140.0
        probe = (ox + ux * a, oy + uy * a)
        _a2, c2 = to_track(*probe)
        off = -c2
        cl.append((probe[0] + vx * off, probe[1] + vy * off))
        ed_l.append((probe[0] + vx * (off + half), probe[1] + vy * (off + half)))
        ed_r.append((probe[0] + vx * (off - half), probe[1] + vy * (off - half)))
    for pts, style in ((cl, dict(color="black", lw=1.6, ls="-")),
                       (ed_l, dict(color="black", lw=0.9, ls="--")),
                       (ed_r, dict(color="black", lw=0.9, ls="--"))):
        ax.plot([p[0] for p in pts], [p[1] for p in pts], zorder=5, **style)

    # The minor-damage boundary, i == 0.36 -- an irregular ribbon (along-track breathing
    # + edge noise both bend it), so it is drawn as a CONTOUR of the same raw
    # field every record was drawn from, not a fixed-offset line the way the
    # corridor edges above are (those ARE fixed-offset by construction --
    # `width_m` is a constant, the intensity profile inside it is not).
    if Z is not None:
        try:
            ax.contour(xs, ys, Z, levels=[_MINOR_DAMAGE_I], colors="#8b0000",
                      linewidths=1.6, linestyles="dashed", zorder=5)
        except Exception:
            pass

    if window and not (manifest.get("level") is None):
        # a whole-plate "window" (the bench preset, `level is None` — see
        # `check_1_corridor_coverage`'s own note) coincides with the plate
        # border already drawn below; a second rectangle on top of it is
        # visual noise, not information.
        wx0, wy0, wx1, wy1 = window
        ax.add_patch(Rectangle((wx0, wy0), wx1 - wx0, wy1 - wy0, fill=False,
                               ec="#1f4fd8", lw=2.0, zorder=6))

    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                           ec="0.35", lw=1.2, zorder=2))

    # ---- legend -------------------------------------------------------------
    handles = [Patch(facecolor=_LEVEL_COLOUR[lv], edgecolor="k", label=f"level {lv}")
              for lv in tc.LEVELS]
    handles.append(Patch(facecolor="none", edgecolor="k", hatch="////",
                         label="highrise/tower (hatched)"))
    for typ, colour in _TYPOLOGY_COLOUR.items():
        handles.append(Patch(facecolor=colour, edgecolor="none", alpha=0.60,
                             label=f"district: {typ}"))
    handles.append(Patch(facecolor="#8a7a5c", edgecolor="none", alpha=0.28,
                         label=f"ground evidence (i >= {_GROUND_EVIDENCE_I})"))
    handles.append(plt.Line2D([0], [0], color="#8b0000", lw=1.6, ls="--",
                              label=f"minor-damage boundary (i = {_MINOR_DAMAGE_I})"))
    handles.append(plt.Line2D([0], [0], color="black", lw=1.6, label="centreline"))
    handles.append(plt.Line2D([0], [0], color="black", lw=0.9, ls="--",
                              label="corridor edge"))
    if window and not (manifest.get("level") is None):
        handles.append(plt.Line2D([0], [0], color="#1f4fd8", lw=2.0,
                                  label="crop window"))
    handles.append(Patch(facecolor="#8f8f8f", edgecolor="k", alpha=0.85,
                         label="refused (firebreak-equivalent)"))
    for grade, colour in _INDUSTRIAL_COLOUR.items():
        handles.append(Patch(facecolor=colour, edgecolor="k",
                             label=f"industrial {grade}"))
    handles.append(plt.Line2D([0], [0], marker=_INDUSTRIAL_MARKER, color="w",
                              markerfacecolor="#ffcf40", markeredgecolor="black",
                              markersize=10, linestyle="None",
                              label="industrial collapse (R11)"))
    # OUTSIDE the plate, not `loc="upper left"` -- a legend sitting on top of
    # the map hides the exact corner (tower/highrise, in this preset's own
    # layout) a reader most needs to check R1 against. "Legible at a glance"
    # (plan §7 R6's own words) means the map and the key are never in the
    # same pixels.
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0),
             fontsize=8, framealpha=0.95, borderaxespad=0.0)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    # ---- (e) title block: composition counts + level histogram -------------
    from collections import Counter
    hist = Counter(r["level"] for r in manifest["records"])
    typ_counts = manifest.get("typology_counts") or {}
    n_zoned = sum(v for k, v in typ_counts.items() if k != "_none")
    n_low_mid = sum(v for k, v in typ_counts.items() if k in COMPOSITION_LOW_MID)
    # R1-PROTECTED count (class tower OR H >= 75 m), the SAME number check 9
    # gates on -- NOT the typology tower+highrise bucket (lead review: those
    # disagreed, see `check_composition`'s own docstring).
    n_sky = int(manifest.get("n_protected_sky") or 0)
    frac_low_mid = (n_low_mid / float(n_zoned)) if n_zoned else 0.0
    frac_sky = (n_sky / float(n_zoned)) if n_zoned else 0.0
    ax.set_title(
        "{0} seed {1} (level {2}) -- {3:.0f} m track toward {4:.0f} deg, "
        "peak {5:.2f}\n"
        "composition: low/mid {6}/{7} ({8:.0%}), protected sky (tower or "
        "H>=75m) {9}/{7} ({10:.0%})  --  levels T1={11} T2={12} T3={13} "
        "T4={14}, "
        "{15} refused".format(
            manifest["preset"], manifest["seed"], manifest.get("level"),
            tcfg["width_m"], tcfg["heading_deg"], tcfg.get("peak", 0.0),
            n_low_mid, n_zoned, frac_low_mid, n_sky, frac_sky,
            hist.get("T1", 0), hist.get("T2", 0), hist.get("T3", 0),
            hist.get("T4", 0), len(manifest["refused"])),
        fontsize=10)
    ax.set_xlim(x0 - 40, x1 + 40)
    ax.set_ylim(y0 - 40, y1 + 40)
    ax.set_aspect("equal")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _out_paths(seed, out_dir):
    out_dir = out_dir or os.path.join(_SCENE_GEN_DIR, "_plans")
    return (os.path.join(out_dir, f"tornado_city_{seed}.json"),
           os.path.join(out_dir, f"tornado_city_{seed}_report.md"),
           os.path.join(out_dir, f"tornado_city_{seed}.png"))


def run_one(preset, dump, seed, out_dir, verbose=True):
    dump_path = dump or default_dump_path(preset)
    if dump_path is None and preset not in BENCH_PRESETS:
        raise SystemExit(f"no default dump known for preset {preset!r}; pass "
                         f"--dump, or add it to BENCH_PRESETS if it has no "
                         f"real Kit dump and should host-build instead")
    if dump_path is not None and not os.path.isfile(dump_path):
        raise SystemExit(f"dump not found: {dump_path}")

    manifest, extras = solve(preset, dump_path, seed=seed, verbose=verbose)
    checks = run_all_checks(manifest, tcfg=extras["tcfg"])
    det_ok, det_detail = check_6_determinism(preset, dump_path, manifest["seed"])

    out_json, out_md, out_png = _out_paths(manifest["seed"], out_dir)
    os.makedirs(os.path.dirname(out_json), exist_ok=True)
    with open(out_json, "w") as fh:
        json.dump(manifest, fh, indent=1)
    with open(out_md, "w") as fh:
        fh.write(format_markdown(manifest, checks, det_ok, det_detail))
    draw_png(manifest, extras, out_png)

    if verbose:
        print(f"[tornado_dry_run] wrote {out_json}")
        print(f"[tornado_dry_run] wrote {out_md}")
        print(f"[tornado_dry_run] wrote {out_png}")
        n_cap_refused = len(cap_refused_in_corridor(manifest))
        print(f"\n[tornado_dry_run] {preset} seed={manifest['seed']} "
             f"level={manifest.get('level')} records={len(manifest['records'])} "
             f"refused={len(manifest['refused'])} "
             f"t0_gate_passed={manifest['n_t0_gate_passed']} "
             f"cap_refused_in_corridor={n_cap_refused}")
        for key, (ok, detail) in checks.items():
            print(f"  {key:<32} {'PASS' if ok else 'FAIL'}  {detail}")
        print(f"  {'6_determinism':<32} {'PASS' if det_ok else 'FAIL'}  {det_detail}")

    all_ok = det_ok and all(ok for ok, _ in checks.values())
    return manifest, checks, det_ok, det_detail, all_ok


def run_check_only(preset, seed, out_dir):
    out_json, out_md, out_png = _out_paths(seed, out_dir)
    if not os.path.isfile(out_json):
        raise SystemExit(f"--check-only: no existing manifest at {out_json}")
    with open(out_json) as fh:
        manifest = json.load(fh)
    checks = run_all_checks(manifest)
    print(f"[tornado_dry_run] --check-only {out_json}")
    for key, (ok, detail) in checks.items():
        print(f"  {key:<32} {'PASS' if ok else 'FAIL'}  {detail}")
    print("  6_determinism                    SKIPPED (--check-only does not re-solve)")
    return manifest, checks


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--preset", default="downtown_tornado_1500")
    ap.add_argument("--dump", default=None,
                    help="path to a fire_city_placements_dump.v1 JSON; "
                         "default resolved from --preset via PRESET_DUMP")
    ap.add_argument("--seed", type=int, default=None,
                    help="override the intensity-field/level-jitter seed; "
                         "default is the dump's own seed")
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--all-levels", action="store_true",
                    help="run all three known presets in one go")
    ap.add_argument("--check-only", action="store_true",
                    help="load an existing manifest (at the standard path "
                         "for --preset/--seed) and rerun the checks, with no "
                         "re-solve and no dump needed")
    ap.add_argument("--tune-track", action="store_true",
                    help="plan §7 R1's automatic track search: grid epicenter "
                         "x heading, print the top 5 candidates and the "
                         "winner's epicenter/heading_deg ready to paste into "
                         "the preset. Does NOT write the preset file — see "
                         "tune_track()'s own docstring for why.")
    ap.add_argument("--ex-range", default="-150,150,25",
                    help="--tune-track epicenter x/y range 'lo,hi,step' (m)")
    ap.add_argument("--heading-range", default="0,170,10",
                    help="--tune-track heading range 'lo,hi,step' (deg)")
    ap.add_argument("--top-n", type=int, default=5,
                    help="--tune-track: how many candidates to print")
    args = ap.parse_args()

    if args.tune_track:
        ex_lo, ex_hi, ex_step = (float(v) for v in args.ex_range.split(","))
        hd_lo, hd_hi, hd_step = (float(v) for v in args.heading_range.split(","))
        results, _b, _p, _t, _s = tune_track(
            args.preset, seed=args.seed, dump_path=args.dump,
            epicenter_range=(ex_lo, ex_hi, ex_step),
            heading_range=(hd_lo, hd_hi, hd_step), top_n=args.top_n)
        sys.exit(0 if results else 1)

    if args.check_only:
        if args.seed is None:
            raise SystemExit("--check-only requires --seed (the manifest's "
                             "own seed, to find its file)")
        run_check_only(args.preset, args.seed, args.out_dir)
        return

    presets = list(ALL_PRESETS) if args.all_levels else [args.preset]
    results = []
    for preset in presets:
        dump = args.dump if (not args.all_levels and args.dump) else None
        manifest, checks, det_ok, det_detail, all_ok = run_one(
            preset, dump, args.seed, args.out_dir)
        results.append((preset, manifest, checks, det_ok, all_ok))

    if len(results) > 1:
        print("\n" + "=" * 78)
        print("[tornado_dry_run] SUMMARY")
        print("=" * 78)
        for preset, manifest, checks, det_ok, all_ok in results:
            n_cap_refused = len(cap_refused_in_corridor(manifest))
            print(f"  {preset:<32} seed={manifest['seed']:<3} "
                 f"records={len(manifest['records']):<4} "
                 f"refused={len(manifest['refused']):<4} "
                 f"cap_refused_in_corridor={n_cap_refused:<3} "
                 f"ALL_CHECKS={'PASS' if all_ok else 'FAIL'}")

    if not all(r[-1] for r in results):
        sys.exit(1)


if __name__ == "__main__":
    main()
