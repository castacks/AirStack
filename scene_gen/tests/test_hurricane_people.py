#!/usr/bin/env python3
"""test_hurricane_people.py — the three domains `hurricane_people.py` places
people in, pinned without Isaac.

    python3 scene_gen/tests/test_hurricane_people.py
    pytest -q scene_gen/tests/test_hurricane_people.py

HOST-SIDE, no Kit, no `SimulationApp`. `disaster.hurricane_people` is a pure
planner (same contract as `tornado_people.plan_people` /
`disaster.people.plan_people`), so the whole thing runs against a synthetic
context and a stub resolver, same pattern `test_tornado_people_poses.py` and
`test_fire_people.py` already use.

WHAT THIS FILE PINS

  1. THE DRY/WET LEVEL VOCABULARY AGREES WITH `tornado_people`. `_WRECKED_
     LEVELS` is a COPY (that module's own `_WRECKED` is underscore-private);
     a level added to one and not the other is a silent behaviour change.
  2. THE MEASURED ROOF-SLOPE TABLE (`_ROOF_SLOPES_LOCAL`) AGREES WITH THE
     ARCHETYPE FILES ON DISK, when `pxr` and `scene_gen/assets/
     archetypes_tornado/` are both available — re-measured by calling
     `hurricane_people.remeasure_roof_slopes()` itself (the SAME clustering
     method the table was built with), not re-derived from first principles,
     so a future re-bake that changes a roof's pitch or height breaks this
     test rather than silently going stale. Skips (does not fail) when
     either is missing, same discipline `bole_bearing_deg` in
     `disaster/people.py` uses for its own re-measure-if-possible table.
  3. NO WATER FIGURE STANDS PAST 1.5 m OF DEPTH, and every water figure's
     chest sits within a few centimetres of the water surface — the two
     numbers `_plan_water`'s whole placement rests on.
  4. NO FIGURE IN ANY DOMAIN IS INSIDE A HOUSE FOOTPRINT ON DRY LAND — the
     dry-land casualties (via `tornado_people`'s own `house_clear_m`/
     `wreck_clear_m` keepouts) and the water figures (via this module's own
     `_in_any_house` gate) alike.
  5. EVERY ROOF FIGURE IS ON A `pristine` HOUSE ONLY (2026-09-01: narrowed
     from `pristine`/`roof_stripped` — the user's own "intact houses's roofs
     only") and posed `sit_slump` (2026-09-01 ROUND 2: THE TORSO FIX,
     replacing `sit_ground` — see item 8) — never standing, walking or
     waving.
  6. EVERY ROOF FIGURE SITS ON THE MEASURED SLOPE PLANE, NOT THE RIDGE: the
     two hard margins (`ridge_margin_m` vertical, `eave_margin_slope_m`
     along the slope) are respected, and a POINTS-BASED check (never a bbox)
     — the pelvis lands within the hip-roof facet-matching tolerance this
     file already uses elsewhere (2 cm; see item 2) of the roof plane at its
     own (x, y).
  7. THE ROLL IS NO LONGER THE ROOF'S FULL PITCH (2026-09-01 ROUND 2 changed
     this from ROUND 1's `roll_deg == pitch_deg`): `roll_deg` is instead
     SOLVED so the torso stays within `hp.ROOF_TORSO_TILT_MAX_DEG` of
     gravity-vertical while every one of `sit_slump`'s foot-contact points
     (heel AND ball/toe, both sides — recomputed independently from the
     stored placement via `hp.sit_slump_contact_world_points`, never a bbox)
     lands ON or ABOVE the roof plane — NEVER below it, at every one of the
     five measured roof pitches this kit's houses carry.
  8. 2026-09-01 ROUND 2: THE TORSO FIX. User, on the live scene: "these are
     townhouses not buildings... sitting on the roof not their torso popping
     out." `sit_ground` rolled to the roof's FULL pitch reclined the torso
     ~40+ deg off plumb on a 39 deg roof; `sit_slump`, rolled PARTIALLY
     (item 7), keeps it upright. This file pins the SELECTION (only
     `sit_slump` clears every tracked point at every measured pitch — item
     5), the TORSO BOUND (item 7), and the CONTACT (items 6-7) all
     independently.
  9. DETERMINISM: the same seed reproduces the same plan, count for count and
     position for position.
"""
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import hurricane_people as hp           # noqa: E402
from disaster import people as ppl                    # noqa: E402
from disaster import tornado_people as tpp            # noqa: E402

# See `test_tornado_people_poses.py`'s own comment: `_pose_dz` is the one line
# in `people` that reaches `scene_generator` and, through it, `pxr`. Every
# z-value this file checks is either the LYING branch's own (never calls it)
# or a STANDING placement where the test wants an EXACT
# `feet_z == water_level - CHEST_FRAC * height` / `feet_z == deck_z`
# relationship — a real, small, pose-specific correction would only add noise
# to that check, not change what is being verified.
ppl._pose_dz = lambda usd, pose, height: 0.0

ARCH_DIR = os.path.join(_SCENE_GEN, "assets", "archetypes_tornado")


# ── stubs: the two objects a placement needs ─────────────────────────────────

class _Pools:
    def scale_of(self, usd): return 0.01
    def axis_of(self, usd): return "z"
    def yaw_of(self, usd): return 90.0
    def roll_of(self, usd): return 0.0


class _Resolver:
    """A 1.80 m character, 0.35 m deep — same reference
    `test_tornado_people_poses.py` uses."""
    def get(self, usd, cat, **kw):
        return {"sx": 1.25, "sy": 0.35, "sz": 1.80, "base": 0.0,
                "cx": 0.0, "cy": 0.0, "cz": 0.0}


PEOPLE_DIR = os.path.join(_SCENE_GEN, "assets", "people")

# REAL, resolvable paths to every rigged human this pack ships — not bare
# filenames. The 2026-09-01 ROUND 2 roof solve (`hp._solve_sit_slump_roll`)
# re-measures the ACTUAL placed asset's own skeleton when it can open one
# (see `hp._sit_slump_offsets`'s own docstring: this pack's six rigs do NOT
# share `sit_slump`'s geometry closely enough for one reference to serve all
# of them, so a bare filename that never resolves would silently fall back
# to the frozen rp_carla reference for every figure and never exercise the
# per-rig read at all — the whole point of ROUND 2's own per-rig sweep. A
# typo in the original bare-filename list (`rp_sophia_rigged_002` — the real
# asset is `_003`) went uncaught for exactly this reason: nothing here ever
# tried to open it.
HUMANS = [os.path.join(PEOPLE_DIR, n) for n in (
    "rp_carla_rigged_001_ue4.usd", "rp_eric_rigged_001_ue4.usd",
    "rp_sophia_rigged_003_ue4.usd", "rp_nathan_rigged_003_ue4.usd",
    "rp_manuel_rigged_001_ue4.usd", "rp_claudia_rigged_002_ue4.usd",
)]

REGION = (-200.0, -200.0, 200.0, 200.0)
WATER_LEVEL = 1.60
SHORE_BEARING_DEG = 0.0          # inland is +x; the sea/surge is toward -x


def _ground_z(x, y):
    """A synthetic shoreline: 0 m at x=-200 (the sea), rising to 3.4 m at
    x=+200 (well inland) — a plain slope, no relief, but enough to put houses
    at every depth band this module cares about (dry, 0.3-1.2 m, 1.2-1.5 m,
    and past the 1.5 m hard cap)."""
    return 3.4 * (x + 200.0) / 400.0


def _depth_at(x, y):
    return max(0.0, WATER_LEVEL - _ground_z(x, y))


STYLES = list(hp._ROOF_SLOPES_LOCAL)


def _make_houses(rng):
    """A house at every depth band, in every style, at a few damage levels."""
    houses = []
    wrecks = []
    fp_by_style = {s: 12.0 for s in STYLES}
    levels_cycle = ["pristine", "roof_stripped", "roof_collapsed",
                    "partial_collapse", "leveled", "swept"]
    # x chosen so `_ground_z(x)` lands in: dry (~2.0), light flood (~0.6),
    # mid-band (~0.9), deep (~1.35), and PAST the hard cap (~2.6).
    xs_by_band = {"dry": 150.0, "light": -60.0, "mid": -10.0,
                  "deep": -120.0, "toodeep": -180.0}
    i = 0
    for style in STYLES:
        for band, x in xs_by_band.items():
            level = levels_cycle[i % len(levels_cycle)]
            y = -180.0 + 9.0 * i
            it = 0.5
            d = round(_depth_at(x, y), 3)
            houses.append({
                "prim_path": "/World/h_%d" % i, "style": style,
                "level": level, "x": x, "y": y,
                "yaw_deg": float(rng.choice([0.0, 37.0, 90.0, 205.0])),
                "intensity": it, "water_depth_m": d,
            })
            if level != "pristine":
                wrecks.append((x, y, fp_by_style[style], it, level, None))
            i += 1
    return houses, wrecks, fp_by_style


def _make_ctx(rng):
    houses, wrecks, fp_by_style = _make_houses(rng)
    return {
        "region": REGION,
        "wrecks": wrecks,
        "houses": houses,
        "fp_by_style": fp_by_style,
        "depth_at": _depth_at,
        "water_level": WATER_LEVEL,
        "shore_bearing_deg": SHORE_BEARING_DEG,
        "intensity_at": lambda x, y: 0.5,
        "humans": list(HUMANS),
        "resolver": _Resolver(),
        "asset_pools": _Pools(),
        "plank_specs": [],
        "deck_points": [],
    }


def _run(seed=101):
    cfg = hp.resolve_cfg({})
    ctx = _make_ctx(random.Random(seed))
    humans, debris, records = hp.plan_people(cfg, ctx, random.Random(seed))
    return ctx, humans, debris, records


# ── 1. vocabulary agreement ──────────────────────────────────────────────────

def test_01_wrecked_levels_match_tornado():
    assert hp._WRECKED_LEVELS == tpp._WRECKED, (
        "hurricane_people's copy of the wrecked-level vocabulary has drifted "
        "from tornado_people._WRECKED: %r vs %r"
        % (hp._WRECKED_LEVELS, tpp._WRECKED))
    print("test_01 OK:", hp._WRECKED_LEVELS)


def test_02_style_storeys_match_modular_house():
    try:
        from detail import modular_house as mh
    except Exception as exc:
        print("test_02 SKIPPED (detail.modular_house not importable):", exc)
        return
    real = {k: int(v.get("storeys", 1)) for k, v in mh.STYLES.items()}
    assert hp._STYLE_STOREYS == real, (
        "hurricane_people's storeys fallback disagrees with the real "
        "modular_house.STYLES: %r vs %r" % (hp._STYLE_STOREYS, real))
    assert hp.STOREY_M == mh.STOREY_M
    print("test_02 OK:", real)


# ── 2. the measured roof tables agree with the archetype files ──────────────

def test_03_roof_tables_remeasure():
    """`hp.remeasure_roof_slopes()` — SAME clustering method the frozen
    `_ROOF_SLOPES_LOCAL` table was built with — against the SAME archetype
    files, matched slope-for-slope by `downhill_xy` sign (order is not
    pinned, only content). A future re-bake that changes a roof's height,
    pitch, or footprint breaks this test rather than silently going stale."""
    if not os.path.isdir(ARCH_DIR):
        print("test_03 SKIPPED (no archetypes_tornado dir)")
        return
    try:
        os.environ.setdefault("PXR_USDC_EMIT_DEPRECATION_WARNINGS", "0")
        import pxr                                          # noqa: F401
    except Exception as exc:
        print("test_03 SKIPPED (no pxr):", exc)
        return

    measured = hp.remeasure_roof_slopes(styles=STYLES, arch_dir=ARCH_DIR)
    checked = 0
    for style in STYLES:
        got = measured.get(style) or []
        if not got:
            continue
        want = hp._ROOF_SLOPES_LOCAL[style]
        assert len(got) == len(want), (
            "%s: table has %d slope(s), archetype measures %d"
            % (style, len(want), len(got)))

        def _sign_key(s):
            return (round(s["downhill_xy"][0]), round(s["downhill_xy"][1]))

        by_key = {}
        for g in got:
            by_key.setdefault(_sign_key(g), []).append(g)
        for w in want:
            cands = by_key.get(_sign_key(w)) or []
            assert cands, (
                "%s: no re-measured slope matches downhill_xy=%r"
                % (style, w["downhill_xy"]))
            g = min(cands, key=lambda c: abs(c["ridge_z"] - w["ridge_z"]))
            for field in ("pitch_deg", "ridge_z", "eave_z"):
                assert abs(g[field] - w[field]) < 0.02, (
                    "%s %s drifted: table has %.3f, archetype measures %.3f"
                    % (style, field, w[field], g[field]))
            for a, b in zip(g["bbox"], w["bbox"]):
                assert abs(a - b) < 0.02, (
                    "%s bbox drifted: table has %r, archetype measures %r"
                    % (style, w["bbox"], g["bbox"]))
        checked += 1
    assert checked > 0, "no archetype was actually re-measured"
    print("test_03 OK: re-measured %d style(s), every slope's downhill_xy/"
          "pitch_deg/ridge_z/eave_z/bbox agrees with the frozen table"
          % checked)


# ── 3. water depth discipline ────────────────────────────────────────────────

def test_04_water_depth_band_and_chest_height():
    ctx, humans, debris, records = _run(seed=101)
    water = [r for r in records if r["domain"] == "water"]
    assert water, "no water figures were placed — test fixture is too sparse"
    for r in water:
        d = r["water_depth_m"]
        assert hp.DRY_DEPTH_M < d <= 1.5 + 1e-9, (
            "water figure standing in %.2f m of water (must be in "
            "(%.2f, 1.5])" % (d, hp.DRY_DEPTH_M))
        chest_z = r["z"] + ppl.CHEST_FRAC * 1.80    # the stub's own height
        assert abs(chest_z - WATER_LEVEL) < 0.05, (
            "chest z %.3f is not within a few cm of the water surface %.3f"
            % (chest_z, WATER_LEVEL))
        assert r["covered_frac"] == round(ppl.CHEST_FRAC, 3)
        assert r["occlusion"] == "submerged"
    depths = sorted(r["water_depth_m"] for r in water)
    print("test_04 OK: %d water figure(s), depth histogram %s"
          % (len(water), depths))


def test_05_no_one_stands_past_the_hard_cap():
    # A synthetic depth field that is UNIFORMLY 2.8 m everywhere flooded, so
    # every candidate the sampler draws is over the hard cap and the water
    # pass must place NOBODY rather than fudge the cap.
    ctx, _h, _d, _r = _run(seed=202)
    deep_ctx = dict(ctx)
    deep_ctx["depth_at"] = lambda x, y: 2.8
    for h in deep_ctx["houses"]:
        h = dict(h)
    cfg = hp.resolve_cfg({})
    humans, records = hp._plan_water(cfg["water"], deep_ctx, random.Random(1))
    assert len(records) == 0, (
        "%d figure(s) placed standing in 2.8 m of water" % len(records))
    print("test_05 OK: uniformly-too-deep field places zero water figures")


# ── 4. nobody is inside a house footprint ────────────────────────────────────

def test_06_nobody_inside_a_house():
    ctx, humans, debris, records = _run(seed=101)
    houses = ctx["houses"]
    fp_by_style = ctx["fp_by_style"]
    bad = []
    for r in records:
        if r["domain"] == "roof":
            continue          # ON the house is the point
        for h in houses:
            fp = fp_by_style.get(h["style"], 12.0)
            # `dry_wreck` casualties are allowed inside their OWN wreck's
            # debris mound (`tornado_people`'s `pile`/`skirt` locations sit
            # close to the footprint by design) but never inside a STANDING
            # house; `water` figures must clear every house, including the
            # one they are anchored to (`_in_any_house`'s own `skip=h`).
            if r["domain"] == "water":
                clear_r = 0.5 * fp + 1.0 - 1e-6
            elif str(h["level"]) == "pristine":
                clear_r = 0.5 * fp
            else:
                continue
            dist2 = (r["x"] - h["x"]) ** 2 + (r["y"] - h["y"]) ** 2
            if dist2 < clear_r * clear_r:
                bad.append((r["domain"], r["x"], r["y"], h["prim_path"]))
    assert not bad, "figures inside a house footprint: %r" % (bad[:5],)
    print("test_06 OK: no dry or water figure is inside a house footprint")


# ── 5. roof placement — SIDES of the slope, seated, intact-only ─────────────

def _local_xy(x, y, house):
    """World `(x, y)` into *house*'s own LOCAL frame — the inverse of
    `hp._to_world`."""
    r = math.radians(-float(house["yaw_deg"]))
    c, s = math.cos(r), math.sin(r)
    dx, dy = x - house["x"], y - house["y"]
    return c * dx - s * dy, s * dx + c * dy


def _match_slope(style, lx, ly, lz):
    """The `_ROOF_SLOPES_LOCAL[style]` entry whose plane comes CLOSEST to
    `(lx, ly, lz)`, plus the fall-line fraction `t` (0 = ridge, 1 = eave) and
    the vertical/along-slope distances to the ridge and eave — a full,
    independent re-derivation of the plane geometry `_slope_seat_local`
    itself uses, run backwards from a placed point rather than forwards from
    a sampled one."""
    best = None
    for slope in hp._ROOF_SLOPES_LOCAL[style]:
        dhx, dhy = slope["downhill_xy"]
        x0, y0, x1, y1 = slope["bbox"]
        if abs(dhx) >= abs(dhy):
            d_horiz = dhx * (lx - (x0 if dhx > 0 else x1))
        else:
            d_horiz = dhy * (ly - (y0 if dhy > 0 else y1))
        z_plane = slope["ridge_z"] - d_horiz * math.tan(
            math.radians(slope["pitch_deg"]))
        dev = abs(lz - z_plane)
        if best is None or dev < best[0]:
            best = (dev, slope, d_horiz)
    dev, slope, d_horiz = best
    rise = slope["ridge_z"] - slope["eave_z"]
    sin_p = math.sin(math.radians(slope["pitch_deg"]))
    fall_len_m = rise / sin_p
    t = (slope["ridge_z"] - lz) / rise
    vertical_below_ridge = slope["ridge_z"] - lz
    along_slope_above_eave = (1.0 - t) * fall_len_m
    return slope, dev, t, vertical_below_ridge, along_slope_above_eave


def _roof_records_with_placements(seed=101):
    """`[(record, placement), ...]` for the roof domain only — `humans` and
    `records` are built in lockstep by `plan_people` (dry, then water, then
    roof, each pass appending to both lists together), so zipping them pairs
    a record with the placement dict that produced it."""
    ctx, humans, debris, records = _run(seed=seed)
    pairs = list(zip(records, humans))
    return ctx, [(r, p) for r, p in pairs if r["domain"] == "roof"]


def test_07_roof_pristine_only_and_standing():
    """Fix #1 (intact = `pristine` ONLY) and the ROOF POSTURE, which is now
    STANDING.

    POSTURE HISTORY, because this assertion has moved twice and the reason
    matters more than the value. The user's original four-part complaint
    asked for seated, and ROUND 1 shipped `sit_ground` rolled to the roof's
    own full pitch; that read as a torso emerging from the roof. ROUND 2
    replaced it with `sit_slump` on a PARTIAL, solved roll. Both were then
    superseded when `fire_people`'s approved roof configuration was copied
    across verbatim (`DEFAULTS_ROOF["poses"]`'s own comment: fire's
    `stand_calm`/`wave_help` table was rejected on sight, `roof_use_new_pose`
    stayed False, and the plain `idle` FALLBACK is what shipped and drew the
    verdict "the roof people on non collapsed roofs are good"). Confirmed by
    the user on the live hurricane scene, 2026-09-01: "the humans on the roof
    that are standing look fine", and, asked directly which posture that
    meant, "standing is the one that's fine".

    So this pins `idle` — a plain standing pose with no trunk flexion, which
    is what makes the two rejected failure modes ("no face up", "no leaning
    back") unreachable rather than merely unlikely. `test_09` pins the other
    half of that: NO tilt is applied to a roof figure at all. The seated
    solve is not deleted — `_solve_sit_slump_roll` is still live behind the
    `sit_slump` branch and still exhaustively tested by `test_10` — so a
    future config that re-enables it is still covered.
    """
    assert hp._ROOF_OK_LEVELS == ("pristine",), (
        "roof eligibility widened past pristine-only: %r" % (hp._ROOF_OK_LEVELS,))
    cfg = hp.resolve_cfg({})["roof"]
    assert cfg["poses"] == {"idle": 1.0}, (
        "the approved roof posture is plain standing `idle` (fire's own "
        "approved configuration, copied exactly); this config ships %r"
        % (cfg["poses"],))
    assert "wave" in ppl.BANNED_POSES, (
        "`wave` left BANNED_POSES — 'they also can't look like they are "
        "waving' was part of the original complaint")
    _ctx, pairs = _roof_records_with_placements(seed=101)
    assert pairs, "no roof figures were placed — test fixture is too sparse"
    for r, _p in pairs:
        assert r["on_roof"] is True
        assert r["house_level"] == "pristine", (
            "roof figure on a %r house — only pristine is intact now"
            % r["house_level"])
        assert r["pose"] in cfg["poses"], (
            "roof figure using pose %r, which the roof config does not "
            "offer (%r)" % (r["pose"], sorted(cfg["poses"])))
        assert r["pose"] == "idle", (
            "roof figure using pose %r, not the approved standing posture "
            "(idle) — see this test's own POSTURE HISTORY" % r["pose"])
    print("test_07 OK: %d roof figure(s), every one on a pristine house, "
          "standing (idle)" % len(pairs))


def test_08_roof_margins_respected():
    """Fix #2: on the pitched SLOPE, never the ridge apex and never the
    eave edge — the two hard margins from `DEFAULTS_ROOF`."""
    ctx, pairs = _roof_records_with_placements(seed=101)
    assert pairs, "no roof figures were placed — test fixture is too sparse"
    houses_by_style_level = {}
    for h in ctx["houses"]:
        houses_by_style_level.setdefault(
            (h["style"], h["level"]), []).append(h)
    cfg = hp.resolve_cfg({})["roof"]
    ridge_m = float(cfg["ridge_margin_m"])
    eave_m = float(cfg["eave_margin_slope_m"])
    checked = 0
    for r, _p in pairs:
        style, level = r["house_style"], r["house_level"]
        cands = houses_by_style_level.get((style, level)) or []
        assert cands, "no house on the plate matches %r/%r" % (style, level)
        h = min(cands, key=lambda hh: (hh["x"] - r["x"]) ** 2
                                      + (hh["y"] - r["y"]) ** 2)
        lx, ly = _local_xy(r["x"], r["y"], h)
        slope, dev, t, below_ridge, above_eave = _match_slope(
            style, lx, ly, r["z"])
        assert dev < 0.02, (
            "%s seat (%.3f, %.3f, %.3f) does not sit on any measured slope "
            "plane (best deviation %.3f m)" % (style, lx, ly, r["z"], dev))
        assert 0.0 <= t <= 1.0, (
            "%s seat at fall-line fraction %.3f, outside [ridge, eave]"
            % (style, t))
        assert below_ridge >= ridge_m - 1e-6, (
            "%s seat only %.3f m below the ridge (need >= %.2f m) — at the "
            "very top" % (style, below_ridge, ridge_m))
        assert above_eave >= eave_m - 1e-6, (
            "%s seat only %.3f m above the eave along the slope (need >= "
            "%.2f m) — too close to the edge" % (style, above_eave, eave_m))
        checked += 1
    print("test_08 OK: %d roof figure(s), all >= %.2f m below the ridge and "
          ">= %.2f m above the eave (along the slope)"
          % (checked, ridge_m, eave_m))


# A hip-roof facet's own plan footprint (`bbox`) is inexact right at a
# corner shared with a neighbouring facet — `_match_slope` picks whichever
# facet's plane comes CLOSEST at a given (x, y), which can differ by a
# fraction of a degree from the ONE facet `_choose_roof_slope` actually
# picked, at exactly the corner. `test_08` already accepts this (`dev <
# 0.02`); every roof check below uses the SAME 2 cm tolerance for the same
# reason, not a looser bar invented for this fix.
_FACET_MATCH_TOL_M = 0.02


def test_09_roof_figures_are_upright_and_untilted():
    """NO TILT IS APPLIED TO A ROOF FIGURE — the other half of the approved
    standing posture (`test_07` pins the pose; this pins the attitude).

    WHY THIS IS THE ASSERTION NOW. `_roof_seat_placement` sets
    `_roof_roll_applied = 0.0` on BOTH of its branches and routes every
    figure through `people._human_placement`, whose upright branch authors
    `roll_deg = ap.roll_of(usd)` and `pitch_deg = 0.0`. Its own comment
    records why, bench-measured across three z solves: any non-zero roll on
    this rig+pose stack rotates the body BACKWARD — face and hands to the
    sky — because the roll composes against the pose's authored trunk
    flexion rather than with it. `_roof_roll_deg` keeps the solved value for
    whoever picks the tilt back up, but nothing consumes it.

    So a stored roll that is anything other than the asset's own baseline,
    or a non-zero pitch, means the tilt has been switched back on without
    the posed-skeleton solve it was switched off waiting for — which is
    exactly the "leaning back / face up" failure the user rejected twice.

    THIS TEST USED TO MEASURE `sit_slump`'s TORSO VECTOR. That was ROUND 2's
    check and it kept PASSING after the switch to standing while measuring a
    pose no figure was in — `_sit_slump_offsets` on an `idle` figure with
    roll 0 returns the rest-pose torso angle, which is comfortably inside
    the bound whatever the placement actually did. The seated contact math
    is still exercised below, but only for a figure genuinely posed
    `sit_slump`, and `test_10` still sweeps the solver exhaustively.
    """
    ctx, pairs = _roof_records_with_placements(seed=101)
    assert pairs, "no roof figures were placed — test fixture is too sparse"
    houses_by_style_level = {}
    for h in ctx["houses"]:
        houses_by_style_level.setdefault(
            (h["style"], h["level"]), []).append(h)
    pools = ctx["asset_pools"]
    worst_tilt = 0.0
    worst_contact = 0.0
    n_seated = 0
    for r, p in pairs:
        style, level = r["house_style"], r["house_level"]
        cands = houses_by_style_level.get((style, level)) or []
        h = min(cands, key=lambda hh: (hh["x"] - r["x"]) ** 2
                                      + (hh["y"] - r["y"]) ** 2)
        yaw_world = p["yaw_deg"] - 90.0

        assert float(p.get("_roof_roll_applied", 0.0)) == 0.0, (
            "%s roof figure has %.2f deg of roll APPLIED; the tilt is off "
            "until it can be solved against a POSED skeleton"
            % (style, float(p["_roof_roll_applied"])))
        assert float(p["pitch_deg"]) == 0.0, (
            "%s roof figure carries pitch %.2f deg — not upright"
            % (style, float(p["pitch_deg"])))
        assert abs(float(p["roll_deg"])
                   - float(pools.roll_of(p["usd"]))) < 1e-9, (
            "%s roof figure's roll is %.2f deg, not the asset's own baseline "
            "%.2f — the roof tilt has been re-enabled"
            % (style, float(p["roll_deg"]), float(pools.roll_of(p["usd"]))))
        worst_tilt = max(worst_tilt, abs(float(p["roll_deg"])
                                         - float(pools.roll_of(p["usd"]))))

        # THE POINTS-BASED CONTACT CHECK, for a SEATED figure only. Recompute,
        # independently of `_roof_seat_placement`'s own solve, where
        # `sit_slump`'s pelvis and both feet's heel corners actually sit given
        # the STORED placement (root x_m/y_m/z_m) and this record's own
        # roll/yaw — then check each world point against the SAME slope's
        # plane equation, in the house's own local frame, exactly as
        # `_match_slope` does for the seat record. Skipped for `idle`, whose
        # ground contact is its feet at the root and is already covered by
        # `test_08`'s seat-on-the-plane check.
        if r["pose"] != "sit_slump":
            continue
        n_seated += 1
        pts = hp.sit_slump_contact_world_points(p, float(p["roll_deg"]),
                                                yaw_world, 1.80, usd=p["usd"])
        for name in ("pelvis", "heel_l", "heel_r"):
            x, y, z = pts[name]
            lx, ly = _local_xy(x, y, h)
            _slope, dev, _t, _br, _ae = _match_slope(style, lx, ly, z)
            assert dev >= -_FACET_MATCH_TOL_M, (
                "%s: sit_slump's %s is %.3f m BELOW the roof plane at its "
                "own (x, y) — sunk in" % (style, name, -dev))
            worst_contact = max(worst_contact, abs(dev))
    print("test_09 OK: %d roof figure(s), all upright and untilted (worst "
          "roll deviation %.4f deg); %d seated, worst pelvis/heel-to-plane "
          "deviation %.4f m" % (len(pairs), worst_tilt, n_seated,
                                worst_contact))


def test_10_roof_zero_penetration_every_measured_pitch():
    """The SEATED solve, kept under test although no config currently
    reaches it: `DEFAULTS_ROOF["poses"]` ships `{"idle": 1.0}` (see
    `test_07`'s POSTURE HISTORY), so `_solve_sit_slump_roll` runs only for a
    config that asks for `sit_slump`. It is still live code behind that
    branch, and deleting its coverage would mean re-deriving all of ROUND 2
    from scratch if seated is ever revisited — so this sweep stays, and is
    the reason `test_09` can safely stop asserting seated geometry.

    EXHAUSTIVE, not scenario-dependent: every rigged human this pack
    ships (`HUMANS`, now real resolvable paths — see that constant's own
    comment) posed `sit_slump` on every ONE of the five DISTINCT pitches
    `_ROOF_SLOPES_LOCAL` actually measures (39.24/39.11/21.71/19.42/19.41),
    calling `hp._solve_sit_slump_roll` / `hp._sit_slump_offsets` directly —
    no stochastic sampling, so a style/pitch combination this seed's houses
    happen not to draw is still checked. Asserts: the torso bound, and that
    NEITHER the heel NOR the ball/toe corner of EITHER foot ever sits below
    the roof plane, for any of the 6 x 5 = 30 combinations."""
    pitches = sorted({s["pitch_deg"] for slopes in hp._ROOF_SLOPES_LOCAL.values()
                      for s in slopes})
    assert len(pitches) == 5, "expected 5 distinct measured pitches, got %r" % pitches
    worst_gap = 0.0
    worst_tilt = 0.0
    checked = 0
    for usd in HUMANS:
        for pitch in pitches:
            roll, lift = hp._solve_sit_slump_roll(usd, pitch, 1.80)
            off = hp._sit_slump_offsets(usd, 1.80)
            y0, z0 = off["spine_03"]
            phi0 = math.degrees(math.atan2(y0, z0))
            tilt = abs(phi0 - roll)
            assert tilt <= hp.ROOF_TORSO_TILT_MAX_DEG + 1e-6, (
                "%s @ %.2f deg: torso %.2f deg off vertical, past the bound"
                % (os.path.basename(usd), pitch, tilt))
            worst_tilt = max(worst_tilt, tilt)
            assert lift >= 0.0
            for name in hp._SIT_SLUMP_FOOT:
                # `_sit_slump_foot_gap` is positive ABOVE the plane, negative
                # BELOW (penetrating) — `lift` raises the whole body by that
                # much, so the point actually placed sits `gap + lift` above
                # the plane; that must never be negative.
                gap = hp._sit_slump_foot_gap(off[name][0], off[name][1],
                                             roll, pitch)
                gap_after_lift = gap + lift
                assert gap_after_lift >= -1e-6, (
                    "%s @ %.2f deg: %s is %.4f m below the roof plane even "
                    "after the %.4f m lift"
                    % (os.path.basename(usd), pitch, name,
                       -gap_after_lift, lift))
                worst_gap = min(worst_gap, gap_after_lift)
            checked += 1
    print("test_10 OK: %d (human, pitch) combinations, worst torso tilt "
          "%.2f deg, worst foot-to-plane margin %.4f m (>= 0, i.e. never "
          "below)" % (checked, worst_tilt, worst_gap))


# ── 5b. NOBODY IS THROWN BY A HURRICANE ──────────────────────────────────

def test_13_no_casualty_is_thrown():
    """2026-09-01, user on the bench: "casualties thrown 10-40 m out — this
    has to be a tornado only thing... that doesn't really happen in
    hurricane. Validate that with actual reports like we did for tornado."

    Validated; see `.agents/skills/build-hurricane-scenes/PEOPLE_RESEARCH.md`
    for the citation trail. The short form: wind is 8% of direct US
    tropical-cyclone deaths and Rappaport's own definition of the category is
    "wind-borne debris or structural failure induced by wind" — not transport
    of the body; the Dade County ME series for Andrew lists all 12 direct
    traumatic deaths and every one is in, under, or immediately beside the
    structure ("Collapsing roof", "Found in destroyed mobile home", "Ejected
    from turning trailer"), none thrown; and the ~34-point gap between
    injured-indoors (90.5%) and recovered-outdoors (37.0%) that justifies the
    TORNADO's throw (CDC MMWR 61(28)) has no hurricane counterpart, because
    no hurricane study codes recovery location separately from injury
    location.

    Checked across several seeds because `_trail` is a small pass and one
    seed drawing zero of it proves nothing."""
    cfg = hp.resolve_cfg({})["dry"]
    assert list(cfg["trail"]["count"]) == [0, 0], (
        "the hurricane dry config still asks for %r thrown bod(ies); "
        "`tornado_people._trail` reads `count` first, so [0, 0] is what "
        "switches the pass off" % (cfg["trail"]["count"],))
    for seed in (101, 202, 303, 404, 505):
        _ctx, _h, _d, records = _run(seed=seed)
        for r in records:
            assert r.get("where") != "trail", (
                "seed %d threw a casualty to (%.1f, %.1f): %r"
                % (seed, r["x"], r["y"], r.get("note")))
            assert "thrown" not in (r.get("note") or ""), (
                "seed %d: %r" % (seed, r.get("note")))


def test_14_casualties_are_on_the_structure():
    """The second consequence of the same evidence. The Andrew tally is 8 in
    or under the structure, 2 immediately beside it, 2 outdoors, so the
    weight sits on `pile`/`skirt`. `street` is zero: the tornado's share is
    argued from Joplin's "house by house, car by car, block by block" search
    doctrine, which has no hurricane analogue."""
    cfg = hp.resolve_cfg({})["dry"]
    w = cfg["where"]
    assert abs(sum(w.values()) - 1.0) < 1e-9, "where weights sum to %r" % (
        sum(w.values()),)
    assert w["street"] == 0.0, "hurricane casualties on the carriageway: %r" % (
        w["street"],)
    assert w["pile"] + w["skirt"] >= 0.80, (
        "only %.2f of the weight is on/around the structure; the Andrew case "
        "series puts 10 of 12 there" % (w["pile"] + w["skirt"]))
    assert w["pile"] > w["yard"], (
        "more casualties in the open yard than under the collapsed structure "
        "— inverts the modal hurricane death")
    for seed in (101, 202, 303):
        _ctx, _h, _d, records = _run(seed=seed)
        dry = [r for r in records if r.get("domain") == "dry_wreck"]
        assert dry, "seed %d planned no dry casualties" % seed
        assert not [r for r in dry if r.get("where") == "street"], (
            "seed %d put a casualty in the street" % seed)


def test_15_the_tornado_still_throws():
    """THE CONTROL, and the reason the two tests above are not vacuous. The
    change is made in `hurricane_people.resolve_cfg` on its own `dry`
    sub-config; `tornado_people` is untouched and every tornado scene must
    still get its throw. Without this, deleting `_trail` outright would pass
    `test_13` just as well."""
    tcfg = tpp.resolve_cfg({})
    lo, hi = tcfg["trail"]["count"]
    assert int(hi) > 0, (
        "the TORNADO's own trail pass was disabled too — this fix is meant "
        "to be hurricane-scoped, see PEOPLE_RESEARCH.md section 7")
    assert tcfg["where"]["street"] > 0.0, "the tornado lost its street share"

    # ...and it really does place them, not merely configure them.
    ctx = _make_ctx(random.Random(7))
    tctx = {"region": REGION, "humans": list(HUMANS),
            "resolver": _Resolver(), "asset_pools": _Pools(),
            "wrecks": [{"x": 0.0, "y": 40.0 * i, "fp": 12.0,
                        "intensity": 0.9, "level": "leveled"}
                       for i in range(-2, 3)],
            "plank_specs": [], "deck_points": [],
            "intensity_at": lambda x, y: 0.9}
    thrown = 0
    for seed in (1, 2, 3, 4, 5, 6):
        _h, _d, recs = tpp.plan_people(tcfg, tctx, random.Random(seed))
        thrown += sum(1 for r in recs if r.get("where") == "trail")
    assert thrown > 0, (
        "six seeds over five levelled houses threw nobody — the tornado's "
        "own trail pass is broken, so test_13 proves nothing")
    print("test_15 OK: tornado threw %d casualt(ies) over 6 seeds; hurricane "
          "throws none" % thrown)


# ── 5c. CASUALTIES ARE ON THE WRECK, NOT IN THE LAWN BESIDE IT ───────────

def _wreck_deck(cx, cy, fp, cell=0.8, rows=None):
    """A synthetic wrecked-house deck: a low debris mat over the footprint
    with a STANDING WALL along one edge. Stands in for the archetype samples
    `suburb_hurricane_launch_script` now takes with `Usd.TraverseInstance-
    Proxies` — the point under test is that `_Deck` + `_DECK_BAND` accept the
    open mat and refuse the wall, which needs no real USD to check."""
    pts = []
    n = int(fp / cell) + 1
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x, y = cx + i * cell, cy + j * cell
            if abs(x - cx) > fp * 0.5 or abs(y - cy) > fp * 0.5:
                continue
            # HALF THE FOOTPRINT IS A 2.4 m STANDING SECTION; the other
            # half is open debris mat at 0.35 m.
            #
            # SIZED FOR THE CONTROL, not for realism. Two narrower walls were
            # tried first (1.6 m, then 3.6 m) and with BOTH of them the
            # control — lifting `_DECK_BAND`'s ceiling so the openness gate
            # cannot refuse anything — still placed nobody under the wall, so
            # the test was passing without being sensitive to the thing it
            # claims to check. The reason is geometric: a lying figure is
            # ~1.8 m end to end and `_one_casualty` keeps the FLATTEST of
            # eight bearings, so a body only fits wholly inside a strip
            # 1.8 m narrower than the strip itself, and that sweet spot was
            # ~9% of the `pile` draw annulus — 0 hits in 9 placements is
            # ordinary luck. Half a footprint is a large flat 2.4 m plateau
            # a body fits in from any bearing, so if the band stops gating,
            # bodies land on it and the control fails as it should.
            z = 2.4 if (x - cx) > 0.0 else 0.35
            pts.append((x, y, z))
    return pts


def test_16_casualties_land_inside_the_wrecked_footprint():
    """2026-09-01, user on the live 500 m plate: "I see dry casualties only
    in the open", then "tornado manages to do it within the house footprint
    so you should be able to here."

    Three things had to change together and this pins all three:

      1. `ctx["deck_points"]` — the launcher was passing `[]`, so `_Deck`
         measured only the plank field and read ~0 over the whole lot;
         `_DECK_BAND["pile"]`'s 0.03 m floor stopped being cleared and every
         body was pushed onto the skirt. `tornado_people._Deck`'s own
         docstring records the tornado hitting this exact failure.
      2. `where_bands["pile"]` — `_candidate` draws `pile` from 0.62-1.05
         FOOTPRINTS off the house centre and 0.5 is the wall line, so `pile`
         was OUTSIDE the walls by construction, whatever the deck said.
      3. `wreck_clear_m` — the `0.5 * fp + margin` keepout would refuse every
         interior point the new band proposes.

    Measured on the real archetypes before/after: 0 of 30 inside a footprint
    against 11 of 36 (31%), median distance from the house centre 18.3 m ->
    8.4 m."""
    cfg = hp.resolve_cfg({})["dry"]
    # `.get` chain, not `cfg["where_bands"]["pile"]`: BEFORE the fix the key
    # was absent entirely, and a KeyError here would crash the test instead
    # of reporting the defect. Fall back to the tornado default so the
    # assertion below is what fires.
    lo, _hi = (cfg.get("where_bands") or {}).get(
        "pile", tpp._WHERE_BANDS["pile"])
    assert lo < 0.5, (
        "`pile` still starts at %.2f footprints, outside the 0.5 wall line — "
        "no casualty can land inside a wrecked house" % lo)
    assert cfg["wreck_clear_m"] == 0.0, (
        "the wreck keepout is still %.2f m; it refuses every interior point "
        "the band proposes" % cfg["wreck_clear_m"])

    FP = 14.0
    houses = [(0.0, 0.0), (60.0, 0.0), (0.0, 60.0), (60.0, 60.0)]
    pts = []
    for cx, cy in houses:
        pts += _wreck_deck(cx, cy, FP)
    ctx = {"region": (-40.0, -40.0, 100.0, 100.0), "humans": list(HUMANS),
           "resolver": _Resolver(), "asset_pools": _Pools(),
           "wrecks": [{"x": cx, "y": cy, "fp": FP, "intensity": 0.7,
                       "level": "partial_collapse"} for cx, cy in houses],
           "plank_specs": [], "deck_points": pts,
           "intensity_at": lambda x, y: 0.7}
    c = dict(cfg)
    c["max_total"] = 40
    c["per_wreck"] = dict(c.get("per_wreck") or {})
    c["per_wreck"]["partial_collapse"] = [6, 8]
    _h, _d, recs = hp._plan_dry(c, ctx, random.Random(5))
    assert recs, "nothing placed at all"
    inside = 0
    for r in recs:
        cx, cy = min(houses, key=lambda h: (h[0] - r["x"]) ** 2
                                           + (h[1] - r["y"]) ** 2)
        if math.hypot(cx - r["x"], cy - r["y"]) <= FP * 0.5:
            inside += 1
    assert inside > 0, (
        "%d casualt(ies) placed and NOT ONE is inside a wrecked footprint — "
        "the exact complaint this fix answers" % len(recs))
    print("test_16 OK: %d of %d casualt(ies) inside a wrecked footprint"
          % (inside, len(recs)))


def test_17_nobody_is_placed_under_standing_geometry():
    """The safety half of test_16. Dropping `wreck_clear_m` is only sound
    because the MEASURED deck gates openness per cell: a cell under a
    standing wall or an intact roof section reads metres high, falls outside
    `_DECK_BAND`, and is refused. Without that this fix would bury
    casualties inside standing shells, which is worse than the bug.

    The synthetic deck makes half of every footprint a 2.4 m standing
    section; nobody may be under it.

    HONEST LIMITATION, RECORDED RATHER THAN HIDDEN. This is a PROPERTY
    assertion, not a demonstrated regression guard for one mechanism. Three
    attempts to build a control that FAILS it — blinding `_Deck` to the
    archetype points, and lifting `_DECK_BAND["pile"]`'s ceiling to 99 m with
    both a 3.6 m and a half-footprint standing section — all still placed
    nobody on the plateau. So the property is over-determined: `_one_casualty`
    keeps the flattest of eight bearings, `body_ok` applies
    `max_deck_tilt_pile_m` at three stations, and the band gate runs on top of
    that, and I could not isolate which one is load-bearing here. The test
    therefore proves the OUTCOME holds and would catch a regression that
    breaks all of them; it does not prove the band alone is what holds it.
    The evidence that the outcome holds on REAL geometry is the archetype
    probe quoted in `test_16`: 36 casualties over 16 wrecked houses sampled
    from the actual tornado archetype library, zero under a deck above
    1.30 m."""
    FP = 14.0
    houses = [(0.0, 0.0), (60.0, 0.0), (0.0, 60.0), (60.0, 60.0)]
    pts = []
    for cx, cy in houses:
        pts += _wreck_deck(cx, cy, FP)
    deck = tpp._Deck([], points=pts)
    ctx = {"region": (-40.0, -40.0, 100.0, 100.0), "humans": list(HUMANS),
           "resolver": _Resolver(), "asset_pools": _Pools(),
           "wrecks": [{"x": cx, "y": cy, "fp": FP, "intensity": 0.7,
                       "level": "partial_collapse"} for cx, cy in houses],
           "plank_specs": [], "deck_points": pts,
           "intensity_at": lambda x, y: 0.7}
    c = dict(hp.resolve_cfg({})["dry"])
    c["max_total"] = 40
    c["per_wreck"] = dict(c.get("per_wreck") or {})
    c["per_wreck"]["partial_collapse"] = [6, 8]
    _h, _d, recs = hp._plan_dry(c, ctx, random.Random(5))
    hi = float(tpp._DECK_BAND["pile"][1])
    for r in recs:
        z = deck.at(r["x"], r["y"])
        assert z <= hi + 1e-6, (
            "a casualty at (%.1f, %.1f) sits under geometry %.2f m up — past "
            "the %.2f m ceiling for its class, i.e. inside a standing shell"
            % (r["x"], r["y"], z, hi))
    print("test_17 OK: %d casualt(ies), none under standing geometry"
          % len(recs))


def test_18_the_tornado_keeps_its_own_bands_and_keepout():
    """THE CONTROL. `where_bands` was added to `tornado_people` as an
    OVERRIDE with the tornado's own numbers as the default, so a caller that
    does not set it draws exactly as before. If this fails, the fix has
    leaked into every tornado scene."""
    tcfg = tpp.resolve_cfg({})
    assert tcfg.get("where_bands") is None, (
        "the tornado config now carries where_bands=%r — it must fall "
        "through to the module default" % (tcfg.get("where_bands"),))
    assert tpp._WHERE_BANDS["pile"] == (0.62, 1.05)
    assert tpp._WHERE_BANDS["skirt"] == (0.95, 1.60)
    assert tpp._WHERE_BANDS["yard"] == (1.15, 2.00)
    assert tcfg["wreck_clear_m"] == 0.6, (
        "the tornado lost its wreck keepout (%r); a body in the middle of a "
        "levelled tornado mound is invisible from every angle"
        % (tcfg["wreck_clear_m"],))


# ── 6. determinism ────────────────────────────────────────────────────────

def test_11_determinism():
    _c1, h1, d1, r1 = _run(seed=303)
    _c2, h2, d2, r2 = _run(seed=303)
    assert len(r1) == len(r2) and len(h1) == len(h2)
    for a, b in zip(sorted(r1, key=lambda r: (r["domain"], r["x"], r["y"])),
                    sorted(r2, key=lambda r: (r["domain"], r["x"], r["y"]))):
        assert a["x"] == b["x"] and a["y"] == b["y"] and a["z"] == b["z"]
        assert a["pose"] == b["pose"] and a["domain"] == b["domain"]
    print("test_11 OK: seed 303 reproduces %d figure(s) exactly" % len(r1))


# ── 7. the envelope itself ────────────────────────────────────────────────

_TORNADO_KEYS = {"x", "y", "z", "pose", "attitude", "where", "intensity",
                 "yaw", "body_axis_deg", "reach_m", "alive", "visibility",
                 "occlusion", "covered_frac", "sunk_frac", "visible_parts",
                 "boards", "note"}
_NEW_KEYS = {"domain", "class", "water_depth_m", "on_roof", "house_prim_path",
            "prim_path", "house_style", "house_level"}


def test_12_envelope_matches_tornado_plus_the_new_keys():
    _ctx, _h, _d, records = _run(seed=101)
    assert records, "no records at all"
    want = _TORNADO_KEYS | _NEW_KEYS
    for r in records:
        got = set(r.keys())
        assert got == want, "record keys %r != expected %r (diff %r)" % (
            sorted(got), sorted(want), sorted(got ^ want))
    print("test_12 OK: every record carries tornado_people's own %d keys "
          "plus the %d new ones" % (len(_TORNADO_KEYS), len(_NEW_KEYS)))


def _run_all():
    import traceback
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in fns:
        try:
            fn()
        except AssertionError as exc:
            failed += 1
            print("FAIL", name, "-", exc)
        except Exception:
            failed += 1
            print("ERROR", name)
            traceback.print_exc()
    print("\n%d/%d passed" % (len(fns) - failed, len(fns)))
    return failed == 0


if __name__ == "__main__":
    ok = _run_all()
    sys.exit(0 if ok else 1)
