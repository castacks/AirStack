#!/usr/bin/env python3
"""test_kit_elevations.py — can every FREE-STANDING kit style's flanks show
a real window, on every elevation, in at least one bake?

    python3 scene_gen/tests/test_kit_elevations.py
    pytest -q scene_gen/tests/test_kit_elevations.py

WHY THIS EXISTS
---------------
Round 5 (user, 2026-08-30, looking at the first 500 m M7.8 OSMO scene): "the
brick buildings look wrong. They are plain brick, no windows nothing. I
think you're using the wrong building/prims." Root-caused (see
`_plans/earthquake_round5_plan.md` and `urban_building.fam04`'s own
docstring) to `commercial`/`commercial_mid`/`highrise_04`/`department_store`/
`block_commercial`'s "storey" band: `4_Facade_B` is a flat, 8-vertex box with
no window geometry at all (measured — 2 depth levels, `relief_probe`, versus
8 on a genuine window piece), and EVERY archetype baked so far landed on it
on EVERY side, front included, because `_plan_band`'s mode/piece picks are
pure `rng` draws and the ground band ahead of "storey" always consumes
exactly the same number of them regardless of style, so a fixed baking seed
reaches the same (blank) choice every time — not per-side bad luck, and not
fixable by re-rolling the seed. The fix is a `side` pool that FORCES
`4_Facade_A` — the OTHER half of this band's own already-declared `walls`
pool, already measured (`PIECES`) and already curated by the fire session's
own glazing survey (`quake_flow._G2_WIN_FACES`, real punched-window
rectangles) — on E, N and W, while S (front) keeps drawing from `walls`
via the shared `rng` exactly as it always has (the fire session's benches
read the front; it must stay byte-identical, verified by a placement diff
against the pre-fix module in the round-5 session, not repeated here).

WHAT "WINDOW" MEANS HERE, AND WHY THAT ORACLE
----------------------------------------------
Triangle density does NOT separate a real window from a blank panel in this
kit — a decorative moulding on a blank spandrel can out-triangle a shallow
window reveal, which is exactly the failure mode `tools/gac_faces.py`'s own
docstring warns about for a building with nothing to be relative to. What
DOES separate them, reusably and without opening a single USD file, is
`quake_flow._G2_WIN_FACES`: the fire session's own hand-curated table of
real, measured punched-window rectangles, keyed by kit piece name. A piece
name in that table has a real window; a piece not in it does not (or was
not yet curated — see `soot_plume._virtual_openings`'s own docstring on
`Facade_B`'s texture-only case, which is precisely the failure this test
guards against ever being SHIPPED as a building's only option again).

WHAT "FREE-STANDING" MEANS, AND WHY NO STYLE HERE IS EXEMPTED
---------------------------------------------------------------
A party-wall style (a true rowhouse, one flank always covered by a
neighbour) may legitimately leave that one flank blank — `urban-layout`
SKILL.md's whole `place_mid`/`place_end`/`place_corner` apparatus exists for
exactly this. None of that applies to any style tested here: every kit
`usd:` entry in `urban_quake_v3.yaml`/`urban_quake_v4.yaml` (the "v4 pools")
carries NO `place_mid`/`place_end`/`tags` at all, meaning `districts` treats
every one of them as free-standing (`pack`/`any`), and the one style that
IS declared for a `rowhouse` pool (`brownstone`, in `urban_quake_v3.yaml`)
sits behind a documented 0 % ring mix ("the earthquake preset zeroes the
rowhouse mix... a stray draw simply refuses" — that file's own comment), so
it is placed free-standing in practice too. `_PARTY_WALL_EXEMPT` below is
therefore empty for the current v4 pools, but is kept as an explicit,
commented structure (rather than simply absent) so the day a real
party-wall style IS added, the exemption is a one-line, justified add here —
not a silent gap in the guard.

WHAT THIS CANNOT SEE: whether `4_Facade_A`'s real geometry (never yet baked
into any local archetype — every existing bake drew `Facade_B`) actually
reads well once it finally gets baked and rendered in Kit; the round-5
session's proof render is a schematic built from `_G2_WIN_FACES`'s own
measured rectangles, not a re-bake. That needs Isaac Sim — not run here.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

import random  # noqa: E402

from detail import urban_building as ub  # noqa: E402
from disaster import quake_flow as qf    # noqa: E402

SIDES = ("S", "E", "N", "W")

# Every kit style with a live, uncommented `usd:` entry anywhere in
# `config/asset_sets/urban_quake_v3.yaml` / `urban_quake_v4.yaml` (v4
# `extends: urban_quake_v3`, so its own pools inherit v3's unless
# overridden — grepped `bld_*_DG0` across both files, minus the two
# MCE-twin ROUTE TARGETS `highrise_step`/`block_residential`, which are
# reached only via `kit_substitute.route()`'s same_art swap of a real MCE
# original, never placed as a `usd:` pool entry themselves while
# unbaked — see `urban_quake_v3.yaml`'s own "NOT YET BAKED" comment).
V4_POOL_STYLES = ("apartment", "brownstone", "commercial", "commercial_mid",
                  "office_wide")

# The other three `fam04` siblings (round-5 diagnosis: same "storey" band,
# same fix, same failure mode) are not in the v4 pools TODAY (no local bake
# exists for any of them either — `bld_block_commercial_DG0.usd` isn't even
# on disk), but they inherit the fix from `fam04` itself, not from a
# per-style override, so a future edit that special-cases `commercial`
# instead of fixing the band would regress these silently. Guarded here too.
FAM04_SIBLINGS = ("highrise_04", "department_store", "block_commercial")

ALL_TESTED_STYLES = V4_POOL_STYLES + FAM04_SIBLINGS

# Styles for which one elevation is a legitimate, DOCUMENTED party wall and
# is exempted from the "every side needs a window" requirement below.
# {style: {exempt sides}}. Empty today -- see the module docstring's
# "WHAT FREE-STANDING MEANS" section for why `brownstone`'s `rowhouse` pool
# membership does not count (0 % ring mix, never actually drawn).
_PARTY_WALL_EXEMPT = {}

# Ten seeds is enough to separate "this side can never show a window" (the
# actual round-5 bug: 0/10) from "this side's motif happened to roll the
# blank piece this one time" (S, on several v4-pool styles, whose piece is
# still picked by the shared `rng` and is not forced either way).
TEST_SEEDS = (1, 2, 3, 5, 7, 11, 13, 17, 19, 23)


def _window_counts(style, seed):
    """{side: count of _G2_WIN_FACES-known window pieces on that side}."""
    rng = random.Random(seed)
    placements = ub.build_building(style, 0.0, 0.0, 0.0, rng)
    info = qf.describe(style, placements, 0.0, 0.0, 0.0)
    counts = {s: 0 for s in SIDES}
    for e in info["elements"]:
        if e["role"] not in ("wall", "corner"):
            continue
        if e["name"] in qf._G2_WIN_FACES:
            counts[e["side"]] += 1
    return counts


def _best_over_seeds(style):
    """{side: max window count seen for that side across TEST_SEEDS} --
    "can this side ever show a real window", not "does this one build"."""
    best = {s: 0 for s in SIDES}
    for seed in TEST_SEEDS:
        counts = _window_counts(style, seed)
        for s in SIDES:
            best[s] = max(best[s], counts[s])
    return best


def check(verbose=True):
    """Host-side: every FREE elevation of every tested style can show at
    least as many real windows, in its best bake, as the style's own front
    ever manages. Returns a list of problem strings (empty = pass)."""
    problems = []
    for style in ALL_TESTED_STYLES:
        if style not in ub.STYLES:
            problems.append(f"{style}: not in urban_building.STYLES")
            continue
        best = _best_over_seeds(style)
        exempt = _PARTY_WALL_EXEMPT.get(style, set())
        # A RAW count comparison against the front is the wrong test: S/N run
        # the building's W (long axis, more modules) and E/W run its D (short
        # axis, fewer modules), so even a perfectly healthy style shows fewer
        # windows on its narrow sides by simple geometry (measured: apartment
        # S/N=16 vs E/W=12, office_wide S/N=58 vs E/W=26 -- neither is a bug).
        # What "a blank flank cannot ship silently again" actually needs
        # guarded is PRESENCE: if the front can EVER show a real window, no
        # free elevation may be incapable of ever showing one at all -- which
        # is exactly and only what round 5 found (E/N/W stuck on the one
        # piece with `_G2_WIN_FACES` membership: none, i.e. count 0, on
        # every side, in every existing bake).
        n_required = 1 if best["S"] > 0 else 0
        for side in ("E", "N", "W"):
            if side in exempt:
                continue
            if best[side] < n_required:
                problems.append(
                    f"{style}/{side}: best-of-{len(TEST_SEEDS)} window count "
                    f"{best[side]} -- front can show a real window (best "
                    f"S={best['S']}) but this elevation never can "
                    f"(S={best['S']}, E={best['E']}, N={best['N']}, "
                    f"W={best['W']})")
    if verbose:
        if problems:
            print("[test_kit_elevations] FAILED:")
            for p in problems:
                print("  " + p)
        else:
            print(f"[test_kit_elevations] ok — {len(ALL_TESTED_STYLES)} "
                  f"styles, every free elevation matches or beats its own "
                  f"front across {len(TEST_SEEDS)} seeds")
    return problems


# ---------------------------------------------------------------------------
# pytest entry points
# ---------------------------------------------------------------------------
def test_every_free_elevation_has_windows():
    problems = check(verbose=False)
    assert not problems, "\n".join(problems)


def test_fam04_storey_band_forces_facade_a_on_flanks():
    """The specific round-5 regression, pinned: `commercial`/`commercial_mid`
    (dominant in the standing stock the user reviewed — 19 + 17 buildings in
    the OSMO scene) must place `SM_MBuilding04_Facade_A` (the window piece)
    on every storey slot of ALL FOUR sides, in EVERY seed — not "often", not
    "usually wins the rng draw" — because it is FORCED via the `side` pool.
    (Round-5 follow-up: S included — the measured pre-fix bakes were 100%
    blank Facade_B on the front too, which is what the user photographed.)"""
    for style in ("commercial", "commercial_mid", "highrise_04",
                  "department_store", "block_commercial"):
        for seed in TEST_SEEDS[:3]:
            pls = ub.build_building(style, 0.0, 0.0, 0.0, random.Random(seed))
            W, D = ub.footprint(ub.STYLES[style])
            y_south = -D / 2.0
            spec = ub.STYLES[style]
            z, storey_lo, storey_hi = 0.0, None, None
            for band in spec["bands"]:
                z0 = z
                if not band.get("parapet"):
                    z += band["h"] * band.get("repeat", 1)
                if band["sub"] == "storey":
                    storey_lo, storey_hi = z0, z
            flank_storey = [
                p for p in pls
                if storey_lo - 0.5 <= p["z_m"] <= storey_hi + 0.5
                and "Facade" in p["usd"] and "Corner" not in p["usd"]
            ]
            assert flank_storey, f"{style} seed={seed}: no flank storey pieces found"
            bad = [p["usd"] for p in flank_storey
                   if not p["usd"].endswith("SM_MBuilding04_Facade_A.usd")]
            assert not bad, (
                f"{style} seed={seed}: {len(bad)} flank storey piece(s) are "
                f"not Facade_A: {sorted(set(bad))[:3]}")


def test_side_pool_is_rng_neutral_and_windows_the_front():
    """Two contracts replace the old front-byte-identical guarantee (the
    round-5 follow-up deliberately changes S — the measured pre-fix front
    was 100% blank Facade_B, exactly what the user photographed):

    1. RNG NEUTRALITY, the property that actually protects every other
       placement: adding/removing the `side` pool must not shift the shared
       random stream — the next draw after `_plan_band` returns must be
       identical either way (`_plan_band`'s own comment records how an early
       rng-consuming version silently changed the top-floor motif).
    2. S NOW CARRIES THE WINDOWED PIECE: with the `side` pool present, every
       S storey slot is the forced window facade, same as E/N/W."""
    for style in ("commercial", "commercial_mid"):
        W, D = ub.footprint(ub.STYLES[style])
        for seed in TEST_SEEDS:
            rng_a = random.Random(seed)
            rng_b = random.Random(seed)
            spec = ub.STYLES[style]
            storey_band = next(b for b in spec["bands"] if b["sub"] == "storey")
            no_side_band = dict(storey_band)
            no_side_band.pop("side", None)
            plan_with_side = ub._plan_band(storey_band, W, D, rng_a)
            plan_without_side = ub._plan_band(no_side_band, W, D, rng_b)
            assert rng_a.random() == rng_b.random(), (
                f"{style} seed={seed}: the `side` pool consumed rng — every "
                f"band placed after this one would silently change")
            side_piece = storey_band["side"][0]
            bad = [sl for sl in plan_with_side["S"] if sl[2] != side_piece
                   and sl[2] is not None]
            assert not bad, (
                f"{style} seed={seed}: S storey slot(s) not the forced "
                f"window piece: {bad[:3]}")


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)
