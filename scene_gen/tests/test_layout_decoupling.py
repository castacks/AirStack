"""
Guardrails for the three-stage generator (layout -> detail -> disaster).

Run from `scene_gen/` with the system python3, which has `pxr`:

    python3 -m pytest tests/ -v

These are pure-Python layout tests: no Isaac Sim, no Nucleus, no stage.
A full run is a couple of seconds.

WHAT EACH TEST IS FOR
---------------------
* `test_deterministic`          — a seed means a scene. Guards the RNG plumbing.
* `test_matches_baseline`       — nothing moved that wasn't meant to. This is
                                  the refactor's "preserve behavior" contract;
                                  when a change is intended, re-baseline with
                                  `python3 tests/snapshot.py --write` and let
                                  the committed diff show what changed.
* `test_geometry_*`             — the street plan must not depend on severity.
                                  Already true today; kept so it stays true.
* `test_structure_*`            — buildings/roads/tiles never move. A guarantee.
* `test_detail_positions_*`     — details are PUT in the same places; the
                                  disaster may then act on them.
* `test_disaster_stage_*`       — the disaster actually reaches city_detail's
                                  props, and does nothing at severity 0.

See `snapshot.py`'s docstring for why the signatures are split three ways.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import snapshot as S  # noqa: E402


# Severity sweeps at a fixed (preset, seed). Each must produce one layout.
# Every disaster type is covered: their damage fields differ in shape (radial,
# path, uniform) and each drives a different mix of damage effects, so a leak
# can hide in one and not another — `car` survived earthquake and moved under
# all four downtown presets.
#
# NOTE the `urban_v2` and `suburban_v2` sweeps are VACUOUS: both are
# `disaster-type: none`, so severity is ignored and the two ends are identical
# by definition. They are kept because they still exercise the detailed passes
# for the *baseline* test, but they prove nothing about decoupling — which is
# why `urban_v2_earthquake` had to exist before the detailed city was ever
# really tested with damage, and why it immediately found a bug.
SWEEPS = [
    ("earthquake", 42, [0.0, 0.4, 0.8]),
    ("tornado", 42, [0.0, 0.8]),
    ("explosion", 42, [0.0, 0.6]),
    ("flood", 42, [0.0, 0.6]),
    ("hurricane", 42, [0.0, 0.6]),
    ("suburb_earthquake", 7, [0.0, 0.5]),
    # Pristine presets are `disaster-type: none`, so a severity sweep on them
    # is vacuous — they are covered by the baseline test instead.
]

@pytest.mark.parametrize("preset,seed,severity", S.DEFAULT_CASES)
def test_matches_baseline(preset, seed, severity):
    """The committed snapshot still describes what the generator produces."""
    try:
        want = S.load(preset, seed, severity)
    except FileNotFoundError:
        pytest.skip(f"no baseline for {S.case_name(preset, seed, severity)}; "
                    "run `python3 tests/snapshot.py --write`")
    got = S.snapshot(preset, seed, severity)
    for key in ("geometry", "layout", "full"):
        assert got["digests"][key] == want["digests"][key], (
            f"{key} signature changed for "
            f"{S.case_name(preset, seed, severity)}. If intended, re-run "
            f"`python3 tests/snapshot.py --write` and commit the diff.")


def test_deterministic():
    """Same preset, seed and severity -> byte-identical scene, twice running."""
    a = S.snapshot("earthquake", 42, 0.6)
    b = S.snapshot("earthquake", 42, 0.6)
    assert a["digests"] == b["digests"]


def test_seed_actually_varies_the_scene():
    """Guards against the snapshot silently comparing constants."""
    a = S.snapshot("earthquake", 42, 0.6)
    b = S.snapshot("earthquake", 7, 0.6)
    assert a["digests"]["geometry"] != b["digests"]["geometry"]


@pytest.mark.parametrize("preset,seed,severities", SWEEPS)
def test_geometry_invariant_to_severity(preset, seed, severities):
    """The street plan is a function of locale and seed alone.

    Already holds — blocks and road corridors are laid out before any damage
    knob is read. This test exists to keep it that way through the refactor.
    """
    base = S.snapshot(preset, seed, severities[0])["digests"]["geometry"]
    for sev in severities[1:]:
        got = S.snapshot(preset, seed, sev)["digests"]["geometry"]
        assert got == base, (
            f"{preset} street plan changed between severity {severities[0]} "
            f"and {sev} — layout must not depend on damage")


@pytest.mark.parametrize("preset,seed,severities", SWEEPS)
def test_structure_invariant_to_severity(preset, seed, severities):
    """Buildings and ground surfaces stand in the same places at any severity.

    The structural half of requirement 4a. A ruin may be swapped in for an
    intact building — that is damage — but it stands on the same slab, so the
    positions do not move.
    """
    base = S.snapshot(preset, seed, severities[0], detail=True)["_positions"]
    for sev in severities[1:]:
        got = S.snapshot(preset, seed, sev, detail=True)["_positions"]
        for cat in sorted(S.STRUCTURE_CATEGORIES):
            want, have = base.get(cat, set()), got.get(cat, set())
            assert want == have, (
                f"{preset}: {cat} positions changed between severity "
                f"{severities[0]} and {sev} "
                f"({len(want - have)} gone, {len(have - want)} new) — "
                "structure must not depend on damage")


@pytest.mark.parametrize("preset,seed,severities", SWEEPS)
def test_detail_positions_preserved(preset, seed, severities):
    """Details are put in the same places; damage may then act on them.

    The detail half of requirement 4a, and the one the old
    `test_layout_invariant_to_severity` got wrong. That test compared
    ``(category, x, y, yaw)`` exactly, which fails on every *legitimate*
    damage effect — a toppled prop re-yaws, a blast displaces, a tornado
    strews new wrecks — so it could not distinguish "damage acted on this"
    from "the detail stage put this somewhere else". Only the second is a bug.

    So: every position present at the base severity must still be present at
    a higher one. Subset, not equality — damage is allowed to ADD (strewn
    wrecks, casualties). Categories damage legitimately relocates are listed
    in `snapshot.DAMAGE_MAY_RELOCATE` with the reason.
    """
    base = S.snapshot(preset, seed, severities[0], detail=True)["_positions"]
    for sev in severities[1:]:
        got = S.snapshot(preset, seed, sev, detail=True)["_positions"]
        for cat, want in sorted(base.items()):
            if cat in S.DAMAGE_MAY_RELOCATE:
                continue
            missing = want - got.get(cat, set())
            assert not missing, (
                f"{preset}: {len(missing)}/{len(want)} {cat} positions "
                f"vanished between severity {severities[0]} and {sev}. "
                "Damage may topple, tilt or swap a prop, but the detail stage "
                "must put it in the same place at every severity. Usually an "
                "RNG leak: a damage-only draw taken from the layout stream, "
                "or a damage branch that skips a draw the other branch makes.")


@pytest.mark.parametrize("preset,severity", [("tornado", 0.8), ("flood", 0.6), ("earthquake", 0.8)])
def test_disaster_stage_affects_detail_props(preset, severity):
    """The disaster reaches `city_detail`'s props.

    It did not, for a while: the downtown locale switches the built-in
    frontage passes off in favour of `city_detail`, and `city_detail` had no
    disaster handling — so enabling the detailed generator silently made
    street furniture immune to the event. Benches and bins stood to attention
    through a tornado.
    """
    from disaster import disaster_stage

    cfg, layout, placements = S.build_for_disaster(preset, 42, severity)
    before = [(p["x_m"], p["y_m"], p.get("roll_deg", 0.0)) for p in placements]
    tally = disaster_stage.apply(cfg, layout, placements)

    assert sum(tally.values()) > 0, f"{preset} at severity {severity} hit nothing"
    rolled = sum(1 for a, p in zip(before, placements)
                 if a[2] != p.get("roll_deg", 0.0))
    assert rolled > 0, "nothing was knocked over"
    # Categories that only `city_detail` places — the ones that used to be immune.
    assert set(tally) - {"car", "human", "tree", "streetlight", "trash_can",
                         "traffic_light"}, \
        "only the legacy categories were affected; city_detail props still immune"


@pytest.mark.parametrize("preset", ["tornado", "flood"])
def test_disaster_stage_is_a_noop_at_zero_severity(preset):
    """A pristine scene stays pristine — severity 0 must touch nothing."""
    from disaster import disaster_stage

    cfg, layout, placements = S.build_for_disaster(preset, 42, 0.0)
    before = [(p["x_m"], p["y_m"], p.get("roll_deg", 0.0)) for p in placements]
    tally = disaster_stage.apply(cfg, layout, placements)
    after = [(p["x_m"], p["y_m"], p.get("roll_deg", 0.0)) for p in placements]
    assert not tally and before == after


@pytest.mark.parametrize("preset,seed,severities", SWEEPS)
def test_structure_invariant_through_full_pipeline(preset, seed, severities):
    """Structure holds even after the disaster has run — the end-to-end claim.

    `test_structure_invariant_to_severity` stops before the disaster stage, so
    it proves the layout and detail stages are severity-blind. This one runs
    all three and asserts the guarantee that actually matters to a user: a
    severity sweep gives you the same city, with damage on top.

    Details are deliberately NOT asserted here. The disaster is entitled to
    blow a bin down the street — `freestanding` props slide ~2 m and `loose`
    ones travel up to ~10 m — and that is the feature, not a regression.
    """
    base = S.build_full_positions(preset, seed, severities[0])
    for sev in severities[1:]:
        got = S.build_full_positions(preset, seed, sev)
        for cat in sorted(S.STRUCTURE_CATEGORIES):
            want, have = base.get(cat, set()), got.get(cat, set())
            assert want == have, (
                f"{preset}: {cat} moved between severity {severities[0]} and "
                f"{sev} through the full pipeline "
                f"({len(want - have)} gone, {len(have - want)} new)")
