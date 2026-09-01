#!/usr/bin/env python3
"""test_hurricane_palettes_draw.py — is the PER-HOUSE palette draw actually a
draw, and is it the pool/weights the config asked for?

    python3 scene_gen/tests/test_hurricane_palettes_draw.py
    pytest -q scene_gen/tests/test_hurricane_palettes_draw.py

WHY THIS EXISTS
---------------
The 2026-08-31 report: "the house builds is using textures that we didn't
want... It seems to be ignoring some things." Measured cause, upstream of any
Isaac render: `suburb_scene.build_placements` gave a DETACHED house no
palette of its own at all — `pal = h.get("palette") or mh.STYLES[ent
["style"]].get("palette")`, and `h.get("palette")` was `None` for every
non-row lot, so every house of one style (27 `cottage`s on the reviewed
hurricane plat) came out in the exact same colour: the style's one hardcoded
default. `modular_house.draw_house_palette` is the fix — a per-house draw
from `HOUSE_PALETTE_POOL` — and this file pins it WITHOUT a stage, without
Isaac Sim and without `pxr`: it is pure arithmetic on a hash, exactly like
`suburb_scene`'s own `mod_share` draw that it is modelled on.

WHAT A STABLE HASH HAS TO PROVE, that an `rng.random()` draw would not need
proving: that it actually VARIES across different house identities (a hash
that always lands in the same bucket is indistinguishable from no draw at
all), that it REPRODUCES for the same identity (the whole point of using a
hash instead of a live draw — a re-run of the same seed must repaint the same
house the same colour), and that it OBEYS a weights override the way
`_pick_weighted`-style draws elsewhere in this codebase do (row_housing.
draw_palette_set is the reference for that contract).

WHAT IT CANNOT SEE: whether the draw's result ever reaches a rendered wall —
that is `test_hurricane_palettes_rebind.py`, which needs `pxr` and checks the
launcher's live-rebind mechanism on a synthetic archetype.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from detail import modular_house as mh                        # noqa: E402


# ---------------------------------------------------------------------------
# The pool itself
# ---------------------------------------------------------------------------
def test_pool_is_residential_and_known_to_palettes():
    """Every name in the pool must resolve in `PALETTES`, and the two
    explicitly commercial finishes (worn-asphalt roof, poured concrete) must
    not be in it — the same exclusion `row_housing.PALETTE_SETS` makes and
    for the same reason: nobody paints a house `concrete`."""
    assert len(mh.HOUSE_PALETTE_POOL) >= 4
    for name in mh.HOUSE_PALETTE_POOL:
        assert name in mh.PALETTES, name
    assert "concrete" not in mh.HOUSE_PALETTE_POOL
    assert "concrete_pale" not in mh.HOUSE_PALETTE_POOL


def test_pool_has_no_duplicates():
    assert len(set(mh.HOUSE_PALETTE_POOL)) == len(mh.HOUSE_PALETTE_POOL)


# ---------------------------------------------------------------------------
# Determinism: the same (seed, house) pair always repaints the same house.
# ---------------------------------------------------------------------------
def test_deterministic_for_the_same_key():
    key = (11, 4, (123.5, -67.25))
    a = mh.draw_house_palette(key)
    b = mh.draw_house_palette(key)
    assert a == b
    assert a in mh.HOUSE_PALETTE_POOL


def test_deterministic_across_repeated_calls_many_keys():
    """Not just one lucky key — a spread of the shapes `suburb_scene` will
    actually pass (seed, parcel id, lot centre)."""
    keys = [(11, p, (float(p) * 17.3, float(p) * -9.1))
            for p in range(40)]
    first = [mh.draw_house_palette(k) for k in keys]
    second = [mh.draw_house_palette(k) for k in keys]
    assert first == second


# ---------------------------------------------------------------------------
# It is a DRAW, not a constant: different house identities land differently.
# ---------------------------------------------------------------------------
def test_varies_across_house_identity_at_one_seed():
    """The bug this replaces was EXACTLY 'every house of a style is the same
    colour'. A fix that always returns one name would reproduce it with a
    different name. Sampled over enough houses that a 6-way pool landing on
    one bucket every time is not a plausible coincidence."""
    seed = 11
    draws = [mh.draw_house_palette((seed, i, (float(i) * 31.7, float(i) * -5.3)))
             for i in range(60)]
    assert len(set(draws)) >= 3, (
        "expected several distinct palettes across 60 houses, got {0}"
        .format(sorted(set(draws))))


def test_varies_across_seed_for_one_house():
    """Two different scene seeds must be able to disagree about the same
    house — otherwise `config["seed"]`/HUR_SEED changing would not actually
    reroll the paint job, the same class of dead knob the surge `SURGE=0`
    bug (see `suburb_hurricane_launch_script.py`) was."""
    house_key = (7, (44.0, -12.0))
    draws = {mh.draw_house_palette((seed,) + house_key) for seed in range(30)}
    assert len(draws) >= 2, draws


# ---------------------------------------------------------------------------
# It respects an explicit weights override, the same contract
# `row_housing._pick_weighted` gives `draw_palette_set`.
# ---------------------------------------------------------------------------
def test_single_nonzero_weight_always_wins():
    for name in mh.HOUSE_PALETTE_POOL:
        weights = {n: (1.0 if n == name else 0.0) for n in mh.HOUSE_PALETTE_POOL}
        for i in range(15):
            got = mh.draw_house_palette((3, i, (i, -i)), weights=weights)
            assert got == name, (name, i, got)


def test_all_zero_weights_returns_none():
    weights = {n: 0.0 for n in mh.HOUSE_PALETTE_POOL}
    assert mh.draw_house_palette((1, 2, (0.0, 0.0)), weights=weights) is None


def test_empty_weights_dict_falls_back_to_uniform_pool():
    """`weights={}` is falsy, same branch as `weights=None` in the caller
    (`suburb_scene.build_placements` reads a config key that may be an empty
    dict when a preset declares the section but not this key)."""
    key = (9, 1, (5.0, 5.0))
    assert mh.draw_house_palette(key, weights={}) == mh.draw_house_palette(key)


def test_weights_outside_the_pool_are_ignored():
    """A config typo or a row-only palette name in `house_palette_weights`
    must not crash the draw or silently win against the whole pool."""
    weights = {"not_a_real_palette": 5.0, "wood_white": 1.0}
    for i in range(20):
        got = mh.draw_house_palette((2, i, (i, i)), weights=weights)
        assert got == "wood_white"


def test_weights_bias_the_distribution():
    """Not a single-key case: a 10x weight on one name should dominate a
    reasonably sized sample without being the ONLY thing drawn (that is
    `test_single_nonzero_weight_always_wins`'s job)."""
    heavy = "brick_red"
    weights = {n: (10.0 if n == heavy else 1.0) for n in mh.HOUSE_PALETTE_POOL}
    n = 300
    draws = [mh.draw_house_palette((5, i, (i * 3.1, i * -2.2)), weights=weights)
             for i in range(n)]
    frac_heavy = draws.count(heavy) / float(n)
    # Expected share under these weights: 10 / (10 + 5*1) = 0.667. Loose
    # bounds — this is a hash walk, not a PRNG, and the point is "clearly
    # biased", not "converges to the analytic fraction".
    assert frac_heavy > 0.40, frac_heavy


# ---------------------------------------------------------------------------
# The style-default fallback this replaces is still reachable, for the
# non-parcel callers (`build_catalogue`, `build_row`, `build_plaza`) that
# have no per-house identity to hash and still want one look per style.
# ---------------------------------------------------------------------------
def test_style_default_still_present_for_non_parcel_callers():
    for style, spec in mh.STYLES.items():
        pal = spec.get("palette")
        if pal:
            assert pal in mh.PALETTES, (style, pal)


if __name__ == "__main__":
    for _name, _fn in sorted(globals().items()):
        if _name.startswith("test_") and callable(_fn):
            _fn()
            print("ok  " + _name)
