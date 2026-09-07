"""The two repeat penalties: don't overuse a model, and never stand it next to
a copy of itself.

`_Skyline` decides WHICH model fills a slot the packer has already sized. Height
match alone concentrated the city on a handful of assets — an asset is eligible
whenever it FITS, so small footprints won far more slots than large ones — and
the global `repeat_penalty` fixed the histogram without fixing what a viewer
actually sees: two identical towers across one street. These pin the local term
that does, plus the bookkeeping the second pass needs to see the first pass's
buildings.
"""
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detail.districts import _Skyline                             # noqa: E402

LOCAL = {"repeat_penalty": 0.0, "repeat_radius_m": 150.0,
         "repeat_local_penalty": 0.02, "repeat_local_falloff": 2.0}


def _entry(usd, h=30.0):
    """The prefix of a pool entry that `_Skyline` reads — usd and footprint.
    `_pool_entries` builds a 6-tuple (yaw-offset and facing meta follow), none
    of which `choose`/`note` look at."""
    return (usd, None, "Z", {"sx": 20.0, "sy": 20.0, "sz": h, "base": 0.0})


def _draws(cfg, seeded, n=400):
    """Draw *n* slots at the origin from two same-height models, after standing
    `seeded` copies of "a" nearby. Returns how often "a" won."""
    hits = 0
    for s in range(n):
        sky = _Skyline(cfg, random.Random(s))
        for i in range(seeded):
            sky.note("a", 10.0 * (i + 1), 0.0)
        got = sky.choose([_entry("a"), _entry("b")], 30.0, 0.0, 0.0)
        hits += got[0] == "a"
    return hits


def test_radius_zero_leaves_the_local_term_inert():
    """The default. Two equal models split the draw, as they did before."""
    assert 150 <= _draws(dict(LOCAL, repeat_radius_m=0.0), seeded=1) <= 250


def test_a_copy_within_sight_all_but_loses_the_slot():
    assert _draws(LOCAL, seeded=1) <= 20


def test_a_second_copy_compounds():
    assert _draws(LOCAL, seeded=2) <= _draws(LOCAL, seeded=1)


def test_distance_grades_the_penalty():
    """A twin across the street is a worse defect than one at the rim, and the
    exponent has to say so — a flat count scores them identically."""
    sky = _Skyline(LOCAL, random.Random(0))
    sky.note("a", 5.0, 0.0)
    near = sky._near("a", 0.0, 0.0)
    sky = _Skyline(LOCAL, random.Random(0))
    sky.note("a", 149.0, 0.0)
    far = sky._near("a", 0.0, 0.0)
    assert near > far > 1.0
    assert sky._near("a", 400.0, 0.0) == 0.0        # outside the radius


def test_the_floor_falls_back_to_fewest_copies_nearby():
    """Every candidate penalised into nothing still has to fill the slot, and
    the fallback is the visible term, not chance and not the histogram."""
    sky = _Skyline(dict(LOCAL, repeat_local_penalty=0.0), random.Random(0))
    for i in range(3):
        sky.note("a", 10.0 * (i + 1), 0.0)
    sky.note("b", 10.0, 0.0)
    assert sky.choose([_entry("a"), _entry("b")], 30.0, 0.0, 0.0)[0] == "b"


def test_note_carries_the_first_pass_into_the_second():
    """`infill_blocks` builds its own `_Skyline` over an already-built city.
    Without `note` it sees an empty histogram and repeats what is standing."""
    sky = _Skyline(LOCAL, random.Random(0))
    assert sky.diversity() == (0, 0, 0.0)
    sky.note("a", 0.0, 0.0)
    assert sky.diversity() == (1, 1, 1.0)
    assert sky._near("a", 10.0, 0.0) > 0.0


def test_an_unreachable_target_still_spreads_the_draw():
    """No model anywhere near the target height used to hand EVERY such slot to
    the single nearest one. It is now a weighted draw like any other."""
    cfg = {"repeat_penalty": 0.0, "repeat_radius_m": 0.0, "pick_sigma": 0.30}
    pool = [_entry("tall", 200.0), _entry("taller", 210.0)]
    got = {_Skyline(cfg, random.Random(s)).choose(pool, 5.0, 0.0, 0.0)[0]
           for s in range(50)}
    assert got == {"tall", "taller"}


def test_diversity_reports_the_share_of_the_most_used():
    sky = _Skyline(LOCAL, random.Random(0))
    for _ in range(3):
        sky.note("a", 0.0, 0.0)
    sky.note("b", 0.0, 0.0)
    assert sky.diversity() == (2, 4, 0.75)
