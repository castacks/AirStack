"""Lock `hurricane._HOUSE_CUTS` against the REAL house population recorded
in GT_hurricane.json, not a synthetic sample.

See `hurricane.py`'s `_HOUSE_CUTS` comment and
`tools/hurricane_house_cut_search.py` for why: the ladder was calibrated
twice against an 800-synthetic-house sample and BOTH times the rendered
scene disagreed (38% structural against a claimed 25%, then a 39%
`roof_stripped` plurality against a claimed "roofs are the story"). This
test is the acceptance gate a future re-tune must clear, using the SAME
94/136-house populations this session was shown.

No pxr needed — `hurricane.py` imports only `math`/`os` at module scope, and
this test reads the recorded GT json directly.
"""
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_HERE)
_TOOLS_DIR = os.path.join(_SCENE_GEN_DIR, "tools")
sys.path.insert(0, _SCENE_GEN_DIR)
sys.path.insert(0, _TOOLS_DIR)

from hurricane_house_cut_search import (DEFAULT_GT, load_gt,  # noqa: E402
                                        named_metrics, struct_frac, tally)
from disaster import hurricane as hu                          # noqa: E402

_HAVE_L2 = os.path.exists(DEFAULT_GT["L2"])
_HAVE_L3 = os.path.exists(DEFAULT_GT["L3"])

pytestmark = pytest.mark.skipif(
    not (_HAVE_L2 and _HAVE_L3),
    reason="GT_hurricane.json fixtures not present at {0}".format(DEFAULT_GT))


def _dist(name):
    return tally(load_gt(DEFAULT_GT[name]), hu._HOUSE_CUTS, trials=800,
                seed=4242)


def test_l3_structural_under_ceiling():
    """The skill's own number: 'less than 15 percent of homes sustained
    structural wind damage' at 116-135 mph -- this scene's level 3 is more
    severe than that band (70 m/s site gust), so structural is allowed to
    run higher, but never past the ~15% ceiling the deliverable set."""
    d = _dist("L3")
    s = struct_frac(d)
    assert s <= 0.15, "L3 structural {0:.1%} exceeds the 15% ceiling".format(s)
    # and it should not have been squeezed to nothing either -- level 3
    # must visibly contain MORE broken houses than level 2.
    assert s >= 0.05, "L3 structural {0:.1%} is suspiciously low".format(s)


def test_l3_matches_five_named_targets():
    """Locks the ROUND-2 re-cut: all five bands at once, not just the top
    of the ladder (round 1 fixed structural alone and pushed the pile-up
    into `roof_stripped`, which then read as 'a field of white boxes')."""
    d = _dist("L3")
    pristine_ship, cover, deck, rstrip, struct = named_metrics(d)
    # generous +-6 pt tolerance: this is a resampled expectation over a
    # fixed 94-house draw, not a hard physical constant.
    assert pristine_ship == pytest.approx(0.15, abs=0.06)
    assert cover == pytest.approx(0.30, abs=0.06)
    assert deck == pytest.approx(0.25, abs=0.06)
    assert rstrip == pytest.approx(0.18, abs=0.06)
    assert struct == pytest.approx(0.12, abs=0.03)
    # no single roof-cover rung should dominate the way `roof_stripped` did
    # at 39% after round 1 -- that is what "a field of white boxes" meant.
    for k in ("shingles_lost", "cover_lost", "deck_panels_lost",
              "roof_stripped"):
        assert d.get(k, 0.0) <= 0.35, (
            "{0} at {1:.1%} is a plurality state again".format(k, d[k]))


def test_l2_essentially_unchanged():
    """L2's houses sit almost entirely below `deck_panels_lost`'s boundary,
    so widening the top three cuts to fix L3 must not perturb L2 at all --
    this is the load-bearing claim in the `_HOUSE_CUTS` comment."""
    d = _dist("L2")
    s = struct_frac(d)
    assert s <= 0.02, "L2 structural {0:.1%} should stay near zero".format(s)
    # L2 must look meaningfully lighter than L3 -- most of the plate still
    # under deck_panels_lost.
    assert d.get("pristine", 0.0) + d.get("shingles_lost", 0.0) >= 0.55


def test_l3_worse_than_l2():
    """The whole point of a size/intensity ladder: level 3 must read as
    worse than level 2, not just differently distributed."""
    d2, d3 = _dist("L2"), _dist("L3")
    assert struct_frac(d3) > struct_frac(d2)
    light2 = d2.get("pristine", 0.0) + d2.get("shingles_lost", 0.0)
    light3 = d3.get("pristine", 0.0) + d3.get("shingles_lost", 0.0)
    assert light3 < light2


def test_ladder_cuts_are_monotone():
    """`_ladder`'s (upper_bound, name) convention silently misclassifies
    everything above the first out-of-order row -- this is the offline
    shape check `tools/hurricane_house_cut_search.py` cannot catch by
    tallying alone (a locally-non-monotone table can still land on
    plausible fractions by accident)."""
    bounds = [b for b, _name in hu._HOUSE_CUTS]
    assert bounds == sorted(bounds)
    assert list(hu._HOUSE_CUTS[i][1] for i in range(len(hu._HOUSE_CUTS))) \
        == list(hu.HOUSE_LEVELS)
