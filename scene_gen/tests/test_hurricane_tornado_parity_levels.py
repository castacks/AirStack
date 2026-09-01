"""test_hurricane_tornado_parity_levels.py — locks
`hurricane.tornado_level_for_intensity` (STREAM S, 2026-08-31: "adjust the
pattern of house damage" onto the tornado's own six-level vocabulary)
against the REAL house populations recorded in two actual built scenes.

WHY REAL GT, NOT A SYNTHETIC SAMPLE. `hurricane._HOUSE_CUTS`'s own comment
documents two rounds of a synthetic-only calibration disagreeing with a
rendered scene (a claimed 25% structural landing at 38% once replayed
against real data). `hurricane.tornado_level_for_intensity`'s cuts
(`hurricane._TORNADO_LEVEL_CUTS` / `_TORNADO_LEVEL_JITTER`) were fit the
same way this file verifies: against the exact (intensity, vulnerability)
pairs recorded in
`~/hurricane_previews/FINAL2_L2_brown/GT_hurricane.json` (136 houses,
site_gust_mps 55) and `FINAL2_L3_brown/GT_hurricane.json` (94 houses,
site_gust_mps 70).

No pxr needed — `hurricane.py` imports only `math`/`os` at module scope.

    python3 scene_gen/tests/test_hurricane_tornado_parity_levels.py
    pytest -q scene_gen/tests/test_hurricane_tornado_parity_levels.py
"""
import json
import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN_DIR)

from disaster import hurricane as hu                           # noqa: E402
from disaster import tornado as tn                             # noqa: E402

GT_PATHS = {
    "L2": os.path.expanduser(
        "~/hurricane_previews/FINAL2_L2_brown/GT_hurricane.json"),
    "L3": os.path.expanduser(
        "~/hurricane_previews/FINAL2_L3_brown/GT_hurricane.json"),
}

_HAVE_L2 = os.path.exists(GT_PATHS["L2"])
_HAVE_L3 = os.path.exists(GT_PATHS["L3"])

pytestmark = pytest.mark.skipif(
    not (_HAVE_L2 and _HAVE_L3),
    reason="GT_hurricane.json fixtures not present at {0}".format(GT_PATHS))

STRUCTURAL = ("roof_collapsed", "partial_collapse", "leveled")

# Target bands from the master brief -- see `hurricane.py`'s
# `_TORNADO_LEVEL_CUTS` comment for the fit that hits them.
# RE-CUT 2026-09-01, user doctrine (live review): "Tornado and hurricane are
# both largely wind damage. Besides directionality the damage is similar."
# The survey-conservative structural share is overridden by design intent:
# a tornado-like damage mix, scattered by the hurricane field. `swept`
# remains surge-only at every level.
TARGETS = {
    "L2": {"pristine": (0.45, 0.65), "roof_stripped": (0.30, 0.45),
          "structural_max": 0.10},
    "L3": {"pristine": (0.20, 0.30), "roof_stripped": (0.38, 0.48),
          "roof_collapsed": (0.13, 0.20), "partial_collapse": (0.05, 0.10),
          "leveled_max": 0.08},
}


def load_gt(path):
    """`[(intensity, vulnerability), ...]` for every house in a GT file."""
    with open(path) as fh:
        doc = json.load(fh)
    return [(float(h["intensity"]), float(h["vulnerability"]))
            for h in doc["houses"]]


def tally(pairs, trials=4000, seed=20260831):
    """`{level: fraction}` -- the SAME per-house blend
    `tornado_level_for_intensity` runs (`_resistance(vuln)` then a jitter
    draw), resampled `trials` times per house for a smooth expectation
    instead of one noisy draw. Mirrors
    `tools/hurricane_house_cut_search.tally`'s own method for the 8-level
    ladder, applied to the six-level one this stream adds.
    """
    rng = random.Random(seed)
    counts = {lv: 0 for lv in tn.HOUSE_LEVELS}  # pristine..swept
    total = 0
    for it, vuln in pairs:
        eff0 = it / hu._resistance(vuln)
        for _ in range(trials):
            lv = hu._ladder(
                hu._TORNADO_LEVEL_CUTS,
                eff0 + rng.uniform(-hu._TORNADO_LEVEL_JITTER,
                                   hu._TORNADO_LEVEL_JITTER))
            counts[lv] += 1
            total += 1
    return {k: v / total for k, v in counts.items()}


def _within(name, dist, key, lo, hi):
    v = dist.get(key, 0.0)
    assert lo <= v <= hi, (
        "{0}: {1} = {2:.1%}, expected [{3:.0%}, {4:.0%}]"
        .format(name, key, v, lo, hi))


@pytest.mark.skipif(not _HAVE_L2, reason="FINAL2_L2_brown GT not present")
def test_l2_distribution_matches_target_bands():
    dist = tally(load_gt(GT_PATHS["L2"]))
    t = TARGETS["L2"]
    _within("L2", dist, "pristine", *t["pristine"])
    _within("L2", dist, "roof_stripped", *t["roof_stripped"])
    struct = sum(dist.get(k, 0.0) for k in STRUCTURAL)
    assert struct <= t["structural_max"], (
        "L2 structural = {0:.1%}, expected <= {1:.0%}"
        .format(struct, t["structural_max"]))
    assert dist.get("swept", 0.0) == 0.0, (
        "wind-only ladder must never emit 'swept': got {0}"
        .format(dist.get("swept")))


@pytest.mark.skipif(not _HAVE_L3, reason="FINAL2_L3_brown GT not present")
def test_l3_distribution_matches_target_bands():
    dist = tally(load_gt(GT_PATHS["L3"]))
    t = TARGETS["L3"]
    _within("L3", dist, "pristine", *t["pristine"])
    _within("L3", dist, "roof_stripped", *t["roof_stripped"])
    _within("L3", dist, "roof_collapsed", *t["roof_collapsed"])
    _within("L3", dist, "partial_collapse", *t["partial_collapse"])
    assert dist.get("leveled", 0.0) <= t["leveled_max"], (
        "L3 leveled = {0:.1%}, expected <= {1:.0%}"
        .format(dist.get("leveled", 0.0), t["leveled_max"]))
    assert dist.get("swept", 0.0) == 0.0, (
        "wind-only ladder must never emit 'swept': got {0}"
        .format(dist.get("swept")))


def test_function_never_returns_swept_regardless_of_input():
    """`swept` must be structurally unreachable from wind alone -- sweep the
    whole intensity range at both vulnerability extremes and confirm."""
    rng = random.Random(1)
    for i100 in range(0, 401):           # 0.00 .. 4.00 in 0.01 steps
        i = i100 / 100.0
        for vuln in (0.0, 0.5, 1.0):
            lv = hu.tornado_level_for_intensity(i, rng, vuln=vuln)
            assert lv != "swept", (i, vuln, lv)
            assert lv in tn.HOUSE_LEVELS, lv


def test_returned_levels_are_the_tornado_vocabulary():
    """Every level `_TORNADO_LEVEL_CUTS` can name is one of the tornado's
    own six -- a typo here would silently mint a level with no archetype
    file to reference."""
    names = {name for _, name in hu._TORNADO_LEVEL_CUTS}
    assert names.issubset(set(tn.HOUSE_LEVELS)), names
    assert "swept" not in names


if __name__ == "__main__":
    if _HAVE_L2:
        test_l2_distribution_matches_target_bands()
        print("ok  test_l2_distribution_matches_target_bands")
    if _HAVE_L3:
        test_l3_distribution_matches_target_bands()
        print("ok  test_l3_distribution_matches_target_bands")
    test_function_never_returns_swept_regardless_of_input()
    print("ok  test_function_never_returns_swept_regardless_of_input")
    test_returned_levels_are_the_tornado_vocabulary()
    print("ok  test_returned_levels_are_the_tornado_vocabulary")
    print("\nALL PASS")
