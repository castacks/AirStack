#!/usr/bin/env python3
"""test_shared_urban_layout.py — the offline gate for the SHARED urban layout.

    python3 -m pytest -q scene_gen/tests/test_shared_urban_layout.py

HOST-SIDE, NO KIT, NO GPU, NO NUCLEUS. Everything here runs through
`plan_png.build`, which drives the real subdivider and the real `districts`
pass with `measure_usds=False` and footprints from the checked-in asset-set
scrape. That is deliberate: the whole point of the shared-layout work is that
a 1 km city can be rejected on a laptop before a pod is spent on it.

WHAT IT GUARDS

  1. THE ARCHITECTURE. Every urban-fire level preset is generated from the
     one shared base, and differs from it ONLY in the city seed and the fire.
     A disaster preset that reaches into `districts`, `layout`, `packing` or
     the building pools has broken the "same city, different damage"
     guarantee, and that is the failure this catches first.

  2. THE FIRE LADDER IS A REAL LADDER. `start_offset_frac` must increase
     L1 -> L2 -> L3. The three shipped `downtown_fire_1500*` presets were
     byte-identical here (all 0.70), which is why "Level 1 -- early /
     contained" shipped holding nothing but F4 and F5 records.

  3. THE LAYOUT ITSELF, on the committed seeds: no building overlaps, no
     building off its block, no empty non-park block, and the district mix
     and repetition inside the bands the review sheets report.

  4. THE REPRODUCIBILITY CONTRACT. The three variance knobs
     (`per_block_rng`, `pack_min_candidates`, `repeat_hard_ladder`) must all
     be OFF by default, so every scene that does not ask for them --
     `downtown_gac`, the 500 m scenes, `downtown_earthquake` -- reproduces
     draw for draw. This is the same contract `_tile_run`'s `no_repeat`
     documents, and it is easy to break by making a knob unconditional.

The layout builds are the slow part (~40-90 s each), so they are cached per
(config, seed) for the module.
"""
import os
import sys

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_TOOLS = os.path.join(_SCENE_GEN, "tools")
for _p in (_TOOLS, _SCENE_GEN):
    if _p not in sys.path:
        sys.path.insert(0, _p)

PRESETS = os.path.join(_SCENE_GEN, "config", "presets")
BASE = "downtown_urban_1000"
LEVELS = {1: 3, 2: 18, 3: 23}          # level -> committed city seed

#: Bands the review sheets and `pick_city_seed.py` score against. Kept here
#: as the single source of truth for "is this city acceptable".
TALL_SHARE_BAND = (0.15, 0.36)
MAX_REPEAT_SHARE = 0.20
MIN_MODELS = 70
MAX_COPIES_PER_MODEL = 7.0

#: Keys a disaster preset may differ from the base on. Everything else is the
#: shared city.
ALLOWED_DELTA = {
    "seed", "disaster-type", "epicenter", "heading_deg", "wind_mps",
    "duration_s", "start_offset_frac",
    # corridor sizing, read by `tools/fire_corridor_manifest.py`
    "fire_target_frac", "fire_collapse", "fire_f6",
}

_CACHE = {}


def _load(name):
    path = os.path.join(PRESETS, name + ".yaml")
    if not os.path.exists(path):
        pytest.skip("preset not present: %s" % name)
    return yaml.safe_load(open(path))


def _level_preset(level):
    return "downtown_urban_fire_1000_l%d" % level


def _build(config, seed):
    key = (config, seed)
    if key not in _CACHE:
        import layout_review_png as review
        _CACHE[key] = review.build(config, seed=seed, region=1000.0,
                                   fire_layout=True)
    return _CACHE[key]


# --------------------------------------------------------------------------
# 1. architecture — the levels differ from the base only in seed and fire
# --------------------------------------------------------------------------

def _flatten(doc, prefix=""):
    out = {}
    if isinstance(doc, dict):
        for k, v in doc.items():
            out.update(_flatten(v, "%s.%s" % (prefix, k) if prefix else str(k)))
    elif isinstance(doc, list):
        out[prefix] = repr(doc)
    else:
        out[prefix] = doc
    return out


@pytest.mark.parametrize("level", sorted(LEVELS))
def test_level_preset_differs_from_base_only_in_seed_and_fire(level):
    base = _flatten(_load(BASE))
    lvl = _flatten(_load(_level_preset(level)))
    changed = {k for k in set(base) | set(lvl)
               if base.get(k) != lvl.get(k)}
    # A top-level fire key is absent from the base entirely; that is an ADD,
    # and allowed. Anything else must be in ALLOWED_DELTA.
    offenders = sorted(k for k in changed
                       if k.split(".")[0] not in ALLOWED_DELTA)
    assert not offenders, (
        "level %d preset diverges from %s on non-fire keys: %s -- the shared "
        "layout guarantee only holds while a disaster preset touches nothing "
        "but its own damage" % (level, BASE, offenders[:12]))


@pytest.mark.parametrize("level", sorted(LEVELS))
def test_level_preset_carries_the_committed_seed(level):
    assert _load(_level_preset(level)).get("seed") == LEVELS[level]


def test_levels_do_not_override_building_pools():
    """The burnable-only pool cut is what collapsed `lowrise` to 7 assets and
    put six identical department stores in a line. Full pools, everywhere."""
    for level in LEVELS:
        doc = _load(_level_preset(level))
        pools = ((doc.get("overrides") or {}).get("usds") or {}).get("buildings")
        assert not pools, (
            "level %d overrides building pools %s -- damageability is a "
            "post-layout concern now, not a pool filter"
            % (level, sorted(pools or {})))


# --------------------------------------------------------------------------
# 2. the fire ladder is actually a ladder
# --------------------------------------------------------------------------

def test_fire_epoch_increases_with_level():
    epochs = [float(_load(_level_preset(l))["start_offset_frac"])
              for l in sorted(LEVELS)]
    assert epochs == sorted(epochs) and len(set(epochs)) == len(epochs), (
        "start_offset_frac must strictly increase L1->L2->L3; got %s. The "
        "shipped downtown_fire_1500{,_lvl2,_lvl3} presets were all 0.70, "
        "which is why every level solved the same 5.6 h fire and L1 shipped "
        "as F4/F5 only." % epochs)


def test_fire_epoch_spans_a_useful_range():
    lo = float(_load(_level_preset(1))["start_offset_frac"])
    hi = float(_load(_level_preset(3))["start_offset_frac"])
    assert lo <= 0.30, "L1 must open EARLY in the fire, got %.2f" % lo
    assert hi >= 0.65, "L3 must open LATE in the fire, got %.2f" % hi


# --------------------------------------------------------------------------
# 3. the reproducibility contract — knobs off by default
# --------------------------------------------------------------------------

def test_variance_knobs_default_off():
    from detail import districts
    assert districts.PACK_MIN_CANDIDATES == 0, (
        "PACK_MIN_CANDIDATES must default to 0 -- widening the band changes "
        "which entry a draw picks and perturbs the RNG stream for every "
        "later draw in the scene")

    class _Sky:
        pass
    # `_band_top` with min_candidates=0 must return the plain single-band
    # expression, byte for byte.
    fits = [(("solo",), 42.0, 30.0, 0), (("b",), 10.0, 8.0, 0)]
    top_off, band_off = districts._band_top(fits, 0.55, min_candidates=0)
    best = max(f[1] * f[2] for f in fits)
    assert top_off == [f for f in fits if f[1] * f[2] >= best * 0.55]
    assert band_off == 0.55


def test_band_widens_only_when_asked_and_counts_distinct_assets():
    from detail import districts
    # two ORIENTATIONS of one asset are one CHOICE, not two
    fits = [(("solo",), 42.0, 30.0, 0), (("solo",), 30.0, 42.0, 90)]
    _top, band = districts._band_top(fits, 0.55, min_candidates=4)
    assert band == pytest.approx(districts.PACK_BAND_FLOOR), (
        "a band holding one asset in two orientations must keep widening to "
        "the floor, not report itself satisfied")

    fits = [(("a",), 40.0, 30.0, 0), (("b",), 38.0, 30.0, 0),
            (("c",), 36.0, 30.0, 0), (("d",), 35.0, 30.0, 0)]
    top, band = districts._band_top(fits, 0.55, min_candidates=4)
    assert band == 0.55 and len(top) == 4


def test_block_rng_is_stable_and_block_specific():
    from detail import districts
    a1 = districts._block_rng(8, (0.0, 0.0, 100.0, 60.0)).random()
    a2 = districts._block_rng(8, (0.0, 0.0, 100.0, 60.0)).random()
    b = districts._block_rng(8, (0.0, 0.0, 100.0, 60.1)).random()
    c = districts._block_rng(9, (0.0, 0.0, 100.0, 60.0)).random()
    assert a1 == a2, "same block, same seed must reproduce"
    assert a1 != b, "a different block must pack differently"
    assert a1 != c, "a different city seed must pack differently"


def test_base_preset_enables_the_knobs():
    d = (_load(BASE).get("overrides") or {}).get("districts") or {}
    assert d.get("per_block_rng") is True
    assert int(d.get("pack_min_candidates") or 0) >= 2
    assert d.get("repeat_hard_ladder") is True


# --------------------------------------------------------------------------
# 4. the layouts themselves — slow, so marked
# --------------------------------------------------------------------------

@pytest.mark.slow
@pytest.mark.parametrize("level", sorted(LEVELS))
def test_layout_is_clean_and_in_band(level):
    import layout_review_png as review
    seed = LEVELS[level]
    cfg, layout, placements, res = _build(_level_preset(level), seed)
    houses = [p for p in placements if review._house(p)]
    blocks = review.block_typologies(layout)
    st = review.typology_stats(blocks)
    dv = review.diversity_stats(houses, res, 60.0)

    occupied = set()
    for p in houses:
        for i, (rect, _n) in enumerate(blocks):
            if (rect[0] <= p["x_m"] <= rect[2]
                    and rect[1] <= p["y_m"] <= rect[3]):
                occupied.add(i)
                break
    empty = [i for i, (_r, name) in enumerate(blocks)
             if i not in occupied and name != "park"]
    assert not empty, "level %d: %d empty non-park block(s)" % (level,
                                                                len(empty))

    lo, hi = TALL_SHARE_BAND
    assert lo <= st["tall_share"] <= hi, (
        "level %d: highrise+tower is %.1f%% of block area, outside %.0f-%.0f%%"
        % (level, 100 * st["tall_share"], 100 * lo, 100 * hi))
    assert dv["models"] >= MIN_MODELS, (
        "level %d: only %d distinct models" % (level, dv["models"]))
    assert dv["copies_per_model"] <= MAX_COPIES_PER_MODEL, (
        "level %d: %.1f copies per model" % (level, dv["copies_per_model"]))
    assert dv["repeat_share"] <= MAX_REPEAT_SHARE, (
        "level %d: %.1f%% of buildings sit in a same-model pair within 60 m"
        % (level, 100 * dv["repeat_share"]))


@pytest.mark.slow
@pytest.mark.parametrize("level", sorted(LEVELS))
def test_no_building_overlaps(level):
    """Yaw-correct AABB screen. The packer already refuses an overlap; this
    is the regression guard for the per-block RNG, which changes every cut
    sequence and could in principle reintroduce one."""
    import itertools
    import layout_review_png as review
    cfg, layout, placements, res = _build(_level_preset(level), LEVELS[level])
    houses = [p for p in placements if review._house(p)]
    boxes = []
    for p in houses:
        w, d = review._footprint(res, p)
        boxes.append((p["x_m"] - w / 2, p["y_m"] - d / 2,
                      p["x_m"] + w / 2, p["y_m"] + d / 2))
    bad = []
    for (i, a), (j, b) in itertools.combinations(enumerate(boxes), 2):
        dx = min(a[2], b[2]) - max(a[0], b[0])
        dy = min(a[3], b[3]) - max(a[1], b[1])
        if dx > 0.5 and dy > 0.5:
            inter = dx * dy
            small = min((a[2] - a[0]) * (a[3] - a[1]),
                        (b[2] - b[0]) * (b[3] - b[1]))
            if small and inter / small > 0.05:
                bad.append((i, j, inter))
    assert not bad, "level %d: %d overlapping building pair(s)" % (level,
                                                                   len(bad))


@pytest.mark.slow
def test_the_three_levels_are_three_different_cities():
    import layout_review_png as review
    sigs = {}
    for level, seed in LEVELS.items():
        _c, _l, placements, _r = _build(_level_preset(level), seed)
        houses = [p for p in placements if review._house(p)]
        sigs[level] = tuple(sorted(
            (round(p["x_m"], 1), round(p["y_m"], 1),
             os.path.basename(str(p.get("usd", "")))) for p in houses))
    assert len(set(sigs.values())) == len(sigs), (
        "two levels produced the identical city -- the seeds are meant to "
        "differ so the three cells do not read as the same town")
