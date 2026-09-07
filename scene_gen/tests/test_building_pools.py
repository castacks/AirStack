"""The condition of a building is a TAG, not a pool name — and the reader that
says so must give the same answers for a pack written either way.

`usds.buildings` is keyed by TYPOLOGY (`tower` / `midrise` / `rowhouse`) and
each entry carries `tags: [intact|damaged|destroyed]`. Older packs key by
CONDITION instead and put the typology underneath (or nowhere, for the flat
`damaged` pool). `scene_generator.building_entries` reads both, and everything
downstream — `build_city`'s pools, `districts`' rezoning, the disaster stage's
fate swap, the Stage A baker — goes through it.

The load-bearing property is that CONVERTING A PACK CHANGES NO SCENE: the same
buildings come out of the same pools in the same order, so the layout RNG draws
identically. That is what `test_layouts_match_across_layouts` pins.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import scene_generator as sg                                       # noqa: E402

TOWER_A = {"usd": "a_tower.usd", "material": "concrete"}
TOWER_RUIN = {"usd": "a_ruin.usd", "material": "steel"}
MID_A = {"usd": "a_block.usd", "material": "brick"}
MID_HURT = {"usd": "a_hurt_block.usd", "scale": 0.01}
ROW_A = {"usd": "a_house.usd", "material": "brick"}


def _tagged():
    return {"usds": {"buildings": {
        "tower": [dict(TOWER_A, tags=["intact"]), dict(TOWER_RUIN, tags=["destroyed"])],
        "midrise": [dict(MID_A, tags=["intact"]), dict(MID_HURT, tags=["damaged"])],
        "rowhouse": [dict(ROW_A, tags=["intact"])],
    }}}


def _legacy():
    return {"usds": {"buildings": {
        "intact": {"tower": [TOWER_A], "midrise": [MID_A], "rowhouse": [ROW_A]},
        "damaged": [MID_HURT],
        "destroyed": {"tower": [TOWER_RUIN]},
    }}}


def _usds(entries):
    return [e["usd"] if isinstance(e, dict) else e for e in entries]


@pytest.mark.parametrize("cfg", [_tagged(), _legacy()], ids=["tagged", "legacy"])
def test_condition_pools_are_the_same_in_both_layouts(cfg):
    assert _usds(sg.building_entries(cfg, condition="intact")) == \
        ["a_tower.usd", "a_block.usd", "a_house.usd"]
    assert _usds(sg.building_entries(cfg, condition="damaged")) == ["a_hurt_block.usd"]
    assert _usds(sg.building_entries(cfg, condition="destroyed")) == ["a_ruin.usd"]


@pytest.mark.parametrize("cfg", [_tagged(), _legacy()], ids=["tagged", "legacy"])
def test_a_typology_pool_is_intact_art_only(cfg):
    """What `districts` rebuilds a block from. A ruin in here would mean
    rezoning could 'repair' a block by dropping an authored collapse on it."""
    assert _usds(sg.building_entries(cfg, condition="intact", typology="tower")) \
        == ["a_tower.usd"]
    assert _usds(sg.building_entries(cfg, condition="intact", typology="midrise")) \
        == ["a_block.usd"]


@pytest.mark.parametrize("cfg", [_tagged(), _legacy()], ids=["tagged", "legacy"])
def test_the_baker_sees_every_intact_building_and_no_ruin(cfg):
    every = set(_usds(sg.every_building(cfg, "intact")))
    assert every == {"a_tower.usd", "a_block.usd", "a_house.usd"}
    assert not every & {"a_ruin.usd", "a_hurt_block.usd"}


def test_an_untagged_entry_is_intact():
    """Only ruins have to say so; a pool of plain strings still builds a city."""
    cfg = {"usds": {"buildings": {"midrise": ["plain.usd", {"usd": "d.usd"}]}}}
    assert sg.condition_of("plain.usd") == "intact"
    assert _usds(sg.building_entries(cfg, condition="intact")) == ["plain.usd", "d.usd"]
    assert sg.building_entries(cfg, condition="destroyed") == []


def test_legacy_typology_pool_at_the_top_level_still_resolves():
    """`urban.yaml` keeps its intact typology art in TOP-LEVEL `tower:` /
    `midrise:` pools beside a flat `intact:` — the shape every pre-2026-08-26
    pack has, and the one whose pools decide those scenes' layouts."""
    cfg = {"usds": {"buildings": {
        "intact": [TOWER_A, MID_A],
        "tower": [TOWER_A],
        "midrise": [MID_A],
        "destroyed": [TOWER_RUIN],
    }}}
    assert _usds(sg.building_entries(cfg, condition="intact")) == \
        ["a_tower.usd", "a_block.usd"]
    assert _usds(sg.building_entries(cfg, condition="intact", typology="tower")) \
        == ["a_tower.usd"]
    # the baker takes the union; the generators take one pool at a time
    assert set(_usds(sg.every_building(cfg, "intact"))) == {"a_tower.usd", "a_block.usd"}


def test_the_shipped_pack_is_tagged_and_holds_no_ruin_in_a_typology_pool():
    cfg = sg.resolve_asset_pack({"asset_pack": "urban_v2"})
    bld = cfg["usds"]["buildings"]
    assert set(bld) == {"tower", "midrise", "rowhouse"}, \
        "urban_v2 is keyed by typology; condition is a tag"
    intact = set(_usds(sg.building_entries(cfg, condition="intact")))
    ruins = set(_usds(sg.building_entries(cfg, condition="damaged"))) | \
        set(_usds(sg.building_entries(cfg, condition="destroyed")))
    assert intact and ruins and not (intact & ruins)
    for typ in ("tower", "midrise", "rowhouse"):
        assert not set(_usds(sg.building_entries(cfg, condition="intact",
                                                 typology=typ))) & ruins


def test_layouts_match_across_layouts():
    """Converting a pack must not move a single building.

    The tagged pack and a mechanically de-tagged copy of it are handed to the
    same builder; the placements have to come out identical, because the pools
    feed an RNG and a different ORDER is a different city.
    """
    import copy
    tagged = sg.resolve_asset_pack({"asset_pack": "urban_v2"})
    legacy = copy.deepcopy(tagged)
    bld, out = legacy["usds"]["buildings"], {}
    for typ, pool in bld.items():
        for e in pool:
            cond = sg.condition_of(e)
            e = {k: v for k, v in e.items() if k != "tags"}
            out.setdefault(cond, {}).setdefault(typ, []).append(e)
    legacy["usds"]["buildings"] = out
    for cond in ("intact", "damaged", "destroyed"):
        assert _usds(sg.building_entries(tagged, condition=cond)) == \
            _usds(sg.building_entries(legacy, condition=cond))
        for typ in ("tower", "midrise", "rowhouse"):
            assert _usds(sg.building_entries(tagged, condition=cond, typology=typ)) == \
                _usds(sg.building_entries(legacy, condition=cond, typology=typ))
