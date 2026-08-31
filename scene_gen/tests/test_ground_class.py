"""Offline checks for `disaster.ground_class` (round 5, WP E).

No `pxr` anywhere — `GroundClass` is pure Python over the plain dicts/tuples
`scene_generator.build_city` already returns as `city_layout`, so every check
here runs against a small synthetic layout built by hand.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from disaster import ground_class as gc  # noqa: E402


def _layout(**overrides):
    """A two-block layout: a LEFT block (0..48, 0..60), a vertical road
    corridor (48..52, 0..60), and a RIGHT block (52..100, 0..60).

    `paved_blocks` marks the left block's interior (once a 2 m sidewalk ring
    is removed) as paved; the right block carries no such record, so its
    interior should read as grass — the same default `build_city` uses for a
    park block or a suburb with `pave_blocks` off.
    """
    layout = {
        "region": (0.0, 0.0, 100.0, 60.0),
        "blocks": [(0.0, 0.0, 48.0, 60.0), (52.0, 0.0, 100.0, 60.0)],
        "road_corridors": [
            {"x0": 48.0, "y0": 0.0, "x1": 52.0, "y1": 60.0,
             "n_lanes": 2, "dir": "ns"},
        ],
        "paved_blocks": [(2.0, 2.0, 46.0, 58.0)],
    }
    layout.update(overrides)
    return layout


# ---------------------------------------------------------------------------
# basic classification, fallback sidewalk ring
# ---------------------------------------------------------------------------

def test_point_in_corridor_is_road():
    g = gc.GroundClass(_layout(), sidewalk_width_m=2.0)
    assert g.at(50.0, 30.0) == "road"
    assert g.at(49.9, 5.0) == "road"


def test_point_just_inside_block_edge_is_sidewalk():
    g = gc.GroundClass(_layout(), sidewalk_width_m=2.0)
    # left block's own edges: x=0 and x=48; a 2 m ring runs just inside both.
    assert g.at(1.0, 30.0) == "sidewalk"
    assert g.at(0.5, 0.5) == "sidewalk"
    # right block's near edge (x=52)
    assert g.at(53.0, 30.0) == "sidewalk"


def test_block_centre_is_paved_or_grass_per_block_type():
    g = gc.GroundClass(_layout(), sidewalk_width_m=2.0)
    # left block interior is in `paved_blocks`
    assert g.at(24.0, 30.0) == "paved"
    # right block carries no paved-block record -> grass, same default a
    # park block or an unpaved suburb block gets in `build_city`
    assert g.at(76.0, 30.0) == "grass"


def test_outside_region_is_grass():
    g = gc.GroundClass(_layout())
    assert g.at(-50.0, -50.0) == "grass"
    assert g.at(1000.0, 1000.0) == "grass"


def test_empty_layout_is_always_grass():
    g = gc.GroundClass({})
    assert g.at(0.0, 0.0) == "grass"
    assert g.at(123.4, -56.7) == "grass"
    g2 = gc.GroundClass(None)
    assert g2.at(5.0, 5.0) == "grass"


# ---------------------------------------------------------------------------
# priority order: road paints over sidewalk/paved, sidewalk over paved
# ---------------------------------------------------------------------------

def test_road_wins_over_an_overlapping_paved_block():
    # paved_blocks deliberately reaches PAST the left block's edge into the
    # road corridor (a badly-formed record, or a caller's own layout that
    # doesn't bother to clip) — the raster must still show the corridor as
    # road, not paved, because road is painted last.
    layout = _layout(paved_blocks=[(2.0, 2.0, 50.0, 58.0)])
    g = gc.GroundClass(layout, sidewalk_width_m=2.0)
    assert g.at(49.0, 30.0) == "road"


def test_sidewalk_wins_over_an_overlapping_paved_block():
    # the same over-reaching paved rect also overlaps the fallback sidewalk
    # ring at the left block's own edge (46..48) — sidewalk is painted AFTER
    # paved, so it must win there.
    layout = _layout(paved_blocks=[(2.0, 2.0, 50.0, 58.0)])
    g = gc.GroundClass(layout, sidewalk_width_m=2.0)
    assert g.at(47.0, 30.0) == "sidewalk"


# ---------------------------------------------------------------------------
# exact sidewalk_rects / paved_blocks are preferred over the fallback ring
# ---------------------------------------------------------------------------

def test_exact_sidewalk_rects_used_when_present():
    # a layout that hands over its own sidewalk strips (as `build_city`
    # always does) must use them verbatim rather than re-deriving a ring —
    # here the exact strip is only 0.5 m deep, much thinner than the 2 m
    # fallback would draw, and a point 1 m in should read "grass" only if
    # the exact record was actually honoured.
    layout = _layout(sidewalk_rects=[(0.0, 0.0, 48.0, 0.5)])
    g = gc.GroundClass(layout, sidewalk_width_m=2.0)
    assert g.at(24.0, 0.2) == "sidewalk"     # inside the thin exact strip
    # y=1.0 is past the exact strip's 0.5 m depth and short of where
    # `paved_blocks` starts (y=2.0) -> grass. A 2 m FALLBACK ring would have
    # claimed this same point as sidewalk, so this is what proves the exact
    # record was actually honoured rather than re-derived.
    assert g.at(24.0, 1.0) == "grass"


def test_small_block_is_all_sidewalk_in_fallback():
    # a block too small to hold two rings and an interior at the given
    # sidewalk width reads as sidewalk everywhere inside it.
    layout = {
        "region": (0.0, 0.0, 10.0, 10.0),
        "blocks": [(0.0, 0.0, 3.0, 3.0)],
        "road_corridors": [],
    }
    g = gc.GroundClass(layout, sidewalk_width_m=2.0)
    assert g.at(1.5, 1.5) == "sidewalk"


# ---------------------------------------------------------------------------
# look_for
# ---------------------------------------------------------------------------

def test_look_for_mapping():
    assert gc.look_for("road") == "asph"
    assert gc.look_for("sidewalk") == "pave"
    assert gc.look_for("paved") == "pave"
    assert gc.look_for("grass") == "soil"
    # unknown class degrades to the safest ("soil"), never raises
    assert gc.look_for("nonsense") == "soil"
    assert gc.look_for(None) == "soil"


def test_look_for_is_also_a_static_method():
    g = gc.GroundClass(_layout())
    assert g.look_for("road") == "asph"
    assert gc.GroundClass.look_for("grass") == "soil"


# ---------------------------------------------------------------------------
# from_config
# ---------------------------------------------------------------------------

def test_from_config_with_city_layout():
    config = {"_city_layout": _layout(), "roads": {"sidewalk_width_m": 2.0}}
    g = gc.GroundClass.from_config(config)
    assert g is not None
    assert g.at(24.0, 30.0) == "paved"
    assert g.at(76.0, 30.0) == "grass"
    assert g.at(50.0, 30.0) == "road"


def test_from_config_without_city_layout_returns_none():
    assert gc.GroundClass.from_config({}) is None
    assert gc.GroundClass.from_config({"roads": {}}) is None
    assert gc.GroundClass.from_config(None) is None


def test_from_config_roads_cfg_overrides_default_sidewalk_width():
    # a wide configured sidewalk should claim more ground near a block edge
    # than the module default when the layout has no exact `sidewalk_rects`.
    layout = _layout()
    config = {"_city_layout": layout, "roads": {"sidewalk_width_m": 6.0}}
    g = gc.GroundClass.from_config(config)
    # 4 m in from the left block's edge is grass at the 2 m default but
    # sidewalk at a configured 6 m ring.
    assert g.at(4.0, 30.0) == "sidewalk"


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
