"""The generated `frozen_*.yaml` scene overlays — offline, no ROS, no sim.

    python3 -m pytest robot/ros_ws/src/global/planners/search_baselines/tests/test_frozen_scene_overlays.py

These files are WRITTEN BY A GENERATOR (`scene_gen/tools/frozen_cell_plan.py
--emit-yaml`), so what needs guarding is not a human's typo — it is the
generator emitting something that parses as YAML, loads as ROS parameters, and
is silently the wrong shape. Every assertion below is a bug that has either
already happened or would be invisible until a whole OSMO iteration had been
flown:

* `search_area_xy` folding two numbers into one string across a wrapped line
  (this happened: the emitter joined lines without the trailing comma, YAML
  read "374.7 -454.8" as one scalar, and the polygon quietly lost a vertex and
  changed shape);
* an extent that does not reach the search polygon from the world origin,
  which crops the CoNavGPT2 team arm's merged grid without any error;
* `search_area_scene_key` naming a key the frozen region file does not carry
  (`burn` / `affected` belong to the fire model, not to a frozen cell);
* `search_area_pad_m` left non-zero, which pads AFTER the sector split and
  hands the two end drones a rectangle that is mostly off the plate.
"""

import glob
import math
import os
import sys

import pytest

yaml = pytest.importorskip("yaml")

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.normpath(os.path.join(_HERE, ".."))
_REPO = os.path.normpath(os.path.join(_PKG, "..", "..", "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(_PKG, "search_baselines"))
sys.path.insert(0, os.path.join(_REPO, "simulation", "isaac-sim", "utils"))

import sector as sect                      # noqa: E402
import frozen_annotations as fa            # noqa: E402

OVERLAYS = sorted(glob.glob(os.path.join(_PKG, "config", "frozen_*.yaml")))
NUM_ROBOTS = 8


def _params(path):
    with open(path, encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)
    # The top-level key must be the NODE NAME or every parameter is ignored,
    # silently, and the run flies planner.yaml's defaults.
    assert list(doc) == ["search_planner"], list(doc)
    return doc["search_planner"]["ros__parameters"]


def _poly(params):
    v = params["search_area_xy"]
    return [(v[i], v[i + 1]) for i in range(0, len(v), 2)]


def test_there_are_overlays_to_check():
    assert OVERLAYS, ("no frozen_*.yaml overlays — run "
                      "`python3 scene_gen/tools/frozen_cell_plan.py --robots 8 "
                      "--emit-yaml <this package>/config`")


@pytest.mark.parametrize("path", OVERLAYS, ids=os.path.basename)
def test_search_area_xy_is_a_flat_list_of_numbers(path):
    """The wrapped-line fold. Every element must be a float and the count even
    — a single string in the list is a lost vertex."""
    v = _params(path)["search_area_xy"]
    bad = [(i, x) for i, x in enumerate(v) if not isinstance(x, float)]
    assert not bad, f"non-numeric entries (a YAML line fold?): {bad[:4]}"
    assert len(v) % 2 == 0
    assert len(v) >= 6


@pytest.mark.parametrize("path", OVERLAYS, ids=os.path.basename)
def test_the_polygon_is_a_sane_convex_ring_on_the_plate(path):
    poly = _poly(_params(path))
    assert fa.polygon_area(poly) > 1e5, "a search area under 0.1 km2 on a 1 km plate"
    for x, y in poly:
        assert -501.0 <= x <= 501.0 and -501.0 <= y <= 501.0, (x, y)
    # The generator emits a convex hull; `sector.partition`'s rect bands are
    # only a tight fit on one.
    hull = fa.convex_hull(poly)
    assert fa.polygon_area(hull) == pytest.approx(fa.polygon_area(poly),
                                                  rel=1e-6)


@pytest.mark.parametrize("path", OVERLAYS, ids=os.path.basename)
def test_the_search_area_comes_from_the_scene_and_names_a_frozen_key(path):
    p = _params(path)
    assert p["search_area_source"] == "scene"
    # `burn` and `affected` are the FIRE MODEL's keys (scene_gen/disaster/
    # region.py) and a frozen cell's region file carries neither.
    assert p["search_area_scene_key"] == "search"
    assert p["search_area_frame"] == "world"
    # The pad is already in the polygon; padding again would pad after the
    # sector split.
    assert p["search_area_pad_m"] == 0.0
    assert p["sector_axis"] == "principal"


@pytest.mark.parametrize("path", OVERLAYS, ids=os.path.basename)
def test_the_extent_reaches_the_whole_area_from_the_world_origin(path):
    """The CoNavGPT2 TEAM arm is `frame_mode: 'global_enu'` with
    `map_origin_xy: [0, 0]` (conavgpt2_team.yaml, which loads AFTER this file
    and so cannot be corrected here). Its grid is centred on the plate, so the
    half-extent has to reach the farthest vertex of the search area or the
    merged map is cropped — with no error, and only a short trajectory to show
    for it."""
    p = _params(path)
    poly = _poly(p)
    need = max(math.hypot(x, y) for x, y in poly)
    assert p["map_extent_m"] / 2.0 >= need, (
        f"map_extent_m {p['map_extent_m']} gives a "
        f"{p['map_extent_m'] / 2.0:.0f} m half-extent; the area reaches "
        f"{need:.0f} m from the origin")


@pytest.mark.parametrize("path", OVERLAYS, ids=os.path.basename)
def test_frontier_threshold_is_about_twenty_metres_of_boundary(path):
    """`frontier_threshold_points` counts CELLS, and `map_cells` is fixed at
    480, so its physical meaning moves with `map_extent_m`. Left at the default
    20 on a 3 m/cell scene it discards every frontier shorter than 60 m, which
    is most of them."""
    p = _params(path)
    cell_m = p["map_extent_m"] / 480.0
    metres = p["frontier_threshold_points"] * cell_m
    assert 12.0 <= metres <= 30.0, f"{metres:.0f} m of frontier boundary"


@pytest.mark.parametrize("path", OVERLAYS, ids=os.path.basename)
def test_the_detector_gate_is_the_one_shared_value(path):
    """§6 of the benchmark skill: ONE gate, measured, never inherited. If an
    overlay ever differs, the arms are no longer being compared on their
    selection policy."""
    p = _params(path)
    assert p["sem_threshold"] == 0.65
    assert p["goal_name"] == "person"


@pytest.mark.parametrize("path", OVERLAYS, ids=os.path.basename)
def test_the_frontier_band_is_not_the_flight_band(path):
    """Sizing the voxel map to the FLIGHT band puts it entirely in empty air
    above the geometry: nothing is ever carved against a surface and voxel3d
    degenerates into the 2D slab it replaced."""
    p = _params(path)
    assert p["frontier_z_min_m"] < p["min_altitude_agl_m"]
    assert p["obstacle_min_z_m"] < p["obstacle_max_z_m"] <= p["frontier_z_max_m"]
    assert p["min_altitude_agl_m"] <= p["flight_altitude_m"] <= p["max_altitude_agl_m"]
    # Arriving must not release the frontier lock before the local planner
    # considers the goal reached.
    assert p["frontier_unlock_radius_m"] >= p["goal_tolerance_m"]


@pytest.mark.parametrize("path", OVERLAYS, ids=os.path.basename)
def test_every_sector_is_non_degenerate_and_they_tile_the_area(path):
    """Cut the overlay's own polygon with the planner's own partitioner, the
    way `search_planner.__init__` does. An empty sector is read as "degenerate"
    downstream, and this module's convention for degenerate is UNBOUNDED —
    which would silently remove a robot's confinement instead of confining it.
    """
    poly = _poly(_params(path))
    sectors = [sect.sector_for(poly, NUM_ROBOTS, i, mode="rect",
                               axis="principal", margin_m=0.0)
               for i in range(NUM_ROBOTS)]
    areas = [sect.polygon_area(s) for s in sectors]
    assert all(len(s) >= 3 for s in sectors)
    assert min(areas) > 10000.0, f"smallest sector is {min(areas):.0f} m2"
    # Rect bands are BOUNDING rectangles of each band's share, so they overlap
    # a little and their union exceeds the polygon; a 3x blow-up would mean the
    # axis or the mode is wrong.
    assert sum(areas) < 3.0 * fa.polygon_area(poly)


@pytest.mark.parametrize("path", OVERLAYS, ids=os.path.basename)
def test_the_overlay_does_not_set_a_method_parameter(path):
    """The scene layer loads LAST for the sectored arms, so anything it sets
    beats the method overlay. Geometry, budget, gate and altitudes are its
    business; `nav_mode`, the scorer and the frontier source are not — an arm
    silently flying another arm's selection policy is the one failure this
    whole layering exists to prevent.

    `vlfm_keyframe_period_s` is the deliberate exception and is asserted, not
    excluded: it is a FLEET-SIZE setting (one shared ITM scorer, eight robots),
    the overlays are generated per fleet, and every arm but vlfm ignores it.
    """
    p = _params(path)
    for key in ("nav_mode", "frontier_source", "sector_partition",
                "detector_mode", "vlm_preflight", "team_mode", "map_origin_xy"):
        assert key not in p, f"{key} belongs to the method overlay, not the scene"
    assert p["vlfm_keyframe_period_s"] >= 0.5, (
        "one BLIP-2 ITM scorer serves ~3 robots at 0.2 s; this sweep is 8")
