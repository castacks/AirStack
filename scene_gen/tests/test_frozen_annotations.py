"""`simulation/isaac-sim/utils/frozen_annotations.py` — offline, no Kit, no GPU.

    python3 -m pytest scene_gen/tests/test_frozen_annotations.py

Two kinds of test, and the split matters:

* SYNTHETIC cases pin the geometry contracts — the hull, the convex pad, the
  clip, the box schemas, the cell-path resolver. They run anywhere.
* CELL cases run against the real `final_disaster_dataset/` if it is on this
  machine and SKIP if it is not, because the frozen cells are hundreds of
  megabytes and live outside the repo on purpose. They are what actually
  guards the benchmark: that the search area contains every survivor, and that
  it is a fraction of the plate rather than all of it.

The pad is cross-checked against `search_baselines/sector.pad_polygon` — the
planner's own, numpy, miter-limited implementation. `pad_convex` exists only
because this module is stdlib-only; if the two ever disagree the planner's is
right and this one is the bug.
"""

import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "simulation", "isaac-sim", "utils"))
sys.path.insert(0, os.path.join(
    _REPO, "robot", "ros_ws", "src", "global", "planners", "search_baselines",
    "search_baselines"))

import frozen_annotations as fa            # noqa: E402

DATASET = os.environ.get("FINAL_DATASET_DIR") or os.path.expanduser(
    "~/SEI-COA/final_disaster_dataset")
CELLS = ["Fire/Suburban/level_1/1", "Fire/Suburban/level_2/1",
         "Fire/Suburban/level_3/1", "Tornado/Suburban/level_1/1",
         "Tornado/Suburban/level_2/1", "Tornado/Suburban/level_3/1"]


def _cell_usd(cell):
    return os.path.join(DATASET, cell, fa.usd_name_for_cell(
        os.path.join(DATASET, cell)))


def _have(cell):
    return os.path.isfile(_cell_usd(cell))


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------

def test_convex_hull_drops_interior_points():
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    hull = fa.convex_hull(square + [(5, 5), (2, 3), (9, 1)])
    assert len(hull) == 4
    assert set(map(tuple, hull)) == set(map(tuple, square))
    assert fa.polygon_area(hull) == pytest.approx(100.0)


def test_convex_hull_is_counter_clockwise():
    # `pad_convex`'s outward normal assumes it, and `sector.partition` reads
    # the same ring.
    hull = fa.convex_hull([(0, 0), (10, 0), (10, 10), (0, 10)])
    s = sum(hull[i][0] * hull[(i + 1) % 4][1] - hull[(i + 1) % 4][0] * hull[i][1]
            for i in range(4))
    assert s > 0


def test_pad_convex_grows_by_the_margin_on_every_side():
    padded = fa.pad_convex([(0, 0), (10, 0), (10, 10), (0, 10)], 5.0)
    xs = [p[0] for p in padded]
    ys = [p[1] for p in padded]
    assert min(xs) == pytest.approx(-5.0)
    assert max(xs) == pytest.approx(15.0)
    assert min(ys) == pytest.approx(-5.0)
    assert max(ys) == pytest.approx(15.0)


def test_pad_convex_matches_the_planners_own_padder():
    """The contract that makes the stdlib copy safe to ship."""
    sect = pytest.importorskip("sector")
    ring = fa.convex_hull([(0, 0), (300, 40), (420, 260), (150, 380),
                           (-40, 200)])
    mine = fa.pad_convex(ring, 50.0)
    theirs = [tuple(p) for p in sect.pad_polygon(ring, 50.0)]
    assert len(mine) == len(theirs)
    # Same vertex set, to a millimetre, in the same order up to rotation.
    for a in mine:
        assert min(math.dist(a, b) for b in theirs) < 1e-3
    assert fa.polygon_area(mine) == pytest.approx(
        sect.polygon_area(theirs), rel=1e-9)


def test_clip_to_rect_trims_a_polygon_that_leaves_the_plate():
    poly = [(-600, -600), (600, -600), (600, 600), (-600, 600)]
    clipped = fa.clip_to_rect(poly, (-500, -500, 500, 500))
    assert fa.polygon_area(clipped) == pytest.approx(1e6)
    for x, y in clipped:
        assert -500.0 - 1e-6 <= x <= 500.0 + 1e-6
        assert -500.0 - 1e-6 <= y <= 500.0 + 1e-6


def test_point_in_polygon():
    sq = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert fa.point_in_polygon(5, 5, sq)
    assert not fa.point_in_polygon(15, 5, sq)
    assert not fa.point_in_polygon(5, -0.1, sq)


# ---------------------------------------------------------------------------
# the annotation payloads
# ---------------------------------------------------------------------------

def _hints(*records):
    return {"meta": {"region_m": [-500, -500, 500, 500], "disaster": "wildfire"},
            "hints": list(records)}


def _hint(cls, cx, cy, w=10.0, h=10.0, z=5.0):
    return {"class": cls,
            "bbox_min": [cx - w / 2, cy - h / 2, 0.0],
            "bbox_max": [cx + w / 2, cy + h / 2, z],
            "centre": [cx, cy, z / 2]}


def test_people_boxes_lift_the_centre_half_a_body():
    boxes = fa.people_boxes({"people": [{"x": 1.0, "y": 2.0, "z": 0.5}]})
    assert boxes[0]["class"] == "person"
    assert boxes[0]["bbox_world"]["center_xyz_m"] == [1.0, 2.0, 1.4]
    assert boxes[0]["bbox_world"]["size_xyz_m"] == [0.7, 0.7, 1.8]


def test_obstacle_boxes_map_to_the_clearance_module_vocabulary():
    clr = pytest.importorskip("clearance")
    doc = _hints(_hint("Building", 0, 0), _hint("Damaged building", 20, 0),
                 _hint("Tree", 40, 0), _hint("Burnt Tree", 60, 0),
                 _hint("Fallen Tree", 80, 0), _hint("Car", 100, 0),
                 _hint("Van", 120, 0), _hint("Truck", 140, 0),
                 _hint("Toppled", 160, 0), _hint("Debris", 180, 0),
                 _hint("Pool", 200, 0), _hint("Parking Lot", 220, 0))
    boxes = fa.obstacle_boxes(doc)
    got = {b["class"] for b in boxes}
    assert got <= set(clr.OBSTACLE_CLASSES), got - set(clr.OBSTACLE_CLASSES)
    assert got == {"house", "tree", "car", "truck", "prop"}
    # Pool and Parking Lot are flat ground and must not become obstacles.
    assert len(boxes) == 10


def test_obstacle_boxes_survive_a_round_trip_through_load_boxes(tmp_path):
    clr = pytest.importorskip("clearance")
    doc = _hints(_hint("Building", 0, 0, 12.0, 8.0, 7.0))
    path = tmp_path / "s_obstacles.json"
    fa.write(str(path), fa.obstacle_boxes(doc))
    boxes = clr.load_boxes(str(path))
    assert len(boxes) == 1
    cx, cy, hx, hy, top, bottom, cls = boxes[0]
    assert (cx, cy, hx, hy, cls) == (0.0, 0.0, 6.0, 4.0, "house")
    assert top == pytest.approx(7.0)
    assert bottom == pytest.approx(0.0)


def test_affected_polygon_bounds_the_damage_not_the_intact_scene():
    doc = _hints(_hint("Damaged building", -100, -100),
                 _hint("Burnt Tree", -100, 100),
                 _hint("Fallen Tree", 100, 100),
                 _hint("Toppled", 100, -100),
                 # A pristine building far away must NOT enlarge the area.
                 _hint("Building", 480, 480))
    poly = fa.affected_polygon(doc)
    assert fa.polygon_area(poly) == pytest.approx(210.0 * 210.0, rel=1e-6)
    assert not fa.point_in_polygon(480, 480, poly)


def test_search_polygon_is_the_damage_plus_the_pad_and_stays_on_the_plate():
    doc = _hints(_hint("Damaged building", -480, -480),
                 _hint("Burnt Tree", -480, 480),
                 _hint("Fallen Tree", 480, 480),
                 _hint("Toppled", 480, -480))
    poly = fa.search_polygon(doc, pad_m=50.0)
    for x, y in poly:
        assert -500.0 - 1e-6 <= x <= 500.0 + 1e-6
        assert -500.0 - 1e-6 <= y <= 500.0 + 1e-6
    # The damage box is 970 m across; +50 m each side is clipped to the plate.
    assert fa.polygon_area(poly) == pytest.approx(1e6, rel=1e-3)


def test_region_entries_use_search_and_damage_never_burn_or_affected():
    """A frozen cell's polygon is NOT the fire model's, and the key says so."""
    doc = _hints(_hint("Damaged building", 0, 0, 100, 100))
    entries = fa.region_entries(doc, {"people": [{"x": 0, "y": 0, "z": 0}]})
    classes = [e["class"] for e in entries]
    assert classes == ["search", "damage", "region", "meta"]
    assert "burn" not in classes and "affected" not in classes
    meta = entries[-1]
    assert meta["people_total"] == 1
    assert meta["people_inside_search"] == 1


def test_region_entries_are_readable_by_the_planners_loader(tmp_path):
    """The shape `planner_node._load_scene_region` expects: a LIST of dicts
    with `class` and `polygon_xy`, from which it takes the named entry."""
    doc = _hints(_hint("Damaged building", 0, 0, 100, 100))
    path = tmp_path / "S_region.json"
    fa.write(str(path), fa.region_entries(doc))
    import json
    entries = json.load(open(path))
    entry = next(e for e in entries if e.get("class") == "search")
    poly = [[float(p[0]), float(p[1])] for p in entry["polygon_xy"]]
    assert len(poly) >= 3


# ---------------------------------------------------------------------------
# resolving a cell path
# ---------------------------------------------------------------------------

def test_usd_name_for_cell_follows_the_contract():
    assert fa.usd_name_for_cell("/d/Fire/Suburban/level_1/1") == \
        "fire_suburban_lvl1_1.usd"
    assert fa.usd_name_for_cell("/d/Tornado/Urban/level_3/5") == \
        "tornado_urban_lvl3_5.usd"
    assert fa.usd_name_for_cell("/d/not/a/cell") is None


def test_resolve_cell_accepts_all_three_spellings(tmp_path):
    cell = tmp_path / "Fire" / "Suburban" / "level_1" / "1"
    cell.mkdir(parents=True)
    usd = cell / "fire_suburban_lvl1_1.usd"
    usd.write_text("#usda 1.0\n")
    root = str(tmp_path)
    assert fa.resolve_cell(str(usd), root) == str(usd)
    assert fa.resolve_cell("Fire/Suburban/level_1/1/fire_suburban_lvl1_1.usd",
                           root) == str(usd)
    assert fa.resolve_cell("Fire/Suburban/level_1/1", root) == str(usd)


def test_resolve_cell_raises_on_a_missing_cell(tmp_path):
    with pytest.raises(ValueError) as exc:
        fa.resolve_cell("Fire/Suburban/level_9/1", str(tmp_path))
    assert "no such file or cell" in str(exc.value)


def test_resolve_cell_refuses_an_ambiguous_directory(tmp_path):
    d = tmp_path / "loose"
    d.mkdir()
    (d / "a.usd").write_text("")
    (d / "b.usd").write_text("")
    with pytest.raises(ValueError) as exc:
        fa.resolve_cell("loose", str(tmp_path))
    assert "name one" in str(exc.value)


# ---------------------------------------------------------------------------
# the real cells — the tests that actually guard the benchmark
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("cell", CELLS)
def test_every_survivor_is_inside_the_search_area(cell):
    """A search area that does not contain the answer key is a benchmark that
    cannot be won. Measured on all six cells: 49/49, 79/79, 84/84, 30/30,
    40/40, 70/70."""
    if not _have(cell):
        pytest.skip(f"{cell} is not on this machine")
    people_doc, hints_doc = fa.load_cell(_cell_usd(cell))
    poly = fa.search_polygon(hints_doc)
    people = fa.people_records(people_doc)
    assert people, "the cell has no survivors at all"
    outside = [(r["x"], r["y"]) for r in people
               if not fa.point_in_polygon(float(r["x"]), float(r["y"]), poly)]
    assert not outside, f"{len(outside)}/{len(people)} survivors outside: {outside[:5]}"


@pytest.mark.parametrize("cell", CELLS)
def test_the_search_area_is_a_fraction_of_the_plate(cell):
    """The point of `search_area_source: scene`. If a cell ever comes out at
    ~100% the search has stopped being "the damage plus a pad" and the arms are
    sweeping the whole plate — which is a different (and much easier to look
    good on) benchmark."""
    if not _have(cell):
        pytest.skip(f"{cell} is not on this machine")
    _people, hints_doc = fa.load_cell(_cell_usd(cell))
    rect = fa.region_m(hints_doc)
    plate = (rect[2] - rect[0]) * (rect[3] - rect[1])
    hull = fa.affected_polygon(hints_doc, rect)
    search = fa.search_polygon(hints_doc, rect)
    assert 0.05 < fa.polygon_area(hull) / plate < 0.90
    assert fa.polygon_area(search) >= fa.polygon_area(hull)
    assert fa.polygon_area(search) / plate <= 0.90


@pytest.mark.parametrize("cell", CELLS)
def test_a_cell_yields_all_three_annotation_files(cell, tmp_path):
    if not _have(cell):
        pytest.skip(f"{cell} is not on this machine")
    people_doc, hints_doc = fa.load_cell(_cell_usd(cell))
    written = fa.write_all("S", [str(tmp_path)], people_doc, hints_doc,
                           scene_usd=_cell_usd(cell), quiet=True)
    assert all(written[k] for k in ("", "_obstacles", "_region"))
    import json
    people = json.load(open(tmp_path / "S.json"))
    obst = json.load(open(tmp_path / "S_obstacles.json"))
    assert all(b["class"] == "person" for b in people)
    assert len(obst) > 1000, "a 1 km suburb has thousands of houses and trees"
