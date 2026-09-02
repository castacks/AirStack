#!/usr/bin/env python3
"""test_city_layout_audit.py — the offline gate for "buildings on the road /
empty blocks" (the 2026-09-01 urban-fire baseline incident; see
`tools/city_layout_audit.py`'s docstring for the measured numbers).

    python3 -m pytest -q scene_gen/tests/test_city_layout_audit.py

HOST-SIDE, NO KIT.  The fire city launcher builds a `SimulationApp` at
module scope, so — as `test_urban_fire_city_launch.py` does — its functions
are extracted from the AST and executed in an empty namespace, never
imported.

WHAT IT GUARDS

  1. `record_xy` in the launcher and in the audit tool are the SAME rule on
     every record shape (x_orig wins, else x/y).
  2. `load_fire` shifts cropped-manifest records back to the full-city
     frame WITHOUT `FC_CROP_WINDOW` (the incident's exact trigger), and
     `_unshift_records_to_full_city(records, 0, 0)` copies `x_orig` over.
  3. `compose_bakes` reads its holder position through `record_xy`, never
     `rec.get("x")`, and refuses a bake whose holder would sit more than
     `CELL_MATCH_TOL_M` from its cell BEFORE `hide_intact` runs.
  4. The audit geometry (footprint rects, block fractions, road overlap,
     empty-block detection) on synthetic layouts.
  5. THE SHIPPED FIXTURES, when present (`_plans/baseline_l{1,2,3}_
     {dump,manifest}.json`, not git-tracked): the shipped rule displaces
     every record by exactly the level's window centre and puts 23/41/86
     of them across a block edge; `record_xy` displaces none and puts 0
     across.  Plus the tornado bench manifest, whose records are solved
     over the whole plate and must already match their cells.
"""
import ast
import json
import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
_REPO = os.path.normpath(os.path.join(_SCENE_GEN, ".."))
_TOOLS = os.path.join(_SCENE_GEN, "tools")
_PLANS = os.path.join(_SCENE_GEN, "_plans")
for _p in (_SCENE_GEN, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import city_layout_audit as cla                                   # noqa: E402

LAUNCHER = os.path.join(_REPO, "simulation", "isaac-sim", "launch_scripts",
                        "urban_fire_city_launch_script.py")
GT_ROOT = os.path.join(os.path.dirname(_REPO), "final_disaster_dataset",
                       "Fire", "Urban")

# the committed (level -> window centre) table, `tools/baseline_layouts.LEVELS`
WINDOW_CENTRE = {1: (-180.0, 180.0), 2: (100.0, 150.0), 3: (20.0, 230.0)}


# ---------------------------------------------------------------------------
# launcher source access (never imported — it starts Kit)
# ---------------------------------------------------------------------------
def _launcher_tree():
    with open(LAUNCHER) as fh:
        return ast.parse(fh.read(), filename=LAUNCHER)


def _func_node(tree, name, cls=None):
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and cls and node.name == cls:
            for sub in node.body:
                if isinstance(sub, ast.FunctionDef) and sub.name == name:
                    return sub
        if cls is None and isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError("no function {0!r} (class {1!r}) in the launcher"
                         .format(name, cls))


def _exec_func(tree, name):
    """Execute one module-level function's source in a bare namespace."""
    node = _func_node(tree, name)
    mod = ast.Module(body=[node], type_ignores=[])
    ns = {"math": math}
    exec(compile(mod, LAUNCHER, "exec"), ns)
    return ns[name]


def _calls_named(node, name):
    out = []
    for n in ast.walk(node):
        if isinstance(n, ast.Call):
            f = n.func
            if (isinstance(f, ast.Name) and f.id == name) or \
               (isinstance(f, ast.Attribute) and f.attr == name):
                out.append(n)
    return out


def _method_calls_on(node, receiver, attr):
    """`receiver.attr(...)` calls, e.g. rec.get(...)."""
    out = []
    for n in ast.walk(node):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
            f = n.func
            if f.attr == attr and isinstance(f.value, ast.Name) and f.value.id == receiver:
                out.append(n)
    return out


# ---------------------------------------------------------------------------
# 1. record_xy — one rule in two places
# ---------------------------------------------------------------------------
RECORD_SHAPES = [
    {"x": 1.0, "y": 2.0},
    {"x": 1.0, "y": 2.0, "x_orig": 181.0, "y_orig": -178.0},
    {"x": -471.9962, "y": 177.467, "x_orig": -451.9961594, "y_orig": 407.46696},
    {"x_orig": 5.0, "y_orig": 6.0},
    {},
]


def test_record_xy_launcher_matches_tool():
    fn = _exec_func(_launcher_tree(), "record_xy")
    for rec in RECORD_SHAPES:
        assert fn(dict(rec)) == cla.record_xy(dict(rec)), rec


def test_record_xy_prefers_orig():
    x, y = cla.record_xy({"x": 1.0, "y": 2.0, "x_orig": 181.0, "y_orig": -178.0})
    assert (x, y) == (181.0, -178.0)
    assert cla.record_xy({"x": 1.0, "y": 2.0}) == (1.0, 2.0)
    assert cla.record_xy({}) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# 2. load_fire un-shifts without a crop window
# ---------------------------------------------------------------------------
def test_unshift_copies_orig_with_zero_shift_and_is_idempotent():
    fn = _exec_func(_launcher_tree(), "_unshift_records_to_full_city")
    recs = [{"x": -482.16, "y": -263.36, "x_orig": -662.16, "y_orig": -83.36},
            {"x": 10.0, "y": 20.0}]
    n = fn(recs, 0.0, 0.0)
    assert n == 2
    assert (recs[0]["x"], recs[0]["y"]) == (-662.16, -83.36)
    assert (recs[1]["x"], recs[1]["y"]) == (10.0, 20.0)   # nothing known, no-op
    fn(recs, 0.0, 0.0)
    assert (recs[0]["x"], recs[0]["y"]) == (-662.16, -83.36)
    # with a real centre the stamped record STILL takes x_orig, the bare one adds
    recs2 = [dict(recs[0], x=-482.16, y=-263.36), {"x": 10.0, "y": 20.0}]
    fn(recs2, -180.0, 180.0)
    assert (recs2[0]["x"], recs2[0]["y"]) == (-662.16, -83.36)
    assert (recs2[1]["x"], recs2[1]["y"]) == (-170.0, 200.0)


def test_load_fire_unshifts_on_stamp_not_only_on_crop_window():
    tree = _launcher_tree()
    lf = _func_node(tree, "load_fire", cls=None)
    calls = _calls_named(lf, "_unshift_records_to_full_city")
    assert len(calls) >= 2, ("load_fire must un-shift stamped records on a "
                             "path that does not depend on CROP_CENTRE")
    # one of them is the no-crop path: literal (0.0, 0.0)
    zero = [c for c in calls
            if len(c.args) == 3 and all(isinstance(a, ast.Constant) and
                                        float(a.value) == 0.0 for a in c.args[1:])]
    assert zero, "the no-crop-window path must call the un-shift with (0, 0)"


# ---------------------------------------------------------------------------
# 3. compose_bakes: record_xy + the frame guard before hide_intact
# ---------------------------------------------------------------------------
def test_compose_bakes_reads_record_xy_and_guards_before_hiding():
    tree = _launcher_tree()
    cb = _func_node(tree, "compose_bakes", cls=None)
    assert _calls_named(cb, "record_xy"), "compose_bakes must use record_xy"
    for c in _method_calls_on(cb, "rec", "get"):
        key = c.args[0].value if c.args and isinstance(c.args[0], ast.Constant) else None
        assert key not in ("x", "y"), "compose_bakes must not read rec.get('x'/'y')"
    dist = _calls_named(cb, "_cell_distance")
    hide = _calls_named(cb, "hide_intact")
    assert dist and hide
    assert dist[0].lineno < hide[0].lineno, "guard must run before hide_intact"
    names = {n.id for n in ast.walk(cb) if isinstance(n, ast.Name)}
    assert "CELL_MATCH_TOL_M" in names
    assert "_frac_inside_blocks" in names, "the post-compose block audit"


def test_cell_distance_and_frac_inside_blocks_helpers():
    tree = _launcher_tree()
    cd = _exec_func(tree, "_cell_distance")
    assert cd(1.0, 1.0, None) == 0.0
    assert cd(1.0, 1.0, {"x_m": 1.0, "y_m": 1.0}) == 0.0
    assert abs(cd(181.0, -178.0, {"x_m": 1.0, "y_m": 2.0}) - math.hypot(180, 180)) < 1e-9
    fib = _exec_func(tree, "_frac_inside_blocks")
    blocks = [(0.0, 0.0, 100.0, 50.0), (120.0, 0.0, 220.0, 50.0)]
    assert fib((10.0, 10.0, 30.0, 20.0), blocks) == 1.0
    assert abs(fib((90.0, 10.0, 130.0, 20.0), blocks) - 0.25) < 1e-9
    assert fib((90.0, 10.0, 90.0, 20.0), blocks) == 0.0
    # the tool's own version agrees
    for r in ((10.0, 10.0, 30.0, 20.0), (90.0, 10.0, 130.0, 20.0)):
        assert abs(fib(r, blocks) - cla.rect_frac_inside(r, blocks)[0]) < 1e-12


# ---------------------------------------------------------------------------
# 4. audit geometry on a synthetic city
# ---------------------------------------------------------------------------
def _synthetic():
    # two blocks side by side with a 20 m road between, a border road ring
    blocks = [(-100.0, -50.0, -10.0, 50.0, "lowrise"),
              (10.0, -50.0, 100.0, 50.0, "midrise"),
              (-100.0, 70.0, 100.0, 120.0, "park"),
              (-100.0, 52.0, 100.0, 60.0, "midrise")]      # 8 m sliver
    corridors = [{"x0": -10.0, "y0": -70.0, "x1": 10.0, "y1": 140.0,
                  "n_lanes": 4, "dir": "ns"},
                 {"x0": -120.0, "y0": -70.0, "x1": 120.0, "y1": -50.0,
                  "n_lanes": 2, "dir": "ew"}]
    return blocks, corridors


def _naive_overlap(recs, tol=0.05):
    """The BUGGY audit method this module's new tests pin as a negative
    control: an axis-aligned box straight off W/D, never rotated by
    `yaw_deg`. This is what produced the "29-43 overlapping pairs per
    level" report that motivated `building_overlap_pairs` -- reproduced
    here so the regression is provable, not just asserted."""
    pairs = []
    for a in range(len(recs)):
        ra = recs[a]
        ax0, ay0 = ra["x_m"] - ra["W"] / 2.0, ra["y_m"] - ra["D"] / 2.0
        ax1, ay1 = ra["x_m"] + ra["W"] / 2.0, ra["y_m"] + ra["D"] / 2.0
        for b in range(a + 1, len(recs)):
            rb = recs[b]
            bx0, by0 = rb["x_m"] - rb["W"] / 2.0, rb["y_m"] - rb["D"] / 2.0
            bx1, by1 = rb["x_m"] + rb["W"] / 2.0, rb["y_m"] + rb["D"] / 2.0
            ox = min(ax1, bx1) - max(ax0, bx0)
            oy = min(ay1, by1) - max(ay0, by0)
            if ox > tol and oy > tol:
                pairs.append((a, b))
    return pairs


# ---------------------------------------------------------------------------
# 3b. building_overlap_pairs -- SAT over REAL, yaw-rotated footprints
#
# THE 2026-09-01 "SYSTEMIC LAYOUT DEFECT" REPORT. A prior audit reported
# ~29-43 "overlapping" building pairs and 6-11 "empty blocks" per baseline
# level, root-caused (unverified) as the packer sizing lots from nominal
# footprints instead of each asset's real measured W/D. Reproduced here:
# `detail.districts._pool_entries`/`_fp_of` both call `resolver.get()` (the
# packer already uses real, resolver-measured footprints, never a nominal
# constant); `districts.repair_overlaps` already runs on that real geometry
# for all three baseline seeds and finds NOTHING to repair (`checked=0`,
# host-rebuilt, verified interactively); and the shipped dumps' own
# Kit-measured W/D show ZERO true (yaw-rotated) overlaps, ZERO buildings
# spilling off their block, and ZERO non-boundary empty blocks -- see the
# parametrized tests below. What DOES reproduce the reported numbers is a
# NAIVE axis-aligned overlap check that skips the yaw rotation
# `_pool_entries`/`_rotated_wh` apply everywhere else in the packer: a
# building placed at yaw 90/270 has its W/D swapped in the world, and a
# check that does not swap them flags every legitimately touching
# 90-degree-turned neighbour as "overlapping". `_naive_overlap` above
# reproduces this exact mechanism.
# ---------------------------------------------------------------------------
def test_building_overlap_pairs_party_wall_touch_is_not_an_overlap():
    """The exact pair the "systemic defect" report cited by name:
    `SM_Building_03` (yaw 180) and `SM_Building_06_Small` (yaw 90) in
    `baseline_l1_dump.json`, at (-335.09, 126.89) and (-313.27, 126.94).
    Rotated correctly, they share a party wall to within 2 cm -- not an
    overlap. A naive (non-rotated) box reads it as a ~7 m overlap."""
    a = {"usd": "SM_Building_03.usd", "x_m": -335.0853, "y_m": 126.8926,
        "yaw_deg": 180.0, "W": 28.9268473815918, "D": 28.952466888427736}
    b = {"usd": "SM_Building_06_Small.usd", "x_m": -313.2719, "y_m": 126.9406,
        "yaw_deg": 90.0, "W": 28.8566162109375, "D": 14.700000000000001}
    assert cla.building_overlap_pairs([a, b]) == []
    assert _naive_overlap([a, b]) == [(0, 1)], (
        "the naive (non-yaw) check should reproduce the false positive")


def test_building_overlap_pairs_detects_a_genuine_overlap():
    a = {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0, "W": 20.0, "D": 20.0}
    b = {"x_m": 10.0, "y_m": 0.0, "yaw_deg": 0.0, "W": 20.0, "D": 20.0}
    # centres 10 m apart, half-widths 10 + 10 -- a genuine 10 m interpenetration
    assert cla.building_overlap_pairs([a, b]) == [(0, 1)]
    # nudge apart past the tolerance: no overlap
    c = dict(b, x_m=20.2)
    assert cla.building_overlap_pairs([a, c]) == []


def test_building_overlap_pairs_rotates_by_yaw_not_just_axis_swap():
    # two 40 x 10 boxes, one yawed 90, sharing an edge exactly -- a
    # continuous rotation (not a literal W/D swap) must land the same place
    # a swap would for an exact 90-degree yaw.
    a = {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0, "W": 40.0, "D": 10.0}
    b = {"x_m": 25.0, "y_m": 0.0, "yaw_deg": 90.0, "W": 40.0, "D": 10.0}
    # a's right edge at x=20; b yawed 90 is 10 wide in x, half-width 5,
    # left edge at 20 -- touching, not overlapping
    assert cla.building_overlap_pairs([a, b]) == []
    assert cla.building_overlap_pairs([a, dict(b, x_m=24.0)]) == [(0, 1)]


@pytest.mark.parametrize("level", [1, 2, 3])
def test_shipped_baseline_dumps_have_zero_real_footprint_overlaps(level):
    d = os.path.join(_PLANS, "baseline_l{0}_dump.json".format(level))
    if not os.path.exists(d):
        pytest.skip("baseline level {0} dump not present".format(level))
    with open(d) as fh:
        dump = json.load(fh)
    recs = dump["placements"]
    naive = _naive_overlap(recs)
    real = cla.building_overlap_pairs(recs)
    # the gate: zero TRUE overlaps once yaw is respected
    assert real == [], (level, real[:5])
    # document the false-positive mechanism is live on this exact fixture
    # (guards against the fixture changing under the test without notice --
    # if this ever goes to 0 the naive-vs-real contrast above has nothing
    # left to demonstrate and should be revisited, not silently dropped)
    assert len(naive) > 0, (
        "expected the naive (non-yaw) check to still find false positives "
        "on level {0} -- if the fixture changed, update this test's intent"
        .format(level))


@pytest.mark.parametrize("level", [1, 2, 3])
def test_shipped_baseline_dumps_have_no_interior_empty_or_overhanging_blocks(level):
    """The dump's OWN buildings against its OWN typology blocks, in the
    dump's OWN (crop-recentred) frame, with the standard 70 m boundary
    margin so a crop-edge sliver (legitimate, see `tools/crop_window.py`)
    is not counted as an "empty block" or an "overhanging" building."""
    d = os.path.join(_PLANS, "baseline_l{0}_dump.json".format(level))
    if not os.path.exists(d):
        pytest.skip("baseline level {0} dump not present".format(level))
    with open(d) as fh:
        dump = json.load(fh)
    blocks = [(b["rect"][0], b["rect"][1], b["rect"][2], b["rect"][3],
              b.get("name")) for b in dump["typology"]["blocks"]]
    fps = [{"name": r.get("usd", ""),
           "rect": cla.footprint_rect(r["x_m"], r["y_m"], r["W"], r["D"],
                                      r.get("yaw_deg", 0.0)),
           "centre": (r["x_m"], r["y_m"])} for r in dump["placements"]]
    window = tuple(dump["crop"]["window"])
    sx, sy = dump["crop"]["shift"]
    window = (window[0] + sx, window[1] + sy, window[2] + sx, window[3] + sy)
    rep = cla.audit_footprints(fps, blocks, window=window, margin_m=70.0)
    assert rep["offenders"] == [], (level, rep["offenders"][:5])
    assert rep["empty_blocks"] == [], (level, rep["empty_blocks"][:5])


def test_footprint_rect_yaw():
    r0 = cla.footprint_rect(0.0, 0.0, 40.0, 20.0, 0.0)
    r90 = cla.footprint_rect(0.0, 0.0, 40.0, 20.0, 90.0)
    r270 = cla.footprint_rect(0.0, 0.0, 40.0, 20.0, 270.0)
    r180 = cla.footprint_rect(0.0, 0.0, 40.0, 20.0, 180.0)
    assert r0 == (-20.0, -10.0, 20.0, 10.0) == r180
    assert r90 == (-10.0, -20.0, 10.0, 20.0) == r270
    r45 = cla.footprint_rect(0.0, 0.0, 40.0, 20.0, 45.0)
    assert r45[2] - r45[0] > 40.0 and r45[3] - r45[1] > 20.0


def test_audit_footprints_offenders_and_empty_blocks():
    blocks, corridors = _synthetic()
    inside = {"name": "a", "prim_path": "/a", "rect": (-90.0, -40.0, -50.0, -20.0),
              "centre": (-70.0, -30.0), "cls": "Building", "style": "x",
              "substitute": False}
    on_road = {"name": "b", "prim_path": "/b", "rect": (-30.0, 0.0, 10.0, 20.0),
               "centre": (-10.0, 10.0), "cls": "Damaged building", "style": "y",
               "substitute": True}
    rep = cla.audit_footprints([inside, on_road], blocks)
    assert rep["n"] == 2
    assert [o["name"] for o in rep["offenders"]] == ["b"]
    assert abs(rep["offenders"][0]["frac_in"] - 0.5) < 1e-9
    # block 1 (midrise, right) is empty; the park and the 8 m sliver are not reported
    assert [e["block"] for e in rep["empty_blocks"]] == [1]
    assert abs(cla.road_overlap(on_road["rect"], corridors) - 0.5) < 1e-9
    assert cla.road_overlap(inside["rect"], corridors) == 0.0


def test_manifest_frame_check_synthetic():
    dump = {"placements": [
        {"cell": "/World/stage/generated/house_1_1", "x_m": -482.16, "y_m": -263.36,
         "x_m_orig": -662.16, "y_m_orig": -83.36}]}
    rec = {"cell": "/World/stage/generated/house_1_1", "x": -482.16, "y": -263.36,
           "x_orig": -662.16, "y_orig": -83.36, "W": 30.0, "D": 20.0}
    fc = cla.manifest_frame_check([rec], dump)
    assert fc["n"] == 1
    assert abs(fc["shipped"]["max_d"] - math.hypot(180.0, 180.0)) < 1e-6
    assert fc["shipped"]["n_bad"] == 1
    assert fc["fixed"]["max_d"] < 1e-9 and fc["fixed"]["n_bad"] == 0


def test_unshifted_blocks_and_compare_blocks():
    dump = {"crop": {"shift": [180.0, -180.0]},
            "typology": {"blocks": [{"rect": [-500.0, -500.0, -268.7, -429.8],
                                     "name": "lowrise"}]}}
    ub = cla.unshifted_blocks(dump)
    assert ub[0][:4] == (-680.0, -320.0, -448.7, -249.8)
    host = [(-680.0, -320.0, -448.7, -249.8), (0.0, 0.0, 50.0, 50.0)]
    assert cla.compare_blocks(host, ub)[:2] == (1, 1)
    clipped = [(-680.0, -320.0, -448.7, -260.0, "lowrise")]     # window cut the top
    assert cla.compare_blocks(host, clipped)[:2] == (1, 1)
    assert cla.compare_blocks([(0.0, 0.0, 1.0, 1.0)], ub)[:2] == (0, 1)


def test_cli_end_to_end(tmp_path):
    blocks, corridors = _synthetic()
    dump = {"schema": "fire_city_placements_dump.v1", "region_m": [240.0, 240.0],
            "crop": {"window": [-120.0, -120.0, 120.0, 120.0], "shift": [0.0, 0.0]},
            "typology": {"blocks": [{"rect": list(b[:4]), "name": b[4]} for b in blocks]},
            "placements": [{"cell": "/World/stage/generated/house_1_1",
                            "x_m": -70.0, "y_m": -30.0, "usd": "a.usd",
                            "W": 40.0, "D": 20.0, "yaw_deg": 0.0}]}
    man = {"records": [{"cell": "/World/stage/generated/house_1_1", "x": 110.0,
                        "y": 150.0, "x_orig": -70.0, "y_orig": -30.0,
                        "W": 40.0, "D": 20.0, "yaw_deg": 0.0, "style": "s"}]}
    dp, mp, jp = tmp_path / "dump.json", tmp_path / "man.json", tmp_path / "r.json"
    dp.write_text(json.dumps(dump))
    mp.write_text(json.dumps(man))
    rep = cla.main(["--dump", str(dp), "--manifest", str(mp), "--json", str(jp)])
    assert rep["frame"]["shipped"]["n_bad"] == 1
    assert rep["frame"]["fixed"]["n_bad"] == 0
    assert rep["record_offenders_shipped"] == 1
    assert rep["record_offenders_fixed"] == 0
    assert json.loads(jp.read_text())["record_offenders_fixed"] == 0


# ---------------------------------------------------------------------------
# 5. the shipped fixtures (skip when the files are not on this machine)
# ---------------------------------------------------------------------------
def _fixture(level):
    d = os.path.join(_PLANS, "baseline_l{0}_dump.json".format(level))
    m = os.path.join(_PLANS, "baseline_l{0}_manifest.json".format(level))
    if not (os.path.exists(d) and os.path.exists(m)):
        pytest.skip("baseline level {0} dump/manifest not present".format(level))
    with open(d) as fh:
        dump = json.load(fh)
    with open(m) as fh:
        man = json.load(fh)
    return dump, man


@pytest.mark.parametrize("level,expect_shipped_over",
                         [(1, 23), (2, 41), (3, 86)])
def test_shipped_baselines_frame_and_blocks(level, expect_shipped_over):
    dump, man = _fixture(level)
    recs = man["records"]
    fc = cla.manifest_frame_check(recs, dump)
    assert fc["n"] == len(recs)
    cx, cy = WINDOW_CENTRE[level]
    # the shipped rule displaces EVERY record by exactly the window centre
    assert fc["shipped"]["n_bad"] == fc["n"]
    assert abs(fc["shipped"]["max_d"] - math.hypot(cx, cy)) < 0.01
    # the fixed rule displaces none
    assert fc["fixed"]["n_bad"] == 0 and fc["fixed"]["max_d"] < 0.01
    blocks = cla.unshifted_blocks(dump)
    shipped = cla.audit_footprints(cla.manifest_footprints(
        recs, lambda r: (float(r["x"]), float(r["y"]))), blocks)
    fixed = cla.audit_footprints(cla.manifest_footprints(recs), blocks)
    assert len(shipped["offenders"]) == expect_shipped_over
    assert fixed["offenders"] == []


@pytest.mark.parametrize("level", [1, 2, 3])
def test_shipped_gt_shows_the_displacement(level):
    gt_path = os.path.join(GT_ROOT, "level_{0}".format(level), "1", "GT_hints.json")
    if not os.path.exists(gt_path):
        pytest.skip("shipped GT_hints.json for level {0} not present".format(level))
    dump, man = _fixture(level)
    with open(gt_path) as fh:
        gt = json.load(fh)
    fps = cla.hints_footprints(gt)
    subs = [f for f in fps if f["substitute"]]
    intact = [f for f in fps if not f["substitute"]]
    blocks = cla.unshifted_blocks(dump)
    win = dump["crop"]["window"]
    a_sub = cla.audit_footprints(subs, blocks, window=win, margin_m=70.0)
    a_int = cla.audit_footprints(intact, blocks, window=win, margin_m=70.0)
    # the intact city itself is clean; the shipped substitutes are not
    assert a_int["offenders"] == []
    assert a_sub["n"] > 0 and len(a_sub["offenders"]) > 0
    # and every substitute stands exactly one window-centre from a record
    # (6 m: a partially collapsed F5c shell's bbox centre sits up to ~4.6 m
    # off its holder in the shipped level 3 — measured, not a guess)
    cx, cy = WINDOW_CENTRE[level]
    recs = man["records"]
    n_hit = 0
    for f in subs:
        for r in recs:
            ox, oy = cla.record_xy(r)
            if math.hypot(f["centre"][0] - (ox - cx), f["centre"][1] - (oy - cy)) < 6.0:
                n_hit += 1
                break
    assert n_hit == len(subs), (n_hit, len(subs))


@pytest.mark.parametrize("seed", [4, 2, 3])
def test_tornado_manifests_are_in_their_dumps_frame(seed):
    """A tornado manifest is solved over the WHOLE plate (no re-centred
    crop), so its records must already sit on their dump cells.  A manifest
    solved on a HOST build (`dump.path` = `<host-build:...>`) has no Kit
    dump to compare against — skipped, that is a different trap (the
    `generate-urban-city` skill's "size source, never the layout")."""
    m = os.path.join(_PLANS, "tornado_city_{0}.json".format(seed))
    if not os.path.exists(m):
        pytest.skip("tornado manifest seed {0} not present".format(seed))
    with open(m) as fh:
        man = json.load(fh)
    dpath = (man.get("dump") or {}).get("path") or ""
    if not dpath or dpath.startswith("<"):
        pytest.skip("tornado manifest seed {0} was solved on a host build".format(seed))
    d = os.path.join(_PLANS, os.path.basename(dpath))
    if not os.path.exists(d):
        pytest.skip("dump {0} not present".format(d))
    with open(d) as fh:
        dump = json.load(fh)
    fc = cla.manifest_frame_check(man["records"], dump)
    assert fc["n"] > 0
    assert fc["shipped"]["n_bad"] == 0 and fc["fixed"]["n_bad"] == 0, fc
