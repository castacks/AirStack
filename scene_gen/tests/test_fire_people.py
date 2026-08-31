#!/usr/bin/env python3
"""test_fire_people.py — every rule `disaster/fire_people.py` claims.

    python3 scene_gen/tests/test_fire_people.py
    pytest -q scene_gen/tests/test_fire_people.py

HOST-SIDE, no `pxr`, no Kit, well under a second. The planner touches no
stage, which is the whole reason it was written as a pure planner: a people
pass fails SILENTLY (the placement succeeds, the figure is counted, the
ground truth is written, and nothing is visible), so the only cheap defence
is to assert the geometry before anything is authored.

TWO DISCIPLINES CARRIED OVER FROM `tests/test_tornado_people_poses.py`, and
both matter more than the individual assertions:

1. **A GATE IS TESTED WITH ITS OWN CONTROL.** Every gate test also runs the
   gate OFF and asserts that the unfiltered run DOES produce violations. A
   gate that is wired to nothing passes a one-sided test trivially, and that
   is exactly how `tornado_people`'s `min_intensity` shipped as a paragraph
   of documentation with no code behind it.
2. **THE COPIES ARE CHECKED AGAINST THEIR ORIGINALS.** `fire_people` copies
   four tables out of other modules so it has no import-time dependency on
   them (`urban_fire.BAND`, `quake_flow._SIDE_NORMAL`, `people.LYING_POSES`,
   `people.LYING_SPIN`) and reproduces two functions (`quake_flow.
   _b_face_pt`, `fire_bake.place`'s frame branch). Every one of those is
   asserted equal to the real thing here, so a copy cannot silently drift.
"""
import math
import os
import random
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_TESTS)
for p in (_SCENE_GEN, os.path.join(_SCENE_GEN, "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)

from disaster import fire_people as fp                 # noqa: E402
from disaster import fire_bake as fb                   # noqa: E402
from disaster import ground_class as gc                # noqa: E402
from disaster import people as ppl                     # noqa: E402
from disaster import quake_flow as qf                  # noqa: E402
from disaster import urban_fire as uf                  # noqa: E402
import fire_people_dry_run as dry                      # noqa: E402


def _inputs(seed=4):
    return dry.synth_inputs(seed=seed)


def _plan(seed=7, cfg=None, manifest=None, dump=None, sidecars=None,
          heading_deg=45.0):
    d, m = _inputs()
    return fp.plan_people(dump or d, manifest or m, seed=seed, cfg=cfg,
                          sidecars=sidecars, heading_deg=heading_deg)


def _checks(plan):
    return {name: (ok, detail) for name, ok, detail in fp.check_rules(plan)}


class _FakePools(object):
    """The two accessors `people._human_placement` uses off `asset_pools`,
    answering with the pack's real per-asset constants."""

    def scale_of(self, usd):
        return fp.HUMAN_SCALE

    def axis_of(self, usd):
        return fp.HUMAN_AXIS_UP

    def yaw_of(self, usd):
        return fp.HUMAN_YAW_OFFSET_DEG

    def roll_of(self, usd):
        return 0.0


class _FakeResolver(object):
    """`SizeResolver.get`'s answer for a human, with the pack's measured
    stature/depth. Nominal here: the point of the ctx test is that the
    converter DELEGATES, not that the numbers are the live ones."""

    def get(self, usd, category, scale=1.0, axis_up="Z"):
        return {"sx": 1.24, "sy": 0.36, "sz": fp.NOMINAL_HEIGHT_M,
                "base": 0.0, "cx": 0.0, "cy": 0.0, "cz": 0.0}


def _fake_ctx():
    return {"asset_pools": _FakePools(), "resolver": _FakeResolver()}


def _ringed_inputs():
    """One burning building with intact ones packed all round it at exactly
    the standoff band — so every street candidate the pass can draw lands
    inside a footprint. The control fixture for the footprint gate."""
    H, side = 24.0, 20.0
    stand = 0.33 * H                      # glass_fall_frac * H
    placements = [{"i": 0, "cell": "/c0", "usd": "u.usd",
                   "x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0,
                   "scale": 1.0, "category": "house", "axis_up": "Z",
                   "W": side, "D": side, "H": H}]
    k = 1
    r = side / 2.0 + stand + 0.5
    for a in range(0, 360, 12):
        placements.append({
            "i": k, "cell": "/c%d" % k, "usd": "n.usd",
            "x_m": r * math.cos(math.radians(a)),
            "y_m": r * math.sin(math.radians(a)),
            "z_m": 0.0, "yaw_deg": 0.0, "scale": 1.0,
            "category": "house", "axis_up": "Z",
            "W": 14.0, "D": 14.0, "H": 12.0})
        k += 1
    dump = {"schema": "fire_city_placements_dump.v1", "preset": "t",
            "seed": 0, "region_m": [300.0, 300.0],
            "n_placements_total": k, "placements": placements,
            "typology": {"blocks": [{"rect": [-140.0, -140.0, 140.0, 140.0],
                                     "name": "midrise"}]}}
    man = {"seed": 0, "preset": "t", "n": 1, "n_achieved": 1, "origin": 0,
           "epoch_s": 10080.0, "records": [{
               "usd": "u.usd", "x": 0.0, "y": 0.0, "yaw_deg": 0.0, "z": 0.0,
               "kind": "t", "asset": "u", "style": None,
               "typology": "midrise", "W": side, "D": side, "H": H,
               "cell": "/c0", "i": 0, "level": "F3", "origin": 0,
               "sides": ["N", "E", "S", "W"], "t_ignite_s": 0.0,
               "age_s": 3000.0, "via": None, "how": "origin", "seed": 1,
               "btype": "urm", "entry_side": "N", "origin_frac": 0.1,
               "n_storeys": 7}], "refused": []}
    return dump, man


# ===========================================================================
# 1. The copied tables and the reproduced functions
# ===========================================================================
def test_01_side_normals_match_quake_flow():
    """`SIDE_NORMAL` is `quake_flow._SIDE_NORMAL`. Everything that turns a
    manifest `sides` entry into a world direction goes through it."""
    assert fp.SIDE_NORMAL == qf._SIDE_NORMAL


def test_02_band_matches_urban_fire():
    """`BAND` is `urban_fire.BAND`'s (lo, hi). The WINDOW RULE is derived
    from it — F4 and worse have `hi >= 99`, i.e. no storey above the fire —
    so a drift here silently changes which buildings can carry a window
    figure."""
    for level, (lo, hi) in fp.BAND.items():
        assert (lo, hi) == uf.BAND[level][:2], level
    assert set(fp.BAND) == set(uf.BAND)
    for level in ("F4", "F5", "F5c", "F6"):
        assert fp.BAND[level][1] >= 99, level


def test_03_lying_tables_match_people():
    assert fp.LYING_ROLL == ppl.LYING_POSES
    assert fp.LYING_SPIN == ppl.LYING_SPIN
    assert abs(fp._LATERAL_HALF_BREADTH_H
               - ppl._LATERAL_HALF_BREADTH_H) < 1e-12


def test_04_lying_lift_matches_people():
    """`lying_lift` reproduces `people._lying_lift`: half the body DEPTH
    face-up/face-down, 0.115 H on a side."""
    depth = 2.0 * fp._BODY_HALF_DEPTH_M
    for pose in fp.LYING_ROLL:
        assert abs(fp.lying_lift(pose, 1.78, depth)
                   - ppl._lying_lift(pose, depth, 1.78)) < 1e-12, pose
    # ...and the lateral poses really are lifted more than the flat ones.
    assert fp.lying_lift("lying_side_l") > fp.lying_lift("lying_supine")


def test_05_face_point_matches_quake_flow():
    """`_face_point` IS `quake_flow._b_face_pt`. It is the whole
    opening-to-world transform; a sign error here puts every window figure
    inside the building or a metre out in the air."""
    rng = random.Random(3)
    for _ in range(200):
        fr = (rng.uniform(-40, 40), rng.uniform(-40, 40),
              rng.uniform(-math.pi, math.pi), rng.uniform(4, 40),
              rng.uniform(4, 40), rng.uniform(0, 0.6),
              rng.random() < 0.5)
        u, v, out = rng.uniform(0, 40), rng.uniform(0, 30), rng.uniform(-1, 1)
        a = fp._face_point(fr, u, v, out)
        b = qf._b_face_pt(fr, u, v, out)
        assert max(abs(p - q) for p, q in zip(a, b)) < 1e-12


def test_06_place_frame_matches_fire_bake_place():
    """`place_frame` IS `fire_bake.place`'s frame branch. The city assembly
    uses `place`; if this module moved an opening differently the window
    figures would be at one set of coordinates and the flames at another."""
    rng = random.Random(5)
    for _ in range(60):
        fr = (rng.uniform(-20, 20), rng.uniform(-20, 20),
              rng.uniform(-math.pi, math.pi), 12.0, 9.0, 0.3, False)
        dx, dy, yaw = rng.uniform(-200, 200), rng.uniform(-200, 200), \
            rng.choice([0.0, 90.0, 180.0, 270.0, 37.5])
        events = [{"ops": [{"fr": tuple(fr)}]}]
        fb.place({}, events, None, dx, dy, yaw)
        got = fp.place_frame(fr, dx, dy, yaw)
        want = events[0]["ops"][0]["fr"]
        assert max(abs(a - float(b)) for a, b in zip(got[:6], want[:6])) < 1e-9


def test_07_occlusion_patterns_all_fit_under_the_cap():
    """THE PATTERN MUST MEAN WHAT IT SAYS. `tornado_people`'s `test_20`: a
    pattern that had to be TRIMMED to fit the cap keeps its name and stops
    describing the figure, which is a worse label than none. So every
    pattern this module can draw fits under `MAX_COVERED_FRAC` untrimmed."""
    for name, frac, weight in fp.OCCLUSION:
        assert weight > 0.0, name
        assert frac <= fp.MAX_COVERED_FRAC + 1e-9, (name, frac)


# ===========================================================================
# 2. The ground derivation
# ===========================================================================
def test_08_derived_layout_tiles_the_region_exactly():
    """`road_corridors` is the EXACT complement of the blocks: the two areas
    sum to the region's, with no overlap."""
    dump, _m = _inputs()
    lay = fp.derive_layout(dump)
    x0, y0, x1, y1 = lay["region"]
    region_a = (x1 - x0) * (y1 - y0)
    blk_a = sum((b[2] - b[0]) * (b[3] - b[1]) for b in lay["blocks"])
    road_a = sum((r[2] - r[0]) * (r[3] - r[1]) for r in lay["road_corridors"])
    assert abs(blk_a + road_a - region_a) < 1e-6 * region_a
    # ...and no corridor rect overlaps a block.
    for r in lay["road_corridors"]:
        mx, my = (r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0
        assert not any(b[0] <= mx <= b[2] and b[1] <= my <= b[3]
                       for b in lay["blocks"])


def test_09_sidewalk_rings_come_from_ground_class_itself():
    """The rings are `ground_class._sidewalk_rings`, called — not
    reimplemented — so the two can never disagree."""
    dump, _m = _inputs()
    lay = fp.derive_layout(dump, sidewalk_width_m=2.0)
    assert lay["sidewalk_rects"] == gc._sidewalk_rings(lay["blocks"], 2.0)
    assert lay["sidewalk_rects"]


def test_10_ground_class_answers_road_in_a_street():
    """The sampler is wired: a point in a corridor is `road`, a point deep
    inside a block is not."""
    dump, _m = _inputs()
    ground, lay = fp.make_ground(dump)
    r = lay["road_corridors"][len(lay["road_corridors"]) // 2]
    assert ground.at((r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0) == "road"
    b = lay["blocks"][0]
    assert ground.at((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0) != "road"


# ===========================================================================
# 3. The placement rules, each with its gate-off control
# ===========================================================================
def test_11_nobody_is_inside_a_building_footprint():
    plan = _plan()
    ok, detail = _checks(plan)["no_one_in_a_footprint"]
    assert ok, detail
    assert detail["n_checked"] > 20

    # CONTROL, TWO HALVES, because "no violations" is what a gate wired to
    # NOTHING also reports:
    #   (a) the predicate answers True where it must — a building centre;
    #   (b) the gate actually FIRED during this run, i.e. candidates were
    #       drawn inside a footprint and refused there.
    sol = plan.solver
    cx, cy, W, D, yaw = sol.footprints[0]
    assert sol.in_any_footprint(cx, cy)
    assert not sol.in_any_footprint(cx + 4000.0, cy)
    # (b) a plate where EVERY street candidate lands inside a neighbour:
    #     one burning building ringed by intact ones at exactly the standoff
    #     band. The pass must come back empty and the tally must say why.
    ringed = _ringed_inputs()
    tight = fp.plan_people(ringed[0], ringed[1], seed=3, heading_deg=45.0)
    assert tight.refused.get("in_footprint", 0) > 0, (
        "the footprint gate refused nothing — it may be unwired")
    # The evacuee band (1.0-1.55 x standoff) falls entirely inside the ring,
    # so that class comes back EMPTY. `onlooker` reaches past it at 2.4 x and
    # is expected to survive — the gate is a filter, not a wall.
    assert not [r for r in tight.records if r["cls"] == "evacuee"], (
        [r["reason"] for r in tight.records if r["cls"] == "evacuee"][:3])


def test_12_street_standoff_is_respected():
    plan = _plan()
    ok, detail = _checks(plan)["street_standoff_respected"]
    assert ok, detail

    # CONTROL: crank the standoff up and the SAME positions become illegal —
    # i.e. the check is measuring the standoff and not something else.
    tight = fp.resolve_cfg({"standoff_scale": 4.0})
    sol = plan.solver
    sol.cfg = tight
    bad = [r for r in plan.records if r["cls"] in fp.STREET_CLASSES
           and not sol.clear_of_aprons(r["x"], r["y"])]
    assert bad, "the standoff test is not sensitive to the standoff"


def test_13_the_crowd_is_upwind_and_asymmetric():
    """The single most defensible directional rule in a structure fire, and
    the one that makes the field asymmetric instead of a ring. Measured as
    the mean of every evacuee's own `upwind_cos`."""
    plan = _plan()
    ev = [r["upwind_cos"] for r in plan.records if r["cls"] == "evacuee"]
    assert len(ev) >= 8
    assert sum(ev) / len(ev) > 0.35, sum(ev) / len(ev)

    # CONTROL: turn the wind round and the crowd goes with it.
    flipped = _plan(heading_deg=225.0)
    ev2 = [r for r in flipped.records if r["cls"] == "evacuee"]
    # `upwind_cos` is measured against the run's OWN upwind, so it stays
    # positive; what must move is the crowd's world-space centroid.
    def cen(recs):
        return (sum(r["x"] for r in recs) / len(recs),
                sum(r["y"] for r in recs) / len(recs))
    a = cen([r for r in plan.records if r["cls"] == "evacuee"])
    b = cen(ev2)
    assert math.hypot(a[0] - b[0], a[1] - b[1]) > 20.0, (a, b)


def test_14_street_groups_are_groups_never_singletons():
    plan = _plan()
    ok, detail = _checks(plan)["street_groups_are_groups"]
    assert ok, detail
    counts = {}
    for r in plan.records:
        if r["cls"] in ("evacuee", "onlooker"):
            counts[(r["cls"], r["group"])] = \
                counts.get((r["cls"], r["group"]), 0) + 1
    assert counts
    assert min(counts.values()) >= 2
    lo_e, hi_e = fp.DEFAULTS["group_sizes"]["evacuee"]
    for (cls, _g), n in counts.items():
        if cls == "evacuee":
            assert n <= hi_e, (cls, n)


def test_15_min_separation_holds():
    plan = _plan()
    ok, detail = _checks(plan)["min_separation"]
    assert ok, detail


def test_16_everybody_is_on_the_plate():
    plan = _plan()
    ok, detail = _checks(plan)["on_the_plate"]
    assert ok, detail


# ===========================================================================
# 4. The window class
# ===========================================================================
def test_17_window_storeys_are_strictly_above_the_fire_band():
    """`urban_fire.BAND` gives F4 and worse `hi >= 99` — everything from the
    origin up — so those buildings have NO storey above their fire and
    contribute no window figure. A figure at a window on a floor that is
    itself alight is a fatality behind flame, not a rescue target."""
    plan = _plan()
    ok, detail = _checks(plan)["windows_above_the_fire"]
    assert ok, detail
    wins = [r for r in plan.records if r["cls"] == "window"]
    assert wins
    for r in wins:
        assert r["storey"] > r["band_top"], r
        assert r["building_level"] not in ("F4", "F5", "F5c", "F6"), r


def test_18_every_window_figure_breaks_the_facade_plane():
    plan = _plan()
    ok, detail = _checks(plan)["windows_protrude"]
    assert ok, detail
    for r in plan.records:
        if r["cls"] == "window":
            assert r["protrusion_m"] >= fp.MIN_PROTRUSION_M, r

    # CONTROL: push the pelvis so far inside that nothing can clear the
    # facade, and the whole class is REFUSED rather than placed invisibly.
    deep = _plan(cfg={"sill_inset_m": 3.0, "lean_out_inset_m": 3.0})
    assert not [r for r in deep.records if r["cls"] == "window"]
    assert deep.refused.get("no_protrusion", 0) > 0


def test_19_a_window_figure_sits_on_its_own_facade():
    """The placement point is on the elevation it names — the whole
    `place_frame` / `_face_point` chain, end to end."""
    plan = _plan()
    by_i = {b.i: b for b in plan.solver.buildings}
    for r in plan.records:
        if r["cls"] != "window":
            continue
        b = by_i[r["building_i"]]
        # `sill_inset_m` inside the wall, so within a metre of the surface.
        assert fp.dist_to_obb(r["x"], r["y"], b.x, b.y, b.W, b.D,
                              b.yaw) < 1.0, r
        # The yaw points OUT of the building, not into it.
        ux = math.cos(math.radians(r["yaw_deg"]))
        uy = math.sin(math.radians(r["yaw_deg"]))
        assert (r["x"] - b.x) * ux + (r["y"] - b.y) * uy > 0.0, r
        # z is a real storey, above the fire band, below the roof.
        assert 0.0 < r["z"] < b.H, r


def test_20_a_sidecar_supplies_the_real_opening_grid():
    """With a sidecar the openings are measured, not defaulted, and the
    record says so. The bake records only VENTING openings, so the grid is
    lifted to a storey above the fire — which is the whole point."""
    dump, man = _inputs()
    # Give the F1 building (which has a storey above its band) a sidecar
    # carrying two measured openings on its own venting side.
    rec = [r for r in man["records"] if r["level"] == "F1"][0]
    side = rec["sides"][0]
    period = rec["H"] / rec["n_storeys"]
    lx, ly = fp.SIDE_NORMAL[side]
    half = (rec["W"] / 2.0) if side in ("N", "S") else (rec["D"] / 2.0)
    fr = (-half, -((rec["D"] / 2.0) if side in ("N", "S")
                   else (rec["W"] / 2.0)),
          0.0, 2.0 * half, rec["H"], 0.0, False)
    ops = [{"fr": list(fr), "side": side, "storey": rec["origin"],
            "span": [u - 0.6, u + 0.6, rec["origin"] * period + 1.05,
                     rec["origin"] * period + 2.30]}
           for u in (3.0, 7.4, 11.8)]
    doc = {"fire": {"top": rec["origin"], "roof": False,
                    "n_storeys": rec["n_storeys"], "deck_z": rec["H"] - 0.8},
           "events": [{"side": side, "storey": rec["origin"], "ops": ops}],
           "masses": {}, "notes": [], "top_z": rec["H"]}
    plan = fp.plan_people(dump, man, seed=7,
                          sidecars={rec["cell"]: doc}, heading_deg=45.0,
                          cfg={"shares": {"window": 0.9, "evacuee": 0.05,
                                          "onlooker": 0.05, "at_car": 0.0,
                                          "roof": 0.0, "casualty_apron": 0.0,
                                          "roof_debris": 0.0},
                               "window_max_per_building": 40})
    got = [r for r in plan.records
           if r["cls"] == "window" and r["building_i"] == rec["i"]]
    assert got, "the sidecar building got no window figure"
    assert any(r["openings_source"] == "sidecar_grid" for r in got)
    for r in got:
        if r["openings_source"] != "sidecar_grid":
            continue
        # The measured sill sits 1.05 m over its storey floor; the grid is
        # lifted verbatim, so every record's sill does too.
        assert abs((r["sill_z"] - r["floor_z"]) - 1.05) < 1e-6, r
        assert r["storey"] > rec["origin"], r


def test_21_a_derived_opening_is_flagged_for_the_bench():
    """No sidecar means the grid is a DEFAULT, and a default grid is a claim
    nobody has measured — so it carries `needs_bench` and the census can
    count it."""
    plan = _plan()
    wins = [r for r in plan.records if r["cls"] == "window"]
    assert wins
    for r in wins:
        if r["openings_source"] == "derived":
            assert r["needs_bench"], r


# ===========================================================================
# 5. The roof class
# ===========================================================================
def test_22_roofs_only_on_intact_decks():
    plan = _plan()
    ok, detail = _checks(plan)["roofs_on_intact_decks"]
    assert ok, detail
    roofs = [r for r in plan.records if r["cls"] == "roof"]
    assert roofs
    for r in roofs:
        assert r["building_level"] in fp.DEFAULTS["roof_ok_levels"], r
    # F4 and worse can NEVER carry one — `urban_fire.BAND` again.
    assert plan.refused.get("roof:roof_deck_involved(F4)", 0) > 0


def test_23_a_roof_group_sits_on_deck_z_not_top_z():
    """`gac_fire`: the bbox top is the PARAPET COPING, not the deck. A
    figure on `top_z` is standing on the coping."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] in ("F1", "F3")][0]
    doc = {"fire": {"top": 0, "roof": False, "deck_z": 11.5},
           "events": [], "masses": {}, "notes": [], "top_z": rec["H"]}
    plan = fp.plan_people(dump, man, seed=7, sidecars={rec["cell"]: doc},
                          heading_deg=45.0,
                          cfg={"shares": {"roof": 0.9, "evacuee": 0.05,
                                          "onlooker": 0.05, "at_car": 0.0,
                                          "window": 0.0,
                                          "casualty_apron": 0.0,
                                          "roof_debris": 0.0}})
    got = [r for r in plan.records
           if r["cls"] == "roof" and r["building_i"] == rec["i"]]
    assert got
    for r in got:
        assert abs(r["z"] - 11.5) < 1e-9, r
        assert r["deck_source"] == "sidecar"
        assert r["z"] != rec["H"]


def test_24_a_roof_group_is_inside_the_roof_plan():
    """Inset from the parapet, never past it — otherwise the figure is
    hanging in the air beside the building."""
    plan = _plan()
    by_i = {b.i: b for b in plan.solver.buildings}
    for r in plan.records:
        if r["cls"] != "roof":
            continue
        b = by_i[r["building_i"]]
        assert fp.point_in_obb(r["x"], r["y"], b.x, b.y, b.W, b.D, b.yaw,
                               margin=0.0), r


def test_25_a_tall_building_gets_no_roof_group():
    plan = _plan(cfg={"roof_max_h_m": 1.0})
    assert not [r for r in plan.records if r["cls"] == "roof"]
    assert any(k.startswith("roof:too_tall") for k in plan.refused)


# ===========================================================================
# 6. The two burial classes
# ===========================================================================
def test_26_nothing_is_ever_fully_buried_over_five_seeds():
    for seed in (1, 2, 3, 5, 8):
        plan = _plan(seed=seed)
        ok, detail = _checks(plan)["nothing_fully_buried"]
        assert ok, (seed, detail)
        for r in plan.records:
            if r.get("covered_frac") is not None:
                assert r["covered_frac"] <= fp.MAX_COVERED_FRAC, (seed, r)


def test_27_burials_lie_in_the_outer_band_never_the_mound():
    """`tornado_people`'s `in_wreck` lesson: a body in the middle of the
    deepest material is invisible from every angle."""
    plan = _plan()
    ok, detail = _checks(plan)["burials_in_the_outer_band"]
    assert ok, detail
    got = [r for r in plan.records if r.get("apron_t") is not None]
    assert got
    for r in got:
        band = (fp.DEFAULTS["apron_band"] if r["cls"] == "casualty_apron"
                else fp.DEFAULTS["roof_debris_band"])
        assert band[0] - 1e-3 <= r["apron_t"] <= band[1] + 1e-3, r
        # ...and the surface it lies on is a TAIL, not the wall-line depth.
        assert r["surface_z"] < r["debris_depth_m"] * 0.5, r


def test_28_every_burial_is_flagged_for_the_bench():
    """The tornado skill is unambiguous: a partially-buried figure can only
    be verified on a dedicated render bench. Nothing in a 2-D run clears
    this flag and nothing may set it False."""
    plan = _plan()
    ok, detail = _checks(plan)["burials_flagged_for_the_bench"]
    assert ok, detail
    got = [r for r in plan.records
           if r["cls"] in ("casualty_apron", "roof_debris")]
    assert got
    assert all(r["needs_bench"] for r in got)


def test_29_a_lying_figure_is_laid_down_with_its_own_roll():
    """`people._human_placement` RAISES on a lying pose placed upright, and
    the roll SIGN comes from the pose: +90 is face-down, -90 face-up. A coin
    flip buries a raised knee."""
    plan = _plan()
    ok, detail = _checks(plan)["lying_poses_are_laid_down"]
    assert ok, detail
    got = [r for r in plan.records if str(r.get("pose")) in fp.LYING_ROLL]
    assert got
    for r in got:
        assert r["prone"] is True, r
        assert r["roll_deg"] == ppl.LYING_POSES[r["pose"]], r
        assert r["pitch_deg"] == ppl.LYING_SPIN.get(r["pose"], 0.0), r
        assert r["rigged"] is True, r          # a static cannot take a pose


def test_30_roof_debris_refuses_a_building_with_four_walls_standing():
    """An F5/F6 shell dropped its floors and its deck INWARD, where no
    camera goes. Placing a figure there is `people.py`'s retired
    `exposed_interior` under a new name."""
    dump, man = _inputs()
    for r in man["records"]:
        if r["level"] == "F5c":
            r["level"] = "F5"          # roof down, every wall up
    plan = fp.plan_people(dump, man, seed=7, heading_deg=45.0)
    assert not [r for r in plan.records if r["cls"] == "roof_debris"]
    assert plan.refused.get("roof_debris_indoors", 0) > 0


# ===========================================================================
# 7. Degradation, determinism, the visibility filter, pose rules
# ===========================================================================
def test_31_a_class_with_no_eligible_building_degrades_gracefully():
    """Roof collapse will be RARE city-wide, so `roof_debris` may have no
    candidate at all. That is expected: the class places nothing, the
    degradation is RECORDED, and the head count is still met because the
    share goes back to the street classes."""
    dump, man = _inputs()
    for r in man["records"]:
        if r["level"] == "F5c":
            r["level"] = "F4"          # nothing collapsed anywhere
    plan = fp.plan_people(dump, man, seed=7, heading_deg=45.0)
    assert not [r for r in plan.records if r["cls"] == "casualty_apron"]
    assert not [r for r in plan.records if r["cls"] == "roof_debris"]
    assert "casualty_apron" in plan.degraded
    assert "roof_debris" in plan.degraded
    base = _plan()
    assert len(plan.records) >= len(base.records) - 4
    # ...and the give-back landed on the street classes, not nowhere.
    n_street = len([r for r in plan.records if r["cls"] in fp.STREET_CLASSES])
    n_street0 = len([r for r in base.records
                     if r["cls"] in fp.STREET_CLASSES])
    assert n_street > n_street0


def test_32_the_plan_is_deterministic_in_its_seed():
    a = _plan(seed=11)
    b = _plan(seed=11)
    c = _plan(seed=12)
    key = lambda p: [(r["cls"], r["x"], r["y"], r["z"], r["pose"])
                     for r in p.records]
    assert key(a) == key(b)
    assert key(a) != key(c)


def test_33_the_aerial_visibility_filter_drops_rather_than_flags():
    """An unlabelable target in the ground truth is worse than a missing
    one, so a record that fails the filter is DROPPED and counted."""
    plan = _plan()
    assert plan.records
    assert all(r["aerial_visible"] for r in plan.records)
    deep = _plan(cfg={"sill_inset_m": 3.0, "lean_out_inset_m": 3.0})
    assert deep.refused.get("no_protrusion", 0) > 0
    assert all(r["aerial_visible"] for r in deep.records)


def test_34_pose_rules():
    """No banned pose anywhere; a posed static NEVER carries one (it has no
    skeleton, so binding a pose does nothing and ships the figure upright,
    silently)."""
    plan = _plan()
    ok, detail = _checks(plan)["pose_rules"]
    assert ok, detail
    for r in plan.records:
        assert r["pose"] not in ppl.BANNED_POSES
        if not r["rigged"]:
            assert r["pose"] in (None, "idle"), r
            assert r["usd"] in fp.POSED_HUMANS, r
        else:
            assert r["usd"] in fp.RIGGED_HUMANS, r
    # A seated pose always names the seat the caller can see — the rule
    # `people.add_person` enforces.
    for r in plan.records:
        if r["pose"] == "sit_edge":
            assert r["seat"], r


def test_35_nobody_is_indoors_and_no_class_is_secretly_interior():
    """The single largest population in a real structure fire is INSIDE, and
    it is out of scope by construction. Nothing may creep back in: every
    record is on the ground, on a facade, on a deck or in an apron."""
    plan = _plan()
    assert {r["z_mode"] for r in plan.records} <= {
        "ground", "sill", "floor", "deck", "debris"}
    for r in plan.records:
        if r["z_mode"] == "floor":
            # the ONLY interior-standing case, and it must lean out
            assert r["cls"] == "window" and r["protrusion_m"] > 0.0, r


def test_36_the_dry_run_gate_passes_end_to_end():
    """The tool itself, including the census and the checks — so a change
    that breaks the reporting is caught here and not on a review."""
    plan = _plan()
    cen = fp.summarise(plan)
    assert cen["total"] == len(plan.records)
    assert cen["locations"] >= 10
    assert cen["needs_bench"] >= len(
        [r for r in plan.records
         if r["cls"] in ("casualty_apron", "roof_debris")])
    assert all(ok for _n, ok, _d in fp.check_rules(plan))


def test_36b_both_window_variants_are_reachable():
    """BOTH VARIANTS MUST BE ABLE TO EXIST. `lean_out` was charged the sill
    inset AND its own body depth, which left it 0.02 m of protrusion — under
    `MIN_PROTRUSION_M`, so every candidate was refused and a whole branch was
    dead behind a number that looked reasonable. The inset is the only term
    that positions the body; there is no second correction."""
    for variant, inset in (("sill_sit", fp.DEFAULTS["sill_inset_m"]),
                           ("lean_out", fp.DEFAULTS["lean_out_inset_m"])):
        prot = fp.window_protrusion_m(variant, inset)
        assert prot >= fp.MIN_PROTRUSION_M, (variant, prot)
    # ...and with sitting turned off, the planner really does produce them.
    plan = _plan(cfg={"sill_sit_share": 0.0})
    got = [r for r in plan.records if r["cls"] == "window"]
    assert got
    assert {r["variant"] for r in got} == {"lean_out"}
    for r in got:
        assert r["needs_bench"], r          # 0.22 m is a render question
        assert r["z_mode"] == "floor" and r["pose"] == "stand_slump", r


def test_37_a_window_above_the_drone_ceiling_is_refused():
    """`benchmark-disaster-dataset` flies the urban search at 15-40 m AGL.
    The first real-city run put a figure at a 27th-storey window 83 m up:
    real, and above every camera that will look for it. `window_max_z_m`
    refuses it by name rather than shipping an unfindable label."""
    plan = _plan(cfg={"window_max_z_m": 6.0})
    assert not [r for r in plan.records if r["cls"] == "window"]
    assert plan.refused.get("window_too_high", 0) > 0
    # ...and with the ceiling raised the class comes back, so the knob is
    # doing the work and nothing else is.
    back = _plan(cfg={"window_max_z_m": 400.0})
    assert [r for r in back.records if r["cls"] == "window"]
    for r in _plan().records:
        if r["cls"] == "window":
            assert r["z"] <= fp.DEFAULTS["window_max_z_m"], r


def test_37b_the_ceiling_is_enforced_on_the_measured_opening():
    """The storey pre-filter screens with the DEFAULT sill height; a sidecar's
    MEASURED sill can sit metres higher up its own storey. The first real
    sidecar run shipped four records above the ceiling because only the
    pre-filter was checked. The rule is enforced on the opening that was
    actually chosen."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F1"][0]
    side = rec["sides"][0]
    period = rec["H"] / rec["n_storeys"]
    half = (rec["W"] / 2.0) if side in ("N", "S") else (rec["D"] / 2.0)
    fr = (-half, -((rec["D"] / 2.0) if side in ("N", "S")
                   else (rec["W"] / 2.0)),
          0.0, 2.0 * half, rec["H"], 0.0, False)
    # A sill measured 6 m above its storey floor — absurd as art, exact as a
    # regression: the pre-filter allows the storey on `sill_h_m` = 1.0 m.
    ops = [{"fr": list(fr), "side": side, "storey": rec["origin"],
            "span": [u - 0.6, u + 0.6, rec["origin"] * period + 6.0,
                     rec["origin"] * period + 7.4]}
           for u in (3.0, 7.4, 11.8)]
    doc = {"fire": {"top": rec["origin"], "roof": False,
                    "n_storeys": rec["n_storeys"]},
           "events": [{"side": side, "storey": rec["origin"], "ops": ops}],
           "masses": {}, "notes": [], "top_z": rec["H"]}
    ceiling = rec["origin"] * period + 3.0
    plan = fp.plan_people(dump, man, seed=7, sidecars={rec["cell"]: doc},
                          heading_deg=45.0,
                          cfg={"window_max_z_m": ceiling,
                               "shares": {"window": 0.9, "evacuee": 0.05,
                                          "onlooker": 0.05, "at_car": 0.0,
                                          "roof": 0.0, "casualty_apron": 0.0,
                                          "roof_debris": 0.0}})
    got = [r for r in plan.records
           if r["cls"] == "window" and r["building_i"] == rec["i"]]
    assert not got, [r["z"] for r in got]
    assert plan.refused.get("window_too_high", 0) > 0
    ok, detail = _checks(plan)["windows_under_the_drone_ceiling"]
    assert ok, detail


def test_38_roof_groups_spread_over_the_eligible_decks():
    """Coverage is counted in LOCATIONS. Nine figures on one roof and none
    on the eight other eligible decks is ONE location — the defect the first
    real-city run shipped."""
    plan = _plan()
    roofs = [r for r in plan.records if r["cls"] == "roof"]
    assert roofs
    per = {}
    for r in roofs:
        per[r["building_i"]] = per.get(r["building_i"], 0) + 1
    eligible = [b for b in plan.solver.buildings
                if b.roof_ok(plan.cfg)[0]]
    assert len(per) >= min(len(eligible), 2), (per, len(eligible))
    lo, hi = fp.DEFAULTS["group_sizes"]["roof"]
    cap = hi * int(fp.DEFAULTS["roof_max_groups_per_building"])
    for i, n in per.items():
        assert n <= cap, (i, n, cap)


def test_39_the_surface_preference_prefers_the_frontage():
    """A `paved` BLOCK INTERIOR is the back of the block — real, reachable by
    a rear exit, and much harder to see from above between two buildings. The
    nudge moves a figure to a better-ranked surface when one is reachable and
    keeps it where it is when none is.

    TESTED ON THE MECHANISM, NOT ON THE HEAD COUNT. The synthetic fixture's
    blocks are 99 m across, so most of a block interior is further from a
    kerb than the nudge's reach and the aggregate mix says nothing either
    way — a share assertion there would be measuring the fixture.
    """
    dump, man = _inputs()
    plan = fp.plan_people(dump, man, seed=7, heading_deg=45.0)
    sol = plan.solver

    # Every placed street figure is on a ranked surface.
    street = [r for r in plan.records if r["cls"] in fp.STREET_CLASSES]
    assert street
    assert all(r["surface"] in fp.DEFAULTS["surface_rank"] for r in street)

    # A point on a block interior with a sidewalk ring within reach MOVES.
    ring = sol.layout["sidewalk_rects"][0]
    sx, sy = (ring[0] + ring[2]) / 2.0, (ring[1] + ring[3]) / 2.0
    assert sol.ground.at(sx, sy) == "sidewalk"
    # ...step 3 m into the block from that ring; that is `paved` here.
    for dx, dy in ((3.0, 0.0), (-3.0, 0.0), (0.0, 3.0), (0.0, -3.0)):
        px, py = sx + dx, sy + dy
        if sol.ground.at(px, py) == "paved":
            break
    else:
        raise AssertionError("no paved point beside a sidewalk ring")
    rng = random.Random(0)
    nx, ny, ns = fp._nudge_surface(sol, px, py, "paved", rng)
    assert ns == "sidewalk", (ns, px, py)
    assert (nx, ny) != (px, py)

    # CONTROL: rank `paved` first and the SAME point stays put, so the
    # ranking is what is doing the work.
    sol.cfg = fp.resolve_cfg({"surface_rank": ("paved", "sidewalk", "road")})
    nx2, ny2, ns2 = fp._nudge_surface(sol, px, py, "paved",
                                      random.Random(0))
    assert (nx2, ny2, ns2) == (px, py, "paved")


# ===========================================================================
# 8. The converter — records -> `apply_placements` dicts
# ===========================================================================
_CONTRACT = {"usd", "x_m", "y_m", "z_m", "yaw_deg", "roll_deg", "pitch_deg",
             "scale", "category", "axis_up"}


def _one(plan, cls, **want):
    for r in plan.records:
        if r["cls"] != cls:
            continue
        if all(r.get(k) == v for k, v in want.items()):
            return r
    raise AssertionError("no {0} record with {1}".format(cls, want))


def test_39b_a_skyscraper_cannot_throw_the_crowd_onto_the_next_block():
    """`glass_fall_frac * H` is calibrated on mid-rise frontage and does not
    extrapolate. The 2026-08-31 manifest put 140-302 m skyscrapers in the
    burnable set, where it is a 100 m keepout and the onlooker band threw
    figures 182 m from their own building. Capped — a scene constraint, and
    it says so."""
    dump, man = _inputs()
    tall = dict(man["records"][0], H=302.0, n_storeys=101, level="F4",
                W=60.0, D=60.0)
    man = dict(man, records=[tall] + list(man["records"][1:]))
    plan = fp.plan_people(dump, man, seed=7, heading_deg=45.0)
    b = [x for x in plan.solver.buildings if x.H > 300.0][0]
    assert b.standoff <= fp.DEFAULTS["standoff_max_m"] + 1e-9, b.standoff
    for r in plan.records:
        if r.get("d_wall_m") is not None:
            assert r["d_wall_m"] <= fp.DEFAULTS["max_wall_dist_m"] + 0.01, r
    ok, detail = _checks(plan)["crowd_belongs_to_its_building"]
    assert ok, detail

    # CONTROL: lift the cap and the same seed DOES throw people out there,
    # so the cap is what is holding them in.
    loose = fp.plan_people(dump, man, seed=7, heading_deg=45.0,
                           cfg={"standoff_max_m": 1e6,
                                "max_wall_dist_m": 1e6})
    assert max(r["d_wall_m"] for r in loose.records
               if r.get("d_wall_m") is not None) > 90.0


def test_40_converted_dicts_match_the_contract_exactly():
    """`apply_placements` reads a fixed key set and nothing else. Extra keys
    are not an error but they are not the contract either, so the default
    output carries none — `tag_ids=True` is the opt-in."""
    plan = _plan()
    ps, skipped = fp.to_placements(plan)
    assert ps
    for p in ps:
        assert _CONTRACT <= set(p), (set(p) ^ _CONTRACT, p)
        assert set(p) - _CONTRACT <= {"pose"}, set(p) - _CONTRACT
        assert p["category"] == "human"
        assert p["scale"] == fp.HUMAN_SCALE
        assert p["axis_up"] == fp.HUMAN_AXIS_UP
        assert isinstance(p["z_m"], float)
    tagged, _ = fp.to_placements(plan, tag_ids=True)
    assert all("fire_people_id" in p for p in tagged)
    # 1:1 and in record order over the kept records.
    n_skipped = sum(len(v) for v in skipped.values())
    assert len(ps) + n_skipped == len(plan.records)


def test_41_a_street_stander_converts():
    """Ground support + the pose drop + the pack's yaw offset, and no roll or
    pitch on anything that is standing."""
    plan = _plan()
    rec = _one(plan, "evacuee", pose="idle", rigged=True)
    p, _sk = fp.to_placements([rec])
    p = p[0]
    assert p["yaw_deg"] == rec["yaw_deg"] + fp.HUMAN_YAW_OFFSET_DEG
    assert p["roll_deg"] == 0.0 and p["pitch_deg"] == 0.0
    # `idle` has a zero pose offset, so the prim z IS the support surface.
    assert abs(p["z_m"] - rec["z"]) < 1e-9
    assert p["pose"] == "idle"
    assert p["x_m"] == rec["x"] and p["y_m"] == rec["y"]


def test_42_a_kerb_sitter_gets_the_seated_drop():
    """`sit_edge` is placed ON a seat the caller can see (the kerb), so the
    record's z is the seat pan and the pose's own drop lands the pelvis on
    it."""
    import scene_generator as sg
    plan = _plan()
    rec = _one(plan, "evacuee", pose="sit_edge")
    p, _sk = fp.to_placements([rec])
    p = p[0]
    dz = sg.pose_z_offset(rec["usd"], "sit_edge", fp.NOMINAL_HEIGHT_M)
    assert dz < -0.5, dz                       # it really is a big drop
    assert abs(p["z_m"] - (rec["z"] + dz)) < 1e-9
    assert p["pose"] == "sit_edge"


def test_43_a_prone_burial_converts():
    """LAID DOWN BY THE PLACEMENT'S ROLL, and the SIGN comes from the pose:
    +90 is face-down, -90 face-up. The lift is added here, once, on top of
    the debris SURFACE the record carries."""
    plan = _plan()
    rec = _one(plan, "casualty_apron")
    p, _sk = fp.to_placements([rec])
    p = p[0]
    pose = rec["pose"]
    assert p["pose"] == pose
    assert p["roll_deg"] == ppl.LYING_POSES[pose]
    assert p["pitch_deg"] == ppl.LYING_SPIN.get(pose, 0.0)
    lift = fp.lying_lift(pose, fp.NOMINAL_HEIGHT_M)
    assert lift > 0.10
    assert abs(p["z_m"] - (rec["z"] + lift)) < 1e-9
    # THE RECORD'S z IS THE SURFACE, NOT THE PRIM z — the lift must not be
    # baked in twice.
    assert abs(rec["z"] - rec["surface_z"]) < 1e-9


def test_44_a_window_sill_sitter_and_a_roof_figure_convert():
    plan = _plan()
    import scene_generator as sg

    w = _one(plan, "window", variant="sill_sit")
    p = fp.to_placements([w])[0][0]
    assert w["z"] == w["sill_z"]                 # the seat IS the sill
    dz = sg.pose_z_offset(w["usd"], "sit_edge", fp.NOMINAL_HEIGHT_M)
    assert abs(p["z_m"] - (w["sill_z"] + dz)) < 1e-9
    assert p["roll_deg"] == 0.0 and p["pitch_deg"] == 0.0

    r = _one(plan, "roof")
    p = fp.to_placements([r])[0][0]
    base = r["z"]
    dz = (sg.pose_z_offset(r["usd"], r["pose"], fp.NOMINAL_HEIGHT_M)
          if r["pose"] else 0.0)
    assert abs(p["z_m"] - (base + dz)) < 1e-9
    assert p["z_m"] > 5.0                        # it is on a roof


def test_45_a_posed_static_never_carries_a_pose_key():
    """`apply_placements` calls `_bind_human_pose` on any truthy `pose`.
    Three of the nine characters have NO SKELETON, so the key must be absent
    rather than None — and it must never be some other pose."""
    plan = _plan()
    ps, _sk = fp.to_placements(plan)
    statics = [p for p in ps if p["usd"] in fp.POSED_HUMANS]
    assert statics
    for p in statics:
        assert "pose" not in p, p
        assert p["roll_deg"] == 0.0 and p["pitch_deg"] == 0.0


def test_46_unconvertible_records_are_skipped_with_a_reason():
    """A figure that cannot be authored correctly is DROPPED and counted —
    never fudged into something wrong. Same discipline as the
    aerial-visibility filter."""
    good = _one(_plan(), "evacuee", pose="idle", rigged=True)
    cases = [
        (dict(good, id=1, pose="lying_prone", prone=False),
         "lying_pose_not_prone"),
        (dict(good, id=2, pose="idle", prone=True),
         "prone_without_lying_pose"),
        (dict(good, id=3, usd=fp.POSED_HUMANS[0], rigged=False,
              pose="crouch"), "static_with_pose"),
        (dict(good, id=4, usd=fp.POSED_HUMANS[0], rigged=False,
              pose="lying_supine", prone=True), "static_with_pose"),
        (dict(good, id=5, usd="omniverse://nowhere/rp_ghost.usd"),
         "unknown_asset"),
    ]
    for rec, why in cases:
        ps, skipped = fp.to_placements([rec])
        assert ps == [], (why, ps)
        assert list(skipped) == [why], (why, skipped)
        assert skipped[why] == [rec["id"]]

    # A static laid down is its own reason when the pose is a legal one for
    # a static to be holding (i.e. none).
    rec = dict(good, id=6, usd=fp.POSED_HUMANS[0], rigged=False, pose=None,
               prone=True)
    ps, skipped = fp.to_placements([rec])
    assert ps == [] and "prone_without_lying_pose" in skipped

    # ...and with a `ctx` the unknown-asset gate is off, because the asset
    # pools answer for it instead.
    ps, skipped = fp.to_placements([cases[4][0]], ctx=_fake_ctx())
    assert len(ps) == 1 and not skipped


def test_47_needs_bench_records_still_convert():
    """`needs_bench` is a note to a REVIEWER, not a defect: those figures are
    placed and their arithmetic is done. Skipping them would silently drop
    every burial and every derived-opening window figure."""
    plan = _plan()
    ps, skipped = fp.to_placements(plan)
    bench = [r for r in plan.records if r.get("needs_bench")]
    assert bench
    skipped_ids = {i for v in skipped.values() for i in v}
    assert not ({r["id"] for r in bench} & skipped_ids)


def test_48_the_ctx_path_delegates_to_people_human_placement():
    """With a `ctx` the whole job goes to `people._human_placement` — the
    measured path — and this asserts it is really that function's output,
    not a lookalike."""
    plan = _plan()
    ctx = _fake_ctx()
    for cls in ("evacuee", "window", "roof", "casualty_apron"):
        rec = _one(plan, cls)
        got = fp.to_placements([rec], ctx=ctx)[0][0]
        want = ppl._human_placement(ctx, rec["usd"], rec["x"], rec["y"],
                                    rec["z"], rec["yaw_deg"], rec.get("pose"),
                                    prone=bool(rec.get("prone")))
        want.pop("pose", None) if not want.get("pose") else None
        for k in _CONTRACT:
            assert got[k] == want[k], (cls, k, got[k], want[k])


def test_49_a_two_origin_manifest_needs_no_generalisation():
    """A manifest may carry several ignition points (`origins`). Nothing in
    the planner reads them — every pass keys off a building's OWN sides,
    level and age — so the only thing that changes is the record list."""
    dump, man = _inputs()
    man = dict(man)
    man["origins"] = [man["records"][0]["i"], man["records"][5]["i"]]
    man["records"][5] = dict(man["records"][5], via=None, how="origin",
                             t_ignite_s=0.0)
    plan = fp.plan_people(dump, man, seed=7, heading_deg=45.0)
    assert plan.meta["origins"] == man["origins"]
    assert plan.records
    assert all(ok for _n, ok, _d in fp.check_rules(plan))
    # ...and both fires actually carry people.
    served = {r["building_i"] for r in plan.records}
    assert len(served) >= 3


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
    print("\nall fire_people rules pinned")
