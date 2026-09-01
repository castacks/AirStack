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
import json
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


def _street_plan(seed=7, cfg=None, **kw):
    """A plan with the street classes ON.

    ITEM 3 (2026-08-31 review, "No people on the ground that aren't like part
    of the damage") turned `evacuee`/`onlooker`/`at_car` OFF by default, so
    every street-mechanics test — the standoff, the upwind asymmetry, group
    sizes, the surface nudge, the kerb-sitter conversions — now pins the
    KNOB-ON path explicitly rather than the default census. The default path
    (no street figures at all) has its own tests in section 12.
    """
    c = dict(cfg or {})
    c.setdefault("street_classes", True)
    return _plan(seed=seed, cfg=c, **kw)


def _new_pose_plan(seed=7, cfg=None, roof=True, window=True, **kw):
    """A plan with the NEW roof/window poses turned back ON.

    ITEM 1 (2026-08-31 bench-v2 REJECTION: "these poses are completely
    wrong ... Just spawn people only") took `stand_calm`/`wave_help`
    (roof/roof_victim) and `lean_window` (window) out of the DEFAULT path —
    every class now ships on `idle` until a render confirms the fix (see
    `people.WINDOW_POSE_DIAGNOSIS`). The mechanism itself (the pose table
    entries, the measured-fraction protrusion formula, the standing-only
    class-pose restriction) is still real code and still has to be pinned,
    so tests that check IT explicitly opt back in here rather than testing
    against a default that no longer exercises it.
    """
    c = dict(cfg or {})
    if roof:
        c.setdefault("roof_use_new_pose", True)
    if window:
        c.setdefault("window_use_lean_pose", True)
    return _plan(seed=seed, cfg=c, **kw)


def _local_wall_frame(rec, side):
    """A geometrically CORRECT local (pre-cell-yaw) opening frame for one
    `side` of `rec`, built the same way `openings_for_side`'s own 'derived'
    branch builds one — for a building placed at the ORIGIN with yaw 0, so
    the caller's `place_frame(fr, rec["x"], rec["y"], rec["yaw_deg"])` (which
    is exactly what `openings_for_side`'s `sidecar_grid` branch does to
    `ops[0]["fr"]`) lands it correctly on the real building.

    2026-08-31: the fixtures this replaces (`test_20`, `test_37b`) hand-rolled
    `fr = (-half, -otherhalf, 0.0, ...)` — a LOCAL yaw of 0 regardless of
    which side it was for, which is only correct for one of the four sides.
    On the others `place_frame` still "worked" (nothing raised), but the
    reconstructed point landed several METRES off the building's own wall —
    invisible until `_pass_window`'s new `opening_off_building` guard (added
    the same day the geometry was fixed to "hips at the sill") started
    checking a window figure's distance to its own building at all. Reusing
    `face_center`/`side_normal_world`, the same functions the production
    'derived' branch calls, means this cannot drift into a second, unverified
    implementation of the same geometry.
    """
    fake = dict(rec, x=0.0, y=0.0, yaw_deg=0.0)
    cx, cy, half = fp.face_center(fake, side)
    nx, ny = fp.side_normal_world(side, 0.0)
    tx, ty = -ny, nx
    fr = (cx - tx * half, cy - ty * half, math.atan2(ty, tx),
          2.0 * half, float(rec["H"]), 0.0, False)
    px, py, _z = fp._face_point(fr, half, 0.0, 1.0)
    if (px - cx) * nx + (py - cy) * ny < 0.0:
        fr = (cx + tx * half, cy + ty * half, math.atan2(-ty, -tx),
              2.0 * half, float(rec["H"]), 0.0, False)
    return fr


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
    plan = _street_plan()
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
    tight = fp.plan_people(ringed[0], ringed[1], seed=3, heading_deg=45.0,
                           cfg={"street_classes": True})
    assert tight.refused.get("in_footprint", 0) > 0, (
        "the footprint gate refused nothing — it may be unwired")
    # The evacuee band (1.0-1.55 x standoff) falls entirely inside the ring,
    # so that class comes back EMPTY. `onlooker` reaches past it at 2.4 x and
    # is expected to survive — the gate is a filter, not a wall.
    assert not [r for r in tight.records if r["cls"] == "evacuee"], (
        [r["reason"] for r in tight.records if r["cls"] == "evacuee"][:3])


def test_12_street_standoff_is_respected():
    plan = _street_plan()
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
    plan = _street_plan()
    ev = [r["upwind_cos"] for r in plan.records if r["cls"] == "evacuee"]
    assert len(ev) >= 8
    assert sum(ev) / len(ev) > 0.35, sum(ev) / len(ev)

    # CONTROL: turn the wind round and the crowd goes with it.
    flipped = _street_plan(heading_deg=225.0)
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
    plan = _street_plan()
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
    # `lean_window_inset_m` (NOT `lean_out_inset_m`, which the retired
    # spine-hinged pose used) drives the FINALIZED pelvis-hinged pose's
    # geometry — see that constant's own account, 2026-09-01.
    deep = _plan(cfg={"lean_window_inset_m": 3.0})
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
    fr = _local_wall_frame(rec, side)
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
# 5b2. roof_victim — stranded figures on an intact roof (section 5b2)
# ===========================================================================
def test_25b_roof_victim_only_on_intact_non_collapsed_roofs():
    """The user's ask, verbatim: "Only on non-collapsed roofs." Every
    surviving record's own building must be genuinely intact per
    `roof_victim_ok` — never `roof_involved`, never `roof_collapsed`, never a
    `roof_victim_excluded_levels` level."""
    plan = _plan()
    ok, detail = _checks(plan)["roof_victim_on_intact_roofs"]
    assert ok, detail
    got = [r for r in plan.records if r["cls"] == "roof_victim"]
    assert got
    by_i = {b.i: b for b in plan.solver.buildings}
    for r in got:
        b = by_i[r["building_i"]]
        assert not b.roof_involved, r
        assert not b.roof_collapsed, r
        assert r["building_level"] not in fp.DEFAULTS[
            "roof_victim_excluded_levels"], r


def test_25c_roof_victim_excludes_the_f5c_origin_by_name():
    """The synthetic fixture's own origin building is F5c (`synth_inputs`'s
    level histogram). It must never carry a `roof_victim`, and the refusal
    must say so BY THE EXCLUDED-LEVEL NAME, not fall through to
    `band_reaches_top` only by accident."""
    plan = _plan()
    f5c = [b for b in plan.solver.buildings if b.level == "F5c"]
    assert f5c, "fixture drifted: no F5c building to test the exclusion on"
    got = [r for r in plan.records
           if r["cls"] == "roof_victim"
           and r["building_i"] in {b.i for b in f5c}]
    assert not got
    assert any(k.startswith("roof_victim:collapse_level(F5c")
              for k in plan.refused)


def test_25d_roof_victim_is_higher_than_roof_but_still_capped():
    """`roof_victim_max_h_m` is deliberately above `roof_max_h_m` (section
    5b2: on the real city, `roof` alone leaves nothing to place) — but it is
    still a real cap, not none."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F1"][0]
    doc = {"fire": {"top": rec["origin"], "roof": False},
          "events": [], "masses": {}, "notes": [], "top_z": rec["H"]}
    cfg = {"roof_max_h_m": 20.0, "roof_victim_max_h_m": 20.0,
          "shares": {"roof_victim": 0.9, "evacuee": 0.05, "onlooker": 0.05,
                     "at_car": 0.0, "window": 0.0, "roof": 0.0,
                     "casualty_apron": 0.0, "roof_debris": 0.0}}
    capped = fp.plan_people(dump, man, seed=7, sidecars={rec["cell"]: doc},
                            heading_deg=45.0, cfg=cfg)
    got_capped = [r for r in capped.records
                  if r["cls"] == "roof_victim" and r["building_i"] == rec["i"]]
    # ...and raising ONLY `roof_victim_max_h_m` (not `roof`'s own cap)
    # recovers this building for `roof_victim` alone.
    cfg2 = dict(cfg)
    cfg2["roof_victim_max_h_m"] = 200.0
    raised = fp.plan_people(dump, man, seed=7, sidecars={rec["cell"]: doc},
                            heading_deg=45.0, cfg=cfg2)
    got_raised = [r for r in raised.records
                  if r["cls"] == "roof_victim" and r["building_i"] == rec["i"]]
    if float(rec["H"]) > 20.0:
        assert not got_capped
        assert got_raised


def test_25e_roof_victim_sits_on_deck_z_not_top_z():
    """`gac_fire`'s own account, section 5b: the bbox top is the parapet
    coping, not the deck."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] in ("F1", "F3")][0]
    doc = {"fire": {"top": 0, "roof": False, "deck_z": 9.4},
          "events": [], "masses": {}, "notes": [], "top_z": rec["H"]}
    plan = fp.plan_people(dump, man, seed=7, sidecars={rec["cell"]: doc},
                          heading_deg=45.0,
                          cfg={"shares": {"roof_victim": 0.9, "evacuee": 0.05,
                                          "onlooker": 0.05, "at_car": 0.0,
                                          "window": 0.0, "roof": 0.0,
                                          "casualty_apron": 0.0,
                                          "roof_debris": 0.0}})
    got = [r for r in plan.records
           if r["cls"] == "roof_victim" and r["building_i"] == rec["i"]]
    assert got
    for r in got:
        assert abs(r["z"] - 9.4) < 1e-9, r
        assert r["z"] != rec["H"]
        assert r["z_mode"] == "deck"
    ok, detail = _checks(plan)["roof_on_deck"]
    assert ok, detail


def test_25f_roof_victim_keeps_clear_of_the_buildings_own_smoke():
    """"Keep them away from the smoke tho" (user). A seat planted right where
    the edge geometry would otherwise land a figure must push it away by at
    least `roof_victim_min_smoke_dist_m`."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F1"][0]
    cfg = {"roof_victim_max_h_m": 200.0,
          "shares": {"roof_victim": 0.9, "evacuee": 0.05, "onlooker": 0.05,
                     "at_car": 0.0, "window": 0.0, "roof": 0.0,
                     "casualty_apron": 0.0, "roof_debris": 0.0}}
    clean = fp.plan_people(dump, man, seed=7,
                           sidecars={rec["cell"]: {
                               "fire": {"top": rec["origin"], "roof": False},
                               "events": [], "masses": {}, "notes": [],
                               "top_z": rec["H"], "seats": {"interior": [],
                                                            "roof": []}}},
                           heading_deg=45.0, cfg=cfg)
    got = [r for r in clean.records
           if r["cls"] == "roof_victim" and r["building_i"] == rec["i"]]
    assert got, "fixture drifted: no roof_victim landed on the F1 building"
    ok, detail = _checks(clean)["roof_victim_clear_of_smoke"]
    assert ok, detail
    # CONTROL: a seat planted AT the first record's own position, in the
    # SAME bake-local frame `_clear_of_smoke` reads. Rotating a world point
    # back into that frame is `_rot` by the NEGATIVE cell yaw.
    b = {bb.i: bb for bb in clean.solver.buildings}[rec["i"]]
    dx, dy = got[0]["x"] - b.x, got[0]["y"] - b.y
    lx, ly = fp._rot(dx, dy, -b.yaw)
    seeded_doc = {"fire": {"top": rec["origin"], "roof": False},
                 "events": [], "masses": {}, "notes": [], "top_z": rec["H"],
                 "seats": {"interior": [{"x": lx, "y": ly, "z": 0.0}],
                          "roof": []}}
    seeded = fp.plan_people(dump, man, seed=7,
                            sidecars={rec["cell"]: seeded_doc},
                            heading_deg=45.0, cfg=cfg)
    got2 = [r for r in seeded.records
            if r["cls"] == "roof_victim" and r["building_i"] == rec["i"]]
    for r in got2:
        d = math.hypot(r["x"] - got[0]["x"], r["y"] - got[0]["y"])
        assert d >= float(cfg.get(
            "roof_victim_min_smoke_dist_m",
            fp.DEFAULTS["roof_victim_min_smoke_dist_m"])) - 1e-6, r
    assert seeded.refused.get("roof_victim_near_smoke", 0) > 0


def test_25g_roof_victim_groups_are_never_a_singleton():
    """The same lone-figure argument `roof` (5b) and the street classes
    already carry: a group of one is withdrawn."""
    plan = _plan()
    counts = {}
    for r in plan.records:
        if r["cls"] == "roof_victim":
            counts[r["group"]] = counts.get(r["group"], 0) + 1
    assert counts
    assert all(n >= 2 for n in counts.values()), counts


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
        # ...and the surface it lies on is SUBSTANTIAL — re-anchored closer
        # to the wall 2026-09-01 (user: burial figures were "too far away
        # from the actual debris") — but still short of the full wall-line
        # depth (`apron_surface_z`'s (1-t)^1.3 falloff at the band's own
        # LOWER bound is the least-thinned point this class ever draws).
        max_frac = (1.0 - band[0]) ** 1.3
        assert r["surface_z"] <= r["debris_depth_m"] * max_frac + 1e-6, r


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
    # STREET CLASSES ON: the "share goes back to the street classes" half of
    # this test is the knob-on behaviour (with the 2026-08-31 default the
    # give-back goes to the remaining damage-tied classes instead — section
    # 12's own tests).
    plan = fp.plan_people(dump, man, seed=7, heading_deg=45.0,
                          cfg={"street_classes": True})
    assert not [r for r in plan.records if r["cls"] == "casualty_apron"]
    assert not [r for r in plan.records if r["cls"] == "roof_debris"]
    assert "casualty_apron" in plan.degraded
    assert "roof_debris" in plan.degraded
    base = _street_plan()
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
    # `no_protrusion` is the LEAN pose's own up-front refusal (see
    # `test_18`'s identical note) — `lean_window_inset_m` drives it now.
    deep = _plan(cfg={"lean_window_inset_m": 3.0})
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


def test_36b_the_window_figure_leans_out_hips_at_the_sill():
    """The user's own framing (2026-08-31): "lower half inside the building
    and upper half outside." The retired `sill_sit` variant had this
    backwards — seated on the sill with the pelvis recessed, the dangling
    calves were the visible protruding part while the torso and head stayed
    in the dark recess. The single surviving `lean_out` geometry is checked
    against BOTH halves, not just the aggregate protrusion number: the feet
    (and with them the legs) sit AT OR BELOW the sill — inside the wall —
    and the reported protrusion is strictly positive past the facade."""
    prot = fp.window_protrusion_m(fp.DEFAULTS["lean_window_inset_m"])
    assert prot >= fp.MIN_PROTRUSION_M, prot
    # `window_use_lean_pose` is the DEFAULT since 2026-09-01 finalization
    # (the pelvis-hinged pose the bench row settled on); `_plan()` alone
    # exercises it now (`test_63b` covers the explicit fallback instead).
    plan = _plan()
    got = [r for r in plan.records if r["cls"] == "window"]
    assert got
    for r in got:
        assert r["variant"] == "lean_out", r
        assert r["z_mode"] == "floor" and r["pose"] == "lean_window", r
        assert r["seat"] is None, r
        # HIPS AT THE SILL: the feet sit exactly `_HIP_H * H` below it.
        # (both `z` and `sill_z` are rounded to 3 dp in the record.)
        assert abs((r["sill_z"] - r["z"])
                   - fp._HIP_H * fp.NOMINAL_HEIGHT_M) < 2e-3, r
        # LOWER HALF INSIDE: the feet never sit above the sill they lean out
        # of — there is no "standing on the sill" reading.
        assert r["z"] <= r["sill_z"] + 1e-6, r
        # UPPER HALF OUTSIDE: the torso/head break the facade plane.
        assert r["protrusion_m"] > 0.0, r


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
    fr = _local_wall_frame(rec, side)
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
    plan = fp.plan_people(dump, man, seed=7, heading_deg=45.0,
                          cfg={"street_classes": True})
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
    i0 = man["records"][0]["i"]
    tall = dict(man["records"][0], H=302.0, n_storeys=101, level="F4",
                W=60.0, D=60.0)
    man = dict(man, records=[tall] + list(man["records"][1:]))
    # Keep the DUMP's own copy of this house in sync: `_manifest_matches_
    # dump` re-verifies every manifest record against the dump's geometry at
    # its `i` (test_manifest_records_are_reverified_against_the_dump), and a
    # synthetic skyscraper that exists only in the manifest would otherwise
    # (correctly) be treated as a stale/reindexed record and skipped.
    dump = dict(dump, placements=[
        dict(p, W=60.0, D=60.0, H=302.0) if p.get("i") == i0 else p
        for p in dump["placements"]])
    plan = fp.plan_people(dump, man, seed=7, heading_deg=45.0,
                          cfg={"street_classes": True})
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
                                "max_wall_dist_m": 1e6,
                                "street_classes": True})
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
    plan = _street_plan()
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
    it. `sit_edge` is also a `people._SEAT_PLACED_POSES` member, so a MALE
    rig additionally carries `people._seated_asset_dz`
    (`_MALE_SEATED_DZ_M`) — `_placement_no_ctx` REUSES that function (not a
    copy) so this path cannot drift from `people._human_placement`'s own
    correction; asserted against whichever rig the plan actually drew,
    male or not, rather than assuming one."""
    import scene_generator as sg
    plan = _street_plan()
    rec = _one(plan, "evacuee", pose="sit_edge")
    p, _sk = fp.to_placements([rec])
    p = p[0]
    dz = sg.pose_z_offset(rec["usd"], "sit_edge", fp.NOMINAL_HEIGHT_M)
    dz += ppl._seated_asset_dz(rec["usd"], "sit_edge")
    assert dz < -0.5, dz                       # it really is a big drop
    assert abs(p["z_m"] - (rec["z"] + dz)) < 1e-9
    assert p["pose"] == "sit_edge"


def test_42b_a_male_kerb_sitter_is_not_authored_0_15m_high():
    """FOLLOW-UP, 2026-08-31 (`tools/people_float_audit.py`, part 2):
    `_placement_no_ctx` — the path the LIVE launcher actually uses, since
    `urban_fire_city_launch_script.place_people` calls
    `to_placements(recs)` with no `ctx=` — used to have no equivalent of
    `people._seated_asset_dz` at all, so every MALE rig
    (`rp_eric`/`rp_manuel`/`rp_nathan`/`rp_dennis`) in `sit_edge` was
    authored `_MALE_SEATED_DZ_M` (0.15 m) ABOVE its kerb. Force one and
    check the correction actually fires without a `ctx`."""
    plan = _street_plan()
    rec = dict(_one(plan, "evacuee", pose="sit_edge"))
    rec["usd"] = fp.RIGGED_HUMANS[2]        # rp_eric — a `_MALE_RIGS` member
    assert os.path.basename(rec["usd"]).lower().startswith(ppl._MALE_RIGS)
    p, _sk = fp.to_placements([rec])        # no ctx=
    p = p[0]
    female = dict(rec, usd=fp.RIGGED_HUMANS[0])   # rp_carla — not male
    assert not os.path.basename(female["usd"]).lower().startswith(
        ppl._MALE_RIGS)
    pf, _sk = fp.to_placements([female])
    pf = pf[0]
    assert abs((pf["z_m"] - p["z_m"]) - abs(ppl._MALE_SEATED_DZ_M)) < 1e-9, (
        pf["z_m"], p["z_m"])


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
    # ITEM 2 (2026-08-31): the body is SUNK into the heap by `sink_m`
    # (tornado_people's own sink mechanic), so the record's support
    # surface is the windrow top MINUS the sink, never the bare top.
    assert abs(rec["z"] - (rec["surface_z"] - rec["sink_m"])) < 1.5e-3
    assert 0.0 <= rec["sunk_frac"] <= fp.DEFAULTS["sink_frac"][1] + 1e-6


def test_44_a_window_leaner_and_a_roof_figure_convert():
    # window/roof are in the DEFAULT census; the LEAN/new-pose geometry this
    # test checks is not (bench-v2 pose rejection — `test_63b`/`test_61b`
    # cover the default `idle` fallback), so it is turned on explicitly.
    plan = _new_pose_plan()
    import scene_generator as sg

    w = _one(plan, "window")
    p = fp.to_placements([w])[0][0]
    # THE FEET, NOT THE SILL: `z` is the support surface (the feet), sunk
    # `_HIP_H * H` below the sill so the hips land there. (both are rounded
    # to 3 dp in the record.)
    assert abs(w["z"] - (w["sill_z"] - fp._HIP_H * fp.NOMINAL_HEIGHT_M)) \
        < 2e-3
    dz = sg.pose_z_offset(w["usd"], w["pose"], fp.NOMINAL_HEIGHT_M)
    assert abs(p["z_m"] - (w["z"] + dz)) < 1e-9
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
    plan = _street_plan()
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
    good = _one(_street_plan(), "evacuee", pose="idle", rigged=True)
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
    plan = _street_plan()
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



def test_50_to_placements_accepts_the_on_disk_plan_dict():
    """THE 2026-08-31 LAUNCH-KILLING CRASH, pinned on the REAL file.

    `urban_fire_city_launch_script.place_people` did
    `to_placements(json.load(open(FC_PEOPLE_JSON)))`, and `to_placements`
    fell back to `list(plan_or_records)` — which on a dict yields its KEYS.
    `_convertible` then got the string "census" and raised

        AttributeError: 'str' object has no attribute 'get'

    out of `run()`, killing the 500 m city launch after every bake and every
    Flow emitter was already up (no people AND no captures).

    Fed the EXACT on-disk shape, not a reconstruction: `_plans/
    fire_people_final.json` is what the launcher is pointed at, and its
    records live under `people` — there is no `records` key in it at all,
    which is why "read doc['records']" is not the fix either.
    """
    path = os.path.join(_SCENE_GEN, "_plans", "fire_people_final.json")
    if not os.path.exists(path):
        return                       # not every checkout carries the plan
    doc = json.load(open(path))
    assert "records" not in doc, sorted(doc)
    assert isinstance(doc.get("people"), list) and doc["people"]

    from_dict, sk_dict = fp.to_placements(doc)
    from_list, sk_list = fp.to_placements(doc["people"])
    assert from_dict == from_list
    assert sk_dict == sk_list
    assert len(from_dict) == len(doc["people"]), (len(from_dict),
                                                  len(doc["people"]))
    # and the placements are authorable: the RenderPeople exports are in
    # centimetres, so a wrong scale here is 1.8 cm humans.
    assert {p["scale"] for p in from_dict} == {fp.HUMAN_SCALE} == {0.01}
    assert {p["category"] for p in from_dict} == {"human"}


def test_50_a_to_placements_names_the_shapes_it_takes():
    """A shape it cannot read raises a TypeError that says so — never an
    AttributeError three frames deeper in `_convertible`."""
    rec = {"usd": fp.RIGGED_HUMANS[0], "x": 0.0, "y": 0.0, "z": 0.0,
           "yaw_deg": 0.0, "pose": "idle", "prone": False, "rigged": True,
           "id": 0}
    # the three shapes that ARE accepted
    for doc in ({"people": [rec]}, {"records": [rec]}, [rec]):
        got, _sk = fp.to_placements(doc)
        assert len(got) == 1, doc

    for bad in ({"meta": {}, "census": {}}, "some/path.json"):
        try:
            fp.to_placements(bad)
        except TypeError as exc:
            assert "to_placements" in str(exc), exc
        else:
            raise AssertionError("no TypeError for %r" % (bad,))


# ===========================================================================
# 9. Manifest <-> dump reverification, the burial distance cap, and the
#    sidecar-completeness report — the 2026-08-31 "final manifest/bakes
#    land later today" re-run readiness work.
# ===========================================================================
def test_51_manifest_records_are_reverified_against_the_dump_not_just_indexed():
    """`i` existing in the dump is necessary but NOT sufficient. MEASURED,
    2026-08-31: `_plans/fire_city_500m.json` (20 records) against a FRESH
    `_plans/fc_dump_500.json` (87 houses) has every `i` present, yet 17 of
    the 20 now name a DIFFERENT house (up to 309 m away, several a
    different size entirely) — a re-laid-out BSP city reassigns which house
    lands at a given index. Both failure modes -- an index absent outright,
    and one present but moved/resized -- are skipped, COUNTED by reason,
    and never crash the planner."""
    dump, man = _inputs()
    recs = list(man["records"])
    drifted = dict(recs[0], x=recs[0]["x"] + 500.0, y=recs[0]["y"] + 500.0)
    missing = dict(recs[1], i=999999)
    man2 = dict(man, records=[drifted, missing] + recs[2:])

    plan = fp.plan_people(dump, man2, seed=7, heading_deg=45.0)  # must not raise

    skipped = plan.solver.skipped_records
    reasons = {s["reason"] for s in skipped}
    assert reasons == {"geometry_drift", "index_not_in_dump"}, skipped
    assert plan.meta["manifest_records_skipped"] == 2, plan.meta
    assert plan.meta["n_manifest_records"] == len(recs)
    assert plan.meta["n_burning"] == len(recs) - 2
    kept_i = {b.i for b in plan.solver.buildings}
    assert drifted["i"] not in kept_i
    assert 999999 not in kept_i


def test_51b_a_fully_stale_manifest_degrades_to_an_empty_plan_not_a_crash():
    """Every `i` reindexed beyond anything the dump has -> zero surviving
    buildings, zero records, and every rule trivially PASSES on 0 checked --
    which is exactly why `fire_people_dry_run.main()` fails the RUN (not the
    plan) when `n_burning` is 0. Never an exception."""
    dump, man = _inputs()
    stale = dict(man, records=[dict(r, i=int(r["i"]) + 1000000)
                               for r in man["records"]])
    plan = fp.plan_people(dump, stale, seed=7, heading_deg=45.0)
    assert plan.meta["n_burning"] == 0
    assert plan.records == []
    assert plan.meta["manifest_records_skipped"] == len(man["records"])
    for _name, ok, detail in fp.check_rules(plan):
        assert ok, detail


def test_52_burial_distance_is_capped_like_the_street_crowd():
    """`apron_run_m` (`apron_spread * H`, no cap of its own) can throw a
    burial figure further from the wall than `FC_PEOPLE_MAX_DIST_M` (the
    launcher's cull, default 120 m) will keep on stage -- the same failure
    `test_39b` pins for the street classes. `max_wall_dist_m` (60 m) must
    hold here too, with the SAME control discipline: the gate is tested on
    AND off."""
    dump, man = _inputs()
    i0 = man["records"][0]["i"]
    # `man["records"][0]` is already the synthetic city's one F5c building
    # (`synth_inputs`'s level histogram); make it tall and wide enough that
    # its run-out (`0.34 * H`) clears `max_wall_dist_m` outright.
    # H bumped 250 -> 400 with `apron_band`'s 2026-09-01 re-anchor (0.72-1.00
    # -> 0.40-0.65, "too far away from the actual debris"): the run-out is
    # now sampled at a LOWER max fraction of itself, so a shorter building's
    # worst-case draw no longer clears `max_wall_dist_m` the way it used to.
    huge = dict(man["records"][0], level="F5c", H=400.0, n_storeys=90,
                W=40.0, D=40.0, sides=["S"])
    man2 = dict(man, records=[huge] + man["records"][1:])
    dump2 = dict(dump, placements=[
        dict(p, W=40.0, D=40.0, H=400.0) if p.get("i") == i0 else p
        for p in dump["placements"]])
    assert fp.apron_run_m(huge, fp.DEFAULTS) > fp.DEFAULTS["max_wall_dist_m"]

    cfg = {"total": 20, "shares": {"evacuee": 0.0, "onlooker": 0.0,
                                   "at_car": 0.0, "window": 0.0,
                                   "roof": 0.0, "casualty_apron": 0.5,
                                   "roof_debris": 0.5}}
    plan = fp.plan_people(dump2, man2, seed=3, cfg=cfg, heading_deg=45.0)
    burial = [r for r in plan.records
              if r["cls"] in ("casualty_apron", "roof_debris")]
    for r in burial:
        assert r["d_wall_m"] <= fp.DEFAULTS["max_wall_dist_m"] + 0.01, r
    assert plan.refused.get("too_far_from_building", 0) > 0, plan.refused
    ok, detail = _checks(plan)["crowd_belongs_to_its_building"]
    assert ok, detail

    # CONTROL: lift the cap and the SAME run-out DOES throw burial figures
    # out there, so the cap is what is holding them in.
    loose = fp.plan_people(dump2, man2, seed=3,
                           cfg=dict(cfg, max_wall_dist_m=1e6),
                           heading_deg=45.0)
    loose_burial = [r for r in loose.records
                    if r["cls"] in ("casualty_apron", "roof_debris")]
    assert loose_burial
    assert max(r["d_wall_m"] for r in loose_burial) > 60.0


def test_53_a_synthetic_record_round_trips_the_contract():
    """A HAND-BUILT record -- never solved by `plan_people` -- still
    converts to exactly what `urban_fire_city_launch_script.place_people`
    hands `scene_generator.apply_placements`: `usd/x_m/y_m/z_m/yaw_deg/
    roll_deg/pitch_deg/scale/category="human"/axis_up(/pose)`, read from
    under the `people` key of a loaded plan dict."""
    rec = {"id": 0, "cls": "evacuee", "group": 1,
           "usd": fp.RIGGED_HUMANS[0], "rigged": True,
           "x": 12.5, "y": -30.25, "z": 0.13, "yaw_deg": 200.0,
           "pose": "idle", "prone": False, "seat": None, "z_mode": "ground",
           "surface": "sidewalk", "alive": True, "needs_bench": False,
           "building_i": 7, "building_cell": "/World/stage/generated/house_1_7",
           "building_level": "F4", "building_sides": ["S"],
           "d_wall_m": 18.4, "standoff_m": 15.8, "collapse_zone_frac": 0.2,
           "upwind_cos": 0.5, "reason": "synthetic"}
    doc = {"meta": {"seed": 0}, "people": [rec]}

    placements, skipped = fp.to_placements(doc)
    assert skipped == {}, skipped
    assert len(placements) == 1
    p = placements[0]
    assert _CONTRACT <= set(p), (_CONTRACT - set(p), p)
    assert set(p) - _CONTRACT <= {"pose"}, set(p) - _CONTRACT
    assert p["usd"] == rec["usd"]
    assert p["x_m"] == rec["x"] and p["y_m"] == rec["y"]
    assert p["yaw_deg"] == rec["yaw_deg"] + fp.HUMAN_YAW_OFFSET_DEG
    assert p["category"] == "human"
    assert p["scale"] == fp.HUMAN_SCALE
    assert p["axis_up"] == fp.HUMAN_AXIS_UP
    assert p["roll_deg"] == 0.0 and p["pitch_deg"] == 0.0
    assert isinstance(p["z_m"], float)
    assert p["pose"] == "idle"


def test_54_max_wall_dist_stays_under_the_launchers_cull_horizon():
    """`FC_PEOPLE_MAX_DIST_M` (`urban_fire_city_launch_script.py`:
    `PEOPLE_MAX_DIST_M = float(_env("FC_PEOPLE_MAX_DIST_M", "120"))`,
    measured over the burning FOOTPRINT, not the centre — same datum
    `dist_to_obb` computes here) must stay ABOVE this module's own
    `max_wall_dist_m`, or the launcher's guard becomes the thing that drops
    a figure this solver placed on purpose. The launcher builds a
    `SimulationApp` at import, so its default is read from source TEXT
    rather than imported (see the module's own account of why a second Kit
    app in one process segfaults)."""
    import re
    launcher = os.path.normpath(os.path.join(
        _SCENE_GEN, "..", "simulation", "isaac-sim", "launch_scripts",
        "urban_fire_city_launch_script.py"))
    with open(launcher) as fh:
        src = fh.read()
    m = re.search(
        r'PEOPLE_MAX_DIST_M\s*=\s*float\(_env\("FC_PEOPLE_MAX_DIST_M",\s*'
        r'"([0-9.]+)"\)\)', src)
    assert m, "FC_PEOPLE_MAX_DIST_M's default moved or was renamed"
    launcher_default = float(m.group(1))
    assert fp.DEFAULTS["max_wall_dist_m"] < launcher_default, (
        fp.DEFAULTS["max_wall_dist_m"], launcher_default)


def test_55_sidecar_report_flags_the_estimated_deck_and_derived_windows():
    """No sidecar at all: `sidecar_report()` says so for every building,
    `deck_z_source` is `'estimated'`, no side has a measured opening, and an
    eligible window class lands in `needs_bench_classes`, never
    `bench_free_classes` — the report's "safe class" signal has to be
    computed, not asserted."""
    plan = _plan()          # no sidecars
    rows = fp.sidecar_reports(plan)
    assert rows
    saw_window_eligible = False
    for r in rows:
        assert r["has_sidecar"] is False, r
        assert r["deck_z_source"] == "estimated", r
        assert r["window_sides_measured"] == [], r
        assert not any(c.startswith("window(") for c in r["bench_free_classes"])
        if r["window_eligible"]:
            saw_window_eligible = True
            assert any(c.startswith("window(derived)")
                      for c in r["needs_bench_classes"]), r
    assert saw_window_eligible


def test_56_sidecar_report_reflects_a_real_match():
    """The companion case: a building WITH a matching sidecar carrying real
    ops reports `has_sidecar=True`, the measured side, and — when the
    sidecar's own `fire.deck_z` is set — `deck_z_source == 'sidecar'`
    (reproducing `test_20`'s fixture, since that is the one place this
    module's own tests hand it a `deck_z` a real bake never actually
    supplies — see `SIDECAR_FIELD_USE`)."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F1"][0]
    side = rec["sides"][0]
    period = rec["H"] / rec["n_storeys"]
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
    plan = fp.plan_people(dump, man, seed=7, sidecars={rec["cell"]: doc},
                          heading_deg=45.0)
    rows = {r["building_i"]: r for r in fp.sidecar_reports(plan)}
    row = rows[rec["i"]]
    assert row["has_sidecar"] is True
    assert row["deck_z_source"] == "sidecar"
    assert side in row["window_sides_measured"]
    if row["window_eligible"]:
        assert any(c.startswith("window(measured)")
                  for c in row["bench_free_classes"])


def test_57_deck_z_now_persists_and_roof_figures_use_it():
    """FOLLOW-UP, 2026-08-31: `fire_bake.sidecar()` no longer drops `deck_z`
    -- it writes the measured value into BOTH `fire.deck_z` and
    `masses.main.deck_z` (numeric for a gac/dtc bake, e.g. 69.2755 for
    SM_Building_19; `None` for a kit bake, which never measured one).
    `deck_z()` must read `fire.deck_z` FIRST: a roof group solved against a
    sidecar carrying it is seated exactly there, `deck_source == "sidecar"`
    -- not on the parapet estimate `test_55` pins for the no-sidecar case."""
    dump, man = _inputs()
    rec = [r for r in man["records"]
           if r["level"] in fp.DEFAULTS["roof_ok_levels"]][0]
    doc = {"fire": {"top": rec["origin"], "roof": False,
                    "n_storeys": rec["n_storeys"], "deck_z": 69.2755},
           "events": [], "masses": {"main": {"deck_z": None}}, "notes": [],
           "top_z": rec["H"]}
    plan = fp.plan_people(
        dump, man, seed=7, sidecars={rec["cell"]: doc}, heading_deg=45.0,
        cfg={"shares": {"roof": 0.9, "evacuee": 0.05, "onlooker": 0.05,
                        "at_car": 0.0, "window": 0.0, "casualty_apron": 0.0,
                        "roof_debris": 0.0},
             "roof_max_h_m": 1e6})
    got = [r for r in plan.records
           if r["cls"] == "roof" and r["building_i"] == rec["i"]]
    assert got, "the sidecar building got no roof figure"
    for r in got:
        assert r["deck_source"] == "sidecar", r
        # `_pass_roof` rounds `z` to 3 dp when it authors the record (every
        # other class does the same), so compare against that same rounding
        # rather than the raw sidecar float.
        assert abs(r["z"] - round(69.2755, 3)) < 1e-6, r

    # `masses.main.deck_z` is the SECOND priority (only when `fire.deck_z`
    # is absent), and a kit bake's explicit `None` there falls through to
    # the estimate rather than being mistaken for a real zero.
    z2, src2 = fp.deck_z(rec, {"fire": {}, "masses": {"main": {"deck_z": 41.1}}})
    assert (z2, src2) == (41.1, "sidecar_mass")
    z3, src3 = fp.deck_z(rec, {"fire": {}, "masses": {"main": {"deck_z": None}}})
    assert src3 == "estimated"


def test_58_deck_z_prefers_sidecar_top_z_over_a_stale_manifest_h():
    """FOLLOW-UP, 2026-08-31 (`tools/people_float_audit.py`, run against the
    live `city_138` 39-manifest): the "brownstone mini blocks" mechanic
    (`tools/fire_city_force_blocks.py`) re-skins a cell's `style` to a kit
    archetype without touching the manifest record's `x/y/W/D/H` — those
    still describe whatever building the fire was originally solved against
    at that cell, and can be METRES taller than the archetype actually
    baked there. Measured live: `i=261/264/269` carried `rec["H"]`
    21.8-22.3 m against a real baked `kit_brownstone` height
    (`doc["top_z"]`) of 17.1-18.8 m — every `roof`/`roof_victim` figure on
    those three decks was floating 2.4-4.1 m above the actual roof line,
    silently, under `deck_source == "estimated"`.

    `deck_z()` must prefer `doc["top_z"]` (measured off the geometry
    actually standing in the scene) the moment a sidecar exists at all, even
    with neither `fire.deck_z` nor `masses.main.deck_z` set — falling all
    the way to `rec["H"]` only when there is NO sidecar (`test_55`'s own
    case, still pinned above)."""
    dump, man = _inputs()
    rec = [r for r in man["records"]
           if r["level"] in fp.DEFAULTS["roof_ok_levels"]][0]
    real_h = rec["H"] - 4.0        # a real archetype 4 m shorter than H
    stale = dict(rec)
    stale["H"] = rec["H"]          # the manifest's own (stale) claim
    doc = {"fire": {"top": rec["origin"], "roof": False,
                    "n_storeys": rec["n_storeys"], "deck_z": None},
           "events": [], "masses": {"main": {"deck_z": None}}, "notes": [],
           "top_z": real_h}
    z, src = fp.deck_z(stale, doc, fp.DEFAULTS["parapet_est_m"])
    assert src == "sidecar_top_z", src
    assert abs(z - (real_h - fp.DEFAULTS["parapet_est_m"])) < 1e-9
    # It must NOT be the stale H-based estimate.
    assert abs(z - (rec["H"] - fp.DEFAULTS["parapet_est_m"])) > 1.0

    # No sidecar at all still falls all the way to the manifest H estimate.
    z2, src2 = fp.deck_z(stale, None, fp.DEFAULTS["parapet_est_m"])
    assert src2 == "estimated"
    assert abs(z2 - (rec["H"] - fp.DEFAULTS["parapet_est_m"])) < 1e-9

    # And the actual `roof` pass seats the group on the sidecar-derived
    # deck, not the manifest one.
    plan = fp.plan_people(
        dump, man, seed=7, sidecars={rec["cell"]: doc}, heading_deg=45.0,
        cfg={"shares": {"roof": 0.9, "evacuee": 0.05, "onlooker": 0.05,
                        "at_car": 0.0, "window": 0.0, "casualty_apron": 0.0,
                        "roof_debris": 0.0},
             "roof_max_h_m": 1e6})
    got = [r for r in plan.records
           if r["cls"] == "roof" and r["building_i"] == rec["i"]]
    assert got, "the sidecar building got no roof figure"
    for r in got:
        assert r["deck_source"] == "sidecar_top_z", r
        assert abs(r["z"] - round(real_h - fp.DEFAULTS["parapet_est_m"], 3)) \
            < 1e-6, r


def test_59_load_sidecars_disambiguates_a_reused_cell_by_level():
    """FOLLOW-UP, 2026-08-31 (`tools/people_float_audit.py`, cross-checking
    the live `city_138` manifest against its own sidecar directory): 10
    cells in that bake set carry TWO sidecar files — the same `tag`, the
    same `city.cell`, different `level` — left over from iterating a
    building's severity (`kit_apartment_tall_F3_o4_SNW_s758.json` AND
    `..._F5_o4_SNW_s758.json` for `house_39_257`, among others).
    `load_sidecars` used to key ONLY by the bare `cell`/`tag`, so
    `sorted(os.listdir(...))` visiting `F3` before `F5` meant the F5 doc
    silently overwrote the F3 one — `i=257`'s manifest record is `F3`, but
    every lookup by its cell returned the F5 sidecar (wrong `fire.top`,
    wrong band, wrong openings), with no error or count anywhere.

    Reproduced with two on-disk sidecars sharing a cell and a tag, real
    files (`load_sidecars` reads from disk, not from a dict a test hands
    it) so the exact collision is exercised end to end."""
    import tempfile
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F3"][0]
    other = dict(rec, level="F5")
    tag = "collide0"
    with tempfile.TemporaryDirectory() as d:
        for lvl, name, r in (("F3", "a_F3.json", rec), ("F5", "b_F5.json", other)):
            doc = {"tag": tag, "level": lvl,
                  "city": {"cell": rec["cell"]},
                  "fire": {"top": 2 if lvl == "F3" else 9, "roof": lvl != "F3",
                           "n_storeys": r["n_storeys"]},
                  "masses": {}, "notes": [], "top_z": r["H"]}
            with open(os.path.join(d, name), "w") as fh:
                json.dump(doc, fh)
        sc = fp.load_sidecars(d)
        # The bare keys collide (last-alphabetical-wins, unchanged
        # behaviour) — but the level-qualified ones do not.
        assert sc[rec["cell"]]["level"] == "F5"
        assert sc[tag]["level"] == "F5"
        assert sc[(rec["cell"], "F3")]["level"] == "F3"
        assert sc[(rec["cell"], "F5")]["level"] == "F5"
        assert sc[(tag, "F3")]["level"] == "F3"

        plan = fp.plan_people(dump, man, seed=7, sidecars=sc, heading_deg=45.0)
        rows = {r["building_i"]: r for r in fp.sidecar_reports(plan)}
        row = rows[rec["i"]]
        assert row["has_sidecar"] is True
        # The F3 building must be resolved against the F3 doc (fire.top=2),
        # not the colliding F5 one (fire.top=9, which would have refused
        # window eligibility outright for a building this manifest says is
        # only 3 storeys past its own fire).
        b = [x for x in plan.solver.buildings if x.i == rec["i"]][0]
        assert b.band_top == 2, b.band_top


# ===========================================================================
# 12. THE 2026-08-31 REVIEW ROUND — the street-class knob, standing-only
#     roofs, lean_window + the flame clearance, REAL burial cover, and
#     `interior_trapped`. Same two disciplines as everything above: every
#     gate is tested with its own control, and every copied table is checked
#     against its original.
# ===========================================================================
def test_60_street_classes_are_off_by_default():
    """ITEM 3: "No people on the ground that aren't like part of the
    damage." The default census carries NO evacuee/onlooker/at_car — every
    figure is damage-tied — and the knob-ON control proves the classes are
    behind a knob, not deleted."""
    plan = _plan()
    assert not [r for r in plan.records if r["cls"] in fp.STREET_CLASSES]
    for c in fp.STREET_CLASSES:
        assert c in plan.degraded, c
    assert plan.meta["street_classes"] is False
    for c in fp.STREET_CLASSES:
        assert plan.meta["budget"][c] == 0, plan.meta["budget"]
    ok, detail = _checks(plan)["street_classes_off_means_no_street_figures"]
    assert ok, detail
    # ...and the head count did not silently collapse: the damage-tied
    # classes absorbed the street share (the give-back sinks are the
    # eligible classes, not the hard-coded evacuee/onlooker pair).
    assert len(plan.records) >= 20, len(plan.records)
    # CONTROL: the knob-on run DOES place street figures.
    on = _street_plan()
    assert [r for r in on.records if r["cls"] in fp.STREET_CLASSES]
    assert on.meta["street_classes"] is True


def test_60b_fp_street_classes_env_overrides_the_default():
    """`FP_STREET_CLASSES` (env) beats the cfg when set — the same
    precedence every env-driven knob in this pipeline uses."""
    old = os.environ.pop(fp.STREET_CLASSES_ENV, None)
    try:
        assert fp.resolve_cfg()["street_classes"] is False
        os.environ[fp.STREET_CLASSES_ENV] = "1"
        assert fp.resolve_cfg()["street_classes"] is True
        # env wins over a programmatic False too — it is the human's word
        assert fp.resolve_cfg({"street_classes": False})["street_classes"] \
            is True
        os.environ[fp.STREET_CLASSES_ENV] = "0"
        assert fp.resolve_cfg({"street_classes": True})["street_classes"] \
            is False
    finally:
        if old is None:
            os.environ.pop(fp.STREET_CLASSES_ENV, None)
        else:
            os.environ[fp.STREET_CLASSES_ENV] = old


def test_61_roof_and_roof_victim_poses_are_standing_only():
    """ITEM 4 ("The roof people need to just be standing. not like bent"):
    both roof classes draw ONLY `stand_calm`/`wave_help` — no crouch, no
    sit_ground, no `stand_slump` forward pitch — and both poses need a
    skeleton, so a posed static can never ship on a roof.

    Tested with `roof_use_new_pose=True` explicitly — the bench-v2 rejection
    (2026-08-31) took this OFF the default path (see `test_61b`), but the
    restricted pose SET itself is still real, still-shipped code and still
    has to be pinned so a re-tune cannot silently readmit `crouch`/
    `sit_ground`.
    """
    for cls in ("roof", "roof_victim"):
        names = {p for p, _w in fp._CLASS_POSES[cls]}
        assert names <= {"stand_calm", "wave_help"}, (cls, names)
    plan = _new_pose_plan(window=False)
    got = [r for r in plan.records if r["cls"] in ("roof", "roof_victim")]
    assert got
    for r in got:
        assert r["pose"] in ("stand_calm", "wave_help"), r
        assert r["rigged"] is True, r


def test_61b_roof_poses_default_to_idle_pending_a_render():
    """ITEM 1 FALLBACK, 2026-08-31 bench-v2 REJECTION ("still on the edge in
    unnatural poses" / "Just spawn people only"): with no override,
    `roof`/`roof_victim` ship on `idle` — the one pose this pipeline has
    always rendered correctly — not `stand_calm`/`wave_help`."""
    assert fp.DEFAULTS["roof_use_new_pose"] is False
    plan = _plan()
    got = [r for r in plan.records if r["cls"] in ("roof", "roof_victim")]
    assert got
    for r in got:
        assert r["pose"] == "idle", r


def test_61c_no_roof_class_on_a_breached_roof_building():
    """SECOND bench-v2 round, quoted: "we can't have people on roofs where
    the roof has collapsed" / "the partial collapse has a roof collapse but
    people are still on the ledge there." Neither `roof` NOR `roof_victim`
    may carry a single figure on the synthetic fixture's F5c origin building
    — same building `test_25c` already pins for `roof_victim` alone — and
    the NEW gate rule (`no_roof_figure_on_a_breached_roof`) must both pass on
    a clean plan and be capable of catching a violation if the eligibility
    gate is ever bypassed."""
    plan = _plan()
    f5c = {b.i for b in plan.solver.buildings if b.level == "F5c"}
    assert f5c, "fixture drifted: no F5c building to test the exclusion on"
    got = [r for r in plan.records
          if r["cls"] in ("roof", "roof_victim") and r["building_i"] in f5c]
    assert not got, got
    ok, detail = _checks(plan)["no_roof_figure_on_a_breached_roof"]
    assert ok, detail
    assert detail["n_checked"] > 0        # the rule actually ran on records

    # THE GATE CANNOT PASS BY PLACING NOTHING — same discipline
    # `tornado_people`'s own gate-off controls use (`test_21`/`test_22`
    # there): forge a `roof` record directly ONTO the F5c building and
    # confirm the rule catches it rather than passing vacuously.
    b_i = next(iter(f5c))
    forged = dict(plan.records[0]) if plan.records else {
        "id": 0, "cls": "roof", "x": 0.0, "y": 0.0}
    forged["cls"] = "roof"
    forged["building_i"] = b_i
    plan.records.append(forged)
    ok2, detail2 = _checks(plan)["no_roof_figure_on_a_breached_roof"]
    assert not ok2, "the forged F5c roof record was not caught"
    plan.records.pop()


def test_62_roof_groups_sit_well_off_the_parapet():
    """ITEM 4 ("don't need to be on the ledge. Centre is fine"): the edge
    band starts at 2.5 m (roof) / 2.0 m (roof_victim) in from the coping,
    up from the 1.0-1.2 m that hugged it."""
    assert fp.DEFAULTS["roof_edge_band_m"][0] >= 2.5
    assert fp.DEFAULTS["roof_victim_edge_band_m"][0] >= 2.0
    plan = _plan()
    got = [r for r in plan.records if r["cls"] in ("roof", "roof_victim")]
    assert got
    for r in got:
        assert r["roof_clear_m"] >= 2.0 - 1e-6, r
    ok, detail = _checks(plan)["roof_clear_of_parapet"]
    assert ok, detail


def test_63_window_pose_is_lean_window_with_a_measured_protrusion():
    """ITEM 5 first half: the window figure hip-hinges (`lean_window`), and
    `window_protrusion_m` reads the pose's own MEASURED head fraction
    (`_LEAN_WINDOW_HEAD_FRAC_H`), so a re-tuned pose that forgets the
    constant fails a test rather than a render.

    `window_use_lean_pose` is the DEFAULT since the 2026-09-01
    finalization — the 8-variant bench pose row settled the mapping
    question and `lean_window` is now PELVIS-hinged, not spine-chain.
    """
    plan = _plan()
    got = [r for r in plan.records if r["cls"] == "window"]
    assert got
    for r in got:
        assert r["pose"] == "lean_window", r
        assert r["protrusion_m"] >= fp.MIN_PROTRUSION_M, r
    inset = fp.DEFAULTS["lean_window_inset_m"]
    want = fp._LEAN_WINDOW_HEAD_FRAC_H * fp.NOMINAL_HEIGHT_M - inset
    assert abs(fp.window_protrusion_m(inset) - want) < 1e-9
    # ...and the pose really exists in the pose table, PELVIS hinge and
    # all — `pelvis`, not `spine_01/02/03` (the retired mechanism).
    import scene_generator as sg
    assert "lean_window" in sg._HUMAN_POSES
    pelvis_axis, pelvis_deg = sg._HUMAN_POSES["lean_window"]["pelvis"]
    assert pelvis_axis == (1.0, 0.0, 0.0), pelvis_axis
    assert 20.0 <= pelvis_deg <= 40.0, pelvis_deg   # "~25-35 deg", moderate
    assert "spine_01" not in sg._HUMAN_POSES["lean_window"]


def test_63b_window_pose_defaults_to_lean_window_now_finalized():
    """FINALIZED 2026-09-01: the bench-v2 fallback (`idle`, `test_63c`) was
    the interim state while `lean_window` was unproven; the 8-variant pose
    row settled the mapping question and `lean_window` (now pelvis-hinged)
    is the DEFAULT window pose again."""
    assert fp.DEFAULTS["window_use_lean_pose"] is True
    plan = _plan()
    got = [r for r in plan.records if r["cls"] == "window"]
    assert got
    for r in got:
        assert r["pose"] == "lean_window", r
        assert r["variant"] == "lean_out", r
        assert r["protrusion_m"] >= fp.MIN_PROTRUSION_M, r


def test_63c_window_idle_fallback_still_works_when_asked():
    """The fallback geometry (`idle`, standing recessed behind the sill) is
    NOT deleted — `window_use_lean_pose=False` still ships it, always
    flagged `needs_bench`, for a future round that wants it back."""
    plan = _plan(cfg={"window_use_lean_pose": False})
    got = [r for r in plan.records if r["cls"] == "window"]
    assert got
    for r in got:
        assert r["pose"] == "idle", r
        assert r["variant"] == "standing_at_opening", r
        assert r["needs_bench"] is True, r
        assert r["protrusion_m"] >= fp.MIN_PROTRUSION_M, r


def test_64_window_figures_keep_clear_of_the_flame():
    """ITEM 5 second half: "they can't be right next to the open flame."
    TWO LAYERS, both tested: (1) an elevation with any flame-bearing event
    is AVOIDED OUTRIGHT while a flame-free candidate side exists
    (`_side_has_flame` pre-filter — "conservative: prefer non-flame
    elevations entirely"), and (2) when every candidate side burns, the
    per-opening `_clear_of_flame` check refuses any opening within
    `window_flame_clear_m` of a flame, by name. The control (clearance 0)
    places figures right against the flame, so the gate is measuring the
    flame and not something else."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F1"][0]
    W, Dd = float(rec["W"]), float(rec["D"])

    def _row(side):
        # flame events every ~4 m along one wall, in the bake's own frame.
        if side in ("S", "N"):
            y = -Dd / 2.0 if side == "S" else Dd / 2.0
            return [{"intensity": 0.9, "side": side,
                     "ops": [{"side": side, "e": {"x": u, "y": y}}]}
                    for u in [-W / 2.0 + 1.0 + 4.0 * k
                              for k in range(int(W // 4) + 1)]]
        x = -W / 2.0 if side == "W" else W / 2.0
        return [{"intensity": 0.9, "side": side,
                 "ops": [{"side": side, "e": {"x": x, "y": v}}]}
                for v in [-Dd / 2.0 + 1.0 + 4.0 * k
                          for k in range(int(Dd // 4) + 1)]]

    shares = {"window": 0.9, "evacuee": 0.0, "onlooker": 0.0, "at_car": 0.0,
              "roof": 0.05, "roof_victim": 0.05, "casualty_apron": 0.0,
              "roof_debris": 0.0, "interior_trapped": 0.0}
    man2 = dict(man, records=[dict(r, sides=["S"]) if r["i"] == rec["i"]
                              else r for r in man["records"]])

    # (1) flame on S only: the pass prefers the flame-free E/W outright.
    doc_s = {"fire": {"top": 0, "roof": False}, "notes": [],
             "top_z": rec["H"], "events": _row("S"), "masses": {}}
    strict = fp.plan_people(dump, man2, seed=7, heading_deg=45.0,
                            sidecars={rec["cell"]: doc_s},
                            cfg={"shares": shares})
    mine = [r for r in strict.records
            if r["cls"] == "window" and r["building_i"] == rec["i"]]
    assert mine, "fixture drifted: no window figures on the flame building"
    assert not [r for r in mine if r["side"] == "S"], mine
    ok, detail = _checks(strict)["windows_clear_of_flame"]
    assert ok, detail

    # (2) every candidate side burns (S + its adjacents E/W): the
    # per-opening clearance is the only gate left, and it refuses BY NAME.
    doc_all = dict(doc_s, events=_row("S") + _row("E") + _row("W"))
    walled = fp.plan_people(dump, man2, seed=7, heading_deg=45.0,
                            sidecars={rec["cell"]: doc_all},
                            cfg={"shares": shares})
    assert walled.refused.get("too_close_to_flame", 0) > 0, walled.refused
    # ...and with the walls flame-covered every ~4 m, nothing survives the
    # 6 m clearance on this building at all.
    assert not [r for r in walled.records
                if r["cls"] == "window" and r["building_i"] == rec["i"]]
    ok, detail = _checks(walled)["windows_clear_of_flame"]
    assert ok, detail

    # CONTROL: clearance off, and the same all-sides fixture places figures
    # on burning elevations again — the gate was doing the work.
    loose = fp.plan_people(dump, man2, seed=7, heading_deg=45.0,
                           sidecars={rec["cell"]: doc_all},
                           cfg={"shares": shares,
                                "window_flame_clear_m": 0.0})
    assert loose.refused.get("too_close_to_flame", 0) == 0
    assert [r for r in loose.records
            if r["cls"] == "window" and r["building_i"] == rec["i"]]


def test_65_buried_reach_is_in_the_mix_and_lays_face_up():
    """ITEM 1's fourth pose: `buried_reach` is drawn by the burial classes,
    laid FACE-UP (roll -90) — and `people.LYING_POSES` carries the same roll,
    so the ctx conversion path cannot coin-flip it onto its face (the
    2026-08-31 latent bug: an absent key fell through to the legacy
    `90 if (x+y)>=0 else -90`)."""
    assert fp.LYING_ROLL["buried_reach"] == -90.0
    assert dict(fp._LYING_POSES)["buried_reach"] > 0.0
    assert ppl.LYING_POSES["buried_reach"] == -90.0
    assert "buried_reach" not in fp.LYING_SPIN     # no long-axis spin
    import scene_generator as sg
    assert "buried_reach" in sg._HUMAN_POSES
    br = None
    for seed in range(10):
        plan = _plan(seed=seed)
        got = [r for r in plan.records if r.get("pose") == "buried_reach"]
        if got:
            br = got[0]
            break
    assert br is not None, "no seed in 0..9 ever drew buried_reach"
    assert br["prone"] is True and br["roll_deg"] == -90.0
    p = fp.to_placements([br])[0][0]
    assert p["roll_deg"] == -90.0 and p["pitch_deg"] == 0.0
    # ...and the ctx path agrees now that `people.LYING_POSES` has the key.
    pc = fp.to_placements([br], ctx=_fake_ctx())[0][0]
    assert pc["roll_deg"] == -90.0, pc


def test_66_burial_cover_is_real_pieces_with_a_real_fraction():
    """ITEM 2 first half, per the coordinator's "look at tornado code"
    instruction: a partly-buried figure's `covered_frac` comes from PIECES
    actually authored over the body (`plan.covering`, `tornado_people.
    _cover_piece` specs), never from a drawn number — and the sink is
    bounded by `sink_frac` exactly as `tornado_people` bounds its own."""
    from disaster import tornado_people as tp
    plan = _plan()
    burials = [r for r in plan.records
               if r["cls"] in ("casualty_apron", "roof_debris")]
    assert burials
    for name in ("burial_cover_is_authored", "burial_sink_bounded",
                 "nothing_fully_buried"):
        ok, detail = _checks(plan)[name]
        assert ok, (name, detail)
    covered = [r for r in burials if r["occlusion"] != "none"]
    assert covered
    for r in covered:
        assert r["boards"] > 0, r
        assert 0.0 < r["covered_frac"] <= fp.MAX_COVERED_FRAC + 1e-9, r
    assert plan.covering
    stock = {s[0] for s in fp._FIRE_COVER_STOCK}
    for sp in plan.covering:
        assert sp["for"] == "fire_burial", sp
        assert sp["class"] in stock, sp
    # The sink is real and its record fields agree with each other.
    for r in burials:
        body_depth = 2.0 * fp._BODY_HALF_DEPTH_M
        assert abs(r["z"] - (r["surface_z"] - r["sink_m"])) < 1.5e-3, r
        assert abs(r["sunk_frac"] - r["sink_m"] / body_depth) < 5e-3, r
        assert r["sunk_frac"] <= fp.DEFAULTS["sink_frac"][1] + 1e-6, r
    # The flattened OCCLUSION table and the span table are the same claim.
    for name, frac, _w in fp.OCCLUSION:
        span = fp._FIRE_OCCLUSION_SPANS[name]
        if span not in (None, "lateral"):
            assert abs((span[1] - span[0]) - frac) < 1e-9, name
    # ...and every span the scene can draw fits under the cap untrimmed —
    # `tornado_people.test_20`'s pairing, carried over.
    for name, spans in fp._FIRE_OCCLUSION_SPANS.items():
        if spans in (None, "lateral"):
            continue
        assert tp._trim_spans((spans,), fp.MAX_COVERED_FRAC) == [spans], name


def _interior_doc(rec, origin=2):
    """A minimal sidecar that makes `rec` `interior_trapped`-eligible: a
    per-storey levels array on the main mass and the fire's origin storey."""
    n = max(2, int(rec["n_storeys"]))
    period = float(rec["H"]) / n
    return {"fire": {"top": min(n - 1, origin + 2), "roof": False,
                     "origin": origin, "sides": list(rec["sides"])},
            "masses": {"main": {"levels": [i * period for i in range(n)],
                                "W": rec["W"], "D": rec["D"]}},
            "events": [], "notes": [], "top_z": rec["H"]}


def test_67_interior_trapped_stands_inside_on_the_surviving_slab():
    """ITEM 2 second half: "in the building itself visible through the
    broken parts" — INSIDE its own footprint (the class is
    aerial-exempt), on the slab BELOW the fire's origin (`fire_collapse`'s
    own "nothing below the origin, ever" invariant), set back from the
    broken edge, with a sightline a >= 25 deg oblique camera can use."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F5c"][0]
    doc = _interior_doc(rec)
    shares = {"interior_trapped": 0.9, "evacuee": 0.0, "onlooker": 0.0,
              "at_car": 0.0, "window": 0.05, "roof": 0.0, "roof_victim": 0.0,
              "casualty_apron": 0.05, "roof_debris": 0.0}
    plan = fp.plan_people(dump, man, seed=7, heading_deg=45.0,
                          sidecars={rec["cell"]: doc},
                          cfg={"shares": shares})
    got = [r for r in plan.records if r["cls"] == "interior_trapped"]
    assert got
    n = max(2, int(rec["n_storeys"]))
    period = float(rec["H"]) / n
    # BENCH-V4 LEDGE-STANDER FIX, 2026-09-01: `passed_out` stays at the base
    # slab (origin - 1 = storey 1); `conscious` (standing) goes ONE STOREY
    # LOWER (origin - 2 = storey 0) so it never reads as a ledge-stander at
    # the broken roof line. Two different expected z/storey now, not one.
    want_z = {"passed_out": (2 - 1) * period, "conscious": (2 - 2) * period}
    want_storey = {"passed_out": 1, "conscious": 0}
    lo_sb, hi_sb = fp.DEFAULTS["interior_setback_m"]
    seen_variants = set()
    for r in got:
        seen_variants.add(r["variant"])
        assert r["z_mode"] == "slab", r
        assert abs(r["z"] - want_z[r["variant"]]) < 1e-3, (r["z"], r["variant"])
        assert r["storey"] == want_storey[r["variant"]], r
        assert fp.point_in_obb(r["x"], r["y"], rec["x"], rec["y"],
                               rec["W"], rec["D"],
                               rec.get("yaw_deg", 0.0), margin=0.0), r
        assert lo_sb - 1e-6 <= r["setback_m"] <= hi_sb + 1e-6, r
        assert r["side"] in rec["sides"], r
        assert r["needs_bench"] is True, r
        assert r["variant"] in ("conscious", "passed_out"), r
        if r["variant"] == "conscious":
            assert r["pose"] in {p for p, _w in
                                 fp._INTERIOR_CONSCIOUS_POSES}, r
            assert r["prone"] is False
        else:
            assert r["pose"] in {p for p, _w in
                                 fp._INTERIOR_UNCONSCIOUS_POSES}, r
            assert r["prone"] is True
            assert r["roll_deg"] == fp.LYING_ROLL[r["pose"]], r
    # Both variants actually got exercised — a 0.9 share over several draws
    # should hit both branches of the conscious coin flip.
    assert seen_variants == {"conscious", "passed_out"}, seen_variants
    for name in ("interior_trapped_is_inside_its_building",
                 "interior_trapped_on_its_own_slab",
                 "interior_trapped_setback_respected",
                 "interior_trapped_has_a_sightline",
                 "interior_trapped_eligible_building"):
        ok, detail = _checks(plan)[name]
        assert ok, (name, detail)
    # Both variants appear over a handful of seeds — the split is real.
    variants = set()
    for seed in range(8):
        p2 = fp.plan_people(dump, man, seed=seed, heading_deg=45.0,
                            sidecars={rec["cell"]: doc},
                            cfg={"shares": shares})
        variants |= {r["variant"] for r in p2.records
                     if r["cls"] == "interior_trapped"}
        if variants == {"conscious", "passed_out"}:
            break
    assert variants == {"conscious", "passed_out"}, variants


def test_68_interior_trapped_needs_a_real_sidecar_and_degrades():
    """The class places a figure on a SPECIFIC storey of a SPECIFIC
    building — too particular a claim to synthesise. No sidecar, no class,
    and the degradation is recorded rather than silently absorbed."""
    plan = _plan()                                # synth: no sidecars at all
    assert not [r for r in plan.records if r["cls"] == "interior_trapped"]
    assert "interior_trapped" in plan.degraded
    # ...and an F5-not-F5c building is refused even WITH a sidecar: a shell
    # that lost its floors inward on every side has no slab-behind-a-hole.
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F5c"][0]
    man2 = dict(man, records=[dict(r, level="F5") if r["i"] == rec["i"]
                              else r for r in man["records"]])
    doc = _interior_doc(rec)
    p2 = fp.plan_people(dump, man2, seed=7, heading_deg=45.0,
                        sidecars={rec["cell"]: doc})
    assert not [r for r in p2.records if r["cls"] == "interior_trapped"
                and r["building_i"] == rec["i"]]


def test_69_the_rule_gate_grew_with_the_review_and_passes():
    """The review asked for ~22+ rules; the gate now runs 33. The new names
    are present, and the default plan clears every one of them."""
    plan = _plan()
    checks = fp.check_rules(plan)
    assert len(checks) >= 22, len(checks)
    names = {n for n, _ok, _d in checks}
    for want in ("windows_clear_of_flame", "burial_cover_is_authored",
                 "burial_sink_bounded",
                 "street_classes_off_means_no_street_figures",
                 "roof_on_deck", "roof_clear_of_parapet",
                 "no_roof_figure_on_a_breached_roof",
                 "interior_trapped_is_inside_its_building",
                 "interior_trapped_on_its_own_slab",
                 "interior_trapped_has_a_sightline",
                 "burial_covering_contacts_the_body",
                 "window_standing_fallback_is_recessed"):
        assert want in names, want
    assert all(ok for _n, ok, _d in checks), [
        n for n, ok, _d in checks if not ok]


def test_71_burial_covering_pieces_contact_their_own_body():
    """BENCH-V3 REJECTION, 2026-08-31: "figures FLOAT ... covering pieces
    render as ... boxes hovering ABOVE the bodies, touching nothing."
    `_cover_burial`'s `top_z` is clamped to `lift * 2.2` above the body's
    own support surface (`_BODY_RISE`'s per-pose bands, calibrated in
    `tornado_people`'s own context, do not always agree with THIS module's
    `lying_lift` — measured on the synthetic fixture's own `lying_curled_l`
    record: a 0.41 m gap before the clamp existed). Every piece traces back
    to the ONE body it covers via `over_record_id`, so this is a per-figure
    proof, not "some piece exists somewhere."""
    plan = _plan()
    burials = [r for r in plan.records
              if r["cls"] in ("casualty_apron", "roof_debris")
              and float(r.get("covered_frac", 0.0)) > 1e-6]
    assert burials, "fixture drifted: no covered burial figure to test"
    by_rec = {}
    for sp in plan.covering:
        by_rec.setdefault(sp.get("over_record_id"), []).append(sp)
    tol = fp.DEFAULTS["cover_contact_tol_m"]
    for r in burials:
        pieces = by_rec.get(r["id"])
        assert pieces, r
        body_surface = r["z"] + r["lift_m"]
        gaps = [abs(float(sp.get("cover_top_z", sp["z"])) - body_surface)
               for sp in pieces]
        assert min(gaps) <= tol, (r["id"], r["pose"], body_surface, gaps)
    ok, detail = _checks(plan)["burial_covering_contacts_the_body"]
    assert ok, detail

    # CONTROL: an absurdly small clamp (well under any real lift) must
    # start failing the gate — proves the check can actually catch a gap,
    # the same discipline `tornado_people`'s own gate-off controls use.
    tight = _plan(cfg={"cover_contact_tol_m": 0.001})
    ok2, detail2 = _checks(tight)["burial_covering_contacts_the_body"]
    assert not ok2, "an unreasonably tight tolerance was not caught"


def test_72_window_standing_fallback_is_recessed_with_a_spandrel():
    """BENCH-V3 REJECTION: "she stands FULLY VISIBLE head-to-toe in FRONT of
    the glass, zero leg occlusion." The `idle` fallback (`window_use_lean_
    pose=False` — no longer the default since the 2026-09-01 finalization,
    but still real code) sits `window_stand_inset_m` (a real body depth)
    behind the facade, and only ever draws an opening whose sill clears
    `window_min_spandrel_m` above its own floor — a floor-to-ceiling
    glazing bay is skipped (`no_spandrel`), never placed anyway."""
    plan = _plan(cfg={"window_use_lean_pose": False})
    got = [r for r in plan.records if r["cls"] == "window"]
    assert got
    for r in got:
        assert r["variant"] == "standing_at_opening", r
        assert r["inset_m"] >= fp.DEFAULTS["window_stand_inset_m"] - 1e-6, r
        spandrel = r["sill_z"] - r["floor_z"]
        assert spandrel >= fp.DEFAULTS["window_min_spandrel_m"] - 1e-6, \
            (r, spandrel)
    ok, detail = _checks(plan)["window_standing_fallback_is_recessed"]
    assert ok, detail

    # CONTROL: a spandrel floor above every real sill height refuses the
    # whole class rather than placing a bare-legged stander anyway. The
    # spandrel check only runs on the fallback (`not use_lean`) branch — see
    # `_pass_window`'s own comment — so this control needs it explicit too.
    starved = _plan(cfg={"window_min_spandrel_m": 50.0,
                         "window_use_lean_pose": False})
    assert not [r for r in starved.records if r["cls"] == "window"]
    assert starved.refused.get("no_spandrel", 0) > 0


def test_73_window_never_in_the_top_storeys():
    """2026-09-01 user follow-up on the bench-v4 ledge-stander complaint:
    "Don't do any window leans on the top 2-3 stories always below." Applied
    on every building, every level."""
    plan = _plan()
    got = [r for r in plan.records if r["cls"] == "window"]
    assert got
    by_i = {b.i: b for b in plan.solver.buildings}
    top_excl = fp.DEFAULTS["window_top_storeys_excluded"]
    for r in got:
        b = by_i[r["building_i"]]
        assert r["storey"] <= b.n_storeys - 1 - top_excl, (r, b.n_storeys)
    ok, detail = _checks(plan)["window_below_the_top_storeys"]
    assert ok, detail

    # CONTROL: excluding the whole building empties the class rather than
    # bending the rule, and is counted distinctly from "no storey above the
    # fire" (`top_storeys_excluded`, not `no_storey_above_fire`).
    starved = _plan(cfg={"window_top_storeys_excluded": 999})
    assert not [r for r in starved.records if r["cls"] == "window"]
    assert starved.refused.get("top_storeys_excluded", 0) > 0


def test_74_no_standing_interior_figure_at_the_break():
    """Bench-v4 ledge-stander fix, 2026-09-01: a `conscious` interior_
    trapped figure is never on the top surviving storey (`origin - 1`,
    right at the broken roof line) — always at least one storey lower.
    `passed_out` is exempt, and both variants are re-checked against the
    gate rule directly."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F5c"][0]
    doc = _interior_doc(rec)
    shares = {"interior_trapped": 0.9, "evacuee": 0.0, "onlooker": 0.0,
              "at_car": 0.0, "window": 0.0, "roof": 0.0, "roof_victim": 0.0,
              "casualty_apron": 0.0, "roof_debris": 0.0}
    plan = fp.plan_people(dump, man, seed=7, heading_deg=45.0,
                          sidecars={rec["cell"]: doc},
                          cfg={"shares": shares})
    conscious = [r for r in plan.records
                if r["cls"] == "interior_trapped"
                and r["variant"] == "conscious"]
    assert conscious, "fixture drifted: no conscious record to test"
    base_storey = 1                    # origin 2 -> base slab storey 1
    for r in conscious:
        assert r["storey"] < base_storey, r
    ok, detail = _checks(plan)["no_standing_interior_figure_at_the_break"]
    assert ok, detail

    # CONTROL: a forged conscious record AT the base storey must be caught.
    forged = dict(conscious[0], storey=base_storey)
    plan.records.append(forged)
    ok2, detail2 = _checks(plan)["no_standing_interior_figure_at_the_break"]
    assert not ok2, "a forged ledge-standing record was not caught"
    plan.records.pop()


def test_70_local_roof_z_degrades_to_none_never_raises():
    """ITEM 4's railed-roof fix: `local_roof_z` samples the REAL bake mesh
    when the file is there and `pxr` imports, and hands back None — never an
    exception, never a guess — when either is missing. The caller's contract
    is to fall through to the scalar `deck_z`."""
    assert fp.local_roof_z("/nonexistent/bake.usd", 0.0, 0.0, 10.0) is None
    assert fp.bake_usd_path(None) is None
    assert fp.bake_usd_path({}) is None
    assert fp.bake_usd_path({"_sidecar_path": "/nonexistent/x.json"}) is None


# ===========================================================================
# 13. Mechanisms the review round above exercises only END TO END — the
#     fallback tuple `_roof_seat_z` returns with no bake, `_roof_interior_
#     point`'s own e_min floor and its centred-not-hugging SPREAD (a
#     distribution claim, not just a floor), the `author_burial_cover: False`
#     escape hatch, `interior_trapped`'s two eligibility primitives pinned
#     directly rather than only through a full solve, the pose tables swept
#     for a stray name, and the rule gate's own name hygiene.
# ===========================================================================
def test_71_roof_seat_z_falls_back_to_the_global_deck_scalar_with_no_bake():
    """`_roof_seat_z` prefers a local mesh sample (item 4) but every building
    in this host-side suite has no `.usd` on disk — so the path every OTHER
    test in this file actually exercises is the fallback: `(b.deck_z,
    b.deck_source, needs_bench)`, unchanged from what `deck_z()` already
    computed. Pinned directly on a building with no sidecar at all, and one
    whose sidecar DOES supply a measured `deck_z` (so it is bench-free) but
    still carries no `_sidecar_path` for `bake_usd_path` to find — the local
    sample never even gets attempted."""
    dump, man = _inputs()
    rec = [r for r in man["records"]
           if r["level"] in fp.DEFAULTS["roof_ok_levels"]][0]
    cfg = fp.resolve_cfg(None)

    b_no_doc = fp._Building(rec, cfg, None)
    z, src, needs_bench = fp._roof_seat_z(b_no_doc, b_no_doc.x, b_no_doc.y)
    assert (z, src) == (b_no_doc.deck_z, b_no_doc.deck_source)
    assert src == "estimated"
    assert needs_bench is True                # "estimated" is not bench-free

    doc = {"fire": {"top": rec["origin"], "roof": False, "deck_z": 12.3},
          "events": [], "masses": {}, "notes": [], "top_z": rec["H"]}
    b_doc = fp._Building(rec, cfg, doc)
    z2, src2, needs_bench2 = fp._roof_seat_z(b_doc, b_doc.x, b_doc.y)
    assert (z2, src2) == (12.3, "sidecar")
    assert needs_bench2 is False


def test_72_roof_interior_point_never_lands_closer_than_e_min():
    """The hard floor, called directly and repeatedly rather than trusted
    from the handful of records one real plan happens to produce — a fake
    `sol` that never refuses on region/spacing isolates the geometry alone."""
    class _FakeSol(object):
        def in_region(self, x, y):
            return True
        def spaced(self, x, y):
            return True
    class _FakeB(object):
        W, D, x, y, yaw = 40.0, 30.0, 0.0, 0.0, 0.0

    cfg = fp.resolve_cfg(None)
    e_min = float(cfg["roof_edge_band_m"][0])
    sol, b, rng = _FakeSol(), _FakeB(), random.Random(1)
    clears = []
    for _ in range(400):
        found = fp._roof_interior_point(sol, b, cfg, "N", rng, e_min)
        assert found is not None
        x, y, clear = found
        assert clear >= e_min - 1e-9, (x, y, clear)
        clears.append(clear)
    assert len(clears) == 400


def test_73_roof_groups_are_centred_not_edge_hugging():
    """ITEM 4, verbatim: "don't need to be on the ledge. Centre is fine." The
    retired geometry inset a group `roof_edge_band_m` (2.5-6.0 m by default)
    from ONE edge and hugged it, so EVERY draw it ever produced fell inside
    that narrow band by construction. `test_62` already pins the floor on
    real records; this is the DISTRIBUTION claim `test_62` cannot make from a
    handful of placements — over many draws from `_roof_interior_point`
    itself, most land further from every wall than the old band's own upper
    bound, which a hugging geometry could never do."""
    class _FakeSol(object):
        def in_region(self, x, y):
            return True
        def spaced(self, x, y):
            return True
    class _FakeB(object):
        W, D, x, y, yaw = 40.0, 30.0, 0.0, 0.0, 0.0

    cfg = fp.resolve_cfg(None)
    e_min = float(cfg["roof_edge_band_m"][0])
    old_hi = float(cfg["roof_edge_band_m"][1])
    sol, b, rng = _FakeSol(), _FakeB(), random.Random(2)
    clears = []
    for _ in range(400):
        found = fp._roof_interior_point(sol, b, cfg, "N", rng, e_min)
        assert found is not None
        clears.append(found[2])
    beyond_old_band = sum(1 for c in clears if c > old_hi)
    assert beyond_old_band / len(clears) > 0.5, (
        beyond_old_band, len(clears), old_hi)
    assert sum(clears) / len(clears) > old_hi, sum(clears) / len(clears)


def test_74_cover_burial_off_is_a_metadata_only_escape_hatch():
    """`test_66` pins the `author_burial_cover: True` default (real pieces in
    `plan.covering`). The companion case: `False` is the OLD path — a caller
    that wants the class without paying for the extra debris authoring still
    gets a coherent record (`occlusion`/`covered_frac` are still drawn), but
    `boards` is always 0 and nothing lands in `plan.covering`."""
    dump, man = _inputs()
    cfg = {"total": 30, "author_burial_cover": False, "shares": {
        "evacuee": 0.0, "onlooker": 0.0, "at_car": 0.0, "window": 0.0,
        "roof": 0.0, "roof_victim": 0.0, "interior_trapped": 0.0,
        "casualty_apron": 0.5, "roof_debris": 0.5}}
    plan = fp.plan_people(dump, man, seed=7, cfg=cfg, heading_deg=45.0)
    burial = [r for r in plan.records
              if r["cls"] in ("casualty_apron", "roof_debris")]
    assert burial
    assert plan.covering == []
    assert all(r["boards"] == 0 for r in burial)
    # ...and the draw still produces some non-"none" patterns, so the field
    # is not vacuously true.
    assert any(r["occlusion"] != "none" for r in burial)


def test_75_interior_trapped_eligibility_needs_a_real_sidecar():
    """`test_68` pins "no sidecar at all" and "wrong level" as ineligible.
    `_interior_trapped_ok` (via `_slab_z`) actually needs THREE separate
    things and this isolates each: the level in `collapse_levels`, a sidecar
    carrying `masses.main.levels`, AND `fire.origin` — missing EITHER of the
    latter two alone (sidecar present but incomplete, the shape a partial
    bake would actually produce) is ineligible, never a crash."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F5c"][0]
    assert rec["sides"], "fixture drifted: the F5c building has no sides"
    cfg = fp.resolve_cfg(None)
    period = rec["H"] / rec["n_storeys"]
    levels = [i * period for i in range(rec["n_storeys"])]

    good = {"fire": {"top": rec["origin"], "roof": False,
                     "origin": rec["origin"]},
           "masses": {"main": {"levels": levels}}, "notes": [],
           "top_z": rec["H"]}
    assert fp._interior_trapped_ok(fp._Building(rec, cfg, good))

    no_levels = {"fire": {"top": rec["origin"], "roof": False,
                          "origin": rec["origin"]},
                "masses": {}, "notes": [], "top_z": rec["H"]}
    assert not fp._interior_trapped_ok(fp._Building(rec, cfg, no_levels))

    no_origin = {"fire": {"top": rec["origin"], "roof": False},
                "masses": {"main": {"levels": levels}}, "notes": [],
                "top_z": rec["H"]}
    assert not fp._interior_trapped_ok(fp._Building(rec, cfg, no_origin))

    assert not fp._interior_trapped_ok(fp._Building(rec, cfg, None))


def test_76_interior_sightline_ok_has_a_real_boundary():
    """`test_67` proves every PLACED record clears the sightline end to end;
    this pins the standalone predicate's own boundary directly. A
    conservative planar model: a camera at `min_deg` above the horizontal,
    grazing the top of one storey's headroom above the figure's own eye
    height, reaches `clear / tan(min_deg)` metres in. Two concrete setbacks
    straddling that exact boundary (period 3.0 m, eye height 1.4 m, `min_deg`
    25 deg -> boundary ~3.431 m)."""
    period, eye_h, min_deg = 3.0, 1.4, 25.0
    assert fp._interior_sightline_ok(3.4, period, min_deg, eye_h) is True
    assert fp._interior_sightline_ok(3.5, period, min_deg, eye_h) is False
    # CONTROL: a headroom so shallow the eye height itself barely clears it
    # refuses EVERY setback, including one at the doorway.
    assert fp._interior_sightline_ok(0.01, 1.5, min_deg, 1.46) is False


def test_77_the_new_poses_compose_and_every_table_only_references_real_poses():
    """`stand_calm`/`wave_help`/`lean_window`/`buried_reach` are the four new
    entries in `scene_generator._HUMAN_POSES` this review added; `test_63`/
    `test_65` already spot-check three of them. No real FK verification here
    (needs `pxr`) — just that the keys exist, and that EVERY pose name any
    table in this module can draw is one of them, the discipline the
    module's own top-of-file comment states ("Every name is a key of
    `scene_generator._HUMAN_POSES`"). Plain `import scene_generator` reads
    `_HUMAN_POSES` as a dict with no `pxr` needed at import time in this
    environment (verified: `test_42`/`test_44`/`test_63`/`test_65` already
    import it the same way)."""
    import scene_generator as sg
    for want in ("stand_calm", "wave_help", "lean_window", "buried_reach"):
        assert want in sg._HUMAN_POSES, want

    names = set(fp._STAND_POSES)
    for tbl in fp._CLASS_POSES.values():
        names |= {p for p, _w in tbl}
    names |= {p for p, _w in fp._LYING_POSES}
    names |= {p for p, _w in fp._INTERIOR_CONSCIOUS_POSES}
    names |= {p for p, _w in fp._INTERIOR_UNCONSCIOUS_POSES}
    names.add("lean_window")        # hardcoded in `_pass_window`, not drawn
                                     # from a weighted table (section 5a).
    missing = names - set(sg._HUMAN_POSES)
    assert not missing, missing
    assert not (names & set(ppl.BANNED_POSES)), names & set(ppl.BANNED_POSES)


def test_79_window_side_choice_prefers_a_measured_elevation_over_a_guess():
    """BENCH-V7 REJECTION, 2026-09-01: "the leaners [are] pasted flat on a
    BLANK BRICK wall (no windows anywhere in frame)." Reproduces the bug in
    miniature and offline: a building whose only VENTING side (E) is both
    flame-free AND has a real sidecar-measured opening, next to two ADJACENT
    sides (N, S) that are also flame-free but carry NO sidecar data at all
    (`_side_ops` empty — a "derived" grid there is an unverified guess, and
    on the real GAC tower asset that guessed grid pointed at a blank party
    wall). Before the fix, `_pass_window` drew `side` uniformly from every
    flame-free candidate (E, N, S) — occasionally landing a figure on the
    unmeasured guess even though a measured elevation was available. The fix
    intersects flame-free with MEASURED first; run across enough seeds, EVERY
    window figure this building produces must land on E, `sidecar_grid`, and
    never N or S, `derived`."""
    dump, man = _inputs()
    rec = [r for r in man["records"] if r["level"] == "F1"][0]
    side = "E"
    period = float(rec["H"]) / float(rec["n_storeys"])
    fr = _local_wall_frame(rec, side)
    ops = [{"fr": list(fr), "side": side, "storey": rec["origin"] + 1,
            "span": [u - 0.6, u + 0.6, (rec["origin"] + 1) * period + 1.05,
                     (rec["origin"] + 1) * period + 2.30]}
           for u in (3.0, 7.4, 11.8)]
    # No flame anywhere (intensity 0.0 on the one event housing the real
    # ops) — every side is flame-free, so ONLY the measured-vs-derived
    # preference is under test, not the flame-avoidance layer test_64
    # already covers.
    doc = {"fire": {"top": rec["origin"], "roof": False,
                    "n_storeys": rec["n_storeys"], "deck_z": rec["H"] - 0.8},
           "events": [{"side": side, "storey": rec["origin"] + 1,
                       "intensity": 0.0, "ops": ops}],
           "masses": {}, "notes": [], "top_z": rec["H"]}
    man2 = dict(man, records=[dict(r, sides=[side]) if r["i"] == rec["i"]
                              else r for r in man["records"]])
    shares = {"window": 0.9, "evacuee": 0.0, "onlooker": 0.0, "at_car": 0.0,
              "roof": 0.05, "roof_victim": 0.05, "casualty_apron": 0.0,
              "roof_debris": 0.0, "interior_trapped": 0.0}
    seen_sides = set()
    for seed in range(7, 27):
        plan = fp.plan_people(dump, man2, seed=seed, heading_deg=45.0,
                              sidecars={rec["cell"]: doc},
                              cfg={"shares": shares,
                                   "window_max_per_building": 40})
        mine = [r for r in plan.records
                if r["cls"] == "window" and r["building_i"] == rec["i"]]
        for r in mine:
            seen_sides.add(r["side"])
            assert r["side"] == "E", r
            assert r["openings_source"] == "sidecar_grid", r
    assert seen_sides == {"E"}, seen_sides


def test_80_window_figure_is_in_an_opening_gate_catches_a_frame_mismatch():
    """The coordinator's own ask this round: "compute each window figure's
    world position and confirm it lies within an ...opening rect of the
    PLACED building... That assertion becomes a gate rule ... so a frame
    mismatch can never render again." POSITIVE: every real window figure
    the default plan produces already passes (its `x`/`y` came from the same
    `openings_for_side`/`_face_point` chain the rule re-derives). NEGATIVE:
    a forged record whose `x`/`y` is shifted 5 m along the facade — the
    shape of an actual yaw/frame bug, where the placement lands outside
    every real opening's own span — is caught by name."""
    plan = _plan()
    got = [r for r in plan.records if r["cls"] == "window"]
    assert got, "fixture drifted: no window figures in the default plan"
    ok, detail = _checks(plan)["window_figure_is_in_an_opening"]
    assert ok, detail

    victim = dict(got[0])
    # Shift the world position 5 m along the wall's own u-axis (tangent to
    # the facade, 90 deg off the record's stamped outward-normal `yaw_deg`)
    # — well past `bay_pitch_m` (3.2 m default) mod-distance from any bay
    # centre (5.0 mod 3.2 = 1.8 m, outside every opening's own half-width +
    # tolerance both ways), so the point is no longer within ANY
    # recomputed opening's [u0, u1] on its own stamped side/storey.
    ux = math.cos(math.radians(victim["yaw_deg"] + 90.0))
    uy = math.sin(math.radians(victim["yaw_deg"] + 90.0))
    victim["x"] = round(victim["x"] + 5.0 * ux, 3)
    victim["y"] = round(victim["y"] + 5.0 * uy, 3)
    plan.records.append(victim)
    ok2, detail2 = _checks(plan)["window_figure_is_in_an_opening"]
    assert not ok2, "the forged off-opening window record was not caught"
    assert detail2["n_violations"] > 0, detail2


def test_78_check_rules_names_are_unique():
    """`test_69` pins the FLOOR (>= 22) via a set comprehension that would
    silently swallow a collision — two rules sharing one name would still
    pass that test with `len(checks)` unchanged. This checks the list itself,
    not the set it collapses to."""
    plan = _plan()
    rules = fp.check_rules(plan)
    names = [name for name, _ok, _d in rules]
    assert len(rules) >= 22, len(rules)
    assert len(set(names)) == len(names), names


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
    print("\nall fire_people rules pinned")
