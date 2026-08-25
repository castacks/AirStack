"""Stage C's population model — the parts that are wrong silently.

A target that lands in the wrong place still renders, still gets a label, and
still shows up in `targets.json`. Nothing raises. What breaks is the ground
truth, and a search run scored against a wrong ground truth is worse than one
not scored at all — so each of these is a claim `targets.py`'s docstring makes,
pinned:

1. The population is REPRODUCIBLE from the target seed, and INDEPENDENT of it
   in the other direction — re-rolling the people must not need a re-bake.
2. Every cohort ends up where its name says. Trapped victims are inside
   collapsed footprints; gathered survivors are clear of every facade.
3. The mix responds to occupancy and to what the disaster actually did, and a
   cohort with nowhere to go hands its share on rather than dropping it.
4. N does not move with severity. That is what makes a severity sweep
   comparable, and it is exactly the kind of thing a later "scale it by sev"
   edit would destroy.

Pure Python: `sample_targets` never touches USD, which is why this runs in
milliseconds on the host.
"""

import json
import os
import sys
import tempfile

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import targets as T                                              # noqa: E402
from disaster import levels                                      # noqa: E402

REGION = (400.0, 400.0)
LADDER = levels.level_names("earthquake")


def _config(severity=0.8, occupancy="day", seed=42, target_seed=None,
            **targets_over):
    """A minimal compiled-config shape — only what Stage C reads."""
    tgt = {
        "enabled": True,
        "count_per_km2": 200.0,
        "count_clamp": [4, 60],
        "seed": target_seed,
        "occupancy": occupancy,
        "owns_humans": True,
        "cohorts": {"inside_rubble": 0.35, "exit_ring": 0.20, "street": 0.15,
                    "open_space": 0.25, "rubble_edge": 0.05},
        "seat_frac": 0.45,
    }
    tgt.update(targets_over)
    return {
        "seed": seed,
        "layout": {"region_m": list(REGION)},
        "disaster": {"type": "earthquake", "severity": severity,
                     "field": {"kind": "uniform", "inside": 1.0}},
        "targets": tgt,
    }


def _survey(levels_=("pancaked", "partial_collapse", "cracked", "pristine"),
            n_per_level=6, spacing=45.0):
    """A grid of buildings at assorted damage rungs, plus two road corridors."""
    buildings = []
    i = 0
    for lvl in levels_:
        for _ in range(n_per_level):
            col, row = i % 8, i // 8
            buildings.append({
                "x": -180.0 + col * spacing, "y": -180.0 + row * spacing,
                "w": 20.0, "h": 20.0, "yaw": 0.0,
                "level": lvl, "level_i": LADDER.index(lvl),
                # Genuinely wrecked geometry, not just a damage label — see
                # `targets.mark_cut_geometry`. `cut=False` is the tilt-and-sink
                # stand-in, and `test_a_standin_is_not_a_place_to_be_trapped`
                # is the case that matters.
                "cut": True,
                "prim_path": f"/World/generated/building_{i}",
            })
            i += 1
    roads = [{"x0": -200.0, "y0": -6.0, "x1": 200.0, "y1": 6.0},
             {"x0": -6.0, "y0": -200.0, "x1": 6.0, "y1": 200.0}]
    # Upright cars parked along the east-west corridor, at assorted yaws so a
    # sampler that reads the footprint as axis-aligned is caught.
    vehicles = [{"x": -150.0 + i * 20.0, "y": 3.0, "w": 4.5, "h": 2.0,
                 "z": 1.5, "yaw": (i * 37) % 360,
                 "prim_path": f"/World/generated/car_{i}"} for i in range(12)]
    return {"region": REGION, "buildings": buildings, "roads": roads,
            "vehicles": vehicles, "ladder": LADDER}


def _sample(config=None, survey=None):
    config = config or _config()
    return T.sample_targets(survey or _survey(), config)


# -- 1. reproducible, and re-rollable -------------------------------------

def test_same_seed_same_people():
    assert _sample() == _sample()


def test_target_seed_re_rolls_the_people():
    a = _sample(_config(target_seed=1))
    b = _sample(_config(target_seed=2))
    assert [(v["x"], v["y"]) for v in a] != [(v["x"], v["y"]) for v in b]
    # …and the same target seed on a different scene seed still reproduces,
    # which is what "re-populate the SAME baked city" means: the city's seed
    # is not an input to who is in it.
    assert _sample(_config(seed=7, target_seed=1)) == a


def test_target_seed_defaults_off_the_scene_seed():
    assert T.rng_for({"seed": 42}).random() == T.rng_for(
        {"seed": 42, "targets": {"seed": 42 + T.SEED_OFFSET}}).random()


# -- 2. every cohort where its name says ----------------------------------

def _by_cohort(victims):
    out = {}
    for v in victims:
        out.setdefault(v["cohort"], []).append(v)
    return out


def test_every_victim_is_well_formed():
    for v in _sample():
        assert v["cohort"] in T.COHORTS
        assert v["pose"] in dict(T._POSES[v["cohort"]])
        assert v["visibility"] in dict(T._VISIBILITY[v["cohort"]])
        assert v["lying"] == (v["pose"] in T.LYING)
        assert 0.0 <= v["yaw_deg"] <= 360.0
    assert [v["id"] for v in _sample()] == list(range(len(_sample())))


def test_trapped_victims_are_inside_a_collapsed_building():
    sv = _survey()
    floor = LADDER.index("soft_storey")
    ruins = [b for b in sv["buildings"] if b["level_i"] >= floor]
    for v in _by_cohort(_sample(survey=sv)).get("inside_rubble", []):
        assert any(abs(v["x"] - b["x"]) <= b["w"] / 2.0 + 1e-6
                   and abs(v["y"] - b["y"]) <= b["h"] / 2.0 + 1e-6
                   for b in ruins), v


def test_trapped_victims_are_biased_to_the_edge():
    """The void-space finding: survivors come out at the collapse perimeter,
    not from the middle of the pile. A uniform draw would average ~0.5."""
    sv = _survey()
    fr = []
    for v in _by_cohort(_sample(_config(count_per_km2=400.0), sv)).get(
            "inside_rubble", []):
        b = min(sv["buildings"],
                key=lambda b: (b["x"] - v["x"]) ** 2 + (b["y"] - v["y"]) ** 2)
        fr.append(max(abs(v["x"] - b["x"]) / (b["w"] / 2.0),
                      abs(v["y"] - b["y"]) / (b["h"] / 2.0)))
    assert fr and sum(fr) / len(fr) > 0.6


def test_a_standin_is_not_a_place_to_be_trapped():
    """A building past `mesh_damage`'s budget keeps its damage LABEL and its
    intact geometry. Trapping someone in one writes a victim into the ground
    truth inside an undamaged tower — measured at 50-60 m up on
    urban_quake_live before `cut` existed."""
    sv = _survey()
    for b in sv["buildings"]:
        b["cut"] = False
    vs = _sample(survey=sv)
    assert not [v for v in vs if v["cohort"] in ("inside_rubble", "exit_ring",
                                                 "rubble_edge")]
    # The people still exist; they are outdoors.
    assert len(vs) >= T.target_count(sv, T.settings(_config())) - 2


# -- the vehicle stratum ---------------------------------------------------

def _in_vehicle_config(**over):
    cfg = _config(occupancy="commute", **over)
    cfg["targets"]["cohorts"] = {"in_vehicle": 1.0}
    return cfg


def test_victims_in_vehicles_are_in_a_seat():
    """Inside the car's footprint — in the CAR's frame, since it is parked at a
    yaw — and at seat height rather than on the road or on the roof."""
    sv = _survey()
    # 12 cars, more people than seats — the rest go outdoors through the usual
    # shortfall path, which is why this filters rather than asserting on all.
    vs = _by_cohort(_sample(_in_vehicle_config(), sv)).get("in_vehicle", [])
    assert len(vs) >= 8
    for v in vs:
        car = next(c for c in sv["vehicles"] if c["prim_path"] == v["vehicle"])
        assert T._inside(car, v["x"], v["y"]), v
        assert v["seat_z"] == pytest.approx(0.45 * car["z"])
        assert v["pose"] == "seated" and not v["lying"]


def test_a_toppled_car_is_not_a_seat():
    """`disaster.cars_toppled_fraction` turns cars over on every seed, and a
    car on its roof is not somewhere to put a person."""
    placements = [
        {"category": "car", "x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0,
         "roll_deg": 0.0, "prim_path": "/upright"},
        {"category": "car", "x_m": 10.0, "y_m": 0.0, "yaw_deg": 0.0,
         "roll_deg": 92.0, "prim_path": "/on_its_side"},
        {"category": "car", "x_m": 20.0, "y_m": 0.0, "yaw_deg": 0.0,
         "pitch_deg": -80.0, "prim_path": "/nose_down"},
    ]
    sv = T.survey_from_placements(placements, None, None, "earthquake")
    assert [c["prim_path"] for c in sv["vehicles"]] == ["/upright"]


def test_a_scene_with_no_cars_hands_that_share_on():
    """Same rule as every other cohort: the people existed either way."""
    sv = _survey()
    sv["vehicles"] = []
    vs = _sample(_in_vehicle_config(), sv)
    assert not [v for v in vs if v["cohort"] == "in_vehicle"]
    assert len(vs) >= T.target_count(sv, T.settings(_config())) - 2


def test_gathered_survivors_keep_clear_of_every_facade():
    sv = _survey()
    clear = T.DEFAULTS["clearance_m"]
    for v in _by_cohort(_sample(survey=sv)).get("open_space", []):
        assert T._dist_to_buildings(v["x"], v["y"], sv["buildings"]) >= clear


def test_nobody_stands_on_top_of_anybody():
    import math
    vs = _sample()
    sep = T.DEFAULTS["min_separation_m"]
    for i, a in enumerate(vs):
        for b in vs[i + 1:]:
            assert math.hypot(a["x"] - b["x"], a["y"] - b["y"]) >= sep - 1e-9


def test_nobody_but_the_trapped_is_under_a_rubble_pile():
    """A severe quake lays 10-15 m piles; a victim sampled without regard to
    them is inside one, labelled `open`, and invisible from every angle. Caught
    in the viewport before it was caught here."""
    sv = _survey()
    sv["debris"] = [{"x": b["x"] + 12.0, "y": b["y"], "r": 9.0}
                    for b in sv["buildings"][:12]]
    for v in _sample(survey=sv):
        if v["cohort"] == "inside_rubble":
            continue
        assert not T._in_debris(v["x"], v["y"], sv), v


def test_occluded_victims_exist_and_are_labelled():
    """The point of keeping them: a run is scored on what was FINDABLE."""
    vis = {v["visibility"] for v in _sample()}
    assert "occluded" in vis and "open" in vis


# -- 3. the mix responds; the population does not --------------------------

def test_night_puts_more_people_indoors():
    def indoor(occ):
        vs = _sample(_config(occupancy=occ))
        return sum(1 for v in vs if v["cohort"] == "inside_rubble") / len(vs)

    assert indoor("night") > indoor("day") > indoor("commute")


def test_commute_puts_more_people_in_the_street():
    def street(occ):
        vs = _sample(_config(occupancy=occ))
        return sum(1 for v in vs if v["cohort"] == "street") / len(vs)

    assert street("commute") > street("day") > street("night")


def test_an_undamaged_city_traps_nobody_and_still_has_its_people():
    """No collapsed buildings: the trapped share goes outdoors rather than
    being dropped, so the population is intact and nobody is 'in' a ruin."""
    sv = _survey(levels_=("pristine",), n_per_level=24)
    vs = _sample(survey=sv)
    assert not [v for v in vs if v["cohort"] in ("inside_rubble", "exit_ring",
                                                 "rubble_edge")]
    assert len(vs) >= T.target_count(sv, T.settings(_config())) - 2


def test_population_does_not_move_with_severity():
    counts = {sev: len(_sample(_config(severity=sev)))
              for sev in (0.2, 0.5, 0.9)}
    assert len(set(counts.values())) == 1, counts


def test_count_scales_with_area_and_clamps():
    cfg = T.settings(_config(count_per_km2=100.0, count_clamp=[4, 60]))
    assert T.target_count({"region": (1000.0, 1000.0)}, cfg) == 60      # clamped
    assert T.target_count({"region": (400.0, 400.0)}, cfg) == 16
    assert T.target_count({"region": (50.0, 50.0)}, cfg) == 4           # clamped


def test_no_cohort_weights_means_no_targets():
    cfg = _config()
    cfg["targets"]["cohorts"] = {c: 0.0 for c in T.COHORTS}
    assert _sample(cfg) == []
    cfg2 = _config()
    cfg2["targets"]["enabled"] = False
    assert _sample(cfg2) == []


# -- 4. the artifacts ------------------------------------------------------

def test_ground_truth_round_trips():
    vs = _sample()
    with tempfile.TemporaryDirectory() as d:
        path = T.write_ground_truth(vs, _config(), os.path.join(d, "t.json"))
        doc = json.load(open(path))
    assert doc["victims"] == vs
    assert doc["summary"]["total"] == len(vs)
    assert doc["disaster"]["type"] == "earthquake"
    assert sum(doc["summary"]["by_cohort"].values()) == len(vs)


def test_every_pose_is_authored():
    """A cohort naming a pose `_HUMAN_POSES` does not carry would silently
    place a T-posed mannequin — the bug this table exists to prevent."""
    import scene_generator as sg

    for cohort, table in T._POSES.items():
        for pose, _w in table:
            assert pose in sg._HUMAN_POSES, (cohort, pose)


def test_region_bounds_are_not_read_as_an_extent():
    """`city_layout` records bounds, the config records an extent. Reading one
    as the other would sample the whole population into a quarter of the city."""
    assert T._wh((-200.0, -200.0, 200.0, 200.0)) == (400.0, 400.0)
    assert T._wh((400.0, 400.0)) == (400.0, 400.0)


# -- the compiler side -----------------------------------------------------

def _compiled(spec):
    import yaml

    import compile_disaster as cd

    with open(cd.DEFAULT_BASE) as fh:
        return cd.compile_spec(spec, yaml.safe_load(fh))


def test_earthquake_compiles_targets_and_disowns_scenery_humans():
    cfg = _compiled({"disaster-type": "earthquake", "severity": 0.7})
    assert cfg["targets"]["owns_humans"] is True
    assert sum(cfg["targets"]["cohorts"].values()) > 0
    # Stage B must not leave unlabelled people in a scene Stage C owns.
    assert cfg["detail"]["humans"]["sidewalk_spacing_m"] == 0.0
    assert cfg["detail"]["humans"]["trail_spacing_m"] == 0.0
    assert cfg["detail"]["humans"]["per_block"][-1] == 0
    assert cfg["disaster"]["humans_prone_fraction"] == 0.0
    assert list(cfg["disaster"]["humans_strewn"]) == [0, 0]


def test_other_disasters_place_no_targets_and_keep_their_scenery():
    cfg = _compiled({"disaster-type": "tornado", "severity": 0.7})
    assert not any((cfg.get("targets") or {}).get("cohorts", {}).values())
    assert cfg["detail"]["humans"]["sidewalk_spacing_m"] > 0.0
