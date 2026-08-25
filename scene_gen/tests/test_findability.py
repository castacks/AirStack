"""G3 — every placed human is possible to find, on every seed.

`test_targets.py` pins where each cohort GOES. This pins the property that
makes the ground truth worth having: a search run scored against a victim
nobody could ever have seen is being penalised for a placement bug, and the
number it produces is not a measurement of the search.

The definition and both backends live in `findability.py`; its docstring is
where the numbers come from. What is asserted here, in order:

1. The rules the model is made of — an intact wall is opaque, wreckage is not,
   the earth is opaque, a collapsed building is not as tall as the model it
   was cut from. Each of these was wrong once and each cost a whole seed.
2. The three placement invariants that make the cohorts findable BY
   CONSTRUCTION rather than by luck: the trapped are in the rim band, nobody
   is inside intact geometry, nobody is settled below grade.
3. The gate itself, on real scenes: `urban_quake_tiny` over three seeds, plus
   the full `earthquake` preset — no victim `buried`, same seed reproduces,
   different seeds differ.

The scene-building cases run the whole generator on the host with
`measure_usds` off — a couple of seconds each, no Isaac, no Nucleus. The stage
backend (`check_on_stage`) is the one thing not covered here, because it needs
PhysX; it is exercised by a real launch, which prints the same report.
"""

import io
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import findability as F                                          # noqa: E402
import targets as T                                              # noqa: E402
from disaster import levels                                      # noqa: E402

LADDER = levels.level_names("earthquake")

#: The scenes the gate is run on. `urban_quake_tiny` is the small-map loop
#: MISSIONS.md Phase 1 works in; `earthquake` is a full city, where there are
#: enough trapped victims for the rim band to matter.
CASES = [("urban_quake_tiny", 1), ("urban_quake_tiny", 2),
         ("urban_quake_tiny", 3), ("earthquake", 42)]


def _survey(cut=True, level="pancaked", h=24.0):
    """One building at the origin, and nothing else."""
    return {"region": (200.0, 200.0), "roads": [], "debris": [],
            "ladder": LADDER,
            "buildings": [{"x": 0.0, "y": 0.0, "w": 30.0, "h": 20.0,
                           "z": h, "yaw": 0.0, "level": level,
                           "level_i": LADDER.index(level), "cut": cut,
                           "prim_path": "/World/b"}]}


def _victim(x, y, z=0.0, cohort="street", **kw):
    v = {"id": 0, "cohort": cohort, "x": x, "y": y, "z": z, "lying": True}
    v.update(kw)
    return v


def _verdict(victim, survey):
    return F.check([victim], F.occluders_from_survey(survey))[0]["verdict"]


@pytest.fixture(scope="module")
def scenes():
    """`(config, survey, victims)` per case, built once — a few seconds each."""
    import contextlib

    out = {}
    for preset, seed in CASES:
        with contextlib.redirect_stdout(io.StringIO()):
            out[(preset, seed)] = F.offline_scene(preset, seed)
    return out


# -- 1. the rules the model is made of ------------------------------------

#: Two metres in from the +X face of `_survey`'s 30x20 building — inside the
#: rim band `targets._s_inside_rubble` samples, so the only thing that decides
#: the verdict is what the building is made of.
RIM_XY = (13.0, 0.0)


def test_an_intact_wall_is_opaque_at_any_thickness():
    """The `cut` distinction, which is the whole reason `targets.
    mark_cut_geometry` asks the stage instead of trusting the damage label."""
    assert _verdict(_victim(*RIM_XY), _survey(cut=False)) == "buried"


def test_wreckage_is_not_a_wall():
    """Same spot, same person, geometry actually cut open: a way out."""
    assert _verdict(_victim(*RIM_XY), _survey(cut=True)) == "partial"


def test_the_middle_of_a_pile_is_still_too_deep():
    """Porous is not a free pass. Two metres of wreckage is an obstruction;
    ten metres of it, which is what the centre of a 30x20 footprint is, is
    not somewhere a search sightline goes."""
    assert _verdict(_victim(0.0, 0.0), _survey(cut=True)) == "buried"


def test_the_earth_is_opaque():
    """Every bearing points upward, so the ground is the one thing between a
    victim below grade and daylight. Without it in the occluder list, a body
    sunk past the bottom of its pile leaves the box out of the underside and
    reports open air."""
    assert _verdict(_victim(0.0, 0.0, z=-4.0), _survey()) == "buried"


def test_a_collapsed_building_is_not_as_tall_as_the_model():
    """The survey measures a wrecked building from the INTACT asset it was cut
    out of. Charging a ray that height buries every trapped victim under their
    own building — `urban_quake_tiny` seeds 3 and 6 failed at 3.30 m and
    3.12 m, which was the pile height, not the person."""
    b = _survey()["buildings"][0]
    tall = F.rubble_height(dict(b, level_i=LADDER.index("cracked")), LADDER)
    flat = F.rubble_height(dict(b, level_i=LADDER.index("pancaked")), LADDER)
    assert b["z"] > tall > flat
    assert flat == pytest.approx(b["z"] * F.FLAT_FRAC)


def test_a_gap_narrower_than_open_m_is_not_a_way_out():
    """Two slabs a hand's width apart are one slab. Otherwise a victim reads
    `clear` through a crack nothing could reach them by."""
    sv = _survey()
    sv["debris"] = [{"x": 0.0, "y": 0.0, "r": 40.0,
                     "z": F.OPEN_M + 30.0}]         # a lid over the footprint
    assert _verdict(_victim(0.0, 0.0), sv) == "buried"


def test_a_car_is_an_obstruction_and_not_a_wall():
    """The spec allows a victim inside a vehicle and calls them findable. A car
    is the one enclosure in the scene made mostly of glass: obstructed from
    every bearing, hidden from none."""
    sv = _survey()
    sv["vehicles"] = [{"x": 40.0, "y": 40.0, "w": 4.5, "h": 2.0, "z": 1.5,
                       "yaw": 30.0, "prim_path": "/World/car"}]
    seated = _victim(40.0, 40.0, z=0.675, cohort="in_vehicle", lying=False)
    row = F.check([seated], F.occluders_from_survey(sv))[0]
    assert row["verdict"] == "partial"
    assert 0.0 < row["escape_m"] <= F.COVER_M


def test_a_seated_victim_is_not_settled_onto_the_car():
    """`settle_offline` and `targets.settle_on_surface` both skip this cohort.
    A downward probe from above finds the ROOF of the car they are inside."""
    v = {"id": 0, "cohort": "in_vehicle", "x": 0.0, "y": 0.0, "seat_z": 0.675}
    F.settle_offline([v], [])
    assert v["z"] == pytest.approx(0.675)


# -- 2. findable by construction ------------------------------------------

def test_the_trapped_are_in_the_rim_band(scenes):
    """`rubble_rim_m` metres in from a face, and no further. The predecessor's
    fraction-of-footprint draw put people 6-10 m into a 30 m pile."""
    for (preset, seed), (cfg, _sv, victims) in scenes.items():
        rim = float(T.settings(cfg).get("rubble_rim_m"))
        for v in victims:
            if v["cohort"] != "inside_rubble":
                continue
            assert 0.0 <= v["rubble_depth_m"] <= rim + 1e-6, (preset, seed, v)


def test_the_rim_band_is_within_what_a_sightline_reaches():
    """The coupling that makes the cohort findable rather than lucky: the
    sampler must not place people deeper than the validator can see."""
    assert float(T.DEFAULTS["rubble_rim_m"]) <= F.COVER_M


def test_nobody_is_inside_intact_geometry(scenes):
    """A footprint is NOT axis-aligned — `w`/`h` are measured in the asset's
    frame and the placement yaws it. Reading the pair as a world AABB let an
    `exit_ring` victim be sited inside a 30x20 building at yaw 90 on
    `urban_quake_tiny` seed 3, opaque from every bearing."""
    for (preset, seed), (_cfg, sv, victims) in scenes.items():
        intact = [b for b in sv["buildings"] if not b.get("cut")]
        for v in victims:
            assert not T._inside_any(v["x"], v["y"], intact), (preset, seed, v)


def test_nobody_is_settled_below_grade(scenes):
    """`bury_frac` is a fraction of the BODY, and a crouching victim is 1.7 m
    tall where a pancaked pile is often under 2 m. Grade is the floor."""
    for (preset, seed), (_cfg, _sv, victims) in scenes.items():
        for v in victims:
            assert v["z"] >= 0.0, (preset, seed, v)


def test_a_yawed_footprint_is_not_read_as_an_axis_aligned_one():
    b = {"x": 0.0, "y": 0.0, "w": 30.0, "h": 20.0, "yaw": 90.0}
    assert T._inside(b, 0.0, 14.0)          # inside once yaw is applied
    assert not T._inside(b, 14.0, 0.0)      # outside, though |dx| <= w/2
    assert T.edge_depth_m(b, 0.0, 14.0) == pytest.approx(1.0)


# -- 3. the gate ----------------------------------------------------------

def test_no_victim_is_buried(scenes):
    for (preset, seed), (_cfg, sv, victims) in scenes.items():
        rows = F.check(victims, F.occluders_from_survey(sv))
        s = F.summarize(rows)
        assert s["pass"], (preset, seed, F.format_report(rows))
        assert s["total"] == len(victims)


def test_the_gate_is_not_a_rubber_stamp(scenes):
    """It has to be able to fail. Same scene, same people, each one moved to
    the middle of the nearest collapsed footprint — which is where the
    pre-rim-band sampler was putting them."""
    _cfg, sv, victims = scenes[("earthquake", 42)]
    ruins = [b for b in sv["buildings"] if b.get("cut")]
    assert ruins, "no cut buildings to bury anyone in"
    moved = [dict(v, x=ruins[0]["x"], y=ruins[0]["y"], z=-1.0) for v in victims]
    assert not F.summarize(F.check(moved, F.occluders_from_survey(sv)))["pass"]


def test_same_seed_same_people(scenes):
    import contextlib

    for preset, seed in CASES:
        _c, _s, want = scenes[(preset, seed)]
        with contextlib.redirect_stdout(io.StringIO()):
            _c2, _s2, got = F.offline_scene(preset, seed)
        assert got == want, (preset, seed)


def test_different_seeds_different_people(scenes):
    """Pairwise over the three `urban_quake_tiny` seeds — G3's seed-diversity
    half. A generator that ignored the seed would pass every other test here."""
    xy = [tuple((v["x"], v["y"]) for v in scenes[("urban_quake_tiny", s)][2])
          for s in (1, 2, 3)]
    assert len(set(xy)) == 3, xy


def test_the_report_says_which_victim_and_why(scenes):
    """The standalone report is the artefact a G3 sign-off cites, so it has to
    name the failures rather than just count them."""
    _cfg, sv, victims = scenes[("earthquake", 42)]
    rows = F.check(victims, F.occluders_from_survey(sv))
    text = F.format_report(rows, "earthquake")
    assert "PASS" in text and str(len(rows)) in text
    for r in rows:
        assert r["verdict"] in F.VERDICTS
        assert r["dirs"] == len(F.directions())
