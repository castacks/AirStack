"""
Tests for `disaster/tornado.py` — the tornado building-damage ladder.

The pipeline itself (thicken, cut, propagate, settle) is `mesh_damage`'s and is
tested in `test_mesh_damage.py`. What is tested here is the only thing this
script owns: that its four rungs are FOUR DISTINCT KINDS OF DAMAGE, in order,
and that they read as a storm rather than as a collapse.

Every assertion below is a property that a screenshot cannot check and that has
a known way of silently going wrong:

  * the rungs must be monotone, or the ladder is not a ladder and Stage B's
    quantiser is picking between archetypes that do not differ;
  * the failure must START at the roof and only reach the sill at the top rung,
    or a tornado is an earthquake with a different name;
  * every mechanism in a rung must share ONE bearing, or the track's signature
    is gone and a street reads as a row of unrelated bomb sites;
  * the mass that leaves must be the LIGHT mass, or a swept lot comes out as a
    uniform thinning of a building-shaped pile instead of a bare slab.

Needs only numpy and `pxr`; no Kit, no PhysX.
"""

import os
import sys

import numpy as np
import pytest
from pxr import Gf, Usd, UsdGeom

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from disaster import levels as L        # noqa: E402
from disaster import mesh_damage as M   # noqa: E402
from disaster import tornado as T       # noqa: E402

#: A house, not a tower: the suburban case the tornado ladder is tuned on.
HOUSE = M.Bounds([-6.0, -5.0, 0.0], [6.0, 5.0, 8.0])

#: The storm's bearing for every test, so "windward" means -x throughout.
HEADING = 0.0

#: The rungs, worst last. `pristine` is excluded — it has no recipe by design.
RUNGS = [r.name for r in L.ladder("tornado", L.STRUCTURE)][1:]


def _grid(b, n=13, nz=21):
    """Points spanning the building's box."""
    lo, hi = b.lo, b.hi
    return np.array([[x, y, z]
                     for x in np.linspace(lo[0], hi[0], n)
                     for y in np.linspace(lo[1], hi[1], n)
                     for z in np.linspace(lo[2], hi[2], nz)])


def _damage(level, bounds=HOUSE, seed=3, heading=HEADING):
    g = _grid(bounds)
    return g, T.field_for(level, seed, heading)(bounds).damage(g)


def _released(level):
    """Share of the building whose own damage clears the rung's threshold."""
    g, d = _damage(level)
    return float((d >= T.plan_for(level).release).mean())


# ---------------------------------------------------------------------------
# the ladder
# ---------------------------------------------------------------------------


def test_every_rung_of_the_ladder_has_a_recipe():
    """A rung with no `RUNG_PLAN` entry bakes nothing and Stage B references a
    hole. `pristine` is the one exception, and it is meant to be untouched."""
    assert set(RUNGS) == set(T.RUNG_PLAN), (
        f"ladder {RUNGS} vs recipes {sorted(T.RUNG_PLAN)}")
    assert T.plan_for("pristine") is None
    assert T.plan_for("no_such_rung") is None      # a typo must not raise


def test_the_ladder_is_monotone():
    """Each rung must free strictly more of the building than the one below.

    Not a formality: `quake`'s rungs were tuned independently against different
    fields and came out non-monotonic — the gentler rung looked worse than the
    harsher one — which is the bug `RUNG_PLAN` composition exists to prevent.
    """
    freed = [_released(r) for r in RUNGS]
    assert freed == sorted(freed), dict(zip(RUNGS, freed))
    assert freed[-1] > freed[0] + 0.3, dict(zip(RUNGS, freed))


def test_wind_fails_the_building_from_the_top_down():
    """The bottom rungs must not touch the ground floor at all.

    This is the whole difference from an earthquake, whose demand is largest at
    the base. If `roof_stripped` reaches the sill, the two disasters are one
    disaster and the type axis means nothing from the air.
    """
    for level in ("roof_stripped", "roof_lost"):
        g, d = _damage(level)
        base = d[g[:, 2] < HOUSE.frac(0.1)]
        top = d[g[:, 2] > 7.0]
        assert base.max() < T.plan_for(level).release, level
        assert top.mean() > 5.0 * max(base.mean(), 1e-6), level


def test_a_direct_hit_fails_the_sill_as_well_as_the_roof():
    """`swept_clean` is a house crushed and taken away, not a house with its
    roof off — so the load has to reach the ground. Without a `floor` under the
    height profile it cannot, however hard the field is driven, and the top
    rung renders as the rung below it standing on an untouched ground floor.
    """
    g, d = _damage("swept_clean")
    base = d[g[:, 2] < 1.0]
    assert base.mean() > T.plan_for("swept_clean").release, base.mean()


def test_the_worst_rung_takes_the_whole_plan_and_the_gentlest_does_not():
    """A direct hit has no sheltered face — the vortex loads every side in turn
    — while a glancing one strips the windward roof and leaves the lee. That
    contrast is `windward`, and it is what stops the top two rungs from looking
    like the same damage at two amplitudes."""
    def lee_share(level):
        g, d = _damage(level)
        high = g[:, 2] > 6.0
        wind = d[high & (g[:, 0] < -4.0)].mean()
        lee = d[high & (g[:, 0] > 4.0)].mean()
        return lee / max(wind, 1e-6)

    assert lee_share("swept_clean") > 2.0 * lee_share("roof_stripped")


# ---------------------------------------------------------------------------
# the storm signature
# ---------------------------------------------------------------------------


def test_every_mechanism_in_a_rung_shares_the_storm_bearing():
    """`quake.field_for` spreads its mechanisms around the plan on a golden
    angle, because ground motion has no bearing a street shares. A storm does,
    and the composed field must fail the same face for every mechanism in the
    rung — otherwise a building has its roof off on one side and its wall out
    on another, which is not a storm and does not read as a track.
    """
    for level in RUNGS:
        g, d = _damage(level)
        wind = d[g[:, 0] < -4.0].mean()
        lee = d[g[:, 0] > 4.0].mean()
        assert wind > lee, f"{level}: windward {wind:.2f} <= leeward {lee:.2f}"


def test_the_bearing_turns_the_damage_with_it():
    """The configured heading must actually steer the failure, or every
    building on the track picks its own and the corridor loses its signature."""
    def windward_at(heading):
        g, d = _damage("walls_breached", heading=heading)
        return d[g[:, 0] < -4.0].mean()

    # Storm travelling toward +x strikes the -x face; toward -x it does not.
    assert windward_at(0.0) > windward_at(180.0) + 0.05


def test_debris_all_travels_the_same_way():
    """Released material is displaced by `Failure.ejecta` before PhysX sees it,
    and every piece of every mechanism must go downwind together."""
    g = _grid(HOUSE)
    e = T.field_for("swept_clean", 3, HEADING)(HOUSE).ejecta(g)
    moved = np.abs(e[:, 0]) > 1e-6
    assert moved.any()
    assert (e[moved, 0] > 0.0).all(), "some debris went upwind"
    assert e[:, 2].min() >= 0.0, "some debris was thrown into the ground"


def test_the_throw_separates_fragments_without_launching_the_pile():
    """The throw is bounded, and the bound is the finding this test exists for.

    A storm's material ends up a long way from the building, so the first cut
    of this file made `throw` up to 2.2x the building radius — and every heavy
    rung exploded: +7.5 m mean vertical displacement on `walls_breached`,
    +19.8 m on `swept_clean`, i.e. the wreckage going UP (measured on four
    bungalows via `tools/damage_spread.py --set suburban`).

    The cause is that `ejecta` displaces each fragment by an amount
    proportional to its OWN damage, while the gust noise varies on a ~5 m scale
    and fragments are ~1.4 m. Neighbours therefore separate by a metre or two
    at a large throw — through each other — and PhysX resolves that by
    launching the pile. Carrying debris to the next street is
    `compile_tornado`'s scene-level `debris.path_*` band, not this pipeline's.

    So: enough to clear a fragment from its neighbours, and less than the
    building's own radius.
    """
    g = _grid(HOUSE)
    e = T.field_for("swept_clean", 3, HEADING)(HOUSE).ejecta(g)
    worst = float(e[:, 0].max())
    assert worst > 2.0 * T.DEBRIS_M, f"too small to separate a fragment: {worst}"
    assert worst < HOUSE.radius, f"large enough to launch the pile: {worst}"


def test_no_mechanism_throws_far_enough_to_explode():
    """The bound above holds for every mechanism, not just the worst rung."""
    for name, m in T.MECHANISMS.items():
        assert m.throw <= 0.6, f"{name} throw {m.throw} risks interpenetration"
        assert m.blast <= 0.5, f"{name} blast {m.blast} launches small pieces"


# ---------------------------------------------------------------------------
# severity is not an argument
# ---------------------------------------------------------------------------


def test_a_rung_is_the_same_damage_at_every_severity():
    """`levels.local_damage` is the ONLY place severity may enter. Two scenes
    at different severities must give a building on the same rung the same
    field, or Stage A's one-library-per-disaster stops being reusable."""
    import inspect
    for fn in (T.at_level, T.field_for, T.plan_for, T.shatter):
        assert "severity" not in inspect.signature(fn).parameters, fn.__name__
    # And the recipes themselves are constants, not functions of anything.
    assert all(isinstance(m.release, float) for m in T.MECHANISMS.values())


def test_the_same_seed_and_rung_reproduce_the_same_building():
    """Config plus seed reproduces the scene — the generator's core invariant."""
    _, a = _damage("swept_clean", seed=11)
    _, b = _damage("swept_clean", seed=11)
    _, c = _damage("swept_clean", seed=12)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


# ---------------------------------------------------------------------------
# what the storm carries off
# ---------------------------------------------------------------------------


def _stage_with_fragments(sizes):
    """A stage of cubes of the given edge lengths, and a fake shatter report."""
    st = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(st, "/World")
    paths = []
    for i, s in enumerate(sizes):
        p = f"/World/f{i}"
        cube = UsdGeom.Cube.Define(st, p)
        cube.CreateSizeAttr(float(s))
        UsdGeom.Xformable(cube).AddTranslateOp().Set(
            Gf.Vec3d(i * 100.0, 0.0, 0.0))
        paths.append(p)
    return st, {"paths": list(paths), "loose": list(paths)}


def test_the_storm_carries_off_the_light_material_first():
    """The bias is REVERSED from `quake.consume`, and that is the point.

    A collapse pulverises its biggest panels into the voids of its own pile. A
    storm lofts what it can lift, so what leaves is the small stuff and what is
    left on the slab is the heavy slabs and the foundation. Draw from the wrong
    end and a swept lot comes out as a uniformly thinned building-shaped pile.
    """
    sizes = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    st, rep = _stage_with_fragments(sizes)
    gone = T.carry_off(st, rep, fraction=0.5, pool=1.0, seed=0)
    assert gone == 4
    kept = {p.rsplit("f", 1)[1] for p in rep["loose"]}
    assert kept == {"4", "5", "6", "7"}, kept
    assert all(not st.GetPrimAtPath(f"/World/f{i}").IsActive()
               for i in range(4))


def test_carrying_off_nothing_is_a_no_op():
    st, rep = _stage_with_fragments([1.0, 2.0, 3.0])
    assert T.carry_off(st, rep, fraction=0.0) == 0
    assert len(rep["loose"]) == 3


@pytest.mark.parametrize("level", RUNGS)
def test_a_direct_hit_loses_more_mass_than_a_glancing_one(level):
    """`consume` must climb with the rung: a stripped roof has not lost 70% of
    the house. It is a property of the RUNG, not of any severity."""
    share = T.plan_for(level).consume
    assert 0.0 <= share <= 0.8
    if level == "swept_clean":
        assert share > T.plan_for("roof_stripped").consume + 0.5
