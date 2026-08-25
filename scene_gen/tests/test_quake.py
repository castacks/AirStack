"""
Tests for `disaster/quake.py` — the earthquake ladder ON A SUBURBAN HOUSE.

The pipeline (thicken, cut, propagate, settle) is `mesh_damage`'s and is
tested in `test_mesh_damage.py`; the ladder's shape is exercised by the
mechanism grid. What is pinned here is the thing that was missing entirely:
the ladder was quoted against 50-96 m masonry towers, and a detached timber
house is not a small one.

  * a rung must mean the same FAILURE whatever the walls are made of — the
    thresholds are the rung's, and a material that moved them would make
    `soft_storey` on a house a different rung from `soft_storey` on a tower,
    which is what the archetype library keys its art on;
  * but it must mean a different PIECE — a stud wall breaks into planks a
    metre long, not into half-metre-thick isotropic lumps of concrete, and
    the blast that separates them has to come down with them or the pile
    launches (`tornado`'s BLAST note measured exactly that on these houses);
  * and `at_level` must ACCEPT the declaration at all. It did not: every
    suburban earthquake cell of `tools/damage_spread.py` died with
    `fracture_to_stage() got an unexpected keyword argument 'material'`,
    so the earthquake ladder had never once run on a house.

Needs only numpy and `pxr`; no Kit, no PhysX.
"""

import inspect
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from disaster import levels as L        # noqa: E402
from disaster import mesh_damage as M   # noqa: E402
from disaster import quake as Q         # noqa: E402

#: The rungs, worst last. `pristine` is excluded — it has no recipe by design.
RUNGS = [r.name for r in L.ladder("earthquake", L.STRUCTURE)][1:]


@pytest.mark.parametrize("level", RUNGS)
def test_a_rung_means_the_same_failure_in_timber_as_in_masonry(level):
    """Thresholds belong to the RUNG. A house does not fail at a house rung."""
    masonry = Q.recipe(level, material="masonry")
    timber = Q.recipe(level, material="timber")
    for k in ("support", "release", "collapse", "consume", "intensity"):
        assert masonry[k] == timber[k], f"{level}: {k} moved with the material"


@pytest.mark.parametrize("level", RUNGS)
def test_a_timber_house_breaks_into_smaller_pieces_than_a_masonry_tower(level):
    """...and gets a proportionally smaller push. See `recipe`."""
    masonry = Q.recipe(level, material="masonry")
    timber = Q.recipe(level, material="timber")
    assert timber["fragment_m"] < masonry["fragment_m"]
    assert timber["blast"] <= masonry["blast"]
    # One ratio for both, so a smaller piece is never given a larger push.
    if masonry["blast"]:
        assert (timber["blast"] / masonry["blast"]
                == pytest.approx(timber["fragment_m"] / masonry["fragment_m"]))


def test_the_default_material_is_the_one_every_number_was_quoted_in():
    """Masonry, or the urban library silently changes under this ladder."""
    assert Q.MATERIAL == M.DEFAULT_MATERIAL == "masonry"
    assert Q.recipe("pancaked")["fragment_m"] == pytest.approx(
        Q.MECHANISMS["pancake"].fragment_m)


def test_a_stud_wall_is_not_thickened_into_a_bunker():
    """`shatter` takes the wall from the material, not from `WALL_M`.

    0.5 m of masonry in a 6.4 m bungalow is a wall an eighth of the height of
    the house it is in; `MATERIALS` says 0.15 m and the thickening has to read
    it, or a timber house comes out of the cut as concrete however fine the
    fragments are.
    """
    assert M.material("timber").wall_m < M.material("masonry").wall_m
    sig = inspect.signature(Q.shatter)
    # Defaulted to None so the MATERIAL supplies them, not to a masonry number
    # that would win over the declaration.
    assert sig.parameters["wall_m"].default is None
    assert sig.parameters["grain"].default is None
    assert sig.parameters["fragment_m"].default is None
    assert sig.parameters["material"].default == Q.MATERIAL


def test_the_ladder_takes_a_material_all_the_way_down():
    """The crash this file exists for: `material` must reach the cut.

    `at_level` -> `shatter` -> `mesh_damage.damage_building`. Anything
    `at_level` does not name is forwarded as a `shatter` argument, and
    `shatter` forwards what it does not name to `fracture_to_stage`, which has
    no `material` — so an unnamed parameter is not a pass-through here, it is
    a TypeError one house into a bake.
    """
    for fn in (Q.at_level, Q.shatter):
        assert "material" in inspect.signature(fn).parameters


def test_the_compiler_and_the_material_table_agree():
    """`compile_disaster.LOCALE_MATERIAL` repeats `MATERIALS`' wall figures.

    It has to repeat them — the compiler runs on a plain `python3` with no
    numpy and no `pxr`, and importing `mesh_damage` needs both — so the two
    can drift, and the drift is silent: the scene would thicken a timber house
    to a masonry wall while calling it timber.
    """
    import compile_disaster as C

    for locale, (name, wall_m) in C.LOCALE_MATERIAL.items():
        assert name in M.MATERIALS, f"{locale}: no such material {name!r}"
        assert M.MATERIALS[name].wall_m == pytest.approx(wall_m), locale


def test_a_suburb_is_built_out_of_timber_and_a_downtown_is_not():
    """The locale decides, and the disaster does not get a say."""
    import yaml

    import compile_disaster as C

    base = yaml.safe_load(open(os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "low_level", "default.yaml")))

    for dtype in ("earthquake", "tornado"):
        sub = C.compile_spec({"locale": "suburban", "disaster-type": dtype,
                              "severity": 0.8}, base)
        urb = C.compile_spec({"locale": "urban", "disaster-type": dtype,
                              "severity": 0.8}, base)
        assert sub["disaster"]["mesh_damage"]["material"] == "timber"
        assert urb["disaster"]["mesh_damage"]["material"] == "masonry"
        assert sub["disaster"]["mesh_damage"]["thickness"]["wall_m"] == \
            pytest.approx(M.MATERIALS["timber"].wall_m)
