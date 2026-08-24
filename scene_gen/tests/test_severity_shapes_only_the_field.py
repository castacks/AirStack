"""The invariant Stage A's archetype library rests on.

Severity shapes the FIELD. It never shapes the LADDER, and it never shapes the
per-(type, level) damage model. If it ever does, the library stops being
reusable across severities — Stage A would have to be re-baked per scene, which
is exactly the per-scene cost the staged pipeline exists to abolish.

This is the kind of invariant a later "just scale this by sev" edit destroys
silently: nothing crashes, the scenes still render, and the library is quietly
wrong for every severity but the one it was baked at. Hence a test.
"""

import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import compile_disaster as cd                                    # noqa: E402
from disaster import levels as L                                 # noqa: E402

REGION = (800.0, 800.0)          # what compile_* takes: (w, h)
#: The same region as a box centred on the origin, so (0, 0) is both the
#: default epicentre and the midpoint of a default tornado track.
REGION_BOX = (-400.0, -400.0, 400.0, 400.0)
SEVERITIES = (0.1, 0.3, 0.5, 0.7, 0.9, 1.0)

#: Keys under `disaster:` that describe the per-asset damage MODEL rather than
#: the field or a Stage B placement decision. These must not move with severity.
_MODEL_KEYS = ("mesh_damage",)


KINDS = (L.STRUCTURE, L.VEGETATION)

TYPE_KIND = [(d, k) for d in sorted(cd.DISASTERS) for k in KINDS]


@pytest.mark.parametrize("dtype,kind", TYPE_KIND)
def test_ladder_does_not_depend_on_severity(dtype, kind):
    """The rungs are a property of the disaster, not of how bad it was."""
    base = L.ladder(dtype, kind)
    assert base, f"{dtype}/{kind} has no ladder"
    # The ladder is a module constant, so the real assertion is that nothing in
    # compilation can reach it: compiling at every severity must leave it equal.
    for sev in SEVERITIES:
        cd.DISASTERS[dtype](sev, {}, REGION)
        assert L.ladder(dtype, kind) == base


@pytest.mark.parametrize("dtype", sorted(cd.DISASTERS))
def test_damage_model_does_not_depend_on_severity(dtype):
    """The per-(type, level) damage model is keyed off disaster type alone.

    This is what lets one archetype library serve every severity.
    """
    ref = {k: cd.DISASTERS[dtype](SEVERITIES[0], {}, REGION).get(k)
           for k in _MODEL_KEYS}
    for sev in SEVERITIES[1:]:
        block = cd.DISASTERS[dtype](sev, {}, REGION)
        for k in _MODEL_KEYS:
            assert block.get(k) == ref[k], (
                f"{dtype}.{k} moved with severity ({SEVERITIES[0]} -> {sev}). "
                "The archetype library is baked once per (type, level); a "
                "damage-model knob that reads severity makes it valid at only "
                "one severity.")


@pytest.mark.parametrize("dtype", sorted(cd.DISASTERS))
def test_local_damage_does_depend_on_severity(dtype):
    """The other half: severity must actually reach the asset, or it is inert.

    Note this is asserted on LOCAL DAMAGE, not on the field. A hurricane's
    field is `uniform inside 1.0` at every severity and that is correct — the
    storm covers the whole region; what changes is how hard. Severity enters
    through `local_damage`, not through the field's own numbers.
    """
    from disaster.field import make_damage_field
    seen = set()
    for sev in SEVERITIES:
        cfg = cd.DISASTERS[dtype](sev, {}, REGION)["field"]
        f = make_damage_field(cfg, REGION_BOX)
        seen.add(round(L.local_damage(f(0.0, 0.0), sev), 4))
    assert len(seen) > 1, (
        f"{dtype}: local damage at the centre is the same at every severity — "
        "severity is inert")


@pytest.mark.parametrize("dtype", sorted(cd.DISASTERS))
def test_severity_is_not_folded_into_the_field(dtype):
    """The field must stay a pure spatial shape.

    `disaster_stage` draws against the raw field (`rng.random() < frac *
    field(x, y)`) while `frac` has ALREADY been lerped by severity. A field
    that also carried severity would apply it twice and thin every topple and
    debris draw quadratically. So the field's peak value must not scale down
    with severity.
    """
    from disaster.field import make_damage_field
    peaks = []
    for sev in SEVERITIES:
        cfg = cd.DISASTERS[dtype](sev, {}, REGION)["field"]
        peaks.append(make_damage_field(cfg, REGION_BOX).hi)
    assert max(peaks) - min(peaks) < 1e-9, (
        f"{dtype}'s field peak moves with severity ({peaks}) — severity looks "
        "folded into the field, which double-counts against the fraction draws")


@pytest.mark.parametrize("dtype,kind", TYPE_KIND)
def test_level_at_is_monotonic_in_intensity(dtype, kind):
    """A harder-hit asset never lands on a gentler rung."""
    rungs = L.ladder(dtype, kind)
    order = {r.name: i for i, r in enumerate(rungs)}
    seen = -1
    for step in range(0, 101):
        i = order[L.level_at(dtype, step / 100.0, kind).name]
        assert i >= seen, f"{dtype}/{kind} ladder went backwards at {step/100}"
        seen = i


@pytest.mark.parametrize("dtype,kind", TYPE_KIND)
def test_ladder_starts_pristine_and_reaches_its_top(dtype, kind):
    rungs = L.ladder(dtype, kind)
    assert rungs[0].at == 0.0 and rungs[0].name == "pristine"
    assert L.level_at(dtype, 0.0, kind).name == "pristine"
    assert L.level_at(dtype, 1.0, kind).name == rungs[-1].name
    # Thresholds strictly increase, or a rung is unreachable.
    ats = [r.at for r in rungs]
    assert ats == sorted(set(ats)), f"{dtype}/{kind} has unordered rungs"


@pytest.mark.parametrize("dtype,kind", TYPE_KIND)
def test_bake_levels_covers_every_rung_and_variant(dtype, kind):
    """Stage A must bake an archetype for everything Stage B can ask for.

    A variant is a distinct asset even though it is not a distinct intensity:
    `stand_outcome` cannot roll a snag to `fallen` if nothing baked one.
    """
    want = L.bake_levels(dtype, kind)
    assert len(want) == len(set(want)), f"{dtype}/{kind} bakes a name twice"
    for r in L.ladder(dtype, kind):
        assert r.name in want
        for v in r.variants:
            assert v in want, f"{dtype}/{kind}: variant {v} is never baked"


def test_clamps_rather_than_raising():
    assert L.level_at("fire", -5.0).name == "pristine"
    assert L.level_at("fire", 99.0).name == "rubble"
    assert L.level_at("nonexistent", 1.0).name == "pristine"


# --------------------------------------------------------------------------
# Ladder rungs have to be renderable
# --------------------------------------------------------------------------

@pytest.mark.parametrize("dtype", sorted(cd.DISASTERS))
def test_vegetation_rungs_are_levels_the_renderer_knows(dtype):
    """A rung name is not a wish. `disaster.vegetation.plan_for` is the table
    that renders a tree, and a level it does not know raises AT BAKE TIME —
    which is how `leaning`, `uprooted`, `defoliated`, `debarked`, `silted` and
    `drowned` were caught, four archetypes into a run that had already cost
    minutes. Cheaper to catch here.
    """
    from disaster import vegetation as veg

    for name in L.bake_levels(dtype, L.VEGETATION):
        veg.plan_for(name)          # raises ValueError if unknown


def test_non_fire_vegetation_is_not_charred():
    """Only a fire chars a tree. The felling geometry is shared — a snapped
    trunk is snapped however it happened — so the fire columns are zeroed
    rather than the geometry being duplicated under new names."""
    from disaster import vegetation as veg

    for name in ("snag", "fallen", "stump"):
        _kb, _kt, cov, brown, scorch_h, geom = veg.plan_for(name, fire=False)
        assert (cov, brown, scorch_h) == (0.0, 0.0, 0.0)
        assert geom, f"{name} must still carry its felling geometry"
        # …and the fire path is untouched.
        assert veg.plan_for(name)[2] > 0.0
