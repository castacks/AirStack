"""levels — the damage LADDER, and the one quantiser that reads it.

THE DISTINCTION THIS MODULE EXISTS TO KEEP
-------------------------------------------
Two different things have been called "severity" in this codebase, and
conflating them is what kept the archetype library from generalising:

    severity   how bad the event is overall. Continuous 0-1, scene-wide, set
               in the high-level config.

    field      a pure spatial SHAPE, `f(x, y) -> 0..1` (see `field.py`). Its
               geometry moves with severity — a worse quake reaches further —
               but its value is "how central is this spot to the event", not
               "how hard was it hit".

    level      which rung of this disaster's ladder ONE asset lands on. A
               *kind* of damage — `roof_collapsed`, `burned_out` — not a
               magnitude. Discrete, per asset.

The composition, which `local_damage` is the single home for::

    local damage at (x, y)  =  field(x, y)  x  severity
    level                   =  the rung that damage falls on

Severity must NOT be folded into the field itself. `disaster_stage` also draws
against the raw field (`rng.random() < frac * field(x, y)`) while `frac` has
already been lerped by severity, so a field carrying severity would apply it
twice — thinning every topple and debris draw quadratically.

Stage A bakes ``<disaster>/<type>_<level>.usd`` for every (type, level) pair.
Stage B evaluates the field at each placement, quantises to a level, and
references the matching archetype. Because the ladder does not depend on
severity, **one library serves every severity, seed and scene** — that is the
whole reason Stage A is affordable.

THE LADDER
----------
Each disaster declares an ordered tuple of rungs, each a ``Rung`` carrying the
local intensity at which it *begins*. Rung 0 always begins at 0.0 and means
"the field did not reach here". `level_at` walks the ladder and returns the
last rung whose threshold the intensity has passed — so adding a rung is a
one-line change and the quantiser never needs touching.

Named rungs rather than bare numbers because the name is what the archetype
file is called and what a person reads in a manifest; `burned_out` says
something `0.6` does not. The `finish` field rides along for the fire ladder,
which needs to know whether a ruin is char or ash — that used to be a second
return value from `damage.level_for_age`, reachable only from fire code.
"""

from __future__ import annotations

from collections import namedtuple

#: One rung of a ladder.
#:
#: `at`       local damage at which this rung takes over.
#: `finish`   optional surface treatment (fire's char/ash), None elsewhere.
#: `variants` alternate archetypes at the SAME rung, drawn at random during
#:            assembly rather than chosen by intensity. A burnt tree is a
#:            `snag` by intensity, but a minority of snags are `fallen` or
#:            `stump` — that is a coin flip, not a hotter fire, and modelling
#:            it as extra rungs would make the ladder non-monotonic.
Rung = namedtuple("Rung", "name at finish variants")


def _ladder(*rows):
    out = []
    for row in rows:
        name, at, finish = row[0], float(row[1]), row[2]
        variants = tuple(row[3]) if len(row) > 3 else ()
        out.append(Rung(name, at, finish, variants))
    return tuple(out)


# ---------------------------------------------------------------------------
# The ladders — one per disaster type in `compile_disaster.DISASTERS`
#
# Thresholds are local field intensity, NOT scene severity. A rung's share of
# [0, 1] is how much of the damaged area lands on it: fire's `scorched` band is
# wide because a burn scar has a broad margin the front only grazed, while a
# tornado's `intact` band is narrow because inside the corridor almost nothing
# is spared.
# ---------------------------------------------------------------------------

#: What a ladder is keyed on. A tree and a building do not fail the same way
#: in the same event — a wildfire leaves a house `burned_out` and the oak
#: beside it a `snag` — so the ladder is per (disaster, KIND), not per
#: disaster. `structure` is the default kind; `vegetation` is anything with a
#: trunk and a canopy.
STRUCTURE = "structure"
VEGETATION = "vegetation"

#: VEGETATION RUNG NAMES ARE CONSTRAINED. They must be levels
#: `disaster.vegetation.plan_for` knows — `pristine`, `scorched`, `torched`,
#: `snag`, `fallen`, `stump` — because that table is what renders them. The
#: first two are fire-specific; the rest are pure geometry (a snapped trunk is
#: snapped however it happened), which is why the non-fire ladders below reuse
#: them and the baker passes `fire=False` so nothing comes out charred.
#: Inventing a name here does not create art: it raises at bake time, which is
#: how `leaning`, `uprooted`, `defoliated`, `debarked`, `silted` and `drowned`
#: were caught, four archetypes into a run.

LADDERS = {
  "fire": {
    # Ported from `damage.level_for_age`, which mapped seconds-since-ignition
    # to these six names. The `ellipse` field returns "how long the front has
    # been here" normalised to 0-1, so the ordering is preserved exactly and
    # only the units change.
    STRUCTURE: _ladder(
        ("pristine",         0.00, None),
        ("scorched",         0.12, "char"),
        ("roof_collapsed",   0.35, "char"),
        ("partial_collapse", 0.55, "char"),
        ("burned_out",       0.75, "ash"),
        ("rubble",           0.92, "ash"),
    ),
    # `vegetation.level_for_age`'s rungs. `fallen` and `stump` are variants of
    # `snag` rather than rungs above it: `stand_outcome` rolls a minority of
    # snags down to them, which is why the mix is weighted to STANDING
    # outcomes and why a hotter fire does not produce more stumps.
    VEGETATION: _ladder(
        ("pristine", 0.00, None),
        ("scorched", 0.12, None),
        ("torched",  0.45, None),
        ("snag",     0.75, None, ("fallen", "stump")),
    ),
  },
  # Shaking fails a storey at a time, so the ladder is a collapse sequence.
  "earthquake": {
    STRUCTURE: _ladder(
        ("pristine",         0.00, None),
        ("cracked",          0.15, None),
        ("soft_storey",      0.40, None),
        ("partial_collapse", 0.65, None),
        ("pancaked",         0.88, None),
    ),
    # Shaking rarely fells a tree — `compile_earthquake` puts
    # `trees_toppled_fraction` at 0.0-0.1 — so one rung, and it is uprooting.
    VEGETATION: _ladder(
        ("pristine", 0.00, None),
        ("fallen",   0.55, None, ("stump",)),
    ),
  },
  # Wind strips before it breaks: the roof goes first and the walls last.
  "tornado": {
    STRUCTURE: _ladder(
        ("pristine",         0.00, None),
        ("roof_stripped",    0.20, None),
        ("roof_lost",        0.45, None),
        ("walls_breached",   0.70, None),
        ("swept_clean",      0.90, None),
    ),
    # Wind takes the canopy, then the trunk.
    VEGETATION: _ladder(
        ("pristine", 0.00, None),
        ("snag",     0.20, None),
        ("fallen",   0.50, None),
        ("stump",    0.85, None),
    ),
  },
  # The same failures as a tornado but never the top rung — a hurricane is
  # broad and even, and does not sweep a slab bare.
  "hurricane": {
    STRUCTURE: _ladder(
        ("pristine",         0.00, None),
        ("roof_stripped",    0.25, None),
        ("roof_lost",        0.55, None),
        ("walls_breached",   0.80, None),
    ),
    VEGETATION: _ladder(
        ("pristine", 0.00, None),
        ("snag",     0.25, None),
        ("fallen",   0.65, None),
    ),
  },
  # Water displaces and stains; it rarely takes a structure down. The ladder
  # is therefore short and its top rung is still a standing building.
  "flood": {
    STRUCTURE: _ladder(
        ("pristine",   0.00, None),
        ("waterline",  0.20, None),
        ("inundated",  0.50, None),
        ("undermined", 0.80, None),
    ),
    # Standing water undermines roots; it does not break a trunk.
    VEGETATION: _ladder(
        ("pristine", 0.00, None),
        ("fallen",   0.75, None),
    ),
  },
}

#: What a scene with no disaster gets. Also the fallback for an unknown type
#: or kind, so a typo degrades to "nothing happened" rather than raising
#: halfway through a bake that has already cost an hour.
NONE_LADDER = _ladder(("pristine", 0.00, None))


def ladder(disaster_type: str, kind: str = STRUCTURE) -> tuple:
    """The ordered rungs for *disaster_type* on an asset of *kind*."""
    bykind = LADDERS.get(str(disaster_type or "none").lower()) or {}
    return bykind.get(str(kind or STRUCTURE).lower(), NONE_LADDER)


def level_names(disaster_type: str, kind: str = STRUCTURE) -> list:
    """Just the rung names, in intensity order."""
    return [r.name for r in ladder(disaster_type, kind)]


def bake_levels(disaster_type: str, kind: str = STRUCTURE) -> list:
    """Every level Stage A must produce an archetype for.

    Rungs PLUS their variants, because a variant is a distinct asset even
    though it is not a distinct intensity — `stand_outcome` cannot roll a snag
    to `fallen` if nothing baked a fallen tree. `pristine` is included: the
    library is the one place an assembler looks, and special-casing the
    undamaged rung is how a scene ends up with a hole where a healthy tree
    should be.
    """
    out = []
    for r in ladder(disaster_type, kind):
        out.append(r.name)
        out.extend(r.variants)
    return out


def local_damage(field_value: float, severity: float) -> float:
    """Local damage at a position: the field's shape times how bad the event was.

    The one place this product is written down. It used to be an inline
    ``float(k) * sev`` in `disaster_stage`, with a twelve-line comment
    explaining why — because `compile_*` gives earthquake and hurricane a field
    that reads exactly 1.0 in the core at EVERY severity, so anything reading
    the field alone wrecked every building it touched at full strength and a
    0.2 quake produced the same ruins as a 1.0 one.
    """
    return max(0.0, min(1.0, float(field_value) * float(severity)))


def level_at(disaster_type: str, intensity: float,
             kind: str = STRUCTURE) -> Rung:
    """The rung *intensity* falls on. One function for every disaster.

    *intensity* is local damage — see `local_damage`, not a raw field value.
    Out-of-range values clamp rather than raise: a field that reads slightly
    over 1.0 is a tuning artefact, not a reason to fail a bake halfway through.
    """
    rungs = ladder(disaster_type, kind)
    i = max(0.0, min(1.0, float(intensity)))
    hit = rungs[0]
    for r in rungs:
        if i >= r.at:
            hit = r
        else:
            break
    return hit


def archetype_key(disaster_type: str, asset_type: str, level: str) -> str:
    """The Stage A filename stem: ``<type>_<level>``.

    The disaster is the DIRECTORY, not part of the stem, so one asset type's
    ladder can be compared across disasters by listing sibling folders.
    """
    return f"{asset_type}_{level}"


def level_for(disaster_type: str, field, x: float, y: float,
              severity: float, kind: str = STRUCTURE) -> Rung:
    """The rung the asset at ``(x, y)`` lands on. Stage B's entry point.

    Composes the three pieces so a caller never has to remember the order:
    evaluate the field, apply severity, quantise.
    """
    return level_at(disaster_type, local_damage(field(x, y), severity), kind)
