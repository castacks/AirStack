#!/usr/bin/env python3
"""
compile_disaster.py — turn a HIGH-LEVEL disaster spec into a LOW-LEVEL scene
config that scene_generator.py can build directly.

    presets/tornado.yaml          (high level: what happened, how bad)
              |
              |  compile_disaster.py
              v
    low_level/compiled/tornado.yaml   (low level: every knob the generator reads)
              |
              |  scene_generator.py
              v
    the scene

HIGH-LEVEL SPEC
---------------
    disaster-type: tornado    # none | earthquake | tornado | hurricane
                              # | fire | explosion | flood
    severity: 0.6             # 0..1 — 0 is pristine, 1 is as bad as that
                              #        disaster gets

    asset-pack: urban          # optional — which asset library to build with
                              #            (config/asset_packs/)
    seed: 42                  # optional — city layout + disaster RNG
    region_m: [400, 400]      # optional — city extent

    # optional, disaster-specific (sensible defaults derived from the region):
    epicenter: [0, 0]         # earthquake / explosion / fire — where it struck
    heading_deg: 35           # tornado / hurricane — direction of travel

    # optional, Stage C (the people to be found — see targets.py):
    occupancy: day            # night | day | commute — time-of-day population split
    target-seed: 7            # re-roll the victims without moving the city

    overrides:                # optional escape hatch, deep-merged last:
      packing:                # any low-level setting, verbatim
        min_parks: 4

Compilation is: base low-level config (low_level/default.yaml)
              + the disaster function's output
              + the spec's `overrides`
and the result is written as a self-contained low-level config, so a scene
stays reproducible from that one file even if default.yaml later changes.

ADDING A DISASTER
-----------------
Write ``compile_<name>(sev, spec, region) -> dict`` returning a ``disaster``
block (fates, debris, aftermath, field), then register it in DISASTERS. All
disaster knobs are *maxima* reached where ``disaster.field`` reads 1.0 — the
field is what gives each disaster its shape (uniform / radial / path).

USAGE
-----
    python3 compile_disaster.py                 # compile every preset
    python3 compile_disaster.py tornado         # one, by name
    python3 compile_disaster.py path/to/spec.yaml --output out.yaml
    python3 compile_disaster.py --list          # show disaster types

AS A LIBRARY
------------
``load_scene_config(path_or_name)`` accepts a config at either level and
returns a validated low-level dict — high-level specs are compiled in memory.
Launch scripts use it so SCENE_CONFIG can name either kind, or just a bare
name ("tornado") resolved against presets/ and low_level/compiled/.
"""

import argparse
import copy
import datetime
import os
import sys

import yaml

from compile_locale import (LOCALES, compile_locale_settings,
                            default_asset_pack)

_SCENE_GEN_DIR = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR = os.path.join(_SCENE_GEN_DIR, "config")
# Printed paths are relative to the repo root, so they read as
# "scene_gen/config/presets/tornado.yaml" wherever the CLI is run from.
_REPO_ROOT = os.path.dirname(_SCENE_GEN_DIR)
DEFAULT_BASE = os.path.join(_CONFIG_DIR, "low_level", "default.yaml")
DEFAULT_PRESET_DIR = os.path.join(_CONFIG_DIR, "presets")
DEFAULT_OUT_DIR = os.path.join(_CONFIG_DIR, "low_level", "compiled")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

from config_merge import deep_merge                    # noqa: E402,F401


def lerp(lo, hi, t):
    """Interpolate lo->hi over t in [0, 1], rounded to 3 decimals."""
    return round(lo + (hi - lo) * max(0.0, min(1.0, t)), 3)


def lerp_pair(lo_pair, hi_pair, t):
    """Interpolate a [min, max] count range, as ints."""
    return [int(round(lerp(lo_pair[0], hi_pair[0], t))),
            int(round(lerp(lo_pair[1], hi_pair[1], t)))]


def _ramp(sev, knee=0.0):
    """Severity remapped so it only starts biting past *knee* (0..1)."""
    if sev <= knee:
        return 0.0
    return (sev - knee) / (1.0 - knee)


# ---------------------------------------------------------------------------
# Disaster compilers — one per type
#
# Each returns the low-level ``disaster`` block for its type at severity
# *sev* (0..1). *spec* is the high-level dict (for epicenter / heading /
# per-disaster extras); *region* is (width_m, height_m).
#
# The shared vocabulary, so the types stay comparable:
#   damaged_/destroyed_fraction  structural loss
#   debris.*                     rubble volume, spread, and lean/sink
#   *_toppled_fraction           street furniture knocked down
#   *_toppled_fraction           street furniture and trees knocked down
#   debris.path_*_per_100m2      ground scour along the damage corridor —
#                                per 100 m2 of AFFECTED ground, so the number
#                                means the same for a narrow tornado track and
#                                a region-wide hurricane
#   *_scatter_m / *_strewn       how far light things were moved
#   humans_*                     casualties
#   field                        WHERE all of the above applies
# ---------------------------------------------------------------------------

def compile_none(sev, spec, region):
    """No disaster: a pristine city. Severity is ignored."""
    return {
        "damaged_fraction": 0.0,
        "destroyed_fraction": 0.0,
        "debris": {"shed_m3_per_m": 0.0,
                   "damaged_debris_scale": 0.0,
                   "tilt_chance": 0.0,
                   "path_pieces_per_100m2": 0.0,
                   "path_piles_per_100m2": 0.0},
        "trees_toppled_fraction": 0.0,
        "streetlights_toppled_fraction": 0.0,
        "traffic_lights_toppled_fraction": 0.0,
        "traffic_lights_leaning_fraction": 0.0,
        "trash_cans_toppled_fraction": 0.0,
        "trash_cans_scatter_m": 0.0,
        "cars_toppled_fraction": 0.0,
        "cars_strewn": [0, 0],
        "humans_prone_fraction": 0.0,
        "humans_strewn": [0, 0],
        "field": {"kind": "uniform", "inside": 0.0},
    }


#: Wall thickness for `mesh_damage.solidify`, which extrudes a building's shell
#: inward before anything breaks it. Every disaster gets the same block: the
#: value is a property of masonry, not of what knocked it down, and all five
#: profiles both punch holes and (flood aside) shatter — the two things a
#: zero-thickness shell renders as paper. The budget mirrors `fracture`'s,
#: because it is the same buildings and the same reason (see `solidify`).
#: WHAT THE BUILT FABRIC OF A LOCALE IS MADE OF, and how thick its walls are.
#:
#: A downtown block is masonry and concrete; a subdivision is timber frame,
#: and a rural one is timber too. That is a property of the PLACE and not of
#: what hit it — the same stud wall breaks into the same planks whether it was
#: shaken or blown down — so it is compiled off the locale and every disaster
#: reads it from one place (`mesh_damage.apply_to_stage`, which hands it to
#: the disaster's script).
#:
#: The wall figures MIRROR `disaster/mesh_damage.py`'s `MATERIALS` and are
#: repeated rather than imported: this compiler must run on a plain `python3`
#: with no numpy and no `pxr`, and `mesh_damage` needs both. `tests/
#: test_quake.py` holds the two tables to the same numbers.
LOCALE_MATERIAL = {
    "urban": ("masonry", 0.5),
    "suburban": ("timber", 0.15),
    "rural": ("timber", 0.15),
}


def thickness_block(max_buildings: int) -> dict:
    # 0.5 m, not 0.25: a fragment cut from a quarter-metre slab still
    # reads as a sheet at rubble scale. `max_span_frac` keeps it from
    # turning a window mullion into a block.
    return {"enabled": True, "wall_m": 0.5, "max_buildings": max_buildings}


def compile_earthquake(sev, spec, region):
    """Ground shaking: structures fail in place.

    Signature — buildings pancake, lean and sink (liquefaction); rubble drops
    straight down at the facades rather than flying; poles shake down but
    nothing is blown anywhere, so light objects tip over where they stood.
    Attenuates radially from the epicenter over a wide radius.
    """
    w, h = region
    cx, cy = spec.get("epicenter", [0.0, 0.0])
    return {
        # Mesh damage. WHAT it does — where the building fails, how finely
        # it comes apart and where the pieces go — is the FAILURE FIELD in
        # `mesh_damage`, keyed off this type, because it differs in kind
        # between the disasters and this block was being copied identically
        # into every one of them. What is left here is the budget: how many
        # buildings can afford to be broken.
        "mesh_damage": {"fracture": {"enabled": True,
                                     "max_buildings": 60},
                        "thickness": thickness_block(60)},
        "damaged_fraction": lerp(0.05, 0.35, sev),
        "destroyed_fraction": lerp(0.02, 0.55, sev),
        "debris": {
            # Collapse rubble: lots of it, piled tight against the ruin, and
            # the most of any type. `shed_m3_per_m` is cubic metres per metre
            # of the building's own perimeter, at total collapse;
            # `debris.budget_m3` turns it into a budget and the piece count
            # follows from the piece size. See `disaster/debris.py` for why an
            # amount and not a count.
            "shed_m3_per_m": lerp(0.25, 0.7, sev),
            "spread_m": lerp(3.0, 7.0, sev),           # gravity, not wind
            "pile_share": 0.55,                        # mounds, not scatter
            # Rubble around buildings that are damaged but still
            # standing, relative to a destroyed one. Not zero: a
            # cracked, half-collapsed building on a spotless lot is
            # the most obviously wrong thing in an aerial view.
            "damaged_debris_scale": lerp(0.35, 0.55, sev),
            # Leaning/sinking is the earthquake tell.
            "tilt_chance": lerp(0.25, 0.7, sev),
            "tilt_deg": [3, lerp(7.0, 12.0, sev)],
            "sink_m": [0.5, lerp(1.2, 2.2, sev)],
        },
        # SHAKING DOES NOT FELL TREES — see `levels.LADDERS["earthquake"]`.
        "trees_toppled_fraction": 0.0,
        "streetlights_toppled_fraction": lerp(0.05, 0.5, sev),
        "traffic_lights_toppled_fraction": lerp(0.04, 0.45, sev),
        "traffic_lights_leaning_fraction": lerp(0.15, 0.5, sev),
        "traffic_lights_lean_deg": [8, lerp(20.0, 38.0, sev)],
        "trash_cans_toppled_fraction": lerp(0.25, 0.8, sev),
        "trash_cans_scatter_m": lerp(0.5, 2.0, sev),   # tip, don't fly
        "cars_toppled_fraction": lerp(0.01, 0.15, sev),  # crushed, not flipped
        "cars_strewn": lerp_pair([0, 1], [2, 5], sev),
        "strewn_topple_fraction": 0.4,
        "humans_prone_fraction": lerp(0.05, 0.55, sev),
        "humans_strewn": lerp_pair([0, 2], [4, 10], sev),
        # Wide radial attenuation — the whole city feels it, the epicenter
        # feels it worst. Never fully zero anywhere.
        "field": {
            "kind": "radial",
            "center": [float(cx), float(cy)],
            "radius_m": round(max(w, h) * lerp(0.15, 0.45, sev), 1),
            "falloff_m": round(max(w, h) * 0.55, 1),
            "inside": 1.0,
            "outside": lerp(0.1, 0.45, sev),
        },
    }


def compile_tornado(sev, spec, region):
    """A narrow track of extreme wind across an otherwise intact city.

    Signature — total destruction inside a corridor, near-nothing outside it.
    Everything light is thrown a long way: cans fly, cars are flipped and
    strewn, debris is scattered far downwind. Buildings are torn apart rather
    than settling, so tilt/sink stays low.
    """
    w, h = region
    return {
        # Mesh damage. WHAT it does — where the building fails, how finely
        # it comes apart and where the pieces go — is the FAILURE FIELD in
        # `mesh_damage`, keyed off this type, because it differs in kind
        # between the disasters and this block was being copied identically
        # into every one of them. What is left here is the budget: how many
        # buildings can afford to be broken.
        "mesh_damage": {"fracture": {"enabled": True,
                                     "max_buildings": 80},
                        "thickness": thickness_block(80)},
        "damaged_fraction": lerp(0.1, 0.3, sev),
        # -> 1.0 at sev=1: "total destruction in a corridor" per the docstring
        # above means everything on the track's centerline is destroyed, not
        # 65% of it. Below sev=1 this still leaves the damaged/intact bands
        # doing their job away from the centerline and near the corridor edge.
        "destroyed_fraction": lerp(0.15, 1.0, sev),
        "debris": {
            # Signature: many fragments, thrown far. Low `pile_share` is
            # what makes it a debris FIELD rather than a heap — a storm sorts
            # by weight and carries the light half away from the lot. The
            # AMOUNT is no higher than a quake's; only the reach is.
            "shed_m3_per_m": lerp(0.2, 0.6, sev),
            "spread_m": lerp(6.0, 18.0, sev),
            "pile_share": 0.3,
            # Rubble around buildings that are damaged but still
            # standing, relative to a destroyed one. Not zero: a
            # cracked, half-collapsed building on a spotless lot is
            # the most obviously wrong thing in an aerial view.
            "damaged_debris_scale": lerp(0.4, 0.6, sev),
            "tilt_chance": lerp(0.05, 0.2, sev),    # ripped, not sunk
            "tilt_deg": [2, 5],
            "sink_m": [0.2, 0.6],
            # The track itself, not just the lots on it: a continuous band of
            # dirt and splintered wood dragged across everything in the
            # corridor. This is the tornado's most recognisable feature from
            # the air, and the densest scour of any disaster type.
            "path_pieces_per_100m2": lerp(0.8, 3.5, sev),
            "path_piles_per_100m2": lerp(0.15, 0.7, sev),
            "path_min_intensity": 0.2,
        },
        "trees_toppled_fraction": lerp(0.4, 0.95, sev),
        "streetlights_toppled_fraction": lerp(0.15, 0.85, sev),
        "traffic_lights_toppled_fraction": lerp(0.12, 0.8, sev),
        "traffic_lights_leaning_fraction": lerp(0.2, 0.6, sev),
        "traffic_lights_lean_deg": [12, lerp(28.0, 45.0, sev)],
        "trash_cans_toppled_fraction": lerp(0.4, 0.95, sev),
        "trash_cans_scatter_m": lerp(4.0, 14.0, sev),   # flung
        "cars_toppled_fraction": lerp(0.1, 0.6, sev),
        "cars_strewn": lerp_pair([2, 5], [10, 20], sev),
        "strewn_topple_fraction": 0.85,
        "humans_prone_fraction": lerp(0.1, 0.5, sev),
        "humans_strewn": lerp_pair([1, 3], [5, 12], sev),
        # The track: a corridor across the region, sharp edges, nothing
        # outside it. Wider and less sharply bounded as severity climbs.
        "field": {
            "kind": "path",
            "heading_deg": float(spec.get("heading_deg", 35.0)),
            "width_m": round(max(w, h) * lerp(0.08, 0.3, sev), 1),
            "falloff_m": round(max(w, h) * 0.08, 1),
            "inside": 1.0,
            "outside": 0.0,
        },
    }


def compile_fire(sev, spec, region):
    """A conflagration: gutted inside the burn scar, untouched a street away.

    Signature — the sharpest PERIMETER of any type, as opposed to the
    explosion's sharpest gradient. A fire burns until something stops it, so
    within the scar every building is a shell and just outside it the houses
    are fine; there is no long tail of diminishing damage. Nothing is thrown
    anywhere — fire moves no mass — so scatter distances are the lowest of any
    type and debris drops at the facades. Vegetation is what carries it, so
    trees go almost completely while steel poles stand.
    """
    w, h = region
    cx, cy = spec.get("epicenter", [0.0, 0.0])
    return {
        # The roof and the floors under it are consumed and drop straight in;
        # `mesh_damage.field_fire` owns that shape. Budget is high because a
        # burnt-out shell is cheap — only the top of each building is cut.
        "mesh_damage": {"fracture": {"enabled": True,
                                     "max_buildings": 70},
                        "thickness": thickness_block(70)},
        # Fire guts rather than flattens, so `damaged` (a standing shell) is
        # the common outcome and total loss stays comparatively rare.
        "damaged_fraction": lerp(0.15, 0.75, sev),
        "destroyed_fraction": lerp(0.02, 0.30, sev),
        "debris": {
            # Collapsed roof material and charred timber, at the facades.
            "shed_m3_per_m": lerp(0.2, 0.6, sev),
            "spread_m": lerp(2.0, 5.0, sev),           # fire moves no mass
            "pile_share": 0.5,
            # Rubble around buildings that are damaged but still
            # standing, relative to a destroyed one. High here: a gutted
            # shell IS the common outcome, and it drops its whole roof.
            "damaged_debris_scale": lerp(0.45, 0.7, sev),
            "tilt_chance": lerp(0.05, 0.2, sev),       # burnt, not undermined
            "tilt_deg": [2, 5],
            "sink_m": [0.2, 0.6],
            # A thin scatter of ash and burnt debris over the whole scar.
            "path_pieces_per_100m2": lerp(0.3, 1.2, sev),
            "path_piles_per_100m2": lerp(0.05, 0.3, sev),
            "path_min_intensity": 0.3,
        },
        "trees_toppled_fraction": lerp(0.5, 0.95, sev),   # the fuel
        "streetlights_toppled_fraction": lerp(0.02, 0.12, sev),  # steel stands
        "traffic_lights_toppled_fraction": lerp(0.02, 0.1, sev),
        "traffic_lights_leaning_fraction": lerp(0.05, 0.2, sev),
        "traffic_lights_lean_deg": [5, 15],
        "trash_cans_toppled_fraction": lerp(0.3, 0.7, sev),   # melted in place
        "trash_cans_scatter_m": lerp(0.2, 0.8, sev),
        "cars_toppled_fraction": lerp(0.01, 0.08, sev),   # burnt where parked
        "cars_strewn": lerp_pair([0, 1], [1, 3], sev),
        "strewn_topple_fraction": 0.15,
        "humans_prone_fraction": lerp(0.1, 0.6, sev),
        "humans_strewn": lerp_pair([0, 2], [3, 8], sev),
        # The burn scar: a wide core at full strength with a SHORT falloff and
        # nothing outside — a perimeter, not a gradient. That short falloff
        # against a large radius is what separates it from `explosion`, which
        # is a small radius against a long one.
        "field": {
            "kind": "radial",
            "center": [float(cx), float(cy)],
            "radius_m": round(max(w, h) * lerp(0.12, 0.38, sev), 1),
            "falloff_m": round(max(w, h) * 0.05, 1),
            "inside": 1.0,
            "outside": 0.0,
        },
        # What the fire looks like while it is still burning. Inert unless the
        # launch script calls `disaster.fire.apply_wildfire`, so an urban fire
        # scene that only wants the aftermath simply never reads it.
        "fire": _fire_block(sev, spec, region),
    }


def compile_flood(sev, spec, region):
    """Water came through: little structural loss, everything light displaced.

    Signature — buildings mostly stand (some undermined), but anything that
    floats has been carried off and dumped: cars strewn and rolled, bins
    washed away, debris deposited in the streets. No wind, so poles stand.
    """
    w, h = region
    return {
        "damaged_fraction": lerp(0.05, 0.35, sev),
        "destroyed_fraction": lerp(0.0, 0.12, sev),
        "debris": {
            # Deposited rather than dropped, and floated outward — so much
            # less material than a collapse leaves, over a wider ring.
            "shed_m3_per_m": lerp(0.1, 0.3, sev),
            "spread_m": lerp(5.0, 12.0, sev),
            "pile_share": 0.5,
            # Rubble around buildings that are damaged but still
            # standing, relative to a destroyed one. Not zero: a
            # cracked, half-collapsed building on a spotless lot is
            # the most obviously wrong thing in an aerial view.
            "damaged_debris_scale": lerp(0.25, 0.4, sev),
            "tilt_chance": lerp(0.1, 0.4, sev),         # undermined footings
            "tilt_deg": [2, lerp(5.0, 9.0, sev)],
            "sink_m": [0.3, lerp(0.8, 1.5, sev)],
        },
        "streetlights_toppled_fraction": lerp(0.02, 0.15, sev),   # poles hold
        "traffic_lights_toppled_fraction": lerp(0.02, 0.12, sev),
        "traffic_lights_leaning_fraction": lerp(0.05, 0.3, sev),
        "traffic_lights_lean_deg": [5, 20],
        "trash_cans_toppled_fraction": lerp(0.5, 0.95, sev),      # all float
        "trash_cans_scatter_m": lerp(5.0, 15.0, sev),
        "cars_toppled_fraction": lerp(0.1, 0.45, sev),            # rolled
        "cars_strewn": lerp_pair([2, 5], [10, 18], sev),
        "strewn_topple_fraction": 0.5,
        "humans_prone_fraction": lerp(0.1, 0.45, sev),
        "humans_strewn": lerp_pair([1, 3], [5, 12], sev),
        # Broad and even — water finds everywhere, so only a gentle radial
        # bias toward the low ground at the epicenter.
        "field": {
            "kind": "radial",
            "center": [float(c) for c in spec.get("epicenter", [0.0, 0.0])],
            "radius_m": round(max(w, h) * 0.5, 1),
            "falloff_m": round(max(w, h) * 0.5, 1),
            "inside": 1.0,
            "outside": lerp(0.3, 0.7, sev),
        },
    }


def compile_hurricane(sev, spec, region):
    """City-wide wind and rain: broad, even damage without a sharp edge.

    Signature — tornado-like damage mechanisms (things blown over and moved)
    but spread across the whole region at lower intensity, with no untouched
    zone and no narrow track.
    """
    return {
        # Mesh damage. WHAT it does — where the building fails, how finely
        # it comes apart and where the pieces go — is the FAILURE FIELD in
        # `mesh_damage`, keyed off this type, because it differs in kind
        # between the disasters and this block was being copied identically
        # into every one of them. What is left here is the budget: how many
        # buildings can afford to be broken.
        "mesh_damage": {"fracture": {"enabled": True,
                                     "max_buildings": 60},
                        "thickness": thickness_block(60)},
        "damaged_fraction": lerp(0.08, 0.4, sev),
        "destroyed_fraction": lerp(0.03, 0.3, sev),
        "debris": {
            "shed_m3_per_m": lerp(0.18, 0.55, sev),
            "spread_m": lerp(5.0, 12.0, sev),
            "pile_share": 0.4,
            # Rubble around buildings that are damaged but still
            # standing, relative to a destroyed one. Not zero: a
            # cracked, half-collapsed building on a spotless lot is
            # the most obviously wrong thing in an aerial view.
            "damaged_debris_scale": lerp(0.3, 0.5, sev),
            "tilt_chance": lerp(0.1, 0.3, sev),
            "tilt_deg": [2, 7],
            "sink_m": [0.3, 0.9],
            # Uniform field: thin scour everywhere rather than a band.
            "path_pieces_per_100m2": lerp(0.2, 0.9, sev),
            "path_piles_per_100m2": lerp(0.05, 0.25, sev),
        },
        "streetlights_toppled_fraction": lerp(0.1, 0.6, sev),
        "traffic_lights_toppled_fraction": lerp(0.08, 0.5, sev),
        "traffic_lights_leaning_fraction": lerp(0.25, 0.6, sev),
        "traffic_lights_lean_deg": [10, lerp(25.0, 40.0, sev)],
        "trash_cans_toppled_fraction": lerp(0.45, 0.9, sev),
        "trash_cans_scatter_m": lerp(3.0, 10.0, sev),
        "cars_toppled_fraction": lerp(0.05, 0.35, sev),
        "cars_strewn": lerp_pair([1, 3], [6, 12], sev),
        "strewn_topple_fraction": 0.7,
        "humans_prone_fraction": lerp(0.08, 0.45, sev),
        "humans_strewn": lerp_pair([0, 2], [4, 10], sev),
        # Uniform: the whole region is in the storm.
        "field": {"kind": "uniform", "inside": 1.0},
    }


def _fire_block(sev, spec, region):
    """The NVIDIA Flow fire front: emitters, spread rates and the burn window.

    Split out of what used to be a separate `wildfire` disaster type. That type
    was `compile_none` plus this block — the fire with every structural knob
    deliberately zeroed, "so a wildfire scene can be looked at and tuned before
    any question of what the fire does to the buildings is opened". That
    question is open now, so the two halves are one disaster: `compile_fire`
    supplies what the fire DOES and this supplies what it LOOKS like.

    Consumed by `disaster.fire.apply_wildfire`, which runs AFTER generation:
    an emitter is fitted to the bounding box of a real prim.
    """
    w, h = region
    span = max(w, h)

    # Ignition point and wind direction reuse the shared spec keys, so a
    # wildfire is steered the same way a tornado or hurricane is.
    ox, oy = spec.get("epicenter", [-span * 0.35, -span * 0.35])
    heading = float(spec.get("heading_deg", 45.0))

    return {
        "enabled": True,
        "origin_m": [float(ox), float(oy)],
        "heading_deg": heading,

        # Rate of spread. A creeping surface fire at severity 0, a running
        # crown fire at 1. Severity makes the fire faster, not rounder: the
        # 4:1 head-to-flank ratio holds across the range.
        #
        # 4:1 rather than the 6:1 of an extreme wind event, chosen by measuring
        # rather than by taste — at 6:1 the burnt region on this plat is a
        # ~330 m cigar that reads as scattered burning trees from the air,
        # and 4:1 doubles the fuel inside the front while still being visibly
        # wind-driven. `tools/fire_png.py` is how that was compared.
        "head_mps": lerp(0.25, 2.0, sev),
        "flank_mps": round(lerp(0.25, 2.0, sev) / 4.0, 4),
        "back_mps": round(lerp(0.25, 2.0, sev) / 12.0, 4),
        "wind_mps": lerp(1.5, 9.0, sev),

        # How far the front runs, and how much of that it has already done
        # when the scene opens.
        #
        # Both are fractions of the plat rather than absolute seconds, because
        # the thing being controlled is what you SEE. A front let loose for the
        # full crossing spends its emitter budget over 1.5 km and reads as
        # scattered burning trees; held to part of it, the same budget buys a
        # dense band. The offset then guarantees fire on frame one — a scene
        # that opens with six minutes of nothing is not a wildfire scene.
        "duration_s": round(span * lerp(0.50, 1.00, sev)
                            / max(1e-6, lerp(0.25, 2.0, sev)), 1),
        # THIS IS A POST-DISASTER SCENE. The offset is most of the way through
        # the burn on purpose: the front has already crossed the bulk of the
        # plat, so the majority of fuels open in smoulder or residual and only
        # a trailing band is still active. A low offset gives an advancing
        # firestorm, which is a different scene.
        "start_offset_frac": round(lerp(0.92, 0.70, sev), 3),

        # How much of the burn is still genuinely alight. The rest smokes and
        # smoulders — remnants, not flames. Severity is what moves this.
        "flaming_fraction": round(lerp(0.07, 0.30, sev), 3),
        "intensity_range": [round(lerp(0.15, 0.35, sev), 3), 1.0],
        "residual_smoke_frac": round(lerp(0.16, 0.10, sev), 3),

        # Embers. A severe fire throws more of them, and further.
        "spot_chance": lerp(0.01, 0.09, sev),
        "spot_lead_s": lerp(30.0, 150.0, sev),
        "jitter_s": lerp(4.0, 14.0, sev),

        # How long one fuel spends in each phase. FLAMING is the only phase
        # that renders as fire — igniting and smouldering are smoke-only — so
        # this ratio is what decides whether the scene reads as a fire or as a
        # smoke machine. Flame now dominates; the earlier 126 s flame against
        # 228 s smoulder meant most of the front was smoke at any instant.
        "ignition_s": lerp(10.0, 3.0, sev),
        "flame_s": lerp(240.0, 150.0, sev),
        "smoulder_s": lerp(120.0, 60.0, sev),

        # Emitter budget. Only the emitters inside their burn window are
        # enabled, so the frame-rate number is the PEAK CONCURRENT count, not
        # this one — `tools/fire_png.py` prints both. Trees in this suburb sit
        # ~24 m apart, so the spacing knob only bites on denser planting.
        # Peak concurrent runs at roughly half of `max_emitters`, and this
        # suburb's ~24 m tree spacing means the cap does all the work while
        # `emitter_spacing_m` only bites on denser planting.
        "emitter_spacing_m": lerp(12.0, 6.0, sev),
        "max_emitters": int(round(lerp(90, 260, sev))),

        # Solver resolution, metres per voxel, and the Flow block pool.
        #
        # 0.1 rather than 0.25: a coarse cell averages the burn over so much
        # volume that temperature never reaches the flame band of the colormap
        # and the fire renders as grey smoke. 0.1 costs ~15x the blocks of
        # 0.25, which is what the pool below is for — one block is
        # 32x16x16 cells, so at 0.1 m it covers 8.2 cubic metres and a tree
        # fire with its plume runs to roughly 200 blocks.
        "density_cell_size_m": 0.1,
        "max_blocks": 16384,
        # NVIDIA's ramp, verbatim. Pull DOWN toward 0.3 if the flames are
        # too dim; that brings the orange band within reach of a lower
        # temperature, at the cost of blowing out a hot fire.
        "colormap_x_max": 1.0,
    }


#: The disaster types SPEC.md names. `none` is deliberately absent: a pristine
#: scene is severity 0 (or no `disaster-type` at all), not a sixth type — one
#: fewer thing to keep in sync, and it makes "pristine" a point on the severity
#: axis rather than a special case beside it.
DISASTERS = {
    "earthquake": compile_earthquake,
    "tornado": compile_tornado,
    "hurricane": compile_hurricane,
    "fire": compile_fire,
    "flood": compile_flood,
}


# ---------------------------------------------------------------------------
# Compilation
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# STAGE C — who is in the scene to be found
#
# The disaster compilers above decide what happened to the CITY. This decides
# what happened to the PEOPLE, and it is a separate axis: the population is a
# property of the place and the hour, while the disaster decides only where
# that population ends up. Hence no severity lerp here — `targets.py` holds N
# fixed across a severity sweep on purpose (see its docstring), so "how many
# did the search find" means the same thing at every severity.
#
# Weights follow the 2023 Türkiye casualty split (36.8% rescued from under
# rubble, 40.1% injured escaping, 19.8% recovered from under rubble) plus the
# post-quake convergence on open space that every account of a large urban
# earthquake describes. Sources are cited in `targets.py`.
# ---------------------------------------------------------------------------

TARGET_COHORTS = {
    # `in_vehicle` is the one share not in the Türkiye split, which counted
    # casualties by how they were RESCUED rather than where they were. It comes
    # out of `street`: someone stopped in traffic was caught outdoors in every
    # sense but the one that matters to a camera. Small by day and multiplied
    # by 2.5 at `occupancy: commute` (targets.OCCUPANCY).
    "earthquake": {"inside_rubble": 0.35, "in_vehicle": 0.03, "exit_ring": 0.20,
                   "street": 0.12, "open_space": 0.25, "rubble_edge": 0.05},
}

#: What "Stage C owns every human" costs the Stage B detail pass. Scenery
#: people are unlabelled, so leaving them in would put humans in the scene that
#: are not in the ground truth — every one of them a false positive a search
#: run would be penalised for finding.
_NO_SCENERY_HUMANS = {
    "detail": {"humans": {"per_block": [0, 0],
                          "sidewalk_spacing_m": 0.0,
                          "trail_spacing_m": 0.0}},
}


def compile_targets(dtype, sev, spec):
    """The low-level ``targets`` block for a disaster type at severity *sev*.

    Empty for a type with no cohort weights. NOT gated on severity, and that is
    load-bearing rather than tidy: `owns_humans` zeroes the Stage B scenery
    people, so gating it on severity would make a DETAIL-stage input a function
    of severity — the exact thing `tests/test_layout_decoupling.py` exists to
    forbid. It cost 100 of 128 benches moving between severity 0.0 and 0.4 on
    the first attempt, because the humans that vanished had been holding places
    in the shared occupancy grid.

    At severity 0 the field is flat zero, so nobody is trapped and the whole
    population ends up outdoors, alive: an undamaged city whose people are all
    labelled targets. That is the right answer for a control run.
    """
    cohorts = TARGET_COHORTS.get(dtype)
    if not cohorts:
        return {}
    out = {"cohorts": dict(cohorts), "owns_humans": True}
    for key, dest in (("occupancy", "occupancy"), ("target-seed", "seed"),
                      ("target_seed", "seed")):
        if key in spec:
            out[dest] = spec[key]
    return out


def compile_spec(spec: dict, base: dict) -> dict:
    """High-level *spec* + *base* low-level config -> low-level config."""
    dtype = str(spec.get("disaster-type",
                         spec.get("disaster_type", "none"))).lower()
    if dtype != "none" and dtype not in DISASTERS:
        raise ValueError(
            f"unknown disaster-type {dtype!r}; expected one of "
            f"{', '.join(sorted(DISASTERS))}, or omit it for a pristine scene")

    sev = float(spec.get("severity", 1.0))
    if not 0.0 <= sev <= 1.0:
        raise ValueError(f"severity must be in [0, 1], got {sev}")

    # Deep copy. NOT `yaml.safe_load(yaml.safe_dump(base))`: safe_dump sorts
    # keys by default, so that round-trip silently ALPHABETISES the base
    # config. The generator is order-sensitive — `city_detail` walks its
    # `categories` dict in order, placing furniture into a shared occupancy
    # grid, so whichever category comes first wins the contested kerb — which
    # means a setting produced a different scene depending on whether it was
    # authored in `default.yaml` (sorted) or in a preset's `overrides:`
    # (insertion order). That cost an afternoon; keep the copy order-preserving.
    cfg = copy.deepcopy(base)

    # City-level passthroughs, before the disaster reads the region.
    if "seed" in spec:
        cfg["seed"] = spec["seed"]
    if "region_m" in spec:
        cfg.setdefault("layout", {})["region_m"] = spec["region_m"]

    # ---- LOCALE axis: how the place is laid out (see compile_locale.py).
    # Applied before the disaster so a disaster's overrides still win, and
    # before `overrides:` so the user's escape hatch wins over both.
    locale = spec.get("locale", "urban")
    deep_merge(cfg, compile_locale_settings(locale, spec))
    cfg["locale"] = str(locale).lower()

    # An explicit asset-pack wins; otherwise the locale picks a sensible one
    # (urban -> urban art, suburban -> houses).
    asset_pack = spec.get("asset-pack", spec.get("asset_pack"))
    cfg["asset_pack"] = asset_pack or default_asset_pack(locale)

    region = tuple(float(v) for v in cfg["layout"]["region_m"])

    # Severity 0 means untouched, whatever the type claims to be — and no type
    # at all means the same thing.
    fn = DISASTERS[dtype] if (dtype != "none" and sev > 0.0) else compile_none
    cfg["disaster"] = fn(sev, spec, region)
    # The compiled config drops `disaster-type` (it is a high-level key),
    # which left the low level unable to say what had happened to it —
    # `mesh_damage` needs the type to pick a deformation profile. Record
    # it inside the block it describes, so the artifact is self-contained.
    cfg["disaster"]["type"] = dtype
    cfg["disaster"]["severity"] = sev

    # ---- WHAT THE BUILDINGS ARE MADE OF (see `LOCALE_MATERIAL`). Written
    # after the disaster block because the disaster owns the budgets and the
    # locale owns the fabric, and a suburb of timber houses thickened to a
    # half-metre masonry wall is the single most visible way an urban-tuned
    # ladder goes wrong on a house.
    mat, wall_m = LOCALE_MATERIAL.get(cfg["locale"], LOCALE_MATERIAL["urban"])
    mdb = cfg["disaster"].get("mesh_damage")
    if mdb:
        mdb["material"] = mat
        if mdb.get("thickness"):
            mdb["thickness"]["wall_m"] = wall_m

    # ---- STAGE C axis: the people. Separate from the damage model, and
    # before `overrides:` so the escape hatch still wins over it.
    tgt = compile_targets(dtype, sev, spec)
    if tgt:
        deep_merge(cfg.setdefault("targets", {}), tgt)
        if cfg["targets"].get("owns_humans"):
            deep_merge(cfg, copy.deepcopy(_NO_SCENERY_HUMANS))
            cfg["disaster"]["humans_prone_fraction"] = 0.0
            cfg["disaster"]["humans_strewn"] = [0, 0]

    # Escape hatch: raw low-level overrides win over everything.
    if spec.get("overrides"):
        # Restaged first: `overrides:` is authored flat (one line per knob) and
        # the config it merges into is grouped by stage, so a raw merge would
        # bury a second, shadow copy of e.g. `packing` at the top level where
        # nothing reads it.
        import scene_generator
        deep_merge(cfg, scene_generator.restage(spec["overrides"]))

    # Compilation is a build step; leaving the high-level keys in the output
    # would suggest editing them there has an effect. Keep them only as
    # provenance in the header comment.
    for k in ("disaster-type", "disaster_type", "severity", "overrides",
              "epicenter", "heading_deg", "asset-pack",
              "occupancy", "target-seed", "target_seed",
              "presets_path",
              "preset_file"):
        cfg.pop(k, None)
    return cfg


#: Top-level keys only a compiled low-level config carries. `default.yaml` is
#: grouped by stage, so these three ARE the low level's shape.
_LOW_LEVEL_KEYS = ("layout", "detail", "disaster")


def is_high_level(cfg: dict) -> bool:
    """True if *cfg* is a high-level scene spec rather than a compiled config.

    Tests for the low level's SHAPE rather than for one high-level key.
    Keying off `disaster-type` was fine only while every spec had to carry one;
    it is optional now (omitting it means a pristine scene), so a pristine
    preset would otherwise be mistaken for an already-compiled config and fed
    to the generator raw.
    """
    if any(k in cfg for k in _LOW_LEVEL_KEYS):
        return False
    return bool(cfg.get("disaster-type") or cfg.get("disaster_type")
                or cfg.get("locale") or cfg.get("severity") is not None
                or cfg.get("asset-pack") or cfg.get("overrides"))


def resolve_config_path(name_or_path: str) -> str:
    """Accept a path, or a bare name looked up in the known config dirs.

    Search order: the path as given, then ``presets/`` (high level), then
    ``low_level/compiled/`` and ``low_level/`` (low level), each with and
    without a ``.yaml`` / ``.yml`` suffix. Raises FileNotFoundError listing
    what *is* available, so a typo says so instead of failing obscurely.
    """
    if os.path.isfile(name_or_path):
        return name_or_path

    search = [DEFAULT_PRESET_DIR, DEFAULT_OUT_DIR,
              os.path.join(_CONFIG_DIR, "low_level")]
    stem = os.path.basename(name_or_path)
    for d in search:
        for cand in (os.path.join(d, stem),
                     os.path.join(d, stem + ".yaml"),
                     os.path.join(d, stem + ".yml")):
            if os.path.isfile(cand):
                return cand

    available = []
    for d in search:
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if f.endswith((".yaml", ".yml")):
                available.append(f"    {os.path.relpath(os.path.join(d, f), _REPO_ROOT)}")
    raise FileNotFoundError(
        f"scene config not found: {name_or_path!r}\n"
        "  available:\n" + "\n".join(available))


def load_scene_config(name_or_path: str, base_path: str = None) -> dict:
    """Load a scene config at **either** level and return a low-level dict.

    A high-level disaster spec is compiled in memory against *base_path*
    (default ``low_level/default.yaml``); a low-level config is returned as
    is. Either way the result is validated the way scene_generator expects,
    so launch scripts can point ``SCENE_CONFIG`` at whichever is convenient:

        SCENE_CONFIG = ".../presets/tornado.yaml"            # high level
        SCENE_CONFIG = ".../low_level/compiled/tornado.yaml" # low level
        SCENE_CONFIG = "tornado"                             # by name

    Compiling in memory means a high-level spec always reflects the current
    ``default.yaml``, with no stale compiled artifact in between — at the
    cost of no on-disk record of what ran. Run ``compile_disaster.py`` to
    get that record.
    """
    path = resolve_config_path(name_or_path)
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError(f"{path}: config must be a mapping")

    if is_high_level(cfg):
        base_path = base_path or DEFAULT_BASE
        with open(base_path) as f:
            base = yaml.safe_load(f)
        cfg = compile_spec(cfg, base)
        print(f"[compile_disaster] compiled high-level spec in memory: "
              f"{os.path.relpath(path, _REPO_ROOT)} "
              f"(base {os.path.relpath(base_path, _REPO_ROOT)})")
    else:
        print(f"[compile_disaster] loaded low-level config: "
              f"{os.path.relpath(path, _REPO_ROOT)}")

    # Which config this is. Nothing downstream can otherwise tell — a compiled
    # config carries its asset pack and its disaster type but not its own name,
    # so tools that write a file per scene had to guess and named a `urban`
    # run after its asset pack, `urban`. Underscore-prefixed like every other
    # internal marker (`_footprint_m`, `_mesh_damage`), so it never collides
    # with a generator setting.
    cfg["_name"] = os.path.splitext(os.path.basename(path))[0]

    # Lazy so `--list` and plain compilation stay free of the pxr dependency.
    import scene_generator
    cfg = scene_generator.resolve_asset_pack(cfg, path)
    return scene_generator.validate_config(cfg, path)


def _header(spec: dict, source: str, base: str) -> str:
    dtype = spec.get("disaster-type", spec.get("disaster_type", "none"))
    sev = spec.get("severity", 1.0)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    extras = {k: v for k, v in spec.items()
              if k not in ("disaster-type", "disaster_type", "severity",
                           "locale")}
    lines = [
        "# GENERATED FILE — do not edit by hand.",
        "#",
        "# Low-level scene config compiled by scene_gen/compile_disaster.py.",
        "# Edit the high-level spec (or low_level/default.yaml) and recompile:",
        f"#     python3 compile_disaster.py {os.path.basename(source)}",
        "#",
        f"# locale        : {spec.get('locale', 'urban')}",
        f"# disaster-type : {dtype}",
        f"# severity      : {sev}",
        f"# spec          : {source}",
        f"# base          : {base}",
        f"# compiled      : {stamp}",
    ]
    if extras:
        lines.append(f"# spec extras   : {extras}")
    return "\n".join(lines) + "\n\n"


def compile_file(spec_path: str, base_path: str, out_path: str) -> str:
    with open(spec_path) as f:
        spec = yaml.safe_load(f) or {}
    if not isinstance(spec, dict):
        raise ValueError(f"{spec_path}: high-level spec must be a mapping")
    with open(base_path) as f:
        base = yaml.safe_load(f)

    cfg = compile_spec(spec, base)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(_header(spec, os.path.relpath(spec_path, _REPO_ROOT),
                        os.path.relpath(base_path, _REPO_ROOT)))
        yaml.safe_dump(cfg, f, sort_keys=False, default_flow_style=False,
                       width=100)
    return out_path


def main():
    ap = argparse.ArgumentParser(
        description="Compile high-level disaster specs into low-level scene configs.")
    ap.add_argument("specs", nargs="*",
                    help="high-level spec paths or bare names (default: every "
                         f"*.yaml in {os.path.relpath(DEFAULT_PRESET_DIR, _REPO_ROOT)})")
    ap.add_argument("--base", default=DEFAULT_BASE,
                    help="low-level base config to build on")
    ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR,
                    help="where compiled configs are written")
    ap.add_argument("--output", default=None,
                    help="explicit output path (single spec only)")
    ap.add_argument("--list", action="store_true",
                    help="list the known disaster types and exit")
    args = ap.parse_args()

    if args.list:
        print("disaster types:")
        for name, fn in sorted(DISASTERS.items()):
            summary = (fn.__doc__ or "").strip().splitlines()[0]
            print(f"  {name:<11} {summary}")
        return

    specs = args.specs
    if not specs:
        specs = sorted(os.path.join(DEFAULT_PRESET_DIR, f)
                       for f in os.listdir(DEFAULT_PRESET_DIR)
                       if f.endswith((".yaml", ".yml")))
        if not specs:
            raise SystemExit(f"no specs found in {DEFAULT_PRESET_DIR}")
    else:
        # Accept bare names ("tornado") as well as paths.
        resolved = []
        for s in specs:
            if os.path.isfile(s):
                resolved.append(s)
                continue
            cand = os.path.join(DEFAULT_PRESET_DIR, s)
            for c in (cand, cand + ".yaml", cand + ".yml"):
                if os.path.isfile(c):
                    resolved.append(c)
                    break
            else:
                raise SystemExit(f"spec not found: {s}")
        specs = resolved

    if args.output and len(specs) > 1:
        raise SystemExit("--output takes a single spec")

    for spec_path in specs:
        name = os.path.splitext(os.path.basename(spec_path))[0]
        out = args.output or os.path.join(args.out_dir, f"{name}.yaml")
        compile_file(spec_path, args.base, out)
        print(f"[compile_disaster] {os.path.relpath(spec_path, _REPO_ROOT)}"
              f"  ->  {os.path.relpath(out, _REPO_ROOT)}")


if __name__ == "__main__":
    sys.exit(main())
