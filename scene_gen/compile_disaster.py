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
    disaster-type: tornado    # none | earthquake | tornado | explosion
                              # | flood | hurricane | wildfire
    severity: 0.6             # 0..1 — 0 is pristine, 1 is as bad as that
                              #        disaster gets

    asset-set: urban          # optional — which asset library to build with
                              #            (config/asset_sets/)
    seed: 42                  # optional — city layout + disaster RNG
    region_m: [400, 400]      # optional — city extent

    # optional, disaster-specific (sensible defaults derived from the region):
    epicenter: [0, 0]         # earthquake / explosion — where it struck;
                              #   for a wildfire, where it was set
    heading_deg: 35           # tornado / hurricane / wildfire — direction
                              #   of travel; for a wildfire, the wind

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
                            default_asset_set)

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

def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge *override* into *base* in place: nested dicts merge
    key-by-key, everything else (scalars, lists) is replaced outright."""
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v
    return base


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
        "debris": {"piles_per_building": [0, 0],
                   "pieces_per_building": [0, 0],
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


def magnitude_to_severity(mag):
    """Moment/Richter magnitude of a SHALLOW crustal event with the city at
    its epicentre -> the 0..1 severity this compiler works in.

    Two published relations, chained:
      * epicentral intensity   I0 = 1.5 * M - 1.5   (Gutenberg & Richter
        1956, M = 2/3 I0 + 1; ~10 km focal depth), capped at XII;
      * EMS-98 damage: intensity VI = DG1-2 in URM, VII = DG2-3 (parapet and
        chimney falls, Lorca 2011 M5.1 / Napa 2014 M6.0), VIII = DG3-4
        (Christchurch 2011 M6.2 URM), IX-X = DG4-5 widespread (Kobe 1995
        M6.9 core, Izmit 1999 M7.6, Antakya 2023 M7.8), XI-XII = total.
    The ladder cuts in quake_flow are calibrated at intensity X-XI, so
    severity 1.0 == I0 >= XI and severity ~0.3 == I0 VII.

        M   5.0  5.5  6.0  6.5  7.0  7.5  8.0  8.5+
        I0  VI   VI+  VII+ VIII IX   IX+  X+   XI-XII
        sev .08  .29  .42  .54  .67  .79  .92  1.0
    """
    i0 = min(12.0, 1.5 * float(mag) - 1.5)
    return max(0.05, min(1.0, (i0 - 5.0) / 6.0))


def compile_earthquake(sev, spec, region):
    """Ground shaking: structures fail in place.

    Signature — buildings pancake, lean and sink (liquefaction); rubble drops
    straight down at the facades rather than flying; poles shake down but
    nothing is blown anywhere, so light objects tip over where they stood.
    Attenuates radially from the epicenter over a wide radius.

    `spec.magnitude` (optional) additionally shapes the FIELD: a great
    earthquake (M >= 7.5) has a rupture tens to hundreds of km long, so a
    200-1000 m plate is shaken uniformly — the core widens to ~60 % of the
    plate and the far corners keep ~85 %. A moderate event (M 5-6.5) has a
    km-scale source and a real gradient across a district (Christchurch: CBD
    intensity IX, 3 km away VII), which the radial field models. Liquefaction
    needs duration: essentially absent below M 5.5, widespread from M 7.5
    (Youd & Perkins 1978 magnitude scaling), so the soft-soil rate is scaled
    by `liq`.
    """
    w, h = region
    cx, cy = spec.get("epicenter", [0.0, 0.0])
    mag = spec.get("magnitude")
    if mag is not None:
        mag = float(mag)
        uni = max(0.0, min(1.0, (mag - 6.5) / 2.5))     # 0 at M6.5, 1 at M9
        liq = max(0.0, min(1.0, (mag - 5.5) / 2.0))     # 0 at M5.5, 1 at M7.5
        # a great earthquake is not "the worst block everywhere": Antakya /
        # Kahramanmaras 2023 (M7.8) collapsed ~10-15 % of the stock district-
        # wide, but M9-class shaking sustained for 3-5 minutes (Tohoku 2011,
        # Chile 1960) takes the URM stock to DG4-5 nearly everywhere. Over-
        # drive the grade draw past the ladder's calibration point.
        over = max(0.0, min(1.0, (mag - 8.0) / 1.5))    # 0 at M8, 1 at M9.5
        # research §13: shaking DURATION (M5-6 ~5-20 s, M6.5-7.5 ~20-60 s,
        # M8+ 2-6 min) cuts the collapse capacity of engineered frames
        # (-29 % median for a 42 s vs 6 s record); the boost multiplies the
        # DG4/DG5 share of rc / rc_glass in the grade draw, 1.0 at M6.5,
        # 2.0 at M8, 2.5 at M9+. Brittle URM is left alone.
        dur = 1.0 + 1.0 * max(0.0, min(1.0, (mag - 6.5) / 1.5)) \
            + 0.5 * max(0.0, min(1.0, (mag - 8.0) / 1.0))
    else:
        uni, liq, over, dur = 0.0, 1.0, 0.0, 1.0
    return {
        "damaged_fraction": lerp(0.05, 0.35, sev),
        "destroyed_fraction": lerp(0.02, 0.55, sev),
        "debris": {
            # Collapse rubble: lots of it, piled tight against the ruin.
            "piles_per_building": lerp_pair([1, 2], [4, 7], sev),
            "pile_max_offset_m": lerp(2.0, 4.0, sev),
            "pieces_per_building": lerp_pair([4, 8], [14, 26], sev),
            "pieces_scatter_m": lerp(3.0, 7.0, sev),   # gravity, not wind
            # Leaning/sinking is the earthquake tell.
            "tilt_chance": lerp(0.25, 0.7, sev),
            "tilt_deg": [3, lerp(7.0, 12.0, sev)],
            "sink_m": [0.5, lerp(1.2, 2.2, sev)],
            "lean_piles": lerp_pair([2, 3], [3, 5], sev),
        },
        "trees_toppled_fraction": lerp(0.0, 0.1, sev),   # shaking rarely fells a tree
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
        # Radial attenuation from the epicentre, AS FRACTIONS OF THE PLATE so
        # `region_m` and `severity` move it together: the full-intensity core
        # is 10-28 % of the plate's long side and the far corners keep 5-35 %.
        # (0.15-0.45 / 0.55 put an entire 250 m downtown at intensity 1.0 and
        # a quarter of its buildings pancaked; measured 2026-08-26.)
        "field": {
            "kind": "radial",
            "center": [float(cx), float(cy)],
            "radius_m": round(max(w, h) * lerp(lerp(0.10, 0.28, sev), 0.60, uni), 1),
            "falloff_m": round(max(w, h) * lerp(0.45, 0.60, uni), 1),
            "inside": 1.0,
            "outside": lerp(lerp(0.05, 0.35, sev), 0.85, uni),
        },
        # ---- the urban building pass (`disaster/quake.py`) reads these ----
        # `grade_scale` multiplies the field before the EMS-98 grade draw, so
        # severity compresses the ladder as well as shrinking the core.
        # Reaches 1.0 at severity ~0.8: the ladder cuts are calibrated at
        # intensity 1.0, and 0.93 at severity 0.85 drew ZERO pancakes on a
        # 47-building plate (city 9) where 1.0 draws ~4.
        # 0.3 + 0.85 sev: 0.55 at severity 0.29 (M5.5 -> URM DG3 at most,
        # no collapses), 1.0 from severity 0.82. The earlier 0.55-1.1 gave an
        # M5.5 core a 22 % URM collapse rate, which a Lorca never had.
        "grade_scale": min(1.0, 0.3 + 0.85 * sev) + 0.35 * over,
        "duration_boost": round(dur, 2),
        # The liquefaction patch: one ellipse, sized to the plate, placed
        # AWAY from the epicentre by default (the ground fails where the soil
        # is soft, not where the shaking is worst — and near the epicentre
        # nothing is left standing to tilt). `spec: soft-soil: false` turns
        # it off; `soft-soil: {center: [x, y], rx_m, ry_m, rate, angle_deg}`
        # overrides any part of it.
        "soft_soil": _soft_soil(sev, spec, region, liq),
        # Ground dust round DG4-5: reach in building heights, band opacity.
        "dust": {"reach_h5": lerp(0.7, 1.2, sev), "reach_h4": lerp(0.35, 0.6, sev),
                 "opacity_max": lerp(0.25, 0.45, sev)},
    }


def _soft_soil(sev, spec, region, liq=1.0):
    """The earthquake's soft-soil ellipse, from severity and the plate;
    `liq` (0..1, from the magnitude) scales the rate — no liquefaction in a
    short, small event."""
    w, h = region
    user = spec.get("soft-soil", spec.get("soft_soil"))
    if user is False:
        return False
    user = user if isinstance(user, dict) else {}
    cx, cy = spec.get("epicenter", [0.0, 0.0])
    # opposite quadrant to the epicentre, clamped inside the plate
    # centre 0.22 / radius 0.26 of the plate: the ellipse stays INSIDE the
    # plate (0.48 w), so its boils and fissures do not land in the void
    # beside a 200 m city (two-city run 2).
    dx = -0.22 * w if float(cx) >= 0.0 else 0.22 * w
    dy = 0.20 * h if float(cy) <= 0.0 else -0.20 * h
    out = {
        "center": [float(v) for v in user.get("center", [dx, dy])],
        "rx_m": float(user.get("rx_m", round(0.26 * w, 1))),
        "ry_m": float(user.get("ry_m", round(0.20 * h, 1))),
        "angle_deg": float(user.get("angle_deg", 25.0)),
        # share of the still-standing buildings inside it that settle / tilt
        "rate": float(user.get("rate", round(lerp(0.25, 0.85, sev) * liq, 2))),
    }
    return out


def compile_tornado(sev, spec, region):
    """A narrow track of extreme wind across an otherwise intact city.

    Signature — total destruction inside a corridor, near-nothing outside it.
    Everything light is thrown a long way: cans fly, cars are flipped and
    strewn, debris is scattered far downwind. Buildings are torn apart rather
    than settling, so tilt/sink stays low.
    """
    w, h = region
    # The track passes through `epicenter` if one is given, and through the
    # middle of the plate otherwise. Centre rather than a corner: a corridor
    # entering at a corner clips the thin end of the fabric and most of the
    # plate never sees it, which is the same mistake the wildfire preset
    # records about corner ignitions.
    ox, oy = spec.get("epicenter", [0.0, 0.0])
    out = {
        "damaged_fraction": lerp(0.1, 0.3, sev),
        # -> 1.0 at sev=1: "total destruction in a corridor" per the docstring
        # above means everything on the track's centerline is destroyed, not
        # 65% of it. Below sev=1 this still leaves the damaged/intact bands
        # doing their job away from the centerline and near the corridor edge.
        "destroyed_fraction": lerp(0.15, 1.0, sev),
        "debris": {
            "piles_per_building": lerp_pair([1, 2], [2, 4], sev),
            "pile_max_offset_m": lerp(2.0, 3.5, sev),
            # Signature: many fragments, thrown far.
            "pieces_per_building": lerp_pair([6, 12], [18, 34], sev),
            "pieces_scatter_m": lerp(6.0, 18.0, sev),
            "tilt_chance": lerp(0.05, 0.2, sev),    # ripped, not sunk
            "tilt_deg": [2, 5],
            "sink_m": [0.2, 0.6],
            "lean_piles": [1, 2],
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

        # THE TRACK AS THE DAMAGE PIPELINE READS IT, alongside `field` rather
        # than instead of it. `field` is the generic per-disaster mask the
        # scene generator applies to placement-level knobs; this block is the
        # tornado's own model, consumed by `disaster.tornado` and by the bake
        # and assembly launchers. Same relationship `fire` has to the wildfire
        # compiler's `field`, and for the same reason: the launchers need the
        # geometry of the event, not a scalar per placement.
        "tornado": {
            "enabled": True,
            "origin_m": [float(ox), float(oy)],
            "heading_deg": float(spec.get("heading_deg", 35.0)),

            # WIDTH IN METRES, NOT AS A FRACTION OF THE PLATE, and that is a
            # deliberate departure from `field.width_m` above. A tornado path
            # is a physical width — US significant tracks run a few hundred
            # metres — and scaling it with the plate means a 500 m scene and
            # a 1600 m one get corridors that damage the same PROPORTION of
            # the fabric, which is exactly wrong: the smaller scene should
            # show a narrower path through fewer houses, not the same picture
            # at a different zoom. Capped at 40% of the plate so a small
            # region still keeps intact suburb on both sides — a corridor
            # whose edges are off-frame is not a corridor. 0.32 rather than a
            # half: measured off `tornado.jpeg`, the swathe is roughly a
            # quarter to a third of the frame, and the intact fabric either
            # side is what makes it read as a track at all.
            "width_m": round(min(0.32 * min(w, h),
                                 lerp(80.0, 320.0, sev)), 1),
            "core_frac": lerp(0.22, 0.38, sev),
            "peak": lerp(0.45, 1.0, sev),

            # The meander and the breathing, both as fractions of the plate so
            # a track crossing a small scene still wanders visibly within it.
            # A WANDER, NOT A SNAKE. At a period of 0.7 of the plate the
            # centreline completes two full cycles across it and reads as a
            # drawn S-curve — the exact failure the wobble exists to avoid.
            # Long wavelength and modest amplitude: the track should look like
            # it drifted, not like it was steered.
            "wobble_m": round(max(w, h) * 0.032, 1),
            "wobble_period_m": round(max(w, h) * 1.60, 1),
            "along_period_m": round(max(w, h) * 0.95, 1),
            # A SEVERE TRACK IS MORE UNIFORM, not less. Weak tornadoes skip —
            # they touch down, lift and touch down again — while a violent one
            # is continuously on the ground for its whole length.
            "along_min": lerp(0.35, 0.72, sev),
            "width_min": lerp(0.55, 0.80, sev),
            "edge_noise_m": round(max(w, h) * 0.05, 1),

            # Where the debris went. `curl_deg` is toward the LEFT of travel:
            # for a cyclonic vortex the rotational and translational winds add
            # on the right flank and oppose on the left, so material lofted on
            # the strong side is carried across and deposited on the weak one.
            "curl_deg": 20.0,
            "spread_deg": lerp(42.0, 28.0, sev),
            "throw_m": lerp(12.0, 42.0, sev),

            # The settle bias, in m/s. This is what `settle.run(bias=...)`
            # gets when the archetypes are baked, and it is capped low on
            # purpose: the fragments only have to CLEAR the footprint and lean
            # downtrack, because the long-range debris is authored by
            # `disaster.planks` rather than simulated. Ten metres a second of
            # initial velocity against 0.18 damping carries a board a few
            # metres, which is the whole job.
            "throw_speed_mps": lerp(4.0, 11.0, sev),
        },
    }

    # PASS-THROUGH, ONLY WHEN THE SPEC SETS THEM. `disaster.tornado.DEFAULTS`
    # carries these at neutral defaults (0.0 / None / None / 120.0) already,
    # so a preset that never mentions them has to compile to the exact same
    # dict it always has — every existing tornado preset was reviewed on
    # today's values and must not drift. See `disaster.tornado.DEFAULTS` for
    # what each one is: `curvature_deg_per_km` (a gentle arc), `touchdown_m`
    # / `liftoff_m` / `ramp_m` (a finite track that ramps up and ropes out
    # rather than spanning the whole plate).
    for _k in ("curvature_deg_per_km", "touchdown_m", "liftoff_m", "ramp_m"):
        if _k in spec:
            out["tornado"][_k] = spec[_k]

    return out


def compile_explosion(sev, spec, region):
    """A blast: ground zero is obliterated, damage drops off fast with range.

    Signature — the tightest, most extreme gradient of any type. At the
    center nothing is left standing; a few hundred meters out the city is
    barely touched. Debris is thrown outward hard.
    """
    w, h = region
    cx, cy = spec.get("epicenter", [0.0, 0.0])
    return {
        "damaged_fraction": lerp(0.1, 0.25, sev),
        # -> 1.0 at sev=1: "nothing is left standing" at the center per the
        # docstring above.
        "destroyed_fraction": lerp(0.3, 1.0, sev),
        "debris": {
            "piles_per_building": lerp_pair([2, 3], [4, 7], sev),
            "pile_max_offset_m": lerp(2.5, 4.5, sev),
            "pieces_per_building": lerp_pair([8, 16], [20, 38], sev),
            "pieces_scatter_m": lerp(7.0, 16.0, sev),
            "tilt_chance": lerp(0.15, 0.45, sev),
            "tilt_deg": [3, lerp(8.0, 14.0, sev)],
            "sink_m": [0.4, lerp(1.0, 1.8, sev)],
            "lean_piles": lerp_pair([2, 3], [3, 5], sev),
            # Tight radial field: the scour is a scorched ring at
            # ground zero, not a corridor.
            "path_pieces_per_100m2": lerp(0.6, 2.5, sev),
            "path_piles_per_100m2": lerp(0.1, 0.5, sev),
        },
        "trees_toppled_fraction": lerp(0.3, 0.8, sev),
        "streetlights_toppled_fraction": lerp(0.3, 0.9, sev),
        "traffic_lights_toppled_fraction": lerp(0.25, 0.85, sev),
        "traffic_lights_leaning_fraction": lerp(0.3, 0.6, sev),
        "traffic_lights_lean_deg": [10, 40],
        "trash_cans_toppled_fraction": lerp(0.5, 0.95, sev),
        "trash_cans_scatter_m": lerp(5.0, 12.0, sev),
        "cars_toppled_fraction": lerp(0.2, 0.7, sev),
        "cars_strewn": lerp_pair([2, 5], [8, 16], sev),
        "strewn_topple_fraction": 0.9,
        "humans_prone_fraction": lerp(0.25, 0.75, sev),
        "humans_strewn": lerp_pair([2, 5], [8, 16], sev),
        # Small full-strength core, fast falloff, clean outside.
        "field": {
            "kind": "radial",
            "center": [float(cx), float(cy)],
            "radius_m": round(max(w, h) * lerp(0.06, 0.22, sev), 1),
            "falloff_m": round(max(w, h) * lerp(0.15, 0.3, sev), 1),
            "inside": 1.0,
            "outside": 0.0,
        },
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
            "piles_per_building": lerp_pair([1, 2], [2, 4], sev),
            "pile_max_offset_m": lerp(2.5, 5.0, sev),   # deposited, not dropped
            "pieces_per_building": lerp_pair([3, 7], [10, 20], sev),
            "pieces_scatter_m": lerp(5.0, 12.0, sev),   # floated outward
            "tilt_chance": lerp(0.1, 0.4, sev),         # undermined footings
            "tilt_deg": [2, lerp(5.0, 9.0, sev)],
            "sink_m": [0.3, lerp(0.8, 1.5, sev)],
            "lean_piles": [1, 3],
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
    """City-wide wind, ROOF-FIRST, on a coastal roughness gradient, plus a
    static surge footprint — no track, no untouched zone.

    Signature — the opposite emphasis of the tornado at almost every turn.
    Roof cover and openings carry the damage story; wall/structural loss
    stays a minority even at high severity, because "dismantling develops
    first at roof level" (Marshall 2004) and the survey record backs it —
    90% of Katrina's 8,119 surveyed residences lost under 20% of their roof
    cover, and only 1 in 2,700 lost a large section of roof STRUCTURE.
    Debris is mostly leaves and limbs, not sawn timber: Lee County's Ian
    collection ran 72% vegetative by volume. Trees stand, bare and brown,
    rather than down. Bare slabs appear only inside the surge footprint,
    never scattered inland — "all homes that suffered complete destruction
    in hurricanes were associated with storm surge, not high winds only"
    (Roueche et al.). Cars are moved by water, essentially never by wind
    alone (0 wind-relocated cars below ~130 mph gusts across 30 surveys).

    THIS IS THE FIRST VERSION THAT READS `spec` AND `region` AT ALL — the
    version this replaces computed everything from `sev` alone, which meant
    `heading_deg` and every hurricane-specific spec key were silent no-ops
    (verified 2026-08-29). `region` sizes exactly one knob below
    (`wrack_lines`); everything else `disaster.hurricane` carries is a
    PHYSICAL quantity — a gust speed, a surge depth, a land slope, a roll-
    vortex wavelength — and must NOT grow with the plate, the same argument
    `compile_tornado`'s `width_m` comment makes about track width. The
    legacy `debris.path_pieces_per_100m2`-style knobs below need no such
    line either: they are already expressed per unit area, so the region's
    actual footprint scales their total at the point of use, in
    `scene_generator.py`, not here.
    """
    w, h = region

    # ---- THE WIND, as `disaster.hurricane` — the sub-block that fixes the
    # `_disaster_kind` mislabel (freeze_dataset_launch_script falls through
    # to "wildfire" without one) and gives `hurricane.py` / `hurricane_flow.py`
    # every steering knob the parameter set in the skill asks for.
    #
    # THE DRIVER. Marshall's Katrina aerial survey of 11,105 structures found
    # the damage-derived gusts across the whole impact zone averaged 41 m/s —
    # Cat 1-2 LOCAL intensity inside a storm rated Cat 3 at landfall, because
    # most buildings sat in Exposure B, not open-terrain C. The category is a
    # label for the record; this is the number the ladder is actually driven
    # from. 0 is a fresh-breeze fringe brush; 1 is a low-end Cat 5 local gust
    # (matching the 175 mph ceiling in the vehicle-movement record below).
    site_gust_mps = round(float(spec.get("site_gust_mps", lerp(25.0, 78.0, sev))), 1)
    # Dominant azimuth of the strongest gust episode — the shared top-level
    # `heading_deg` vocabulary (tornado/hurricane/wildfire all read it as
    # "direction of travel"), not a hurricane-only key. Named `heading_deg`
    # in the output block too, matching `hurricane.resolve_cfg` exactly —
    # calling it `bearing_deg` there was a silent no-op the field module
    # never read (measured against `hurricane.py` after it landed).
    heading_deg = float(spec.get("heading_deg", 35.0))
    # Spread about `heading_deg`, +-30..60 as the wind backs through a
    # landfall (Marshall dated individual Keesler AFB failures rotating
    # 060 -> 110 deg over seven hours). Widens with severity because a more
    # violent boundary layer backs through more of that arc before the
    # structure fails; 180 (an eye passage, a second population roughly
    # opposite the first) is a narrative choice left to the preset, not
    # something severity alone should produce.
    rotation_deg = float(spec.get("rotation_deg", round(lerp(30.0, 55.0, sev), 1)))
    # The AXIS the rotation spread is centred on, when it differs from
    # `heading_deg` itself (e.g. a wind veer whose midpoint has drifted from
    # the storm's translational heading). `hurricane.resolve_cfg` defaults
    # this to `None` and derives it internally when unset; passed through
    # rather than guessed at here, since nothing in the research gives a
    # reason to diverge from the field module's own default.
    rotation_axis_deg = spec.get("rotation_axis_deg")
    # The spatial companion to `rotation_deg`: how far across the plate the
    # gust bearing visibly swings through that spread. A PHYSICAL span (like
    # `streak_period_m` below), not scaled by region. Ramped gently with
    # severity — a more violent boundary layer completes its veer over a
    # shorter run — and centred on `hurricane.resolve_cfg`'s own 900 m
    # default at mid-severity.
    rotation_span_m = round(float(spec.get("rotation_span_m",
                                            lerp(700.0, 1100.0, sev))), 1)
    # B (suburban) | C (open) | D (coastal). Every suburb preset on this
    # branch is Exposure B by construction — Marshall's finding above is
    # specifically that Exposure B cuts the open-terrain gust 13-15%, so this
    # is not a cosmetic label, it is load-bearing for `site_gust_mps`'s
    # downstream use.
    exposure = str(spec.get("exposure", "B"))
    # Where the coast lies, as a bearing from plate centre. Default ties it
    # to the storm's own heading: a landfalling hurricane's translational
    # motion and its onshore push are in the same quarter, so absent a
    # scene-specific shoreline this is the least-wrong guess. Override it
    # whenever the plate's coastline and the wind direction genuinely
    # diverge — the two are independent inputs on purpose.
    shore_bearing_deg = float(spec.get("shore_bearing_deg", heading_deg))
    # Fractional reduction of the mean wind per km inland of the coastline,
    # and the cap on how much of it can ever apply. Vickery, Wadhera & Powell
    # (2009): ~60% of the transition is complete by 1 km, with a MAXIMUM
    # mean-wind reduction of ~17% (`coastal_max_reduction`); NHC separately
    # states peak winds "diminish by one category within a short distance,
    # perhaps a kilometre." Both are properties of coastal terrain
    # roughness, not of storm intensity, so both are constants rather than a
    # function of `sev`. Named `coastal_falloff_per_km` to match
    # `hurricane.resolve_cfg` — this was `roughness_gradient` before the
    # field module landed and its real key names were measured.
    coastal_falloff_per_km = 0.10
    coastal_max_reduction = 0.17
    # OPTIONAL banded overlay from boundary-layer roll vortices — Morrison et
    # al. (2005) found them in 35-69% of radar volumes, Foster (2005)
    # quantifies ~14 m/s peak-to-peak over ~725 m, oriented `streak_skew_deg`
    # = -10 deg off the mean wind (toward the storm centre, per the skill).
    # `streak_amp_mps` is HALF the peak-to-peak figure (an amplitude, not a
    # swing), ramped from nothing at `sev`=0 up to the full +-7 m/s the
    # record supports; `streak_period_m` is the wavelength itself, a
    # PHYSICAL span (like the tornado's `throw_m`) that does not grow with
    # the plate. `streak` names the KIND of overlay — "roll" is the only one
    # these numbers describe; "mesovortex" / "tornado" bands are a different,
    # unresearched model and must not be selected by this compiler. No
    # published fragility study maps roll bands onto damage fractions, which
    # is why this stays a low-amplitude visual overlay rather than a primary
    # driver. `noise_amp_mps` is the separate, unstructured residual: the
    # skill's own field-uniformity measurement ("uniform to better than 1 m/s
    # against a category step of 7-12 m/s") is the ceiling this ramps toward.
    streak = "roll"
    streak_amp_mps = round(lerp(0.0, 7.0, sev), 1)
    streak_period_m = round(lerp(700.0, 1300.0, sev), 1)
    streak_skew_deg = -10.0
    noise_amp_mps = round(lerp(0.5, 1.5, sev), 2)

    # ---- THE SURGE — independent of category, per the skill's water plan.
    #
    # A PHYSICAL DEPTH, never scaled by the plate. 0.9 / 2.0 / 2.8 m are the
    # plan's own three dataset levels; 2.8 rather than a "true" Cat 4 3.6 m is
    # deliberate — at 3.6 the whole plate is underwater and featureless,
    # correct to the reference imagery but also the level with the LEAST for
    # a search benchmark to find, so the ladder is capped short of it on
    # purpose.
    surge_m = round(float(spec.get("surge_m", lerp(0.9, 2.8, sev))), 2)
    # Land grade, as a percentage, also independent of plate size. The water
    # plan runs `land_slope` 1/600 (0.167%) at the mild end down to 1/900
    # (0.111%) at the violent end — a GENTLER grade carries the same surge
    # depth farther inland, which is how the level-3 scene reaches "whole
    # plate" at 2.8 m without needing the featureless 3.6 m depth above.
    slope_pct = round(float(spec.get("slope_pct", lerp(0.167, 0.111, sev))), 3)
    # How many separate debris windrow segments to author along the
    # shoreline crossing THIS plate. This is the one knob here that
    # genuinely should grow with `region`: a real wrack line does not run
    # unbroken for kilometres, it breaks into a fresh segment roughly every
    # 150-250 m of coast, so a longer shoreline crossing needs proportionally
    # more segments to read as a broken tideline rather than one line
    # stretched thin. Severity adds a mild multiplier on top — a more
    # violent surge leaves more distinguishable strand lines from its rise
    # and its recession, not just a longer one.
    wrack_lines = max(1, int(round(max(w, h) / 220.0 * lerp(0.7, 1.3, sev))))
    # The proud-ribbon height the L3 waterline-band pass reads. Numerically
    # the same physical high-water mark as `surge_m`, kept as its own key
    # because the two are consumed by different code: `surge_m` sizes the
    # depth FIELD across the whole plate, `mudline_m` is the single scalar a
    # per-building band pass needs for where to paint the ring. Decoupling
    # them now is what lets a later pass model recession (a current mud line
    # below the historic peak `wrack_lines` mark) without touching the field.
    mudline_m = float(spec.get("mudline_m", surge_m))

    # ---- THE VARIANCE TERM. Marshall, in three separate storms: "flaws in
    # construction or poor attachment/anchoring can better explain sharp
    # gradations in damage rather than a rapid change in wind speed" — and
    # the code-era record proves it from the other side: post-1994
    # manufactured homes showed 0% damage above rating 1 in the same wind
    # that put 64.7% of pre-1976 units at rating >=2. A per-building
    # CONSTRUCTION-QUALITY draw, not the smooth field, is what should
    # dominate here — a destroyed house next to an untouched one is the
    # norm, the exact inverse of the tornado's cross-track gradient. This
    # mix is a property of the TOWN'S building stock, not of the storm, so
    # it does not scale with `sev`; a preset overrides it wholesale for a
    # scene meant to read as an older or newer development.
    #
    # FIVE NAMES, MATCHING `hurricane.CODE_ERAS` — a hurricane-only vocabulary
    # this compiler used to invent its own coarser three-bucket version of
    # (`pre_1980`/`1980_2004`/`post_2004`) before the field module landed and
    # its real era table could be measured. `CODE_ERAS`'s own population
    # shares (0.22 / 0.20 / 0.28 / 0.22 / 0.08) are a national baseline; this
    # default is shifted OLDER, which is the same Marshall/HUD argument as
    # before, just over the finer split: `post_2004_ringshank` and
    # `fortified` retrofits are still rare outside targeted incentive
    # programs, so 4 points move from `post_2004_ringshank` into the two
    # pre-1994 buckets `CODE_ERAS` itself cites the HUD post-Charley survey
    # for (post-1994 HUD-code 0% damage >=rating 2; pre-1994 HUD 28.6%;
    # pre-1976 non-HUD 64.7%). `fortified` is left at the module default —
    # nothing here argues a suburban plat should carry more or fewer
    # FORTIFIED retrofits than the national baseline.
    code_era_mix = spec.get("code_era_mix", {
        "pre_1976": 0.25,             # no meaningful wind provisions
        "hud_1976_94": 0.22,          # pre-1994 HUD code, still weak
        "post_1994": 0.27,            # post-1994 HUD-code reforms
        "post_2004_ringshank": 0.18,  # modern IBC/IRC ring-shank nailing
        "fortified": 0.08,            # IBHS FORTIFIED retrofit, unchanged
    })

    return {
        # 90% roof-cover-only losses even at strong local gusts push the
        # damaged ceiling well above the tornado's, since almost every
        # building shows SOME damage; the destroyed ceiling stays low
        # because full structural loss is the rare tail (1 in 2,700 lost a
        # large roof section in the record above) and `swept` belongs to
        # the surge footprint, never to wind alone (see `compile_flood` and
        # the module docstring).
        "damaged_fraction": lerp(0.15, 0.55, sev),
        "destroyed_fraction": lerp(0.02, 0.15, sev),
        "debris": {
            # FEWER big piles than a tornado or explosion: a hurricane's
            # coarse debris is panels and sheets peeled off IN PLACE, not a
            # whole structure heaped into a mound.
            "piles_per_building": lerp_pair([1, 2], [2, 3], sev),
            # Locally sourced and short-travelled — "within 1-3 building
            # widths... against the first obstruction downwind" — never the
            # tornado's flung fragments.
            "pile_max_offset_m": lerp(1.5, 3.0, sev),
            # RAISED over the previous ceiling on purpose: this is where the
            # leaf/limb litter lives. Lee County's Ian collection was 72%
            # vegetative by volume, and the field is UNIFORM, so every lot
            # gets some litter rather than a corridor concentrating it.
            "pieces_per_building": lerp_pair([8, 16], [26, 46], sev),
            "pieces_scatter_m": lerp(3.0, 8.0, sev),
            # Wind alone rarely undermines a footing — that is a surge/flood
            # mechanism (`compile_flood`'s job) — so tilt/sink stay low.
            "tilt_chance": lerp(0.03, 0.12, sev),
            "tilt_deg": [1, 4],
            "sink_m": [0.1, 0.3],
            "lean_piles": [1, 2],
            # Uniform field: thin scour everywhere, mostly vegetative,
            # raised over the previous range for the same reason as
            # `pieces_per_building` above.
            "path_pieces_per_100m2": lerp(0.3, 1.3, sev),
            # Coarse piles are the MINORITY fraction (28% of Ian's volume by
            # the same census), so this stays below the tornado's range.
            "path_piles_per_100m2": lerp(0.03, 0.15, sev),
        },
        # NEW: the legacy compiler never toppled a tree at all (the key was
        # simply absent), which is backwards for the strongest class
        # separator the research found. Kept well under the tornado's
        # 0.4-0.95 ladder on purpose — broadleaf canopy is stripped bare at
        # Cat 3+ (86-94% leaf loss) but the TRUNK STAYS STANDING; Andrew's
        # 38% urban-forest mortality is the ceiling across the whole record
        # (Erin 11%, Georges 13%, Jeanne 16%, Charley 18%, Rita 21%, Katrina
        # 23%), and mortality itself is not the same as toppled. Standing-
        # but-bare is the norm; this fraction is the minority that actually
        # goes down.
        "trees_toppled_fraction": lerp(0.05, 0.28, sev),
        "streetlights_toppled_fraction": lerp(0.1, 0.6, sev),
        "traffic_lights_toppled_fraction": lerp(0.08, 0.5, sev),
        "traffic_lights_leaning_fraction": lerp(0.25, 0.6, sev),
        "traffic_lights_lean_deg": [10, lerp(25.0, 40.0, sev)],
        "trash_cans_toppled_fraction": lerp(0.45, 0.9, sev),
        "trash_cans_scatter_m": lerp(3.0, 10.0, sev),
        # NEAR ZERO, DOWN FROM THE LEGACY 0.05-0.35. The complete record
        # across 30 hurricane surveys: nothing below ~130 mph gusts; Charley
        # (130-145) "a few automobiles were flipped"; Andrew (175) "several
        # ... shifted, rolled, or flipped." Cars are moved by WATER (30 cm
        # floats one, 45-60 cm carries most away) — `surge.py`'s job, not
        # this legacy knob's — so this is kept barely above zero rather than
        # exactly 0.0, for the handful a genuinely violent local gust can
        # still account for.
        "cars_toppled_fraction": lerp(0.0, 0.03, sev),
        # Same reasoning applied to the count: the tornado's scattered-lot
        # signature does not belong here at all.
        "cars_strewn": lerp_pair([0, 1], [1, 3], sev),
        # Of the handful that do move, most are found sitting upright where
        # the water put them down, not flipped — the opposite emphasis of
        # the tornado's 0.85.
        "strewn_topple_fraction": 0.15,
        "humans_prone_fraction": lerp(0.05, 0.3, sev),
        "humans_strewn": lerp_pair([0, 1], [3, 7], sev),
        # Uniform: the whole region is in the storm. Unchanged from the
        # legacy compiler — this part was correct, and the skill's own
        # measurement (the parametric field is uniform to better than 1 m/s
        # against a category step of 7-12 m/s) is what confirms it.
        "field": {"kind": "uniform", "inside": 1.0},

        "hurricane": {
            "enabled": True,
            "site_gust_mps": site_gust_mps,
            "heading_deg": heading_deg,
            "rotation_deg": rotation_deg,
            "rotation_axis_deg": rotation_axis_deg,
            "rotation_span_m": rotation_span_m,
            "exposure": exposure,
            "shore_bearing_deg": shore_bearing_deg,
            "coastal_falloff_per_km": coastal_falloff_per_km,
            "coastal_max_reduction": coastal_max_reduction,
            "streak": streak,
            "streak_amp_mps": streak_amp_mps,
            "streak_period_m": streak_period_m,
            "streak_skew_deg": streak_skew_deg,
            "noise_amp_mps": noise_amp_mps,
            "surge_m": surge_m,
            "slope_pct": slope_pct,
            "wrack_lines": wrack_lines,
            "mudline_m": mudline_m,
            "code_era_mix": code_era_mix,
        },
    }


def compile_wildfire(sev, spec, region):
    """Fire in the scene, and nothing else — yet.

    Signature — a wind-driven fire front running through the vegetation. Every
    structural knob is zero on purpose: this stage adds the FIRE ONLY, so a
    wildfire scene can be looked at and tuned before any question of what the
    fire does to the buildings is opened.

    That is why `field` is uniform-zero rather than a burn scar. When scorched
    facades and burnt-out props arrive they belong on the field, driven by the
    same ellipse the emitters already follow — see `disaster/fire.py`.

    The fire block itself is consumed by `disaster.fire.apply_wildfire`, which
    runs AFTER the scene is generated: it needs the placements the generator
    returns, because an emitter is fitted to the bounding box of a real prim.
    """
    w, h = region
    span = max(w, h)

    # Ignition point and wind direction reuse the shared spec keys, so a
    # wildfire is steered the same way a tornado or hurricane is.
    ox, oy = spec.get("epicenter", [-span * 0.35, -span * 0.35])
    heading = float(spec.get("heading_deg", 45.0))

    block = compile_none(sev, spec, region)
    block["fire"] = {
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
    return block


DISASTERS = {
    "none": compile_none,
    "earthquake": compile_earthquake,
    "tornado": compile_tornado,
    "explosion": compile_explosion,
    "flood": compile_flood,
    "hurricane": compile_hurricane,
    "wildfire": compile_wildfire,
}


# ---------------------------------------------------------------------------
# Compilation
# ---------------------------------------------------------------------------

def compile_spec(spec: dict, base: dict) -> dict:
    """High-level *spec* + *base* low-level config -> low-level config."""
    dtype = str(spec.get("disaster-type", spec.get("disaster_type", "none"))).lower()
    if dtype not in DISASTERS:
        raise ValueError(
            f"unknown disaster-type {dtype!r}; expected one of "
            f"{', '.join(sorted(DISASTERS))}")

    if "magnitude" in spec and spec.get("magnitude") is not None:
        # `magnitude` is the physical knob and it DEFINES severity: a preset
        # carries `severity: 0.85` as its default, and a MAGNITUDE override
        # that silently lost to it gave an "M5.5" city ten DG3s and a pancake
        # (two-city run 1, 2026-08-27). To pin severity with a magnitude, set
        # `overrides.disaster.grade_scale` instead.
        _m_sev = magnitude_to_severity(spec["magnitude"])
        if "severity" in spec and abs(float(spec["severity"]) - _m_sev) > 1e-6:
            print("[compile_disaster] magnitude {0} sets severity {1:.2f} (spec severity "
                  "{2} ignored)".format(spec["magnitude"], _m_sev, spec["severity"]))
        spec = dict(spec, severity=_m_sev)
    sev = float(spec.get("severity", 1.0))
    if not 0.0 <= sev <= 1.0:
        raise ValueError(f"severity must be in [0, 1], got {sev}")

    cfg = yaml.safe_load(yaml.safe_dump(base))   # deep copy

    # City-level passthroughs, before the disaster reads the region.
    if "seed" in spec:
        cfg["seed"] = spec["seed"]
    if "region_m" in spec:
        cfg.setdefault("layout", {})["region_m"] = spec["region_m"]
    # THE SKY, and it was being DROPPED. `cfg` starts as a copy of the base
    # low-level config and only the keys named here are carried across from the
    # spec, so a top-level `sky:` in a high-level preset never reached
    # `scene_generator.resolve_sky` — which documents it as "the top-level
    # `sky` config entry" and is therefore describing something that could not
    # work from a preset. The symptom is the worst kind: no error, no warning,
    # and a scene that quietly keeps whatever sky its asset set inherits.
    # Found 2026-08-28 setting a night dome on the tornado preset; the give-away
    # was that `scene_prep.add_sky`'s own "Dome light set at ..." line never
    # appeared in the log.
    if "sky" in spec:
        cfg["sky"] = spec["sky"]
    # The dome's brightness travels with it. A moonlit HDRI at `add_sky`'s
    # daylight default (intensity 3500, exposure -3) renders as a flat bright
    # grey — the image is right and the exposure is three stops wrong, which
    # reads as "that is not a night sky".
    for _k in ("sky_intensity", "sky_exposure"):
        if _k in spec:
            cfg[_k] = spec[_k]

    # ---- LOCALE axis: how the place is laid out (see compile_locale.py).
    # Applied before the disaster so a disaster's overrides still win, and
    # before `overrides:` so the user's escape hatch wins over both.
    locale = spec.get("locale", "downtown")
    deep_merge(cfg, compile_locale_settings(locale, spec))
    cfg["locale"] = str(locale).lower()

    # An explicit asset-set wins; otherwise the locale picks a sensible one
    # (downtown -> urban art, suburban -> houses).
    asset_set = spec.get("asset-set", spec.get("asset_set"))
    cfg["asset_set"] = asset_set or default_asset_set(locale)

    region = tuple(float(v) for v in cfg["layout"]["region_m"])

    # Severity 0 means untouched, whatever the type claims to be.
    fn = DISASTERS[dtype] if sev > 0.0 else compile_none
    cfg["disaster"] = fn(sev, spec, region)

    # Escape hatch: raw low-level overrides win over everything.
    if spec.get("overrides"):
        deep_merge(cfg, spec["overrides"])

    # REGION IS THE ONE EXCEPTION to "overrides win", because it is not just a
    # setting — the disaster above was already COMPILED against it (`region` at
    # the top of this function). Five presets repeat `region_m` under
    # `overrides.layout` as well as at the top level, so without this the merge
    # silently restored the preset's size and left a fire field sized for a
    # different one: a 250 m burn on a 1600 m plat, and `REGION_M` a no-op on
    # exactly the purpose-built presets. Re-assert it so the plat and the
    # disaster agree, and so the top-level key means what it looks like.
    if "region_m" in spec:
        _ovr = ((spec.get("overrides") or {}).get("layout") or {}).get("region_m")
        if _ovr is not None and list(map(float, _ovr)) != list(map(float, spec["region_m"])):
            print("[compile_disaster] overrides.layout.region_m {0} ignored; "
                  "the spec's region_m {1} is authoritative (the disaster is "
                  "compiled against it)".format(_ovr, spec["region_m"]))
        cfg.setdefault("layout", {})["region_m"] = spec["region_m"]

    # Compilation is a build step; leaving the high-level keys in the output
    # would suggest editing them there has an effect. Keep them only as
    # provenance in the header comment.
    for k in ("disaster-type", "disaster_type", "severity", "overrides",
              "epicenter", "magnitude", "heading_deg", "asset-set",
              "presets_path",
              "preset_file"):
        cfg.pop(k, None)
    return cfg


def is_high_level(cfg: dict) -> bool:
    """True if *cfg* is a high-level disaster spec rather than a scene config."""
    return bool(cfg.get("disaster-type") or cfg.get("disaster_type"))


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


def load_scene_config(name_or_path: str, base_path: str = None,
                      spec_overrides: dict = None) -> dict:
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

    # Applied BEFORE compilation, so overriding `disaster-type` actually
    # changes what gets compiled rather than leaving a compiled disaster
    # section behind that the launcher then has to remember to ignore.
    if spec_overrides:
        applied = {k: v for k, v in spec_overrides.items() if v is not None}
        if applied:
            cfg = dict(cfg)
            # `overrides` is DEEP-MERGED, not replaced: it is the preset's own
            # low-level escape hatch and clobbering it would silently drop
            # everything the preset set there (materials, roads, instancing).
            _ovr = applied.pop("overrides", None)
            cfg.update(applied)
            if _ovr:
                merged = copy.deepcopy(cfg.get("overrides") or {})
                deep_merge(merged, _ovr)
                cfg["overrides"] = merged
                applied["overrides"] = _ovr
            print(f"[compile_disaster] spec overrides: {applied}")

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

    # Lazy so `--list` and plain compilation stay free of the pxr dependency.
    import scene_generator
    cfg = scene_generator.resolve_asset_set(cfg, path)
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
        f"# locale        : {spec.get('locale', 'downtown')}",
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
