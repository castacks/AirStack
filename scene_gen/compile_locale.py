#!/usr/bin/env python3
"""
compile_locale.py — the LOCALE axis of a high-level scene spec.

A locale is *how the place is laid out*: block size, how much of a block is
built on, whether the ground between buildings is pavement or lawn, how dense
the street furniture is. It is not the same thing as the asset set, which is
only *what the buildings look like* — a downtown built out of house models is
still a downtown, which is exactly the failure this axis exists to fix.

The governing difference:

    downtown  default ground = pavement, default state = full
    suburb    default ground = grass,    default state = mostly empty

Each locale is a function returning low-level generator settings, registered
in LOCALES. compile_disaster.py applies it under the disaster settings, so
the two axes compose: any locale × any disaster × any severity.

Locales also name a *default* asset set, used when the spec doesn't give one.

See GENERATION.md ("Locales: downtown vs suburb") for the characteristics
each of these encodes.
"""

import os

_LOCALE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "config", "low_level", "locales")


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursive in-place merge. Same semantics as compile_disaster.deep_merge,
    duplicated rather than imported to keep this module free of that import
    cycle (compile_disaster imports this one)."""
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def _locale_config(name: str) -> dict:
    """Load `config/low_level/locales/<name>.yaml`, or {} if there is none.

    Most locale settings are a handful of scalars and read best as the dicts
    below. A few are bulky, heavily-cited tables — downtown's eighteen
    street-furniture categories run to ~200 lines — and those live in YAML
    beside the other config rather than being transcribed into Python.
    """
    import yaml

    path = os.path.join(_LOCALE_DIR, f"{name}.yaml")
    if not os.path.exists(path):
        return {}
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


# ---------------------------------------------------------------------------
# Locale compilers — one per locale
#
# Each returns a dict of low-level settings, deep-merged over default.yaml.
# Only keys that actually differ from the base are listed, so the diff between
# two locales is readable.
# ---------------------------------------------------------------------------

def compile_downtown(spec):
    """Dense urban core: paved wall-to-wall, built to the sidewalk, busy.

    Downtown is the locale the *detailed* generator was built for, so this is
    where the detailed subsystems get switched on: anisotropic blocks
    (`layout.anisotropic`), radial zoning (`districts`) and NACTO-zoned street
    furniture (`city_detail`). Their settings live in `default.yaml` — this
    only enables them and supplies the values that are genuinely a property of
    *downtown* rather than of the generator.

    Enabling `city_detail` means the built-in frontage passes must go quiet:
    the two would otherwise both place benches, and the built-in one puts every
    category on a single kerb line where the later ones get dropped. One owner
    per concern, so the built-in spacings are zeroed below. (They are deleted
    outright in Phase 6; until then a zero is how a pass is switched off.)
    """
    settings = {
        # Manhattan runs ~80 m between streets and ~280 m between avenues.
        # `anisotropic` supplies that; `max_block_m` bounds the long axis.
        "layout": {"min_block_m": 30, "max_block_m": 200,
                   "anisotropic": {"enabled": True}},
        "districts": {"enabled": True},        # radial core/mid/edge zoning
        "packing": {
            "building_gap_m": 6.0,
            "pave_blocks": True,      # concrete wall-to-wall between buildings
            "setback_m": 0.0,         # built out to the sidewalk
            # Parks are carved as superblocks before subdivision
            # (`layout.anisotropic.parks`), so the BSP must make none of its
            # own — two adjacent park leaves read as one design mirrored about
            # the street between them.
            "park_block_chance": 0.0,
            "min_parks": 0, "max_parks": 0,
        },
        "frontage": {"verge_m": 0.0}, # sidewalk runs kerb-to-building
        "roads": {"main_road_chance": 0.25, "main_road_lanes": 4,
                  "secondary_road_lanes": 2,
                  # NACTO Urban Street Design Guide: 10 ft travel lanes.
                  "lane_width_m": 3.3,
                  "lane_lines": {"dash_length_m": 3.0, "dash_gap_m": 9.0}},
        # Street trees are potted; nothing grows out of a downtown block.
        "trees": {"lawn_density_per_100m2": 0.0},
        "plants": {"lawn_density_per_100m2": 0.0},
        # --- built-in frontage passes: off, city_detail owns the sidewalk ---
        "planters": {"tree_spacing_m": 0.0, "plant_spacing_m": 0.0},
        "streetlights": {"spacing_m": 0.0},
        "benches": {"spacing_m": 0.0},
        "trash_cans": {"spacing_m": 0.0},
        "bus_stops": {"spacing_m": 0.0},
        "fire_hydrants": {"spacing_m": 0.0},
        "traffic_lights": {"intersection_chance": 0.0},
        # -------------------------------------------------------------------
        "driveways": {"chance": 0.0},
        "cars": {"density": 0.15},
        "humans": {"sidewalk_spacing_m": 45.0, "per_block": [0, 3]},
        # The superblock park composer (`parks.py`) and the built-in BSP park
        # pass share the `parks:` namespace but are different subsystems, and
        # they disagree on these three. They live here rather than in
        # default.yaml because a single key cannot hold both values — putting
        # the composer's numbers in the shared block silently retuned every
        # locale's parks (caught by check_duplicate_yaml_keys).
        "parks": {
            "furniture_offset_m": 1.2,      # kerb-to-prop gap off the walk edge
            # Trees are laid first and plants take what is left, so a high tree
            # density keeps shrubs as underplanting rather than a substitute
            # canopy. At 1.2/0.8 the park read as full of bushes.
            "tree_density_per_100m2": 1.8,
            "tree_min_separation_m": 4.0,
        },
    }
    # The street-furniture category table (locales/downtown.yaml) — bulky and
    # heavily cited, so it stays in YAML.
    _deep_merge(settings, _locale_config("downtown"))
    return settings


# Locales that keep the built-in frontage passes need `city_detail` silent.
# An explicit flag rather than clearing `categories`: `deep_merge` recurses
# into nested dicts, so an empty `categories: {}` merges to a no-op and the 18
# inherited entries survive. Same shape as `districts.enabled`.
_NO_CITY_DETAIL = {"city_detail": {"enabled": False}}


def compile_suburban(spec):
    """Residential suburb: lawn by default, houses set back, sparse street.

    Every change here follows from "grass, not pavement, is the default
    surface": blocks stop being paved, houses hold back from the street,
    trees stand in the lawn instead of in planters, cars move onto driveways,
    and the downtown street kit (benches, bins, shelters, signals) goes away.
    """
    settings = {
        # Long blocks, few intersections.
        "layout": {"min_block_m": 60, "max_block_m": 150},
        "packing": {
            "building_gap_m": 9.0,    # side yards between detached houses
            "pave_blocks": False,     # THE difference: the block is lawn
            "setback_m": 9.0,         # front yard between house and sidewalk
            "park_block_chance": 0.15,
            "min_parks": 2, "max_parks": 4,
        },
        "frontage": {"verge_m": 1.5}, # planting strip between kerb and walk
        # Residential streets: two lanes, arterials only occasionally.
        "roads": {"main_road_chance": 0.05, "main_road_lanes": 2,
                  "secondary_road_lanes": 2},
        # Vegetation lives on the ground, not in boxes.
        "trees": {"lawn_density_per_100m2": 0.9, "lawn_min_separation_m": 6.0},
        "plants": {"lawn_density_per_100m2": 1.4, "lawn_house_margin_m": 0.6},
        "planters": {"tree_spacing_m": 0.0, "plant_spacing_m": 0.0},
        # --- built-in frontage passes: off, city_detail owns the sidewalk ---
        # A suburb's street kit is sparse, but it is the same *pass* that
        # places it as downtown's now — see locales/suburban.yaml.
        "streetlights": {"spacing_m": 0.0},
        "benches": {"spacing_m": 0.0},
        "trash_cans": {"spacing_m": 0.0},
        "bus_stops": {"spacing_m": 0.0},
        "fire_hydrants": {"spacing_m": 0.0},
        # No `traffic_lights` entry: signals are placed by city_detail now,
        # and its rule is better than the flat 5% chance this used to carry —
        # a junction gets a signal when it carries `signals.signal_lanes` (4)
        # or more. A suburb's roads are two lanes, so it gets none, which is
        # what GENERATION.md means by "a signal reads as downtown". Stop
        # signs are the suburban answer (locales/suburban.yaml), pending art.
        # Cars belong on the drive beside the house.
        "driveways": {"chance": 0.75, "width_m": 3.2, "car_chance": 0.6},
        "cars": {"density": 0.04},
        "humans": {"sidewalk_spacing_m": 150.0, "per_block": [0, 1]},
        "parks": {"playground_chance": 0.7},
    }
    _deep_merge(settings, _locale_config("suburban"))
    return settings


def compile_rural(spec):
    """Open country: buildings are incidents in a landscape, not a fabric.

    The suburban axis pushed further — huge blocks, near-zero coverage, no
    kerb furniture of any kind, and vegetation as the dominant surface rather
    than a decoration on it.
    """
    return {
        # Built-in frontage passes keep the sidewalk here; the
        # NACTO-zoned pass is a downtown thing.
        **_NO_CITY_DETAIL,
        "layout": {"min_block_m": 150, "max_block_m": 320},
        "packing": {
            "building_gap_m": 30.0,   # farmsteads, not neighbours
            "pave_blocks": False,
            "setback_m": 20.0,        # set well back off the road
            "park_block_chance": 0.35,  # most "blocks" are just land
            "min_parks": 3, "max_parks": 8,
        },
        "frontage": {"verge_m": 2.5},
        "roads": {"main_road_chance": 0.0, "main_road_lanes": 2,
                  "secondary_road_lanes": 2},
        "trees": {"lawn_density_per_100m2": 1.6, "lawn_min_separation_m": 5.0},
        "plants": {"lawn_density_per_100m2": 2.5, "lawn_house_margin_m": 0.5},
        "planters": {"tree_spacing_m": 0.0, "plant_spacing_m": 0.0},
        # No street furniture at all — no kerb to put it on.
        "streetlights": {"spacing_m": 0.0},
        "benches": {"spacing_m": 0.0},
        "trash_cans": {"spacing_m": 0.0},
        "bus_stops": {"spacing_m": 0.0},
        "fire_hydrants": {"spacing_m": 0.0},
        "traffic_lights": {"intersection_chance": 0.0},
        "driveways": {"chance": 0.9, "width_m": 3.5, "car_chance": 0.5},
        "cars": {"density": 0.01},
        "humans": {"sidewalk_spacing_m": 0.0, "per_block": [0, 1]},
        # A rural "park block" is open land — a field or woodland, not a
        # municipal park, so it gets the trees but none of the trail kit.
        "parks": {
            "playground_chance": 0.05,
            "tree_density_per_100m2": 1.4,
            "bench_spacing_m": 0.0,
            "streetlight_spacing_m": 0.0,
            "trash_can_spacing_m": 0.0,
        },
    }


LOCALES = {
    "downtown": compile_downtown,
    "suburban": compile_suburban,
    "rural": compile_rural,
}

# Asset set used when a spec names a locale but no asset-set.
DEFAULT_ASSET_SETS = {
    "downtown": "urban",
    "suburban": "suburban",
    "rural": "suburban",   # no rural set yet — suburban art is the closest fit
}


def compile_locale_settings(locale: str, spec: dict) -> dict:
    """Low-level generator settings for *locale*. Raises on an unknown name."""
    key = str(locale).lower()
    if key not in LOCALES:
        raise ValueError(
            f"unknown locale {locale!r}; expected one of "
            f"{', '.join(sorted(LOCALES))}")
    from scene_generator import restage   # lazy: keeps pxr off the CLI path
    return restage(LOCALES[key](spec))


def default_asset_set(locale: str) -> str:
    return DEFAULT_ASSET_SETS.get(str(locale).lower(), "urban")


if __name__ == "__main__":
    print("locales:")
    for name, fn in sorted(LOCALES.items()):
        summary = (fn.__doc__ or "").strip().splitlines()[0]
        print(f"  {name:<10} (assets: {DEFAULT_ASSET_SETS[name]:<9}) {summary}")
