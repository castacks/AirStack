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

# ---------------------------------------------------------------------------
# Locale compilers — one per locale
#
# Each returns a dict of low-level settings, deep-merged over default.yaml.
# Only keys that actually differ from the base are listed, so the diff between
# two locales is readable.
# ---------------------------------------------------------------------------

def compile_downtown(spec):
    """Dense urban core: paved wall-to-wall, built to the sidewalk, busy.

    This is the generator's historical behavior — every knob at its
    default.yaml value — so the entry is mostly explicit documentation of
    what "downtown" means, and the thing the other locales are a delta from.
    """
    return {
        "layout": {"min_block_m": 30, "max_block_m": 70},
        "packing": {
            "building_gap_m": 2.5,
            "pave_blocks": True,      # concrete wall-to-wall between buildings
            "setback_m": 0.0,         # built out to the sidewalk
            "park_block_chance": 0.12,
            "min_parks": 2, "max_parks": 3,
        },
        "frontage": {"verge_m": 0.0}, # sidewalk runs kerb-to-building
        "roads": {"main_road_chance": 0.25, "main_road_lanes": 4,
                  "secondary_road_lanes": 2},
        # Street trees are potted; nothing grows out of a downtown block.
        "trees": {"lawn_density_per_100m2": 0.0},
        "plants": {"lawn_density_per_100m2": 0.0},
        "planters": {"tree_spacing_m": 30.0, "plant_spacing_m": 40.0},
        "streetlights": {"spacing_m": 18.0},
        "benches": {"spacing_m": 30.0},
        "trash_cans": {"spacing_m": 25.0},
        "bus_stops": {"spacing_m": 130.0},
        "fire_hydrants": {"spacing_m": 55.0},
        "traffic_lights": {"intersection_chance": 0.9},
        "driveways": {"chance": 0.0},
        "cars": {"density": 0.15},
        "humans": {"sidewalk_spacing_m": 45.0, "per_block": [0, 3]},
    }


def compile_suburban(spec):
    """Residential suburb: lawn by default, houses set back, sparse street.

    Every change here follows from "grass, not pavement, is the default
    surface": blocks stop being paved, houses hold back from the street,
    trees stand in the lawn instead of in planters, cars move onto driveways,
    and the downtown street kit (benches, bins, shelters, signals) goes away.
    """
    return {
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
        # The downtown street kit is absent from residential streets.
        "streetlights": {"spacing_m": 50.0},
        "benches": {"spacing_m": 0.0},
        "trash_cans": {"spacing_m": 0.0},
        "bus_stops": {"spacing_m": 0.0},
        "fire_hydrants": {"spacing_m": 70.0},
        "traffic_lights": {"intersection_chance": 0.05},   # stop signs, not lights
        # Cars belong on the drive beside the house.
        "driveways": {"chance": 0.75, "width_m": 3.2, "car_chance": 0.6},
        "cars": {"density": 0.04},
        "humans": {"sidewalk_spacing_m": 150.0, "per_block": [0, 1]},
        "parks": {"playground_chance": 0.7},
        # HOUSES COME FROM THE MODULAR KIT, not the whole-house USD pack.
        #
        # This is a locale property, not a per-preset one: every purpose-built
        # suburb preset already turns it on, and the generic `suburb` preset
        # did not — so it silently fell through to `house_catalogue` art, which
        # uses a DIFFERENT yaw convention (the art path applies `yaw` while the
        # kit path applies `yaw + 90`). The result was a plat whose houses were
        # turned 90 degrees relative to their own driveways, with nothing in
        # the config to suggest the two presets differed.
        #
        # The lot sizing comes with it: the [21, 30] knee was measured against
        # the whole-house pack, whose widest entry is ~16 m, and the kit's
        # L-plans are 20 m — narrower lots reject them and the plat thins out.
        "suburb_parcel": {
            "modular_houses": True,
            "modular_share": 1.0,
            "lot_width_m": [30.0, 44.0],
            "min_lot_depth_m": 21.0,
            "house_yaw_offset_deg": -90.0,
        },
    }


def compile_rural(spec):
    """Open country: buildings are incidents in a landscape, not a fabric.

    The suburban axis pushed further — huge blocks, near-zero coverage, no
    kerb furniture of any kind, and vegetation as the dominant surface rather
    than a decoration on it.
    """
    return {
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
    return LOCALES[key](spec)


def default_asset_set(locale: str) -> str:
    return DEFAULT_ASSET_SETS.get(str(locale).lower(), "urban")


if __name__ == "__main__":
    print("locales:")
    for name, fn in sorted(LOCALES.items()):
        summary = (fn.__doc__ or "").strip().splitlines()[0]
        print(f"  {name:<10} (assets: {DEFAULT_ASSET_SETS[name]:<9}) {summary}")
