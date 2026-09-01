#!/usr/bin/env python3
"""
seed_standalone_cache.py -- pre-populates a `scene_generator.SizeResolver`'s
internal `_raw` cache from the checked-in, real-Nucleus-measured
`config/harvested/standalone_buildings.json` (see `measure_standalone_via_
nucleus.py`'s docstring for how that file was produced).

WHY THIS EXISTS -- THE HOST/KIT PACKING DIVERGENCE
---------------------------------------------------------------------------
`fire_city_dry_run.build_layout`'s own docstring already documents that a
host build substitutes GAC/downtowncity footprints from an offline cache
(`_plans/gac_buildings.json`/`dtc_buildings.json`) because Nucleus is not
locally mirrored. It does NOT do the same for the OTHER packs that are also
unmirrored -- `standalone/buildings/...`, Muyang (`BG_Building_*`,
`SM_MERGED_BP_MBuilding*`), Dmytro (`Building_Type*`) -- so `SizeResolver`
falls back to a flat 30 x 20 x 24 m box for every one of them. Measured,
2026-08-31 review: `stepped_tower.usdc` is REALLY 65 x 78 x 81 m (the
authoritative Kit dump, `_plans/fc_dump_500.json`, agrees to 6 significant
figures with this file's own Nucleus measurement) -- 30 x 20 x 24 undercounts
its footprint area by roughly 8x. That is not a rounding error, it is a
different city: the host packer fits multiple small buildings where Kit
fits one huge one, `districts._burnable_substitute`'s area-ratio test
rejects real substitutes that are only "too small" because of the fallback
(`SM_MERGED_BP_MBuilding02` is REALLY 91 x 96 m, easily large enough to
stand in for `stepped_tower`'s 65 x 78 m footprint -- 30 x 20 m is not), and
even facing violates the same way `house_16_223` did, just against
different geometry.

USAGE
---------------------------------------------------------------------------
Call `seed_resolver(resolver)` on a `SizeResolver` BEFORE any packing reads
it, or use `patched_make_resolver()` as a context manager around a call to
`fire_city_dry_run.build_layout` (which constructs its own resolver
internally and gives no other hook to reach it) -- see `patched_make_
resolver`'s own docstring for exactly why that shape is necessary and safe.

    from tools import seed_standalone_cache as ssc
    with ssc.patched_make_resolver():
        config, layout, placements, resolver = fcd.build_layout(preset, seed=seed)

This is READ-ONLY with respect to `scene_generator.py`/`fire_city_dry_run.
py` themselves -- both are owned by another agent on this branch. The
monkeypatch lives here, is applied for the duration of one call, and is
always restored in a `finally`.
"""
import contextlib
import json
import os

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TOOLS_DIR)
CACHE_PATH = os.path.join(_SCENE_GEN_DIR, "config", "harvested",
                          "standalone_buildings.json")

_RAW_FIELDS = ("sx", "sy", "sz", "base", "cx", "cy", "cz")


def load_cache() -> list:
    """The checked-in cache's `records` list, or `[]` if missing/unreadable
    -- fails OPEN (no seeding at all, the resolver measures/falls back as
    it always has) rather than raise, so a stale or absent cache never
    blocks an otherwise-working dry run."""
    try:
        with open(CACHE_PATH) as fh:
            return list(json.load(fh).get("records") or [])
    except (OSError, ValueError):
        return []


def seed_resolver(resolver, records=None) -> int:
    """Populates *resolver*'s `_raw` dict (the SAME cache key shape
    `SizeResolver.get` itself uses: `(usd_path, axis_up)` -> raw footprint
    at scale 1.0 -- see `scene_generator.SizeResolver.get`) with every
    record in *records* (default: `load_cache()`) NOT ALREADY PRESENT.
    Never overwrites an existing entry -- a resolver that has already
    measured (or been seeded for) a path keeps whatever it has. Returns how
    many entries were newly seeded.
    """
    if records is None:
        records = load_cache()
    if not hasattr(resolver, "_raw"):
        resolver._raw = {}
    n = 0
    for rec in records:
        key = (rec["usd"], rec.get("axis_up", "Z"))
        if key in resolver._raw:
            continue
        resolver._raw[key] = {k: float(rec[k]) for k in _RAW_FIELDS}
        n += 1
    return n


@contextlib.contextmanager
def patched_make_resolver(module_name: str = "scene_generator"):
    """Wraps `<module_name>._make_resolver` for the duration of the `with`
    block so every resolver it constructs is seeded (`seed_resolver`)
    immediately after creation, before any caller can pack against it.

    NEEDED BECAUSE `fire_city_dry_run.build_layout` constructs its OWN
    `SizeResolver` internally (`scene_generator._make_resolver(config)`)
    and returns it only AFTER the whole layout is already built -- by then
    every packing decision that would have benefited from a real size has
    already been made against the fallback. Wrapping the FACTORY function
    (the same "patch a function on its owning module for one call, restore
    in `finally`" idiom `fire_city_dry_run.build_layout` itself uses for
    `scene_generator.build_city`) is the only way to seed the cache before
    `build_city`/`generate_scene_on_stage` ever call `resolver.get()`.

    Restores the original function in a `finally`, so an exception inside
    the `with` block never leaves the patch in place for a later,
    unrelated call in the same process.
    """
    import importlib

    mod = importlib.import_module(module_name)
    orig = mod._make_resolver
    records = load_cache()

    def _patched(config):
        resolver = orig(config)
        seed_resolver(resolver, records)
        return resolver

    mod._make_resolver = _patched
    try:
        yield
    finally:
        mod._make_resolver = orig
