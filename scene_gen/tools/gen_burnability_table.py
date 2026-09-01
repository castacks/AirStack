#!/usr/bin/env python3
"""
gen_burnability_table.py -- generates the CHECKED-IN static burnability
table `scene_gen/config/harvested/burnability_table.json` that `detail/
districts.py` consults at layout-gen time to keep an unburnable asset from
fragmenting the fire-spread graph into firebreaks (the user's ask,
2026-08-31: "I know we aren't using some buildings for fire at all, we need
to account for that in the layout gen").

KEYED BY (TYPOLOGY, ASSET BASENAME), NOT ASSET ALONE (2026-08-31 review fix)
---------------------------------------------------------------------------
The first version of this table evaluated every pool asset under ONE
synthetic layout (every point answering `typology_at() == "midrise"`) and
kept a single `{basename: bool}` table. Two bugs, found by review:

  1. `usds.buildings` pool KEYS are not typology NAMES. `districts.
     typologies.<name>.pools` is the map that matters -- e.g. the `midrise`
     TYPOLOGY draws from the `midrise_v2` POOL, not a pool literally named
     `midrise` (that pool exists too, 26 entries, but no typology in this
     preset's `districts.typologies` list ever references it -- it is dead
     weight for `downtown_fire_500`). The original `DEFAULT_POOLS` list
     named POOL keys directly and included both `midrise` and `midrise_v2`,
     so it audited an unused pool and risked silently mismatching a
     typology's real stock elsewhere. Fixed by reading `districts.
     typologies` itself and resolving each typology's pool LIST from there
     -- the exact map `rezone_blocks`/`infill_blocks` use.
  2. A `same_art` asset's `btype` (the construction-type tag `kit_
     substitute.route`'s `best_style(..., prefer_type=btype)` uses as a
     SOFT +0.25 mismatch penalty -- see that function; it never turns a
     qualifying match into a refusal, so this alone cannot flip a verdict,
     but it is still the field the real pipeline sets before calling
     `burnable()` and this tool silently left `None`) was never computed.
     Fixed by mirroring `fire_city_dry_run.gather_burnable`'s own call:
     `placement["btype"] = quake._same_art_material(usd, config)` whenever
     `kit_substitute.pack_of(usd) == "same_art"`.

Measured verdict of switching from one blanket "midrise" label to each
asset's REAL typology: IDENTICAL for every asset in the pool (see the
"PROOF" section this tool's own `--prove` mode prints, and `disaster.
urban_fire_city`'s own self-test at the "district no longer gates candidacy"
comment: moving a placement between typology blocks changes only the
record's `typology` field, never `ok`/candidacy). The keyed-by-typology
table is still the right structure -- correct is correct even when it does
not move today's numbers, and a future gate that DOES read typology would
silently mis-verify under the old single-table shape. `btype` is now
computed regardless, for the same reason.

WHY A CHECKED-IN TABLE INSTEAD OF A LIVE CALL
---------------------------------------------------------------------------
The real gate is `disaster.urban_fire_city.burnable()`, and it is correct by
construction (it is the SAME function `fire_city_dry_run.gather_burnable`
and the real fire pipeline both call), but reaching it drags in `disaster.
gac_fire`, `disaster.kit_substitute`, `disaster.quake` and `disaster.
urban_fire_city` -- the fire stack. `districts.py` is imported for EVERY
scene, fire disaster or not (a suburb build, a hurricane build, an
earthquake build all import it too), so importing that stack at layout
BUILD time would be a heavy, disaster-specific dependency leaking into
every other preset for a fact that is fixed per (asset, typology) and does
not change scene to scene. Baking the six gates' verdict into a small
static JSON lets `districts.py` do a plain dict lookup with zero extra
imports, and this tool is the one place that pays for the fire-stack import
-- run offline, host-side, only when the pool of assets, which typology
draws from which pool, or the gates themselves change.

WHAT "BURNABLE" MEANS HERE
---------------------------------------------------------------------------
Four of `burnable()`'s six gates are properties of the ASSET (and, for
`same_art`, its declared `material:`), not of where it is placed:
`kit_substitute.route()`'s action (a pack with real, unnamed parts -- AEC
brownstones, `standalone/buildings/...` -- is always refused as `'slice'`;
Muyang DownTown is always `'skip'`), `bake_kind()`'s routing (GAC/DTC by
Nucleus path prefix, else the kit archetype naming convention), the GAC/DTC
pack blacklist (`gac_fire.PACKS[...]['blacklist']`, name-based), and the
max-fire-height cap (`FIRE_MAX_H_M`, height-based, and height is fixed per
asset at its pool's own `scale`). The two REMAINING gates -- `category ==
"house"` and `typology_at(layout, x, y) is not None` -- are properties of
the PLACEMENT's position, not the asset, so this tool satisfies them with a
synthetic placement (`category: "house"`) inside a synthetic layout whose
one block covers all of the plane and is named for the CURRENT typology
being audited.

HOW HEIGHT/FOOTPRINT IS MEASURED WITHOUT A LIVE NUCLEUS RESOLVER
---------------------------------------------------------------------------
Same trick `fire_city_dry_run.py` already uses and this tool imports
directly rather than re-deriving: `_gac_dtc_cache()` (`_plans/gac_buildings.
json` / `_plans/dtc_buildings.json`, both offline measurements, no Nucleus)
answers GAC/DTC assets by basename; everything else goes through a real
`SizeResolver` seeded from the config, which resolves a KIT ARCHETYPE
locally (`_localize_building_urls` rewrites `.../scene_gen/assets/
archetype/...` paths to their on-disk mirror) and falls back to `fallback_
sizes.house` (30 x 20 x 24 m) for anything else unmirrored (AEC brownstones,
Muyang, `standalone/buildings/...`). The fallback is generic, but it is
harmless here: for every one of those packs, `kit_substitute.route()`'s
verdict ('slice' or 'skip') depends on which PACK the path is in, not on
its exact measured size -- confirmed by `fire_city_dry_run.gather_burnable`
itself reaching the identical refusal reason for these assets with this
exact same fallback already in play. (A SEPARATE, real-Nucleus-measured
cache for these packs exists at `config/harvested/standalone_buildings.
json` -- see `tools/seed_standalone_cache.py` -- for the different problem
of the HOST PACKER's own size-driven layout decisions, not this table.)

USAGE (regeneration)
---------------------------------------------------------------------------
    python3 scene_gen/tools/gen_burnability_table.py
    python3 scene_gen/tools/gen_burnability_table.py --preset downtown_fire_500
    python3 scene_gen/tools/gen_burnability_table.py --prove --seed 4

Re-run this whenever: a new building asset is added to any typology's
pool(s), a typology's `pools:` list changes, `gac_fire.PACKS[...]
['blacklist']` changes (exactly what triggered the first version of this
tool -- `Building_12` was just added), `urban_fire_city.FIRE_MAX_H_M`
changes, or `kit_substitute.route`/`bake_kind`'s own routing rules change.
The tool prints a diff-friendly report (typology, asset, verdict, reason)
and always overwrites the full table -- there is no incremental mode.

`--prove` additionally builds the REAL seed-4 host layout (`fire_city_
dry_run.build_layout` + `gather_burnable`) and prints the two empty-
intersection checks a correct table must satisfy against the real gate:
table-says-unburnable-but-gate-says-burnable, and table-says-burnable-but-
gate-refuses. Both must be the empty set.

The output is CHECKED IN (`scene_gen/config/harvested/`, unlike `_plans/`
which is gitignored -- see `.gitignore:116`) because `districts.py` reads it
at layout-gen time and must not depend on this tool having been run in the
current checkout.
"""
import argparse
import json
import os
import sys

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TOOLS_DIR)
if _SCENE_GEN_DIR not in sys.path:
    sys.path.insert(0, _SCENE_GEN_DIR)
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

DEFAULT_OUT = os.path.join(_SCENE_GEN_DIR, "config", "harvested",
                          "burnability_table.json")


def _typology_pools(config, typology_names=None):
    """`{typology_name: [pool_name, ...]}` -- the SAME map `rezone_blocks`/
    `infill_blocks` read (`districts.typologies.<name>.pools`, defaulting to
    `[name]` when a typology doesn't override it, matching `infill_blocks.
    _pool_for`'s own `tc.get("pools") or [tname]`). Terrace typologies are
    included too -- burnability doesn't care about morphology, only about
    which pool an asset is drawn from.
    """
    typs = ((config.get("districts") or {}).get("typologies")) or {}
    names = typology_names or sorted(typs.keys())
    out = {}
    for name in names:
        tc = typs.get(name) or {}
        out[name] = list(tc.get("pools") or [name])
    return out


def _entries_for_pools(config, pool_names):
    bld = (config.get("usds") or {}).get("buildings") or {}
    out = []
    for name in pool_names:
        for e in bld.get(name, []) or []:
            if isinstance(e, str):
                out.append({"usd": e, "scale": 1.0})
            elif isinstance(e, dict) and e.get("usd"):
                out.append(e)
    return out


def build_table(preset: str, typology_names=None):
    """`[(typology, basename, usd, ok, reason_or_None)]` -- one row per
    (typology, unique asset in that typology's pool(s)).
    """
    import compile_disaster
    import scene_generator as sg
    import fire_city_dry_run as fcd
    from disaster import gac_fire as gf
    from disaster import kit_substitute as ks
    from disaster import quake as q
    from disaster import urban_fire_city as ufc

    import seed_standalone_cache as ssc

    config = compile_disaster.load_scene_config(
        preset, spec_overrides={"disaster-type": "none"})
    fcd._localize_building_urls(config["usds"])
    resolver = sg._make_resolver(config)
    fcd._patch_resolver_for_gac_dtc(resolver)
    # REAL sizes for the same "standalone"/Muyang/Dmytro assets the host
    # packer now seeds (`tools/seed_standalone_cache.py`) -- without this,
    # a `same_art` asset's `best_style()` kit-twin match (and, in general,
    # any size-sensitive gate) is judged against the WRONG, generic 30 x
    # 20 x 24 m fallback instead of its measured footprint, and the table
    # can disagree with what the guard sees at swap time for exactly the
    # assets this seeding exists for.
    ssc.seed_resolver(resolver)
    cache = fcd._gac_dtc_cache()

    pools_by_typ = _typology_pools(config, typology_names)

    verdicts = []
    for typ_name, pool_names in pools_by_typ.items():
        # Covers every (x, y) for THIS typology's own synthetic layout --
        # the placement/typology gates are position-only and this tool
        # wants exactly one verdict per (asset, typology), so the layout
        # must never itself be the reason for a refusal.
        layout = {"_typology_of": {(-1.0e9, -1.0e9, 1.0e9, 1.0e9): typ_name}}

        entries = _entries_for_pools(config, pool_names)
        seen = {}          # usd -> scale, first one wins
        for e in entries:
            usd = e["usd"]
            if usd not in seen:
                seen[usd] = float(e.get("scale", 1.0))

        for usd, scale in seen.items():
            placement = {"category": "house", "usd": usd, "x_m": 0.0,
                        "y_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0,
                        "scale": scale, "axis_up": "Z",
                        "prim_path": "/synthetic/burnability_probe"}
            # Mirrors `fire_city_dry_run.gather_burnable`'s own call exactly
            # -- `route()`'s `best_style(..., prefer_type=btype)` is the ONE
            # place a `same_art` asset's construction-type tag is read.
            if ks.pack_of(usd) == "same_art":
                placement["btype"] = q._same_art_material(usd, config)
            W, D, H = fcd._measure_wdh(usd, placement, resolver, cache, gf)
            ok, result = ufc.burnable(layout, placement, {usd: (W, D, H)})
            basename = fcd._basename_noext(usd)
            verdicts.append((typ_name, basename, usd, bool(ok),
                            None if ok else result))

    return verdicts


def prove_against_real_gate(preset: str, seed: int, table: dict):
    """Builds the REAL host layout and cross-checks *table* (`{typology:
    {basename: bool}}`) against `fire_city_dry_run.gather_burnable`'s
    per-placement verdicts on that SAME layout, indexed by POSITION (the
    dump/placements list carries no `"i"` key of its own -- `enumerate`).
    Prints, and returns, the two sets that must both be empty for the table
    to be sound: table-unburnable-but-gate-burnable (false firebreaks) and
    table-burnable-but-gate-refused (false fuel).
    """
    from detail import districts as dd
    import fire_city_dry_run as fcd

    config, layout, placements, resolver = fcd.build_layout(preset, seed=seed)
    burnable, refused, _btyp = fcd.gather_burnable(config, layout, placements,
                                                   resolver)
    gate_burnable_idx = {i for i, _rec in burnable}
    gate_refused_idx = {r["i"]: r["reason"] for r in refused}

    typ_of = layout.get("_typology_of") or {}

    def typology_of_point(x, y):
        for (x0, y0, x1, y1), name in typ_of.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
        return None

    false_firebreak = []       # table unburnable, gate burnable
    false_fuel = []             # table burnable, gate refused
    for i, p in enumerate(placements):
        if p.get("category") != "house":
            continue
        typ = typology_of_point(p["x_m"], p["y_m"])
        basename = dd._asset_basename(p["usd"])
        tbl_unb = bool((table.get(typ) or {}).get(basename) is False)
        if i in gate_burnable_idx and tbl_unb:
            false_firebreak.append((i, typ, basename))
        if i in gate_refused_idx and not tbl_unb:
            false_fuel.append((i, typ, basename, gate_refused_idx[i][:90]))

    print(f"[prove] {sum(1 for p in placements if p.get('category') == 'house')} "
          f"house placements; gate: {len(gate_burnable_idx)} burnable, "
          f"{len(gate_refused_idx)} refused")
    print(f"[prove] table-unburnable (cap FALSE FIREBREAK) intersect "
          f"gate-burnable: {len(false_firebreak)} (must be 0)")
    for row in false_firebreak:
        print("    ", row)
    print(f"[prove] table-burnable intersect gate-refused (FALSE FUEL): "
          f"{len(false_fuel)} (must be 0)")
    for row in false_fuel:
        print("    ", row)
    return false_firebreak, false_fuel


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--preset", default="downtown_fire_500")
    ap.add_argument("--typologies", default=None,
                    help="comma-separated typology names (default: every "
                         "typology `districts.typologies` defines)")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--prove", action="store_true",
                    help="also cross-check the freshly built table against "
                         "the real gate on the seed's own host layout")
    ap.add_argument("--seed", type=int, default=4)
    args = ap.parse_args()

    typ_names = ([t.strip() for t in args.typologies.split(",") if t.strip()]
                if args.typologies else None)
    verdicts = build_table(args.preset, typ_names)

    verdicts.sort(key=lambda v: (v[0], v[1]))
    table: dict = {}
    for typ_name, basename, _usd, ok, _reason in verdicts:
        table.setdefault(typ_name, {})[basename] = ok

    n_true = sum(1 for typ in table.values() for v in typ.values() if v)
    n_false = sum(1 for typ in table.values() for v in typ.values()) - n_true
    print(f"[gen_burnability_table] preset={args.preset!r} "
          f"typologies={sorted(table.keys())}")
    print(f"[gen_burnability_table] {n_true + n_false} (typology, asset) "
          f"row(s): {n_true} burnable, {n_false} unburnable")
    for typ_name, basename, _usd, ok, reason in verdicts:
        if not ok:
            print(f"    UNBURNABLE  {typ_name:14s} {basename:24s} {reason}")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    payload = {
        "_generated_by": "scene_gen/tools/gen_burnability_table.py",
        "_preset": args.preset,
        "_regenerate": "python3 scene_gen/tools/gen_burnability_table.py "
                       "--preset {0}".format(args.preset),
        "_schema": "assets[typology][asset_basename] -> bool",
        "assets": table,
    }
    with open(args.out, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
        fh.write("\n")
    print(f"[gen_burnability_table] wrote {args.out}")

    if args.prove:
        false_firebreak, false_fuel = prove_against_real_gate(
            args.preset, args.seed, table)
        if false_firebreak or false_fuel:
            print("[gen_burnability_table] PROOF FAILED -- table disagrees "
                  "with the real gate, see rows above")
            return 1
        print("[gen_burnability_table] PROOF OK -- both intersections empty")
    return 0


if __name__ == "__main__":
    sys.exit(main())
