#!/usr/bin/env python3
"""
measure_standalone_via_nucleus.py -- real Nucleus measurement for the
`downtown_fire_500` building pools' assets that a HOST build cannot measure
at all (`standalone/buildings/...`, Muyang `BG_Building_*` / `SM_MERGED_BP_
MBuilding*`, Dmytro `Building_Type*`): none of these are locally mirrored,
so `scene_generator.SizeResolver` falls back to a generic 30 x 20 x 24 m box
for every one of them on a bare host checkout.

MUST BE RUN INSIDE THE isaac-sim CONTAINER, via `usd_python.sh` (bare
python, no `SimulationApp` -- the `nucleus-usd-without-kit` pattern, "safe
beside a running sim"). A bare host `pip install usd-core` has no
`omni_usd_resolver` plugin and cannot open an `omniverse://` URL at all
(confirmed: `Usd.Stage.Open(...)` raises `Failed to open layer` for every
one of these paths from the plain host Python) -- this script is the one
place in the repo that reaches past that.

USAGE
-----
    # from the HOST, with an isaac-sim container already running (docker ps):
    docker cp scene_gen/tools/measure_standalone_via_nucleus.py \\
        isaac-sim:/tmp/measure_standalone_via_nucleus.py
    docker exec isaac-sim bash -c \\
        "cd /isaac-sim/AirStack && ./scene_gen/tools/usd_python.sh \\
         /tmp/measure_standalone_via_nucleus.py --preset downtown_fire_500 \\
         --out /tmp/measured.json"
    docker cp isaac-sim:/tmp/measured.json /tmp/measured.json
    # then merge /tmp/measured.json's "records" into
    # scene_gen/config/harvested/standalone_buildings.json (or overwrite --
    # the enumeration below is deterministic given the same preset/pools).

Enumerates every `usds.buildings.<pool>` entry (for the pools the preset's
`districts.typologies` actually reference) whose resolved `usd` is an
`omniverse://` path that is NOT a GAC/DTC pack (those have their own
offline cache, `_plans/gac_buildings.json`/`dtc_buildings.json`) and is not
already on disk (a locally-mirrored kit archetype needs no help), measures
its bbox exactly as `scene_generator._measure_footprint` does (Z-up only --
none of the divergent assets here are Y-up), and writes the RAW (scale 1.0)
`{sx, sy, sz, base, cx, cy, cz}` per usd.

Cannot resolve `airstack://` URIs (the AEC brownstones) -- that scheme needs
a Kit-side resolver extension this bare mode does not load. Harmless for
burnability (their pack-identity 'slice' route is refused regardless of
size, confirmed by `gen_burnability_table.py`'s own module docstring) but
means AEC footprint/packing representativeness on the host is unchanged by
this tool.
"""
import argparse
import json
import os
import sys

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TOOLS_DIR)
if _SCENE_GEN_DIR not in sys.path:
    sys.path.insert(0, _SCENE_GEN_DIR)


def _list_targets(preset, typology_names=None):
    import compile_disaster
    import scene_generator as sg
    from detail import districts as dd
    from disaster import gac_fire as gf

    config = compile_disaster.load_scene_config(
        preset, spec_overrides={"disaster-type": "none"})
    resolver = sg._make_resolver(config)

    typs = ((config.get("districts") or {}).get("typologies")) or {}
    names = typology_names or sorted(typs.keys())
    pool_names = set()
    for name in names:
        tc = typs.get(name) or {}
        pool_names.update(tc.get("pools") or [name])

    seen = {}
    for pool_name in pool_names:
        for e in dd._pool_entries(config, resolver, pool_name):
            usd, sc, au = e[0], e[1], e[2]
            seen[usd] = au

    targets = []
    for usd, axis_up in seen.items():
        if usd.startswith(gf.GAC_DIR) or usd.startswith(gf.DTC_DIR):
            continue
        if usd.startswith("/") and os.path.isfile(usd):
            continue
        if not usd.startswith("omniverse://"):
            continue        # airstack:// etc -- see module docstring
        targets.append({"usd": usd, "axis_up": axis_up})
    return targets


def _measure(usd, axis_up):
    from pxr import Usd, UsdGeom

    try:
        stage = Usd.Stage.Open(usd)
    except Exception as e:                                # noqa: BLE001
        print(f"OPEN FAILED {usd}: {e}")
        return None
    if stage is None:
        print(f"STAGE NONE {usd}")
        return None
    prim = stage.GetDefaultPrim()
    if not (prim and prim.IsValid()):
        children = list(stage.GetPseudoRoot().GetChildren())
        prim = children[0] if children else None
    if not (prim and prim.IsValid()):
        print(f"NO PRIM {usd}")
        return None
    try:
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            useExtentsHint=True)
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    except Exception as e:                                # noqa: BLE001
        print(f"BBOX FAILED {usd}: {e}")
        return None
    if rng.IsEmpty():
        print(f"EMPTY BBOX {usd}")
        return None
    mn, sz = rng.GetMin(), rng.GetSize()
    if axis_up == "Y":
        return {"usd": usd, "axis_up": "Y",
                "sx": sz[0], "sy": sz[2], "sz": sz[1], "base": -mn[1],
                "cx": mn[0] + sz[0] / 2, "cy": 0.0, "cz": mn[2] + sz[2] / 2}
    return {"usd": usd, "axis_up": "Z",
            "sx": sz[0], "sy": sz[1], "sz": sz[2], "base": -mn[2],
            "cx": mn[0] + sz[0] / 2, "cy": mn[1] + sz[1] / 2, "cz": 0.0}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--preset", default="downtown_fire_500")
    ap.add_argument("--typologies", default=None)
    ap.add_argument("--out", default="/tmp/measured_standalone.json")
    args = ap.parse_args()

    typ_names = ([t.strip() for t in args.typologies.split(",") if t.strip()]
                if args.typologies else None)
    targets = _list_targets(args.preset, typ_names)
    print(f"{len(targets)} omniverse:// target(s) to measure")

    out = []
    for t in targets:
        rec = _measure(t["usd"], t["axis_up"])
        if rec is not None:
            out.append(rec)
            print(f"OK {rec['usd'].rsplit('/', 1)[-1]} "
                 f"{rec['sx']:.2f} x {rec['sy']:.2f} x {rec['sz']:.2f}")

    with open(args.out, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"TOTAL MEASURED {len(out)} of {len(targets)} -> {args.out}")


if __name__ == "__main__":
    main()
