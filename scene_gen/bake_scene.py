#!/usr/bin/env python3
"""Entrypoint 1 — generate a scene offline and cache it.

    python3 scene_gen/bake_scene.py --config urban_quake_tiny
    python3 scene_gen/bake_scene.py --config earthquake --severity 0,0.4,0.8

Needs only `usd-core` — no Isaac Sim, no Kit. Writes into the two-tier cache
(`scene_cache.py`), so the pristine layout is computed once and every severity
hangs off it.

WHY A SWEEP IS THE INTERESTING CASE
-----------------------------------
`--severity a,b,c` recompiles the same spec at each value and writes each as a
child of ONE pristine entry. That is not a convenience: the layout is identical
across them by construction, which is what makes comparing a search algorithm
across severities mean anything. Running three separate commands would produce
three pristine entries that happen to match; this produces one, and a mismatch
would be visible as a second tier-1 key rather than as a subtle difference
nobody looks for.

WHAT "OFFLINE" DOES AND DOES NOT COVER
--------------------------------------
Covered: everything that places geometry — layout, detail, disaster fate,
debris, ground, markings, and referencing Stage A archetypes where a library
is baked. The result is a complete, loadable scene.

Not covered: the PhysX settle, and there are TWO different things wearing that
name. They are worth keeping apart, because Stage A only removes one of them.

  * FRACTURE FRAGMENTS from live mesh damage (`debris_fragment`). These are
    cut where the geometry was and genuinely need physics. A baked Stage A
    library removes them entirely — the archetype was settled once, at bake
    time — which is Stage B's "no per-scene physics" invariant.

  * APPROXIMATED PROP POSES — scattered debris, toppled poles, flipped cars.
    The disaster stage places these at plausible-but-approximate poses and
    marks them `settle: True` so a Kit pass can refine them. Stage A does NOT
    remove this: they are placements, not fractures. A tiny scene shows ~36 of
    them and they look fine unsettled; a severe city shows thousands and the
    refinement is visible.

So an offline scene is loadable either way. `--require-archetypes` guards the
first case only, and the report counts the two separately.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys

_SCENE_GEN = os.path.dirname(os.path.abspath(__file__))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def _severities(arg: str):
    if not arg:
        return [None]
    return [float(v) for v in str(arg).split(",") if v.strip() != ""]


def _compile(name: str, severity):
    """The compiled config for *name*, optionally re-pinned to *severity*."""
    import yaml

    import compile_disaster as cd
    import scene_generator as sg

    if severity is None:
        return sg.resolve_asset_pack(cd.load_scene_config(name))

    path = cd.resolve_config_path(name)
    with open(path) as fh:
        spec = yaml.safe_load(fh) or {}
    if not cd.is_high_level(spec):
        raise SystemExit(
            f"--severity needs a high-level preset; {name} is already compiled")
    spec = dict(spec)
    spec["severity"] = float(severity)
    with open(cd.DEFAULT_BASE) as fh:
        base = yaml.safe_load(fh)
    return sg.resolve_asset_pack(cd.compile_spec(spec, base))


def bake_one(config: dict, cache, resolver, force: bool,
             require_archetypes: bool, allow_fallback: bool = False) -> dict:
    """Build one scene into the cache. Returns a small report."""
    from pxr import Usd, UsdGeom

    import generate_scene as gs
    import scene_cache as sc

    key = (f"{sc.pristine_key(config)}"
           + ("" if sc.is_pristine(config)
              else f"/{sc.disaster_key(config)}"))

    existing = cache.get(config)
    if existing and not force:
        return {"key": key, "usd": existing, "cached": True}

    if require_archetypes:
        from disaster import disaster_stage as ds
        dtype = str((config.get("disaster") or {}).get("type", "none"))
        if dtype != "none" and ds._archetypes(dtype) is None:
            raise SystemExit(
                f"--require-archetypes: no Stage A library for {dtype!r}. "
                f"Bake one with archetypes/bake_cli.py --disaster {dtype}, or "
                f"drop the flag to accept a scene whose loose debris still "
                f"needs the sim to settle.")

    out = cache.reserve(config)
    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    # Which assets this scene had to GUESS the footprint of. Snapshot first:
    # the resolver is shared across a severity sweep, so an earlier scene's
    # guesses must not be attributed to this one.
    guessed_before = set(getattr(resolver, "fallbacks", ()))
    placements = gs.generate_scene_on_stage(stage, config, "/World/generated",
                                            1.0, resolver=resolver)
    stage.GetRootLayer().Save()

    guessed = sorted(set(getattr(resolver, "fallbacks", ())) - guessed_before)
    if guessed:
        if not allow_fallback:
            # Discard rather than keep: a half-written entry that a later run
            # finds and serves is exactly the failure this guard exists for.
            shutil.rmtree(cache.scene_dir(config), ignore_errors=True)
            raise SystemExit(
                f"[bake_scene] REFUSING to cache {key}: {len(guessed)} asset(s) "
                f"had no measurable footprint, so this scene's LAYOUT is not "
                f"the layout this config names — footprints drive block "
                f"sizing.\n  " + "\n  ".join(guessed[:10])
                + (f"\n  … and {len(guessed) - 10} more" if len(guessed) > 10 else "")
                + "\n\nA plain `python3` cannot open a Nucleus asset. Bake where "
                  "the assets are reachable, or warm scene_gen/assets/"
                  ".measurements.json from a run that can reach them. "
                  "--allow-footprint-fallback overrides this, and marks the "
                  "entry unservable so nothing loads it by accident.")
        cache.mark_footprint_fallback(config, guessed)
        print(f"[bake_scene] WARNING: {key} built with {len(guessed)} guessed "
              f"footprint(s); marked unservable.")

    # Counted apart because only the first is a correctness gap — see the
    # module docstring.
    frags = sum(1 for p in placements
                if p.get("settle") and p.get("category") == "debris_fragment")
    approx = sum(1 for p in placements
                 if p.get("settle") and p.get("category") != "debris_fragment")
    arche = sum(1 for p in placements if p.get("_archetype"))
    return {"key": key, "usd": out, "cached": False,
            "placements": len(placements), "archetypes": arche,
            "fragments": frags, "approx": approx}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Generate a scene offline into the two-tier cache.")
    ap.add_argument("--config", required=True, help="preset name or path")
    ap.add_argument("--severity", default="",
                    help="comma-separated severities to sweep, e.g. 0,0.4,0.8")
    ap.add_argument("--root", default="", help="cache root")
    ap.add_argument("--force", action="store_true",
                    help="rebuild even when cached")
    ap.add_argument("--require-archetypes", action="store_true",
                    help="fail unless a Stage A library exists, so the result "
                         "needs no PhysX settle")
    ap.add_argument("--allow-footprint-fallback", action="store_true",
                    help="bake even when a footprint had to be guessed. The "
                         "entry is marked unservable, so it is for inspection "
                         "and tests, not for loading.")
    ap.add_argument("--list", action="store_true",
                    help="show what would be built, and exit")
    args = ap.parse_args(argv)

    import scene_cache as sc
    import scene_generator as sg
    from measure_cache import MeasureCache

    cache = sc.SceneCache(args.root)
    configs = [_compile(args.config, s) for s in _severities(args.severity)]

    if args.list:
        for cfg in configs:
            dis = cfg.get("disaster") or {}
            print(f"  {sc.pristine_key(cfg)}"
                  f"{'' if sc.is_pristine(cfg) else '/' + sc.disaster_key(cfg)}"
                  f"  {cfg.get('locale')} {dis.get('type')} "
                  f"sev={dis.get('severity')}  "
                  f"{'CACHED' if cache.get(cfg) else 'to build'}")
        return 0

    # One measurement cache and one resolver across the whole sweep: every
    # scene in it draws on the same asset pack, so re-measuring per severity
    # would be the dominant cost of a sweep.
    mc = MeasureCache()
    resolver = sg._make_resolver(configs[0], cache=mc)

    reports = []
    for cfg in configs:
        rep = bake_one(cfg, cache, resolver, args.force,
                       args.require_archetypes, args.allow_footprint_fallback)
        reports.append(rep)
        if rep["cached"]:
            print(f"[bake_scene] {rep['key']}  cached  {rep['usd']}")
        else:
            print(f"[bake_scene] {rep['key']}  built   {rep['usd']}  "
                  f"({rep['placements']} placements, "
                  f"{rep['archetypes']} archetypes, "
                  f"{rep['fragments']} fragments, "
                  f"{rep['approx']} approximated poses)")

    # The layout belongs to tier 1 and is shared, so write it once.
    import generate_scene as gs
    _pl, layout, _c = gs.build_scene(configs[0], resolver, stop_after="layout")
    cache.put_layout(configs[0], {
        "region": layout.get("region"),
        "blocks": [list(b) for b in (layout.get("blocks") or [])],
        "road_corridors": [list(r) for r in
                           (layout.get("road_corridors") or [])],
    })
    mc.save()

    frags = sum(r.get("fragments", 0) for r in reports if not r["cached"])
    approx = sum(r.get("approx", 0) for r in reports if not r["cached"])
    if frags:
        print(f"[bake_scene] {frags} live fracture fragment(s) need a PhysX "
              f"settle. Bake a Stage A library to remove them entirely.")
    if approx:
        print(f"[bake_scene] {approx} prop(s) sit at approximated poses "
              f"(scattered debris, toppled poles, flipped cars). A Kit settle "
              f"pass refines them; Stage A does not, and they load fine as-is.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
