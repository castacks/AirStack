#!/usr/bin/env python3
"""Stage A, offline. Bake an archetype library and exit.

    # everything one scene needs, on the host
    scene_gen/archetypes/bake_cli.py --config urban_quake_tiny --used-only

    # every disaster, whole pack — the real Stage A, once
    scene_gen/archetypes/bake_cli.py --config urban --disaster all

SPEC calls Stage A "once; exhaustive; layout-independent", which means it is a
BATCH JOB: it must run unattended, report what it did, and exit with a status
code. The previous entry point could not — it was an Isaac Sim launch script
that opened a GUI and ended in `while simulation_app.is_running(): app.update()`,
so it never terminated and could not be scripted, scheduled or chained.

WHERE IT RUNS
-------------
Anywhere Isaac Sim's Python is importable. Two known-good ways:

    # HOST — AirStack/.venv carries Isaac Sim 5.1. `.env.host` supplies the
    # EULA vars, without which Kit prompts on stdin and dies with
    # "Unable to bootstrap inner kit kernel: EOF when reading a line".
    set -a; . AirStack/.env.host; set +a
    AirStack/.venv/bin/python scene_gen/archetypes/bake_cli.py --config fire

    # CONTAINER
    docker exec isaac-sim bash -c \\
      "cd /isaac-sim && ./python.sh AirStack/scene_gen/archetypes/bake_cli.py \\
       --config fire"

Headless by default; `--gui` opens a window to watch a bake you are debugging.

COST
----
Measured ~40 s per library archetype, so a full pack is hours: `--config urban
--disaster earthquake` is 198 archetypes, about 2.2 h. Check before you commit:

    python3 scene_gen/archetypes/plan.py --config urban --disaster earthquake

`--used-only` bakes just the types one scene places, and `--only` takes an
explicit list. Both are how you try one asset before paying for all of them.

Prefer `--used-only` over pasting a list from `plan.py`: the set depends on
measured footprints, and a plain `python3` cannot open a Nucleus asset, so it
packs differently and names different buildings (measured: 5 types on the host
against 2 under Kit for the same config). This flag resolves the plan after
Isaac Sim has started, so it sees what the scene will actually place.

EXIT CODE
---------
0 when every planned archetype exported, 1 otherwise. A partial library is
still usable — Stage B falls back down the ladder and says so — but a batch
job should not report success for it.
"""

from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def _parse(argv=None):
    ap = argparse.ArgumentParser(
        prog="bake_cli.py",
        description="Stage A: bake the archetype library offline.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True,
                    help="scene config naming the asset pack (preset name, "
                         "high- or low-level path)")
    ap.add_argument("--disaster", default="",
                    help="disaster to bake; 'all' bakes every type. "
                         "Default: the config's own type.")
    ap.add_argument("--out", default="",
                    help="output root (default: scene_gen/assets/archetypes)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--skip-existing", action="store_true",
                    help="leave alone any archetype already baked (per the "
                         "manifest or the bake trace, cross-checked against "
                         "the file); how you resume an interrupted bake, or "
                         "re-cut just what a changed knob affects")
    ap.add_argument("--only", default="",
                    help="comma-separated type slugs to bake")
    ap.add_argument("--cells", default="",
                    help="comma-separated TYPE:LEVEL pairs — bake exactly "
                         "those rungs and nothing else (what the gallery "
                         "picker sends). Unlike --only x --levels this is not "
                         "a cross product: picking A:cracked and B:pancaked "
                         "bakes two cells, not four")
    ap.add_argument("--used-only", action="store_true",
                    help="only the types this scene actually places "
                         "(seed-specific — see plan.used_by_scene)")
    ap.add_argument("--census", default="",
                    help="a census written by a scene run under Kit "
                         "(SCENE_CENSUS=). Marks `used_by` on every record it "
                         "recognises, and BAKES THE USED TYPES FIRST, "
                         "most-placed first — so a bake stopped early has "
                         "finished the archetypes a scene actually needs.")
    ap.add_argument("--census-only", action="store_true",
                    help="with --census, bake ONLY the types it saw placed")
    ap.add_argument("--exclude", default="",
                    help="comma-separated type slugs to SKIP. For an asset "
                         "the pipeline cannot currently cut: `SM_Building_28` "
                         "spun inside vtkClipClosedSurface for 44 minutes on "
                         "its `cracked` rung, at constant memory and "
                         "unresponsive to SIGINT and SIGTERM, so the "
                         "try/except fallback to numpy never fired (a spin "
                         "raises nothing). Excluding it costs the two "
                         "placements it has in the 1 km layout and keeps the "
                         "rest of the bake at VTK speed.")
    ap.add_argument("--gui", action="store_true",
                    help="open a window; default is headless")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and exit without starting Isaac Sim")
    return ap.parse_args(argv)


def _disasters(arg: str, config: dict) -> list:
    import compile_disaster as cd

    if str(arg).lower() == "all":
        return sorted(cd.DISASTERS)
    if arg:
        return [str(arg).lower()]
    return [str((config.get("disaster") or {}).get("type") or "none").lower()]


def _select(config, dtype, args, census=None):
    """The (type, kind) set to bake, or None for everything."""
    from archetypes import plan as P
    from archetypes import census as C

    items = P.build_plan(config, dtype)
    if args.used_only:
        items = P.used_by_scene(config, items)
    if census:
        # USED FIRST, MOST-PLACED FIRST. Not a filter unless asked
        # (`--census-only`): the rest of the pack is still worth baking, it is
        # just worth baking second. A bake stopped by the disk or the morning
        # then has the archetypes the scene actually references.
        rank = {t: i for i, t in enumerate(C.used_types(census))}
        if args.census_only:
            items = [i for i in items if i.type in rank]
        items.sort(key=lambda i: rank.get(i.type, len(rank)))
        hit = sum(1 for i in items if i.type in rank)
        print(f"[stage-a] census '{census.get('config', 'scene')}': "
              f"{hit} of {len(items)} planned types are placed by that scene")
    if args.only:
        # THE ORDER OF --only IS THE BAKE ORDER. `tools/bake_order.py` emits
        # the list cheapest-first so that a bake stopped by the disk or by the
        # morning has completed whole assets rather than a third of a tower;
        # collapsing it to a set here threw that away silently.
        want = [t.strip() for t in args.only.split(",") if t.strip()]
        rank = {t: i for i, t in enumerate(want)}
        items = [i for i in items if i.type in rank]
        items.sort(key=lambda i: rank[i.type])
        missing = set(want) - {i.type for i in items}
        if missing:
            print(f"[stage-a] WARNING: --only named {sorted(missing)}, which "
                  f"this config does not plan", file=sys.stderr)
    if args.cells:
        # EXACT (type, level) PAIRS. The picker selects individual rungs, and
        # a type filter crossed with a level filter cannot express that: it
        # would re-cut rungs the user did not choose, which on this pipeline
        # costs minutes each and overwrites files they may have hand-posed.
        want = {}
        for tok in args.cells.split(","):
            tok = tok.strip()
            if not tok:
                continue
            t, _, lv = tok.partition(":")
            want.setdefault(t.strip(), set()).add(lv.strip())
        items = [i for i in items if i.type in want]
        for i in items:
            i.levels = [lv for lv in i.levels if lv in want[i.type]]
        items = [i for i in items if i.levels]
        got = {(i.type, lv) for i in items for lv in i.levels}
        missing = {(t, lv) for t, lvs in want.items() for lv in lvs} - got
        if missing:
            print(f"[stage-a] WARNING: --cells named {sorted(missing)}, which "
                  f"this config does not plan", file=sys.stderr)
        print(f"[stage-a] --cells selected {len(got)} rung(s)")
    if args.exclude:
        drop = {t.strip() for t in args.exclude.split(",") if t.strip()}
        before = len(items)
        items = [i for i in items if i.type not in drop]
        print(f"[stage-a] --exclude dropped {before - len(items)} type(s): "
              f"{sorted(drop)}")
    if not (args.used_only or args.only or args.census_only
            or args.exclude or args.cells):
        # `None` means "everything", which discards the census ORDER — so a
        # census run must always pass an explicit selection, even when it
        # selects the whole plan.
        if not census:
            return None, items
    return [(i.type, i.kind) for i in items], items


def _cells_set(spec: str):
    """`--cells` as a set of ``(type, level)``, or None. See `bake._trim_to_cells`."""
    if not spec:
        return None
    out = set()
    for tok in spec.split(","):
        tok = tok.strip()
        if not tok:
            continue
        t, _, lv = tok.partition(":")
        out.add((t.strip(), lv.strip()))
    return out or None


def _plan_all(args):
    """Resolve the config and the per-disaster selections. Imports `pxr`."""
    from compile_disaster import load_scene_config
    from archetypes import plan as P
    from archetypes import census as C

    config = load_scene_config(args.config)
    targets = _disasters(args.disaster, config)

    census = C.read(args.census) if args.census else {}
    if args.census and not census:
        raise SystemExit(f"no census at {args.census} — write one with "
                         f"SCENE_CENSUS= on a scene run under Kit")
    if census:
        print(C.summarise(census))

    selections, total = {}, 0
    for dtype in targets:
        only, items = _select(config, dtype, args, census)
        selections[dtype] = only
        print(P.summarise(items, dtype))
        total += sum(len(i.levels) for i in items)
    if len(targets) > 1:
        print(f"\n{total} archetypes across {len(targets)} disasters")
    return config, targets, selections, total, census


def main(argv=None) -> int:
    args = _parse(argv)

    # --dry-run never starts Kit, so it is free to import `pxr` here.
    if args.dry_run:
        _c, _t, _s, total, _cen = _plan_all(args)
        if not total:
            print("[stage-a] nothing to bake", file=sys.stderr)
            return 1
        return 0

    # ---- Isaac Sim from here on -----------------------------------------
    # SIMULATIONAPP MUST BE CONSTRUCTED BEFORE ANYTHING IMPORTS `pxr`.
    # `archetypes.plan` imports `scene_generator`, which imports `pxr` at
    # module scope; pulling usd-core into the process first stops Kit
    # prepending its own USD build, and Kit then dies — here it segfaulted in
    # `omni::usd::UsdContext::getNameSv` about three seconds into startup,
    # with the plan already printed and no other clue. `asset_pack.py`
    # documents the same hazard for the same reason.
    #
    # So the plan is resolved AFTER the boot, which costs a bad `--config` the
    # ~30 s Kit startup before it reports the typo. `--dry-run` above is the
    # fast path for checking a config, and it never boots Kit at all.
    try:
        from isaacsim import SimulationApp
    except ImportError as exc:
        print(f"[stage-a] Isaac Sim is not importable ({exc}).\n"
              f"  host:      set -a; . AirStack/.env.host; set +a  then use "
              f"AirStack/.venv/bin/python\n"
              f"  container: docker exec isaac-sim bash -c "
              f"'cd /isaac-sim && ./python.sh <this script>'",
              file=sys.stderr)
        return 2

    app = SimulationApp(launch_config={"headless": not args.gui})
    try:
        import omni.usd
        from archetypes import bake as stage_a

        config, targets, selections, total, census = _plan_all(args)
        if not total:
            print("[stage-a] nothing to bake", file=sys.stderr)
            return 1

        failed = 0
        for dtype in targets:
            ctx = omni.usd.get_context()
            ctx.new_stage()
            stage_a.prepare_stage(ctx.get_stage())
            res = stage_a.run(ctx.get_stage(), config, dtype,
                              out_dir=args.out, seed=args.seed,
                              only=selections[dtype],
                              skip_existing=args.skip_existing,
                              census=census, cells=_cells_set(args.cells))
            failed += res["wanted"] - res["baked"]
    except BaseException:
        # PRINT IT FIRST. `SimulationApp.close()` hard-exits the process (see
        # the note below), so the `raise` under it never runs and the traceback
        # never reaches the log: a bake that died on its third cell looked
        # exactly like one that finished, right down to the exit code. Three
        # runs were spent theorising about process contention and physics
        # before anyone noticed the error was simply being swallowed.
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        app.close()
        raise

    if failed:
        print(f"[stage-a] {failed} archetype(s) did not export", file=sys.stderr)

    # EXIT BEFORE CLOSING KIT, and do not fall through to `return`.
    #
    # `SimulationApp.close()` hard-exits the process (the app is launched with
    # `/app/fastShutdown`), so anything after it never runs and the exit code
    # is whatever Kit chose — always 0. A 43-minute bake that dropped 4 of 16
    # archetypes reported success, which is precisely the state a batch job
    # must not report.
    #
    # Everything is already durable at this point: each archetype was saved as
    # it was exported and the manifest is written. Skipping the tidy shutdown
    # costs nothing the OS will not reclaim, and it is the only way to own the
    # exit code.
    code = 1 if failed else 0
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == "__main__":
    sys.exit(main())
