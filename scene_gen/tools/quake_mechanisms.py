#!/usr/bin/env python3
"""quake_mechanisms.py — every earthquake MECHANISM on its own, then combined.

`tools/quake_preview.py` and `tools/damage_spread.py` both render the LADDER,
which is what ships. This renders the thing the ladder is assembled from.

A rung is a set of mechanisms over regions of the plan (`quake.RUNG_PLAN`), so
when a rung looks wrong there are two different questions and the ladder cannot
separate them: is the mechanism itself wrong, or is the composition wrong? A
grid with one column per mechanism ALONE, followed by the pairs, answers both —
and it is the only view in which "the soft storey is fine, it is `crack` that
was dicing the block above it" is a statement you can see rather than infer.

    cd AirStack
    uv run --env-file .env.host python scene_gen/tools/quake_mechanisms.py
    ... --assets bg_c --columns soft_storey,soft_storey+crack
    ... --list                                    # what it would do, no Isaac

Then render the sheet (separate 3.13 + `bpy` env):

    cd scene_gen && uv run --script tools/render_damage_gallery.py \
        _damage_lab/mechanisms/manifest.json --res 560

RUNS ON THE HOST. `AirStack/.venv` carries Isaac Sim 5.1; `.env.host` supplies
OMNI_KIT_ACCEPT_EULA. Assets must be LOCAL (the mirror under
`assets/nucleus/`), because Blender cannot resolve `omniverse://` either.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_TOOLS)
for _p in (_SCENE_GEN, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from damage_spread import ASSETS, _MIRROR                      # noqa: E402

OUT_DIR = os.path.join(_SCENE_GEN, "_damage_lab", "mechanisms")

#: THE COMBINATIONS WORTH SEEING, and why each one earns a column.
#:
#: Isolation first, at `share` 1.0 — the whole footprint, no sector mask — so
#: the column shows what the mechanism DOES rather than what a wedge of it
#: looks like next to something else. Then the pairs, at the shares the ladder
#: actually uses, because a mechanism sharing a plan with another is a
#: different picture: the boundary between them is the part that reads as
#: damage, and it does not exist in either column on its own.
COMBOS = [
    ("crack",                  [("crack", 1.00)]),
    ("soft_storey",            [("soft_storey", 1.00)]),
    ("shear_off",              [("shear_off", 1.00)]),
    ("pancake",                [("pancake", 1.00)]),
    # A sheared wing with the rest of the building merely cracked — the
    # untouched half is the one that has to keep its own textures and its
    # clean vertical face where the cut stopped.
    ("shear_off+crack",        [("shear_off", 0.45), ("crack", 1.00)]),
    # A ground floor gone under part of the plan, cracks over the rest. The
    # test of the support graph: the block above the failed storey has to come
    # down as a block while the cracked part stands.
    ("soft_storey+crack",      [("soft_storey", 0.55), ("crack", 1.00)]),
    # Two collapse mechanisms side by side, which is what the top of the
    # ladder is: does the boundary between them read as an edge or a smear?
    ("pancake+shear_off",      [("pancake", 0.60), ("shear_off", 0.45)]),
    ("soft_storey+shear_off",  [("soft_storey", 0.45), ("shear_off", 0.45)]),
]


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--assets", default="bg_c,bg_f",
                   help="comma-separated keys; "
                        f"any of {','.join(k for k, _, _, _ in ASSETS)}")
    p.add_argument("--columns", default="",
                   help="comma-separated column names; default all")
    p.add_argument("--steps", type=int, default=4000, help="settle ceiling")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--out", default=OUT_DIR)
    p.add_argument("--list", action="store_true",
                   help="print the grid and exit; starts no Isaac Sim")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    keys = [k.strip() for k in args.assets.split(",") if k.strip()]
    rows = [a for a in ASSETS if not keys or a[0] in keys]
    want = {c.strip() for c in args.columns.split(",") if c.strip()}
    cols = [c for c in COMBOS if not want or c[0] in want]

    if args.list:
        print(f"{len(rows)} asset(s) x {1 + len(cols)} column(s) = "
              f"{len(rows) * (1 + len(cols))} cells")
        for key, label, path, _scale in rows:
            here = os.path.join(_MIRROR, path)
            print(f"  {key:5s} {'ok ' if os.path.exists(here) else 'MISSING'} "
                  f"{label}")
        for name, plan in cols:
            print(f"  {name:24s} {plan}")
        return 0

    missing = [k for k, _l, p, _s in rows
               if not os.path.exists(os.path.join(_MIRROR, p))]
    if missing:
        print(f"[mech] not in the local mirror: {missing}\n"
              f"  run tools/localize_nucleus_assets.py first", file=sys.stderr)
        return 2

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True})

    from _lab_stage import export, fresh_stage                 # noqa: E402
    from disaster import mesh_damage as md                     # noqa: E402
    from disaster import quake                                 # noqa: E402

    usd_dir = os.path.join(args.out, "usd")
    os.makedirs(usd_dir, exist_ok=True)
    manifest = {
        "title": "Earthquake mechanisms, alone and combined",
        "subtitle": ("one column per quake.MECHANISMS entry at share 1.0, "
                     "then the pairs; note = cells / loose / slabs / seconds"),
        "columns": ["pristine"] + [c[0] for c in cols],
        "rows": [],
    }

    t_all = time.time()
    for key, label, rel, scale in rows:
        src = os.path.join(_MIRROR, rel)
        row = {"name": label, "cells": {}, "stats": {}}

        stage = fresh_stage(src, scale)
        bnd = md.bounds_of(md.mesh_prims(stage.GetPrimAtPath("/World/Building")))
        row["cells"]["pristine"] = os.path.relpath(
            export(stage, usd_dir, f"{key}_pristine"), args.out)
        row["stats"]["pristine"] = {
            "footprint_m": [round(float(bnd.dims[0])),
                            round(float(bnd.dims[1]))] if bnd else ["?", "?"],
            "height_m": round(float(bnd.dims[2])) if bnd else "?",
            "note": "reference",
        }

        for name, plan in cols:
            stage = fresh_stage(src, scale)
            prim = stage.GetPrimAtPath("/World/Building")
            seed = args.seed + md.stable_seed(key, name)
            t = time.time()
            try:
                # `plan` overrides the ladder: `name` is only a label here.
                rep = quake.at_level(stage, prim, name, seed=seed,
                                     plan=plan, steps=args.steps)
            except Exception as exc:                            # noqa: BLE001
                # One bad (asset, column) must not cost the rest of the grid.
                print(f"[mech] SKIP {key} {name}: "
                      f"{type(exc).__name__}: {exc}", flush=True)
                continue
            took = time.time() - t
            st = rep.get("settle") or {}
            n_slab = len(rep.get("slabs") or ())
            row["cells"][name] = os.path.relpath(
                export(stage, usd_dir, f"{key}_{name}"), args.out)
            row["stats"][name] = {
                "seconds": round(took, 1),
                "solve_s": round(float(st.get("solve_s", 0.0)), 1),
                "mechanisms": [m for m, _ in plan],
                "cells": rep.get("cells", 0),
                "fragments": len(rep.get("loose") or ()) - n_slab,
                "slabs": n_slab,
                "consumed": rep.get("consumed", 0),
                # A MEDIAN, not the mean. `drop_mean` is dragged positive by a
                # handful of launched pieces and stops describing the pile —
                # see the note in QUAKE_STATE.md.
                "drop_m": round(float(st.get("drop_median",
                                             st.get("drop_mean", 0.0))), 2),
                "spread_m": round(float(st.get("spread_mean", 0.0)), 2),
                "still_moving": st.get("still_moving", 0),
                "note": (f"{rep.get('cells', 0)} cells · "
                         f"{len(rep.get('loose') or ()) - n_slab} loose · "
                         f"{n_slab} slab · {took:.0f}s"),
            }
            print("[mech] {0:5s} {1:24s} {2:6.1f}s  cells={3:<5d} "
                  "loose={4:<5d} slabs={5:<3d} drop={6:+.2f} m".format(
                      key, name, took, rep.get("cells", 0),
                      len(rep.get("loose") or ()) - n_slab, n_slab,
                      float(st.get("drop_median", st.get("drop_mean", 0.0)))),
                  flush=True)

        manifest["rows"].append(row)
        # Written after every ROW: a grid this size runs for hours and a
        # partial sheet is still worth rendering.
        with open(os.path.join(args.out, "manifest.json"), "w") as fh:
            json.dump(manifest, fh, indent=1)

    print(f"[mech] {sum(len(r['cells']) for r in manifest['rows'])} cells in "
          f"{(time.time() - t_all) / 60.0:.1f} min -> {args.out}")
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
