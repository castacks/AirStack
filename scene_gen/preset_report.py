#!/usr/bin/env python3
"""
preset_report.py — dry-run the city generator across low-level configs and
print a comparison table (buildings, ruins, debris, casualties, topples, and
how the damage is distributed in space).

Runs the pure-Python layout only — nothing is written to a stage — so it's a
fast way to sanity-check disaster tuning before launching Isaac Sim. Pair it
with compile_disaster.py:

    python3 utils/compile_disaster.py      # presets/ -> low_level/compiled/
    python3 utils/preset_report.py         # compare what those will build

USAGE
-----
    python3 utils/preset_report.py                       # every compiled config
    python3 utils/preset_report.py tornado explosion     # by name
    python3 utils/preset_report.py --configs a.yaml b.yaml
    python3 utils/preset_report.py --seed 7

By default assets are NOT measured (no Nucleus access needed): footprints
come from fallback_sizes, with the house footprint overridden to
--house-fallback (default 15x15 m) so blocks actually pack — absolute counts
therefore differ from a real run, but the comparison between configs is what
matters here.

--measure uses real USD bboxes. omniverse:// URLs only resolve with Isaac
Sim's bundled python (it registers the Nucleus resolver + cached auth), so
run it as:

    ~/isaacsim/python.sh utils/preset_report.py --measure          # local
    /isaac-sim/python.sh utils/preset_report.py --measure          # container

The script starts a headless SimulationApp automatically in that case
(~1 min startup); under plain python3, --measure warns and omniverse://
assets fall back to fallback_sizes.

COLUMNS
-------
    buildings / damaged / destroyed   structures placed, by pool
    debris                            rubble piles + settled fragments —
                                      BUT ONLY THE PART THIS PASS PLACES. A
                                      building the cutter reaches sheds its
                                      debris during the damage phase, from its
                                      own report (`disaster/debris.py`), and a
                                      dry run does no cutting; a baked
                                      archetype carries its debris inside the
                                      USD and is never a placement at all. So
                                      a preset whose buildings are all cut or
                                      all archetyped reads 0 here and is not
                                      wrong. Compare presets, not absolutes.
    prone / downed / flipped          casualties, poles, vehicles
    hit%                              share of the region with damage
                                      intensity >= 0.5 (the disaster's extent)
    core:edge                         ruin rate inside vs outside that zone —
                                      the spatial signature. "uniform" when
                                      the field is flat, so there is no edge.
"""

import argparse
import contextlib
import copy
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

_SCENE_GEN_DIR = os.path.dirname(os.path.abspath(__file__))
COMPILED_DIR = os.path.join(_SCENE_GEN_DIR, "config", "low_level", "compiled")

sg = None            # scene_generator; imported in main() after the optional
                     # SimulationApp bootstrap (pxr must load after it)
_RESOLVER_CACHE = {}  # one shared measuring resolver across config runs


def _resolved_paths(cfg, entries):
    """Config usds entries -> resolved asset paths (asset_root applied)."""
    asset_root = str(cfg.get("asset_root", "") or "")
    return set(sg._normalize_usd_list(entries or [],
                                      float(cfg.get("asset_scale", 1.0)),
                                      asset_root)[0])


def build_stats(config_path, args):
    cfg = sg.load_config(config_path)
    if args.seed is not None:
        cfg["seed"] = args.seed
    if not args.measure:
        cfg["measure_usds"] = False
        cfg.setdefault("fallback_sizes", {})["house"] = list(args.house_fallback)

    # When measuring, share one resolver across runs so each asset is opened
    # over the network only once (configs differ in disaster settings, not
    # asset scales).
    if args.measure:
        resolver = _RESOLVER_CACHE.setdefault("r", sg._make_resolver(cfg))
    else:
        resolver = sg._make_resolver(cfg)

    # The FULL pipeline, not just build_city: damage moved to the disaster
    # stage, so calling build_city alone now reports zero ruins for every
    # preset — which looked like a tuning problem and was a plumbing one.
    import generate_scene

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        placements, layout, _ = generate_scene.build_scene(cfg, resolver)
    if args.verbose:
        print(buf.getvalue(), end="")

    by = {}
    for p in placements:
        by.setdefault(p["category"], []).append(p)

    bld = cfg["usds"].get("buildings") or cfg["usds"].get("houses") or {}
    from scene_generator import building_entries
    damaged_set = _resolved_paths(cfg, building_entries(cfg, condition="damaged"))
    destroyed_set = _resolved_paths(cfg, building_entries(cfg, condition="destroyed"))
    ruin_set = damaged_set | destroyed_set

    houses = by.get("house", [])

    def down(cat):
        """Placements tipped >45° from upright. Y-up assets carry a +90°
        axis-correction roll when upright, so measure against that baseline."""
        n = 0
        for p in by.get(cat, []):
            upright = 90.0 if p.get("axis_up") == "Y" else 0.0
            if abs(p["roll_deg"] - upright) > 45:
                n += 1
        return n

    # --- spatial signature: how much of the region was hit, and whether
    # ruins actually cluster there.
    field = sg.make_damage_field(cfg.get("disaster", {}).get("field"),
                                 layout["region"])
    rx0, ry0, rx1, ry1 = layout["region"]
    n = 40
    samples = [field(rx0 + (rx1 - rx0) * (i + 0.5) / n,
                     ry0 + (ry1 - ry0) * (j + 0.5) / n)
               for i in range(n) for j in range(n)]
    hit_pct = 100.0 * sum(1 for s in samples if s >= 0.5) / len(samples)

    if field.hi - field.lo < 1e-9:
        signature = "uniform"
    else:
        def _ruin_rate(group):
            """Ruin percentage, or None when nothing was built there."""
            if not group:
                return None
            return 100.0 * sum(1 for h in group if h["usd"] in ruin_set
                               or h.get("_mesh_damage")) / len(group)

        core = _ruin_rate([h for h in houses if field(h["x_m"], h["y_m"]) >= 0.5])
        edge = _ruin_rate([h for h in houses if field(h["x_m"], h["y_m"]) < 0.5])
        # "-" distinguishes "no buildings in that band" from "0% ruined".
        signature = "{}:{}".format(
            "-" if core is None else f"{core:.0f}%",
            "-" if edge is None else f"{edge:.0f}%")

    return {
        "locale": str(cfg.get("locale", "-"))[:9],
        "assets": str(cfg.get("asset_pack", "-"))[:9],
        "buildings": len(houses),
        # A "damaged" building is one that took the damaged fate by EITHER
        # route: an asset swap (its usd is in the damaged pool) or mesh damage
        # (its usd stays intact and the disaster stage marks it). Counting only
        # the swap reported zero damaged for every preset the moment mesh
        # damage became the preferred mechanism for that fate.
        "damaged": sum(1 for h in houses
                       if h["usd"] in damaged_set or h.get("_mesh_damage")),
        "destroyed": sum(1 for h in houses if h["usd"] in destroyed_set),
        "debris": len(by.get("debris", [])) + len(by.get("debris_pile", [])),
        "prone": down("human"),
        "downed": down("streetlight") + down("traffic_light"),
        "flipped": down("car"),
        "settle": sum(1 for p in placements if p.get("settle")),
        "hit%": round(hit_pct),
        "core:edge": signature,
    }


def main():
    ap = argparse.ArgumentParser(
        description="Compare low-level scene configs via dry-run city builds.")
    ap.add_argument("configs", nargs="*",
                    help="low-level config paths or bare names (default: every "
                         f"*.yaml in {os.path.relpath(COMPILED_DIR, _SCENE_GEN_DIR)})")
    ap.add_argument("--configs", dest="configs_opt", nargs="*", default=None,
                    help="same as the positional form")
    ap.add_argument("--seed", type=int, default=None,
                    help="override each config's seed")
    ap.add_argument("--measure", action="store_true",
                    help="measure real USD bboxes (needs asset access) "
                         "instead of offline fallback footprints")
    ap.add_argument("--house-fallback", nargs=2, type=float,
                    default=[15.0, 15.0], metavar=("W", "H"),
                    help="offline house footprint in m (default 15 15)")
    ap.add_argument("--verbose", action="store_true",
                    help="show the generator's own log output")
    args = ap.parse_args()

    # omniverse:// URLs only resolve inside Isaac Sim's python — boot a
    # headless SimulationApp (registers the Nucleus resolver + cached auth)
    # before anything imports pxr.
    sim_app = None
    if args.measure:
        try:
            from isaacsim import SimulationApp
            print("[preset_report] starting headless Isaac Sim for "
                  "omniverse:// access (~1 min)…", flush=True)
            sim_app = SimulationApp(launch_config={"headless": True})
        except ImportError:
            print("[preset_report] WARNING: 'isaacsim' not importable — "
                  "omniverse:// assets can't be measured under plain "
                  "python3 and will use fallback_sizes. For real bboxes "
                  "run with Isaac Sim's python, e.g.:\n"
                  "    ~/isaacsim/python.sh utils/preset_report.py --measure",
                  flush=True)

    global sg
    import scene_generator as sg  # noqa: E402 (after SimulationApp on purpose)

    configs = (args.configs_opt or []) + list(args.configs)
    if not configs:
        configs = sorted(os.path.join(COMPILED_DIR, f)
                         for f in os.listdir(COMPILED_DIR)
                         if f.endswith((".yaml", ".yml")))
        if not configs:
            raise SystemExit(
                f"no compiled configs in {COMPILED_DIR}\n"
                "run: python3 utils/compile_disaster.py")
    else:
        resolved = []
        for c in configs:
            if os.path.isfile(c):
                resolved.append(c)
                continue
            cand = os.path.join(COMPILED_DIR, c)
            for x in (cand, cand + ".yaml", cand + ".yml"):
                if os.path.isfile(x):
                    resolved.append(x)
                    break
            else:
                raise SystemExit(f"config not found: {c}")
        configs = resolved

    cols = ("locale", "assets", "buildings", "damaged", "destroyed", "debris",
            "prone", "downed", "flipped", "hit%", "core:edge")
    name_w = max(len(os.path.splitext(os.path.basename(c))[0])
                 for c in configs) + 2
    print(f"{'config':<{name_w}}" + "".join(f"{c:>11}" for c in cols))
    for path in configs:
        stats = build_stats(path, args)
        name = os.path.splitext(os.path.basename(path))[0]
        print(f"{name:<{name_w}}" + "".join(f"{stats[c]:>11}" for c in cols))

    if not args.measure:
        print("\n(offline footprints — counts are relative, not what a "
              "measured run produces; pass --measure for real bboxes)")

    if sim_app is not None:
        sim_app.close()


if __name__ == "__main__":
    main()
