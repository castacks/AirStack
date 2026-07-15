#!/usr/bin/env python3
"""
preset_report.py — dry-run the city generator across disaster presets and
print a comparison table (buildings, ruins, debris, casualties, topples).

Runs the pure-Python layout only — nothing is written to a stage — so it's a
fast way to sanity-check preset tuning before launching Isaac Sim.

USAGE
-----
    python3 utils/preset_report.py                  # all presets + baseline
    python3 utils/preset_report.py --presets severe.yaml total.yaml
    python3 utils/preset_report.py --seed 7

By default assets are NOT measured (no Nucleus access needed): footprints
come from fallback_sizes, with the house footprint overridden to
--house-fallback (default 15x15 m) so blocks actually pack — absolute counts
therefore differ from a real run, but the relative differences between
presets are what matter here.

--measure uses real USD bboxes. omniverse:// URLs only resolve with Isaac
Sim's bundled python (it registers the Nucleus resolver + cached auth), so
run it as:

    ~/isaacsim/python.sh utils/preset_report.py --measure          # local
    /isaac-sim/python.sh utils/preset_report.py --measure          # container

The script starts a headless SimulationApp automatically in that case
(~1 min startup); under plain python3, --measure warns and omniverse://
assets fall back to fallback_sizes.
"""

import argparse
import contextlib
import copy
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

sg = None            # scene_generator; imported in main() after the optional
                     # SimulationApp bootstrap (pxr must load after it)
_RESOLVER_CACHE = {}  # one shared measuring resolver across preset runs


def _resolved_paths(cfg, entries):
    """Config usds entries -> resolved asset paths (asset_root applied)."""
    asset_root = str(cfg.get("asset_root", "") or "")
    return set(sg._normalize_usd_list(entries or [],
                                      float(cfg.get("asset_scale", 1.0)),
                                      asset_root)[0])


def build_stats(raw_cfg, config_path, preset_file, args):
    cfg = copy.deepcopy(raw_cfg)
    cfg.pop("preset_file", None)
    if preset_file:
        path = sg._resolve_preset_path(
            config_path, str(cfg.get("presets_path", "") or ""), preset_file)
        if not path:
            raise SystemExit(f"preset not found: {preset_file}")
        import yaml
        with open(path) as f:
            sg._deep_merge(cfg, yaml.safe_load(f))
    if args.seed is not None:
        cfg["seed"] = args.seed
    if not args.measure:
        cfg["measure_usds"] = False
        cfg.setdefault("fallback_sizes", {})["house"] = list(args.house_fallback)

    # When measuring, share one resolver across preset runs so each asset is
    # opened/measured over the network only once (presets only change
    # disaster settings, not asset scales).
    if args.measure:
        resolver = _RESOLVER_CACHE.setdefault("r", sg._make_resolver(cfg))
    else:
        resolver = sg._make_resolver(cfg)

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        placements, layout = sg.build_city(cfg, resolver)
    if args.verbose:
        print(buf.getvalue(), end="")

    by = {}
    for p in placements:
        by.setdefault(p["category"], []).append(p)

    bld = cfg["usds"].get("buildings") or cfg["usds"].get("houses") or {}
    damaged_set = _resolved_paths(cfg, bld.get("damaged"))
    destroyed_set = _resolved_paths(cfg, bld.get("destroyed"))

    houses = by.get("house", [])
    trail_pts = [(p["x_m"], p["y_m"]) for p in by.get("trail", [])]
    parks = sum(1 for b in layout["blocks"]
                if any(b[0] <= x <= b[2] and b[1] <= y <= b[3]
                       for x, y in trail_pts))

    def down(cat):
        """Placements tipped >45° from upright. Y-up assets carry a +90°
        axis-correction roll when upright, so measure against that baseline."""
        n = 0
        for p in by.get(cat, []):
            upright = 90.0 if p.get("axis_up") == "Y" else 0.0
            if abs(p["roll_deg"] - upright) > 45:
                n += 1
        return n

    return {
        "buildings": len(houses),
        "damaged": sum(1 for h in houses if h["usd"] in damaged_set),
        "destroyed": sum(1 for h in houses if h["usd"] in destroyed_set),
        "debris": len(by.get("debris", [])) + len(by.get("debris_pile", [])),
        "prone_humans": down("human"),
        "downed_lights": down("streetlight") + down("traffic_light"),
        "flipped_cars": down("car"),
        "parks": parks,
        "settle_bodies": sum(1 for p in placements if p.get("settle")),
        "total": len(placements),
    }


def main():
    default_config = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "config", "scene_generator_config.yaml"))

    ap = argparse.ArgumentParser(
        description="Compare disaster presets via dry-run city builds.")
    ap.add_argument("--config", default=default_config)
    ap.add_argument("--presets", nargs="*", default=None,
                    help="preset filenames (default: every .yaml in "
                         "presets_path, plus the no-preset baseline)")
    ap.add_argument("--seed", type=int, default=None,
                    help="override the config seed")
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

    import yaml
    with open(args.config) as f:
        raw_cfg = yaml.safe_load(f)

    presets = args.presets
    if presets is None:
        ppath = str(raw_cfg.get("presets_path", "") or "")
        config_dir = os.path.dirname(os.path.abspath(args.config))
        pdir = next((d for d in (ppath,
                                 os.path.join(config_dir, ppath),
                                 os.path.join(config_dir,
                                              os.path.basename(ppath)))
                     if d and os.path.isdir(d)), "")
        presets = (sorted(f for f in os.listdir(pdir) if f.endswith(".yaml"))
                   if pdir else [])
        presets.append(None)   # baseline: config's own disaster values

    cols = ("buildings", "damaged", "destroyed", "debris", "prone_humans",
            "downed_lights", "flipped_cars", "parks", "settle_bodies", "total")
    name_w = max(len(str(p or "(no preset)")) for p in presets) + 2
    print(f"{'preset':<{name_w}}" + "".join(f"{c:>14}" for c in cols))
    for preset in presets:
        stats = build_stats(raw_cfg, args.config, preset, args)
        name = preset or "(no preset)"
        print(f"{name:<{name_w}}"
              + "".join(f"{stats[c]:>14}" for c in cols))

    if not args.measure:
        print("\n(offline footprints — counts are relative, not what a "
              "measured run produces; pass --measure for real bboxes)")

    if sim_app is not None:
        sim_app.close()


if __name__ == "__main__":
    main()
