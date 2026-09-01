#!/usr/bin/env python3
"""baseline_layouts.py — the committed (seed, window) table for the THREE
urban-fire BASELINE levels, host-side (no Kit): generates the full 1.5 km
`downtown_fire_1500` layout per seed, draws it with the level's 1 km crop
window overlaid, crops+repairs it (`tools/crop_window.py`), draws the cropped
result, and prints the exact pod command lines for the real dump + crop.

    python3 scene_gen/tools/baseline_layouts.py
    python3 scene_gen/tools/baseline_layouts.py --out-dir ~/fire_previews/baseline_layouts

WHY THESE THREE DRAWS (see the LEVELS table below for the exact numbers):
every window is a 1000 x 1000 m box whose CENTRE may sit anywhere in
`[-250, 250]^2` (half of 1500 minus half of 1000) and still land fully inside
the 1.5 km plate — that is the whole "not always centred" degree of freedom
the user asked for. Each level's (seed, window) pair was chosen by first
measuring where THAT seed's own park superblock(s) actually land
(`detail.districts` typology, read off `plan_png.build`'s real output, not
guessed) and then choosing a window that gives each level a DIFFERENT
relationship to it:

  level 1 (seed 4)  the park sits comfortably INSIDE an off-centre window —
                     "wherever", a normal crop that happens to keep it whole.
  level 2 (seed 2)  the window's own edge cuts through a park superblock —
                     "clipped at an edge".
  level 3 (seed 3)  the window is pushed to the far side of the plate from
                     the park — "absent-ish", only a sliver (if any) survives.

Re-run `--sweep` to re-measure any seed's park rect(s) before trusting a new
window draw — a district/park config edit moves them.
"""
import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.normpath(os.path.join(_HERE, ".."))
for _p in (_HERE, _SCENE_GEN_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import crop_window as cw                                    # noqa: E402
import plan_png                                              # noqa: E402

PRESET = "downtown_fire_1500"
WINDOW_SIZE_M = 1000.0

# ---------------------------------------------------------------------------
# THE COMMITTED TABLE. `window_centre` is (cx, cy) in the FULL 1.5 km plate's
# own frame; the window itself is `[cx - 500, cy - 500, cx + 500, cy + 500]`.
# Measured park rects this table was chosen against (park_blocks/_typology_of
# read off a real host build, `--sweep` reproduces this):
#   seed 2: (156.7,-404.7)-(464.4,-285.2) and (-464.4,-278.1)-(-161.9,-113.9)
#   seed 3: (61.1,-490.1)-(193.4,-165.3)
#   seed 4: (-435.5,242.9)-(-110.1,364.9)
# ---------------------------------------------------------------------------
LEVELS = [
    {"level": 1, "seed": 4, "window_centre": (-180.0, 180.0),
     "story": "wherever -- an off-centre window that happens to keep seed "
              "4's park (roughly x[-436,-110] y[243,365]) whole"},
    {"level": 2, "seed": 2, "window_centre": (100.0, 150.0),
     "story": "clipped at an edge -- the window's own y0=-350 line cuts "
              "through seed 2's larger park (roughly x[157,464] "
              "y[-405,-285]); its second park is clipped on x too"},
    {"level": 3, "seed": 3, "window_centre": (20.0, 230.0),
     "story": "absent-ish -- pushed to the window's own y1=730 edge, "
              "leaving only an ~85 m sliver of seed 3's park (roughly "
              "x[61,193] y[-490,-165]) inside, if any"},
]


def _window_of(level_entry):
    cx, cy = level_entry["window_centre"]
    half = WINDOW_SIZE_M / 2.0
    return (cx - half, cy - half, cx + half, cy + half)


def _park_area(typ_of):
    """Total area of every block whose typology is `"park"` -- `layout[
    "_typology_of"]` survives `crop_window.crop_layout` (clipped/re-keyed),
    so this is safe to call before AND after a crop; `districts.park_blocks`
    is not (it matches against `city_layout.PARK_RESERVES`'s stale module-
    global rects, which a crop's clipped/shifted block keys no longer equal)."""
    area = 0.0
    for (x0, y0, x1, y1), name in typ_of.items():
        if name == "park":
            area += max(0.0, x1 - x0) * max(0.0, y1 - y0)
    return area


def build_level(entry, out_dir):
    seed = entry["seed"]
    window = _window_of(entry)
    print("\n{0}\n[baseline_layouts] LEVEL {1} -- seed {2}, window {3}\n"
         "  {4}\n{0}".format("=" * 78, entry["level"], seed, window,
                             entry["story"]))

    cfg, layout, placements, res = plan_png.build(
        PRESET, seed=seed, spec_overrides={"disaster-type": "none"})

    park_before = _park_area(layout.get("_typology_of") or {})

    full_png = os.path.join(
        out_dir, "level{0}_full_seed{1}.png".format(entry["level"], seed))
    plan_png.draw(cfg, layout, placements, res, full_png,
                 title="{0} seed {1} -- FULL 1.5km, level {2} crop window "
                       "in red".format(PRESET, seed, entry["level"]),
                 crop_window=window)

    def footprint_of(p):
        fp = res.get(p.get("usd", ""), p.get("category", "house"))
        return fp["sx"], fp["sy"]

    new_layout, new_placements, rpt = cw.crop_layout(
        layout, placements, window, footprint_of=footprint_of)
    repair = cw.repair_after_crop(cfg, new_layout, new_placements, res)

    park_after = _park_area(new_layout.get("_typology_of") or {})
    park_frac = (park_after / park_before) if park_before > 0 else 0.0

    crop_png = os.path.join(
        out_dir, "level{0}_crop_seed{1}.png".format(entry["level"], seed))
    plan_png.draw(cfg, new_layout, new_placements, res, crop_png,
                 title="{0} seed {1} -- CROPPED to 1km at level {2}"
                       .format(PRESET, seed, entry["level"]))

    print("[baseline_layouts] wrote {0}".format(full_png))
    print("[baseline_layouts] wrote {0}".format(crop_png))
    n_total = rpt["buildings_kept"] + rpt["buildings_dropped"]
    n_repair_dropped = repair["facing"]["dropped_unrepairable_blank"]
    n_final = rpt["buildings_kept"] - n_repair_dropped
    print("[baseline_layouts] buildings: {0} total -> {1} kept by the "
         "window, {2} more dropped by repair_after_crop's blank-wall pass "
         "-> {3} final ({4} dropped by the window)".format(
             n_total, rpt["buildings_kept"], n_repair_dropped, n_final,
             rpt["buildings_dropped"]))
    n_clipped = 0
    for b in layout.get("blocks", []):
        clipped = cw._clip_rect(tuple(b), window)
        if clipped is None:
            continue
        orig_area = (b[2] - b[0]) * (b[3] - b[1])
        clip_area = (clipped[2] - clipped[0]) * (clipped[3] - clipped[1])
        if clip_area < orig_area - 1.0:      # 1 m^2 slack for float noise
            n_clipped += 1
    print("[baseline_layouts] blocks: {0} kept ({1} of those clipped at the "
         "window edge, area shrunk), {2} dropped".format(
             rpt["blocks_kept"], n_clipped, rpt["blocks_dropped"]))
    print("[baseline_layouts] props: {0} kept, {1} dropped "
         "({2} orphaned by a dropped building)".format(
             rpt["props_kept"], rpt["props_dropped"],
             rpt["props_orphan_dropped"]))
    print("[baseline_layouts] park area: {0:.0f} m^2 before crop -> "
         "{1:.0f} m^2 after ({2:.0%} retained)".format(
             park_before, park_after, park_frac))
    print("[baseline_layouts] facing repair: {0}".format(repair["facing"]))
    print("[baseline_layouts] overlap repair: {0}".format(repair["overlaps"]))

    return {
        "level": entry["level"], "seed": seed, "window": list(window),
        "story": entry["story"], "buildings_kept": rpt["buildings_kept"],
        "buildings_dropped": rpt["buildings_dropped"],
        "blocks_kept": rpt["blocks_kept"], "blocks_dropped": rpt["blocks_dropped"],
        "park_area_before_m2": round(park_before, 1),
        "park_area_after_m2": round(park_after, 1),
        "park_area_frac_retained": round(park_frac, 3),
        "repair": repair, "full_png": full_png, "crop_png": crop_png,
    }


#: `urban_fire_city_launch_script.py` has NO per-run seed-override env var
#: for the city-layout seed (unlike the host-side dry-run tools'
#: `spec_overrides={"seed": N}`) -- so each level that needs a seed other
#: than the base preset's own seed 4 gets its own thin-copy preset file,
#: `downtown_fire_1500.yaml`'s own header explains why. Level 1 uses the
#: base preset directly.
_SCENE_CONFIG_BY_LEVEL = {1: "downtown_fire_1500",
                         2: "downtown_fire_1500_lvl2",
                         3: "downtown_fire_1500_lvl3"}


def pod_commands(entry):
    """The exact env/command lines for the POD: the real Kit dump at 1.5 km,
    then the crop step. `SG_INSTANCE_PLACEMENTS=1` is the unconditional env
    override the `generate-urban-city` skill documents for whichever
    config-loading path a launcher uses; the preset also sets
    `instance_placements: true` itself, so this is redundant-but-safe, the
    same belt-and-braces the 500 m preset's own docs recommend."""
    level, seed = entry["level"], entry["seed"]
    cx, cy = entry["window_centre"]
    scene_config = _SCENE_CONFIG_BY_LEVEL[level]
    dump = ("scene_gen/_plans/city_placements_{0}.json".format(scene_config))
    cropped = ("scene_gen/_plans/city_placements_{0}_crop.json"
              .format(scene_config))
    return [
        "# --- level {0}: {1} (seed {2}), window centre ({3}, {4}) ---"
        .format(level, scene_config, seed, cx, cy),
        "# 1) the real Kit dump, full 1.5 km plate (pod, isaac-sim container):",
        "SCENE_CONFIG={0} \\".format(scene_config),
        "SG_INSTANCE_PLACEMENTS=1 \\",
        "FC_INTACT_ONLY=1 \\",
        "FC_DUMP={0} \\".format(dump),
        "ISAAC_SIM_SCRIPT_NAME=urban_fire_city_launch_script.py \\",
        "airstack up isaac-sim",
        "",
        "# 2) crop the dump to this level's 1 km window (host or pod, no Kit):",
        "python3 scene_gen/tools/fc_dump_crop.py \\",
        "    --in  {0} \\".format(dump),
        "    --centre {0} {1} --size 1000 1000 \\".format(cx, cy),
        "    --out {0}".format(cropped),
        "",
        "# 3) downstream tools then run on the CROPPED dump exactly as on a",
        "#    native 1 km one, e.g.:",
        "python3 scene_gen/tools/fire_city_dry_run.py "
        "--placements-json {0}".format(cropped),
        "",
        "# 4) ASSEMBLY (the freeze/fire-damage wave), once a manifest is",
        "#    solved on the cropped dump above: FC_CROP_WINDOW is the SAME",
        "#    cx,cy,W,H this level's window used, comma-separated, no spaces.",
        "#    Deactivates everything outside the window, matches the manifest",
        "#    back to Kit's own (never-translated) full-city stage via",
        "#    x_orig/y_orig, and frames captures on the window instead of the",
        "#    full 1.5 km plate -- see urban_fire_city_launch_script.py's own",
        "#    FC_CROP_WINDOW docstring. Unset/empty on level 1's own env means",
        "#    'no crop' -- always pass this explicitly for a cropped level.",
        "FC_CROP_WINDOW={0},{1},1000,1000".format(cx, cy),
    ]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default=os.path.expanduser(
        "~/fire_previews/baseline_layouts"))
    ap.add_argument("--sweep", nargs="*", type=int, default=None,
                    help="instead of building the committed table, just "
                         "print each given seed's park typology rects (no "
                         "PNGs) -- how the LEVELS table above was chosen")
    a = ap.parse_args()

    if a.sweep is not None:
        seeds = a.sweep or list(range(1, 9))
        for s in seeds:
            _cfg, layout, _p, _res = plan_png.build(
                PRESET, seed=s, spec_overrides={"disaster-type": "none"})
            parks = [rect for rect, name in
                    (layout.get("_typology_of") or {}).items() if name == "park"]
            print("seed {0}: {1} park block(s): {2}".format(
                s, len(parks),
                ["({0:.1f},{1:.1f})-({2:.1f},{3:.1f})".format(*r) for r in parks]))
        return

    os.makedirs(a.out_dir, exist_ok=True)
    results = []
    for entry in LEVELS:
        results.append(build_level(entry, a.out_dir))

    print("\n{0}\n[baseline_layouts] POD COMMAND LINES\n{0}".format("=" * 78))
    for entry in LEVELS:
        for line in pod_commands(entry):
            print(line)
        print()

    report_path = os.path.join(a.out_dir, "baseline_layouts_report.json")
    with open(report_path, "w") as fh:
        json.dump(results, fh, indent=1)
    print("[baseline_layouts] wrote {0}".format(report_path))


if __name__ == "__main__":
    main()
