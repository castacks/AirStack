#!/usr/bin/env python3
"""
layout_dry_run.py — HOST-SIDE (no Isaac Sim, no docker, no Nucleus) dry run of
the `downtown_earthquake` layout at an arbitrary `region_m`, so the building
count / block structure / road network can be checked before spending a GPU
run on the real assembly.

WHY THIS WORKS WITHOUT KIT: `generate_scene.generate_scene_on_stage` and
everything it calls (`scene_generator.build_city`, `layout/city_layout.py`,
`detail/districts.py`, `detail/city_detail.py`, `detail/road_markings.py`)
import only `pxr` (Usd/UsdGeom/Sdf/Gf/UsdShade/UsdSkel/Vt) — no `omni`, no
`carb`, no `isaacsim`. usd-core (a plain pip wheel) is enough. The one
omni-dependent branch (`_make_physx_ground_snap`, PhysX raycast ground snap)
is only reached when `generate_scene_on_stage(..., snap_to_ground=True)`; we
call it with the default `False`.

WHY BUILDING FOOTPRINTS ARE REAL, NOT GUESSED: `asset-set: urban_quake`
points every building pool at
    omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype/bld_<style>_DG0.usd
and that Nucleus subtree is a straight mirror of the local
`scene_gen/assets/archetype/` directory (same relative path under
`.../Projects/SEI-COA/`) — the DG0 (pristine) .usd files are checked out on
this host as real USD crate files. So instead of monkeypatching the
measurement function or hand-copying archetypes.json's W/D/H, this script
just REWRITES the omniverse:// URL prefix to the local repo root in the
loaded config (in memory, before `build_city` runs) and lets the existing
`scene_generator._measure_footprint` (`Usd.Stage.Open` + `UsdGeom.BBoxCache`)
open the real local file and measure the real bbox — exactly the code path
production uses, just against a local file instead of a Nucleus one.

Every OTHER category (street furniture, trees, vehicles, park props — from
`shared.yaml` / `urban.yaml`, rooted at
`omniverse://.../Library/Stages/...`) is NOT locally mirrored and is left
alone. Confirmed harmless: `Usd.Stage.Open`/`AddReference` on an
unresolvable omniverse:// URL fails closed (a warning on stderr, no
exception — `SizeResolver.get` catches it and falls back to the config's own
`fallback_sizes` constants), which is the offline degrade path the codebase
already documents ("if a USD can't be opened ... it falls back to
per-category constants" — scene_generator.py module docstring). This is
exactly the intended "no Nucleus access" behaviour, not a hack.

We do NOT run `disaster.quake.assemble` (the pass that swaps each intact
archetype for its damaged bake by the shaking field) — that needs the
DG1..DG5 damage bakes, is orthogonal to the layout question this dry run
answers, and is not imported by anything in the `generate_scene_on_stage`
call chain, so skipping it costs nothing.

Run with:
    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        --with pyyaml python tools/layout_dry_run.py --region 250 500
"""
import argparse
import json
import os
import re
import sys

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TOOLS_DIR)
_REPO_ROOT = os.path.dirname(_SCENE_GEN_DIR)
sys.path.insert(0, _SCENE_GEN_DIR)

from pxr import Usd  # noqa: E402

# The Nucleus prefix every quake archetype URL carries, and its local mirror.
# `.../Projects/SEI-COA/scene_gen/assets/archetype/bld_x_DG0.usd` on Nucleus
# is `<repo_root>/scene_gen/assets/archetype/bld_x_DG0.usd` on disk.
_NUCLEUS_PREFIX_RE = re.compile(
    r"^omniverse://[^/]+/Projects/SEI-COA/")


def _localize_building_urls(usds_cfg: dict) -> dict:
    """Rewrite every `omniverse://.../Projects/SEI-COA/...` building URL to
    its local repo path, in place. Returns {style: local_path} for reporting.
    Leaves non-matching / non-building entries untouched (graceful fallback
    handles those, see module docstring)."""
    bld = (usds_cfg or {}).get("buildings") or {}
    rewritten = {}
    missing = []

    def _rewrite_one(entry):
        if isinstance(entry, str):
            url = entry
        elif isinstance(entry, dict):
            url = entry.get("usd", "")
        else:
            return entry
        if not _NUCLEUS_PREFIX_RE.match(url):
            return entry
        local = _NUCLEUS_PREFIX_RE.sub(_REPO_ROOT + "/", url)
        if not os.path.isfile(local):
            missing.append(local)
            return entry
        base = os.path.basename(local)
        style = re.sub(r"_DG\d+\.usd$", "", re.sub(r"^bld_", "", base))
        rewritten[style] = local
        if isinstance(entry, str):
            return local
        entry = dict(entry)
        entry["usd"] = local
        return entry

    for pool_name, pool in list(bld.items()):
        if not isinstance(pool, list):
            continue
        bld[pool_name] = [_rewrite_one(e) for e in pool]

    if missing:
        print(f"[dry_run] WARNING: {len(missing)} building URL(s) rewritten "
              f"to a local path that does not exist: {missing[:5]}")
    return rewritten


def _style_of(usd_path: str) -> str:
    base = os.path.basename(str(usd_path))
    return re.sub(r"_DG\d+\.usd$", "", re.sub(r"^bld_", "", base))


def _corridor_length_m(c: dict) -> float:
    """A corridor's length runs along its own direction: 'ns' spans y, 'ew'
    spans x (see scene_generator._subdivide_region_metric's docstring)."""
    if c.get("dir") == "ns":
        return abs(c["y1"] - c["y0"])
    return abs(c["x1"] - c["x0"])


def run_one(region_m: float, seed_note: str = "") -> dict:
    # Plain (non-reloaded) imports: build_city creates its own
    # `random.Random(config["seed"])` per call and layout.city_layout's
    # PARK_RESERVES is slice-reassigned (`PARK_RESERVES[:] = ...`), not
    # appended, on every subdivision — so nothing here carries state between
    # the two region sizes run in this one process.
    import compile_disaster
    import generate_scene
    import scene_generator as sg
    from detail import districts as districts_mod

    print(f"\n{'=' * 70}\n[dry_run] region_m = {region_m} x {region_m}\n{'=' * 70}")

    config = compile_disaster.load_scene_config(
        "downtown_earthquake",
        spec_overrides={"region_m": [float(region_m), float(region_m)]})

    style_paths = _localize_building_urls(config["usds"])
    print(f"[dry_run] localized {len(style_paths)} building style(s) to disk: "
          f"{sorted(style_paths)}")

    # generate_scene_on_stage doesn't return `layout` (blocks/road_corridors),
    # only `placements` — and `districts.remap_buildings` REPACKS each block
    # by height-matching after `build_city`'s own greedy pack, so a SEPARATE
    # standalone `build_city` call would report a different (pre-rezone)
    # house count than the real pipeline (measured: 57 vs the 50 that survive
    # rezoning, on this same config). Rather than risk that drift, capture the
    # one-and-only `layout` the real call computes by wrapping `sg.build_city`
    # for the duration of this one `generate_scene_on_stage` call.
    _captured = {}
    _orig_build_city = sg.build_city

    def _capturing_build_city(cfg, resolver):
        placements_, layout_ = _orig_build_city(cfg, resolver)
        _captured["layout"] = layout_
        _captured["resolver"] = resolver
        return placements_, layout_

    sg.build_city = _capturing_build_city
    try:
        stage = Usd.Stage.CreateInMemory()
        placements = generate_scene.generate_scene_on_stage(
            stage, config, parent_path="/World/stage/generated",
            scene_scale_factor=1.0, snap_to_ground=False)
    finally:
        sg.build_city = _orig_build_city

    layout = _captured["layout"]
    resolver = _captured["resolver"]

    houses = [p for p in placements if p.get("category") == "house"]
    by_style: dict = {}
    for p in houses:
        by_style[_style_of(p["usd"])] = by_style.get(_style_of(p["usd"]), 0) + 1

    footprints = {}
    for style, path in style_paths.items():
        fp = resolver.get(path, "house", scale=1.0, axis_up="Z")
        footprints[style] = {"W": round(fp["sx"], 2), "D": round(fp["sy"], 2),
                              "H": round(fp["sz"], 2)}

    blocks = layout.get("blocks", [])
    corridors = layout.get("road_corridors", [])
    total_road_m = sum(_corridor_length_m(c) for c in corridors)
    park_rects = districts_mod.park_blocks(layout, placements)

    xs = [p["x_m"] for p in placements] or [0.0]
    ys = [p["y_m"] for p in placements] or [0.0]
    extent = (min(xs), min(ys), max(xs), max(ys))

    area_ha = (region_m * region_m) / 10000.0
    cat_tally: dict = {}
    for p in placements:
        cat_tally[p["category"]] = cat_tally.get(p["category"], 0) + 1

    result = {
        "region_m": region_m,
        "n_buildings": len(houses),
        "by_style": dict(sorted(by_style.items())),
        "footprints_m": footprints,
        "n_blocks": len(blocks),
        "n_road_corridors": len(corridors),
        "total_road_length_m": round(total_road_m, 1),
        "n_parks": len(park_rects),
        "placements_total": len(placements),
        "category_tally": dict(sorted(cat_tally.items())),
        "extent_m": [round(v, 1) for v in extent],
        "area_ha": round(area_ha, 3),
        "buildings_per_ha": round(len(houses) / area_ha, 3) if area_ha else 0.0,
    }
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--region", nargs="+", type=float, default=[250.0, 500.0])
    ap.add_argument("--out", default=None,
                     help="write JSON results to this path")
    args = ap.parse_args()

    results = []
    for r in args.region:
        try:
            res = run_one(r)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            res = {"region_m": r, "error": f"{type(exc).__name__}: {exc}"}
        results.append(res)
        print(f"\n[dry_run] RESULT region={r}: "
              f"{json.dumps(res, indent=1)}")

    if args.out:
        with open(args.out, "w") as fh:
            json.dump(results, fh, indent=1)
        print(f"\n[dry_run] wrote {args.out}")


if __name__ == "__main__":
    main()
