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


# ---------------------------------------------------------------------------
# ROUND 5, WP A — the offline SAME_ART decision tally.
#
# `disaster.quake.decide_building` is PURE (no pxr, no stage — see its own
# docstring) precisely so this file can call it directly: the exact
# grade -> route -> twin-or-keep decision `disaster.quake.assemble` makes on
# a real stage, reproduced here with no Isaac Sim at all.
#
# WHY THE THREE MEASURED TRIPLES ARE REPEATED HERE, not read off the asset:
# a SAME_ART original (`Muyang/ModernCityEnvironment/...`) is joined against
# `asset_root` (`omniverse://.../Library/Stages/`), which is NOT the
# `.../Projects/SEI-COA/` subtree `_localize_building_urls` mirrors locally
# (see its own docstring) — these three assets are simply not on this host,
# same as every other non-archetype building in the library. There is no way
# to measure them offline, so the three numbers below are the same measured
# triples `kit_substitute.check()` asserts `route()` against by name
# (`kit_substitute.py`'s module docstring / its three named assertions) —
# not a second opinion, just the one fact repeated where geometry can't
# stand in for it.
_SAME_ART_DIMS = {
    "SM_MERGED_BP_MBuilding01.usd": (28.5, 18.5, 29.0),
    "SM_MERGED_BP_MBuilding02.usd": (91.1, 96.1, 68.7),
    "SM_MERGED_BP_MBuilding05.usd": (27.7, 20.6, 62.4),
}

# ---------------------------------------------------------------------------
# `urban_quake_v4` — GreatAmericanCity (GAC) per-building bakes.
#
# UNLIKE THE THREE SAME_ART TRIPLES ABOVE, GAC's real W/D/H does not need to
# be hand-copied: `_plans/gac_buildings.json` (`tools/gac_measure.py`'s own
# probe output) already ships with the repo and covers all 31 buildings, so
# this reads it once instead of maintaining a second hand-typed table.
_GAC_DIMS_CACHE = None


def _gac_dims():
    """{name: (W, D, H)} from the local `_plans/gac_buildings.json` probe.
    Cached at module scope — this dry run calls it once per region."""
    global _GAC_DIMS_CACHE
    if _GAC_DIMS_CACHE is None:
        path = os.path.join(_SCENE_GEN_DIR, "_plans", "gac_buildings.json")
        with open(path) as fh:
            recs = json.load(fh)
        _GAC_DIMS_CACHE = {r["name"]: (r["W"], r["D"], r["H"]) for r in recs}
    return _GAC_DIMS_CACHE


def synthetic_gac_manifest(names, grades=("DG1", "DG2", "DG3", "DG4", "DG5")):
    """A synthetic `{(name, grade): record}` GAC manifest covering `grades`
    for every name in `names` — a stand-in for the real `gac_quake.json` the
    parallel bake pipeline has not exported yet, so this dry run can show
    what WOULD be swapped once it has (`_plans/eq_v4_city_dry_run.md`'s
    "with the manifest" tally) alongside what falls back TODAY with no
    manifest at all (its "without the manifest" tally). Never used by
    `disaster.quake.assemble` itself — offline-report use only."""
    out = {}
    for name in names:
        for grade in grades:
            out[(name, grade)] = {
                "name": name, "grade": grade, "style": name, "level": grade,
                "usd": "/synthetic/gac_quake/gac_{0}_{1}_s0.usd".format(name, grade),
            }
    return out


def _gac_names_placed(placements: list) -> list:
    """Distinct GreatAmericanCity building names actually placed in this
    run's `placements` (category == 'house' only, sorted) — however many
    turn out to appear is exactly what `synthetic_gac_manifest` needs to
    cover for the "with the manifest" decision tally."""
    from disaster import kit_substitute as ks
    from disaster import quake as q

    names = set()
    for p in placements:
        if p.get("category") != "house":
            continue
        usd = p.get("usd", "")
        if ks.pack_of(usd) == "other" and q._is_gac(usd):
            names.add(q.gac_name_of(usd))
    return sorted(names)


def decision_tally(config: dict, placements: list, arch_dir: str,
                   gac_manifest: dict = None) -> dict:
    """Run `disaster.quake.decide_building` / `decide_gac_building` for every
    house placement, exactly as `disaster.quake.assemble` would (same field,
    same grade draw ORDER, same manifest, same fallback) — offline, no
    stage. `gac_manifest` (round 5, `urban_quake_v4`) is `None` or `{}` to
    report what happens with NO GAC bakes at all (today's reality — every
    GAC building kept, reasons recorded) or a real/`synthetic_gac_manifest`
    dict to report what the SAME placements would do once bakes exist —
    `assemble` itself always merges whatever `load_gac_manifest` returns, so
    calling this twice with `{}` and then a manifest reproduces both of
    `assemble`'s possible outcomes for one city.

    Returns `{"twins": {style: {grade: count}}, "kit_grades": {style:
    {grade: count}}, "gac_twins": {name: {grade: count}}, "kept":
    {"same_art": {asset: count}, "kit": count, "gac": count},
    "skip_reasons": {reason: count}, "gac_skip_reasons": {reason: count},
    "missing_dims": [...], "gac_missing_dims": [...]}` — the report
    `assemble`'s own per-building decisions would produce.
    """
    import random as _random

    import scene_generator as sg
    from disaster import kit_substitute as ks
    from disaster import quake as q
    from disaster import quake_flow as qf

    gac_manifest = gac_manifest or {}
    dis = config.get("disaster") or {}
    w, h = config["layout"]["region_m"]
    region = (-float(w) / 2.0, -float(h) / 2.0, float(w) / 2.0, float(h) / 2.0)
    field = sg.make_damage_field(dis.get("field") or {"kind": "uniform", "inside": 0.0},
                                 region)
    grade_scale = float(dis.get("grade_scale", 1.0))
    dur_boost = float(dis.get("duration_boost", 1.0))
    manifest = q.load_manifest(arch_dir)
    gac_dims = _gac_dims()
    # SAME rng construction `assemble`'s default (`seed=11`) uses for its own
    # grade-drawing rng, so this offline tally draws the identical sequence
    # `assemble` would for the same config/placements. GAC buildings now draw
    # from this SAME shared sequence too (round 5, `urban_quake_v4`'s
    # `quake.assemble` gac branch) — one more reason this must iterate
    # `placements` in the same order `assemble` does (it does: both walk the
    # same list, filtering the same way).
    rng = _random.Random(11 + 4242)

    # `twins` is SAME_ART -> kit-twin swaps ONLY (asset -> style -> grade ->
    # count) — the interesting number this whole work package is about.
    # `kit_grades` is a native kit archetype's OWN grade draw (style -> grade
    # -> count) — not a substitution at all (`route()` just names the same
    # style back), kept separate so it cannot be mistaken for a twin count.
    # `gac_twins` is GAC name -> its OWN per-building bake's grade -> count —
    # never a best-fit substitution (see `decide_gac_building`'s docstring:
    # there is no `route()` call for GAC at all).
    twins: dict = {}
    kit_grades: dict = {}
    gac_twins: dict = {}
    kept = {"same_art": {}, "kit": 0, "gac": 0}
    skip_reasons: dict = {}
    gac_skip_reasons: dict = {}
    missing_dims = []
    gac_missing_dims = []

    for p in placements:
        if p.get("category") != "house":
            continue
        usd = p.get("usd", "")
        pack = ks.pack_of(usd)
        gac = False
        if pack == "same_art":
            base = os.path.basename(str(usd))
            dims = _SAME_ART_DIMS.get(base)
            if not dims:
                missing_dims.append(base)
                continue
            W, D, H = dims
            btype = q._same_art_material(usd, config)
        elif pack == "kit":
            base = None
            style0, _lvl = q.style_of(usd)
            rec0 = manifest.get((style0, "DG0")) or {}
            if not rec0:
                continue
            W = rec0.get("W", 20.0)
            D = rec0.get("D", 20.0)
            H = rec0.get("H", 12.0)
            btype = rec0.get("type") or qf.FAMILY_TYPE.get(rec0.get("family", ""), "urm")
        elif pack == "other" and q._is_gac(usd):
            gac = True
            name = q.gac_name_of(usd)
            dims = gac_dims.get(name)
            if not dims:
                gac_missing_dims.append(name)
                continue
            W, D, H = dims
            btype = q._same_art_material(usd, config)
        else:
            continue

        x, y = float(p["x_m"]), float(p["y_m"])
        inten = float(field(x, y))
        grade = qf.level_for_intensity(inten * grade_scale, btype, rng,
                                       duration_boost=dur_boost)
        if gac:
            decision = q.decide_gac_building(usd, grade, gac_manifest, rng,
                                             x=x, y=y, yaw_deg=p.get("yaw_deg", 0.0))
        else:
            decision = q.decide_building(usd, grade, W, D, H, btype, manifest, rng,
                                         x=x, y=y, yaw_deg=p.get("yaw_deg", 0.0))
        if decision["action"] == "twin":
            g = decision["grade"]
            if gac:
                bucket = gac_twins.setdefault(name, {})
            elif pack == "same_art":
                bucket = twins.setdefault(base, {}).setdefault(decision["style"], {})
            else:
                bucket = kit_grades.setdefault(decision["style"], {})
            bucket[g] = bucket.get(g, 0) + 1
        else:
            if gac:
                kept["gac"] += 1
            elif pack == "same_art":
                kept["same_art"][base] = kept["same_art"].get(base, 0) + 1
            else:
                kept["kit"] += 1
            reason = decision.get("reason")
            if reason:
                if gac:
                    gac_skip_reasons[reason] = gac_skip_reasons.get(reason, 0) + 1
                else:
                    skip_reasons[reason] = skip_reasons.get(reason, 0) + 1

    return {"twins": {a: {s: dict(sorted(g.items())) for s, g in sorted(styles.items())}
                     for a, styles in sorted(twins.items())},
           "kit_grades": {k: dict(sorted(v.items())) for k, v in sorted(kit_grades.items())},
           "gac_twins": {n: dict(sorted(g.items())) for n, g in sorted(gac_twins.items())},
           "kept": kept, "skip_reasons": skip_reasons,
           "gac_skip_reasons": gac_skip_reasons,
           "missing_dims": sorted(set(missing_dims)),
           "gac_missing_dims": sorted(set(gac_missing_dims))}


def _corridor_length_m(c: dict) -> float:
    """A corridor's length runs along its own direction: 'ns' spans y, 'ew'
    spans x (see scene_generator._subdivide_region_metric's docstring)."""
    if c.get("dir") == "ns":
        return abs(c["y1"] - c["y0"])
    return abs(c["x1"] - c["x0"])


def run_one(region_m: float, seed_note: str = "", asset_set: str = None) -> dict:
    # Plain (non-reloaded) imports: build_city creates its own
    # `random.Random(config["seed"])` per call and layout.city_layout's
    # PARK_RESERVES is slice-reassigned (`PARK_RESERVES[:] = ...`), not
    # appended, on every subdivision — so nothing here carries state between
    # the two region sizes run in this one process.
    import compile_disaster
    import generate_scene
    import scene_generator as sg
    from detail import districts as districts_mod

    print(f"\n{'=' * 70}\n[dry_run] region_m = {region_m} x {region_m}"
          f"{f', asset_set={asset_set}' if asset_set else ''}\n{'=' * 70}")

    overrides = {"region_m": [float(region_m), float(region_m)]}
    if asset_set:
        overrides["asset-set"] = asset_set          # e.g. urban_quake_v3
    config = compile_disaster.load_scene_config(
        "downtown_earthquake", spec_overrides=overrides)

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
    if asset_set:
        result["asset_set"] = asset_set
    # ROUND 5, WP A: run `disaster.quake.decide_building`'s SAME_ART / kit
    # decision offline (no stage) against the REAL local archetype library —
    # `by_style` above already answers "how many of each pool member", this
    # answers "what would `assemble` actually DO with them".
    arch_dir = os.path.join(_SCENE_GEN_DIR, "assets", "archetype")
    if os.path.isfile(os.path.join(arch_dir, "archetypes.json")):
        try:
            # NO gac_manifest — this is the "today" tally: real GAC bakes do
            # not exist on this host yet, so every placed GAC building comes
            # back `kept` with a reason (round 5, `urban_quake_v4`).
            result["decision_tally"] = decision_tally(config, placements, arch_dir)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            result["decision_tally"] = {"error": f"{type(exc).__name__}: {exc}"}
        # ROUND 5, `urban_quake_v4`: the SAME placements against a SYNTHETIC
        # GAC manifest covering every GAC name this run actually placed, at
        # DG1-DG5 — "what WILL happen once the real bakes exist" alongside
        # the "what falls back today" tally just above. Only meaningful for
        # an asset set that actually puts GAC buildings in a pool
        # (`urban_quake_v4`); a v3 (or earlier) run places none, so
        # `gac_names` is empty and this is skipped entirely.
        gac_names = _gac_names_placed(placements)
        if gac_names:
            result["gac_names_placed"] = gac_names
            try:
                synth = synthetic_gac_manifest(gac_names)
                result["decision_tally_with_gac_bakes"] = decision_tally(
                    config, placements, arch_dir, gac_manifest=synth)
            except Exception as exc:
                import traceback
                traceback.print_exc()
                result["decision_tally_with_gac_bakes"] = {
                    "error": f"{type(exc).__name__}: {exc}"}
    return result


def _format_markdown(results: list) -> str:
    """A short markdown report for `--md`: pool mix + the offline decision
    tally (twins per style per grade, kept counts, refusal reasons) per
    region run — the numbers `_plans/eq_v3_city_dry_run.md` records."""
    lines = ["# Earthquake layout dry run\n"]
    for res in results:
        if res.get("error"):
            lines.append(f"## region {res['region_m']} m — ERROR\n\n"
                         f"    {res['error']}\n")
            continue
        aset = res.get("asset_set", "(preset default)")
        lines.append(f"## region {res['region_m']} m x {res['region_m']} m "
                     f"(asset_set: {aset})\n")
        lines.append(f"* {res['n_buildings']} buildings, "
                     f"{res['buildings_per_ha']} per ha, "
                     f"{res['n_blocks']} blocks, {res['n_parks']} parks, "
                     f"{res['total_road_length_m']} m of road\n")
        lines.append("### Pool mix (`by_style`)\n")
        lines.append("| style / asset | count |")
        lines.append("|---|---|")
        for style, n in res.get("by_style", {}).items():
            lines.append(f"| {style} | {n} |")
        lines.append("")

        dt = res.get("decision_tally")
        if not dt:
            continue
        if dt.get("error"):
            lines.append(f"### Decision tally — ERROR\n\n    {dt['error']}\n")
            continue
        lines.append("### Decision tally (`disaster.quake.decide_building`, offline)\n")
        lines.append("**SAME_ART twins** (original asset -> kit style -> grade -> count):\n")
        if dt.get("twins"):
            lines.append("| original asset | kit twin style | grade | count |")
            lines.append("|---|---|---|---|")
            for asset, styles in dt["twins"].items():
                for style, grades in styles.items():
                    for grade, n in grades.items():
                        lines.append(f"| {asset} | {style} | {grade} | {n} |")
        else:
            lines.append("(none)")
        lines.append("")
        if dt.get("kit_grades"):
            lines.append("**Native kit archetype grades** (style's own damage "
                         "ladder, not a substitution — style -> grade -> "
                         "count):\n")
            lines.append("| style | grade | count |")
            lines.append("|---|---|---|")
            for style, grades in dt["kit_grades"].items():
                for grade, n in grades.items():
                    lines.append(f"| {style} | {grade} | {n} |")
            lines.append("")
        kept = dt.get("kept", {})
        lines.append(f"**Kept** — same_art originals: "
                     f"{kept.get('same_art', {})}; already-kit at DG0 (or with "
                     f"no bake at any grade): {kept.get('kit', 0)}; GAC "
                     f"originals (no bake at any grade — see the GAC "
                     f"section below): {kept.get('gac', 0)}\n")
        if dt.get("skip_reasons"):
            lines.append("**Skip reasons** (once per distinct message, with count):\n")
            for reason, n in dt["skip_reasons"].items():
                lines.append(f"* ({n}x) {reason}")
            lines.append("")
        if dt.get("missing_dims"):
            lines.append(f"**Missing dims** (same_art asset with no entry in "
                         f"`_SAME_ART_DIMS`): {dt['missing_dims']}\n")

        # ROUND 5, `urban_quake_v4` — GAC. Two tallies against the IDENTICAL
        # placements/field/rng sequence: WITHOUT any GAC bake manifest (what
        # happens on this host today — every GAC building kept, one reason
        # per distinct name), and WITH a synthetic manifest covering every
        # GAC name this run actually placed at DG1-DG5 (what WILL happen
        # once the real bakes land at `assets/gac_quake/`).
        gac_names = res.get("gac_names_placed")
        if gac_names:
            lines.append(f"### GAC per-building bakes ({len(gac_names)} distinct "
                         f"name(s) placed: {', '.join(gac_names)})\n")
            lines.append("**Without a GAC manifest (today)** — every placed GAC "
                         "building above DG0 is `kept`, one reason per distinct "
                         "name:\n")
            if dt.get("gac_skip_reasons"):
                for reason, n in dt["gac_skip_reasons"].items():
                    lines.append(f"* ({n}x) {reason}")
            else:
                lines.append("(none reached above DG0 in this run)")
            lines.append(f"\nTotal kept (no bake): {kept.get('gac', 0)}\n")

            dtg = res.get("decision_tally_with_gac_bakes")
            if dtg and dtg.get("error"):
                lines.append(f"**With a synthetic manifest — ERROR**\n\n"
                             f"    {dtg['error']}\n")
            elif dtg:
                lines.append("**With a synthetic manifest covering DG1-DG5 for "
                             "every name above** — what WILL be swapped once "
                             "the real bakes exist (name -> grade -> count):\n")
                shopping: dict = {}
                if dtg.get("gac_twins"):
                    lines.append("| GAC building | grade | count |")
                    lines.append("|---|---|---|")
                    for name, grades in dtg["gac_twins"].items():
                        for grade, n in grades.items():
                            lines.append(f"| {name} | {grade} | {n} |")
                            shopping[(name, grade)] = n
                else:
                    lines.append("(none)")
                lines.append(f"\nStill kept even with the manifest (e.g. DG0, "
                             f"or `_mono_dims` failed to measure the placed "
                             f"prim): {dtg.get('kept', {}).get('gac', 0)}\n")
                if dtg.get("gac_missing_dims"):
                    lines.append(f"**Missing dims** (GAC name with no entry in "
                                 f"`_plans/gac_buildings.json`): "
                                 f"{dtg['gac_missing_dims']}\n")

                # THE GPU BAKE SHOPPING LIST: every (name, grade) this scene
                # actually needs, sorted by how often it would be used —
                # bake the top of this list first.
                if shopping:
                    lines.append("### GPU bake shopping list "
                                 "(`(name, grade)` -> count this scene needs, "
                                 "sorted by count)\n")
                    lines.append("| name | grade | count |")
                    lines.append("|---|---|---|")
                    for (name, grade), n in sorted(
                            shopping.items(), key=lambda kv: (-kv[1], kv[0])):
                        lines.append(f"| {name} | {grade} | {n} |")
                    lines.append("")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--region", nargs="+", type=float, default=[250.0, 500.0])
    ap.add_argument("--asset-set", default=None,
                     help="override the preset's asset-set, e.g. urban_quake_v3")
    ap.add_argument("--out", default=None,
                     help="write JSON results to this path")
    ap.add_argument("--md", default=None,
                     help="write a markdown summary (pool mix + decision "
                          "tally) to this path")
    args = ap.parse_args()

    results = []
    for r in args.region:
        try:
            res = run_one(r, asset_set=args.asset_set)
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

    if args.md:
        with open(args.md, "w") as fh:
            fh.write(_format_markdown(results))
        print(f"\n[dry_run] wrote {args.md}")


if __name__ == "__main__":
    main()
