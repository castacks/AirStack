#!/usr/bin/env python3
"""
fire_city_dry_run.py — HOST-SIDE (no Isaac Sim, no docker, no Nucleus) dry run
of a CITY-SCALE urban fire: the REAL `downtown_gac`-family layout (typologies,
blocks, real building footprints) + the REAL `disaster.urban_fire_spread`
solve, producing `scene_gen/_plans/fire_city_<seed>.json` and a markdown
report — the offline proof called for by
`scene_gen/_plans/urban_fire_city_plan.md` §5 (work items #4/#5 of its §6).

Modelled directly on `tools/layout_dry_run.py` — same "localize the
omniverse:// building URLs to the local repo mirror, then let the real
`scene_generator`/`generate_scene` code measure real footprints" trick, same
"monkeypatch `scene_generator.build_city` for one call to capture `layout`"
trick (`generate_scene.generate_scene_on_stage` never returns or stashes
`layout` itself — see that file's own docstring for why a second, separate
`build_city` call is not equivalent). Read that file first if this one is
confusing; this file assumes it.

WHY `disaster-type: fire` DOES NOT GO THROUGH `compile_disaster.DISASTERS`
---------------------------------------------------------------------------
`compile_disaster.DISASTERS` has no `"fire"` entry — only `"wildfire"`, a
different model entirely (a continuous-fuel-bed ellipse over vegetation, not
this module's discrete building graph). Adding a `"fire"` entry is out of
scope for this file (`urban_fire_city_plan.md`'s work-breakdown table names
only this tool, the preset, its test and `_plans/` for work items #4/#5 — the
eventual city launcher, item #7, does not touch `compile_disaster.py` either).

So `downtown_fire_500.yaml` carries a literal `disaster-type: fire` line for
readability, plus five plain top-level keys in EXACTLY the shape
`compile_wildfire` (`compile_disaster.py:842-905`) already uses for the same
five physical quantities — `epicenter`, `heading_deg`, `wind_mps`,
`duration_s`, `start_offset_frac` — but this file reads them off the RAW YAML
directly (`yaml.safe_load`, before any compilation) rather than through
`compile_spec`, and feeds them straight to `disaster.urban_fire_spread`.
Getting the CITY LAYOUT (typologies, blocks, real placements — what
`disaster.urban_fire_city.burnable` gates against) goes through the ordinary
`compile_disaster.load_scene_config` path with `disaster-type` OVERRIDDEN to
`"none"` FOR COMPILATION ONLY (`spec_overrides={"disaster-type": "none"}` —
the same override mechanism `layout_dry_run.py` already uses for
`region_m`/`asset-set`). That override is legitimate, not a workaround:
`scene_generator.build_city` / `generate_scene.generate_scene_on_stage` never
read `config["disaster"]` for placement — only a later damage-application
pass would, and this file does not run one (same precedent
`layout_dry_run.py`'s own docstring cites for skipping `disaster.quake.
assemble`: orthogonal to the layout question, not imported by the call chain
this file exercises).

OFFLINE MEASUREMENT FOR THE TWO WHOLE-ASSET PACKS THE LOCAL MIRROR DOES NOT
HAVE
---------------------------------------------------------------------------
`layout_dry_run.py`'s local mirror covers `scene_gen/assets/archetype/`
(the kit archetypes) only — `.../Projects/SEI-COA/` is the ONE Nucleus
subtree checked out locally, and neither GreatAmericanCity (`.../Projects/
SEI-COA/GreatAmericanCity/...`, which DOES match that prefix but is not
mirrored under this path on disk) nor the AEC brownstones / Dmytro / Muyang
packs (which live under `.../Library/Stages/...`, a different Nucleus
subtree entirely) resolve locally. `SizeResolver.get` fails closed for all of
these (a stderr warning, no exception) and falls back to generic per-category
constants — WRONG for a GAC/downtowncity building specifically, because this
file's own burnability gate needs each building's REAL height (the `gac_fire.
prepare` construction-type rule is `"urm" if H <= 25.0 else "rc"`, height-
sensitive) and the spread solve needs its real footprint (`gap_m` is edge-to-
edge, not a category average).

Fortunately both packs were already measured once and the numbers checked
into the repo: `_plans/gac_buildings.json` (31 GreatAmericanCity assets — the
exact W/D/H this file's own `config/asset_sets/urban_gac.yaml` comments
quote) and `_plans/dtc_buildings.json` (15 downtowncity assets, plus a
`storeys.n_storeys` field GAC's cache lacks). `_gac_dtc_cache()` loads both,
keyed by basename, and `_measure_wdh` consults it BEFORE falling through to
`resolver.get` for anything else (every asset that DOES resolve locally —
kit archetypes — and everything else, which is harmless to mismeasure here
because it is refused at the bake-kind gate regardless of its footprint: see
the module docstring of `disaster.urban_fire_city` on why AEC brownstones /
Dmytro / non-DTC Muyang packs never carry a `fire_bake.KINDS` entry no matter
how big or small `SizeResolver`'s fallback constant says they are).

WHY EXCLUDING `rc_glass` (AND, WHEN NEEDED, `rc`) FROM THE ORIGIN DRAW MATTERS
-------------------------------------------------------------------------
`urban_fire_spread.level_for_age` has a DETERMINISTIC branch to F5/F6 for
`btype == "urm"` once `age >= T_COLD`, but for `rc` (and always for
`rc_glass`) the only way past F4 is the STOCHASTIC `collapse_p` draw to F5c —
miss it, and an `rc`/`rc_glass` building is F4 forever regardless of how much
time has elapsed. `rc_glass` additionally can NEVER be F5c
(`level_for_age`'s own docstring). Since §5 check 3 requires the ORIGIN
building's own level to be F5/F5c/F6, this file:
  (a) never draws an `rc_glass` building as the origin (`pick_origin`'s own
      `blocked=` parameter, reused here purely as an origin-eligibility
      filter — it does NOT remove those buildings from the spread GRAPH
      passed to `solve()`, only from origin CANDIDACY);
  (b) after solving, if the origin's natural level is still below F5, forces
      it to F5c (`_enforce_target_f5c`) — exactly the fallback plan section 4
      and the work order specify ("force the origin ... to F5c"), and then
      demotes any OTHER F5c to F5 so the collapse count stays at the
      requested target (default 1).

For `downtown_fire_500` at its accepted seed (202) this fallback never fires:
the origin (a native `apartment` kit archetype, `urm`) reaches F6 on the
DETERMINISTIC branch. It is exercised by the synthetic tests and left in
place as a safety net for a different seed/epicenter.

HEIGHT-CLASS AWARE SINCE 2026-08-31 -- `_enforce_target_f5c` never forces a
`"skyscraper"`-class origin (`urban_fire_spread.HEIGHT_CLASS_SKYSCRAPER`,
`tower`/`highrise`) to F5c: that class's own rank cap forbids it (fire only,
never any collapse), so it forces `"F5"` there instead, which still clears
§5 check 3's `rank >= RANK["F5"]` bar. See `disaster.urban_fire_spread`'s
"HEIGHT CLASS" docstring section and `urban_fire_city.damaged_manifest`
(the AUTHORITATIVE gate this file's own `plan`-level forcing only tries to
pre-empt, not replace) for the full policy, including the separate
roof-outcome eligibility/share-budget rule `check_district_rule` also now
checks.

STOREY COUNT IS AN ESTIMATE, NOT A MEASUREMENT
------------------------------------------------
`entry_for_plan_fire` needs a storey count to turn `origin_frac` (a fraction
of height) into a storey INDEX. The real bake measures this off the sliced
mesh (`gac_fire.prepare`'s `mass_from_grid`); that needs a stage. This file
estimates `n_storeys = round(H / period_m)`, using `downtowncity`'s own
per-asset `storeys.period_m` when the cache has one (`dtc`), a generic 3.0 m
GAC storey height (matching the measured DTC median) for `gac`, and a
generic 3.5 m for kit archetypes. It only has to be non-degenerate (>= 1) for
§5 check 4's "storey in range" to be meaningful — refine when the real bake
runs.

Run with the SAME invocation `layout_dry_run.py` documents:

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        --with pyyaml python tools/fire_city_dry_run.py \\
        --preset downtown_fire_500 --n 16

THE 2026-08-30 MANIFEST/CITY MISMATCH, AND `--placements-json`
------------------------------------------------------------------
The invocation above builds the layout HOST-SIDE, with the patched
`SizeResolver` described above (real GAC/DTC footprints from the checked-in
cache, everything else measured locally or falling back). The CITY
LAUNCHER (`urban_fire_city_launch_script.py`) builds the SAME preset/seed
IN KIT, with a real `SizeResolver` that has live Nucleus access — real
footprints for every pack, not just GAC/DTC. Those two builds can pack
differently (a bigger real footprint than the local mirror measures can
tip two buildings from "clear" to "touching" in the packer's own spacing
decision), so a manifest solved host-side can name cells that the actual
Kit-built city never placed a matching building at. First observed
2026-08-30: the launcher refused 15 of 16 bakes with "no placement matches
i=..., cell=..., or a same-asset house within 2 m of (...)" — the one that
matched was luck, not agreement.

The fix is to never let the two builds diverge: run `FC_INTACT_ONLY=1
FC_DUMP=<path>` on the city launcher first (builds the city IN KIT, writes
every house placement's cell/usd/position/measured-W-D-H and the typology
block map to `<path>`, and stops — no manifest needed for it), then run
this tool with `--placements-json <path>` instead of `--preset`/relying on
the host-side layout build. `--placements-json` mode skips `build_layout`
entirely — no Nucleus, no packer, and no `pxr` EITHER unless the dump
carries a `same_art` MCE placement (see `load_placements_dump`'s own note
on why that one case still needs it) — and runs the SAME burnable-set /
spread / manifest steps directly on the dump's own layout and placements —
see `load_placements_dump`, `run_dry_from_dump`. The resulting manifest's
records reference the dump's `cell` paths verbatim, and carries the dump's
own path + sha256 under `"placements_dump"` so a later reader can tell
which dump it was solved on. The seven §5 checks are unchanged; they read
the manifest, not how it was built.
"""
import argparse
import json
import math
import os
import random
import re
import sys

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TOOLS_DIR)
_REPO_ROOT = os.path.dirname(_SCENE_GEN_DIR)
if _SCENE_GEN_DIR not in sys.path:
    sys.path.insert(0, _SCENE_GEN_DIR)

# `pxr` is needed for the real layout build (in-memory stage) but NOT for the
# seven checks themselves, which are pure python over a JSON-shaped manifest
# — importing it lazily inside `build_layout` keeps `python3 -c "from tools
# import fire_city_dry_run"`-style unit testing free of a `usd-core` install.
_NUCLEUS_PREFIX_RE = re.compile(r"^omniverse://[^/]+/Projects/SEI-COA/")


# ---------------------------------------------------------------------------
# The localize-URL trick — verbatim from `layout_dry_run.py` (kept local
# rather than imported so this file has no import-time dependency on that
# script, which is a `__main__`-style tool, not a library).
# ---------------------------------------------------------------------------
def _localize_building_urls(usds_cfg: dict) -> dict:
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
        if isinstance(pool, list):
            bld[pool_name] = [_rewrite_one(e) for e in pool]
    if missing:
        print(f"[fire_dry_run] {len(missing)} building URL(s) rewritten to a "
              f"local path that does not exist: {missing[:5]}")
    return rewritten


def _basename_noext(usd) -> str:
    base = os.path.basename(str(usd))
    for ext in (".usd", ".usdc", ".usda"):
        if base.lower().endswith(ext):
            return base[: -len(ext)]
    return base


# ---------------------------------------------------------------------------
# GAC / downtowncity offline measurement cache — see module docstring.
# ---------------------------------------------------------------------------
def _gac_dtc_cache():
    """`{basename: raw_record}` merged from `_plans/gac_buildings.json` and
    `_plans/dtc_buildings.json`. Raw records (not just W/D/H) so `dtc`'s
    `storeys` field survives for the storey-count estimate."""
    out = {}
    for fn in ("gac_buildings.json", "dtc_buildings.json"):
        path = os.path.join(_SCENE_GEN_DIR, "_plans", fn)
        if not os.path.isfile(path):
            continue
        for rec in json.load(open(path)):
            out[rec["name"]] = rec
    return out


def _measure_wdh(usd, placement, resolver, cache, gf):
    """`(W, D, H)` for one placement's asset — the cache first for GAC/DTC
    (see module docstring), `resolver.get` (the same `SizeResolver` the real
    layout used) for everything else."""
    u = str(usd)
    if u.startswith(gf.GAC_DIR) or u.startswith(gf.DTC_DIR):
        rec = cache.get(_basename_noext(u))
        if rec:
            return float(rec["W"]), float(rec["D"]), float(rec["H"])
    fp = resolver.get(u, "house", scale=placement.get("scale"),
                      axis_up=placement.get("axis_up", "Z"))
    return fp["sx"], fp["sy"], fp["sz"]


def _final_btype(kind, name_or_style, H, manifest_dg0, cache):
    """The construction type FED TO THE SPREAD SOLVE — mirrors the rule the
    real fire bake actually uses per kind (see module docstring), all of it
    pure arithmetic / table lookup, no stage:

      gac  `gac_fire.prepare`'s own rule when `construction_table` is False:
           "urm" if H <= 25.0 else "rc".
      dtc  `quake_sliced.construction_type` — the SAME per-asset table +
           height fallback `gac_fire.prepare` consults when
           `construction_table` is True.
      kit  the style's own `type` field in `assets/archetype/archetypes.json`
           (DG0 record) — already `urm`/`rc`/`rc_glass`, no guessing needed.
    """
    if kind == "gac":
        return "urm" if H <= 25.0 else "rc"
    if kind == "dtc":
        from disaster import quake_sliced as qs
        return qs.construction_type(name_or_style, H=H)
    if kind == "kit":
        rec = manifest_dg0.get(name_or_style) or {}
        t = rec.get("type")
        if t:
            return t
        from disaster import quake_flow as qf
        return qf.FAMILY_TYPE.get(rec.get("family", ""), "urm")
    return "urm"


def _estimate_storeys(kind, name_or_style, H, cache):
    """See module docstring — an ESTIMATE, not a measurement."""
    if kind == "dtc":
        rec = cache.get(name_or_style) or {}
        st = (rec.get("storeys") or {}).get("n_storeys")
        if st:
            return max(1, int(st))
    period = 3.0 if kind == "gac" else 3.5
    return max(1, int(round(H / period)))


def _patch_resolver_for_gac_dtc(resolver):
    """Patches *resolver*'s bound `.get` (in place, for the life of this
    process — there is exactly one resolver per dry run) so the REAL packing
    pass sizes GreatAmericanCity / downtowncity buildings by their TRUE
    measured footprint instead of `fallback_sizes.house` ([30, 20, 24] m,
    `config/asset_sets/urban.yaml`).

    WHY THIS MATTERS, MEASURED: without it, `build_city`'s own packer/
    spacing decisions are made as if EVERY GAC/DTC building were a uniform
    30 x 20 m box (`SizeResolver.get`'s fallback branch — these two packs
    are not locally mirrored; see the module docstring). Several real GAC
    buildings are much bigger in one or both axes (SM_Building_30 is
    28.4 x 42.4 m, SM_Building_08 is 86.2 x 58.4 m) — packed as if 30 x 20,
    two of them can legitimately end up close enough that substituting their
    REAL footprint back in for the burnability/spread analysis (this file's
    own `_measure_wdh`) makes them overlap, even though the packer itself
    respected its own (wrong) spacing rule. Confirmed on `downtown_fire_500`
    seed 202: buildings 248/249/249/250/251/252/252/253 (all GAC, all in the
    `brick_midrise` terrace) overlap by 3.6-15.4 m along the best separating
    axis with the fallback-sized layout; patching the resolver before
    `build_city` runs removes every one of them (`tools/fire_city_dry_run.py`
    footprint check, `_plans/fire_city_202_report.md`).

    Idempotent (checks a marker attribute) so calling this twice on the same
    resolver — e.g. a caller that builds the layout more than once — is a
    no-op the second time."""
    if getattr(resolver, "_fire_dry_run_patched", False):
        return
    from disaster import gac_fire as gf

    cache = _gac_dtc_cache()
    orig_get = resolver.get

    def _patched_get(usd_path, category, scale=None, axis_up="Z"):
        u = str(usd_path)
        if u.startswith(gf.GAC_DIR) or u.startswith(gf.DTC_DIR):
            rec = cache.get(_basename_noext(u))
            if rec:
                return {"sx": float(rec["W"]), "sy": float(rec["D"]),
                        "sz": float(rec["H"]), "base": -float(rec.get("z0", 0.0)),
                        "cx": float(rec.get("cx", 0.0)),
                        "cy": float(rec.get("cy", 0.0)), "cz": 0.0}
        return orig_get(usd_path, category, scale=scale, axis_up=axis_up)

    resolver.get = _patched_get
    resolver._fire_dry_run_patched = True


# ---------------------------------------------------------------------------
# Stage 1: the real layout (host-side, `pxr` needed from here down)
# ---------------------------------------------------------------------------
def build_layout(preset: str, seed=None):
    """Compile *preset* (with `disaster-type` forced to `"none"` for
    compilation — see module docstring), run the REAL `generate_scene.
    generate_scene_on_stage` pipeline in memory, and return `(config,
    layout, placements, resolver)`. `seed`, if given, overrides the
    preset's own `seed:` (the whole run — layout AND fire — is reproduced
    from this one number)."""
    from pxr import Usd
    import compile_disaster
    import generate_scene
    import scene_generator as sg

    overrides = {"disaster-type": "none"}
    if seed is not None:
        overrides["seed"] = int(seed)
    config = compile_disaster.load_scene_config(preset, spec_overrides=overrides)
    _localize_building_urls(config["usds"])

    captured = {}
    orig_build_city = sg.build_city

    def _capturing_build_city(cfg, resolver):
        _patch_resolver_for_gac_dtc(resolver)
        placements_, layout_ = orig_build_city(cfg, resolver)
        captured["layout"] = layout_
        captured["resolver"] = resolver
        return placements_, layout_

    sg.build_city = _capturing_build_city
    try:
        stage = Usd.Stage.CreateInMemory()
        placements = generate_scene.generate_scene_on_stage(
            stage, config, parent_path="/World/stage/generated",
            scene_scale_factor=1.0, snap_to_ground=False)
    finally:
        sg.build_city = orig_build_city

    return config, captured["layout"], placements, captured["resolver"]


def _raw_fire_spec(preset: str) -> dict:
    """The five fire keys off the RAW preset YAML — see module docstring on
    why these are never compiled through `compile_disaster.DISASTERS`."""
    import compile_disaster
    import yaml

    path = compile_disaster.resolve_config_path(preset)
    with open(path) as fh:
        spec = yaml.safe_load(fh)
    return {
        "epicenter": [float(v) for v in spec["epicenter"]],
        "heading_deg": float(spec["heading_deg"]),
        "wind_mps": float(spec["wind_mps"]),
        "duration_s": float(spec["duration_s"]),
        "start_offset_frac": float(spec["start_offset_frac"]),
    }


# ---------------------------------------------------------------------------
# Stage 1b: the ALTERNATIVE to `build_layout` -- load a placements dump the
# CITY LAUNCHER wrote (`FC_DUMP`, `FC_INTACT_ONLY=1`) instead of
# reconstructing a layout host-side. See the module docstring's "2026-08-30
# manifest/city mismatch" note: the host-side reconstruction substitutes
# GAC/DTC footprints because Nucleus is not mirrored locally, so its packer
# can make different placement decisions than Kit's real one and describe a
# city that does not exist. The layout itself is never rebuilt here (no
# `pxr`, no Nucleus, no packer) — the ONE thing that still needs `pxr` is
# `compile_disaster.load_scene_config` (it lazily imports `scene_generator`
# for `resolve_asset_set`/`validate_config`, and THAT module's own top-level
# import is `from pxr import ...`), and only `disaster.quake.
# _same_art_material` ever reads the config it returns (for a `same_art`
# MCE merged asset's tagged construction type) — so it is compiled LAZILY,
# ONLY when the dump actually carries a `same_art` placement. A GAC/DTC/kit
# archetype-only manifest (the common case for `downtown_fire_500`) never
# pays for it and needs no `pxr` install at all.
# ---------------------------------------------------------------------------
def load_placements_dump(path):
    """`(config, layout, placements, seed, preset, sha256)` from a city
    placements dump written by `urban_fire_city_launch_script.
    dump_city_placements`.

    `placements` is reconstructed at the dump's own `n_placements_total`
    length (padded with non-house placeholder dicts at every index the dump
    did not carry a house at) so each house record keeps its ORIGINAL index
    into the FULL city placement list — `urban_fire_city_launch_script.
    resolve_cell`'s most-trusted route (match the record's `i`, verified by
    usd + distance) depends on that index still being right when the
    FULL launcher rebuilds the same (deterministic) city later. A manifest
    whose `i`s were silently renumbered relative to a house-only list would
    point that route at the wrong placement almost every time.

    Raises `ValueError`, with the reason, on a schema this function does not
    recognise or a placement missing a required field — never silently
    treats a malformed dump as an empty one.
    """
    import hashlib

    with open(path, "rb") as fh:
        raw = fh.read()
    sha256 = hashlib.sha256(raw).hexdigest()
    doc = json.loads(raw.decode("utf-8"))

    if doc.get("schema") != "fire_city_placements_dump.v1":
        raise ValueError(
            "{0!r} does not carry schema=='fire_city_placements_dump.v1' "
            "(got {1!r}) -- wrong file, or written by an older/newer "
            "FC_DUMP".format(path, doc.get("schema")))
    preset = doc.get("preset")
    seed = doc.get("seed")
    if not preset or seed is None:
        raise ValueError(
            "{0!r} is missing 'preset' or 'seed' at the top level -- top-"
            "level keys: {1}".format(path, sorted(doc.keys())))

    raw_placements = doc.get("placements") or []
    required = ("i", "cell", "usd", "x_m", "y_m", "z_m", "yaw_deg",
               "W", "D", "H")
    max_i = -1
    for k, p in enumerate(raw_placements):
        missing = [f for f in required if p.get(f) is None]
        if missing:
            raise ValueError(
                "{0!r} placement #{1} is missing required field(s) {2}: "
                "{3}".format(path, k, missing, p))
        max_i = max(max_i, int(p["i"]))
    n_total = int(doc.get("n_placements_total") or 0)
    length = max(n_total, max_i + 1)

    placements = [{"category": "_dump_non_house_placeholder"}
                  for _ in range(length)]
    for p in raw_placements:
        i = int(p["i"])
        if placements[i].get("category") != "_dump_non_house_placeholder":
            raise ValueError(
                "{0!r} has two placements claiming index {1}".format(
                    path, i))
        rec = {
            "category": p.get("category", "house"), "usd": p["usd"],
            "x_m": float(p["x_m"]), "y_m": float(p["y_m"]),
            "z_m": float(p["z_m"]), "yaw_deg": float(p["yaw_deg"]),
            "scale": float(p.get("scale", 1.0)),
            "axis_up": p.get("axis_up", "Z"),
            "prim_path": p["cell"], "cell": p["cell"],
            "W": float(p["W"]), "D": float(p["D"]), "H": float(p["H"]),
        }
        # `tools/fc_dump_crop.py`'s own additive fields -- the PRE-crop,
        # full-city position, when this dump was cropped. Absent on an
        # ordinary (uncropped) dump, so `rec` is unchanged there.
        # `urban_fire_city.burnable()` reads these (falling back to x_m/y_m
        # when absent) and carries them onto every manifest record as
        # `x_orig`/`y_orig`, which `urban_fire_city_launch_script.
        # resolve_cell` prefers when matching against a live, un-translated
        # Kit stage — see that launcher's `FC_CROP_WINDOW` docstring.
        if "x_m_orig" in p:
            rec["x_m_orig"] = float(p["x_m_orig"])
        if "y_m_orig" in p:
            rec["y_m_orig"] = float(p["y_m_orig"])
        placements[i] = rec

    typ_of = {}
    for block in ((doc.get("typology") or {}).get("blocks") or []):
        rect = block.get("rect")
        name = block.get("name")
        if not rect or len(rect) != 4 or name is None:
            raise ValueError(
                "{0!r} has a malformed typology block entry: {1}".format(
                    path, block))
        typ_of[tuple(float(v) for v in rect)] = name
    layout = {"_typology_of": typ_of}

    # LAZY, and usually skipped entirely -- see the block comment above this
    # function. `pack_of` is pure python (no `pxr`); `load_scene_config`
    # is not, so it is only paid for when a `same_art` placement is
    # actually present.
    from disaster import kit_substitute as ks

    if any(ks.pack_of(p["usd"]) == "same_art" for p in raw_placements):
        import compile_disaster
        config = compile_disaster.load_scene_config(
            preset, spec_overrides={"disaster-type": "none"})
    else:
        config = {}

    return config, layout, placements, int(seed), preset, sha256


# ---------------------------------------------------------------------------
# Stage 2: gate every house placement through `urban_fire_city.burnable`
# ---------------------------------------------------------------------------
def gather_burnable(config, layout, placements, resolver):
    """`(burnable_list, refused_gate, building_typology_tally)`.
    `burnable_list` is `[(global_i, record), ...]`; `refused_gate` is
    `[{"i", "usd", "reason"}, ...]` for every house placement `urban_fire_
    city.burnable` refused. Mutates `placements[i]["btype"]` in place for
    `same_art` candidates (the ONE case `kit_substitute.route`'s soft
    `prefer_type` scoring actually reads) — the same convention `tools/
    layout_dry_run.py`'s `decision_tally` uses via `disaster.quake.
    _same_art_material`.

    `resolver=None` is the `--placements-json` path: every placement was
    already measured, by a real (Kit-side, live-Nucleus) `SizeResolver`, at
    dump time (`urban_fire_city_launch_script.dump_city_placements`), and
    those numbers are what `placement["W"]/["D"]/["H"]` carry — re-measuring
    here (this process has no Nucleus access on the host) would only put
    the host-side GAC/DTC cache substitutes back in, which is the exact bug
    this mode exists to avoid. See `_measure_wdh`."""
    from disaster import gac_fire as gf
    from disaster import kit_substitute as ks
    from disaster import quake as q
    from disaster import urban_fire_city as ufc

    cache = _gac_dtc_cache()
    burnable_list = []
    refused_gate = []
    building_typology = {}
    for i, p in enumerate(placements):
        if p.get("category") != "house":
            continue
        usd = p.get("usd")
        typ = ufc.typology_at(layout, float(p.get("x_m", 0.0)),
                              float(p.get("y_m", 0.0)))
        building_typology[i] = typ
        pk = ks.pack_of(usd)
        if pk == "same_art":
            p["btype"] = q._same_art_material(usd, config)
        if resolver is not None:
            W, D, H = _measure_wdh(usd, p, resolver, cache, gf)
        else:
            W, D, H = float(p["W"]), float(p["D"]), float(p["H"])
        ok, result = ufc.burnable(layout, p, {usd: (W, D, H)})
        if not ok:
            refused_gate.append({"i": i, "usd": usd, "reason": result})
            continue
        burnable_list.append((i, result))
    return burnable_list, refused_gate, building_typology


def build_solve_inputs(burnable_list, manifest_dg0, cache):
    """`(buildings, local_to_global, final_btype_by_local, n_storeys_by_local,
    height_class_by_local)` — `buildings` is the list `urban_fire_spread.
    solve` wants, `i` in each entry its OWN position in that list (a
    SEPARATE index space from `placements`'); `local_to_global[k]` maps
    back. `height_class_by_local` is `urban_fire_spread.height_class`
    applied to each building's own `typology` (from `burnable()`'s record,
    the SAME field `urban_fire_city.damaged_manifest` re-derives it from
    downstream) — see `disaster.urban_fire_spread`'s "HEIGHT CLASS" section
    and this module's own docstring's 2026-08-31 policy note."""
    from disaster import urban_fire_spread as ufs

    buildings = []
    local_to_global = []
    final_btype = {}
    n_storeys = {}
    height_class = {}
    for i, rec in burnable_list:
        kind = rec["kind"]
        name_or_style = rec["asset"] if kind in ("gac", "dtc") else rec["style"]
        bt = _final_btype(kind, name_or_style, rec["H"], manifest_dg0, cache)
        k = len(buildings)
        buildings.append({"x": rec["x"], "y": rec["y"], "W": rec["W"],
                          "D": rec["D"], "yaw": rec["yaw_deg"], "H": rec["H"],
                          "style": name_or_style, "i": k})
        local_to_global.append(i)
        final_btype[k] = bt
        n_st = _estimate_storeys(kind, name_or_style, rec["H"], cache)
        n_storeys[k] = n_st
        height_class[k] = ufs.height_class(typology=rec.get("typology"),
                                           n_storeys=n_st)
    return buildings, local_to_global, final_btype, n_storeys, height_class


# ---------------------------------------------------------------------------
# Stage 3: the spread solve + the exactly-N-F5c enforcement
# ---------------------------------------------------------------------------
def run_spread(buildings, local_to_global, final_btype, height_class,
              fire_spec, seed, n, extra_blocked_local=frozenset()):
    """Runs `pick_origin` + `solve`, capped to `n`. Returns `(plan,
    origin_local, notes)` — `plan` is `urban_fire_spread.solve`'s own return
    (local index space), `notes` a list of human-readable strings describing
    anything this function had to force (empty when nothing did).

    `height_class` (local index -> `urban_fire_spread.HEIGHT_CLASSES` member)
    is threaded straight through to `solve()`'s own `height_class_of` so the
    RANK CAP is already applied to `plan`'s own levels — the same policy
    `urban_fire_city.damaged_manifest` re-applies (from `typology`, not this
    dict) as the authoritative gate on the final manifest. Applying it here
    too means `_enforce_target_f5c` below (which reads and writes `plan`
    directly) sees an already-honest starting point.

    `extra_blocked_local` (LOCAL index space, i.e. `buildings`' own `"i"` —
    see `_solve_from_layout`'s translation from a caller's GLOBAL placement
    indices) is a CALLER-SUPPLIED firebreak set, unioned with the `rc_glass`
    origin-eligibility block below and handed to BOTH `pick_origin` (so an
    excluded building can never be drawn as the origin either) and `solve`'s
    own `blocked=` (so it is never in the graph at all — the same "not on
    the graph" discipline `urban_fire_city.burnable`'s pack-blacklist/height-
    cap gates already use, see that module's docstring). It exists for
    `tools/fire_city_union.py`'s auto-clean loop: a placements dump can carry
    a genuine geometry defect (two buildings placed close enough to actually
    overlap, not just touch — `check_footprint`'s own `OVERLAP_TOL_M`) that
    no amount of fire-model tuning can paper over, and the honest fix is to
    never light one member of the offending pair, exactly like any other
    firebreak. Default `frozenset()` reproduces the old behaviour (every
    burnable candidate is in the graph) exactly."""
    from disaster import urban_fire_spread as ufs

    rng = random.Random(int(seed))
    if not buildings:
        # ZERO burnable candidates reached the solve at all -- almost always
        # a district-map problem upstream (e.g. a placements dump whose
        # `typology.blocks` came back empty, so `typology_at()` refuses
        # every placement at gate 2 with "outside every zoned block"; see
        # `gather_burnable`'s own `refused_gate`), never a spread-solve
        # concern. Raise with THAT diagnosis, not the misleading rc_glass
        # message below -- an empty `rc_glass_local` trivially satisfies
        # `len(rc_glass_local) >= len(buildings)` (0 >= 0) and would
        # otherwise blame the wrong thing entirely.
        raise ValueError(
            "run_spread: zero burnable candidates were handed to the solve "
            "-- check gather_burnable's refused_gate (a common cause: the "
            "placements dump's typology.blocks is empty, so every "
            "placement is refused at burnable()'s gate 2 before construction "
            "type is even considered)")
    rc_glass_local = frozenset(k for k, bt in final_btype.items() if bt == "rc_glass")
    if len(rc_glass_local) >= len(buildings):
        # every candidate is rc_glass -- pick_origin would raise; let it,
        # with a clearer message than "no unblocked building to ignite".
        raise ValueError("run_spread: every burnable candidate is rc_glass -- "
                          "no origin can ever reach F5/F5c/F6")
    blocked_local = rc_glass_local | frozenset(extra_blocked_local or ())
    if len(blocked_local) >= len(buildings):
        raise ValueError(
            "run_spread: extra_blocked_local plus the rc_glass block covers "
            "every burnable candidate -- no origin can ever be picked "
            "(rc_glass alone: {0} of {1}; combined: {2})".format(
                len(rc_glass_local), len(buildings), len(blocked_local)))
    origin_local = ufs.pick_origin(buildings, blocked=blocked_local, rng=rng,
                                   epicenter=fire_spec["epicenter"])
    elapsed_s = fire_spec["duration_s"] * fire_spec["start_offset_frac"]
    wind_dir = math.radians(fire_spec["heading_deg"])
    plan = ufs.solve(buildings, origin_local, elapsed_s, wind_dir=wind_dir,
                     wind_mps=fire_spec["wind_mps"], rng=rng,
                     btype_of=lambda b: final_btype[b["i"]], max_burnt=n,
                     blocked=blocked_local,
                     height_class_of=lambda b: height_class[b["i"]])
    return plan, origin_local, elapsed_s


def _enforce_target_f5c(plan, origin_local, final_btype, height_class, target=1):
    """Exactly `target` SURVIVING F5c in `plan` (mutates it in place),
    preferring the ORIGIN as the one kept/forced — see module docstring.
    Returns a list of note strings (empty when the natural solve already
    satisfied the target).

    HEIGHT-CLASS AWARE (2026-08-31 policy), TWO BUGS DEEP:

      1. Only ever FORCES `"F5c"` onto a `"low"` (roof-eligible) building.
         `"mid_high"`/`"skyscraper"` candidates force to `"F5"` instead —
         not just `"skyscraper"`, whose own rank cap forbids F5c outright,
         but `"mid_high"` too, because `urban_fire_city.damaged_manifest`'s
         SEPARATE roof-eligibility gate would immediately degrade a
         `mid_high` F5c back to F5 anyway (F5c/F6 are eligible only for the
         `low` class, regardless of what the rank cap alone allows).
      2. (found 2026-08-31, chasing "the union has zero F5c even though a
         target=1 run reported nothing wrong") `solve()`'s own rank cap
         ALSO lets a `mid_high` building's OWN age/rng draw land on `"F5c"`
         NATURALLY (mid_high's cap only forbids F6, not F5c) -- and the OLD
         version of this function counted THAT toward the target and could
         end up "keeping" a non-low F5c as the one survivor, only for
         `damaged_manifest` to demote it right back to F5 downstream. The
         reported target looked satisfied here and was actually zero in the
         final manifest. Fixed by demoting every NON-LOW natural F5c to F5
         UP FRONT, before any counting happens, so every step after this
         only ever sees F5c entries that will actually still be F5c once
         `damaged_manifest` has had its say.
    """
    from disaster import urban_fire_spread as ufs

    notes = []

    def _lit(p):
        return p.get("t_ignite") is not None

    # --- 0) a NATURAL, non-low F5c can never survive the roof-eligibility
    # gate downstream -- demote it here so it is never mistaken for a
    # target-satisfying survivor.
    for p in plan:
        if _lit(p) and p["level"] == "F5c" \
                and height_class.get(p["i"]) != ufs.HEIGHT_CLASS_LOW:
            cls = height_class.get(p["i"])
            p["level"] = "F5"
            notes.append(f"local {p['i']} ({cls}) naturally drew F5c but is "
                         f"not low-class -- demoted to F5 up front (roof "
                         f"eligibility would undo it downstream anyway)")

    # --- 1) the origin must reach at least rank F5 ---
    origin_p = plan[origin_local]
    origin_cls = height_class.get(origin_local, ufs.HEIGHT_CLASS_MIDHIGH)
    if ufs.RANK.get(origin_p["level"], -1) < ufs.RANK["F5"]:
        was = origin_p["level"]
        forced = "F5c" if origin_cls == ufs.HEIGHT_CLASS_LOW else "F5"
        origin_p["level"] = forced
        notes.append(f"origin (local {origin_local}, {origin_cls}) forced "
                     f"from {was} to {forced} (natural level never reached "
                     f"F5+)")

    def _f5c_low():
        # every remaining "F5c" is already low-class, thanks to step 0 --
        # this filter is belt-and-braces, not load-bearing any more.
        return [p for p in plan if _lit(p) and p["level"] == "F5c"
                and height_class.get(p["i"]) == ufs.HEIGHT_CLASS_LOW]

    f5c = _f5c_low()
    if len(f5c) < target:
        # LOW class only -- forcing anything else would just be undone by
        # `damaged_manifest`'s roof-eligibility gate (see step 0's note).
        lit = sorted((p for p in plan if _lit(p)
                     and p["i"] != origin_local and p["level"] != "F5c"
                     and height_class.get(p["i"]) == ufs.HEIGHT_CLASS_LOW),
                    key=lambda p: p["t_ignite"])
        for p in lit:
            if len(_f5c_low()) >= target:
                break
            bt = final_btype.get(p["i"])
            if bt in ("urm", "rc") and ufs.RANK.get(p["level"], 0) >= ufs.RANK["F4"]:
                p["level"] = "F5c"
                notes.append(f"local {p['i']} ({bt}) forced to F5c to reach "
                             f"the target of {target}")
        if len(_f5c_low()) < target:
            notes.append(f"could not reach the F5c target of {target}: no "
                         f"further low-class urm/rc candidate reached F4+")
    elif len(f5c) > target:
        keep = [p for p in f5c if p["i"] == origin_local][:1] or \
               sorted(f5c, key=lambda p: p["t_ignite"])[:1]
        keep_i = keep[0]["i"]
        kept = 0
        for p in sorted(f5c, key=lambda p: p["i"] != keep_i):
            if kept < target:
                kept += 1
                continue
            p["level"] = "F5"
            notes.append(f"local {p['i']} demoted from F5c to F5 (over the "
                         f"target of {target})")
    return notes


def _block_rect_at(layout, x, y):
    """The `(x0, y0, x1, y1)` block rect containing `(x, y)`, or `None` --
    the same iteration `urban_fire_city.typology_at` does over `layout[
    "_typology_of"]`, just returning the RECT instead of the name. This is
    the `block_rect` `disaster.urban_fire_spread.street_side_score` wants:
    the layout already carries block rects (`districts.rezone_blocks`'s own
    keys), and a real road corridor is exactly a block's own complement in
    the region (`disaster.fire_people.derive_layout`'s `rect_complement`) --
    so "how far to the edge of THIS block" is "how far to the street", with
    no separate road-polygon lookup needed."""
    for rect in (layout or {}).get("_typology_of") or {}:
        x0, y0, x1, y1 = rect
        if x0 <= x <= x1 and y0 <= y <= y1:
            return rect
    return None


# ---------------------------------------------------------------------------
# Stage 4: the manifest (urban_fire_city.damaged_manifest + dry-run-only
# enrichment for the seven checks)
# ---------------------------------------------------------------------------
def build_manifest(preset, seed, n, layout, placements, buildings,
                   local_to_global, final_btype, n_storeys, plan, origin_local,
                   elapsed_s, refused_gate, dump_provenance=None,
                   roof_collapse_max=None):
    """`dump_provenance`, when given (the `--placements-json` path), is
    `{"path", "sha256"}` for the placements dump this manifest was solved
    on — written into the manifest under `"placements_dump"` so a later
    reader (a human, or a future launcher-side check) can tell which dump
    produced it, and confirm the dump has not changed since.

    `roof_collapse_max`, if given, overrides `urban_fire_city.
    damaged_manifest`'s own `ROOF_COLLAPSE_MAX_DEFAULT` for the roof-outcome
    share budget (see `disaster.urban_fire_spread`'s "HEIGHT CLASS" section
    and `damaged_manifest`'s own docstring) — `None` here means "use that
    default", not "no budget at all" (that is `roof_collapse_max=None`
    passed EXPLICITLY to `damaged_manifest` itself, a different thing this
    function has no CLI knob for). The manifest's own `"roof_outcome_count"`
    reports how many records actually ended up showing a `ROOF_LEVELS`
    outcome (`F5c`/`F6`), for the report and for a caller that wants to
    confirm the budget held without re-deriving it.

    STREET-FACING SIDE PREFERENCE (2026-08-31 user policy) is wired in
    HERE, at the one call site that has everything it takes: `buildings`
    (every burnable candidate's own footprint, the neighbour set `disaster.
    urban_fire_spread.street_side_score` scores against) and `layout` (for
    `_block_rect_at`, the block `b` was zoned into — a real road corridor is
    exactly that block's own complement in the region, see `disaster.
    fire_people.derive_layout`). Each record gets its OWN `street_score`
    closure bound to its own building and block rect, handed to `ufs.
    entry_for_plan_fire` alongside the per-record `rng` it already got —
    see that function's docstring for exactly where the score can and
    cannot move a side (the entry side itself never moves; the origin's
    free choice and F3+'s single extra corner side do)."""
    from disaster import urban_fire_city as ufc
    from disaster import urban_fire_spread as ufs

    plan_records = []
    entry_meta = {}   # global_i -> {btype, entry_side, origin_frac, n_storeys}
    for p in plan:
        if p.get("t_ignite") is None:
            continue
        if p["t_ignite"] > elapsed_s:
            # A FUTURE ignition (reachable, but not yet caught at this scene's
            # snapshot time) is not a damaged building -- see module
            # docstring's note on the seed sweep. `level_for_age` would have
            # already returned "F0" for it (negative age); dropping it here
            # is the same decision made explicit.
            continue
        gi = local_to_global[p["i"]]
        b = buildings[p["i"]]
        n_st = n_storeys[p["i"]]
        block_rect = _block_rect_at(layout, b["x"], b["y"])
        street_score = (lambda s, _b=b, _rect=block_rect:
                       ufs.street_side_score(_b, s, buildings, _rect))
        storey, sides = ufs.entry_for_plan_fire(
            p, n_st, random.Random(int(seed) + 97 * gi),
            street_score=street_score)
        via_local = p.get("via")
        rec = {
            "i": gi,
            "level": p["level"],
            "origin": storey,
            "sides": list(sides),
            "t_ignite_s": p["t_ignite"],
            "age_s": p["age"],
            "via": None if via_local is None else local_to_global[via_local],
            "how": p["how"],
            "W": b["W"], "D": b["D"], "H": b["H"],
        }
        plan_records.append(rec)
        entry_meta[gi] = {
            "btype": final_btype[p["i"]],
            "entry_side": p.get("entry_side"),
            "origin_frac": p.get("origin_frac"),
            "n_storeys": n_st,
        }

    dm_kwargs = {}
    if roof_collapse_max is not None:
        dm_kwargs["roof_collapse_max"] = roof_collapse_max
    manifest, refused_burn = ufc.damaged_manifest(
        layout, placements, plan_records, seed_base=int(seed), **dm_kwargs)
    for rec in manifest:
        meta = entry_meta.get(rec["i"], {})
        rec.update(meta)

    refused = list(refused_gate) + list(refused_burn)
    origin_gi = local_to_global[origin_local]
    roof_outcome_count = sum(1 for rec in manifest if rec["level"] in ufs.ROOF_LEVELS)
    out = {
        "seed": int(seed), "preset": preset, "n": int(n),
        "n_achieved": len(manifest), "origin": origin_gi,
        "epoch_s": elapsed_s, "records": manifest, "refused": refused,
        "roof_outcome_count": roof_outcome_count,
    }
    if dump_provenance:
        out["placements_dump"] = dict(dump_provenance)
    return out


# ---------------------------------------------------------------------------
# The seven §5 checks — pure functions over a manifest dict, no stage.
# ---------------------------------------------------------------------------
def _urban_fire_module():
    try:
        from disaster import urban_fire as m
        return m
    except Exception:
        return None


def check_district_rule(manifest):
    """THE HEIGHT-CLASS RULE (2026-08-31), not the old district-wide fire
    ban `ufc.NO_FIRE_TYPOLOGIES` used to police (that ban is lifted; the
    constant is kept, now empty, purely for `urban_fire_city_launch_
    script.py`'s own separate re-assertion — see `urban_fire_city`'s module
    docstring). A record is a "violation" here if its OWN `level` is not
    exactly what `disaster.urban_fire_spread.cap_level_for_class` +
    `enforce_roof_eligibility` would produce for its `typology` — i.e. the
    manifest is checked for having ALREADY applied the same policy
    `damaged_manifest` is authoritative for, not for avoiding certain
    typologies altogether."""
    from disaster import urban_fire_spread as ufs

    bad = []
    for r in manifest["records"]:
        typ = r.get("typology")
        level = r.get("level")
        cls = ufs.height_class(typology=typ, n_storeys=r.get("n_storeys"))
        allowed = ufs.enforce_roof_eligibility(
            ufs.cap_level_for_class(level, cls), cls)
        if allowed != level:
            bad.append(r["i"])
    ok = not bad
    return ok, {"violations": bad,
               "typologies_seen": sorted({r.get("typology")
                                          for r in manifest["records"]})}


def check_contiguity(manifest):
    """Every record traces back to a declared origin, with no cycles.

    THE FOREST CASE (2026-08-31, the two-seed union manifest): before the
    FIRE_MAX_H_M height gate (`urban_fire_city`'s gate 6), a two-seed union
    built from `downtown_fire_500` seeds 4 and 35 always ended up with a
    SINGLE root in practice -- seed 35's own fire happened to be a strict
    subset of seed 4's own reach every time (both funnelled through the
    same 302 m `SM_Building_31` stepping stone), so `manifest["origins"]`
    (plural -- the union step's own record of BOTH seeds' starting points)
    was carried purely for documentation and this check's original
    single-root assertion never actually saw a second root to accept or
    reject. With `SM_Building_31` refused as a firebreak, the two seeds'
    fires genuinely stay separate (two disjoint trees) -- exactly what
    "union of two ignition points" always claimed to mean -- so this check
    now validates a FOREST: `manifest["origins"]`, when present, names
    every root a `via=None` record is allowed to be; every record must
    trace back to ONE of them, and the set of actual `via=None` records
    must equal that declared set exactly (no undeclared extra root, no
    declared root missing from the data). Without `"origins"` (every OTHER
    manifest this checks, including every single-fire manifest `run_dry`/
    `run_dry_from_dump` ever produces, and every existing test fixture)
    this falls back to `[manifest.get("origin")]`, so single-fire
    behaviour -- exactly one root, exactly matching `"origin"` -- is
    completely unchanged, including the `detail["root"]` key callers
    already read.
    """
    recs = manifest["records"]
    by_i = {r["i"]: r for r in recs}
    declared = manifest.get("origins")
    if declared is None:
        declared = [manifest.get("origin")]
    declared = sorted(set(o for o in declared if o is not None))
    found = sorted(r["i"] for r in recs if r.get("via") is None)
    if found != declared:
        return False, {"reason": "the records with via=None do not exactly "
                                 "match the manifest's declared origin(s)",
                       "found": found, "declared": declared}
    for r in recs:
        cur = r["i"]
        seen = set()
        while cur is not None:
            if cur in seen:
                return False, {"reason": "cycle in via chain", "at": cur}
            seen.add(cur)
            cur_rec = by_i.get(cur)
            if cur_rec is None:
                return False, {"reason": "via references a building not in "
                                         "the manifest", "from": r["i"],
                               "missing": cur}
            cur = cur_rec.get("via")
        if not seen & set(declared):
            return False, {"reason": "record does not trace back to any "
                                     "declared origin", "i": r["i"]}
    detail = {"n_records": len(recs), "roots": found}
    if len(found) == 1:
        detail["root"] = found[0]          # back-compat: the common case
    return True, detail


def check_level_distribution(manifest):
    recs = manifest["records"]
    from disaster import urban_fire_spread as ufs

    origin_rec = next((r for r in recs if r["i"] == manifest.get("origin")), None)
    if origin_rec is None:
        return False, {"reason": "origin building not present in records"}
    origin_ok = ufs.RANK.get(origin_rec["level"], -1) >= ufs.RANK["F5"]
    levels_present = {r["level"] for r in recs}
    has_f23 = bool(levels_present & {"F2", "F3"})
    has_f1 = "F1" in levels_present
    uf = _urban_fire_module()
    bad_ladder = []
    if uf is not None:
        for r in recs:
            bt, lvl = r.get("btype"), r.get("level")
            if bt and lvl and lvl not in uf.LADDER.get(bt, {}):
                bad_ladder.append([r["i"], bt, lvl])
    ok = origin_ok and has_f23 and has_f1 and not bad_ladder
    return ok, {"origin_level": origin_rec["level"], "origin_ok": origin_ok,
               "levels_present": sorted(levels_present,
                                        key=lambda l: ufs.RANK.get(l, 0)),
               "has_f2_or_f3": has_f23, "has_f1": has_f1,
               "ladder_violations": bad_ladder}


def check_entry_points(manifest):
    """(2026-08-31 policy) F3+ vents 2 OR 3 sides now, never just 1 -- see
    `disaster.urban_fire_spread.entry_for_plan_fire`'s own docstring for the
    side-count draw this checks. F1/F2 is unchanged: exactly 1.

    (2026-08-31, `ORIGIN_FRAC_CAP`) `origin_frac` may never exceed
    `urban_fire_spread.ORIGIN_FRAC_CAP` for ANY mechanism -- this replaces an
    earlier "spot origin_frac should be high (~0.88)" LOWER-bound check that
    is now the wrong invariant on purpose: `solve()` used to let a brand
    landing on the roof start a fire at 0.88 of a mass's height, which on a
    20-30 storey GAC tower pinned every flame/soot/smoke event to a handful
    of storeys under the roof with the entire lower building untouched --
    "fire still seems to be on the higher floors rather than all over" (user
    review against the 39-record `fire_city_500m` manifest). The solver now
    caps every mechanism's `origin_frac` at `ORIGIN_FRAC_CAP`, and
    `tools/fire_city_lower_origins.py` redraws a low-biased `origin_frac` for
    any pre-existing record above it -- `how` stays whatever mechanism lit
    the building (a `spot` record can legitimately have a low `origin_frac`
    now), so the UPPER bound is the only thing left to check."""
    from disaster import urban_fire_spread as ufs

    bad = []
    for r in manifest["records"]:
        sides = r.get("sides") or []
        rank = ufs.RANK.get(r.get("level"), 0)
        if rank >= ufs.RANK["F3"] and len(sides) not in (2, 3):
            bad.append([r["i"], f"F3+ should vent 2-3 sides, got {sides}"])
        elif ufs.RANK["F1"] <= rank < ufs.RANK["F3"] and len(sides) != 1:
            bad.append([r["i"], f"F1/F2 should vent 1 side, got {sides}"])
        for s in sides:
            if s not in ("S", "E", "N", "W"):
                bad.append([r["i"], f"bad compass side {s!r}"])
        n_st, storey = r.get("n_storeys"), r.get("origin")
        if n_st is not None and storey is not None and not (0 <= storey < n_st):
            bad.append([r["i"], f"storey {storey} out of range [0,{n_st})"])
        how, frac = r.get("how"), r.get("origin_frac")
        if frac is not None and frac > ufs.ORIGIN_FRAC_CAP + 1e-9:
            bad.append([r["i"], f"origin_frac {frac} exceeds ORIGIN_FRAC_CAP "
                               f"({ufs.ORIGIN_FRAC_CAP}) -- no mechanism may "
                               f"start a fire above half the mass"])
        if how == "attached" and frac is not None and frac > 0.4:
            bad.append([r["i"], f"attached origin_frac should be low "
                               f"(~0.22), got {frac}"])
    return (not bad), {"violations": bad, "n_checked": len(manifest["records"])}


def check_bakeability(manifest):
    from disaster import fire_bake as fb

    bad_kind = [r["i"] for r in manifest["records"] if r.get("kind") not in fb.KINDS]
    refused = manifest.get("refused", [])
    no_reason = [r for r in refused if not r.get("reason")]
    ok = not bad_kind and not no_reason
    return ok, {"bad_kind": bad_kind, "n_refused": len(refused),
               "refusals_without_reason": len(no_reason)}


def _obb_corners(x, y, W, D, yaw_deg):
    a = math.radians(yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    hw, hd = W / 2.0, D / 2.0
    return [(x + ca * dx - sa * dy, y + sa * dx + ca * dy)
           for dx, dy in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))]


#: how much INTERPENETRATION (not clearance) is tolerated before two
#: footprints count as overlapping — see `_obb_overlap`'s docstring. 5 cm:
#: comfortably above floating-point noise on a party wall (measured exactly
#: 0.0 m separation on a real terrace pair, seed 202, after the resolver
#: patch in `_patch_resolver_for_gac_dtc`) and comfortably below anything
#: that would read as a placement bug.
OVERLAP_TOL_M = 0.05


def _obb_overlap(c1, c2, tol=OVERLAP_TOL_M):
    """Separating-axis test over two axis-yawed rectangles' corners.

    TOUCHING IS NOT OVERLAPPING. A terrace stands its buildings edge to edge
    on purpose — `urban_fire_spread.gap_m` (the same edge-to-edge distance
    the fire model itself uses) reports exactly `0.0` for a party-wall pair,
    which is the ATTACHED mechanism's own domain (`ATTACHED_GAP_M`), not a
    placement bug. A strict `> 0` separating-axis test flags an exact (or
    floating-point-noisy) zero gap as "overlap", which is wrong for exactly
    the pairs this check exists to pass. `tol` shifts the boundary so a gap
    down to `-tol` (a small INTERPENETRATION, not a clearance) still reads
    as "touching"; only a separation worse than `-tol` on EVERY axis is a
    genuine overlap.
    """
    for corners in (c1, c2):
        for i in range(len(corners)):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % len(corners)]
            nx, ny = -(y2 - y1), (x2 - x1)
            p1 = [nx * px + ny * py for px, py in c1]
            p2 = [nx * px + ny * py for px, py in c2]
            if max(p1) <= min(p2) + tol or max(p2) <= min(p1) + tol:
                return False
    return True


def check_footprint(manifest, manifest_dg0=None, area_ratio_max=1.3):
    """No two damaged footprints overlap (beyond `OVERLAP_TOL_M` — touching,
    e.g. a terrace party wall, is not an overlap); for a `kit` record, the
    twin style's own plan area is within `area_ratio_max` of the cell's —
    see plan §5's own allowance to substitute this for `districts.
    free_rects`."""
    recs = manifest["records"]
    overlaps = []
    for a in range(len(recs)):
        ra = recs[a]
        ca = _obb_corners(ra["x"], ra["y"], ra["W"], ra["D"], ra["yaw_deg"])
        for b in range(a + 1, len(recs)):
            rb = recs[b]
            cb = _obb_corners(rb["x"], rb["y"], rb["W"], rb["D"], rb["yaw_deg"])
            if _obb_overlap(ca, cb):
                overlaps.append([ra["i"], rb["i"]])
    ratio_bad = []
    if manifest_dg0:
        for r in recs:
            if r.get("kind") != "kit":
                continue
            twin = manifest_dg0.get(r.get("style"))
            if not twin:
                continue
            twin_area = float(twin.get("W", 0)) * float(twin.get("D", 0))
            cell_area = float(r.get("W", 0)) * float(r.get("D", 0))
            if cell_area <= 0 or twin_area <= 0:
                continue
            ratio = max(twin_area, cell_area) / min(twin_area, cell_area)
            if ratio > area_ratio_max:
                ratio_bad.append([r["i"], r.get("style"), round(ratio, 3)])
    ok = not overlaps and not ratio_bad
    return ok, {"overlaps": overlaps, "area_ratio_violations": ratio_bad,
               "area_ratio_max": area_ratio_max}


def check_determinism(seed, run_fn):
    """`run_fn(seed) -> JSON-serialisable manifest dict`, called three times:
    twice back to back, and once after 50 unrelated draws on the GLOBAL
    `random` module — proving this tool never reads global random state (only
    its own locally-seeded `random.Random` instances)."""
    a = run_fn(seed)
    b = run_fn(seed)
    ja, jb = json.dumps(a, sort_keys=True), json.dumps(b, sort_keys=True)
    repeat_identical = ja == jb
    for _ in range(50):
        random.random()
    c = run_fn(seed)
    stable_after_noise = json.dumps(c, sort_keys=True) == ja
    ok = repeat_identical and stable_after_noise
    return ok, {"repeat_identical": repeat_identical,
               "stable_after_unrelated_draws": stable_after_noise}


CHECKS = (
    ("district_rule", check_district_rule),
    ("contiguity", check_contiguity),
    ("level_distribution", check_level_distribution),
    ("entry_points", check_entry_points),
    ("bakeability", check_bakeability),
    ("footprint", check_footprint),
)


def run_all_checks(manifest, manifest_dg0=None):
    """`{name: (ok, detail)}` for the six manifest-only checks (determinism
    is run separately — see `check_determinism` — because it needs to
    RE-RUN the pipeline, not just read one manifest)."""
    out = {}
    for name, fn in CHECKS:
        if name == "footprint":
            out[name] = fn(manifest, manifest_dg0=manifest_dg0)
        else:
            out[name] = fn(manifest)
    return out


# ---------------------------------------------------------------------------
# The orchestrator
# ---------------------------------------------------------------------------
def _solve_from_layout(preset, resolved_seed, n, collapse, config, layout,
                       placements, resolver, fire_spec, dump_provenance=None,
                       roof_collapse_max=None, extra_blocked_global=None):
    """The burnable-set / spread / manifest tail shared by `run_dry`
    (layout freshly built in-process, `resolver` a real `SizeResolver`) and
    `run_dry_from_dump` (layout loaded from a `FC_DUMP` placements dump,
    `resolver=None` — every W/D/H is already on the placement dict). Neither
    caller does anything with `config`/`layout`/`placements` after this
    point that the other doesn't.

    `roof_collapse_max` — see `build_manifest`'s own docstring — flows
    straight through to it; `None` means "use `urban_fire_city.
    damaged_manifest`'s own default".

    `extra_blocked_global` — see `run_spread`'s own docstring for what this
    is FOR (a caller-supplied firebreak, e.g. `tools/fire_city_union.py`'s
    auto-clean loop dropping a placement that overlaps another one in the
    dump's own geometry). Given here in the placements list's GLOBAL index
    space (an `i` a caller who read `manifest["records"]`/`["refused"]`
    already has); translated to `run_spread`'s LOCAL (`buildings`-list)
    space via `local_to_global` right below, the same translation
    `build_manifest`'s own `via`/`i` fields already do in the other
    direction."""
    from disaster import quake as q

    burnable_list, refused_gate, building_typology = gather_burnable(
        config, layout, placements, resolver)

    arch_dir = os.path.join(_SCENE_GEN_DIR, "assets", "archetype")
    full_manifest = q.load_manifest(arch_dir) if os.path.isfile(
        os.path.join(arch_dir, "archetypes.json")) else {}
    manifest_dg0 = {style: rec for (style, level), rec in full_manifest.items()
                    if level == "DG0"}
    cache = _gac_dtc_cache()

    buildings, local_to_global, final_btype, n_storeys, height_class = \
        build_solve_inputs(burnable_list, manifest_dg0, cache)

    extra_blocked_set = frozenset(extra_blocked_global or ())
    extra_blocked_local = frozenset(
        k for k, gi in enumerate(local_to_global) if gi in extra_blocked_set)

    plan, origin_local, elapsed_s = run_spread(
        buildings, local_to_global, final_btype, height_class, fire_spec,
        resolved_seed, n, extra_blocked_local=extra_blocked_local)
    notes = _enforce_target_f5c(plan, origin_local, final_btype, height_class,
                                target=collapse)

    manifest = build_manifest(preset, resolved_seed, n, layout, placements,
                              buildings, local_to_global, final_btype,
                              n_storeys, plan, origin_local, elapsed_s,
                              refused_gate, dump_provenance=dump_provenance,
                              roof_collapse_max=roof_collapse_max)

    checks = run_all_checks(manifest, manifest_dg0=manifest_dg0)

    from collections import Counter
    block_typ_tally = Counter(layout.get("_typology_of", {}).values())
    building_typ_tally = Counter(building_typology.values())
    refusal_reason_tally = Counter()
    for r in refused_gate:
        reason = r["reason"]
        key = reason.split(" -- ")[0].split(":")[0][:70] if isinstance(reason, str) \
            else str(reason)[:70]
        refusal_reason_tally[key] += 1

    report_extras = {
        "block_typology_tally": dict(sorted(block_typ_tally.items())),
        "building_typology_tally": dict(sorted(building_typ_tally.items())),
        "n_house_placements": sum(1 for p in placements if p.get("category") == "house"),
        "n_burnable_candidates": len(burnable_list),
        "n_refused_gate": len(refused_gate),
        "refusal_reason_tally": dict(refusal_reason_tally),
        "fire_spec": fire_spec,
        "buildings": buildings, "local_to_global": local_to_global,
        "final_btype": final_btype, "plan": plan, "notes": notes,
        "extra_blocked_global": sorted(extra_blocked_set),
    }
    return manifest, checks, report_extras


def run_dry(preset: str, seed=None, n: int = 16, collapse: int = 1,
           roof_collapse_max=None, extra_blocked_global=None):
    """Runs the whole pipeline once, building the layout FRESH, in this
    process (needs `pxr` — see `build_layout`). Returns `(manifest, checks,
    report_extras)` — `report_extras` carries the numbers the markdown
    report wants that are not part of the JSON manifest itself (typology
    tallies, refusal-reason tally, the per-building spread table).

    `extra_blocked_global` — see `run_spread`'s docstring; passed straight
    through to `_solve_from_layout`."""
    config, layout, placements, resolver = build_layout(preset, seed=seed)
    resolved_seed = int(config.get("seed", seed if seed is not None else 0))
    fire_spec = _raw_fire_spec(preset)
    return _solve_from_layout(preset, resolved_seed, n, collapse, config,
                              layout, placements, resolver, fire_spec,
                              roof_collapse_max=roof_collapse_max,
                              extra_blocked_global=extra_blocked_global)


def run_dry_from_dump(dump_path: str, seed=None, n: int = 16, collapse: int = 1,
                      roof_collapse_max=None, extra_blocked_global=None):
    """`run_dry`, but the layout is LOADED from a placements dump the city
    launcher wrote (`FC_DUMP`, `FC_INTACT_ONLY=1`) instead of rebuilt
    host-side — see `load_placements_dump` and the module docstring's
    "2026-08-30 manifest/city mismatch" note. The layout is never packed
    here, only read back, and `pxr` is not needed for that — but see
    `load_placements_dump`'s own note: `compile_disaster.load_scene_config`
    (which does need `pxr`) is still called, LAZILY, if the dump carries a
    `same_art` MCE placement.

    `seed`, if given, overrides the SPREAD SOLVE's own seed only (re-roll
    the fire on the same frozen city) — the layout itself is exactly what
    the dump recorded, regardless of `seed`. `preset` is always the one the
    dump itself names (the layout was solved for that preset's district
    rules; there is no "override" that would still describe the same
    city).

    `extra_blocked_global` — see `run_spread`'s docstring; passed straight
    through to `_solve_from_layout`. This is `tools/fire_city_union.py`'s
    hook for excluding a placement index known (from a prior run's own
    `check_footprint` violation) to overlap another one in THIS dump's
    geometry — a data defect, not a fire-model decision, fenced off the
    same way any other firebreak is."""
    config, layout, placements, dump_seed, preset, dump_sha256 = \
        load_placements_dump(dump_path)
    resolved_seed = int(seed) if seed is not None else dump_seed
    fire_spec = _raw_fire_spec(preset)
    dump_provenance = {"path": os.path.abspath(dump_path), "sha256": dump_sha256}
    return _solve_from_layout(preset, resolved_seed, n, collapse, config,
                              layout, placements, None, fire_spec,
                              dump_provenance=dump_provenance,
                              roof_collapse_max=roof_collapse_max,
                              extra_blocked_global=extra_blocked_global)


def _manifest_only(preset, seed, n=16, collapse=1, roof_collapse_max=None):
    """`run_dry`, returning JUST the JSON-serialisable manifest — what
    `check_determinism`'s `run_fn` wants (re-running the whole pipeline is
    the point: it proves the LAYOUT is deterministic too, not just the
    spread solve)."""
    manifest, _checks, _extras = run_dry(preset, seed=seed, n=n, collapse=collapse,
                                         roof_collapse_max=roof_collapse_max)
    return manifest


def _manifest_only_from_dump(dump_path, seed, n=16, collapse=1,
                             roof_collapse_max=None):
    """`run_dry_from_dump`, returning JUST the manifest — the layout is
    frozen (the dump), so this only proves the spread solve/manifest
    assembly is deterministic for a given seed, not the layout too."""
    manifest, _checks, _extras = run_dry_from_dump(
        dump_path, seed=seed, n=n, collapse=collapse,
        roof_collapse_max=roof_collapse_max)
    return manifest


# ---------------------------------------------------------------------------
# Markdown report
# ---------------------------------------------------------------------------
def _format_markdown(preset, manifest, checks, det_ok, det_detail, extras):
    lines = [f"# Urban fire city dry run — `{preset}` seed {manifest['seed']}\n"]
    fs = extras["fire_spec"]
    lines.append(f"epicenter {fs['epicenter']}, heading {fs['heading_deg']} deg, "
                f"wind {fs['wind_mps']} m/s, duration {fs['duration_s']} s, "
                f"start_offset_frac {fs['start_offset_frac']} -> "
                f"epoch_s {manifest['epoch_s']:.1f} "
                f"({manifest['epoch_s']/60.0:.1f} min)\n")
    lines.append(f"Requested N = {manifest['n']}, achieved = "
                f"{manifest['n_achieved']}.\n")

    lines.append("## Typology tally\n")
    lines.append("### Blocks\n")
    lines.append("| typology | blocks |")
    lines.append("|---|---|")
    for k, v in extras["block_typology_tally"].items():
        lines.append(f"| {k} | {v} |")
    lines.append("")
    lines.append("### Buildings (all house placements, not just burnable)\n")
    lines.append("| typology | buildings |")
    lines.append("|---|---|")
    for k, v in extras["building_typology_tally"].items():
        lines.append(f"| {k} | {v} |")
    lines.append("")

    lines.append("## Burnable / refused\n")
    lines.append(f"* {extras['n_house_placements']} house placements total\n"
                f"* {extras['n_burnable_candidates']} burnable candidates "
                f"(passed all four gates)\n"
                f"* {extras['n_refused_gate']} refused at the gate\n")
    lines.append("### Refusal reasons\n")
    lines.append("| reason (prefix) | count |")
    lines.append("|---|---|")
    for k, v in sorted(extras["refusal_reason_tally"].items(), key=lambda kv: -kv[1]):
        lines.append(f"| {k} | {v} |")
    lines.append("")

    origin_rec = next((r for r in manifest["records"] if r["i"] == manifest["origin"]), None)
    lines.append("## Origin\n")
    if origin_rec:
        lines.append(f"placements index {manifest['origin']}, `{origin_rec.get('kind')}:"
                    f"{origin_rec.get('asset') or origin_rec.get('style')}`, "
                    f"typology `{origin_rec.get('typology')}`, btype "
                    f"`{origin_rec.get('btype')}`, level **{origin_rec.get('level')}**, "
                    f"at ({origin_rec.get('x'):.1f}, {origin_rec.get('y'):.1f})\n")
    if extras["notes"]:
        lines.append("Enforcement notes:\n")
        for note in extras["notes"]:
            lines.append(f"* {note}")
        lines.append("")

    lines.append("## Level histogram\n")
    from collections import Counter
    from disaster import urban_fire_spread as ufs
    hist = Counter(r["level"] for r in manifest["records"])
    hist_by_class = Counter((r.get("height_class"), r["level"])
                            for r in manifest["records"])
    lines.append("| level | count |")
    lines.append("|---|---|")
    for lv in ("F0", "F1", "F2", "F3", "F4", "F5", "F5c", "F6"):
        if hist.get(lv):
            lines.append(f"| {lv} | {hist[lv]} |")
    lines.append("")
    lines.append(f"Roof-affecting outcomes (F5c/F6, `urban_fire_spread."
                f"ROOF_LEVELS`): **{manifest.get('roof_outcome_count', 0)}** "
                f"of {len(manifest['records'])} records -- eligible only "
                f"for the `low` height class, capped by `damaged_manifest`'s "
                f"`roof_collapse_max`.\n")
    lines.append("| height class | level | count |")
    lines.append("|---|---|---|")
    for (cls, lv), cnt in sorted(hist_by_class.items(),
                                 key=lambda kv: (str(kv[0][0]), ufs.RANK.get(kv[0][1], 0))):
        lines.append(f"| {cls} | {lv} | {cnt} |")
    lines.append("")

    lines.append("## Spread tree (who lit whom, how, at what minute)\n")
    lines.append("| i | kind:name | typology | btype | level | t_ignite (min) | "
                "how | via | entry side |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for r in sorted(manifest["records"], key=lambda r: r["t_ignite_s"]):
        name = r.get("asset") or r.get("style")
        lines.append(f"| {r['i']} | {r.get('kind')}:{name} | {r.get('typology')} | "
                    f"{r.get('btype')} | {r['level']} | "
                    f"{r['t_ignite_s']/60.0:.1f} | {r.get('how')} | "
                    f"{r.get('via')} | {r.get('entry_side')} |")
    lines.append("")

    lines.append("## The seven checks\n")
    lines.append("| check | result | detail |")
    lines.append("|---|---|---|")
    for name, (ok, detail) in checks.items():
        lines.append(f"| {name} | {'PASS' if ok else 'FAIL'} | "
                    f"`{json.dumps(detail)}` |")
    lines.append(f"| determinism | {'PASS' if det_ok else 'FAIL'} | "
                f"`{json.dumps(det_detail)}` |")
    lines.append("")

    lines.append("## `fire_bake.sh` entry list\n")
    from disaster import urban_fire_city as ufc
    lines.append("```")
    for r in sorted(manifest["records"], key=lambda r: r["i"]):
        lines.append(ufc.entry_string(r))
    lines.append("```\n")

    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preset", default="downtown_fire_500",
                    help="ignored when --placements-json is given -- the "
                         "preset recorded in the dump is used instead, "
                         "since that is the preset the layout was actually "
                         "solved for")
    ap.add_argument("--placements-json", default=None,
                    help="skip layout generation entirely and run the "
                         "burnable-set/spread/manifest steps on a city "
                         "placements dump the launcher wrote (FC_DUMP, "
                         "FC_INTACT_ONLY=1) -- the SAME layout Kit built, "
                         "not a host-side reconstruction with substituted "
                         "GAC/DTC footprints that can pack differently")
    ap.add_argument("--seed", type=int, default=None,
                    help="override the preset's own seed; drives layout, "
                         "the fire solve and per-building bake seeds. With "
                         "--placements-json the layout is already fixed by "
                         "the dump, so this only re-rolls the fire solve")
    ap.add_argument("--n", type=int, default=16)
    ap.add_argument("--collapse", type=int, default=1,
                    help="target number of F5c (partial-collapse) buildings")
    ap.add_argument("--roof-collapse-max", type=int, default=None,
                    help="max buildings in the manifest that may show a "
                         "roof-affecting outcome (F5c/F6) at all -- overrides "
                         "urban_fire_city.damaged_manifest's own "
                         "ROOF_COLLAPSE_MAX_DEFAULT (2). Roof collapse is "
                         "eligible only for the low/brownstone/timber height "
                         "class regardless of this budget.")
    ap.add_argument("--out", default=None, help="override the JSON out path")
    ap.add_argument("--md", default=None, help="override the markdown out path")
    args = ap.parse_args()

    if args.placements_json:
        manifest, checks, extras = run_dry_from_dump(
            args.placements_json, seed=args.seed, n=args.n,
            collapse=args.collapse, roof_collapse_max=args.roof_collapse_max)
        run_fn = lambda s: _manifest_only_from_dump(
            args.placements_json, s, n=args.n, collapse=args.collapse,
            roof_collapse_max=args.roof_collapse_max)
    else:
        manifest, checks, extras = run_dry(args.preset, seed=args.seed, n=args.n,
                                           collapse=args.collapse,
                                           roof_collapse_max=args.roof_collapse_max)
        run_fn = lambda s: _manifest_only(args.preset, s, n=args.n,
                                          collapse=args.collapse,
                                          roof_collapse_max=args.roof_collapse_max)
    preset = manifest["preset"]
    seed = manifest["seed"]

    det_ok, det_detail = check_determinism(seed, run_fn)

    out_json = args.out or os.path.join(_SCENE_GEN_DIR, "_plans",
                                        f"fire_city_{seed}.json")
    out_md = args.md or os.path.join(_SCENE_GEN_DIR, "_plans",
                                     f"fire_city_{seed}_report.md")
    os.makedirs(os.path.dirname(out_json), exist_ok=True)
    with open(out_json, "w") as fh:
        json.dump(manifest, fh, indent=1)
    print(f"[fire_dry_run] wrote {out_json}")

    with open(out_md, "w") as fh:
        fh.write(_format_markdown(preset, manifest, checks, det_ok,
                                  det_detail, extras))
    print(f"[fire_dry_run] wrote {out_md}")

    print(f"\n[fire_dry_run] seed={seed} n={manifest['n']} "
         f"achieved={manifest['n_achieved']} origin={manifest['origin']} "
         f"roof_outcome_count={manifest.get('roof_outcome_count', 0)}")
    for name, (ok, detail) in checks.items():
        print(f"  {name:<20} {'PASS' if ok else 'FAIL'}  {detail}")
    print(f"  {'determinism':<20} {'PASS' if det_ok else 'FAIL'}  {det_detail}")

    all_ok = det_ok and all(ok for ok, _ in checks.values())
    if not all_ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
