#!/usr/bin/env python3
"""hurricane_house_pose_bake.py — rebuild the top of the hurricane house
ladder OFFLINE, bare pxr, no Isaac Sim / SimulationApp / physics.

WHY THIS EXISTS. `cover_lost` / `deck_panels_lost` / `roof_stripped` used to
be an EMPTY BOX once a bay drops — no rafters, no ceiling joists, nothing
under the hole. This tool builds all three as NEW files
(`house_<style>_<level>_h.usd`) from each style's own
`house_<style>_pristine.usd`, entirely offline, dispatching to the REAL
`hurricane_flow` functions (`strip_roof`, `blow_out_windows`, `_blow_doors`,
`author_rafters`) — the same code the live Kit-based launcher would call, so
this is not a second implementation to drift from the first.

STREAM Q, 2026-08-31 -- `author_rafters` now RAGGEDIZES the cage (see
`hurricane_flow.py`'s "RAGGED RAFTERS" section): a share of rafters removed
outright, another share snapped and drooping, plus ragged sheathing patches
and ridge shreds — a perfectly regular cage read as under-construction
rather than storm-stripped. `--build`/`--verify`/`--promote` (and their
`--*-variants` siblings) rebuild `RAFTER_LEVELS`
(`cover_lost`/`deck_panels_lost`/`roof_stripped`) ONLY — see the next
paragraph for `roof_collapsed`/`partial_collapse`/`leveled`, which this tool
handles through a DIFFERENT pair of commands.

`roof_collapsed` / `partial_collapse` / `leveled` (`hf._POSE_LEVELS`) are NO
LONGER rebuilt through `--build`/`--promote`. Those levels' CANONICAL files
are now the TORNADO library's own fracture+settle output (repaired, not the
pose-authored rigid squash+tilt) — see `cmd_adopt_tornado`'s own docstring
for the full argument and `hurricane_flow.py`'s "TOP RUNGS, NO PHYSICS"
section for why the pose-authored version existed in the first place. Use
`--adopt-tornado` (one-time swap + repair) and `--verify-tornado` (re-audit,
no changes) for these three levels; the pose-authored code
(`hurricane_flow.pose_roof_collapsed`/`pose_partial_collapse`/`pose_leveled`)
is UNCHANGED and still runs — its output is preserved as
`house_<style>_<level>_pose.usd` for an A/B, never deleted.

Every floor mesh in every output file is (re)bound to a hardened
`planks.wood_material` regardless of whether it already had a binding — see
`harden_wood_material`'s docstring for why "already bound" was not enough.

Runs in seconds. Needs `pxr` — use `scene_gen/tools/usd_python.sh` inside
the isaac-sim container (bare pxr, no Kit app, no SimulationApp), or a bare
host `python3` with `pxr` importable (this session's own method — see the
`build-hurricane-scenes`/`build-earthquake-scenes` skills for why host
python3 is preferred: files it writes are user-owned, not root-owned):

    docker exec isaac-sim bash -c 'cd /isaac-sim/AirStack && \
        bash scene_gen/tools/usd_python.sh \
        scene_gen/tools/hurricane_house_pose_bake.py --report'

    ... --build             # RAFTER_LEVELS only: write _h.usd + manifest
    ... --verify            # point-bounds + floor-binding audit on _h files
    ... --promote           # _h -> canonical, canonical -> _tornado backup

    ... --build-variants    # JOB B: house_<style>_<level>_{n,e,s,w}_h.usd
                            # for cover_lost/deck_panels_lost only -- a
                            # bake-time-fixed dropped-bay side, one per
                            # cardinal direction, so the launcher can pick
                            # the one matching a house's own (street yaw,
                            # local wind bearing) via
                            # `hurricane_flow.windward_variant` instead of
                            # always dropping bays toward the same fixed
                            # side regardless of which way the wind hit.
    ... --verify-variants   # same audit as --verify, over the variant files
    ... --promote-variants  # _{side}_h -> _{side} canonical; the "n"
                            # variant's content ALSO becomes the unsuffixed
                            # canonical file (see `cmd_promote_variants`)

    ... --adopt-tornado     # roof_collapsed/partial_collapse/leveled ONLY:
                            # swap canon to the (repaired) tornado library,
                            # backing up the current canon as _pose.usd
    ... --verify-tornado    # re-audit the current canon for those 3 levels

Env:
    ARCH_DIR     archetype library (default scene_gen/assets/archetypes_hurricane)
    ARCH_STYLES  comma list of styles (default: every house_*_pristine.usd found)
    ARCH_SEED    seed for the pose/rafter draws (default 7)
"""
import argparse
import json
import math
import os
import random
import shutil
import sys
import time

os.environ.setdefault("PXR_USDC_EMIT_DEPRECATION_WARNINGS", "0")

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN)

from pxr import Sdf, Usd, UsdGeom, UsdShade  # noqa: E402

from disaster import bake as bk               # noqa: E402
from disaster import hurricane_flow as hf     # noqa: E402
from disaster import planks                   # noqa: E402


def _env(name, default):
    v = os.environ.get(name)
    return default if v is None or not v.strip() else v.strip()


ARCH_DIR = _env("ARCH_DIR", os.path.join(_SCENE_GEN, "assets",
                                         "archetypes_hurricane"))
SEED = int(_env("ARCH_SEED", "7"))

# RAFTER_LEVELS ONLY (3), not `+ hf._POSE_LEVELS` any more -- STREAM Q,
# 2026-08-31: `roof_collapsed`/`partial_collapse`/`leveled` moved to the
# tornado-canon adoption pair (`cmd_adopt_tornado`/`cmd_verify_tornado`),
# which does not go through `_h.usd`/`--promote` at all. See the module
# docstring.
REBUILD_LEVELS = hf.RAFTER_LEVELS
SUFFIX = "_h"          # new-file suffix, kept alongside the original
BACKUP_SUFFIX = "_tornado"   # what the OLD file is renamed to on --promote


def _styles():
    req = _env("ARCH_STYLES", "")
    if req:
        return [s.strip() for s in req.split(",") if s.strip()]
    out = []
    for f in sorted(os.listdir(ARCH_DIR)) if os.path.isdir(ARCH_DIR) else []:
        if f.startswith("house_") and f.endswith("_pristine.usd"):
            out.append(f[len("house_"):-len("_pristine.usd")])
    return out


def _bind_all_floors(stage, house_prim, tag=""):
    """(Re)bind EVERY `house_floor*` mesh under `house_prim` to a hardened
    `planks.wood_material`, unconditionally — not "if unbound".

    WHY UNCONDITIONAL. The archetype library's existing floor materials
    audited CLEAN with a fresh bare-pxr walk (0 of 683 meshes unbound, every
    texture input resolved to a file that exists on disk on both the OSMO
    pod and the local bind-mounted container) and the rendered plate still
    showed a saturated flat colour on those same floors — see `harden_wood_
    material`'s docstring. "Bound" was not sufficient evidence the floor
    would render correctly, so this does not bother checking; it always
    clears whatever is there (`UnbindAllBindings`) and rebinds fresh.

    Returns `(seen, bound)`.
    """
    floors = [p for p in Usd.PrimRange(house_prim, Usd.PrimAllPrimsPredicate)
              if p != house_prim and p.IsA(UsdGeom.Mesh)
              and hf._category_of(p) == "house_floor"]
    if not floors:
        return 0, 0
    root = house_prim.GetPath().pathString
    mat = hf.harden_wood_material(planks.wood_material(
        stage, "{0}/Looks/subfloor{1}".format(root, tag),
        tile_m=2.2, tint=(0.92, 0.88, 0.82), roughness=0.86))
    n = 0
    for prim in floors:
        api = UsdShade.MaterialBindingAPI.Apply(prim)
        api.UnbindAllBindings()
        api.Bind(mat)
        n += 1
    return len(floors), n


def build_one(style, level, rng, variant=None):
    """Build `house_<style>_<level>_h.usd` (or, when `variant` is one of
    `hf.VARIANTS`, `house_<style>_<level>_<variant>_h.usd`) from the style's
    own pristine file. Returns (out_path, stats_dict) or (None, error_str).

    `variant` only changes anything for `level in hf.LEVELS_WITH_VARIANTS`
    (`cover_lost`/`deck_panels_lost`): it selects the `seed_dir` handed to
    `strip_roof` from `hf._VARIANT_SEED_DIR[variant]` instead of the fixed
    `0.0` every level used before JOB B -- see `hurricane_flow`'s "JOB B"
    section for why a single fixed bake-time bearing was wrong for 55% of a
    Level-3 plate's houses. Every OTHER rebuild level ignores `variant`
    entirely (passed or not) and keeps the original `seed_dir=0.0`.
    """
    src = os.path.join(ARCH_DIR, "house_{0}_pristine.usd".format(style))
    if not os.path.isfile(src):
        return None, "no pristine source at {0}".format(src)
    if variant and level in hf.LEVELS_WITH_VARIANTS:
        out = os.path.join(ARCH_DIR, "house_{0}_{1}_{2}{3}.usd".format(
            style, level, variant, SUFFIX))
    else:
        out = os.path.join(ARCH_DIR, "house_{0}_{1}{2}.usd".format(
            style, level, SUFFIX))
    if os.path.exists(out):
        os.remove(out)
    shutil.copy2(src, out)   # self-contained by value (bake.export_object) —
                             # a plain file copy is a fully independent stage

    stage = Usd.Stage.Open(out)
    if not stage:
        return None, "could not open copy at {0}".format(out)
    house = stage.GetDefaultPrim()
    if not house or not house.IsValid():
        return None, "{0} has no default prim".format(out)

    stats = {"level": level, "style": style}
    if variant:
        stats["variant"] = variant
    if level in hf._ROOF_FRAC:
        before = {b.GetPath(): b for b in hf.roof_bay_prims(house)
                 if b.IsActive()}
        roof_frac, win_frac = hf._ROOF_FRAC[level]
        # SEED_DIR: a fixed cardinal bearing (`hf._VARIANT_SEED_DIR[variant]`)
        # for the two levels JOB B gives per-side variants to, `0.0`
        # (`_BAKE_BEARING`'s own convention, same as the Kit-based launcher's
        # direct archetype builds) for every other rebuild level -- those
        # are either isotropic (pose levels) or drop EVERY bay
        # (`roof_stripped`, `_ROOF_FRAC`'s `1.00`), so no side is meaningful
        # for them. The bay ranking is still CONTIGUOUS and starts from one
        # eave rather than scattering, which is what reads as wind rather
        # than hail (`strip_roof`'s own docstring).
        seed_dir = (hf._VARIANT_SEED_DIR[variant]
                   if variant and level in hf.LEVELS_WITH_VARIANTS else 0.0)
        n_dropped = hf.strip_roof(house, roof_frac, rng, seed_dir=seed_dir)
        n_win = hf.blow_out_windows(house, win_frac, rng) if win_frac > 0 else 0
        if level != "shingles_lost":
            hf._blow_doors(house)
        rafters = None
        rafter_stats = {}
        if level in hf.RAFTER_LEVELS:
            dropped = [b for path, b in before.items() if not b.IsActive()]
            rafters, rafter_stats = hf.author_rafters(stage, house, dropped,
                                                       rng=rng)
        stats.update(bays_dropped=n_dropped, windows_blown=n_win,
                     rafters=bool(rafters))
        stats.update(rafter_stats)
    elif level in hf._POSE_LEVELS:
        if level == "roof_collapsed":
            n = hf.pose_roof_collapsed(stage, house, rng)
            stats["bays_posed"] = n
        elif level == "partial_collapse":
            n = hf.pose_partial_collapse(stage, house, rng)
            stats["walls_racked"] = n
        else:
            n = hf.pose_leveled(stage, house, rng)
            stats["walls_racked"] = n
    else:
        return None, "unknown rebuild level {0!r}".format(level)

    seen, bound = _bind_all_floors(stage, house)
    stats["floor_meshes"] = seen
    stats["floor_bound"] = bound

    stage.GetRootLayer().Save()
    return out, stats


# ---------------------------------------------------------------------------
# verification — point bounds, never BBoxCache
# ---------------------------------------------------------------------------
def _pristine_references(style):
    """`(ground_z, roof_rest_z)` read from `house_<style>_pristine.usd` —
    the FIXED, never-posed geometry, so the "does this piece still make
    sense" check has an UNCHANGING yardstick instead of comparing a posed
    file against itself (a roof bay that rotated 30 degrees is not a fair
    baseline for judging whether IT looks right).

    `ground_z` is the lowest floor level in the whole house (there can be
    more than one storey) -- nothing should ever end up more than a small
    tolerance below this. `roof_rest_z` is the highest point any WALL mesh
    reaches in the UNDAMAGED house -- i.e. `storeys * modular_house.STOREY_M`,
    where an intact roof naturally sits. Comparing a roof bay's own low
    point against the FLOOR (a few tens of centimetres off the ground) was
    the bug in the first cut of this audit: an untouched, correctly-seated
    single-storey roof bay sits about 3.2 m up, which is completely correct
    and was being reported as "floating" against a ~0.3 m floor reference.
    """
    src = os.path.join(ARCH_DIR, "house_{0}_pristine.usd".format(style))
    stage = Usd.Stage.Open(src)
    if not stage:
        return 0.0, 0.0
    house = stage.GetDefaultPrim()
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    floor_lo, wall_hi = None, None
    for prim in Usd.PrimRange(house, Usd.PrimAllPrimsPredicate):
        if prim == house or not prim.IsA(UsdGeom.Mesh):
            continue
        cat = hf._category_of(prim)
        wb = bk.world_point_bounds(prim, xcache)
        if wb is None:
            continue
        lo, hi = wb
        if cat == "house_floor":
            floor_lo = float(lo[2]) if floor_lo is None else min(floor_lo, float(lo[2]))
        elif cat == "house_wall":
            wall_hi = float(hi[2]) if wall_hi is None else max(wall_hi, float(hi[2]))
    return (floor_lo if floor_lo is not None else 0.0,
            wall_hi if wall_hi is not None else 3.5)


def audit_file(path, style, margin_above_m=0.05):
    """Per-category (min_z, max_z) over every mesh in `path`, plus:

      * `floating`   -- ROOF/BAY_ROOF meshes whose own lowest point sits MORE
                        than `margin_above_m` above `roof_rest_z` (an intact
                        roof's own resting height, read from the style's
                        PRISTINE file — see `_pristine_references`) — the
                        coordinator's "frozen mid-air ~10 m above the water"
                        check. WALLS are excluded here: a racked wall's own
                        pivot edge is deliberately AT the floor (`_clamp_
                        above` guarantees it), so "wall floats above the
                        roof line" is not a meaningful question for it.
      * `below_slab` -- any mesh (any category) whose lowest point sits more
                        than 30 cm below `ground_z` (the house's own lowest
                        floor, from the pristine reference — never the
                        mutated file's own floors, which a pose might have
                        moved).
      * `unbound_floor` -- floor meshes with no bound material.

    Uses `bake.world_point_bounds`, never `UsdGeom.BBoxCache` — see that
    function's own docstring for the AABB-of-an-AABB trap this avoids.
    """
    stage = Usd.Stage.Open(path)
    if not stage:
        return {"error": "could not open {0}".format(path)}
    house = stage.GetDefaultPrim()
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    ground_z, roof_rest_z = _pristine_references(style)

    per_cat = {}
    floating, below_slab, unbound = [], [], []
    roof_categories = ("house_roof", "house_bay_roof")
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        cat = hf._category_of(prim)
        wb = bk.world_point_bounds(prim, xcache)
        if wb is None:
            continue
        lo, hi = wb
        c = per_cat.setdefault(cat, {"min_z": 1e30, "max_z": -1e30, "n": 0})
        c["min_z"] = min(c["min_z"], float(lo[2]))
        c["max_z"] = max(c["max_z"], float(hi[2]))
        c["n"] += 1
        if cat in roof_categories and float(lo[2]) > roof_rest_z + margin_above_m:
            floating.append((prim.GetName(), float(lo[2])))
        if float(lo[2]) < ground_z - 0.30:   # 30 cm slack below the ground floor
            below_slab.append((prim.GetName(), float(lo[2])))
        if cat == "house_floor":
            mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[:2]
            if not mat or not mat.GetPrim().IsValid():
                unbound.append(prim.GetName())

    return dict(path=path, ground_z=ground_z, roof_rest_z=roof_rest_z,
                per_category=per_cat, floating=floating,
                below_slab=below_slab, unbound_floor=unbound)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def cmd_build():
    styles = _styles()
    if not styles:
        print("[hpose] no house_*_pristine.usd found under {0}".format(ARCH_DIR))
        return 1
    print("[hpose] building {0} level(s) x {1} style(s) from pristine, seed={2}"
          .format(len(REBUILD_LEVELS), len(styles), SEED))
    records = []
    t0 = time.time()
    for style in styles:
        for level in REBUILD_LEVELS:
            rng = random.Random("{0}/{1}/{2}".format(SEED, style, level))
            out, stats = build_one(style, level, rng)
            if out is None:
                print("[hpose] {0}/{1} FAILED: {2}".format(style, level, stats))
                continue
            records.append(dict(usd=os.path.abspath(out), **stats))
            print("[hpose] {0:12s} {1:18s} -> {2}".format(
                style, level, os.path.basename(out)))
    man = os.path.join(ARCH_DIR, "hurricane_house_pose_manifest.json")
    with open(man, "w") as fh:
        json.dump({"seed": SEED, "records": records}, fh, indent=1)
    print("[hpose] built {0} file(s) in {1:.1f}s -> {2}"
          .format(len(records), time.time() - t0, man))
    return 0 if records else 1


def cmd_verify():
    styles = _styles()
    total_floating = total_below = total_unbound_floor = 0
    n_files = 0
    for style in styles:
        for level in REBUILD_LEVELS:
            path = os.path.join(ARCH_DIR, "house_{0}_{1}{2}.usd".format(
                style, level, SUFFIX))
            if not os.path.exists(path):
                continue
            n_files += 1
            report = audit_file(path, style)
            if "error" in report:
                print("[hpose][verify] {0}: {1}".format(path, report["error"]))
                continue
            tag = "{0}/{1}".format(style, level)
            for cat, c in sorted(report["per_category"].items()):
                print("  {0:28s} {1:22s} n={2:3d} z=[{3:7.3f}, {4:7.3f}]"
                      .format(tag, cat, c["n"], c["min_z"], c["max_z"]))
            if report["floating"]:
                total_floating += len(report["floating"])
                print("  {0:28s} !! FLOATING: {1}".format(tag, report["floating"]))
            if report["below_slab"]:
                total_below += len(report["below_slab"])
                print("  {0:28s} !! BELOW SLAB: {1}".format(tag, report["below_slab"]))
            if report["unbound_floor"]:
                total_unbound_floor += len(report["unbound_floor"])
                print("  {0:28s} !! UNBOUND FLOOR: {1}".format(
                    tag, report["unbound_floor"]))
    print("[hpose][verify] {0} file(s): {1} floating piece(s), {2} below-slab "
          "piece(s), {3} unbound floor mesh(es)".format(
              n_files, total_floating, total_below, total_unbound_floor))
    return 0 if (total_floating == 0 and total_below == 0
                and total_unbound_floor == 0) else 1


def cmd_report():
    """Audit the CURRENT canonical files (no `_h` suffix) — the baseline
    this tool is trying to replace, for a before/after comparison."""
    styles = _styles()
    for style in styles:
        for level in REBUILD_LEVELS:
            path = os.path.join(ARCH_DIR, "house_{0}_{1}.usd".format(
                style, level))
            if not os.path.exists(path):
                continue
            report = audit_file(path, style)
            if "error" in report:
                print("{0}: {1}".format(path, report["error"]))
                continue
            tag = "{0}/{1}".format(style, level)
            print("{0:28s} floating={1} below_slab={2} unbound_floor={3}"
                  .format(tag, len(report["floating"]),
                          len(report["below_slab"]),
                          len(report["unbound_floor"])))


def cmd_promote():
    """Swap `_h` files to canonical names, keeping the old canonical file as
    `_tornado` — only call this once `--verify` is clean."""
    styles = _styles()
    n = 0
    for style in styles:
        for level in REBUILD_LEVELS:
            new = os.path.join(ARCH_DIR, "house_{0}_{1}{2}.usd".format(
                style, level, SUFFIX))
            canon = os.path.join(ARCH_DIR, "house_{0}_{1}.usd".format(
                style, level))
            backup = os.path.join(ARCH_DIR, "house_{0}_{1}{2}.usd".format(
                style, level, BACKUP_SUFFIX))
            if not os.path.exists(new):
                print("[hpose][promote] missing {0}, skipping".format(new))
                continue
            if os.path.exists(canon) and not os.path.exists(backup):
                shutil.copy2(canon, backup)
            shutil.copy2(new, canon)
            n += 1
            print("[hpose][promote] {0} -> {1} (old kept as {2})".format(
                os.path.basename(new), os.path.basename(canon),
                os.path.basename(backup)))
    print("[hpose][promote] {0} level(s) promoted".format(n))
    return 0


# ---------------------------------------------------------------------------
# STREAM Q (2026-08-31) -- adopt the TORNADO fracture+settle library as the
# canonical `_POSE_LEVELS` (roof_collapsed / partial_collapse / leveled),
# repairing its two known defects (frozen mid-air fragments; floor material)
# first. See `hurricane_flow.py`'s "TOP RUNGS, NO PHYSICS" section and the
# coordinator's own brief: the pose-authored versions (a rigid squash+tilt,
# no fracture) read as clean/rigid -- "the roof is a WHOLE flattened plate
# dropped into the shell", "every rafter present" -- while the user asked
# for the tornado's own ragged, torn-edge material language. The pose files
# are KEPT (as `_pose.usd`) for an A/B, never deleted.
#
# WHY REPAIR RATHER THAN RE-BAKE. The `_tornado.usd` files are the ORIGINAL
# tornado-disaster fracture+settle bake output -- there is no offline way to
# reproduce the PhysX settle itself (it needs a live Kit + GPU), so the only
# offline-safe move is to repair the geometry that already exists, exactly
# the same "existing library can be repaired in place instead of re-baked"
# argument `bake.reseat_meshes_in_file`'s own docstring makes.
POSE_BACKUP_SUFFIX = "_pose"   # what the CURRENT (pose-authored) canonical
                               # is renamed to before it is overwritten


def _pristine_wall_count(style):
    """How many `house_wall` meshes the style's UNDAMAGED pristine file
    carries -- the "shell stands" yardstick: `roof_collapsed` should retain
    MOST of these as whole `house_wall` meshes (not fractured into `frag_*`)
    or it is shipping a debris fan, not a roof failure with the walls
    intact."""
    src = os.path.join(ARCH_DIR, "house_{0}_pristine.usd".format(style))
    stage = Usd.Stage.Open(src)
    if not stage:
        return 0
    house = stage.GetDefaultPrim()
    return sum(1 for p in Usd.PrimRange(house, Usd.PrimAllPrimsPredicate)
              if p != house and p.IsA(UsdGeom.Mesh)
              and hf._category_of(p) == "house_wall")


def _pristine_ridge_peak(style):
    """The highest world Z any `house_roof`/`house_bay_roof` mesh reaches in
    the style's UNDAMAGED pristine file -- the actual ridge PEAK (unlike
    `_pristine_references`'s `roof_rest_z`, which is the WALL-top/eave
    height an intact roof rests ON). `partial_collapse`'s repaired canonical
    must never show a fragment above this line -- a piece higher than the
    house ever stood is a settle that never finished, not damage."""
    src = os.path.join(ARCH_DIR, "house_{0}_pristine.usd".format(style))
    stage = Usd.Stage.Open(src)
    if not stage:
        return None
    house = stage.GetDefaultPrim()
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    peak = None
    for prim in Usd.PrimRange(house, Usd.PrimAllPrimsPredicate):
        if prim == house or not prim.IsA(UsdGeom.Mesh):
            continue
        if hf._category_of(prim) not in ("house_roof", "house_bay_roof"):
            continue
        wb = bk.world_point_bounds(prim, xcache)
        if wb is None:
            continue
        hz = float(wb[1][2])
        peak = hz if peak is None else max(peak, hz)
    return peak


def _floor_material_report(path):
    """The `house_floor*` SLICE of `hurricane_house_floor_fix.validate_
    bindings` -- Mesh AND Subset prims under a `house_floor` category ONLY.

    THE FULL `validate_bindings` OUTPUT IS THE WRONG CHECK HERE. It also
    flags every `house_wall`/`house_roof`/`house_bay_roof`/`house_door`
    MESH as "unbound", because this kit binds THOSE categories only at
    GeomSubset level and never at mesh level -- each subset (Section0,
    Section1, ...) carries its own direct binding and together they cover
    every face, so the mesh itself correctly resolves to no material of its
    own. Measured: this pattern is UNIVERSAL, present on 272 of 296 files in
    the whole archetype library, INCLUDING every style's own `_pristine.usd`
    (`house_wide_house_pristine.usd`'s wall/roof/door meshes all show it) --
    which renders correctly and has never been reported broken, so it is
    not a defect and must not be reported as one by a check this file adds
    on top of the shared tool.
    """
    from pxr import UsdShade

    stage = Usd.Stage.Open(path)
    unbound, unreal = [], []
    if not stage:
        return dict(unbound=unbound, unreal_material=unreal)
    for prim in stage.Traverse():
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Subset)):
            continue
        # A Subset's own category is its PARENT mesh's -- `_category_of`
        # only makes sense applied to the mesh, never to a subset name like
        # "Section0".
        cat_prim = prim.GetParent() if prim.IsA(UsdGeom.Subset) else prim
        if hf._category_of(cat_prim) != "house_floor":
            continue
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[:2]
        if mat is None or not mat.GetPrim().IsValid():
            unbound.append(prim.GetPath().pathString)
        elif mat.GetPrim().GetName() == "UnrealMaterial":
            unreal.append(prim.GetPath().pathString)
    return dict(unbound=unbound, unreal_material=unreal)


def _audit_tornado_canon(path, style, level, air_tol):
    """One canonical `_POSE_LEVELS` file's full repair/verify report:
    per-category z ranges, FLOOR-ONLY material bindings
    (`_floor_material_report` -- see its own docstring for why the library-
    wide `validate_bindings` cannot be used unfiltered here), wall
    retention against the pristine style, and (for `partial_collapse`) the
    ridge-peak comparison. Does NOT reseat -- call this AFTER a repair pass
    to confirm it, or on its own as a pure audit."""
    report = audit_file(path, style)
    floor_rep = _floor_material_report(path)
    pristine_walls = _pristine_wall_count(style)
    walls_now = report["per_category"].get("house_wall", {}).get("n", 0)
    overall_max_z = max((c["max_z"] for c in report["per_category"].values()),
                        default=None)
    ridge_peak = _pristine_ridge_peak(style) if level == "partial_collapse" \
        else None
    over_ridge = (ridge_peak is not None and overall_max_z is not None
                 and overall_max_z > ridge_peak + air_tol)
    remaining = bk.reseat_meshes_in_file(path, air_tol=air_tol,
                                        dry_run=True, verbose=False)
    return dict(
        path=path, style=style, level=level,
        per_category={k: dict(v) for k, v in report["per_category"].items()},
        pristine_walls=pristine_walls, walls_retained=walls_now,
        overall_max_z=overall_max_z, ridge_peak=ridge_peak,
        over_ridge=over_ridge, below_slab=len(report["below_slab"]),
        floor_unbound=len(floor_rep.get("unbound", [])),
        floor_unreal_material=len(floor_rep.get("unreal_material", [])),
        still_floating=len(remaining))


def cmd_adopt_tornado():
    """Swap every `_POSE_LEVELS` canonical file (`roof_collapsed`/
    `partial_collapse`/`leveled`) to the TORNADO fracture+settle output,
    repairing the frozen-mid-air-fragment defect (`bake.
    reseat_meshes_in_file`, `air_tol=0.05` -- tighter than the module
    default 0.10, matching the coordinator's own 5 cm brief) before it ever
    becomes canonical.

    For each style x level:
      1. back up the CURRENT canonical to `house_<style>_<level>_pose.usd`
         (skipped if that backup already exists -- idempotent, never
         clobbers a hand-reviewed `_pose.usd` on a re-run).
      2. copy `house_<style>_<level>_tornado.usd` over the canonical name.
      3. reseat, then re-audit the result (`_audit_tornado_canon`).

    Writes `hurricane_house_tornado_canon_manifest.json`. Returns 1 (but
    still writes every file — this is a repair pass, not a dry run) if any
    style/level ends up with a below-slab piece, an unbound/UnrealMaterial
    floor, a fragment still floating past `air_tol` after the reseat, or
    (`partial_collapse` only) a point above the pristine ridge.
    """
    styles = _styles()
    air_tol = 0.05
    records = []
    n_bad = 0
    for style in styles:
        for level in hf._POSE_LEVELS:
            canon = os.path.join(ARCH_DIR, "house_{0}_{1}.usd".format(
                style, level))
            tornado = os.path.join(ARCH_DIR, "house_{0}_{1}_tornado.usd"
                                  .format(style, level))
            pose_backup = os.path.join(ARCH_DIR, "house_{0}_{1}{2}.usd"
                                      .format(style, level, POSE_BACKUP_SUFFIX))
            if not os.path.exists(tornado):
                print("[hpose][adopt-tornado] missing {0}, skipping"
                      .format(tornado))
                continue
            if os.path.exists(canon) and not os.path.exists(pose_backup):
                shutil.copy2(canon, pose_backup)
            shutil.copy2(tornado, canon)

            # ITERATE TO CONVERGENCE, not one pass. `reseat_meshes_in_file`
            # sorts bottom-up and reads a candidate SUPPORT's height as
            # whatever it is at the time (already-corrected if its own z0 is
            # lower and so processed earlier THIS pass, still the ORIGINAL
            # height if not) -- a piece resting on another piece that is
            # ITSELF corrected later in dependency order can still be left
            # floating after exactly one pass. Measured: 9 of 24 files had a
            # nonzero `still_floating` after a single pass; re-running the
            # (idempotent -- a settled pile finds nothing left to move)
            # function until it moves nothing converges every one of them
            # (see the manifest's `passes` field).
            total_moved = 0
            max_dz = 0.0
            passes = 0
            for passes in range(1, 6):
                moved = bk.reseat_meshes_in_file(canon, air_tol=air_tol,
                                                verbose=False)
                if not moved:
                    break
                total_moved += len(moved)
                max_dz = max(max_dz, max(abs(dz) for _, dz in moved))
            rep = _audit_tornado_canon(canon, style, level, air_tol)
            rep.update(reseated=total_moved, max_dz=round(max_dz, 4),
                      passes=passes, pose_backup=os.path.abspath(pose_backup))
            records.append(rep)

            bad = (rep["below_slab"] or rep["floor_unbound"]
                  or rep["floor_unreal_material"] or rep["still_floating"]
                  or rep["over_ridge"])
            n_bad += 1 if bad else 0
            print("[hpose][adopt-tornado] {0:12s} {1:18s} reseated={2:3d} "
                  "({3} pass) max_dz={4:+.3f} walls={5}/{6} "
                  "still_floating={7} below_slab={8} floor_bad={9} "
                  "over_ridge={10}{11}".format(
                      style, level, total_moved, passes, max_dz,
                      rep["walls_retained"], rep["pristine_walls"],
                      rep["still_floating"], rep["below_slab"],
                      rep["floor_unbound"] + rep["floor_unreal_material"],
                      rep["over_ridge"], "  !!" if bad else ""))

    man = os.path.join(ARCH_DIR, "hurricane_house_tornado_canon_manifest.json")
    with open(man, "w") as fh:
        json.dump({"air_tol": air_tol, "records": records}, fh, indent=1)
    print("[hpose][adopt-tornado] {0} file(s) adopted, {1} with an open "
          "issue -> {2}".format(len(records), n_bad, man))
    return 0 if n_bad == 0 else 1


def cmd_verify_tornado():
    """Pure audit of whatever is CURRENTLY canonical for `_POSE_LEVELS`
    (does not copy or reseat anything) -- the re-check to run after
    `--adopt-tornado`, or any time later, without re-running the adoption."""
    styles = _styles()
    air_tol = 0.05
    n_bad = 0
    n_files = 0
    for style in styles:
        for level in hf._POSE_LEVELS:
            canon = os.path.join(ARCH_DIR, "house_{0}_{1}.usd".format(
                style, level))
            if not os.path.exists(canon):
                continue
            n_files += 1
            rep = _audit_tornado_canon(canon, style, level, air_tol)
            bad = (rep["below_slab"] or rep["floor_unbound"]
                  or rep["floor_unreal_material"] or rep["still_floating"]
                  or rep["over_ridge"])
            n_bad += 1 if bad else 0
            print("[hpose][verify-tornado] {0:12s} {1:18s} walls={2}/{3} "
                  "still_floating={4} below_slab={5} floor_bad={6} "
                  "over_ridge={7}{8}".format(
                      style, level, rep["walls_retained"],
                      rep["pristine_walls"], rep["still_floating"],
                      rep["below_slab"],
                      rep["floor_unbound"] + rep["floor_unreal_material"],
                      rep["over_ridge"], "  !!" if bad else ""))
    print("[hpose][verify-tornado] {0} file(s), {1} with an open issue"
          .format(n_files, n_bad))
    return 0 if n_bad == 0 else 1


# ---------------------------------------------------------------------------
# JOB B -- cardinal bay-drop variants for `cover_lost` / `deck_panels_lost`
# ---------------------------------------------------------------------------
def cmd_build_variants():
    """Build `house_<style>_<level>_{n,e,s,w}_h.usd` for every style and
    every level in `hf.LEVELS_WITH_VARIANTS` -- see `hurricane_flow`'s "JOB
    B" section and `build_one`'s `variant` argument."""
    styles = _styles()
    if not styles:
        print("[hpose] no house_*_pristine.usd found under {0}".format(ARCH_DIR))
        return 1
    levels = hf.LEVELS_WITH_VARIANTS
    print("[hpose] building {0} variant(s) x {1} level(s) x {2} style(s), "
          "seed={3}".format(len(hf.VARIANTS), len(levels), len(styles), SEED))
    records = []
    t0 = time.time()
    for style in styles:
        for level in levels:
            for side in hf.VARIANTS:
                rng = random.Random("{0}/{1}/{2}/{3}".format(
                    SEED, style, level, side))
                out, stats = build_one(style, level, rng, variant=side)
                if out is None:
                    print("[hpose] {0}/{1}/{2} FAILED: {3}".format(
                        style, level, side, stats))
                    continue
                records.append(dict(usd=os.path.abspath(out), **stats))
                print("[hpose] {0:12s} {1:18s} {2} -> {3}".format(
                    style, level, side, os.path.basename(out)))
    man = os.path.join(ARCH_DIR, "hurricane_house_variant_manifest.json")
    with open(man, "w") as fh:
        json.dump({"seed": SEED, "records": records}, fh, indent=1)
    print("[hpose] built {0} variant file(s) in {1:.1f}s -> {2}"
          .format(len(records), time.time() - t0, man))
    return 0 if records else 1


def cmd_verify_variants():
    styles = _styles()
    total_floating = total_below = total_unbound_floor = 0
    n_files = 0
    for style in styles:
        for level in hf.LEVELS_WITH_VARIANTS:
            for side in hf.VARIANTS:
                path = os.path.join(ARCH_DIR, "house_{0}_{1}_{2}{3}.usd".format(
                    style, level, side, SUFFIX))
                if not os.path.exists(path):
                    continue
                n_files += 1
                report = audit_file(path, style)
                if "error" in report:
                    print("[hpose][verify] {0}: {1}".format(path, report["error"]))
                    continue
                tag = "{0}/{1}/{2}".format(style, level, side)
                if report["floating"]:
                    total_floating += len(report["floating"])
                    print("  {0:32s} !! FLOATING: {1}".format(tag, report["floating"]))
                if report["below_slab"]:
                    total_below += len(report["below_slab"])
                    print("  {0:32s} !! BELOW SLAB: {1}".format(tag, report["below_slab"]))
                if report["unbound_floor"]:
                    total_unbound_floor += len(report["unbound_floor"])
                    print("  {0:32s} !! UNBOUND FLOOR: {1}".format(
                        tag, report["unbound_floor"]))
    print("[hpose][verify-variants] {0} file(s): {1} floating, {2} below-slab, "
          "{3} unbound floor".format(n_files, total_floating, total_below,
                                     total_unbound_floor))
    return 0 if (total_floating == 0 and total_below == 0
                and total_unbound_floor == 0) else 1


def cmd_promote_variants():
    """Copy each `house_<style>_<level>_{side}_h.usd` to its final
    `house_<style>_<level>_{side}.usd` name (brand-new filenames, no
    `_tornado`-style backup needed), and additionally point the UNSUFFIXED
    canonical `house_<style>_<level>.usd` at the "n" variant's content --
    JOB B's "keep the un-suffixed file equal to the n variant" -- backing up
    whatever canonical file is there first, same safety net `cmd_promote`
    already uses (skipped if a `_tornado` backup already exists from an
    earlier promote, so the TRUE original tornado-library file is never
    overwritten twice)."""
    styles = _styles()
    n = 0
    for style in styles:
        for level in hf.LEVELS_WITH_VARIANTS:
            for side in hf.VARIANTS:
                new = os.path.join(ARCH_DIR, "house_{0}_{1}_{2}{3}.usd".format(
                    style, level, side, SUFFIX))
                final = os.path.join(ARCH_DIR, "house_{0}_{1}_{2}.usd".format(
                    style, level, side))
                if not os.path.exists(new):
                    print("[hpose][promote-variants] missing {0}, skipping"
                          .format(new))
                    continue
                shutil.copy2(new, final)
                n += 1
                if side == "n":
                    canon = os.path.join(ARCH_DIR, "house_{0}_{1}.usd".format(
                        style, level))
                    backup = os.path.join(ARCH_DIR, "house_{0}_{1}{2}.usd".format(
                        style, level, BACKUP_SUFFIX))
                    if os.path.exists(canon) and not os.path.exists(backup):
                        shutil.copy2(canon, backup)
                    shutil.copy2(final, canon)
                print("[hpose][promote-variants] {0} -> {1}{2}".format(
                    os.path.basename(new), os.path.basename(final),
                    " (also -> {0})".format(
                        os.path.basename(canon)) if side == "n" else ""))
    print("[hpose][promote-variants] {0} variant file(s) promoted".format(n))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", action="store_true")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--promote", action="store_true")
    ap.add_argument("--build-variants", action="store_true")
    ap.add_argument("--verify-variants", action="store_true")
    ap.add_argument("--promote-variants", action="store_true")
    ap.add_argument("--adopt-tornado", action="store_true",
                    help="swap roof_collapsed/partial_collapse/leveled "
                         "canon to the tornado fracture+settle output, "
                         "backing up the current pose-authored canon as "
                         "_pose.usd and reseating frozen fragments")
    ap.add_argument("--verify-tornado", action="store_true",
                    help="audit the current roof_collapsed/partial_"
                         "collapse/leveled canon (no changes written)")
    args = ap.parse_args()
    rc = 0
    if args.report:
        cmd_report()
    if args.build:
        rc = cmd_build() or rc
    if args.verify:
        rc = cmd_verify() or rc
    if args.promote:
        rc = cmd_promote() or rc
    if args.build_variants:
        rc = cmd_build_variants() or rc
    if args.verify_variants:
        rc = cmd_verify_variants() or rc
    if args.promote_variants:
        rc = cmd_promote_variants() or rc
    if args.adopt_tornado:
        rc = cmd_adopt_tornado() or rc
    if args.verify_tornado:
        rc = cmd_verify_tornado() or rc
    if not (args.report or args.build or args.verify or args.promote
           or args.build_variants or args.verify_variants
           or args.promote_variants or args.adopt_tornado
           or args.verify_tornado):
        ap.print_help()
    return rc


if __name__ == "__main__":
    sys.exit(main())
