#!/usr/bin/env python
"""
quake_gac_bake — the earthquake sibling of `fire_bake_launch_script.py`: ONE
earthquake-damaged GreatAmericanCity / downtowncity building per Kit process,
settled alone and exported static.

    ISAAC_SIM_HEADLESS=true QB_NAME=SM_Building_02 QB_GRADE=DG5 QB_SEED=7 \
    QB_OUT=/isaac-sim/AirStack/scene_gen/assets/gac_quake SETTLE_STEPS=2400 \
    PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
    /isaac-sim/python.sh \
    /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/quake_gac_bake_launch_script.py \
    --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window

Driven one building at a time by `scene_gen/tools/quake_gac_bake.sh`.

WHY THIS EXISTS
----------------
`disaster/quake_sliced.py` (`plan_damage` / `apply_plan` / `wreck_sliced`) is
the earthquake ladder for a SLICED whole-asset building — the same
`gac_storey_slice.slice_to_kit` cut `disaster/gac_fire.py`'s fire ladder
damages — and it already goes all the way to full collapse (`masonry_collapse`,
`pancake`, the foundation family `SETTLE`/`TILT`/`OV`). But as of 2026-08-30
(`slice-buildings-into-kits` skill, "As of 2026-08-30 no launch script wires
`slice_to_kit` and `wreck_sliced` together") nothing had ever run the two
halves back to back inside Kit — only `tools/_gac_region_probe.py` (the slice
alone) and `tests/test_quake_sliced.py` (the ladder alone, on synthetic
placements) exercised either half. This script is that wiring, built on the
same one-building-per-process shape `fire_bake_launch_script.py` proved out
for the fire ladder (bounded memory, a settle that gets the whole step
budget, a fast static assembly load).

STAGE LIST — WHAT IS REUSED VERBATIM FROM `fire_bake_launch_script.py`, AND
WHAT IS NEW (see the `slice-buildings-into-kits` skill's "Baking and
damaging a GAC building" section for the canonical 10-stage list this
follows)
------------------------------------------------------------------------
 1. fresh stage, `/World/bake/<tag>` cell at the origin           REUSED
    (`fire_bake.BAKE_ROOT`/`fire_bake.DEFAULT_PRIM`, `fb.bake_tag`,
    the exact `main()` preamble — new stage, mpu=1, up=Z, `/World`
    defaultPrim, `BAKE_ROOT` Xform, `CELL` Xform).
 2. place + measure + slice                                       NEW SHAPE,
    OLD PARTS. `gac_fire.prepare` does this AND plans a fire AND bakes a
    soot skin — this disaster needs none of that, so this script calls only
    the disaster-agnostic pieces `prepare` itself is built from
    (`gac_fire.split_kind`/`PACKS`/`asset_url`/`asset_scale`/`place_source`,
    all REUSED verbatim, read-only) and skips `window_rects`/`mass_from_grid`/
    `bake_atlases`/`urban_fire.plan_fire` entirely — the sliced element table
    `quake_flow.describe` builds AFTER the slice is everything the ladder
    needs; nothing here has to pre-measure a mass box the way a fire plan
    does. `gac_storey_slice.slice_to_kit` itself is called with
    **`region=None` for EVERY grade, not just DG4/DG5/foundation.** The
    `slice-buildings-into-kits` skill's own "PLAN the damage" section is
    explicit about why: "An earthquake shakes the WHOLE building at once:
    `quake_sliced.wreck_sliced` calls `quake_flow.describe(...)` over every
    placement it is handed... there is no 'band' for a region cut to save."
    Checked against every DG1-DG3 recipe in `LADDER_S` (`glass_loss`,
    `parapet_fall`, `infill_fail`, `corner_fail`, `out_of_plane`,
    `soft_storey`) — none of them is confined to a fire-style origin/top
    band; each draws its own sides/storeys independently, so a region cut
    would arbitrarily hide part of the building from a recipe that might
    need it. `region=None` also GUARANTEES the roof/parapet band is ringed
    in full (`slice_to_kit`'s own docstring), which sidesteps the fire
    ladder's `roof_needed` dance entirely — there is no grade in `LADDER_S`
    that can run without the option of a roof piece (parapet_fall,
    corner_fail and the foundation family all can reach it).
 3. `quake_sliced.wreck_sliced(..., btype=None, usd=<bare asset name>)` NEW.
    `plan_damage` + `apply_plan` in one call — glass, removal, rigid
    displacement, the fit-out, rubble-v2 piles, the foundation family. The
    construction type is resolved from `quake_sliced.CONSTRUCTION`'s
    measured table (all three of the task's buildings — SM_Building_02/12/23
    — are in it), which is also why this script does not bother
    pre-measuring a mass box just to pick a `family` for `register_style`:
    `quake_sliced.construction_type(asset)` (no `H`) resolves every named
    asset from the table alone, and `wreck_sliced` re-resolves it AGAIN
    after the slice with the real measured `H` for the ladder grade itself
    — the two calls can only disagree for an asset the table has never
    heard of, and even then the STRUCTURE cut (masonry piers vs a steel
    frame) is the only thing that would be mildly off; the damage grade
    applied is always the post-slice, correctly-measured one.
 4. settle-alone, `fire_bake_launch_script`'s EXACT `settle.run(...)` call   REUSED
    VERBATIM (`SETTLE_STEPS` default 2400, `kick=0.10`, `bake_result`,
    `density=1600.0`, `max_speed=6.0`, `converge=True`,
    `max_steps=SETTLE_STEPS*3`, `quiet_steps=SETTLE_QUIET`, `ccd=True`,
    `ground_plane_z=floor_z=0.0`, `decompose_larger_than=SETTLE_DECOMP_M`).
    `SETTLE_FABRIC` is NOT passed as a kwarg here, on purpose — `settle.run`
    reads it from `os.environ` itself (`disaster/settle.py`), so leaving it
    out is what "respected via env passthrough" means: the driver exports
    the var, this script does not have an opinion about it.
 5. deactivate still-movers + `fire_bake.deactivate_airborne`     REUSED
    VERBATIM (the exact block `fire_bake_launch_script.main()` runs after
    its own settle — disaster-agnostic real-geometry ray sweep, no fire
    semantics in it at all).
 6. `fire_bake.rehome_for_export` + drop `<cell>/src`             REUSED
    VERBATIM. The material trap (`slice_to_kit` binds every piece to a
    material living inside the merged source's own subtree) is a property
    of the SLICER, not of the fire ladder — `rehome_for_export`'s own
    docstring says as much ("works for a live slice, a baked kit and a kit
    building alike"). A quake building's rubble/mound materials never touch
    `<cell>/src` at all (they come from `quake_flow.materials`'s own
    `QuakeLooks` scope, or from external rubble asset references under
    `RUBBLE_ASSET_ROOT`), so `rehome_for_export`'s work here is exactly the
    same as fire's: rehome every subset still bound to the source, then it
    is safe to drop.
 7. bbox / top_z of the settled cell                              REUSED
    (a straight copy of `fire_bake_launch_script._bbox` — trivial, and a
    launch script cannot be imported for its helpers since constructing
    `SimulationApp` is a side effect of import).
 8. `fire_bake.strip_physics`                                     REUSED VERBATIM.
 9. root-layer export to `<QB_OUT>/gac_<name>_<grade>_s<seed>.usd` REUSED
    MACHINERY, NEW ENTRY SHAPE. `fire_bake.out_stem`/`out_paths`/`bake_tag`
    are pure string formatting over an `entry` dict (`kind`/`name`/`level`/
    `seed`/`origin`/`sides`/`index`) with no fire semantics baked into the
    format itself — an entry with `kind="gac"`, `level=<grade>`,
    `origin=None`, `sides=None` produces exactly `gac_<name>_<grade>_s<seed>`,
    which is the filename the brief asks for, so this script builds that
    entry dict and calls `fire_bake.out_stem`/`out_paths` rather than
    re-deriving the format.
10. sidecar with a NEW `"quake"` schema field (`fire_bake.SCHEMA`/`.sidecar`
    is fire's own field set — `fire`/`masses`/`events`/`seats` — and the
    `slice-buildings-into-kits` skill is explicit that a non-fire disaster
    needs its own sibling rather than overloading those names). See
    `quake_sidecar()` below.
11. `fire_bake.verify_export`                                     REUSED VERBATIM
    (four checks: textures resolve, nothing points into the dropped source,
    no physics, no Flow — none of it is fire-specific).
12. `merge_manifest` into `<QB_OUT>/gac_quake.json` under a file lock,
    keyed by `(name, grade)`                                       NEW,
    copied from `bake_quake_archetypes_launch_script.merge_manifest`'s
    read-merge-write-under-`fcntl.flock` shape (that function cannot be
    imported either — same `SimulationApp`-on-import problem — so this is a
    deliberate small duplication, not a drift risk: both are five lines of
    JSON bookkeeping).

WHAT NEVER HAPPENS HERE (same reasons `fire_bake_launch_script.py` gives)
--------------------------------------------------------------------------
No `fracture.fracture_prim` on a sliced piece, ever — `quake_sliced`'s own
module docstring: VTK segfaults on a clipped shell. Every collapse in
`LADDER_S` is REMOVAL + RIGID DISPLACEMENT, never fracture, by construction.
No physics in the export (`strip_physics`). No source subtree in the export
unless a material could not be rehomed (`src_kept`, same contract as fire's).

Env:
    QB_NAME         merged asset name (`SM_Building_02`) — a `"dtc:Name"`
                    prefix routes to the downtowncity pack exactly the way
                    `gac_fire.split_kind`/`FB_NAME` already does; a bare
                    name is GreatAmericanCity
    QB_GRADE        `quake_sliced.LEVELS` (DG0..DG5, SETTLE, TILT, OV) —
                    DG0 is the pristine no-op export (nothing to damage);
                    the task's own rows are DG3/DG4/DG5
    QB_SEED         this building's rng seed (the driver gives building i
                    `base + 31*i`, matching `fire_bake.sh`'s own column
                    seed convention)
    QB_INDEX        this building's column index — only used for the prim
                    tag and the manifest record (default 0)
    QB_OUT          output directory (default
                    /isaac-sim/AirStack/scene_gen/assets/gac_quake) — INSIDE
                    the repo, unlike fire's container-only cache: these
                    bakes are meant to ship as dataset content, the same
                    convention `bake_quake_archetypes_launch_script.py`
                    uses for `scene_gen/assets/archetype`
    QB_VERIFY       1 (default) reopens the export cold and checks it
    KEEP_PHYSICS    1 leaves the bodies live and skips the physics strip
    KEEP_OPEN       1 keeps the app up after the export
    SETTLE_STEPS    physics step target (default 2400, same as fire_bake)
    SETTLE_QUIET    quiet-phase steps (default 400)
    SETTLE_DECOMP_M convex-decomposition threshold, metres (default 0.8)
    SETTLE_FABRIC   read directly by `disaster/settle.py`; not touched here

Banner: `QUAKE GAC BAKE DONE` (or `QUAKE GAC BAKE FAILED`).
"""

import json
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    """The container exports every launcher knob as an EMPTY STRING, so
    `os.environ.get(name, default)` never reaches its default — same helper
    as `fire_bake_launch_script.py`'s, copied rather than imported (a launch
    script constructs `SimulationApp` at import time, so it cannot be
    imported for its utilities)."""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "true").lower() in ("1", "true", "yes")
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

simulation_app = SimulationApp(launch_config={"headless": _HEADLESS,
                                              "extra_args": KIT_ARGS})

import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Sdf, Usd, UsdGeom                              # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import numpy as np                                             # noqa: E402
from detail import gac_slice as gsl                            # noqa: E402
from detail import gac_storey_slice as gss                     # noqa: E402
from disaster import fire_bake as fb                           # noqa: E402
from disaster import fracture, settle                          # noqa: E402
from disaster import gac_fire as gcf                           # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import quake_sliced as qs                        # noqa: E402

NAME = _env("QB_NAME", "SM_Building_02")
GRADE = _env("QB_GRADE", "DG3")
SEED = int(_env("QB_SEED", "7"))
INDEX = int(_env("QB_INDEX", "0"))
OUT_DIR = _env("QB_OUT", "/isaac-sim/AirStack/scene_gen/assets/gac_quake")
VERIFY = _env("QB_VERIFY", "1") not in ("0", "false", "no")
KEEP_PHYSICS = _env("KEEP_PHYSICS", "0") not in ("0", "false")
SETTLE_STEPS = int(_env("SETTLE_STEPS", "2400"))
SETTLE_QUIET = int(_env("SETTLE_QUIET", "400"))
SETTLE_DECOMP_M = float(_env("SETTLE_DECOMP_M", "0.8"))

#: sidecar schema — a SIBLING of `fire_bake.SCHEMA`, not an overload of it
#: (`fire_bake.sidecar`'s own field names are fire's: `fire`/`masses`/
#: `events`/`seats`). Bump this if the `"quake"` field's shape changes
#: meaning.
QUAKE_SCHEMA = 1

# `fire_bake.out_stem`/`out_paths`/`bake_tag` are pure formatting over this
# shape (`kind`/`name`/`level`/`seed`/`origin`/`sides`/`index`) with no fire
# semantics in the format itself: `origin`/`sides` are OMITTED whenever they
# are `None` (every entry here), which is exactly what turns
# `out_stem(ENTRY)` into `gac_<name>_<grade>_s<seed>` — the filename this
# brief asks for. `kind` is fixed to `"gac"` regardless of whether `NAME`
# carries a `dtc:` prefix: it names this BAKE FAMILY (a sliced whole-asset
# earthquake bake), not the source pack, matching the literal filename the
# task specifies.
ENTRY = {"kind": "gac", "name": NAME, "level": GRADE, "seed": SEED,
         "origin": None, "sides": None, "index": INDEX}
TAG = fb.bake_tag(ENTRY)
CELL = "{0}/{1}".format(fb.BAKE_ROOT, TAG)


def _bbox(stage, path):
    """World-aligned bbox of `path`, or None. A byte-for-byte copy of
    `fire_bake_launch_script._bbox` — disaster-agnostic, and a launch script
    cannot be imported for its helpers (constructing `SimulationApp` is a
    side effect of import)."""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    lo, hi = r.GetMin(), r.GetMax()
    return [float(lo[0]), float(lo[1]), float(lo[2]),
            float(hi[0]), float(hi[1]), float(hi[2])]


def _rubble_fields(ctx):
    """The manifest/sidecar summary of every rubble-v2 pile this bake
    authored — a copy of `bake_quake_archetypes_launch_script._rubble_fields`
    (that function reads `res.get("rubble")`, i.e. `ctx["rubble"]`; this
    reads the same key off the sliced ctx, which is now populated the same
    way — see the `quake_sliced._author_pile` change this launcher depends
    on). Empty when the grade authored no pile (DG1-DG3, the foundation
    family)."""
    piles = ctx.get("rubble") or []
    sides, reach, extent, crown = [], {}, {}, 0.0
    for st_ in piles:
        for sd in (st_.get("fall_sides") or []):
            if sd not in sides:
                sides.append(sd)
        for sd, r in (st_.get("reach_m") or {}).items():
            try:
                reach[sd] = max(float(reach.get(sd, 0.0)), float(r))
            except (TypeError, ValueError):
                continue
        for sd, r in (st_.get("extent_m") or {}).items():
            try:
                extent[sd] = max(float(extent.get(sd, 0.0)), float(r))
            except (TypeError, ValueError):
                continue
        try:
            crown = max(crown, float(st_.get("crown_m") or 0.0))
        except (TypeError, ValueError):
            pass
    if not piles:
        return {}
    out = dict(fall_sides=sides, reach_m=reach, crown_m=round(crown, 2),
               n_piles=len(piles))
    if extent:
        out["extent_m"] = extent
    return out


def quake_sidecar(entry, ctx, bbox, top_z, timings, counts, settle_info=None,
                  usd="", src_kept=False, extra=None):
    """The `.json` that travels beside a baked earthquake-damaged `.usd`.

    A SIBLING of `fire_bake.sidecar`, not an extension of it — the
    `slice-buildings-into-kits` skill's own "What the sidecar must carry for
    a non-fire disaster" section: no Flow-equivalent effect ships with this
    disaster (geometry alone is the whole of the damage), so this carries
    only enough provenance to identify the bake plus the settled `bbox`/
    `top_z` and the damage plan itself — `quake_sliced.plan_to_json(ctx
    ["plan"])`, the json-safe serialiser `plan_damage`'s own docstring
    ("JSON-serialisable throughout") makes trivial.
    """
    plan = ctx.get("plan")
    rf = _rubble_fields(ctx)
    sliced = ctx.get("sliced") or {}
    doc = {
        "schema": QUAKE_SCHEMA,
        "kind": entry["kind"], "name": entry["name"], "level": entry["level"],
        "seed": int(entry["seed"]), "index": int(entry.get("index", 0)),
        "tag": fb.bake_tag(entry),
        "usd": usd,
        "default_prim": fb.DEFAULT_PRIM,
        "root": fb.BAKE_ROOT,
        "cell": "{0}/{1}".format(fb.BAKE_ROOT, fb.bake_tag(entry)),
        "src_kept": bool(src_kept),
        "quake": {
            "grade": entry["level"],
            "btype": sliced.get("btype"),
            "plan": (qs.plan_to_json(plan) if plan is not None else None),
            "fall_sides": rf.get("fall_sides", []),
            "extent_m": rf.get("extent_m", {}),
            "crown_m": rf.get("crown_m", 0.0),
        },
        "bbox": [float(v) for v in bbox] if bbox else None,
        "top_z": (float(top_z) if top_z is not None else None),
        "notes": list(ctx.get("notes") or []),
        "timings": dict(timings or {}),
        "counts": dict(counts or {}),
        "settle": dict(settle_info or {}),
    }
    if extra:
        doc.update(extra)
    return doc


def merge_manifest(path, records):
    """Read-merge-write `<QB_OUT>/gac_quake.json`, keyed by `(name, grade)` —
    a copy of `bake_quake_archetypes_launch_script.merge_manifest`'s shape
    (that function cannot be imported: constructing `SimulationApp` is a
    side effect of importing a launch script)."""
    old = []
    if os.path.exists(path):
        try:
            with open(path) as fh:
                old = json.load(fh)
        except Exception as exc:
            print("[qgac] existing manifest unreadable, replacing it: {0}"
                  .format(exc))
    fresh = set((r.get("name"), r.get("grade")) for r in records)
    kept = [r for r in old if (r.get("name"), r.get("grade")) not in fresh]
    return kept + records


def build(stage, mats, rng, nrng):
    """Place, measure and slice the merged asset, then run the earthquake
    ladder over the sliced placements. Returns `(ctx, doomed, pls)` —
    `doomed` is `[cell/src]`, exactly `build_gac`'s own contract in
    `fire_bake_launch_script.py`, for `fire_bake.rehome_for_export`.
    """
    kind, asset = gcf.split_kind(NAME)
    pack = gcf.PACKS[kind]
    style = pack["style_prefix"] + asset
    url = gcf.asset_url(asset, kind)
    scale = gcf.asset_scale(url, pack["scale"], verbose=True)
    src = gcf.place_source(stage, CELL, url, scale)
    if not src:
        raise RuntimeError("{0}: nothing composed".format(NAME))

    # THE CONSTRUCTION-TYPE GUESS BEFORE THE SLICE. `slice_to_kit`'s own
    # `family` argument decides which structural vocabulary the fit-out uses
    # (masonry piers vs a steel frame — `gac_fire.burn_gac`'s own comment:
    # "THE SYNTHETIC STYLE'S FAMILY FOLLOWS THE CONSTRUCTION TYPE"), so it
    # has to be picked before there is anything sliced to measure a real mass
    # box from. `quake_sliced.CONSTRUCTION` resolves every asset this task
    # names (and the rest of the GreatAmericanCity/downtowncity catalogue)
    # from its own measured table with NO `H` at all, so this script does not
    # duplicate `gac_fire.prepare`'s window/mass measuring just to get one —
    # `wreck_sliced` below re-resolves the same construction type AFTER the
    # slice, off the REAL measured height, for the ladder itself.
    btype_guess = qs.construction_type(asset)
    family = {"urm": "01", "rc": "02", "rc_glass": "05"}.get(btype_guess, "01")
    force_regular = asset in (pack.get("force_regular_grid") or ())

    # `region=None` FOR EVERY GRADE — see the module docstring's stage-2
    # note for why this is not just DG4/DG5/foundation.
    pls, grid, measured = gss.slice_to_kit(
        stage, src, CELL, style, region=None, family=family, verbose=True,
        force_regular=force_regular)

    tag = TAG
    ctx = qs.wreck_sliced(stage, CELL, pls, style, GRADE, rng, nrng, mats,
                          tag, usd=asset, mat_cache={}, verbose=True)
    return ctx, [src], pls


def main():
    t0 = time.time()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path(fb.BAKE_ROOT))
    # THE CELL IS AN XFORM AT THE ORIGIN — same reason `fire_bake_launch_
    # script.py` gives: a bake is always built at the origin and PLACED by
    # whatever assembles it later.
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))

    fracture.ensure_deps()
    fracture.ensure_vtk(verbose=True)

    problems = (qf.check(verbose=False) + qs.check(verbose=False)
                + gsl.check(verbose=False))
    if problems:
        raise RuntimeError("; ".join(problems))
    if GRADE not in qs.LEVELS:
        raise RuntimeError("unknown earthquake grade {0!r} (expected one of "
                           "{1})".format(GRADE, "/".join(qs.LEVELS)))

    print("[qgac] {0} {1} seed {2} -> {3}".format(NAME, GRADE, SEED, OUT_DIR))

    mats = qf.materials(stage, fb.BAKE_ROOT)
    rng = random.Random(SEED)
    nrng = np.random.default_rng(SEED)
    t_build = time.time()
    ctx, doomed, pls = build(stage, mats, rng, nrng)
    build_s = time.time() - t_build
    for _ in range(6):
        omni.kit.app.get_app().update()
    for n in ctx["notes"]:
        print("[qgac]     " + n)
    print("[qgac] built in {0:.0f} s: {1} piece(s), {2} loose, {3} static, "
          "{4} authored".format(build_s, len(pls), len(ctx["loose"]),
                                len(ctx["static_extra"]), len(ctx["authored"])))

    # -- settle, ALONE -- `fire_bake_launch_script.main()`'s call, verbatim --
    t_settle = time.time()
    info = {}
    if ctx["loose"]:
        info = settle.run(
            stage, ctx["loose"], ctx["static_extra"], steps=SETTLE_STEPS,
            kick=0.10, rng=random.Random(SEED),
            bake_result=not KEEP_PHYSICS, velocity_map=ctx["velocity"],
            density=1600.0, max_speed=6.0, converge=True,
            max_steps=int(SETTLE_STEPS * 3), quiet_steps=SETTLE_QUIET,
            ccd=True, ground_plane_z=0.0, floor_z=0.0,
            decompose_larger_than=(SETTLE_DECOMP_M or None)) or {}
    settle_s = time.time() - t_settle
    for _ in range(6):
        omni.kit.app.get_app().update()
    settle_info = {k: info.get(k) for k in (
        "steps_used", "steps_cap", "quiet_used", "converged", "stop_reason",
        "still_moving", "still_moving_paths", "below_grade", "clamped",
        "rescued", "baked", "moved_mean", "moved_max", "spread_max", "solve_s")}
    settle_info["bodies"] = len(info.get("bodies") or [])
    print("[qgac] settled {0} body(s) in {1:.0f} s: converged={2}, still "
          "moving {3}, below grade {4}".format(
              settle_info["bodies"], settle_s, settle_info.get("converged"),
              settle_info.get("still_moving"), settle_info.get("below_grade")))
    if settle_info.get("still_moving"):
        print("[qgac] WARNING: {0} body(s) were STILL MOVING at bake time — "
              "raise SETTLE_STEPS/SETTLE_QUIET; examples: {1}".format(
                  settle_info["still_moving"],
                  ", ".join(info.get("still_moving_examples") or [])))
        n_off = 0
        for pth in (info.get("still_moving_paths")
                    or info.get("still_moving_examples") or []):
            pr = stage.GetPrimAtPath(Sdf.Path(str(pth)))
            if pr and pr.IsValid():
                pr.SetActive(False)
                n_off += 1
        settle_info["deactivated"] = n_off
        print("[qgac] deactivated {0} body(s) frozen mid-flight".format(n_off))

    # -- nothing hangs in the air -- `fire_bake.deactivate_airborne`, verbatim
    settle_info["airborne_off"] = fb.deactivate_airborne(stage, fb.BAKE_ROOT)
    for _ in range(2):
        omni.kit.app.get_app().update()

    # -- the export -----------------------------------------------------
    t_exp = time.time()
    looks = fb.BAKE_ROOT + "/Looks"
    rh = fb.rehome_for_export(stage, fb.BAKE_ROOT, doomed, looks, verbose=True)
    src_kept = bool(rh["failed"])
    if doomed and not src_kept:
        for d in doomed:
            if stage.GetPrimAtPath(Sdf.Path(d)).IsValid():
                stage.RemovePrim(Sdf.Path(d))
        print("[qgac] dropped the merged source subtree ({0})".format(
            ", ".join(doomed)))
    elif doomed:
        print("[qgac] *** KEEPING {0} *** — {1} material(s) could not be "
              "rehomed: {2}".format(", ".join(doomed), rh["failed"],
                                     ", ".join(rh["failed_paths"][:4])))

    bbox = _bbox(stage, CELL)
    top_z = bbox[5] if bbox else None

    if not KEEP_PHYSICS:
        fb.strip_physics(stage, root=None, remove_prims=fb.STRIP_PRIMS,
                         verbose=True)
    for p in list(stage.GetPseudoRoot().GetChildren()):
        if p.GetName() != "World":
            print("[qgac] removing stray root prim {0}".format(p.GetPath()))
            stage.RemovePrim(p.GetPath())
    stage.SetDefaultPrim(stage.GetPrimAtPath(Sdf.Path(fb.DEFAULT_PRIM)))

    os.makedirs(OUT_DIR, exist_ok=True)
    out_usd, out_json = fb.out_paths(ENTRY, OUT_DIR)
    # ROOT LAYER ONLY — never `stage.Export()` / `stage.Flatten()` (Kit
    # meshes/subsets/materials carry an `assetInfo` dict core USD cannot
    # unpack; see `fire_bake.py`'s module docstring).
    stage.GetRootLayer().Export(out_usd)
    export_s = time.time() - t_exp
    mb = os.path.getsize(out_usd) / 1e6
    print("[qgac] exported {0:.1f} MB in {1:.0f} s -> {2}".format(
        mb, export_s, out_usd))

    # -- the sidecar ------------------------------------------------------
    prims_n = sum(1 for _ in Usd.PrimRange(
        stage.GetPrimAtPath(Sdf.Path(CELL)), Usd.PrimAllPrimsPredicate))
    counts = {"loose": len(ctx["loose"]), "static": len(ctx["static_extra"]),
              "authored": len(ctx["authored"]), "pieces": len(pls),
              "prims": prims_n, "usd_mb": round(mb, 2)}
    timings = {"build_s": round(build_s, 1), "settle_s": round(settle_s, 1),
               "export_s": round(export_s, 1),
               "total_s": round(time.time() - t0, 1)}
    extra = {"rehome": {k: v for k, v in rh.items() if k != "failed_paths"}}
    doc = quake_sidecar(ENTRY, ctx, bbox, top_z, timings, counts,
                        settle_info=settle_info, usd=out_usd,
                        src_kept=src_kept, extra=extra)
    fb.write_sidecar(out_json, doc)
    print("[qgac] sidecar -> {0}".format(out_json))

    if VERIFY:
        try:
            fb.verify_export(out_usd, doomed=tuple(doomed) or ("/src",),
                             expect_root=fb.BAKE_ROOT, check_remote=False)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[qgac] verify FAILED: {0}".format(exc))

    # -- the manifest, keyed by (name, grade), under a file lock -----------
    sliced = ctx.get("sliced") or {}
    m = ctx["info"]["masses"]["main"]
    rf = _rubble_fields(ctx)
    record = {
        "usd": os.path.abspath(out_usd), "name": NAME, "grade": GRADE,
        "btype": sliced.get("btype"), "W": float(m["W"]), "D": float(m["D"]),
        "H": float(m["top"] - m["z0"]),
        "fall_sides": rf.get("fall_sides", []),
        "extent_m": rf.get("extent_m", {}), "crown_m": rf.get("crown_m", 0.0),
        "mb": round(mb, 2), "prims": prims_n,
    }
    man_path = os.path.join(OUT_DIR, "gac_quake.json")
    import fcntl
    with open(man_path + ".lock", "w") as _lk:
        fcntl.flock(_lk, fcntl.LOCK_EX)
        merged = merge_manifest(man_path, [record])
        with open(man_path, "w") as fh:
            json.dump(merged, fh, indent=1, sort_keys=True)
        fcntl.flock(_lk, fcntl.LOCK_UN)
    print("[qgac] manifest -> {0} ({1} record(s))".format(
        man_path, len(merged)))

    print("\n" + "=" * 78)
    print("QUAKE GAC BAKE  {0} {1} seed {2}".format(NAME, GRADE, SEED))
    print("  usd        {0}  ({1:.1f} MB, {2} prim(s) in the cell)".format(
        out_usd, mb, prims_n))
    print("  sidecar    {0}".format(out_json))
    print("  btype      {0}, {1:.0f} x {2:.0f} x {3:.0f} m".format(
        record["btype"], record["W"], record["D"], record["H"]))
    if record["fall_sides"]:
        print("  rubble     fall {0}, crown ~{1:.1f} m".format(
            "/".join(record["fall_sides"]), record["crown_m"]))
    if bbox:
        print("  bbox       {0:.1f} x {1:.1f} x {2:.1f} m, top {3:.1f} m"
              .format(bbox[3] - bbox[0], bbox[4] - bbox[1], bbox[5] - bbox[2],
                      top_z))
    print("  settle     {0} body(s), converged={1}, {2} still moving".format(
        settle_info["bodies"], settle_info.get("converged"),
        settle_info.get("still_moving")))
    print("  timing     {0} s build + {1} s settle + {2} s export = {3} s"
          .format(timings["build_s"], timings["settle_s"],
                  timings["export_s"], timings["total_s"]))
    print("=" * 78 + "\n")
    print("QUAKE GAC BAKE DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception as _exc:
        import traceback
        traceback.print_exc()
        print("QUAKE GAC BAKE FAILED: {0}".format(_exc))
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    if keep:
        _app = omni.kit.app.get_app()
        while simulation_app.is_running():
            _app.update()
    simulation_app.close()
