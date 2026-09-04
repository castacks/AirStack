#!/usr/bin/env python
"""
fire_bake — ONE burnt building per Kit process, settled alone and exported.

    ISAAC_SIM_HEADLESS=true FB_KIND=gac FB_NAME=SM_Building_02 FB_LEVEL=F1 \
    FB_SEED=7 FB_OUT=/isaac-sim/.cache/fire_bakes SETTLE_STEPS=2400 \
    PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
    /isaac-sim/python.sh \
    /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/fire_bake_launch_script.py \
    --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window

Driven one building at a time by `scene_gen/tools/fire_bake.sh`; the results
are assembled by `fire_assembly_launch_script.py`.

WHY ONE BUILDING PER PROCESS
-----------------------------
`gac_fire_bench_launch_script.py` builds the whole row in one process — six
sliced GAC buildings plus two MCE kit buildings at `F5c`. Measured
2026-08-30: **25 GB RSS**, ONE combined settle of ~2,350 loose bodies that
finished with **688 bodies STILL MOVING at bake time**, and a stage that is
slow to open because every building's merged source, slicer state, position-
map cache and physics scene is still in it.

Splitting it (user, 2026-08-30: *"you can do 1 building at a time, bake it
then launch them together as static. That might be faster."*) fixes all
three at once:

  * memory is bounded by ONE building, so the pathological
    `soot_posmap` cache (3,158 unique sliced pieces x 7 MB) never has six
    buildings' worth of entries alive together;
  * the settle budget is spent on ONE pile, so `converge=True` can actually
    reach rest instead of running out of steps eight buildings in — that is
    what the 688 movers were, not a physics failure;
  * the assembly loads finished static geometry.

WHAT A BAKE CONTAINS, AND WHAT IT DELIBERATELY DOES NOT
-------------------------------------------------------
**No Flow.** Smoke and flame are authored in the ASSEMBLY (the user's stated
plan), so `flow_root=None` here and `r_flames` no-ops with a note. What the
bake DOES carry is the fire EVENT list, written to the sidecar by
`fire_bake.events_to_json`, which is everything `urban_fire._flame_sources`
needs to put the emitters back on the right window heads afterwards.

**No physics.** `settle.run(bake_result=True)` freezes every body's transform
but leaves `PhysicsRigidBodyAPI` (disabled), the colliders and ~22
`physics:`/`physx*` attributes applied — right for the settling process,
dead weight in a file that is only ever referenced.
`fire_bake.strip_physics` takes them off along with the settle's own
`/World/physicsScene` and `/World/settleGroundPlane`.

**No source subtree.** See below.

THE MATERIAL TRAP — READ IT BEFORE CHANGING THE EXPORT
-------------------------------------------------------
`tools/bake_gac_kits.py` and `detail/gac_slice.rehome_materials` have the
long version. Short version: a sliced GAC piece binds a Material prim living
INSIDE the merged source's subtree (`<cell>/src`), and so does every sooted
copy of one — `soot_plume.piece_material_like` composes the source material
with an `AddInternalReference` and overrides only its diffuse map. Drop the
source and export, and every piece renders WHITE with its geometry and UVs
perfectly intact. So `fire_bake.rehome_for_export` re-anchors each such
material onto the material's OWN Nucleus file and re-applies the local PNG,
and only THEN is `<cell>/src` removed. If any material cannot be rehomed the
source is KEPT (invisible) and the sidecar says `src_kept: true` — an
invisible source subtree costs load time; a white building is not shippable.

And the export is the **ROOT LAYER ONLY** (`stage.GetRootLayer().Export`),
never `stage.Flatten()` / `stage.Export()`: kit meshes, subsets and materials
carry an `assetInfo` dict core USD cannot unpack (`Usd_CrateFile::
_UnpackValue: unsupported type enum value 0` — see the
`freeze-disaster-dataset` skill and `disaster/freeze.py`). Everything this
script authors is either a fresh prim or a fresh reference, which is exactly
what a root-layer export preserves without composing a spec out of the
poisoned source.

WHERE THE TEXTURES LIVE. The soot PNGs are written by
`soot_plume.merge_piece` / `urban_fire._bind_soot` / `gac_fire.bake_atlases`
to `soot_plume.OUT_DIR`, which is inside the REPO (`scene_gen/assets/
materials/soot`) and therefore on the host through the bind mount. Since the
bakes are container-only working files, this script re-points `OUT_DIR` at
`<FB_OUT>/textures` (module global, read at call time by all three writers —
no edit to `soot_plume.py` needed) so a bake and its maps travel together.
The materials reference them by ABSOLUTE container path, which is what
`fire_bake.verify_export` re-checks by reopening the file cold.

Env:
    FB_KIND         `gac` (a GreatAmericanCity merged asset, sliced),
                    `dtc` (a downtowncity merged asset, sliced the same way —
                    `gac_fire.PACKS` carries the per-pack directory, file
                    extension, unit scale and construction-type source) or
                    `kit` (a ModernCityEnvironment kit style)
    FB_NAME         merged asset name (`SM_Building_02`, `Amar_Tower`) or kit
                    style (`commercial_mid`)
    FB_LEVEL        fire level, `urban_fire.LEVELS` (F0..F6, F5c)
    FB_SEED         this building's rng seed (the driver gives building i
                    `base + 31*i`, matching `gac_fire_bench`'s own column
                    seed)
    FB_BUILD_SEED   seed for `urban_building.build_building` on the kit path
                    (default FB_SEED; the bench uses `base + 7*i` there, so
                    the driver passes that to reproduce a bench column)
    FB_INDEX        this building's column index — only used for the prim
                    tag and the sidecar (default 0)
    FB_ORIGIN       force the fire's origin storey (default: drawn)
    FB_SIDES        force the venting elevations, e.g. `S` or `S,E`
    FB_OUT          output directory (default /isaac-sim/.cache/fire_bakes) —
                    CONTAINER-SIDE, not the repo
    FB_TEX_DIR      soot PNG directory (default <FB_OUT>/textures)
    FB_CITY_JSON    path to a small JSON file holding one CITY-CELL record
                    (`cell`/`x`/`y`/`yaw_deg`/`z`/`typology`/`orig_usd`,
                    `urban_fire_city_plan.md` sec 3) — written by
                    `tools/fire_city_manifest.py --write-city-json` and read
                    by `tools/fire_city_bake.sh`. Merged into the sidecar as
                    `extra={"city": ...}`. Empty (default) is a no-op: no
                    file is read, no `"city"` key is added — every plain row
                    bake (`fire_bake.sh`) is unaffected.
    FB_BAKED_KITS   1 (default) uses a pre-baked GAC kit where one exists
    FB_KEEP_SRC     1 keeps `<cell>/src` composed (debugging the trap above)
    FB_VERIFY       1 (default) reopens the export cold and checks it
    SETTLE_STEPS    physics step target (default 2400 — HIGHER than the
                    bench's 1600 on purpose: one building's pile gets the
                    whole budget)
    SETTLE_QUIET    quiet-phase steps (default 400, bench 60 — "the single
                    cheapest way to drive still_moving to zero",
                    `settle.run`'s own docstring)
    SETTLE_DECOMP_M convex-decomposition threshold, metres (default 0.8 —
                    see `urban_fire_bench_launch_script.py` for the
                    measurement behind 0.8 rather than 2.5; 0 disables)
    SETTLE_RETRY_PASSES number of in-process continuation attempts after a
                    failed rest check (default 3; 0 restores old behaviour)
    SETTLE_RETRY_FRACTION fraction of SETTLE_STEPS advanced per continuation
                    attempt (default 1/3). The stage stays live: no rebuild,
                    re-kick, or collider recook occurs between attempts.
    KEEP_PHYSICS    1 leaves the bodies live and skips the physics strip
    KEEP_OPEN       1 keeps the app up after the export

Banner: `FIRE BAKE DONE` (or `FIRE BAKE FAILED`).
"""

import json
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    """The container exports every launcher knob as an EMPTY STRING, so
    `os.environ.get(name, default)` never reaches its default — a numeric
    knob raises partway into the launch and a path knob silently becomes "".
    Treat empty as absent."""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


# HEADLESS BY DEFAULT here, unlike the benches: a bake has nothing to look at
# and the driver runs several of them back to back.
_HEADLESS = _env("ISAAC_SIM_HEADLESS", "true").lower() in ("1", "true", "yes")
# `disaster/ground.py:KIT_ARGS` is the source of truth; kept as a literal
# because `scene_gen` is not on `sys.path` until after `SimulationApp` is
# constructed. Nothing is RENDERED here, so fractional cutout opacity does
# not change the output — it is set anyway so that a `KEEP_OPEN=1` look at a
# bake shows the same graded staining the assembly will.
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
import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import gac_slice as gsl                            # noqa: E402
from detail import urban_building as ub                        # noqa: E402
from disaster import fire_bake as fb                           # noqa: E402
from disaster import fire_collapse as fcol                     # noqa: E402
from disaster import fracture, settle                          # noqa: E402
from disaster import gac_fire as gcf                           # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import soot_plume as spl                         # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

KIND = _env("FB_KIND", "gac").lower()
NAME = _env("FB_NAME", "SM_Building_02")
LEVEL = _env("FB_LEVEL", "F3")          # NOT .upper(): `F5c` is a real level
SEED = int(_env("FB_SEED", "7"))
BUILD_SEED = int(_env("FB_BUILD_SEED", str(SEED)))
INDEX = int(_env("FB_INDEX", "0"))
ORIGIN = _env("FB_ORIGIN", "")
SIDES = fb.parse_sides(_env("FB_SIDES", ""))
# Path to a small JSON file holding one `urban_fire_city.damaged_manifest`
# record's static placement facts (`cell`/`x`/`y`/`yaw_deg`/`z`/`typology`/
# `orig_usd`) -- `scene_gen/tools/fire_city_bake.sh` writes one of these per
# record (via `fire_city_manifest.py --write-city-json`) and points this at
# it so the sidecar can carry `extra={"city": ...}` (`urban_fire_city_plan.md`
# sec 3), which is what the city launcher will use to find the cell's own
# transform and hide the intact prim without re-deriving either from the
# manifest. Empty (the default -- every existing `fire_bake.sh` row entry)
# is a strict no-op: no file is read and no `"city"` key is added.
CITY_JSON = _env("FB_CITY_JSON", "")
OUT_DIR = _env("FB_OUT", fb.DEFAULT_OUT_DIR)
TEX_DIR = _env("FB_TEX_DIR", os.path.join(OUT_DIR, "textures"))
BAKED_KITS = _env("FB_BAKED_KITS", "1") not in ("0", "false", "no")
KEEP_SRC = _env("FB_KEEP_SRC", "0") not in ("0", "false", "no")
VERIFY = _env("FB_VERIFY", "1") not in ("0", "false", "no")
SETTLE_STEPS = int(_env("SETTLE_STEPS", "2400"))
SETTLE_QUIET = int(_env("SETTLE_QUIET", "400"))
SETTLE_DECOMP_M = float(_env("SETTLE_DECOMP_M", "0.8"))
SETTLE_RETRY_PASSES = int(_env("SETTLE_RETRY_PASSES", "3"))
SETTLE_RETRY_FRACTION = float(_env("SETTLE_RETRY_FRACTION", "0.333333333333"))
KEEP_PHYSICS = _env("KEEP_PHYSICS", "0") not in ("0", "false")

ENTRY = {"kind": KIND, "name": NAME, "level": LEVEL, "seed": SEED,
         "origin": (int(ORIGIN) if ORIGIN else None), "sides": SIDES,
         "index": INDEX}
TAG = fb.bake_tag(ENTRY)
CELL = "{0}/{1}".format(fb.BAKE_ROOT, TAG)


def _load_city_json(path):
    """`FB_CITY_JSON`'s content as a plain dict, or `None`. A strict no-op
    when `path` is empty (the default): no file access at all, and the
    sidecar's `extra` never gets a `"city"` key. A path that IS set but
    cannot be read/parsed is a WARNING, not a crash -- a bake with no
    `"city"` in its sidecar just cannot be placed at its cell by the (not
    yet written) city launcher; it should not fail the bake itself."""
    if not path:
        return None
    try:
        with open(path) as fh:
            return json.load(fh)
    except Exception as exc:
        print("[fb] WARNING: FB_CITY_JSON={0!r} could not be read ({1}) -- "
              "sidecar will carry no \"city\" record".format(path, exc))
        return None


CITY_RECORD = _load_city_json(CITY_JSON)


def _bbox(stage, path):
    """World-aligned bbox of `path`, or None. Respects visibility, so an
    invisible `<cell>/src` that had to be kept does not inflate it."""
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


def _interior_seats(stage, ctx, rng, top_z, budget=None):
    """`urban_fire._interior_smoke`'s SELECTION, recorded as plain points.

    The assembly has no `ctx["fit"]` — the slabs are geometry in a referenced
    file by then — so the choice of WHICH storey smoulders and WHERE on it is
    made here, while the fit-out is still addressable, and only the resulting
    world points travel. Same rules as `_interior_smoke`: a storey in the
    involved band, whose slab survived `r_floor_burnthrough`, severity >=
    0.25, hottest first within the budget.

    The z is CLAMPED to the settled geometry's own top. That is the fix for
    "smoke floating over a collapsed building": `m["levels"][storey]` is
    where the floor was PLANNED, and after `fire_collapse` has dropped the
    top storeys into the shell it can be metres above anything that is left.
    """
    budget = uf.INTERIOR_SMOKE_MAX if budget is None else budget
    f = ctx["fire"]
    fit = ctx.get("fit") or {}
    cand = []
    for (mtag, storey), slab in (fit.get("slabs") or {}).items():
        if not slab or storey not in f["storeys"]:
            continue
        p = stage.GetPrimAtPath(Sdf.Path(slab))
        if not p or not p.IsValid() or not p.IsActive():
            continue
        sev = uf._severity(ctx, storey, mtag)
        if sev < 0.25:
            continue
        cand.append((float(sev), mtag, int(storey)))
    cand.sort(key=lambda t: -t[0])
    out = []
    for sev, mtag, storey in cand[:budget]:
        m = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
        lx = rng.uniform(-0.25, 0.25) * m["W"]
        ly = rng.uniform(-0.25, 0.25) * m["D"]
        wx, wy = qf._to_world(m, lx, ly)
        z = m["levels"][min(storey, len(m["levels"]) - 1)] + 0.4
        if top_z is not None:
            z = min(z, float(top_z) - 0.5)
        out.append({"x": float(wx), "y": float(wy), "z": float(z),
                    "radius": 1.1, "scale": 1.1 * (0.6 + 0.6 * sev),
                    "mass": mtag, "storey": storey, "sev": sev})
    if not fit.get("slabs") and f.get("storeys"):
        # NO SLAB TO SEAT ON — but NOT "nothing was hot enough" (that case,
        # a grid that WAS authored but had nothing above the severity
        # floor, is unchanged: it still returns the empty list `out`
        # already is here). This is the documented shape of a shell that
        # stays closed (`gac_fire.burn_gac`/`urban_fire.shows_interior`
        # skip `quake_flow.fit_interior`'s grid outright for a
        # (construction type, level) whose ladder never reaches a
        # burnthrough or a collapse), so there is no slab in `fit` at all
        # to pick from, hot or not.
        #
        # A MASS-CENTRE FALLBACK, NOT A SKIP: `fire_assembly_lib.
        # author_fire`'s own "last resort" branch (`if not is_flame or
        # (n_flame == 0 and n_smoke == 0)`) reads `seats["interior"]` as ITS
        # OWN ONLY smoke source for exactly this case — an F1 building has
        # no `ACTIVE` flame state at all, so it hits that branch on EVERY
        # bake, and any other level whose flame/side-smoke pick happens to
        # come up empty hits it too. An empty seat list here would leave
        # such a building showing NOTHING — indistinguishable from a fire
        # that never ran, the exact "invisible fire" failure mode the event-
        # starvation fix (`gac_fire.prepare`'s `openings_provider`) already
        # exists to prevent one level up. One seat at the fire's own mass
        # centre, at the middle storey of the burning band, is enough to
        # read as smoke coming from inside the building without pretending
        # a slab is there to stand on.
        mtag = f.get("mass") or "main"
        m = ctx["info"]["masses"].get(mtag) or ctx["info"]["masses"]["main"]
        storeys = sorted(int(s) for s in f["storeys"])
        storey = storeys[len(storeys) // 2]
        z = m["levels"][min(storey, len(m["levels"]) - 1)] + 0.4
        if top_z is not None:
            z = min(z, float(top_z) - 0.5)
        out.append({"x": float(m["cx"]), "y": float(m["cy"]), "z": float(z),
                    "radius": 1.0, "scale": 0.9, "mass": mtag,
                    "storey": storey, "sev": 0.3, "fallback": "mass_centre"})
        print("[fb] interior seats: 0 from the fit-out grid (shell stays "
              "closed for this level, no slab authored) -- 1 mass-centre "
              "fallback seat added so the building still shows interior "
              "smoke")
    elif not out:
        print("[fb] interior seats: 0 (no fire band at all)")
    return out


def _roof_seats(ctx, rng, top_z):
    """`urban_fire._roof_plume`'s two sources, seated on what is LEFT.

    The original sits them at `m["top"] - 0.6` — the roof the building was
    built with. On an F5/F5c that roof is on the floor of the shell, and a
    plume authored at the planned height hangs in clear air over the hole
    (user, 2026-08-30). `top_z` is the settled geometry's own maximum, so
    this seats them on the highest thing still standing.
    """
    m = ctx["info"]["masses"]["main"]
    z = float(m["top"]) - 0.6
    if top_z is not None:
        z = min(z, float(top_z) - 0.6)
    out = []
    for _k in range(2):
        lx = rng.uniform(-0.3, 0.3) * m["W"]
        ly = rng.uniform(-0.3, 0.3) * m["D"]
        wx, wy = qf._to_world(m, lx, ly)
        out.append({"x": float(wx), "y": float(wy), "z": float(z),
                    "radius": 1.8, "scale": 1.25})
    return out


def build_gac(stage, ssf, mats):
    """The sliced whole-asset path — `gac_fire.burn_gac`, exactly as
    `gac_fire_bench_launch_script.py` calls it, minus the Flow root.

    Serves BOTH sliced kinds. `gac_fire.prepare` resolves the pack from a
    `kind:name` prefix (`gac_fire.split_kind`), so a downtowncity asset is
    the same call with `dtc:` in front of the name — nothing else in this
    script, in `burn_gac` or in the ladder has to know which pack it is.
    `FB_KIND=gac` passes the bare name, which is what it always passed.
    """
    qname = NAME if KIND == "gac" else "{0}:{1}".format(KIND, NAME)
    rng = random.Random(SEED)
    nrng = np.random.default_rng(SEED)
    bctx = gcf.burn_gac(
        stage, CELL, qname, LEVEL, rng, nrng, mats, TAG,
        flow_root=None, mat_cache={}, ssf=ssf,
        origin=(int(ORIGIN) if ORIGIN else None), sides=SIDES,
        use_baked_kit=BAKED_KITS, out_dir=TEX_DIR, verbose=True)
    # the merged source, which is what must not ship (see the module
    # docstring's MATERIAL TRAP)
    return bctx, [CELL + "/src"]


def build_aec(stage, ssf, mats):
    """The AEC brownstone path — `disaster/aec_burn.py` on the row asset
    referenced RAW and CENTRED on the cell (`gac_fire.place_source`, the
    same seat every sliced bake is placed by, so the city assembly's
    `place_holder` at the record's centre lands it exactly as it lands a
    GAC bake). Nothing is sliced and nothing is dropped: the asset stays
    referenced, its MDL brick stays bound, the soot rides a conformal layer,
    the damage ladder deactivates named parts, the collapse bodies live in
    the unscaled `<asset>_debris` sibling and are settled by `main` like any
    other bake's `loose`. See the build-urban-fire-scenes skill (2026-09-02
    night) and `aec_burn`'s docstring.

    Units: `FB_UNITS` (`2,3` / `2-3`), else a seeded contiguous pick
    (`aec_burn.pick_units`) so a city of burning rows is not a city of
    identical middle pairs. Recorded in the sidecar's `aec` block.
    """
    from disaster import aec_burn
    url = gcf.asset_url(NAME, "aec")
    scale = float(gcf.PACKS["aec"].get("scale", 0.01))
    holder = gcf.place_source(stage, CELL, url, scale=scale)
    if not holder:
        raise RuntimeError("could not place {0}".format(url))
    root = holder + "/asset"
    rng = random.Random(SEED)
    units = aec_burn.parse_units(_env("FB_UNITS", ""))
    meas = aec_burn.measure_row(stage, root, verbose=True)
    if units is None:
        units = aec_burn.pick_units(len(meas["units"]), random.Random(SEED ^ 0xA5))
    plan = aec_burn.plan_row(meas, level=LEVEL, units=units, seed=SEED,
                             origin=(int(ORIGIN) if ORIGIN else None),
                             sides=(tuple(SIDES) if SIDES else None), verbose=True)
    dstats = aec_burn.damage_row(stage, meas, plan, verbose=True)
    astats = aec_burn.author_row(stage, meas, plan, out_dir=TEX_DIR, verbose=True)
    fire = dict(plan["fire"])
    fire["events"] = plan["events"]
    fire["deck_z"] = plan["m"].get("deck_z")
    fire["units"] = list(units)
    # the storey of origin keeps its floor: that is the slab the interior
    # smoke seat sits on (the floors above it are on the pile)
    origin_floor = None
    for unit in plan["burning"]:
        for mrec in unit["meshes"]:
            b = mrec["bbox"]
            if (mrec["cat"] == "Floors" and not mrec.get("dead")
                    and aec_burn._storey_of(plan["levels"], 0.5 * (b[2] + b[5]))
                    == int(fire["origin"])):
                origin_floor = mrec["path"]
                break
        if origin_floor:
            break
    notes = ["aec: units {0} of {1}, {2} event(s), sides {3}, band {4}".format(
        units, len(meas["units"]), len(plan["events"]), "/".join(fire["sides"]),
        fire["storeys"])]
    notes.append("aec damage: " + ", ".join(
        "{0} {1}".format(k, len(v) if isinstance(v, (list, dict)) else v)
        for k, v in sorted(dstats.items())))
    notes.append("aec soot: {0} tri(s) on {1} part(s), {2} roof tri(s), {3} "
                 "interior part(s) charred".format(
                     astats.get("tris_soot"), astats.get("parts_soot"),
                     astats.get("tris_roof", 0), astats.get("parts_char")))
    bctx = {"fire": fire, "info": plan["info"], "stage": stage, "tag": TAG,
            "rng": rng, "loose": list(dstats.get("loose") or []),
            "static_extra": list(dstats.get("static") or []),
            "velocity": dict(dstats.get("velocity") or {}),
            "authored": [], "notes": notes,
            "fit": {"slabs": {("main", int(fire["origin"])): origin_floor}},
            "aec": {"units": list(units), "n_units": len(meas["units"]),
                    "asset": url, "root": root, "damage": {
                        k: (len(v) if isinstance(v, (list, dict)) else v)
                        for k, v in dstats.items()}}}
    return bctx, []


def build_kit(stage, ssf, mats):
    """The ModernCityEnvironment kit path — `urban_fire.burn_building` over a
    `urban_building.build_building`, the same three lines
    `gac_fire_bench`'s `GF_EXTRA_KIT` block and `mce_fire_launch_script` use.

    BUILD INTO A CHILD, NOT THE CELL. `apply_placements` turns its parent
    into a Scope, which is not Xformable, and the cell's own transform is
    then silently dropped — the `Scope.Define` trap that snapped every kit
    building to the origin.
    """
    rng = random.Random(SEED)
    nrng = np.random.default_rng(SEED)
    pls = ub.build_building(NAME, 0.0, 0.0, 0.0, random.Random(BUILD_SEED))
    sg.apply_placements(stage, pls, CELL + "/parts", ssf)
    ub.apply_glass_tint(stage, pls)
    specs = qf._mass_specs(NAME, 0.0, 0.0, 0.0)
    main_spec = max(specs, key=lambda m: len(m["levels"]))
    n_st = max(1, len(main_spec["levels"]))
    origin = (int(ORIGIN) if ORIGIN
              else max(0, min(n_st - 1, int(round(0.25 * (n_st - 1))))))
    bctx = uf.burn_building(stage, CELL, NAME, pls, 0.0, 0.0, 0.0, LEVEL,
                            rng, nrng, mats, TAG, flow_root=None,
                            origin=origin, sides=SIDES, mat_cache={})
    return bctx, []


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
    # THE CELL IS AN XFORM AT THE ORIGIN. `gac_fire.place_source` reads the
    # cell's own local-to-world translation to seat the asset on it, so it
    # has to be Xformable — and it is left at identity because a bake is
    # always built at the origin and PLACED by the assembly.
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))
    _, ssf = get_stage_meters_per_unit(stage)

    # THE SOOT PNGS TRAVEL WITH THE BAKE. Module global, read at call time by
    # `gac_fire.bake_atlases` (via its `out_dir`), `urban_fire._bind_soot`
    # and `soot_plume.merge_piece` — so re-pointing it here is enough and no
    # file anybody else owns has to change.
    os.makedirs(TEX_DIR, exist_ok=True)
    spl.OUT_DIR = TEX_DIR

    fracture.ensure_deps()
    fracture.ensure_vtk(verbose=True)

    problems = uf.check(verbose=False)
    if KIND in fb.SLICED_KINDS:
        problems += gsl.check(verbose=False)
    if LEVEL == fcol.FIRE_LEVEL:
        problems += fcol.check(verbose=False)
    if problems:
        raise RuntimeError("; ".join(problems))
    if KIND not in fb.KINDS:
        raise RuntimeError("FB_KIND must be one of {0}, not {1!r}".format(
            "/".join(fb.KINDS), KIND))
    if LEVEL not in uf.LEVELS:
        raise RuntimeError("unknown fire level {0}".format(LEVEL))
    if KIND == "kit" and NAME not in ub.STYLES:
        raise RuntimeError("unknown kit style {0!r}".format(NAME))

    print("[fb] {0}:{1} {2} seed {3} -> {4}".format(
        KIND, NAME, LEVEL, SEED, OUT_DIR))

    mats = uf.materials(stage, fb.BAKE_ROOT)
    t_build = time.time()
    if KIND in fb.SLICED_KINDS:
        bctx, doomed = build_gac(stage, ssf, mats)
    elif KIND == "aec":
        bctx, doomed = build_aec(stage, ssf, mats)
    else:
        bctx, doomed = build_kit(stage, ssf, mats)
    build_s = time.time() - t_build
    for _ in range(6):
        omni.kit.app.get_app().update()
    for n in bctx["notes"]:
        print("[fb]     " + n)
    print("[fb] built in {0:.0f} s: {1} loose, {2} static, {3} authored"
          .format(build_s, len(bctx["loose"]), len(bctx["static_extra"]),
                  len(bctx["authored"])))

    # -- settle, ALONE ------------------------------------------------------
    # The bench's own argument set (`ccd` / `ground_plane_z` / `floor_z` /
    # `decompose_larger_than` — see `urban_fire_bench_launch_script.py` for
    # why a plain `settle.run(stage, loose, static)` is not enough), with the
    # budget raised because it is not shared with seven other buildings.
    # There is no `/World/ground` mesh here: `ground_plane_z=0.0` authors a
    # real PhysX half-space, which is the floor the settle actually wants.
    t_settle = time.time()
    info = {}
    if bctx["loose"]:
        info = settle.run(
            stage, bctx["loose"], bctx["static_extra"], steps=SETTLE_STEPS,
            kick=0.10, rng=random.Random(SEED),
            bake_result=not KEEP_PHYSICS, velocity_map=bctx["velocity"],
            density=1600.0, max_speed=6.0, converge=True,
            max_steps=int(SETTLE_STEPS * 3), quiet_steps=SETTLE_QUIET,
            ccd=True, ground_plane_z=0.0, floor_z=0.0,
            decompose_larger_than=(SETTLE_DECOMP_M or None),
            retry_passes=SETTLE_RETRY_PASSES,
            retry_fraction=SETTLE_RETRY_FRACTION) or {}
    settle_s = time.time() - t_settle
    for _ in range(6):
        omni.kit.app.get_app().update()
    settle_info = {k: info.get(k) for k in (
        "steps_used", "steps_cap", "quiet_used", "converged", "stop_reason",
        "still_moving", "still_moving_paths", "below_grade", "clamped",
        "rescued", "baked", "moved_mean", "moved_max", "spread_max", "solve_s")}
    settle_info["retry_passes"] = info.get("retry_passes", 0)
    settle_info["retry_steps"] = info.get("retry_steps", 0)
    settle_info["bodies"] = len(info.get("bodies") or [])
    print("[fb] settled {0} body(s) in {1:.0f} s: converged={2}, still "
          "moving {3}, below grade {4}".format(
              settle_info["bodies"], settle_s, settle_info.get("converged"),
              settle_info.get("still_moving"), settle_info.get("below_grade")))
    if settle_info.get("still_moving"):
        print("[fb] WARNING: {0} body(s) were STILL MOVING at bake time — "
              "raise SETTLE_STEPS/SETTLE_QUIET; examples: {1}".format(
                  settle_info["still_moving"],
                  ", ".join(info.get("still_moving_examples") or [])))
        # FROZEN MID-FLIGHT IS WORSE THAN ABSENT. A body the settle could
        # not bring to rest is baked wherever its last step left it — five
        # black flecks hung in the sky beside SM_Building_09 F6 in the first
        # assembled row (fire_row1, 2026-08-30). Nobody misses five chips of
        # debris; everybody sees them floating.
        n_off = 0
        for pth in (info.get("still_moving_paths")
                    or info.get("still_moving_examples") or []):
            pr = stage.GetPrimAtPath(Sdf.Path(str(pth)))
            if pr and pr.IsValid():
                pr.SetActive(False)
                n_off += 1
        settle_info["deactivated"] = n_off
        print("[fb] deactivated {0} body(s) frozen mid-flight".format(n_off))

    # -- nothing hangs in the air -------------------------------------------
    # after the settle and before the bbox is measured: a fragment frozen
    # 40 m up would otherwise set `top_z` and seat the smoke there
    settle_info["airborne_off"] = fb.deactivate_airborne(stage, fb.BAKE_ROOT)
    for _ in range(2):
        omni.kit.app.get_app().update()

    # -- the export ---------------------------------------------------------
    t_exp = time.time()
    looks = fb.BAKE_ROOT + "/Looks"
    rh = fb.rehome_for_export(stage, fb.BAKE_ROOT, doomed, looks, verbose=True)
    src_kept = bool(rh["failed"]) or KEEP_SRC
    if doomed and not src_kept:
        for d in doomed:
            if stage.GetPrimAtPath(Sdf.Path(d)).IsValid():
                stage.RemovePrim(Sdf.Path(d))
        print("[fb] dropped the merged source subtree ({0})".format(
            ", ".join(doomed)))
    elif doomed:
        print("[fb] *** KEEPING {0} *** ({1}) — the assembly will pay to "
              "compose it".format(", ".join(doomed),
                                  "FB_KEEP_SRC=1" if KEEP_SRC else
                                  "{0} material(s) could not be rehomed: {1}"
                                  .format(rh["failed"],
                                          ", ".join(rh["failed_paths"][:4]))))

    bbox = _bbox(stage, CELL)
    top_z = bbox[5] if bbox else None
    srng = random.Random(SEED ^ 0xF12E)
    seats = {"interior": _interior_seats(stage, bctx, srng, top_z),
             "roof": (_roof_seats(bctx, srng, top_z)
                      if bctx["fire"].get("roof") else [])}

    if not KEEP_PHYSICS:
        fb.strip_physics(stage, root=None, remove_prims=fb.STRIP_PRIMS,
                         verbose=True)
    # NOTHING BUT `/World` AT THE ROOT. A root-layer export writes every root
    # prim; Kit's own viewport cameras live in the SESSION layer and are
    # therefore already excluded, but anything else that appeared here would
    # ride into every reference of this bake.
    for p in list(stage.GetPseudoRoot().GetChildren()):
        if p.GetName() != "World":
            print("[fb] removing stray root prim {0}".format(p.GetPath()))
            stage.RemovePrim(p.GetPath())
    stage.SetDefaultPrim(stage.GetPrimAtPath(Sdf.Path(fb.DEFAULT_PRIM)))

    # Normalize the reusable BAKE, not only the eventual city. Kit's city
    # flatten can reconstruct referenced prototypes from this root layer and
    # resurrect its original local texture values after a live-stage rewrite.
    # Fixing the small source layer closes that composition path and also
    # keeps material reference arcs from degrading to fallback white.
    fb.rewrite_shared_asset_paths(stage.GetRootLayer())

    os.makedirs(OUT_DIR, exist_ok=True)
    out_usd, out_json = fb.out_paths(ENTRY, OUT_DIR)
    # ROOT LAYER ONLY — see the module docstring. Never `stage.Export()` /
    # `stage.Flatten()`.
    stage.GetRootLayer().Export(out_usd)
    export_s = time.time() - t_exp
    mb = os.path.getsize(out_usd) / 1e6
    print("[fb] exported {0:.1f} MB in {1:.0f} s -> {2}".format(
        mb, export_s, out_usd))

    # -- the sidecar --------------------------------------------------------
    events = list(bctx["fire"].get("events") or [])
    masses = {}
    for ev in events:
        for op in (ev.get("ops") or []):
            tagm = op.get("mass") or "main"
            if tagm not in masses and op.get("m") is not None:
                masses[tagm] = op["m"]
    # ...and anything the events never touched, so `fire["mass"]` always
    # resolves. THE OPS' OWN MASS WINS: on the GAC path `openings_provider`
    # frames are built from the MEASURED box (`gac_fire.mass_from_grid`),
    # which is not the same box `quake_flow.describe` derives from the
    # registered synthetic style — and the measured one is the box every
    # emitter was placed against.
    for k, v in (bctx["info"]["masses"] or {}).items():
        masses.setdefault(k, v)

    gac = bctx.get("gac") or {}
    counts = {"loose": len(bctx["loose"]),
              "static": len(bctx["static_extra"]),
              "authored": len(bctx["authored"]),
              "pieces": int(gac.get("n_pieces") or 0),
              "atlases": int(gac.get("n_atlases") or 0),
              "prims": sum(1 for _ in Usd.PrimRange(
                  stage.GetPrimAtPath(Sdf.Path(CELL)),
                  Usd.PrimAllPrimsPredicate)),
              "usd_mb": round(mb, 2)}
    timings = {"build_s": round(build_s, 1), "settle_s": round(settle_s, 1),
               "export_s": round(export_s, 1),
               "total_s": round(time.time() - t0, 1)}
    extra = {"rehome": {k: v for k, v in rh.items() if k != "failed_paths"},
             "build_seed": BUILD_SEED,
             "baked_kit": (BAKED_KITS if KIND in fb.SLICED_KINDS else None)}
    if bctx.get("aec"):
        extra["aec"] = bctx["aec"]        # units, damage census, asset url
    if CITY_RECORD is not None:
        # Namespaced, per `fire_bake.sidecar`'s own `extra` contract -- a
        # loose splice of the city record's keys onto `doc` could silently
        # overwrite one of `sidecar()`'s own (e.g. a record that somehow
        # carried its own "kind"/"level").
        extra["city"] = CITY_RECORD
    doc = fb.sidecar(ENTRY, bctx["fire"], masses, events, bbox, top_z, seats,
                     bctx["notes"], timings, counts,
                     settle_info=settle_info, usd=out_usd,
                     textures_dir=TEX_DIR, src_kept=src_kept, extra=extra)
    fb.write_sidecar(out_json, doc)
    print("[fb] sidecar -> {0}  ({1} event(s), {2} interior + {3} roof "
          "seat(s))".format(out_json, len(events), len(seats["interior"]),
                            len(seats["roof"])))

    if VERIFY:
        try:
            # an AEC bake KEEPS its asset under `<cell>/src` (nothing is
            # sliced, nothing is doomed), so the "/src" default must not be
            # applied to it: every one of its materials legitimately lives
            # there
            never = ("/__nothing_doomed__",) if KIND == "aec" else ("/src",)
            fb.verify_export(out_usd, doomed=tuple(doomed) or never,
                             expect_root=fb.BAKE_ROOT, check_remote=False)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[fb] verify FAILED: {0}".format(exc))

    print("\n" + "=" * 78)
    print("FIRE BAKE  {0}:{1} {2} seed {3}".format(KIND, NAME, LEVEL, SEED))
    print("  usd        {0}  ({1:.1f} MB, {2} prim(s) in the cell)".format(
        out_usd, mb, counts["prims"]))
    print("  sidecar    {0}".format(out_json))
    print("  fire       storeys {0}-{1} on {2}{3}, {4} event(s)".format(
        bctx["fire"]["origin"], bctx["fire"]["top"],
        "/".join(bctx["fire"]["sides"]),
        ", roof through" if bctx["fire"].get("roof") else "", len(events)))
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
    print("FIRE BAKE DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception as _exc:
        import traceback
        traceback.print_exc()
        print("FIRE BAKE FAILED: {0}".format(_exc))
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    if keep:
        _app = omni.kit.app.get_app()
        while simulation_app.is_running():
            _app.update()
    simulation_app.close()
