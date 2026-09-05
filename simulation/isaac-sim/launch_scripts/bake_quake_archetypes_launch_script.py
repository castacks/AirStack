#!/usr/bin/env python
"""
Bake one USD per (urban kit style x earthquake grade) — the archetype library
`downtown_quake_launch_script.py` assembles the damaged district from.

    ARCH_DIR=/isaac-sim/AirStack/omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype \
    ISAAC_SIM_SCRIPT_NAME=bake_quake_archetypes_launch_script.py \
    airstack up isaac-sim

The earthquake counterpart of `bake_tornado_archetypes_launch_script.py`:
every style in `ARCH_STYLES` is built ONCE per grade on a spread-out grid,
fitted out and broken with `disaster.quake_flow` (the same code the bench
uses), settled, and exported RE-CENTRED to the origin via `disaster.bake`.
`DG0` is the pristine building (no fit-out, nothing broken) and is what the
layout's building pools point at, so the packer lays out real footprints.

WHY IT SETTLES IN BATCHES, NOT ONCE
-----------------------------------
A URM total collapse is ~1,000 rigid bodies onto a ~1,000-chunk heap, and
there are eight masonry styles. One settle over the whole grid would be
tens of thousands of bodies in one PhysX scene; the tornado bake's 7k took
20+ minutes once. So the grid is settled one ROW (one style) at a time —
`settle.run` bakes each row to static before the next starts, and a baked
body is a disabled body, so earlier rows cost the later ones nothing but
their (sleeping) colliders.

`tilt_sink` IS NOT BAKED. It is a rigid transform of the pristine
archetype, applied at assembly, so it costs no archetype and any building
can get it.

Env:
    ARCH_DIR      output directory (default omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype)
    ARCH_SEED     rng seed (default 4 — see the note at SEED)
    ARCH_STYLES   comma list of urban_building styles, OR one of the group
                  names `mix250` (default, the 16-style 250 m mix), `tall`
                  (the 14 the 250 m mix omits) or `full` (all 30)
    ARCH_GRADES   comma list of levels (default DG0..DG5,SETTLE,TILT,OV — the
                  last three are the foundation family, quake_flow.FOUNDATION)
    ARCH_VARIANTS variants per damaged grade (default 1) — `_v1`, `_v2`, ...
    SETTLE_STEPS  per-row ceiling (default 2200)
    SETTLE_BODY_BUDGET  max loose (physics) bodies per row (default 3000);
                  `-1` = unlimited, today's behaviour. See
                  `disaster.quake_collapse.apply_settle_budget`. A "block"
                  style (main mass + several storeys-tall wings) is where
                  this actually bites — measured at 18,771 bodies / 1.5 h+
                  for `block_residential` DG3-5 before this existed.
"""

import math
import os
import random
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower() in ("1", "true", "yes")})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import numpy as np                                             # noqa: E402
import scene_generator as sg                                  # noqa: E402
from scene_prep import get_stage_meters_per_unit              # noqa: E402
from detail import urban_building as ub                       # noqa: E402
from disaster import bake, fracture, settle, quake_flow as qf  # noqa: E402
from disaster import quake_collapse as qc                      # noqa: E402
try:
    # BELT AND BRACES FOR ANY FLOATER IN A KIT ARCHETYPE BAKE, not just roof
    # props: `fire_bake_launch_script.py`'s own call site, after the settle
    # and before the bbox/export — a body the settle could not bring down in
    # its step budget bakes wherever it froze, and that includes a body a
    # `qc_*` recipe never itself resolved (see `quake_collapse._sweep_roof_
    # props`, plan time) as much as any ordinary fracture chip. A missing
    # import degrades to a warning, never a hard failure of the bake.
    from disaster import fire_bake as fb                       # noqa: E402
except Exception as _exc:                                      # pragma: no cover
    fb = None
    print("[qarch] WARNING: disaster.fire_bake unavailable ({0}) — the "
          "airborne sweep is disabled for this run".format(_exc))

PARENT = "/World/stage/generated"
# 4, not 7: seed 7 rolls "no balconies" on every balcony-capable style
# (the planner draws one balcony mode per band), so the concrete families
# had nothing for `balcony_fail` to break. Measured host-side over seeds 1-12.
SEED = int(os.environ.get("ARCH_SEED") or "4")
OUT_DIR = os.environ.get(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetype"))
# THE 250 m MIX. Nothing taller than ~60 m: an 88-103 m skyscraper on a
# 250 m plate is a landmark, not a district, and its ladder is glass loss
# anyway. Eight families are represented.
DEFAULT_STYLES = ("apartment", "office", "brownstone", "commercial", "tower",
                  "office_wide", "office_plain", "apartment_tall",
                  "apartment_long", "walkup", "brownstone_row",
                  "commercial_mid", "department_store", "dw_terrace",
                  "civic_offices", "highrise_04")
# THE 1 km MIX. The 14 styles `urban_building` kitbashes that the 250 m mix
# leaves on the floor — every skyscraper, every high-rise above 41 m, the slab
# office, all four full-block massings, the long storefront terrace, the
# government hall and the church. On a 1 km plate the argument above inverts:
# a 103 m tower is a DISTRICT CORE, not a landmark, and their absence is what
# "the diversity in buildings just isn't there" was about. Nothing here is
# baked yet — see the urban-layout skill.
TALL_STYLES = ("skyscraper_a", "skyscraper_b", "skyscraper_c", "highrise_step",
               "highrise_01", "highrise_02", "office_slab", "block_residential",
               "block_office", "block_stone", "block_commercial",
               "dw_terrace_long", "civic_hall", "church")
# Named groups for ARCH_STYLES, so a 1 km bake does not mean pasting a
# 30-item comma list into a shell and getting one of them wrong.
STYLE_GROUPS = {
    "mix250": DEFAULT_STYLES,          # the shipped default, unchanged
    "tall": TALL_STYLES,               # only what mix250 omits
    "full": DEFAULT_STYLES + TALL_STYLES,   # all 30 kitbashed styles
}
_req = os.environ.get("ARCH_STYLES", "").strip()
if _req in STYLE_GROUPS:
    STYLES = list(STYLE_GROUPS[_req])
else:
    STYLES = [q.strip() for q in (_req or ",".join(DEFAULT_STYLES)).split(",")
              if q.strip()]
GRADES = [q.strip() for q in os.environ.get(
    "ARCH_GRADES", "DG0,DG1,DG2,DG3,DG4,DG5,SETTLE,TILT,OV").split(",") if q.strip()]
VARIANTS = max(1, int(os.environ.get("ARCH_VARIANTS") or "1"))
SETTLE_STEPS = int(os.environ.get("SETTLE_STEPS") or "2200")
# ROUND 8 — SETTLE BODY BUDGET. User: "15000 bodies seems like too much, you
# wanna place some by hand or something." A style row's settle is ALL of its
# grades' loose bodies at once (see the module docstring above); a "block"
# style (`block_residential`, `block_office`, `block_stone`,
# `block_commercial` — a main mass plus several storeys-tall WINGS,
# `quake_collapse.collapse_masses`'s own docstring) multiplies every
# per-element/per-storey break population by both the mass count and each
# wing's own storey count, and a total/pancake grade on one of these hit
# 18,771 bodies and a 1.5 h settle that had to be killed
# (`quake_collapse.SETTLE_BODY_BUDGET_ENV`'s own module docstring has the
# full arithmetic). `qc.settle_body_budget()` reads the SAME env var this
# launcher does, because the budget has to live somewhere the GAC/sliced
# bake can reach it too later — only the archetype path actually calls it
# this round. `None` (env `SETTLE_BODY_BUDGET=-1`) is unlimited, i.e. today's
# behaviour, byte-for-byte.
SETTLE_BODY_BUDGET = qc.settle_body_budget()
# OPT-IN, NOT STRICT BY DEFAULT (round 7 — was strict-by-default in round 6).
# `steps` used to be a hard CEILING with no convergence loop and no failure
# mode: a body that could not reach a real rest inside the budget (a roof
# prop with a whole storey to fall on a 0.3-0.9 m/s kick,
# `disaster/quake_collapse.py`'s old `_sweep_roof_props`) simply froze
# mid-air and got exported — the manifest's own `still_moving` stat read 0
# ("converged") because that only measures 4 mm/step-chunk velocity at the
# end of a fixed budget, never whether anything is actually supported.
# `converge=True` (below, unchanged) makes `steps` a TARGET instead, run up
# to `settle.run`'s own `max_steps` (3x by default); `strict=SETTLE_STRICT`
# turns a settle that still could not converge into a raised
# `SettleNotConverged` instead of a silent warning.
#
# Round-6 shipped that strict by default, and round-7's re-bake measured the
# cost: 5 of 18 styles' DG3-5 piles do not converge within the 3x step
# budget (a big total-collapse heap with a genuinely long tail, not a bug
# each time), so strict-by-default raised on the FIRST one of those and
# killed the whole batch behind it — a third of the library never exported,
# reported as "0 failed" (see the `os._exit` note at the end of `main`, and
# the per-style try/except above that no longer lets one row's raise take
# the rest of the batch with it). `SETTLE_STRICT=1` opts back into the hard
# guarantee for a caller who would rather fail loudly than ship anything
# unconverged; `settle.run`'s own `strict=None` default reads the SAME env
# var the OPPOSITE way (opt IN to strict already, absent an override), so
# this driver still resolves it itself rather than leaving the ambiguous
# default. Either way, `deactivate_airborne` (the per-building sweep that runs
# right after each row's settle, further down in `main`) remains the floater
# backstop: a body that never converges is still swept out of the export if
# it is left hanging in the air, whether or not `SETTLE_STRICT` would also
# have raised on it.
SETTLE_STRICT = os.environ.get("SETTLE_STRICT", "").strip() in (
    "1", "true", "True")
# _o_ MERGE. ON for this driver unless told otherwise (`.env` leaks EMPTY
# strings for everything compose forwards, so `or "on"` and not a default
# argument). `both` also writes the UNMERGED file under `<ARCH_DIR>/_raw/`
# from the SAME settled geometry, which is the only honest before/after: a
# second bake would diverge on the rng and measure a different building.
MERGE = bake.merge_mode(os.environ.get("BAKE_MERGE", "").strip() or "on")


def build_ground_and_light(stage):
    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/arch_ground"))
    e = 2000.0
    plane.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                            Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    plane.CreateDisplayColorAttr([Gf.Vec3f(0.3, 0.3, 0.3)])
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2200.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))


def _rubble_fields(res):
    """Round 4: the rubble-v2 piles a recipe authored (`quake_flow._rubble`
    appends each `plan_pile` stats dict to `ctx["rubble"]`). The manifest
    record carries the UNION of the fall sides, the per-side MAX reach and
    the max crown so `quake._clear_under_heaps` reads where the pile really
    went instead of drawing a side (`quake._heap_reach_for`). Empty when the
    recipe authored no pile (DG1-DG3, foundation levels) or under
    `EQ_RUBBLE=v1`."""
    piles = res.get("rubble") or []
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
        # the MEASURED per-side run-out of the trimmed mound mesh (round-4
        # Isaac pass): what `quake._clear_under_heaps` prefers over `reach_m`
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


def _failure_fields(res):
    """Structural failure faces, kept separate from rubble fall direction.

    ``_rubble_fields`` intentionally unions every pile, including the shallow
    windrow made by a fallen parapet.  Calling that union "failed faces" made
    the 2-D review map promise missing façade on sides where the bake only
    contained curb-side parapet debris.  The collapse planner is the source
    of truth for actual shell/frame failure; record its sides independently.
    """
    sides, modes = [], []
    for plan in (res.get("quake_collapse") or []):
        mode = str(plan.get("mode") or "")
        if mode and mode not in modes:
            modes.append(mode)
        for side in (plan.get("sides") or []):
            if side in ("S", "E", "N", "W") and side not in sides:
                sides.append(side)
    if not sides and not modes:
        return {}
    return {"failure_sides": sides, "failure_modes": modes}


def merge_manifest(path, records):
    old = []
    if os.path.exists(path):
        try:
            old = bake.read_manifest(path)
        except Exception as exc:
            print("[qarch] existing manifest unreadable, replacing it: {0}".format(exc))
    fresh = set((r.get("style"), r.get("level")) for r in records)
    kept = [r for r in old if (r.get("style"), r.get("level")) not in fresh]
    return kept + records


def main():
    omni.timeline.get_timeline_interface().stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    _, ssf = get_stage_meters_per_unit(stage)
    build_ground_and_light(stage)
    fracture.ensure_deps()
    fracture.ensure_vtk(verbose=True)
    os.makedirs(OUT_DIR, exist_ok=True)
    t0 = time.time()
    problems = ub.check(verbose=False) + qf.check(verbose=False)
    if problems:
        raise RuntimeError("; ".join(problems))

    mats = qf.materials(stage, PARENT)
    cache = {}
    levels = []
    for g in GRADES:
        if g == "DG0":
            levels.append((g, "DG0"))
        else:
            for v in range(VARIANTS):
                levels.append((g, g if VARIANTS == 1 else "{0}_v{1}".format(g, v + 1)))
    # Grid: one row per style along +Y, one column per level along +X. The
    # pitch is per style — a department store is 42 x 30 m and a total
    # collapse spreads 0.5 H past that.
    records, miss = [], 0
    # ROUND 7: one style's `SettleNotConverged` (or anything else raised while
    # authoring/settling/exporting its row) used to kill the WHOLE process —
    # Kit reports that as rc=0 regardless (see the note by `os._exit` below),
    # so 6/15 real batch crashes shipped as "0 failed". Each style row now
    # runs in its own try/except (below) and a failure is recorded here
    # instead of stopping the styles after it.
    failed = []
    y = 0.0
    for si, st in enumerate(STYLES):
        spec = ub.STYLES[st]
        W, D = ub.footprint(spec)
        H = ub.height(spec)
        pitch = max(60.0, max(W, D) + 1.2 * H + 20.0)
        try:
            row = []
            timing = {}
            loose, static, vel = [], ["/World/arch_ground"], {}
            # path -> the ONE building scope (this style/grade's own
            # "a_{style}_{level}" parent) that authored it — SETTLE_BODY_
            # BUDGET's per-piece `_deck_support_z` query is scoped to this
            # rather than the whole row, since a piece never needs to land
            # on a DIFFERENT grade's own building 60+ m away on the grid.
            piece_root = {}
            # (style, level) -> this building's OWN `res["loose"]`, before
            # any settle-budget split — `z_outlier_sweep`'s `paths=`
            # allowlist, so it only ever groups genuine loose fracture
            # cells and never a static prop/shell mesh (a legitimate prop
            # repeated once per storey, same topology by DESIGN, would
            # otherwise collide with a real outlier's own family — measured
            # on `bld_brownstone_row_DG4.usd`: `prop_main_4_7`, `quake_
            # collapse.z_outlier_sweep`'s own docstring has the numbers).
            loose_by_building = {}
            for li, (grade, level) in enumerate(levels):
                X = li * pitch
                parent = "{0}/a_{1}_{2}".format(PARENT, st, level)
                UsdGeom.Scope.Define(stage, Sdf.Path(parent))
                pls = ub.build_building(st, X, y, 0.0, random.Random(SEED))
                sg.apply_placements(stage, pls, parent, ssf)
                ub.apply_glass_tint(stage, pls)
                paths = [p["prim_path"] for p in pls if p.get("prim_path")]
                if grade == "DG0":
                    # PRISTINE STILL GETS ITS ROOFTOP PLANT. `wreck_building` with
                    # no recipes only runs `dress_roof`; without it a DG0 roof is
                    # bare while its DG1 neighbour carries tanks and AC units.
                    res0 = qf.wreck_building(stage, parent, st, pls, X, y, 0.0, "DG0",
                                             random.Random(SEED + 1), np.random.default_rng(SEED + 1),
                                             mats, "{0}_{1}".format(st, level), mat_cache=cache)
                    static += paths + res0["static_extra"]
                    row.append((st, level, X, y, paths + res0["authored"]))
                    continue
                # Python's built-in hash is salted per process: two bakes
                # with the same ARCH_SEED previously produced different
                # failure faces and rubble, making one-building A/B review
                # impossible.  Use the fracture module's stable crc32 seed.
                seed = fracture.stable_seed(
                    "quake_archetype", SEED, st, level)
                tf0 = time.time()
                res = qf.wreck_building(stage, parent, st, pls, X, y, 0.0, grade,
                                        random.Random(seed), np.random.default_rng(seed),
                                        mats, "{0}_{1}".format(st, level), mat_cache=cache)
                loose += res["loose"]
                for _p in res["loose"]:
                    piece_root[_p] = parent
                loose_by_building[(st, level)] = list(res["loose"])
                static += res["static_extra"]
                vel.update(res["velocity"])
                everything = (paths + res["loose"] + res["static_extra"]
                              + res["authored"] + list(res["fit"]["all"]))
                rect = res.get("detached_rectangles") or {}
                timing[(st, level)] = dict(
                    fracture_s=round(time.time() - tf0, 1),
                    loose=len(res["loose"]),
                    detached_rectangles_replaced=int(rect.get("replaced", 0)),
                    detached_rectangle_pieces=int(rect.get("pieces", 0)),
                    detached_rectangle_violations=len(rect.get("violations") or ()),
                    damage_seed=seed,
                    **_failure_fields(res),
                    **_rubble_fields(res))
                row.append((st, level, X, y, everything))
                print("[qarch] {0:<16} {1:<7} {2:5d} loose {3:5d} static {4:5d} authored  {5:.0f} s"
                      .format(st, level, len(res["loose"]), len(res["static_extra"]),
                              len(res["authored"]), time.time() - tf0))
            for _ in range(5):
                omni.kit.app.get_app().update()
            settle_info = {}
            n_loose_authored = len(loose)
            budget_report = []
            if loose:
                budget_seed = fracture.stable_seed(
                    "quake_settle_budget", SEED, st)
                loose, geo, budget_report = qc.apply_settle_budget(
                    stage, loose, SETTLE_BODY_BUDGET,
                    root=lambda p: piece_root.get(p, PARENT),
                    ground_z=0.0, rng=random.Random(budget_seed))
                static += geo
                print("[qarch]   settle budget row {0} ({1}): {2} loose -> "
                      "{3} kept for physics (budget {4}) + {5} placed "
                      "geometrically".format(
                          si, st, n_loose_authored, len(loose),
                          SETTLE_BODY_BUDGET, len(geo)))
                # BUDGET=0 (or every piece placed geometrically): `loose` can
                # come back empty, so the settle below never runs and would
                # otherwise leave `settle_info` without a record of what
                # happened to this row at all — record it here too, the
                # settle-stats fields below simply overwrite these on top
                # when a settle does run.
                settle_info = dict(settle_bodies=len(loose),
                                   settle_budget=SETTLE_BODY_BUDGET,
                                   settle_budget_geometric=len(budget_report),
                                   settle_budget_authored=n_loose_authored)
            if loose:
                print("[qarch] settling row {0} ({1}): {2} bodies".format(si, st, len(loose)))
                ts0 = time.time()
                info = settle.run(stage, loose, static, steps=SETTLE_STEPS, kick=0.12,
                                  rng=random.Random(SEED + si), bake_result=True,
                                  velocity_map=vel, density=1900.0, max_speed=6.0,
                                  settle_note=True, converge=True,
                                  strict=SETTLE_STRICT)
                settle_info = dict(settle_s=round(time.time() - ts0, 1),
                                   settle_solve_s=round(float(info.get("solve_s", 0.0)), 1),
                                   settle_bodies=len(loose),
                                   settle_steps=int(info.get("steps_used", 0)),
                                   still_moving=int(info.get("still_moving", 0)),
                                   settle_budget=SETTLE_BODY_BUDGET,
                                   settle_budget_geometric=len(budget_report),
                                   settle_budget_authored=n_loose_authored)
                print("[qarch]   row {0} settle: {1}".format(si, settle_info))
            for _ in range(5):
                omni.kit.app.get_app().update()

            # -- nothing hangs in the air -----------------------------------
            # `fire_bake_launch_script.py`'s own call site: after the settle
            # and before the export, per BUILDING (`fb.BAKE_ROOT` is one
            # building there; here one row is several, so this loop is the
            # same check scoped to each building's own parent scope) — a
            # body the settle above could not bring down in SETTLE_STEPS
            # bakes wherever it froze, roof prop or ordinary fracture chip
            # alike.
            airborne_by = {}
            if fb is not None:
                row_airborne = 0
                for st_, level, X, Y, paths in row:
                    parent_path = "{0}/a_{1}_{2}".format(PARENT, st_, level)
                    try:
                        n_off = fb.deactivate_airborne(stage, parent_path,
                                                       verbose=False)
                    except Exception as exc:
                        print("[qarch] WARNING: deactivate_airborne failed for "
                              "{0}: {1}".format(parent_path, exc))
                        n_off = 0
                    if n_off:
                        airborne_by[(st_, level)] = n_off
                        row_airborne += n_off
                if row_airborne:
                    print("[qarch]   row {0} ({1}) airborne sweep: {2} "
                          "deactivated across {3} building(s)".format(
                              si, st, row_airborne, len(airborne_by)))
                for _ in range(2):
                    omni.kit.app.get_app().update()

            # -- one settled body a whole storey off its own siblings -------
            # `quake_collapse.z_outlier_sweep`, run right after the airborne
            # sweep above and scoped the same way (per BUILDING): a body the
            # settle brought down onto REAL geometry — so the points-based
            # airborne check above never flags it — but not the geometry its
            # own topology-identical siblings landed on. Measured on
            # `bld_brownstone_row_DG4.usd`'s `LOD0_108`: one wall-break
            # event's fracture cell (2762 pts / 2754 faces) sat at z~=17 on
            # the roof deck while its five siblings (same signature) landed
            # at z~=11 at the actual break line — a -0.07 m gap to the deck
            # it stopped on, which is exactly why no existing net caught it.
            # `paths=loose_by_building[...]` restricts the sweep to this
            # building's OWN loose fracture cells (never a static prop/
            # shell mesh — `z_outlier_sweep`'s own docstring has the
            # measured false positive, `prop_main_4_7`, that skipping this
            # would let through). A DG0 (or any grade with nothing loose)
            # has no entry at all and is skipped outright. `EQ_Z_OUTLIER=0`
            # disables the sweep; `z_outlier_sweep` itself is the no-op in
            # that case, so this call site needs no gate of its own.
            zo_reseated = zo_deactivated = 0
            for st_, level, X, Y, paths in row:
                building_loose = loose_by_building.get((st_, level))
                if not building_loose:
                    continue
                parent_path = "{0}/a_{1}_{2}".format(PARENT, st_, level)
                try:
                    zo = qc.z_outlier_sweep(stage, parent_path,
                                            paths=building_loose, verbose=False)
                except Exception as exc:
                    print("[qarch] WARNING: z_outlier_sweep failed for "
                          "{0}: {1}".format(parent_path, exc))
                    zo = {"reseated": 0, "deactivated": 0}
                zo_reseated += zo.get("reseated", 0)
                zo_deactivated += zo.get("deactivated", 0)
            if zo_reseated or zo_deactivated:
                print("[qarch] z-outlier sweep: {0} re-seated, {1} "
                      "deactivated".format(zo_reseated, zo_deactivated))
            for _ in range(2):
                omni.kit.app.get_app().update()

            # export the row now, so a crash later still leaves it on disk
            for st_, level, X, Y, paths in row:
                name = "bld_{0}_{1}.usd".format(st_, level)
                out = os.path.join(OUT_DIR, name)
                try:
                    raw_mb = raw_prims = None
                    if MERGE == "both":                                   # _o_
                        raw_dir = os.path.join(OUT_DIR, "_raw")
                        os.makedirs(raw_dir, exist_ok=True)
                        rout = os.path.join(raw_dir, name)
                        rs = {}
                        if bake.export_object(stage, None, paths, rout,
                                              recenter=(X, Y, 0.0), merge="off",
                                              stats_out=rs):
                            raw_mb = round(os.path.getsize(rout) / 1e6, 2)
                            raw_prims = rs.get("out_prims")
                    es = {}
                    if bake.export_object(stage, None, paths, out,
                                          recenter=(X, Y, 0.0),
                                          merge=("off" if MERGE == "off" else "on"),
                                          stats_out=es):
                        # A reusable archetype is itself a published cache,
                        # not merely an intermediate consumed by the final
                        # scene freeze.  Normalize every local/shared asset
                        # path before validation so opening this file directly
                        # from Nucleus on another machine cannot fall back to
                        # white materials or local-only rubble references.
                        out_layer = Sdf.Layer.FindOrOpen(out)
                        portable_paths = bake.normalize_archetype_asset_paths(
                            out_layer, verbose=False)
                        out_stage = Usd.Stage.Open(out_layer)
                        binding_apis = bake.apply_material_binding_api(
                            out_stage, verbose=False)
                        if portable_paths or binding_apis:
                            if not out_layer.Save():
                                raise RuntimeError(
                                    "failed to save portable metadata in " + out)
                        del out_stage
                        # The live author/settle passes protect new roof
                        # equipment, but the final by-material merge can hide
                        # several physical tanks/AC units inside one Mesh.
                        # Audit and repair those clusters against the actual
                        # EXPORTED support geometry before accepting the file.
                        # This is deterministic geometry work, not another
                        # simulation, and shifts only an unsupported cluster's
                        # point range (never every item sharing its material).
                        roof_plant_reseated = \
                            bake.reseat_roof_plant_clusters_in_file(
                                out, verbose=False)
                        m, ok, ms = bake.validate(out)
                        mb = round(os.path.getsize(out) / 1e6, 2)
                        print("[qarch]   {0:<34} {1:6.2f} MB  {2:6d} prims  "
                              "({3} src meshes -> {4} merged + {5} kept, {6} mats)"
                              .format(name, mb, es.get("out_prims", 0),
                                      es.get("src_meshes", 0), es.get("merged_prims", 0),
                                      es.get("kept_src", 0), es.get("materials", 0))
                              + ("" if raw_mb is None else
                                 "   [raw {0:.2f} MB / {1} prims]".format(raw_mb, raw_prims)))
                        records.append(dict(usd=os.path.abspath(out), meshes=m,
                                            bound_missing=ms, kind="bld", style=st_,
                                            level=level, W=W, D=D, H=H,
                                            family=spec.get("family"),
                                            type=qf.FAMILY_TYPE.get(spec.get("family")),
                                            mb=mb, prims=es.get("out_prims"),
                                            src_meshes=es.get("src_meshes"),
                                            merge=es.get("mode"),
                                            raw_mb=raw_mb, raw_prims=raw_prims,
                                            airborne_off=airborne_by.get(
                                                (st_, level), 0),
                                            roof_plant_reseated=len(
                                                roof_plant_reseated),
                                            portable_paths=portable_paths,
                                            binding_apis=binding_apis,
                                            **timing.get((st_, level), {}), **settle_info))
                        miss += ms
                except Exception as exc:
                    import traceback
                    traceback.print_exc()
                    print("[qarch] export FAILED for {0}: {1}".format(os.path.basename(out), exc))
            man_path = os.path.join(OUT_DIR, "archetypes.json")
            # read-merge-write under a file lock: bake_quake_headless.sh runs two
            # styles at once and the last writer would otherwise drop the other's
            # records
            import fcntl
            with open(man_path + ".lock", "w") as _lk:
                fcntl.flock(_lk, fcntl.LOCK_EX)
                bake.write_manifest(man_path, merge_manifest(man_path, records))
                fcntl.flock(_lk, fcntl.LOCK_UN)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            grades_str = ",".join(sorted(set(g for g, _lvl in levels)))
            print("[qarch] STYLE FAILED {0} {1}: {2}".format(st, grades_str, exc))
            failed.append({"style": st, "grades": grades_str, "error": str(exc)})
        y += pitch + 20.0

    dt = time.time() - t0
    sz = sum(os.path.getsize(r["usd"]) for r in records if os.path.exists(r["usd"])) / 1e6
    print("\n" + "=" * 72)
    print("QUAKE ARCHETYPE BAKE")
    print("  {0} archetype(s) ({1} styles x {2} levels) -> {3}".format(
        len(records), len(STYLES), len(levels), OUT_DIR))
    print("  {0:.0f} MB, {1:.0f} s, {2} unresolved material(s)".format(sz, dt, miss))
    print("=" * 72 + "\n")

    n_ok = len(STYLES) - len(failed)
    fail_names = ", ".join(f["style"] for f in failed)
    print("[qarch] batch summary: {0} ok / {1} failed{2}".format(
        n_ok, len(failed), " ({0})".format(fail_names) if failed else ""))

    # A companion to `archetypes.json`, not a replacement: `quake.load_
    # manifest` indexes every record in that file by (style, level) and
    # would either KeyError or silently mis-key on a summary row with no
    # "style"/"level", so this run's ok/failed tally lives in its OWN file
    # next to it instead of inside the manifest.
    summary_path = os.path.join(OUT_DIR, "archetypes_run_summary.json")
    try:
        import json
        with open(summary_path, "w") as _sf:
            json.dump(dict(styles=list(STYLES), grades=GRADES, ok=n_ok,
                           failed=failed, seconds=round(dt, 1)), _sf, indent=1)
    except Exception as exc:
        print("[qarch] WARNING: could not write {0}: {1}".format(summary_path, exc))

    app = omni.kit.app.get_app()
    # headless: exit once the manifest is written (KEEP_OPEN=1 to stay)
    if (os.environ.get("KEEP_OPEN", "").strip() == "1"
             or os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower() not in ("1", "true", "yes")):
        while simulation_app.is_running():
            app.update()
    simulation_app.close()
    if failed:
        # KIT SWALLOWS AN UNCAUGHT PYTHON EXCEPTION AND EXITS 0 — that is
        # the round-7 bug this file exists to fix (`SettleNotConverged`
        # crashed 6/15 batches and every one of them still reported rc=0;
        # the traceback only shows up as an `[omni.kit.app._impl] [py
        # stderr]` log line, which is Kit's OWN script-runner catching it,
        # logging it, and then shutting the app down cleanly regardless).
        # A `sys.exit(1)` here raises `SystemExit`, which is STILL just an
        # exception — if Kit's wrapper is the broad `except:`/`except
        # BaseException:` its own log line implies, it swallows that
        # exactly the same way and the caller is back to rc=0. `os._exit()`
        # is the raw `_exit()` syscall: no exception is raised, so there is
        # no handler anywhere — Kit's or Python's own atexit machinery — to
        # intercept it, which makes it the only one of the two verified (by
        # elimination, from the log evidence above) to still reach the
        # container's own exit status. `simulation_app.close()` already ran
        # above so this only forces the FINAL exit code, not the shutdown
        # itself.
        os._exit(1)


if __name__ == "__main__":
    main()
