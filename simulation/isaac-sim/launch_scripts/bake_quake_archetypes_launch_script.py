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
    y = 0.0
    for si, st in enumerate(STYLES):
        spec = ub.STYLES[st]
        W, D = ub.footprint(spec)
        H = ub.height(spec)
        pitch = max(60.0, max(W, D) + 1.2 * H + 20.0)
        row = []
        timing = {}
        loose, static, vel = [], ["/World/arch_ground"], {}
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
            seed = SEED + (abs(hash((st, level))) % 100000)
            tf0 = time.time()
            res = qf.wreck_building(stage, parent, st, pls, X, y, 0.0, grade,
                                    random.Random(seed), np.random.default_rng(seed),
                                    mats, "{0}_{1}".format(st, level), mat_cache=cache)
            loose += res["loose"]
            static += res["static_extra"]
            vel.update(res["velocity"])
            everything = (paths + res["loose"] + res["static_extra"]
                          + res["authored"] + list(res["fit"]["all"]))
            timing[(st, level)] = dict(fracture_s=round(time.time() - tf0, 1),
                                       loose=len(res["loose"]))
            row.append((st, level, X, y, everything))
            print("[qarch] {0:<16} {1:<7} {2:5d} loose {3:5d} static {4:5d} authored  {5:.0f} s"
                  .format(st, level, len(res["loose"]), len(res["static_extra"]),
                          len(res["authored"]), time.time() - tf0))
        for _ in range(5):
            omni.kit.app.get_app().update()
        settle_info = {}
        if loose:
            print("[qarch] settling row {0} ({1}): {2} bodies".format(si, st, len(loose)))
            ts0 = time.time()
            info = settle.run(stage, loose, static, steps=SETTLE_STEPS, kick=0.12,
                              rng=random.Random(SEED + si), bake_result=True,
                              velocity_map=vel, density=1900.0, max_speed=6.0,
                              settle_note=True)
            settle_info = dict(settle_s=round(time.time() - ts0, 1),
                               settle_solve_s=round(float(info.get("solve_s", 0.0)), 1),
                               settle_bodies=len(loose),
                               settle_steps=int(info.get("steps_used", 0)),
                               still_moving=int(info.get("still_moving", 0)))
            print("[qarch]   row {0} settle: {1}".format(si, settle_info))
        for _ in range(5):
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
        y += pitch + 20.0

    dt = time.time() - t0
    sz = sum(os.path.getsize(r["usd"]) for r in records if os.path.exists(r["usd"])) / 1e6
    print("\n" + "=" * 72)
    print("QUAKE ARCHETYPE BAKE")
    print("  {0} archetype(s) ({1} styles x {2} levels) -> {3}".format(
        len(records), len(STYLES), len(levels), OUT_DIR))
    print("  {0:.0f} MB, {1:.0f} s, {2} unresolved material(s)".format(sz, dt, miss))
    print("=" * 72 + "\n")

    app = omni.kit.app.get_app()
    # headless: exit once the manifest is written (KEEP_OPEN=1 to stay)
    if (os.environ.get("KEEP_OPEN", "").strip() == "1"
             or os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower() not in ("1", "true", "yes")):
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
