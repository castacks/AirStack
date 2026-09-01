#!/usr/bin/env python
"""
Bake the HURRICANE archetype library — the three roof-damage states that the
tornado library does not contain.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_hurricane \
    TORNADO_ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
    ISAAC_SIM_SCRIPT_NAME=bake_hurricane_archetypes_launch_script.py \
    airstack up isaac-sim

WHY THIS IS A SMALL SCRIPT AND `bake_tornado_archetypes_launch_script.py` IS
A LARGE ONE
---------------------------------------------------------------------------
UPDATED 2026-08-31: this file used to LINK `roof_stripped` / `roof_collapsed`
/ `partial_collapse` / `leveled` out of the tornado library (`hurricane_flow.
BREAK_PLAN` referenced `wind_flow.BREAK_PLAN` for them). That library's
output is fracture+settle debris — a fan of roof triangles and wall shards
heaped on the lawn — which is a TORNADO signature, and the review found it
still reads as one for a hurricane, plus fragments FROZEN IN MID-AIR where a
truncated settle never finished (PhysX GPU dynamics does not engage in the
OSMO workspace container, below). All SIX non-trivial house rungs are now
built HERE, in this script, and NONE of them needs physics:

    shingles_lost      a few bays of covering gone, deck intact         )
    cover_lost         about half the covering gone, some windows in    ) SetActive(False)
    deck_panels_lost   most covering/sheathing gone, holes to the attic ) + rafter lattice
    roof_stripped      every bay gone, structure/walls whole            )
    roof_collapsed     every roof bay hinged 15-35 deg, drops inward    ) rigid POSE,
    partial_collapse   windward wall row racked 70-90 deg outward       ) no fracture,
    leveled            every wall racked flat, roof settled low on top  ) no settle

The first four are `SetActive(False)` on the kit's own per-bay roof meshes
plus a window-piece swap (`hurricane_flow.strip_roof` / `.blow_out_windows`),
now followed by `author_rafters` so a dropped bay does not read as an empty
box — a lost shingle course is gone-or-there, not crazed, so fracturing for
these would be both slower and wrong. The last three are rigid transform
edits to the walls/roof that already exist (`hurricane_flow.pose_roof_
collapsed` / `.pose_partial_collapse` / `.pose_leveled`) — no rigid body, no
settle, no PhysX. `ROOF_LEVELS` below covers all seven non-pristine,
non-swept rungs; `hf.wreck_building` dispatches each to the right one
internally.

`SHARED_LEVELS` is now just `pristine` (the assembly's own missing-archetype
fallback) and `swept` (never produced by this ladder — `disaster.surge` owns
it, see `_link_shared`).

MEASURED ON THIS POD, AND THE REASON THE SPLIT MATTERS HERE MORE THAN USUAL:
PhysX GPU dynamics does NOT engage in the OSMO workspace container. The
settle reports "GPU" (`settle.py:1011` prints the flag it was ASKED for, not
the backend it got) while `nvidia-smi` shows 0% utilisation and the solver
runs at ~1.6 CPU cores. A 6,863-body tornado house bake that the skill costs
at ~6 minutes on a GPU ran past 25 minutes here. Anything that can avoid the
settle should.

Env knobs:

    ARCH_DIR          output library (default `scene_gen/assets/archetypes_hurricane`)
    TORNADO_ARCH_DIR  library to take the shared rungs from; "" disables linking
    ARCH_SEED         seed for the bay/window draws (default 7)
    ARCH_STYLES       comma list of modular_house styles (default: all)
    HUR_LINK_MODE     `symlink` (default) or `copy`
    BAKE_STRICT       non-empty makes a missing shared rung fatal
"""

import json
import math
import os
import random
import shutil
import sys
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

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

import scene_generator as sg                                    # noqa: E402
from scene_prep import get_stage_meters_per_unit                # noqa: E402
from detail import modular_house as mh                          # noqa: E402
from disaster import bake                                       # noqa: E402
from disaster import planks                                     # noqa: E402
from disaster import hurricane as hu                            # noqa: E402
from disaster import hurricane_flow as hf                       # noqa: E402

PARENT = "/World/stage/generated"


def _env(name, default):
    """`os.environ.get` with EMPTY TREATED AS ABSENT — the compose file
    exports these declared-but-unset, so they arrive as `""` and the default
    is never reached. `ARCH_DIR` is the dangerous one: it does not raise, it
    silently writes the whole library into the container's CWD with a clean
    banner. Every knob here goes through this."""
    v = os.environ.get(name)
    return default if v is None or not v.strip() else v.strip()


SEED = int(_env("ARCH_SEED", "7"))
OUT_DIR = _env("ARCH_DIR",
               os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_hurricane"))
TOR_DIR = _env("TORNADO_ARCH_DIR",
               os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_tornado"))
LINK_MODE = _env("HUR_LINK_MODE", "copy")
BAKE_STRICT = bool(_env("BAKE_STRICT", ""))
_req = _env("ARCH_STYLES", "")
STYLES = ([s.strip() for s in _req.split(",") if s.strip()]
          if _req else list(mh.STYLES.keys()))

# The rungs this script BUILDS by SetActive-and-rafter (`hurricane_flow.
# _ROOF_FRAC`'s keys) or by rigid POSE (`hurricane_flow._POSE_LEVELS`) —
# read from the module rather than restated, so a new state there does not
# need an edit here. `roof_stripped` joined `_ROOF_FRAC` 2026-08-31 (it used
# to be linked from the tornado library, see the note below); `roof_
# collapsed` / `partial_collapse` / `leveled` joined this script the same
# day, replacing the tornado library's fracture+settle output for the same
# reason `roof_stripped` did: a fan of roof triangles and wall shards heaped
# on the lawn is a tornado signature, and it shipped fragments frozen in
# mid-air where a truncated settle never finished. NONE of the six needs
# `items`/`tag`/`nrng`/`planks_mats` — `hf.wreck_building` dispatches all of
# them internally (see its own docstring).
ROOF_LEVELS = tuple(hf._ROOF_FRAC.keys()) + hf._POSE_LEVELS

# The rungs this script LINKS from the tornado library. Down to two:
# `pristine` (the assembly falls back to it for any missing archetype, so a
# hurricane library without one is a library that renders empty lots) and
# `swept` (never produced by this file's own ladder at all — `disaster.
# surge` owns it, and it rides along only so a shared archetype directory
# has something to reference if surge ever wants the tornado's own swept
# geometry as a starting point).
SHARED_LEVELS = ("pristine", "swept")

GRID = 40.0     # no settle here, so nothing is thrown between cells; the
                # tornado baker's 50 m is sized for a 9 m/s debris throw.


def build_ground_and_light(stage):
    """A ground quad and a key light. The ground exists so the export's
    seating logic has a datum, not for the render — this bake is headless."""
    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/arch_ground"))
    e = 4000.0
    plane.CreatePointsAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, -e, 0.0),
                            Gf.Vec3f(e, e, 0.0), Gf.Vec3f(-e, e, 0.0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.34, 0.24)])
    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/arch_key"))
    light.CreateIntensityAttr(3000.0)
    light.CreateAngleAttr(0.8)


def _link_shared(out_dir, tor_dir, records):
    """Bring the shared rungs across from the tornado library.

    COPY BY DEFAULT SINCE 2026-08-30. It was symlink, on the argument that a
    link cannot drift — but the shared rungs need the same unbound-floor
    repair as the built ones (see `_bind_floors`), and post-processing a
    symlink would reach through and edit the TORNADO library. Set
    `HUR_LINK_MODE=symlink` to go back to sharing the files, and then the
    floor repair is skipped for those rungs.

    The original argument, kept because it is still the reason to prefer a
    link when nothing has to be rewritten:
    a symlink cannot drift. If the tornado library is re-baked the hurricane
    library follows it, which is what "referenced, not copied" already means
    one level up in `hurricane_flow.BREAK_PLAN`. `HUR_LINK_MODE=copy` gives a
    standalone directory for a dataset freeze, where drift is exactly what
    you want to prevent instead.
    """
    if not tor_dir or not os.path.isdir(tor_dir):
        print("[harch] no tornado library at {0} — shared rungs NOT linked. "
              "The assembly will fall back to live intact houses for every "
              "collapse level.".format(tor_dir))
        return 0
    n = miss = 0
    for st in STYLES:
        for lv in SHARED_LEVELS:
            name = "house_{0}_{1}.usd".format(st, lv)
            src, dst = os.path.join(tor_dir, name), os.path.join(out_dir, name)
            if not os.path.isfile(src):
                miss += 1
                continue
            if os.path.islink(dst) or os.path.isfile(dst):
                os.remove(dst)
            if LINK_MODE == "copy":
                shutil.copy2(src, dst)
            else:
                os.symlink(os.path.abspath(src), dst)
            records.append(dict(usd=os.path.abspath(dst), style=st, level=lv,
                                kind="house", source="tornado_library"))
            n += 1
    print("[harch] shared rungs: {0} linked ({1}), {2} missing"
          .format(n, LINK_MODE, miss))
    if miss and BAKE_STRICT:
        raise SystemExit("[harch] BAKE_STRICT: {0} shared rung(s) missing "
                         "from {1}".format(miss, tor_dir))
    return n


def _bind_floors(path):
    """Give a baked archetype's FLOOR meshes a material. Returns (seen, bound).

    THE BLUE SLABS. Measured on the first baked library: every
    `house_floor_*` mesh in every archetype — the ones this script builds AND
    the ones inherited from the tornado library — came out with NO MATERIAL
    BOUND AT ALL and `displayColor` (1, 1, 1), so RTX fell back to its default
    and rendered them a saturated blue. On an intact house nobody ever sees
    it, because the roof is over it. The moment a roof bay is dropped or a
    wall comes down, that floor plate is the largest surface in the frame and
    the house reads as having a bright blue interior.

    `bake.validate` did NOT catch this: it reported `bound_missing: 0` for all
    72 records, so whatever it checks does not include a mesh with no binding
    at all. Worth fixing there; this is the immediate repair.

    A hurricane-exposed floor is wet subfloor — sheathing and joists, pale
    sawn timber — so the material is built from the same `planks` vocabulary
    the loose debris uses, and a floor plate and a board off the same house
    agree with each other.

    Done by REOPENING THE WRITTEN FILE rather than on the live stage, because
    the shared rungs are never on this stage at all: they come from the
    tornado bake and only exist as files.
    """
    from pxr import Usd, UsdGeom, UsdShade, Sdf
    st = Usd.Stage.Open(path)
    if not st:
        return (0, 0)
    seen = [p for p in st.Traverse()
            if p.IsA(UsdGeom.Mesh) and "floor" in p.GetName().lower()]
    if not seen:
        return (0, 0)
    unbound = [p for p in seen
               if not UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]]
    if not unbound:
        return (len(seen), 0)
    mat = planks.wood_material(
        st, "/Baked/Looks/subfloor",
        # A floor plate is 5-10 m across; the 1.1 m board repeat reads as a
        # visible grid on it from above.
        tile_m=2.2, tint=(0.92, 0.88, 0.82), roughness=0.86)
    m = (UsdShade.Material.Get(st, Sdf.Path(mat))
         if isinstance(mat, str) else mat)
    n = 0
    for prim in unbound:
        try:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(m)
            n += 1
        except Exception:
            pass
    st.GetRootLayer().Save()
    return (len(seen), n)


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
    os.makedirs(OUT_DIR, exist_ok=True)
    app = omni.kit.app.get_app()

    # ARCH_LEVELS (comma list) scopes a rebake to specific rungs. Added
    # 2026-08-31: the three structural rungs are now TORNADO-CANON files
    # adopted by `hurricane_house_pose_bake.py --adopt-tornado`; an unscoped
    # rebake would silently overwrite them with rigid pose output. A palette
    # rebake therefore runs with
    #   ARCH_LEVELS=shingles_lost,cover_lost,deck_panels_lost,roof_stripped
    _lv_req = [x.strip() for x in _env("ARCH_LEVELS", "").split(",") if x.strip()]
    _levels = tuple(x for x in ROOF_LEVELS if not _lv_req or x in _lv_req)
    combos = [(st, lv) for st in STYLES for lv in _levels]
    ncol = max(1, int(math.ceil(math.sqrt(len(combos)))))
    print("[harch] building {0} roof-state archetype(s): {1} style(s) x {2}"
          .format(len(combos), len(STYLES), list(ROOF_LEVELS)))

    # THE WIND BEARING THE BAYS ARE STRIPPED AGAINST. Each archetype is built
    # at yaw 0 and the assembly yaws it to the street, so an authored wind
    # direction inside a per-style archetype comes out pointing wherever that
    # house happens to face — the same argument `wind_flow`'s header makes for
    # keeping the throw direction out of the bake. What IS preserved and does
    # matter is that the loss is CONTIGUOUS and starts at one eave rather than
    # being scattered over the roof, which is what `strip_roof`'s bias gives.
    _BAKE_BEARING = 0.0

    built = []
    t0 = time.time()
    for idx, (st, lv) in enumerate(combos):
        X, Y = (idx % ncol) * GRID, (idx // ncol) * GRID
        parent = "{0}/h_{1}_{2}".format(PARENT, st, lv)
        UsdGeom.Scope.Define(stage, Sdf.Path(parent))
        rng = random.Random(SEED + idx)
        pls = mh.build_building(st, X, Y, 0.0, random.Random(SEED),
                                category="house")
        # STAMP THE STYLE'S PALETTE ONTO THE PLACEMENTS BEFORE apply_palette,
        # exactly as `bake_tornado_archetypes_launch_script.py` does. Without
        # this `pl.get("palette")` is empty on every placement `build_building`
        # returns (it never stamps one itself — see its docstring and every
        # OTHER caller in `modular_house.py`: `build_catalogue`, `build_row`,
        # `build_plaza` and `suburb_scene.build_placements` all do this same
        # assignment themselves), so `apply_palette`'s `pal = PALETTES.get(
        # pl.get("palette") or "")` resolved `PALETTES.get("")` -> None for
        # every subset on every style and the call two lines down rebound
        # ZERO of them — silently: `apply_palette` has no "0 subsets" print.
        # Every baked hurricane house therefore wore the KIT'S OWN untouched
        # defaults regardless of style: `Cladding_01` walls, `Brick_01_Dirt`
        # gables, and `Roof_Tiles_01_Inst`/`Roof_Felt_01` roofs — the two roof
        # materials this module's own header calls "about a third acid-lime
        # moss" and "more than half moss" and says are "replaced everywhere",
        # which they were not. That is the uniform white/cream wall, repeated
        # brick gable and mossy shingle roof on every style in every render.
        pal = mh.STYLES[st].get("palette")
        if pal:
            for q in pls:
                q["palette"] = pal
        sg.apply_placements(stage, pls, parent, ssf)
        mh.apply_palette(stage, pls, parent)
        built.append((st, lv, X, Y, parent, pls))

    for _ in range(20):
        app.update()
    print("[harch] kit build in {0:.0f}s".format(time.time() - t0))

    # ---- the damage, such as it is: bays off/posed, windows in, doors gone -
    t1 = time.time()
    n_bay = n_posed = 0
    for st, lv, X, Y, parent, pls in built:
        prim = stage.GetPrimAtPath(Sdf.Path(parent))
        rng = random.Random(SEED + hash((st, lv)) % 9973)
        try:
            if lv in hf._ROOF_FRAC:
                # COUNT THE BAYS BEFORE AND AFTER RATHER THAN TRUST A RETURN
                # VALUE. `wreck_building` returns FRAGMENT PATHS, empty for
                # every level this branch handles (that is the whole point
                # of them) — so the only honest measure of whether the roof
                # was actually stripped is the active-bay count on the prim
                # itself. A bake that reports "12 styles done" while every
                # roof is still whole is precisely the silent failure this
                # pipeline has hit before.
                before = sum(1 for q in hf.roof_bay_prims(prim) if q.IsActive())
                hf.wreck_building(stage, prim, lv, rng,
                                  wind_bearing_deg=_BAKE_BEARING, items=pls)
                after = sum(1 for q in hf.roof_bay_prims(prim) if q.IsActive())
                n_bay += max(0, before - after)
                if before and before == after:
                    print("[harch] {0}/{1}: {2} bay(s) and NONE dropped — "
                          "the roof is untouched".format(st, lv, before))
            else:
                # `_POSE_LEVELS` — bays/walls stay ACTIVE (they are ROTATED,
                # not deactivated), so the bay-count check above is
                # meaningless here. `wreck_building` returns `[]` for these
                # too (no fracture), so the honest check is instead: did any
                # roof/wall transform actually change? Measured via a
                # transform snapshot, not `roof_bay_prims(...).IsActive()`.
                before_xf = {q.GetPath(): q.GetAttribute(
                    "xformOp:transform").Get()
                            for q in hf._meshes_of(prim, "house_roof")}
                hf.wreck_building(stage, prim, lv, rng,
                                  wind_bearing_deg=_BAKE_BEARING, items=pls)
                moved = sum(1 for path, m0 in before_xf.items()
                           if stage.GetPrimAtPath(path).GetAttribute(
                               "xformOp:transform").Get() != m0)
                n_posed += moved
                if before_xf and not moved:
                    print("[harch] {0}/{1}: {2} roof bay(s) and NONE posed "
                          "— the roof is untouched".format(
                              st, lv, len(before_xf)))
        except Exception as exc:
            print("[harch] {0}/{1} wreck FAILED: {2}".format(st, lv, exc))
    for _ in range(10):
        app.update()
    print("[harch] roof damage in {0:.0f}s: {1} bay(s) dropped, {2} bay(s) "
          "posed".format(time.time() - t1, n_bay, n_posed))

    # ---- export -----------------------------------------------------------
    records, miss = [], 0
    for st, lv, X, Y, parent, _pls in built:
        out = os.path.join(OUT_DIR, "house_{0}_{1}.usd".format(st, lv))
        est = {}
        try:
            ok = bake.export_object(stage, None, [parent], out,
                                    recenter=(X, Y, 0.0), stats_out=est)
        except Exception as exc:
            print("[harch] export {0}/{1} FAILED: {2}".format(st, lv, exc))
            ok = False
        if not ok:
            miss += 1
            continue
        m, _okc, ms = bake.validate(out)
        records.append(dict(usd=os.path.abspath(out), meshes=m,
                            bound_missing=ms, style=st, level=lv,
                            kind="house", source="hurricane_bake"))
    print("[harch] exported {0} roof archetype(s), {1} failed"
          .format(len(records), miss))

    _link_shared(OUT_DIR, TOR_DIR, records)

    # ---- repair the floors in EVERY archetype, built and inherited -------
    f_seen = f_bound = f_files = 0
    if LINK_MODE == "copy" or True:
        for rec in records:
            _p = rec["usd"]
            if os.path.islink(_p):
                continue          # never reach through into another library
            try:
                _s, _b = _bind_floors(_p)
            except Exception as exc:
                print("[harch] floor bind failed on {0}: {1}"
                      .format(os.path.basename(_p), exc))
                continue
            f_seen += _s
            f_bound += _b
            f_files += 1 if _b else 0
    print("[harch] floors: {0} mesh(es) across {1} file(s) were unbound and "
          "are now subfloor timber ({2} floor meshes seen)"
          .format(f_bound, f_files, f_seen))

    man = os.path.join(OUT_DIR, "archetypes.json")
    with open(man, "w") as fh:
        json.dump({"kinds": ["house"], "records": records}, fh, indent=1)
    print("[harch] manifest -> {0} ({1} record(s))".format(man, len(records)))
    print("[harch] ARCH_DONE")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
