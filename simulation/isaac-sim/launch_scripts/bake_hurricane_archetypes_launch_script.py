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
The hurricane house ladder is the tornado ladder EXTENDED DOWNWARD. Its top
four rungs are not merely similar to the tornado's — `hurricane_flow.BREAK_PLAN`
REFERENCES `wind_flow.BREAK_PLAN` for them rather than copying it, so
`roof_stripped`, `roof_collapsed`, `partial_collapse` and `leveled` are the
same archetype, and `swept` is produced by the same bake. Re-baking them here
would spend twenty minutes of solver time reproducing files that already exist
byte-for-byte in intent.

What the tornado library genuinely lacks is the bottom of the hurricane
ladder — the Cat 1-2 signature, and the majority outcome on a hurricane plate:

    shingles_lost      a few bays of covering gone, deck intact
    cover_lost         about half the covering gone, some windows in
    deck_panels_lost   most covering and sheathing gone, holes to the attic

AND NONE OF THE THREE NEEDS PHYSICS. They are `SetActive(False)` on the kit's
own per-bay roof meshes plus a window-piece swap — `hurricane_flow.strip_roof`
and `.blow_out_windows`, whose whole argument is that fracturing for these
would be slower AND look wrong (a lost shingle course is gone-or-there, not
crazed). So there is no fracture, no rigid body, no settle and no solver in
this file at all, and it runs in seconds rather than in tens of minutes.

That is also why this script SYMLINKS (or copies) the shared rungs out of the
tornado library instead of rebuilding them: see `_link_shared`.

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

# The rungs this script BUILDS. Exactly `hurricane_flow._ROOF_FRAC`'s keys —
# read from the module rather than restated, so adding a fourth roof state
# there does not need an edit here.
ROOF_LEVELS = tuple(hf._ROOF_FRAC.keys())

# The rungs this script LINKS from the tornado library. `pristine` is included
# because the assembly falls back to it for any missing archetype, so a
# hurricane library without one is a library that renders empty lots.
SHARED_LEVELS = ("pristine", "roof_stripped", "roof_collapsed",
                 "partial_collapse", "leveled", "swept")

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

    combos = [(st, lv) for st in STYLES for lv in ROOF_LEVELS]
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
        sg.apply_placements(stage, pls, parent, ssf)
        mh.apply_palette(stage, pls, parent)
        built.append((st, lv, X, Y, parent, pls))

    for _ in range(20):
        app.update()
    print("[harch] kit build in {0:.0f}s".format(time.time() - t0))

    # ---- the damage, such as it is: bays off, windows in, doors gone -------
    t1 = time.time()
    n_bay = n_win = 0
    for st, lv, X, Y, parent, pls in built:
        prim = stage.GetPrimAtPath(Sdf.Path(parent))
        rng = random.Random(SEED + hash((st, lv)) % 9973)
        # COUNT THE BAYS BEFORE AND AFTER RATHER THAN TRUST A RETURN VALUE.
        # `wreck_building` returns FRAGMENT PATHS, which are empty for every
        # level this script builds (that is the whole point of them) — so the
        # only honest measure of whether the roof was actually stripped is the
        # active-bay count on the prim itself. A bake that reports "12 styles
        # done" while every roof is still whole is precisely the silent
        # failure this pipeline has hit before.
        try:
            before = sum(1 for q in hf.roof_bay_prims(prim) if q.IsActive())
            hf.wreck_building(stage, prim, lv, rng,
                              wind_bearing_deg=_BAKE_BEARING, items=pls)
            after = sum(1 for q in hf.roof_bay_prims(prim) if q.IsActive())
            n_bay += max(0, before - after)
            if before and before == after:
                print("[harch] {0}/{1}: {2} bay(s) and NONE dropped — the "
                      "roof is untouched".format(st, lv, before))
        except Exception as exc:
            print("[harch] {0}/{1} wreck FAILED: {2}".format(st, lv, exc))
    for _ in range(10):
        app.update()
    print("[harch] roof damage in {0:.0f}s: {1} bay(s) dropped, {2} window(s) "
          "blown".format(time.time() - t1, n_bay, n_win))

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
