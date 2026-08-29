#!/usr/bin/env python
"""
Urban fire CITY, 250 x 250 m — a downtown block grid on the good building
stock, with a fire spreading through it.

    UF_REGION=250 UF_ELAPSED=140 SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/city250 \
    ISAAC_SIM_SCRIPT_NAME=urban_fire_city250_launch_script.py airstack up isaac-sim

THE STOCK, AND WHAT IS DELIBERATELY NOT IN IT
----------------------------------------------
From the Nucleus survey (`_plans/nucleus_asset_survey.json`), with three
exclusions all made on review:

  * the 112 `selected_citydemo` MONOLITHS are out. They are photographs on
    8-to-90-point boxes; the greeble route that would have given them depth
    was dropped ("this looks weird ... let's not do this greebled method").
  * `RuinedModernBuildings` is out — already-collapsed buildings are a
    different event from a fire.
  * the CHURCH style is out.

What is left, and what each kind can actually take:

  KIT (`urban_building`, 5 ModernCityEnvironment01 families + the
  Downtown_West storefront kit) — assembled from façade MODULES, so
  `urban_fire.burn_building` can do the full ladder on them: gutted
  interiors, roof burn-through, floor failure, partial collapse.

  WHOLE BUILDINGS (ModernCityEnvironment x4, DownTown BG x6) — single
  high-detail meshes with no elements, so they take
  `urban_fire.burn_monolith`: their own materials sooted in place, soot
  plumes rooted on the measured wall, flame out of the burning band, charred
  roof. No gutting — there is nothing there to take apart.

UNITS ARE PER PACK AND USD DOES NOT CONVERT. DownTown is authored in
centimetres (`metersPerUnit` 0.01) and three of the four
ModernCityEnvironment buildings have centimetre POINTS with no scale op.
`SCALE` carries the correction; without it a block arrives 100x and one
building fills the plate.

Env:
    UF_REGION    plate size, m (default 250)
    UF_ELAPSED   minutes since ignition (default 140)
    UF_WIND      "<deg>,<mps>", the direction it blows TOWARD (default 20,7)
    UF_SEED      (default 21)
    UF_FLOW      1 authors NVIDIA Flow and the flames
    UF_KIT_SHARE share of the plate from the modular kit (default 0.55)
    UF_MAX_BLDG  ceiling on buildings placed (default 44)
    SETTLE_STEPS physics ceiling (default 2600)
    SNAP_DIR     captures, under /isaac-sim/.nvidia-omniverse/logs/
"""

import math
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(n, d=""):
    v = os.environ.get(n)
    return d if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
simulation_app = SimulationApp(launch_config={"headless": _HEADLESS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
enable_extension("omni.flowusd")

import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade        # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

import numpy as np                                             # noqa: E402
import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import urban_building as ub                        # noqa: E402
from disaster import fire as fx                                # noqa: E402
from disaster import fracture, settle                          # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402
from disaster import urban_fire_spread as ufs                  # noqa: E402

PARENT = "/World/stage/generated"
NUC = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"

# (name, rel_url, W, D, H, unit_scale) — sizes are the MEASURED ones after
# the scale is applied (`_plans/nucleus_asset_survey.json`).
WHOLE = [
    ("MBuilding01", "ModernCityEnvironment/Collected_Building01/"
     "SM_MERGED_BP_MBuilding01.usd", 28.5, 18.5, 29.0, 1.0),
    ("MBuilding02", "ModernCityEnvironment/Collected_Building02/"
     "SM_MERGED_BP_MBuilding02.usd", 91.1, 96.1, 68.7, 0.01),
    ("MBuilding03", "ModernCityEnvironment/Collected_Building03/"
     "SM_MERGED_BP_MBuilding03.usd", 15.6, 24.3, 22.4, 0.01),
    ("MBuilding05", "ModernCityEnvironment/Collected_Building05/"
     "SM_MERGED_BP_MBuilding05.usd", 27.7, 20.6, 62.4, 0.01),
    ("BG_Building_A", "DownTown/Assets/BG_Building_A.usd", 51.3, 51.3, 44.7, 0.01),
    ("BG_Building_B", "DownTown/Assets/BG_Building_B.usd", 60.3, 40.2, 84.1, 0.01),
    ("BG_Building_C", "DownTown/Assets/BG_Building_C.usd", 50.1, 40.1, 66.0, 0.01),
    ("BG_Building_D", "DownTown/Assets/BG_Building_D.usd", 64.9, 34.9, 33.0, 0.01),
    ("BG_Building_E", "DownTown/Assets/BG_Building_E.usd", 60.5, 36.5, 131.0, 0.01),
    ("BG_Building_F", "DownTown/Assets/BG_Building_F.usd", 80.9, 50.4, 72.3, 0.01),
]

# The modular kit — every family, church excluded. These are the ones that
# can be gutted, holed and collapsed.
KIT = ["brownstone", "walkup", "apartment", "apartment_tall", "commercial",
       "commercial_mid", "highrise_04", "highrise_01", "office",
       "office_plain", "civic_offices", "highrise_02", "tower",
       "dw_terrace", "brownstone_row", "apartment_long", "dw_terrace_long"]

REGION = float(_env("UF_REGION", "250"))
ELAPSED_MIN = float(_env("UF_ELAPSED", "140"))
SEED = int(_env("UF_SEED", "21"))
SETTLE_STEPS = int(_env("SETTLE_STEPS", "2600"))
SNAP_DIR = _env("SNAP_DIR", "")
FLOW = _env("UF_FLOW", "1") not in ("0", "false")
KEEP_PHYSICS = _env("KEEP_PHYSICS", "0") not in ("0", "false")
KIT_SHARE = float(_env("UF_KIT_SHARE", "0.55"))
MAX_BLDG = int(_env("UF_MAX_BLDG", "44"))
_w = [q.strip() for q in _env("UF_WIND", "20,7").split(",") if q.strip()]
WIND_DEG, WIND_MPS = float(_w[0]), (float(_w[1]) if len(_w) > 1 else 7.0)

STREET = 9.0          # half width of the two main streets
GAP = 2.5             # nominal gap between neighbours in a row
EDGE = 3.0


def _dims(e):
    if e["kind"] == "kit":
        W, D = ub.footprint(ub.STYLES[e["style"]])
        return W, D, ub.height(ub.STYLES[e["style"]])
    return e["W"], e["D"], e["H"]


def _overlaps(a, b, gap):
    """AABB test. Every yaw here is a multiple of 90 deg, so a footprint is
    axis-aligned once its W and D have been swapped for the two side-on
    orientations — which `_slot` has already done."""
    return (abs(a[0] - b[0]) * 2.0 < (a[2] + b[2] + 2.0 * gap) and
            abs(a[1] - b[1]) * 2.0 < (a[3] + b[3] + 2.0 * gap))


def plan_city(rng):
    """Four blocks round a central cross, buildings packed on ALL FOUR sides.

    The BLOCK is the unit, not the row: a downtown has buildings round the
    whole perimeter of a block, and the gaps between them are what the fire
    crosses.

    FILLED IN ROUNDS, NOT FRONTAGE BY FRONTAGE. The first cut walked the
    frontage list in order and stopped when the pool ran out, so 22 buildings
    filled the first three frontages and left five sixths of the plate bare:
    the city read as one clump in a corner of a 250 m field rather than as a
    grid (`city_top.png`, 2026-08-28). Placing one building per frontage per
    round spreads the same stock evenly, and CYCLING the pool means the
    frontages fill to their real capacity instead of to the pool's length —
    a downtown repeats its stock anyway, and a repeat lands at a different
    fire level, a different yaw and a different neighbour.
    """
    half = REGION / 2.0
    pool = []
    for st in KIT:
        pool.append({"kind": "kit", "style": st})
    for (nm, rel, W, D, H, sc) in WHOLE:
        pool.append({"kind": "whole", "style": nm, "usd": rel,
                     "W": W, "D": D, "H": H, "scale": sc})
    # interleave so a frontage is a mix of stock, not a run of one
    kits = [e for e in pool if e["kind"] == "kit"]
    whole = [e for e in pool if e["kind"] == "whole"]
    rng.shuffle(kits)
    rng.shuffle(whole)
    mixed = []
    while kits or whole:
        if kits and (not whole or rng.random() < KIT_SHARE):
            mixed.append(kits.pop(0))
        elif whole:
            mixed.append(whole.pop(0))

    # (axis, frontage line, inward sign, along-lo, along-hi, cursor)
    fronts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            bx0, bx1 = ((STREET, half - EDGE) if sx > 0
                        else (-half + EDGE, -STREET))
            by0, by1 = ((STREET, half - EDGE) if sy > 0
                        else (-half + EDGE, -STREET))
            fronts.append(["y", by0, +1, bx0, bx1, bx0 + 1.0])
            fronts.append(["y", by1, -1, bx0, bx1, bx0 + 1.0])
            fronts.append(["x", bx0, +1, by0, by1, by0 + 1.0])
            fronts.append(["x", bx1, -1, by0, by1, by0 + 1.0])
    rng.shuffle(fronts)

    placed, boxes, k, rounds = [], [], 0, 0
    alive = list(range(len(fronts)))
    while alive and len(placed) < MAX_BLDG and rounds < 40:
        rounds += 1
        nxt = []
        for fi in alive:
            if len(placed) >= MAX_BLDG:
                break
            axis, line, sgn, a0, a1, t = fronts[fi]
            got = False
            # up to a few candidates: a building that will not fit in what is
            # left of this frontage must not stop the ones behind it
            for _ in range(10):
                e = mixed[k % len(mixed)]
                W, D, H = _dims(e)
                # A BUILDING ALWAYS PRESENTS ITS WIDTH TO THE STREET, on every
                # frontage — the axis only decides which WORLD axis the width
                # runs along. The first cut swapped them on the N-S frontages
                # (`along, deep = (D, W)`), which both mis-sized the packing
                # rectangle — 15 overlapping pairs in the dry run — and, with
                # the yaw below, stood those buildings back-to-front.
                if t + W > a1 - 1.0:
                    k += 1
                    continue
                if axis == "y":
                    cx, cy = t + W / 2.0, line + sgn * (D / 2.0)
                    # the front is -Y local, so yaw 0 faces -Y and 180 faces +Y
                    yaw = 0.0 if sgn > 0 else 180.0
                    box = (cx, cy, W, D)
                else:
                    cy, cx = t + W / 2.0, line + sgn * (D / 2.0)
                    # +90 about Z maps the -Y front to +X, so a building set
                    # EAST of its frontage line (sgn +1, street to its west)
                    # needs 270, not 90. Backwards on the first cut.
                    yaw = 270.0 if sgn > 0 else 90.0
                    box = (cx, cy, D, W)
                # THE CORNERS ARE SHARED. Two frontages of the same block meet
                # there, and a 50 m-deep block placed on one reaches straight
                # through the other's first slot.
                if any(_overlaps(box, b, 1.2) for b in boxes):
                    t += 3.0
                    fronts[fi][5] = t
                    continue
                placed.append(dict(e, x=cx, y=cy, yaw=yaw, W=W, D=D, H=H))
                boxes.append(box)
                fronts[fi][5] = t + W + rng.uniform(0.6, GAP)
                k += 1
                got = True
                break
            # A FRONTAGE RETIRES WHEN IT IS FULL, NOT WHEN ONE ROUND MISSES.
            # Dropping it on a failed round cost a third of the city: the
            # first slot of a frontage is exactly where a deep neighbour on
            # the adjoining edge sits, so the corner rejection retired the
            # frontage before it had placed anything past the corner.
            del got
            if fronts[fi][5] < a1 - 8.0:
                nxt.append(fi)
        alive = nxt
    return placed


def ground_and_light(stage, span):
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(700.0)
    dome.CreateColorAttr(Gf.Vec3f(0.72, 0.76, 0.86))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(3200.0)
    key.CreateAngleAttr(0.9)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.86))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-27.0, 0.0, 30.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = max(500.0, span * 1.8)
    g.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                        Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.28, 0.27)])
    g.CreateExtentAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, e, 0)])
    try:
        mp = stage.DefinePrim(Sdf.Path("/World/Looks/asphalt"))
        mp.GetReferences().AddReference(sg._join_asset_root(
            "airstack://scene_gen/assets/materials/megascans/"
            "Road_Asphalt.usda", ""))
        mp.Load()
        m = UsdShade.Material.Get(stage, "/World/Looks/asphalt")
        if m:
            UsdShade.MaterialBindingAPI(g.GetPrim()).Bind(m)
    except Exception as exc:
        print("[city250] ground material: {0}".format(exc))


def main():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    _, ssf = get_stage_meters_per_unit(stage)
    fracture.ensure_deps()
    fracture.ensure_vtk(verbose=True)
    t0 = time.time()
    problems = ub.check(verbose=False) + qf.check(verbose=False) \
        + uf.check(verbose=False) + ufs.check(verbose=False)
    if problems:
        raise RuntimeError("; ".join(problems))

    rng = random.Random(SEED)
    placed = plan_city(rng)
    ground_and_light(stage, REGION)
    if FLOW:
        fx.setup_flow_stack(stage, density_cell_size_m=0.10, max_blocks=32768,
                            scene_scale_factor=ssf)
    flow_root = fx.FLOW_ROOT if FLOW else None

    # 1) place the KIT buildings now; the whole ones are referenced by
    #    `burn_monolith`, which is also what puts an untouched one on stage.
    cols = []
    for i, e in enumerate(placed):
        parent = "{0}/b{1}".format(PARENT, i)
        UsdGeom.Scope.Define(stage, Sdf.Path(parent))
        rec = dict(e, i=i, parent=parent, pls=[])
        if e["kind"] == "kit":
            rec["pls"] = ub.build_building(e["style"], e["x"], e["y"],
                                           e["yaw"], random.Random(SEED + 7 * i))
            sg.apply_placements(stage, rec["pls"], parent, ssf)
        cols.append(rec)
    ub.apply_glass_tint(stage, [p for c in cols for p in c["pls"]])
    for _ in range(6):
        omni.kit.app.get_app().update()
    n_kit = sum(1 for c in cols if c["kind"] == "kit")
    print("[city250] {0:.0f} m plate: {1} building(s) — {2} kit, {3} whole"
          .format(REGION, len(cols), n_kit, len(cols) - n_kit))

    # 2) the spread solve sets every severity
    bl = [{"x": c["x"], "y": c["y"], "W": c["W"], "D": c["D"], "yaw": c["yaw"],
           "H": c["H"], "style": c["style"], "kind": c["kind"]} for c in cols]

    def _btype(b):
        if b["kind"] == "kit":
            return qf.FAMILY_TYPE.get(
                ub.STYLES[b["style"]].get("family"), "urm")
        # the whole buildings are concrete/steel commercial stock
        return "rc"

    wx, wy = math.cos(math.radians(WIND_DEG)), math.sin(math.radians(WIND_DEG))
    ign = min(range(len(bl)),
              key=lambda k: bl[k]["x"] * wx + bl[k]["y"] * wy)
    plan = ufs.solve(bl, ign, ELAPSED_MIN * 60.0,
                     wind_dir=math.radians(WIND_DEG), wind_mps=WIND_MPS,
                     rng=random.Random(SEED + 991), btype_of=_btype)
    print("[city250] ignition {0} ({1}), wind {2:.0f} deg @ {3:.0f} m/s, "
          "T+{4:.0f} min".format(ign, bl[ign]["style"], WIND_DEG, WIND_MPS,
                                 ELAPSED_MIN))
    for ln in ufs.summarise(bl, plan, ELAPSED_MIN * 60.0):
        print("[city250] " + ln)

    # 3) burn
    mats = uf.materials(stage, PARENT)
    cache = {}
    loose, static, vel = [], ["/World/ground"], {}
    tally = {}
    for c, p in zip(cols, plan):
        c["plan"] = p
        tally[p["level"]] = tally.get(p["level"], 0) + 1
        brng = random.Random(SEED + 101 * c["i"])
        nrng = np.random.default_rng(SEED + 101 * c["i"])
        entry = p["entry_side"] or brng.choice(("S", "E", "N", "W"))
        tb = time.time()
        if c["kind"] == "whole":
            dims = {"W": c["W"], "D": c["D"], "H": c["H"],
                    "cx": 0.0, "cy": 0.0, "zmin": 0.0}
            res = uf.burn_monolith(
                stage, c["parent"], NUC + c["usd"], c["x"], c["y"], c["yaw"],
                dims, p["level"], brng, nrng, mats, "b{0}".format(c["i"]),
                flow_root=flow_root, entry_side=entry,
                origin_frac=p["origin_frac"], ssf=ssf,
                asset_scale=c["scale"])
        else:
            n_st = max(1, len(qf._mass_specs(c["style"], c["x"], c["y"],
                                             c["yaw"])[0]["levels"]))
            origin = max(0, min(n_st - 1,
                                int(round(p["origin_frac"] * (n_st - 1)))))
            order = ("S", "E", "N", "W")
            sides = ((entry,) if p["level"] in ("F1", "F2")
                     else (entry, order[(order.index(entry) + 1) % 4]))
            res = uf.burn_building(
                stage, c["parent"], c["style"], c["pls"], c["x"], c["y"],
                c["yaw"], p["level"], brng, nrng, mats,
                "b{0}".format(c["i"]), flow_root=flow_root, origin=origin,
                sides=sides, mat_cache=cache)
        loose += res["loose"]
        static += res["static_extra"]
        vel.update(res["velocity"])
        c["fire"] = res["fire"]
        print("[city250] {0:<17} {1:<5} {2} {3:4d} loose ({4:.0f} s)".format(
            c["style"][:17], c["kind"], p["level"], len(res["loose"]),
            time.time() - tb))
    for _ in range(10):
        omni.kit.app.get_app().update()

    if loose:
        # `ccd` + `ground_plane_z` + `floor_z`: `/World/ground` (below) is a
        # flat four-vertex quad, exactly the shape `settle.py`'s own module
        # docstring names as tunnel-prone at speed. Without a real
        # half-space and CCD a fast/thin piece can pass clean through it in
        # one 60 Hz step and never generate a contact — measured on the
        # identical settle call in `urban_fire_bench_launch_script.py`
        # (uf_bench_ref, 2026-08-29): 1 body STILL MOVING at bake time,
        # 454.69 m of horizontal "spread", more than this settle's own
        # max_speed cap can produce in the time budget unless the body lost
        # contact with the world. `floor_z=0.0` is the belt: anything still
        # under grade is clamped back onto it before the bake.
        settle.run(stage, loose, static, steps=SETTLE_STEPS, kick=0.10,
                   rng=random.Random(SEED), bake_result=not KEEP_PHYSICS,
                   velocity_map=vel, density=1600.0, max_speed=6.0,
                   converge=True, max_steps=int(SETTLE_STEPS * 2.5),
                   quiet_steps=60, ccd=True, ground_plane_z=0.0,
                   floor_z=0.0)
    for _ in range(10):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 78)
    print("URBAN FIRE CITY  {0:.0f} x {0:.0f} m   T+{1:.0f} min   wind {2:.0f} "
          "deg @ {3:.0f} m/s".format(REGION, ELAPSED_MIN, WIND_DEG, WIND_MPS))
    print("  {0} buildings ({1} modular kit, {2} whole), {3} distinct assets"
          .format(len(cols), n_kit, len(cols) - n_kit,
                  len({c["style"] for c in cols})))
    print("  mix " + ", ".join("{0}:{1}".format(k, tally[k])
                               for k in sorted(tally)))
    print("  {0} loose bodies, {1:.0f} s".format(len(loose), time.time() - t0))
    print("=" * 78 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            if FLOW:
                timeline.play()
                for _ in range(340):
                    omni.kit.app.get_app().update()
            tall = max(c["H"] for c in cols)
            # NADIR MUST CLEAR THE TALLEST ROOF — `overview` frames off the
            # height ABOVE what it looks at, so a 131 m tower on the plate
            # turns the plumb shot into a close-up of one roof.
            span = REGION * 1.35
            sn.place_camera(stage, (0.0, 0.0, tall + span / 1.164),
                            (0.0, 0.0, 0.0))
            sn.snapshot(os.path.join(SNAP_DIR, "city_top.png"))
            for nm, (ax, ay) in (("sw", (-1, -1)), ("se", (1, -1)),
                                 ("ne", (1, 1)), ("nw", (-1, 1))):
                d = REGION * 1.05
                sn.place_camera(stage, (ax * d, ay * d, 0.55 * d + tall * 0.4),
                                (0.0, 0.0, tall * 0.30))
                sn.snapshot(os.path.join(SNAP_DIR, "city_" + nm + ".png"))
            for nm, eye, tgt in (
                    ("street_ew", (-REGION * 0.62, 0.0, 4.0),
                     (REGION * 0.4, 0.0, 16.0)),
                    ("street_ns", (0.0, -REGION * 0.62, 4.0),
                     (0.0, REGION * 0.4, 16.0))):
                sn.place_camera(stage, eye, tgt)
                sn.snapshot(os.path.join(SNAP_DIR, nm + ".png"))
            print("[city250] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[city250] snapshots FAILED: {0}".format(exc))

    print("CITY250 DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        timeline.play()
        while simulation_app.is_running():
            app.update()
        timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
