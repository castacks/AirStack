#!/usr/bin/env python
"""
Urban fire CITY — a 100 x 100 m downtown block with a fire spreading through
it, every building a different kit style.

    UF_REGION=100 UF_ELAPSED=80 SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/uf_city \
    ISAAC_SIM_SCRIPT_NAME=urban_fire_city_launch_script.py airstack up isaac-sim

WHAT MAKES THIS DIFFERENT FROM `urban_fire_bench_launch_script.py`
------------------------------------------------------------------
The bench stands buildings 60 m apart and gives each one a severity by hand,
because the question there is "does one burnt building read". Here they are a
metre or two apart and NOBODY sets a severity: `disaster.urban_fire_spread`
solves when the fire reached each building from ONE ignition point, and the
level, the burning elevation and the storey the fire got in at all fall out of
that solve. That is the whole point — a street of independently-assigned
severities reads as a row of unrelated fires, and a street whose severities
come from one spreading front reads as a conflagration.

Three things carry the read, and all three come from the solve:

  * A GRADIENT ALONG THE WIND. The origin is burnt out and cold, its
    neighbours are fully involved, the next ones are just alight, and the
    upwind end of the block is untouched.
  * THE FIRE FACES ITS SOURCE. Every building burns on the elevation looking
    at whichever building lit it (`entry_side`), so the fires all lean the
    same way down the street.
  * THE ENTRY HEIGHT SAYS THE MECHANISM. Wall-to-wall spread comes in low;
    a building lit by a BRAND is alight at the roof and nowhere else, which
    is instantly legible as the fire having jumped rather than crept.

LAYOUT
------
Two blocks either side of a central street, each a front row on the street and
a back row on a rear alley. Buildings are packed shoulder to shoulder with
0.4-3 m gaps, which is deliberate: the gap distribution IS the spread graph,
and a plate where everything is 20 m apart has no conflagration in it.

Env:
    UF_REGION    plate size in metres (default 100)
    UF_ELAPSED   minutes since ignition (default 110)
    UF_WIND      "<deg>,<mps>" the direction the wind blows TOWARD
                 (default "10,6" — an east wind, so the fire runs east)
    UF_IGNITE    index of the building that started it (default: drawn from
                 the middle of the plate)
    UF_STYLES    comma list overriding the style pool
    UF_SEED      rng seed (default 12)
    UF_FLOW      1 (default) authors NVIDIA Flow and the flames
    SETTLE_STEPS physics step ceiling (default 2600)
    SNAP_DIR     viewport captures, under /isaac-sim/.nvidia-omniverse/logs/
    KEEP_OPEN    1 keeps the app up after the captures
"""

import math
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
simulation_app = SimulationApp(launch_config={"headless": _HEADLESS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
enable_extension("omni.flowusd")

import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade             # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import numpy as np                                             # noqa: E402
import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import urban_building as ub                        # noqa: E402
from disaster import fire as fx                                # noqa: E402
from disaster import fracture, settle                          # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402
from disaster import urban_fire_spread as ufs                  # noqa: E402

# THE 112-ASSET LIBRARY, MEASURED. `config/asset_sets/urban_v2.yaml` carries
# 25 towers, 80 midrise and 7 rowhouse as standalone USDs, against the 30
# buildings `urban_building` assembles from façade modules — and the fire
# scenes had used only the modular ones, so every review saw variations on
# the same seven blocks (user, 2026-08-28). `_plans/urban_monoliths.json` is
# the bare-`pxr` measurement of all 112 (footprint, height, bbox centre,
# z-min; 112 measured, 0 errors), so the packer can lay them out host-side
# without opening any of them.
MONO_JSON = os.path.join(_SCENE_GEN_DIR, "_plans", "urban_monoliths.json")

PARENT = "/World/stage/generated"

# THE STYLE POOL, AND WHY IT IS THIS ONE.
# `urban_building.STYLES` has 30 entries and the fire bench had only ever
# shown seven of them (user review, 2026-08-28: "rn you're showing me the same
# 5-6 buildings"). Everything here is one of the OTHER 23, chosen for two
# things: a footprint that fits a 100 m block (nothing over 42 m on its long
# side, so several fit in a row), and a spread of construction TYPE, because
# the type is what decides how a building burns — `urm` gets gutted and can
# collapse, `rc` spalls and stands, `rc_glass` loses a vertical stripe of
# bays. Heights run 8 m (church) to 61 m (highrise_02), which is what a real
# downtown edge looks like; the 85-103 m towers are left out because on a
# 100 m plate they are taller than the block is wide.
POOL = [
    # urm — masonry, the stock that guts and collapses
    "brownstone",        # 22 x 14 x 14
    "walkup",            # 22 x 18 x 23
    "apartment",         # 22 x 17 x 18
    "commercial_mid",    # 30 x 18 x 26
    "highrise_04",       # 22 x 18 x 41
    "highrise_01",       # 22 x 17 x 54
    "apartment_long",    # 42 x 12 x 21
    "dw_terrace_long",   # 40 x 15 x 12  (storefront terrace)
    "church",            # 16 x 28 x  8
    # rc — concrete frames, they spall and stand
    "office",            # 32 x 16 x 19
    "office_plain",      # 32 x 24 x 25
    "civic_offices",     # 30 x 20 x 12
    "highrise_02",       # 24 x 16 x 61
    # rc_glass — a curtain-wall tower, for the vertical stripe
    "tower",             # 25 x 20 x 43
]
# `skyscraper_c` (20 x 20 x 85) was in this pool and had to come out. The
# comment above already said the 85-103 m towers do not belong on a 100 m
# plate; leaving one in proved it twice over — it took the ignition (it is the
# most upwind building on the seed), and from the nadir camera its roof sits
# 85 m up, so the plumb view framed 40 m of the plate and blew its roof out
# white while the actual city was off-frame (uf_city, 2026-08-28).

# What share of the block comes from the standalone library. The modular kit
# is kept in the mix on purpose and not out of caution: it is the only stock
# that can be GUTTED — floors, roof holes, collapse and interiors all need
# elements to work on — so a plate of pure monoliths would have surface fire
# damage and no structural damage anywhere in it. Two thirds monolith, one
# third kit, gives the variety and keeps the deep damage.
MONO_SHARE = float(_env("UF_MONO_SHARE", "0.55"))

REGION = float(_env("UF_REGION", "100"))
ELAPSED_MIN = float(_env("UF_ELAPSED", "110"))
SEED = int(_env("UF_SEED", "12"))
SETTLE_STEPS = int(_env("SETTLE_STEPS", "2600"))
SNAP_DIR = _env("SNAP_DIR", "")
FLOW = _env("UF_FLOW", "1") not in ("0", "false")
KEEP_PHYSICS = _env("KEEP_PHYSICS", "0") not in ("0", "false")
_w = [q.strip() for q in _env("UF_WIND", "10,6").split(",") if q.strip()]
WIND_DEG = float(_w[0]) if _w else 10.0
WIND_MPS = float(_w[1]) if len(_w) > 1 else 6.0
IGNITE = _env("UF_IGNITE", "")
_sp = _env("UF_STYLES", "")
if _sp:
    POOL = [q.strip() for q in _sp.split(",") if q.strip()]

STREET_HALF = 7.0        # half the central carriageway
ALLEY = 5.0              # rear service gap between the two rows of a block
EDGE = 2.0               # setback from the plate boundary


def pack_row(queue, y_front, yaw, x0, x1, rng, front_is_minus_y):
    """Lay buildings along a frontage, shoulder to shoulder.

    Returns [(style, cx, cy, yaw)] and the styles it could not fit.

    THE GAPS ARE THE SPREAD GRAPH. 0.4-3.0 m is drawn deliberately: under
    1.2 m `urban_fire_spread` calls the pair ATTACHED and the fire crosses in
    minutes, and above it the pair is a RADIATION edge whose delay grows with
    the square of the gap. A row laid with one uniform gap gives one uniform
    delay and the whole street lights at once.
    """
    out = []
    x = x0
    # GREEDY FROM A SHARED QUEUE, not a fixed slice per row. Handing each row
    # four styles and dropping whatever did not fit wasted one building a row
    # — four rows of three on a plate that has room for fifteen. Take the next
    # entry that fits and leave the rest for the next row.
    #
    # An entry is either a kit STYLE NAME (a string) or a measured monolith
    # RECORD (a dict), and `_dims` is the only place that has to care.
    while queue:
        pick = None
        for k, ent in enumerate(queue):
            W, _D = _dims(ent)
            if x + W <= x1:
                pick = k
                break
        if pick is None:
            break
        ent = queue.pop(pick)
        W, D = _dims(ent)
        cx = x + W / 2.0
        # `front_is_minus_y` says which way this row faces; the building's own
        # front is -Y at yaw 0 (`urban_building`), so the row's y offset is
        # measured from the frontage line to the CENTRE.
        cy = (y_front + D / 2.0) if front_is_minus_y else (y_front - D / 2.0)
        out.append((ent, cx, cy, yaw))
        x = cx + W / 2.0 + rng.uniform(0.4, 3.0)
    return out


def _dims(ent):
    """(W, D) for a kit style name or a measured monolith record."""
    if isinstance(ent, dict):
        return ent["W"], ent["D"]
    return ub.footprint(ub.STYLES[ent])


def _height(ent):
    if isinstance(ent, dict):
        return ent["H"]
    return ub.height(ub.STYLES[ent])


def _name(ent):
    if isinstance(ent, dict):
        return ent["usd"].rsplit("/", 1)[-1].rsplit(".", 1)[0]
    return ent


def load_monoliths(rng, max_span=42.0, max_h=70.0):
    """The measured standalone buildings that fit this plate, shuffled."""
    import json
    try:
        rows = json.load(open(MONO_JSON))
    except Exception as exc:
        print("[uf_city] monolith table unavailable ({0}); kit only".format(exc))
        return []
    ok = [r for r in rows if "err" not in r
          and max(r["W"], r["D"]) <= max_span and r["H"] <= max_h
          and min(r["W"], r["D"]) >= 5.0]
    rng.shuffle(ok)
    return ok


def plan_city(rng):
    """Four frontages round a central street; returns placements."""
    half = REGION / 2.0
    x0, x1 = -half + EDGE, half - EDGE
    # interleave the two libraries so a row is a mix, not a run of one
    monos = load_monoliths(rng)
    kit = list(POOL)
    rng.shuffle(kit)
    n_total = 22                       # more than will fit; the packer stops
    n_mono = int(round(n_total * MONO_SHARE))
    pool = []
    take_m, take_k = monos[:n_mono], kit[:max(1, n_total - n_mono)]
    # THE PACKER TAKES WHATEVER FITS NEXT, and the standalone assets are much
    # smaller than the kit assemblies (9 x 18 m against 22 x 17 m), so a fair
    # coin gave 13 monoliths to 2 kit buildings — and the kit is the only
    # stock that can be gutted, so the plate lost almost all its structural
    # damage. Seed each row with a KIT building first, then interleave.
    while take_m or take_k:
        if take_k and (len(pool) % 4 == 0 or not take_m):
            pool.append(take_k.pop(0))
        elif take_m:
            pool.append(take_m.pop(0))
    # deepest buildings go in the back rows, where there is more room
    rows = []
    # (frontage y, yaw, front_is_minus_y)
    specs = [
        (STREET_HALF, 0.0, True),                    # north side of the street
        (half - EDGE, 180.0, False),                 # north block, rear
        (-STREET_HALF, 180.0, False),                # south side of the street
        (-half + EDGE, 0.0, True),                   # south block, rear
    ]
    placed = []
    for (yf, yaw, minus) in specs:
        if not pool:
            break
        got = pack_row(pool, yf, yaw, x0, x1, rng, minus)
        placed += got
        rows.append(len(got))
    return placed, rows


def build_ground_and_light(stage, span):
    e = max(260.0, span * 2.2)
    ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    ground.CreatePointsAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, -e, 0.0),
                             Gf.Vec3f(e, e, 0.0), Gf.Vec3f(-e, e, 0.0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.28, 0.27)])
    ground.CreateExtentAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, e, 0.0)])
    try:
        mp = stage.DefinePrim(Sdf.Path("/World/Looks/pavement"))
        mp.GetReferences().AddReference(sg._join_asset_root(
            "airstack://scene_gen/assets/materials/megascans/Road_Asphalt.usda", ""))
        mp.Load()
        m = UsdShade.Material.Get(stage, "/World/Looks/pavement")
        if m:
            UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(m)
    except Exception as exc:
        print("[uf_city] ground material unavailable: {0}".format(exc))
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(700.0)
    dome.CreateColorAttr(Gf.Vec3f(0.72, 0.76, 0.86))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(3200.0)
    key.CreateAngleAttr(0.9)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.86))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-25.0, 0.0, 28.0))


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
    placed, rows = plan_city(rng)
    build_ground_and_light(stage, REGION)
    if FLOW:
        fx.setup_flow_stack(stage, density_cell_size_m=0.09, max_blocks=32768,
                            scene_scale_factor=ssf)
    flow_root = fx.FLOW_ROOT if FLOW else None

    # 1) place every building
    cols = []
    for i, (ent, cx, cy, yaw) in enumerate(placed):
        parent = "{0}/b{1}".format(PARENT, i)
        UsdGeom.Scope.Define(stage, Sdf.Path(parent))
        W, D = _dims(ent)
        rec = dict(i=i, style=_name(ent), x=cx, y=cy, yaw=yaw, parent=parent,
                   W=W, D=D, H=_height(ent), mono=isinstance(ent, dict),
                   ent=ent, pls=[])
        if not rec["mono"]:
            # the kit path: build the façade modules now, damage them later
            rec["pls"] = ub.build_building(ent, cx, cy, yaw,
                                           random.Random(SEED + 7 * i))
            sg.apply_placements(stage, rec["pls"], parent, ssf)
        cols.append(rec)
    ub.apply_glass_tint(stage, [p for c in cols for p in c["pls"]])
    for _ in range(5):
        omni.kit.app.get_app().update()
    print("[uf_city] {0} m plate, {1} building(s) in rows of {2}".format(
        REGION, len(cols), rows))

    # 2) SOLVE THE SPREAD — this is what sets every severity in the scene
    bl = [{"x": c["x"], "y": c["y"], "W": c["W"], "D": c["D"],
           "yaw": c["yaw"], "H": c["H"], "style": c["style"],
           "mono": c["mono"], "mat": (c["ent"].get("mat", "-")
                                      if c["mono"] else None)} for c in cols]

    def _btype(b):
        if b.get("mono"):
            # the library tags each asset with its cladding; brick and stone
            # burn like masonry, everything else like a frame
            return "urm" if b.get("mat") in ("brick", "stone") else "rc"
        return qf.FAMILY_TYPE.get(ub.STYLES[b["style"]].get("family"), "urm")

    if IGNITE:
        ign = int(IGNITE) % len(bl)
    else:
        # start it near the middle of the UPWIND half, so there is a plate's
        # worth of block for the fire to run through
        wx, wy = math.cos(math.radians(WIND_DEG)), math.sin(math.radians(WIND_DEG))
        ign = min(range(len(bl)),
                  key=lambda k: (bl[k]["x"] * wx + bl[k]["y"] * wy)
                  + 0.35 * abs(bl[k]["y"]))
    plan = ufs.solve(bl, ign, ELAPSED_MIN * 60.0,
                     wind_dir=math.radians(WIND_DEG), wind_mps=WIND_MPS,
                     rng=random.Random(SEED + 991), btype_of=_btype)
    print("[uf_city] ignition at {0} ({1}), wind {2:.0f} deg at {3:.0f} m/s, "
          "T+{4:.0f} min".format(ign, bl[ign]["style"], WIND_DEG, WIND_MPS,
                                 ELAPSED_MIN))
    for ln in ufs.summarise(bl, plan, ELAPSED_MIN * 60.0):
        print("[uf_city] " + ln)

    # 3) burn each building at the level the solve gave it
    mats = uf.materials(stage, PARENT)
    cache = {}
    loose, static, vel = [], ["/World/ground"], {}
    tally = {}
    for c, p in zip(cols, plan):
        c["plan"] = p
        tally[p["level"]] = tally.get(p["level"], 0) + 1
        if p["level"] == "F0":
            if c["mono"]:
                # untouched, but it still has to BE there: `burn_monolith`
                # is what references the asset onto the stage.
                uf.burn_monolith(
                    stage, c["parent"], c["ent"]["usd"], c["x"], c["y"],
                    c["yaw"], c["ent"], "F0", random.Random(SEED + c["i"]),
                    np.random.default_rng(SEED + c["i"]), mats,
                    "b{0}".format(c["i"]), flow_root=None,
                    entry_side="S", origin_frac=0.2, ssf=ssf)
            static += [q["prim_path"] for q in c["pls"] if q.get("prim_path")]
            continue
        brng = random.Random(SEED + 101 * c["i"])
        nrng = np.random.default_rng(SEED + 101 * c["i"])
        entry = p["entry_side"] or brng.choice(("S", "E", "N", "W"))
        tb = time.time()
        if c["mono"]:
            # A STANDALONE ASSET HAS NO ELEMENTS, so it takes the
            # bounding-box path: its own materials sooted in place, a stripe
            # of tongues up the burning face, flame out of that band, a
            # charred roof and glass on the pavement. No gutting, no roof
            # hole, no collapse — there is nothing there to take apart.
            res = uf.burn_monolith(
                stage, c["parent"], c["ent"]["usd"], c["x"], c["y"], c["yaw"],
                c["ent"], p["level"], brng, nrng, mats,
                "b{0}".format(c["i"]), flow_root=flow_root,
                entry_side=entry, origin_frac=p["origin_frac"], ssf=ssf)
        else:
            # THE STOREY AND THE ELEVATION COME FROM THE SOLVE, NOT A DRAW.
            n_st = max(1, len(qf._mass_specs(c["style"], c["x"], c["y"],
                                             c["yaw"])[0]["levels"]))
            origin = max(0, min(n_st - 1,
                                int(round(p["origin_frac"] * (n_st - 1)))))
            order = ("S", "E", "N", "W")
            k = order.index(entry)
            sides = ((entry,) if p["level"] in ("F1", "F2")
                     else (entry, order[(k + 1) % 4]))
            res = uf.burn_building(
                stage, c["parent"], c["style"], c["pls"], c["x"], c["y"],
                c["yaw"], p["level"], brng, nrng, mats,
                "b{0}".format(c["i"]), flow_root=flow_root, origin=origin,
                sides=sides, mat_cache=cache)
        loose += res["loose"]
        static += res["static_extra"]
        vel.update(res["velocity"])
        c["fire"] = res["fire"]
        print("[uf_city] {0:<26} {1} {2}  storey {3}-{4} on {5:<6} {6:4d} "
              "loose ({7:.0f} s)".format(
                  c["style"][:26], "MONO" if c["mono"] else "kit ", p["level"],
                  res["fire"]["origin"], res["fire"]["top"],
                  "/".join(res["fire"]["sides"]), len(res["loose"]),
                  time.time() - tb))
        for nt in res["notes"][:2]:
            print("[uf_city]      " + nt)
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
    n_mono = sum(1 for c in cols if c["mono"])
    print("  {0} buildings ({1} from the standalone library, {2} modular "
          "kit), {3} distinct assets, mix {4}".format(
              len(cols), n_mono, len(cols) - n_mono,
              len({c["style"] for c in cols}),
              ", ".join("{0}:{1}".format(k, tally[k]) for k in sorted(tally))))
    print("  {0} loose bodies, {1:.0f} s".format(len(loose), time.time() - t0))
    print("=" * 78 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp2 = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots", _sp2)
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            if FLOW:
                timeline.play()
                for _ in range(320):
                    omni.kit.app.get_app().update()
            span = REGION * 1.25
            tall = max(c["H"] for c in cols)
            # THE NADIR CAMERA HAS TO CLEAR THE TALLEST ROOF. `overview` puts
            # its eye at 0.95 x span and frames 1.16 x the eye HEIGHT — which
            # is the height above what it is looking at. With a 60 m building
            # on the plate the roof is 60 m nearer the camera than the street
            # is, so the frame at roof level is a fraction of the plate and
            # the shot is a close-up of one roof (uf_city, 2026-08-28).
            _snaps.place_camera(stage, (0.0, 0.0, tall + span / 1.164),
                                (0.0, 0.0, 0.0))
            _snaps.snapshot(os.path.join(SNAP_DIR, "city_top.png"))
            import omni.kit.viewport.utility as vp
            # FOUR OBLIQUES ROUND THE PLATE. The fire runs with the wind, so
            # which corner you stand in decides whether you are looking at the
            # burnt-out end or the end that is still catching — both are worth
            # having, and a single hero shot hides half the gradient.
            for name, (ax, ay) in (("sw", (-1, -1)), ("se", (1, -1)),
                                   ("ne", (1, 1)), ("nw", (-1, 1))):
                d = REGION * 0.95
                _snaps.place_camera(stage, (ax * d, ay * d, 0.52 * d + tall * 0.5),
                                    (0.0, 0.0, tall * 0.28))
                _snaps.snapshot(os.path.join(SNAP_DIR,
                                             "city_{0}.png".format(name)))
            # a street-level view down the central carriageway, from upwind
            _snaps.place_camera(stage, (-REGION * 0.62, 0.0, 3.0),
                                (REGION * 0.4, 0.0, 12.0))
            _snaps.snapshot(os.path.join(SNAP_DIR, "city_street.png"))
            # and one per building, from the side its fire is on
            for c in cols:
                f = c.get("fire")
                if not f:
                    continue
                side = f["sides"][0]
                nx, ny = {"S": (0.0, -1.0), "N": (0.0, 1.0),
                          "E": (1.0, 0.0), "W": (-1.0, 0.0)}[side]
                a = math.radians(c["yaw"])
                wx = nx * math.cos(a) - ny * math.sin(a)
                wy = nx * math.sin(a) + ny * math.cos(a)
                back = (c["D"] if side in ("S", "N") else c["W"]) / 2.0
                fit = max(max(c["W"], c["D"]) / 1.164,
                          c["H"] / 0.655) * 1.1 + back
                _snaps.place_camera(
                    stage, (c["x"] + wx * fit * 0.8 - wy * fit * 0.5,
                            c["y"] + wy * fit * 0.8 + wx * fit * 0.5,
                            0.42 * fit + 0.3 * c["H"]),
                    (c["x"], c["y"], c["H"] * 0.45))
                _snaps.snapshot(os.path.join(
                    SNAP_DIR, "{0}_{1}_{2}.png".format(
                        c["i"], c["style"], c["plan"]["level"])))
            vp.get_active_viewport().camera_path = _snaps.CAM
            print("[uf_city] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[uf_city] snapshots FAILED: {0}".format(exc))

    print("URBAN FIRE CITY DONE")
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
