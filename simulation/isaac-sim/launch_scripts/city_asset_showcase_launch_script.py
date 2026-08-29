#!/usr/bin/env python
"""
City asset showcase — one instance of every building we can actually use,
the greebled monolith, and a park built from the CityPark props.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/showcase \
    ISAAC_SIM_SCRIPT_NAME=city_asset_showcase_launch_script.py airstack up isaac-sim

WHAT IS HERE AND WHY
--------------------
From the Nucleus survey (`scene_gen/_plans/nucleus_asset_survey.json`), the
building stock that is NOT the low-poly `selected_citydemo` monoliths, with
the ruins excluded by request:

  row 1  ModernCityEnvironment   4 merged whole buildings, 63k-336k tris
  row 2  DownTown                6 background blocks, 1.4k-7.3k tris
  row 3  urban_building          8 assembled kit styles, one per family —
                                 the modular stock, which can be built at any
                                 size, so one instance is a sample not a limit
  row 4  the greebled monolith, next to the same asset untouched

THE ×0.01 THAT IS NOT OPTIONAL. Three of the four ModernCityEnvironment
buildings have their points in CENTIMETRES with no scale op on the layer —
only `MBuilding01` carries one. Referenced raw they come in at 100x, which
measures as a 9.1 km wide building. `MCE_SCALE` fixes them, and getting this
wrong is not subtle.

THE PARK IS AUTHORED HERE, NOT BY `detail/parks.py`. This is a look-first
pass: the CityPark props are laid out by hand so the set can be judged before
any of it is wired into the real park builders. Every one carries a measured
scale, because this pack is authored inconsistently — `SM_Trash_can` comes in
at 1.3 m across (the `urban.yaml` comment already flags it as out of scale)
while `SM_Small_Trash_can` is right at 1.0.

Env:
    CS_PARK      1 (default) builds the park plot
    CS_BUILDINGS 1 (default) builds the building rows
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

import omni.kit.app                                            # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, Vt    # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import greeble                                     # noqa: E402
from detail import urban_building as ub                        # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

NUC = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
SNAP_DIR = _env("SNAP_DIR", "")
DO_PARK = _env("CS_PARK", "1") not in ("0", "false")
DO_BLD = _env("CS_BUILDINGS", "1") not in ("0", "false")

# ---- row 1: the merged whole buildings -----------------------------------
# `scale` is per asset because the pack is inconsistent: only Building01 has a
# scale op on its layer, so the rest need the cm->m conversion applied here.
MCE_SCALE = 0.01
MCE = [
    ("MBuilding01", "ModernCityEnvironment/Collected_Building01/"
     "SM_MERGED_BP_MBuilding01.usd", 1.0),          # already 28x19x29 m
    ("MBuilding02", "ModernCityEnvironment/Collected_Building02/"
     "SM_MERGED_BP_MBuilding02.usd", MCE_SCALE),    # -> 91x96x69 m
    ("MBuilding03", "ModernCityEnvironment/Collected_Building03/"
     "SM_MERGED_BP_MBuilding03.usd", MCE_SCALE),    # -> 16x24x22 m
    ("MBuilding05", "ModernCityEnvironment/Collected_Building05/"
     "SM_MERGED_BP_MBuilding05.usd", MCE_SCALE),    # -> 28x21x62 m
]

# ---- row 2: the DownTown background blocks -------------------------------
DTOWN = [("BG_Building_" + c, "DownTown/Assets/BG_Building_{0}.usd".format(c))
         for c in "ABCDEF"]

# ---- row 3: one assembled kit style per family ---------------------------
KIT = ["apartment", "office", "brownstone", "commercial", "tower",
       "dw_terrace", "civic_offices", "church"]

# ---- row 4 -----------------------------------------------------------------
MONO = NUC + "selected_citydemo/tower/tower_03_0015.usd"

# ---- the park props, each with a MEASURED scale --------------------------
# (usd, scale, note) — scale is `target / measured`, and the measurements are
# the ones in `_plans/nucleus_asset_survey.json`.
P = "CityPark/Props/"
PARK_PROPS = {
    "wall_run":   (P + "SM_Fence_BrickStone.prop.usd", 1.0),        # 2.0 x 0.6 x 0.9
    "wall_end":   (P + "SM_Fence_BrickStone_end.prop.usd", 1.0),    # 1.0 x 0.6 x 0.9
    "wall_angle": (P + "SM_Fence_BrickStone_angle.prop.usd", 1.0),  # 1.3 x 1.3 x 0.9
    "rail":       (P + "SM_Fence_metall.prop.usd", 1.0),            # 1.0 x 0.1 x 1.1
    "low_wall":   (P + "SM_Stone_Wall_01_01.prop.usd", 1.0),        # 3.0 x 0.5 x 1.1
    "paving":     (P + "SM_Stones_Floor.prop.usd", 1.0),            # 1.6 x 1.7 tile
    "bench_log":  (P + "SM_Log_shop.prop.usd", 1.0),                # 2.2 x 0.9 x 0.4
    # 3.7 m is a genuine long park bench, not a mis-scale — cutting it to 2.0
    # made it read as a folded wedge. Left at its authored length.
    "bench":      (P + "SM_Lavka_03.prop.usd", 1.0),                # 3.7 m
    "lantern":    (P + "SM_Lantern.prop.usd", 0.88),                # 4.2 -> 3.7 m
    "bin":        (P + "SM_Trash_can.prop.usd", 0.50),              # 1.3 -> 0.65 m
    "bin_small":  (P + "SM_Small_Trash_can.prop.usd", 1.0),         # 0.3 x 0.5
    "bollard":    (P + "SM_Pillar_metall.prop.usd", 1.0),           # 0.2 x 1.1
    "boulder":    (P + "SM_Rock_03.prop.usd", 1.0),                 # 4.5 x 2.5 x 2.3
    "rock":       (P + "SM_Stone_05.prop.usd", 1.0),                # 3.1 x 1.3 x 1.0
    "stump":      (P + "SM_Stump_02.prop.usd", 1.0),                # 0.9 x 0.9 x 1.0
    "log":        (P + "SM_Timber_02.prop.usd", 1.0),               # 0.9 x 0.4
    "steps":      (P + "SM_Stair_01.prop.usd", 1.0),                # 1.4 x 1.6 x 0.6
    "grass":      (P + "SM_Grass_03.prop.usd", 1.0),                # 0.6 tuft
    "tree_a":     (P + "SM_Tree_05.prop.usd", 1.0),                 # 3.7 x 3.8 x 5.0
    "tree_b":     (P + "SM_Tree_08.prop.usd", 1.0),                 # 4.0 x 3.1 x 5.4
    "tree_c":     (P + "SM_Tree_07_Model.prop.usd", 1.0),           # 1.4 x 1.3 x 2.7
}


def light_and_ground(stage, span):
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.76, 0.80, 0.88))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(3000.0)
    key.CreateAngleAttr(0.8)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-32.0, 0.0, 30.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = span
    g.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                        Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.31, 0.31, 0.30)])
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
        print("[showcase] ground material: {0}".format(exc))


# THE PACKS DISAGREE ABOUT UNITS, AND USD DOES NOT CONVERT FOR YOU.
# Measured `metersPerUnit` per pack:
#     CityPark              0.01   (centimetres)
#     DownTown              0.01   (centimetres)
#     ModernCityEnvironment 1.0
#     selected_citydemo     1.0
# `Usd.Stage.Open` on the asset applies its own mpu, so measuring it in
# isolation gives sensible metres — but REFERENCING it into a metres stage
# does not rescale anything, so a 3.7 m bench arrives 370 m long and one tree
# fills a 620 m frame. That is the whole of the first showcase's "giant tree".
# This is the same `scene_scale_factor` the repo threads through
# `apply_placements`; the hand-rolled reference below has to apply it too.
MPU = {"CityPark/": 0.01, "DownTown/": 0.01}


def _unit_scale(rel):
    for k, v in MPU.items():
        if k in rel:
            return v
    return 1.0


def _ref(stage, path, url, x, y, z=0.0, yaw=0.0, scale=1.0):
    # THE HOLDER MUST NOT DECLARE A TYPE, AND THE TRANSFORM GOES ON ITS
    # PARENT. `BG_Building_A.usd`'s default prim is a **Mesh**, not an Xform.
    # Referencing it onto a prim this code had already declared as an Xform
    # leaves the LOCAL type winning, so the composed prim is an Xform holding
    # mesh attributes: `IsA(UsdGeom.Mesh)` is false, nothing draws it and its
    # world bound is empty — which is why every DownTown block measured
    # 0.0 x 0.0 x 0.0 m. Same family as the quake bake's "a Scope is not
    # Xformable so the ops are skipped", one level along: there the asset's
    # root type was wrong for the holder, here the holder's is wrong for the
    # asset. A typeless child lets the asset's own type win, and the parent
    # Xform carries the placement whatever that type turns out to be.
    p = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    child = stage.DefinePrim(Sdf.Path(path + "/asset"))
    child.GetReferences().AddReference(url)
    stage.Load(Sdf.Path(path))
    xf = UsdGeom.Xformable(p)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))
    if abs(yaw) > 1e-6:
        xf.AddRotateZOp().Set(float(yaw))
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    return p


def _bbox(stage, path):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    if r.IsEmpty():
        return 0.0, 0.0, 0.0, 0.0
    mn, mx = r.GetMin(), r.GetMax()
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2], mn[2])


def grass_plot(stage, path, cx, cy, w, d):
    g = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    hw, hd = w / 2.0, d / 2.0
    g.CreatePointsAttr([Gf.Vec3f(cx - hw, cy - hd, 0.02),
                        Gf.Vec3f(cx + hw, cy - hd, 0.02),
                        Gf.Vec3f(cx + hw, cy + hd, 0.02),
                        Gf.Vec3f(cx - hw, cy + hd, 0.02)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.16, 0.24, 0.10)])
    g.CreateExtentAttr([Gf.Vec3f(cx - hw, cy - hd, 0.0),
                        Gf.Vec3f(cx + hw, cy + hd, 0.05)])
    # A `displayColor` QUAD IS A MIRROR. With no material bound the lawn takes
    # the renderer's default surface — smooth and specular — so a 60 m plate of
    # it reflected the whole skyline and read as a pond, not grass. The suburb
    # already owns the right material (`suburban.yaml`: `grass_park` ->
    # `Grass_Countryside.usda`); bind it here too rather than inventing one.
    try:
        gp = "/World/Looks/grass_park"
        if not stage.GetPrimAtPath(gp).IsValid():
            mp = stage.DefinePrim(Sdf.Path(gp))
            mp.GetReferences().AddReference(sg._join_asset_root(
                "airstack://scene_gen/assets/materials/"
                "Grass_Countryside.usda", ""))
            mp.Load()
        m = UsdShade.Material.Get(stage, "/World/Looks/grass_park")
        if m:
            UsdShade.MaterialBindingAPI(g.GetPrim()).Bind(m)
    except Exception as exc:
        print("[showcase] grass material: {0}".format(exc))
    return g


def build_park(stage, cx, cy, w, d, rng):
    """A 60 x 60 m park laid out by hand from the CityPark set.

    NOT `detail/parks.py`. This is the look-first pass the real builders get
    wired to once the set is approved: a boundary wall with a gated entrance,
    a paved path across it, seating and lighting along the path, a boulder
    group, and trees with stumps and undergrowth at the corners.
    """
    n = [0]

    def _p(kind):
        n[0] += 1
        return "/World/park/{0}_{1}".format(kind, n[0])

    def put(kind, x, y, yaw=0.0, z=0.0, jitter=0.0):
        usd, sc = PARK_PROPS[kind]
        if jitter:
            x += rng.uniform(-jitter, jitter)
            y += rng.uniform(-jitter, jitter)
        # the per-asset scale is a REAL-WORLD correction (a 1.3 m bin down to
        # 0.65); the unit scale is the cm->m conversion. Both apply.
        return _ref(stage, _p(kind), NUC + usd, x, y, z, yaw,
                    sc * _unit_scale(usd))

    UsdGeom.Scope.Define(stage, Sdf.Path("/World/park"))
    grass_plot(stage, "/World/park/lawn", cx, cy, w, d)
    hw, hd = w / 2.0, d / 2.0
    made = 0

    # 1) THE BOUNDARY — a low brick-and-stone wall on a 2 m module, with the
    #    corner piece where it turns and a gap left for the entrance. A park
    #    reads as a park because it is ENCLOSED; the fence is the first thing
    #    to get right and the current pool is one chain-link panel.
    step = 2.0
    gate = (cx - 4.0, cx + 4.0)          # the gap on the south side
    x = cx - hw + 1.0
    while x < cx + hw - 1.0:
        if not (gate[0] <= x <= gate[1]):
            put("wall_run", x, cy - hd, 0.0)
            made += 1
        put("wall_run", x, cy + hd, 180.0)
        made += 1
        x += step
    y = cy - hd + step
    while y < cy + hd - 0.5:
        put("wall_run", cx - hw, y, 90.0)
        put("wall_run", cx + hw, y, 270.0)
        made += 2
        y += step
    for (sx, sy, sa) in ((-1, -1, 0), (1, -1, 90), (1, 1, 180), (-1, 1, 270)):
        put("wall_angle", cx + sx * hw, cy + sy * hd, sa)
        made += 1

    # 2) THE PATH — stone paving tiles from the gate to the far side, two
    #    tiles wide, with the steps up onto the lawn at the entrance.
    tile = 1.6
    put("steps", cx, cy - hd + 1.2, 0.0)
    made += 1
    yy = cy - hd + 3.0
    while yy < cy + hd - 2.0:
        for ox in (-tile * 0.5, tile * 0.5):
            put("paving", cx + ox, yy, rng.choice((0.0, 90.0, 180.0, 270.0)))
            made += 1
        yy += tile

    # 3) SEATING AND LIGHTING along the path, alternating sides — a bench
    #    facing the path, a lantern between every second pair.
    side = 1
    for k, yy in enumerate([cy - hd + 8.0 + i * 9.0
                            for i in range(int((d - 14.0) // 9.0))]):
        put("bench", cx + side * 4.2, yy, 90.0 if side > 0 else 270.0)
        made += 1
        if k % 2 == 0:
            put("lantern", cx - side * 4.6, yy + 2.0)
            made += 1
        if k % 3 == 1:
            put("bin_small", cx + side * 5.6, yy - 1.4)
            made += 1
        side = -side

    # 4) A BOULDER GROUP in one quadrant, with logs and stumps round it —
    #    the naturalistic corner every park has.
    bx, by = cx - hw * 0.52, cy + hd * 0.45
    put("boulder", bx, by, rng.uniform(0, 360))
    put("rock", bx + 4.5, by - 2.0, rng.uniform(0, 360))
    put("rock", bx - 3.2, by + 2.6, rng.uniform(0, 360))
    made += 3
    for _ in range(6):
        put("log", bx, by, rng.uniform(0, 360), jitter=6.0)
        put("grass", bx, by, rng.uniform(0, 360), jitter=7.5)
        made += 2
    for _ in range(3):
        put("stump", bx, by, rng.uniform(0, 360), jitter=8.0)
        made += 1

    # 5) TREES round the edge, clear of the path and the wall
    for _ in range(14):
        a = rng.uniform(0, 2 * math.pi)
        r = rng.uniform(0.45, 0.86)
        tx, ty = cx + math.cos(a) * hw * r, cy + math.sin(a) * hd * r
        if abs(tx - cx) < 4.0:
            continue                      # keep the path clear
        put(rng.choice(("tree_a", "tree_b", "tree_c")), tx, ty,
            rng.uniform(0, 360))
        made += 1
    # 6) a low retaining wall making a raised planter in the far corner
    for i in range(4):
        put("low_wall", cx + hw * 0.45 + i * 3.0, cy - hd * 0.42, 0.0)
        made += 1
    for i in range(3):
        put("bench_log", cx + hw * 0.40, cy - hd * 0.25 + i * 3.2, 90.0)
        made += 1
    return made


def main():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    w = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(w.GetPrim())
    light_and_ground(stage, 520.0)
    _, ssf = get_stage_meters_per_unit(stage)
    rng = random.Random(5)
    t0 = time.time()
    rows = []

    if DO_BLD:
        # ---- row 1: ModernCityEnvironment ---------------------------------
        x = -170.0
        for name, rel, sc in MCE:
            p = "/World/mce_{0}".format(name)
            _ref(stage, p, NUC + rel, x, 120.0, 0.0, 0.0, sc)
            for _ in range(3):
                omni.kit.app.get_app().update()
            W, D, H, z0 = _bbox(stage, p)
            rows.append(("ModernCityEnv", name, W, D, H))
            print("[showcase] MCE  {0:<14} {1:6.1f} x {2:6.1f} x {3:6.1f} m"
                  .format(name, W, D, H))
            x += max(60.0, W * 1.25)

        # ---- row 2: DownTown BG -------------------------------------------
        x = -170.0
        for name, rel in DTOWN:
            p = "/World/dt_{0}".format(name)
            _ref(stage, p, NUC + rel, x, 20.0, scale=_unit_scale(rel))
            for _ in range(4):
                omni.kit.app.get_app().update()
            W, D, H, z0 = _bbox(stage, p)
            rows.append(("DownTown", name, W, D, H))
            print("[showcase] DT   {0:<14} {1:6.1f} x {2:6.1f} x {3:6.1f} m"
                  .format(name, W, D, H))
            x += max(55.0, W * 1.2)

        # ---- row 3: assembled kit styles ----------------------------------
        x = -170.0
        for st in KIT:
            W, D = ub.footprint(ub.STYLES[st])
            H = ub.height(ub.STYLES[st])
            parent = "/World/kit_{0}".format(st)
            UsdGeom.Scope.Define(stage, Sdf.Path(parent))
            pls = ub.build_building(st, x, -70.0, 0.0, random.Random(7))
            sg.apply_placements(stage, pls, parent, ssf)
            rows.append(("urban_building", st, W, D, H))
            print("[showcase] KIT  {0:<14} {1:6.1f} x {2:6.1f} x {3:6.1f} m  "
                  "{4} module(s)".format(st, W, D, H, len(pls)))
            x += max(50.0, W * 1.25)
        ub.apply_glass_tint(stage, [])

        # ---- row 4: the monolith, plain and greebled -----------------------
        _ref(stage, "/World/mono_plain", MONO, -150.0, -170.0)
        gp = "/World/mono_greebled"
        _ref(stage, gp, MONO, -60.0, -170.0)
        for _ in range(4):
            omni.kit.app.get_app().update()
        parts = uf.mono_parts(stage, gp)
        gmats = greeble.materials(stage, "/World")
        made = greeble.greeble_parts(stage, "/World/greeble", parts, gmats)
        print("[showcase] MONO greebled: {0} part(s) -> {1} prim(s)".format(
            len(parts), len(made)))
        rows.append(("monolith", "tower_03_0015 (greebled)", 33.0, 50.0, 83.0))

    n_park = 0
    if DO_PARK:
        n_park = build_park(stage, 150.0, -120.0, 60.0, 60.0, rng)
        print("[showcase] park: {0} prop(s) from {1} CityPark asset(s)".format(
            n_park, len(PARK_PROPS)))

    for _ in range(15):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 76)
    print("CITY ASSET SHOWCASE")
    print("  {0} building(s):".format(len(rows)))
    for pack, name, W, D, H in rows:
        print("    {0:<16} {1:<26} {2:6.1f} x {3:6.1f} x {4:6.1f} m".format(
            pack, name, W, D, H))
    print("  park: {0} props from {1} distinct CityPark assets".format(
        n_park, len(PARK_PROPS)))
    print("  built in {0:.0f} s".format(time.time() - t0))
    print("=" * 76 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            sn.overview(stage, (0.0, 0.0), 620.0,
                        os.path.join(SNAP_DIR, "all_top.png"), ssf)
            for nm, eye, tgt in (
                    ("row_mce", (-60.0, 20.0, 90.0), (0.0, 120.0, 25.0)),
                    ("row_downtown", (-60.0, -70.0, 80.0), (0.0, 20.0, 30.0)),
                    ("row_kit", (-60.0, -150.0, 70.0), (0.0, -70.0, 20.0)),
                    ("mono_pair", (-105.0, -250.0, 60.0),
                     (-105.0, -170.0, 40.0)),
                    ("park", (150.0, -205.0, 46.0), (150.0, -120.0, 4.0)),
                    ("park_low", (150.0, -158.0, 3.0), (150.0, -100.0, 6.0))):
                sn.place_camera(stage, eye, tgt)
                sn.snapshot(os.path.join(SNAP_DIR, nm + ".png"))
            print("[showcase] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[showcase] snapshots FAILED: {0}".format(exc))

    print("SHOWCASE DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
