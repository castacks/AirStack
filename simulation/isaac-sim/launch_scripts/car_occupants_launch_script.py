#!/usr/bin/env python
"""
Occupant bench — five cars, a person in each, and nothing else in frame.

    ISAAC_SIM_SCRIPT_NAME=car_occupants_launch_script.py \
    SCENE_CONFIG=suburb_wildfire airstack up isaac-sim

WHY THIS EXISTS
---------------
Putting somebody in a car is the one placement in this dataset that cannot be
derived. Everything else is decided by geometry we measure — a footprint, a
lot line, a road tangent — but a seat pan is INSIDE an asset nobody modelled
for passengers, and where it is varies per car with no attribute that says so.
The bench that covers every survivor situation at once
(`people_showcase_launch_script.py`) can tell you that this looks wrong; it
cannot tell you by how much, because the figure is 30 px tall in every useful
shot of it.

So: five cars, 16 m apart, on asphalt, each with one occupant, and three
cameras per car — from above, from the side at window height, and through the
windscreen. Nothing else is built.

HOW TO TUNE IT
--------------
`CARS` below is the whole interface. One row per car, and every column is
an offset from what the code would otherwise pick:

    seat_dz   raise (+) or lower (-) the seat pan, metres. The default pan is
              a third of the roof height, which is about right for a saloon
              and wrong for anything with a high floor.
    fwd       move the occupant toward the nose (+) or the boot (-), metres.
              The default sits them on the car's centre, and a driver's seat
              is ahead of that on most cars and behind it on a van.
    lat       move them toward the driver's side (+) or the passenger's (-).
    dyaw      turn them, degrees, relative to the car's heading. Should be 0
              on every correctly-authored car; a non-zero value here is
              recording an asset defect, not a preference.
    pose      which entry in `scene_generator._HUMAN_POSES` to bind.

Change a row, relaunch, look. The banner prints the resolved numbers for every
car so a screenshot of it is a complete record of what produced the picture.

WHAT TO JUDGE, PER CAR
----------------------
  1. Is the head UNDER the roof and clear of it?
  2. Are the hips at the seat, rather than the figure standing through the
     floor or floating above the seat squab?
  3. Is the body along the car, facing the nose?
  4. Can you SEE any of that from outside? Three of these five have glass that
     can be removed; two do not, and on those the answer is no by
     construction. That is a property of the art and the reason this bench
     shows both kinds side by side.
"""

import math
import os
import random
import sys

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": False,
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                   "--/rtx/pathtracing/fractionalCutoutOpacity=true"],
})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.flowusd")
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

import scene_generator as sg                                    # noqa: E402
import suburb_scene as ss                                       # noqa: E402
from scene_prep import get_stage_meters_per_unit                # noqa: E402
from compile_disaster import load_scene_config                  # noqa: E402
from disaster import people as ppl                              # noqa: E402
from disaster import damage, fire                               # noqa: E402
from disaster import vegetation as veg                          # noqa: E402
from detail import vehicles as veh                              # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_wildfire")
ARCH_DIR = os.environ.get(
    "ARCH_DIR", "/isaac-sim/AirStack/scene_gen/assets/archetypes")
SNAP_DIR = os.environ.get("SNAP_DIR", "")
if SNAP_DIR:
    os.makedirs(SNAP_DIR, exist_ok=True)

GRASS_MAT = "airstack://scene_gen/assets/materials/Grass_Cut.usda"
ASPHALT_MAT = ("airstack://scene_gen/assets/materials/megascans/"
               "Road_Asphalt.usda")
# The photographed charred surface the vegetation pass uses on burnt wood.
# A flat dark OmniPBR reads as painted PIPE at this scale — it has no normal
# or ORM map, so nothing catches the light along a cylinder. This one is
# world-triplanar at ~9 m a tile, so a 12 m log shows no repeat.

_LIB = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/"

# ---------------------------------------------------------------------------
# THE TABLE. This is the file's interface — see HOW TO TUNE IT above.
#
# `x` IS PINNED PER CAR rather than derived from the row order, so that a car
# keeps its world position when another is added or dropped. Every offset
# below was read off the stage as an absolute prim translate and converted
# once; if the cars moved, all of those numbers would silently mean something
# else the next time somebody quoted a coordinate at this bench.
# ---------------------------------------------------------------------------
CARS = [
    {
        "label": "130", "x": -32.0,
        "usd": _LIB + "RetroNeighborhood/130.usdz",
        "strip_extra": (),
        "occupants": [
            # Driver and passenger, both with their arms down: there is no
            # steering wheel modelled to put a hand on.
            dict(fwd=-0.4446, lat=+0.3599, dz=-0.0439, dyaw=0.0,
                 pose="seated_car_arms_down"),
            dict(fwd=-0.4446, lat=-0.4525, dz=-0.0439, dyaw=0.0,
                 pose="seated_car_arms_down"),
        ],
    },
    {
        "label": "fairlady", "x": -16.0,
        "usd": _LIB + "RetroNeighborhood/"
                      "Nissan_Fairlady_Z_S30240Z_1978.usdz",
        # `Object_16` is opaque and sits between the camera and the cabin.
        # It is not caught by `strip_glass` because this asset's transparent
        # materials are also bound to bodywork (see `shared.yaml`), so the
        # mesh is named here instead of widening a rule that would take the
        # roof off with it.
        "strip_extra": ("Object_16",),
        "occupants": [
            dict(fwd=-0.4589, lat=-0.4425, dz=-0.0794, dyaw=0.0,
                 pose="seated_car_arms_down"),
            dict(fwd=-0.4589, lat=+0.3813, dz=-0.0794, dyaw=0.0,
                 pose="seated_car_arms_down"),
        ],
    },
    {
        # A VAN, not a motorhome, and the useful one of the three: it is tall
        # enough that a seated adult clears the roof with room to spare, and
        # its glass comes off cleanly.
        "label": "van", "x": 32.0,
        "usd": _LIB + "RetroNeighborhood/"
                      "FREE_GMC_Motorhome_reimagined_low_poly.usdz",
        "strip_extra": (),
        "occupants": [
            dict(fwd=+2.7250, lat=+0.7700, dz=+0.3382, dyaw=0.0,
                 pose="seated_car_arms_down"),
            # THE REAR SEAT FACES BACKWARDS in this van, so its occupant is
            # turned to match the seat rather than the vehicle: -120 absolute
            # against the +90 every other occupant carries, i.e. -210 relative.
            dict(fwd=+2.7250, lat=-0.6797, dz=+0.3382, dyaw=-210.0,
                 pose="seated_car_arms_down"),
        ],
    },
]

# DROPPED, and worth recording so they are not tried again: the modular kit's
# `Car_01_0` and DownTown's `Vehicle_A`. Both are correctly sized and both are
# unusable for an occupant — their windows are painted into a single-mesh
# texture with no separable glass, so a person inside is invisible from every
# angle. They stay in the asset set for parked cars, which is all they can do.

SPACING_M = 16.0
# A seated adult needs about this much room above the seat pan. Used only to
# report whether a car can hold one, never to move anybody.
HEAD_ROOM_M = 0.85


def _mat(stage, key, url):
    path = "/World/ground/materials/" + key
    prim = stage.DefinePrim(Sdf.Path(path))
    prim.GetReferences().AddReference(sg._join_asset_root(url, ""))
    prim.Load()
    return path


def build_ground(stage, ssf, n):
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground"))
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground/materials"))
    grass = _mat(stage, "grass", GRASS_MAT)
    asphalt = _mat(stage, "asphalt", ASPHALT_MAT)
    span = SPACING_M * n
    sg._make_plane_mesh(stage, "/World/ground/lawn", -span, -60.0, span, 60.0,
                        -0.01, 3.0, ssf, display_color=(0.24, 0.36, 0.17),
                        mat_prim_path=grass)
    # A strip of asphalt under the row, so each car reads as parked rather
    # than as an asset floating on a lawn.
    # The carriageway runs WELL PAST the last car, because the point of the
    # blockage is that there is road beyond it that nobody can reach.
    sg._make_plane_mesh(stage, "/World/ground/pad", -span * 0.62, -5.5,
                        span * 0.62 + 40.0, 5.5, 0.02, 3.0, ssf,
                        display_color=(0.21, 0.23, 0.26),
                        mat_prim_path=asphalt)

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(1000.0)
    dome.CreateColorAttr(Gf.Vec3f(0.76, 0.80, 0.87))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(2200.0)
    key.CreateAngleAttr(0.9)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-48.0, 0.0, 28.0))


# Everything in the blockage is lifted to this height. The carriageway is at
# 0.02 and a piece authored at 0 sinks into it; 0.1 sets it ON the road with a
# visible contact shadow, which is what makes it read as lying there.
BLOCK_Z_M = 0.10


def _tube(stage, path, p0, p1, r0, r1, ssf, sides=10):
    """A tapered cylinder from p0 to p1 — one log, one limb.

    THE BLOCKAGE IS BUILT, NOT REFERENCED, and that is the whole point of this
    function. A `tree_*_fallen` archetype is a tree that GREW where you put it
    and then came down: it brings a stump rooted at its origin and a 10 m disc
    of ground debris, so dropping one on a carriageway plants a stump in the
    tarmac and rings it with litter that no road would have. What a blocked
    road actually has is the part that came off — trunk sections and limbs
    lying across it, with the stump back at the verge where the tree stood.
    So the timber here is authored directly, which also means its height is a
    number this file chooses rather than one baked into an asset.
    """
    ax, ay, az = p0
    bx, by, bz = p1
    dx, dy, dz = bx - ax, by - ay, bz - az
    ln = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
    ux, uy, uz = dx / ln, dy / ln, dz / ln
    # Any vector not parallel to the axis gives the first radial direction.
    tmp = (0.0, 0.0, 1.0) if abs(uz) < 0.9 else (1.0, 0.0, 0.0)
    vx = uy * tmp[2] - uz * tmp[1]
    vy = uz * tmp[0] - ux * tmp[2]
    vz = ux * tmp[1] - uy * tmp[0]
    vl = math.sqrt(vx * vx + vy * vy + vz * vz) or 1.0
    vx, vy, vz = vx / vl, vy / vl, vz / vl
    wx = uy * vz - uz * vy
    wy = uz * vx - ux * vz
    wz = ux * vy - uy * vx

    pts, counts, idx = [], [], []
    for k in range(sides):
        a = 2.0 * math.pi * k / sides
        ca, sa = math.cos(a), math.sin(a)
        for (px, py, pz, r) in ((ax, ay, az, r0), (bx, by, bz, r1)):
            pts.append(Gf.Vec3f((px + (vx * ca + wx * sa) * r) * ssf,
                                (py + (vy * ca + wy * sa) * r) * ssf,
                                (pz + (vz * ca + wz * sa) * r) * ssf))
    for k in range(sides):
        a0, a1 = 2 * k, 2 * ((k + 1) % sides)
        counts.append(4)
        idx += [a0, a1, a1 + 1, a0 + 1]
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    return m.GetPrim()


def _bind(stage, prim, mat_path):
    """Bind a material referenced by PATH (as `_mat` returns it)."""
    if not mat_path:
        return
    mat = UsdShade.Material.Get(stage, mat_path)
    if not mat:
        for c in stage.GetPrimAtPath(mat_path).GetChildren():
            if c.IsA(UsdShade.Material):
                mat = UsdShade.Material(c)
                break
    if mat:
        UsdShade.MaterialBindingAPI(prim).Bind(mat)


def build_blockage(stage, ssf, arch, rng):
    """Timber and house wreckage across the carriageway, and it is burning.

    Composed by hand rather than scattered: a blockage is a specific pile in a
    specific place, and the thing being judged is whether a driver could get
    past it. Three layers, from the road up —

      TRUNK SECTIONS  the big pieces, laid across the lane at different
                      bearings and overlapping each other. These are what
                      actually stop a car; a road covered in twigs does not.
      LIMBS           thinner pieces around and on top of them, so the pile
                      has a scale range instead of reading as milled poles.
      HOUSE WRECKAGE  `*_rubble` archetypes at the kerb, sited so their spread
                      reaches well onto the asphalt. This is the second half
                      of a real WUI blockage and the half people forget.

    Standing burnt trunks go at the VERGE, never on the road — a stump in the
    tarmac is the tell that a whole tree archetype was dropped where a fallen
    one was wanted.
    """
    rng = rng or __import__("random").Random(7)
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT + "/blockage"))
    # Composited soot, not the Megascans burnt-ground surface: a
    # photographed forest floor wrapped on a trunk reads as ground, and
    # no tree in this dataset carries it. Same path every generated wood
    # piece takes (`vegetation.wood_material` with no bark map).
    _wood = veg.wood_material(stage, {}, PARENT + "/blockage",
                              coverage=0.94)
    char = str(_wood.GetPath()) if _wood else ""
    n_log = 0

    # -- the trunk sections ------------------------------------------------
    # (x, y, bearing deg, length, butt radius, tip radius, lift at the far end)
    TRUNKS = [
        (60.0,  0.5,  74.0, 11.0, 0.46, 0.34, 0.00),
        (64.5, -1.5, 108.0,  9.5, 0.40, 0.28, 0.55),
        (70.0,  1.0,  56.0, 12.0, 0.52, 0.36, 0.00),
        (75.5, -0.5,  96.0,  8.5, 0.38, 0.26, 0.35),
        (81.0,  1.5,  68.0, 10.5, 0.44, 0.30, 0.00),
        (86.5, -1.0, 122.0,  7.5, 0.34, 0.24, 0.70),
    ]
    for i, (x, y, brg, ln, r0, r1, lift) in enumerate(TRUNKS):
        a = math.radians(brg)
        hx, hy = math.cos(a) * ln / 2.0, math.sin(a) * ln / 2.0
        z0 = BLOCK_Z_M + r0
        z1 = BLOCK_Z_M + r1 + lift
        prim = _tube(stage, "{0}/blockage/log_{1}".format(PARENT, i),
                     (x - hx, y - hy, z0), (x + hx, y + hy, z1),
                     r0, r1, ssf)
        _bind(stage, prim, char)
        n_log += 1

    # -- the limbs ---------------------------------------------------------
    for i in range(46):
        x = rng.uniform(55.0, 92.0)
        y = rng.uniform(-5.0, 5.0)
        a = rng.uniform(0.0, math.pi)
        ln = rng.uniform(1.2, 4.2)
        r = rng.uniform(0.05, 0.15)
        # A few rest ON the trunks rather than on the road.
        z = BLOCK_Z_M + r + (rng.uniform(0.3, 0.8) if rng.random() < 0.3
                             else 0.0)
        hx, hy = math.cos(a) * ln / 2.0, math.sin(a) * ln / 2.0
        tilt = rng.uniform(-0.25, 0.25)
        prim = _tube(stage, "{0}/blockage/limb_{1}".format(PARENT, i),
                     (x - hx, y - hy, z), (x + hx, y + hy, z + tilt),
                     r, r * rng.uniform(0.6, 0.9), ssf, sides=6)
        _bind(stage, prim, char)
        n_log += 1

    # -- house wreckage and the standing stand -----------------------------
    # The rubble heaps are pulled IN toward the centreline so their spread
    # crosses the kerb; the standing trunks stay out at the verge.
    refs = [
        ("house_cottage_rubble",          68.0,  9.0,  28.0),
        ("house_l_bungalow_rubble",       84.0, -8.5, 200.0),
        ("house_ranch_rubble",            96.0,  7.5, 120.0),
        ("tree_Black_Oak_snag",           52.0, 12.5,   0.0),
        ("tree_Shumard_Oak_torched",      62.0,-12.5,   0.0),
        ("tree_American_Beech_snag",      90.0, 12.0,   0.0),
        ("tree_Largetooth_Aspen_torched",100.0,-12.0,   0.0),
    ]
    n_ref = 0
    for i, (key, x, y, yaw) in enumerate(refs):
        usd = (arch or {}).get(key)
        if not usd:
            continue
        path = "{0}/blockage/ref_{1}_{2}".format(PARENT, key, i)
        # Typed "Xform": a bare DefinePrim lets the referenced `/Baked` Scope
        # type win, and a Scope is not Xformable, so the transform is dropped
        # and every archetype stacks up at the origin.
        prim = stage.DefinePrim(Sdf.Path(path), "Xform")
        if not prim.GetReferences().AddReference(usd):
            continue
        prim.Load()
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf,
                                         BLOCK_Z_M * 0.5 * ssf))
        xf.AddRotateZOp().Set(float(yaw))
        n_ref += 1

    print("[bench] blockage: {0} authored piece(s), {1} archetype(s)"
          .format(n_log, n_ref))
    return n_log


def build_fire(stage, ssf):
    """Flow fire and smoke over the pile, so it reads as still burning.

    One stack, three emitters: a flaming pocket in the timber and two
    smouldering ones in the wreckage either side. `set_emission` carries the
    fuel/smoke/temperature triple per state — temperature is the GLOW, so a
    smoke-only emitter that still injects it renders as a glowing ball with no
    smoke.
    """
    root = "/World/flow_blockage"
    fire.setup_flow_stack(stage, density_cell_size_m=0.14, max_blocks=16384,
                          scene_scale_factor=ssf, root=root)
    spots = [("flame", 72.0, 0.5, 1.1, (3.0, 2.2, 1.4)),
             ("rubble", 68.0, 9.0, 1.0, (3.4, 3.0, 1.2)),
             ("rubble", 84.0, -8.5, 1.0, (3.2, 2.8, 1.2)),
             ("smoke", 60.0, 0.5, 0.9, (2.4, 2.0, 1.0))]
    made = 0
    for i, (state, x, y, z, half) in enumerate(spots):
        prim = fire._flow_create(stage, "{0}/emitter_{1}".format(root, i),
                                 "FlowEmitterBox")
        if not prim or not prim.IsValid():
            continue
        fire._set(prim, "layer", Sdf.ValueTypeNames.Int, fire.FLOW_LAYER)
        fire._set(prim, "position", Sdf.ValueTypeNames.Float3,
                  Gf.Vec3f(x * ssf, y * ssf, z * ssf))
        fire._set(prim, "halfSize", Sdf.ValueTypeNames.Float3,
                  Gf.Vec3f(half[0] * ssf, half[1] * ssf, half[2] * ssf))
        fire.set_emission(prim, state, scale=1.0)
        made += 1
    print("[bench] fire: {0} emitter(s) over the blockage".format(made))
    return made


def main():
    omni.timeline.get_timeline_interface().stop()
    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))

    _mpu, ssf = get_stage_meters_per_unit(stage)
    config = load_scene_config(SCENE_CONFIG)
    build_ground(stage, ssf, 5)

    pools = ss.AssetPools(config)
    resolver = sg._make_resolver(config)
    raw_h = ss._raw_pool(config, "humans")
    raw_c = ss._raw_pool(config, "cars")
    humans = pools.load_tagged(raw_h, "rigged") or pools.load(raw_h)
    glassy = frozenset(pools.load_tagged(raw_c, "glass_separable"))
    ctx = {"asset_pools": pools, "resolver": resolver, "glassy": glassy}

    arch = {os.path.splitext(f)[0]: os.path.join(ARCH_DIR, f)
            for f in os.listdir(ARCH_DIR) if f.endswith(".usd")} \
        if os.path.isdir(ARCH_DIR) else {}
    build_blockage(stage, ssf, arch, random.Random(7))
    build_fire(stage, ssf)

    cars, people, rows, fpm = [], [], [], {}
    for spec in CARS:
        usd, x = spec["usd"], float(spec["x"])
        car = ppl._car_placement(ctx, usd, x, 0.0, 0.0, "bench")
        car["strip_extra"] = tuple(spec.get("strip_extra") or ())
        cars.append(car)

        fp = resolver.get(usd, "car", scale=pools.scale_of(usd),
                          axis_up=pools.axis_of(usd))
        roof = float(fp.get("sz", 1.5))
        for k, occ in enumerate(spec["occupants"]):
            seat = max(0.05, roof / 3.0 + float(occ.get("dz", 0.0)))
            rad = math.radians(0.0)          # heading 0 for the whole row
            fwd, lat = float(occ.get("fwd", 0.0)), float(occ.get("lat", 0.0))
            hx = x + math.cos(rad) * fwd - math.sin(rad) * lat
            hy = math.sin(rad) * fwd + math.cos(rad) * lat
            hu = humans[(len(people) + k) % len(humans)]
            people.append(ppl._human_placement(
                ctx, hu, hx, hy, seat, float(occ.get("dyaw", 0.0)),
                occ.get("pose", "seated_car_arms_down")))
        rows.append((spec["label"], os.path.basename(usd), roof,
                     len(spec["occupants"]), usd in glassy, x))
        fpm[spec["label"]] = max(float(fp.get("sx", 4.5)),
                                 float(fp.get("sy", 4.5)))

        post = UsdGeom.Cube.Define(stage,
                                   Sdf.Path(f"{PARENT}/post_{spec['label']}"))
        post.CreateSizeAttr(1.0)
        px = UsdGeom.Xformable(post)
        px.ClearXformOpOrder()
        px.AddTranslateOp().Set(Gf.Vec3d(x * ssf, -7.5 * ssf, 0.9 * ssf))
        px.AddScaleOp().Set(Gf.Vec3f(0.3 * ssf, 0.3 * ssf, 1.8 * ssf))
        post.CreateDisplayColorAttr([Gf.Vec3f(0.86, 0.24, 0.20)])

    # CARS, THEN GLASS, THEN PEOPLE — the plat's order, and load-bearing: a
    # passenger authored before the windows come off is a passenger you cannot
    # see, which is the whole question this bench asks.
    sg.apply_placements(stage, cars, PARENT + "/cars", ssf, resolver=resolver,
                        instance_categories=set())
    n_glass = n_extra = 0
    for q in cars:
        if not q.get("prim_path"):
            continue
        if q.get("glass_separable"):
            n_glass += veh.strip_glass(stage, q["prim_path"])
        # NAMED MESHES, deactivated one by one. Some assets carry an opaque
        # panel across the cabin that no rule can tell from bodywork; naming
        # it is honest, and narrower than loosening `strip_glass` until it
        # catches the panel and the roof together.
        want = set(q.get("strip_extra") or ())
        if want:
            root = stage.GetPrimAtPath(q["prim_path"])
            doomed = [p.GetPath() for p in Usd.PrimRange(root)
                      if p.IsA(UsdGeom.Mesh) and p.GetName() in want]
            for path in doomed:
                pr = stage.GetPrimAtPath(path)
                if pr and pr.IsValid() and pr.SetActive(False):
                    n_extra += 1
    sg.apply_placements(stage, people, PARENT + "/people", ssf,
                        resolver=resolver, instance_categories=set())

    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    app = omni.kit.app.get_app()
    for _ in range(30):
        app.update()

    print("\n" + "=" * 78)
    print("OCCUPANT BENCH — {0} cars, {1} people, {2} glass + {3} named "
          "mesh(es) removed".format(len(cars), len(people), n_glass, n_extra))
    print("  {0:<10} {1:<34} {2:>6} {3:>5} {4:>4}  {5}".format(
        "label", "asset", "x", "roof", "occ", "glass"))
    for (label, base, roof, nocc, gl, x) in rows:
        print("  {0:<10} {1:<34} {2:6.1f} {3:5.2f} {4:>4}  {5}".format(
            label, base[:34], x, roof, nocc, "strip" if gl else "baked"))
    print("\n  Edit CARS at the top of this file to tune any row.")
    print("=" * 78 + "\n")

    # FLOW NEEDS TIME BEFORE IT IS PHOTOGRAPHED. The emitters inject fuel per
    # step, so a capture taken at t=0 shows an empty grid — the fire is not a
    # static prop that is either on or off. Start the timeline and pump frames
    # until the plume has grown, THEN take the pictures.
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(240):
        app.update()

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots", _sp)
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            for (label, base, roof, nocc, gl, x) in rows:
                # Straight down, from the side at window height, and from
                # ahead through the windscreen. The side view is the one that
                # answers "are the hips on the seat"; the top view answers
                # "is the body along the car".
                # FRAMED TO THE CAR, not to a constant. A fixed 5.2 m
                # stand-off suits a coupe and puts the camera inside an 8.4 m
                # van — the first van capture was a close-up of one door
                # panel. Everything scales off the measured length instead.
                ln = max(4.0, float(fpm.get(label, 4.5)))
                _snaps.place_camera(stage, (x * ssf, 0.0, (ln * 2.2) * ssf),
                                    (x * ssf, 0.0, 0.0))
                _snaps.snapshot(os.path.join(SNAP_DIR, f"{label}_top.png"))
                _snaps.place_camera(stage,
                                    (x * ssf, -(ln * 1.25) * ssf,
                                     (roof * 0.80) * ssf),
                                    (x * ssf, 0.0, (roof * 0.55) * ssf),
                                    focal_mm=30.0)
                _snaps.snapshot(os.path.join(SNAP_DIR, f"{label}_side.png"))
                _snaps.place_camera(stage,
                                    ((x + ln * 1.15) * ssf, 0.0,
                                     (roof * 0.85) * ssf),
                                    (x * ssf, 0.0, (roof * 0.6) * ssf),
                                    focal_mm=30.0)
                _snaps.snapshot(os.path.join(SNAP_DIR, f"{label}_front.png"))
            _snaps.place_camera(stage, (40.0 * ssf, -34.0 * ssf,
                                        17.0 * ssf),
                                (66.0 * ssf, 0.0, 1.0 * ssf), focal_mm=24.0)
            _snaps.snapshot(os.path.join(SNAP_DIR, "blockage.png"))
            _snaps.overview(stage, (10.0, 0.0), 190.0,
                            os.path.join(SNAP_DIR, "row.png"), ssf)
            print("[bench] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            print("[bench] snapshots FAILED: {0}".format(exc))

    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
