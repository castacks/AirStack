#!/usr/bin/env python
"""
Preview launcher for the detailed city (scene_gen/generate_scene.py).

Deliberately a near-copy of scene_preview_launch_script.py — same base
environment, same stage prep, same lighting — so that running the two side by
side isolates the thing under test: the city itself. The only differences are
which generator entry point is called and which config it defaults to.

    SCENE_CONFIG=urban_v2 \
    ISAAC_SIM_SCRIPT_NAME=scene_launch_script.py \
    airstack up isaac-sim

Compare against the unchanged original with:

    SCENE_CONFIG=none \
    ISAAC_SIM_SCRIPT_NAME=scene_preview_launch_script.py \
    airstack up isaac-sim

No drone, no sensors, no ROS 2 bridge, so startup is fast and the scene can be
regenerated in place after editing the YAML — see RELOAD below.

RELOAD WITHOUT RESTARTING
--------------------------
Open the Kit Script Editor (Window -> Script Editor) and run:

    import os, sys, importlib
    SCENE_GEN = "/isaac-sim/AirStack/scene_gen"
    ISAAC_DIR = "/isaac-sim/AirStack/simulation/isaac-sim"
    sys.path[:0] = [SCENE_GEN, os.path.join(ISAAC_DIR, "utils")]
    import scene_generator
    from detail import city_detail, districts, road_markings
    from layout import city_layout
    import generate_scene, scene_prep
    for m in (scene_generator, city_detail, city_layout, districts,
              road_markings, generate_scene):
        importlib.reload(m)          # pick up edits to the new passes too
    import omni.usd, omni.timeline
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().stop()
    _, ssf = scene_prep.get_stage_meters_per_unit(stage)
    # add_colliders_skip_empty, not scene_prep.add_colliders — the stock one
    # floods the log with PhysX errors for point-less stub meshes.
    import scene_launch_script as v2
    generate_scene.reload_scene_on_stage(
        stage, os.path.join(SCENE_GEN, "config", "presets", "urban_v2.yaml"),
        scene_scale_factor=ssf, add_colliders_fn=sl.add_colliders_skip_empty)
    omni.timeline.get_timeline_interface().play()
"""

import os
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension
# The reload loop above depends on the Script Editor every run.
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics
from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)
from scene_prep import (ensure_scene_queries, scale_stage_prim, add_colliders, add_sky,
                        get_stage_meters_per_unit, settle_rigid_props,
                        settle_selection)
from scene_generator import resolve_sky
from generate_scene import generate_scene_on_stage
from compile_disaster import load_scene_config
from disaster import kinds

# ----- CONFIGURATION -----
ENV_URL     = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
SCENE_CONFIG = os.environ.get("SCENE_CONFIG") or os.path.join(
    _SCENE_GEN_DIR, "config", "presets", "urban_v2.yaml")
# -------------------------


_ENV_CLUTTER = {"GroundPlane", "Environment"}

#: Where to write review captures; empty = none (the default run).
SNAP_DIR = os.environ.get("SNAP_DIR", "").strip()

#: The rungs that read as a collapse — what a close-up should be of.
_COLLAPSED = ("pancaked", "partial_collapse", "soft_storey")


#: How many wreck close-ups the review writes (`REVIEW_WRECKS=` to override).
REVIEW_WRECKS = int(os.environ.get("REVIEW_WRECKS", "8") or 8)

#: `REVIEW_SURVEY=N` adds a SURVEY on top of the review set: N azimuths of the
#: whole map from three elevations, then street-level and rubble close-ups.
#: 0 (the default) leaves the run exactly as it was. 12 gives ~57 frames.
REVIEW_SURVEY = int(os.environ.get("REVIEW_SURVEY", "0") or 0)

#: `SCENE_CENSUS=<path>` writes down which asset types this run actually
#: placed. Worth doing HERE rather than on the host because packing keys off
#: measured footprints, and only this process can open a Nucleus asset to
#: measure one — see `archetypes/census.py`. It is what tells the baker which
#: archetypes a scene depends on.
SCENE_CENSUS = os.environ.get("SCENE_CENSUS", "").strip()

#: Camera geometry, from `utils/snapshots.py`: 18 mm on a 20.955 mm aperture,
#: 16:9. The half-angles are what decides how far back the whole map has to be
#: photographed from, and getting them wrong is how an overview comes out as a
#: picture of the middle third of the scene.
_TAN_H = (20.955 / 2.0) / 18.0                 # 0.582 — horizontal
_TAN_V = _TAN_H * 9.0 / 16.0                   # 0.327 — vertical


def _write_people_gt(config):
    """Survivor ground-truth boxes for the people Stage C just placed.

    Off unless `GT_ANNOTATIONS=1` and `RESULTS_SCENE=<name>` are set — see
    `utils/scene_annotations.gt_from_env`. Never fails the run: a scene without
    ground truth is still a scene, and a launch that died writing a JSON file
    after the stage was built would be the worst trade in this script.
    """
    try:
        import scene_annotations as sa

        sa.gt_for_config(config)
    except Exception as exc:
        print(f"[annotations] skipped: {exc}")


def review_snapshots(stage, config, placements, victims, ssf, out_dir):
    """The G2 review set: overview, street obliques, wrecks, one victim.

    Everything in metres, `snapshots` applies `ssf`. Files are numbered in
    the order a reviewer should look at them and named by what they show.
    """
    import importlib.util as _ilu
    import math

    spec = _ilu.spec_from_file_location(
        "snapshots", os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py"))
    snaps = _ilu.module_from_spec(spec)
    spec.loader.exec_module(snaps)

    def shot(name, eye, target, frames=48):
        snaps.place_camera(stage, tuple(v * ssf for v in eye),
                           tuple(v * ssf for v in target))
        return snaps.snapshot(os.path.join(out_dir, name), frames)

    region = (config.get("layout") or {}).get("region_m") or [200, 200]
    span = float(max(region))
    field = (config.get("disaster") or {}).get("field") or {}
    ex, ey = (float(v) for v in (field.get("center") or (0.0, 0.0)))
    houses = [p for p in placements if p.get("category") == "house"]
    # Footprints, so "clear of a building" means clear of its EXTENT: a
    # centre-distance test put the eye inside an 80 m tower whose centre was
    # 45 m away. Circumscribed radius plus a margin, from the resolver's
    # measured (and cached) footprints.
    import scene_generator as sg
    resolver = sg._make_resolver(config)
    blocks = []
    tallest = 0.0
    size = {}                       # placement id -> (radius_m, height_m)
    for p in houses:
        h = 0.0
        try:
            fp = sg.placement_footprint(resolver, p, "house")
            r = 0.5 * math.hypot(float(fp.get("sx", 20.0)),
                                 float(fp.get("sy", 20.0)))
            h = float(fp.get("sz", 0.0))
            tallest = max(tallest, h)
        except Exception:                                     # noqa: BLE001
            r = 15.0
        size[id(p)] = (r, h)
        blocks.append((float(p["x_m"]), float(p["y_m"]), r + 4.0))

    def oblique(name, x, y, dist, h, tz, away_from=None):
        """Eye *dist* out at height *h*, on the bearing with the most room.

        A fixed south-west eye lands inside a neighbouring tower in a dense
        downtown (measured: five of seven first captures were of the inside
        of a wall). Score each of 16 bearings by the distance from the eye to
        the nearest OTHER building centre, and prefer the side away from
        *away_from* — a victim's own building, say.
        """
        best, best_s = None, -1e9
        for k in range(16):
            a = 2.0 * math.pi * k / 16.0
            e = (x + dist * math.cos(a), y + dist * math.sin(a))
            s_ = min((math.hypot(e[0] - cx, e[1] - cy) - r
                      for cx, cy, r in blocks
                      if math.hypot(cx - x, cy - y) > 3.0), default=1e9)
            if away_from is not None:
                bx, by = away_from[0] - x, away_from[1] - y
                nb = math.hypot(bx, by) or 1.0
                s_ -= 30.0 * (bx * math.cos(a) + by * math.sin(a)) / nb
            if s_ > best_s:
                best, best_s = e, s_
        return shot(name, (best[0], best[1], h), (x, y, tz))
    wrecks = sorted((p for p in houses
                     if p.get("_damage_level") in _COLLAPSED),
                    key=lambda p: -float(p.get("_mesh_damage") or
                                         (1.0 if p.get("_archetype") else 0.0)))
    wrecks.sort(key=lambda p: math.hypot(p["x_m"] - ex, p["y_m"] - ey))

    # A KEY LIGHT FOR THE REVIEW ONLY. The dome alone leaves the scene dim
    # and every north face black; `_disable_sky_sun` switches the sky rig's
    # own sun off on purpose (it is a hard directional light on top of the
    # dome), so the review adds its own, angled from the south-west where
    # the obliques look from, and leaves the default run's lighting alone.
    from pxr import Gf, Sdf, UsdLux
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/ReviewKey"))
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(1.0)
    UsdGeom.Xformable(key.GetPrim()).ClearXformOpOrder()
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(-50.0, 0.0, 35.0))

    written = []
    # 1. the whole map, plumb
    # High enough to clear the tallest tower with room to spare: on a 105 m
    # map the eye at 0.95 x span sat just above a 70 m roof and the overview
    # was a picture of that roof.
    # FRAME THE MAP, not the tallest asset. `1.6 * tallest + 60` put the eye
    # 270 m up over a 200 m block — the whole scene rendered as a small square
    # in the middle. Clearing the tallest roof is a floor on the height, not
    # the thing being framed.
    written.append(snaps.overview(stage, (ex, ey),
                                  max(span * 1.05, tallest + 40.0),
                                  os.path.join(out_dir, "01_overview_plumb.png"),
                                  ssf, frames=48))
    # 2. three street-height obliques: the epicentre and two other spots
    spots = [("epicentre", ex, ey)]
    far = sorted(houses, key=lambda p: -math.hypot(p["x_m"] - ex, p["y_m"] - ey))
    if far:
        spots.append(("far_corner", far[0]["x_m"], far[0]["y_m"]))
    mid = [p for p in houses if 0.25 * span < math.hypot(p["x_m"] - ex,
                                                          p["y_m"] - ey)]
    if mid:
        spots.append(("mid_block", mid[len(mid) // 2]["x_m"],
                      mid[len(mid) // 2]["y_m"]))
    for i, (name, x, y) in enumerate(spots[:3], start=2):
        written.append(oblique(f"{i:02d}_street_oblique_{name}.png",
                               x, y, dist=60.0, h=14.0, tz=6.0))
    # 3. close-ups of collapsed buildings, nearest the epicentre first. EVERY
    # instance, not the first two: the library places the same asset several
    # times and they do not come out alike — a texture or a material that is
    # wrong on one instance and right on the others is invisible in a review
    # that only ever photographs two of them.
    # FRAME THE BUILDING, NOT A FIXED 48 m. These assets run from a 20 m shed
    # to a 96 m tower, and one distance cannot hold both: at 48 m the camera
    # sat INSIDE an 81 m wreck and photographed the inside of its own facade,
    # or the sky through it. Pull back with the footprint and climb with the
    # height, and aim a third of the way up rather than at the ground.
    for i, p in enumerate(wrecks[:REVIEW_WRECKS], start=5):
        stem = os.path.splitext(os.path.basename(str(p.get("usd", "b"))))[0]
        r, hgt = size.get(id(p), (15.0, 0.0))
        written.append(oblique(
            f"{i:02d}_collapsed_{p.get('_damage_level')}_{stem}.png",
            p["x_m"], p["y_m"],
            dist=max(45.0, 1.6 * r + 25.0),
            h=max(20.0, 0.55 * hgt + 12.0),
            tz=max(4.0, 0.3 * hgt)))
    # 4. one victim, the most visible one (numbered after the wrecks)
    order = {"open": 0, "partial": 1, "occluded": 2}
    vis = sorted((v for v in victims if "x" in v and "y" in v),
                 key=lambda v: order.get(str(v.get("visibility")), 3))
    if vis:
        v = vis[0]
        home = next(((float(p["x_m"]), float(p["y_m"])) for p in houses
                     if p.get("prim_path") == v.get("building")), None)
        written.append(oblique(
            f"{5 + min(len(wrecks), REVIEW_WRECKS):02d}_victim_"
            f"{v.get('cohort', 'x')}_{v.get('visibility', 'x')}.png",
            float(v["x"]), float(v["y"]), dist=9.0, h=4.0,
            tz=float(v.get("z", 0.0)) + 0.8, away_from=home))
    written = [w for w in written if w]
    print(f"[snapshots] {len(written)} review capture(s) -> {out_dir}")
    return written


def survey_snapshots(stage, config, placements, ssf, out_dir, azimuths=12):
    """A SURVEY, on top of the seven-frame review: the whole map from all round
    it, then the ground the drones actually fly over.

    Three sets, prefixed so they sort into the order to look at them:

        A  the map from the upper hemisphere — *azimuths* bearings at three
           elevations, every camera aimed at the epicentre. This is what says
           whether the damage reads as an event from any approach, rather than
           only from the one bearing a fixed review camera happened to use.
        B  street level, looking down a corridor from behind a parked car.
           The cars ARE the street finder: `city_detail` parks them at the
           kerb aligned to the road, so a camera behind one along its yaw is
           looking down the street with no road geometry to query.
        C  rubble at eye height. Aimed at the BASE of a wreck rather than at
           its middle, because the debris a building shed is on the ground
           around it and a close-up framed on the silhouette misses it.

    Returns the paths written.

    FRAMING THE WHOLE MAP IS NOT ONE DISTANCE. The frame is 0.58 x distance
    wide and 0.33 x distance tall, and a map seen from 25 degrees up is wide
    and shallow while the same map from 70 degrees is nearly square — so the
    binding constraint swaps from the horizontal half-angle to the vertical
    one as the camera climbs. `overview()`'s fixed `span * 0.95` eye height is
    the plumb case of the same sum, and it crops a 500 m map to its middle
    third. Both terms are taken here and the larger one wins.
    """
    import importlib.util as _ilu
    import math

    spec = _ilu.spec_from_file_location(
        "snapshots", os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py"))
    snaps = _ilu.module_from_spec(spec)
    spec.loader.exec_module(snaps)

    def shot(name, eye, target, frames=40):
        snaps.place_camera(stage, tuple(v * ssf for v in eye),
                           tuple(v * ssf for v in target))
        return snaps.snapshot(os.path.join(out_dir, name), frames)

    region = (config.get("layout") or {}).get("region_m") or [200, 200]
    span = float(max(region))
    field = (config.get("disaster") or {}).get("field") or {}
    ex, ey = (float(v) for v in (field.get("center") or (0.0, 0.0)))
    houses = [p for p in placements if p.get("category") == "house"]
    written = []

    # ---- A. the upper hemisphere ------------------------------------------
    # 20 degrees is low enough that the towers occlude each other, which is
    # the view a drone climbing out actually has; 75 is near-plumb.
    for ei, elev in enumerate((20.0, 42.0, 75.0)):
        th = math.radians(elev)
        dist = max(0.5 * span / _TAN_H, 0.5 * span * math.sin(th) / _TAN_V) * 1.12
        for ai in range(int(azimuths)):
            az = 2.0 * math.pi * ai / float(azimuths)
            eye = (ex + dist * math.cos(th) * math.cos(az),
                   ey + dist * math.cos(th) * math.sin(az),
                   dist * math.sin(th))
            written.append(shot(
                f"A{ei}{ai:02d}_map_el{int(elev):02d}_az{int(math.degrees(az)):03d}.png",
                eye, (ex, ey, 0.0)))

    # Footprints first: both the street shots and the rubble shots need to know
    # where the buildings are, and measuring is the expensive part.
    import scene_generator as sg
    resolver = sg._make_resolver(config)
    radius = {}
    for p in houses:
        try:
            fp = sg.placement_footprint(resolver, p, "house")
            radius[id(p)] = 0.5 * math.hypot(float(fp.get("sx", 20.0)),
                                             float(fp.get("sy", 20.0)))
        except Exception:                                     # noqa: BLE001
            radius[id(p)] = 15.0
    blocks = [(float(p["x_m"]), float(p["y_m"]), radius[id(p)] + 3.0)
              for p in houses]
    hw, hh = 0.5 * float(region[0]), 0.5 * float(region[1])

    def clearance(x, y):
        """Distance from (x, y) to the nearest building's footprint circle.

        Negative means inside one. A camera does not survive that: the frame
        comes back black, or a picture of the inside of a wall.
        """
        return min((math.hypot(x - bx, y - by) - br for bx, by, br in blocks),
                   default=1e9)

    # ---- B. down the street ------------------------------------------------
    # Spread over the map rather than taken in placement order, which is
    # packing order and would photograph one corner eight times.
    #
    # BOTH WAYS ALONG THE KERB, and the better one wins. A parked car's yaw
    # says which way the road runs, not which way there is anything to see:
    # taking it literally put the camera inside the building behind the car
    # (black frame) or aimed it off the edge of the map at empty ground —
    # measured, 4 of 10 on the first run. Score each direction by how clear the
    # eye is and whether the target is still on the map.
    cars = [p for p in placements if p.get("category") == "car"]
    cars.sort(key=lambda p: math.atan2(p["y_m"] - ey, p["x_m"] - ex))
    step = max(1, len(cars) // 10)
    for i, p in enumerate(cars[::step][:10]):
        a = math.radians(float(p.get("yaw_deg", 0.0)))
        x, y = float(p["x_m"]), float(p["y_m"])
        best, best_s = None, -1e9
        for sign in (1.0, -1.0):
            ca, sa = sign * math.cos(a), sign * math.sin(a)
            e = (x - 11.0 * ca, y - 11.0 * sa)
            t = (x + 55.0 * ca, y + 55.0 * sa)
            on_map = abs(t[0]) <= hw and abs(t[1]) <= hh
            s_ = clearance(*e) + (25.0 if on_map else -60.0)
            if s_ > best_s:
                best, best_s = (e, t), s_
        (e, t) = best
        written.append(shot(f"B{i:02d}_street.png",
                            (e[0], e[1], 3.2), (t[0], t[1], 3.0)))

    # ---- C. rubble, at the base --------------------------------------------
    wrecks = [p for p in houses if p.get("_damage_level") in _COLLAPSED
              or p.get("_debris_r_m")]
    wrecks.sort(key=lambda p: math.hypot(p["x_m"] - ex, p["y_m"] - ey))
    for i, p in enumerate(wrecks[:12]):
        x, y = float(p["x_m"]), float(p["y_m"])
        # Out past the debris ring and low, aimed at the ground at the facade.
        d = max(28.0, radius[id(p)] + float(p.get("_debris_r_m") or 0.0) + 14.0)
        # THE BEARING WITH THE MOST ROOM, not a fixed one. A downtown block is
        # dense enough that a fixed south-west eye lands inside a neighbouring
        # tower and photographs the inside of its wall — measured on the review
        # set, five of seven. Score 16 bearings by clearance and take the best.
        # AND BACK OFF IF EVEN THE BEST BEARING IS BLOCKED. On a dense block
        # every bearing at the first distance can be inside something; two of
        # twelve were, measured on this scene. Stepping out is the only move
        # left, and it costs a wider frame rather than a lost picture.
        best, best_s = None, -1e9
        for grow in (1.0, 1.4, 1.9, 2.6):
            for k in range(16):
                a = 2.0 * math.pi * k / 16.0
                e = (x + d * grow * math.cos(a), y + d * grow * math.sin(a))
                # Its own footprint does not count — the camera is meant to be
                # looking AT this building.
                s_ = min((math.hypot(e[0] - bx, e[1] - by) - br
                          for bx, by, br in blocks
                          if math.hypot(bx - x, by - y) > 3.0), default=1e9)
                if s_ > best_s:
                    best, best_s = e, s_
            if best_s > 6.0:
                break
        stem = os.path.splitext(os.path.basename(str(p.get("usd", "b"))))[0][:28]
        written.append(shot(f"C{i:02d}_rubble_{stem}.png",
                            (best[0], best[1], 6.5), (x, y, 2.5)))

    written = [w for w in written if w]
    print(f"[snapshots] survey: {len(written)} capture(s) -> {out_dir}")
    return written


def _remove_env_clutter(stage):
    """Deactivate the GroundPlane and Environment xforms the base environment
    brings in. The generator lays its own ground, so these cause z-fighting and
    an unwanted visual backdrop — `Environment/Geometry` is the grid mesh that
    reads as a blue square under the spawn point.

    `default_environment.usd` has defaultPrim `/World`, which Pegasus references
    at `/World/stage`, so everything here composes from a referenced layer.
    `RemovePrim` cannot delete across a reference: it returns False rather than
    raising, so a try/except around it never fires and the prim stays visible.
    Deactivation does compose, and is reversible in the viewport.

    `SphereLight` is a sibling of these, not a child, so lighting survives.
    """
    n = 0
    for root_path in ("/", "/World", "/World/stage"):
        root = (stage.GetPseudoRoot() if root_path == "/"
                else stage.GetPrimAtPath(root_path))
        if not root or not root.IsValid():
            continue
        for child in root.GetChildren():
            if child.GetName() not in _ENV_CLUTTER or not child.IsActive():
                continue
            if child.SetActive(False):
                n += 1
                carb.log_info(f"[scene_gen] deactivated {child.GetPath()}")
            else:
                UsdGeom.Imageable(child).MakeInvisible()
                carb.log_info(f"[scene_gen] hid {child.GetPath()}")
    print(f"[scene_gen] env clutter: {n} prim(s) deactivated")


def _disable_sky_sun(stage):
    """Switch off the DistantLight the sky rig hangs under its axis chain.

    `add_sky` builds /World/Environment/sky/AxisNorth/.../DistantLight — a hard
    directional sun on top of the dome. It runs AFTER `_remove_env_clutter`, so
    the clutter pass cannot see it; this has to come after `add_sky` instead.
    The dome itself is left alone, since that is what lights the scene.
    """
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "DistantLight":
            continue
        if "/Environment/sky/" not in str(prim.GetPath()):
            continue
        if prim.IsActive() and prim.SetActive(False):
            n += 1
            carb.log_info(f"[scene_gen] disabled sun {prim.GetPath()}")
    print(f"[scene_gen] sky sun: {n} DistantLight(s) disabled")
    return n


def add_colliders_skip_empty(prim):
    """`scene_prep.add_colliders`, but skipping meshes that have no points.

    The stock version applies UsdPhysics.CollisionAPI to every gprim it finds.
    Some referenced props carry empty stub Meshes — `Planter_03`, pulled in by
    the Tower pack's `Bench_01`, is the one that shows up here — and PhysX then
    logs "Provided mesh geom with a PhysicsCollisionAPI does not have points,
    collision will not be created" once per empty mesh **per instance**, which
    at city density is thousands of error lines that bury real problems.

    Skipping them changes nothing physically: a mesh with no points has no
    geometry to collide with either way. Returns (applied, skipped).
    """
    applied = skipped = 0
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Gprim):
            continue
        if p.IsA(UsdGeom.Mesh):
            pts = p.GetAttribute("points")
            if not pts or not pts.HasAuthoredValue() or not (pts.Get() or []):
                skipped += 1
                continue
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(p)
            applied += 1
    print(f"[scene_gen] colliders: {applied} applied, {skipped} empty meshes "
          f"skipped")
    return applied, skipped


def wait_for_stage(stage, timeout_s: float = 10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren()
                           if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False


class CityV2PreviewApp:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()

        pg = PegasusInterface()
        pg._world = World(**pg._world_settings)
        pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        _remove_env_clutter(stage)

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders_skip_empty(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        config = load_scene_config(SCENE_CONFIG)

        _, ssf = get_stage_meters_per_unit(stage)
        placements = generate_scene_on_stage(
            stage, config, parent_path="/World/stage/generated",
            scene_scale_factor=ssf)

        # The census is a fact about the PLACEMENT list, so it is taken as
        # soon as that exists — before the settle, the sky or the targets,
        # none of which can change which assets were drawn, and any of which
        # could fail and cost the run its census.
        if SCENE_CENSUS:
            try:
                from archetypes import census as _census
                _doc = _census.record(config, placements,
                                      os.path.splitext(os.path.basename(
                                          SCENE_CONFIG))[0])
                _census.write(SCENE_CENSUS, _doc)
                print(_census.summarise(_doc))
                print(f"[census] -> {SCENE_CENSUS}")
            except Exception as exc:                             # noqa: BLE001
                print(f"[census] FAILED: {exc}")

        generated_prim = stage.GetPrimAtPath("/World/stage/generated")
        if generated_prim.IsValid():
            add_colliders_skip_empty(generated_prim)
        for _ in range(10):
            omni.kit.app.get_app().update()

        settle_rigid_props(
            stage,
            settle_selection(placements),
            ground_path="/World/stage/generated/ground",
            # For the per-category outcome breakdown — the fracture's loose
            # fragments and the generator's toppled props settle in the same
            # pass and fail in completely different ways.
            placements=placements,
        )

        add_sky(stage, resolve_sky(config))
        _disable_sky_sun(stage)

        # ----- Stage C: the people, and anything the USD could not carry -----
        # See scene_gen/targets.py: victims are placed at load time, not baked,
        # so one city can be searched repeatedly with a fresh population.
        # The settle above ended with `reset_simulation()`, which leaves
        # PhysX with no scene to query — Stage C's raycasts would all miss.
        ensure_scene_queries(stage)
        disaster = kinds.get(config)
        victims = disaster.place_targets(stage, config, placements=placements,
                                         parent_path="/World/stage/targets",
                                         scene_scale_factor=ssf)
        disaster.attach_runtime(stage)
        _write_people_gt(config)

        print("\n" + "=" * 70)
        print("CITY V2 PREVIEW READY")
        print(f"  config: {SCENE_CONFIG}")
        print("  reload: see the snippet in this script's docstring")
        print("=" * 70 + "\n")

        # ----- review captures, only when asked (SNAP_DIR=...) --------------
        # After the banner so nothing that parses the launcher's prints
        # (scene_gen/tools/load_bench.py) sees a different run. Point SNAP_DIR
        # under /isaac-sim/.nvidia-omniverse/logs/ so the host can read it.
        if SNAP_DIR:
            try:
                review_snapshots(stage, config, placements, victims or [],
                                 ssf, SNAP_DIR)
            except Exception as exc:                             # noqa: BLE001
                print(f"[snapshots] review captures FAILED: {exc}")
            if REVIEW_SURVEY > 0:
                try:
                    survey_snapshots(stage, config, placements, ssf, SNAP_DIR,
                                     azimuths=REVIEW_SURVEY)
                except Exception as exc:                         # noqa: BLE001
                    print(f"[snapshots] survey FAILED: {exc}")

        self.timeline.play()

    def run(self):
        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    CityV2PreviewApp().run()


if __name__ == "__main__":
    main()
