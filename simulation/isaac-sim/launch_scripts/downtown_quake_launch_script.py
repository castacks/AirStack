#!/usr/bin/env python
"""
Earthquake-damaged downtown: the detailed city (`generate_scene.py`) built
from the PRISTINE kit archetypes, then every building re-pointed at its
damaged archetype by the disaster field (`disaster/quake.assemble`).

    ARCH_DIR=/isaac-sim/AirStack/omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype \
    SCENE_CONFIG=downtown_earthquake \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/quake_city \
    ISAAC_SIM_SCRIPT_NAME=downtown_quake_launch_script.py airstack up isaac-sim

`scene_launch_script.py` with one pass added after the city is generated.
No drone, no sensors: this is the LOOKING launcher for the earthquake scene,
the way `suburb_assemble_launch_script.py` is for the wildfire plat. Nothing
is fractured or simulated here — every damaged building is a reference to a
bake, so the plat loads in seconds after the ~30-60 s layout.

Env:
    SCENE_CONFIG   preset (default downtown_earthquake)
    ARCH_DIR       the quake archetype bake (default omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype)
    GAC_ARCH_DIR   optional GAC quake-bake directory; default remains the
                  ``gac_quake`` sibling of ARCH_DIR. Useful for an isolated
                  review candidate without replacing the current library.
    QUAKE_SEED     the grade / tilt draws (default 11); the layout seed is the preset's
    QUAKE_TILT     override `disaster.debris.tilt_chance` (0 disables leaning)
    QUAKE_GROUND   0 skips the ground pass (dust halo, fissures, boils, pounding)
    QUAKE_REVIEW_PRIMS  comma-separated building prim paths to append to the
                  close-review captures even when they are outside the top-10
                  severity ranking (for example ``.../house_25_81``)
    QUAKE_CASUALTIES casualty-only population target; unset derives a bounded
                  count from DG3-DG5 buildings.  Ordinary city pedestrians
                  are always deactivated.
    QUAKE_REVIEW_PEOPLE number of earthquake casualties to capture at close
                  range after the building views, or ``all`` for every
                  casualty; each gets top + oblique views (default 0)
    QUAKE_CASUALTY_REVIEW_ONLY 1 runs the deadline review pass instead of the
                  general camera suite: selected Cirrus sky, incompatible red
                  fit-out props hidden, and two close views of every casualty
    QUAKE_EXTENSIVE_REVIEW 1 captures every casualty twice, whole-plate and
                  district views from multiple bearings, and a bounded set of
                  the most severe damaged buildings; writes review_manifest.json
    SNAP_DIR       viewport captures under /isaac-sim/.nvidia-omniverse/logs/<name>
    FREEZE_OUT     dataset cell directory. When set, ground truth is written
                  there and SNAP_DIR defaults to FREEZE_OUT/snaps.
    FREEZE_EXPORT  1 freezes the live stage after review and enforces the
                  portable-scene gate. FREEZE_EXIT=1 always exits afterward.
    REGION_M / DISASTER_TYPE / SEVERITY   spec overrides, as on the drone launcher
    ASSET_SET      asset set override: urban_quake (kit only, default of the preset) or
                   urban_quake_v2 (kit + a few standalone monoliths + ruin towers)
    MAGNITUDE      e.g. 6.0 — sets severity AND the field shape from the research
                   (compile_disaster.magnitude_to_severity); SEVERITY, if also set, wins
    CITIES         TWO OR MORE CITIES IN ONE STAGE: "M9.5,M5.5" (magnitudes) or
                   "0.9,0.3" (severities), laid out west->east with CITY_GAP_M of
                   empty ground between; each is CITY_SIZE_M square (default 200)
                   and gets its own layout seed (CITY_SEEDS="3,5", default seed+i)
                   and its own damage draw. Captures are prefixed c0_, c1_, ...
    EPICENTER      "x,y" metres — where the shaking is worst (default: the preset's)
    SOFT_SOIL      "off", or "x,y[,rx,ry[,rate]]" — the liquefaction patch (default:
                   compiled from severity, opposite the epicentre)
"""

import json
import math
import os
import sys
import time

# ---------------------------------------------------------------------------
# PURE REVIEW-CAMERA HELPERS -- stdlib only (`math`), no carb/omni/pxr. Kept
# above the Kit imports below on purpose: `carb` and `isaacsim` are not
# installed in a bare `python3` (measured -- ModuleNotFoundError for both,
# offline, alongside this repo's `pxr` which IS installed standalone), so a
# scratch offline test cannot `import` this whole module. It CAN lift just
# these functions' source (e.g. via `ast`) and exercise the exact
# placement/clearance math with no Kit and no SimulationApp.
#
# THE BUG THIS FIXES (`epi_obl` / `nw_top`, `urban_quake_v5`'s taller
# skyline, review round on a 500x500 seed-9 scene): the five review points
# below (`epi`/`ne`/`se`/`sw`/`nw`) all shared ONE fixed camera pose --
# `top_h=95 / obl_dist=80 / obl_h=40` -- fine on the earlier kit-only
# skylines, where nothing nearby came close to those numbers. `urban_quake_
# v5` places GAC/downtowncity towers that do: a 302 m tower whose footprint
# the fixed SW-oblique eye (`azimuth_deg=225`, `views_around`'s own default)
# landed inside, 12 m from its centre (`epi_obl`); a 131 m "DG4+tilt" tower
# 31 m from `nw`, whose LEAN puts its mass over that point at the 95 m
# camera height even though its upright footprint alone falls half a metre
# short (`nw_top`).
#
# THE FIX raises `top_h`/`obl_dist`/`obl_h` PER POINT, only when the fixed
# pose is actually occluded, instead of raising the three constants for
# every point -- which would also re-pose `ne`/`se`/`sw` (already fine, and
# the user diffs frames across rounds). `ne`/`se`/`sw`/`epi`'s own top-down
# come back byte-identical to the constants below; only the two points a
# tall building actually blocks get lifted.
#
# NO YAW SURVIVES into `quake.assemble`'s records -- `W`/`D` are the
# archetype's own LOCAL-frame footprint (`disaster/quake.py`'s
# `_bld_masses` docstring: "in ITS OWN yaw frame"), and the per-building
# yaw a real run draws is not carried into the review-camera pass at all.
# So this uses the INSCRIBED-circle radius (0.5 * min(W, D)) for a
# building's footprint reach: the one estimate that can never OVER-claim a
# rotated rectangle's true reach regardless of which way it is actually
# yawed (verified against this exact scene's `quake_buildings.json`: it is
# what keeps `SM_Building_13`, 18.9 m from `epi` at H=139.9 m, from being
# mis-flagged, matching the real `epi_top.png` -- a clean shot, not
# embedded).
#
# A building whose `grade` records a lean (`"tilt"` or `"lean"` --
# `disaster/quake.py`'s own foundation-failure/tilt damage states) gets its
# reach WIDENED at height `z` by `z * tan(TILT_DEG_MAX)`. `TILT_DEG_MAX` is
# `quake.py`'s OWN `tilt_deg` default upper bound
# (`debris.get("tilt_deg", [3.0, 9.0])`), not a new number invented here: a
# structure leaning at up to 9 deg has its mass at height `z` shifted
# horizontally by `z * tan(9 deg)` -- exactly what puts `nw`'s tilted
# tower's mass over a point its own upright footprint (radius 18.25 m)
# misses by under half a metre at `z=95 m` (95 * tan(9 deg) = 15 m of
# reach, twelve times the gap).
TILT_DEG_MAX = 9.0


def _review_points(span, ex, ey, dx=0.0, dy=0.0, pre=""):
    """The five named review points (world metres): the epicentre and the
    four quadrant corners at `span * 0.25` from the city centre `(dx, dy)`.
    Same points the inline dict this replaces always built -- pulled into a
    function only so the clearance fix and an offline test can both call it
    without Kit."""
    return {pre + "epi": (dx + float(ex), dy + float(ey)),
            pre + "ne": (dx + span * 0.25, dy + span * 0.25),
            pre + "sw": (dx - span * 0.25, dy - span * 0.25),
            pre + "nw": (dx - span * 0.25, dy + span * 0.25),
            pre + "se": (dx + span * 0.25, dy - span * 0.25)}


def _footprint_radius(rec, z, margin_m=0.0):
    """Inscribed-circle radius (m) of building record `rec` (a
    `quake.assemble` records entry: `x`/`y`/`W`/`D`/`H`/`grade`) at height
    `z`, plus `margin_m` -- widened for a leaning grade, see the module
    comment above `TILT_DEG_MAX`."""
    r = 0.5 * min(float(rec.get("W", 20.0)), float(rec.get("D", 20.0))) + margin_m
    grade = str(rec.get("grade", ""))
    if "tilt" in grade or "lean" in grade:
        h = float(rec.get("H", 12.0))
        r += min(max(z, 0.0), h) * math.tan(math.radians(TILT_DEG_MAX))
    return r


def _building_blocks(rec, x, y, z, margin_m=0.0):
    """True if a record's tilt-widened footprint contains ``(x, y, z)``.

    New records carry the placement yaw, so use the actual oriented
    rectangle.  Old manifests lack it; retain the conservative inscribed
    circle for those rather than pretending an axis-aligned box is exact.
    """
    try:
        rh = float(rec["H"])
    except (KeyError, TypeError, ValueError):
        return False
    if z >= rh:
        return False
    dx = x - float(rec.get("x", 0.0))
    dy = y - float(rec.get("y", 0.0))
    if "yaw_deg" in rec:
        yaw = math.radians(float(rec.get("yaw_deg", 0.0)))
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        lean = 0.0
        grade = str(rec.get("grade", ""))
        if "tilt" in grade or "lean" in grade:
            lean = min(max(z, 0.0), rh) * math.tan(
                math.radians(TILT_DEG_MAX))
        return (abs(lx) <= 0.5 * float(rec.get("W", 20.0)) + margin_m + lean
                and abs(ly) <= 0.5 * float(rec.get("D", 20.0)) + margin_m + lean)
    return math.hypot(dx, dy) <= _footprint_radius(rec, z, margin_m)


def _camera_eye_blocked(records, x, y, z, margin_m=0.0):
    """True if the review camera's OWN position `(x, y, z)` sits inside any
    building's tilt-widened footprint, below that building's height
    (`_building_blocks`).

    DELIBERATELY AN EYE TEST, NOT A FULL EYE-TO-TARGET RAY MARCH: a target
    point's own neighbourhood is EXPECTED to have buildings near it in a
    downtown oblique review shot -- that is the subject's surroundings, not
    an occlusion bug. Measured against this scene's real renders: sampling
    the WHOLE ray flags nearly every oblique (there is almost always some
    building somewhere between the camera and a ground-level target in a
    dense city core), while `ne_obl`/`se_obl`/`sw_obl` are fine in the real
    screenshots. What is actually wrong in `epi_obl`/`nw_top` is the CAMERA
    ITSELF sitting inside, or immediately against, a tall mass -- exactly
    what this checks, and the only place a foreground-black or
    close-blurry-wall frame (see the launcher docstring) actually comes
    from."""
    for rec in records or []:
        if _building_blocks(rec, x, y, z, margin_m):
            return True
    return False


def _review_camera_pose(x, y, top_h, obl_dist, obl_h, azimuth_deg=225.0,
                        aim_h=1.0):
    """`(top_eye, top_target, obl_eye, obl_target)` for one review point, in
    the same world-metre units `views_around` uses (its `ssf` scaling is
    applied by the caller, same as today). Mirrors `snapshots_rp.
    views_around`'s own eye/target formula for its region=None/avoid=None
    fast path -- the ONLY path this launcher's call site (region/avoid never
    passed) actually takes, so there is nothing else here to mirror."""
    az = math.radians(azimuth_deg)
    ex, ey = x + obl_dist * math.cos(az), y + obl_dist * math.sin(az)
    return (x, y, top_h), (x, y, 0.0), (ex, ey, obl_h), (x, y, aim_h)


def _building_review_pose(rec):
    """A close review pose scaled to one building's footprint and height.

    The old 70/55/28-m pose was simultaneously too distant for a 14-m
    brownstone and *inside* a 131-m tower (the top camera was below its
    roof).  Return ``top_h, obl_dist, obl_h, aim_h`` in world metres.  The
    oblique aims at the damage-bearing middle of the elevation, while the
    top camera clears the roof by enough to contain its footprint.
    """
    try:
        w = max(1.0, float(rec.get("W", 20.0)))
        d = max(1.0, float(rec.get("D", 20.0)))
        h = max(1.0, float(rec.get("H", 12.0)))
    except (TypeError, ValueError):
        w, d, h = 20.0, 20.0, 12.0
    footprint = max(w, d)
    top_h = max(28.0, h + 0.72 * footprint + 6.0)
    aim_h = max(2.0, min(h - 0.5, 0.45 * h))
    obl_dist = max(28.0, 0.90 * h, 0.68 * footprint + 10.0)
    obl_h = aim_h + max(7.0, 0.28 * h)
    return top_h, obl_dist, obl_h, aim_h


def _review_camera_clearance(records, x, y, base_top_h, base_obl_dist,
                             base_obl_h, azimuth_deg=225.0, margin_m=0.0,
                             max_mult=5.0):
    """`(top_h, obl_dist, obl_h)` for ONE review point: the base constants,
    each raised only if the fixed pose's own camera EYE is actually blocked
    (`_camera_eye_blocked`), by stepping the relevant number up until it
    clears or `max_mult` * the base is reached (past that, a taller building
    than this launcher's job to clear).

    `margin_m` DEFAULTS TO 0 -- not a generous safety pad. A city core this
    dense has SOME building within any double-digit metres of nearly every
    point, so a positive `margin_m` here does not just add clearance to the
    two genuinely blocked points, it also nudges `ne`/`se`/`sw`/`nw_obl` (all
    already clear at `margin_m=0`) off their fixed pose for no reason --
    measured while building this: `margin_m=15` moved `ne`'s and `nw`'s
    oblique standoff even though neither shot was ever occluded, which
    breaks the "byte-identical for everything but the two broken shots"
    goal just as much as leaving them occluded would. The discrete step
    size (10% of the base number) already overshoots the exact clearing
    point by a few metres on its own, which is margin enough for the two
    points this actually moves.

    TOP steps `top_h` up alone -- the point IS the eye's (x, y), so nothing
    else to move. OBLIQUE steps `obl_dist` OUT along the same fixed
    `azimuth_deg` (never changes direction or the target) and scales
    `obl_h` with it, so the shot keeps roughly today's look-down angle
    instead of going flat.

    Returns the UNCHANGED base triple, unmodified, whenever the fixed pose
    was already clear -- so a caller can group points by this return value
    and re-issue the exact original `views_around` call, byte-identical,
    for every point that needed no fix.
    """
    top_h = base_top_h
    step_h = max(5.0, base_top_h * 0.1)
    while True:
        if not _camera_eye_blocked(records, x, y, top_h, margin_m):
            break
        if top_h >= base_top_h * max_mult:
            break
        top_h += step_h

    obl_dist, obl_h = base_obl_dist, base_obl_h
    step_d = max(5.0, base_obl_dist * 0.1)
    ratio = base_obl_h / base_obl_dist
    az = math.radians(azimuth_deg)
    while True:
        ex, ey = x + obl_dist * math.cos(az), y + obl_dist * math.sin(az)
        if not _camera_eye_blocked(records, ex, ey, obl_h, margin_m):
            break
        if obl_dist >= base_obl_dist * max_mult:
            break
        obl_dist += step_d
        obl_h = obl_dist * ratio
    return top_h, obl_dist, obl_h


def _review_camera_azimuth(records, target, obl_dist, obl_h, aim_h,
                           preferred=225.0, margin_m=2.0):
    """Nearest clear bearing for a per-building oblique review.

    Eye clearance alone missed a real failure in the seed-9 scene: the
    brownstone camera sat just outside a tall neighbour's inscribed circle
    but inside its rotated rectangular footprint, producing a black wall.
    Records now carry yaw, and this searches the full circle for the bearing
    closest to the lighting-friendly preference whose eye *and sightline*
    clear every other building.  The target itself is excluded because the
    ray is supposed to enter it at the end.
    """
    x, y = float(target["x"]), float(target["y"])
    target_path = str(target.get("prim", ""))
    others = [r for r in records or []
              if str(r.get("prim", "")) != target_path]

    def intrusion(azimuth):
        a = math.radians(azimuth)
        ex = x + obl_dist * math.cos(a)
        ey = y + obl_dist * math.sin(a)
        if _camera_eye_blocked(others, ex, ey, obl_h, margin_m):
            return 1000
        hits = 0
        # Do not sample the last 8%: at attached/terraced footprints the
        # subject's immediate neighbour may touch the target wall and is not
        # a foreground obstruction. Forty samples keep spacing under ~3 m at
        # the largest review distance used by this launcher.
        for k in range(1, 38):
            t = 0.92 * k / 38.0
            qx, qy = ex + (x - ex) * t, ey + (y - ey) * t
            qz = obl_h + (aim_h - obl_h) * t
            if _camera_eye_blocked(others, qx, qy, qz, margin_m=0.2):
                hits += 1
        return hits

    offsets = [0.0]
    for d in range(15, 181, 15):
        offsets.extend((-float(d), float(d)))
    candidates = [preferred + d for d in offsets]
    return min(candidates,
               key=lambda az: (intrusion(az), abs((az - preferred + 180.0)
                                                   % 360.0 - 180.0))) % 360.0


import carb
from isaacsim import SimulationApp

# BOTH fractional-cutout flags, on the command line: the dust halo is a
# translucent overlay and RTX discards fractional cutout opacity unless
# these are set at start-up (wildfire skill, ground-overlay history). They
# are re-asserted after the stage is built, because loading the Pegasus
# environment resets the render property they map onto.
simulation_app = SimulationApp(launch_config={
    "headless": os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower() in ("1", "true", "yes"),
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                   "--/rtx/pathtracing/fractionalCutoutOpacity=true"]})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)
from scene_prep import (scale_stage_prim, add_sky,               # noqa: E402
                        get_stage_meters_per_unit, settle_rigid_props)
from scene_generator import resolve_sky                          # noqa: E402
from generate_scene import generate_scene_on_stage               # noqa: E402
from compile_disaster import load_scene_config                   # noqa: E402
from disaster import quake, quake_export, quake_people           # noqa: E402

# NOT `import scene_launch_script`: that module builds its own SimulationApp
# at import, and a second Kit app in one process is a segfault inside the
# first second (measured). The three helpers it would have lent are copied.
_ENV_CLUTTER = {"GroundPlane", "Environment"}


def vram_mb(tag):
    """Print and return the card's used VRAM (MiB) at a named stage.

    MEASURE AS YOU GO (user, 2026-08-30): the deliverable is a 1 km x 1 km
    scene that has to fit a 5090 (32 GB) / RTX PRO 5000 (48 GB) beside the
    rest of the stack. Readings at env-loaded / assembled / hydra-synced /
    after-captures give the per-building content cost, and the READY banner
    projects it to a square kilometre of the same density.
    """
    import subprocess
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used,memory.total",
                              "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=10).stdout
        rows = [[float(x) for x in row.split(",")[:2]]
                for row in out.strip().splitlines() if row.strip()]
        requested = os.environ.get("ISAAC_SIM_ACTIVE_GPU", "").strip()
        index = int(requested) if requested.isdigit() else 0
        # Some pod runtimes expose only the assigned card to nvidia-smi while
        # Kit retains the host ordinal (for example activeGpu=2).  In that
        # case the sole visible row is necessarily the card Kit is using.
        if index >= len(rows):
            index = 0
        used, total = rows[index]
    except Exception as exc:                        # no nvidia-smi in the image?
        print("[quake_city] VRAM {0}: unavailable ({1})".format(tag, exc), flush=True)
        return None
    print("[quake_city] VRAM {0}: {1:.0f} / {2:.0f} MiB (nvidia-smi row {3})"
          .format(tag, used, total, index), flush=True)
    return used


def _remove_env_clutter(stage):
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
            else:
                UsdGeom.Imageable(child).MakeInvisible()
    print("[quake_city] env clutter: {0} prim(s) deactivated".format(n))


def _disable_sky_sun(stage):
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "DistantLight":
            continue
        if "/Environment/sky/" not in str(prim.GetPath()):
            continue
        if prim.IsActive() and prim.SetActive(False):
            n += 1
    print("[quake_city] sky sun: {0} DistantLight(s) disabled".format(n))


def _wait_for_stage(stage, timeout_s=10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            if [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]:
                return True
        time.sleep(0.1)
    return False

def _env(name, default=""):
    value = os.environ.get(name)
    return default if value is None or not value.strip() else value.strip()


def _flag(name, default="0"):
    return _env(name, default).lower() not in ("", "0", "false", "no")


def _default_freeze_name(out_dir):
    parts = [q for q in os.path.abspath(out_dir).split(os.sep) if q]
    if len(parts) >= 4 and parts[-2].startswith("level_"):
        return "{0}_{1}_lvl{2}_{3}".format(
            parts[-4].lower(), parts[-3].lower(),
            parts[-2].split("_", 1)[1], parts[-1])
    return "scene"


ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
SCENE_CONFIG = os.environ.get("SCENE_CONFIG") or "downtown_earthquake"
ARCH_DIR = os.environ.get("ARCH_DIR") or os.path.join(
    _SCENE_GEN_DIR, "assets", "archetype")
GAC_ARCH_DIR = os.environ.get("GAC_ARCH_DIR", "").strip() or None
QUAKE_SEED = int(os.environ.get("QUAKE_SEED") or "11")
QUAKE_TILT = os.environ.get("QUAKE_TILT", "").strip()
FREEZE_OUT = _env("FREEZE_OUT")
FREEZE_NAME = _env("FREEZE_NAME")
FREEZE_EXPORT = _flag("FREEZE_EXPORT")
FREEZE_COLLECT = _flag("FREEZE_COLLECT")
FREEZE_SNAPS = _flag("FREEZE_SNAPS", "1")
FREEZE_EXIT = _flag("FREEZE_EXIT")
QUAKE_EXTENSIVE_REVIEW = _flag("QUAKE_EXTENSIVE_REVIEW")
PEOPLE_VARIANT = int(_env("PEOPLE_VARIANT", "1"))
if FREEZE_OUT:
    os.makedirs(FREEZE_OUT, exist_ok=True)
SNAP_DIR = _env(
    "SNAP_DIR",
    os.path.join(FREEZE_OUT, "snaps") if FREEZE_OUT and FREEZE_SNAPS else "")
# The exact-casualty review is executed as a standalone helper inside this
# process and intentionally gets its output contract from the environment.
# Publish the resolved default as well as an explicit caller value so that a
# frozen cell never falls back to the helper's old interactive-review folder.
if SNAP_DIR:
    os.environ["SNAP_DIR"] = SNAP_DIR
PARENT = "/World/stage/generated"


def _spec_overrides():
    ov = {}
    r = os.environ.get("REGION_M", "").strip()
    if r:
        parts = [float(v) for v in r.replace("x", ",").split(",") if v.strip()]
        ov["region_m"] = [parts[0], parts[-1]]
    d = os.environ.get("DISASTER_TYPE", "").strip().lower()
    # `.env` ships DISASTER_TYPE=none for the suburb missions and the
    # container inherits it; honouring that here compiles the earthquake
    # preset with no earthquake. This launcher IS the earthquake launcher, so
    # only `earthquake` (or a deliberate `none` via QUAKE_ALLOW_NONE=1) counts.
    if d and (d == "earthquake" or os.environ.get("QUAKE_ALLOW_NONE") == "1"):
        ov["disaster-type"] = d
    elif d:
        print("[quake_city] ignoring DISASTER_TYPE={0} from the environment".format(d))
    s = os.environ.get("SEVERITY", "").strip()
    if s:
        ov["severity"] = float(s)
    m = os.environ.get("MAGNITUDE", "").strip()
    if m:
        ov["magnitude"] = float(m.lstrip("Mm"))
    aset = os.environ.get("ASSET_SET", "").strip()
    if aset:
        ov["asset-set"] = aset           # e.g. urban_quake_v2 (kit + monoliths)
    e = os.environ.get("EPICENTER", "").strip()
    if e:
        parts = [float(v) for v in e.replace("x", ",").split(",") if v.strip()]
        ov["epicenter"] = [parts[0], parts[-1]]
    ss = os.environ.get("SOFT_SOIL", "").strip().lower()
    if ss in ("0", "off", "false", "none"):
        ov["soft-soil"] = False
    elif ss:
        parts = [float(v) for v in ss.replace("x", ",").split(",") if v.strip()]
        ov["soft-soil"] = {"center": [parts[0], parts[1]]}
        if len(parts) >= 4:
            ov["soft-soil"].update({"rx_m": parts[2], "ry_m": parts[3]})
        if len(parts) >= 5:
            ov["soft-soil"]["rate"] = parts[4]
    return ov


def _city_specs(base_ov):
    """The list of cities to build: one (the preset, plus env overrides) or,
    with CITIES set, several `CITY_SIZE_M` plates in a row along x with
    `CITY_GAP_M` of bare ground between them. Each entry is a magnitude
    (`M9.5`, `9.5` when > 1.5) or a severity (`0.3`)."""
    spec = os.environ.get("CITIES", "").strip()
    if not spec:
        return [{"name": "city", "label": SCENE_CONFIG, "ov": dict(base_ov),
                 "offset": (0.0, 0.0), "parent": PARENT}]
    size = float(os.environ.get("CITY_SIZE_M") or "200")
    gap = float(os.environ.get("CITY_GAP_M") or "100")
    seeds = [int(v) for v in os.environ.get("CITY_SEEDS", "").replace(";", ",").split(",") if v.strip()]
    items = [v.strip() for v in spec.replace(";", ",").split(",") if v.strip()]
    n = len(items)
    pitch = size + gap
    out = []
    for i, it in enumerate(items):
        ov = dict(base_ov)
        ov["region_m"] = [size, size]
        ov.pop("magnitude", None)
        ov.pop("severity", None)
        val = float(it.lstrip("Mm"))
        if it[:1] in "Mm" or val > 1.5:
            ov["magnitude"] = val
            label = "M{0:.1f}".format(val)
        else:
            ov["severity"] = val
            label = "sev {0:.2f}".format(val)
        ov["seed"] = seeds[i] if i < len(seeds) else (QUAKE_SEED + 101 * i)
        # the compiled soft-soil / epicentre defaults are per plate already
        # (fractions of region_m); an explicit EPICENTER applies to every city
        x = (i - (n - 1) / 2.0) * pitch
        out.append({"name": "c{0}".format(i), "label": label, "ov": ov,
                    "offset": (x, 0.0), "parent": "{0}_c{1}".format(PARENT, i)})
    return out


def _offset_parent(stage, parent, dx, dy):
    """Translate a city's root so its local (0, 0) lands at (dx, dy)."""
    prim = stage.GetPrimAtPath(parent)
    if not prim or not prim.IsValid():
        prim = UsdGeom.Xform.Define(stage, Sdf.Path(parent)).GetPrim()
    # the generator defines its parent as a typeless holder / Scope, and a
    # translate op on a non-Xformable is silently ignored (two-city run 2:
    # both cities composed at the origin). Same fix as the bake root.
    if not prim.IsA(UsdGeom.Xformable):
        prim.SetTypeName("Xform")
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    tr = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            tr = op
            break
    if tr is None:
        tr = xf.AddTranslateOp()
        xf.SetXformOpOrder([tr] + list(ops))
    cur = tr.Get() or Gf.Vec3d(0, 0, 0)
    tr.Set(Gf.Vec3d(cur[0] + dx, cur[1] + dy, cur[2]))


def _void_ground(stage, cities, ssf):
    """Bare ground under the whole row so the gap between cities is land,
    not the void: one flat dark-earth quad just under every plate."""
    from pxr import Vt
    xs = [c["offset"][0] for c in cities]
    sizes = [float(c["ov"]["region_m"][0]) for c in cities]
    x0 = min(xs) - max(sizes) * 0.75
    x1 = max(xs) + max(sizes) * 0.75
    y = max(sizes) * 0.75
    # the Pegasus default environment's own ground plane would otherwise show
    # through at z = 0 in the gap; the plates carry their own ground
    n_off = 0
    for prim in stage.Traverse():
        nm = prim.GetName().lower()
        if "groundplane" in nm and not str(prim.GetPath()).startswith(PARENT):
            if prim.IsActive() and prim.SetActive(False):
                n_off += 1
    path = PARENT + "_void/ground"
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    z = -0.02 * ssf
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(x0 * ssf, -y * ssf, z), Gf.Vec3f(x1 * ssf, -y * ssf, z),
                                         Gf.Vec3f(x1 * ssf, y * ssf, z), Gf.Vec3f(x0 * ssf, y * ssf, z)]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.30, 0.27, 0.22)]))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    # RTX ignores displayColor on an unbound mesh (it rendered pale pink):
    # bind the quake palette's triplanar soil so the gap reads as bare earth
    try:
        from disaster import quake_flow as _qf
        _qf._bind(stage, path, _qf.materials(stage, PARENT + "_void")["soil"])
    except Exception as exc:
        print("[quake_city] void ground material: {0}".format(exc))
    print("[quake_city] void ground {0:.0f} x {1:.0f} m under {2} cities; {3} default "
          "ground plane(s) off".format(x1 - x0, 2 * y, len(cities), n_off))


class QuakeCityApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()
        t_launch = time.time()
        pg = PegasusInterface()
        pg._world = World(**pg._world_settings)
        pg.load_environment(ENV_URL)
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not _wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")
        self.stage = stage
        _remove_env_clutter(stage)
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", 1.0)
            for _ in range(10):
                omni.kit.app.get_app().update()

        self._vram = {"env": vram_mb("env loaded")}
        ov = _spec_overrides()
        if ov:
            print("[quake_city] spec overrides: {0}".format(ov))
        _, ssf = get_stage_meters_per_unit(stage)
        self.ssf = ssf
        cities = _city_specs(ov)
        self.cities = []
        t_layout = t_assemble = 0.0
        placements_all = []
        tally = {}
        records = []
        n_bld = n_tilt = n_miss = 0
        for ci, city in enumerate(cities):
            parent = city["parent"]
            config = load_scene_config(SCENE_CONFIG, spec_overrides=city["ov"] or None)
            _generic_assets = quake_people.disable_generic_population(config)
            print("[quake_city] generic city population disabled before "
                  "authoring ({0} human asset(s)); casualties are authored "
                  "against final damage geometry".format(_generic_assets),
                  flush=True)
            t0 = time.time()
            placements = generate_scene_on_stage(
                stage, config, parent_path=parent, scene_scale_factor=ssf)
            t_layout += time.time() - t0
            print("[quake_city] {0}: layout in {1:.0f} s".format(city["name"], time.time() - t0))
            for _ in range(5):
                omni.kit.app.get_app().update()

            # THE EARTHQUAKE. Every placed archetype is re-pointed by the field.
            kw = {}
            if QUAKE_TILT:
                kw["tilt_chance"] = float(QUAKE_TILT)
            t1 = time.time()
            # `parent=parent` (round 7): lets every mild lean's corner
            # fissures cut a real opening through THIS city's own ground
            # meshes (`apply_ground_planes` authored them under
            # `parent + "/ground"` a few lines up, in `generate_scene_on_
            # stage`) instead of sitting on top of them as a mound — see
            # `quake.assemble`'s own docstring on `parent`.
            stats = quake.assemble(stage, config, placements, ARCH_DIR,
                                   seed=QUAKE_SEED + ci, ssf=ssf, parent=parent,
                                   gac_dir=GAC_ARCH_DIR,
                                   **kw)
            t_assemble += time.time() - t1
            print("[quake_city] {0}: damage assembled in {1:.1f} s".format(
                city["name"], time.time() - t1))
            for _ in range(10):
                omni.kit.app.get_app().update()
            # THE GROUND AND THE GAPS (dust halo, fissures, boils, pounding)
            ground = {}
            if os.environ.get("QUAKE_GROUND", "1") not in ("0", "false", "no"):
                t2 = time.time()
                try:
                    ground = quake.ground_effects(
                        stage, config, stats, placements, ARCH_DIR, parent, ssf,
                        seed=QUAKE_SEED + ci)
                except Exception as exc:
                    import traceback
                    traceback.print_exc()
                    print("[quake_city] ground effects FAILED: {0}".format(exc))
                print("[quake_city] {0}: ground effects in {1:.1f} s".format(
                    city["name"], time.time() - t2))
                for _ in range(5):
                    omni.kit.app.get_app().update()
            # MOVE THE WHOLE CITY to its slot: everything the generator and
            # the quake passes made sits under `parent`, in city-local metres.
            dx, dy = city["offset"]
            if dx or dy:
                _offset_parent(stage, parent, dx * ssf, dy * ssf)
            for r in stats.get("records", []):
                r["x"] += dx
                r["y"] += dy
                r["city"] = city["name"]
            for p in placements:
                p["_city_dx"], p["_city_dy"] = dx, dy
            for k, v in stats["tally"].items():
                tally[k] = tally.get(k, 0) + v
            records += stats.get("records", [])
            n_bld += stats["buildings"]
            n_tilt += stats["tilted"]
            n_miss += stats["missing"]
            placements_all += placements
            self.cities.append({"name": city["name"], "config": config, "stats": stats,
                                "ground": ground, "offset": (dx, dy), "parent": parent,
                                "label": city["label"]})
            print("[quake_city] {0} ({1}) at ({2:+.0f}, {3:+.0f}): {4} buildings: {5}".format(
                city["name"], city["label"], dx, dy, stats["buildings"],
                ", ".join("{0}={1}".format(k, v) for k, v in sorted(stats["tally"].items()))))
        self._vram["assembled"] = vram_mb("assembled (USD authored, {0} buildings)".format(n_bld))
        self._area_m2 = sum(float(c["config"]["layout"]["region_m"][0])
                            * float(c["config"]["layout"]["region_m"][-1]) for c in self.cities)
        if len(cities) > 1:
            _void_ground(stage, cities, ssf)
        config = self.cities[0]["config"]
        self.config = config
        placements = placements_all
        self.stats = {"buildings": n_bld, "tally": tally, "tilted": n_tilt,
                      "missing": n_miss, "records": records}
        self.ground = self.cities[0]["ground"]
        for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                     "/rtx/pathtracing/fractionalCutoutOpacity"):
            carb.settings.get_settings().set_bool(_key, True)
        for city in self.cities:
            settle_rigid_props(
                stage,
                [p["prim_path"] for p in placements
                 if p.get("settle") and p.get("prim_path")
                 and p["prim_path"].startswith(city["parent"] + "/")],
                # The whole-region asphalt mesh is sufficient support for
                # these few toppled street props.  Passing the ground Scope
                # recursively put CollisionAPI on every lane dash, bike
                # marking, pavement strip, and material prim (thousands of
                # edits) and could crash RTX scene-db while syncing them.
                # Sidewalks are only centimetres above this plane, so using
                # the base preserves the visible resting pose while making
                # the settle both bounded and reliable.
                ground_path=city["parent"] + "/ground/asphalt_base")
        add_sky(stage, resolve_sky(config))
        _disable_sky_sun(stage)
        self.placements = placements
        _raw_people_total = os.environ.get("QUAKE_CASUALTIES", "").strip()
        _people_total = int(_raw_people_total) if _raw_people_total else None
        _city_bounds = {}
        for _city in self.cities:
            _region = _city["config"]["layout"]["region_m"]
            _rw, _rh = float(_region[0]), float(_region[-1])
            _dx, _dy = _city["offset"]
            _city_bounds[_city["name"]] = (
                _dx - 0.5 * _rw, _dy - 0.5 * _rh,
                _dx + 0.5 * _rw, _dy + 0.5 * _rh)
        _tp = time.time()
        self.people_records, self.people_report = \
            quake_people.replace_population(
                stage, placements, self.stats.get("records", []),
                parent="/World/stage", ssf=ssf, seed=QUAKE_SEED,
                total=_people_total, bounds_by_city=_city_bounds)
        print("[quake_city] casualty population in {0:.1f} s".format(
            time.time() - _tp), flush=True)
        t_ready = time.time() - t_launch

        recs = self.stats.get("records", [])
        try:
            out = os.path.join(SNAP_DIR or "/tmp", "quake_buildings.json")
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, "w") as fh:
                json.dump(recs, fh, indent=1)
            people_out = os.path.join(SNAP_DIR or "/tmp", "quake_people.json")
            people_doc = quake_export.people_document(
                self.people_records, self.people_report)
            self.people_doc = people_doc
            with open(people_out, "w") as fh:
                json.dump(people_doc, fh, indent=1)
            if FREEZE_OUT:
                _regions = [c["config"]["layout"]["region_m"]
                            for c in self.cities]
                _region = [sum(float(q[0]) for q in _regions),
                           max(float(q[-1]) for q in _regions)]
                _mag = _env("MAGNITUDE")
                quake_export.write_sidecars(
                    stage, FREEZE_OUT, self.stats, placements, people_doc,
                    ssf, SCENE_CONFIG, QUAKE_SEED, ARCH_DIR, GAC_ARCH_DIR,
                    _region, magnitude=(float(_mag.lstrip("Mm"))
                                        if _mag else None),
                    people_variant=PEOPLE_VARIANT)
            print("[quake_city] people: {0} casualty-only ({1} interior, "
                  "{2} rubble), 0 standing/walking; ground truth -> {3}"
                  .format(len(self.people_records),
                          self.people_report["interior_casualties"],
                          self.people_report["rubble_casualties"],
                          (os.path.join(FREEZE_OUT, "GT_people.json")
                           if FREEZE_OUT else people_out)))
        except Exception as exc:
            print("[quake_city] could not write review records: {0}".format(exc))
            if FREEZE_OUT:
                raise

        print("\n" + "=" * 70)
        print("EARTHQUAKE DOWNTOWN READY")
        print("  config: {0}   archetypes: {1}".format(SCENE_CONFIG, ARCH_DIR))
        for city in self.cities:
            print("  {0} ({1}) at ({2:+.0f}, {3:+.0f}): {4}".format(
                city["name"], city["label"], city["offset"][0], city["offset"][1],
                ", ".join("{0}={1}".format(k, v) for k, v in sorted(city["stats"]["tally"].items()))))
        print("  buildings {0}: {1}".format(
            self.stats["buildings"],
            ", ".join("{0}={1}".format(k, v) for k, v in sorted(self.stats["tally"].items()))))
        print("  tilted {0}, stepped-down {1}".format(self.stats["tilted"], self.stats["missing"]))
        print("  TIMING  layout {0:.0f} s   assemble {1:.1f} s   env->ready {2:.0f} s "
              "(no physics; every damaged building is a reference)".format(
                  t_layout, t_assemble, t_ready))
        print("=" * 70 + "\n")
        self.timeline.play()

    def run(self):
        app = omni.kit.app.get_app()
        # _o_ WHAT "READY" LEAVES OUT. The TIMING banner above stops when the
        # USD is authored; the scene is not usable until Hydra has synced it
        # and the RTX BLASes are built, which is the first `app.update()`
        # after that. So: prim count, the first frame, and the steady-state
        # frame time — the three numbers the "scenes must not take hours to
        # load" constraint is actually about. Costs ~2 s and prints one line.
        try:
            n_prims = sum(1 for _ in self.stage.Traverse())
            _t = time.time()
            app.update()
            first_s = time.time() - _t
            for _ in range(10):
                app.update()
            _n = 60
            _t = time.time()
            for _ in range(_n):
                app.update()
            _dt = time.time() - _t
            try:
                import omni.kit.viewport.utility as _vp
                _res = "{0}x{1}".format(*_vp.get_active_viewport().resolution)
            except Exception:
                _res = "?"
            print("[quake_city] HYDRA  {0} stage prims, first frame {1:.1f} s, "
                  "{2:.1f} fps / {3:.0f} ms per frame at {4}".format(
                      n_prims, first_s, _n / _dt if _dt else 0.0,
                      1000.0 * _dt / _n, _res), flush=True)
        except Exception as exc:
            print("[quake_city] hydra timing failed: {0}".format(exc))
        self._vram["hydra"] = vram_mb("hydra synced (BLAS built)")
        _casualty_review_only = os.environ.get(
            "QUAKE_CASUALTY_REVIEW_ONLY", "0").strip().lower() in (
                "1", "true", "yes")
        _review_expected = 0
        _review_success = 0
        _review_errors = []
        _casualty_report = None
        if _casualty_review_only or QUAKE_EXTENSIVE_REVIEW:
            _review_script = os.path.join(
                _SCENE_GEN_DIR, "tools", "live_quake_casualty_review.py")
            print("[quake_city] running exact casualty review pass: {0}".format(
                _review_script), flush=True)
            try:
                with open(_review_script, "rb") as _fh:
                    _review_code = compile(
                        _fh.read(), _review_script, "exec")
                exec(_review_code, {"__name__": "__main__",
                                    "__file__": _review_script})
                with open(os.path.join(
                        SNAP_DIR, "casualty_review_done.json")) as _fh:
                    _casualty_report = json.load(_fh)
                _review_expected += int(
                    _casualty_report.get("expected_views", 0))
                _review_success += int(
                    _casualty_report.get("successful_views", 0))
                _review_errors.extend(
                    _casualty_report.get("failed_views", ()))
            except Exception as exc:
                import traceback
                traceback.print_exc()
                _review_errors.append("casualty review: {0}".format(exc))
                print("[quake_city] casualty review FAILED: {0}".format(
                    exc), flush=True)
        if not _casualty_review_only and SNAP_DIR:
            try:
                import importlib.util as _ilu
                # On an OSMO livestream pod the viewport capture SEGFAULTS
                # (no X server behind the viewport — build-scenes-on-osmo
                # skill); snapshots_rp has the same API over a Replicator
                # render product. Prefer it, fall back to the viewport path.
                _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots_rp.py")
                if not os.path.exists(_sp) or os.environ.get("SNAP_VIEWPORT") == "1":
                    _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
                _spec = _ilu.spec_from_file_location("snapshots", _sp)
                _snaps = _ilu.module_from_spec(_spec)
                _spec.loader.exec_module(_snaps)
                print("[quake_city] snapshots via " + os.path.basename(_sp), flush=True)
                os.makedirs(SNAP_DIR, exist_ok=True)
                for _ in range(60):
                    app.update()
                xs = [c["offset"][0] for c in self.cities]
                ws = [max(map(float, c["config"]["layout"]["region_m"])) for c in self.cities]
                span_all = (max(xs) - min(xs)) + max(ws)
                _whole_centre = ((max(xs) + min(xs)) / 2.0, 0.0)
                _review_expected += 1
                _review_success += bool(_snaps.overview(
                    self.stage, _whole_centre, span_all * 1.05,
                    os.path.join(SNAP_DIR, "plat_top.png"), self.ssf))
                # One nadir plus four true whole-scene obliques.  These are
                # direct render-product captures so the four views share one
                # subject/target instead of pretending four district points
                # are equivalent to four bearings of the complete plate.
                if QUAKE_EXTENSIVE_REVIEW and os.path.basename(_sp) == "snapshots_rp.py":
                    _cx, _cy = _whole_centre
                    _dist = 0.70 * span_all
                    _height = max(180.0, 0.48 * span_all)
                    for _az in (45.0, 135.0, 225.0, 315.0):
                        _a = math.radians(_az)
                        _snaps.place_camera(
                            self.stage,
                            ((_cx + _dist * math.cos(_a)) * self.ssf,
                             (_cy + _dist * math.sin(_a)) * self.ssf,
                             _height * self.ssf),
                            (_cx * self.ssf, _cy * self.ssf,
                             min(45.0, 0.06 * span_all) * self.ssf))
                        _review_expected += 1
                        _review_success += bool(_snaps.snapshot(
                            self.stage, os.path.join(
                                SNAP_DIR, "plat_obl_{0:03d}.png".format(
                                    int(_az)))))
                for ci, city in enumerate(self.cities):
                    pre = "" if len(self.cities) == 1 else "c{0}_".format(ci)
                    dx, dy = city["offset"]
                    span = ws[ci]
                    if pre:
                        _review_expected += 1
                        _review_success += bool(_snaps.overview(
                            self.stage, (dx, dy), span * 1.05,
                            os.path.join(SNAP_DIR, pre + "plat_top.png"),
                            self.ssf))
                    # obliques from the four corners, and the epicentre close up
                    fld = (city["config"].get("disaster") or {}).get("field") or {}
                    ex, ey = fld.get("center", [0.0, 0.0])
                    pts = _review_points(span, ex, ey, dx, dy, pre)
                    # PER-POINT CLEARANCE (urban_quake_v5's taller skyline --
                    # see the module comment above `TILT_DEG_MAX`): the base
                    # pose is unchanged for every point where it is already
                    # clear, so this groups points by the (possibly raised)
                    # triple and re-issues `views_around` once per group --
                    # a point needing no fix gets the EXACT SAME call as
                    # before (byte-identical camera pose).
                    _base_pose = (95.0, 80.0, 40.0)
                    _groups = {}
                    for _name, (_px, _py) in pts.items():
                        _clear = _review_camera_clearance(
                            self.stats.get("records", []), _px, _py, *_base_pose)
                        _groups.setdefault(_clear, {})[_name] = (_px, _py)
                    for (_top_h, _obl_dist, _obl_h), _group_pts in _groups.items():
                        if (_top_h, _obl_dist, _obl_h) != _base_pose:
                            print("[quake_city] review camera clearance: {0} -> "
                                  "top_h={1:.0f} obl_dist={2:.0f} obl_h={3:.0f} "
                                  "(tall building nearby)".format(
                                      sorted(_group_pts), _top_h, _obl_dist, _obl_h))
                        _review_expected += 2 * len(_group_pts)
                        _review_success += int(_snaps.views_around(
                            self.stage, _group_pts, SNAP_DIR, self.ssf,
                            top_h=_top_h, obl_dist=_obl_dist,
                            obl_h=_obl_h))
                        if QUAKE_EXTENSIVE_REVIEW:
                            _opposite = {
                                name + "_opposite": xy
                                for name, xy in _group_pts.items()}
                            _review_expected += 2 * len(_opposite)
                            _review_success += int(_snaps.views_around(
                                self.stage, _opposite, SNAP_DIR, self.ssf,
                                top_h=_top_h, obl_dist=_obl_dist,
                                obl_h=_obl_h, azimuth_deg=45.0))
                # the worst buildings, one oblique each
                # rank: collapses first, then the foundation family, then
                # the grades — `grade` is not always `DGn` any more
                _rank = {"DG5": 9, "OV": 8, "DG4": 7, "TILT": 6, "DG3": 5,
                         "SETTLE": 4, "DG2": 3, "DG1": 2, "DG0": 1}
                _ordered = sorted(
                    self.stats.get("records", []),
                    key=lambda r: (-_rank.get(
                        quake_export.base_grade(r.get("grade")), 0),
                        float(r.get("x", 0.0)), float(r.get("y", 0.0))))
                if QUAKE_EXTENSIVE_REVIEW:
                    _major = [r for r in _ordered
                              if quake_export.base_grade(r.get("grade"))
                              in ("DG4", "DG5", "OV", "TILT")]
                    _max_major = max(1, int(_env(
                        "QUAKE_REVIEW_MAJOR_MAX", "48")))
                    worst = _major[:_max_major]
                    print("[quake_city] extensive review: {0}/{1} major "
                          "building(s) selected".format(
                              len(worst), len(_major)), flush=True)
                else:
                    worst = _ordered[:10]
                _requested = {q.strip() for q in os.environ.get(
                    "QUAKE_REVIEW_PRIMS", "").split(",") if q.strip()}
                if _requested:
                    _found = {str(r.get("prim")) for r in worst}
                    for _r in self.stats.get("records", []):
                        if str(_r.get("prim")) in _requested \
                                and str(_r.get("prim")) not in _found:
                            worst.append(_r)
                            _found.add(str(_r.get("prim")))
                    _missing = sorted(_requested - _found)
                    if _missing:
                        print("[quake_city] requested review prim(s) absent: "
                              + ", ".join(_missing))
                for i, r in enumerate(worst):
                    _name = "b{0}_{1}_{2}".format(
                        i, r["style"], r["grade"].replace("+", "_"))
                    _top_h, _obl_dist, _obl_h, _aim_h = \
                        _building_review_pose(r)
                    _top_h, _obl_dist, _obl_h = _review_camera_clearance(
                        self.stats.get("records", []), r["x"], r["y"],
                        _top_h, _obl_dist, _obl_h, margin_m=2.0)
                    _azimuth = _review_camera_azimuth(
                        self.stats.get("records", []), r, _obl_dist,
                        _obl_h, _aim_h)
                    print("[quake_city] building review {0}: top={1:.0f}m, "
                          "oblique={2:.0f}m/{3:.0f}m, aim={4:.0f}m, "
                          "azimuth={5:.0f}deg".format(
                              _name, _top_h, _obl_dist, _obl_h, _aim_h,
                              _azimuth))
                    _review_expected += 2
                    _review_success += int(_snaps.views_around(
                        self.stage, {_name: (r["x"], r["y"])}, SNAP_DIR,
                        self.ssf, top_h=_top_h, obl_dist=_obl_dist,
                        obl_h=_obl_h, aim_h=_aim_h,
                        azimuth_deg=_azimuth))
                # Optional close review of REAL earthquake-field casualties.
                # Sampling evenly through their deterministic placement order
                # shows several parts of the plate without adding a second
                # population or moving anyone just for the camera.
                _people_review_raw = os.environ.get(
                    "QUAKE_REVIEW_PEOPLE", "0").strip().lower()
                _casualties = [r for r in self.people_records
                               if r.get("state") in quake_people.CASUALTY_STATES
                               and r.get("active")]
                _want_people = (len(_casualties)
                                if _people_review_raw == "all"
                                else max(0, int(_people_review_raw or 0)))
                if _want_people and _casualties and not QUAKE_EXTENSIVE_REVIEW:
                    _n = min(_want_people, len(_casualties))
                    if _n == 1:
                        _picked = [_casualties[len(_casualties) // 2]]
                    else:
                        _picked = [_casualties[int(round(
                            j * (len(_casualties) - 1) / float(_n - 1)))]
                                   for j in range(_n)]
                    for _r in _picked:
                        _name = "person_" + _r["id"]
                        print("[quake_city] person review {0}: ({1:.1f}, {2:.1f}), "
                              "{3}, near {4} {5}".format(
                                  _r["id"], _r["x"], _r["y"],
                                  _r.get("state", "casualty"),
                                  _r.get("nearest_style", "?"),
                                  _r.get("nearest_grade", "?")))
                        _z = float(_r.get("z", 0.0))
                        _review_expected += 2
                        _review_success += int(_snaps.views_around(
                            self.stage, {_name: (_r["x"], _r["y"])},
                            SNAP_DIR, self.ssf,
                            top_h=max(14.0, _z + 11.0),
                            obl_dist=9.0, obl_h=_z + 4.2,
                            aim_h=_z + 0.45))
                print("[quake_city] snapshots -> {0}".format(SNAP_DIR))
            except Exception as exc:
                import traceback
                traceback.print_exc()
                _review_errors.append("general review: {0}".format(exc))
                print("[quake_city] snapshots FAILED: {0}".format(exc))
        self._vram["end"] = vram_mb("after captures")
        try:
            env, hyd = self._vram.get("env"), self._vram.get("hydra")
            n = max(1, int(self.stats.get("buildings", 0)))
            if env is not None and hyd is not None:
                content = max(0.0, hyd - env)
                per = content / n
                scale = 1.0e6 / max(1.0, getattr(self, "_area_m2", 1.0e6))
                n_km2 = n * scale
                proj = env + per * n_km2
                print("[quake_city] VRAM BUDGET: baseline {0:.0f} MiB | content {1:.0f} MiB for {2} "
                      "building(s) on {3:.2f} km2 = {4:.0f} MiB/building".format(
                          env, content, n, 1.0 / scale, per), flush=True)
                for card, tot in (("5090", 32768.0), ("RTX PRO 5000", 49152.0)):
                    print("[quake_city]   projection: 1 km2 at this density ({0:.0f} buildings) -> "
                          "{1:.0f} MiB on a {2} ({3:.0f}% of {4:.0f} MiB)".format(
                              n_km2, proj, card, 100.0 * proj / tot, tot), flush=True)
        except Exception as _exc:
            print("[quake_city] VRAM budget summary failed: {0}".format(_exc))
        if SNAP_DIR:
            import glob as _glob
            _pngs = sorted(os.path.relpath(p, SNAP_DIR) for p in
                           _glob.glob(os.path.join(SNAP_DIR, "**", "*.png"),
                                      recursive=True))
            _review_ok = bool(
                _review_expected > 0
                and _review_success == _review_expected
                and not _review_errors)
            _review_manifest = {
                "schema": "airstack.earthquake-review/1",
                "extensive": bool(QUAKE_EXTENSIVE_REVIEW),
                "expected_views": int(_review_expected),
                "successful_views": int(_review_success),
                "failed": list(_review_errors),
                "png_count": len(_pngs),
                "pngs": _pngs,
                "casualties": {
                    "count": len(self.people_records),
                    "expected_views": (0 if _casualty_report is None else
                                       int(_casualty_report.get(
                                           "expected_views", 0))),
                    "successful_views": (0 if _casualty_report is None else
                                         int(_casualty_report.get(
                                             "successful_views", 0))),
                },
                "coverage": {
                    "whole_scene": "nadir plus four oblique bearings",
                    "districts": "epicentre and four quadrants, two bearings",
                    "major_buildings": len(worst) if "worst" in locals() else 0,
                    "people": "two geometry-verified exterior/drone views each",
                },
                "ok": _review_ok,
            }
            with open(os.path.join(SNAP_DIR, "review_manifest.json"), "w") as _fh:
                json.dump(_review_manifest, _fh, indent=1)
            if FREEZE_OUT:
                with open(os.path.join(FREEZE_OUT,
                                       "review_manifest.json"), "w") as _fh:
                    json.dump(_review_manifest, _fh, indent=1)
            print("[quake_city] REVIEW GATE {0}: {1}/{2} views, {3} PNG(s)"
                  .format("OK" if _review_ok else "FAILED",
                          _review_success, _review_expected, len(_pngs)),
                  flush=True)
            if QUAKE_EXTENSIVE_REVIEW and not _review_ok:
                raise RuntimeError("earthquake extensive review gate failed")

        # Export last: all labels and review evidence remain available if the
        # portable-scene gate identifies a bad dependency.  A failed gate is
        # fatal; the batch runner must never upload a merely written USD.
        if FREEZE_EXPORT:
            if not FREEZE_OUT:
                raise RuntimeError("FREEZE_EXPORT=1 requires FREEZE_OUT")
            from disaster import freeze as _freeze
            _name = FREEZE_NAME or _default_freeze_name(FREEZE_OUT)
            try:
                _finfo = _freeze.export_scene(
                    FREEZE_OUT, _name, collect=FREEZE_COLLECT)
                _freeze.report(_finfo)
                with open(os.path.join(
                        FREEZE_OUT, "freeze_report.json"), "w") as _fh:
                    json.dump(_finfo, _fh, indent=1)
            except _freeze.PortabilityError as _exc:
                with open(os.path.join(
                        FREEZE_OUT, "freeze_report.json"), "w") as _fh:
                    json.dump(_exc.info, _fh, indent=1)
                raise
            print("[quake_city] FREEZE DONE: {0}".format(
                os.path.join(FREEZE_OUT, _name + ".usd")), flush=True)
        # headless: exit once the captures are on disk (KEEP_OPEN=1 to stay)
        if (not FREEZE_EXIT
             and (os.environ.get("KEEP_OPEN", "").strip() == "1"
             or os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower()
             not in ("1", "true", "yes"))):
            while simulation_app.is_running():
                app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    QuakeCityApp().run()


if __name__ == "__main__":
    main()
