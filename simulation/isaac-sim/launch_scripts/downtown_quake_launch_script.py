#!/usr/bin/env python
"""
Earthquake-damaged downtown: the detailed city (`generate_scene.py`) built
from the PRISTINE kit archetypes, then every building re-pointed at its
damaged archetype by the disaster field (`disaster/quake.assemble`).

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_quake \
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
    ARCH_DIR       the quake archetype bake (default scene_gen/assets/archetypes_quake)
    QUAKE_SEED     the grade / tilt draws (default 11); the layout seed is the preset's
    QUAKE_TILT     override `disaster.debris.tilt_chance` (0 disables leaning)
    QUAKE_GROUND   0 skips the ground pass (dust halo, fissures, boils, pounding)
    SNAP_DIR       viewport captures under /isaac-sim/.nvidia-omniverse/logs/<name>
    REGION_M / DISASTER_TYPE / SEVERITY   spec overrides, as on the drone launcher
    EPICENTER      "x,y" metres — where the shaking is worst (default: the preset's)
    SOFT_SOIL      "off", or "x,y[,rx,ry[,rate]]" — the liquefaction patch (default:
                   compiled from severity, opposite the epicentre)
"""

import json
import math
import os
import sys
import time

import carb
from isaacsim import SimulationApp

# BOTH fractional-cutout flags, on the command line: the dust halo is a
# translucent overlay and RTX discards fractional cutout opacity unless
# these are set at start-up (wildfire skill, ground-overlay history). They
# are re-asserted after the stage is built, because loading the Pegasus
# environment resets the render property they map onto.
simulation_app = SimulationApp(launch_config={
    "headless": False,
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
from disaster import quake                                       # noqa: E402

# NOT `import scene_launch_script`: that module builds its own SimulationApp
# at import, and a second Kit app in one process is a segfault inside the
# first second (measured). The three helpers it would have lent are copied.
_ENV_CLUTTER = {"GroundPlane", "Environment"}


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

ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
SCENE_CONFIG = os.environ.get("SCENE_CONFIG") or "downtown_earthquake"
ARCH_DIR = os.environ.get("ARCH_DIR") or os.path.join(
    _SCENE_GEN_DIR, "assets", "archetypes_quake")
QUAKE_SEED = int(os.environ.get("QUAKE_SEED", "11"))
QUAKE_TILT = os.environ.get("QUAKE_TILT", "").strip()
SNAP_DIR = os.environ.get("SNAP_DIR", "").strip()
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

        ov = _spec_overrides()
        if ov:
            print("[quake_city] spec overrides: {0}".format(ov))
        config = load_scene_config(SCENE_CONFIG, spec_overrides=ov or None)
        self.config = config
        _, ssf = get_stage_meters_per_unit(stage)
        self.ssf = ssf
        t0 = time.time()
        placements = generate_scene_on_stage(
            stage, config, parent_path=PARENT, scene_scale_factor=ssf)
        t_layout = time.time() - t0
        print("[quake_city] layout in {0:.0f} s".format(t_layout))
        for _ in range(5):
            omni.kit.app.get_app().update()

        # THE EARTHQUAKE. Every placed archetype is re-pointed by the field.
        kw = {}
        if QUAKE_TILT:
            kw["tilt_chance"] = float(QUAKE_TILT)
        t1 = time.time()
        self.stats = quake.assemble(stage, config, placements, ARCH_DIR,
                                    seed=QUAKE_SEED, ssf=ssf, **kw)
        t_assemble = time.time() - t1
        print("[quake_city] damage assembled in {0:.1f} s".format(t_assemble))
        for _ in range(10):
            omni.kit.app.get_app().update()
        # THE GROUND AND THE GAPS (dust halo, fissures, boils, pounding)
        if os.environ.get("QUAKE_GROUND", "1") not in ("0", "false", "no"):
            t2 = time.time()
            try:
                self.ground = quake.ground_effects(
                    stage, config, self.stats, placements, ARCH_DIR, PARENT, ssf,
                    seed=QUAKE_SEED)
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print("[quake_city] ground effects FAILED: {0}".format(exc))
                self.ground = {}
            print("[quake_city] ground effects in {0:.1f} s".format(time.time() - t2))
            for _ in range(5):
                omni.kit.app.get_app().update()
        for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                     "/rtx/pathtracing/fractionalCutoutOpacity"):
            carb.settings.get_settings().set_bool(_key, True)
        t_ready = time.time() - t_launch

        settle_rigid_props(
            stage,
            [p["prim_path"] for p in placements
             if p.get("settle") and p.get("prim_path")],
            ground_path=PARENT + "/ground")
        add_sky(stage, resolve_sky(config))
        _disable_sky_sun(stage)
        self.placements = placements

        recs = self.stats.get("records", [])
        try:
            out = os.path.join(SNAP_DIR or "/tmp", "quake_buildings.json")
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, "w") as fh:
                json.dump(recs, fh, indent=1)
        except Exception as exc:
            print("[quake_city] could not write building records: {0}".format(exc))

        print("\n" + "=" * 70)
        print("EARTHQUAKE DOWNTOWN READY")
        print("  config: {0}   archetypes: {1}".format(SCENE_CONFIG, ARCH_DIR))
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
        if SNAP_DIR:
            try:
                import importlib.util as _ilu
                _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
                _spec = _ilu.spec_from_file_location("snapshots", _sp)
                _snaps = _ilu.module_from_spec(_spec)
                _spec.loader.exec_module(_snaps)
                os.makedirs(SNAP_DIR, exist_ok=True)
                for _ in range(60):
                    app.update()
                w, h = self.config["layout"]["region_m"]
                span = max(float(w), float(h))
                _snaps.overview(self.stage, (0.0, 0.0), span * 1.05,
                                os.path.join(SNAP_DIR, "plat_top.png"), self.ssf)
                # obliques from the four corners, and the epicentre close up
                fld = (self.config.get("disaster") or {}).get("field") or {}
                ex, ey = fld.get("center", [0.0, 0.0])
                pts = {"epi": (float(ex), float(ey)),
                       "ne": (span * 0.25, span * 0.25), "sw": (-span * 0.25, -span * 0.25),
                       "nw": (-span * 0.25, span * 0.25), "se": (span * 0.25, -span * 0.25)}
                _snaps.views_around(self.stage, pts, SNAP_DIR, self.ssf,
                                    top_h=95.0, obl_dist=80.0, obl_h=40.0)
                # the worst buildings, one oblique each
                # rank: collapses first, then the foundation family, then
                # the grades — `grade` is not always `DGn` any more
                _rank = {"DG5": 9, "OV": 8, "DG4": 7, "TILT": 6, "DG3": 5,
                         "SETTLE": 4, "DG2": 3, "DG1": 2, "DG0": 1}
                worst = sorted(self.stats.get("records", []),
                               key=lambda r: -_rank.get(r["grade"].split("+")[0], 0))[:10]
                _snaps.views_around(
                    self.stage, {"b{0}_{1}_{2}".format(i, r["style"], r["grade"].replace("+", "_")):
                                 (r["x"], r["y"]) for i, r in enumerate(worst)},
                    SNAP_DIR, self.ssf, top_h=70.0, obl_dist=55.0, obl_h=28.0)
                print("[quake_city] snapshots -> {0}".format(SNAP_DIR))
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print("[quake_city] snapshots FAILED: {0}".format(exc))
        while simulation_app.is_running():
            app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    QuakeCityApp().run()


if __name__ == "__main__":
    main()
