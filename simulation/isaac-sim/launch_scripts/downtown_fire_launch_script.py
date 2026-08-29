#!/usr/bin/env python
"""
Urban fire downtown: the detailed city (`scene_gen/generate_scene.py`) at
500 x 500 m, then a structure fire spread through it
(`disaster/urban_fire_city.assemble`).

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/fire_city \
    ISAAC_SIM_SCRIPT_NAME=downtown_fire_launch_script.py airstack up isaac-sim

The fire twin of `downtown_quake_launch_script.py`, and built the same way:
`scene_launch_script.py` with ONE pass added after the city is generated. No
drone, no sensors — this is the LOOKING launcher.

WHY THIS REPLACES `urban_fire_city250_launch_script.py`
-------------------------------------------------------
Two things changed.

1. THE HOLLOW-SHELL KIT IS GONE. That launcher drew 55% of its buildings from
   `detail/urban_building.py` — façade modules assembled into a hollow shell,
   which is what `urban_fire.burn_building`'s gutted interiors, burnt-through
   floors and partial collapses need to exist. The city library is now WHOLE
   assets (GreatAmericanCity + downtowncity + the Muyang and AEC stock, see
   `asset_sets/urban_gac.yaml`), every one a single mesh with a handful of
   `GeomSubset`s and no elements at all. So every building here takes the
   `burn_monolith` path: sooted in its own materials, plume tongues on the
   measured wall, flame out of the burning band, a charred roof, glass and
   spalled render on the pavement. Nothing is fractured and nothing collapses.

2. IT ROLLED ITS OWN PACKER. `generate-urban-city` names that launcher as one
   of the two places the mistake was made. The city here is the generator's —
   road hierarchy, OSM-calibrated blocks, district zoning, NACTO sidewalk
   furniture, crosswalks, rooftop plant — and the fire is a pass over the
   placements it returns, exactly as `quake.assemble` is.

MODERATE, AND WHAT THAT MEANS
------------------------------
`UF_LEVEL=moderate` (the default) is **22% OF THE CITY ALIGHT**, and the clock
is solved for that rather than fixed. That is not a nicety: the same T+195 min
involved 8 buildings from a poorly-connected ignition and 63 from a
well-connected one, and an unrelated edit to `districts.py` moved the old
fixed-clock "moderate" from 21% of the plate to 6% with no change to the fire
code at all. Stating the SHARE makes a layout change move the time, which
nobody looks at, instead of the scene, which everybody does.

MEASURED over three layout seeds (`tools/urban_fire_dryrun.py --wind 20,8`,
97-111 buildings on 15 blocks):

    rung       solved T+     involved   a typical ladder
    light       83- 88 min   10%        F1=2  F2=1  F3=7  F4=1
    moderate   130-149 min   22%        F1=3  F2=5  F3=8  F4=8
    severe     173-237 min   40%        F1=5  F2=8  F3=20 F4=11

Moderate is the rung where the WHOLE LADDER is present at once and most of
downtown is still standing clean — a burnt-out core, a ring of fully-involved
façades, an outer edge only just alight. `UF_ELAPSED` sets the clock directly
instead and lets the share fall out of it, which is the right way round for
"what does this plate look like an hour in".

Preview any of it host-side in a second, no container:

    python3 scene_gen/tools/urban_fire_dryrun.py --config downtown_gac \
        --region 500 --wind 20,8

`.env` SHIPS `SCENE_CONFIG="suburb"` AND `REGION_M="250x250"`, AND THE
CONTAINER INHERITS BOTH. Honouring them here builds a 250 m SUBURB and calls
it a downtown fire — the same trap `downtown_quake_launch_script.py` records
for `DISASTER_TYPE`. So this launcher has its OWN names, which always win, and
it accepts the shared pair only when `SCENE_CONFIG` actually names a downtown
preset (the two travel together: a region without its preset is how the 250 m
suburb value would get in on its own).

Env:
    UF_CONFIG      preset (default downtown_gac). `SCENE_CONFIG` is honoured
                   instead only when it names a `downtown*` preset.
    UF_REGION      plate size, m (default 500) — "N", "NxN" or "N,N".
                   `REGION_M` is honoured instead only when `SCENE_CONFIG`
                   was accepted above.
    UF_LAYOUT_SEED layout seed override (default: the preset's)
    UF_LEVEL       light | moderate | severe  (or 1 | 2 | 3; default moderate)
                   — a SHARE of the city, from which the clock is solved
    UF_ELAPSED     minutes since ignition; sets the clock directly instead,
                   and the share then falls out of the layout
    UF_WIND        "<deg>,<mps>", the direction it blows TOWARD. ABOVE 8 m/s
                   the spread model saturates and a bigger number changes
                   nothing (`urban_fire_spread._wind_factor`).
    UF_ORIGIN      "x,y" metres — where it started (default: upwind quarter)
    UF_SEED        the fire's own draws (default 21); the layout seed is the
                   preset's, so a re-roll of one does not disturb the other
    UF_FLOW        0 authors no NVIDIA Flow — geometry and materials only,
                   which is much faster and still shows every burnt façade
    UF_EMITTERS    Flow emitters per burning building (default 4)
    UF_FLOW_CELL   Flow density cell, m (default 0.40). THE BENCH'S 0.10 RAN
                   THE GPU OUT OF MEMORY ON THIS PLATE and rendered a city
                   with no smoke in it — see `urban_fire_city.FLOW_CELL_M`.
    UF_FLOW_BLOCKS Flow block-pool ceiling (default 8192)
    UF_FLOW_BUDGET total emitters across the plate (default 48); the buildings
                   that miss out still carry their fire in geometry
    AUTOPLAY       0 keeps the timeline stopped even with Flow on
    SNAP_DIR       viewport captures, under /isaac-sim/.nvidia-omniverse/logs/
    KEEP_OPEN      1 keeps a HEADLESS run up after the captures are written
"""

import json
import os
import sys
import time

import carb
from isaacsim import SimulationApp


def _env(name, default=""):
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
simulation_app = SimulationApp(launch_config={"headless": _HEADLESS})

from isaacsim.core.utils.extensions import enable_extension          # noqa: E402

enable_extension("omni.kit.window.script_editor")
# omni.flowusd MUST be enabled before any FlowEmitter prim is authored, or the
# prims compose as untyped defs and nothing simulates.
enable_extension("omni.flowusd")

import omni.kit.app                                                  # noqa: E402
import omni.timeline                                                 # noqa: E402
import omni.usd                                                      # noqa: E402
from pxr import UsdGeom                                              # noqa: E402
from omni.isaac.core.world import World                              # noqa: E402
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS         # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import (    # noqa: E402
    PegasusInterface)

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)
from scene_prep import (scale_stage_prim, add_sky, add_dome_light,   # noqa: E402
                        get_stage_meters_per_unit, settle_rigid_props)
from scene_generator import resolve_sky                              # noqa: E402
from generate_scene import generate_scene_on_stage                   # noqa: E402
from compile_disaster import load_scene_config                       # noqa: E402
from disaster import urban_fire as uf                                # noqa: E402
from disaster import urban_fire_city as ufc                          # noqa: E402
from disaster import urban_fire_spread as ufs                        # noqa: E402

ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]


def _resolve_config():
    """(preset, whether the shared SCENE_CONFIG/REGION_M pair may be used).

    See the `.env` note in this module's docstring: `SCENE_CONFIG="suburb"`
    and `REGION_M="250x250"` are inherited from the compose environment by
    every launcher, and silently building the suburb at 250 m is a failure
    that looks like a bad scene rather than a bad variable.
    """
    own = _env("UF_CONFIG")
    if own:
        return own, False
    shared = _env("SCENE_CONFIG")
    if shared and os.path.basename(shared).startswith("downtown"):
        return shared, True
    if shared:
        print("[fire_city] ignoring inherited SCENE_CONFIG={0!r} — this is the "
              "downtown fire launcher; set UF_CONFIG to override".format(shared))
    return "downtown_gac", False


SCENE_CONFIG, _SHARED_OK = _resolve_config()
PARENT = "/World/stage/generated"
SNAP_DIR = _env("SNAP_DIR")
UF_LEVEL = _env("UF_LEVEL", "moderate")
UF_SEED = int(_env("UF_SEED", "21"))
UF_FLOW = _env("UF_FLOW", "1") not in ("0", "false", "no")
UF_EMITTERS = int(_env("UF_EMITTERS", str(ufc.FLOW_EMITTERS_PER_BUILDING)))
UF_FLOW_CELL = float(_env("UF_FLOW_CELL", str(ufc.FLOW_CELL_M)))
UF_FLOW_BLOCKS = int(_env("UF_FLOW_BLOCKS", str(ufc.FLOW_MAX_BLOCKS)))
UF_FLOW_BUDGET = int(_env("UF_FLOW_BUDGET", str(ufc.FLOW_EMITTER_BUDGET)))
AUTOPLAY = _env("AUTOPLAY", "1" if UF_FLOW else "0") in ("1", "true", "yes")

# NOT `import scene_launch_script`: that module builds its own SimulationApp at
# import, and a second Kit app in one process segfaults inside the first
# second. The three helpers it would have lent are copied, as the quake city
# launcher copies them.
_ENV_CLUTTER = {"GroundPlane", "Environment"}


def _remove_env_clutter(stage):
    """Deactivate the base environment's ground plane and grid backdrop.

    The generator lays its own ground, so these z-fight with it and the grid
    mesh reads as a blue square under the spawn point. `RemovePrim` cannot
    delete across the reference Pegasus composes them through — it returns
    False rather than raising — so deactivation is the operation that works.
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
            if not child.SetActive(False):
                UsdGeom.Imageable(child).MakeInvisible()
            n += 1
    print("[fire_city] env clutter: {0} prim(s) deactivated".format(n))


def _disable_sky_sun(stage):
    """Switch off the hard sun the sky RIG hangs under its axis chain.

    A `.hdr` sky (which `downtown_gac` uses) takes the dome branch and builds
    no rig, so this correctly finds nothing; it is here for a preset that
    points `sky:` at a `.usd`.
    """
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "DistantLight":
            continue
        if "/Environment/sky/" not in str(prim.GetPath()):
            continue
        if prim.IsActive() and prim.SetActive(False):
            n += 1
    print("[fire_city] sky sun: {0} DistantLight(s) disabled".format(n))


def _wait_for_stage(stage, timeout_s=10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            if [c for c in world_prim.GetChildren()
                    if c.GetName() != "PhysicsScene"]:
                return True
        time.sleep(0.1)
    return False


def _spec_overrides():
    """The region and layout seed, applied to the PRESET before compiling.

    500 m, not the preset's 800: this scene is asked for at 500. The block
    targets are left alone — at 500 m they give 15 blocks and 112 buildings,
    which is a dense enough graph for a fire to spread through (measured,
    `tools/urban_fire_dryrun.py`). What 500 m does cost is the biggest stock:
    the generator reports ~30 of 43 large buildings with no block that fits
    them, so the 300 m towers mostly go unplaced. Raise REGION_M, or shrink
    `districts.typologies.*.block_long_m`, if those are wanted.
    """
    ov = {}
    raw = _env("UF_REGION") or (_env("REGION_M") if _SHARED_OK else "") or "500"
    parts = [p.strip() for p in raw.replace("x", ",").replace("X", ",").split(",")
             if p.strip()]
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        raise SystemExit("region {0!r}: expected N, NxN or N,N".format(raw))
    if not vals or any(v <= 0.0 for v in vals):
        raise SystemExit("region {0!r}: sides must be positive".format(raw))
    ov["region_m"] = [vals[0], vals[0]] if len(vals) == 1 else vals[:2]
    s = _env("UF_LAYOUT_SEED")
    if s:
        ov["seed"] = int(s)
    return ov


def _fire_kwargs():
    kw = {"level": UF_LEVEL, "seed": UF_SEED, "flow": UF_FLOW,
          "max_emitters": UF_EMITTERS, "flow_cell_m": UF_FLOW_CELL,
          "flow_max_blocks": UF_FLOW_BLOCKS, "flow_budget": UF_FLOW_BUDGET}
    e = _env("UF_ELAPSED")
    if e:
        kw["elapsed_min"] = float(e)
    w = _env("UF_WIND")
    if w:
        p = [float(v) for v in w.replace("x", ",").split(",") if v.strip()]
        kw["wind"] = (p[0], p[1] if len(p) > 1 else 8.0)
        if kw["wind"][1] > ufc.WIND_SATURATION_MPS:
            print("[fire_city] NOTE: wind {0:.0f} m/s is above the {1:.0f} m/s "
                  "saturation in urban_fire_spread._wind_factor — the solve "
                  "is identical to {1:.0f}".format(kw["wind"][1],
                                                   ufc.WIND_SATURATION_MPS))
    o = _env("UF_ORIGIN")
    if o:
        p = [float(v) for v in o.replace("x", ",").split(",") if v.strip()]
        kw["origin"] = (p[0], p[1])
    return kw


class FireCityApp:

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
        if stage.GetPrimAtPath("/World/stage").IsValid():
            scale_stage_prim(stage, "/World/stage", 1.0)
            for _ in range(10):
                omni.kit.app.get_app().update()

        # ---- the city -------------------------------------------------------
        ov = _spec_overrides()
        print("[fire_city] spec overrides: {0}".format(ov))
        problems = uf.check(verbose=False) + ufs.check(verbose=False) \
            + ufc.check(verbose=False)
        if problems:
            raise RuntimeError("; ".join(problems))
        config = load_scene_config(SCENE_CONFIG, spec_overrides=ov)
        self.config = config
        _, ssf = get_stage_meters_per_unit(stage)
        self.ssf = ssf
        t0 = time.time()
        placements = generate_scene_on_stage(
            stage, config, parent_path=PARENT, scene_scale_factor=ssf)
        t_layout = time.time() - t0
        print("[fire_city] layout in {0:.0f} s".format(t_layout))
        for _ in range(10):
            omni.kit.app.get_app().update()

        # PHYSICS COLLIDERS ARE OFF, deliberately — the same call the plain
        # city preview leaves commented out. Applying CollisionAPI to every
        # gprim under the generated root makes PhysX cook collision for all of
        # them inside ONE `app.update()`, which ran long enough that X
        # invalidated the window and Kit tore down a render thread without the
        # GIL. Nothing here needs physics: the fire pass authors no rigid
        # bodies, and neither rendering nor LiDAR reads CollisionAPI.

        # ---- the fire -------------------------------------------------------
        t1 = time.time()
        self.stats = ufc.assemble(stage, config, placements, parent=PARENT,
                                  ssf=ssf, **_fire_kwargs())
        t_fire = time.time() - t1
        for _ in range(10):
            omni.kit.app.get_app().update()

        settle_rigid_props(
            stage,
            [p["prim_path"] for p in placements
             if p.get("settle") and p.get("prim_path")],
            ground_path=PARENT + "/ground")
        add_sky(stage, resolve_sky(config),
                intensity=float(config.get("sky_intensity", 3500.0)),
                exposure=float(config.get("sky_exposure", -3.0)))
        _disable_sky_sun(stage)
        # AMBIENT FILL, SEPARATELY — `add_sky` RETURNS EARLY when it borrows a
        # `.usd` sky, so the intensity and exposure handed to it above are
        # silently ignored and the only light left is whatever the borrowed
        # stage brought, minus the sun `_disable_sky_sun` just switched off.
        # `downtown_gac` has no `sky:` key any more, so it takes exactly that
        # branch and the first run of this scene came out reading as evening.
        # Same fix, and the same reasoning, as `scene_launch_script.py`.
        fill = float(config.get("sky_intensity", 0.0) or 0.0)
        if fill > 0.0:
            add_dome_light(stage, "/World/FillDome", intensity=fill,
                           exposure=float(config.get("sky_exposure", 0.0)))
        self.placements = placements
        t_ready = time.time() - t_launch

        if SNAP_DIR:
            try:
                os.makedirs(SNAP_DIR, exist_ok=True)
                with open(os.path.join(SNAP_DIR, "fire_buildings.json"), "w") as fh:
                    json.dump({k: v for k, v in self.stats.items()
                               if k != "notes"}, fh, indent=1)
            except Exception as exc:
                print("[fire_city] could not write fire records: {0}".format(exc))

        st = self.stats
        region = max(float(v) for v in config["layout"]["region_m"])
        print("\n" + "=" * 74)
        print("URBAN FIRE DOWNTOWN READY   {0:.0f} x {0:.0f} m".format(region))
        print("  config: {0}   level: {1}   T+{2:.0f} min   wind {3:.0f} deg "
              "@ {4:.0f} m/s".format(SCENE_CONFIG, UF_LEVEL, st["elapsed_min"],
                                     st["wind"][0], st["wind"][1]))
        if st["ignition"]:
            print("  ignition: {0} at ({1:.0f}, {2:.0f})".format(
                st["ignition"]["style"], st["ignition"]["x"],
                st["ignition"]["y"]))
        print("  buildings {0}, involved {1} ({2:.0%}): {3}".format(
            st["buildings"], st["involved"],
            st["involved"] / float(max(1, st["buildings"])),
            "  ".join("{0}={1}".format(k, st["tally"][k])
                      for k in ufs.LEVELS if k in st["tally"])))
        print("  flow: {0}   timeline: {1}".format(
            "on" if UF_FLOW else "OFF (UF_FLOW=1 for flame and smoke)",
            "PLAYING" if AUTOPLAY else "stopped (AUTOPLAY=1 to play)"))
        print("  TIMING  layout {0:.0f} s   fire {1:.0f} s   env->ready {2:.0f} s"
              .format(t_layout, t_fire, t_ready))
        print("=" * 74 + "\n")

        # PLAY ONLY WHEN THERE IS SOMETHING TO SIMULATE. A static review scene
        # is judged on a deterministic frame and a running timeline is not free
        # — which is why `scene_launch_script.py` refuses to autoplay. Flow is
        # the exception it names: flame and smoke do not exist until the
        # simulation has stepped.
        if AUTOPLAY:
            self.timeline.play()

    def snapshots(self, app):
        import importlib.util as _ilu
        sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
        spec = _ilu.spec_from_file_location("snapshots", sp)
        sn = _ilu.module_from_spec(spec)
        spec.loader.exec_module(sn)
        os.makedirs(SNAP_DIR, exist_ok=True)
        region = max(float(v) for v in self.config["layout"]["region_m"])
        # LET THE FIRE DEVELOP BEFORE CAPTURING. A Flow volume is empty on the
        # frame the emitters are authored; the plume needs a few hundred steps
        # to fill and rise, and a capture taken before that shows a burnt city
        # with no smoke in it.
        for _ in range(340 if AUTOPLAY else 60):
            app.update()
        sn.overview(self.stage, (0.0, 0.0), region * 1.05,
                    os.path.join(SNAP_DIR, "city_top.png"), self.ssf)
        for nm, (ax, ay) in (("sw", (-1, -1)), ("se", (1, -1)),
                             ("ne", (1, 1)), ("nw", (-1, 1))):
            d = region * 0.78
            tall = max([r["H"] for r in self.stats["records"]] or [60.0])
            sn.place_camera(self.stage,
                            ((ax * d) * self.ssf, (ay * d) * self.ssf,
                             (0.42 * d + tall * 0.55) * self.ssf),
                            (0.0, 0.0, tall * 0.22 * self.ssf))
            sn.snapshot(os.path.join(SNAP_DIR, "city_" + nm + ".png"))
        # THE WORST BUILDINGS, ONE PAIR OF VIEWS EACH. A city-wide oblique
        # cannot show whether a single façade reads; these are what a burnt
        # building is actually judged on.
        rank = {"F5": 5, "F4": 4, "F3": 3, "F2": 2, "F1": 1}
        worst = sorted(self.stats["records"],
                       key=lambda r: -rank.get(r["level"], 0))[:8]
        sn.views_around(
            self.stage,
            {"b{0}_{1}_{2}".format(i, r["level"], r["style"][:18]):
             (r["x"], r["y"]) for i, r in enumerate(worst)},
            SNAP_DIR, self.ssf, top_h=max(70.0, 1.4 * max(
                [r["H"] for r in worst] or [50.0])),
            obl_dist=60.0, obl_h=30.0)
        print("[fire_city] snapshots -> {0}".format(SNAP_DIR))

    def run(self):
        app = omni.kit.app.get_app()
        try:
            n_prims = sum(1 for _ in self.stage.Traverse())
            t = time.time()
            app.update()
            first_s = time.time() - t
            for _ in range(10):
                app.update()
            t, n = time.time(), 60
            for _ in range(n):
                app.update()
            dt = time.time() - t
            print("[fire_city] HYDRA  {0} stage prims, first frame {1:.1f} s, "
                  "{2:.1f} fps / {3:.0f} ms per frame".format(
                      n_prims, first_s, n / dt if dt else 0.0,
                      1000.0 * dt / n), flush=True)
        except Exception as exc:
            print("[fire_city] hydra timing failed: {0}".format(exc))
        if SNAP_DIR:
            try:
                self.snapshots(app)
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print("[fire_city] snapshots FAILED: {0}".format(exc))
        if _env("KEEP_OPEN") == "1" or not _HEADLESS:
            while simulation_app.is_running():
                app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    FireCityApp().run()


if __name__ == "__main__":
    main()
