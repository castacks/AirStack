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
QUAKE_SEED = int(os.environ.get("QUAKE_SEED") or "11")
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
    m = os.environ.get("MAGNITUDE", "").strip()
    if m:
        ov["magnitude"] = float(m.lstrip("Mm"))
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
        from pxr import UsdShade
        soil = _qf.materials(stage, PARENT + "_void")["soil"]
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath(soil)))
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
            stats = quake.assemble(stage, config, placements, ARCH_DIR,
                                   seed=QUAKE_SEED + ci, ssf=ssf, **kw)
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
        t_ready = time.time() - t_launch

        for city in self.cities:
            settle_rigid_props(
                stage,
                [p["prim_path"] for p in placements
                 if p.get("settle") and p.get("prim_path")
                 and p["prim_path"].startswith(city["parent"] + "/")],
                ground_path=city["parent"] + "/ground")
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
                xs = [c["offset"][0] for c in self.cities]
                ws = [max(map(float, c["config"]["layout"]["region_m"])) for c in self.cities]
                span_all = (max(xs) - min(xs)) + max(ws)
                _snaps.overview(self.stage, ((max(xs) + min(xs)) / 2.0, 0.0), span_all * 1.05,
                                os.path.join(SNAP_DIR, "plat_top.png"), self.ssf)
                for ci, city in enumerate(self.cities):
                    pre = "" if len(self.cities) == 1 else "c{0}_".format(ci)
                    dx, dy = city["offset"]
                    span = ws[ci]
                    if pre:
                        _snaps.overview(self.stage, (dx, dy), span * 1.05,
                                        os.path.join(SNAP_DIR, pre + "plat_top.png"), self.ssf)
                    # obliques from the four corners, and the epicentre close up
                    fld = (city["config"].get("disaster") or {}).get("field") or {}
                    ex, ey = fld.get("center", [0.0, 0.0])
                    pts = {pre + "epi": (dx + float(ex), dy + float(ey)),
                           pre + "ne": (dx + span * 0.25, dy + span * 0.25),
                           pre + "sw": (dx - span * 0.25, dy - span * 0.25),
                           pre + "nw": (dx - span * 0.25, dy + span * 0.25),
                           pre + "se": (dx + span * 0.25, dy - span * 0.25)}
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
        # headless: exit once the captures are on disk (KEEP_OPEN=1 to stay)
        if (os.environ.get("KEEP_OPEN", "").strip() == "1"
             or os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower() not in ("1", "true", "yes")):
            while simulation_app.is_running():
                app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    QuakeCityApp().run()


if __name__ == "__main__":
    main()
