#!/usr/bin/env python
"""
Assemble the 1600 x 1200 burnt plat by REFERENCE — no live fracture or settle.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes \
    SCENE_CONFIG=suburb_wildfire \
    ISAAC_SIM_SCRIPT_NAME=suburb_assemble_launch_script.py airstack up isaac-sim

This is the THIN launcher for `scene_gen/scene_api.build_scene`: boot an app,
give Pegasus a World, hand the stage to the API, print what came back, look at
it. The plat itself — layout, fire field, damage archetypes, survivors, ground
scar — lives in the API, so the drone launcher
(`example_multi_drone_scene_import.py` with `SCENE_CONFIG` set) builds the
identical scene from the identical code. See `.agents/skills/launch-generated-
scene-with-drones/SKILL.md`.

`generate_suburb_on_stage(assembly=True)` builds the CHEAP layer live — streets,
ground, driveways, walks, fences, props, ground scar — and hands back each
house's (style, pose) and each tree's (species, pose) instead of building their
geometry. The API then references a pre-baked damage archetype
(`bake_archetypes_launch_script.py`) for every house and tree at the damage
level the fire gives it. The whole plat's collapse is thus O(reference), not
O(fracture+settle) — the scaling path the mini bake proved at 255x.

SURVIVORS. `disaster.people` plans ~60 live people into the six situations
search-and-rescue after-action reports actually find them in (see that
module's docstring for the Camp Fire / Lahaina numbers the shares come from),
plus the cars of a gridlocked egress queue and whatever blocked it. THE ORDER
IS LOAD-BEARING — the queue's CARS are authored before the burnable/scorch pass
and the PEOPLE after it — and `scene_api.build_scene` is where that order now
lives. Ground truth lands in ``PEOPLE_JSON`` (default
``scene_gen/assets/archetypes/humans_<seed>.json``).
"""

import os
import sys
import time

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
from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

from scene_prep import get_stage_meters_per_unit     # noqa: E402
from scene_api import build_scene, LOCAL_ARCH_DIR    # noqa: E402
from disaster import people as ppl                   # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_wildfire")
SEED = int(os.environ.get("MINI_SEED", "11"))
ARCH_DIR = os.environ.get("ARCH_DIR") or None
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
# Written with plain `open()`, so it stays on the FILESYSTEM even when the bake
# it is named after lives on Nucleus.
PEOPLE_JSON = os.environ.get("PEOPLE_JSON") or os.path.join(
    LOCAL_ARCH_DIR, "humans_{0}.json".format(SEED))
# PEOPLE_POLES=1 authors the magenta group markers.
POLES = os.environ.get("PEOPLE_POLES", "").strip().lower() in ("1", "true", "yes")
# MINI_BURN_FRAC is the share of houses inside the burn; MINI_ELAPSED still
# overrides outright.
BURN_FRAC = float(os.environ.get("MINI_BURN_FRAC", "0.45"))
ELAPSED = float(os.environ.get("MINI_ELAPSED", "0"))
# Where the launcher photographs itself. Empty = no captures.
SNAP_DIR = os.environ.get("SNAP_DIR", "")
if SNAP_DIR:
    os.makedirs(SNAP_DIR, exist_ok=True)


def wait_for_stage(stage, timeout_s=20.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        wp = stage.GetPrimAtPath("/World")
        if wp.IsValid() and [c for c in wp.GetChildren()
                             if c.GetName() != "PhysicsScene"]:
            return True
    return False


def snapshots(stage, ssf, info, stats):
    """Photograph the result. Nobody is at the GUI on a scripted run, and a
    scene this size can only be judged by looking at it: an overview of the
    plate, then a top-down and an oblique of each thing the build is supposed
    to have got right."""
    import importlib.util as _ilu
    _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
    _spec = _ilu.spec_from_file_location("snapshots", os.path.normpath(_sp))
    _snaps = _ilu.module_from_spec(_spec)
    _spec.loader.exec_module(_snaps)
    binfo = info.get("binfo") or {}
    pts = {}
    lot = ((binfo.get("park") or {}).get("parking") or {})
    if lot.get("centre"):
        pts["refuge_lot"] = tuple(lot["centre"])
    for i, b in enumerate((info.get("blockers") or [])[:2]):
        if b.get("x") is not None:
            pts["blocker_%d" % i] = (b["x"], b["y"])
    byscen = {}
    for r in info.get("records") or ():
        byscen.setdefault(r["scenario"], r)
    for name in ("pools", "open_ground", "at_home", "exposed_interior"):
        r = byscen.get(name)
        if r:
            pts[name] = (r["x"], r["y"])
    reg = stats["region"]
    _snaps.overview(stage, (0.5 * (reg[0] + reg[2]), 0.5 * (reg[1] + reg[3])),
                    max(reg[2] - reg[0], reg[3] - reg[1]),
                    os.path.join(SNAP_DIR, "overview.png"), ssf)
    _snaps.views_around(stage, pts, SNAP_DIR, ssf)
    print("[assemble] snapshots -> {0} ({1} subject(s))"
          .format(SNAP_DIR, len(pts)))


def main():
    t0 = time.time()
    omni.timeline.get_timeline_interface().stop()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    # Pegasus' flat default env is loaded only to give the World a valid base;
    # its ground plane and lighting go straight back off because the generated
    # plat brings its own ground and the config's own sky.
    pg.load_environment(ENV_URL)
    stage = omni.usd.get_context().get_stage()
    wait_for_stage(stage)
    for name in ("GroundPlane", "Environment"):
        p = stage.GetPrimAtPath("/World/" + name)
        if p and p.IsValid():
            p.SetActive(False)

    _, ssf = get_stage_meters_per_unit(stage)

    info = {}
    st = build_scene(stage, SCENE_CONFIG, ssf, arch_dir=ARCH_DIR, seed=SEED,
                     burn_frac=BURN_FRAC, elapsed=ELAPSED, poles=POLES,
                     parent_path=PARENT, people_json=PEOPLE_JSON,
                     info_out=info)

    # The extra_args form only covers startup; re-assert once the stage is
    # composed so car glass stays see-through for the whole run.
    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    for _ in range(30):
        omni.kit.app.get_app().update()

    reg = st["region"]
    print("\n" + "=" * 72)
    print("ASSEMBLED {0:.0f} x {1:.0f} (by reference, no live fracture/settle)"
          .format(reg[2] - reg[0], reg[3] - reg[1]))
    print("  total       {0:.0f} s".format(time.time() - t0))
    print("  houses      {0} referenced ({1} missing); {2}".format(
        st["houses"], st["houses_missing"],
        ", ".join("%s=%d" % kv for kv in sorted(st["house_tally"].items()))))
    print("  trees       {0} referenced ({1} missing); {2}".format(
        st["trees"], st["trees_missing"],
        ", ".join("%s=%d" % kv for kv in sorted(st["tree_tally"].items()))))
    print("  ground scar {0} band(s)".format(st["bands"]))
    print("  people      {0} alive, {1} car(s), {2} blocker(s) -> {3}".format(
        st["people_alive"], st["cars"], st["blockers"], st["people_json"]))
    for _name in ppl.SCENARIOS:
        print("    {0:<18} {1:>3}".format(_name,
                                          st["people_tally"].get(_name, 0)))
    print("=" * 72 + "\n")

    if SNAP_DIR:
        try:
            snapshots(stage, ssf, info, st)
        except Exception as _exc:
            print("[assemble] snapshots FAILED: {0}".format(_exc))

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
