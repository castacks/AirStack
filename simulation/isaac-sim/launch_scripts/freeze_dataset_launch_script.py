#!/usr/bin/env python
"""Build one FROZEN dataset scene: geometry + GT_people.json + GT_hints.json.

    FREEZE_OUT=/isaac-sim/AirStack/final_disaster_dataset/Fire/Suburban/level_1/1 \
    SCENE_CONFIG=suburb_wildfire_1000 \
    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes \
    ISAAC_SIM_SCRIPT_NAME=freeze_dataset_launch_script.py \
    airstack up isaac-sim

This is the DATASET launcher. `suburb_assemble_launch_script.py` is the same
build for LOOKING at; this one is the same build for KEEPING, and the
difference is entirely in what it writes out beside the stage:

    <FREEZE_OUT>/GT_people.json    every survivor, from `disaster.people`
    <FREEZE_OUT>/GT_hints.json     every other labelled object, `disaster.gt_hints`
    <FREEZE_OUT>/snaps/            what to review it by

`FREEZE_EXPORT=1` also writes the frozen scene itself — one self-contained
`<FREEZE_NAME>.usd` plus a `Materials/` folder, via `disaster.freeze`. It runs
AFTER the ground truth, so a failed export still leaves usable labels behind.

WHY THE PEOPLE PASS IS A VARIANT AXIS. The dataset ships N people placements per
frozen scene, so the geometry is built once and the survivor plan is re-drawn.
`PEOPLE_VARIANT=k` offsets the people rng ONLY (`people.seed`), leaving layout,
fire and damage bit-identical — which is the whole point: the same scene with
the targets somewhere else.

Env knobs (everything `suburb_assemble_launch_script.py` takes, plus):

    FREEZE_OUT        output directory. REQUIRED.
    FREEZE_DISASTER   hint ladder to use; default from the config's
                      `disaster-type` ("wildfire", "tornado", ...)
    PEOPLE_VARIANT    integer, added to `people.seed`. Default 0.
    FREEZE_SNAPS      1 (default) to photograph the plate for review.
    FREEZE_EXPORT     1 to flatten the scene into <FREEZE_OUT>.
    FREEZE_COLLECT    1 to also localise textures/MDL (off — see disaster.freeze).
    FREEZE_EXIT       1 to close the app after the export, for batch loops.
    FREEZE_NAME       basename of the frozen usd. Default: derived from
                      FREEZE_OUT as <disaster>_<locale>_lvl<n>_<k>.
    FREEZE_LAYOUT_SEED  the preset's own `seed` — THE LAYOUT. Changes the
                      street network, the blocks, the lots, the park and its
                      internal zones. This is what makes one intensity level a
                      different PLACE rather than the same place burnt harder,
                      and it is NOT `MINI_SEED` (damage, vegetation, people).
    FREEZE_EPICENTER  "x,y" in metres — where the fire started.
    FREEZE_HEADING    degrees the wind pushes it; 0 = toward +x, 45 = up the
                      diagonal. THE PARK IS CENTRAL on most seeds, so a
                      diagonal from a corner runs straight through it and the
                      scene reads as "the fire started in the park". Running
                      the front along an EDGE avoids that.
    FREEZE_SEVERITY   the preset's `severity`, 0..1. For a wildfire this is the
                      RATE OF SPREAD (`head_mps` 0.25 -> 2.0) and the wind, not
                      the damage ladder directly — a slower front reaches fewer
                      houses and gives those it reaches less burn age, which is
                      what makes a level milder. Pair it with MINI_BURN_FRAC,
                      the share of houses inside the burn.
"""

import json
import os
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": (os.environ.get("FREEZE_HEADLESS") or "").strip().lower()
                in ("1", "true", "yes"),
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
from scene_api import build_scene                    # noqa: E402
from disaster import gt_hints                        # noqa: E402
from disaster import people as ppl                   # noqa: E402

def env(name, default=""):
    """`os.environ.get` that treats an EMPTY value as absent.

    THE CONTAINER EXPORTS EVERY LAUNCHER KNOB AS `""`. `ARCH_DIR`, `ARCH_SEED`,
    `SETTLE_STEPS` and friends are declared-but-unset in the image, so they are
    PRESENT with an empty value and `os.environ.get(name, default)` never
    reaches its default: numeric knobs raise `int('')` fourteen seconds into a
    launch and path knobs silently become `""`, which `os.makedirs` accepts as
    a no-op. See run-isaac-sim-launcher.
    """
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


PARENT = "/World/stage/generated"
SCENE_CONFIG = env("SCENE_CONFIG", "suburb_wildfire_1000")
SEED = int(env("MINI_SEED", "11"))
ARCH_DIR = env("ARCH_DIR") or None
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
BURN_FRAC = float(env("MINI_BURN_FRAC", "0.45"))
ELAPSED = float(env("MINI_ELAPSED", "0"))
VARIANT = int(env("PEOPLE_VARIANT", "0"))
LAYOUT_SEED = env("FREEZE_LAYOUT_SEED")
SEVERITY = env("FREEZE_SEVERITY")
EPICENTER = env("FREEZE_EPICENTER")      # "x,y" in metres
HEADING = env("FREEZE_HEADING")          # degrees, 0 = toward +x
OUT = env("FREEZE_OUT")
DISASTER = env("FREEZE_DISASTER")
SNAPS = env("FREEZE_SNAPS", "1").lower() not in ("0", "false", "no")
EXPORT = env("FREEZE_EXPORT", "0").lower() in ("1", "true", "yes")
NAME = env("FREEZE_NAME")

if not OUT:
    raise SystemExit("freeze_dataset_launch_script: FREEZE_OUT is required")
os.makedirs(OUT, exist_ok=True)
SNAP_DIR = os.path.join(OUT, "snaps")
if SNAPS:
    os.makedirs(SNAP_DIR, exist_ok=True)
PEOPLE_JSON = os.path.join(OUT, "GT_people.json")
HINTS_JSON = os.path.join(OUT, "GT_hints.json")


def _default_name(out_dir):
    """`<disaster>_<locale>_lvl<n>_<k>` from the dataset path.

    The contract is `.../<Disaster>/<Locale>/level_<n>/<k>/`, capitalised in the
    PATH and lowercase in the FILENAME, so the name is derivable and does not
    need a second env var to disagree with the directory it lands in.
    """
    parts = [q for q in os.path.abspath(out_dir).split(os.sep) if q]
    if len(parts) >= 4 and parts[-2].startswith("level_"):
        dis, loc = parts[-4].lower(), parts[-3].lower()
        return "{0}_{1}_lvl{2}_{3}".format(
            dis, loc, parts[-2].split("_", 1)[1], parts[-1])
    return "scene"


def _disaster_kind(config):
    """"wildfire" / "tornado" / ... from a COMPILED config.

    `compile_spec` STRIPS `disaster-type` from its output (it is a build input,
    and leaving it in would suggest editing it there has an effect), so the
    kind has to be recovered from the shape of the compiled `disaster` block.
    `FREEZE_DISASTER` overrides this and is the honest answer for anything the
    section below cannot tell apart.
    """
    d = config.get("disaster") or {}
    if d.get("fire"):
        return "wildfire"
    for key, kind in (("tornado", "tornado"), ("track", "tornado"),
                      ("quake", "earthquake"), ("earthquake", "earthquake"),
                      ("hurricane", "hurricane"), ("flood", "hurricane")):
        if d.get(key):
            return kind
    return "wildfire"


def wait_for_stage(stage, timeout_s=20.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        wp = stage.GetPrimAtPath("/World")
        if wp.IsValid() and [c for c in wp.GetChildren()
                             if c.GetName() != "PhysicsScene"]:
            return True
    return False


def _load_snaps():
    import importlib.util as _ilu
    _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
    _spec = _ilu.spec_from_file_location("snapshots", os.path.normpath(_sp))
    _m = _ilu.module_from_spec(_spec)
    _spec.loader.exec_module(_m)
    return _m


def review_snaps(stage, ssf, info, stats):
    """Photograph the plate for a human to sign off on.

    A 1 km plate cannot be judged from one overview: at 1000 m the whole thing
    is 1.6 px a metre and a house is a smudge. So it is the overview PLUS a
    grid of tiles across the plate, PLUS a close pair on each of the features
    the build is supposed to have got right.
    """
    snaps = _load_snaps()
    reg = stats["region"]
    cx, cy = 0.5 * (reg[0] + reg[2]), 0.5 * (reg[1] + reg[3])
    span = max(reg[2] - reg[0], reg[3] - reg[1])
    snaps.overview(stage, (cx, cy), span, os.path.join(SNAP_DIR, "overview.png"),
                   ssf)
    # A 3 x 3 walk of the plate at ~1/3 span, which is the altitude a house
    # reads at.
    pts = {}
    step = span / 3.0
    for iy in range(3):
        for ix in range(3):
            pts["tile_%d%d" % (iy, ix)] = (reg[0] + step * (ix + 0.5),
                                           reg[1] + step * (iy + 0.5))
    binfo = info.get("binfo") or {}
    lot = ((binfo.get("park") or {}).get("parking") or {})
    if lot.get("centre"):
        pts["refuge_lot"] = tuple(lot["centre"])
    for i, b in enumerate((info.get("blockers") or [])[:2]):
        if b.get("x") is not None:
            pts["blocker_%d" % i] = (b["x"], b["y"])
    byscen = {}
    for r in info.get("records") or ():
        byscen.setdefault(r["scenario"], r)
    for name, r in byscen.items():
        pts["people_" + name] = (r["x"], r["y"])
    snaps.views_around(stage, pts, SNAP_DIR, ssf)
    print("[freeze] snapshots -> {0} ({1} subject(s))".format(SNAP_DIR, len(pts)))


def main():
    t0 = time.time()
    omni.timeline.get_timeline_interface().stop()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    pg.load_environment(ENV_URL)
    stage = omni.usd.get_context().get_stage()
    wait_for_stage(stage)
    for name in ("GroundPlane", "Environment"):
        p = stage.GetPrimAtPath("/World/" + name)
        if p and p.IsValid():
            p.SetActive(False)

    _, ssf = get_stage_meters_per_unit(stage)

    # THE PEOPLE VARIANT IS A `people.seed` OFFSET AND NOTHING ELSE. Passing it
    # through `seed` would re-draw the layout, the fire and the damage — five
    # different scenes rather than one scene with five casts.
    # SPEC OVERRIDES ARE APPLIED PRE-COMPILE, so changing `severity` really
    # recompiles the fire rather than leaving a compiled block behind the
    # launcher then has to ignore.
    overrides = {}
    if LAYOUT_SEED:
        overrides["seed"] = int(LAYOUT_SEED)
    if SEVERITY:
        overrides["severity"] = float(SEVERITY)
    # WHERE THE FIRE STARTED AND WHICH WAY THE WIND PUSHED IT. Two levels that
    # share these read as the same event however their damage tallies differ —
    # the burn scar lands in the same place and the aerial view is the same
    # picture twice. Moving them is the cheapest way to make one level look
    # like a different day.
    if EPICENTER:
        overrides["epicenter"] = [float(v) for v in EPICENTER.split(",")]
    if HEADING:
        overrides["heading_deg"] = float(HEADING)
    overrides = overrides or None
    if overrides:
        print("[freeze] spec overrides: {0}".format(overrides))
    if VARIANT:
        from compile_disaster import load_scene_config
        # `load_scene_config` takes the overrides itself; `build_scene` IGNORES
        # `spec_overrides` once the config is a dict, so they must be applied
        # here or a people variant would silently lose the level.
        cfg = load_scene_config(SCENE_CONFIG, spec_overrides=overrides)
        base = int((cfg.get("people") or {}).get("seed", 91))
        cfg = dict(cfg)
        cfg["people"] = dict(cfg.get("people") or {})
        cfg["people"]["seed"] = base + 1000 * VARIANT
        # `build_scene` names a dict config `config["name"]`; without this
        # every variant's ground truth records the scene as "<dict>".
        cfg.setdefault("name", SCENE_CONFIG)
        scene_cfg = cfg
        print("[freeze] people variant {0}: people.seed {1} -> {2}"
              .format(VARIANT, base, cfg["people"]["seed"]))
    else:
        scene_cfg = SCENE_CONFIG

    info = {}
    st = build_scene(stage, scene_cfg, ssf, arch_dir=ARCH_DIR, seed=SEED,
                     burn_frac=BURN_FRAC, elapsed=ELAPSED, poles=False,
                     parent_path=PARENT, people_json=PEOPLE_JSON,
                     info_out=info, spec_overrides=overrides)

    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    for _ in range(30):
        omni.kit.app.get_app().update()

    disaster = DISASTER or _disaster_kind(info.get("config") or {})

    recs = gt_hints.build(stage, info, ssf, disaster=disaster)
    reg = st["region"]
    gt_hints.write(HINTS_JSON, recs, meta={
        "scene_config": st["scene_config"], "seed": SEED,
        "people_variant": VARIANT, "disaster": disaster,
        "region_m": [float(v) for v in reg],
        "elapsed_s": st.get("elapsed_s"), "burn_frac": BURN_FRAC,
        "fire_origin_m": st.get("fire_origin_m"),
        "fire_heading_deg": st.get("fire_heading_deg"),
        "arch_dir": st.get("arch_dir"),
        "units": "metres, world frame, plate centred on the origin",
    })

    print("\n" + "=" * 72)
    print("FROZEN SCENE  {0}  ({1:.0f} x {2:.0f} m)".format(
        st["scene_config"], reg[2] - reg[0], reg[3] - reg[1]))
    print("  out         {0}".format(OUT))
    print("  total       {0:.0f} s".format(time.time() - t0))
    print("  houses      {0} referenced ({1} missing); {2}".format(
        st["houses"], st["houses_missing"],
        ", ".join("%s=%d" % kv for kv in sorted(st["house_tally"].items()))))
    print("  trees       {0} referenced ({1} missing); {2}".format(
        st["trees"], st["trees_missing"],
        ", ".join("%s=%d" % kv for kv in sorted(st["tree_tally"].items()))))
    print("  people      {0} alive, {1} car(s), {2} blocker(s)".format(
        st["people_alive"], st["cars"], st["blockers"]))
    for _name in ppl.SCENARIOS:
        if st["people_tally"].get(_name):
            print("    {0:<18} {1:>3}".format(_name, st["people_tally"][_name]))
    print("  GT_people   {0}".format(PEOPLE_JSON))
    print("  GT_hints    {0}".format(HINTS_JSON))
    for k, v in gt_hints.summarise(recs).items():
        print("    {0:<20} {1:>6}".format(k, v))
    print("=" * 72 + "\n")

    with open(os.path.join(OUT, "build_stats.json"), "w") as fh:
        json.dump({k: v for k, v in st.items()
                   if isinstance(v, (int, float, str, list, dict, type(None)))},
                  fh, indent=1)

    if SNAPS:
        try:
            review_snaps(stage, ssf, info, st)
        except Exception as _exc:
            print("[freeze] snapshots FAILED: {0}".format(_exc))

    # THE EXPORT RUNS LAST, after the ground truth and the review captures.
    # Every one of those is cheap and the export is not, so an export that
    # fails must not cost the labels — and a scene whose GT is already on disk
    # can be re-frozen from the same seeds without rebuilding anything else.
    if EXPORT:
        from disaster import freeze
        name = NAME or _default_name(OUT)
        try:
            finfo = freeze.export_scene(
                OUT, name,
                collect=env("FREEZE_COLLECT", "0").lower()
                in ("1", "true", "yes"))
            freeze.report(finfo)
            with open(os.path.join(OUT, "freeze_report.json"), "w") as fh:
                json.dump(finfo, fh, indent=1)
        except Exception as _exc:
            import traceback
            print("[freeze] EXPORT FAILED: {0}".format(_exc))
            traceback.print_exc()

    # BATCH MODE. The default is to hold the app open so the scene can be
    # looked at — which is right for a review run and wrong for a loop over
    # people variants, where the shell's `for` never reaches the second
    # iteration because the first never returns.
    if env("FREEZE_EXIT", "0").lower() in ("1", "true", "yes"):
        print("[freeze] FREEZE_EXIT set — closing after the export")
        simulation_app.close()
        return

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
