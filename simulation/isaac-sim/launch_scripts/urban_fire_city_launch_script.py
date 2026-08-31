#!/usr/bin/env python
"""
urban_fire_city — the 500 m downtown, built intact, with its BURNT buildings
swapped in from the per-building fire bakes and the flames and smoke put back
on top. Work item #7 of `scene_gen/_plans/urban_fire_city_plan.md`.

    ISAAC_SIM_HEADLESS=true SCENE_CONFIG=downtown_fire_500 \\
    FC_MANIFEST=/isaac-sim/AirStack/scene_gen/_plans/fire_city_4417.json \\
    FC_BAKES=/isaac-sim/.cache/fire_bakes/city_4417 \\
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/fire_city \\
    PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \\
    /isaac-sim/python.sh \\
    /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py \\
    --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window

`downtown_quake_launch_script.py` for the city half (Pegasus environment ->
`generate_scene_on_stage` -> post-process), `fire_assembly_launch_script.py`
for the fire half (`place_fire` on a bake, the fire-facing review cameras,
`vram_mb` at every stage) — the shared half of the latter now lives in
`scene_gen/disaster/fire_assembly_lib.py`, because THIS file cannot import
that launcher: it builds a `SimulationApp` at import and a second Kit app in
one process is a segfault inside the first second (measured, and documented
in `downtown_quake_launch_script.py`).

THE TRANSFORM TRAP (plan sec 4a) — the one thing to get right here
------------------------------------------------------------------
`quake.assemble`'s idiom is "keep the placement's transform, swap the
reference" (`quake.py:~676`: `refs.ClearReferences(); refs.AddReference(...)`).
That is WRONG for a fire bake, and it is wrong silently.
`scene_generator.apply_placements` authors, for the INTACT asset:

    translate = (x_m - rotated_centroid_offset) * ssf
    rotateXYZ = (roll, pitch, yaw)        # roll 90 deg for a Y-up asset
    scale     = p["scale"]                # 0.01 for a GAC asset

— every part of which is about that asset's own units, up-axis and pivot. A
BAKE is none of those things: `gac_fire.place_source` has already applied the
pack scale and re-centred it, the kit path builds at (0,0,0), and the export
is metres, Z-up, base at z = 0. Inheriting the cell's transform would shrink
it 100x and lay it on its side.

So this launcher HIDES the intact prim and authors a FRESH holder:

    /World/fire/<stem>          translate (x, y, z) * ssf
                                rotateXYZ (0, 0, yaw_deg)      <- the
                                    placement's FINAL yaw, which already
                                    includes the per-asset yaw-offset
                                scale     1  (x ssf: a bake is in metres)
    /World/fire/<stem>/bake     the reference

with the reference onto a CHILD, never onto the Xform that carries the
transform — the same rule `gac_fire.place_source`, `kit_bake.load_kit` and
the row assembler all follow, so a referenced asset's own transform can never
compose with (or clobber) the one authored here.

Proved offline, on a real bake, by `scene_gen/tools/fc_transform_probe.py`:
the composed centre is `R(yaw) . own_centre + (x, y)` to 0.0000 m, the base z
is unchanged, and at yaw 90 the footprint's W and D exchange to 0.0000 m.
That probe also found the one thing the plan got wrong: the plan asks for the
composed BBOX centre to land on the cell, and it does not — a burnt bake's
bbox includes the DEBRIS it dropped into the street (measured: +1.64 m of
spall east of a building whose fire is on its east elevation), while its
`masses["main"]` centre is (0.000, 0.000). The SHELL is what is plan-centred
and the shell is what has to land on the cell, so nothing is re-centred here.

THE EMITTER BUDGET (plan sec 4b) — the bug that reports success
---------------------------------------------------------------
Flow's block pool is finite and shared. Past it, Flow logs "Out of GPU memory
allocating resource 'flow'" / "Maximum Flow blocks ... in use" and every
emitter after the first few gets NO VOXELS — a city that renders with no
smoke while every count in the log looks right. A row of 6 buildings could
ignore this; 12-20 damaged buildings at the row's own per-building budget is
150-350 emitters, so this launcher spends a GLOBAL budget
(`FA_EMITTER_BUDGET`, default 200):

  * buildings are RANKED by state — flame (F2/F3, actively burning) first,
    then smoulder (F4/F5/F6), then the F1 wisps and anything residual;
  * every accepted building gets one opening's worth first, in rank order,
    so the worst buildings are never the ones dropped;
  * the remainder is handed out in rounds, again in rank order, up to
    `FA_EMITTERS` openings each;
  * a building that cannot fit even its minimum carries GEOMETRY AND BAKED
    SOOT ONLY, and says so — the plan's "the rest carry soot and geometry
    only". Its damage is still there; only its live fire is not.

FIRST FIT BY RANK, not strict rank order: once a building is refused the
allocator keeps going down the list, so a cheap F1 wisp can still take the
change a 14-emitter F4 could not fit into. That spends the budget instead of
stranding it, and it is why the printed table can show a smouldering building
dropped beside a wisp that was kept — read the `note` column, which carries
the exact figure that did not fit.

The predicted count is printed per building beside the ACTUAL count
`place_fire` returns, and the Kit log is grepped for the two Flow OOM strings
after the captures. Do not trust a capture until that grep is clean.

A TALL BUILDING HAS AN OPENING FLOOR. `place_fire` widens the budget for a
tall building on purpose (`max_open = max(max_emitters, min(16, n_st // 2))`
from 12 storeys up — "the taller buildings need more fire/flames, looks weird
otherwise"). That floor cannot be allocated below, so the allocator uses the
same formula to price a building and, if even the floor does not fit, drops
the whole building rather than pretending a smaller allocation would help.

Env:
    SCENE_CONFIG   preset (default `downtown_fire_500`). NOTE: that preset
                   carries `disaster-type: fire`, which `compile_disaster.
                   DISASTERS` has no entry for — it is a spec vocabulary for
                   the spread solver, not a compiled field — so it is
                   compiled with `disaster-type` overridden to `none`, the
                   same override `tools/fire_city_dry_run.py` uses, and the
                   raw preset is read again for the wind.
    FC_MANIFEST    `_plans/fire_city_<seed>.json` — `urban_fire_city.
                   damaged_manifest` records. Default: the newest
                   `scene_gen/_plans/fire_city_*.json`.
    FC_BAKES       bake directory or comma list of `.usd`
                   (default `/isaac-sim/.cache/fire_bakes/city_<seed>/`,
                   exactly where `tools/fire_city_bake.sh` puts them and
                   prints as its `FA_BAKES` line)
    FC_INTACT_ONLY 1 builds the city and STOPS — the "measure the intact
                   city first" step of plan sec 4c, the only unknown in the
                   VRAM projection. No manifest is needed for it. In this
                   mode the launcher ALSO writes the placements dump (see
                   FC_DUMP) — the whole reason FC_INTACT_ONLY exists as a
                   separate step is that it is the one place this pipeline
                   has both the REAL Nucleus-measured footprints and no
                   manifest yet to get wrong.
    FC_DUMP        path to write the city-placements dump to, in
                   FC_INTACT_ONLY mode only (default `scene_gen/_plans/
                   city_placements_<preset>_<seed>.json`). Every house
                   placement's cell/usd/x_m/y_m/z_m/yaw_deg/scale/category
                   plus its MEASURED W/D/H (a fresh `SizeResolver`, same
                   class `generate_scene_on_stage` used, so the numbers are
                   real Nucleus footprints, not the dry run's host-side
                   GAC/DTC substitutes) and the typology block map
                   (`config["_city_layout"]["_typology_of"]`) — everything
                   `tools/fire_city_dry_run.py --placements-json` needs to
                   solve a fire on the EXACT layout Kit just built, instead
                   of reconstructing one host-side that can pack
                   differently and describe a city that does not exist.
    FC_ENV         `default` (the historical behavior: `PegasusInterface().
                   load_environment(ENV_URL)`, the same "Default
                   Environment" every other launcher in this repo loads),
                   `none` (skip `load_environment` entirely), or a URL to
                   load instead. `load_environment(ENV_URL)` measured at
                   ~9.5 GiB of the ~12.0 GiB "empty stage" baseline this
                   launcher prints — see this file's own note above
                   `FireCityApp.__init__` for what it actually buys (spoiler:
                   `_remove_env_clutter` deactivates its GroundPlane/
                   Environment children a few lines later, and the city
                   authors its own ground and sky regardless).
    FC_HIDE        `invisible` (default, `MakeInvisible`) or `deactivate`
                   (`SetActive(False)`). An invisible prim still composes and
                   still costs memory; a deactivated one does not, but its
                   subtree can no longer be inspected in the viewport.
    FA_FLOW        1 (default) authors the Flow stack and the emitters
    FA_SMOKE       1 (default) — 0 gives flames only, no plumes
    FA_CELL_M      Flow density cell size, m (default 0.3 — the row's own
                   low-fidelity grid)
    FA_MAX_BLOCKS  the Flow block POOL (default 6144). `rtx/flow/maxBlocks`
                   is a carb setting, not the USD attribute.
    FA_EMITTERS    per-building opening budget before the global budget
                   trims it (default 6)
    FA_EMITTER_BUDGET  the GLOBAL emitter cap (default 200)
    FA_SCALE       emission scale multiplier (default 1.0)
    FA_SEED        rng seed for the per-emitter jitter (default 7)
    FC_SCORCH_VEG  1 (default) darkens street trees / greenery within reach
                   of a damaged building (F2 up — see `fire_assembly_lib.
                   veg_scorch_radius_m`) to a flat charred material. 0 skips
                   the whole pass; nothing about a tree changes, it stays
                   whatever the city generator authored (green).
    FC_FIRE_APRON  1 (default) scatters a small ground-debris apron along
                   the VENTING sides of every F1-F5 (non-collapse) burning
                   building — see `fire_assembly_lib.fire_apron_pass`. F5c/
                   F6 already drop a collapse heap and are not touched by
                   this pass. 0 skips it.
    SNAP_DIR       viewport captures, MUST be under
                   /isaac-sim/.nvidia-omniverse/logs/
    KEEP_OPEN      1 keeps the app up after the captures
    ISAAC_SIM_HEADLESS  true for a headless run

Banner: `URBAN FIRE CITY DONE`.
"""

import glob as _glob
import json
import math
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    """The container exports every launcher knob as an EMPTY STRING, so
    `os.environ.get(name, default)` never reaches its default. Treat empty as
    absent."""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


def _flag(name, default="0"):
    return _env(name, default).lower() in ("1", "true", "yes")


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
# FRACTIONAL CUTOUT OPACITY — `extra_args`, NOT `carb.settings`, and BOTH
# forms are required (the startup flag does not survive stage composition;
# the carb form alone is too late for startup). The bakes carry
# `soot_plume`'s baked atlases, `urban_fire._glass_pane`'s smoke deposits and
# the void tone, all fractional-cutout, which RTX discards unless this is on
# — the staining then renders as a hard binary stamp.
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

simulation_app = SimulationApp(launch_config={"headless": _HEADLESS,
                                              "extra_args": KIT_ARGS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
enable_extension("omni.flowusd")

import carb                                                    # noqa: E402
import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, UsdGeom                               # noqa: E402
from omni.isaac.core.world import World                        # noqa: E402
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS   # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import (  # noqa: E402
    PegasusInterface)

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

from scene_prep import (add_sky, get_stage_meters_per_unit,     # noqa: E402
                        scale_stage_prim, settle_rigid_props)
from scene_generator import resolve_sky, _make_resolver         # noqa: E402
from generate_scene import generate_scene_on_stage              # noqa: E402
from compile_disaster import load_scene_config, resolve_config_path  # noqa: E402
from disaster import fire as fx                                 # noqa: E402
from disaster import fire_assembly_lib as fal                   # noqa: E402
from disaster import fire_bake as fb                            # noqa: E402
from disaster import soot_plume as spl                          # noqa: E402
from disaster import urban_fire as uf                           # noqa: E402
from disaster import urban_fire_city as ufc                     # noqa: E402



def _load_by_path(name, path):
    """Import a module from a FILE, without putting its directory on
    `sys.path` — the idiom this repo already uses for `utils/snapshots.py`.
    `scene_gen/tools` is not a package, and prepending it to `sys.path` would
    put every helper script in there ahead of the standard library for the
    rest of the process."""
    import importlib.util as _ilu
    spec = _ilu.spec_from_file_location(name, path)
    mod = _ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# `fire_city_manifest` is the ONE place a manifest record becomes an entry
# string and a cache stem — `fire_city_bake.sh` classifies with the same
# module — so a bake this launcher looks for and a bake that driver produced
# can never disagree about the name.
fcm = _load_by_path("fire_city_manifest",
                    os.path.join(_SCENE_GEN_DIR, "tools",
                                 "fire_city_manifest.py"))

ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
PARENT = "/World/stage/generated"
FIRE_ROOT = "/World/fire"
_ENV_CLUTTER = {"GroundPlane", "Environment"}

#: the two strings Flow logs when its block pool is exhausted. It keeps
#: reporting success while every emitter past the first few gets no voxels,
#: so this is the only way to know a capture is worth looking at.
FLOW_OOM_NEEDLES = ("Out of GPU memory allocating resource 'flow'",
                    "Maximum Flow blocks")
KIT_LOG_GLOBS = ("/isaac-sim/kit/logs/Kit/Isaac-Sim Python/*/*.log",
                 "/isaac-sim/kit/logs/Kit/*/*/*.log")

SCENE_CONFIG = _env("SCENE_CONFIG", "downtown_fire_500")
MANIFEST = _env("FC_MANIFEST", "")
BAKES = _env("FC_BAKES", "")
INTACT_ONLY = _flag("FC_INTACT_ONLY", "0")
DUMP_PATH = _env("FC_DUMP", "")
FC_ENV = _env("FC_ENV", "default")
HIDE_MODE = _env("FC_HIDE", "invisible").lower()
FLOW = _env("FA_FLOW", "1") not in ("0", "false", "no")
SMOKE = _env("FA_SMOKE", "1") not in ("0", "false", "no")
CELL_M = float(_env("FA_CELL_M", "0.3"))
MAX_BLOCKS = int(_env("FA_MAX_BLOCKS", "6144"))
MAX_EMITTERS = int(_env("FA_EMITTERS", "6"))
EMITTER_BUDGET = int(_env("FA_EMITTER_BUDGET", "200"))
SCALE = float(_env("FA_SCALE", "1.0"))
SEED = int(_env("FA_SEED", "7"))
SCORCH_VEG = _flag("FC_SCORCH_VEG", "1")
FIRE_APRON = _flag("FC_FIRE_APRON", "1")
# THE PEOPLE PASS RUNS LAST ("after everything is baked in", user
# 2026-08-31): FC_PEOPLE_JSON is the REVIEWED records file the fire_people
# dry run wrote (its PNG is the 2-D gate) — the launcher never re-solves
# placement, it authors what was approved.
PEOPLE = _flag("FC_PEOPLE", "1")
PEOPLE_JSON = _env("FC_PEOPLE_JSON", "")
SNAP_DIR = _env("SNAP_DIR", "")
KEEP_OPEN = _flag("KEEP_OPEN", "0")


def vram_mb(tag):
    """`fire_assembly_lib.vram_mb`, printing under this launcher's prefix."""
    return fal.vram_mb(tag, prefix="fc")


# ---------------------------------------------------------------------------
# The city (copied from `downtown_quake_launch_script`, which copied it from
# `scene_launch_script` — NOT imported, for the second-SimulationApp reason)
# ---------------------------------------------------------------------------
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
    print("[fc] env clutter: {0} prim(s) deactivated".format(n))


def _disable_sky_sun(stage):
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "DistantLight":
            continue
        if "/Environment/sky/" not in str(prim.GetPath()):
            continue
        if prim.IsActive() and prim.SetActive(False):
            n += 1
    print("[fc] sky sun: {0} DistantLight(s) disabled".format(n))


def _wait_for_stage(stage, timeout_s=10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            if [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]:
                return True
        time.sleep(0.1)
    return False


def preset_fire_block(name):
    """`epicenter` / `heading_deg` / `wind_mps` / `duration_s` /
    `start_offset_frac` read off the RAW preset yaml.

    They cannot come through `load_scene_config`: `compile_disaster.DISASTERS`
    has no `"fire"` entry (only `"wildfire"`, a continuous-fuel-bed ellipse
    over vegetation — the wrong model for discrete buildings separated by
    streets), so `compile_spec` would RAISE on `disaster-type: fire`. The
    preset says so in its own header; the dry run reads them the same way.
    Only `heading_deg` is used here — it aims the downwind "wave" capture —
    but all five are printed, because a capture that does not look like the
    wind in the manifest is a manifest/preset mismatch, not a camera bug.
    """
    import yaml
    out = {"epicenter": [0.0, 0.0], "heading_deg": 45.0, "wind_mps": 4.0,
           "duration_s": None, "start_offset_frac": None}
    try:
        with open(resolve_config_path(name)) as fh:
            raw = yaml.safe_load(fh) or {}
    except Exception as exc:
        print("[fc] could not read the raw preset for the wind ({0}) — "
              "defaulting to heading {1} deg".format(exc, out["heading_deg"]))
        return out
    for k in list(out):
        if raw.get(k) is not None:
            out[k] = raw[k]
    return out


# ---------------------------------------------------------------------------
# FC_DUMP — the placements Kit actually built, for the dry run to solve on
# instead of a host-side reconstruction that can pack differently (2026-08-30
# incident: the dry run's patched SizeResolver substitutes GAC/DTC footprints
# because Nucleus is not mirrored locally, so its packer makes different
# placement decisions than Kit's real one — the manifest it produced named
# cells that did not exist in the city this launcher actually built).
# ---------------------------------------------------------------------------
def default_dump_path(preset, seed):
    return os.path.join(_SCENE_GEN_DIR, "_plans",
                        "city_placements_{0}_{1}.json".format(preset, seed))


def _typology_rects(layout):
    """`[{"rect": [x0, y0, x1, y1], "name": name}, ...]` — JSON has no tuple
    keys, so `layout["_typology_of"]` (`{(x0,y0,x1,y1): name}`, written by
    `districts.rezone_blocks` and stashed onto `config["_city_layout"]` by
    `generate_scene_on_stage`) is serialised as a list, in the same
    iteration order `urban_fire_city.typology_at` itself walks."""
    return [{"rect": [float(v) for v in rect], "name": name}
            for rect, name in (layout or {}).get("_typology_of", {}).items()]


def dump_city_placements(path, preset, seed, config, placements, layout):
    """Write the city-placements dump `tools/fire_city_dry_run.py
    --placements-json` consumes — see the block comment above and this
    module's own docstring (FC_DUMP) for why the manifest must be solved on
    the SAME layout Kit built, not a host-side reconstruction.

    Only `category == "house"` placements are written: `urban_fire_city.
    burnable()`'s gate 1 refuses everything else outright, and a real
    downtown has one to two orders of magnitude more non-house placements
    (trees, benches, cars, streetlights, ...) that would bloat this file
    for no downstream use.

    EVERY HOUSE RECORD CARRIES ITS OWN `i` — its index in the FULL
    `placements` list (every category, not just houses) THIS PROCESS just
    built. `urban_fire_city_launch_script.resolve_cell`'s most-trusted route
    (route 1: match the record's `i` against the placement at that index,
    verified by usd + distance) needs `i` to still be the right index when
    the FULL launcher rebuilds the same (deterministic, same preset/seed)
    city later — a manifest built from indices renumbered relative to a
    house-only list would silently point route 1 at the wrong placement
    every time a non-house placement came before it. `n_placements_total`
    (the length of the full list) travels alongside so the dry run can
    rebuild a same-length list with non-house placeholders at every index
    it never saw, preserving exactly this alignment.

    W/D/H are MEASURED HERE, with a FRESH `SizeResolver` built the same way
    `generate_scene_on_stage`'s own internal one is
    (`scene_generator._make_resolver`) — same asset_scale / fallback_sizes /
    measure_usds, same `"house"` category, same per-placement scale/axis_up
    — so re-measuring in THIS process (which, unlike the host-side dry run,
    has live Nucleus access) reproduces the exact numbers `apply_placements`
    used for packing, rather than the dry run's patched GAC/DTC cache.
    """
    resolver = _make_resolver(config)
    houses = []
    for i, p in enumerate(placements):
        if p.get("category") != "house":
            continue
        usd = p.get("usd")
        fp = resolver.get(usd, "house", scale=float(p.get("scale", 1.0)),
                          axis_up=p.get("axis_up", "Z"))
        houses.append({
            "i": i, "cell": p.get("prim_path"), "usd": usd,
            "x_m": float(p.get("x_m", 0.0)), "y_m": float(p.get("y_m", 0.0)),
            "z_m": float(p.get("z_m", 0.0)),
            "yaw_deg": float(p.get("yaw_deg", 0.0)),
            "scale": float(p.get("scale", 1.0)),
            "category": p.get("category"),
            "axis_up": p.get("axis_up", "Z"),
            "W": float(fp["sx"]), "D": float(fp["sy"]), "H": float(fp["sz"]),
        })
    doc = {
        "schema": "fire_city_placements_dump.v1",
        "preset": preset, "seed": int(seed),
        "region_m": [float(v) for v in
                    config.get("layout", {}).get("region_m", [0.0, 0.0])],
        "n_placements_total": len(placements),
        "placements": houses,
        "typology": {"blocks": _typology_rects(layout)},
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        json.dump(doc, fh, indent=1)
    print("[fc] wrote city placements dump ({0} house placement(s) of {1} "
          "total, {2} typology block(s)) -> {3}".format(
              len(houses), len(placements),
              len(doc["typology"]["blocks"]), path))
    return path


# ---------------------------------------------------------------------------
# The manifest and its bakes
# ---------------------------------------------------------------------------
def find_manifest(spec):
    """`FC_MANIFEST`, or the newest `_plans/fire_city_*.json`."""
    if spec:
        return spec
    pats = sorted(_glob.glob(os.path.join(_SCENE_GEN_DIR, "_plans",
                                          "fire_city_*.json")),
                  key=lambda p: os.path.getmtime(p), reverse=True)
    return pats[0] if pats else ""


def bake_paths(records, seed, spec):
    """One `(record, usd, json, stem)` per manifest record, in manifest order.

    `stem` is `fire_bake.out_stem` of the record's entry — via
    `fire_city_manifest.build_entry_and_stem`, the same call
    `fire_city_bake.sh` classifies with, so a bake this launcher cannot find
    is genuinely not baked rather than merely named differently. `usd` is
    `""` when nothing on disk matches.
    """
    if spec:
        items = [q.strip() for q in str(spec).split(",") if q.strip()]
    else:
        items = [os.path.join(fb.DEFAULT_OUT_DIR, "city_{0}".format(seed))]
    dirs = [q for q in items if os.path.isdir(q)]
    by_stem = {}
    for q in items:
        if os.path.isdir(q):
            for u in sorted(_glob.glob(os.path.join(q, "*.usd"))):
                by_stem.setdefault(
                    os.path.splitext(os.path.basename(u))[0], u)
        elif q.lower().endswith(".usd"):
            by_stem.setdefault(os.path.splitext(os.path.basename(q))[0], q)
    print("[fc] bakes: {0} file(s) visible in {1}".format(
        len(by_stem), ", ".join(dirs) or spec or "(nothing)"))
    out = []
    for i, rec in enumerate(records):
        try:
            _entry, stem = fcm.build_entry_and_stem(rec, i)
        except Exception as exc:
            print("[fc] *** record {0} is not a valid bake entry: {1}"
                  .format(i, exc))
            out.append((rec, "", "", None))
            continue
        usd = by_stem.get(stem, "")
        js = (os.path.splitext(usd)[0] + ".json") if usd else ""
        out.append((rec, usd, js if (js and os.path.exists(js)) else "", stem))
    return out


def resolve_cell(stage, placements, rec):
    """`(prim_path, how)` for the INTACT city prim this bake replaces.

    THREE ROUTES, MOST TRUSTWORTHY FIRST, because `cell` is a path the DRY
    RUN computed in a different process: `apply_placements` names a prim
    `<parent>/<category>_<group>_<i>` where `group` is the first-appearance
    order of its USD, so ANY change to the placement list — a different
    generator revision, a re-seeded pass — renumbers every cell after it, and
    a stale `cell` would hide the wrong building silently.

      1. the record's `i` (its index into the same placement list this
         process just rebuilt), accepted only if the placement at `i` has the
         SAME usd and is within 0.5 m of the record's own (x, y);
      2. the record's `cell` path, if it is a valid prim on this stage;
      3. the nearest `house` placement with the same usd within 2 m.

    `(None, reason)` when all three fail — the caller then refuses to place
    the bake, because a burnt shell composed inside an intact one is worse
    than a missing fire: it z-fights, and it reads as a different bug.
    """
    i = rec.get("i")
    rx, ry = float(rec.get("x", 0.0)), float(rec.get("y", 0.0))
    if isinstance(i, int) and 0 <= i < len(placements):
        p = placements[i]
        if p.get("usd") == rec.get("usd"):
            d = math.hypot(float(p.get("x_m", 0.0)) - rx,
                           float(p.get("y_m", 0.0)) - ry)
            if d <= 0.5 and p.get("prim_path"):
                return p["prim_path"], "index i={0} (d={1:.2f} m)".format(i, d)
    cell = rec.get("cell")
    if cell:
        prim = stage.GetPrimAtPath(Sdf.Path(cell))
        if prim and prim.IsValid():
            return cell, "manifest cell path"
    best, best_d = None, 2.0
    for p in placements:
        if p.get("category") != "house" or p.get("usd") != rec.get("usd"):
            continue
        d = math.hypot(float(p.get("x_m", 0.0)) - rx,
                       float(p.get("y_m", 0.0)) - ry)
        if d < best_d and p.get("prim_path"):
            best, best_d = p["prim_path"], d
    if best:
        return best, "nearest same-asset placement (d={0:.2f} m)".format(best_d)
    return None, ("no placement matches i={0!r}, cell={1!r}, or a same-asset "
                  "house within 2 m of ({2:.1f}, {3:.1f}) — the layout this "
                  "launcher built is NOT the one the manifest was solved on"
                  .format(i, cell, rx, ry))


def hide_intact(stage, path):
    """`MakeInvisible` (default) or `SetActive(False)` on the intact prim.

    `UsdGeom.Imageable.MakeInvisible()` RETURNS NOTHING (it is void in C++),
    so `bool(...)` on it is always False and a naive caller reports every
    hide as a failure — or, worse, trusts a hide that did not take. The
    visibility attribute is read back instead.
    """
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return False
    if HIDE_MODE.startswith("deact"):
        return bool(prim.SetActive(False))
    img = UsdGeom.Imageable(prim)
    if not img:
        # a typeless holder whose reference composed nothing typed
        return bool(prim.SetActive(False))
    img.MakeInvisible()
    return img.GetVisibilityAttr().Get() == UsdGeom.Tokens.invisible


def place_holder(stage, stem, x, y, z, yaw_deg, ssf):
    """THE FRESH HOLDER — see this module's docstring. Returns its path."""
    holder = "{0}/{1}".format(FIRE_ROOT, stem)
    xf = UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x) * ssf, float(y) * ssf,
                                     float(z) * ssf))
    # THE PLACEMENT'S FINAL YAW, and only the yaw: the bake is already Z-up
    # and already in metres, so the roll/pitch `apply_placements` authors for
    # a Y-up intact asset must NOT be carried over.
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    # scale 1 IN METRES -> `ssf` in stage units (both are 1.0 on this stage;
    # authored as ssf so a stage with a different metersPerUnit is right too).
    xf.AddScaleOp().Set(Gf.Vec3f(float(ssf), float(ssf), float(ssf)))
    # REFERENCE ONTO A CHILD, not onto the Xform that carries the transform.
    kid = stage.DefinePrim(Sdf.Path(holder + "/bake"))
    return holder, kid


# ---------------------------------------------------------------------------
# The global emitter budget
# ---------------------------------------------------------------------------
#: rank 0 burns hardest and is served first.
STATE_RANK = {"flame": 0, "smoulder": 1}


def _live(ev):
    return all(not (o.get("e") or {}).get("dead") for o in (ev.get("ops") or []))


def fire_state(doc, events):
    """`(state, wisp_only)` exactly as `place_fire` decides it — including
    the F1 case, where the level has no ACTIVE state but `soot_plume` gave it
    one `smoulder` event and it becomes a wisp."""
    state = ((doc or {}).get("fire") or {}).get("state")
    if state:
        return state, False
    if any(ev.get("state") == "smoulder" and ev.get("ops")
           for ev in (events or [])):
        return "smoulder", True
    return None, False


def emitter_estimate(doc, events, max_emitters, smoke):
    """Predicted emitter count for `place_fire(..., max_emitters=...)`.

    Mirrors `fire_assembly_lib.place_fire`'s branch structure exactly,
    counting instead of authoring, so the allocator prices a building the
    way it will actually cost. An UPPER BOUND: `_flame_sources` can author
    fewer than `per_opening` prims if a source fails to create, which is why
    the predicted and the actual counts are both printed per building.
    """
    f = (doc or {}).get("fire") or {}
    state, wisp = fire_state(doc, events)
    zero = {"flame": 0, "smoke": 0, "interior": 0, "roof": 0, "openings": 0,
            "total": 0}
    if not state:
        return dict(zero)
    evs = [ev for ev in (events or []) if _live(ev) and ev.get("ops")]
    is_flame = state == "flame"
    n_st = int(f.get("n_storeys") or 0)
    max_open = (max(max_emitters, min(16, n_st // 2)) if n_st >= 12
                else max_emitters)
    n_flame = n_open = 0
    for ev in [e for e in evs if e["state"] == "flame"]:
        for _op in ev["ops"]:
            if n_open >= max_open:
                break
            n_flame += uf.FLAME_PER_OPENING
            n_open += 1
    if state == "smoulder" and not wisp:
        for ev in [e for e in evs if e["state"] == "smoulder"]:
            for _op in ev["ops"]:
                if n_open >= max(2, max_open // 2):
                    break
                n_flame += max(1, uf.FLAME_PER_OPENING - 1)
                n_open += 1
    if not smoke:
        return {"flame": n_flame, "smoke": 0, "interior": 0, "roof": 0,
                "openings": n_open, "total": n_flame}
    if wisp:
        n = len([e for e in evs if e["state"] == "smoulder"][:2])
        return {"flame": 0, "smoke": n, "interior": 0, "roof": 0,
                "openings": 0, "total": n}
    if is_flame:
        n_smoke = len([e for e in evs if e["state"] == "out"][:uf.SMOKE_EXTRA_MAX])
    else:
        n_smoke = len([e for e in evs
                       if e["state"] == "smoulder"][:spl.SMOULDER_EVENTS_MAX])
    seats = (doc or {}).get("seats") or {}
    n_int = 0 if is_flame else len(seats.get("interior") or [])
    n_roof = len(seats.get("roof") or []) if f.get("roof") else 0
    return {"flame": n_flame, "smoke": n_smoke, "interior": n_int,
            "roof": n_roof, "openings": n_open,
            "total": n_flame + n_smoke + n_int + n_roof}


def allocate_emitters(rows, budget, cap, smoke):
    """Spend `budget` emitters across `rows`, worst building first.

    Sets `row["alloc"]` (the `max_emitters` to pass `place_fire`, or `None`
    for "geometry and baked soot only") and `row["est"]`. Returns the
    predicted total. See this module's docstring for the rule; the ranking is
    state first (flame > smoulder > wisp/residual), then height, then
    manifest order, so the decision is deterministic.
    """
    live = [r for r in rows if r.get("doc")]
    for r in rows:
        if not r.get("doc"):
            r["alloc"] = None
            r["est"] = {"total": 0}
            r["drop_reason"] = "no sidecar — geometry only, no emitters"
    for r in live:
        st, wisp = fire_state(r["doc"], r["events"])
        r["state"] = ("wisp" if wisp else st) or "none"
        r["rank"] = 2 if (wisp or not st) else STATE_RANK.get(st, 2)
        r["n_st"] = int((r["doc"].get("fire") or {}).get("n_storeys") or 0)
        r["alloc"] = None
        # zeros, NOT `emitter_estimate(..., 0, ...)`: a tall building's
        # opening FLOOR is non-zero even at max_emitters=0, and printing it
        # against a building that was dropped would claim emitters that were
        # never authored.
        r["est"] = {"flame": 0, "smoke": 0, "interior": 0, "roof": 0,
                    "openings": 0, "total": 0}
    order = sorted(live, key=lambda r: (r["rank"], -r["n_st"], r["i"]))
    spent = 0
    # 1) one opening each, in rank order — the floor a tall building cannot
    #    go below is priced here too, so a building that does not fit is
    #    dropped whole rather than quietly given a fire nobody can see.
    for r in order:
        est = emitter_estimate(r["doc"], r["events"], 1, smoke)
        if est["total"] == 0:
            r["alloc"], r["est"] = 1, est          # nothing to spend anyway
            continue
        if spent + est["total"] <= budget:
            r["alloc"], r["est"] = 1, est
            spent += est["total"]
        else:
            r["drop_reason"] = ("global emitter budget spent ({0}/{1}) before "
                                "this building's minimum {2}"
                                .format(spent, budget, est["total"]))
    # 2) top up, one opening at a time, again in rank order
    for step in range(2, max(2, cap) + 1):
        for r in order:
            if r["alloc"] is None:
                continue
            est = emitter_estimate(r["doc"], r["events"], step, smoke)
            delta = est["total"] - r["est"]["total"]
            if delta <= 0:
                r["alloc"] = step                  # free: no more openings
                continue
            if spent + delta <= budget:
                r["alloc"], r["est"] = step, est
                spent += delta
    return spent


# ---------------------------------------------------------------------------
# The Kit log — Flow fails SILENTLY
# ---------------------------------------------------------------------------
def grep_kit_log(tail_bytes=8 * 1024 * 1024):
    """`(path, {needle: count})` for the newest Kit log, or `(None, {})`.

    Flow keeps reporting success when its block pool is exhausted, so a
    capture is only worth looking at once this comes back clean. Only the
    tail is read — a Kit log runs to hundreds of MB.
    """
    cands = []
    for pat in KIT_LOG_GLOBS:
        for p in _glob.glob(pat):
            try:
                cands.append((os.path.getmtime(p), p))
            except OSError:
                pass
    if not cands:
        return None, {}
    cands.sort(reverse=True)
    path = cands[0][1]
    hits = {n: 0 for n in FLOW_OOM_NEEDLES}
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as fh:
            if size > tail_bytes:
                fh.seek(size - tail_bytes)
            blob = fh.read().decode("utf-8", "replace")
        for n in FLOW_OOM_NEEDLES:
            hits[n] = blob.count(n)
    except Exception as exc:
        print("[fc] could not read the Kit log {0}: {1}".format(path, exc))
    return path, hits


# ---------------------------------------------------------------------------
class FireCityApp:
    def __init__(self):
        self.t0 = time.time()
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()
        self.vram = {}
        self.rows = []
        self.n_hidden = 0

        pg = PegasusInterface()
        pg._world = World(**pg._world_settings)
        env_mode = FC_ENV.lower()
        loaded_env_url = None
        if env_mode in ("none", "0", "off", "skip"):
            print("[fc] FC_ENV=none — skipping PegasusInterface()."
                  "load_environment(); the city authors its own ground "
                  "(scene_generator.apply_ground_planes, under PARENT + "
                  "'/ground') and sky (add_sky), so nothing this launcher "
                  "runs downstream should need the Pegasus environment")
        else:
            loaded_env_url = ENV_URL if env_mode in ("", "default") else FC_ENV
            pg.load_environment(loaded_env_url)
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if loaded_env_url is not None and not _wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")
        self.stage = stage
        _remove_env_clutter(stage)
        if stage.GetPrimAtPath("/World/stage").IsValid():
            scale_stage_prim(stage, "/World/stage", 1.0)
            for _ in range(10):
                omni.kit.app.get_app().update()
        _, self.ssf = get_stage_meters_per_unit(stage)
        if abs(self.ssf - 1.0) > 1e-9:
            print("[fc] NOTE: this stage is {0} units/m — a bake is authored "
                  "in METRES, so the holders carry scale {0} rather than the "
                  "plan's literal 1.0".format(self.ssf))
        if loaded_env_url is None:
            env_tag = "empty stage (no Pegasus env, FC_ENV=none)"
        elif loaded_env_url == ENV_URL:
            env_tag = "empty stage (Pegasus env loaded)"
        else:
            env_tag = "empty stage (Pegasus env {0} loaded)".format(loaded_env_url)
        self.vram["empty"] = vram_mb(env_tag)

        # -- 1) the city, exactly as `scene_launch_script` builds it --------
        # `disaster-type: fire` is spec vocabulary for the spread solver, not
        # a `compile_disaster.DISASTERS` entry, so it is compiled as `none`.
        # Legitimate because `build_city`/`generate_scene_on_stage` never read
        # `config["disaster"]` for placement — the damage comes from the
        # BAKES, not from a compiled field.
        print("[fc] compiling {0} with disaster-type overridden to 'none' "
              "(the fire is the manifest, not a compiled field)"
              .format(SCENE_CONFIG))
        config = load_scene_config(SCENE_CONFIG,
                                   spec_overrides={"disaster-type": "none"})
        self.config = config
        self.wind = preset_fire_block(SCENE_CONFIG)
        print("[fc] wind: heading {0} deg (blowing TOWARD), {1} m/s; "
              "epicenter {2}; duration {3} s x start_offset {4}".format(
                  self.wind["heading_deg"], self.wind["wind_mps"],
                  self.wind["epicenter"], self.wind["duration_s"],
                  self.wind["start_offset_frac"]))
        t_layout = time.time()
        self.placements = generate_scene_on_stage(
            stage, config, parent_path=PARENT, scene_scale_factor=self.ssf)
        for _ in range(10):
            omni.kit.app.get_app().update()
        settle_rigid_props(
            stage,
            [p["prim_path"] for p in self.placements
             if p.get("settle") and p.get("prim_path")],
            ground_path=PARENT + "/ground")
        add_sky(stage, resolve_sky(config),
                intensity=float(config.get("sky_intensity", 3500.0)),
                exposure=float(config.get("sky_exposure", -3.0)))
        _disable_sky_sun(stage)
        for _ in range(10):
            omni.kit.app.get_app().update()
        self.t_layout = time.time() - t_layout
        self.region = [float(v) for v in config["layout"]["region_m"]]
        self.n_houses = sum(1 for p in self.placements
                            if p.get("category") == "house")
        print("[fc] city: {0:.0f} x {1:.0f} m, {2} placement(s), {3} "
              "building(s), layout in {4:.0f} s".format(
                  self.region[0], self.region[-1], len(self.placements),
                  self.n_houses, self.t_layout))
        if INTACT_ONLY:
            # THE FIX FOR THE 2026-08-30 MANIFEST/CITY MISMATCH: dump the
            # layout Kit just built (real Nucleus footprints, real packing
            # decisions) so `tools/fire_city_dry_run.py --placements-json`
            # solves the fire on THIS city, not a host-side reconstruction
            # that can pack differently and describe a city that never
            # existed. See FC_DUMP in this module's own docstring.
            dump_path = DUMP_PATH or default_dump_path(
                SCENE_CONFIG, config.get("seed", 0))
            dump_city_placements(dump_path, SCENE_CONFIG, config.get("seed", 0),
                                 config, self.placements,
                                 config.get("_city_layout") or {})
        self.vram["intact"] = vram_mb(
            "city built INTACT ({0} buildings)".format(self.n_houses))

    # -- 2/3) the manifest, the bakes, the swap ----------------------------
    def load_fire(self):
        path = find_manifest(MANIFEST)
        if not path or not os.path.exists(path):
            raise RuntimeError(
                "FC_MANIFEST={0!r} not found and no scene_gen/_plans/"
                "fire_city_*.json to fall back on — run tools/"
                "fire_city_dry_run.py first (plan work item #4)".format(MANIFEST))
        top_seed, records = fcm.load_manifest(path)
        seed = fcm.resolve_city_seed(path, top_seed)
        print("[fc] manifest {0}: {1} record(s), city seed {2}".format(
            path, len(records), seed))
        self.manifest_path, self.city_seed = path, seed

        # THE DISTRICT RULE, re-asserted at assembly time. The dry run proved
        # it; a hand-edited manifest can still break it, and a burning tower
        # is the one thing this scene is defined by NOT having.
        tally = {}
        offenders = []
        for i, rec in enumerate(records):
            t = rec.get("typology")
            tally[t] = tally.get(t, 0) + 1
            if t in ufc.NO_FIRE_TYPOLOGIES:
                offenders.append((i, t, rec.get("asset") or rec.get("style")))
        print("[fc] damaged by typology: {0}".format(
            ", ".join("{0}={1}".format(k, v) for k, v in sorted(
                tally.items(), key=lambda kv: str(kv[0])))))
        if offenders:
            print("[fc] *** DISTRICT RULE VIOLATED: {0} record(s) sit in a "
                  "no-fire district {1} — {2}".format(
                      len(offenders), ufc.NO_FIRE_TYPOLOGIES, offenders))

        rows = []
        missing = []
        for i, (rec, usd, js, stem) in enumerate(
                bake_paths(records, self.city_seed, BAKES)):
            if not usd:
                missing.append((i, stem, rec.get("asset") or rec.get("style"),
                                rec.get("level")))
                continue
            doc = masses = events = None
            if js:
                try:
                    doc, masses, events = fb.load_for_assembly(js)
                except Exception as exc:
                    print("[fc] sidecar {0} unreadable ({1}) — geometry only"
                          .format(js, exc))
            rows.append({"i": i, "rec": rec, "usd": usd, "json": js,
                         "stem": stem, "doc": doc or {},
                         "masses": masses or {}, "events": events or []})
        if missing:
            print("\n[fc] " + "!" * 70)
            print("[fc] {0} of {1} manifest record(s) HAVE NO BAKE — those "
                  "buildings stay INTACT in this scene:".format(
                      len(missing), len(records)))
            for i, stem, name, level in missing:
                print("[fc]    record {0:<3} {1:<24} {2:<5} -> no {3}.usd"
                      .format(i, name, level, stem))
            print("[fc]   run  scene_gen/tools/fire_city_bake.sh {0}"
                  .format(self.manifest_path))
            print("[fc] " + "!" * 70 + "\n")
        self.rows = rows
        self.missing = missing
        return rows

    def compose_bakes(self):
        """Hide each intact cell and reference its bake under a fresh holder."""
        stage = self.stage
        UsdGeom.Xform.Define(stage, Sdf.Path(FIRE_ROOT))
        placed, refused = [], []
        n_cell_matched = 0
        for r in self.rows:
            rec = r["rec"]
            cell, how = resolve_cell(stage, self.placements, rec)
            if not cell:
                print("[fc] *** record {0} ({1}): {2}".format(
                    r["i"], r["stem"], how))
                print("[fc]     refusing to place the bake — a burnt shell "
                      "inside an intact one z-fights and reads as a "
                      "different bug entirely")
                r["skip"] = "cell not found"
                refused.append(r)
                continue
            n_cell_matched += 1
            x = float(rec.get("x", 0.0))
            y = float(rec.get("y", 0.0))
            z = float(rec.get("z", 0.0))
            yaw = float(rec.get("yaw_deg", 0.0))
            # the bake's own record of the cell it replaces, if the bake
            # driver wrote one — a disagreement means the manifest and the
            # bake are from different solves.
            city = (r["doc"] or {}).get("city") or {}
            if city:
                dx = math.hypot(float(city.get("x", x)) - x,
                                float(city.get("y", y)) - y)
                dyaw = abs(float(city.get("yaw_deg", yaw)) - yaw)
                if dx > 0.5 or dyaw > 1.0:
                    print("[fc] WARNING {0}: the bake's own sidecar says it "
                          "belongs at ({1:.1f}, {2:.1f}) yaw {3:.0f}, the "
                          "manifest says ({4:.1f}, {5:.1f}) yaw {6:.0f} — "
                          "different solves?".format(
                              r["stem"], city.get("x"), city.get("y"),
                              city.get("yaw_deg", 0.0), x, y, yaw))
            if hide_intact(stage, cell):
                self.n_hidden += 1
            else:
                print("[fc] WARNING: could not hide {0}".format(cell))
            holder, kid = place_holder(stage, r["stem"], x, y, z, yaw, self.ssf)
            if not kid.GetReferences().AddReference(r["usd"]):
                print("[fc] *** FAILED to reference {0}".format(r["usd"]))
                r["skip"] = "reference failed"
                refused.append(r)
                continue
            stage.Load(Sdf.Path(holder + "/bake"))
            r["prim"], r["cell"], r["cell_how"] = holder, cell, how
            r["x"], r["y"], r["yaw"] = x, y, yaw
            r["bbox"] = fal.bbox(stage, holder)
            b = r["bbox"]
            print("[fc] d{0:<2} ({1:+7.1f},{2:+7.1f}) yaw {3:+4.0f}  {4:<30} "
                  "{5:<4} {6}  [cell: {7}]".format(
                      r["i"], x, y, yaw, r["stem"],
                      (r["doc"].get("level") or "?"),
                      ("bbox {0:.0f}x{1:.0f}x{2:.0f} m".format(
                          b[3] - b[0], b[4] - b[1], b[5] - b[2])
                       if b else "*** EMPTY BBOX ***"), how))
            if r["doc"].get("src_kept"):
                print("[fc]     NOTE: this bake still carries its merged "
                      "source subtree (src_kept) — it composes and costs "
                      "memory")
            placed.append(r)
            for _ in range(2):
                omni.kit.app.get_app().update()
        for _ in range(10):
            omni.kit.app.get_app().update()
        self.placed, self.refused = placed, refused
        print("[fc] {0} bake(s) composed, {1} refused, {2} intact prim(s) "
              "hidden ({3})".format(len(placed), len(refused), self.n_hidden,
                                    HIDE_MODE))
        # THE REGRESSION CHECK FOR THE 2026-08-30 INCIDENT: with a manifest
        # derived from `FC_DUMP`/`--placements-json`, Kit-to-Kit determinism
        # (same preset/seed -> same layout) means every record's cell should
        # resolve — a mismatch here means the manifest was NOT solved on
        # this city.
        print("[fc] manifest/city match: {0}/{1} record(s) found their "
              "placement cell in the layout this launcher just built"
              .format(n_cell_matched, len(self.rows)))
        self.vram["bakes"] = vram_mb(
            "{0} bake(s) composed, no Flow".format(len(placed)))

    # -- 3b) scorched vegetation near a burning building (2026-08-31) ------
    def scorch_vegetation(self):
        """Darken every street tree / greenery placement within reach of a
        damaged building — see `fire_assembly_lib.scorch_vegetation_pass`
        for the radius rule and the material reuse. Runs on `self.placed`
        (composed bakes only), independent of the Flow emitter budget: a
        building whose fire did not get live flames because the budget ran
        out still did real damage and still scorches what is next to it.
        """
        if not SCORCH_VEG or not self.placed:
            self.veg_scorch = {"trees": 0, "leaf_binds": 0,
                               "trunk_binds": 0, "fuels_total": 0,
                               "scorched": 0, "targets": {}}
            return
        stats = fal.scorch_vegetation_pass(self.stage, self.placements,
                                           self.placed, FIRE_ROOT)
        self.veg_scorch = stats
        print("[fc] scorched vegetation: {0}/{1} fuel placement(s) within "
              "reach of a burning building ({2} tree(s) touched, {3} leaf "
              "bind(s), {4} trunk bind(s))".format(
                  stats["scorched"], stats["fuels_total"], stats["trees"],
                  stats["leaf_binds"], stats["trunk_binds"]))
        for path, t in sorted(stats["targets"].items(),
                              key=lambda kv: kv[1]["dist_m"])[:8]:
            print("[fc]   d{0:<3} {1:<5} {2:5.1f} m of {3:5.1f} m radius  "
                  "{4}".format(t["i"], t["level"], t["dist_m"],
                              t["radius_m"], path))
        self.vram["scorch_veg"] = vram_mb(
            "vegetation scorched ({0} tree(s))".format(stats["trees"]))

    # -- 3c) fire-side debris apron, non-collapse burning buildings --------
    def fire_debris_apron(self):
        """Small ground-debris apron along the venting sides of every F1-F5
        (non-collapse) burning building — see `fire_assembly_lib.
        fire_apron_pass`. One merged Mesh per qualifying building, so the
        prim cost is bounded regardless of how many lumps it carries.
        """
        if not FIRE_APRON or not self.placed:
            self.aprons = []
            return
        rows = fal.fire_apron_pass(self.stage, FIRE_ROOT, self.placed,
                                   seed=SEED)
        self.aprons = rows
        n_lumps = sum(r["n"] for r in rows)
        n_buildings = sum(1 for r in rows if r.get("prim"))
        print("[fc] fire-side debris apron: {0} lump(s) on {1} building(s), "
              "{2} mesh prim(s) ({3} building(s) not gated in: collapse or "
              "unburnt level)".format(n_lumps, n_buildings, n_buildings,
                                      len(rows) - n_buildings))
        for r in rows:
            if not r.get("prim"):
                continue
            print("[fc]   d{0:<3} {1:<30} {2:<4} {3:>3} lump(s) on side(s) "
                  "{4}".format(r["i"], r["stem"], r["level"], r["n"],
                              "/".join(r["sides"])))
        self.vram["apron"] = vram_mb(
            "fire debris apron ({0} lump(s), {1} mesh(es))".format(
                n_lumps, n_buildings))

    # -- 4) the fire, under a GLOBAL emitter budget -------------------------
    def place_people(self):
        """Author the approved people records onto the assembled city.

        `fire_people.to_placements` converts the dry run's records into
        `apply_placements` dicts (pose binding, prone rolls, pose-z drops);
        NOT instanced — `_bind_human_pose` edits inside each rig. Runs after
        `put_the_fire_back` so figures land on the final scene.
        """
        if not PEOPLE:
            print("[fc] people: FC_PEOPLE=0 — skipped")
            return
        if not PEOPLE_JSON or not os.path.isfile(PEOPLE_JSON):
            print("[fc] people: FC_PEOPLE_JSON missing ({0!r}) — skipped; "
                  "run tools/fire_people_dry_run.py and pass its records "
                  "JSON".format(PEOPLE_JSON))
            return
        import json as _json
        import scene_generator as _sg
        from disaster import fire_people as fpl
        doc = _json.load(open(PEOPLE_JSON))
        placements, skipped = fpl.to_placements(doc)
        _sg.apply_placements(self.stage, placements, PARENT + "/people",
                             self.ssf)
        by = {}
        for r in (doc.get("records") if isinstance(doc, dict) else doc) or []:
            by[r.get("cls", "?")] = by.get(r.get("cls", "?"), 0) + 1
        print("[fc] people: {0} authored, {1} skipped ({2})".format(
            len(placements), len(skipped),
            ", ".join("{0} {1}".format(k, v) for k, v in sorted(by.items()))))
        vram_mb("people placed")

    def put_the_fire_back(self):
        if not (FLOW and self.placed):
            self.fires = []
            return
        stage = self.stage
        fx.setup_flow_stack(stage, density_cell_size_m=CELL_M,
                            max_blocks=MAX_BLOCKS, scene_scale_factor=self.ssf)
        flow_root = fx.FLOW_ROOT
        print("[fc] flow stack up at {0} ({1} m cells, {2} block pool)".format(
            flow_root, CELL_M, MAX_BLOCKS))

        predicted = allocate_emitters(self.placed, EMITTER_BUDGET,
                                      MAX_EMITTERS, SMOKE)
        print("\n[fc] EMITTER BUDGET {0} (FA_EMITTER_BUDGET), {1} damaged "
              "building(s), per-building cap {2} opening(s) (FA_EMITTERS)"
              .format(EMITTER_BUDGET, len(self.placed), MAX_EMITTERS))
        print("[fc]   {0:<3} {1:<30} {2:<5} {3:<9} {4:>3} {5:>5} {6:>6}  {7}"
              .format("#", "stem", "level", "state", "st", "alloc", "pred",
                      "note"))
        for r in sorted(self.placed,
                        key=lambda q: (q.get("rank", 9), -q.get("n_st", 0),
                                       q["i"])):
            print("[fc]   {0:<3} {1:<30} {2:<5} {3:<9} {4:>3} {5:>5} {6:>6}  "
                  "{7}".format(
                      r["i"], r["stem"], r["doc"].get("level") or "?",
                      r.get("state", "?"), r.get("n_st", 0),
                      "-" if r.get("alloc") is None else r["alloc"],
                      r.get("est", {}).get("total", 0),
                      r.get("drop_reason", "")))
        print("[fc]   predicted {0} emitter(s) of {1} budgeted; {2} building("
              "s) carry geometry and baked soot only".format(
                  predicted, EMITTER_BUDGET,
                  sum(1 for r in self.placed if r.get("alloc") is None)))
        if predicted > EMITTER_BUDGET:
            print("[fc]   *** the allocator could not stay inside the budget "
                  "— check FA_EMITTERS and FA_MAX_BLOCKS")

        fires = []
        for r in self.placed:
            if r.get("alloc") is None or not r.get("doc"):
                continue
            # THE CELL TRANSFORM, ONTO THE FIRE DATA. An emitter is authored
            # under `<flow_root>/emitters`, which inherits NEITHER the
            # holder's rotation nor its translation, so the wall FRAMES, the
            # MASS centres and the recorded SEATS are all moved here instead
            # — `fire_bake.place`, the general (rotating) form of the row
            # launcher's `translate`.
            #
            # SEATS ARE MOVED BY `place`, SO `place_fire` GETS dx=dy=0.
            # `_sphere_source` adds its own `dx/dy` to each seat; passing the
            # cell again there would translate every plume TWICE.
            fb.place(r["masses"], r["events"], r["doc"].get("seats"),
                     r["x"], r["y"], r["yaw"])
            top_z = r["bbox"][5] if r.get("bbox") else r["doc"].get("top_z")
            res = fal.place_fire(
                self.stage, flow_root, r["doc"], r["masses"], r["events"],
                "c{0}".format(r["i"]), random.Random(SEED + 31 * r["i"]),
                top_z, 0.0, 0.0, scale=SCALE, max_emitters=r["alloc"],
                smoke=SMOKE)
            res["i"] = r["i"]
            res["total"] = (res["flame"] + res["smoke"] + res["interior"]
                            + res["roof"])
            fires.append(res)
            print("[fc] d{0:<2} fire: {1} flame over {2} opening(s), {3} "
                  "smoke, {4} interior, {5} roof = {6} (predicted {7}, "
                  "state={8})".format(
                      r["i"], res["flame"], res.get("openings", 0),
                      res["smoke"], res["interior"], res["roof"],
                      res["total"], r.get("est", {}).get("total", 0),
                      res.get("state")))
            if res.get("note"):
                print("[fc]     " + res["note"])
        self.fires = fires
        total = sum(f["total"] for f in fires)
        print("[fc] {0} Flow emitter(s) in all (budget {1}, predicted {2})"
              .format(total, EMITTER_BUDGET, predicted))
        if total > EMITTER_BUDGET:
            print("[fc] *** OVER BUDGET: {0} > {1}. Flow's block pool is "
                  "finite; past it every emitter after the first few gets no "
                  "voxels and the city renders with NO SMOKE while these "
                  "counts still look right. Lower FA_EMITTER_BUDGET or raise "
                  "FA_MAX_BLOCKS (currently {2})".format(
                      total, EMITTER_BUDGET, MAX_BLOCKS))
        if _flag("FC_FIRE_PHASED", "0"):
            # TWO-PHASE VRAM MEASUREMENT (user, 2026-08-31: "have them
            # deactivated first to measure vram then enable them and
            # activate and test VRAM"). Flow allocates its block pool on
            # UPDATE while emitters are ACTIVE — so deactivate the flow
            # root BEFORE the first post-authoring update, measure the
            # city-without-fire cost, then activate and measure the fire
            # cost as a clean delta.
            fr = self.stage.GetPrimAtPath(flow_root)
            if fr and fr.IsValid():
                fr.SetActive(False)
            for _ in range(30):
                omni.kit.app.get_app().update()
            vram_mb("fires staged but DEACTIVATED")
            if fr and fr.IsValid():
                fr.SetActive(True)
            for _ in range(30):                 # Flow allocates its pool now
                omni.kit.app.get_app().update()
            vram_mb("fires ACTIVATED")
        else:
            for _ in range(30):                 # let Flow allocate its pool
                omni.kit.app.get_app().update()
        self.vram["flow"] = vram_mb("Flow up ({0} emitter(s))".format(total))

        # RE-ASSERT FRACTIONAL CUTOUT OPACITY, NOW THAT THE STAGE IS COMPOSED
        # — the second of the two required forms (see `KIT_ARGS`). The
        # startup flag does not survive composition; the carb form alone is
        # too late for startup. Every launcher in this pipeline does both.
        try:
            _s = carb.settings.get_settings()
            for _k in ("/rtx/raytracing/fractionalCutoutOpacity",
                       "/rtx/pathtracing/fractionalCutoutOpacity"):
                _s.set_bool(_k, True)
            print("[fc] fractionalCutoutOpacity re-asserted post-composition "
                  "(raytracing={0}, pathtracing={1})".format(
                      _s.get("/rtx/raytracing/fractionalCutoutOpacity"),
                      _s.get("/rtx/pathtracing/fractionalCutoutOpacity")))
        except Exception as exc:
            print("[fc] WARNING: could not re-assert fractionalCutoutOpacity "
                  "({0}); the soot and the glass deposits will render as "
                  "hard cutouts".format(exc))
        for _ in range(4):
            omni.kit.app.get_app().update()

    # -- 5) the captures ---------------------------------------------------
    def _snaps(self):
        import importlib.util as _ilu
        sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
        spec = _ilu.spec_from_file_location("snapshots", sp)
        mod = _ilu.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    def capture(self):
        if not SNAP_DIR:
            return
        try:
            snaps = self._snaps()
            os.makedirs(SNAP_DIR, exist_ok=True)
            app = omni.kit.app.get_app()
            for _ in range(60):
                app.update()
            # FLOW NEEDS TIME BEFORE IT IS PHOTOGRAPHED — the emitters inject
            # fuel per step, so a capture at t=0 is of an empty grid.
            if FLOW and getattr(self, "fires", None):
                self.timeline.play()
                for _ in range(300):
                    app.update()
            span = max(self.region)
            snaps.overview(self.stage, (0.0, 0.0), span * 1.05,
                           os.path.join(SNAP_DIR, "city_top.png"), self.ssf)
            if INTACT_ONLY:
                print("[fc] snapshots (intact only) -> {0}".format(SNAP_DIR))
                return
            # THE WAVE — the deliverable. One LOW oblique from behind the
            # origin building, looking DOWNWIND along the line the fire
            # travelled, so the whole spread is in one frame instead of
            # sixteen building portraits. `heading_deg` is the direction the
            # wind blows TOWARD (`urban_fire_spread._wind_factor`'s own
            # `wind_dir` convention), so the camera stands UPWIND of the
            # origin and looks the other way.
            org = self.origin_row()
            if org is not None:
                h = math.radians(float(self.wind["heading_deg"]))
                ux, uy = math.cos(h), math.sin(h)
                b = org.get("bbox")
                H = (b[5] - b[2]) if b else 30.0
                back = max(70.0, 1.8 * H)
                fwd = max(150.0, 0.55 * span)
                eye = ((org["x"] - back * ux) * self.ssf,
                       (org["y"] - back * uy) * self.ssf,
                       max(14.0, 0.45 * H) * self.ssf)
                tgt = ((org["x"] + fwd * ux) * self.ssf,
                       (org["y"] + fwd * uy) * self.ssf,
                       (0.30 * H) * self.ssf)
                snaps.place_camera(self.stage, eye, tgt)
                snaps.snapshot(os.path.join(SNAP_DIR, "wave_downwind.png"))
                print("[fc] the wave: from ({0:+.0f}, {1:+.0f}) at {2:.0f} m "
                      "looking {3:.0f} deg downwind past {4}".format(
                          org["x"] - back * ux, org["y"] - back * uy,
                          eye[2] / max(self.ssf, 1e-9),
                          self.wind["heading_deg"], org["stem"]))
            # the fire-facing per-building review pair, framed by the same
            # arithmetic the row launcher uses (`fal.fire_view_params`)
            for r in self.placed:
                b = r.get("bbox")
                if not b:
                    continue
                name = "d{0}_{1}_{2}".format(
                    r["i"], r["doc"].get("name") or r["stem"],
                    r["doc"].get("level", ""))
                vp = fal.fire_view_params(r["doc"], r["masses"], b)
                snaps.views_around(self.stage, {name: (r["x"], r["y"])},
                                   SNAP_DIR, self.ssf, top_h=vp["top_h"],
                                   obl_dist=vp["obl_dist"],
                                   obl_h=vp["obl_h"],
                                   azimuth_deg=vp["azimuth_deg"],
                                   aim_h=vp["aim_h"])
            print("[fc] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[fc] snapshots FAILED: {0}".format(exc))

    def origin_row(self):
        """The building the fire started in: the earliest `t_ignite_s` among
        the placed bakes (the spread solve's own origin has `t_ignite_s` 0),
        falling back to manifest order."""
        cands = [r for r in self.placed if r.get("bbox")]
        if not cands:
            return None
        return sorted(cands, key=lambda r: (
            float(r["rec"].get("t_ignite_s") if r["rec"].get("t_ignite_s")
                  is not None else 1e18), r["i"]))[0]

    # -- 6) the budget and the log ------------------------------------------
    def report(self):
        self.vram["end"] = vram_mb("after captures")
        n_dmg = max(1, len(getattr(self, "placed", []) or []))
        base = self.vram.get("empty")
        intact = self.vram.get("intact")
        bakes = self.vram.get("bakes")
        flow_v = self.vram.get("flow")
        end = self.vram.get("end")
        try:
            if base is not None and intact is not None:
                city_cost = intact - base
                area = self.region[0] * self.region[-1]
                per_km2 = city_cost * (1.0e6 / max(1.0, area))
                bake_cost = (bakes - intact) if bakes is not None else None
                per_b = (bake_cost / n_dmg) if bake_cost is not None else None
                flow_cost = ((flow_v - bakes) if (flow_v is not None
                                                  and bakes is not None)
                             else None)
                print("[fc] VRAM BUDGET: baseline {0:.0f} MiB | INTACT CITY "
                      "{1:.0f} MiB for {2} building(s) on {3:.2f} km2 "
                      "(= {4:.0f} MiB/km2) | bakes {5} for {6} damaged = {7} "
                      "| Flow {8} | end {9:.0f} MiB".format(
                          base, city_cost, self.n_houses, area / 1.0e6,
                          per_km2,
                          "{0:.0f} MiB".format(bake_cost)
                          if bake_cost is not None else "n/a", n_dmg,
                          "{0:.0f} MiB/building".format(per_b)
                          if per_b is not None else "n/a",
                          "{0:.0f} MiB".format(flow_cost)
                          if flow_cost is not None else "off",
                          end if end is not None else -1))
                print("[fc]   NOTE: the intact-city figure INCLUDES the {0} "
                      "building(s) later hidden ({1}) — an invisible prim "
                      "still composes; FC_HIDE=deactivate to drop them"
                      .format(self.n_hidden, HIDE_MODE))
                if per_b is not None:
                    for n_city, gpu, cap in ((16, "5090", 32768),
                                             (16, "RTX PRO 5000", 49152),
                                             (40, "5090", 32768),
                                             (40, "RTX PRO 5000", 49152)):
                        proj = (base + per_km2 + per_b * n_city
                                + (flow_cost or 0.0))
                        print("[fc]   projection: 1 km2 of this city + {0} "
                              "damaged building(s) + this Flow -> {1:.0f} MiB "
                              "on a {2} ({3:.0%} of {4} MiB)".format(
                                  n_city, proj, gpu, proj / cap, cap))
        except Exception as exc:
            print("[fc] VRAM budget summary failed: {0}".format(exc))

        # THE KIT LOG. Flow fails silently — see the module docstring.
        path, hits = grep_kit_log()
        if path is None:
            print("[fc] Kit log not found under {0} — the Flow OOM check did "
                  "NOT run; do not trust a capture that shows no smoke"
                  .format(" / ".join(KIT_LOG_GLOBS)))
        elif any(hits.values()):
            print("\n[fc] " + "!" * 70)
            print("[fc] FLOW STARVED — {0}".format(
                ", ".join("{0!r} x{1}".format(k, v)
                          for k, v in hits.items() if v)))
            print("[fc]   log: {0}".format(path))
            print("[fc]   Flow's block pool ran out. Emitters past the first "
                  "few got NO VOXELS: the city renders with no smoke while "
                  "every count above looks right. RAISE FA_MAX_BLOCKS "
                  "(currently {0}) or LOWER FA_EMITTER_BUDGET (currently "
                  "{1}) — do not go looking at the emitters.".format(
                      MAX_BLOCKS, EMITTER_BUDGET))
            print("[fc] " + "!" * 70 + "\n")
        else:
            print("[fc] Flow OOM check CLEAN ({0})".format(path))

    def run(self):
        if INTACT_ONLY:
            print("\n[fc] FC_INTACT_ONLY=1 — the city is built and nothing "
                  "will be damaged. This is plan sec 4c's 'measure the "
                  "intact city FIRST': it is the only unknown in the VRAM "
                  "projection.")
            self.placed, self.refused, self.rows = [], [], []
            self.missing, self.fires = [], []
            self.n_hidden = 0
            self.capture()
            self.banner()
            return
        self.load_fire()
        self.compose_bakes()
        self.scorch_vegetation()
        self.fire_debris_apron()
        self.put_the_fire_back()
        self.place_people()
        self.capture()
        self.banner()

    def banner(self):
        self.report()
        print("\n" + "=" * 78)
        print("URBAN FIRE CITY   {0}   {1:.0f} x {2:.0f} m   {3} building(s), "
              "{4} damaged".format(SCENE_CONFIG, self.region[0],
                                   self.region[-1], self.n_houses,
                                   len(getattr(self, "placed", []) or [])))
        if not INTACT_ONLY:
            print("  manifest: {0}  (city seed {1})".format(
                getattr(self, "manifest_path", "?"),
                getattr(self, "city_seed", "?")))
            for r in getattr(self, "placed", []) or []:
                b = r.get("bbox")
                print("  ({0:+7.1f},{1:+7.1f}) yaw {2:+4.0f}  {3:<30} {4:<5} "
                      "{5:>6.1f} MB  {6} event(s)  {7}".format(
                          r["x"], r["y"], r["yaw"], r["stem"],
                          r["doc"].get("level") or "?",
                          os.path.getsize(r["usd"]) / 1e6, len(r["events"]),
                          "H {0:.0f} m".format(b[5] - b[2]) if b
                          else "NO GEOMETRY"))
            if getattr(self, "missing", None):
                print("  {0} record(s) NOT BAKED (still intact): {1}".format(
                    len(self.missing),
                    ", ".join(str(m[1]) for m in self.missing)))
            if getattr(self, "refused", None):
                print("  {0} bake(s) refused: {1}".format(
                    len(self.refused),
                    ", ".join("{0} ({1})".format(r["stem"], r.get("skip"))
                              for r in self.refused)))
            veg = getattr(self, "veg_scorch", None)
            if veg is not None:
                print("  scorched vegetation: {0} tree(s) darkened of {1} "
                      "fuel placement(s) checked (FC_SCORCH_VEG={2})".format(
                          veg.get("trees", 0), veg.get("fuels_total", 0),
                          int(SCORCH_VEG)))
            aprons = getattr(self, "aprons", None)
            if aprons is not None:
                n_b = sum(1 for r in aprons if r.get("prim"))
                n_l = sum(r.get("n", 0) for r in aprons)
                print("  fire debris apron: {0} lump(s) on {1} building(s) "
                      "(FC_FIRE_APRON={2})".format(n_l, n_b, int(FIRE_APRON)))
        print("  built in {0:.0f} s".format(time.time() - self.t0))
        print("=" * 78 + "\n")
        print("URBAN FIRE CITY DONE")


def main():
    FireCityApp().run()


if __name__ == "__main__":
    try:
        main()
    except Exception as _exc:
        import traceback
        traceback.print_exc()
        print("URBAN FIRE CITY FAILED: {0}".format(_exc))
    if KEEP_OPEN or not _HEADLESS:
        _app = omni.kit.app.get_app()
        omni.timeline.get_timeline_interface().play()
        while simulation_app.is_running():
            _app.update()
        omni.timeline.get_timeline_interface().stop()
    simulation_app.close()
